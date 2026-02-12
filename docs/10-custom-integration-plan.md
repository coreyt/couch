# Custom SOFA-Unity Integration Plan

> Collaborative plan by the Unity 6 Expert, SOFA Framework Expert, and Integration Expert.
> Goal: Design and build a custom bridge between Unity 6 and SOFA Framework from scratch — no SofaUnity plugin, no off-the-shelf middleware.

---

## 1. Why Custom?

**Integration Expert:** The SofaUnity plugin from InfinyTech3D is a black box. We don't control the C++ DLL API surface, we can't optimize data paths for our specific use case (ankle ROM with topology changes), and we're locked to their release cadence. A custom bridge gives us:

- Full control over the API contract
- Optimized data transfer for our specific mesh sizes (~2K-50K vertices)
- Direct access to SOFA's topology change events (critical for resection)
- Ability to run SOFA stepping on a dedicated thread with our own synchronization
- No licensing ambiguity for advanced features

**Unity 6 Expert:** Agreed. SofaUnity's MonoBehaviour-heavy design also fights against Unity 6's direction — we should use NativeArrays and the Jobs system for mesh data transfer rather than managed arrays through P/Invoke.

**SOFA Expert:** From SOFA's side, we lose the convenience of SofaUnity's scene graph mirroring, but we gain direct access to the C++ API — which means we can use `sofa::simulation` directly, handle topology events properly, and control solver lifecycle precisely.

---

## 2. Architecture Decision

### Rejected Alternatives

| Approach | Why Rejected |
|----------|-------------|
| **SofaUnity plugin** | Black box, no control over API, potential licensing issues for advanced features |
| **Pure ZMQ cross-process** | Adds ~50-100us per message, JSON serialization overhead for mesh data, process management complexity |
| **Pure shared memory** | Complex synchronization, no structured command/response pattern, hard to debug |
| **SofaPython3 embedded in Unity** | Python GIL, embedding Python in Unity process is fragile, version conflicts |

### Selected: Custom C++ Native Plugin (Hybrid Architecture)

```
Unity 6 Process
├── C# Managed Layer (AnkleSim.Bridge assembly)
│   ├── SofaNativeBridge.cs      ← P/Invoke declarations
│   ├── SofaSimulation.cs        ← High-level C# API (lifecycle, commands)
│   ├── SofaMeshTransfer.cs      ← NativeArray-based mesh sync
│   └── SofaCommandQueue.cs      ← Thread-safe command buffer
│
├── Native Plugin (SofaAnkleBridge.dll/.so)  ← Custom C++ DLL
│   ├── bridge_api.h/.cpp        ← Flat C API (extern "C")
│   ├── sofa_simulation.h/.cpp   ← SOFA lifecycle management
│   ├── scene_builder.h/.cpp     ← Programmatic scene construction
│   ├── data_transfer.h/.cpp     ← Position/force readback
│   ├── command_handler.h/.cpp   ← Resection, implant, constraint commands
│   └── thread_manager.h/.cpp    ← Background stepping thread
│
└── SOFA Libraries (linked by our DLL)
    ├── Sofa.Core, Sofa.Simulation.Graph
    ├── Sofa.Component.* (FEM, Collision, Constraint, Topology, etc.)
    └── Plugin DLLs (SofaCarving, CGALPlugin, etc.)
```

**Integration Expert:** This gives us in-process performance (~1us per call) with full API control. The C++ DLL links against SOFA libraries at build time and exposes a flat C API. Unity calls it through P/Invoke. SOFA stepping runs on a dedicated background thread managed by our DLL.

---

## 3. C API Design

### 3.1 Lifecycle API

**SOFA Expert:** SOFA needs a strict init → build scene → simulate → cleanup lifecycle. The C API must enforce this ordering.

```c
// === Lifecycle ===
EXPORT int         sofa_bridge_init(const char* plugin_dir);
EXPORT void        sofa_bridge_shutdown();
EXPORT const char* sofa_bridge_get_error();  // last error string

// === Scene Construction ===
EXPORT int  sofa_scene_create(const SofaSceneConfig* config);
EXPORT void sofa_scene_destroy();
EXPORT int  sofa_scene_is_ready();

// === Simulation Control ===
EXPORT int   sofa_step(float dt);              // blocking single step
EXPORT int   sofa_step_async();                // non-blocking, starts step on bg thread
EXPORT int   sofa_step_async_is_complete();    // poll for completion
EXPORT void  sofa_step_async_wait();           // block until bg step finishes
EXPORT float sofa_get_last_step_time_ms();
EXPORT int   sofa_is_solver_diverged();
EXPORT void  sofa_reset();
```

### 3.2 Scene Construction API

**SOFA Expert:** Rather than exposing SOFA's entire component model through C, we define domain-specific scene construction functions. This keeps the API surface small and prevents misuse.

```c
// === Scene Config (passed to sofa_scene_create) ===
typedef struct {
    float gravity[3];
    float timestep;
    int   constraint_iterations;
    float constraint_tolerance;
    float rayleigh_stiffness;
    float rayleigh_mass;
    float alarm_distance;
    float contact_distance;
    float friction_coefficient;
} SofaSceneConfig;

// === Bone Construction ===
typedef struct {
    const char* name;
    const float* collision_vertices;   // flattened [x,y,z, x,y,z, ...]
    int          collision_vertex_count;
    const int*   collision_triangles;  // flattened [i0,i1,i2, ...]
    int          collision_triangle_count;
    float        position[3];
    float        orientation[4];       // quaternion [x,y,z,w]
    float        mass;
    int          is_fixed;             // 1 = pinned in space
} SofaRigidBoneConfig;

EXPORT int sofa_add_rigid_bone(const SofaRigidBoneConfig* config);

// === Deformable Tissue ===
typedef struct {
    const char*  name;
    const char*  volumetric_mesh_path;   // VTK file path
    float        young_modulus;          // MPa
    float        poisson_ratio;
    float        total_mass;
    const char*  fem_method;            // "small", "large", "polar"
} SofaDeformableConfig;

EXPORT int sofa_add_deformable_tissue(const SofaDeformableConfig* config);

// === Ligament ===
typedef struct {
    const char* name;
    const char* bone_a_name;
    const char* bone_b_name;
    int         attachment_index_a;
    int         attachment_index_b;
    float       stiffness;             // N/mm
    float       damping;
    float       rest_length;           // mm
} SofaLigamentConfig;

EXPORT int sofa_add_ligament(const SofaLigamentConfig* config);

// === Rigid Implant ===
typedef struct {
    const char*  name;
    const float* vertices;
    int          vertex_count;
    const int*   triangles;
    int          triangle_count;
    float        position[3];
    float        orientation[4];
    float        mass;
    float        young_modulus;        // for contact stiffness
} SofaImplantConfig;

EXPORT int sofa_add_rigid_implant(const SofaImplantConfig* config);

// === Scene Finalization ===
EXPORT int sofa_scene_finalize();   // call after adding all objects, before stepping
```

### 3.3 Data Readback API

**Unity Expert:** This is the hot path. Every frame we need mesh positions back from SOFA. The API must support zero-copy or at least minimize copies. We'll use pinned memory buffers that Unity pre-allocates.

```c
// === Mesh Data Readback ===
// Returns number of vertices written. Caller provides buffer.
EXPORT int sofa_get_positions(const char* node_name,
                               float* out_buffer,     // pre-allocated [N*3]
                               int    buffer_capacity);

// Sparse update: only vertices that moved more than threshold
typedef struct {
    int   index;
    float position[3];
} SofaSparseVertex;

EXPORT int sofa_get_changed_positions(const char* node_name,
                                       float threshold,
                                       SofaSparseVertex* out_buffer,
                                       int buffer_capacity);

// === Force/Angle Readback ===
EXPORT float sofa_get_joint_angle(const char* joint_name, int axis); // 0=X, 1=Y, 2=Z
EXPORT float sofa_get_joint_torque(const char* joint_name, int axis);
EXPORT int   sofa_get_contact_forces(const char* body_a, const char* body_b,
                                      float* out_forces,  // [N*3]
                                      int buffer_capacity);

// === Topology Info (post-resection) ===
EXPORT int sofa_get_surface_mesh(const char* node_name,
                                  float* out_vertices, int* out_vertex_count,
                                  int*   out_triangles, int* out_triangle_count,
                                  int vertex_capacity, int triangle_capacity);

EXPORT int sofa_has_topology_changed(const char* node_name);
```

### 3.4 Command API

**SOFA Expert:** Commands modify the scene. These must be queued and executed at safe points (between simulation steps, never during a step).

```c
// === Resection ===
typedef struct {
    const char* bone_name;
    float       plane_point[3];
    float       plane_normal[3];
} SofaResectionCommand;

EXPORT int sofa_execute_resection(const SofaResectionCommand* cmd);
EXPORT int sofa_get_removed_element_count(const char* bone_name);

// === Implant Placement ===
EXPORT int sofa_set_implant_position(const char* implant_name,
                                      float position[3],
                                      float orientation[4]);

// === Joint Constraints ===
EXPORT int sofa_set_joint_limits(const char* joint_name,
                                  int axis,
                                  float min_deg, float max_deg);
EXPORT int sofa_apply_joint_torque(const char* joint_name,
                                    int axis, float torque_nm);

// === Material Properties (runtime adjustment) ===
EXPORT int sofa_set_material_property(const char* node_name,
                                       const char* property_name,
                                       float value);
```

---

## 4. C# Managed Layer Design

### 4.1 P/Invoke Declarations

**Unity Expert:** We wrap the C API in a static class with proper marshaling attributes. All string parameters use `LPStr` for ANSI compatibility with SOFA's C++ strings.

```csharp
// SofaNativeBridge.cs — P/Invoke declarations only
internal static class SofaNativeBridge
{
    private const string DLL = "SofaAnkleBridge";

    // Lifecycle
    [DllImport(DLL)] internal static extern int sofa_bridge_init(
        [MarshalAs(UnmanagedType.LPStr)] string pluginDir);
    [DllImport(DLL)] internal static extern void sofa_bridge_shutdown();
    [DllImport(DLL)] internal static extern IntPtr sofa_bridge_get_error();

    // Scene construction
    [DllImport(DLL)] internal static extern int sofa_scene_create(
        ref SofaSceneConfig config);
    [DllImport(DLL)] internal static extern int sofa_add_rigid_bone(
        ref SofaRigidBoneConfig config);
    // ... etc

    // Simulation
    [DllImport(DLL)] internal static extern int sofa_step(float dt);
    [DllImport(DLL)] internal static extern int sofa_step_async();
    [DllImport(DLL)] internal static extern int sofa_step_async_is_complete();

    // Data readback — uses unsafe for direct buffer access
    [DllImport(DLL)] internal static extern unsafe int sofa_get_positions(
        [MarshalAs(UnmanagedType.LPStr)] string nodeName,
        float* outBuffer, int bufferCapacity);
}
```

### 4.2 High-Level C# API

**Unity Expert:** The high-level API provides a safe, Unity-idiomatic interface. It manages NativeArrays for mesh data and provides async stepping via Unity's job system.

```csharp
// SofaSimulation.cs — High-level API
public class SofaSimulation : IDisposable
{
    private bool _initialized;
    private NativeArray<float> _positionBuffer;
    private NativeArray<SofaSparseVertex> _sparseBuffer;

    public bool IsInitialized => _initialized;
    public bool IsSolverDiverged => SofaNativeBridge.sofa_is_solver_diverged() != 0;
    public float LastStepTimeMs => SofaNativeBridge.sofa_get_last_step_time_ms();

    public bool Initialize(string pluginDir)
    {
        int result = SofaNativeBridge.sofa_bridge_init(pluginDir);
        _initialized = result == 0;
        return _initialized;
    }

    public bool CreateScene(SimulationConfig config)
    {
        var nativeConfig = config.ToNative();
        return SofaNativeBridge.sofa_scene_create(ref nativeConfig) == 0;
    }

    public void AddRigidBone(RigidBoneDescriptor desc) { /* marshal + call */ }
    public void AddDeformableTissue(DeformableTissueDescriptor desc) { /* marshal + call */ }
    public void AddLigament(LigamentDescriptor desc) { /* marshal + call */ }
    public void AddRigidImplant(ImplantDescriptor desc) { /* marshal + call */ }

    public bool FinalizeScene()
    {
        return SofaNativeBridge.sofa_scene_finalize() == 0;
    }

    // Synchronous step
    public bool Step(float dt)
    {
        return SofaNativeBridge.sofa_step(dt) == 0;
    }

    // Async step (call from FixedUpdate, poll in Update)
    public bool StepAsync()
    {
        return SofaNativeBridge.sofa_step_async() == 0;
    }

    public bool IsStepComplete => SofaNativeBridge.sofa_step_async_is_complete() != 0;

    // Mesh readback into pre-allocated NativeArray
    public unsafe int GetPositions(string nodeName, NativeArray<Vector3> buffer)
    {
        return SofaNativeBridge.sofa_get_positions(
            nodeName,
            (float*)buffer.GetUnsafePtr(),
            buffer.Length);
    }

    public void Dispose()
    {
        if (_initialized)
        {
            SofaNativeBridge.sofa_bridge_shutdown();
            _initialized = false;
        }
        if (_positionBuffer.IsCreated) _positionBuffer.Dispose();
        if (_sparseBuffer.IsCreated) _sparseBuffer.Dispose();
    }
}
```

### 4.3 Mesh Synchronization with Jobs

**Unity Expert:** For mesh sync we use Burst-compiled jobs to copy SOFA positions into Unity meshes. This avoids main-thread stalls for large meshes.

```csharp
[BurstCompile]
public struct ApplyPositionsJob : IJobParallelFor
{
    [ReadOnly] public NativeArray<float3> SofaPositions;
    public NativeArray<float3> UnityVertices;

    // SOFA uses mm, Unity uses m (if applicable)
    public float ScaleFactor;

    public void Execute(int index)
    {
        UnityVertices[index] = SofaPositions[index] * ScaleFactor;
    }
}

[BurstCompile]
public struct ApplySparsePositionsJob : IJob
{
    [ReadOnly] public NativeArray<SofaSparseVertex> Changes;
    public NativeArray<float3> UnityVertices;
    public int ChangeCount;
    public float ScaleFactor;

    public void Execute()
    {
        for (int i = 0; i < ChangeCount; i++)
        {
            var change = Changes[i];
            UnityVertices[change.Index] = new float3(
                change.X, change.Y, change.Z) * ScaleFactor;
        }
    }
}
```

---

## 5. C++ Native Plugin Implementation

### 5.1 Build System

**Integration Expert:** We use CMake to build the native plugin. It links against SOFA libraries and produces a shared library that Unity loads.

```cmake
cmake_minimum_required(VERSION 3.16)
project(SofaAnkleBridge LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)

# Find SOFA (installed or build-tree)
find_package(Sofa.Core REQUIRED)
find_package(Sofa.Simulation.Graph REQUIRED)
find_package(Sofa.Component REQUIRED)

add_library(SofaAnkleBridge SHARED
    src/bridge_api.cpp
    src/sofa_simulation.cpp
    src/scene_builder.cpp
    src/data_transfer.cpp
    src/command_handler.cpp
    src/thread_manager.cpp
)

target_link_libraries(SofaAnkleBridge
    Sofa.Core
    Sofa.Simulation.Graph
    Sofa.Component
)

# Platform-specific output names
if(WIN32)
    set_target_properties(SofaAnkleBridge PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
        PREFIX "")
elseif(UNIX)
    set_target_properties(SofaAnkleBridge PROPERTIES
        LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib"
        PREFIX "lib")
endif()
```

### 5.2 Core C++ Implementation

**SOFA Expert:** The core implementation manages the SOFA simulation lifecycle and provides the scene-building primitives.

```cpp
// sofa_simulation.h
#pragma once

#include <sofa/simulation/graph/DAGSimulation.h>
#include <sofa/simulation/graph/DAGNode.h>
#include <sofa/core/objectmodel/BaseObject.h>
#include <mutex>
#include <thread>
#include <atomic>
#include <string>

class SofaSimulationManager {
public:
    bool init(const std::string& pluginDir);
    void shutdown();

    bool createScene(const SofaSceneConfig& config);
    void destroyScene();

    // Scene construction
    int addRigidBone(const SofaRigidBoneConfig& config);
    int addDeformableTissue(const SofaDeformableConfig& config);
    int addLigament(const SofaLigamentConfig& config);
    int addRigidImplant(const SofaImplantConfig& config);
    bool finalizeScene();

    // Simulation
    bool step(float dt);
    bool stepAsync();
    bool isStepComplete() const;
    void waitForStep();
    float lastStepTimeMs() const;
    bool isSolverDiverged() const;

    // Data readback (thread-safe: only call when step is complete)
    int getPositions(const std::string& nodeName, float* buffer, int capacity);
    int getChangedPositions(const std::string& nodeName, float threshold,
                            SofaSparseVertex* buffer, int capacity);
    float getJointAngle(const std::string& jointName, int axis);
    float getJointTorque(const std::string& jointName, int axis);
    bool hasTopologyChanged(const std::string& nodeName);
    int getSurfaceMesh(const std::string& nodeName,
                       float* vertices, int* vertexCount,
                       int* triangles, int* triangleCount,
                       int vertCap, int triCap);

    // Commands (queued, executed between steps)
    int executeResection(const SofaResectionCommand& cmd);
    int setImplantPosition(const std::string& name, float pos[3], float ori[4]);
    int setJointLimits(const std::string& name, int axis, float min, float max);
    int applyJointTorque(const std::string& name, int axis, float torque);

    const std::string& lastError() const { return _lastError; }

private:
    sofa::simulation::Node::SPtr _root;
    std::mutex _sceneMutex;
    std::thread _stepThread;
    std::atomic<bool> _stepping{false};
    std::atomic<bool> _stepComplete{true};
    std::atomic<bool> _diverged{false};
    float _lastStepMs{0.0f};
    std::string _lastError;
    bool _sceneReady{false};

    // Cached node pointers for fast lookup
    std::unordered_map<std::string, sofa::simulation::Node*> _nodeCache;

    // Previous positions for sparse delta detection
    std::unordered_map<std::string, std::vector<sofa::type::Vec3d>> _prevPositions;

    // Topology change flags
    std::unordered_map<std::string, bool> _topologyDirty;

    void stepThread(float dt);
    sofa::simulation::Node* findNode(const std::string& name);
};
```

### 5.3 Thread Safety Model

**Integration Expert:** SOFA is not thread-safe internally, so we enforce a strict access pattern:

```
Timeline:
  Unity Main Thread:     [ReadData] [QueueCmd] [StartStep] --------- [ReadData] [QueueCmd] [StartStep] ...
  SOFA Worker Thread:                            [RunStep+ProcessCmds]                       [RunStep+ProcessCmds]
```

Rules:
1. **Never** access SOFA objects while a step is in progress
2. Data readback only after `isStepComplete()` returns true
3. Commands are queued on the main thread, executed by the worker thread before the next step
4. The C++ `std::mutex` guards all SOFA scene access
5. Atomic flags (`_stepping`, `_stepComplete`) provide lock-free polling

```cpp
bool SofaSimulationManager::stepAsync() {
    if (!_sceneReady || _stepping.load()) return false;

    _stepping.store(true);
    _stepComplete.store(false);

    _stepThread = std::thread([this]() {
        auto start = std::chrono::high_resolution_clock::now();

        std::lock_guard<std::mutex> lock(_sceneMutex);

        // Process queued commands first
        processCommandQueue();

        // Execute SOFA step
        try {
            sofa::simulation::node::animate(_root.get(), _root->getDt());
            _diverged.store(false);
        } catch (const std::exception& e) {
            _lastError = e.what();
            _diverged.store(true);
        }

        auto end = std::chrono::high_resolution_clock::now();
        _lastStepMs = std::chrono::duration<float, std::milli>(end - start).count();

        _stepping.store(false);
        _stepComplete.store(true);
    });
    _stepThread.detach();
    return true;
}
```

---

## 6. Scene Construction Details

### 6.1 Root Node Setup

**SOFA Expert:** The root node establishes the constraint-based pipeline needed for contact simulation.

```cpp
bool SofaSimulationManager::createScene(const SofaSceneConfig& config) {
    _root = sofa::simulation::node::createRootNode("root");

    _root->setGravity({config.gravity[0], config.gravity[1], config.gravity[2]});
    _root->setDt(config.timestep);

    // Animation loop for constraint-based contact
    _root->addObject<sofa::component::animationloop::FreeMotionAnimationLoop>();

    // Constraint solver
    auto cs = _root->addObject<sofa::component::constraint::lagrangian::solver::GenericConstraintSolver>();
    cs->d_maxIt.setValue(config.constraint_iterations);
    cs->d_tolerance.setValue(config.constraint_tolerance);

    // Collision pipeline
    _root->addObject<sofa::component::collision::detection::algorithm::DefaultPipeline>();
    _root->addObject<sofa::component::collision::detection::algorithm::BruteForceBroadPhase>();
    _root->addObject<sofa::component::collision::detection::algorithm::BVHNarrowPhase>();

    auto intersection = _root->addObject<
        sofa::component::collision::detection::intersection::MinProximityIntersection>();
    intersection->d_alarmDistance.setValue(config.alarm_distance);
    intersection->d_contactDistance.setValue(config.contact_distance);

    auto contactManager = _root->addObject<
        sofa::component::collision::response::contact::DefaultContactManager>();
    contactManager->d_response.setValue("FrictionContact");
    // friction coefficient set via responseParams

    return true;
}
```

### 6.2 Rigid Bone Node

```cpp
int SofaSimulationManager::addRigidBone(const SofaRigidBoneConfig& config) {
    auto node = _root->addChild(config.name);

    // Solver (needed even for rigid for constraint correction)
    node->addObject<sofa::component::odesolver::backward::EulerImplicitSolver>();
    node->addObject<sofa::component::linearsolver::iterative::CGLinearSolver>();

    // Rigid DOF
    auto mobj = node->addObject<sofa::component::statecontainer::MechanicalObject<Rigid3Types>>();
    sofa::type::Rigid3Types::Coord initialPos;
    initialPos.getCenter() = {config.position[0], config.position[1], config.position[2]};
    initialPos.getOrientation() = {config.orientation[0], config.orientation[1],
                                    config.orientation[2], config.orientation[3]};
    mobj->x.setValue({initialPos});

    // Mass
    auto mass = node->addObject<sofa::component::mass::UniformMass<Rigid3Types>>();
    mass->d_totalMass.setValue(config.mass);

    // Fixed constraint if pinned
    if (config.is_fixed) {
        auto fix = node->addObject<sofa::component::constraint::projective::FixedConstraint<Rigid3Types>>();
        fix->d_indices.setValue({0});
    }

    // Constraint correction
    node->addObject<sofa::component::constraint::lagrangian::correction::UncoupledConstraintCorrection<Rigid3Types>>();

    // Collision sub-node
    auto colNode = node->addChild("Collision");
    // ... load collision mesh from provided vertices/triangles ...
    // ... add TriangleCollisionModel, LineCollisionModel, PointCollisionModel ...
    // ... add RigidMapping ...

    _nodeCache[config.name] = node.get();
    return 0;
}
```

### 6.3 Deformable Tissue Node (Cartilage)

```cpp
int SofaSimulationManager::addDeformableTissue(const SofaDeformableConfig& config) {
    auto node = _root->addChild(config.name);

    // Solver
    node->addObject<sofa::component::odesolver::backward::EulerImplicitSolver>();
    auto ldl = node->addObject<sofa::component::linearsolver::direct::SparseLDLSolver<
        sofa::linearalgebra::CompressedRowSparseMatrixd>>();

    // Load volumetric mesh
    auto loader = node->addObject<sofa::component::io::mesh::MeshVTKLoader>();
    loader->d_filename.setValue(config.volumetric_mesh_path);
    loader->load();

    // Topology
    auto topo = node->addObject<sofa::component::topology::container::dynamic::TetrahedronSetTopologyContainer>();
    topo->d_src.setValue("@loader");
    node->addObject<sofa::component::topology::container::dynamic::TetrahedronSetTopologyModifier>();
    node->addObject<sofa::component::topology::container::dynamic::TetrahedronSetGeometryAlgorithms<Vec3Types>>();

    // DOFs
    auto mobj = node->addObject<sofa::component::statecontainer::MechanicalObject<Vec3Types>>();

    // Mass
    auto mass = node->addObject<sofa::component::mass::MeshMatrixMass<Vec3Types>>();
    mass->d_totalMass.setValue(config.total_mass);

    // FEM
    auto fem = node->addObject<sofa::component::solidmechanics::fem::elastic::TetrahedronFEMForceField<Vec3Types>>();
    fem->d_method.setValue(config.fem_method);
    fem->d_youngModulus.setValue(config.young_modulus);
    fem->d_poissonRatio.setValue(config.poisson_ratio);

    // Constraint correction
    node->addObject<sofa::component::constraint::lagrangian::correction::GenericConstraintCorrection>();

    // Collision sub-node with topological mapping
    auto colNode = node->addChild("Collision");
    colNode->addObject<sofa::component::topology::container::dynamic::TriangleSetTopologyContainer>();
    colNode->addObject<sofa::component::topology::container::dynamic::TriangleSetTopologyModifier>();
    colNode->addObject<sofa::component::topology::mapping::Tetra2TriangleTopologicalMapping>();
    auto colMobj = colNode->addObject<sofa::component::statecontainer::MechanicalObject<Vec3Types>>();
    colNode->addObject<sofa::component::collision::geometry::TriangleCollisionModel<Vec3Types>>();
    colNode->addObject<sofa::component::collision::geometry::LineCollisionModel<Vec3Types>>();
    colNode->addObject<sofa::component::collision::geometry::PointCollisionModel<Vec3Types>>();
    colNode->addObject<sofa::component::mapping::linear::BarycentricMapping<Vec3Types, Vec3Types>>();

    _nodeCache[config.name] = node.get();
    return 0;
}
```

---

## 7. Data Transfer Strategy

### 7.1 Position Buffer Protocol

**Unity Expert:** We use a double-buffer scheme. While SOFA computes step N+1, Unity renders with step N's data. This decouples the frame rates.

```
Frame N:
  Unity Main Thread:     [Read Buffer A] [Render] [Swap] [Start SOFA Step]
  SOFA Worker Thread:                     [Step → Write Buffer B]

Frame N+1:
  Unity Main Thread:     [Read Buffer B] [Render] [Swap] [Start SOFA Step]
  SOFA Worker Thread:                     [Step → Write Buffer A]
```

**Integration Expert:** The double buffer lives in the C++ DLL. Unity reads the "front" buffer while SOFA writes to the "back" buffer. Buffer swap happens atomically when the step completes.

```cpp
class DoubleBuffer {
    std::vector<float> _bufferA;
    std::vector<float> _bufferB;
    std::atomic<int> _front{0};  // 0 = A is front, 1 = B is front

public:
    float* getReadBuffer()  { return _front == 0 ? _bufferA.data() : _bufferB.data(); }
    float* getWriteBuffer() { return _front == 0 ? _bufferB.data() : _bufferA.data(); }
    void swap() { _front.store(1 - _front.load()); }
};
```

### 7.2 Topology Change Notification

**SOFA Expert:** When a resection modifies the topology, the surface mesh changes. We need to signal this to Unity so it rebuilds the mesh rather than just updating vertex positions.

```cpp
// After resection, set dirty flag
_topologyDirty[boneName] = true;

// Unity polls:
bool SofaSimulationManager::hasTopologyChanged(const std::string& nodeName) {
    auto it = _topologyDirty.find(nodeName);
    if (it != _topologyDirty.end() && it->second) {
        it->second = false;  // clear on read
        return true;
    }
    return false;
}
```

**Unity Expert:** When topology changes, Unity fetches the full new surface mesh via `sofa_get_surface_mesh()` and rebuilds the Unity Mesh object.

---

## 8. Resection Implementation

### 8.1 SOFA Side

**SOFA Expert:** Resection removes tetrahedra that are fully on the negative side of the cutting plane. We use `TetrahedronSetTopologyModifier::removeTetrahedra()` which automatically updates the surface mesh through `Tetra2TriangleTopologicalMapping`.

```cpp
int SofaSimulationManager::executeResection(const SofaResectionCommand& cmd) {
    auto node = findNode(cmd.bone_name);
    if (!node) return -1;

    auto topo = node->get<TetrahedronSetTopologyContainer>();
    auto modifier = node->get<TetrahedronSetTopologyModifier>();
    auto mobj = node->get<MechanicalObject<Vec3Types>>();

    if (!topo || !modifier || !mobj) return -2;

    // Identify tetrahedra to remove
    sofa::type::Vec3d planePoint(cmd.plane_point[0], cmd.plane_point[1], cmd.plane_point[2]);
    sofa::type::Vec3d planeNormal(cmd.plane_normal[0], cmd.plane_normal[1], cmd.plane_normal[2]);
    planeNormal.normalize();

    const auto& tetras = topo->getTetrahedronArray();
    const auto& positions = mobj->x.getValue();

    std::vector<sofa::Index> toRemove;
    for (sofa::Index i = 0; i < tetras.size(); ++i) {
        const auto& tet = tetras[i];
        bool allBelowPlane = true;
        for (int v = 0; v < 4; ++v) {
            auto diff = positions[tet[v]] - planePoint;
            if (diff * planeNormal >= 0) {
                allBelowPlane = false;
                break;
            }
        }
        if (allBelowPlane) {
            toRemove.push_back(i);
        }
    }

    if (toRemove.empty()) return 0;

    // Remove tetrahedra (triggers topological mapping updates)
    modifier->removeTetrahedra(toRemove, true);

    // Flag topology change
    _topologyDirty[cmd.bone_name] = true;

    return static_cast<int>(toRemove.size());
}
```

### 8.2 Unity Side

**Unity Expert:** On the Unity side, resection is a two-step process: EzySlice for visual preview (instant), then SOFA for physics-accurate topology update.

```csharp
// ResectionEngine integration with custom bridge
public class ResectionEngine : MonoBehaviour
{
    [SerializeField] private SofaBridgeComponent sofaBridge;

    public ResectionResult ExecuteCut(BoneType target, Plane cutPlane)
    {
        // 1. Visual cut with EzySlice (immediate feedback)
        var hull = targetObj.Slice(cutPlane.normal * cutPlane.distance, cutPlane.normal);

        // 2. Physics cut via SOFA bridge (may take a few ms)
        var cmd = new SofaResectionCommand {
            boneName = target.ToString(),
            planePoint = cutPlane.ClosestPointOnPlane(Vector3.zero),
            planeNormal = cutPlane.normal
        };
        int removedCount = sofaBridge.Simulation.ExecuteResection(cmd);

        // 3. On next frame, check for topology change and rebuild mesh
        // (handled by SofaMeshSync component)

        return new ResectionResult { success = removedCount > 0 };
    }
}
```

---

## 9. Build & Deployment

### 9.1 Directory Structure

```
unity-sofa-integration/
├── native/                          # C++ native plugin project
│   ├── CMakeLists.txt
│   ├── src/
│   │   ├── bridge_api.h
│   │   ├── bridge_api.cpp
│   │   ├── sofa_simulation.h
│   │   ├── sofa_simulation.cpp
│   │   ├── scene_builder.h
│   │   ├── scene_builder.cpp
│   │   ├── data_transfer.h
│   │   ├── data_transfer.cpp
│   │   ├── command_handler.h
│   │   ├── command_handler.cpp
│   │   └── thread_manager.h
│   ├── include/
│   │   └── sofa_ankle_bridge.h      # Public C API header
│   └── tests/
│       └── test_bridge.cpp          # Native unit tests (Google Test)
│
├── unity-project/
│   ├── Assets/
│   │   ├── AnkleSim/
│   │   │   ├── Bridge/              # C# bridge layer
│   │   │   │   ├── SofaNativeBridge.cs
│   │   │   │   ├── SofaSimulation.cs
│   │   │   │   ├── SofaMeshTransfer.cs
│   │   │   │   ├── SofaBridgeComponent.cs   # MonoBehaviour wrapper
│   │   │   │   └── NativeStructs.cs         # C# struct mirrors
│   │   │   └── ... (rest of application)
│   │   ├── Plugins/
│   │   │   └── x86_64/
│   │   │       ├── SofaAnkleBridge.dll       # Our custom plugin
│   │   │       ├── Sofa.Core.dll             # SOFA core libs
│   │   │       ├── Sofa.Simulation.Graph.dll
│   │   │       └── Sofa.Component.*.dll      # SOFA component libs
│   │   └── StreamingAssets/
│   │       └── SOFA/
│   │           └── meshes/                    # VTK volumetric meshes
│   └── Tests/
│       ├── EditMode/
│       ├── PlayMode/
│       └── SOFA/                              # pytest-based
│
└── docs/
```

### 9.2 Build Pipeline

**Integration Expert:** The build pipeline is:

1. **Build SOFA from source** (or use pre-built binaries) with required plugins
2. **Build SofaAnkleBridge.dll** with CMake, linking against SOFA libraries
3. **Copy all DLLs** (our plugin + SOFA libraries) to Unity's `Assets/Plugins/x86_64/`
4. **Unity builds** as normal

```bash
# Step 1: Build SOFA (one-time setup)
cd sofa-build && cmake -DCMAKE_BUILD_TYPE=Release \
    -DSOFA_BUILD_TESTS=OFF \
    -DPLUGIN_SOFACARVING=ON \
    -DPLUGIN_CGALPLUGIN=ON \
    ../sofa-src && cmake --build . -j$(nproc)

# Step 2: Build our bridge
cd native && cmake -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_PREFIX_PATH=/path/to/sofa-build \
    -B build && cmake --build build -j$(nproc)

# Step 3: Deploy
cp build/bin/SofaAnkleBridge.dll ../unity-project/Assets/Plugins/x86_64/
cp /path/to/sofa-build/lib/*.dll ../unity-project/Assets/Plugins/x86_64/
```

---

## 10. Testing Strategy for Custom Bridge

### 10.1 Native C++ Tests (Google Test)

```cpp
// test_bridge.cpp
TEST(SofaBridge, InitAndShutdown) {
    ASSERT_EQ(sofa_bridge_init("./plugins"), 0);
    sofa_bridge_shutdown();
}

TEST(SofaBridge, CreateEmptySceneAndStep) {
    sofa_bridge_init("./plugins");
    SofaSceneConfig config = default_config();
    ASSERT_EQ(sofa_scene_create(&config), 0);
    ASSERT_EQ(sofa_scene_finalize(), 0);
    ASSERT_EQ(sofa_step(0.01f), 0);
    sofa_bridge_shutdown();
}

TEST(SofaBridge, AddRigidBoneAndReadPositions) {
    // ... setup ...
    SofaRigidBoneConfig bone = make_test_bone();
    ASSERT_EQ(sofa_add_rigid_bone(&bone), 0);
    sofa_scene_finalize();
    sofa_step(0.01f);

    float positions[3];
    int count = sofa_get_positions("TestBone", positions, 1);
    ASSERT_EQ(count, 1);
    sofa_bridge_shutdown();
}

TEST(SofaBridge, ResectionRemovesTetrahedra) {
    // ... setup scene with deformable bone ...
    SofaResectionCommand cmd = { "Tibia", {0,0,0}, {0,1,0} };
    int removed = sofa_execute_resection(&cmd);
    ASSERT_GT(removed, 0);
    ASSERT_TRUE(sofa_has_topology_changed("Tibia"));
}

TEST(SofaBridge, AsyncStepCompletesWithoutDeadlock) {
    // ... setup ...
    ASSERT_EQ(sofa_step_async(), 0);
    // Poll with timeout
    auto deadline = now() + 5s;
    while (!sofa_step_async_is_complete() && now() < deadline) {
        std::this_thread::sleep_for(1ms);
    }
    ASSERT_TRUE(sofa_step_async_is_complete());
}
```

### 10.2 C# Integration Tests (Unity Test Framework)

```csharp
[UnityTest] public IEnumerator Bridge_InitAndShutdown_Succeeds()
[UnityTest] public IEnumerator Bridge_CreateScene_WithValidConfig_Succeeds()
[UnityTest] public IEnumerator Bridge_AddRigidBone_CreatesNode()
[UnityTest] public IEnumerator Bridge_AsyncStep_CompletesWithinTimeout()
[UnityTest] public IEnumerator Bridge_GetPositions_ReturnsCorrectCount()
[UnityTest] public IEnumerator Bridge_MeshSync_TransfersPositionsToUnityMesh()
[UnityTest] public IEnumerator Bridge_Resection_TriggersTopologyChange()
[UnityTest] public IEnumerator Bridge_MeshSync_CompletesUnder2ms()
[UnityTest] public IEnumerator Bridge_SolverDivergence_IsDetected()
[UnityTest] public IEnumerator Bridge_Shutdown_CanReinitialize()
```

---

## 11. Risk Assessment

| Risk | Severity | Mitigation |
|------|----------|------------|
| SOFA C++ API breaks between versions | High | Pin to SOFA v25.12; version-check in `sofa_bridge_init()` |
| DLL dependency hell (SOFA has 50+ libs) | High | Script to collect all transitive deps; test deployment early |
| Thread safety bugs | High | Single-mutex model; extensive async step tests |
| Topology changes crash SOFA | Medium | Wrap all topology ops in try/catch; validate mesh before modification |
| CMake/SOFA build complexity | Medium | Dockerized build environment; pre-built SOFA binaries as fallback |
| Unity editor DLL locking | Medium | Editor restart required for native plugin updates during dev |
| Platform differences (Windows/Linux) | Medium | CI builds on both platforms; abstract platform-specific code |
| Performance regression vs SofaUnity | Low | SofaUnity has similar overhead; our direct API may be faster |

---

## 12. Agent Agreement Summary

**Unity Expert:** I'm satisfied with the NativeArray-based mesh sync and Burst jobs approach. The double-buffer decoupling is essential for maintaining frame rate.

**SOFA Expert:** The domain-specific C API is the right call — it keeps the SOFA scene graph manipulation inside the C++ layer where it belongs. Exposing raw SOFA component APIs through C would be unmaintainable.

**Integration Expert:** The hybrid architecture (custom C++ plugin with flat C API) gives us the performance of in-process integration with full control. The thread safety model is simple enough to reason about correctly. The main risk is build system complexity, which we mitigate with CI and deployment scripts.

All three agents agree on this architecture. Key design principles:

1. **Thin C API** — Domain-specific, not a general SOFA wrapper
2. **SOFA stays in C++** — No leaking SOFA types across the boundary
3. **NativeArray mesh sync** — Burst-compatible, zero-GC data path
4. **Double-buffered async stepping** — Decouples simulation rate from render rate
5. **Topology change signaling** — Explicit dirty flags for mesh rebuild triggers
