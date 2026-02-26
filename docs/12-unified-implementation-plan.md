# Unified Implementation Plan — Best Practices

> Definitive plan for the Unity-SOFA Ankle ROM Simulation.
> Supersedes: [09 - TDD Implementation Plan], [10 - Custom Integration Plan], [11 - Agent Review & Negotiation].
>
> Uses a **custom C++ native plugin** (`SofaAnkleBridge`) with a flat C API — no SofaUnity or off-the-shelf middleware.
> Tests use Unity Test Framework (NUnit) for C#, Google Test for native C++, and pytest for standalone SOFA validation.

---

## Part 1: Architecture & Design

### 1.1 Why Custom?

| Approach | Why Rejected |
|----------|-------------|
| **SofaUnity plugin** (InfinyTech3D) | Black box API, no control over data paths, locked to their release cadence, licensing ambiguity for advanced features, MonoBehaviour-heavy design conflicts with Unity 6 best practices |
| **Pure ZMQ cross-process** | ~50-100µs per message, JSON serialization overhead for mesh data, process management complexity |
| **Pure shared memory** | Complex synchronization, no structured command/response, hard to debug |
| **SofaPython3 embedded** | Python GIL, fragile embedding, version conflicts |

**Selected: Custom C++ Native Plugin (In-Process)** — ~1µs per call with full API control.

### 1.2 System Architecture

```
Unity 6 Process
├── C# Managed Layer (AnkleSim.Bridge assembly)
│   ├── SofaNativeBridge.cs        ← P/Invoke declarations [DllImport("SofaAnkleBridge")]
│   ├── NativeStructs.cs           ← C# struct mirrors [StructLayout(Sequential)]
│   ├── SofaSimulation.cs          ← High-level C# API (lifecycle, commands, error handling)
│   ├── SofaBridgeComponent.cs     ← MonoBehaviour [DefaultExecutionOrder(-100)]
│   ├── SofaMeshTransfer.cs        ← NativeArray-based mesh sync + Burst jobs
│   └── SofaBridgeException.cs     ← Domain-specific exceptions
│
├── Native Plugin (SofaAnkleBridge.dll/.so)  ← Custom C++ DLL
│   ├── bridge_api.h/.cpp          ← Flat C API (extern "C")
│   ├── sofa_simulation.h/.cpp     ← SOFA lifecycle + PluginManager loading
│   ├── scene_builder.h/.cpp       ← Programmatic scene construction
│   ├── data_transfer.h/.cpp       ← Triple-buffered snapshot + double-buffered mesh
│   ├── command_handler.h/.cpp     ← Resection, implant, constraint commands
│   └── thread_manager.h/.cpp      ← Background stepping thread + cancellation
│
└── SOFA Libraries (linked at build time)
    ├── Sofa.Core, Sofa.Simulation.Graph
    ├── Sofa.Component.* (FEM, Collision, Constraint, Topology, ...)
    └── Optional plugins (SofaCarving, CGALPlugin)
```

### 1.3 Key Design Principles

1. **Thin C API** — Domain-specific functions, not a general SOFA wrapper
2. **SOFA stays in C++** — No SOFA types leak across the native boundary
3. **NativeArray mesh sync** — Burst-compiled jobs, zero GC allocations
4. **Async stepping with safe buffers** — Triple-buffered snapshots, double-buffered mesh with reader-count guard
5. **Topology change signaling** — Explicit dirty flags trigger mesh rebuild
6. **Emergent joint model** — Ankle joint emerges from ligament springs + bone-bone contact
7. **Dual-representation resection** — EzySlice visual cut + SOFA centroid-based physics cut
8. **Scene rebuild for undo** — Simpler and more robust than SOFA topology undo

---

### 1.4 C API Design

#### 1.4.1 Lifecycle

```c
// === Version ===
typedef struct {
    int bridge_version_major;  // bump on breaking API changes
    int bridge_version_minor;  // bump on additions
    int bridge_version_patch;  // bump on fixes
    int sofa_version_major;
    int sofa_version_minor;
} SofaBridgeVersion;

EXPORT SofaBridgeVersion sofa_bridge_get_version();

// === Lifecycle ===
EXPORT int         sofa_bridge_init(const char* plugin_dir);  // loads SOFA plugins
EXPORT void        sofa_bridge_shutdown();                     // join worker thread, cleanup
EXPORT const char* sofa_bridge_get_error();                    // last error string (thread-local)
```

#### 1.4.2 Scene Construction

```c
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

typedef struct {
    const char* name;
    const float* collision_vertices;   // flattened [x,y,z, ...]
    int          collision_vertex_count;
    const int*   collision_triangles;  // flattened [i0,i1,i2, ...]
    int          collision_triangle_count;
    float        position[3];
    float        orientation[4];       // quaternion [x,y,z,w]
    float        mass;
    int          is_fixed;             // 1 = pinned in space
} SofaRigidBoneConfig;

typedef struct {
    const char*  name;
    const char*  volumetric_mesh_path;   // VTK file path
    float        young_modulus;          // MPa
    float        poisson_ratio;
    float        total_mass;
    const char*  fem_method;            // "small", "large", "polar"
} SofaDeformableConfig;

typedef struct {
    const char* name;
    const char* bone_a_name;
    const char* bone_b_name;
    int         attachment_index_a;
    int         attachment_index_b;
    float       toe_stiffness;         // N/mm (low, for strain < threshold)
    float       linear_stiffness;      // N/mm (full stiffness above threshold)
    float       toe_region_strain;     // e.g. 0.03 = 3% strain
    float       damping;
    float       rest_length;           // mm
} SofaLigamentConfig;

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

EXPORT int  sofa_scene_create(const SofaSceneConfig* config);
EXPORT void sofa_scene_destroy();
EXPORT int  sofa_scene_is_ready();
EXPORT int  sofa_scene_finalize();     // call after adding all objects, before stepping

EXPORT int sofa_add_rigid_bone(const SofaRigidBoneConfig* config);
EXPORT int sofa_add_deformable_tissue(const SofaDeformableConfig* config);
EXPORT int sofa_add_ligament(const SofaLigamentConfig* config);
EXPORT int sofa_add_rigid_implant(const SofaImplantConfig* config);
```

#### 1.4.3 Simulation Control

```c
EXPORT int   sofa_step(float dt);              // blocking single step
EXPORT int   sofa_step_async();                // non-blocking, starts step on bg thread
EXPORT int   sofa_step_async_is_complete();    // poll for completion
EXPORT void  sofa_step_async_wait();           // block until bg step finishes
EXPORT void  sofa_reset();
```

#### 1.4.4 Batched Frame Snapshot (reduces P/Invoke from ~10 to 2 per frame)

```c
typedef struct {
    // Bone positions (rigid bodies): pos(3) + quat(4) = 7 floats each
    float tibia_position[7];
    float talus_position[7];
    float fibula_position[7];
    float calcaneus_position[7];

    // Joint angles (degrees) — computed from relative tibia/talus orientation
    float dorsiflexion_angle;
    float plantarflexion_angle;
    float inversion_angle;
    float eversion_angle;

    // Joint torques (Nm)
    float sagittal_torque;
    float frontal_torque;

    // Simulation status
    float step_time_ms;
    int   solver_diverged;
    int   topology_changed_flags;  // bitmask per bone

    // Deformable mesh vertex counts (for pre-allocating readback buffers)
    int   cartilage_vertex_count;
} SofaFrameSnapshot;

EXPORT int sofa_get_frame_snapshot(SofaFrameSnapshot* out);
```

#### 1.4.5 Mesh Data Readback

```c
// Full position readback (for deformable meshes)
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

// Post-resection: full surface mesh (topology changed)
EXPORT int sofa_get_surface_mesh(const char* node_name,
                                  float* out_vertices, int* out_vertex_count,
                                  int*   out_triangles, int* out_triangle_count,
                                  int vertex_capacity, int triangle_capacity);

EXPORT int sofa_has_topology_changed(const char* node_name);
```

#### 1.4.6 Commands

```c
typedef struct {
    const char* bone_name;
    float       plane_point[3];
    float       plane_normal[3];
} SofaResectionCommand;

EXPORT int sofa_execute_resection(const SofaResectionCommand* cmd);
EXPORT int sofa_get_removed_element_count(const char* bone_name);

EXPORT int sofa_set_implant_position(const char* implant_name,
                                      float position[3], float orientation[4]);

EXPORT int sofa_apply_joint_torque(const char* joint_name,
                                    int axis, float torque_nm);

EXPORT int sofa_set_material_property(const char* node_name,
                                       const char* property_name, float value);
```

#### 1.4.7 Profiling

```c
typedef struct {
    float scene_step_ms;
    float collision_ms;
    float fem_solve_ms;
    float constraint_solve_ms;
    float mesh_readback_ms;
    int   collision_contacts;
    int   constraint_iterations;
} SofaProfilingData;

EXPORT int sofa_get_profiling_data(SofaProfilingData* out);
```

---

### 1.5 Thread Safety Model

```
Timeline:
  Unity Main Thread:     [ReadSnapshot] [ScheduleJobs] [StartStep] --------- [ReadSnapshot] ...
  SOFA Worker Thread:                                    [ProcessCmds → Step]
```

**Rules:**
1. Never access SOFA objects while a step is in progress
2. Data readback only after `sofa_step_async_is_complete()` returns true
3. Commands are mutex-guarded `std::vector<Command>`, executed by worker before next step
4. Frame snapshot: **triple-buffered** (tiny struct, zero latency concern)
5. Deformable mesh: **double-buffered** with atomic reader-count guard

```cpp
// Triple buffer for frame snapshot (~200 bytes)
class TripleBuffer<T> {
    T _buffers[3];
    std::atomic<int> _readIdx{0}, _writeIdx{1}, _latestIdx{2};
public:
    const T& read()  { _readIdx = _latestIdx.load(); return _buffers[_readIdx]; }
    T& write()       { return _buffers[_writeIdx]; }
    void publish()   { _latestIdx.store(_writeIdx); _writeIdx = 3 - _readIdx - _latestIdx; }
};

// Double buffer with reader guard for mesh positions
class MeshDoubleBuffer {
    std::vector<float> _buffers[2];
    std::atomic<int> _front{0};
    std::atomic<int> _readerCount{0};
public:
    float* beginRead()  { _readerCount++; return _buffers[_front].data(); }
    void   endRead()    { _readerCount--; }
    float* getWriteBuffer() { return _buffers[1 - _front].data(); }
    void   swap() {
        while (_readerCount.load() > 0) { /* spin-wait, rarely hit */ }
        _front.store(1 - _front.load());
    }
};
```

**Shutdown:** Join with 5-second timeout + cancellation token. Detach only as last resort on shutdown.

---

### 1.6 C# Managed Layer

#### P/Invoke Declarations

```csharp
internal static class SofaNativeBridge
{
    private const string DLL = "SofaAnkleBridge";

    [DllImport(DLL)] internal static extern SofaBridgeVersion sofa_bridge_get_version();
    [DllImport(DLL)] internal static extern int sofa_bridge_init(
        [MarshalAs(UnmanagedType.LPStr)] string pluginDir);
    [DllImport(DLL)] internal static extern void sofa_bridge_shutdown();
    [DllImport(DLL)] internal static extern IntPtr sofa_bridge_get_error();

    [DllImport(DLL)] internal static extern int sofa_scene_create(ref SofaSceneConfig config);
    [DllImport(DLL)] internal static extern int sofa_add_rigid_bone(ref SofaRigidBoneConfig config);
    [DllImport(DLL)] internal static extern int sofa_add_deformable_tissue(ref SofaDeformableConfig config);
    [DllImport(DLL)] internal static extern int sofa_add_ligament(ref SofaLigamentConfig config);
    [DllImport(DLL)] internal static extern int sofa_add_rigid_implant(ref SofaImplantConfig config);
    [DllImport(DLL)] internal static extern int sofa_scene_finalize();

    [DllImport(DLL)] internal static extern int sofa_step(float dt);
    [DllImport(DLL)] internal static extern int sofa_step_async();
    [DllImport(DLL)] internal static extern int sofa_step_async_is_complete();

    [DllImport(DLL)] internal static extern int sofa_get_frame_snapshot(ref SofaFrameSnapshot snapshot);
    [DllImport(DLL)] internal static extern unsafe int sofa_get_positions(
        [MarshalAs(UnmanagedType.LPStr)] string nodeName,
        float* outBuffer, int bufferCapacity);
    [DllImport(DLL)] internal static extern int sofa_get_profiling_data(ref SofaProfilingData data);
}
```

#### High-Level API

```csharp
public class SofaSimulation : IDisposable
{
    private bool _initialized;
    private NativeArray<float> _positionBuffer;

    public bool Initialize(string pluginDir)
    {
        var version = SofaNativeBridge.sofa_bridge_get_version();
        if (version.bridge_version_major != ExpectedMajor)
            throw new SofaVersionMismatchException(version);

        int result = SofaNativeBridge.sofa_bridge_init(pluginDir);
        if (result != 0) throw new SofaBridgeException(GetLastError());
        _initialized = true;
        return true;
    }

    public void AddRigidBone(RigidBoneDescriptor desc) { /* marshal + call + check */ }
    public void AddDeformableTissue(DeformableTissueDescriptor desc) { /* marshal + call + check */ }
    public void AddLigament(LigamentDescriptor desc) { /* includes toe/linear stiffness */ }
    public void AddRigidImplant(ImplantDescriptor desc) { /* marshal vertex/triangle arrays */ }

    public SofaFrameSnapshot GetFrameSnapshot() { /* single P/Invoke, returns snapshot */ }
    public unsafe int GetPositions(string nodeName, NativeArray<Vector3> buffer) { /* direct ptr */ }

    public void Dispose() { if (_initialized) { SofaNativeBridge.sofa_bridge_shutdown(); } }
}
```

#### Burst-Compiled Mesh Sync Jobs

```csharp
[BurstCompile]
public struct ApplyPositionsJob : IJobParallelFor
{
    [ReadOnly] public NativeArray<float3> SofaPositions;
    public NativeArray<float3> UnityVertices;
    public float ScaleFactor;  // SOFA mm → Unity m (if applicable)

    public void Execute(int index) {
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

    public void Execute() {
        for (int i = 0; i < ChangeCount; i++) {
            var c = Changes[i];
            UnityVertices[c.Index] = new float3(c.X, c.Y, c.Z) * ScaleFactor;
        }
    }
}
```

---

### 1.7 Unity Frame Execution Order

```
Unity Frame:
  1. FixedUpdate:  [DefaultExecutionOrder(-100)] SofaBridgeComponent
     - Check if async step completed
     - If yes: read frame snapshot, schedule mesh sync job, kick next async step
     - If no: skip (SOFA still computing, use last frame's data)

  2. Update:  Application logic
     - ROMEngine reads angles from cached snapshot
     - WorkflowManager processes state transitions
     - UI updates

  3. LateUpdate:  Rendering prep
     - JobHandle.Complete() on mesh sync
     - Apply vertex data to Unity Mesh via Advanced Mesh API
     - Render
```

---

### 1.8 SOFA Scene Construction (C++)

#### Root Node

```cpp
bool SofaSimulationManager::createScene(const SofaSceneConfig& config) {
    _root = sofa::simulation::node::createRootNode("root");
    _root->setGravity({config.gravity[0], config.gravity[1], config.gravity[2]});
    _root->setDt(config.timestep);

    _root->addObject<FreeMotionAnimationLoop>();

    auto cs = _root->addObject<GenericConstraintSolver>();
    cs->d_maxIt.setValue(config.constraint_iterations);
    cs->d_tolerance.setValue(config.constraint_tolerance);

    _root->addObject<DefaultPipeline>();
    _root->addObject<BruteForceBroadPhase>();
    _root->addObject<BVHNarrowPhase>();

    auto intersection = _root->addObject<MinProximityIntersection>();
    intersection->d_alarmDistance.setValue(config.alarm_distance);
    intersection->d_contactDistance.setValue(config.contact_distance);

    auto cm = _root->addObject<DefaultContactManager>();
    cm->d_response.setValue("FrictionContact");
    return true;
}
```

#### Rigid Bone Node

```cpp
int SofaSimulationManager::addRigidBone(const SofaRigidBoneConfig& config) {
    auto node = _root->addChild(config.name);

    node->addObject<EulerImplicitSolver>();
    node->addObject<CGLinearSolver>();

    auto mobj = node->addObject<MechanicalObject<Rigid3Types>>();
    // set initial position + orientation from config

    auto mass = node->addObject<UniformMass<Rigid3Types>>();
    mass->d_totalMass.setValue(config.mass);

    if (config.is_fixed) {
        auto fix = node->addObject<FixedConstraint<Rigid3Types>>();
        fix->d_indices.setValue({0});
    }

    node->addObject<UncoupledConstraintCorrection<Rigid3Types>>();

    // Collision sub-node with RigidMapping
    auto colNode = node->addChild("Collision");
    // ... load collision mesh, add TriangleCollisionModel + RigidMapping ...

    _nodeCache[config.name] = node.get();
    return 0;
}
```

#### Deformable Tissue (Cartilage)

```cpp
int SofaSimulationManager::addDeformableTissue(const SofaDeformableConfig& config) {
    auto node = _root->addChild(config.name);

    node->addObject<EulerImplicitSolver>();
    node->addObject<SparseLDLSolver<CompressedRowSparseMatrixd>>();

    auto loader = node->addObject<MeshVTKLoader>();
    loader->d_filename.setValue(config.volumetric_mesh_path);
    loader->load();

    auto topo = node->addObject<TetrahedronSetTopologyContainer>();
    topo->d_src.setValue("@loader");
    node->addObject<TetrahedronSetTopologyModifier>();
    node->addObject<TetrahedronSetGeometryAlgorithms<Vec3Types>>();

    auto mobj = node->addObject<MechanicalObject<Vec3Types>>();

    auto mass = node->addObject<MeshMatrixMass<Vec3Types>>();
    mass->d_totalMass.setValue(config.total_mass);

    auto fem = node->addObject<TetrahedronFEMForceField<Vec3Types>>();
    fem->d_method.setValue(config.fem_method);
    fem->d_youngModulus.setValue(config.young_modulus);
    fem->d_poissonRatio.setValue(config.poisson_ratio);

    node->addObject<GenericConstraintCorrection>();

    // Collision sub-node with topological mapping
    auto colNode = node->addChild("Collision");
    colNode->addObject<TriangleSetTopologyContainer>();
    colNode->addObject<TriangleSetTopologyModifier>();
    colNode->addObject<Tetra2TriangleTopologicalMapping>();
    colNode->addObject<MechanicalObject<Vec3Types>>();
    colNode->addObject<TriangleCollisionModel<Vec3Types>>();
    colNode->addObject<LineCollisionModel<Vec3Types>>();
    colNode->addObject<PointCollisionModel<Vec3Types>>();
    colNode->addObject<BarycentricMapping<Vec3Types, Vec3Types>>();

    _nodeCache[config.name] = node.get();
    return 0;
}
```

#### Bilinear Ligament Springs

```cpp
int SofaSimulationManager::addLigament(const SofaLigamentConfig& config) {
    // Create spring force field between two rigid body attachment points
    // Store config for runtime stiffness modulation
    _ligaments.push_back({config, springFF});
    return 0;
}

// Called by worker thread before each SOFA animate():
void SofaSimulationManager::updateLigamentStiffness() {
    for (auto& lig : _ligaments) {
        float currentLength = computeSpringLength(lig);
        float strain = (currentLength - lig.config.rest_length) / lig.config.rest_length;

        float k = (strain < lig.config.toe_region_strain)
            ? lig.config.toe_stiffness     // toe region: slack
            : lig.config.linear_stiffness; // linear region: stiff

        auto& springs = lig.springFF->d_springsValue.beginEdit();
        springs[0].ks = k;
        lig.springFF->d_springsValue.endEdit();
    }
}
```

#### SOFA Plugin Loading (in `sofa_bridge_init`)

```cpp
bool SofaSimulationManager::init(const std::string& pluginDir) {
    sofa::simulation::graph::init();
    auto& pm = sofa::helper::system::PluginManager::getInstance();

    // Required plugins
    const std::vector<std::string> required = {
        "Sofa.Component.StateContainer",
        "Sofa.Component.SolidMechanics.FEM.Elastic",
        "Sofa.Component.SolidMechanics.Spring",
        "Sofa.Component.Mass",
        "Sofa.Component.ODESolver.Backward",
        "Sofa.Component.LinearSolver.Direct",
        "Sofa.Component.LinearSolver.Iterative",
        "Sofa.Component.Constraint.Lagrangian.Solver",
        "Sofa.Component.Constraint.Lagrangian.Correction",
        "Sofa.Component.Constraint.Projective",
        "Sofa.Component.Collision.Detection.Algorithm",
        "Sofa.Component.Collision.Detection.Intersection",
        "Sofa.Component.Collision.Geometry",
        "Sofa.Component.Collision.Response.Contact",
        "Sofa.Component.AnimationLoop",
        "Sofa.Component.Mapping.Linear",
        "Sofa.Component.Topology.Container.Dynamic",
        "Sofa.Component.Topology.Mapping",
        "Sofa.Component.IO.Mesh",
    };

    for (const auto& p : required)
        if (!pm.loadPlugin(p)) { _lastError = "Missing: " + p; return false; }

    // Optional
    pm.loadPlugin("SofaCarving");
    pm.loadPlugin("CGALPlugin");

    return true;
}
```

---

### 1.9 Emergent Joint Model

The ankle joint is **not** modeled as a first-class SOFA joint. Instead:

1. **Tibia** = fixed rigid body
2. **Talus** = free rigid body
3. **Ligament springs** (ATFL, CFL, deltoid, etc.) constrain relative motion
4. **Bone-bone contact** (collision pipeline) prevents interpenetration
5. **ROM sweep** = `ConstantForceField` torque applied to talus
6. **Joint angle** = relative quaternion decomposition of talus orientation w.r.t. tibia

```cpp
float SofaSimulationManager::getJointAngle(const std::string& jointName, int axis) {
    auto tibiaMO = findNode("Tibia")->get<MechanicalObject<Rigid3Types>>();
    auto talusMO = findNode("Talus")->get<MechanicalObject<Rigid3Types>>();

    auto tibiaOri = tibiaMO->x.getValue()[0].getOrientation();
    auto talusOri = talusMO->x.getValue()[0].getOrientation();

    auto relOri = tibiaOri.inverse() * talusOri;
    auto euler = relOri.toEulerVector();  // radians
    return euler[axis] * 180.0 / M_PI;
}
```

This is biomechanically realistic — the ankle's instantaneous rotation axis shifts during the ROM arc, which this model captures naturally.

---

### 1.10 Dual-Representation Resection

| Layer | Method | Purpose |
|-------|--------|---------|
| Visual | EzySlice plane cut | Instant, clean geometry for user feedback |
| Physics | SOFA centroid-based tetrahedral removal | Accurate FEM topology change for simulation |

**SOFA side:** Remove tetrahedra whose centroid falls below the cut plane (smoother boundary than all-vertices-below).

```cpp
int SofaSimulationManager::executeResection(const SofaResectionCommand& cmd) {
    auto topo = findNode(cmd.bone_name)->get<TetrahedronSetTopologyContainer>();
    auto modifier = findNode(cmd.bone_name)->get<TetrahedronSetTopologyModifier>();
    auto mobj = findNode(cmd.bone_name)->get<MechanicalObject<Vec3Types>>();

    Vec3d planePoint(cmd.plane_point), planeNormal(cmd.plane_normal);
    planeNormal.normalize();

    std::vector<sofa::Index> toRemove;
    for (sofa::Index i = 0; i < topo->getTetrahedronArray().size(); ++i) {
        // Compute centroid of tetrahedron
        Vec3d centroid = (positions[tet[0]] + positions[tet[1]] +
                          positions[tet[2]] + positions[tet[3]]) * 0.25;
        if ((centroid - planePoint) * planeNormal < 0)
            toRemove.push_back(i);
    }

    modifier->removeTetrahedra(toRemove, true);  // triggers Tetra2TriangleTopologicalMapping
    _topologyDirty[cmd.bone_name] = true;
    return toRemove.size();
}
```

**Unity side:** EzySlice for immediate visual feedback; on next frame, `sofa_has_topology_changed()` triggers full mesh rebuild via `sofa_get_surface_mesh()` + Advanced Mesh API (`Mesh.SetVertexBufferData` with `NativeArray`).

**Undo:** Rebuild entire SOFA scene from cached configuration via `sofa_scene_destroy()` + `sofa_scene_create()`.

---

### 1.11 Error Handling Strategy

| Level | Mechanism | Example |
|-------|-----------|---------|
| C++ internal | `try/catch` around all SOFA calls | Solver divergence, topology crash |
| C API | Return codes (0 = OK, negative = error) | `-1` = node not found |
| C# bridge | Check return + throw `SofaBridgeException` | Includes native error message |
| C# high-level | Domain-specific exceptions | `SolverDivergedException`, `TopologyException` |

---

## Part 2: TDD Sprint Plan

### Testing Strategy

#### Test Pyramid

```
         /  E2E Tests  \          <- 5-10 workflow integration tests
        / Integration    \        <- 15-20 component interaction tests
       / Unit Tests        \      <- 50-80 pure logic tests (C# + C++)
      /____________________\
```

#### Test Categories

| Category | Framework | Location | What It Tests |
|----------|-----------|----------|---------------|
| **Unit (C#)** | NUnit + Unity Test Framework | `Tests/EditMode/` | Pure logic: angle math, alignment, data models |
| **Unit (C++)** | Google Test | `native/tests/` | Native bridge: SOFA lifecycle, scene construction, data readback, threading |
| **Unit (SOFA)** | pytest + SofaPython3 | `Tests/SOFA/` | Standalone SOFA scenes: material properties, solver convergence, ROM ranges |
| **Integration** | Unity Test Framework (Play Mode) | `Tests/PlayMode/` | Component interactions via P/Invoke |
| **E2E** | Unity Test Framework (Play Mode) | `Tests/PlayMode/E2E/` | Full surgical workflow |

#### Test Naming Convention
```
[MethodUnderTest]_[Scenario]_[ExpectedBehavior]
```

---

### Sprint 0: Project Scaffolding & Native Bridge Foundation (Weeks 1-2)

> 2 weeks because we're building the native C++ bridge from scratch.

#### 0.1 Unity Project Setup

**Tests First:**
```
TEST: Project compiles with zero errors
TEST: Assembly definitions resolve correctly
TEST: Required packages installed (TextMeshPro, InputSystem, Addressables)
```

**Implementation:**
- Unity 6 project with URP
- Assembly definitions: `AnkleSim.Core`, `AnkleSim.Runtime`, `AnkleSim.Bridge`, `AnkleSim.Editor`, `AnkleSim.Tests.EditMode`, `AnkleSim.Tests.PlayMode`
- Physics: TGS solver, 200Hz fixed timestep (0.005s)

#### 0.2 Native Plugin Build System

**Tests (C++ / Google Test):**
```cpp
TEST(Build, NativePluginLoads) {
    void* handle = dlopen("SofaAnkleBridge.so", RTLD_LAZY);
    ASSERT_NE(handle, nullptr);
    auto init_fn = dlsym(handle, "sofa_bridge_init");
    ASSERT_NE(init_fn, nullptr);
    dlclose(handle);
}
```

**Implementation:**
- CMake project in `native/`
- SOFA library linking
- Build `SofaAnkleBridge.dll/.so`
- Deploy script to copy DLLs to `Assets/Plugins/x86_64/`
- Dependency walker script (`collect_sofa_deps.sh`) for transitive SOFA DLLs

#### 0.3 Native Bridge Lifecycle

**Tests (C++ / Google Test):**
```cpp
TEST(SofaBridge, Init_WithValidPluginDir_ReturnsSuccess)
TEST(SofaBridge, Init_WithInvalidPluginDir_ReturnsError)
TEST(SofaBridge, Shutdown_AfterInit_Succeeds)  // run under valgrind/ASAN
TEST(SofaBridge, VersionHandshake_ReturnsValidVersion)
TEST(SofaBridge, CreateEmptyScene_AndStepOnce)
TEST(SofaBridge, AsyncStep_CompletesWithoutDeadlock)
TEST(SofaBridge, Shutdown_WhileAsyncStepRunning_DoesNotCrash)
TEST(SofaBridge, Shutdown_CanReinitializeAfter)
```

**Implementation:**
- `bridge_api.cpp` — flat C API wrapping `SofaSimulationManager`
- `sofa_simulation.cpp` — init (loads plugins), createScene (root node + pipeline), shutdown (join + cleanup)
- `thread_manager.cpp` — background thread with cancellation token, join-with-timeout

#### 0.4 C# P/Invoke Layer

**Tests (PlayMode):**
```csharp
[UnityTest] public IEnumerator NativeBridge_DllLoads_WithoutError()
[UnityTest] public IEnumerator NativeBridge_Init_ReturnsSuccess()
[UnityTest] public IEnumerator NativeBridge_VersionCheck_Passes()
[UnityTest] public IEnumerator NativeBridge_CreateScene_WithDefaultConfig()
[UnityTest] public IEnumerator NativeBridge_StepSync_Completes()
[UnityTest] public IEnumerator NativeBridge_StepAsync_CompleteWithinTimeout()
[UnityTest] public IEnumerator NativeBridge_Shutdown_ReleasesResources()
[UnityTest] public IEnumerator NativeBridge_Shutdown_CanReinitializeAfter()
```

**Implementation:**
- `SofaNativeBridge.cs` — P/Invoke declarations
- `NativeStructs.cs` — C# struct mirrors (`[StructLayout(LayoutKind.Sequential)]`)
- `SofaSimulation.cs` — high-level API with version check + error handling
- `SofaBridgeComponent.cs` — MonoBehaviour with `[DefaultExecutionOrder(-100)]`

#### 0.5 Directory Structure

```
project-root/
├── native/                              # C++ native plugin
│   ├── CMakeLists.txt
│   ├── include/
│   │   └── sofa_ankle_bridge.h          # Public C API header
│   ├── src/
│   │   ├── bridge_api.cpp
│   │   ├── sofa_simulation.h/.cpp
│   │   ├── scene_builder.h/.cpp
│   │   ├── data_transfer.h/.cpp
│   │   ├── command_handler.h/.cpp
│   │   └── thread_manager.h/.cpp
│   └── tests/
│       ├── test_lifecycle.cpp
│       ├── test_scene_construction.cpp
│       ├── test_data_transfer.cpp
│       ├── test_resection.cpp
│       └── test_threading.cpp
│
├── unity-project/
│   ├── Assets/
│   │   ├── AnkleSim/
│   │   │   ├── Core/                    # Assembly: AnkleSim.Core
│   │   │   │   ├── DataModels/
│   │   │   │   ├── Validation/
│   │   │   │   └── Math/
│   │   │   ├── Bridge/                  # Assembly: AnkleSim.Bridge
│   │   │   │   ├── SofaNativeBridge.cs
│   │   │   │   ├── NativeStructs.cs
│   │   │   │   ├── SofaSimulation.cs
│   │   │   │   ├── SofaBridgeComponent.cs
│   │   │   │   ├── SofaMeshTransfer.cs
│   │   │   │   └── SofaBridgeException.cs
│   │   │   ├── Runtime/                 # Assembly: AnkleSim.Runtime
│   │   │   │   ├── Anatomy/
│   │   │   │   ├── ROM/
│   │   │   │   ├── Resection/
│   │   │   │   ├── Implant/
│   │   │   │   ├── Comparison/
│   │   │   │   ├── Workflow/
│   │   │   │   └── UI/
│   │   │   ├── Editor/                  # Assembly: AnkleSim.Editor
│   │   │   ├── ScriptableObjects/
│   │   │   ├── Prefabs/
│   │   │   ├── Materials/
│   │   │   ├── Scenes/
│   │   │   └── StreamingAssets/SOFA/meshes/
│   │   ├── Plugins/
│   │   │   ├── x86_64/
│   │   │   │   ├── SofaAnkleBridge.dll
│   │   │   │   ├── Sofa.Core.dll
│   │   │   │   └── Sofa.Component.*.dll  (~80 files)
│   │   │   └── EzySlice/
│   │   └── Tests/
│   │       ├── EditMode/
│   │       ├── PlayMode/
│   │       └── SOFA/
│
├── scripts/
│   ├── build_native.sh
│   ├── deploy_dlls.sh
│   └── collect_sofa_deps.sh
│
└── docs/
```

---

### Sprint 1: Core Data Models & Anatomy Loading (Week 3)

#### 1.1 Data Models (RED → GREEN → REFACTOR)

**Tests (EditMode):**
```csharp
[Test] public void TotalSagittalArc_IsSum_OfDFAndPF()
[Test] public void ROMComparison_CalculatesImprovement_Correctly()
[Test] public void ROMRecord_DefaultValues_AreZero()
[Test] public void AlignmentMetrics_IsAcceptable_WhenAllWithinRange()
[Test] public void AlignmentMetrics_IsNotAcceptable_WhenTibiotalarExceeds10()
[Test] public void AlignmentMetrics_IsNotAcceptable_WhenADTAOutsideRange()
[Test] public void AlignmentMetrics_GeneratesWarning_ForEachViolation()
[Test] public void CoverageAnalysis_DetectsOverhang_WhenCoverageAbove100Percent()
[Test] public void ResectionRecord_StoresCorrectVolume()
```

**Implementation:** `ROMRecord`, `ROMComparison`, `AlignmentMetrics`, `CoverageAnalysis`, `ResectionRecord` — all in `AnkleSim.Core` (no Unity dependencies).

**Requirements:** REQ-ROM-06, REQ-ROM-08, REQ-IMP-04, REQ-IMP-05

#### 1.2 Alignment Validation (RED → GREEN → REFACTOR)

**Tests (EditMode):**
```csharp
[Test] public void Validate_TibiotalarAngle0_ReturnsAcceptable()
[Test] public void Validate_TibiotalarAngle11_ReturnsWarning()
[Test] public void Validate_ADTA89_ReturnsAcceptable()
[Test] public void Validate_ADTA85_ReturnsWarning()
[Test] public void Validate_PosteriorSlope6_ReturnsWarning()
[Test] public void Validate_AllGood_ReturnsNoWarnings()
[Test] public void Validate_MultipleViolations_ReturnsAllWarnings()
```

**Implementation:** `AlignmentValidator` static class with threshold constants from clinical data.

#### 1.3 Anatomy Manager (RED → GREEN → REFACTOR)

**Tests (PlayMode):**
```csharp
[UnityTest] public IEnumerator LoadAnatomy_CreatesAllBoneGameObjects()
[UnityTest] public IEnumerator LoadAnatomy_MeshesHaveCorrectVertexCount()
[UnityTest] public IEnumerator SetVisibility_HidesSpecifiedStructure()
[UnityTest] public IEnumerator SetVisibility_ShowsSpecifiedStructure()
[UnityTest] public IEnumerator GetBoneMesh_ReturnsMeshForValidBoneType()
[UnityTest] public IEnumerator GetBoneMesh_ReturnsNullForInvalidType()
```

**Implementation:** `AnatomyManager` MonoBehaviour + `AnatomyConfig` ScriptableObject.

**Requirements:** REQ-ANAT-01, REQ-ANAT-02, REQ-ANAT-04, REQ-ANAT-05

---

### Sprint 2: SOFA Scene Construction & Mesh Sync (Week 4)

#### 2.1 Native Scene Construction (RED → GREEN → REFACTOR)

**Tests (C++ / Google Test):**
```cpp
TEST(SceneBuilder, AddRigidBone_CreatesNodeWithRigidMechanicalObject)
TEST(SceneBuilder, AddRigidBone_Fixed_HasFixedConstraint)
TEST(SceneBuilder, AddRigidBone_Unfixed_HasNoFixedConstraint)
TEST(SceneBuilder, AddDeformableTissue_HasTetrahedronFEMForceField)
TEST(SceneBuilder, AddDeformableTissue_FEMHasCorrectYoungModulus)
TEST(SceneBuilder, AddDeformableTissue_HasTetra2TriangleMapping)
TEST(SceneBuilder, AddLigament_BilinearSpring_HasCorrectToeStiffness)
TEST(SceneBuilder, AddLigament_BilinearSpring_HasCorrectLinearStiffness)
TEST(SceneBuilder, AddRigidImplant_CreatesRigidNode)
TEST(SceneBuilder, Finalize_InitializesAllNodes)
TEST(SceneBuilder, Finalize_ThenStep_NoSolverDivergence)
TEST(SceneBuilder, FullAnkleScene_1000Steps_Stable)
```

**Tests (SOFA / pytest):**
```python
def test_rigid_bone_creation():
    """Rigid bone node has MechanicalObject<Rigid3d> and FixedConstraint."""
def test_deformable_tissue_creation():
    """Tissue node has TetrahedronFEMForceField with correct E and nu."""
def test_bilinear_ligament_toe_region():
    """Ligament at low strain uses toe stiffness, not linear stiffness."""
def test_bilinear_ligament_linear_region():
    """Ligament beyond toe threshold uses full linear stiffness."""
def test_collision_pipeline():
    """Scene has FreeMotionAnimationLoop and GenericConstraintSolver."""
def test_simulation_stability():
    """Run 1000 steps without solver divergence."""
def test_material_properties():
    """Cortical bone E=17000 MPa, cartilage E=1.0 MPa as configured."""
```

**Implementation (C++):** `scene_builder.cpp` — rigid bones, deformable tissue, bilinear ligaments, rigid implants. `finalizeScene()` calls `sofa::simulation::init(root)`.

**Requirements:** REQ-INT-06, REQ-INT-07, REQ-SIM-01, REQ-SIM-02, REQ-SIM-04

#### 2.2 C# Scene Construction API (RED → GREEN → REFACTOR)

**Tests (PlayMode):**
```csharp
[UnityTest] public IEnumerator AddRigidBone_CreatesSOFANode()
[UnityTest] public IEnumerator AddDeformableTissue_CreatesSOFANodeWithFEM()
[UnityTest] public IEnumerator AddLigament_CreatesBilinearSpring()
[UnityTest] public IEnumerator AddRigidImplant_CreatesRigidNode()
[UnityTest] public IEnumerator FinalizeScene_CanStepAfterFinalize()
[UnityTest] public IEnumerator FinalizeScene_CannotAddNodesAfterFinalize()
```

**Implementation:** `SofaSimulation.AddRigidBone()`, `AddDeformableTissue()`, `AddLigament()`, `AddRigidImplant()`, `FinalizeScene()`.

#### 2.3 Mesh Synchronization (RED → GREEN → REFACTOR)

**Tests (C++ / Google Test):**
```cpp
TEST(DataTransfer, GetPositions_ReturnsCorrectVertexCount)
TEST(DataTransfer, GetPositions_AfterStep_PositionsChange)
TEST(DataTransfer, GetChangedPositions_ReturnsOnlyMovedVertices)
TEST(DataTransfer, GetFrameSnapshot_ReturnsAllFields)
TEST(DataTransfer, DoubleBuffer_SwapIsAtomic)
TEST(DataTransfer, TripleBuffer_SnapshotNeverTornRead)
```

**Tests (PlayMode):**
```csharp
[UnityTest] public IEnumerator MeshSync_TransfersPositionsFromSOFA()
[UnityTest] public IEnumerator MeshSync_SkipsSyncBelowThreshold()
[UnityTest] public IEnumerator MeshSync_CompletesUnder2ms()
[UnityTest] public IEnumerator MeshSync_HandlesTopologyChange()
[UnityTest] public IEnumerator MeshSync_UsesAdvancedMeshAPI_NoGCAlloc()
[UnityTest] public IEnumerator FrameSnapshot_SinglePInvokeCall_ReturnsAllData()
```

**Implementation:** `data_transfer.cpp` (triple-buffered snapshot + double-buffered mesh), `SofaMeshTransfer.cs` (Burst jobs + Advanced Mesh API).

**Requirements:** REQ-INT-02, REQ-INT-04, REQ-PERF-05

---

### Sprint 3: ROM Engine (Week 5)

#### 3.1 Angle Measurement (RED → GREEN → REFACTOR)

**Tests (EditMode):**
```csharp
[Test] public void CalculateDorsiflexion_At0Degrees_Returns0()
[Test] public void CalculateDorsiflexion_At20Degrees_Returns20()
[Test] public void CalculatePlantarflexion_At50Degrees_Returns50()
[Test] public void CalculateTotalArc_20DFPlus50PF_Returns70()
[Test] public void CalculateInversion_At35Degrees_Returns35()
[Test] public void AngleFromQuaternion_IdentityRotation_ReturnsZero()
[Test] public void AngleFromQuaternion_KnownRotation_ReturnsCorrectAngle()
[Test] public void RelativeOrientation_TibiaToTalus_DecomposesCorrectly()
```

**Tests (C++ / Google Test):**
```cpp
TEST(JointAngle, RelativeOrientation_IdentityBodies_ReturnsZero)
TEST(JointAngle, RelativeOrientation_KnownRotation_ReturnsCorrectEuler)
TEST(JointAngle, DorsiflexionAxis_MatchesAnatomicalConvention)
```

**Implementation:** Joint angle from relative tibia/talus orientation (emergent joint model). C++ computes from `MechanicalObject<Rigid3d>` positions; C# `AngleMath` for UI/recording.

#### 3.2 ROM Engine Core (RED → GREEN → REFACTOR)

**Tests (PlayMode):**
```csharp
[UnityTest] public IEnumerator StartSweep_AppliesExternalMoment()
[UnityTest] public IEnumerator StartSweep_RecordsAnglesOverTime()
[UnityTest] public IEnumerator StopSweep_ReturnsCompletedRecord()
[UnityTest] public IEnumerator GetCurrentAngle_DuringSimulation_ReturnsLiveValue()
[UnityTest] public IEnumerator GetCurrentTorque_DuringSimulation_ReturnsLiveValue()
[UnityTest] public IEnumerator PreOpROM_WithConstraints_RecordsTotalArcNear25()
```

**Tests (SOFA / pytest):**
```python
def test_emergent_joint_angle_at_neutral():
    """Tibia-talus relative angle is ~0 at neutral with ligament constraints."""
def test_preop_rom_dorsiflexion():
    """Pre-op DF with arthritic constraints produces 5-10 degrees."""
def test_preop_rom_plantarflexion():
    """Pre-op PF with arthritic constraints produces 15-20 degrees."""
def test_preop_rom_total_arc():
    """Total pre-op sagittal arc is 22-31 degrees."""
def test_postop_rom_with_implant():
    """Post-op ROM with implant produces 33-53 degrees total arc."""
def test_rom_improvement():
    """Post-op minus pre-op shows 4-14 degree improvement."""
def test_ligament_stiffness_affects_rom():
    """Stiffer ligaments reduce ROM; looser ligaments increase ROM."""
def test_bilinear_ligament_toe_region_increases_rom():
    """Toe region allows slightly more ROM than pure linear model."""
```

**Implementation:** `ROMEngine` reads angles from `SofaFrameSnapshot`. ROM sweep driven by `ConstantForceField` torque on talus via `sofa_apply_joint_torque()`. Emergent joint: tibia (fixed) + talus (free) + bilinear ligaments + bone-bone contact.

**Requirements:** REQ-ROM-01 through REQ-ROM-08, REQ-SIM-06, REQ-SIM-07, REQ-SIM-09

---

### Sprint 4: Resection Engine (Week 6)

#### 4.1 Cut Plane Geometry (RED → GREEN → REFACTOR)

**Tests (EditMode):**
```csharp
[Test] public void CutPlane_DefaultTibial_Perpendicular90Degrees()
[Test] public void CutPlane_SagittalAngle_DefaultsTo89Degrees()
[Test] public void CutPlane_AdjustDepth_MovesPlaneAlongNormal()
[Test] public void CutPlane_AdjustDepthBy1mm_AccurateWithin0Point1mm()
[Test] public void CutPlane_IntersectsBone_ReturnsTrueForValidPosition()
[Test] public void CutPlane_MissesBody_ReturnsFalseForInvalidPosition()
[Test] public void VolumeCalculation_KnownCube_ReturnsCorrectVolume()
```

**Implementation:** `CutPlaneController` (positioning math, volume estimation).

#### 4.2 Dual-Representation Resection (RED → GREEN → REFACTOR)

**Tests (C++ / Google Test):**
```cpp
TEST(Resection, CentroidBased_RemovesCorrectTetrahedra)
TEST(Resection, CentroidBased_SmoothCutBoundary)
TEST(Resection, TopologyChanged_FlagSetAfterCut)
TEST(Resection, GetSurfaceMesh_ReturnsUpdatedSurface)
TEST(Resection, Resection_PreservesBoundaryConditions)
TEST(Resection, Resection_CollisionModelRemainsValid)
```

**Tests (PlayMode):**
```csharp
[UnityTest] public IEnumerator ExecuteCut_EzySlice_ProducesCleanVisualCut()
[UnityTest] public IEnumerator ExecuteCut_SOFA_RemovesTetrahedra()
[UnityTest] public IEnumerator ExecuteCut_DualRepresentation_BothComplete()
[UnityTest] public IEnumerator ExecuteCut_CompletesUnder500ms()
[UnityTest] public IEnumerator PreviewCut_ShowsIntersectionContour()
[UnityTest] public IEnumerator Undo_RebuildsSofaScene_RestoresOriginalMesh()
[UnityTest] public IEnumerator SafetyCheck_WarnsWhenNearMalleolus()
```

**Tests (SOFA / pytest):**
```python
def test_centroid_resection_removes_tetrahedra():
    """Cut plane removes tetrahedra whose centroids are below the plane."""
def test_resection_updates_surface():
    """Tetra2TriangleTopologicalMapping updates surface after cut."""
def test_resection_maintains_collision():
    """Collision model remains valid after topology change."""
def test_resection_preserves_boundary_conditions():
    """Fixed constraints still apply after resection."""
```

**Implementation:** EzySlice for visual (instant), SOFA centroid-based for physics, scene rebuild for undo.

**Requirements:** REQ-RES-01 through REQ-RES-09, REQ-PERF-03

---

### Sprint 4.5: Foot Bone Anatomy + Calcaneus Simulation (Week 6.5) ✓ DONE

#### 4.5.1 Mesh Acquisition & BoneType Expansion (RED → GREEN → REFACTOR)

**Tests (EditMode):**
```csharp
[Test] public void BoneType_HasExpected28Values()
[Test] public void BoneType_IsSimulated_TrueForTibiaTalusCalcaneus()
[Test] public void BoneType_IsSimulated_FalseForContextBones()
[Test] public void AnatomyConfig_GetMeshForBone_ReturnsCorrectMesh()
[Test] public void AnatomyConfig_GetMeshForBone_ReturnsNullForMissingBone()
```

**Implementation:** Download 23 additional right-foot bone STLs from BodyParts3D. Expand `BoneType` enum from 4 to 28 values. Add `IsSimulated()` extension method. Refactor `AnatomyConfig` from hardcoded mesh fields to `BoneMeshEntry[]` array.

#### 4.5.2 SofaFrameSnapshot Calcaneus & fillSnapshot Refactor (RED → GREEN → REFACTOR)

**Tests (C++ / Google Test):**
```cpp
TEST(SceneBuilder, FillSnapshot_WithCalcaneus_PopulatesCalcaneusFrame)
TEST(SceneBuilder, FillSnapshot_WithoutCalcaneus_CalcaneusFrameIsZero)
TEST(SceneBuilder, FillSnapshot_NameBasedLookup_IndependentOfBoneOrder)
```

**Implementation:** Add calcaneus frame to `SofaFrameSnapshot` (ABI-breaking, version bump to 0.3.0). Refactor `fillSnapshot()` from index-based to name-based bone lookup via `findBone()`.

#### 4.5.3 Calcaneus Simulation & Subtalar Ligaments (RED → GREEN → REFACTOR)

**Tests (C++ / Google Test):**
```cpp
TEST(SceneBuilder, AddCalcaneus_CreatesThirdBone)
TEST(SceneBuilder, SubtalarLigaments_ConnectTalusAndCalcaneus)
TEST(SceneBuilder, ThreeBoneScene_1000Steps_Stable)
TEST(SceneBuilder, SubtalarJointAngle_RespondsToTorque)
```

**Tests (PlayMode):**
```csharp
[UnityTest] public IEnumerator SofaSimulation_AddCalcaneus_Succeeds()
[UnityTest] public IEnumerator SofaSimulation_GetSnapshot_CalcaneusFramePopulated()
[UnityTest] public IEnumerator SofaSimulation_SubtalarLigaments_ConstrainMotion()
```

**Implementation:** Add calcaneus as free rigid body via existing `sofa_add_rigid_bone()` API. Add 4 subtalar ligaments (ITCL, CL, ATCL, PTCL) via existing `sofa_add_ligament()` API. Attachment offsets determined empirically from BodyParts3D mesh geometry.

**Requirements:** Anatomical context for surgeon demonstrations, subtalar joint modeling.

---

### Sprint 4.6: Unity Bone Visualization (Week 6.75) ✓ DONE

#### 4.6.1 BoneVisualizer & Materials (RED → GREEN → REFACTOR)

**Tests (EditMode):**
```csharp
[Test] public void BoneMaterial_SimulatedBones_AreOpaque()
[Test] public void BoneMaterial_ContextBones_AreSemiTransparent()
```

**Tests (PlayMode):**
```csharp
[UnityTest] public IEnumerator BoneVisualizer_UpdatesTransformFromSnapshot()
[UnityTest] public IEnumerator BoneVisualizer_StaticBonesDoNotMove()
[UnityTest] public IEnumerator AnatomyManager_28Bones_AllRendered()
[UnityTest] public IEnumerator OrbitCamera_FramesBounds_Correctly()
```

**Implementation:** `BoneVisualizer` MonoBehaviour reads `SofaFrameSnapshot` each frame and updates transforms for simulated bones. `BoneMaterialConfig` differentiates simulated (opaque) vs context (semi-transparent) bones. `OrbitCamera` for ankle visualization. Import all 28 STL files as Unity mesh assets.

**Requirements:** Visual ankle context for surgeon demonstrations.

---

### Sprint 5: Implant System (Week 7)

#### 5.1 Implant Library & Data (RED → GREEN → REFACTOR)

**Tests (EditMode):**
```csharp
[Test] public void STARSystem_HasFiveSizes()
[Test] public void STARSystem_BearingThickness_Range6To10()
[Test] public void STARSystem_IsMobileBearing()
[Test] public void InfinitySystem_IsFixedBearing()
[Test] public void MaterialProperties_CoCr_E200To250GPa()
[Test] public void MaterialProperties_UHMWPE_E0Point8GPa()
```

**Implementation:** `ImplantLibrary` ScriptableObject, `ImplantSystem`/`ImplantSize` data structures, placeholder meshes.

#### 5.2 Implant Positioning & Alignment (RED → GREEN → REFACTOR)

**Tests (EditMode):**
```csharp
[Test] public void Alignment_Neutral_AllMetricsAcceptable()
[Test] public void Alignment_TibiotalarAngle12_GeneratesWarning()
[Test] public void Alignment_ADTA85_GeneratesWarning()
[Test] public void Alignment_Slope6_GeneratesWarning()
[Test] public void Coverage_FullCoverage_Returns100Percent()
[Test] public void Coverage_WithOverhang_DetectsOverhangTrue()
```

**Tests (PlayMode):**
```csharp
[UnityTest] public IEnumerator SelectSystem_LoadsCorrectMeshes()
[UnityTest] public IEnumerator PositionImplant_UpdatesAlignmentMetrics()
[UnityTest] public IEnumerator FinalizeImplant_RegistersInSOFA()
[UnityTest] public IEnumerator ChangeBearingThickness_UpdatesJointSpace()
```

**Implementation:** `ImplantManager` with 6-DOF positioning, alignment metrics, coverage analysis, SOFA registration via `SofaSimulation.AddRigidImplant()`.

**Requirements:** REQ-IMP-01 through REQ-IMP-09

---

### Sprint 6: Post-Op ROM & Comparison (Week 8)

#### 6.1 Post-Op ROM (RED → GREEN → REFACTOR)

**Tests (PlayMode):**
```csharp
[UnityTest] public IEnumerator PostOpROM_WithImplant_ProducesGreaterArcThanPreOp()
[UnityTest] public IEnumerator PostOpROM_RecordsSeparateFromPreOp()
[UnityTest] public IEnumerator PostOpROM_ChangeBearingThickness_AffectsROM()
```

**Tests (SOFA / pytest):**
```python
def test_postop_rom_with_star_implant():
    """STAR implant produces ROM in expected 33-53 degree range."""
def test_bearing_thickness_affects_rom():
    """Thicker PE bearing reduces ROM; thinner increases ROM."""
def test_implant_contact_forces():
    """Contact forces between CoCr talar and PE bearing are physiological."""
```

**Implementation:** Reuse `ROMEngine` with post-op SOFA scene (rebuilt with implant nodes after placement).

#### 6.2 Comparison Engine (RED → GREEN → REFACTOR)

**Tests (EditMode):**
```csharp
[Test] public void GenerateReport_IncludesROMComparison()
[Test] public void GenerateReport_IncludesAlignmentMetrics()
[Test] public void GenerateReport_IncludesImplantDetails()
[Test] public void GenerateReport_IncludesResectionDetails()
[Test] public void ExportJSON_ProducesValidJSON()
[Test] public void ExportCSV_ContainsAllFields()
[Test] public void ROMImprovement_CalculatedCorrectly()
```

**Implementation:** `ComparisonEngine` + JSON/CSV export.

**Requirements:** REQ-UI-03, REQ-UI-04, REQ-UI-05

---

### Sprint 7: UI & Workflow Integration (Week 9)

#### 7.1 Workflow State Machine (RED → GREEN → REFACTOR)

**Tests (EditMode):**
```csharp
[Test] public void InitialState_IsInit()
[Test] public void TransitionTo_PreOpView_FromInit_Succeeds()
[Test] public void TransitionTo_Resection_FromPreOpView_Fails_MustDoROMFirst()
[Test] public void CanTransitionTo_ValidNext_ReturnsTrue()
[Test] public void CanTransitionTo_InvalidNext_ReturnsFalse()
[Test] public void GoBack_FromResection_ReturnsToPreOpROM()
[Test] public void OnPhaseChanged_FiresEvent_WithCorrectArgs()
```

**Implementation:** `WorkflowManager` state machine with transition validation.

#### 7.2 UI Components (RED → GREEN → REFACTOR)

**Tests (PlayMode):**
```csharp
[UnityTest] public IEnumerator ROMGauge_DisplaysCurrentAngle()
[UnityTest] public IEnumerator WorkflowPanel_ShowsCurrentPhase()
[UnityTest] public IEnumerator ResectionPanel_AdjustsDepthOnSliderChange()
[UnityTest] public IEnumerator ComparisonView_ShowsSideBySideViewports()
```

**Implementation:** UI Toolkit panels — ROM gauge, resection controls, implant selector, comparison dashboard, workflow navigation.

**Requirements:** REQ-UI-01, REQ-UI-02, REQ-UI-03, REQ-UI-04, REQ-UI-06

---

### Sprint 8: End-to-End Integration & Validation (Week 10)

#### 8.1 E2E Workflow Tests

**Tests (PlayMode/E2E):**
```csharp
[UnityTest] public IEnumerator FullWorkflow_LoadThroughComparison_Completes()
{
    // 1. Init → Load anatomy → Construct SOFA scene
    // 2. Pre-Op ROM → record ~25deg total arc
    // 3. Resection → tibial cut 5mm (EzySlice visual + SOFA physics)
    // 4. Implant → place STAR size 3, PE 8mm → register in SOFA
    // 5. Post-Op ROM → record ~40deg total arc
    // 6. Compare → improvement ~15deg
}

[UnityTest] public IEnumerator FullWorkflow_FrameRateAbove30fps()
[UnityTest] public IEnumerator FullWorkflow_SOFAStepUnder20ms()
[UnityTest] public IEnumerator FullWorkflow_ResectionUnder500ms()
[UnityTest] public IEnumerator FullWorkflow_UndoResection_RebuildsSofaScene()
```

#### 8.2 Biomechanical Validation

**Tests (SOFA / pytest):**
```python
def test_preop_rom_matches_clinical_data():
    """Pre-op ROM within +/-5deg of Glazebrook 2008 (22-31 deg)."""
def test_postop_rom_matches_clinical_data():
    """Post-op ROM within +/-5deg of published ranges (33-53 deg)."""
def test_contact_forces_physiological():
    """Tibiotalar contact force < 5x body weight at neutral stance."""
def test_ligament_forces_physiological():
    """ATFL force < 150N * 15% strain = ~22.5N at end range."""
def test_implant_contact_pressure():
    """PE contact pressure < 30 MPa (below yield stress)."""
```

#### 8.3 Performance Validation

**Tests (PlayMode):**
```csharp
[UnityTest] public IEnumerator Performance_RenderingAbove30fps()
[UnityTest] public IEnumerator Performance_SOFAStepBelow20ms()
[UnityTest] public IEnumerator Performance_MeshSyncBelow2ms()
[UnityTest] public IEnumerator Performance_MemoryBelow4GB()
[UnityTest] public IEnumerator Performance_PInvokeCalls_Under10PerFrame()
```

#### 8.4 Native Bridge Profiling

**Tests (PlayMode):**
```csharp
[UnityTest] public IEnumerator Profiling_FEMSolveTime_Reported()
[UnityTest] public IEnumerator Profiling_CollisionTime_Reported()
[UnityTest] public IEnumerator Profiling_ConstraintIterations_Reported()
```

**Implementation:** `SofaProfilingData` from C++ fed into Unity Profiler as custom `ProfilerMarker` entries.

**Requirements:** REQ-PERF-01 through REQ-PERF-07, REQ-SIM-09

---

### Sprint 9: Polish, Documentation & Packaging (Week 11)

#### 9.1 Bug Fixes & Edge Cases
- Address all failing tests from Sprint 8
- Degenerate meshes, solver instability, extreme parameter values
- SOFA divergence recovery: detect via `SofaFrameSnapshot.solver_diverged`, pause, notify user
- Scene rebuild recovery: destroy + recreate SOFA scene on critical errors

#### 9.2 Asset Pipeline Finalization
- Replace placeholder implant meshes with accurate geometry
- Bone mesh optimization (LOD groups, material tweaks)
- Representative test case with realistic anatomy

#### 9.3 Build & Distribution
- Standalone build (Windows/Linux)
- `collect_sofa_deps.sh` bundles all SOFA DLLs (walks transitive dependency tree)
- Test deployment on clean machines (no SOFA installed)
- Test on target hardware configurations

---

## Part 3: Test Summary, CI & Risks

### Test Execution Summary

| Sprint | EditMode | PlayMode | C++ (Google Test) | SOFA (pytest) | Total |
|--------|----------|----------|-------------------|---------------|-------|
| 0 | 0 | 8 | 8 | 0 | **16** |
| 1 | 16 | 6 | 0 | 0 | **22** |
| 2 | 0 | 6 | 12 | 7 | **25** |
| 3 | 8 | 6 | 3 | 8 | **25** |
| 4 | 7 | 7 | 6 | 4 | **24** |
| 4.5 | 5 | 4 | 0 | 0 | **9** |
| 4.6 | 2 | 4 | 0 | 0 | **6** |
| 5 | 12 | 4 | 0 | 0 | **16** |
| 6 | 7 | 3 | 0 | 3 | **13** |
| 7 | 7 | 4 | 0 | 0 | **11** |
| 8 | 0 | 12 | 0 | 5 | **17** |
| **Total** | **64** | **64** | **29** | **27** | **184** |

### Continuous Integration

```yaml
# .github/workflows/ci.yml
on: [push, pull_request]

jobs:
  native-build:
    runs-on: ubuntu-latest  # also windows-latest matrix
    steps:
      - Checkout
      - Install SOFA v25.12 (pre-built binaries or Docker)
      - cmake --build native/build
      - ctest --test-dir native/build       # Google Test suite
      - Upload SofaAnkleBridge artifact

  unity-tests:
    needs: [native-build]
    steps:
      - Checkout
      - Download native plugin artifact
      - Deploy to Assets/Plugins/x86_64/
      - Activate Unity license
      - unity-test-runner -testPlatform EditMode
      - unity-test-runner -testPlatform PlayMode
      - Upload test results

  sofa-tests:
    steps:
      - Checkout
      - Install SOFA + SofaPython3
      - pytest Tests/SOFA/
      - Upload test results

  build:
    needs: [native-build, unity-tests, sofa-tests]
    steps:
      - Build standalone player (Win64)
      - Run collect_sofa_deps.sh
      - Test deployment on clean runner
      - Upload artifact
```

### Risk Mitigation

| Risk | Severity | Mitigation | Sprint |
|------|----------|-----------|--------|
| SOFA C++ API breaks between versions | High | Pin to v25.12; version handshake in `sofa_bridge_init()`; CI catches regressions | 0 |
| DLL dependency hell (~80 SOFA libs) | High | Automated `collect_sofa_deps.sh`; test deployment on clean machines in CI | 0, 9 |
| Thread safety bugs in async stepping | High | Single-mutex + triple-buffer snapshots + reader-count mesh guard; TSAN in CI | 0, 2 |
| SOFA solver divergence with ankle params | Medium | Validate materials in isolated SOFA tests; divergence flag in snapshot | 2, 3 |
| Topology changes crash SOFA | Medium | try/catch all topology ops; centroid removal is conservative; scene rebuild recovery | 4 |
| CMake/SOFA build complexity | Medium | Docker-based CI; pre-built SOFA binaries as fallback | 0 |
| Unity editor DLL locking | Medium | Restart required for native plugin updates; IL2CPP for release | 0 |
| EzySlice non-manifold meshes | Medium | Dual representation means SOFA physics isn't affected by visual mesh issues | 4 |
| Real-time performance insufficient | Medium | Profile early (Sprint 2); `SofaProfilingData` identifies bottlenecks | 2-8 |
| Implant meshes unavailable | Low | Parametric placeholder geometry | 5 |
| Clinical validation gap | Medium | Orthopedic surgeon parameter review at Sprint 3 and 8 | 3, 8 |
| Platform differences (Win/Linux) | Medium | CI builds both platforms; abstract paths in scripts | 0, 9 |

---

## Part 4: Key Design Decisions (Negotiated)

These decisions were reached through agent negotiation (see doc 11 for full discussion):

| ID | Decision | Rationale |
|----|----------|-----------|
| D-01 | Custom C++ native plugin over SofaUnity | Full API control, optimized data paths, no third-party lock-in |
| D-02 | Flat C API (`extern "C"`) — domain-specific, not general | Small API surface, prevents misuse, SOFA types stay in C++ |
| D-03 | Emergent joint from ligaments + contact | Biomechanically realistic — captures shifting rotation axis |
| D-04 | Bilinear ligament springs (toe + linear region) | +/- 2-3 deg ROM accuracy improvement over linear springs |
| D-05 | Dual-representation resection (EzySlice + SOFA centroid) | Clean visual, physically accurate FEM topology |
| D-06 | Scene rebuild for undo | Simpler and more robust than SOFA topology undo |
| D-07 | Batched frame snapshot struct | Reduces per-frame P/Invoke from ~10 to 2 |
| D-08 | Triple-buffer snapshot + double-buffer mesh | Thread-safe without latency penalty |
| D-09 | Join-with-timeout on shutdown | Clean shutdown; detach only as last resort |
| D-10 | Version handshake struct | ABI stability across DLL/C# updates |
| D-11 | `[DefaultExecutionOrder(-100)]` on bridge | Deterministic Unity frame timing |
| D-12 | Explicit SOFA plugin loading list | Prevents silent component-not-found failures |
| D-13 | Layered error model (C++ → C → C# bridge → domain) | Clean, debuggable error propagation |

---

## Architecture Decision Updates

The following decisions from doc 06 are superseded:

| Original | Updated | Rationale |
|----------|---------|-----------|
| **AD-01:** SofaUnity in-process plugin | **AD-01 (rev):** Custom SofaAnkleBridge native plugin | Full API control, optimized data paths, no third-party dependency |
| REQ-INT-01: SofaUnity via P/Invoke | **REQ-INT-01 (rev):** Custom bridge DLL via P/Invoke with domain-specific C API | Same mechanism, our own API surface |

All other architectural decisions (AD-02 through AD-05) remain unchanged.
