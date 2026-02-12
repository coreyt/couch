# TDD-Based Implementation Plan

> Test-Driven Development plan organized into sprints.
> Each feature starts with tests, then implementation, then integration.
> Uses a **custom SOFA-Unity bridge** (C++ native plugin + C# managed layer) — no SofaUnity plugin.
> Tests use Unity Test Framework (NUnit) for C#, Google Test for native C++, and pytest for SOFA Python scenes.
>
> See also:
> - [10 - Custom Integration Plan](10-custom-integration-plan.md) — architecture and API design
> - [11 - Agent Review & Negotiation](11-agent-review-negotiation.md) — design decisions and rationale

---

## Testing Strategy

### Test Pyramid

```
         /  E2E Tests  \          <- 5-10 workflow integration tests
        / Integration    \        <- 15-20 component interaction tests
       / Unit Tests        \      <- 50-80 pure logic tests (C# + C++)
      /____________________\
```

### Test Categories

| Category | Framework | Location | What It Tests |
|----------|-----------|----------|---------------|
| **Unit (C#)** | NUnit + Unity Test Framework | `Tests/EditMode/` | Pure logic: angle math, alignment validation, data structures |
| **Unit (C++)** | Google Test | `native/tests/` | Native bridge: SOFA lifecycle, scene construction, data readback |
| **Unit (SOFA)** | pytest + SofaPython3 | `Tests/SOFA/` | SOFA scene configs: material properties, solver convergence, ROM output |
| **Integration** | Unity Test Framework (Play Mode) | `Tests/PlayMode/` | Component interactions: SofaBridge + ROMEngine, ResectionEngine + AnatomyManager |
| **E2E** | Unity Test Framework (Play Mode) | `Tests/PlayMode/E2E/` | Full workflow: load anatomy -> measure ROM -> resect -> implant -> measure post-op ROM -> compare |

### Test Naming Convention
```
[MethodUnderTest]_[Scenario]_[ExpectedBehavior]
```
Example: `MeasureDorsiflexion_ArticulationAt20Degrees_Returns20()`

---

## Sprint 0: Project Scaffolding & Native Bridge Foundation (Weeks 1-2)

> This sprint is expanded to 2 weeks because we are building the native C++ bridge from scratch.

### 0.1 Unity Project Setup

**Tests First:**
```
TEST: Project compiles with zero errors
TEST: Assembly definitions resolve correctly
TEST: Required packages installed (TextMeshPro, InputSystem, Addressables)
```

**Implementation:**
- Create Unity 6 project with URP
- Set up assembly definitions:
  - `AnkleSim.Core` (pure logic, no Unity dependencies)
  - `AnkleSim.Runtime` (MonoBehaviours, Unity API)
  - `AnkleSim.Bridge` (P/Invoke declarations, NativeArray mesh sync)
  - `AnkleSim.Editor` (editor tools)
  - `AnkleSim.Tests.EditMode`
  - `AnkleSim.Tests.PlayMode`
- Configure physics: TGS solver, 200Hz fixed timestep (0.005s)
- Import required packages

### 0.2 Native Plugin Build System

**Tests First (C++ / Google Test):**
```cpp
// native/tests/test_build.cpp
TEST(Build, NativePluginLoads) {
    // Verify DLL/SO can be loaded and exports are found
    void* handle = dlopen("SofaAnkleBridge.so", RTLD_LAZY);
    ASSERT_NE(handle, nullptr);
    auto init_fn = dlsym(handle, "sofa_bridge_init");
    ASSERT_NE(init_fn, nullptr);
    dlclose(handle);
}
```

**Implementation:**
- Set up CMake project in `native/`
- Configure SOFA library linking (find_package or manual paths)
- Build `SofaAnkleBridge.dll/.so`
- Create deployment script to copy DLLs to `Assets/Plugins/x86_64/`
- Create dependency walker script (collects all transitive SOFA DLL dependencies)

### 0.3 Native Bridge Lifecycle

**Tests First (C++ / Google Test):**
```cpp
// native/tests/test_lifecycle.cpp
TEST(SofaBridge, Init_WithValidPluginDir_ReturnsSuccess) {
    ASSERT_EQ(sofa_bridge_init("./plugins"), 0);
    sofa_bridge_shutdown();
}

TEST(SofaBridge, Init_WithInvalidPluginDir_ReturnsError) {
    ASSERT_NE(sofa_bridge_init("/nonexistent"), 0);
    const char* err = sofa_bridge_get_error();
    ASSERT_NE(err, nullptr);
}

TEST(SofaBridge, Shutdown_AfterInit_Succeeds) {
    sofa_bridge_init("./plugins");
    sofa_bridge_shutdown();
    // No crash, no leak (run under valgrind/ASAN)
}

TEST(SofaBridge, VersionHandshake_ReturnsValidVersion) {
    SofaBridgeVersion v = sofa_bridge_get_version();
    ASSERT_EQ(v.bridge_version_major, 1);
    ASSERT_GT(v.sofa_version_major, 0);
}

TEST(SofaBridge, CreateEmptyScene_AndStepOnce) {
    sofa_bridge_init("./plugins");
    SofaSceneConfig config = default_test_config();
    ASSERT_EQ(sofa_scene_create(&config), 0);
    ASSERT_EQ(sofa_scene_finalize(), 0);
    ASSERT_EQ(sofa_step(0.01f), 0);
    sofa_bridge_shutdown();
}

TEST(SofaBridge, AsyncStep_CompletesWithoutDeadlock) {
    sofa_bridge_init("./plugins");
    SofaSceneConfig config = default_test_config();
    sofa_scene_create(&config);
    sofa_scene_finalize();
    ASSERT_EQ(sofa_step_async(), 0);
    // Poll with 5-second timeout
    auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (!sofa_step_async_is_complete()) {
        ASSERT_LT(std::chrono::steady_clock::now(), deadline) << "Step timed out";
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    sofa_bridge_shutdown();
}

TEST(SofaBridge, Shutdown_WhileAsyncStepRunning_DoesNotCrash) {
    sofa_bridge_init("./plugins");
    SofaSceneConfig config = default_test_config();
    sofa_scene_create(&config);
    sofa_scene_finalize();
    sofa_step_async();
    // Immediate shutdown — must not crash or deadlock
    sofa_bridge_shutdown();
}
```

**Implementation:**
- `bridge_api.cpp` — flat C API (`extern "C"`) wrapping `SofaSimulationManager`
- `sofa_simulation.cpp` — SOFA lifecycle: `init()` loads plugins, `createScene()` builds root node with FreeMotionAnimationLoop + GenericConstraintSolver + collision pipeline
- `thread_manager.cpp` — background stepping thread with join-on-shutdown and cancellation token
- Version handshake struct with bridge + SOFA version numbers

### 0.4 C# P/Invoke Layer

**Tests First (PlayMode):**
```csharp
// Tests/PlayMode/Integration/NativeBridgeTests.cs
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
- `SofaNativeBridge.cs` — P/Invoke declarations with `[DllImport("SofaAnkleBridge")]`
- `NativeStructs.cs` — C# struct mirrors (`SofaSceneConfig`, `SofaBridgeVersion`, etc.) with `[StructLayout(LayoutKind.Sequential)]`
- `SofaSimulation.cs` — high-level C# API wrapping P/Invoke calls with error checking and `SofaBridgeException` throwing
- `SofaBridgeComponent.cs` — MonoBehaviour with `[DefaultExecutionOrder(-100)]` for Unity lifecycle integration

### 0.5 Directory & Configuration Structure

```
project-root/
├── native/                              # C++ native plugin
│   ├── CMakeLists.txt
│   ├── include/
│   │   └── sofa_ankle_bridge.h          # Public C API header
│   ├── src/
│   │   ├── bridge_api.cpp               # Flat C API (extern "C")
│   │   ├── sofa_simulation.h/.cpp       # SOFA lifecycle manager
│   │   ├── scene_builder.h/.cpp         # Programmatic scene construction
│   │   ├── data_transfer.h/.cpp         # Position/force readback + buffers
│   │   ├── command_handler.h/.cpp       # Resection/implant/constraint commands
│   │   └── thread_manager.h/.cpp        # Background thread with cancellation
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
│   │   │   │   ├── DataModels/          # ROMRecord, AlignmentMetrics, etc.
│   │   │   │   ├── Validation/          # Alignment validators, range checkers
│   │   │   │   └── Math/                # Angle calculations, plane geometry
│   │   │   ├── Bridge/                  # Assembly: AnkleSim.Bridge
│   │   │   │   ├── SofaNativeBridge.cs  # P/Invoke declarations
│   │   │   │   ├── NativeStructs.cs     # C# struct mirrors
│   │   │   │   ├── SofaSimulation.cs    # High-level C# API
│   │   │   │   ├── SofaBridgeComponent.cs # MonoBehaviour wrapper
│   │   │   │   ├── SofaMeshTransfer.cs  # NativeArray mesh sync + Burst jobs
│   │   │   │   └── SofaBridgeException.cs
│   │   │   ├── Runtime/                 # Assembly: AnkleSim.Runtime
│   │   │   │   ├── Anatomy/             # AnatomyManager
│   │   │   │   ├── ROM/                 # ROMEngine
│   │   │   │   ├── Resection/           # ResectionEngine
│   │   │   │   ├── Implant/             # ImplantManager
│   │   │   │   ├── Comparison/          # ComparisonEngine
│   │   │   │   ├── Workflow/            # WorkflowManager
│   │   │   │   └── UI/                  # UI controllers
│   │   │   ├── Editor/                  # Assembly: AnkleSim.Editor
│   │   │   ├── ScriptableObjects/       # Config assets
│   │   │   ├── Prefabs/
│   │   │   ├── Materials/
│   │   │   ├── Scenes/
│   │   │   └── StreamingAssets/
│   │   │       └── SOFA/
│   │   │           └── meshes/          # VTK volumetric meshes
│   │   ├── Plugins/
│   │   │   ├── x86_64/
│   │   │   │   ├── SofaAnkleBridge.dll  # Our custom plugin
│   │   │   │   ├── Sofa.Core.dll        # SOFA framework libs
│   │   │   │   └── Sofa.Component.*.dll # SOFA component libs (~80 files)
│   │   │   └── EzySlice/               # Mesh cutting library
│   │   └── Tests/
│   │       ├── EditMode/
│   │       │   ├── Core/
│   │       │   ├── Validation/
│   │       │   └── Math/
│   │       ├── PlayMode/
│   │       │   ├── Integration/
│   │       │   └── E2E/
│   │       └── SOFA/                    # pytest-based SOFA scene tests
│
├── scripts/
│   ├── build_native.sh                  # Build SofaAnkleBridge
│   ├── deploy_dlls.sh                   # Copy all DLLs to Unity Plugins/
│   └── collect_sofa_deps.sh             # Walk dependency tree for SOFA libs
│
└── docs/
```

---

## Sprint 1: Core Data Models & Anatomy Loading (Week 3)

### 1.1 Data Models (RED -> GREEN -> REFACTOR)

**Tests (EditMode):**
```csharp
// Tests/EditMode/Core/ROMRecordTests.cs
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

**Implementation:**
- `ROMRecord`, `ROMComparison` (structs/classes with computed properties)
- `AlignmentMetrics`, `AlignmentWarning` (with `IsAcceptable` logic)
- `CoverageAnalysis`, `ResectionRecord`, `ResectionResult`
- All data models in `AnkleSim.Core` assembly (no Unity dependencies)

**Requirements Covered:** REQ-ROM-06, REQ-ROM-08, REQ-IMP-04, REQ-IMP-05

### 1.2 Alignment Validation (RED -> GREEN -> REFACTOR)

**Tests (EditMode):**
```csharp
// Tests/EditMode/Validation/AlignmentValidatorTests.cs
[Test] public void Validate_TibiotalarAngle0_ReturnsAcceptable()
[Test] public void Validate_TibiotalarAngle11_ReturnsWarning()
[Test] public void Validate_ADTA89_ReturnsAcceptable()
[Test] public void Validate_ADTA85_ReturnsWarning()
[Test] public void Validate_PosteriorSlope6_ReturnsWarning()
[Test] public void Validate_AllGood_ReturnsNoWarnings()
[Test] public void Validate_MultipleViolations_ReturnsAllWarnings()
```

**Implementation:**
- `AlignmentValidator` static class with validation methods
- Threshold constants from clinical data

### 1.3 Anatomy Manager (RED -> GREEN -> REFACTOR)

**Tests (PlayMode):**
```csharp
// Tests/PlayMode/Integration/AnatomyManagerTests.cs
[UnityTest] public IEnumerator LoadAnatomy_CreatesAllBoneGameObjects()
[UnityTest] public IEnumerator LoadAnatomy_MeshesHaveCorrectVertexCount()
[UnityTest] public IEnumerator SetVisibility_HidesSpecifiedStructure()
[UnityTest] public IEnumerator SetVisibility_ShowsSpecifiedStructure()
[UnityTest] public IEnumerator GetBoneMesh_ReturnsMeshForValidBoneType()
[UnityTest] public IEnumerator GetBoneMesh_ReturnsNullForInvalidType()
```

**Implementation:**
- `AnatomyManager` MonoBehaviour
- `AnatomyConfig` ScriptableObject
- Mesh loading from FBX/OBJ prefabs
- Visibility toggling per structure

**Requirements Covered:** REQ-ANAT-01, REQ-ANAT-02, REQ-ANAT-04, REQ-ANAT-05

---

## Sprint 2: SOFA Scene Construction & Mesh Sync (Week 4)

### 2.1 Native Scene Construction (RED -> GREEN -> REFACTOR)

**Tests (C++ / Google Test):**
```cpp
// native/tests/test_scene_construction.cpp
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
# Tests/SOFA/test_scene_construction.py
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

**Implementation (C++):**
- `scene_builder.cpp` — `addRigidBone()`, `addDeformableTissue()`, `addLigament()`, `addRigidImplant()`
- Root node: `FreeMotionAnimationLoop` + `GenericConstraintSolver` + collision pipeline
- Rigid bones: `MechanicalObject<Rigid3d>` + `UniformMass` + optional `FixedConstraint` + collision sub-node with `RigidMapping`
- Deformable tissue: `MechanicalObject<Vec3d>` + `SparseLDLSolver` + `TetrahedronFEMForceField` + `Tetra2TriangleTopologicalMapping` for collision surface
- Ligaments: bilinear `StiffSpringForceField` with runtime stiffness modulation (toe region at strain < 3%, then linear)
- `finalizeScene()` calls `sofa::simulation::init(root)` and validates scene graph

### 2.2 C# Scene Construction API (RED -> GREEN -> REFACTOR)

**Tests (PlayMode):**
```csharp
// Tests/PlayMode/Integration/SofaBridgeTests.cs
[UnityTest] public IEnumerator AddRigidBone_CreatesSOFANode()
[UnityTest] public IEnumerator AddDeformableTissue_CreatesSOFANodeWithFEM()
[UnityTest] public IEnumerator AddLigament_CreatesBilinearSpring()
[UnityTest] public IEnumerator AddRigidImplant_CreatesRigidNode()
[UnityTest] public IEnumerator FinalizeScene_CanStepAfterFinalize()
[UnityTest] public IEnumerator FinalizeScene_CannotAddNodesAfterFinalize()
```

**Implementation (C#):**
- `SofaSimulation.AddRigidBone(RigidBoneDescriptor)` — marshals to `SofaRigidBoneConfig` and calls P/Invoke
- `SofaSimulation.AddDeformableTissue(DeformableTissueDescriptor)` — marshals mesh path, material properties
- `SofaSimulation.AddLigament(LigamentDescriptor)` — includes toe/linear stiffness and strain threshold
- `SofaSimulation.AddRigidImplant(ImplantDescriptor)` — marshals vertex/triangle arrays
- `SofaSimulation.FinalizeScene()` — validates configuration, calls native finalize

**Requirements Covered:** REQ-INT-06, REQ-INT-07, REQ-SIM-01, REQ-SIM-02, REQ-SIM-04

### 2.3 Mesh Synchronization (RED -> GREEN -> REFACTOR)

**Tests (C++ / Google Test):**
```cpp
// native/tests/test_data_transfer.cpp
TEST(DataTransfer, GetPositions_ReturnsCorrectVertexCount)
TEST(DataTransfer, GetPositions_AfterStep_PositionsChange)
TEST(DataTransfer, GetChangedPositions_ReturnsOnlyMovedVertices)
TEST(DataTransfer, GetFrameSnapshot_ReturnsAllFields)
TEST(DataTransfer, DoubleBuffer_SwapIsAtomic)
TEST(DataTransfer, TripleBuffer_SnapshotNeverTornRead)
```

**Tests (PlayMode):**
```csharp
// Tests/PlayMode/Integration/MeshSyncTests.cs
[UnityTest] public IEnumerator MeshSync_TransfersPositionsFromSOFA()
[UnityTest] public IEnumerator MeshSync_SkipsSyncBelowThreshold()
[UnityTest] public IEnumerator MeshSync_CompletesUnder2ms()
[UnityTest] public IEnumerator MeshSync_HandlesTopologyChange()
[UnityTest] public IEnumerator MeshSync_UsesAdvancedMeshAPI_NoGCAlloc()
[UnityTest] public IEnumerator FrameSnapshot_SinglePInvokeCall_ReturnsAllData()
```

**Implementation:**
- `data_transfer.cpp` — double-buffered position arrays, triple-buffered frame snapshot
- `SofaMeshTransfer.cs` — Burst-compiled `ApplyPositionsJob` and `ApplySparsePositionsJob`
- `SofaFrameSnapshot` struct mirrored in C# for single-call per-frame readback
- Advanced Mesh API (`Mesh.SetVertexBufferData` with `NativeArray`) for topology changes
- Reader-count guard on mesh double buffer to prevent torn reads

**Requirements Covered:** REQ-INT-02, REQ-INT-04, REQ-PERF-05

---

## Sprint 3: ROM Engine (Week 5)

### 3.1 Angle Measurement (RED -> GREEN -> REFACTOR)

**Tests (EditMode):**
```csharp
// Tests/EditMode/Math/AngleMathTests.cs
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
// native/tests/test_joint_angle.cpp
TEST(JointAngle, RelativeOrientation_IdentityBodies_ReturnsZero)
TEST(JointAngle, RelativeOrientation_KnownRotation_ReturnsCorrectEuler)
TEST(JointAngle, DorsiflexionAxis_MatchesAnatomicalConvention)
```

**Implementation:**
- `AngleMath` utility class (quaternion decomposition into anatomical axes)
- Joint angle computed from relative orientation of talus w.r.t. tibia rigid bodies
- C++ side: `getJointAngle()` computes from two `MechanicalObject<Rigid3d>` positions
- C# side: `AngleMath` for UI/recording, `SofaFrameSnapshot` for live values

### 3.2 ROM Engine Core (RED -> GREEN -> REFACTOR)

**Tests (PlayMode):**
```csharp
// Tests/PlayMode/Integration/ROMEngineTests.cs
[UnityTest] public IEnumerator StartSweep_AppliesExternalMoment()
[UnityTest] public IEnumerator StartSweep_RecordsAnglesOverTime()
[UnityTest] public IEnumerator StopSweep_ReturnsCompletedRecord()
[UnityTest] public IEnumerator GetCurrentAngle_DuringSimulation_ReturnsLiveValue()
[UnityTest] public IEnumerator GetCurrentTorque_DuringSimulation_ReturnsLiveValue()
[UnityTest] public IEnumerator PreOpROM_WithArticulationAt25Deg_RecordsTotalArcNear25()
```

**Tests (SOFA / pytest):**
```python
# Tests/SOFA/test_rom_simulation.py
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
    """Toe region in ligament model allows slightly more ROM than pure linear."""
```

**Implementation:**
- `ROMEngine` MonoBehaviour reads angles from `SofaFrameSnapshot`
- ROM sweep driven by `ConstantForceField` applied to talus via `sofa_apply_joint_torque()`
- Emergent joint model: tibia (fixed rigid), talus (free rigid), constrained by bilinear ligament springs + bone-bone contact
- Recording and analysis stored in `ROMRecord`

**Requirements Covered:** REQ-ROM-01 through REQ-ROM-08, REQ-SIM-06, REQ-SIM-07, REQ-SIM-09

---

## Sprint 4: Resection Engine (Week 6)

### 4.1 Cut Plane Geometry (RED -> GREEN -> REFACTOR)

**Tests (EditMode):**
```csharp
// Tests/EditMode/Math/CutPlaneTests.cs
[Test] public void CutPlane_DefaultTibial_Perpendicular90Degrees()
[Test] public void CutPlane_SagittalAngle_DefaultsTo89Degrees()
[Test] public void CutPlane_AdjustDepth_MovesPlaneAlongNormal()
[Test] public void CutPlane_AdjustDepthBy1mm_AccurateWithin0Point1mm()
[Test] public void CutPlane_IntersectsBone_ReturnsTrueForValidPosition()
[Test] public void CutPlane_MissesBody_ReturnsFalseForInvalidPosition()
[Test] public void VolumeCalculation_KnownCube_ReturnsCorrectVolume()
```

**Implementation:**
- `CutPlaneController` (plane positioning math)
- Volume estimation from mesh geometry

### 4.2 Dual-Representation Resection (RED -> GREEN -> REFACTOR)

**Tests (C++ / Google Test):**
```cpp
// native/tests/test_resection.cpp
TEST(Resection, CentroidBased_RemovesCorrectTetrahedra)
TEST(Resection, CentroidBased_SmoothCutBoundary)
TEST(Resection, TopologyChanged_FlagSetAfterCut)
TEST(Resection, GetSurfaceMesh_ReturnsUpdatedSurface)
TEST(Resection, Resection_PreservesBoundaryConditions)
TEST(Resection, Resection_CollisionModelRemainsValid)
```

**Tests (PlayMode):**
```csharp
// Tests/PlayMode/Integration/ResectionEngineTests.cs
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
# Tests/SOFA/test_resection.py
def test_centroid_resection_removes_tetrahedra():
    """Cut plane removes tetrahedra whose centroids are below the plane."""

def test_resection_updates_surface():
    """Tetra2TriangleTopologicalMapping updates surface after cut."""

def test_resection_maintains_collision():
    """Collision model remains valid after topology change."""

def test_resection_preserves_boundary_conditions():
    """Fixed constraints still apply after resection."""
```

**Implementation:**
- **Visual cut:** EzySlice plane cut on Unity mesh (instant, clean geometry)
- **Physics cut:** SOFA centroid-based tetrahedral removal via `sofa_execute_resection()`
- `ResectionEngine` drives both representations from a single cut command
- **Undo:** Rebuilds entire SOFA scene from cached configuration (simpler and more robust than SOFA topology undo)
- Topology change triggers full mesh rebuild via `sofa_get_surface_mesh()` + Advanced Mesh API

**Requirements Covered:** REQ-RES-01 through REQ-RES-09, REQ-PERF-03

---

## Sprint 5: Implant System (Week 7)

### 5.1 Implant Library & Data (RED -> GREEN -> REFACTOR)

**Tests (EditMode):**
```csharp
// Tests/EditMode/Core/ImplantLibraryTests.cs
[Test] public void STARSystem_HasFiveSizes()
[Test] public void STARSystem_BearingThickness_Range6To10()
[Test] public void STARSystem_IsMobileBearing()
[Test] public void InfinitySystem_IsFixedBearing()
[Test] public void MaterialProperties_CoCr_E200To250GPa()
[Test] public void MaterialProperties_UHMWPE_E0Point8GPa()
```

**Implementation:**
- `ImplantLibrary` ScriptableObject with STAR and Infinity data
- `ImplantSystem`, `ImplantSize` data structures
- Implant mesh prefabs (placeholder geometry initially)

### 5.2 Implant Positioning & Alignment (RED -> GREEN -> REFACTOR)

**Tests (EditMode):**
```csharp
// Tests/EditMode/Validation/ImplantAlignmentTests.cs
[Test] public void Alignment_Neutral_AllMetricsAcceptable()
[Test] public void Alignment_TibiotalarAngle12_GeneratesWarning()
[Test] public void Alignment_ADTA85_GeneratesWarning()
[Test] public void Alignment_Slope6_GeneratesWarning()
[Test] public void Coverage_FullCoverage_Returns100Percent()
[Test] public void Coverage_WithOverhang_DetectsOverhangTrue()
```

**Tests (PlayMode):**
```csharp
// Tests/PlayMode/Integration/ImplantManagerTests.cs
[UnityTest] public IEnumerator SelectSystem_LoadsCorrectMeshes()
[UnityTest] public IEnumerator PositionImplant_UpdatesAlignmentMetrics()
[UnityTest] public IEnumerator FinalizeImplant_RegistersInSOFA()
[UnityTest] public IEnumerator ChangeBearingThickness_UpdatesJointSpace()
```

**Implementation:**
- `ImplantManager` MonoBehaviour
- Interactive 6-DOF positioning (translation + rotation handles)
- Alignment metric calculation from implant transform relative to bone axis
- Coverage calculation (ray casting from implant onto bone surface)
- SOFA rigid body registration via `SofaSimulation.AddRigidImplant()`

**Requirements Covered:** REQ-IMP-01 through REQ-IMP-09

---

## Sprint 6: Post-Op ROM & Comparison (Week 8)

### 6.1 Post-Op ROM (RED -> GREEN -> REFACTOR)

**Tests (PlayMode):**
```csharp
[UnityTest] public IEnumerator PostOpROM_WithImplant_ProducesGreaterArcThanPreOp()
[UnityTest] public IEnumerator PostOpROM_RecordsSeparateFromPreOp()
[UnityTest] public IEnumerator PostOpROM_ChangeBearingThickness_AffectsROM()
```

**Tests (SOFA / pytest):**
```python
# Tests/SOFA/test_postop_rom.py
def test_postop_rom_with_star_implant():
    """STAR implant produces ROM in expected 33-53 degree range."""

def test_bearing_thickness_affects_rom():
    """Thicker PE bearing reduces ROM; thinner increases ROM."""

def test_implant_contact_forces():
    """Contact forces between CoCr talar and PE bearing are physiological."""
```

**Implementation:**
- Reuse ROMEngine with post-op SOFA scene (includes implant rigid body nodes)
- Implant articulation surface contact via SOFA collision pipeline
- Scene is rebuilt with implant nodes after placement is finalized

### 6.2 Comparison Engine (RED -> GREEN -> REFACTOR)

**Tests (EditMode):**
```csharp
// Tests/EditMode/Core/ComparisonEngineTests.cs
[Test] public void GenerateReport_IncludesROMComparison()
[Test] public void GenerateReport_IncludesAlignmentMetrics()
[Test] public void GenerateReport_IncludesImplantDetails()
[Test] public void GenerateReport_IncludesResectionDetails()
[Test] public void ExportJSON_ProducesValidJSON()
[Test] public void ExportCSV_ContainsAllFields()
[Test] public void ROMImprovement_CalculatedCorrectly()
```

**Implementation:**
- `ComparisonEngine` MonoBehaviour
- `ComparisonReport` data aggregation
- JSON/CSV export

**Requirements Covered:** REQ-UI-03, REQ-UI-04, REQ-UI-05

---

## Sprint 7: UI & Workflow Integration (Week 9)

### 7.1 Workflow State Machine (RED -> GREEN -> REFACTOR)

**Tests (EditMode):**
```csharp
// Tests/EditMode/Core/WorkflowTests.cs
[Test] public void InitialState_IsInit()
[Test] public void TransitionTo_PreOpView_FromInit_Succeeds()
[Test] public void TransitionTo_Resection_FromPreOpView_Fails_MustDoROMFirst()
[Test] public void CanTransitionTo_ValidNext_ReturnsTrue()
[Test] public void CanTransitionTo_InvalidNext_ReturnsFalse()
[Test] public void GoBack_FromResection_ReturnsToPreOpROM()
[Test] public void OnPhaseChanged_FiresEvent_WithCorrectArgs()
```

**Implementation:**
- `WorkflowManager` with state machine logic
- Phase transition validation

### 7.2 UI Components (RED -> GREEN -> REFACTOR)

**Tests (PlayMode):**
```csharp
[UnityTest] public IEnumerator ROMGauge_DisplaysCurrentAngle()
[UnityTest] public IEnumerator WorkflowPanel_ShowsCurrentPhase()
[UnityTest] public IEnumerator ResectionPanel_AdjustsDepthOnSliderChange()
[UnityTest] public IEnumerator ComparisonView_ShowsSideBySideViewports()
```

**Implementation:**
- UI Toolkit-based panels for each workflow phase
- ROM gauge (circular dial showing DF/PF angle)
- Resection control panel (depth slider, angle readout, execute button)
- Implant selection panel (system dropdown, size selector, thickness slider)
- Comparison dashboard (side-by-side viewports, data table)
- Workflow navigation bar

**Requirements Covered:** REQ-UI-01, REQ-UI-02, REQ-UI-03, REQ-UI-04, REQ-UI-06

---

## Sprint 8: End-to-End Integration & Validation (Week 10)

### 8.1 E2E Workflow Tests (RED -> GREEN -> REFACTOR)

**Tests (PlayMode/E2E):**
```csharp
// Tests/PlayMode/E2E/FullWorkflowTests.cs
[UnityTest] public IEnumerator FullWorkflow_LoadThroughComparison_Completes()
{
    // 1. Init -> Load anatomy -> Construct SOFA scene
    // 2. Pre-Op ROM -> record ~25deg total arc
    // 3. Resection -> tibial cut 5mm (EzySlice visual + SOFA physics)
    // 4. Implant -> place STAR size 3, PE 8mm -> register in SOFA
    // 5. Post-Op ROM -> record ~40deg total arc
    // 6. Compare -> improvement ~15deg
}

[UnityTest] public IEnumerator FullWorkflow_FrameRateAbove30fps()
[UnityTest] public IEnumerator FullWorkflow_SOFAStepUnder20ms()
[UnityTest] public IEnumerator FullWorkflow_ResectionUnder500ms()
[UnityTest] public IEnumerator FullWorkflow_UndoResection_RebuildsSofaScene()
```

### 8.2 Biomechanical Validation

**Tests (SOFA / pytest):**
```python
# Tests/SOFA/test_validation.py
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

### 8.3 Performance Validation

**Tests (PlayMode):**
```csharp
[UnityTest] public IEnumerator Performance_RenderingAbove30fps()
[UnityTest] public IEnumerator Performance_SOFAStepBelow20ms()
[UnityTest] public IEnumerator Performance_MeshSyncBelow2ms()
[UnityTest] public IEnumerator Performance_MemoryBelow4GB()
[UnityTest] public IEnumerator Performance_PInvokeCalls_Under10PerFrame()
```

### 8.4 Native Bridge Profiling

**Tests (PlayMode):**
```csharp
[UnityTest] public IEnumerator Profiling_FEMSolveTime_Reported()
[UnityTest] public IEnumerator Profiling_CollisionTime_Reported()
[UnityTest] public IEnumerator Profiling_ConstraintIterations_Reported()
```

**Implementation:**
- `SofaProfilingData` struct returned via `sofa_get_profiling_data()`
- Feed into Unity Profiler as custom `ProfilerMarker` entries

**Requirements Covered:** REQ-PERF-01 through REQ-PERF-07, REQ-SIM-09

---

## Sprint 9: Polish, Documentation & Packaging (Week 11)

### 9.1 Bug Fixes & Edge Cases
- Address all failing tests from Sprint 8
- Handle edge cases: degenerate meshes, solver instability, extreme parameter values
- Error recovery for SOFA divergence (detect via `SofaFrameSnapshot.solver_diverged`, pause simulation, notify user)
- Scene rebuild recovery (destroy + recreate SOFA scene on critical errors)

### 9.2 Asset Pipeline Finalization
- Replace placeholder implant meshes with accurate geometry
- Final bone mesh optimization (LOD groups, material tweaks)
- Create representative test case with realistic anatomy

### 9.3 Build & Distribution
- Create standalone build (Windows/Linux)
- Run `collect_sofa_deps.sh` to bundle all SOFA DLLs
- Test deployment on clean machines (no SOFA installed)
- Test on target hardware configurations
- Create user-facing documentation

---

## Test Execution Summary

| Sprint | EditMode Tests | PlayMode Tests | C++ Tests | SOFA Tests | Total |
|--------|---------------|----------------|-----------|------------|-------|
| 0 | 0 | 8 | 8 | 0 | 16 |
| 1 | 16 | 6 | 0 | 0 | 22 |
| 2 | 0 | 6 | 6+6 | 7 | 25 |
| 3 | 8 | 6 | 3 | 8 | 25 |
| 4 | 7 | 7 | 6 | 4 | 24 |
| 5 | 12 | 4 | 0 | 0 | 16 |
| 6 | 7 | 3 | 0 | 3 | 13 |
| 7 | 7 | 4 | 0 | 0 | 11 |
| 8 | 0 | 4+4+1+3 | 0 | 5 | 17 |
| **Total** | **57** | **56** | **29** | **27** | **169** |

---

## Continuous Integration

```yaml
# .github/workflows/ci.yml (conceptual)
on: [push, pull_request]

jobs:
  native-build:
    runs-on: ubuntu-latest  # also windows-latest
    steps:
      - Checkout
      - Install SOFA v25.12 (from pre-built binaries or Docker image)
      - Build SofaAnkleBridge: cmake --build native/build
      - Run native tests: ctest --test-dir native/build
      - Upload SofaAnkleBridge.dll/.so as artifact

  unity-tests:
    needs: [native-build]
    steps:
      - Checkout
      - Download native plugin artifact
      - Deploy DLLs to Assets/Plugins/x86_64/
      - Activate Unity license
      - Run EditMode tests: unity-test-runner -testPlatform EditMode
      - Run PlayMode tests: unity-test-runner -testPlatform PlayMode
      - Upload test results

  sofa-tests:
    steps:
      - Checkout
      - Install SOFA + SofaPython3
      - Run pytest Tests/SOFA/
      - Upload test results

  build:
    needs: [native-build, unity-tests, sofa-tests]
    steps:
      - Build standalone player (Win64)
      - Run collect_sofa_deps.sh to bundle all DLLs
      - Upload artifact
```

---

## Risk Mitigation

| Risk | Severity | Mitigation | Sprint |
|------|----------|-----------|--------|
| SOFA C++ API breaks between versions | High | Pin to SOFA v25.12; version handshake in `sofa_bridge_init()`; CI tests catch regressions | 0 |
| DLL dependency hell (~80 SOFA libs) | High | Automated `collect_sofa_deps.sh` script; test deployment on clean machine in CI | 0, 9 |
| Thread safety bugs in async stepping | High | Single-mutex model; triple-buffer for snapshots; extensive threading tests under TSAN | 0, 2 |
| SOFA solver divergence with ankle params | Medium | Validate material properties in isolated SOFA tests first; divergence detection in snapshot | 2, 3 |
| Topology changes crash SOFA | Medium | Wrap all topology ops in try/catch; centroid-based removal is conservative; scene rebuild as recovery | 4 |
| CMake/SOFA build complexity | Medium | Docker-based CI build; pre-built SOFA binaries as fallback; build docs in README | 0 |
| Unity editor DLL locking | Medium | Native plugin requires editor restart for updates; document workflow; consider IL2CPP for release | 0 |
| EzySlice produces non-manifold meshes | Medium | Post-cut mesh validation; dual-representation means SOFA physics isn't affected by visual mesh issues | 4 |
| Real-time performance insufficient | Medium | Profile early (Sprint 2); use `SofaProfilingData` to identify bottlenecks; reduce mesh resolution or increase timestep | 2-8 |
| Implant meshes unavailable | Low | Use parametric placeholder geometry; replace when assets available | 5 |
| Clinical validation gap | Medium | Partner with orthopedic surgeon for parameter review at Sprint 3 and 8 | 3, 8 |
| Platform differences (Windows/Linux) | Medium | CI builds on both platforms; abstract platform paths in build scripts | 0, 9 |

---

## Architecture Decision Updates

The following architectural decisions from doc 06 are superseded by the custom integration approach:

| Original Decision | Updated Decision | Rationale |
|-------------------|-----------------|-----------|
| **AD-01:** SofaUnity in-process plugin | **AD-01 (rev):** Custom C++ native plugin (SofaAnkleBridge) | Full API control, no third-party dependency, optimized data paths |
| REQ-INT-01: SofaUnity via P/Invoke | **REQ-INT-01 (rev):** Custom bridge DLL via P/Invoke with domain-specific C API | Same P/Invoke mechanism, our own API surface |

All other architectural decisions (AD-02 through AD-05) remain unchanged.
