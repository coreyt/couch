# TDD-Based Implementation Plan

> Test-Driven Development plan organized into sprints.
> Each feature starts with tests, then implementation, then integration.
> Tests use Unity Test Framework (NUnit) for C# and pytest for SOFA Python scenes.

---

## Testing Strategy

### Test Pyramid

```
         /  E2E Tests  \          <- 5-10 workflow integration tests
        / Integration    \        <- 15-20 component interaction tests
       / Unit Tests        \      <- 50-80 pure logic tests
      /____________________\
```

### Test Categories

| Category | Framework | Location | What It Tests |
|----------|-----------|----------|---------------|
| **Unit (C#)** | NUnit + Unity Test Framework | `Tests/EditMode/` | Pure logic: angle math, alignment validation, data structures |
| **Unit (SOFA)** | pytest + SofaPython3 | `Tests/SOFA/` | SOFA scene configs: material properties, solver convergence, ROM output |
| **Integration** | Unity Test Framework (Play Mode) | `Tests/PlayMode/` | Component interactions: SofaBridge + ROMEngine, ResectionEngine + AnatomyManager |
| **E2E** | Unity Test Framework (Play Mode) | `Tests/PlayMode/E2E/` | Full workflow: load anatomy -> measure ROM -> resect -> implant -> measure post-op ROM -> compare |

### Test Naming Convention
```
[MethodUnderTest]_[Scenario]_[ExpectedBehavior]
```
Example: `MeasureDorsiflexion_ArticulationAt20Degrees_Returns20()`

---

## Sprint 0: Project Scaffolding & Infrastructure (Week 1)

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
  - `AnkleSim.Editor` (editor tools)
  - `AnkleSim.Tests.EditMode`
  - `AnkleSim.Tests.PlayMode`
- Configure physics: TGS solver, 200Hz fixed timestep (0.005s)
- Import required packages

### 0.2 SOFA Integration Setup

**Tests First:**
```
TEST: SofaUnity DLLs load without error
TEST: SofaContext initializes and returns valid handle
TEST: SofaContext can create empty scene and step once
TEST: SofaContext cleanup releases all resources
```

**Implementation:**
- Import SofaUnity package
- Place SOFA DLLs in `Assets/Plugins/x86_64/`
- Create `SofaContext` wrapper MonoBehaviour
- Verify initialization/shutdown lifecycle

### 0.3 Directory & Configuration Structure

```
Assets/
├── AnkleSim/
│   ├── Core/                    # Assembly: AnkleSim.Core
│   │   ├── DataModels/          # ROMRecord, AlignmentMetrics, etc.
│   │   ├── Validation/          # Alignment validators, range checkers
│   │   └── Math/                # Angle calculations, plane geometry
│   ├── Runtime/                 # Assembly: AnkleSim.Runtime
│   │   ├── Anatomy/             # AnatomyManager
│   │   ├── ROM/                 # ROMEngine
│   │   ├── Resection/           # ResectionEngine
│   │   ├── Implant/             # ImplantManager
│   │   ├── Comparison/          # ComparisonEngine
│   │   ├── Workflow/            # WorkflowManager
│   │   ├── Bridge/              # SofaBridge, SofaContext
│   │   └── UI/                  # UI controllers
│   ├── Editor/                  # Assembly: AnkleSim.Editor
│   ├── ScriptableObjects/       # Config assets
│   ├── Prefabs/
│   ├── Materials/
│   ├── Scenes/
│   └── StreamingAssets/
│       ├── SOFA/                # .scn/.py scene files
│       └── Meshes/              # VTK/STL volumetric meshes
├── Plugins/
│   ├── SofaUnity/               # SofaUnity C# asset
│   ├── x86_64/                  # SOFA native DLLs
│   └── EzySlice/                # Mesh cutting library
├── Tests/
│   ├── EditMode/
│   │   ├── Core/
│   │   ├── Validation/
│   │   └── Math/
│   ├── PlayMode/
│   │   ├── Integration/
│   │   └── E2E/
│   └── SOFA/                    # pytest-based SOFA scene tests
```

---

## Sprint 1: Core Data Models & Anatomy Loading (Week 2)

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

## Sprint 2: SOFA Bridge & Scene Construction (Week 3)

### 2.1 SOFA Bridge Core (RED -> GREEN -> REFACTOR)

**Tests (PlayMode):**
```csharp
// Tests/PlayMode/Integration/SofaBridgeTests.cs
[UnityTest] public IEnumerator Initialize_WithValidConfig_IsInitializedTrue()
[UnityTest] public IEnumerator Initialize_WithInvalidConfig_IsInitializedFalse()
[UnityTest] public IEnumerator Step_AfterInit_CompletesWithoutError()
[UnityTest] public IEnumerator Step_ReportsTimeInMs()
[UnityTest] public IEnumerator Shutdown_ReleasesResources()
[UnityTest] public IEnumerator Shutdown_CanReinitializeAfter()
```

**Implementation:**
- `SofaBridge` MonoBehaviour wrapping SofaUnity API
- `SimulationConfig` ScriptableObject
- Init/Step/Shutdown lifecycle

### 2.2 SOFA Scene Construction (RED -> GREEN -> REFACTOR)

**Tests (SOFA / pytest):**
```python
# Tests/SOFA/test_scene_construction.py
def test_rigid_bone_creation():
    """Rigid bone node has MechanicalObject<Rigid3d> and FixedConstraint."""

def test_deformable_tissue_creation():
    """Tissue node has TetrahedronFEMForceField with correct E and nu."""

def test_ligament_creation():
    """Ligament modeled as StiffSpringForceField with correct stiffness."""

def test_collision_pipeline():
    """Scene has FreeMotionAnimationLoop and GenericConstraintSolver."""

def test_simulation_stability():
    """Run 1000 steps without solver divergence."""

def test_material_properties():
    """Cortical bone E=17000 MPa, cartilage E=1.0 MPa as configured."""
```

**Tests (PlayMode):**
```csharp
[UnityTest] public IEnumerator AddRigidBone_CreatesSOFANode()
[UnityTest] public IEnumerator AddDeformableTissue_CreatesSOFANodeWithFEM()
[UnityTest] public IEnumerator AddLigament_CreatesSpringBetweenBones()
```

**Implementation:**
- `SofaBridge.AddRigidBone()`, `AddDeformableTissue()`, `AddLigament()`
- SOFA scene graph construction from Unity parameters
- Material property propagation from ScriptableObjects to SOFA Data fields

**Requirements Covered:** REQ-INT-01, REQ-INT-06, REQ-INT-07, REQ-SIM-01, REQ-SIM-02, REQ-SIM-04

### 2.3 Mesh Synchronization (RED -> GREEN -> REFACTOR)

**Tests (PlayMode):**
```csharp
[UnityTest] public IEnumerator MeshSync_TransfersPositionsFromSOFA()
[UnityTest] public IEnumerator MeshSync_SkipsSyncBelowThreshold()
[UnityTest] public IEnumerator MeshSync_CompletesUnder2ms()
[UnityTest] public IEnumerator MeshSync_HandlesTopologyChange()
```

**Implementation:**
- Sparse vertex update (only changed vertices)
- Performance-optimized transfer using NativeArray
- Topology change handling for post-resection mesh updates

**Requirements Covered:** REQ-INT-02, REQ-PERF-05

---

## Sprint 3: ROM Engine (Week 4)

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
```

**Implementation:**
- `AngleMath` utility class (quaternion decomposition into anatomical axes)
- Pure math, no SOFA dependency

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
```

**Implementation:**
- `ROMEngine` MonoBehaviour
- Integration with SofaBridge for joint constraint configuration
- Force/torque readback via SofaBridge
- Recording and analysis

**Requirements Covered:** REQ-ROM-01 through REQ-ROM-08, REQ-SIM-06, REQ-SIM-07, REQ-SIM-09

---

## Sprint 4: Resection Engine (Week 5)

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

### 4.2 Mesh Cutting via EzySlice (RED -> GREEN -> REFACTOR)

**Tests (PlayMode):**
```csharp
// Tests/PlayMode/Integration/ResectionEngineTests.cs
[UnityTest] public IEnumerator ExecuteCut_ProducesProximalAndDistalFragments()
[UnityTest] public IEnumerator ExecuteCut_PreservesMeshIntegrity_NoOpenEdges()
[UnityTest] public IEnumerator ExecuteCut_CompletesUnder500ms()
[UnityTest] public IEnumerator PreviewCut_ShowsIntersectionContour()
[UnityTest] public IEnumerator Undo_RestoresOriginalMesh()
[UnityTest] public IEnumerator Redo_ReappliesCut()
[UnityTest] public IEnumerator SafetyCheck_WarnsWhenNearMalleolus()
```

### 4.3 SOFA Topology Update (RED -> GREEN -> REFACTOR)

**Tests (SOFA / pytest):**
```python
# Tests/SOFA/test_resection.py
def test_resection_removes_tetrahedra():
    """Cut plane removes correct tetrahedra from topology."""

def test_resection_updates_surface():
    """Tetra2TriangleTopologicalMapping updates surface after cut."""

def test_resection_maintains_collision():
    """Collision model remains valid after topology change."""

def test_resection_preserves_boundary_conditions():
    """Fixed constraints still apply after resection."""
```

**Tests (PlayMode):**
```csharp
[UnityTest] public IEnumerator ExecuteResection_UpdatesSOFATopology()
[UnityTest] public IEnumerator ExecuteResection_SyncsMeshBackToUnity()
```

**Implementation:**
- `ResectionEngine` MonoBehaviour
- EzySlice integration for visual cutting
- SOFA topology modification via SofaBridge
- Undo/redo stack (stores pre-cut mesh snapshots)

**Requirements Covered:** REQ-RES-01 through REQ-RES-09, REQ-PERF-03

---

## Sprint 5: Implant System (Week 6)

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
- SOFA rigid body registration

**Requirements Covered:** REQ-IMP-01 through REQ-IMP-09

---

## Sprint 6: Post-Op ROM & Comparison (Week 7)

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
- Reuse ROMEngine with post-op SOFA scene (includes implant nodes)
- Implant articulation surface contact via SOFA collision

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

## Sprint 7: UI & Workflow Integration (Week 8)

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

## Sprint 8: End-to-End Integration & Validation (Week 9)

### 8.1 E2E Workflow Tests (RED -> GREEN -> REFACTOR)

**Tests (PlayMode/E2E):**
```csharp
// Tests/PlayMode/E2E/FullWorkflowTests.cs
[UnityTest] public IEnumerator FullWorkflow_LoadThroughComparison_Completes()
{
    // 1. Init -> Load anatomy
    // 2. Pre-Op ROM -> record ~25deg total arc
    // 3. Resection -> tibial cut 5mm, talar cut 2mm
    // 4. Implant -> place STAR size 3, PE 8mm
    // 5. Post-Op ROM -> record ~40deg total arc
    // 6. Compare -> improvement ~15deg
}

[UnityTest] public IEnumerator FullWorkflow_FrameRateAbove30fps()
[UnityTest] public IEnumerator FullWorkflow_SOFAStepUnder20ms()
[UnityTest] public IEnumerator FullWorkflow_ResectionUnder500ms()
[UnityTest] public IEnumerator FullWorkflow_UndoResection_RestoresState()
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
```

**Requirements Covered:** REQ-PERF-01 through REQ-PERF-07, REQ-SIM-09

---

## Sprint 9: Polish, Documentation & Packaging (Week 10)

### 9.1 Bug Fixes & Edge Cases
- Address all failing tests from Sprint 8
- Handle edge cases: degenerate meshes, solver instability, extreme parameter values
- Error recovery for SOFA divergence

### 9.2 Asset Pipeline Finalization
- Replace placeholder implant meshes with accurate geometry
- Final bone mesh optimization (LOD groups, material tweaks)
- Create representative test case with realistic anatomy

### 9.3 Build & Distribution
- Create standalone build (Windows/Linux)
- Verify SOFA DLLs bundle correctly
- Test on target hardware configurations
- Create user-facing documentation

---

## Test Execution Summary

| Sprint | EditMode Tests | PlayMode Tests | SOFA Tests | Total |
|--------|---------------|----------------|------------|-------|
| 0 | 0 | 4 | 0 | 4 |
| 1 | 16 | 6 | 0 | 22 |
| 2 | 0 | 10 | 5 | 15 |
| 3 | 7 | 6 | 6 | 19 |
| 4 | 7 | 7 | 4 | 18 |
| 5 | 12 | 4 | 0 | 16 |
| 6 | 7 | 3 | 3 | 13 |
| 7 | 7 | 4 | 0 | 11 |
| 8 | 0 | 4+4 | 5 | 13 |
| **Total** | **56** | **52** | **23** | **131** |

---

## Continuous Integration

```yaml
# .github/workflows/ci.yml (conceptual)
on: [push, pull_request]

jobs:
  unity-tests:
    - Checkout
    - Activate Unity license
    - Run EditMode tests: unity-test-runner -testPlatform EditMode
    - Run PlayMode tests: unity-test-runner -testPlatform PlayMode
    - Upload test results

  sofa-tests:
    - Checkout
    - Install SOFA + SofaPython3
    - Run pytest Tests/SOFA/
    - Upload test results

  build:
    needs: [unity-tests, sofa-tests]
    - Build standalone player (Win64)
    - Upload artifact
```

---

## Risk Mitigation

| Risk | Mitigation | Sprint |
|------|-----------|--------|
| SofaUnity DLLs incompatible with Unity 6 | Test in Sprint 0; fallback to ZMQ bridge | 0 |
| SOFA solver divergence with ankle parameters | Validate material properties in isolated SOFA tests first | 2, 3 |
| EzySlice produces non-manifold meshes | Post-cut mesh validation; fallback to manual cutting algorithm | 4 |
| Real-time performance insufficient | Profile early (Sprint 2); reduce mesh resolution, increase SOFA timestep | 2-8 |
| Implant meshes unavailable | Use parametric placeholder geometry; replace when assets available | 5 |
| Clinical validation gap | Partner with orthopedic surgeon for parameter review at Sprint 3 and 8 | 3, 8 |
