# Software Component Designs

> Detailed component designs for each major subsystem.
> Each component specifies its interface, dependencies, and data contracts.

---

## Component 1: WorkflowManager

**Purpose:** Orchestrates the surgical simulation phases as a finite state machine.

**Scene Object:** `SimulationController` (root-level GameObject)

```
States:
  INIT -> PRE_OP_VIEW -> PRE_OP_ROM -> RESECTION -> IMPLANT_PLACEMENT -> POST_OP_ROM -> COMPARISON

Transitions:
  PRE_OP_VIEW     -- "Measure ROM" -->  PRE_OP_ROM
  PRE_OP_ROM      -- "Proceed"     -->  RESECTION
  RESECTION       -- "Place Implant" -> IMPLANT_PLACEMENT
  IMPLANT_PLACEMENT -- "Test ROM"   --> POST_OP_ROM
  POST_OP_ROM     -- "Compare"      --> COMPARISON
  Any state       -- "Back"         --> Previous state (with undo)
```

**Class Design:**

```csharp
// WorkflowManager.cs
public enum SimulationPhase
{
    Init,
    PreOpView,
    PreOpROM,
    Resection,
    ImplantPlacement,
    PostOpROM,
    Comparison
}

public class WorkflowManager : MonoBehaviour
{
    [SerializeField] private SimulationPhase currentPhase;

    // Component references
    [SerializeField] private AnatomyManager anatomyManager;
    [SerializeField] private ROMEngine romEngine;
    [SerializeField] private ResectionEngine resectionEngine;
    [SerializeField] private ImplantManager implantManager;
    [SerializeField] private ComparisonEngine comparisonEngine;
    [SerializeField] private SofaBridge sofaBridge;

    // Events
    public event Action<SimulationPhase, SimulationPhase> OnPhaseChanged;

    public void TransitionTo(SimulationPhase newPhase);
    public void GoBack();
    public SimulationPhase GetCurrentPhase();
    public bool CanTransitionTo(SimulationPhase target);
}
```

**Dependencies:** All other application-layer components. UI navigation panel.

---

## Component 2: AnatomyManager

**Purpose:** Loads, manages, and displays anatomical structures.

**Scene Objects:** `AnatomyRoot/Tibia`, `AnatomyRoot/Talus`, `AnatomyRoot/Fibula`, `AnatomyRoot/Calcaneus`, `AnatomyRoot/Cartilage`, `AnatomyRoot/Ligaments`

```csharp
// AnatomyManager.cs
public class AnatomyManager : MonoBehaviour
{
    [SerializeField] private AnatomyConfig config;  // ScriptableObject

    // Bone references (assigned in inspector or loaded at runtime)
    public GameObject tibia;
    public GameObject talus;
    public GameObject fibula;
    public GameObject calcaneus;
    public GameObject cartilage;
    public GameObject[] ligaments;

    // Public API
    public void LoadAnatomy(AnatomyConfig config);
    public void SetStructureVisibility(string structureName, bool visible);
    public void SetLabelsVisible(bool visible);
    public Mesh GetBoneMesh(BoneType type);
    public Bounds GetAnatomyBounds();
}

// AnatomyConfig.cs (ScriptableObject)
[CreateAssetMenu(menuName = "Ankle Sim/Anatomy Config")]
public class AnatomyConfig : ScriptableObject
{
    public MeshReference tibiaMesh;
    public MeshReference talusMesh;
    public MeshReference fibulaMesh;
    public MeshReference calcaneusMesh;
    public CartilageConfig cartilageConfig;
    public LigamentConfig[] ligamentConfigs;
}

public enum BoneType { Tibia, Talus, Fibula, Calcaneus }

[Serializable]
public class LigamentConfig
{
    public string name;              // e.g., "ATFL"
    public Vector3 originPoint;      // attachment on bone A
    public Vector3 insertionPoint;   // attachment on bone B
    public float stiffness;          // N/mm
    public float restLength;         // mm
    public Color displayColor;
}
```

**Dependencies:** Asset loading system, SofaBridge (to register anatomy in SOFA scene).

---

## Component 3: ROMEngine

**Purpose:** Measures and records joint range of motion in pre-op and post-op states.

```csharp
// ROMEngine.cs
public class ROMEngine : MonoBehaviour
{
    [SerializeField] private SofaBridge sofaBridge;
    [SerializeField] private ROMConfig config;

    // State
    private ROMRecord preOpRecord;
    private ROMRecord postOpRecord;
    private bool isRecording;

    // Public API
    public void StartROMSweep(ROMSweepParams sweepParams);
    public void StopROMSweep();
    public float GetCurrentAngle(RotationAxis axis);
    public float GetCurrentTorque(RotationAxis axis);
    public ROMRecord GetPreOpRecord();
    public ROMRecord GetPostOpRecord();
    public ROMComparison CompareRecords();

    // Events
    public event Action<float, float> OnAngleUpdated;  // (angle, torque)
    public event Action<ROMRecord> OnSweepCompleted;
}

// ROMRecord.cs
[Serializable]
public class ROMRecord
{
    public float maxDorsiflexion;    // degrees (positive)
    public float maxPlantarflexion;  // degrees (positive)
    public float totalSagittalArc;   // DF + PF
    public float maxInversion;
    public float maxEversion;
    public float endRangeTorqueDF;   // Nm
    public float endRangeTorquePF;   // Nm
    public AnimationCurve angleTorqueCurve;
    public DateTime timestamp;
}

// ROMComparison.cs
[Serializable]
public class ROMComparison
{
    public ROMRecord preOp;
    public ROMRecord postOp;
    public float dfImprovement;      // degrees
    public float pfImprovement;      // degrees
    public float totalArcImprovement;
}

public enum RotationAxis { DorsiPlantarflexion, InversionEversion, InternalExternalRotation }

// ROMConfig.cs (ScriptableObject)
[CreateAssetMenu(menuName = "Ankle Sim/ROM Config")]
public class ROMConfig : ScriptableObject
{
    public float sweepTorque = 5f;         // Nm applied during passive ROM test
    public float sweepSpeed = 10f;         // deg/sec
    public float measurementInterval = 0.1f; // seconds between samples
    public float endRangeThreshold = 0.5f;   // deg/sec - below this = end range reached
}
```

**Dependencies:** SofaBridge (for constraint configuration and force readback).

---

## Component 4: ResectionEngine

**Purpose:** Manages bone resection (cut plane positioning, execution, undo).

```csharp
// ResectionEngine.cs
public class ResectionEngine : MonoBehaviour
{
    [SerializeField] private SofaBridge sofaBridge;
    [SerializeField] private AnatomyManager anatomyManager;

    // Cut state
    private List<ResectionRecord> resectionHistory;
    private int historyIndex;

    // Public API
    public void SetCutPlane(BoneType target, Plane cutPlane);
    public CutPreview PreviewCut(BoneType target, Plane cutPlane);
    public ResectionResult ExecuteCut(BoneType target, Plane cutPlane);
    public void Undo();
    public void Redo();
    public float GetResectedVolume();  // mm^3

    // Events
    public event Action<CutPreview> OnCutPreviewUpdated;
    public event Action<ResectionResult> OnCutExecuted;
}

// ResectionRecord.cs
[Serializable]
public class ResectionRecord
{
    public BoneType targetBone;
    public Plane cutPlane;
    public float resectionDepth;       // mm
    public float coronalAngle;         // degrees
    public float sagittalAngle;        // degrees
    public float volumeRemoved;        // mm^3
    public Mesh preCutMesh;            // for undo
}

// CutPreview.cs
public struct CutPreview
{
    public Vector3[] intersectionContour;  // polygon where plane meets bone
    public float estimatedDepth;           // mm at deepest point
    public float estimatedVolume;          // mm^3
    public bool intersectsMalleolus;       // safety warning
    public float distanceToMalleolus;      // mm
}

// ResectionResult.cs
public struct ResectionResult
{
    public bool success;
    public Mesh proximalFragment;     // retained bone
    public Mesh distalFragment;       // resected piece
    public float actualVolume;        // mm^3 removed
    public float actualDepthMin;      // mm
    public float actualDepthMax;      // mm
}
```

**Dependencies:** AnatomyManager (for bone meshes), SofaBridge (for FEM topology update), EzySlice (for mesh cutting).

---

## Component 5: ImplantManager

**Purpose:** Manages implant selection, sizing, positioning, and alignment validation.

```csharp
// ImplantManager.cs
public class ImplantManager : MonoBehaviour
{
    [SerializeField] private ImplantLibrary implantLibrary;
    [SerializeField] private SofaBridge sofaBridge;

    // Current state
    private ImplantSystem currentSystem;
    private GameObject tibialComponent;
    private GameObject talarComponent;
    private GameObject bearingComponent;  // null for fixed-bearing

    // Public API
    public void SelectSystem(string systemName);
    public void SetTibialSize(int sizeIndex);
    public void SetTalarSize(int sizeIndex);
    public void SetBearingThickness(float thicknessMm);
    public void PositionTibialComponent(Vector3 position, Quaternion rotation);
    public void PositionTalarComponent(Vector3 position, Quaternion rotation);
    public AlignmentMetrics GetAlignmentMetrics();
    public CoverageAnalysis GetCoverageAnalysis();
    public void FinalizeImplantPlacement();

    // Events
    public event Action<AlignmentMetrics> OnAlignmentChanged;
    public event Action<AlignmentWarning> OnAlignmentWarning;
}

// ImplantLibrary.cs (ScriptableObject)
[CreateAssetMenu(menuName = "Ankle Sim/Implant Library")]
public class ImplantLibrary : ScriptableObject
{
    public ImplantSystem[] systems;
}

[Serializable]
public class ImplantSystem
{
    public string name;                    // e.g., "STAR", "INFINITY"
    public BearingType bearingType;        // Mobile or Fixed
    public ImplantSize[] tibialSizes;
    public ImplantSize[] talarSizes;
    public float[] bearingThicknessOptions; // mm
    public MaterialProperties tibialMaterial;
    public MaterialProperties talarMaterial;
    public MaterialProperties bearingMaterial;
}

[Serializable]
public class ImplantSize
{
    public string label;           // e.g., "Size 3"
    public Vector2 dimensions;     // width x depth (mm)
    public GameObject meshPrefab;
    public Mesh collisionMesh;
}

public enum BearingType { Mobile, Fixed }

// AlignmentMetrics.cs
public struct AlignmentMetrics
{
    public float tibiotalarAngle;      // degrees (target: 0, acceptable: <10)
    public float anteriorDistalTibialAngle; // degrees (target: 89, acceptable: 86-92)
    public float posteriorSlope;       // degrees (target: <5)
    public float tibiotalarCongruence; // degrees (target: <2)
    public bool isAcceptable;          // all within range
    public List<AlignmentWarning> warnings;
}

public struct AlignmentWarning
{
    public string parameter;
    public float value;
    public float threshold;
    public string message;
}

// CoverageAnalysis.cs
public struct CoverageAnalysis
{
    public float tibialCoveragePercent;
    public float talarCoveragePercent;
    public bool hasOverhang;
    public Vector3[] overhangRegions;
    public float contactArea;         // mm^2
    public float maxGap;              // mm
}
```

**Dependencies:** ResectionEngine (needs resected bone surface), SofaBridge (for contact analysis).

---

## Component 6: ComparisonEngine

**Purpose:** Generates pre-op vs post-op comparison data and visualizations.

```csharp
// ComparisonEngine.cs
public class ComparisonEngine : MonoBehaviour
{
    [SerializeField] private ROMEngine romEngine;
    [SerializeField] private ImplantManager implantManager;

    // Public API
    public ComparisonReport GenerateReport();
    public void ExportReport(string filePath, ExportFormat format);

    // Events
    public event Action<ComparisonReport> OnReportGenerated;
}

// ComparisonReport.cs
[Serializable]
public class ComparisonReport
{
    // ROM comparison
    public ROMComparison romComparison;

    // Alignment comparison
    public float preOpDeformity;       // degrees varus/valgus
    public AlignmentMetrics postOpAlignment;

    // Implant details
    public string implantSystem;
    public string tibialSize;
    public string talarSize;
    public float bearingThickness;

    // Resection details
    public float tibialResectionDepth;
    public float talarResectionDepth;
    public float totalBoneRemoved;     // mm^3

    // Timestamps
    public DateTime generatedAt;
}

public enum ExportFormat { JSON, CSV }
```

---

## Component 7: SofaBridge (Integration Layer)

**Purpose:** Manages all communication between Unity and SOFA engine.

```csharp
// SofaBridge.cs
public class SofaBridge : MonoBehaviour
{
    // Sub-components
    [SerializeField] private SofaContext sofaContext;

    // Status
    public bool IsInitialized { get; private set; }
    public bool IsSolverDiverged { get; private set; }
    public float LastStepTimeMs { get; private set; }

    // Lifecycle
    public void InitializeSOFA(SimulationConfig config);
    public void ShutdownSOFA();

    // Scene construction
    public void AddRigidBone(string name, Mesh visualMesh, Mesh collisionMesh, Vector3 position);
    public void AddDeformableTissue(string name, string volumetricMeshPath,
                                     float youngModulus, float poissonRatio, float mass);
    public void AddLigament(string name, Vector3 origin, Vector3 insertion,
                            float stiffness, float restLength);
    public void AddRigidImplant(string name, Mesh mesh, Vector3 position,
                                Quaternion rotation, float mass);

    // Simulation control
    public void Step();                    // single simulation step
    public void SetTimestep(float dt);
    public void Pause();
    public void Resume();

    // Data readback
    public Vector3[] GetMeshPositions(string sofaNodeName);
    public float GetJointAngle(string jointName, RotationAxis axis);
    public float GetJointTorque(string jointName, RotationAxis axis);
    public Vector3[] GetContactForces(string bodyA, string bodyB);

    // Resection
    public void ExecuteResection(string boneName, Plane cutPlane);
    public int[] GetRemovedElements(string boneName);  // tetrahedra indices

    // Joint constraints
    public void SetJointLimits(string jointName, RotationAxis axis, float min, float max);
    public void ApplyJointTorque(string jointName, RotationAxis axis, float torqueNm);

    // Events
    public event Action OnStepCompleted;
    public event Action<string> OnSolverDiverged;
}

// SimulationConfig.cs (ScriptableObject)
[CreateAssetMenu(menuName = "Ankle Sim/Simulation Config")]
public class SimulationConfig : ScriptableObject
{
    [Header("Solver")]
    public float timestep = 0.01f;               // seconds
    public int constraintIterations = 100;
    public float constraintTolerance = 1e-6f;
    public float rayleighStiffness = 0.1f;
    public float rayleighMass = 0.1f;

    [Header("Materials")]
    public BoneMaterialConfig corticalBone;
    public BoneMaterialConfig cancellousBone;
    public CartilageMaterialConfig cartilage;
    public LigamentMaterialConfig[] ligaments;
    public ImplantMaterialConfig[] implantMaterials;

    [Header("Collision")]
    public float alarmDistance = 2f;              // mm
    public float contactDistance = 1f;            // mm
    public float frictionCoefficient = 0.5f;

    [Header("Performance")]
    public int maxSimStepsPerFrame = 3;
    public float meshSyncThreshold = 0.01f;      // mm - skip sync if movement less than this
}

[Serializable]
public class BoneMaterialConfig
{
    public float youngModulus = 17000f;  // MPa
    public float poissonRatio = 0.3f;
}

[Serializable]
public class CartilageMaterialConfig
{
    public float youngModulus = 1.0f;    // MPa
    public float poissonRatio = 0.45f;
}
```

**Dependencies:** SofaUnity native plugin DLLs.

---

## Component Dependency Graph

```
WorkflowManager
├── AnatomyManager
│   └── SofaBridge
├── ROMEngine
│   └── SofaBridge
├── ResectionEngine
│   ├── AnatomyManager
│   ├── SofaBridge
│   └── EzySlice (third-party)
├── ImplantManager
│   ├── ResectionEngine
│   └── SofaBridge
├── ComparisonEngine
│   ├── ROMEngine
│   └── ImplantManager
└── SofaBridge
    └── SofaUnity Native (C++ DLL)
        └── SOFA Framework Libraries
```
