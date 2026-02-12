# Software Architecture

> High-level architecture for the Unity-SOFA Ankle ROM Simulation.

---

## 1. System Context

```
┌────────────────────────────────────────────────────────────────────────┐
│                        User Workstation                                │
│                                                                        │
│  ┌──────────────────────────────────────────────────────────────────┐  │
│  │                     Unity 6 Application                          │  │
│  │                                                                  │  │
│  │  ┌──────────────┐  ┌──────────────┐  ┌───────────────────────┐  │  │
│  │  │ Presentation │  │ Application  │  │   Integration Layer   │  │  │
│  │  │    Layer     │  │    Layer     │  │                       │  │  │
│  │  │              │  │              │  │  ┌─────────────────┐  │  │  │
│  │  │ - 3D Viewer  │  │ - Workflow   │  │  │ SofaUnity       │  │  │  │
│  │  │ - UI Panels  │  │   Manager    │  │  │ Bridge          │  │  │  │
│  │  │ - ROM Gauge  │  │ - ROM Engine │  │  │ (C# <-> C++ DLL)│  │  │  │
│  │  │ - Comparison │  │ - Resection  │  │  └────────┬────────┘  │  │  │
│  │  │   Dashboard  │  │   Engine     │  │           │            │  │  │
│  │  └──────────────┘  │ - Implant    │  │  ┌────────┴────────┐  │  │  │
│  │                    │   Manager    │  │  │ SOFA Libraries   │  │  │  │
│  │                    │ - Comparison │  │  │ (native DLLs)    │  │  │  │
│  │                    │   Engine     │  │  │ - FEM Solvers    │  │  │  │
│  │                    └──────────────┘  │  │ - Collision      │  │  │  │
│  │                                      │  │ - Carving        │  │  │  │
│  │                                      │  │ - Constraints    │  │  │  │
│  │                                      │  └─────────────────┘  │  │  │
│  │                                      └───────────────────────┘  │  │
│  └──────────────────────────────────────────────────────────────────┘  │
│                                                                        │
│  ┌─────────────────────────┐  ┌──────────────────────────────────┐    │
│  │ Asset Pipeline (Offline)│  │ External Data                    │    │
│  │ DICOM -> 3D Slicer ->   │  │ - Implant geometry libraries     │    │
│  │ Blender -> FBX -> Unity │  │ - Biomechanical parameter DB     │    │
│  └─────────────────────────┘  └──────────────────────────────────┘    │
└────────────────────────────────────────────────────────────────────────┘
```

---

## 2. Architecture Style: Layered + Plugin

The system uses a **layered architecture** with a clear separation between:

| Layer | Responsibility | Technology |
|-------|---------------|------------|
| **Presentation** | 3D rendering, UI, user interaction | Unity URP/HDRP, UI Toolkit |
| **Application** | Surgical workflow orchestration, business logic | C# MonoBehaviours/ScriptableObjects |
| **Integration** | SOFA communication bridge | SofaUnity (C# P/Invoke -> C++ DLL) |
| **Simulation** | FEM physics, collision, constraints | SOFA Framework (C++ native) |
| **Data** | Anatomical models, parameters, results | ScriptableObjects, JSON, VTK/STL meshes |

---

## 3. Key Architectural Decisions

### AD-01: SofaUnity In-Process Integration (over ZMQ Cross-Process)

**Decision:** Use SofaUnity plugin for in-process SOFA integration.

**Rationale:**
- Lowest latency (~1us function call vs ~50-100us ZMQ)
- Shared memory space eliminates serialization overhead
- Proven architecture (InfinyTech3D, available on Unity Asset Store)
- Single-process deployment simplifies distribution
- Async stepping model already handles SOFA/Unity frame rate mismatch

**Tradeoff:** Couples SOFA library versions to Unity project; harder to update independently.

### AD-02: Dual Physics Model

**Decision:** Use SOFA for biomechanical FEM simulation; Unity physics for UI interaction and non-critical physics.

**Rationale:**
- SOFA provides validated FEM (corotational, hyperelastic) that Unity PhysX cannot
- SOFA handles topological changes (cutting) that Unity mesh API alone cannot physically simulate
- Unity physics handles object picking, UI collision, camera collision
- Clear separation of concerns: accuracy (SOFA) vs interactivity (Unity)

### AD-03: Render Pipeline Selection

**Decision:** Default to URP; support HDRP as optional high-fidelity mode.

**Rationale:**
- URP provides adequate visual quality for bone/implant visualization
- URP supports VR/XR at 90fps (future expansion)
- URP Deferred+ (Unity 6.1) narrows the gap with HDRP
- Custom bone/implant shaders work in both pipelines
- HDRP SSS needed only for soft tissue photorealism (secondary priority)

### AD-04: Workflow-Driven Scene Architecture

**Decision:** Single Unity scene with state-machine-driven workflow phases.

**Rationale:**
- Avoids scene loading overhead between phases
- All anatomy remains loaded; phases show/hide/modify objects
- State machine provides clear transitions and undo support
- Phases: `PreOpView` -> `PreOpROM` -> `Resection` -> `ImplantPlacement` -> `PostOpROM` -> `Comparison`

### AD-05: ScriptableObject-Based Configuration

**Decision:** Store all biomechanical parameters, implant specifications, and simulation settings in ScriptableObjects.

**Rationale:**
- Inspector-editable without code changes
- Version-controllable (serialized as YAML/JSON)
- Swappable at runtime (change implant system, patient data)
- Testable in isolation (inject mock configurations)

---

## 4. Component Architecture

```
┌──────────────────────────────────────────────────────────────────┐
│                      Unity Application                           │
│                                                                  │
│  ┌─────────────────────────────────────────────────────────┐    │
│  │                 Workflow Manager                          │    │
│  │  (State Machine: PreOp -> ROM -> Resect -> Implant ->    │    │
│  │   PostOpROM -> Compare)                                  │    │
│  └──────┬──────────┬──────────┬──────────┬─────────────────┘    │
│         │          │          │          │                        │
│  ┌──────┴───┐ ┌────┴────┐ ┌──┴──────┐ ┌┴──────────┐           │
│  │ Anatomy  │ │  ROM    │ │Resection│ │ Implant   │           │
│  │ Manager  │ │ Engine  │ │ Engine  │ │ Manager   │           │
│  │          │ │         │ │         │ │           │           │
│  │-LoadMesh │ │-Measure │ │-CutPlane│ │-LoadModel │           │
│  │-Materials│ │-Animate │ │-Execute │ │-Position  │           │
│  │-Labels   │ │-Record  │ │-Undo    │ │-Align     │           │
│  │-Segment  │ │-Compare │ │-Volume  │ │-Size      │           │
│  └──────┬───┘ └────┬────┘ └──┬──────┘ └┬──────────┘           │
│         │          │          │          │                        │
│  ┌──────┴──────────┴──────────┴──────────┴──────────────────┐   │
│  │              SOFA Bridge (Integration Layer)              │   │
│  │                                                           │   │
│  │  SofaContext        - Initialize SOFA engine              │   │
│  │  SofaSceneManager   - Build/modify SOFA scene graph      │   │
│  │  SofaMeshSync       - Transfer mesh positions             │   │
│  │  SofaForceQuery     - Read contact forces / joint torques │   │
│  │  SofaResectionCmd   - Trigger topological changes         │   │
│  │  SofaConstraintCtrl - Configure joint constraints for ROM │   │
│  └──────────────────────────┬────────────────────────────────┘   │
│                              │ P/Invoke (DllImport)              │
│  ┌───────────────────────────┴───────────────────────────────┐   │
│  │            SOFA Native Layer (C++ DLLs)                    │   │
│  │                                                            │   │
│  │  Scene Graph: Root -> Bone(Rigid3d) + Tissue(Vec3d)       │   │
│  │  Solvers: EulerImplicit + SparseLDL                       │   │
│  │  FEM: TetrahedronFEMForceField (large deformation)        │   │
│  │  Collision: FreeMotionAnimationLoop + GenericConstraint    │   │
│  │  Cutting: CarvingManager + TopologyModifier               │   │
│  │  Mappings: BarycentricMapping, RigidMapping               │   │
│  └────────────────────────────────────────────────────────────┘   │
└──────────────────────────────────────────────────────────────────┘
```

---

## 5. Data Flow

### 5.1 Initialization
```
1. Unity starts -> SofaContext initializes SOFA engine
2. AnatomyManager loads bone meshes (FBX) into Unity for rendering
3. SofaSceneManager constructs SOFA scene:
   - Rigid bones (Rigid3d + RigidMapping)
   - Deformable cartilage (Vec3d + TetrahedronFEM)
   - Spring-based ligaments (StiffSpringForceField)
   - Collision pipeline (FreeMotionAnimationLoop)
4. SofaMeshSync establishes mapping between Unity GameObjects and SOFA nodes
```

### 5.2 ROM Measurement Loop
```
1. WorkflowManager enters ROM phase
2. SofaConstraintCtrl configures joint constraints (range limits, stiffness)
3. Per Unity FixedUpdate:
   a. SofaBridge requests SOFA simulation step
   b. SOFA solves: forces -> ODE integration -> constraint correction
   c. SofaMeshSync reads updated positions from SOFA MechanicalObjects
   d. Unity updates mesh vertex positions for rendering
   e. ROMEngine reads joint angles from SofaConstraintCtrl
   f. UI displays current angle measurements
4. ROMEngine records min/max angles over sweep
```

### 5.3 Resection Flow
```
1. User positions cut plane via gizmo (Unity interaction)
2. ResectionEngine validates cut parameters (angle, depth)
3. SofaResectionCmd sends cut command to SOFA:
   - Identify tetrahedra intersecting cut plane
   - TopologyModifier removes elements
   - Tetra2TriangleTopologicalMapping updates surface mesh
4. SofaMeshSync transfers updated topology to Unity
5. Unity creates new mesh from modified surface
6. ResectionEngine calculates and displays volume removed
```

### 5.4 Implant Placement Flow
```
1. User selects implant type and size
2. ImplantManager loads implant mesh (from Addressables library)
3. User interactively positions implant on resected surface
4. SofaSceneManager adds rigid implant node to SOFA scene
5. SOFA computes contact between implant and bone
6. SofaForceQuery returns contact area and gap analysis
7. ImplantManager displays alignment metrics
```

---

## 6. Technology Stack

| Component | Technology | Version |
|-----------|-----------|---------|
| Game Engine | Unity 6 | 6000.x LTS |
| Render Pipeline | URP (primary), HDRP (optional) | 17.x |
| Physics Backend | SOFA Framework | v25.12 |
| SOFA-Unity Bridge | SofaUnity (InfinyTech3D) | Latest |
| Mesh CSG | EzySlice (MIT) | Latest |
| IPC (fallback) | NetMQ (ZeroMQ C# port) | 4.x |
| Segmentation | 3D Slicer | 5.x |
| 3D Modeling | Blender (mesh cleanup) | 4.x |
| Volumetric Meshing | CGAL (via SOFA CGALPlugin) or Gmsh | Latest |
| VR (optional) | XR Interaction Toolkit + OpenXR | 3.x |
| Language | C# (Unity), C++ (SOFA native), Python (SOFA scripting) | - |
| Testing | Unity Test Framework (NUnit), pytest (SOFA) | - |

---

## 7. Deployment Architecture

### Desktop (Primary)
```
Application Bundle/
├── UnityPlayer.exe (or .app)
├── Data/
│   ├── Managed/          (C# assemblies)
│   ├── Plugins/
│   │   ├── SofaAdvancedPhysicsAPI.dll
│   │   ├── libSofa.Core.dll
│   │   ├── libSofa.Component.*.dll
│   │   └── ... (SOFA plugin DLLs)
│   ├── StreamingAssets/
│   │   ├── scenes/       (SOFA .scn/.py scene files)
│   │   ├── meshes/       (VTK volumetric, STL collision)
│   │   └── config/       (biomechanical parameters JSON)
│   └── Resources/        (built-in assets)
├── Addressables/          (remote implant library, optional)
```

### VR Extension (Future)
- Same bundle with XR plugin loader
- URP single-pass instanced rendering
- Hand tracking for instrument manipulation via XR Hands
