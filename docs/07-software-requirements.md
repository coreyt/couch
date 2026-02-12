# Software Requirements

> Traceable requirements derived from User Needs.
> Format: REQ-[Category]-[Number]. Categories: ANAT (Anatomy), ROM, RES (Resection), IMP (Implant), SIM (Simulation), INT (Integration), PERF (Performance), UI.

---

## Anatomy & Visualization (ANAT)

| ID | Requirement | Source | Priority |
|----|------------|--------|----------|
| REQ-ANAT-01 | System shall load and render 3D bone meshes (tibia, talus, fibula, calcaneus) from FBX/OBJ files | UN-01 | Must |
| REQ-ANAT-02 | System shall render articular cartilage as a semi-transparent overlay on tibial plafond and talar dome surfaces | UN-01 | Must |
| REQ-ANAT-03 | System shall visualize ligaments (ATFL, CFL, PTFL, deltoid complex, syndesmosis) as geometric primitives (cylinders/splines) with configurable color coding | UN-01 | Should |
| REQ-ANAT-04 | System shall support orbit, zoom, and pan camera controls with smooth interpolation | UN-01 | Must |
| REQ-ANAT-05 | System shall provide toggleable anatomical labels for all displayed structures | UN-01 | Should |
| REQ-ANAT-06 | Each bone mesh shall maintain <0.5mm deviation from source segmentation data | UN-01, UN-07 | Must |
| REQ-ANAT-07 | System shall differentiate cortical and cancellous bone regions via distinct materials/colors | UN-01, UN-03 | Should |

## Range of Motion (ROM)

| ID | Requirement | Source | Priority |
|----|------------|--------|----------|
| REQ-ROM-01 | System shall simulate dorsiflexion/plantarflexion with configurable angular limits (default pre-op: 5-10deg DF, 15-20deg PF) | UN-02 | Must |
| REQ-ROM-02 | System shall simulate inversion/eversion with configurable angular limits (default: 35deg inv, 25deg ev) | UN-02 | Should |
| REQ-ROM-03 | System shall display real-time angle measurements in degrees during ROM simulation | UN-02 | Must |
| REQ-ROM-04 | System shall apply configurable spring-damper joint stiffness derived from ligament properties | UN-02, UN-07 | Must |
| REQ-ROM-05 | System shall support passive ROM testing by applying a configurable external moment and measuring resulting angular excursion | UN-02 | Must |
| REQ-ROM-06 | System shall record and display total sagittal arc (sum of DF + PF) | UN-02 | Must |
| REQ-ROM-07 | System shall log force/torque values (Nm) at end-range positions | UN-02 | Should |
| REQ-ROM-08 | System shall store ROM measurement results for pre-op/post-op comparison | UN-05, UN-06 | Must |

## Bone Resection (RES)

| ID | Requirement | Source | Priority |
|----|------------|--------|----------|
| REQ-RES-01 | System shall provide an interactive cut plane gizmo for tibial resection, adjustable in position and orientation | UN-03 | Must |
| REQ-RES-02 | Tibial cut plane shall default to perpendicular to tibial mechanical axis (90deg coronal, 89deg sagittal) | UN-03 | Must |
| REQ-RES-03 | Resection depth shall be adjustable in 1mm increments (range: 0-15mm) | UN-03 | Must |
| REQ-RES-04 | System shall provide talar resection (flat or dome geometry) configurable per implant system | UN-03 | Must |
| REQ-RES-05 | System shall display real-time visual preview of cut plane intersection with bone before committing | UN-03 | Must |
| REQ-RES-06 | System shall compute and display resected bone volume (mm^3) | UN-03 | Should |
| REQ-RES-07 | System shall implement undo/redo for resection operations (minimum 5 levels) | UN-03 | Must |
| REQ-RES-08 | System shall update SOFA FEM topology after resection (remove tetrahedra, update surface) | UN-03, UN-07 | Must |
| REQ-RES-09 | Resection shall respect safety zones around medial and lateral malleoli (warn if cut intersects within 5mm) | UN-03 | Should |

## Implant Placement (IMP)

| ID | Requirement | Source | Priority |
|----|------------|--------|----------|
| REQ-IMP-01 | System shall provide a library of at least 2 implant systems (one mobile-bearing: STAR; one fixed-bearing) | UN-04 | Must |
| REQ-IMP-02 | Each implant system shall offer multiple component sizes matching manufacturer catalogs | UN-04 | Must |
| REQ-IMP-03 | System shall support interactive 6-DOF positioning of implant components on resected bone surfaces | UN-04 | Must |
| REQ-IMP-04 | System shall display alignment metrics: tibiotalar angle, ADTA, posterior slope | UN-04 | Must |
| REQ-IMP-05 | System shall flag malalignment: tibiotalar >10deg, ADTA outside 86-92deg, slope >5deg | UN-04 | Must |
| REQ-IMP-06 | System shall calculate and display implant-bone coverage percentage | UN-04 | Should |
| REQ-IMP-07 | System shall detect and highlight implant overhang regions | UN-04 | Should |
| REQ-IMP-08 | System shall register implant as rigid body in SOFA scene for contact computation | UN-04, UN-07 | Must |
| REQ-IMP-09 | System shall support PE bearing thickness selection (range: 6-12mm per system) | UN-04 | Must |

## Simulation & Physics (SIM)

| ID | Requirement | Source | Priority |
|----|------------|--------|----------|
| REQ-SIM-01 | Bone shall be modeled with clinically validated material properties: cortical E=17-20 GPa, cancellous E=0.2-5 GPa, nu=0.3 | UN-07 | Must |
| REQ-SIM-02 | Cartilage shall be modeled with E=0.7-1.19 MPa, nu=0.14-0.49 | UN-07 | Must |
| REQ-SIM-03 | Ligaments shall be modeled with nonlinear elastic behavior (toe region + linear region) | UN-07 | Should |
| REQ-SIM-04 | Ligament stiffness values shall match published data: ATFL ~150 N/mm, CFL ~55.8 N/mm, PTFL ~63.9 N/mm | UN-07 | Must |
| REQ-SIM-05 | Implant materials shall be modeled: CoCr 200-250 GPa, Ti 110-116 GPa, UHMWPE 0.8 GPa | UN-07 | Must |
| REQ-SIM-06 | FEM simulation shall use corotational method for large deformation accuracy | UN-07 | Must |
| REQ-SIM-07 | Time integration shall use implicit method (EulerImplicit or Newmark) for stability | UN-07 | Must |
| REQ-SIM-08 | Contact simulation shall use constraint-based approach (FreeMotionAnimationLoop + GenericConstraintSolver) | UN-07 | Must |
| REQ-SIM-09 | Simulation ROM results shall be within +/-5deg of published clinical data for equivalent input conditions | UN-07 | Must |
| REQ-SIM-10 | System shall support gravity and body-weight loading configurations | UN-07 | Should |

## Integration (INT)

| ID | Requirement | Source | Priority |
|----|------------|--------|----------|
| REQ-INT-01 | SOFA shall be integrated via SofaUnity in-process plugin (C++ DLL loaded via P/Invoke) | Architecture | Must |
| REQ-INT-02 | Mesh position data shall transfer from SOFA to Unity each frame with sparse updates (changed vertices only) | UN-08 | Must |
| REQ-INT-03 | Unity user interactions (cut commands, implant positions, force application) shall be relayed to SOFA within the same frame | Architecture | Must |
| REQ-INT-04 | SOFA simulation step shall execute asynchronously from Unity render frame | Architecture | Must |
| REQ-INT-05 | System shall gracefully handle SOFA solver divergence (detect, pause, notify user) | Architecture | Must |
| REQ-INT-06 | SOFA scene shall be constructable programmatically from Unity (no requirement for pre-authored .scn files) | Architecture | Should |
| REQ-INT-07 | All biomechanical parameters shall be configurable from Unity ScriptableObjects and propagated to SOFA at initialization | Architecture | Must |

## Performance (PERF)

| ID | Requirement | Source | Priority |
|----|------------|--------|----------|
| REQ-PERF-01 | Rendering frame rate shall be >= 30 fps on a desktop with GTX 1070 or equivalent | UN-08 | Must |
| REQ-PERF-02 | SOFA physics step shall complete in <= 20ms (50Hz minimum simulation rate) | UN-08 | Must |
| REQ-PERF-03 | Bone resection visual update shall complete in < 500ms | UN-08 | Must |
| REQ-PERF-04 | Implant interactive positioning shall maintain >= 30 fps | UN-08 | Must |
| REQ-PERF-05 | SOFA-to-Unity mesh data transfer shall add < 2ms overhead per frame | UN-08 | Must |
| REQ-PERF-06 | System initialization (SOFA engine + scene construction) shall complete in < 10 seconds | UN-08 | Should |
| REQ-PERF-07 | Total memory usage shall not exceed 4 GB | UN-08 | Should |

## User Interface (UI)

| ID | Requirement | Source | Priority |
|----|------------|--------|----------|
| REQ-UI-01 | System shall provide a workflow navigation panel showing current phase and available transitions | UN-06 | Must |
| REQ-UI-02 | System shall provide a real-time ROM gauge/dial displaying current joint angles | UN-02, UN-05 | Must |
| REQ-UI-03 | System shall provide a side-by-side comparison viewport for pre-op vs post-op | UN-06 | Must |
| REQ-UI-04 | System shall provide a comparison data table (ROM, alignment, coverage metrics) | UN-06 | Must |
| REQ-UI-05 | System shall support exporting comparison results to structured data format (JSON/CSV) | UN-06 | Should |
| REQ-UI-06 | System shall provide parameter adjustment panels for resection depth, angle, implant size, PE thickness | UN-03, UN-04 | Must |
