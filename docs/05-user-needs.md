# User Needs

> 8 User Needs driving the Unity-SOFA Ankle ROM Simulation.
> Each need maps to specific acceptance criteria and downstream requirements.

---

## UN-01: Visualize Pre-Operative Ankle Anatomy

**As a** surgeon/researcher,
**I need to** view a 3D model of the patient's ankle (tibia, talus, fibula, calcaneus, ligaments, cartilage)
**so that** I can assess the current anatomical state before planning surgery.

**Acceptance Criteria:**
- Display segmented bone meshes from CT-derived data (tibia, talus, fibula, calcaneus)
- Render articular cartilage surfaces on tibial plafond and talar dome
- Visualize key ligament structures (ATFL, CFL, PTFL, deltoid complex)
- Allow orbit, zoom, pan camera controls
- Anatomical labels togglable on/off
- Mesh fidelity: 50K-100K triangles per bone, <0.5mm deviation from source data

---

## UN-02: Measure Pre-Operative Range of Motion

**As a** surgeon/researcher,
**I need to** simulate and measure the ankle's range of motion in the arthritic/pre-operative state
**so that** I can establish a baseline for comparison with post-operative results.

**Acceptance Criteria:**
- Simulate dorsiflexion/plantarflexion ROM with configurable constraints (default: ~5-10deg DF, ~15-20deg PF)
- Simulate inversion/eversion with configurable constraints
- Display real-time angle measurements (degrees) during motion
- Apply clinically realistic joint stiffness (spring/damper from ligament properties)
- Support passive ROM testing (apply external moment, measure angular excursion)
- Record and display total sagittal arc (expected: 22-31deg for arthritic ankle)
- Log force/torque at end-range positions

---

## UN-03: Simulate Bone Resection

**As a** surgeon/researcher,
**I need to** perform virtual tibial and talar bone cuts
**so that** I can visualize the resection geometry and assess bone removal volume.

**Acceptance Criteria:**
- Tibial cut: adjustable plane perpendicular to mechanical axis (target: 90deg coronal, 89deg sagittal)
- Talar cut: adjustable plane/dome geometry matching implant system
- Resection depth adjustable in 1-2mm increments (range: 2-10mm tibial, ~2mm talar)
- Real-time visual feedback of cut plane position and orientation
- Display resected bone volume (mm^3) and resection depth
- Preserve medial and lateral malleolar integrity (collision/safety zones)
- Differentiate cortical and cancellous bone in cut surface visualization
- Undo/redo resection operations

---

## UN-04: Place Implant Components

**As a** surgeon/researcher,
**I need to** position tibial and talar implant components onto the resected bone surfaces
**so that** I can evaluate implant fit, alignment, and coverage.

**Acceptance Criteria:**
- Library of implant models (at minimum: STAR 3-component, one fixed-bearing 2-component)
- Implant sizing options matching manufacturer catalogs (e.g., STAR: 5 sizes, PE: 6-10mm)
- Interactive positioning: translate and rotate implant on resected surface
- Snap-to-surface alignment assistance
- Display alignment metrics: tibiotalar angle, ADTA, posterior slope
- Flag malalignment (tibiotalar >10deg, ADTA outside 86-92deg, slope >5deg)
- Show implant-bone coverage percentage (overhang detection)
- Display implant-bone contact area and gap analysis

---

## UN-05: Measure Post-Operative Range of Motion

**As a** surgeon/researcher,
**I need to** simulate and measure the ankle's ROM after implant placement
**so that** I can compare post-op ROM to pre-op baseline and evaluate surgical outcome.

**Acceptance Criteria:**
- Simulate DF/PF ROM with implant articulation surfaces in place
- Apply material-appropriate contact mechanics (CoCr-on-UHMWPE)
- Measure total sagittal arc post-op (expected: 33-53deg)
- Calculate ROM improvement from pre-op baseline (expected: +4-14deg)
- Display side-by-side or overlay comparison of pre-op vs post-op ROM
- Simulate ligament tension changes due to resection depth and PE thickness
- Support iterative "what-if" analysis (change PE thickness, re-measure ROM)

---

## UN-06: Compare Pre-Op vs Post-Op Outcomes

**As a** surgeon/researcher,
**I need to** view a quantitative comparison of pre-operative and post-operative states
**so that** I can evaluate the predicted surgical outcome and optimize the plan.

**Acceptance Criteria:**
- Side-by-side 3D viewport: pre-op anatomy vs post-op with implants
- Comparison table: pre-op ROM vs post-op ROM (DF, PF, total arc, inversion, eversion)
- Alignment metrics comparison (pre-op deformity vs post-op correction)
- Estimated AOFAS score improvement (based on ROM and alignment data)
- Contact pressure distribution heatmap (pre-op vs post-op)
- Export comparison report (PDF or structured data)

---

## UN-07: Achieve Biomechanically Accurate Simulation

**As a** researcher,
**I need** the simulation physics to be grounded in validated biomechanical data
**so that** simulation results are credible for research and clinical decision support.

**Acceptance Criteria:**
- Bone material properties: cortical E=17-20 GPa, cancellous E=0.2-5 GPa, nu=0.3
- Cartilage properties: E=0.7-1.19 MPa, nu=0.14-0.49
- Ligament behavior: nonlinear elastic (toe region + linear region), stiffness per published values
- Implant materials: CoCr 200-250 GPa, Ti 110-116 GPa, UHMWPE 0.8 GPa
- Contact forces within physiological range (up to 5x body weight under gait loading)
- FEM simulation using validated methods (corotational FEM, implicit time integration)
- ROM results within +/-5deg of published clinical data for equivalent cases
- Simulation validated against at least 3 published FEA studies of ankle arthroplasty

---

## UN-08: Real-Time Interactive Performance

**As a** user,
**I need** the simulation to run at interactive frame rates
**so that** I can manipulate the view, perform cuts, and place implants without noticeable lag.

**Acceptance Criteria:**
- Visualization rendering: >= 30 fps on desktop (>=90 fps if VR)
- Physics simulation step: <=20ms per step (50Hz minimum)
- Bone resection operation: <500ms from cut command to visual update
- Implant placement: real-time dragging at >=30 fps
- ROM sweep animation: smooth playback at >=30 fps
- Data transfer (SOFA -> Unity): <2ms per frame for mesh position updates
- Total system latency (user input to visual response): <100ms
