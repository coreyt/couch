# SOFA (Simulation Open Framework Architecture) for Biomechanical / Orthopedic Surgical Simulation

## Comprehensive Research Report

---

## Table of Contents

1. [SOFA Overview](#1-sofa-overview)
2. [FEM Simulation](#2-fem-simulation)
3. [Biomechanical Capabilities](#3-biomechanical-capabilities)
4. [Relevant Plugins](#4-relevant-plugins)
5. [Integration](#5-integration)
6. [Meshing](#6-meshing)
7. [SOFA-Unity Integration](#7-sofa-unity-integration)

---

## 1. SOFA Overview

### 1.1 What is SOFA?

SOFA (Simulation Open Framework Architecture) is an open-source C++ framework for real-time multi-physics simulation with an emphasis on medical simulation and robotics. Originally developed at INRIA (France), it has been maintained since 2015 by the SOFA Consortium. The framework targets interactive computational simulation with particular strengths in:

- Finite Element Method (FEM) based deformable body simulation
- Rigid body dynamics
- Collision detection and response
- Constraint-based mechanics
- Real-time surgical simulation

**Current version:** v25.12 (latest stable release)
**License:** LGPL (core), permitting commercial derivative works
**Repository:** [github.com/sofa-framework/sofa](https://github.com/sofa-framework/sofa)
**Documentation:** [sofa-framework.github.io/doc](https://sofa-framework.github.io/doc/)

### 1.2 Architecture: The Scene Graph

SOFA organizes simulations using a **scene graph** -- a hierarchical tree of `Node` objects that contain `Component` objects. Each node represents a logical grouping (e.g., "Liver", "Bone", "Implant") and holds the components that define that entity's behavior.

```
Root
├── RequiredPlugin (loads modules)
├── DefaultAnimationLoop
├── DefaultPipeline (collision)
├── BruteForceDetection
├── MinProximityIntersection
├── DefaultContactManager
│
├── Node: "Bone" (rigid)
│   ├── EulerImplicitSolver
│   ├── CGLinearSolver
│   ├── MechanicalObject template="Rigid3d"
│   ├── UniformMass
│   ├── FixedConstraint
│   └── Node: "Visual"
│       ├── MeshOBJLoader
│       ├── OglModel
│       └── RigidMapping
│
└── Node: "SoftTissue" (deformable)
    ├── EulerImplicitSolver
    ├── SparseLDLSolver
    ├── MeshVTKLoader
    ├── TetrahedronSetTopologyContainer
    ├── MechanicalObject template="Vec3d"
    ├── MeshMatrixMass totalMass="1.0"
    ├── TetrahedronFEMForceField method="large" youngModulus="5000" poissonRatio="0.45"
    ├── Node: "Collision"
    │   ├── TriangleSetTopologyContainer
    │   ├── MechanicalObject
    │   ├── TriangleCollisionModel
    │   ├── LineCollisionModel
    │   ├── PointCollisionModel
    │   └── BarycentricMapping
    └── Node: "Visual"
        ├── OglModel
        └── BarycentricMapping
```

**Key architectural principles:**

- **Components** are the fundamental building blocks. Each implements a specific interface (solver, force field, mass, constraint, mapping, etc.).
- **Data** fields on components are typed containers with a reflective API. Data fields support serialization (XML/Python), UI widget generation, and lazy evaluation.
- **Data dependency graph:** Two `Data` instances of the same type can be linked (`@path/to/data`), forming a directed acyclic graph with automatic dirty-flag propagation.
- **Visitors** traverse the scene graph to execute operations (e.g., computing forces, solving systems). This keeps solvers decoupled from the objects they act upon.

### 1.3 Core Component Model

#### MechanicalObject

`MechanicalObject<DataTypes>` is the central state container. It stores all degrees of freedom (DOFs) and their derivatives:

| State Vector | Description |
|---|---|
| `position` (x) | Current positions of DOFs |
| `velocity` (v) | Current velocities |
| `force` (f) | Accumulated forces |
| `dx` | Displacement increment |
| `freePosition` | Positions after free motion (before constraints) |
| `freeVelocity` | Velocities after free motion |

**Template types:**
- `Vec3d` / `Vec3f` -- 3D deformable (most common for FEM)
- `Vec2d` -- 2D deformable
- `Vec1d` -- 1D (used with BeamAdapter)
- `Vec6d` -- 6 DOF per node (Cosserat beams)
- `Rigid3d` -- rigid body (position + quaternion orientation)
- `CudaVec3f` -- GPU-accelerated variant

Nodes with different DOF types must use separate `MechanicalObject` instances.

State vectors are accessed indirectly via `MultiVecId` symbolic identifiers, which allows solvers to work abstractly without directly handling data arrays.

#### ForceField

`ForceField<DataTypes>` components compute internal and external forces. They must implement:
- `addForce(...)` -- compute and accumulate forces given the current state
- `addDForce(...)` -- compute force differentials (for implicit integration)

Multiple ForceFields can coexist on a single node; their contributions are summed.

#### Solvers (ODE + Linear)

A simulation requires two cooperating solver components:

1. **ODE Solver (Integration Scheme):** Defines how to advance from time `t` to `t+dt`. Constructs the linear system **A**x = b.
2. **Linear Solver:** Solves the resulting linear system.

### 1.4 Multi-Model Representation

SOFA's signature architectural feature is allowing **multiple geometric representations** of the same physical object:

| Representation | Purpose | Typical Topology |
|---|---|---|
| **Mechanical Model** | Physics computation (FEM) | Coarse tetrahedral mesh |
| **Collision Model** | Contact detection and response | Medium-resolution triangles |
| **Visual Model** | Rendering (OglModel) | High-resolution surface mesh |

These representations are kept coherent through **Mappings**:

- **Top-down:** Positions and velocities propagate from parent (mechanical) to children (collision, visual) via the mapping function: `p_child = J(q_parent)`
- **Bottom-up:** Forces propagate from children back to the parent via the transposed Jacobian: `f_parent = J^T * f_child`

**Key mapping types:**

| Mapping Class | Use Case |
|---|---|
| `BarycentricMapping` | Deformable-to-deformable, most common for visual/collision attachment |
| `RigidMapping` | Rigid body to attached mesh (visual, collision) |
| `SubsetMapping` | Maps a subset of parent DOFs to child |
| `SubsetMultiMapping` | Combines subsets from multiple parent objects |
| `SkinningMapping` | Bone-based deformation with blending weights |
| `IdentityMapping` | 1:1 correspondence |
| `Tetra2TriangleTopologicalMapping` | Extracts triangle surface from tet mesh |
| `Hexa2QuadTopologicalMapping` | Extracts quad surface from hex mesh |

---

## 2. FEM Simulation

### 2.1 Available ODE Solvers (Time Integration)

SOFA provides both explicit and implicit time integrators:

#### Explicit (Forward) Solvers

| Component | Method | Notes |
|---|---|---|
| `EulerExplicitSolver` | Symplectic Euler (default) | Fast, conditionally stable; symplectic by default (velocities updated before positions) |
| `CentralDifferenceSolver` | Central difference | Second-order explicit |
| `RungeKutta2Solver` | RK2 | Second-order explicit |

#### Implicit (Backward) Solvers

| Component | Method | Notes |
|---|---|---|
| `EulerImplicitSolver` | Backward Euler | Most commonly used; dissipative; supports Rayleigh damping (`rayleighMass`, `rayleighStiffness`); `trapezoidalScheme=true` reduces dissipation. Based on Baraff & Witkin 1998. |
| `NewmarkImplicitSolver` | Newmark-beta | Energy-conserving with appropriate beta/gamma. Good for structural dynamics. |
| `VariationalSymplecticSolver` | Variational integrator | Energy-preserving symplectic scheme |
| `BDFOdeSolver` | Backward Differentiation Formula | Multi-step implicit method using previous time steps |
| `StaticSolver` | Quasi-static equilibrium | No inertia/damping; finds static equilibrium under loads. Suitable for pre-operative planning. |

### 2.2 Available Linear Solvers

| Component | Type | Notes |
|---|---|---|
| `CGLinearSolver` | Iterative (Conjugate Gradient) | Default template `GraphScattered` (matrix-free); also supports assembled matrices. Good for well-conditioned problems. |
| `SparseLDLSolver` | Direct (LDL^T factorization) | Default template `CompressedRowSparseMatrixd`; also supports `CompressedRowSparseMatrixMat3x3d` (3x3 block entries). Robust for stiff/ill-conditioned systems. |
| `SparseLUSolver` | Direct (LU) | For non-symmetric systems (e.g., from non-linear mappings with geometric stiffness) |
| `AsyncSparseLDLSolver` | Asynchronous direct | For parallel/decoupled solving |
| `ShewchukPCGLinearSolver` | Preconditioned CG | Better convergence for ill-conditioned systems |
| `SofaCUDALinearSolver` (plugin) | GPU-based direct solver | Uses cuSolver; beneficial above a DOF threshold |

### 2.3 Constraint-Based Solvers

For contact and joint constraints, SOFA uses a Lagrange multiplier approach requiring:

1. **`FreeMotionAnimationLoop`**: Replaces `DefaultAnimationLoop`. Executes simulation in two phases:
   - Free motion: solve physics ignoring constraints, producing free positions/velocities
   - Correction: solve constraint problem, apply correction to satisfy constraints

2. **Constraint Solvers:**

| Component | Purpose |
|---|---|
| `GenericConstraintSolver` | Handles all constraint types; Projected Gauss-Seidel (PGS) or NNCG resolution |
| `LCPConstraintSolver` | Targets unilateral collision constraints with friction (Gauss-Seidel on LCP) |

3. **Constraint Correction** (defines how the compliance matrix W is computed):

| Component | Approach |
|---|---|
| `UncoupledConstraintCorrection` | Diagonal approximation; fast; set `compliance ~ 1/totalMass` |
| `LinearSolverConstraintCorrection` | Uses the linear solver's factorization for accurate compliance |
| `GenericConstraintCorrection` | Declares direct solvers once for all constraint objects |
| `PrecomputedConstraintCorrection` | Pre-computes compliance offline for speed |

**Typical constraint pipeline (XML):**
```xml
<FreeMotionAnimationLoop />
<GenericConstraintSolver maxIterations="100" tolerance="1e-6" />
<!-- Per-object: -->
<UncoupledConstraintCorrection compliance="0.001" />
```

### 2.4 Tetrahedral vs. Hexahedral Meshing

| Aspect | Tetrahedral | Hexahedral |
|---|---|---|
| Mesh generation | Easier from arbitrary surfaces (CGAL, Gmsh) | Harder; often requires structured grids |
| Element quality | Robust with quadratic elements; linear tets suffer from volume locking | Better accuracy per DOF for structured problems |
| Topological changes | Well-supported in SOFA (cutting, carving) | Limited support for topological changes |
| Available ForceFields | `TetrahedronFEMForceField`, `TetrahedralCorotationalFEMForceField`, `TetrahedronHyperelasticityFEMForceField` | `HexahedronFEMForceField`, `HexahedronFEMForceFieldAndMass`, `NonUniformHexahedronFEMForceFieldAndMass` |
| Common usage in SOFA | Standard for soft tissue, organ simulation | Used for structured domains, some bone models |

### 2.5 FEM ForceField Components

#### Elastic (Linear + Corotational)

| Component | Element | Notes |
|---|---|---|
| `TetrahedronFEMForceField` | Tet | Supports `method="small"` (linear), `"large"` (corotational), `"polar"` (polar decomposition corotational). Per-element Young's modulus via `localStiffnessFactor`. |
| `TetrahedralCorotationalFEMForceField` | Tet | Dedicated corotational formulation |
| `HexahedronFEMForceField` | Hex | Supports `method="small"`, `"large"`, `"polar"` |
| `HexahedronFEMForceFieldAndMass` | Hex | Combined FEM + consistent mass |
| `TriangleFEMForceField` | Triangle | Shell/membrane elements |
| `BeamFEMForceField` | Beam (1D) | Euler-Bernoulli beam elements |
| `HexahedronCompositeFEMForceFieldAndMass` | Hex | Composite/multi-resolution hex |

**Corotational FEM** is the workhorse for interactive simulation. It extracts the rotation from each element's deformation gradient, allowing a linear elastic law in the rotated frame while supporting large displacements. This gives near-linear-elastic speed with geometric nonlinearity.

```xml
<TetrahedronFEMForceField name="FEM"
    method="large"
    youngModulus="5000"
    poissonRatio="0.45" />
```

#### Hyperelastic

| Component | Element | Material Models |
|---|---|---|
| `TetrahedronHyperelasticityFEMForceField` | Tet | NeoHookean, MooneyRivlin, Ogden, STVenantKirchhoff, Costa, VerondaWestman, BoyceArruda, StableNeoHookean |

```xml
<TetrahedronHyperelasticityFEMForceField name="HyperFEM"
    materialName="NeoHookean"
    ParameterSet="3000 50000" />
<!-- ParameterSet: [mu, lambda] for NeoHookean -->
```

**Material model selection guide for biomechanics:**

| Tissue | Recommended Model | Notes |
|---|---|---|
| Soft tissue (general) | Corotational FEM | Good balance of speed and accuracy |
| Large-deformation soft tissue | NeoHookean | Simplest hyperelastic; stable for strains < 30% |
| Rubber-like / ligaments | MooneyRivlin | Adds second invariant term for better fit |
| Complex soft tissue | Ogden | Multiple-parameter fit for complex response curves |
| Tendon/ligament (anisotropic) | Custom via SofaViscoElastic or external | SOFA's built-in models are isotropic |

#### Viscoelastic (Plugin)

The **SofaViscoElastic** plugin (SofaDefrost/SofaViscoElastic) provides:
- Linear viscoelastic models (Maxwell, Kelvin-Voigt variants)
- Visco-hyperelastic models (combining hyperelastic + Maxwell branches)
- Up to second-order (two Maxwell branches in parallel)
- Nine different constitutive models
- Tetrahedral mesh support

### 2.6 Mass Components

| Component | Description | Accuracy |
|---|---|---|
| `UniformMass` | Constant mass per node; no topology needed | Lowest (suitable for rigid bodies) |
| `DiagonalMass` | Volume-integrated, lumped to diagonal | Medium |
| `MeshMatrixMass` | Full FEM-consistent mass matrix with off-diagonal terms | Highest |

---

## 3. Biomechanical Capabilities

### 3.1 Bone Modeling

#### Rigid Bones (Typical for Orthopedic Simulation)

Bones are commonly modeled as **rigid bodies** since bone stiffness is orders of magnitude greater than surrounding soft tissue:

```xml
<Node name="Bone">
    <MechanicalObject template="Rigid3d" position="0 0 0 0 0 0 1" />
    <UniformMass totalMass="1.0" />
    <FixedConstraint indices="0" />  <!-- Pin the bone in space -->
    <Node name="Visual">
        <MeshSTLLoader filename="bone_surface.stl" />
        <OglModel src="@loader" color="0.9 0.9 0.8 1" />
        <RigidMapping />
    </Node>
    <Node name="Collision">
        <MeshSTLLoader filename="bone_collision.stl" />
        <MechanicalObject />
        <TriangleCollisionModel />
        <LineCollisionModel />
        <PointCollisionModel />
        <RigidMapping />
    </Node>
</Node>
```

#### Deformable Bones (For Drilling, Cutting, Fracture)

When bone deformation matters (e.g., simulating a drill penetrating cortical bone, or stress analysis around an implant):

```xml
<Node name="DeformableBone">
    <EulerImplicitSolver />
    <SparseLDLSolver />
    <MeshVTKLoader name="loader" filename="bone_tet.vtk" />
    <TetrahedronSetTopologyContainer src="@loader" />
    <MechanicalObject template="Vec3d" />
    <MeshMatrixMass totalMass="0.5" />
    <TetrahedronFEMForceField method="large"
        youngModulus="17000"
        poissonRatio="0.3" />
    <!-- Cortical bone: E ~ 17-20 GPa, nu ~ 0.3 -->
    <!-- Cancellous bone: E ~ 0.1-2 GPa, nu ~ 0.2-0.3 -->
</Node>
```

### 3.2 Soft Tissue Simulation

#### Ligaments and Tendons

Ligaments and tendons can be modeled as:

1. **Spring bundles** (simplified): Using `StiffSpringForceField` or `RestShapeSpringsForceField` between attachment points on two bones:

```xml
<StiffSpringForceField name="ACL"
    object1="@Femur/DOFs" object2="@Tibia/DOFs"
    spring="0 5 500 10 0.1"  />
<!-- indices, stiffness, damping, restLength -->
```

2. **1D beam elements**: Using `BeamFEMForceField` for slender structures
3. **Volumetric FEM**: For detailed ligament models using tetrahedral meshes with hyperelastic material
4. **Cosserat rod theory**: Via the Cosserat plugin for rod-like structures with 6-DOF material points

#### Cartilage

Cartilage can be modeled as a thin deformable layer:
- **Hexahedral elements** are preferred for thin layers (better aspect ratio control)
- Material: nearly incompressible, viscoelastic (`poissonRatio` close to 0.5, e.g., 0.45-0.49)
- Young's modulus: 1-10 MPa depending on cartilage type
- The `SofaViscoElastic` plugin enables time-dependent response

#### Muscle (Basic)

SOFA does not include dedicated muscle activation models in the core, but:
- The `SoftRobots` plugin provides cable and pressure actuators applicable to muscle-like actuation
- External coupling with musculoskeletal solvers is possible via the Communication plugin

### 3.3 Contact Detection and Response

#### Collision Pipeline

```xml
<DefaultPipeline depth="6" />
<BruteForceBroadPhase />       <!-- or BVHNarrowPhase -->
<BVHNarrowPhase />
<MinProximityIntersection alarmDistance="0.5" contactDistance="0.3" />
<DefaultContactManager response="FrictionContact" />
```

**Collision primitive types** (all require topology containers):

| CollisionModel | Geometry |
|---|---|
| `PointCollisionModel` | Point cloud |
| `LineCollisionModel` | Edges |
| `TriangleCollisionModel` | Triangles |
| `SphereCollisionModel` | Spheres (radius per point) |
| `CubeCollisionModel` | Axis-aligned bounding boxes |
| `CylinderCollisionModel` | Cylinders |

**Intersection methods:**
- `MinProximityIntersection` -- proximity-based (detects contact before actual intersection)
- `LocalMinDistance` -- local proximity method
- `NewProximityIntersection` -- newer proximity approach
- `DiscreteIntersection` -- exact intersection tests

**Contact response methods:**
- `PenaltyContactForceField` -- penalty method (fast but can be unstable)
- `FrictionContact` -- Lagrange multiplier-based with Coulomb friction
- `BilateralInteractionConstraint` -- equality constraint (attachment)

### 3.4 Cutting and Resection (Topological Changes)

SOFA provides a sophisticated topological change framework for simulating surgical cutting, carving, and tissue removal:

#### Core Topology Components

| Component | Role |
|---|---|
| `TetrahedronSetTopologyContainer` | Stores tetrahedral topology |
| `TetrahedronSetTopologyModifier` | Provides add/remove element operations |
| `TetrahedronSetGeometryAlgorithms` | Geometric queries on topology |
| `TopologicalChangeProcessor` | Schedules topology changes (from file or events) |
| `TopologicalChangeManager` | Manages topology change events |
| `Tetra2TriangleTopologicalMapping` | Maintains surface mesh from volume mesh |

#### SofaCarving Plugin

The `SofaCarving` plugin provides the `CarvingManager` component for interactive tissue removal (e.g., bone drilling, tissue resection):

```xml
<RequiredPlugin name="SofaCarving" />
<CarvingManager active="true"
    carvingDistance="0.01" />

<Node name="Tissue">
    <TetrahedronSetTopologyContainer />
    <TetrahedronSetTopologyModifier />
    <TetrahedralCorotationalFEMForceField />
    <TriangleCollisionModel tags="CarvingSurface" />
</Node>

<Node name="Tool">
    <SphereCollisionModel tags="CarvingTool" radius="0.005" />
</Node>
```

**Known limitation:** Topological changes are not yet fully propagated through `BarycentricMapping`. When carving modifies the tetrahedral mesh, the visual model mapped via `BarycentricMapping` may not update correctly. The SOFA team has this on their topology roadmap.

**Workaround:** Use `Tetra2TriangleTopologicalMapping` to derive the collision surface directly from the tetrahedral topology, ensuring the surface updates as tetrahedra are removed.

### 3.5 Implant-Bone Interface

Modeling the interface between an orthopedic implant (rigid) and bone (rigid or deformable) can be achieved through several mechanisms:

#### AttachConstraint (Projective Coupling)

Enforces positional equality between paired nodes of two objects:

```xml
<AttachConstraint name="implant_bone_interface"
    object1="@Implant/DOFs" object2="@Bone/DOFs"
    indices1="0 1 2 3" indices2="10 11 12 13"
    constraintFactor="1.0" />
<!-- constraintFactor: 0 = loose, 1 = rigid attachment -->
```

**Requirements:** Both `MechanicalObject` instances must have the same template type. Interface nodes should be co-located (or very close) in space. The `constraintFactor` parameter (0 to 1) can model varying osseointegration levels.

#### BilateralInteractionConstraint (Lagrange Multiplier Coupling)

More physically accurate coupling using Lagrange multipliers:

```xml
<BilateralInteractionConstraint
    object1="@Implant/DOFs" object2="@Bone/DOFs"
    first_point="0 1 2" second_point="5 6 7" />
```

Requires `FreeMotionAnimationLoop` + `GenericConstraintSolver`.

#### Spring-Based Coupling

For partial/compliant interfaces (e.g., early osseointegration):

```xml
<StiffSpringForceField
    object1="@Implant/DOFs" object2="@Bone/DOFs"
    spring="0 5 10000 50 0.0
            1 6 10000 50 0.0" />
```

### 3.6 Joint Constraints

SOFA supports articulated body modeling for joint simulation:

- **BilateralInteractionConstraint:** Can model fixed joints between bodies
- **SlidingLagrangianConstraint:** Allows sliding in certain directions (e.g., translational joint)
- **ArticulatedSystemPlugin:** Provides articulation centers, hierarchies, and joint types
- **SkinningMapping:** Links rigid bones to a deformable skin/tissue envelope with blending weights (useful for musculoskeletal models)

---

## 4. Relevant Plugins

### 4.1 SofaCarving

- **Repository:** Part of SOFA main repository (`applications/plugins/SofaCarving`)
- **Key component:** `CarvingManager`
- **Purpose:** Interactive tissue carving/resection via collision-triggered element removal
- **Use case:** Bone drilling, tumor resection, soft tissue cutting
- **Tags:** `CarvingTool` (on tool collision model), `CarvingSurface` (on tissue collision model)

### 4.2 BeamAdapter

- **Repository:** [github.com/sofa-framework/BeamAdapter](https://github.com/sofa-framework/BeamAdapter)
- **Theory:** Kirchhoff rod FEM (1D elements)
- **Key features:**
  - Adaptive deployment of concentric tools (catheters inside guidewires)
  - Interactive insertion/retraction simulation
  - Specifically designed for interventional radiology
- **Use case:** Catheter, guidewire, coil, flexible instrument simulation
- **Example:** Simulating the Penumbra Neuron MAX 088 guide catheter with 160 vertices, E = 47 MPa

### 4.3 Cosserat Plugin

- **Repository:** [github.com/SofaDefrost/Cosserat](https://github.com/SofaDefrost/Cosserat)
- **Theory:** Cosserat rod theory (6 DOF per material point: 3 translations + 3 rotations)
- **Key features:**
  - Piecewise Constant Strain (PCS) approach
  - Models rigid and flexible 1D entities
  - Cable and needle simulation
- **Difference from BeamAdapter:** Cosserat treats each material point as a rigid body; broader scope including continuum robotics
- **Use case:** Needles, wires, continuum robots, concentric tube robots

### 4.4 SoftRobots

- **Repository:** [github.com/SofaDefrost/SoftRobots](https://github.com/SofaDefrost/SoftRobots)
- **Key components:**
  - `CableConstraint` / `CableActuator` -- cable-driven actuation
  - `SurfacePressureConstraint` -- pressure-based actuation
  - `CommunicationController` -- external device communication
- **Inverse plugin:** `SoftRobots.Inverse` for inverse kinematics/dynamics
- **Use case:** While designed for soft robotics, the cable/pressure actuation models are directly applicable to tendon-driven mechanisms and pneumatic surgical tools

### 4.5 STLIB (SOFA Template Library)

- **Repository:** [github.com/SofaDefrost/STLIB](https://github.com/SofaDefrost/STLIB)
- **Purpose:** Reusable Python scene templates for common SOFA patterns
- **Key templates:**
  - `ElasticMaterialObject` -- complete deformable body with collision, visual, FEM
  - `RigidObject` -- complete rigid body setup
  - Scene setup utilities (solver configuration, contact pipeline)
- **Use case:** Rapid prototyping; reduces scene boilerplate significantly

```python
from stlib3.scene import MainHeader
from stlib3.physics.deformable import ElasticMaterialObject

def createScene(rootNode):
    MainHeader(rootNode, plugins=["SoftRobots"])
    ElasticMaterialObject(rootNode,
        volumeMeshFileName="tissue.vtk",
        surfaceMeshFileName="tissue.stl",
        youngModulus=5000,
        poissonRatio=0.45,
        totalMass=1.0)
```

### 4.6 CGALPlugin

- **Repository:** [github.com/sofa-framework/CGALPlugin](https://github.com/sofa-framework/CGALPlugin)
- **License:** LGPL (plugin wrapper); CGAL itself is GPL3+ or commercial
- **Key component:** `MeshGenerationFromPolyhedron`
- **Purpose:** Generate tetrahedral volumetric meshes from closed triangle surfaces inside SOFA
- **Additional components:** `TriangularConvexHull3D`, `CylinderMesh`, `Refine2DMesh`
- **Tuning parameters:** Cell size, facet angle, facet size, cell radius-edge ratio, cell size control

```xml
<RequiredPlugin name="CGALPlugin" />
<MeshGenerationFromPolyhedron name="generator"
    inputPoints="@loader.position"
    inputTriangles="@loader.triangles"
    cellSize="5.0"
    facetAngle="30"
    facetSize="3.0"
    cellRatioEdge="3.0"
    cellSizing="5.0" />
```

### 4.7 SofaPython3

- **Repository:** [github.com/sofa-framework/SofaPython3](https://github.com/sofa-framework/SofaPython3)
- **Documentation:** [sofapython3.readthedocs.io](https://sofapython3.readthedocs.io)
- **Purpose:** Python 3 bindings for SOFA -- scene creation, runtime control, data access
- **Modules:** `Sofa.Core`, `Sofa.Simulation`, `Sofa.Types`, `Sofa.Helper`, `Sofa.Gui`
- See [Section 5.1](#51-sofapython3-api) for detailed API reference

### 4.8 ModelOrderReduction

- **Repository:** [github.com/SofaDefrost/ModelOrderReduction](https://github.com/SofaDefrost/ModelOrderReduction)
- **License:** GPL
- **Documentation:** [modelorderreduction.readthedocs.io](https://modelorderreduction.readthedocs.io)
- **Theory:** Proper Orthogonal Decomposition (POD) + hyper-reduction
- **Workflow:**
  1. **Offline phase:** Run full simulation under various configurations, collect state "snapshots"
  2. **POD:** Extract reduced basis vectors capturing dominant deformation modes
  3. **Hyper-reduction:** Select a small subset of mesh elements for computation
  4. **Online phase:** Run reduced model in real-time
- **Speed-up:** Dramatic reduction in DOFs and computational cost; recommended when mesh exceeds ~2200 vertices
- **Use case:** Real-time surgical simulation where full FEM would be too slow

### 4.9 SofaViscoElastic

- **Repository:** [github.com/SofaDefrost/SofaViscoElastic](https://github.com/SofaDefrost/SofaViscoElastic)
- **Purpose:** Linear viscoelastic and visco-hyperelastic constitutive laws on tetrahedral meshes
- **Models:** 9 different viscoelastic models combining hyperelastic base + Maxwell branches (up to 2nd order)
- **Developed by:** Brubotics (VUB) + DEFROST (INRIA Lille)

---

## 5. Integration

### 5.1 SofaPython3 API

#### Scene Creation

```python
import Sofa
import SofaRuntime

# Import required plugins
SofaRuntime.importPlugin("Sofa.Component")

def createScene(rootNode):
    # Configure root node
    rootNode.gravity = [0, -9.81, 0]
    rootNode.dt = 0.01

    # Add plugins
    rootNode.addObject('RequiredPlugin', name='SofaPlugins',
        pluginName=['Sofa.Component.StateContainer',
                     'Sofa.Component.SolidMechanics.FEM.Elastic',
                     'Sofa.Component.Mass',
                     'Sofa.Component.ODESolver.Backward',
                     'Sofa.Component.LinearSolver.Direct'])

    # Create object node
    tissue = rootNode.addChild('Tissue')
    tissue.addObject('EulerImplicitSolver', rayleighStiffness=0.1, rayleighMass=0.1)
    tissue.addObject('SparseLDLSolver', template='CompressedRowSparseMatrixd')

    # Load mesh
    tissue.addObject('MeshVTKLoader', name='loader', filename='tissue.vtk')
    tissue.addObject('TetrahedronSetTopologyContainer', src='@loader')
    tissue.addObject('TetrahedronSetTopologyModifier')

    # Mechanical DOFs
    tissue.addObject('MechanicalObject', template='Vec3d', name='dofs')
    tissue.addObject('MeshMatrixMass', totalMass=1.0)

    # FEM
    tissue.addObject('TetrahedronFEMForceField',
        method='large', youngModulus=5000, poissonRatio=0.45)

    # Visual sub-node
    visual = tissue.addChild('Visual')
    visual.addObject('MeshSTLLoader', name='vloader', filename='tissue_surface.stl')
    visual.addObject('OglModel', src='@vloader', color='1 0 0 1')
    visual.addObject('BarycentricMapping')

    return rootNode
```

#### Headless Simulation Loop (No GUI)

```python
import Sofa
import Sofa.Simulation
import SofaRuntime

SofaRuntime.importPlugin("Sofa.Component")

root = Sofa.Core.Node("root")
createScene(root)
Sofa.Simulation.init(root)

# Run 1000 steps headlessly
for step in range(1000):
    Sofa.Simulation.animate(root, root.dt.value)

# Access results
positions = root.Tissue.dofs.position.value
print(f"Final positions shape: {positions.shape}")
```

#### Custom Controller (Event-Driven)

```python
import Sofa.Core

class MyController(Sofa.Core.Controller):
    def __init__(self, *args, **kwargs):
        Sofa.Core.Controller.__init__(self, *args, **kwargs)
        self.root = kwargs.get('rootNode')

    def onAnimateBeginEvent(self, event):
        # Called before each simulation step
        positions = self.root.Tissue.dofs.position.value
        # ... process positions, apply forces, etc.

    def onAnimateEndEvent(self, event):
        # Called after each simulation step
        pass

    def onKeypressedEvent(self, event):
        if event['key'] == 'A':
            # Handle key press
            pass
```

#### Data Access (Read/Write at Runtime)

```python
# Read positions as numpy array
pos = root.Tissue.dofs.position.value         # numpy array (N, 3)
vel = root.Tissue.dofs.velocity.value         # numpy array (N, 3)
forces = root.Tissue.dofs.force.value         # numpy array (N, 3)

# Write positions
with root.Tissue.dofs.position.writeable() as positions:
    positions[0] = [1.0, 2.0, 3.0]

# Access component Data fields
young_modulus = root.Tissue.FEM.youngModulus.value
root.Tissue.FEM.youngModulus.value = 10000

# Fast hierarchical access
val = root["Tissue.dofs.position"]
```

#### Adding Nodes at Runtime

```python
# Create new node dynamically
marker = rootNode.addChild("RuntimeNode")
marker.addObject('MechanicalObject', template='Vec3d', position=[[0,0,0]])
marker.addObject('SphereCollisionModel', radius=0.01)
marker.init()  # Must manually init nodes added after scene init
Sofa.Simulation.initVisual(marker)  # Required for runtime visual models
```

### 5.2 libSOFA (C++ Library Integration)

SOFA is fundamentally a **bundle of C++ libraries**, not just a standalone application. You can embed SOFA's physics engine into any C++ application:

```cpp
#include <sofa/simulation/graph/DAGSimulation.h>
#include <sofa/simulation/graph/init.h>
#include <sofa/helper/system/PluginManager.h>

// Initialize SOFA
sofa::simulation::graph::init();

// Load plugins
sofa::helper::system::PluginManager::getInstance().loadPlugin("Sofa.Component");

// Create scene programmatically or load from file
auto root = sofa::simulation::getSimulation()->createNewNode("root");
// ... add components ...

// Initialize and run
sofa::simulation::getSimulation()->init(root.get());
for (int i = 0; i < 1000; ++i) {
    sofa::simulation::getSimulation()->animate(root.get(), 0.01);
}
```

### 5.3 Communication Plugins

#### Communication Plugin (ZMQ, OSC, VRPN)

- **Repository:** [github.com/sofa-framework/Communication](https://github.com/sofa-framework/Communication)
- **Architecture:** `ServerCommunication` (abstract base) with protocol-specific implementations
- **Supported protocols:** ZMQ, OSC, VRPN

**Key components:**

| Component | Role |
|---|---|
| `ServerCommunication` | Abstract server; configure as `"sender"` or `"receiver"` |
| `Subscriber` | Defines which Data fields to send/receive and their subject/topic |

**ZMQ-specific configuration:**

```xml
<RequiredPlugin name="Communication" />
<ServerCommunicationZMQ name="zmqServer"
    job="sender"
    port="5556"
    refreshRate="30"
    pattern="publish/subscribe" />

<Subscriber name="sub1"
    communication="@zmqServer"
    subject="positions"
    target="@Tissue/dofs"
    datas="position" />
```

**ZMQ patterns supported:**
- `publish/subscribe` -- one-way data streaming (default)
- `request/reply` -- bidirectional request-response

### 5.4 Data Exchange Formats

| Format | Loader Component | Volume? | Notes |
|---|---|---|---|
| VTK (.vtk) | `MeshVTKLoader` | Yes | Most common for volumetric meshes in SOFA |
| VTU (.vtu) | `MeshVTKLoader` | Yes | XML variant of VTK |
| Gmsh (.msh) | `MeshGmshLoader` | Yes | Version 2 ASCII recommended |
| OBJ (.obj) | `MeshOBJLoader` | Surface only | Standard surface mesh |
| STL (.stl) | `MeshSTLLoader` | Surface only | Binary or ASCII |
| PLY (.ply) | `MeshPLYLoader` | Surface only | Stanford format |
| OFF (.off) | `MeshOFFLoader` | Surface only | Simple mesh format |
| XPBD (.xpbd) | `MeshXspLoader` | No | SOFA particle format |

**Exporters:**
- `VTKExporter` -- exports simulation state to VTK/VTU files per timestep
- `MeshExporter` -- general mesh export
- `WriteState` / `ReadState` -- binary state serialization for recording/replay

### 5.5 Headless Operation as Computation Backend

SOFA can serve as a pure computation backend in three ways:

#### 1. runSofa Batch Mode

```bash
# Run 500 steps, print timing every 10 steps
runSofa -g batch -n 500 --computationTimeSampling 10 scene.scn

# Output timing as JSON
runSofa -g batch -n 1000 --computationTimeOutputType json scene.py
```

The `BatchGUI` class (`sofa::gui::batch::BatchGUI`) provides the headless execution environment.

#### 2. Pure Python (No runSofa)

```python
import Sofa, Sofa.Simulation, SofaRuntime
# Create, init, and step entirely from Python -- see Section 5.1
```

#### 3. Custom C++ Application

Link against SOFA libraries and drive the simulation loop programmatically (see Section 5.2).

### 5.6 GPU / CUDA Plugins

#### SofaCUDA

- **Location:** `applications/plugins/SofaCUDA` in SOFA repository
- **Mechanism:** Template specialization -- `CudaVec3f` replaces `Vec3f`

**Switching CPU to GPU in a scene:**

```xml
<!-- CPU version -->
<MechanicalObject template="Vec3f" />

<!-- GPU version -->
<RequiredPlugin name="SofaCUDA" />
<MechanicalObject template="CudaVec3f" />
```

**All components templated with CudaVec3Types automatically use GPU computation**, including:
- `MechanicalObject<CudaVec3f>`
- `TetrahedronFEMForceField<CudaVec3f>`
- `UniformMass<CudaVec3f>`
- `BoxROI<CudaVec3f>`
- CGLinearSolver (GPU variant)

**CMake options:**

| Option | Description |
|---|---|
| `PLUGIN_SOFACUDA` | Enable the SofaCUDA plugin |
| `SOFACUDA_DOUBLE` | Enable double-precision (requires GPU with compute capability >= 1.3) |
| `SOFACUDA_PRECISE` | IEEE 754-compliant floating point |
| `SOFACUDA_CUBLAS` | Enable cuBLAS |
| `SOFACUDA_THRUST` | Enable Thrust (recommended for RadixSort; included in CUDA SDK 4.0+) |

**Limitations:**
- Not all components have CUDA implementations (e.g., no Lagrange multiplier collision, no SparseLDLSolver)
- Penalty contact and CG solver are available on GPU
- Beneficial primarily for large DOF counts; small simulations may be faster on CPU

#### SofaCUDALinearSolver

- **Repository:** [github.com/SofaDefrost/SofaCUDALinearSolver](https://github.com/SofaDefrost/SofaCUDALinearSolver)
- **Purpose:** GPU-based direct linear solver using cuSolver
- **Use case:** Large linear systems where CPU direct solvers are bottlenecked
- **Note:** Performance crossover depends on hardware; smaller systems may be faster on CPU

---

## 6. Meshing

### 6.1 CGAL Integration (Inside SOFA)

The `CGALPlugin` enables volumetric mesh generation directly within the SOFA scene graph:

```xml
<RequiredPlugin name="CGALPlugin" />

<!-- Load surface mesh -->
<MeshSTLLoader name="surfaceLoader" filename="organ_surface.stl" />

<!-- Generate tetrahedral volume mesh -->
<MeshGenerationFromPolyhedron name="meshGen"
    inputPoints="@surfaceLoader.position"
    inputTriangles="@surfaceLoader.triangles"
    cellSize="5.0"
    facetAngle="30"
    facetSize="3.0"
    cellRatioEdge="3.0"
    cellSizing="5.0" />

<!-- Use generated mesh -->
<TetrahedronSetTopologyContainer name="topo"
    position="@meshGen.outputPoints"
    tetrahedra="@meshGen.outputTetras" />
```

**Tuning for real-time performance:**
- Default CGAL parameters often generate very fine meshes (e.g., 26,000 vertices)
- Increasing `cellSize` and `cellSizing` reduces vertex count (e.g., to ~600 for real-time)
- A mesh of ~2,250 vertices provides a good accuracy/performance balance for many applications

### 6.2 Gmsh Integration (External)

Gmsh is used as an external preprocessing tool:

**Workflow: STL/OBJ to Volumetric Mesh**

1. **Clean surface mesh:** Import STL into CAD tool (e.g., FreeCAD), convert to solid, export as STEP
2. **Volume mesh in Gmsh:**
   ```
   gmsh model.step -3 -format msh2 -o output.msh
   ```
3. **Load in SOFA:**
   ```xml
   <MeshGmshLoader filename="output.msh" />
   ```
   **Important:** Use Gmsh format Version 2 ASCII for SOFA compatibility.

**Alternative using meshio (Python):**

```python
import meshio

# Convert Gmsh to VTK
mesh = meshio.read("model.msh")
meshio.write("model.vtk", mesh)
```

**meshio** supports format conversion between: Gmsh (.msh v2.2/4.0/4.1), VTK (.vtk), VTU (.vtu), OBJ (.obj), STL (.stl), and many more.

### 6.3 Other Volume Meshing Options in SOFA

| Component | Source |
|---|---|
| `MeshTetraStuffing` | Generates tets from surface by "stuffing" (SofaMisc module) |
| `MeshGenerationFromPolyhedron` | CGAL-based tet generation |
| `CylinderMesh` | Generates cylindrical meshes (CGALPlugin) |

### 6.4 Topological Changes During Cutting

SOFA supports runtime topological modifications through a layered system:

```
TopologyContainer (stores elements)
    └── TopologyModifier (add/remove operations)
        └── TopologyAlgorithms (geometric queries)
            └── TopologicalChangeManager (event coordination)
```

**For each element type:**

| Level | Tet | Hex | Triangle |
|---|---|---|---|
| Container | `TetrahedronSetTopologyContainer` | `HexahedronSetTopologyContainer` | `TriangleSetTopologyContainer` |
| Modifier | `TetrahedronSetTopologyModifier` | `HexahedronSetTopologyModifier` | `TriangleSetTopologyModifier` |
| Algorithms | `TetrahedronSetGeometryAlgorithms` | `HexahedronSetGeometryAlgorithms` | `TriangleSetGeometryAlgorithms` |

**Key patterns for cutting:**

1. **Element removal:** Remove tetrahedra intersecting a cutting plane/tool
2. **Refinement + removal:** Subdivide elements along cut path, then separate
3. **CarvingManager:** Collision-triggered automatic removal (SofaCarving plugin)
4. **TopologicalChangeProcessor:** Script topology changes from a file

**Topological mappings** maintain correspondence between volume and surface:
- `Tetra2TriangleTopologicalMapping`: Automatically updates the surface triangle mesh when the underlying tetrahedral mesh changes
- This is critical for maintaining collision detection during cutting

---

## 7. SOFA-Unity Integration

### 7.1 SofaUnity Plugin

- **Developer:** InfinyTech3D
- **Repository:** [github.com/InfinyTech3D/SofaUnity](https://github.com/InfinyTech3D/SofaUnity)
- **Unity Asset Store:** [SOFA Physics engine for Unity](https://assetstore.unity.com/packages/tools/physics/sofa-physics-engine-for-unity-270170)
- **License:** Core freely available; advanced features (haptics, medical imaging) available on request

### 7.2 Architecture

The integration has a two-layer architecture:

```
┌─────────────────────────────────────────┐
│  Unity3D (C#)                           │
│  ┌───────────────────────────────────┐  │
│  │  SofaUnity C# Asset               │  │
│  │  - SofaUnityAPI/ (C# P/Invoke)    │  │
│  │  - Core/ (SOFA component classes)  │  │
│  │  - Modules/ (specialized types)    │  │
│  │  - Editor/ (Unity inspectors)      │  │
│  └───────────────┬───────────────────┘  │
│                  │ Native Plugin Call    │
│  ┌───────────────┴───────────────────┐  │
│  │  SofaAdvancedPhysicsAPI (C++ DLL)  │  │
│  │  (also called SofaVerseAPI)        │  │
│  │  - Wraps SOFA concepts             │  │
│  │  - Manages SOFA scene graph        │  │
│  │  - Asynchronous stepping           │  │
│  └───────────────┬───────────────────┘  │
│                  │                       │
│  ┌───────────────┴───────────────────┐  │
│  │  SOFA Libraries (C++ DLLs)         │  │
│  │  - Core simulation engine          │  │
│  │  - All loaded plugins              │  │
│  └───────────────────────────────────┘  │
└─────────────────────────────────────────┘
```

### 7.3 Communication Pattern

- SOFA runs as a **background physics engine** within the Unity process
- At each Unity `Update()`, a SOFA simulation step is requested
- Execution is **asynchronous**: SOFA's timestep can differ from Unity's frame rate
- Data flows bidirectionally: SOFA results -> Unity rendering; Unity input -> SOFA forces/constraints

### 7.4 Data Serialization

- SOFA component `Data` fields are **serialized in Unity3D**, enabling save/reload of projects
- Supported data types: bool, float, int, Vector3, vector, string
- **Mesh position transfer optimization:** Only vertices that have changed are copied (sparse vector updates)
- The SOFA DAG Node hierarchy is reconstructed in Unity's scene hierarchy
- 10 different component categories are specialized; all others use a generic interface

### 7.5 Three Levels of Integration

1. **Load existing SOFA scenes:** Load `.scn` files directly in Unity and have them execute
2. **High-level SOFA-Unity objects:** Create pre-configured simulation objects via Unity Inspector
3. **Component-level control:** Create individual SOFA components as Unity GameObjects with full parameter control

### 7.6 VR/XR Extension (SofaUnityXR)

- **Repository:** [github.com/InfinyTech3D/SofaUnityXR](https://github.com/InfinyTech3D/SofaUnityXR)
- Adds scripts for VR/XR device interaction with SOFA simulations
- Uses Unity's XR system for controller and headset integration
- Enables immersive medical training and surgical planning applications

### 7.7 Getting Started

```
1. Import SofaUnity.unitypackage in Unity Editor (Assets > Import Package > Custom Package)
2. Download SOFA DLLs from GitHub releases (e.g., SofaUnity_v23.12_windows_dll.zip)
3. Extract DLLs into the Assets/Plugins directory
4. Add SofaContext component to a GameObject to initialize SOFA
5. Load a .scn file or create SOFA objects via Unity Inspector
```

### 7.8 Alternative: Custom Socket/ZMQ Bridge

For projects requiring decoupled SOFA and Unity processes (e.g., SOFA on a compute server, Unity on a client), a custom communication bridge can be built:

```
┌──────────────┐      ZMQ/TCP       ┌──────────────┐
│ SOFA Process │ ◄──────────────► │ Unity Process │
│ (headless)   │   positions,       │ (rendering)  │
│ + ZMQ Plugin │   forces, etc.     │ + C# ZMQ     │
└──────────────┘                    └──────────────┘
```

**SOFA side (Python):**
```python
import zmq, json, numpy as np

context = zmq.Context()
socket = context.socket(zmq.PUB)
socket.bind("tcp://*:5556")

# After each simulation step:
positions = root.Tissue.dofs.position.value.tolist()
socket.send_json({"positions": positions})
```

**Unity side (C# with NetMQ):**
```csharp
using NetMQ;
using NetMQ.Sockets;

var subscriber = new SubscriberSocket();
subscriber.Connect("tcp://localhost:5556");
subscriber.Subscribe("");

// In Update():
if (subscriber.TryReceiveFrameString(out string msg)) {
    var data = JsonUtility.FromJson<SimData>(msg);
    UpdateMeshVertices(data.positions);
}
```

### 7.9 SOFA-Godot Alternative

For open-source game engine integration, the **sofa_godot** plugin ([github.com/ScheiklP/sofa_godot](https://github.com/ScheiklP/sofa_godot)) provides a similar bridge for the Godot engine, allowing SOFA scene creation within Godot's editor.

---

## Appendix: Quick Reference

### Minimal Orthopedic Simulation Scene (SofaPython3)

```python
import Sofa
import SofaRuntime

def createScene(rootNode):
    rootNode.gravity = [0, -9810, 0]  # mm/s^2 if using mm units
    rootNode.dt = 0.01

    # Plugins
    rootNode.addObject('RequiredPlugin', pluginName=[
        'Sofa.Component.StateContainer',
        'Sofa.Component.SolidMechanics.FEM.Elastic',
        'Sofa.Component.SolidMechanics.FEM.HyperElastic',
        'Sofa.Component.Mass',
        'Sofa.Component.ODESolver.Backward',
        'Sofa.Component.LinearSolver.Direct',
        'Sofa.Component.Constraint.Lagrangian.Solver',
        'Sofa.Component.Constraint.Lagrangian.Correction',
        'Sofa.Component.Collision.Detection.Algorithm',
        'Sofa.Component.Collision.Detection.Intersection',
        'Sofa.Component.Collision.Geometry',
        'Sofa.Component.Collision.Response.Contact',
        'Sofa.Component.AnimationLoop',
        'Sofa.Component.Mapping.Linear',
        'Sofa.Component.Topology.Container.Dynamic',
        'Sofa.Component.Visual',
        'Sofa.Component.IO.Mesh',
    ])

    # Constraint-based contact pipeline
    rootNode.addObject('FreeMotionAnimationLoop')
    rootNode.addObject('GenericConstraintSolver', maxIterations=100, tolerance=1e-6)
    rootNode.addObject('DefaultPipeline')
    rootNode.addObject('BruteForceBroadPhase')
    rootNode.addObject('BVHNarrowPhase')
    rootNode.addObject('MinProximityIntersection', alarmDistance=2.0, contactDistance=1.0)
    rootNode.addObject('DefaultContactManager', response='FrictionContact',
                        responseParams='mu=0.5')

    # === BONE (Rigid) ===
    bone = rootNode.addChild('Bone')
    bone.addObject('MechanicalObject', template='Rigid3d',
                    position=[0, 0, 0, 0, 0, 0, 1])
    bone.addObject('UniformMass', totalMass=1.0)
    bone.addObject('FixedConstraint', indices=[0])
    bone.addObject('UncoupledConstraintCorrection')

    boneCol = bone.addChild('Collision')
    boneCol.addObject('MeshSTLLoader', name='loader', filename='bone_collision.stl')
    boneCol.addObject('MechanicalObject')
    boneCol.addObject('TriangleCollisionModel', moving=False, simulated=False)
    boneCol.addObject('LineCollisionModel', moving=False, simulated=False)
    boneCol.addObject('PointCollisionModel', moving=False, simulated=False)
    boneCol.addObject('RigidMapping')

    boneVis = bone.addChild('Visual')
    boneVis.addObject('MeshSTLLoader', name='loader', filename='bone_visual.stl')
    boneVis.addObject('OglModel', src='@loader', color='0.9 0.85 0.7 1.0')
    boneVis.addObject('RigidMapping')

    # === SOFT TISSUE (Deformable) ===
    tissue = rootNode.addChild('Tissue')
    tissue.addObject('EulerImplicitSolver', rayleighStiffness=0.1, rayleighMass=0.1)
    tissue.addObject('SparseLDLSolver', template='CompressedRowSparseMatrixd')
    tissue.addObject('GenericConstraintCorrection')

    tissue.addObject('MeshVTKLoader', name='loader', filename='tissue.vtk')
    tissue.addObject('TetrahedronSetTopologyContainer', src='@loader')
    tissue.addObject('TetrahedronSetTopologyModifier')
    tissue.addObject('TetrahedronSetGeometryAlgorithms')

    tissue.addObject('MechanicalObject', template='Vec3d', name='dofs')
    tissue.addObject('MeshMatrixMass', totalMass=0.5)
    tissue.addObject('TetrahedronFEMForceField',
                      method='large', youngModulus=5000, poissonRatio=0.45)

    # Tissue collision
    tissueCol = tissue.addChild('Collision')
    tissueCol.addObject('TriangleSetTopologyContainer')
    tissueCol.addObject('TriangleSetTopologyModifier')
    tissueCol.addObject('Tetra2TriangleTopologicalMapping')
    tissueCol.addObject('MechanicalObject')
    tissueCol.addObject('TriangleCollisionModel')
    tissueCol.addObject('LineCollisionModel')
    tissueCol.addObject('PointCollisionModel')
    tissueCol.addObject('BarycentricMapping')

    # Tissue visual
    tissueVis = tissue.addChild('Visual')
    tissueVis.addObject('MeshSTLLoader', name='loader', filename='tissue_visual.stl')
    tissueVis.addObject('OglModel', src='@loader', color='0.8 0.2 0.2 0.8')
    tissueVis.addObject('BarycentricMapping')

    # === IMPLANT (Rigid, interactive) ===
    implant = rootNode.addChild('Implant')
    implant.addObject('EulerImplicitSolver')
    implant.addObject('CGLinearSolver', iterations=25, tolerance=1e-5, threshold=1e-5)
    implant.addObject('MechanicalObject', template='Rigid3d', name='dofs',
                       position=[0, 50, 0, 0, 0, 0, 1])
    implant.addObject('UniformMass', totalMass=0.1)
    implant.addObject('UncoupledConstraintCorrection')

    implantCol = implant.addChild('Collision')
    implantCol.addObject('MeshSTLLoader', name='loader', filename='implant_collision.stl')
    implantCol.addObject('MechanicalObject')
    implantCol.addObject('TriangleCollisionModel')
    implantCol.addObject('LineCollisionModel')
    implantCol.addObject('PointCollisionModel')
    implantCol.addObject('RigidMapping')

    implantVis = implant.addChild('Visual')
    implantVis.addObject('MeshSTLLoader', name='loader', filename='implant_visual.stl')
    implantVis.addObject('OglModel', src='@loader', color='0.7 0.7 0.8 1.0')
    implantVis.addObject('RigidMapping')

    return rootNode
```

### Key SOFA Namespaces (v25.12)

```
sofa::component::statecontainer          -> MechanicalObject
sofa::component::solidmechanics::fem::elastic    -> TetrahedronFEMForceField, HexahedronFEMForceField
sofa::component::solidmechanics::fem::hyperelastic -> TetrahedronHyperelasticityFEMForceField
sofa::component::solidmechanics::spring   -> StiffSpringForceField, RestShapeSpringsForceField
sofa::component::odesolver::backward      -> EulerImplicitSolver, NewmarkImplicitSolver
sofa::component::odesolver::forward       -> EulerExplicitSolver, StaticSolver
sofa::component::linearsolver::direct     -> SparseLDLSolver, SparseLUSolver
sofa::component::linearsolver::iterative  -> CGLinearSolver
sofa::component::constraint::lagrangian   -> GenericConstraintSolver, BilateralInteractionConstraint
sofa::component::collision::detection     -> BruteForceBroadPhase, BVHNarrowPhase
sofa::component::collision::geometry      -> TriangleCollisionModel, PointCollisionModel
sofa::component::collision::response      -> FrictionContact, PenaltyContact
sofa::component::mapping::linear          -> BarycentricMapping, IdentityMapping
sofa::component::mapping::nonlinear       -> RigidMapping
sofa::component::mass                     -> UniformMass, DiagonalMass, MeshMatrixMass
sofa::component::topology::container      -> TetrahedronSetTopologyContainer
sofa::component::animationloop            -> FreeMotionAnimationLoop
sofa::component::visual                   -> OglModel
sofa::component::io::mesh                 -> MeshVTKLoader, MeshSTLLoader, MeshOBJLoader
```

---

## Sources

### Official Documentation and Repositories
- [SOFA Framework Official Website](https://www.sofa-framework.org/)
- [SOFA GitHub Repository](https://github.com/sofa-framework/sofa)
- [SOFA Documentation](https://sofa-framework.github.io/doc/)
- [SofaPython3 Documentation](https://sofapython3.readthedocs.io/en/v23.06/)
- [SOFA Scene Graph](https://www.sofa-framework.org/community/doc/simulation-principles/scene-graph/)
- [SOFA MechanicalObject](https://www.sofa-framework.org/community/doc/simulation-principles/mechanicalobject/)
- [SOFA Mapping](https://sofa-framework.github.io/doc/simulation-principles/multi-model-representation/mapping/)
- [SOFA Collision](https://sofa-framework.github.io/doc/simulation-principles/multi-model-representation/collision/)
- [SOFA Visual Model](https://www.sofa-framework.org/community/doc/simulation-principles/multi-model-representation/visual-model/)
- [SOFA ForceField](https://sofa-framework.github.io/doc/simulation-principles/multi-model-representation/forcefield/)
- [SOFA Topology](https://sofa-framework.github.io/doc/simulation-principles/topology/)
- [SOFA Integration Scheme](https://sofa-framework.github.io/doc/simulation-principles/system-resolution/integration-scheme/)
- [SOFA Linear Solver](https://sofa-framework.github.io/doc/simulation-principles/system-resolution/linear-solver/)
- [SOFA Lagrange Constraint](https://sofa-framework.github.io/doc/simulation-principles/constraint/lagrange-constraint/)
- [SOFA Collision Models](https://sofa-framework.github.io/doc/components/collision/geometry/collisionmodels/)
- [SOFA Features](https://www.sofa-framework.org/about/features/)
- [SOFA runSofa](https://sofa-framework.github.io/doc/using-sofa/runsofa/)
- [SOFA Using CUDA](https://sofa-framework.github.io/doc/plugins/usual-plugins/using-cuda/)
- [SOFA CGAL Library](https://www.sofa-framework.org/community/doc/plugins/usual-plugins/cgal-library/)
- [SOFA Mass Documentation](https://sofa-framework.github.io/doc/simulation-principles/multi-model-representation/mass/)

### Specific Component Documentation
- [EulerImplicitSolver](https://sofa-framework.github.io/doc/components/odesolver/backward/eulerimplicitsolver/)
- [EulerExplicitSolver](https://sofa-framework.github.io/doc/components/odesolver/forward/eulerexplicitsolver/)
- [StaticSolver](https://sofa-framework.github.io/doc/components/odesolver/forward/staticsolver/)
- [NewmarkImplicitSolver](https://sofa-framework.github.io/doc/components/odesolver/backward/newmarkimplicitsolver/)
- [BDFOdeSolver](https://sofa-framework.github.io/doc/components/odesolver/backward/bdfodesolver/)
- [VariationalSymplecticSolver](https://sofa-framework.github.io/doc/components/odesolver/backward/variationalsymplecticsolver/)
- [FreeMotionAnimationLoop](https://sofa-framework.github.io/doc/components/animationloop/freemotionanimationloop/)
- [TetrahedronFEMForceField](https://sofa-framework.github.io/doc/components/solidmechanics/fem/elastic/tetrahedronfemforcefield/)
- [TetrahedronHyperelasticityFEMForceField](https://sofa-framework.github.io/doc/components/solidmechanics/fem/hyperelastic/tetrahedronhyperelasticityfemforcefield/)
- [BeamFEMForceField](https://sofa-framework.github.io/doc/components/solidmechanics/fem/elastic/beamfemforcefield/)
- [BarycentricMapping](https://sofa-framework.github.io/doc/components/mapping/linear/barycentricmapping/)
- [MeshMatrixMass](https://sofa-framework.github.io/doc/components/mass/meshmatrixmass/)
- [DiagonalMass](https://sofa-framework.github.io/doc/components/mass/diagonalmass/)

### Plugin Repositories
- [BeamAdapter](https://github.com/sofa-framework/BeamAdapter)
- [Cosserat](https://github.com/SofaDefrost/Cosserat)
- [SoftRobots](https://github.com/SofaDefrost/SoftRobots)
- [STLIB](https://github.com/SofaDefrost/STLIB)
- [CGALPlugin](https://github.com/sofa-framework/CGALPlugin)
- [SofaPython3](https://github.com/sofa-framework/SofaPython3)
- [ModelOrderReduction](https://github.com/SofaDefrost/ModelOrderReduction)
- [SofaViscoElastic](https://github.com/SofaDefrost/SofaViscoElastic)
- [SofaCUDALinearSolver](https://github.com/SofaDefrost/SofaCUDALinearSolver)
- [Communication Plugin](https://github.com/sofa-framework/Communication)
- [SofaCarving](https://github.com/sofa-framework/sofa/tree/master/applications/plugins/SofaCarving)
- [SofaCUDA](https://github.com/sofa-framework/sofa/tree/master/applications/plugins/SofaCUDA)

### Unity Integration
- [SOFA & Unity3D Full Integration](https://www.sofa-framework.org/applications/plugins/sofa-unity3d-full-integration/)
- [SofaUnity GitHub](https://github.com/InfinyTech3D/SofaUnity)
- [SofaUnityXR GitHub](https://github.com/InfinyTech3D/SofaUnityXR)
- [InfinyTech3D SofaUnity](https://infinytech3d.com/sapapi-unity3d/)
- [SOFA Physics engine for Unity (Asset Store)](https://assetstore.unity.com/packages/tools/physics/sofa-physics-engine-for-unity-270170)
- [sofa_godot (Godot alternative)](https://github.com/ScheiklP/sofa_godot)

### Publications
- [SOFA: A Multi-Model Framework for Interactive Physical Simulation (Faure et al., 2012)](https://inria.hal.science/hal-00681539)
- [SOFA - an Open Source Framework for Medical Simulation](https://inria.hal.science/inria-00319416)
- [Model Order Reduction Plugin for SOFA (INRIA)](https://project.inria.fr/modelorderreduction/)
- [CGAL Mesh Generation in SOFA](https://www.sofa-framework.org/applications/plugins/cgal-mesh-generation/)
- [Cosserat Beam: Cable & Needle](https://www.sofa-framework.org/applications/plugins/cosserat-beam-cable-needle/)
- [SOFA Supported Plugins Wiki](https://github.com/sofa-framework/sofa/wiki/Supported-SOFA-plugins)
- [Volumetric Mesh Generation Using CGAL (Soft Robotics)](https://project.inria.fr/softrobot/documentation/volumetric-mesh-generation-using-cgal-plugin/)
- [Soft Robotics Toolkit - SOFA](https://softroboticstoolkit.com/sofa)
- [BatchGUI API Reference](https://www.sofa-framework.org/api/master/sofa/html/classsofa_1_1gui_1_1batch_1_1_batch_g_u_i.html)
