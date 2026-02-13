# Vertical Spike Implementation Plan

> **Date:** 2026-02-12
> **Goal:** Retire the three highest technical risks before committing to the full sprint plan.
> **Duration:** 3 weeks (1 week per spike)
> **Prerequisite:** SOFA Framework v25.12 installed (source build or pre-built binaries)

---

## Why Spike First

The project has 3,000+ lines of planning (docs 04-12) and zero lines of running code. The plans contain educated guesses about SOFA APIs, joint modeling, and build toolchains that need ground-truth validation. The three highest risks — SOFA build/deploy, emergent joint stability, and topology modification — can all be tested cheaply before investing in 169 tests and 10 sprints of infrastructure.

**Docs 09, 10, 11, 12 are hypothesis status until spikes validate their assumptions.**

Docs 01-03 (research), 05 (user needs), 07 (requirements), 08 (component designs) remain valid reference material throughout.

---

## Spike 1: SOFA Python — Prove the Joint Model (Week 1)

**Risk addressed:** Emergent joint stability, ligament tuning, ROM accuracy

**What to build:** A standalone SofaPython3 scene (no Unity, no C++) with a minimal ankle joint.

### Checklist

```
[ ] SofaPython3 loads and runs a trivial scene (gravity + falling cube)
[ ] Create root node: FreeMotionAnimationLoop + GenericConstraintSolver + collision pipeline
[ ] Add fixed rigid tibia (MechanicalObject<Rigid3d> + FixedConstraint + collision mesh)
[ ] Add free rigid talus (MechanicalObject<Rigid3d> + UniformMass + collision mesh)
[ ] Add 3 ligament springs between tibia and talus (StiffSpringForceField)
    - ATFL: lateral side, ~70 N/mm
    - CFL: lateral side, ~40 N/mm
    - Deltoid (simplified): medial side, ~90 N/mm
[ ] Verify stable simulation: 1000 steps without divergence
[ ] Apply ConstantForceField torque to talus in sagittal plane
[ ] Measure relative orientation (tibia→talus quaternion decomposition)
[ ] Does total sagittal arc fall in 22-31 deg range? (Glazebrook 2008)
[ ] Experiment with bilinear stiffness: reduce ligament stiffness below 3% strain
[ ] Does bilinear model change ROM by 2-5 deg vs linear? (doc 11 prediction)
[ ] Record parameter values that produce valid ROM
```

### Success Criteria

- [ ] Simulation runs stably with contact + ligament springs
- [ ] Total sagittal arc is in a plausible clinical range (±10 deg of 22-31)
- [ ] Joint angle measurement via relative quaternion produces sensible values
- [ ] At least one set of ligament parameters produces stable, repeatable results

### Failure Modes & Pivots

| Failure | Pivot |
|---------|-------|
| Talus flies off / simulation diverges | Reduce timestep, increase constraint iterations, add damping. If still unstable: add BilateralInteractionConstraint to limit DOFs (sacrifice "emergent" purity for stability) |
| ROM way outside clinical range | Tune stiffness/rest-length. If unsolvable: the ligament parameter space may need optimization, flag as research task |
| Contact detection doesn't work between convex hulls | Simplify collision meshes to primitives (spheres/capsules) for spike, refine later |
| SofaPython3 won't load / version mismatch | Use SOFA GUI (runSofa) with XML scene instead. Less convenient but validates the same physics |

### Findings Log

```
Date:
SOFA version confirmed:
Actual class names used (vs doc 10/12 assumptions):
  - Animation loop:
  - Constraint solver:
  - Rigid body type:
  - Spring force field:
  - Collision models:
Parameters that produced stable ROM:
  - Timestep:
  - Constraint iterations:
  - ATFL stiffness / rest length:
  - CFL stiffness / rest length:
  - Deltoid stiffness / rest length:
  - Applied torque:
  - Measured ROM (DF / PF / total arc):
Surprises / things docs got wrong:
Decisions to revise in planning docs:
```

---

## Spike 2: C++ Build Chain — Prove the Toolchain (Week 2)

**Risk addressed:** SOFA C++ build/link/deploy, DLL dependency hell, Unity P/Invoke loading

**What to build:** A minimal native plugin that initializes SOFA, steps an empty scene, and shuts down — loaded by Unity via P/Invoke.

### Checklist

```
[ ] Identify SOFA's CMake package names (find_package targets)
    - Check: Sofa.Core, Sofa.Simulation.Graph, Sofa.Component — do these exist as packages?
    - Alternative: manual include/link paths
[ ] Create minimal CMakeLists.txt that builds SofaAnkleBridge.dll/.so
[ ] Implement 3 exported functions:
    - sofa_bridge_init(const char* plugin_dir) → int
    - sofa_step(float dt) → int
    - sofa_bridge_shutdown() → void
[ ] sofa_bridge_init loads at least one SOFA plugin via PluginManager
    - Verify: is PluginManager::loadPlugin() the right call? Does it take a string name or path?
[ ] sofa_step creates root node, calls animate, returns 0
[ ] Build succeeds on target platform (Windows and/or Linux)
[ ] Identify all transitive DLL dependencies (ldd / dumpbin)
    - Count them. Doc 10 estimates ~80. Actual count:
[ ] Copy SofaAnkleBridge + all SOFA deps to a test directory
[ ] Verify DLL loads from the test directory (not relying on system paths)
[ ] Create Unity 6 project with single C# script:
    - [DllImport("SofaAnkleBridge")] static extern int sofa_bridge_init(string dir);
    - [DllImport("SofaAnkleBridge")] static extern int sofa_step(float dt);
    - [DllImport("SofaAnkleBridge")] static extern void sofa_bridge_shutdown();
[ ] Place all DLLs in Assets/Plugins/x86_64/
[ ] Unity Play Mode: init → step → shutdown without crash
[ ] Unity Play Mode: call init a second time after shutdown (lifecycle re-entry)
```

### Success Criteria

- [ ] CMake builds the plugin against SOFA v25.12
- [ ] All transitive dependencies identified and collected
- [ ] Unity loads and calls the plugin without crash
- [ ] Init → step → shutdown → re-init lifecycle works

### Failure Modes & Pivots

| Failure | Pivot |
|---------|-------|
| `find_package(Sofa.Core)` doesn't work | Use manual `target_include_directories` + `target_link_libraries` with explicit paths. Less portable but functional |
| SOFA headers require C++20 / incompatible compiler | Match SOFA's build compiler. On Windows: must use same MSVC version SOFA was built with |
| Missing DLLs cause crash at load time | Use Dependency Walker / `ldd -r` systematically. May need to set `SOFA_ROOT` environment variable |
| Unity editor locks DLLs, can't update | Expected. Document the restart workflow. Consider out-of-editor test harness for rapid iteration |
| Plugin loads but `PluginManager::loadPlugin()` fails | Check plugin search paths. May need `sofa::helper::system::DataRepository` configuration |

### Findings Log

```
Date:
SOFA build type (source / pre-built):
Compiler used:
CMake find_package results:
  - What worked:
  - What didn't:
Actual link targets needed:
Total transitive DLL count:
DLL collection method that worked:
Unity version:
P/Invoke issues encountered:
Plugin loading issues:
PluginManager API confirmed:
Decisions to revise in planning docs:
```

---

## Spike 3: Vertical Slice — Prove the Integration (Week 3)

**Risk addressed:** C++ scene construction API accuracy, data readback via P/Invoke, end-to-end viability

**What to build:** Port the Spike 1 Python scene into C++ scene construction inside the native plugin. Read back the talus rigid body position. Display it in Unity.

### Prerequisites

- Spike 1 findings: validated scene parameters, confirmed SOFA class names
- Spike 2 findings: working build chain, confirmed CMake targets, DLL deployment

### Checklist

```
[ ] Add scene construction to C++ plugin:
    - sofa_scene_create(config) → builds root node with pipeline (use exact code from Spike 1)
    - sofa_add_rigid_bone(config) → tibia (fixed) and talus (free)
    - sofa_add_ligament(config) → 3 springs with Spike 1 parameters
    - sofa_scene_finalize() → calls sofa::simulation::init(root)
[ ] Add data readback:
    - sofa_get_rigid_position(name, float[7]) → pos + quat
[ ] Add corresponding C# P/Invoke declarations
[ ] Unity script: construct scene → step 100 times → read talus position each step
[ ] Display talus position as a moving cube/sphere in Unity scene view
[ ] Verify talus moves in a plausible arc when torque applied
[ ] Measure: how long does sofa_step take? (target: <20ms)
[ ] Measure: how long does sofa_get_rigid_position take? (target: <0.1ms)
[ ] Try async stepping: sofa_step_async + polling — does it work without deadlock?
```

### Success Criteria

- [ ] C++ scene construction produces same behavior as Python scene from Spike 1
- [ ] Talus position readable from Unity via P/Invoke
- [ ] Something moves on screen in Unity that corresponds to SOFA physics
- [ ] Step time is in a viable range for real-time use

### Failure Modes & Pivots

| Failure | Pivot |
|---------|-------|
| C++ scene construction produces different results than Python | Compare node trees. Common cause: different initialization order, missing init() call, different default parameter values |
| Step time >50ms | Reduce collision mesh resolution. Switch from BVHNarrowPhase to BruteForceBroadPhase (fewer objects). Increase timestep |
| Async step deadlocks | Fall back to synchronous stepping for now. Threading can be added incrementally once the synchronous path is solid |
| Data readback returns garbage | Check coordinate system conventions. SOFA uses mm, may use different handedness. Verify MechanicalObject accessor API |

### Findings Log

```
Date:
C++ class names that differed from Python:
Scene construction gotchas:
sofa_step timing (empty scene):
sofa_step timing (ankle scene):
sofa_get_rigid_position timing:
Async step: worked / deadlocked / skipped:
Visual result in Unity: plausible / wrong / nothing:
Parameter differences from Spike 1 (if any):
What docs 10/12 got right:
What docs 10/12 got wrong:
Ready to proceed to Sprint 0? YES / NO / NEEDS:
```

---

## Decision Gate

After all three spikes, answer these questions:

| Question | Answer |
|----------|--------|
| Does the emergent joint model produce clinically plausible ROM? | |
| Can we build and deploy a SOFA-linked DLL into Unity? | |
| Does C++ scene construction match Python scene behavior? | |
| Is step time viable for real-time use (<20ms, or <50ms with async)? | |
| How many assumptions in docs 10/12 were wrong? | |

**If all YES:** Proceed to Sprint 0 as defined in doc 12, incorporating spike findings. Update docs 10/12 with ground truth.

**If any NO:** Revisit the specific failure. Options:
- Adjust the architecture (e.g., add explicit joint constraints if emergent model fails)
- Adjust scope (e.g., drop real-time requirement, use offline simulation)
- Adjust approach (e.g., fall back to ZMQ cross-process if DLL linking is intractable)
- Adjust timeline (e.g., expand Sprint 0 if build system needs more work)

---

## Reference Map

During spike work, consult these docs for specific details:

| Need | Doc | Section |
|------|-----|---------|
| SOFA component class names | 03 (SOFA research) | §2-5 |
| Material property values | 01 (clinical research) | §4 |
| Clinical ROM ranges | 01 (clinical research) | §2.3 |
| C API design (to validate) | 12 (unified plan) | §1.4 |
| C++ scene construction (to validate) | 12 (unified plan) | §1.8 |
| Thread safety design (to validate) | 12 (unified plan) | §1.5 |
| Ligament attachment points | 03 (SOFA research) | §3.2 |
| Unity P/Invoke patterns | 02 (Unity research) | §5 |
| Emergent joint rationale | 11 (negotiation) | Negotiation-6 |
| Bilinear ligament rationale | 11 (negotiation) | Negotiation-5 |
