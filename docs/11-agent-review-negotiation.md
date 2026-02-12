# Agent Review & Negotiation

> Three-agent review of the Custom SOFA-Unity Integration Plan (doc 10).
> Each agent identifies concerns, disagreements are negotiated, and improved solutions are reached.

---

## Round 1: Individual Reviews

### SOFA Expert Review

**Strengths:**
- Correct use of `FreeMotionAnimationLoop` + `GenericConstraintSolver` for contact
- Scene construction via C++ API is the right approach — avoids fragile XML/Python intermediary
- Topology change handling via `TetrahedronSetTopologyModifier::removeTetrahedra()` is correct
- `Tetra2TriangleTopologicalMapping` for automatic surface mesh maintenance is essential

**Concerns:**

1. **CONCERN-S1: Resection algorithm is too aggressive.** The current plan removes tetrahedra where ALL four vertices are below the plane. This leaves a ragged, staircase-like cut surface. A better approach: remove elements that are *intersected* by the plane (any vertex below), then re-mesh the cut surface. Or use SOFA's `TopologicalChangeProcessor` with element subdivision for cleaner cuts.

2. **CONCERN-S2: Missing SOFA plugin loading.** The `sofa_bridge_init()` call needs to explicitly load SOFA plugins via `PluginManager`. The current C++ code initializes the simulation graph but doesn't show plugin loading. Without `RequiredPlugin` or `PluginManager::loadPlugin()`, components like `TetrahedronFEMForceField` won't be found at runtime.

3. **CONCERN-S3: Ligament modeling is too simplistic.** The plan uses `StiffSpringForceField` for ligaments, which is linear. Real ligaments have a toe region (slack at low strain, then stiffening). We should at least use a piecewise-linear spring or consider the `RestShapeSpringsForceField` with a custom force law. This directly impacts ROM accuracy.

4. **CONCERN-S4: No mention of scene graph initialization order.** SOFA requires `sofa::simulation::init(root)` to be called after all nodes and components are added. The plan has `sofa_scene_finalize()` but the implementation detail is missing. This is a common failure point — forgetting init causes all-zero states.

5. **CONCERN-S5: Joint constraint modeling is absent.** The plan mentions `sofa_set_joint_limits()` and `sofa_apply_joint_torque()` in the C API but doesn't describe how joints are modeled in SOFA. For the ankle joint, we need `BilateralInteractionConstraint` between tibia and talus with configurable DOFs. This is a significant gap.

### Unity Expert Review

**Strengths:**
- NativeArray + Burst job pipeline for mesh sync is the right architecture
- Double-buffering decouples frame rates correctly
- P/Invoke declarations are clean and correct

**Concerns:**

6. **CONCERN-U1: No consideration for Unity's managed-to-native transition cost.** Each P/Invoke call has overhead (~100-500ns). The current API has separate calls for `sofa_get_positions`, `sofa_get_joint_angle`, `sofa_get_joint_torque`, `sofa_get_contact_forces`, etc. Per frame, that's potentially 10+ P/Invoke calls. We should batch readback into a single struct or use a "snapshot" pattern — one call returns all data for the frame.

7. **CONCERN-U2: Unity Mesh rebuild after topology change is expensive.** Calling `Mesh.Clear()` + `Mesh.SetVertices()` + `Mesh.SetTriangles()` every time topology changes will cause GC pressure from array allocations. We should use the Advanced Mesh API (`Mesh.SetVertexBufferData` with `NativeArray`) even for topology changes.

8. **CONCERN-U3: MonoBehaviour lifecycle coupling.** The plan wraps the bridge in a `SofaBridgeComponent` MonoBehaviour, but doesn't specify when in Unity's execution order SOFA steps should happen. `FixedUpdate` is the natural choice (matching physics), but the async step model means we might be out of sync. Need explicit Script Execution Order or a custom PlayerLoop injection.

9. **CONCERN-U4: No editor workflow.** The plan only covers runtime. During development, we need the ability to test SOFA scenes from the Unity Editor without entering Play Mode. An Editor window that can initialize SOFA, step, and visualize results would accelerate iteration.

10. **CONCERN-U5: Missing error propagation to C#.** The C API returns `int` error codes and sets a string via `sofa_bridge_get_error()`. But in C#, we should throw `SofaException` with the error message rather than requiring callers to check return codes. The high-level API should provide safe error handling.

### Integration Expert Review

**Strengths:**
- Thread safety model is correct and simple to reason about
- Build system approach (CMake + deploy script) is practical
- Risk assessment covers the right items

**Concerns:**

11. **CONCERN-I1: The double-buffer swap has a race condition.** The current `DoubleBuffer::swap()` uses `std::atomic<int>` for the front pointer, but the actual buffer data isn't protected by the atomic. If Unity reads buffer A while SOFA finishes writing to buffer B and swaps, the read is safe. BUT if SOFA starts the *next* step immediately, it writes to buffer A (now the write buffer) while Unity might still be reading it. We need a three-buffer scheme or a reader-count guard.

12. **CONCERN-I2: `std::thread::detach()` is dangerous.** The async step detaches the thread, which means we can't guarantee it completes before `sofa_bridge_shutdown()`. If Unity calls shutdown while a step is in progress, we get undefined behavior. Use `std::thread::join()` or a thread pool with proper lifecycle management.

13. **CONCERN-I3: Command queue is underspecified.** The plan mentions "queued commands executed between steps" but doesn't show the queue data structure or how commands are serialized across threads. We need a lock-free queue (like SPSC ring buffer) or a mutex-guarded command list with clear ownership semantics.

14. **CONCERN-I4: DLL dependency collection is harder than stated.** SOFA v25.12 has 80+ shared libraries. We need a script that walks the dependency tree (ldd on Linux, dumpbin /dependents on Windows) and copies all transitive dependencies. Missing even one DLL causes a cryptic Unity crash.

15. **CONCERN-I5: No versioning or ABI stability strategy.** If the C API changes, old Unity projects with a new DLL will crash. We need a version handshake: `sofa_bridge_init()` should accept and return a version struct. The C# side checks version compatibility at startup.

---

## Round 2: Negotiation & Resolution

### NEGOTIATION-1: Resection Quality (CONCERN-S1)

**SOFA Expert:** We need cleaner cuts. Remove-all-below gives staircasing artifacts.

**Unity Expert:** But subdividing elements along the cut plane is expensive and changes vertex count unpredictably. That complicates our mesh sync.

**SOFA Expert:** Agreed, full remeshing is too expensive at runtime. Counter-proposal: remove elements where the element *centroid* is below the plane (not all vertices). This gives a much smoother cut boundary with minimal computational cost. Then on the Unity side, we use EzySlice for the visual cut — the visual mesh is always clean. SOFA's cut is the physics approximation.

**Integration Expert:** So two separate representations of the cut: visual (EzySlice, pixel-perfect) and physical (SOFA tetrahedral removal, approximate). The user sees the EzySlice result; SOFA computes physics on the tetrahedral approximation.

**RESOLUTION:** Dual-representation resection:
1. Visual: EzySlice plane cut on Unity mesh (instant, clean)
2. Physical: SOFA centroid-based tetrahedral removal (approximate but fast)
3. Post-resection mesh sync only replaces the collision/physics mesh, not the visual mesh
4. For ROM computation accuracy, the centroid-based cut is sufficient — the exact cut surface geometry has minimal impact on joint angle results

### NEGOTIATION-2: Batched Readback (CONCERN-U1)

**Unity Expert:** We need to minimize P/Invoke calls per frame. I want a single "snapshot" call.

**SOFA Expert:** But different data lives in different SOFA nodes. Collecting it all into one struct requires traversing the scene graph.

**Integration Expert:** Proposal: add a `sofa_get_frame_snapshot()` call that populates a pre-defined struct with all per-frame data. The C++ side caches this at the end of each step.

**SOFA Expert:** Acceptable if the snapshot struct is fixed at compile time and the fields are known. For our ankle sim, the per-frame data is well-defined.

**RESOLUTION:** Add a frame snapshot API:

```c
typedef struct {
    // Bone positions (rigid bodies)
    float tibia_position[7];    // pos(3) + quat(4)
    float talus_position[7];
    float fibula_position[7];
    float calcaneus_position[7];

    // Joint angles (degrees)
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

    // Deformable mesh vertex count (for pre-allocating readback buffers)
    int   cartilage_vertex_count;
} SofaFrameSnapshot;

EXPORT int sofa_get_frame_snapshot(SofaFrameSnapshot* out);
```

This reduces per-frame P/Invoke calls from ~10 to 2 (one snapshot + one optional position readback for deformable meshes). The generic `sofa_get_positions()` call is kept for deformable mesh vertex data, which is too large for a fixed struct.

### NEGOTIATION-3: Thread Safety — Triple Buffer (CONCERN-I1)

**Integration Expert:** The double buffer has a race. We need triple buffering.

**Unity Expert:** Triple buffering adds latency — one extra frame of delay.

**Integration Expert:** Only if SOFA is faster than Unity. In practice, SOFA steps take 5-20ms and Unity renders at 30+ fps (33ms). SOFA is slower, so the triple buffer doesn't add visible latency — it just ensures we never read a buffer being written.

**SOFA Expert:** The frame snapshot struct is small (~200 bytes). Triple buffering it has zero memory cost. For the large deformable mesh buffers, we can use the double buffer with a read-lock guard instead.

**RESOLUTION:** Hybrid approach:
- **Frame snapshot:** Triple-buffered (tiny, no latency concern)
- **Deformable mesh positions:** Double-buffered with an atomic reader count. SOFA worker waits if reader count > 0 before swapping (rarely blocks since reads are fast).

```cpp
class MeshDoubleBuffer {
    std::vector<float> _buffers[2];
    std::atomic<int> _front{0};
    std::atomic<int> _readerCount{0};

public:
    // Unity side
    float* beginRead() { _readerCount++; return _buffers[_front].data(); }
    void   endRead()   { _readerCount--; }

    // SOFA side
    float* getWriteBuffer() { return _buffers[1 - _front].data(); }
    void   swap() {
        while (_readerCount.load() > 0) { /* spin-wait, rarely hit */ }
        _front.store(1 - _front.load());
    }
};
```

### NEGOTIATION-4: Thread Lifecycle (CONCERN-I2)

**Integration Expert:** `detach()` is unacceptable. We need join-on-shutdown.

**Unity Expert:** Agreed. But we also can't block `sofa_bridge_shutdown()` indefinitely if SOFA is stuck in a divergent solver loop.

**RESOLUTION:** Use a managed worker thread with a cancellation token and join timeout:

```cpp
void SofaSimulationManager::shutdown() {
    _cancelRequested.store(true);

    if (_workerThread.joinable()) {
        // Wait up to 5 seconds for current step to complete
        auto future = std::async(std::launch::async, [this]{ _workerThread.join(); });
        if (future.wait_for(std::chrono::seconds(5)) == std::future_status::timeout) {
            // Force-terminate as last resort (log warning)
            _workerThread.detach();  // only on shutdown, not during normal operation
        }
    }

    // Clean up SOFA scene
    if (_root) {
        sofa::simulation::node::unload(_root);
        _root.reset();
    }

    sofa::simulation::graph::cleanup();
}
```

The worker thread checks `_cancelRequested` between steps and breaks out of any retry loops.

### NEGOTIATION-5: Ligament Nonlinearity (CONCERN-S3)

**SOFA Expert:** Linear springs are inaccurate for ligament ROM. We need the toe region.

**Unity Expert:** Does SOFA have a built-in nonlinear spring?

**SOFA Expert:** Not directly in the core. Two options:
1. Use `RestShapeSpringsForceField` as a baseline, with a custom Python controller that modulates stiffness based on strain (bilinear approximation)
2. Use `TetrahedronHyperelasticityFEMForceField` with a volumetric ligament mesh (MooneyRivlin material)

Option 1 is simpler and sufficient for our accuracy requirements (+/-5 deg ROM). Option 2 is more physically accurate but adds significant mesh complexity for each ligament.

**Integration Expert:** Option 1 keeps our C API simpler — we just need to expose the stiffness modulation. But if the stiffness changes per step, we need to update it from C++, not Python.

**RESOLUTION:** Bilinear spring model implemented in C++:

```cpp
// In the step loop, before SOFA animate:
void SofaSimulationManager::updateLigamentStiffness() {
    for (auto& lig : _ligaments) {
        auto spring = lig.springForceField;
        float currentLength = computeSpringLength(lig);
        float strain = (currentLength - lig.restLength) / lig.restLength;

        float effectiveStiffness;
        if (strain < lig.toeRegionStrain) {
            // Toe region: low stiffness (slack ligament)
            effectiveStiffness = lig.toeStiffness;
        } else {
            // Linear region: full stiffness
            effectiveStiffness = lig.linearStiffness;
        }

        // Update SOFA spring stiffness
        auto& springs = spring->d_springsValue.beginEdit();
        springs[0].ks = effectiveStiffness;
        spring->d_springsValue.endEdit();
    }
}
```

Add to the C API:

```c
typedef struct {
    const char* name;
    const char* bone_a_name;
    const char* bone_b_name;
    int         attachment_index_a;
    int         attachment_index_b;
    float       toe_stiffness;       // N/mm (low, for toe region)
    float       linear_stiffness;    // N/mm (full stiffness)
    float       toe_region_strain;   // strain threshold (e.g., 0.03 = 3%)
    float       damping;
    float       rest_length;
} SofaLigamentConfig;  // updated from original
```

### NEGOTIATION-6: Joint Modeling (CONCERN-S5)

**SOFA Expert:** The plan completely omits how the ankle joint itself is modeled in SOFA. This is the core of the simulation. For the ankle, we need a spherical joint between tibia and talus with configurable ROM limits.

**Unity Expert:** In Unity, we'd use `ArticulationBody` with `SphericalJoint`. What's the SOFA equivalent?

**SOFA Expert:** SOFA doesn't have a built-in "joint" component like Unity/PhysX. Instead, joints are modeled via constraints:
1. `BilateralInteractionConstraint` — locks specific DOFs between two rigid bodies
2. For ROM limits, we apply torques via `ConstantForceField` or modify positions through a custom controller
3. The `ArticulatedSystemPlugin` provides joint hierarchies but is less commonly used

For our ankle simulation, the most practical approach: model tibia as fixed rigid body, talus as free rigid body, connected by a set of `StiffSpringForceField` springs that enforce joint constraint. The ligaments themselves act as the ROM limiters. Apply external moment to the talus via `ConstantForceField` and measure resulting angular displacement.

**Integration Expert:** So the "joint" is emergent from the ligament constraints and bone contact, not a first-class joint component?

**SOFA Expert:** Exactly. That's more biomechanically realistic anyway. The ankle joint isn't a simple hinge — its instantaneous axis of rotation shifts during the ROM arc. Modeling it as ligament-constrained rigid bodies with contact captures this naturally.

**Unity Expert:** Then the angle measurement comes from computing the relative orientation between tibia and talus rigid bodies at each step, not from reading a "joint angle" from SOFA.

**RESOLUTION:** Emergent joint model:
1. Tibia = fixed rigid body
2. Talus = free rigid body constrained by ligaments (springs) and bone-bone contact
3. ROM measurement = relative quaternion decomposition between tibia and talus orientations
4. External moment applied to talus via `ConstantForceField` to drive ROM sweep
5. The C API `sofa_get_joint_angle()` computes this from the two rigid body positions internally

Update the C++ implementation:

```cpp
float SofaSimulationManager::getJointAngle(const std::string& jointName, int axis) {
    // "ankle" joint = relative orientation of talus w.r.t. tibia
    auto tibiaNode = findNode("Tibia");
    auto talusNode = findNode("Talus");

    auto tibiaMO = tibiaNode->get<MechanicalObject<Rigid3Types>>();
    auto talusMO = talusNode->get<MechanicalObject<Rigid3Types>>();

    auto tibiaOri = tibiaMO->x.getValue()[0].getOrientation();
    auto talusOri = talusMO->x.getValue()[0].getOrientation();

    // Relative orientation
    auto relOri = tibiaOri.inverse() * talusOri;

    // Decompose into Euler angles (anatomical convention)
    // axis: 0 = dorsi/plantarflexion, 1 = inversion/eversion, 2 = int/ext rotation
    auto euler = relOri.toEulerVector();  // radians
    return euler[axis] * 180.0 / M_PI;   // to degrees
}
```

### NEGOTIATION-7: Version Handshake (CONCERN-I5)

**Integration Expert:** We need ABI versioning.

**Unity Expert:** Simple solution: `sofa_bridge_init` returns a version struct. C# checks compatibility.

**SOFA Expert:** Also embed the SOFA version so we can detect mismatches with the linked SOFA libraries.

**RESOLUTION:**

```c
typedef struct {
    int bridge_version_major;   // bump on breaking API changes
    int bridge_version_minor;   // bump on additions
    int bridge_version_patch;   // bump on bug fixes
    int sofa_version_major;     // from SOFA's version header
    int sofa_version_minor;
} SofaBridgeVersion;

EXPORT SofaBridgeVersion sofa_bridge_get_version();

// C# side checks at startup:
var version = SofaNativeBridge.sofa_bridge_get_version();
if (version.bridge_version_major != EXPECTED_MAJOR)
    throw new SofaVersionMismatchException(...);
```

### NEGOTIATION-8: SOFA Plugin Loading (CONCERN-S2)

**SOFA Expert:** This is critical. Without explicit plugin loading, nothing works.

**RESOLUTION:** Add to `sofa_bridge_init`:

```cpp
bool SofaSimulationManager::init(const std::string& pluginDir) {
    sofa::simulation::graph::init();

    auto& pm = sofa::helper::system::PluginManager::getInstance();

    // Required plugins for our simulation
    std::vector<std::string> plugins = {
        "Sofa.Component.StateContainer",
        "Sofa.Component.SolidMechanics.FEM.Elastic",
        "Sofa.Component.SolidMechanics.FEM.HyperElastic",
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
        "Sofa.Component.Mapping.NonLinear",
        "Sofa.Component.Topology.Container.Dynamic",
        "Sofa.Component.Topology.Mapping",
        "Sofa.Component.Visual",
        "Sofa.Component.IO.Mesh",
    };

    // Optional plugins
    std::vector<std::string> optionalPlugins = {
        "SofaCarving",
        "CGALPlugin",
    };

    for (const auto& p : plugins) {
        if (!pm.loadPlugin(p)) {
            _lastError = "Failed to load required plugin: " + p;
            return false;
        }
    }

    for (const auto& p : optionalPlugins) {
        pm.loadPlugin(p);  // non-fatal if missing
    }

    return true;
}
```

### NEGOTIATION-9: Script Execution Order (CONCERN-U3)

**Unity Expert:** We need to guarantee that SOFA stepping happens at the right point in Unity's frame.

**Integration Expert:** Proposed execution order:

```
Unity Frame:
  1. FixedUpdate (physics):
     - SofaBridgeComponent checks if async step completed
     - If yes: read frame snapshot, start mesh sync job, kick off next async step
     - If no: skip (SOFA still computing, Unity uses last frame's data)

  2. Update (game logic):
     - ROMEngine reads angles from snapshot
     - WorkflowManager processes state transitions
     - UI updates

  3. LateUpdate (rendering prep):
     - MeshSync job completes, apply to Mesh
     - Render
```

**RESOLUTION:** Use `[DefaultExecutionOrder(-100)]` on `SofaBridgeComponent` to ensure it runs before all other scripts in `FixedUpdate`. The mesh sync job is scheduled in `FixedUpdate` and completed in `LateUpdate` via `JobHandle.Complete()`.

---

## Round 3: Final Improvements

### Improvement 1: Error Handling Strategy

All three agents agree on a layered error model:

| Level | Mechanism | Example |
|-------|-----------|---------|
| C++ internal | `try/catch` around all SOFA calls, store in `_lastError` | Solver divergence, topology crash |
| C API | Return codes (0 = success, negative = error) | `-1` = node not found, `-2` = not initialized |
| C# bridge | Check return code, throw `SofaBridgeException` if non-zero | `if (result != 0) throw new SofaBridgeException(GetLastError())` |
| C# high-level | Domain-specific exceptions | `SolverDivergedException`, `TopologyException` |

### Improvement 2: Scene Rebuild Capability

**SOFA Expert:** We need the ability to tear down and rebuild the SOFA scene without restarting the entire bridge. This supports the "undo resection" workflow — rather than undoing topology changes (complex and error-prone in SOFA), we rebuild the scene from scratch with the original meshes.

**RESOLUTION:** Add `sofa_scene_destroy()` + `sofa_scene_create()` cycle. The C# side caches the original scene configuration and can rebuild at any time. This is simpler and more robust than trying to undo topology changes in SOFA.

### Improvement 3: Profiling Hooks

**Unity Expert:** For performance tuning, we need timing data from inside the C++ plugin.

**RESOLUTION:** Add a profiling API:

```c
typedef struct {
    float scene_step_ms;         // total SOFA step time
    float collision_ms;          // collision detection + response
    float fem_solve_ms;          // FEM solver time
    float constraint_solve_ms;   // constraint solver time
    float mesh_readback_ms;      // position copy to buffer
    int   collision_contacts;    // number of active contacts
    int   constraint_iterations; // iterations until convergence
} SofaProfilingData;

EXPORT int sofa_get_profiling_data(SofaProfilingData* out);
```

This data feeds into Unity's Profiler as custom markers.

---

## Resolution Summary

| Concern | Resolution | Impact |
|---------|-----------|--------|
| S1: Ragged cuts | Centroid-based removal + dual visual/physics representation | Better visual quality, simpler code |
| S2: Plugin loading | Explicit plugin list in `sofa_bridge_init()` | Prevents silent failures |
| S3: Linear ligaments | Bilinear spring model in C++ with toe region | +/- 2-3 deg ROM improvement |
| S4: Init order | `sofa_scene_finalize()` calls `sofa::simulation::init()` | Prevents all-zero bug |
| S5: Joint modeling | Emergent joint from ligaments + contact, angle from relative orientation | More biomechanically realistic |
| U1: P/Invoke overhead | Frame snapshot struct (single call for all per-frame data) | ~10x fewer P/Invoke calls |
| U2: Mesh rebuild GC | Use Advanced Mesh API with NativeArrays for topology changes | Zero GC pressure |
| U3: Execution order | `[DefaultExecutionOrder(-100)]` on bridge component | Deterministic frame timing |
| U4: Editor workflow | Deferred to post-MVP (nice to have, not critical path) | Reduced scope risk |
| U5: Error propagation | Layered exception model (C++ → C → C# bridge → C# domain) | Clean error handling |
| I1: Buffer race | Triple-buffer for snapshot, reader-count guard for mesh buffers | Thread-safe, minimal latency |
| I2: Thread detach | Join with timeout on shutdown, cancellation token | Clean shutdown |
| I3: Command queue | Mutex-guarded `std::vector<Command>` with variant type | Simple, correct |
| I4: DLL dependencies | Automated dependency walker script in build pipeline | Prevents deploy failures |
| I5: Version handshake | Version struct returned from init, checked in C# | ABI stability |
