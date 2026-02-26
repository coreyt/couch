# Documentation Manifest — COUCH Project

Generated: 2026-02-15
Codebase version: main branch (post-Sprint 4, Windows DLL deployment complete)

## Manifest Summary
- Total topics: 28
- P0 (crash/corruption risk): 11
- P1 (wrong behavior risk): 11
- P2 (confusion/productivity risk): 6

---

## P0 Topics — Must Document or Developers Will Ship Bugs

### Topic 1: SofaSurfaceMesh In/Out Buffer Overflow
**Category:** 1 (Crash Trap)
**Risk:** Silent memory corruption if C# allocates buffer too small — C++ writes past buffer without error, corrupting heap. Unity may crash minutes later in unrelated code.

**Code evidence:**
- `sofa_ankle_bridge.h:182-183` — `vertex_count` and `triangle_count` are documented as "in/out: capacity in, actual out"
- `NativeStructs.cs:258-270` — `SofaSurfaceMesh.Create()` allocates buffers with `Marshal.AllocHGlobal(maxVertices * 3 * sizeof(float))`
- `scene_builder.cpp:866-890` — C++ fills arrays up to capacity: `std::min(actual_verts, vert_cap)`, but if actual > capacity, data is silently truncated. If C# wrongly reads using `actual` instead of the returned count, it reads uninitialized memory.

**What the doc must explain:**
1. The in/out contract: C# sets `vertexCount`/`triangleCount` to buffer capacity BEFORE calling `sofa_get_surface_mesh()`. C++ overwrites these fields with actual counts.
2. C# MUST check the returned counts and re-allocate if truncated (actual > capacity).
3. Example code showing correct pattern: allocate generously (10k verts), call, check if `nativeMesh.vertexCount` == original capacity (indicating truncation), re-allocate and retry.

**Suggested doc location:** API Reference > P/Invoke Patterns > In/Out Buffers
**Anti-pattern to warn against:** Allocating exact size from prior mesh, then calling after resection (mesh may have grown). Always allocate 2-3x expected size or implement retry loop.

---

### Topic 2: IntPtr Memory Leak — Missing FreeHGlobal
**Category:** 1 (Crash Trap)
**Risk:** Gradual memory leak (hundreds of MB over a session) if `FreeNativePtrs()` is not called in a `finally` block. Unity Editor becomes unresponsive.

**Code evidence:**
- `NativeStructs.cs:63,71-72` — `SofaLigamentConfig.Create()` allocates 3 IntPtrs with `Marshal.StringToHGlobalAnsi()`
- `NativeStructs.cs:81-101` — `FreeNamePtrs()` frees these, but callers must invoke it
- `SceneBuilderIntegrationTests.cs:130-151` — Correct pattern: `try { sofa_add_ligament(ref lig); } finally { SofaLigamentConfig.FreeNamePtrs(ligs); }`
- `NativeStructs.cs:187-191` — `SofaDeformableConfig.Create()` allocates 4 IntPtrs; same leak risk

**What the doc must explain:**
1. All `*.Create()` methods that return structs with IntPtr fields allocate unmanaged memory.
2. MUST be freed via `*.FreeNativePtrs()` in a `finally` block — even if the native call fails.
3. Table of all structs with this pattern: `SofaLigamentConfig`, `SofaRigidBoneConfig`, `SofaDeformableConfig`, `SofaResectionCommand`, `SofaSurfaceMesh`.

**Suggested doc location:** Integration Guide > Memory Management > Unmanaged Memory Lifecycle
**Anti-pattern to warn against:** Allocating in a loop without `finally` (e.g., adding 100 ligaments in a for-loop). Each iteration leaks ~100 bytes; 100 iterations = 10KB leak per scene rebuild.

---

### Topic 3: Thread Safety of Global State (g_* variables)
**Category:** 1 (Crash Trap)
**Risk:** Data race / crash if two threads call native functions simultaneously. SOFA scene graph is NOT thread-safe; simultaneous access corrupts `g_root`.

**Code evidence:**
- `sofa_ankle_bridge.cpp:19-26` — 7 global variables: `g_initialized`, `g_root`, `g_last_error`, `g_scene`, `g_thread_manager`, `g_builder`, `g_snapshot_buffer`, `g_use_builder`
- `sofa_ankle_bridge.cpp:142` — `sofa_step()` calls `g_thread_manager.wait()` to block if async step is running
- `sofa_ankle_bridge.cpp:435,551` — `sofa_execute_resection()` and `sofa_apply_torque()` also call `wait()` to prevent data races with async step
- No mutex protecting reads of `g_last_error` from multiple threads

**What the doc must explain:**
1. The bridge is single-threaded by design. C# must serialize all calls via a single thread (e.g., Unity's main thread).
2. `sofa_step_async()` launches a background thread, but `sofa_execute_resection()` / `sofa_apply_torque()` internally call `wait()` to prevent corruption.
3. DO NOT call native functions from multiple C# threads (e.g., Unity Job System, async/await). Use a command queue pattern.

**Suggested doc location:** Architecture Guide > Threading Model
**Anti-pattern to warn against:** Calling `sofa_execute_resection()` from a Unity Coroutine while `FixedUpdate()` is calling `sofa_step_async()` on the main thread — this appears to work but corrupts SOFA state silently.

---

### Topic 4: const char* Return Ownership (sofa_bridge_get_error)
**Category:** 1 (Crash Trap)
**Risk:** Use-after-free if C# caches the IntPtr returned by `sofa_bridge_get_error()`. The string is owned by C++ global `g_last_error` and becomes invalid on next error.

**Code evidence:**
- `sofa_ankle_bridge.h:129` — `const char* sofa_bridge_get_error(void);` returns C-owned pointer
- `sofa_ankle_bridge.cpp:215-217` — Returns `g_last_error.c_str()`, which is invalidated when `g_last_error` is reassigned
- `SofaNativeBridge.cs:98-99` — `GetErrorString()` correctly calls `Marshal.PtrToStringAnsi(ptr)` immediately to copy, not cache

**What the doc must explain:**
1. The returned `const char*` is valid ONLY until the next bridge call. C# MUST copy it to a managed string immediately via `Marshal.PtrToStringAnsi()`.
2. DO NOT cache the IntPtr for later use. DO NOT free it (C++ owns the memory).
3. Error strings are cleared on successful calls (`g_last_error.clear()` in every success path).

**Suggested doc location:** API Reference > Error Handling
**Anti-pattern to warn against:** Storing `IntPtr errPtr = sofa_bridge_get_error()` in a field, then calling `Marshal.PtrToStringAnsi(errPtr)` later — the string may have been overwritten or cleared.

---

### Topic 5: Async Step Cancellation Race in Shutdown
**Category:** 1 (Crash Trap)
**Risk:** Crash during Unity Editor stop if async step is running when `OnDisable()` fires. C++ destructor joins thread with 5s timeout; if SOFA is mid-step, Unity freezes.

**Code evidence:**
- `thread_manager.cpp:41-64` — `shutdown()` sets cancel flag, polls for 5s, then force-joins regardless of timeout
- `sofa_ankle_bridge.cpp:186-212` — `sofa_bridge_shutdown()` calls `g_thread_manager.shutdown(5000)`
- `SofaBridgeComponent.cs:65-73` — `OnDisable()` calls `_sim.Dispose()` which calls `sofa_bridge_shutdown()` — no cancellation check first

**What the doc must explain:**
1. Always call `sofa_step_async_wait()` in `OnDisable()` BEFORE calling `Dispose()` to ensure step completes cleanly.
2. If Unity is force-stopping (Application.Quit), the 5s timeout may still block. This is by design — detaching would corrupt SOFA state.
3. Consider reducing timestep or step count if 5s timeout is hit frequently.

**Suggested doc location:** Integration Guide > Component Lifecycle > Shutdown Sequence
**Anti-pattern to warn against:** Calling `Dispose()` immediately in `OnDisable()` without checking `IsStepComplete()` first — causes 5s hang every time user stops play mode.

---

### Topic 6: StructLayout Sequential Mismatch Between C/C#
**Category:** 1 (Crash Trap)
**Risk:** Struct field misalignment causes garbage data or access violations if C# struct layout doesn't match C.

**Code evidence:**
- `NativeStructs.cs:7` — All structs correctly marked `[StructLayout(LayoutKind.Sequential)]`
- `NativeStructs.cs:29-30` — Fixed-size arrays with `[MarshalAs(UnmanagedType.ByValArray, SizeConst = 3)]` match C `double tibia_offset[3]`
- `sofa_ankle_bridge.h:32-35` — `SofaRigidFrame` has doubles (8 bytes each); C# `NativeStructs.cs:18-22` mirrors exactly

**What the doc must explain:**
1. Every C struct must have a corresponding C# struct with `[StructLayout(LayoutKind.Sequential)]`.
2. Array fields MUST use `[MarshalAs(UnmanagedType.ByValArray, SizeConst = N)]` to embed in struct (not pointer).
3. IntPtr fields in C# correspond to `const char*` or `const float*` in C (pointer types).
4. Test new structs by passing non-zero values and verifying C++ receives them correctly.

**Suggested doc location:** API Reference > P/Invoke Patterns > Struct Marshaling
**Anti-pattern to warn against:** Forgetting `[StructLayout(LayoutKind.Sequential)]` — C# may reorder fields for packing, causing silent corruption. Forgetting `SizeConst` on arrays — C# passes a pointer instead of embedding.

---

### Topic 7: Version Handshake — Only Major Version Checked
**Category:** 1 (Crash Trap - prevention mechanism exists but not enforced)
**Risk:** If C# code advances faster than native DLL is updated, API mismatch causes crashes. Version check exists but only checks major version.

**Code evidence:**
- `sofa_ankle_bridge.cpp:38-45` — Bridge returns version 0.2.0, SOFA 24.6
- `SofaSimulation.cs:19-28` — Checks `bridgeVersionMajor == 0`, throws exception if mismatch

**What the doc must explain:**
1. Major version MUST match (breaking changes). Minor version changes (new fields, new functions) are backward compatible.
2. If C# references a function that doesn't exist in the DLL, Unity throws `EntryPointNotFoundException` at runtime (not compile time).
3. Always deploy matching bridge DLL version with C# code.

**Suggested doc location:** Integration Guide > Versioning & Compatibility
**Anti-pattern to warn against:** Copying an old DLL build from a previous commit into Unity without rebuilding. Version check passes (both 0.x), but missing functions crash on first call.

---

### Topic 8: Solver Divergence Detection Delayed by Triple Buffer
**Category:** 1 (Crash Trap)
**Risk:** If solver diverges (NaN in positions), the snapshot's `solver_diverged` flag is set, but C# may read it 1-2 frames late due to triple-buffering. Simulation continues with NaN.

**Code evidence:**
- `sofa_ankle_bridge.cpp:32-36` — `check_diverged()` detects NaN in snapshot positions
- `sofa_ankle_bridge.cpp:157,674` — Sets `wb.solver_diverged = check_diverged(wb) ? 1 : 0` before `publish()`
- `triple_buffer.h:56-68` — `read()` swaps reader/published indices, so reader lags by 1 publish behind writer
- `SofaBridgeComponent.cs:52` — Reads snapshot, but doesn't check `solverDiverged` flag

**What the doc must explain:**
1. Check `snapshot.solverDiverged == 1` after every `GetSnapshot()` call. If true, STOP stepping and alert user.
2. The flag may lag by 1 frame due to triple-buffering.
3. Divergence is unrecoverable — must destroy scene and rebuild with lower stiffness or smaller timestep.

**Suggested doc location:** Integration Guide > Error Handling > Solver Divergence
**Anti-pattern to warn against:** Ignoring the `solverDiverged` flag and continuing to call `StepAsync()` — causes all positions to become NaN indefinitely.

---

### Topic 9: Scene State Machine Violations (Create -> Finalize -> Step)
**Category:** 2 (Temporal Coupling)
**Risk:** Calling functions out of order produces cryptic errors or silent wrong behavior (e.g., stepping an empty scene).

**Code evidence:**
- `scene_builder.h:63-67` — `SceneBuilderState` enum: Empty, Created, Finalized
- `scene_builder.cpp:28-31,89-96,222-228,276-280` — Guards check state and return errors
- `sofa_ankle_bridge.cpp:147-173` — `sofa_step()` doesn't check if scene is finalized — just calls `animate()` on empty scene (no error)

**What the doc must explain:**
1. Required sequence: `sofa_scene_create()` -> (add bones/ligaments/deformables)* -> `sofa_scene_finalize()` -> `sofa_step()`
2. Calling `sofa_step()` before finalize: no error, but scene is empty
3. Calling `sofa_add_rigid_bone()` after finalize: returns error
4. State diagram showing valid transitions

**Suggested doc location:** Integration Guide > Scene Construction > State Machine
**Anti-pattern to warn against:** Forgetting `sofa_scene_finalize()` — stepping "works" but produces zero ROM.

---

### Topic 10: Resection Must Wait for Async Step to Complete
**Category:** 2 (Temporal Coupling + Data Race)
**Risk:** Data race on topology modifier if resection runs during async step.

**Code evidence:**
- `sofa_ankle_bridge.cpp:420-451` — `sofa_execute_resection()` calls `g_thread_manager.wait()` before executing cut
- `scene_builder.cpp:824` — `modifier->removeTetrahedra(to_remove, true);` directly modifies SOFA topology (not thread-safe)

**What the doc must explain:**
1. Resection internally waits for async step to complete — automatic but causes blocking delay.
2. DO NOT attempt to parallelize resection with stepping.
3. Same constraint applies to `sofa_apply_torque()`.

**Suggested doc location:** Integration Guide > Resection Engine > Thread Safety
**Anti-pattern to warn against:** Assuming `sofa_step_async_is_complete()` check in C# is sufficient.

---

### Topic 11: DLL Load Order (SOFA Plugins Must Be Loaded Before Scene Creation)
**Category:** 2 (Temporal Coupling)
**Risk:** If `sofa_bridge_init(pluginDir)` is called with incorrect path or plugins fail to load, later calls crash.

**Code evidence:**
- `sofa_ankle_bridge.cpp:48-129` — `sofa_bridge_init()` loads 18 SOFA plugins via `PluginManager::loadPluginByPath()`
- Lines 94-106: Accumulates load errors and returns failure
- Line 58-60: NULL pluginDir falls back to minimal plugin set

**What the doc must explain:**
1. `sofa_bridge_init(pluginDir)` MUST be called before any scene creation.
2. `pluginDir` should point to `Assets/Plugins/x86_64`.
3. If init returns non-zero, call `sofa_bridge_get_error()` for details.

**Suggested doc location:** Integration Guide > Initialization > Plugin Loading
**Anti-pattern to warn against:** Passing `null` for `pluginDir` in production.

---

## P1 Topics — Should Document or Developers Will Waste Time

### Topic 12: Torque Unit Conversion (N-m -> N-mm)
**Category:** 3 (Semantic Trap)
**Risk:** Developer sees `* 1000.0` in C++ and "fixes" it by passing N-mm directly, producing 1000x too much torque.

**Code evidence:**
- `sofa_ankle_bridge.h:149` — "Torque magnitude in N-m (converted to N-mm internally)"
- `sofa_ankle_bridge.cpp:581`, `scene_builder.cpp:524` — `torque[axis] = static_cast<double>(torque_nm) * 1000.0;`

**Suggested doc location:** API Reference > Unit Conventions
**Anti-pattern to warn against:** Passing pre-converted N-mm values.

---

### Topic 13: Ligament Stiffness Stability Envelope (<3x Default)
**Category:** 4 (Physics Trap)
**Risk:** Increasing stiffness above ~3x default causes solver divergence. No runtime warning — just NaN.

**Code evidence:**
- `CLAUDE.md:67` — "high stiffness (>3x) causes divergence"
- `ankle_scene.cpp:23-40` — Defaults: ATFL 70, PTFL 50, Deltoid 90 N/mm

**Suggested doc location:** SOFA Patterns > Ligament Tuning
**Anti-pattern to warn against:** Setting ATFL stiffness to 280 N/mm without changing damping/timestep.

---

### Topic 14: Gravity Unit (mm/s^2 not m/s^2)
**Category:** 3 (Semantic Trap)
**Risk:** Passing -9.81 instead of -9810 produces 1000x too little gravity.

**Code evidence:**
- `sofa_ankle_bridge.h:135,141` — "Gravity along Z axis in mm/s^2 (e.g. -9810)"
- `SofaBridgeComponent.cs:15` — Default `gravityZ = -9810f`

**Suggested doc location:** API Reference > Unit Conventions
**Anti-pattern to warn against:** Copying Unity's `Physics.gravity` (-9.81f) directly.

---

### Topic 15: One-Step Lag in ConstantForceField
**Category:** 4 (Physics Trap)
**Risk:** Forces applied at step N take effect at step N+1. Confuses debugging.

**Code evidence:**
- `CLAUDE.md:67` — "forces computed at step N apply at step N+1"
- `scene_builder.cpp:372-497` — `applyLigamentForces()` called BEFORE `animate()`

**Suggested doc location:** SOFA Patterns > Force Application Timing
**Anti-pattern to warn against:** Expecting immediate effect from applied torque.

---

### Topic 16: Ligaments Are Tension-Only
**Category:** 4 (Physics Trap)
**Risk:** Ligaments don't resist compression. Bones penetrate if collision disabled.

**Code evidence:**
- `scene_builder.cpp:426-429` — `if (extension <= 0.0) continue;`

**Suggested doc location:** SOFA Patterns > Ligament Mechanics
**Anti-pattern to warn against:** Disabling collision expecting ligaments to keep bones apart.

---

### Topic 17: Joint Angles Depend on Bone Order
**Category:** 3 (Semantic Trap)
**Risk:** Adding bones in wrong order produces inverted joint angles. No error.

**Code evidence:**
- `scene_builder.cpp:540-599` — `fillSnapshot()` reads bones[0] as tibia, bones[1] as talus
- `SceneBuilderIntegrationTests.cs:278-286` — Tibia first, Talus second

**Suggested doc location:** Integration Guide > ROM Measurement > Bone Order
**Anti-pattern to warn against:** Adding bones alphabetically (Talus, Tibia).

---

### Topic 18: Legacy vs New Scene API
**Category:** 5 (Architecture Decision)
**Risk:** Using legacy API for new features that require deformable tissue.

**Code evidence:**
- `sofa_ankle_bridge.h:131-146` — Legacy: `sofa_scene_create_ankle()`
- `sofa_ankle_bridge.h:64-111` — New: `sofa_scene_create()` + builder
- `sofa_ankle_bridge.cpp:26` — `g_use_builder` flag

**Suggested doc location:** SOFA Patterns > API Selection
**Anti-pattern to warn against:** Starting with legacy API then trying `sofa_add_deformable_tissue()`.

---

### Topic 19: Centroid-Based Resection (Not Exact Splitting)
**Category:** 5 (Architecture Decision)
**Risk:** Developer expects smooth cut surface.

**Code evidence:**
- `docs/adr/0001-centroid-based-resection.md` — Design rationale
- `scene_builder.cpp:800-814` — Centroid classification

**Suggested doc location:** SOFA Patterns > Resection Algorithm
**Anti-pattern to warn against:** Attempting to smooth SOFA surface post-cut.

---

### Topic 20: Undo via Scene Rebuild
**Category:** 5 (Architecture Decision)
**Risk:** Developer assumes resection is reversible via SOFA.

**Code evidence:**
- `docs/adr/0003-undo-via-scene-rebuild.md` — No SOFA topology undo
- `ResectionIntegrationTests.cs:182-200` — `UndoCut()` destroys scene

**Suggested doc location:** SOFA Patterns > Undo Strategy
**Anti-pattern to warn against:** Calling `UndoCut()` without rebuilding scene.

---

### Topic 21: StiffSpringForceField Doesn't Work Across RigidMapping
**Category:** 5 (Architecture Decision)
**Risk:** Developer replaces custom controller with SOFA's built-in spring. Forces become zero.

**Code evidence:**
- `CLAUDE.md:63` — "StiffSpringForceField fails across RigidMapping"
- `scene_builder.cpp:372-497` — Custom controller + ConstantForceField

**Suggested doc location:** SOFA Patterns > Custom Ligament Controller
**Anti-pattern to warn against:** Replacing controller with StiffSpringForceField.

---

### Topic 22: DefaultExecutionOrder(-100) Critical
**Category:** 2 (Temporal Coupling)
**Risk:** Null reference if other components access SofaSimulation before initialization.

**Code evidence:**
- `SofaBridgeComponent.cs:5` — `[DefaultExecutionOrder(-100)]`
- `SofaBridgeComponent.cs:23-40` — `OnEnable()` creates `_sim`

**Suggested doc location:** Integration Guide > Component Lifecycle
**Anti-pattern to warn against:** Accessing Simulation in `Awake()`.

---

## P2 Topics — Nice to Document for Developer Productivity

### Topic 23: Assembly Definitions and Test Organization
**Category:** 6 (Navigation Confusion)
**Risk:** Test in wrong assembly doesn't run.

**Suggested doc location:** Testing Guide > Test Organization

---

### Topic 24: SOFA Plugin Directory Path (Windows vs Linux)
**Category:** 6 (Navigation Confusion)
**Risk:** Hardcoded paths break cross-platform.

**Suggested doc location:** Onboarding Guide > Cross-Platform

---

### Topic 25: Triple Buffer Lag (Snapshot 1 Frame Old)
**Category:** 3 (Semantic Trap - low consequence)
**Risk:** Confusing when debugging exact step data.

**Suggested doc location:** SOFA Patterns > Async Stepping

---

### Topic 26: SOFA Coordinate System (Z-Up)
**Category:** 6 (Navigation Confusion)
**Risk:** Bones appear sideways in Unity.

**Suggested doc location:** SOFA Patterns > Coordinate Systems

---

### Topic 27: Bilinear Ligament Model (Toe Region)
**Category:** 4 (Physics Trap - optional feature)
**Risk:** Inverted stiffness values cause divergence.

**Suggested doc location:** SOFA Patterns > Advanced Ligament Model

---

### Topic 28: Build System RPATH for Dependencies
**Category:** 6 (Navigation Confusion)
**Risk:** DLL not found after copying only bridge DLL.

**Suggested doc location:** Onboarding Guide > DLL Deployment

---

## Cross-Cutting Concerns

### Unit Conventions
Spans: API Reference, SOFA Patterns, Onboarding Guide
Topics: 12 (Torque), 14 (Gravity)
Table needed: torque (N-m), force (N), distance (mm), time (s), mass (kg), gravity (mm/s^2), stiffness (N/mm), damping (N-s/mm).

### Memory Management Patterns
Spans: Native Boundary Guide, API Reference
Topics: 1 (SofaSurfaceMesh), 2 (IntPtr leak), 4 (const char*)
Patterns: in/out buffers, C-owned strings, struct lifecycle (Create -> use -> FreeNativePtrs in finally).

### Threading Model
Spans: Native Boundary Guide, SOFA Patterns, API Reference
Topics: 3 (global state), 5 (shutdown race), 10 (resection wait), 25 (triple buffer)
Diagram needed: Unity main thread -> native calls -> async step thread -> triple buffer -> main thread reads.

### Scene Construction State Machine
Spans: SOFA Patterns, API Reference, Onboarding Guide
Topics: 9, 18, 20
State diagram: Empty -> Created (add components) -> Finalized (step/query) -> Destroyed (undo).

### Physics Parameter Tuning
Spans: SOFA Patterns, API Reference
Topics: 13, 15, 16, 27
Guide: default values, safe ranges, stability constraints, clinical validation ranges.

### Error Handling Strategy
Spans: Native Boundary Guide, SOFA Patterns, API Reference
Topics: 4, 8, 22
Checklist: check return codes, read error string immediately, check solverDiverged, try/catch for DllNotFoundException.
