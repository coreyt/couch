# C API Reference

**Status:** Living document (updated with Sprint 4 resection features)
**Audience:** C++ developers and C# developers writing P/Invoke bindings
**Related documents:** [Native Boundary Guide](native-boundary.md), [SOFA Patterns](sofa-patterns.md)

---

## Overview

This document is the **complete reference** for all 26 exported C functions and 8 data structures in the `SofaAnkleBridge` native library. This is a lookup document, not a tutorial. For usage patterns and integration guidance, see the [Native Boundary Guide](native-boundary.md).

**Header file:** `spike/spike2_native/include/sofa_ankle_bridge.h`
**Implementation:** `spike/spike2_native/src/sofa_ankle_bridge.cpp`

All functions use a flat C API (`extern "C"`) with consistent error handling: return 0 on success, non-zero on error. Retrieve the error message via `sofa_bridge_get_error()`.

---

## Unit Conventions

All functions and structs use the following unit conventions. **Violating these units produces incorrect simulation behavior.**

| Quantity | Unit | Example Value |
|----------|------|---------------|
| **Torque** | N·m (converted to N·mm internally) | `0.005` (5 N·m = 5000 N·mm) |
| **Force** | N (Newtons) | `100.0` |
| **Distance** | mm (millimeters) | `50.0` |
| **Time** | s (seconds) | `0.01` |
| **Mass** | kg (kilograms) | `0.1` |
| **Gravity** | mm/s² | `-9810.0` (NOT -9.81) |
| **Stiffness** | N/mm | `70.0` |
| **Damping** | N·s/mm | `5.0` |
| **Young's Modulus** | Pa (Pascals) | `17e9` (cortical bone) |
| **Density** | kg/mm³ | `1.85e-6` |

**Common mistakes:**
- Passing `-9.81` for gravity instead of `-9810` produces 1000× too little gravity.
- Passing pre-converted N·mm torque to `sofa_apply_torque()` produces 1000× too much torque.
- Mixing SI units (meters) with SOFA units (millimeters) causes bones to appear incorrectly scaled.

---

## Data Structures

### `SofaBridgeVersion`

**Purpose:** Version information returned by `sofa_bridge_get_version()`.

**Fields:**

| Name | Type | Description |
|------|------|-------------|
| `bridge_version_major` | `int` | Bridge API major version (0 = pre-release, breaking changes allowed) |
| `bridge_version_minor` | `int` | Bridge API minor version (incremented for new features) |
| `bridge_version_patch` | `int` | Bridge API patch version (incremented for bug fixes) |
| `sofa_version_major` | `int` | SOFA Framework major version (24 = v24.06.00) |
| `sofa_version_minor` | `int` | SOFA Framework minor version (6 = v24.06.00) |

**Ownership:** Value type — no pointers, no cleanup required.

**Usage notes:**
- C# code MUST check `bridge_version_major` matches expected version on initialization. Mismatched major versions cause crashes due to ABI incompatibility.
- Minor version differences are backward compatible (new functions can be added without breaking existing calls).

---

### `SofaRigidFrame`

**Purpose:** Position and orientation of a rigid body in 3D space.

**Fields:**

| Name | Type | Description |
|------|------|-------------|
| `px`, `py`, `pz` | `double` | Position in mm |
| `qx`, `qy`, `qz`, `qw` | `double` | Orientation quaternion (SOFA convention: `[x, y, z, w]`) |

**Ownership:** Value type — embedded in `SofaFrameSnapshot`, no cleanup required.

**Usage notes:**
- Quaternion order differs from Unity (`[w, x, y, z]`). C# code must reorder when converting.
- SOFA uses Z-up coordinate system. Unity uses Y-up. See [SOFA Patterns § Coordinate Systems](sofa-patterns.md#coordinate-systems).

---

### `SofaLigamentConfig`

**Purpose:** Configuration for a ligament connecting two bones or a bone to a fixed point.

**Fields:**

| Name | Type | Ownership | Description |
|------|------|-----------|-------------|
| `name` | `const char*` | Borrowed | Ligament identifier (e.g., "ATFL"). C# must marshal via `StringToHGlobalAnsi`. |
| `tibia_offset[3]` | `double` | Value | Attachment offset in bone A's local frame (mm). |
| `talus_offset[3]` | `double` | Value | Attachment offset in bone B's local frame (mm). |
| `fixed_anchor[3]` | `double` | Value | World-space fixed attachment point (mm). Only used if `use_fixed_anchor == 1`. |
| `use_fixed_anchor` | `int` | Value | If 1, bone A is ignored and `fixed_anchor` is used as proximal attachment. |
| `stiffness` | `double` | Value | Linear region stiffness in N/mm. Default: 70 (ATFL). |
| `damping` | `double` | Value | Velocity-based damping in N·s/mm. Default: 5. |
| `rest_length` | `double` | Value | Ligament rest length in mm. If 0, auto-computed from initial bone positions. |
| `bone_a_name` | `const char*` | Borrowed | Name of proximal bone. NULL defaults to "Tibia" (backward compat). |
| `bone_b_name` | `const char*` | Borrowed | Name of distal bone. NULL defaults to "Talus" (backward compat). |
| `toe_stiffness` | `double` | Value | Toe region stiffness in N/mm. If 0, uses `stiffness` (pure linear model). |
| `toe_region_strain` | `double` | Value | Strain threshold for toe-to-linear transition (fraction of rest length). If 0, pure linear model. |

**Ownership notes:**
- C# MUST allocate `name`, `bone_a_name`, `bone_b_name` via `Marshal.StringToHGlobalAnsi()` and free via `Marshal.FreeHGlobal()` in a `finally` block.
- C++ reads strings immediately; does not retain pointers.

**See also:** `sofa_add_ligament()`, [SOFA Patterns § Ligament Mechanics](sofa-patterns.md#ligament-mechanics)

---

### `SofaFrameSnapshot`

**Purpose:** Per-frame state of the simulation (bone poses, joint angles, forces, solver status).

**Fields:**

| Name | Type | Description |
|------|------|-------------|
| `tibia` | `SofaRigidFrame` | First bone's rigid transform (Z-up, mm). |
| `talus` | `SofaRigidFrame` | Second bone's rigid transform (Z-up, mm). |
| `joint_angles_deg[3]` | `double` | Euler angles in degrees: `[sagittal, frontal, transverse]`. Positive sagittal = dorsiflexion. |
| `ligament_force[3]` | `double` | Total ligament force on second bone in N: `[x, y, z]`. |
| `ligament_torque[3]` | `double` | Total ligament torque on second bone in N·mm: `[x, y, z]`. |
| `step_time_ms` | `float` | Wall-clock time for last `sofa_step()` or `sofa_step_async()` call in milliseconds. |
| `solver_diverged` | `int` | 1 if NaN detected in positions (solver divergence), 0 otherwise. |
| `step_count` | `int` | Total number of simulation steps executed. |

**Ownership:** Value type — no pointers, no cleanup required.

**Critical usage notes:**
- **Always check `solver_diverged` after `sofa_get_frame_snapshot()`.** If 1, stop stepping immediately — divergence is unrecoverable. See [SOFA Patterns § Solver Divergence](sofa-patterns.md#solver-divergence).
- Due to triple-buffering in async mode, `solver_diverged` may lag by 1 frame. Check on every read.
- `joint_angles_deg` depends on bone order. First bone added = tibia, second = talus. See [Native Boundary Guide § Bone Order](native-boundary.md#bone-order).

---

### `SofaSceneConfig`

**Purpose:** Global physics parameters for scene creation.

**Fields:**

| Name | Type | Description |
|------|------|-------------|
| `gravity[3]` | `float` | Gravity vector in mm/s². Example: `[0, 0, -9810]` (Z-down). |
| `timestep` | `float` | Simulation timestep in seconds. Default: `0.01` (10 ms). |
| `constraint_iterations` | `int` | GenericConstraintSolver max iterations. Default: `100`. |
| `constraint_tolerance` | `float` | Convergence tolerance for constraint solver. Default: `1e-5`. |
| `rayleigh_stiffness` | `float` | Rayleigh damping proportional to stiffness matrix. Default: `0.1`. |
| `rayleigh_mass` | `float` | Rayleigh damping proportional to mass matrix. Default: `1.0`. |
| `alarm_distance` | `float` | Distance (mm) at which collision detection activates. Default: `2.0`. |
| `contact_distance` | `float` | Distance (mm) at which collision constraints activate. Default: `0.5`. |
| `friction_coefficient` | `float` | Coulomb friction coefficient. Default: `0.3`. |

**Ownership:** Value type — no pointers, no cleanup required.

**See also:** `sofa_scene_create()`, [SOFA Patterns § Solver Parameters](sofa-patterns.md#solver-parameters)

---

### `SofaRigidBoneConfig`

**Purpose:** Configuration for a rigid bone with collision mesh.

**Fields:**

| Name | Type | Ownership | Description |
|------|------|-----------|-------------|
| `name` | `const char*` | Borrowed | Bone identifier (e.g., "Tibia"). C# must marshal via `StringToHGlobalAnsi`. |
| `collision_vertices` | `const float*` | Borrowed | Flattened vertex array `[x, y, z, ...]` in mm. |
| `collision_vertex_count` | `int` | Value | Number of vertices (length of array / 3). |
| `collision_triangles` | `const int*` | Borrowed | Flattened triangle index array `[i0, i1, i2, ...]`. |
| `collision_triangle_count` | `int` | Value | Number of triangles (length of array / 3). |
| `position[3]` | `float` | Value | Initial position in mm: `[x, y, z]`. |
| `orientation[4]` | `float` | Value | Initial quaternion: `[x, y, z, w]`. |
| `mass` | `float` | Value | Total bone mass in kg. Default: `0.1`. |
| `is_fixed` | `int` | Value | If 1, bone is constrained to initial pose (e.g., tibia). |

**Ownership notes:**
- C# MUST allocate `name` via `Marshal.StringToHGlobalAnsi()` and free via `Marshal.FreeHGlobal()`.
- C# MUST allocate `collision_vertices` and `collision_triangles` via `Marshal.AllocHGlobal()` and free in `finally`.
- C++ reads arrays immediately; does not retain pointers.

**See also:** `sofa_add_rigid_bone()`

---

### `SofaDeformableConfig`

**Purpose:** Configuration for deformable tissue with tetrahedral FEM.

**Fields:**

| Name | Type | Ownership | Description |
|------|------|-----------|-------------|
| `name` | `const char*` | Borrowed | Tissue identifier (e.g., "TibiaDeformable"). |
| `parent_bone` | `const char*` | Borrowed | Rigid bone to attach to (or NULL for unattached). |
| `vertices` | `const float*` | Borrowed | Flattened vertex array `[x, y, z, ...]` in mm. |
| `vertex_count` | `int` | Value | Number of vertices. |
| `tetrahedra` | `const int*` | Borrowed | Flattened tetrahedron index array `[i0, i1, i2, i3, ...]`. |
| `tetra_count` | `int` | Value | Number of tetrahedra. |
| `young_modulus` | `float` | Value | Material stiffness in Pa. Cortical bone: `~17e9`. |
| `poisson_ratio` | `float` | Value | Material compressibility. Bone: `~0.3`. |
| `mass_density` | `float` | Value | Material density in kg/mm³. Bone: `~1.85e-6`. |

**Ownership notes:**
- C# MUST allocate all string/array pointers via `Marshal.AllocHGlobal()` and free in `finally`.
- C++ copies mesh data into SOFA topology structures; does not retain pointers.

**See also:** `sofa_add_deformable_tissue()`, [ADR-0001 Centroid-Based Resection](../adr/0001-centroid-based-resection.md)

---

### `SofaResectionCommand`

**Purpose:** Plane definition for tetrahedral resection (bone cutting).

**Fields:**

| Name | Type | Ownership | Description |
|------|------|-----------|-------------|
| `plane_point[3]` | `float` | Value | Point on the cutting plane in mm: `[x, y, z]`. |
| `plane_normal[3]` | `float` | Value | Plane normal vector (must be unit length). Points "above" the cut. |
| `bone_name` | `const char*` | Borrowed | Name of deformable tissue to cut (e.g., "TibiaDeformable"). |

**Ownership notes:**
- C# MUST allocate `bone_name` via `Marshal.StringToHGlobalAnsi()` and free in `finally`.
- Plane normal MUST be normalized (length = 1). Resection removes tetrahedra whose centroid satisfies `dot(centroid - plane_point, plane_normal) < 0`.

**See also:** `sofa_execute_resection()`, [ADR-0001 Centroid-Based Resection](../adr/0001-centroid-based-resection.md)

---

### `SofaSurfaceMesh`

**Purpose:** In/out buffer for extracting surface triangulation from deformable tissue.

**Fields:**

| Name | Type | Ownership | Description |
|------|------|-----------|-------------|
| `vertices` | `float*` | Caller-allocated | Flattened vertex array `[x, y, z, ...]`. C# allocates buffer before call. |
| `triangles` | `int*` | Caller-allocated | Flattened triangle array `[i0, i1, i2, ...]`. C# allocates buffer before call. |
| `vertex_count` | `int` | **In/out** | **Input:** Buffer capacity (max vertices). **Output:** Actual vertex count written. |
| `triangle_count` | `int` | **In/out** | **Input:** Buffer capacity (max triangles). **Output:** Actual triangle count written. |

**Critical ownership notes:**
- **This is an in/out buffer pattern.** C# MUST allocate `vertices` and `triangles` with capacity N, then set `vertex_count` / `triangle_count` to N BEFORE calling `sofa_get_surface_mesh()`.
- C++ writes up to capacity, then overwrites `vertex_count` / `triangle_count` with actual counts.
- **If actual > capacity, data is silently truncated.** C# MUST check returned counts and re-allocate if truncated.
- C# MUST free buffers via `Marshal.FreeHGlobal()` after use.

**See also:** `sofa_get_surface_mesh()`, [Native Boundary Guide § In/Out Buffers](native-boundary.md#in-out-buffers)

---

## Function Groups

### Version and Lifecycle

- [`sofa_bridge_get_version()`](#sofa_bridge_get_version) — Query version
- [`sofa_bridge_init()`](#sofa_bridge_init) — Initialize SOFA + load plugins
- [`sofa_bridge_shutdown()`](#sofa_bridge_shutdown) — Shutdown and cleanup
- [`sofa_bridge_get_error()`](#sofa_bridge_get_error) — Retrieve last error message

### Scene Construction (New API)

- [`sofa_scene_create()`](#sofa_scene_create) — Create empty scene with physics
- [`sofa_scene_destroy()`](#sofa_scene_destroy) — Destroy current scene
- [`sofa_scene_is_ready()`](#sofa_scene_is_ready) — Check if finalized
- [`sofa_scene_finalize()`](#sofa_scene_finalize) — Finalize scene for stepping
- [`sofa_add_rigid_bone()`](#sofa_add_rigid_bone) — Add rigid bone
- [`sofa_add_ligament()`](#sofa_add_ligament) — Add ligament between bones

### Deformable Tissue and Resection (Sprint 4)

- [`sofa_add_deformable_tissue()`](#sofa_add_deformable_tissue) — Add FEM tissue
- [`sofa_execute_resection()`](#sofa_execute_resection) — Cut tissue with plane
- [`sofa_get_removed_element_count()`](#sofa_get_removed_element_count) — Query last resection count
- [`sofa_has_topology_changed()`](#sofa_has_topology_changed) — Check if mesh changed
- [`sofa_get_surface_mesh()`](#sofa_get_surface_mesh) — Extract surface triangulation

### Legacy Ankle Scene API

- [`sofa_scene_create_ankle()`](#sofa_scene_create_ankle) — Create ankle with defaults
- [`sofa_scene_create_ankle_ex()`](#sofa_scene_create_ankle_ex) — Create ankle with custom ligaments
- [`sofa_apply_torque()`](#sofa_apply_torque) — Apply external torque
- [`sofa_get_frame_snapshot()`](#sofa_get_frame_snapshot) — Read simulation state

### Stepping

- [`sofa_step()`](#sofa_step) — Synchronous step
- [`sofa_step_async()`](#sofa_step_async) — Start async step
- [`sofa_step_async_is_complete()`](#sofa_step_async_is_complete) — Poll completion
- [`sofa_step_async_wait()`](#sofa_step_async_wait) — Block until complete

---

## Function Reference

### `sofa_bridge_get_version`

**Signature:**
```c
SOFA_BRIDGE_API SofaBridgeVersion sofa_bridge_get_version(void);
```

**Purpose:** Returns version information for the native bridge and SOFA Framework.

**Preconditions:** None — can be called before `sofa_bridge_init()`.

**Parameters:** None.

**Returns:** `SofaBridgeVersion` struct by value.

**Side effects:** None.

**Thread safety:** Thread-safe (reads compile-time constants).

**Example:**
```c
SofaBridgeVersion v = sofa_bridge_get_version();
if (v.bridge_version_major != 0) {
    fprintf(stderr, "Version mismatch: expected 0.x, got %d.%d\n",
            v.bridge_version_major, v.bridge_version_minor);
    return 1;
}
printf("Bridge: %d.%d.%d, SOFA: %d.%d\n",
       v.bridge_version_major, v.bridge_version_minor, v.bridge_version_patch,
       v.sofa_version_major, v.sofa_version_minor);
```

**See also:** [Native Boundary Guide § Version Handshake](native-boundary.md#version-handshake)

---

### `sofa_bridge_init`

**Signature:**
```c
SOFA_BRIDGE_API int sofa_bridge_init(const char* plugin_dir);
```

**Purpose:** Initializes SOFA Framework, loads required plugins, and creates an empty root node.

**Preconditions:**
- Must NOT have been called already (returns error if `g_initialized == true`).
- Call `sofa_bridge_shutdown()` first if re-initializing.

**Parameters:**

| Name | Type | Ownership | Description |
|------|------|-----------|-------------|
| `plugin_dir` | `const char*` | Borrowed | Path to directory containing SOFA plugin DLLs (e.g., `Assets/Plugins/x86_64`). If NULL, loads minimal plugin set only. |

**Returns:** 0 on success, non-zero on error.

**Side effects:**
- Initializes global SOFA simulation graph (`sofa::simulation::graph::init()`).
- Loads 18 SOFA plugins from `plugin_dir` (Collision, FEM, Solvers, Topology, etc.). See implementation for full list.
- Creates empty root node stored in `g_root`.
- Sets `g_initialized = true`.

**Thread safety:** Must be called from main thread only. NOT thread-safe if called concurrently.

**Example:**
```c
const char* plugin_dir = "/path/to/Assets/Plugins/x86_64";
int rc = sofa_bridge_init(plugin_dir);
if (rc != 0) {
    fprintf(stderr, "Init failed: %s\n", sofa_bridge_get_error());
    return 1;
}
```

**See also:** `sofa_bridge_shutdown()`, [Native Boundary Guide § Initialization](native-boundary.md#initialization)

---

### `sofa_bridge_shutdown`

**Signature:**
```c
SOFA_BRIDGE_API void sofa_bridge_shutdown(void);
```

**Purpose:** Shuts down SOFA Framework, unloads the scene graph, and releases all global resources. Idempotent.

**Preconditions:** None — safe to call even if not initialized.

**Parameters:** None.

**Returns:** None (void).

**Side effects:**
- Joins any in-progress async step with 5-second timeout (blocks until complete or timeout).
- Unloads current scene (`sofa::simulation::node::unload(g_root)`).
- Cleans up SOFA simulation graph (`sofa::simulation::graph::cleanup()`).
- Sets `g_initialized = false`.
- Resets `g_scene`, `g_builder`, `g_thread_manager`, `g_snapshot_buffer`.

**Thread safety:** Must be called from main thread only. Automatically waits for async step thread to join.

**Example:**
```c
sofa_bridge_shutdown(); // Always succeeds (idempotent)
```

**Critical usage notes:**
- **Always call `sofa_step_async_wait()` in Unity `OnDisable()` BEFORE calling `Dispose()` (which calls this function).** Otherwise Unity hangs for 5 seconds if async step is running.
- After shutdown, must call `sofa_bridge_init()` again before using any other functions.

**See also:** `sofa_bridge_init()`, `sofa_step_async_wait()`, [Native Boundary Guide § Shutdown Sequence](native-boundary.md#shutdown-sequence)

---

### `sofa_bridge_get_error`

**Signature:**
```c
SOFA_BRIDGE_API const char* sofa_bridge_get_error(void);
```

**Purpose:** Returns the last error message from a failed function call, or empty string if no error.

**Preconditions:** None.

**Parameters:** None.

**Returns:** Pointer to C-owned null-terminated string. Valid ONLY until next bridge function call.

**Side effects:** None.

**Thread safety:** NOT thread-safe — `g_last_error` is a global string.

**Example:**
```c
int rc = sofa_scene_create(&config);
if (rc != 0) {
    const char* err = sofa_bridge_get_error();
    fprintf(stderr, "Scene creation failed: %s\n", err);
    // DO NOT cache the pointer — it becomes invalid on next call
}
```

**Critical ownership notes:**
- **C++ owns the returned pointer.** DO NOT free it.
- **Copy the string immediately** via `strcpy()` or `Marshal.PtrToStringAnsi()` in C#. DO NOT cache the pointer for later use.
- Error strings are cleared on successful calls (`g_last_error.clear()`).

**See also:** [Native Boundary Guide § Error Handling](native-boundary.md#error-handling)

---

### `sofa_scene_create`

**Signature:**
```c
SOFA_BRIDGE_API int sofa_scene_create(const SofaSceneConfig* config);
```

**Purpose:** Creates a new empty scene with physics pipeline (collision, constraints, animation loop).

**Preconditions:**
- `sofa_bridge_init()` must have been called.
- No scene currently active (call `sofa_scene_destroy()` first if needed).
- `g_builder.state() == SceneBuilderState::Empty`.

**Parameters:**

| Name | Type | Ownership | Description |
|------|------|-----------|-------------|
| `config` | `const SofaSceneConfig*` | Borrowed | Physics parameters (gravity, timestep, solver settings). Must not be NULL. |

**Returns:** 0 on success, non-zero on error.

**Side effects:**
- Unloads existing root node and creates a fresh one.
- Adds SOFA components: `FreeMotionAnimationLoop`, `GenericConstraintSolver`, `CollisionPipeline`, `BruteForceBroadPhase`, `BVHNarrowPhase`, `LocalMinDistance`, `CollisionResponse`.
- Sets `g_builder` state to `Created`.
- Sets `g_use_builder = true` (activates new API mode).

**Thread safety:** Must be called from main thread only.

**Example:**
```c
SofaSceneConfig cfg = {
    .gravity = {0, 0, -9810},
    .timestep = 0.01f,
    .constraint_iterations = 100,
    .constraint_tolerance = 1e-5f,
    .rayleigh_stiffness = 0.1f,
    .rayleigh_mass = 1.0f,
    .alarm_distance = 2.0f,
    .contact_distance = 0.5f,
    .friction_coefficient = 0.3f
};
int rc = sofa_scene_create(&cfg);
if (rc != 0) {
    fprintf(stderr, "Failed: %s\n", sofa_bridge_get_error());
}
```

**See also:** `sofa_scene_destroy()`, `sofa_scene_finalize()`, [SOFA Patterns § Scene Construction](sofa-patterns.md#scene-construction)

---

### `sofa_scene_destroy`

**Signature:**
```c
SOFA_BRIDGE_API void sofa_scene_destroy(void);
```

**Purpose:** Destroys the current scene, releasing all SOFA nodes and resetting state.

**Preconditions:** `sofa_bridge_init()` must have been called.

**Parameters:** None.

**Returns:** None (void).

**Side effects:**
- Waits for any in-progress async step to complete.
- Unloads current root node (`sofa::simulation::node::unload(g_root)`).
- Creates a fresh empty root node.
- Resets `g_builder` to `Empty` state.
- Sets `g_use_builder = false`.
- Resets `g_snapshot_buffer`.

**Thread safety:** Must be called from main thread only. Automatically waits for async step thread.

**Example:**
```c
sofa_scene_destroy(); // Always succeeds (idempotent)
// Can now create a new scene via sofa_scene_create()
```

**See also:** `sofa_scene_create()`, [ADR-0003 Undo via Scene Rebuild](../adr/0003-undo-via-scene-rebuild.md)

---

### `sofa_scene_is_ready`

**Signature:**
```c
SOFA_BRIDGE_API int sofa_scene_is_ready(void);
```

**Purpose:** Checks if the scene is finalized and ready for stepping.

**Preconditions:** None.

**Parameters:** None.

**Returns:** 1 if ready, 0 if not.

**Side effects:** None.

**Thread safety:** Thread-safe (reads state enum).

**Example:**
```c
if (sofa_scene_is_ready()) {
    sofa_step(0.01f);
}
```

**See also:** `sofa_scene_finalize()`

---

### `sofa_scene_finalize`

**Signature:**
```c
SOFA_BRIDGE_API int sofa_scene_finalize(void);
```

**Purpose:** Finalizes the scene by initializing the root node and resolving typed SOFA component pointers.

**Preconditions:**
- `sofa_bridge_init()` must have been called.
- Scene must be in `Created` state (call `sofa_scene_create()` first, then add bones/ligaments/deformables).
- `g_use_builder == true` (new API active).

**Parameters:** None.

**Returns:** 0 on success, non-zero on error.

**Side effects:**
- Calls `sofa::simulation::node::initRoot(g_root)` — initializes SOFA's scene graph.
- Resolves typed pointers for all bones (`MechanicalObject`, `ConstantForceField`).
- Resolves typed pointers for all deformable tissues (`TetrahedronSetTopologyContainer`, `TetrahedronSetTopologyModifier`, `MechanicalObject`).
- Auto-computes rest lengths for ligaments with `rest_length == 0`.
- Sets `g_builder` state to `Finalized`.

**Thread safety:** Must be called from main thread only.

**Example:**
```c
// After adding bones and ligaments
int rc = sofa_scene_finalize();
if (rc != 0) {
    fprintf(stderr, "Finalize failed: %s\n", sofa_bridge_get_error());
    return 1;
}
// Now ready to call sofa_step()
```

**See also:** `sofa_scene_create()`, `sofa_add_rigid_bone()`, [SOFA Patterns § Scene State Machine](sofa-patterns.md#scene-state-machine)

---

### `sofa_add_rigid_bone`

**Signature:**
```c
SOFA_BRIDGE_API int sofa_add_rigid_bone(const SofaRigidBoneConfig* config);
```

**Purpose:** Adds a rigid bone with collision mesh to the scene. Must be called between `sofa_scene_create()` and `sofa_scene_finalize()`.

**Preconditions:**
- `sofa_bridge_init()` must have been called.
- Scene must be in `Created` state (not `Empty` or `Finalized`).
- `g_use_builder == true`.

**Parameters:**

| Name | Type | Ownership | Description |
|------|------|-----------|-------------|
| `config` | `const SofaRigidBoneConfig*` | Borrowed | Bone configuration. Must not be NULL. |

**Returns:** 0 on success, non-zero on error.

**Side effects:**
- Creates a SOFA node under the root with name `config->name`.
- Adds components: `EulerImplicitSolver`, `CGLinearSolver`, `MechanicalObject` (Rigid3d), `UniformMass`.
- If `is_fixed == 1`, adds `FixedConstraint`.
- If `is_fixed == 0`, adds two `ConstantForceField` components (LigamentFF, TorqueFF) for force application.
- If collision mesh provided, creates `Collision` sub-node with `MeshTopology`, `MechanicalObject` (Vec3d), `TriangleCollisionModel`, `LineCollisionModel`, `PointCollisionModel`, `RigidMapping`.
- Stores bone state in `g_builder._bones`.

**Thread safety:** Must be called from main thread only.

**Example:**
```c
float verts[] = {0, 0, 0, 10, 0, 0, 5, 10, 0}; // Triangle
int tris[] = {0, 1, 2};
SofaRigidBoneConfig bone = {
    .name = "Tibia",
    .collision_vertices = verts,
    .collision_vertex_count = 3,
    .collision_triangles = tris,
    .collision_triangle_count = 1,
    .position = {0, 0, 0},
    .orientation = {0, 0, 0, 1}, // identity quaternion
    .mass = 0.1f,
    .is_fixed = 1
};
int rc = sofa_add_rigid_bone(&bone);
```

**See also:** `sofa_scene_create()`, `sofa_scene_finalize()`

---

### `sofa_add_ligament`

**Signature:**
```c
SOFA_BRIDGE_API int sofa_add_ligament(const SofaLigamentConfig* config);
```

**Purpose:** Adds a ligament connecting two bones or a bone to a fixed point. Must be called between `sofa_scene_create()` and `sofa_scene_finalize()`.

**Preconditions:**
- `sofa_bridge_init()` must have been called.
- Scene must be in `Created` state.
- `g_use_builder == true`.
- Referenced bones (`bone_a_name`, `bone_b_name`) must have been added via `sofa_add_rigid_bone()`.

**Parameters:**

| Name | Type | Ownership | Description |
|------|------|-----------|-------------|
| `config` | `const SofaLigamentConfig*` | Borrowed | Ligament configuration. Must not be NULL. |

**Returns:** 0 on success, non-zero on error.

**Side effects:**
- Stores ligament configuration in `g_builder._ligaments`.
- Ligament forces are computed and applied during `applyLigamentForces()` (called before each step).
- If `rest_length == 0`, rest length is auto-computed at finalize time from initial bone positions.

**Thread safety:** Must be called from main thread only.

**Example:**
```c
SofaLigamentConfig lig = {
    .name = "ATFL",
    .tibia_offset = {5, 0, -10},
    .talus_offset = {-5, 0, 10},
    .fixed_anchor = {0, 0, 0}, // Unused
    .use_fixed_anchor = 0,
    .stiffness = 70.0,
    .damping = 5.0,
    .rest_length = 0.0, // Auto-compute
    .bone_a_name = "Tibia",
    .bone_b_name = "Talus",
    .toe_stiffness = 0.0, // Pure linear
    .toe_region_strain = 0.0
};
int rc = sofa_add_ligament(&lig);
```

**See also:** `sofa_add_rigid_bone()`, [SOFA Patterns § Ligament Mechanics](sofa-patterns.md#ligament-mechanics)

---

### `sofa_add_deformable_tissue`

**Signature:**
```c
SOFA_BRIDGE_API int sofa_add_deformable_tissue(const SofaDeformableConfig* config);
```

**Purpose:** Adds deformable tissue with tetrahedral FEM to the scene. Must be called between `sofa_scene_create()` and `sofa_scene_finalize()`.

**Preconditions:**
- `sofa_bridge_init()` must have been called.
- Scene must be in `Created` state.
- `g_use_builder == true`.

**Parameters:**

| Name | Type | Ownership | Description |
|------|------|-----------|-------------|
| `config` | `const SofaDeformableConfig*` | Borrowed | Deformable tissue configuration. Must not be NULL. |

**Returns:** 0 on success, non-zero on error.

**Side effects:**
- Creates a SOFA node under the root with name `config->name`.
- Adds tetrahedral topology: `TetrahedronSetTopologyContainer`, `TetrahedronSetTopologyModifier`, `TetrahedronSetGeometryAlgorithms`.
- Adds FEM components: `MechanicalObject` (Vec3d), `TetrahedronFEMForceField` (with Young's modulus, Poisson ratio), `UniformMass` (with density).
- Creates `Surface` sub-node with `Tetra2TriangleTopologicalMapping` for extracting surface mesh.
- Stores deformable state in `g_builder._deformables`.

**Thread safety:** Must be called from main thread only.

**Example:**
```c
float verts[] = {0,0,0, 10,0,0, 0,10,0, 0,0,10}; // Single tet
int tets[] = {0, 1, 2, 3};
SofaDeformableConfig tissue = {
    .name = "TibiaDeformable",
    .parent_bone = NULL,
    .vertices = verts,
    .vertex_count = 4,
    .tetrahedra = tets,
    .tetra_count = 1,
    .young_modulus = 17e9f,
    .poisson_ratio = 0.3f,
    .mass_density = 1.85e-6f
};
int rc = sofa_add_deformable_tissue(&tissue);
```

**See also:** `sofa_execute_resection()`, `sofa_get_surface_mesh()`, [ADR-0001 Centroid-Based Resection](../adr/0001-centroid-based-resection.md)

---

### `sofa_execute_resection`

**Signature:**
```c
SOFA_BRIDGE_API int sofa_execute_resection(const SofaResectionCommand* cmd);
```

**Purpose:** Removes tetrahedra from a deformable tissue whose centroids fall below a cutting plane.

**Preconditions:**
- `sofa_bridge_init()` must have been called.
- Scene must be in `Finalized` state.
- `g_use_builder == true`.
- Deformable tissue with name `cmd->bone_name` must exist.

**Parameters:**

| Name | Type | Ownership | Description |
|------|------|-----------|-------------|
| `cmd` | `const SofaResectionCommand*` | Borrowed | Cutting plane specification. Must not be NULL. |

**Returns:** 0 on success, non-zero on error.

**Side effects:**
- **Automatically waits for any in-progress async step** (calls `g_thread_manager.wait()`) to prevent data race on topology modifier.
- For each tetrahedron, computes centroid as average of 4 vertices.
- Removes tetrahedra where `dot(centroid - plane_point, plane_normal) < 0`.
- Calls `TetrahedronSetTopologyModifier::removeTetrahedra()`.
- Sets `topology_dirty = true` flag on the deformable.
- Updates `last_removed_count` (queryable via `sofa_get_removed_element_count()`).
- Surface mesh is automatically updated via `Tetra2TriangleTopologicalMapping`.

**Thread safety:** Must be called from main thread only. Internally synchronized with async stepping.

**Example:**
```c
SofaResectionCommand cut = {
    .plane_point = {0, 0, 20}, // Cut at z=20mm
    .plane_normal = {0, 0, 1}, // Normal points up
    .bone_name = "TibiaDeformable"
};
int rc = sofa_execute_resection(&cut);
if (rc == 0) {
    int removed = sofa_get_removed_element_count();
    printf("Removed %d tetrahedra\n", removed);
}
```

**See also:** `sofa_add_deformable_tissue()`, `sofa_get_removed_element_count()`, `sofa_has_topology_changed()`, [ADR-0001 Centroid-Based Resection](../adr/0001-centroid-based-resection.md)

---

### `sofa_get_removed_element_count`

**Signature:**
```c
SOFA_BRIDGE_API int sofa_get_removed_element_count(void);
```

**Purpose:** Returns the number of tetrahedra removed by the last `sofa_execute_resection()` call.

**Preconditions:** None.

**Parameters:** None.

**Returns:** Count of removed tetrahedra (0 if no resection performed or no elements removed).

**Side effects:** None.

**Thread safety:** Thread-safe (reads integer from state struct).

**Example:**
```c
sofa_execute_resection(&cmd);
int count = sofa_get_removed_element_count();
if (count == 0) {
    printf("No elements removed — plane may not intersect mesh\n");
}
```

**See also:** `sofa_execute_resection()`

---

### `sofa_has_topology_changed`

**Signature:**
```c
SOFA_BRIDGE_API int sofa_has_topology_changed(void);
```

**Purpose:** Checks if any deformable tissue's topology has changed since last query.

**Preconditions:** None.

**Parameters:** None.

**Returns:** 1 if topology changed (resection performed), 0 otherwise.

**Side effects:** None — does NOT reset the dirty flag. Flag is cleared on `sofa_scene_destroy()`.

**Thread safety:** Thread-safe (reads boolean flag).

**Example:**
```c
if (sofa_has_topology_changed()) {
    // Re-extract surface mesh
    sofa_get_surface_mesh(&mesh);
}
```

**See also:** `sofa_execute_resection()`, `sofa_get_surface_mesh()`

---

### `sofa_get_surface_mesh`

**Signature:**
```c
SOFA_BRIDGE_API int sofa_get_surface_mesh(SofaSurfaceMesh* out);
```

**Purpose:** Extracts the current surface triangulation from the first deformable tissue in the scene.

**Preconditions:**
- `sofa_bridge_init()` must have been called.
- At least one deformable tissue must have been added via `sofa_add_deformable_tissue()`.
- Scene must be in `Finalized` state.
- `out->vertices` and `out->triangles` MUST be pre-allocated by caller.
- `out->vertex_count` and `out->triangle_count` MUST be set to buffer capacities BEFORE calling.

**Parameters:**

| Name | Type | Ownership | Description |
|------|------|-----------|-------------|
| `out` | `SofaSurfaceMesh*` | Caller-allocated (in/out) | Buffer to fill. Must not be NULL. |

**Returns:** 0 on success, non-zero on error.

**Side effects:**
- Reads current vertex positions from deformable's `MechanicalObject`.
- Reads surface triangles from `Tetra2TriangleTopologicalMapping` container.
- Fills `out->vertices` and `out->triangles` up to capacity.
- Overwrites `out->vertex_count` and `out->triangle_count` with actual counts.
- If actual count > capacity, data is **silently truncated** (no error returned).

**Thread safety:** Must be called from main thread only.

**Example:**
```c
// Allocate buffers (generous capacity)
float* verts = (float*)malloc(10000 * 3 * sizeof(float));
int* tris = (int*)malloc(10000 * 3 * sizeof(int));

SofaSurfaceMesh mesh = {
    .vertices = verts,
    .triangles = tris,
    .vertex_count = 10000, // Capacity
    .triangle_count = 10000
};

int rc = sofa_get_surface_mesh(&mesh);
if (rc == 0) {
    printf("Actual: %d verts, %d tris\n", mesh.vertex_count, mesh.triangle_count);
    if (mesh.vertex_count == 10000) {
        printf("WARNING: Truncated — re-allocate with larger buffer\n");
    }
}

free(verts);
free(tris);
```

**Critical usage notes:**
- **Always check returned counts against original capacity.** If equal, assume truncation and re-allocate.
- **Allocate 2-3× expected size** to avoid retry loop.
- Surface mesh is automatically updated after resection via `Tetra2TriangleTopologicalMapping`.

**See also:** `sofa_add_deformable_tissue()`, `sofa_execute_resection()`, [Native Boundary Guide § In/Out Buffers](native-boundary.md#in-out-buffers)

---

### `sofa_scene_create_ankle`

**Signature:**
```c
SOFA_BRIDGE_API int sofa_scene_create_ankle(float dt, float gravity_z);
```

**Purpose:** Creates a complete ankle scene with tibia, talus, and default 4 ligaments (ATFL, PTFL, Deltoid anterior/posterior).

**Preconditions:**
- `sofa_bridge_init()` must have been called.
- No scene currently active.

**Parameters:**

| Name | Type | Ownership | Description |
|------|------|-----------|-------------|
| `dt` | `float` | Value | Timestep in seconds (e.g., `0.01`). |
| `gravity_z` | `float` | Value | Gravity along Z axis in mm/s² (e.g., `-9810`). |

**Returns:** 0 on success, non-zero on error.

**Side effects:**
- Creates a complete SOFA scene with default ankle configuration.
- Sets `g_scene.is_active = true`.
- Sets `g_use_builder = false` (legacy API mode).
- Initializes root node.

**Thread safety:** Must be called from main thread only.

**Example:**
```c
int rc = sofa_scene_create_ankle(0.01f, -9810.0f);
if (rc != 0) {
    fprintf(stderr, "Failed: %s\n", sofa_bridge_get_error());
}
```

**See also:** `sofa_scene_create_ankle_ex()`, [SOFA Patterns § Legacy vs New API](sofa-patterns.md#legacy-vs-new-api)

---

### `sofa_scene_create_ankle_ex`

**Signature:**
```c
SOFA_BRIDGE_API int sofa_scene_create_ankle_ex(float dt, float gravity_z,
    const SofaLigamentConfig* ligaments, int num_ligaments);
```

**Purpose:** Creates an ankle scene with custom ligament configurations.

**Preconditions:**
- `sofa_bridge_init()` must have been called.
- No scene currently active.
- `ligaments` must not be NULL.
- `num_ligaments` must be > 0.

**Parameters:**

| Name | Type | Ownership | Description |
|------|------|-----------|-------------|
| `dt` | `float` | Value | Timestep in seconds. |
| `gravity_z` | `float` | Value | Gravity in mm/s². |
| `ligaments` | `const SofaLigamentConfig*` | Borrowed | Array of ligament configs. |
| `num_ligaments` | `int` | Value | Number of ligaments in array. |

**Returns:** 0 on success, non-zero on error.

**Side effects:**
- Creates complete ankle scene with specified ligaments.
- Sets `g_scene.is_active = true`.
- Sets `g_use_builder = false` (legacy API mode).

**Thread safety:** Must be called from main thread only.

**Example:**
```c
SofaLigamentConfig ligs[2] = {
    {.name = "ATFL", .stiffness = 70, .damping = 5, ...},
    {.name = "PTFL", .stiffness = 50, .damping = 5, ...}
};
int rc = sofa_scene_create_ankle_ex(0.01f, -9810.0f, ligs, 2);
```

**See also:** `sofa_scene_create_ankle()`

---

### `sofa_apply_torque`

**Signature:**
```c
SOFA_BRIDGE_API int sofa_apply_torque(float torque_nm, int axis);
```

**Purpose:** Applies an external torque about a specified axis to the free bone (talus in legacy ankle scene, or first non-fixed bone in new API).

**Preconditions:**
- `sofa_bridge_init()` must have been called.
- A scene must be active (either legacy ankle or new API finalized).
- If new API, scene must be in `Finalized` state.

**Parameters:**

| Name | Type | Ownership | Description |
|------|------|-----------|-------------|
| `torque_nm` | `float` | Value | Torque magnitude in N·m (converted to N·mm internally via `× 1000`). |
| `axis` | `int` | Value | Rotation axis: `0` = sagittal (X), `1` = frontal (Y), `2` = transverse (Z). |

**Returns:** 0 on success, non-zero on error.

**Side effects:**
- **Automatically waits for any in-progress async step** (calls `g_thread_manager.wait()`) to prevent data race on force fields.
- Writes torque to the bone's `TorqueFF` ConstantForceField component.
- Torque is applied as a 6-DOF wrench: `[0, 0, 0, tx, ty, tz]`.

**Thread safety:** Must be called from main thread only. Internally synchronized with async stepping.

**Example:**
```c
// Apply 5 N·m dorsiflexion torque (sagittal axis)
int rc = sofa_apply_torque(0.005f, 0);
if (rc != 0) {
    fprintf(stderr, "Failed: %s\n", sofa_bridge_get_error());
}
```

**Critical usage notes:**
- **Always pass torque in N·m, NOT N·mm.** The function multiplies by 1000 internally.
- Torque takes effect at step N+1 due to one-step lag in `ConstantForceField`. See [SOFA Patterns § Force Timing](sofa-patterns.md#force-timing).

**See also:** `sofa_step()`, `sofa_get_frame_snapshot()`

---

### `sofa_get_frame_snapshot`

**Signature:**
```c
SOFA_BRIDGE_API int sofa_get_frame_snapshot(SofaFrameSnapshot* out);
```

**Purpose:** Reads the current simulation state (bone poses, joint angles, ligament forces).

**Preconditions:**
- `sofa_bridge_init()` must have been called.
- A scene must be active.
- `out` must not be NULL.

**Parameters:**

| Name | Type | Ownership | Description |
|------|------|-----------|-------------|
| `out` | `SofaFrameSnapshot*` | Caller-allocated | Snapshot struct to fill. |

**Returns:** 0 on success, non-zero on error.

**Side effects:**
- If triple buffer has published data, reads from buffer (non-blocking).
- Otherwise, waits for in-progress async step, then directly fills snapshot from SOFA state.

**Thread safety:** Thread-safe when reading from triple buffer. Blocks on async step if no published data available.

**Example:**
```c
SofaFrameSnapshot snap;
int rc = sofa_get_frame_snapshot(&snap);
if (rc == 0) {
    if (snap.solver_diverged) {
        fprintf(stderr, "SOLVER DIVERGED — stop simulation\n");
        return 1;
    }
    printf("Dorsiflexion: %.2f deg\n", snap.joint_angles_deg[0]);
}
```

**Critical usage notes:**
- **Always check `snapshot.solver_diverged` after calling this function.** If 1, stop stepping immediately.
- Due to triple-buffering, snapshot may lag by 1 frame when using async stepping.
- Joint angles are computed from first two bones: `[0]` = tibia, `[1]` = talus. Order matters.

**See also:** `sofa_step()`, `sofa_step_async()`, [SOFA Patterns § Solver Divergence](sofa-patterns.md#solver-divergence)

---

### `sofa_step`

**Signature:**
```c
SOFA_BRIDGE_API int sofa_step(float dt);
```

**Purpose:** Advances the simulation by one timestep synchronously (blocks until complete).

**Preconditions:**
- `sofa_bridge_init()` must have been called.
- `dt` must be > 0.

**Parameters:**

| Name | Type | Ownership | Description |
|------|------|-----------|-------------|
| `dt` | `float` | Value | Timestep in seconds (must be positive). |

**Returns:** 0 on success, non-zero on error.

**Side effects:**
- **Automatically waits for any in-progress async step** before running synchronous step.
- If using new API + finalized scene: calls `applyLigamentForces()`, `sofa::simulation::node::animate()`, fills snapshot, publishes to triple buffer.
- If using legacy ankle scene: same sequence but uses `g_scene` state.
- If no scene active: calls `animate()` on empty scene (no error, but no-op).
- Increments `step_count`.
- Detects NaN in positions and sets `solver_diverged` flag.

**Thread safety:** Must be called from main thread only.

**Example:**
```c
for (int i = 0; i < 100; i++) {
    int rc = sofa_step(0.01f);
    if (rc != 0) {
        fprintf(stderr, "Step %d failed: %s\n", i, sofa_bridge_get_error());
        break;
    }
}
```

**See also:** `sofa_step_async()`, `sofa_get_frame_snapshot()`

---

### `sofa_step_async`

**Signature:**
```c
SOFA_BRIDGE_API int sofa_step_async(float dt);
```

**Purpose:** Starts a simulation step on a background thread (non-blocking).

**Preconditions:**
- `sofa_bridge_init()` must have been called.
- `dt` must be > 0.
- No async step currently in progress (returns error if `g_thread_manager.is_complete() == false`).

**Parameters:**

| Name | Type | Ownership | Description |
|------|------|-----------|-------------|
| `dt` | `float` | Value | Timestep in seconds. |

**Returns:** 0 on success, 1 if a step is already in progress.

**Side effects:**
- Spawns a background thread executing the same sequence as `sofa_step()`.
- Sets `g_thread_manager._step_complete = false`.
- Thread automatically publishes snapshot to triple buffer on completion.

**Thread safety:** Must be called from main thread only. Background thread accesses global SOFA state.

**Example:**
```c
int rc = sofa_step_async(0.01f);
if (rc != 0) {
    fprintf(stderr, "Async step already running\n");
}
// Continue with other work...
// Later: check completion or wait
```

**Critical usage notes:**
- **Always check `sofa_step_async_is_complete()` or call `sofa_step_async_wait()` before shutdown.**
- Snapshot may lag by 1 frame due to triple-buffering. Check `solver_diverged` on every read.

**See also:** `sofa_step_async_is_complete()`, `sofa_step_async_wait()`, [SOFA Patterns § Async Stepping](sofa-patterns.md#async-stepping)

---

### `sofa_step_async_is_complete`

**Signature:**
```c
SOFA_BRIDGE_API int sofa_step_async_is_complete(void);
```

**Purpose:** Polls whether the background async step has finished.

**Preconditions:** None.

**Parameters:** None.

**Returns:** 1 if complete (or no step running), 0 if still in progress.

**Side effects:** None.

**Thread safety:** Thread-safe (reads atomic boolean).

**Example:**
```c
sofa_step_async(0.01f);
while (!sofa_step_async_is_complete()) {
    // Do other work
}
// Step is complete
```

**See also:** `sofa_step_async()`, `sofa_step_async_wait()`

---

### `sofa_step_async_wait`

**Signature:**
```c
SOFA_BRIDGE_API void sofa_step_async_wait(void);
```

**Purpose:** Blocks until the background async step finishes. No-op if no step is running.

**Preconditions:** None.

**Parameters:** None.

**Returns:** None (void).

**Side effects:**
- Holds `_step_mutex` to prevent new async steps from starting.
- Spins on `_step_complete` atomic until true.
- Joins worker thread.

**Thread safety:** Must be called from main thread only.

**Example:**
```c
sofa_step_async(0.01f);
// ... do other work ...
sofa_step_async_wait(); // Block until complete
```

**Critical usage notes:**
- **Always call this in Unity `OnDisable()` BEFORE calling `Dispose()`.** Otherwise `sofa_bridge_shutdown()` blocks for 5 seconds.

**See also:** `sofa_step_async()`, `sofa_bridge_shutdown()`, [Native Boundary Guide § Shutdown Sequence](native-boundary.md#shutdown-sequence)

---

## Common Patterns

### Scene Construction Sequence (New API)

```c
// 1. Initialize bridge
sofa_bridge_init("/path/to/plugins");

// 2. Create scene
SofaSceneConfig cfg = { /* ... */ };
sofa_scene_create(&cfg);

// 3. Add bones
SofaRigidBoneConfig tibia = { /* ... */ };
sofa_add_rigid_bone(&tibia);

SofaRigidBoneConfig talus = { /* ... */ };
sofa_add_rigid_bone(&talus);

// 4. Add ligaments
SofaLigamentConfig atfl = { /* ... */ };
sofa_add_ligament(&atfl);

// 5. Finalize
sofa_scene_finalize();

// 6. Step loop
for (int i = 0; i < 100; i++) {
    sofa_step(0.01f);

    SofaFrameSnapshot snap;
    sofa_get_frame_snapshot(&snap);
    if (snap.solver_diverged) {
        fprintf(stderr, "Diverged at step %d\n", i);
        break;
    }
}

// 7. Cleanup
sofa_scene_destroy();
sofa_bridge_shutdown();
```

### Resection Workflow

```c
// After scene finalized and stepped
SofaResectionCommand cut = {
    .plane_point = {0, 0, 20},
    .plane_normal = {0, 0, 1},
    .bone_name = "TibiaDeformable"
};

sofa_execute_resection(&cut);

if (sofa_has_topology_changed()) {
    // Extract updated mesh
    SofaSurfaceMesh mesh = { /* pre-allocated buffers */ };
    sofa_get_surface_mesh(&mesh);

    printf("New mesh: %d verts, %d tris\n",
           mesh.vertex_count, mesh.triangle_count);
}
```

### Async Stepping Pattern

```c
// Start async step
sofa_step_async(0.01f);

// Poll or do other work
while (!sofa_step_async_is_complete()) {
    // Render, input handling, etc.
}

// Read snapshot (from triple buffer)
SofaFrameSnapshot snap;
sofa_get_frame_snapshot(&snap);

// Before shutdown: MUST wait
sofa_step_async_wait();
sofa_bridge_shutdown();
```

---

## Error Handling Checklist

For every function that returns `int`:

1. **Check return value:** `if (rc != 0) { /* handle error */ }`
2. **Read error string immediately:** `const char* err = sofa_bridge_get_error();`
3. **Copy string before next call:** Error buffer is overwritten on next function call.
4. **Check `solver_diverged` flag:** After every `sofa_get_frame_snapshot()`, check if simulation has diverged.

---

## Thread Safety Summary

| Function | Thread Safety |
|----------|---------------|
| `sofa_bridge_get_version()` | Thread-safe (constants) |
| `sofa_bridge_init()` | Main thread only |
| `sofa_bridge_shutdown()` | Main thread only (joins async thread) |
| `sofa_bridge_get_error()` | NOT thread-safe (global string) |
| `sofa_scene_create()` | Main thread only |
| `sofa_scene_destroy()` | Main thread only (joins async thread) |
| `sofa_scene_is_ready()` | Thread-safe (read enum) |
| `sofa_scene_finalize()` | Main thread only |
| `sofa_add_rigid_bone()` | Main thread only |
| `sofa_add_ligament()` | Main thread only |
| `sofa_add_deformable_tissue()` | Main thread only |
| `sofa_execute_resection()` | Main thread only (waits for async step) |
| `sofa_get_removed_element_count()` | Thread-safe (read int) |
| `sofa_has_topology_changed()` | Thread-safe (read bool) |
| `sofa_get_surface_mesh()` | Main thread only |
| `sofa_scene_create_ankle()` | Main thread only |
| `sofa_scene_create_ankle_ex()` | Main thread only |
| `sofa_apply_torque()` | Main thread only (waits for async step) |
| `sofa_get_frame_snapshot()` | Thread-safe when reading from buffer, else waits |
| `sofa_step()` | Main thread only (waits for async step) |
| `sofa_step_async()` | Main thread only (spawns thread) |
| `sofa_step_async_is_complete()` | Thread-safe (atomic read) |
| `sofa_step_async_wait()` | Main thread only (joins thread) |

**General rule:** Assume main-thread-only unless explicitly marked thread-safe. All functions that modify SOFA state or spawn/join threads MUST be called from the main thread.

---

## See Also

- [Native Boundary Guide](native-boundary.md) — Memory ownership, P/Invoke patterns, struct marshaling
- [SOFA Patterns](sofa-patterns.md) — Ligament mechanics, solver stability, resection algorithm
- [Testing Guide](testing.md) — Integration test examples using this API
- [ADR-0001 Centroid-Based Resection](../adr/0001-centroid-based-resection.md)
- [ADR-0003 Undo via Scene Rebuild](../adr/0003-undo-via-scene-rebuild.md)

---

**Revision history:**
- 2026-02-15: Initial version covering all 26 functions and 8 structs (Sprint 4 complete).
