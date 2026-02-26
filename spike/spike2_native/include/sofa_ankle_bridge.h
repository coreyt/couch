#ifndef SOFA_ANKLE_BRIDGE_H
#define SOFA_ANKLE_BRIDGE_H

#ifdef _WIN32
    #ifdef SOFA_ANKLE_BRIDGE_EXPORTS
        #define SOFA_BRIDGE_API __declspec(dllexport)
    #else
        #define SOFA_BRIDGE_API __declspec(dllimport)
    #endif
#else
    #define SOFA_BRIDGE_API __attribute__((visibility("default")))
#endif

#ifdef __cplusplus
extern "C" {
#endif

// ---- Version ----

typedef struct {
    int bridge_version_major;  // 0 (pre-release)
    int bridge_version_minor;  // 2
    int bridge_version_patch;  // 0
    int sofa_version_major;    // 24
    int sofa_version_minor;    // 6
} SofaBridgeVersion;

SOFA_BRIDGE_API SofaBridgeVersion sofa_bridge_get_version(void);

// ---- Data structs ----

typedef struct {
    double px, py, pz;
    double qx, qy, qz, qw;
} SofaRigidFrame;

typedef struct {
    const char* name;
    double tibia_offset[3];
    double talus_offset[3];
    double fixed_anchor[3];     // used only if use_fixed_anchor=1
    int    use_fixed_anchor;
    double stiffness;           // N/mm (= linear_stiffness)
    double damping;             // N·s/mm
    double rest_length;         // mm, 0 = auto-compute
    // --- New bilinear fields (Sprint 2) ---
    const char* bone_a_name;    // NULL = "Tibia" (backward compat)
    const char* bone_b_name;    // NULL = "Talus" (backward compat)
    double toe_stiffness;       // N/mm, 0 = use stiffness
    double toe_region_strain;   // fraction, 0 = pure linear
} SofaLigamentConfig;

typedef struct {
    SofaRigidFrame tibia;
    SofaRigidFrame talus;
    SofaRigidFrame calcaneus;
    double joint_angles_deg[3]; // [sagittal, frontal, transverse]
    double ligament_force[3];   // total force on talus (N)
    double ligament_torque[3];  // total torque on talus (N·mm)
    float  step_time_ms;        // chrono-measured step time
    int    solver_diverged;     // 1 if NaN detected in positions
    int    step_count;
} SofaFrameSnapshot;

// ---- Scene construction API (Sprint 2) ----

typedef struct {
    float gravity[3];
    float timestep;
    int   constraint_iterations;
    float constraint_tolerance;
    float rayleigh_stiffness;
    float rayleigh_mass;
    float alarm_distance;
    float contact_distance;
    float friction_coefficient;
} SofaSceneConfig;

typedef struct {
    const char* name;
    const float* collision_vertices;   // flattened [x,y,z, ...]
    int          collision_vertex_count;
    const int*   collision_triangles;   // flattened [i0,i1,i2, ...]
    int          collision_triangle_count;
    float        position[3];
    float        orientation[4];        // quaternion [x,y,z,w]
    float        mass;
    int          is_fixed;
} SofaRigidBoneConfig;

/// Create a new scene with physics pipeline. Must be called after sofa_bridge_init().
/// @return 0 on success, non-zero on error.
SOFA_BRIDGE_API int sofa_scene_create(const SofaSceneConfig* config);

/// Destroy the current scene, releasing all resources.
SOFA_BRIDGE_API void sofa_scene_destroy(void);

/// Check if the scene is finalized and ready for stepping.
/// @return 1 if ready, 0 if not.
SOFA_BRIDGE_API int sofa_scene_is_ready(void);

/// Finalize the scene: init root, resolve typed pointers.
/// @return 0 on success, non-zero on error.
SOFA_BRIDGE_API int sofa_scene_finalize(void);

/// Add a rigid bone to the scene (between create and finalize).
/// @return 0 on success, non-zero on error.
SOFA_BRIDGE_API int sofa_add_rigid_bone(const SofaRigidBoneConfig* config);

/// Add a ligament between two bones (between create and finalize).
/// @return 0 on success, non-zero on error.
SOFA_BRIDGE_API int sofa_add_ligament(const SofaLigamentConfig* config);

// ---- Lifecycle ----

/// Initialize SOFA simulation engine and create an empty scene.
/// @param plugin_dir Path to SOFA plugin directory (may be NULL).
/// @return 0 on success, non-zero on error.
SOFA_BRIDGE_API int sofa_bridge_init(const char* plugin_dir);

/// Advance the simulation by one timestep.
/// @param dt Timestep in seconds (must be > 0).
/// @return 0 on success, non-zero on error.
SOFA_BRIDGE_API int sofa_step(float dt);

/// Shut down the simulation and release resources. Idempotent.
SOFA_BRIDGE_API void sofa_bridge_shutdown(void);

/// Return the last error message, or empty string if none.
SOFA_BRIDGE_API const char* sofa_bridge_get_error(void);

// ---- Ankle scene (legacy API) ----

/// Create an ankle scene with default 4 ligaments.
/// @param dt Timestep in seconds.
/// @param gravity_z Gravity along Z axis in mm/s² (e.g. -9810).
/// @return 0 on success, non-zero on error.
SOFA_BRIDGE_API int sofa_scene_create_ankle(float dt, float gravity_z);

/// Create an ankle scene with custom ligament configuration.
/// @param dt Timestep in seconds.
/// @param gravity_z Gravity along Z axis in mm/s².
/// @param ligaments Array of ligament configurations.
/// @param num_ligaments Number of ligaments in the array.
/// @return 0 on success, non-zero on error.
SOFA_BRIDGE_API int sofa_scene_create_ankle_ex(float dt, float gravity_z,
    const SofaLigamentConfig* ligaments, int num_ligaments);

/// Apply an external torque about a given axis.
/// @param torque_nm Torque magnitude in N·m (converted to N·mm internally).
/// @param axis 0=sagittal(X), 1=frontal(Y), 2=transverse(Z).
/// @return 0 on success, non-zero on error.
SOFA_BRIDGE_API int sofa_apply_torque(float torque_nm, int axis);

/// Read current frame snapshot (positions, angles, forces).
/// @param out Pointer to snapshot struct to fill.
/// @return 0 on success, non-zero on error.
SOFA_BRIDGE_API int sofa_get_frame_snapshot(SofaFrameSnapshot* out);

// ---- Deformable tissue + Resection (Sprint 4) ----

typedef struct {
    const char* name;           // e.g., "TibiaDeformable"
    const char* parent_bone;    // rigid bone to attach to (or NULL)
    const float* vertices;      // flattened [x,y,z, ...]
    int vertex_count;
    const int* tetrahedra;      // flattened [i0,i1,i2,i3, ...]
    int tetra_count;
    float young_modulus;        // Pa (cortical bone ~17 GPa)
    float poisson_ratio;        // ~0.3 for bone
    float mass_density;         // kg/mm³
} SofaDeformableConfig;

typedef struct {
    float plane_point[3];
    float plane_normal[3];
    const char* bone_name;
} SofaResectionCommand;

typedef struct {
    float* vertices;            // caller-allocated
    int* triangles;             // caller-allocated
    int vertex_count;           // in/out: capacity in, actual out
    int triangle_count;         // in/out: capacity in, actual out
} SofaSurfaceMesh;

/// Add deformable tissue with tetrahedral FEM to the scene.
/// @return 0 on success, non-zero on error.
SOFA_BRIDGE_API int sofa_add_deformable_tissue(const SofaDeformableConfig* config);

/// Execute a resection: remove tetrahedra whose centroid is below the cut plane.
/// @return 0 on success, non-zero on error.
SOFA_BRIDGE_API int sofa_execute_resection(const SofaResectionCommand* cmd);

/// Get the number of tetrahedra removed in the last resection.
/// @return count of removed elements.
SOFA_BRIDGE_API int sofa_get_removed_element_count(void);

/// Check if topology has changed since last query.
/// @return 1 if changed, 0 if not.
SOFA_BRIDGE_API int sofa_has_topology_changed(void);

/// Extract the current surface mesh from the deformable tissue.
/// @param out Pointer to SofaSurfaceMesh with pre-allocated arrays.
/// @return 0 on success, non-zero on error.
SOFA_BRIDGE_API int sofa_get_surface_mesh(SofaSurfaceMesh* out);

// ---- Async stepping ----

/// Start a simulation step on a background thread. Non-blocking.
/// @param dt Timestep in seconds (must be > 0).
/// @return 0 on success, non-zero if a step is already in progress.
SOFA_BRIDGE_API int sofa_step_async(float dt);

/// Poll whether the async step has completed.
/// @return 1 if complete (or no step running), 0 if still in progress.
SOFA_BRIDGE_API int sofa_step_async_is_complete(void);

/// Block until the async step finishes. No-op if no step running.
SOFA_BRIDGE_API void sofa_step_async_wait(void);

#ifdef __cplusplus
}
#endif

#endif // SOFA_ANKLE_BRIDGE_H
