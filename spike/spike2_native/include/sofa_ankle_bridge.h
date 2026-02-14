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
    double stiffness;           // N/mm
    double damping;             // N·s/mm
    double rest_length;         // mm, 0 = auto-compute
} SofaLigamentConfig;

typedef struct {
    SofaRigidFrame tibia;
    SofaRigidFrame talus;
    double joint_angles_deg[3]; // [sagittal, frontal, transverse]
    double ligament_force[3];   // total force on talus (N)
    double ligament_torque[3];  // total torque on talus (N·mm)
    int    step_count;
} SofaFrameSnapshot;

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

// ---- Ankle scene ----

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
/// @param torque_nm Torque magnitude in N·mm.
/// @param axis 0=sagittal(X), 1=frontal(Y), 2=transverse(Z).
/// @return 0 on success, non-zero on error.
SOFA_BRIDGE_API int sofa_apply_torque(float torque_nm, int axis);

/// Read current frame snapshot (positions, angles, forces).
/// @param out Pointer to snapshot struct to fill.
/// @return 0 on success, non-zero on error.
SOFA_BRIDGE_API int sofa_get_frame_snapshot(SofaFrameSnapshot* out);

#ifdef __cplusplus
}
#endif

#endif // SOFA_ANKLE_BRIDGE_H
