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

#ifdef __cplusplus
}
#endif

#endif // SOFA_ANKLE_BRIDGE_H
