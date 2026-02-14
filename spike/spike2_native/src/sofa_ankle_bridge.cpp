#include "sofa_ankle_bridge.h"
#include "ankle_scene.h"

#include <sofa/simpleapi/SimpleApi.h>
#include <sofa/simpleapi/init.h>
#include <sofa/simulation/graph/init.h>
#include <sofa/simulation/Simulation.h>
#include <sofa/simulation/Node.h>
#include <sofa/helper/system/PluginManager.h>

#include <string>
#include <sstream>

static bool g_initialized = false;
static sofa::simulation::NodeSPtr g_root;
static std::string g_last_error;
static AnkleSceneState g_scene;

static void set_error(const std::string& msg) {
    g_last_error = msg;
}

int sofa_bridge_init(const char* plugin_dir) {
    if (g_initialized) {
        set_error("Already initialized — call sofa_bridge_shutdown() first");
        return 1;
    }

    try {
        sofa::simulation::graph::init();
        sofa::simpleapi::init();

        if (plugin_dir) {
            auto& pm = sofa::helper::system::PluginManager::getInstance();
            pm.init(std::string(plugin_dir));
        }

        sofa::simpleapi::importPlugin("Sofa.Component.StateContainer");

        auto sim = sofa::simpleapi::createSimulation("DAG");
        g_root = sofa::simpleapi::createRootNode(sim, "root");
        sofa::simulation::node::initRoot(g_root.get());

        g_scene = AnkleSceneState{}; // reset scene state
        g_initialized = true;
        g_last_error.clear();
        return 0;
    } catch (const std::exception& e) {
        set_error(std::string("Init failed: ") + e.what());
        return 1;
    } catch (...) {
        set_error("Init failed: unknown error");
        return 1;
    }
}

int sofa_step(float dt) {
    if (!g_initialized) {
        set_error("Not initialized — call sofa_bridge_init() first");
        return 1;
    }
    if (dt <= 0.0f) {
        set_error("dt must be > 0");
        return 1;
    }

    try {
        if (g_scene.is_active) {
            apply_ligament_forces(g_scene);
        }
        sofa::simulation::node::animate(g_root.get(), static_cast<double>(dt));
        if (g_scene.is_active) {
            g_scene.step_count++;
        }
        g_last_error.clear();
        return 0;
    } catch (const std::exception& e) {
        set_error(std::string("Step failed: ") + e.what());
        return 1;
    } catch (...) {
        set_error("Step failed: unknown error");
        return 1;
    }
}

void sofa_bridge_shutdown() {
    if (!g_initialized) {
        return;
    }

    try {
        g_scene = AnkleSceneState{}; // reset before unload
        if (g_root) {
            sofa::simulation::node::unload(g_root);
            g_root.reset();
        }
        sofa::simpleapi::cleanup();
        sofa::simulation::graph::cleanup();
    } catch (...) {
        // Idempotent — swallow errors during shutdown
    }

    g_initialized = false;
    g_last_error.clear();
}

const char* sofa_bridge_get_error() {
    return g_last_error.c_str();
}

// ---------------------------------------------------------------------------
// Ankle scene functions
// ---------------------------------------------------------------------------

int sofa_scene_create_ankle(float dt, float gravity_z) {
    return sofa_scene_create_ankle_ex(dt, gravity_z,
        DEFAULT_ANKLE_LIGAMENTS, DEFAULT_ANKLE_LIGAMENT_COUNT);
}

int sofa_scene_create_ankle_ex(float dt, float gravity_z,
                               const SofaLigamentConfig* ligaments,
                               int num_ligaments) {
    if (!g_initialized) {
        set_error("Not initialized — call sofa_bridge_init() first");
        return 1;
    }
    if (g_scene.is_active) {
        set_error("Ankle scene already active — shutdown and reinit first");
        return 1;
    }
    if (!ligaments || num_ligaments <= 0) {
        set_error("Must provide at least one ligament");
        return 1;
    }

    try {
        // Unload existing empty root and create a fresh one for the ankle scene
        sofa::simulation::node::unload(g_root);
        g_root.reset();

        auto sim = sofa::simpleapi::createSimulation("DAG");
        g_root = sofa::simpleapi::createRootNode(sim, "root");

        int rc = build_ankle_scene(g_root, g_scene, dt, gravity_z,
                                   ligaments, num_ligaments);
        if (rc != 0) {
            set_error("Failed to build ankle scene — typed pointer resolution failed");
            return 1;
        }

        sofa::simulation::node::initRoot(g_root.get());

        g_last_error.clear();
        return 0;
    } catch (const std::exception& e) {
        set_error(std::string("Create ankle scene failed: ") + e.what());
        return 1;
    } catch (...) {
        set_error("Create ankle scene failed: unknown error");
        return 1;
    }
}

int sofa_apply_torque(float torque_nm, int axis) {
    if (!g_initialized) {
        set_error("Not initialized");
        return 1;
    }
    if (!g_scene.is_active) {
        set_error("No ankle scene active");
        return 1;
    }
    if (axis < 0 || axis > 2) {
        set_error("Axis must be 0 (sagittal), 1 (frontal), or 2 (transverse)");
        return 1;
    }

    try {
        // Build the 6-component wrench: [fx fy fz tx ty tz]
        double torque[3] = {0, 0, 0};
        torque[axis] = static_cast<double>(torque_nm);

        auto* d_forces = g_scene.torque_ff->findData("forces");
        if (d_forces) {
            std::ostringstream ss;
            ss << "0 0 0 " << torque[0] << " " << torque[1] << " " << torque[2];
            d_forces->read(ss.str());
        }

        g_last_error.clear();
        return 0;
    } catch (const std::exception& e) {
        set_error(std::string("Apply torque failed: ") + e.what());
        return 1;
    } catch (...) {
        set_error("Apply torque failed: unknown error");
        return 1;
    }
}

int sofa_get_frame_snapshot(SofaFrameSnapshot* out) {
    if (!g_initialized) {
        set_error("Not initialized");
        return 1;
    }
    if (!g_scene.is_active) {
        set_error("No ankle scene active");
        return 1;
    }
    if (!out) {
        set_error("Null output pointer");
        return 1;
    }

    try {
        int rc = fill_frame_snapshot(g_scene, out);
        if (rc != 0) {
            set_error("Failed to fill frame snapshot");
            return 1;
        }
        g_last_error.clear();
        return 0;
    } catch (const std::exception& e) {
        set_error(std::string("Get snapshot failed: ") + e.what());
        return 1;
    } catch (...) {
        set_error("Get snapshot failed: unknown error");
        return 1;
    }
}
