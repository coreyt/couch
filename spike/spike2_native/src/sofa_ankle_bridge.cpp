#include "sofa_ankle_bridge.h"

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
        sofa::simulation::node::animate(g_root.get(), static_cast<double>(dt));
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
