#include "sofa_ankle_bridge.h"
#include "ankle_scene.h"
#include "scene_builder.h"
#include "triple_buffer.h"
#include "thread_manager.h"

#include <sofa/simpleapi/SimpleApi.h>
#include <sofa/simpleapi/init.h>
#include <sofa/simulation/graph/init.h>
#include <sofa/simulation/Simulation.h>
#include <sofa/simulation/Node.h>
#include <sofa/helper/system/PluginManager.h>

#include <chrono>
#include <cmath>
#include <string>
#include <sstream>

static bool g_initialized = false;
static sofa::simulation::NodeSPtr g_root;
static std::string g_last_error;
static AnkleSceneState g_scene;          // legacy scene state
static ThreadManager g_thread_manager;
static SceneBuilder g_builder;           // new scene builder
static TripleBuffer<SofaFrameSnapshot> g_snapshot_buffer;
static bool g_use_builder = false;       // true when new API is active

static void set_error(const std::string& msg) {
    g_last_error = msg;
}

// Check if positions contain NaN (solver divergence)
static bool check_diverged(const SofaFrameSnapshot& snap) {
    return std::isnan(snap.tibia.px) || std::isnan(snap.tibia.py) || std::isnan(snap.tibia.pz) ||
           std::isnan(snap.talus.px) || std::isnan(snap.talus.py) || std::isnan(snap.talus.pz) ||
           std::isnan(snap.calcaneus.px) || std::isnan(snap.calcaneus.py) || std::isnan(snap.calcaneus.pz);
}

SofaBridgeVersion sofa_bridge_get_version() {
    SofaBridgeVersion v = {};
    v.bridge_version_major = 0;
    v.bridge_version_minor = 3;
    v.bridge_version_patch = 0;
    v.sofa_version_major = 24;
    v.sofa_version_minor = 6;
    return v;
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

            // Pre-load all required SOFA plugins by full path
            const char* plugins[] = {
                "Sofa.Component.StateContainer",
                "Sofa.Component.Mass",
                "Sofa.Component.ODESolver.Backward",
                "Sofa.Component.LinearSolver.Iterative",
                "Sofa.Component.LinearSolver.Direct",
                "Sofa.Component.Constraint.Lagrangian.Solver",
                "Sofa.Component.Constraint.Lagrangian.Correction",
                "Sofa.Component.Constraint.Projective",
                "Sofa.Component.Collision.Detection.Algorithm",
                "Sofa.Component.Collision.Detection.Intersection",
                "Sofa.Component.Collision.Geometry",
                "Sofa.Component.Collision.Response.Contact",
                "Sofa.Component.AnimationLoop",
                "Sofa.Component.Mapping.Linear",
                "Sofa.Component.Topology.Container.Constant",
                "Sofa.Component.Topology.Container.Dynamic",
                "Sofa.Component.Topology.Mapping",
                "Sofa.Component.MechanicalLoad",
                "Sofa.Component.SolidMechanics.FEM.Elastic",
            };

#ifdef _WIN32
            const std::string ext = ".dll";
            const std::string sep = "\\";
#else
            const std::string ext = ".so";
            const std::string sep = "/";
#endif
            std::string dir(plugin_dir);
            std::string load_errors;
            for (const auto& p : plugins) {
                std::string fullPath = dir + sep + p + ext;
                std::ostringstream errlog;
                auto status = pm.loadPluginByPath(fullPath, &errlog);
                if (status != sofa::helper::system::PluginManager::PluginLoadStatus::SUCCESS &&
                    status != sofa::helper::system::PluginManager::PluginLoadStatus::ALREADY_LOADED) {
                    load_errors += std::string("  ") + p + ": " + errlog.str() + "\n";
                }
            }
            if (!load_errors.empty()) {
                set_error("Plugin loading errors (plugin_dir=" + dir + "):\n" + load_errors);
                return 1;
            }
        } else {
            sofa::simpleapi::importPlugin("Sofa.Component.StateContainer");
        }

        auto sim = sofa::simpleapi::createSimulation("DAG");
        g_root = sofa::simpleapi::createRootNode(sim, "root");
        sofa::simulation::node::initRoot(g_root.get());

        g_scene = AnkleSceneState{}; // reset scene state
        g_builder.reset();
        g_use_builder = false;
        g_snapshot_buffer.reset();
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

    // Wait for any in-progress async step before running a synchronous one
    g_thread_manager.wait();

    try {
        auto start = std::chrono::high_resolution_clock::now();

        if (g_use_builder && g_builder.state() == SceneBuilderState::Finalized) {
            g_builder.applyLigamentForces();
            sofa::simulation::node::animate(g_root.get(), static_cast<double>(dt));
            g_builder.incrementStepCount();

            // Fill triple buffer
            auto& wb = g_snapshot_buffer.writeBuffer();
            g_builder.fillSnapshot(&wb);
            auto end = std::chrono::high_resolution_clock::now();
            wb.step_time_ms = std::chrono::duration<float, std::milli>(end - start).count();
            wb.solver_diverged = check_diverged(wb) ? 1 : 0;
            g_snapshot_buffer.publish();
        } else if (g_scene.is_active) {
            apply_ligament_forces(g_scene);
            sofa::simulation::node::animate(g_root.get(), static_cast<double>(dt));
            g_scene.step_count++;

            // Fill triple buffer for legacy scene too
            auto& wb = g_snapshot_buffer.writeBuffer();
            fill_frame_snapshot(g_scene, &wb);
            auto end = std::chrono::high_resolution_clock::now();
            wb.step_time_ms = std::chrono::duration<float, std::milli>(end - start).count();
            wb.solver_diverged = check_diverged(wb) ? 1 : 0;
            g_snapshot_buffer.publish();
        } else {
            sofa::simulation::node::animate(g_root.get(), static_cast<double>(dt));
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

    // Join any in-progress async step before cleanup
    g_thread_manager.shutdown(5000);

    try {
        g_scene = AnkleSceneState{}; // reset before unload
        g_builder.reset();
        g_use_builder = false;
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

    // Reset thread manager for potential re-init
    g_thread_manager.reset();
}

const char* sofa_bridge_get_error() {
    return g_last_error.c_str();
}

// ---------------------------------------------------------------------------
// New Scene Construction API (Sprint 2)
// ---------------------------------------------------------------------------

int sofa_scene_create(const SofaSceneConfig* config) {
    if (!g_initialized) {
        set_error("Not initialized — call sofa_bridge_init() first");
        return 1;
    }
    if (g_scene.is_active || g_builder.state() != SceneBuilderState::Empty) {
        set_error("Scene already active — destroy first");
        return 1;
    }
    if (!config) {
        set_error("Null config pointer");
        return 1;
    }

    try {
        // Unload existing empty root and create a fresh one
        sofa::simulation::node::unload(g_root);
        g_root.reset();

        auto sim = sofa::simpleapi::createSimulation("DAG");
        g_root = sofa::simpleapi::createRootNode(sim, "root");

        int rc = g_builder.createScene(g_root, *config);
        if (rc != 0) {
            set_error("Scene creation failed: " + g_builder.lastError());
            return 1;
        }

        g_use_builder = true;
        g_snapshot_buffer.reset();
        g_last_error.clear();
        return 0;
    } catch (const std::exception& e) {
        set_error(std::string("Scene creation failed: ") + e.what());
        return 1;
    } catch (...) {
        set_error("Scene creation failed: unknown error");
        return 1;
    }
}

void sofa_scene_destroy() {
    if (!g_initialized) return;

    g_thread_manager.wait();

    g_builder.reset();
    g_use_builder = false;
    g_scene = AnkleSceneState{};

    try {
        if (g_root) {
            sofa::simulation::node::unload(g_root);
            g_root.reset();
        }
        // Create a fresh empty root
        auto sim = sofa::simpleapi::createSimulation("DAG");
        g_root = sofa::simpleapi::createRootNode(sim, "root");
        sofa::simulation::node::initRoot(g_root.get());
    } catch (...) {
        // Best effort
    }

    g_snapshot_buffer.reset();
}

int sofa_scene_is_ready() {
    if (g_use_builder) {
        return g_builder.state() == SceneBuilderState::Finalized ? 1 : 0;
    }
    return g_scene.is_active ? 1 : 0;
}

int sofa_scene_finalize() {
    if (!g_initialized) {
        set_error("Not initialized");
        return 1;
    }
    if (!g_use_builder) {
        set_error("No scene created via new API");
        return 1;
    }

    try {
        int rc = g_builder.finalize();
        if (rc != 0) {
            set_error("Finalize failed: " + g_builder.lastError());
            return 1;
        }
        g_last_error.clear();
        return 0;
    } catch (const std::exception& e) {
        set_error(std::string("Finalize failed: ") + e.what());
        return 1;
    } catch (...) {
        set_error("Finalize failed: unknown error");
        return 1;
    }
}

int sofa_add_rigid_bone(const SofaRigidBoneConfig* config) {
    if (!g_initialized) {
        set_error("Not initialized");
        return 1;
    }
    if (!g_use_builder) {
        set_error("No scene created via new API — call sofa_scene_create() first");
        return 1;
    }
    if (!config) {
        set_error("Null config pointer");
        return 1;
    }

    try {
        int rc = g_builder.addRigidBone(*config);
        if (rc != 0) {
            set_error(g_builder.lastError());
            return 1;
        }
        g_last_error.clear();
        return 0;
    } catch (const std::exception& e) {
        set_error(std::string("Add bone failed: ") + e.what());
        return 1;
    } catch (...) {
        set_error("Add bone failed: unknown error");
        return 1;
    }
}

int sofa_add_ligament(const SofaLigamentConfig* config) {
    if (!g_initialized) {
        set_error("Not initialized");
        return 1;
    }
    if (!g_use_builder) {
        set_error("No scene created via new API — call sofa_scene_create() first");
        return 1;
    }
    if (!config) {
        set_error("Null config pointer");
        return 1;
    }

    try {
        int rc = g_builder.addLigament(*config);
        if (rc != 0) {
            set_error(g_builder.lastError());
            return 1;
        }
        g_last_error.clear();
        return 0;
    } catch (const std::exception& e) {
        set_error(std::string("Add ligament failed: ") + e.what());
        return 1;
    } catch (...) {
        set_error("Add ligament failed: unknown error");
        return 1;
    }
}

// ---------------------------------------------------------------------------
// Deformable tissue + Resection API (Sprint 4)
// ---------------------------------------------------------------------------

int sofa_add_deformable_tissue(const SofaDeformableConfig* config) {
    if (!g_initialized) {
        set_error("Not initialized");
        return 1;
    }
    if (!g_use_builder) {
        set_error("No scene created via new API — call sofa_scene_create() first");
        return 1;
    }
    if (!config) {
        set_error("Null config pointer");
        return 1;
    }

    try {
        int rc = g_builder.addDeformableTissue(*config);
        if (rc != 0) {
            set_error(g_builder.lastError());
            return 1;
        }
        g_last_error.clear();
        return 0;
    } catch (const std::exception& e) {
        set_error(std::string("Add deformable tissue failed: ") + e.what());
        return 1;
    } catch (...) {
        set_error("Add deformable tissue failed: unknown error");
        return 1;
    }
}

int sofa_execute_resection(const SofaResectionCommand* cmd) {
    if (!g_initialized) {
        set_error("Not initialized");
        return 1;
    }
    if (!g_use_builder) {
        set_error("No scene created via new API");
        return 1;
    }
    if (!cmd) {
        set_error("Null command pointer");
        return 1;
    }

    // Wait for any in-progress async step to avoid data race
    g_thread_manager.wait();

    try {
        int rc = g_builder.executeResection(*cmd);
        if (rc != 0) {
            set_error(g_builder.lastError());
            return 1;
        }
        g_last_error.clear();
        return 0;
    } catch (const std::exception& e) {
        set_error(std::string("Resection failed: ") + e.what());
        return 1;
    } catch (...) {
        set_error("Resection failed: unknown error");
        return 1;
    }
}

int sofa_get_removed_element_count() {
    return g_builder.removedElementCount();
}

int sofa_has_topology_changed() {
    return g_builder.hasTopologyChanged() ? 1 : 0;
}

int sofa_get_surface_mesh(SofaSurfaceMesh* out) {
    if (!g_initialized) {
        set_error("Not initialized");
        return 1;
    }
    if (!out) {
        set_error("Null output pointer");
        return 1;
    }

    try {
        int rc = g_builder.getSurfaceMesh(out);
        if (rc != 0) {
            set_error("Failed to get surface mesh");
            return 1;
        }
        g_last_error.clear();
        return 0;
    } catch (const std::exception& e) {
        set_error(std::string("Get surface mesh failed: ") + e.what());
        return 1;
    } catch (...) {
        set_error("Get surface mesh failed: unknown error");
        return 1;
    }
}

// ---------------------------------------------------------------------------
// Legacy ankle scene functions (reimplemented on new primitives)
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
    if (g_scene.is_active || g_builder.state() != SceneBuilderState::Empty) {
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

        g_use_builder = false;
        g_snapshot_buffer.reset();
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

    // Wait for any in-progress async step to avoid data race on force fields
    g_thread_manager.wait();

    if (g_use_builder && g_builder.state() == SceneBuilderState::Finalized) {
        // Apply to first free bone
        BoneState* free_bone = g_builder.firstFreeBone();
        if (!free_bone) {
            set_error("No free bone to apply torque to");
            return 1;
        }
        int rc = g_builder.applyTorque(free_bone->name.c_str(), torque_nm, axis);
        if (rc != 0) {
            set_error(g_builder.lastError());
            return 1;
        }
        g_last_error.clear();
        return 0;
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
        torque[axis] = static_cast<double>(torque_nm) * 1000.0; // N·m → N·mm

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
    if (!out) {
        set_error("Null output pointer");
        return 1;
    }

    // Try triple buffer first (non-blocking — preserves async stepping benefit)
    if (g_snapshot_buffer.hasData()) {
        *out = g_snapshot_buffer.read();
        g_last_error.clear();
        return 0;
    }

    // No published data yet — wait for any in-progress async step, then direct fill
    g_thread_manager.wait();

    // Fall back to direct fill
    if (g_use_builder && g_builder.state() == SceneBuilderState::Finalized) {
        int rc = g_builder.fillSnapshot(out);
        if (rc != 0) {
            set_error("Failed to fill frame snapshot");
            return 1;
        }
        g_last_error.clear();
        return 0;
    }

    if (g_scene.is_active) {
        int rc = fill_frame_snapshot(g_scene, out);
        if (rc != 0) {
            set_error("Failed to fill frame snapshot");
            return 1;
        }
        g_last_error.clear();
        return 0;
    }

    set_error("No scene active");
    return 1;
}

// ---------------------------------------------------------------------------
// Async stepping
// ---------------------------------------------------------------------------

int sofa_step_async(float dt) {
    if (!g_initialized) {
        set_error("Not initialized — call sofa_bridge_init() first");
        return 1;
    }
    if (dt <= 0.0f) {
        set_error("dt must be > 0");
        return 1;
    }

    bool started = g_thread_manager.start_step([dt]() {
        if (g_thread_manager.cancel_requested()) return;

        auto start = std::chrono::high_resolution_clock::now();

        if (g_use_builder && g_builder.state() == SceneBuilderState::Finalized) {
            g_builder.applyLigamentForces();
            sofa::simulation::node::animate(g_root.get(), static_cast<double>(dt));
            g_builder.incrementStepCount();

            auto& wb = g_snapshot_buffer.writeBuffer();
            g_builder.fillSnapshot(&wb);
            auto end = std::chrono::high_resolution_clock::now();
            wb.step_time_ms = std::chrono::duration<float, std::milli>(end - start).count();
            wb.solver_diverged = check_diverged(wb) ? 1 : 0;
            g_snapshot_buffer.publish();
        } else if (g_scene.is_active) {
            apply_ligament_forces(g_scene);
            sofa::simulation::node::animate(g_root.get(), static_cast<double>(dt));
            g_scene.step_count++;

            auto& wb = g_snapshot_buffer.writeBuffer();
            fill_frame_snapshot(g_scene, &wb);
            auto end = std::chrono::high_resolution_clock::now();
            wb.step_time_ms = std::chrono::duration<float, std::milli>(end - start).count();
            wb.solver_diverged = check_diverged(wb) ? 1 : 0;
            g_snapshot_buffer.publish();
        } else {
            sofa::simulation::node::animate(g_root.get(), static_cast<double>(dt));
        }
    });

    if (!started) {
        set_error("Async step already in progress");
        return 1;
    }

    g_last_error.clear();
    return 0;
}

int sofa_step_async_is_complete() {
    return g_thread_manager.is_complete() ? 1 : 0;
}

void sofa_step_async_wait() {
    g_thread_manager.wait();
}
