#ifndef ANKLE_SCENE_H
#define ANKLE_SCENE_H

#include "sofa_ankle_bridge.h"

#include <sofa/simulation/Node.h>
#include <sofa/component/mechanicalload/ConstantForceField.h>
#include <sofa/defaulttype/RigidTypes.h>
#include <sofa/core/behavior/MechanicalState.h>

#include <string>
#include <vector>
#include <array>

// Internal ligament config (C++ side)
struct LigamentState {
    std::string name;
    std::array<double, 3> tibia_offset;
    std::array<double, 3> talus_offset;
    std::array<double, 3> fixed_anchor;
    bool use_fixed_anchor;
    double stiffness;   // N/mm
    double damping;     // N·s/mm
    double rest_length; // mm
};

// Scene state (singleton, managed by bridge)
struct AnkleSceneState {
    bool is_active = false;
    int step_count = 0;

    // SOFA typed pointers for runtime data access
    sofa::core::behavior::MechanicalState<sofa::defaulttype::Rigid3Types>* tibia_mo = nullptr;
    sofa::core::behavior::MechanicalState<sofa::defaulttype::Rigid3Types>* talus_mo = nullptr;
    sofa::component::mechanicalload::ConstantForceField<sofa::defaulttype::Rigid3Types>* ligament_ff = nullptr;
    sofa::component::mechanicalload::ConstantForceField<sofa::defaulttype::Rigid3Types>* torque_ff = nullptr;

    std::vector<LigamentState> ligaments;

    // Last computed ligament forces (for readback)
    std::array<double, 3> last_force = {0, 0, 0};
    std::array<double, 3> last_torque = {0, 0, 0};
};

// Box mesh generation — returns SOFA-format position and triangle strings
void box_mesh_strings(const double half[3],
                      std::string& positions_out,
                      std::string& triangles_out);

// Build the ankle scene graph under the given root node
// Returns 0 on success, sets error string on failure
int build_ankle_scene(sofa::simulation::NodeSPtr root,
                      AnkleSceneState& scene,
                      float dt, float gravity_z,
                      const SofaLigamentConfig* ligaments,
                      int num_ligaments);

// Apply ligament forces — called from sofa_step BEFORE animate
void apply_ligament_forces(AnkleSceneState& scene);

// Compute joint angles from relative quaternion
void compute_joint_angles(const AnkleSceneState& scene, double angles_deg[3]);

// Fill a frame snapshot from current scene state
int fill_frame_snapshot(const AnkleSceneState& scene, SofaFrameSnapshot* out);

// Default ligament configs (matches Python DEFAULT_LIGAMENTS)
extern const SofaLigamentConfig DEFAULT_ANKLE_LIGAMENTS[4];
extern const int DEFAULT_ANKLE_LIGAMENT_COUNT;

#endif // ANKLE_SCENE_H
