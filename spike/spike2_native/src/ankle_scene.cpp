#include "ankle_scene.h"

#include <sofa/simpleapi/SimpleApi.h>
#include <sofa/simulation/Node.h>
#include <sofa/defaulttype/RigidTypes.h>

#include <algorithm>
#include <cmath>
#include <sstream>
#include <string>

using Rigid3Types = sofa::defaulttype::Rigid3Types;
using RigidCoord = Rigid3Types::Coord;
using RigidDeriv = Rigid3Types::Deriv;
using Vec3 = sofa::type::Vec3;
using Quat = sofa::type::Quat<double>;

// ---------------------------------------------------------------------------
// Default ligaments (matches Python DEFAULT_LIGAMENTS)
// ---------------------------------------------------------------------------

const SofaLigamentConfig DEFAULT_ANKLE_LIGAMENTS[4] = {
    {"ATFL",
     {15.0, 10.0, -14.0}, {12.0, 8.0, 11.0}, {0, 0, 0}, 0,
     70.0, 5.0, 0.0,
     nullptr, nullptr, 0.0, 0.0},
    {"PTFL",
     {15.0, -10.0, -14.0}, {12.0, -8.0, 11.0}, {0, 0, 0}, 0,
     50.0, 5.0, 0.0,
     nullptr, nullptr, 0.0, 0.0},
    {"Deltoid_ant",
     {-12.0, 10.0, -14.0}, {-10.0, 8.0, 11.0}, {0, 0, 0}, 0,
     90.0, 5.0, 0.0,
     nullptr, nullptr, 0.0, 0.0},
    {"Deltoid_post",
     {-12.0, -10.0, -14.0}, {-10.0, -8.0, 11.0}, {0, 0, 0}, 0,
     90.0, 5.0, 0.0,
     nullptr, nullptr, 0.0, 0.0},
};
const int DEFAULT_ANKLE_LIGAMENT_COUNT = 4;

// ---------------------------------------------------------------------------
// Box mesh generation
// ---------------------------------------------------------------------------

void box_mesh_strings(const double half[3],
                      std::string& positions_out,
                      std::string& triangles_out) {
    double hx = half[0], hy = half[1], hz = half[2];

    // 8 vertices of axis-aligned box
    double verts[8][3] = {
        {-hx, -hy, -hz}, { hx, -hy, -hz}, { hx,  hy, -hz}, {-hx,  hy, -hz},
        {-hx, -hy,  hz}, { hx, -hy,  hz}, { hx,  hy,  hz}, {-hx,  hy,  hz},
    };

    // 12 triangles (same winding as Python)
    int tris[12][3] = {
        {0,1,2}, {0,2,3},   // bottom
        {4,6,5}, {4,7,6},   // top
        {0,4,5}, {0,5,1},   // front
        {2,6,7}, {2,7,3},   // back
        {0,3,7}, {0,7,4},   // left
        {1,5,6}, {1,6,2},   // right
    };

    std::ostringstream pos_ss;
    for (int i = 0; i < 8; i++) {
        if (i > 0) pos_ss << " ";
        pos_ss << verts[i][0] << " " << verts[i][1] << " " << verts[i][2];
    }
    positions_out = pos_ss.str();

    std::ostringstream tri_ss;
    for (int i = 0; i < 12; i++) {
        if (i > 0) tri_ss << " ";
        tri_ss << tris[i][0] << " " << tris[i][1] << " " << tris[i][2];
    }
    triangles_out = tri_ss.str();
}

// ---------------------------------------------------------------------------
// Scene construction
// ---------------------------------------------------------------------------

int build_ankle_scene(sofa::simulation::NodeSPtr root,
                      AnkleSceneState& scene,
                      float dt, float gravity_z,
                      const SofaLigamentConfig* ligaments,
                      int num_ligaments) {
    namespace sa = sofa::simpleapi;

    // Plugins
    sa::createObject(root, "RequiredPlugin", {
        {"pluginName", "Sofa.Component.StateContainer Sofa.Component.Mass "
         "Sofa.Component.ODESolver.Backward Sofa.Component.LinearSolver.Iterative "
         "Sofa.Component.Constraint.Lagrangian.Solver Sofa.Component.Constraint.Lagrangian.Correction "
         "Sofa.Component.Constraint.Projective "
         "Sofa.Component.Collision.Detection.Algorithm Sofa.Component.Collision.Detection.Intersection "
         "Sofa.Component.Collision.Geometry Sofa.Component.Collision.Response.Contact "
         "Sofa.Component.AnimationLoop Sofa.Component.Mapping.Linear "
         "Sofa.Component.Topology.Container.Constant Sofa.Component.MechanicalLoad"}
    });

    root->setGravity(sofa::type::Vec3(0, 0, gravity_z));
    root->setDt(dt);

    // Animation loop + constraint solver
    sa::createObject(root, "FreeMotionAnimationLoop", {});
    sa::createObject(root, "GenericConstraintSolver", {
        {"maxIterations", "200"},
        {"tolerance", "1e-4"},
    });

    // Collision pipeline
    sa::createObject(root, "CollisionPipeline", {});
    sa::createObject(root, "BruteForceBroadPhase", {});
    sa::createObject(root, "BVHNarrowPhase", {});
    sa::createObject(root, "LocalMinDistance", {
        {"alarmDistance", "8"},
        {"contactDistance", "3"},
        {"angleCone", "0.1"},
    });
    sa::createObject(root, "CollisionResponse", {
        {"response", "FrictionContactConstraint"},
    });

    // ---- Tibia (fixed rigid body) ----
    auto tibia = sa::createChild(root, "Tibia");
    sa::createObject(tibia, "EulerImplicitSolver", {
        {"rayleighStiffness", "0.1"},
        {"rayleighMass", "1.0"},
    });
    sa::createObject(tibia, "CGLinearSolver", {
        {"iterations", "25"},
        {"tolerance", "1e-10"},
        {"threshold", "1e-10"},
    });
    sa::createObject(tibia, "MechanicalObject", {
        {"template", "Rigid3d"},
        {"name", "TibiaMO"},
        {"position", "0 0 0 0 0 0 1"},
    });
    sa::createObject(tibia, "UniformMass", {{"totalMass", "1.0"}});
    sa::createObject(tibia, "FixedConstraint", {{"indices", "0"}});
    sa::createObject(tibia, "UncoupledConstraintCorrection", {});

    // Tibia collision sub-node
    double tibia_half[3] = {20.0, 15.0, 15.0};
    std::string tibia_pos_str, tibia_tri_str;
    box_mesh_strings(tibia_half, tibia_pos_str, tibia_tri_str);

    auto tibia_col = sa::createChild(tibia, "Collision");
    sa::createObject(tibia_col, "MeshTopology", {
        {"position", tibia_pos_str},
        {"triangles", tibia_tri_str},
    });
    sa::createObject(tibia_col, "MechanicalObject", {{"template", "Vec3d"}});
    sa::createObject(tibia_col, "TriangleCollisionModel", {
        {"simulated", "true"}, {"moving", "false"}, {"group", "0"},
    });
    sa::createObject(tibia_col, "LineCollisionModel", {
        {"simulated", "true"}, {"moving", "false"}, {"group", "0"},
    });
    sa::createObject(tibia_col, "PointCollisionModel", {
        {"simulated", "true"}, {"moving", "false"}, {"group", "0"},
    });
    sa::createObject(tibia_col, "RigidMapping", {});

    // ---- Talus (free rigid body) ----
    auto talus = sa::createChild(root, "Talus");
    sa::createObject(talus, "EulerImplicitSolver", {
        {"rayleighStiffness", "0.1"},
        {"rayleighMass", "1.0"},
    });
    sa::createObject(talus, "CGLinearSolver", {
        {"iterations", "25"},
        {"tolerance", "1e-10"},
        {"threshold", "1e-10"},
    });
    sa::createObject(talus, "MechanicalObject", {
        {"template", "Rigid3d"},
        {"name", "TalusMO"},
        {"position", "0 0 -30 0 0 0 1"},
    });
    sa::createObject(talus, "UniformMass", {{"totalMass", "0.1"}});
    sa::createObject(talus, "UncoupledConstraintCorrection", {});

    // ConstantForceFields for ligament forces and external torque
    sa::createObject(talus, "ConstantForceField", {
        {"template", "Rigid3d"},
        {"name", "LigamentFF"},
        {"forces", "0 0 0 0 0 0"},
        {"indices", "0"},
    });
    sa::createObject(talus, "ConstantForceField", {
        {"template", "Rigid3d"},
        {"name", "TorqueFF"},
        {"forces", "0 0 0 0 0 0"},
        {"indices", "0"},
    });

    // Talus collision sub-node
    double talus_half[3] = {18.0, 13.0, 12.0};
    std::string talus_pos_str, talus_tri_str;
    box_mesh_strings(talus_half, talus_pos_str, talus_tri_str);

    auto talus_col = sa::createChild(talus, "Collision");
    sa::createObject(talus_col, "MeshTopology", {
        {"position", talus_pos_str},
        {"triangles", talus_tri_str},
    });
    sa::createObject(talus_col, "MechanicalObject", {{"template", "Vec3d"}});
    sa::createObject(talus_col, "TriangleCollisionModel", {
        {"simulated", "true"}, {"moving", "true"}, {"group", "1"},
    });
    sa::createObject(talus_col, "LineCollisionModel", {
        {"simulated", "true"}, {"moving", "true"}, {"group", "1"},
    });
    sa::createObject(talus_col, "PointCollisionModel", {
        {"simulated", "true"}, {"moving", "true"}, {"group", "1"},
    });
    sa::createObject(talus_col, "RigidMapping", {});

    // ---- Get typed pointers for runtime access ----
    auto* tibia_node = tibia.get();
    auto* talus_node = talus.get();

    scene.tibia_mo = dynamic_cast<sofa::core::behavior::MechanicalState<Rigid3Types>*>(
        tibia_node->getObject("TibiaMO"));
    scene.talus_mo = dynamic_cast<sofa::core::behavior::MechanicalState<Rigid3Types>*>(
        talus_node->getObject("TalusMO"));
    scene.ligament_ff = dynamic_cast<sofa::component::mechanicalload::ConstantForceField<Rigid3Types>*>(
        talus_node->getObject("LigamentFF"));
    scene.torque_ff = dynamic_cast<sofa::component::mechanicalload::ConstantForceField<Rigid3Types>*>(
        talus_node->getObject("TorqueFF"));

    if (!scene.tibia_mo || !scene.talus_mo || !scene.ligament_ff || !scene.torque_ff) {
        return 1; // failed to resolve typed pointers
    }

    // ---- Build ligament state ----
    Vec3 tibia_pos(0, 0, 0);
    Vec3 talus_pos(0, 0, -30);

    scene.ligaments.clear();
    for (int i = 0; i < num_ligaments; i++) {
        LigamentState ls;
        ls.name = ligaments[i].name;
        for (int j = 0; j < 3; j++) {
            ls.tibia_offset[j] = ligaments[i].tibia_offset[j];
            ls.talus_offset[j] = ligaments[i].talus_offset[j];
            ls.fixed_anchor[j] = ligaments[i].fixed_anchor[j];
        }
        ls.use_fixed_anchor = ligaments[i].use_fixed_anchor != 0;
        ls.stiffness = ligaments[i].stiffness;
        ls.damping = ligaments[i].damping;

        // Auto-compute rest length from initial geometry if 0
        if (ligaments[i].rest_length <= 0.0) {
            Vec3 proximal;
            if (ls.use_fixed_anchor) {
                proximal = Vec3(ls.fixed_anchor[0], ls.fixed_anchor[1], ls.fixed_anchor[2]);
            } else {
                proximal = tibia_pos + Vec3(ls.tibia_offset[0], ls.tibia_offset[1], ls.tibia_offset[2]);
            }
            Vec3 distal = talus_pos + Vec3(ls.talus_offset[0], ls.talus_offset[1], ls.talus_offset[2]);
            ls.rest_length = (proximal - distal).norm();
        } else {
            ls.rest_length = ligaments[i].rest_length;
        }

        scene.ligaments.push_back(ls);
    }

    scene.step_count = 0;
    scene.last_force = {0, 0, 0};
    scene.last_torque = {0, 0, 0};
    scene.is_active = true;

    return 0;
}

// ---------------------------------------------------------------------------
// Ligament force controller
// ---------------------------------------------------------------------------

void apply_ligament_forces(AnkleSceneState& scene) {
    if (!scene.is_active) return;

    // Read tibia state
    auto tibia_pos_data = scene.tibia_mo->read(sofa::core::ConstVecCoordId::position());
    const RigidCoord& tibia_coord = tibia_pos_data->getValue()[0];
    Vec3 tibia_pos = tibia_coord.getCenter();
    Quat tibia_quat = tibia_coord.getOrientation();

    // Read talus state
    auto talus_pos_data = scene.talus_mo->read(sofa::core::ConstVecCoordId::position());
    const RigidCoord& talus_coord = talus_pos_data->getValue()[0];
    Vec3 talus_pos = talus_coord.getCenter();
    Quat talus_quat = talus_coord.getOrientation();

    // Read talus velocity
    auto talus_vel_data = scene.talus_mo->read(sofa::core::ConstVecDerivId::velocity());
    const RigidDeriv& talus_deriv = talus_vel_data->getValue()[0];
    Vec3 talus_vel = talus_deriv.getVCenter();

    Vec3 total_force(0, 0, 0);
    Vec3 total_torque(0, 0, 0);

    for (const auto& lig : scene.ligaments) {
        // Proximal attachment point
        Vec3 proximal;
        if (lig.use_fixed_anchor) {
            proximal = Vec3(lig.fixed_anchor[0], lig.fixed_anchor[1], lig.fixed_anchor[2]);
        } else {
            Vec3 offset(lig.tibia_offset[0], lig.tibia_offset[1], lig.tibia_offset[2]);
            proximal = tibia_pos + tibia_quat.rotate(offset);
        }

        // Distal attachment point
        Vec3 talus_offset(lig.talus_offset[0], lig.talus_offset[1], lig.talus_offset[2]);
        Vec3 distal = talus_pos + talus_quat.rotate(talus_offset);

        // Spring direction and length
        Vec3 diff = proximal - distal;
        double length = diff.norm();
        if (length < 1e-6) continue;
        Vec3 direction = diff / length;

        double extension = length - lig.rest_length;

        // Tension only — ligaments don't resist compression
        if (extension <= 0.0) continue;

        // Linear spring force
        double force_mag = lig.stiffness * extension;
        Vec3 spring_force = direction * force_mag;

        // Velocity-based damping along spring axis
        double vel_along = talus_vel * direction; // dot product
        Vec3 damping_force = direction * (-lig.damping * vel_along);

        Vec3 force = spring_force + damping_force;
        total_force += force;

        // Torque: r x F where r is from talus center to attachment
        Vec3 r = distal - talus_pos;
        total_torque += r.cross(force);
    }

    // Write to LigamentFF ConstantForceField
    auto* d_forces = scene.ligament_ff->findData("forces");
    if (d_forces) {
        std::ostringstream ss;
        ss << total_force[0] << " " << total_force[1] << " " << total_force[2]
           << " " << total_torque[0] << " " << total_torque[1] << " " << total_torque[2];
        d_forces->read(ss.str());
    }

    // Store for readback
    scene.last_force = {total_force[0], total_force[1], total_force[2]};
    scene.last_torque = {total_torque[0], total_torque[1], total_torque[2]};
}

// ---------------------------------------------------------------------------
// Joint angle computation
// ---------------------------------------------------------------------------

void compute_joint_angles(const AnkleSceneState& scene, double angles_deg[3]) {
    auto tibia_data = scene.tibia_mo->read(sofa::core::ConstVecCoordId::position());
    const RigidCoord& tibia_coord = tibia_data->getValue()[0];
    Quat tibia_quat = tibia_coord.getOrientation();

    auto talus_data = scene.talus_mo->read(sofa::core::ConstVecCoordId::position());
    const RigidCoord& talus_coord = talus_data->getValue()[0];
    Quat talus_quat = talus_coord.getOrientation();

    // Relative quaternion: tibia_inv * talus
    Quat tibia_inv = tibia_quat.inverse();
    Quat rel = tibia_inv * talus_quat;

    double qx = rel[0], qy = rel[1], qz = rel[2], qw = rel[3];

    // Euler decomposition (XYZ intrinsic = ZYX extrinsic)
    // Sagittal (X-axis rotation)
    double sinr_cosp = 2.0 * (qw * qx + qy * qz);
    double cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy);
    double roll = std::atan2(sinr_cosp, cosr_cosp);

    // Frontal (Y-axis rotation)
    double sinp = 2.0 * (qw * qy - qz * qx);
    sinp = std::max(-1.0, std::min(1.0, sinp));
    double pitch = std::asin(sinp);

    // Transverse (Z-axis rotation)
    double siny_cosp = 2.0 * (qw * qz + qx * qy);
    double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
    double yaw = std::atan2(siny_cosp, cosy_cosp);

    const double RAD2DEG = 180.0 / M_PI;
    angles_deg[0] = roll * RAD2DEG;
    angles_deg[1] = pitch * RAD2DEG;
    angles_deg[2] = yaw * RAD2DEG;
}

// ---------------------------------------------------------------------------
// Frame snapshot
// ---------------------------------------------------------------------------

int fill_frame_snapshot(const AnkleSceneState& scene, SofaFrameSnapshot* out) {
    if (!scene.is_active || !out) return 1;

    // Tibia
    auto tibia_data = scene.tibia_mo->read(sofa::core::ConstVecCoordId::position());
    const RigidCoord& tc = tibia_data->getValue()[0];
    out->tibia.px = tc.getCenter()[0];
    out->tibia.py = tc.getCenter()[1];
    out->tibia.pz = tc.getCenter()[2];
    out->tibia.qx = tc.getOrientation()[0];
    out->tibia.qy = tc.getOrientation()[1];
    out->tibia.qz = tc.getOrientation()[2];
    out->tibia.qw = tc.getOrientation()[3];

    // Talus
    auto talus_data = scene.talus_mo->read(sofa::core::ConstVecCoordId::position());
    const RigidCoord& ac = talus_data->getValue()[0];
    out->talus.px = ac.getCenter()[0];
    out->talus.py = ac.getCenter()[1];
    out->talus.pz = ac.getCenter()[2];
    out->talus.qx = ac.getOrientation()[0];
    out->talus.qy = ac.getOrientation()[1];
    out->talus.qz = ac.getOrientation()[2];
    out->talus.qw = ac.getOrientation()[3];

    // Joint angles
    compute_joint_angles(scene, out->joint_angles_deg);

    // Ligament forces
    for (int i = 0; i < 3; i++) {
        out->ligament_force[i] = scene.last_force[i];
        out->ligament_torque[i] = scene.last_torque[i];
    }

    out->step_time_ms = 0.0f;
    out->solver_diverged = 0;
    out->step_count = scene.step_count;
    return 0;
}
