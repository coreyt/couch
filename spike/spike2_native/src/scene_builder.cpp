#include "scene_builder.h"
#include "sofa_compat.h"

#include <sofa/simpleapi/SimpleApi.h>
#include <sofa/simulation/Node.h>
#include <sofa/defaulttype/RigidTypes.h>
#include <sofa/defaulttype/VecTypes.h>
#include <sofa/component/topology/container/dynamic/TetrahedronSetTopologyContainer.h>
#include <sofa/component/topology/container/dynamic/TetrahedronSetTopologyModifier.h>
#include <sofa/component/topology/container/dynamic/TriangleSetTopologyContainer.h>

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
// Scene creation
// ---------------------------------------------------------------------------

int SceneBuilder::createScene(sofa::simulation::NodeSPtr root, const SofaSceneConfig& config) {
    if (_state != SceneBuilderState::Empty) {
        setError("Scene already created — call reset() first");
        return 1;
    }

    namespace sa = sofa::simpleapi;

    _root = root;

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

    root->setGravity(sofa::type::Vec3(config.gravity[0], config.gravity[1], config.gravity[2]));
    root->setDt(config.timestep);

    // Animation loop + constraint solver
    sa::createObject(root, "FreeMotionAnimationLoop", {});

    std::ostringstream iters_ss, tol_ss;
    iters_ss << config.constraint_iterations;
    tol_ss << config.constraint_tolerance;

    sa::createObject(root, "GenericConstraintSolver", {
        {"maxIterations", iters_ss.str()},
        {"tolerance", tol_ss.str()},
    });

    // Collision pipeline
    std::ostringstream alarm_ss, contact_ss;
    alarm_ss << config.alarm_distance;
    contact_ss << config.contact_distance;

    sa::createObject(root, "CollisionPipeline", {});
    sa::createObject(root, "BruteForceBroadPhase", {});
    sa::createObject(root, "BVHNarrowPhase", {});
    sa::createObject(root, "LocalMinDistance", {
        {"alarmDistance", alarm_ss.str()},
        {"contactDistance", contact_ss.str()},
        {"angleCone", "0.1"},
    });
    sa::createObject(root, "CollisionResponse", {
        {"response", "FrictionContactConstraint"},
    });

    _state = SceneBuilderState::Created;
    return 0;
}

// ---------------------------------------------------------------------------
// Rigid bone addition
// ---------------------------------------------------------------------------

int SceneBuilder::addRigidBone(const SofaRigidBoneConfig& config) {
    if (_state == SceneBuilderState::Empty) {
        setError("No scene created — call createScene() first");
        return 1;
    }
    if (_state == SceneBuilderState::Finalized) {
        setError("Scene already finalized — cannot add bones");
        return 1;
    }

    namespace sa = sofa::simpleapi;

    std::string name = config.name ? config.name : "Bone";

    auto bone_node = sa::createChild(_root, name);

    std::ostringstream rs_ss, rm_ss;
    // Use rayleigh values from root config if we had them, but use defaults
    rs_ss << 0.1;
    rm_ss << 1.0;

    sa::createObject(bone_node, "EulerImplicitSolver", {
        {"rayleighStiffness", rs_ss.str()},
        {"rayleighMass", rm_ss.str()},
    });
    sa::createObject(bone_node, "CGLinearSolver", {
        {"iterations", "25"},
        {"tolerance", "1e-10"},
        {"threshold", "1e-10"},
    });

    // Build position string: "px py pz qx qy qz qw"
    std::ostringstream pos_ss;
    pos_ss << config.position[0] << " " << config.position[1] << " " << config.position[2]
           << " " << config.orientation[0] << " " << config.orientation[1]
           << " " << config.orientation[2] << " " << config.orientation[3];

    std::string mo_name = name + "MO";
    sa::createObject(bone_node, "MechanicalObject", {
        {"template", "Rigid3d"},
        {"name", mo_name},
        {"position", pos_ss.str()},
    });

    std::ostringstream mass_ss;
    mass_ss << config.mass;
    sa::createObject(bone_node, "UniformMass", {{"totalMass", mass_ss.str()}});

    if (config.is_fixed) {
        sa::createObject(bone_node, "FixedConstraint", {{"indices", "0"}});
    }

    sa::createObject(bone_node, "UncoupledConstraintCorrection", {});

    // ConstantForceFields for ligament forces and external torque (only on non-fixed bones)
    if (!config.is_fixed) {
        sa::createObject(bone_node, "ConstantForceField", {
            {"template", "Rigid3d"},
            {"name", "LigamentFF"},
            {"forces", "0 0 0 0 0 0"},
            {"indices", "0"},
        });
        sa::createObject(bone_node, "ConstantForceField", {
            {"template", "Rigid3d"},
            {"name", "TorqueFF"},
            {"forces", "0 0 0 0 0 0"},
            {"indices", "0"},
        });
    }

    // Collision sub-node (only if collision mesh provided)
    if (config.collision_vertices && config.collision_vertex_count > 0 &&
        config.collision_triangles && config.collision_triangle_count > 0) {

        // Build position string from vertex data
        std::ostringstream col_pos_ss;
        for (int i = 0; i < config.collision_vertex_count; i++) {
            if (i > 0) col_pos_ss << " ";
            col_pos_ss << config.collision_vertices[i * 3]
                       << " " << config.collision_vertices[i * 3 + 1]
                       << " " << config.collision_vertices[i * 3 + 2];
        }

        // Build triangle string
        std::ostringstream col_tri_ss;
        for (int i = 0; i < config.collision_triangle_count; i++) {
            if (i > 0) col_tri_ss << " ";
            col_tri_ss << config.collision_triangles[i * 3]
                       << " " << config.collision_triangles[i * 3 + 1]
                       << " " << config.collision_triangles[i * 3 + 2];
        }

        bool is_moving = !config.is_fixed;
        std::string group = config.is_fixed ? "0" : "1";

        auto col_node = sa::createChild(bone_node, "Collision");
        sa::createObject(col_node, "MeshTopology", {
            {"position", col_pos_ss.str()},
            {"triangles", col_tri_ss.str()},
        });
        sa::createObject(col_node, "MechanicalObject", {{"template", "Vec3d"}});
        sa::createObject(col_node, "TriangleCollisionModel", {
            {"simulated", "true"}, {"moving", is_moving ? "true" : "false"}, {"group", group},
        });
        sa::createObject(col_node, "LineCollisionModel", {
            {"simulated", "true"}, {"moving", is_moving ? "true" : "false"}, {"group", group},
        });
        sa::createObject(col_node, "PointCollisionModel", {
            {"simulated", "true"}, {"moving", is_moving ? "true" : "false"}, {"group", group},
        });
        sa::createObject(col_node, "RigidMapping", {});
    }

    // Store bone state
    BoneState bs;
    bs.name = name;
    bs.is_fixed = config.is_fixed != 0;
    bs.node = bone_node;
    bs.initial_position = {
        static_cast<double>(config.position[0]),
        static_cast<double>(config.position[1]),
        static_cast<double>(config.position[2])
    };
    _bones.push_back(std::move(bs));

    return 0;
}

// ---------------------------------------------------------------------------
// Ligament addition
// ---------------------------------------------------------------------------

int SceneBuilder::addLigament(const SofaLigamentConfig& config) {
    if (_state == SceneBuilderState::Empty) {
        setError("No scene created — call createScene() first");
        return 1;
    }
    if (_state == SceneBuilderState::Finalized) {
        setError("Scene already finalized — cannot add ligaments");
        return 1;
    }

    // Determine bone names (backward compat: NULL defaults to Tibia/Talus)
    std::string bone_a = (config.bone_a_name && config.bone_a_name[0]) ? config.bone_a_name : "Tibia";
    std::string bone_b = (config.bone_b_name && config.bone_b_name[0]) ? config.bone_b_name : "Talus";

    // Validate bones exist (check both names)
    bool found_a = false, found_b = false;
    for (const auto& b : _bones) {
        if (b.name == bone_a) found_a = true;
        if (b.name == bone_b) found_b = true;
    }

    // For fixed-anchor ligaments, bone_a validation is skipped
    if (!config.use_fixed_anchor && !found_a) {
        setError("Bone '" + bone_a + "' not found");
        return 1;
    }
    if (!found_b) {
        setError("Bone '" + bone_b + "' not found");
        return 1;
    }

    LigamentState2 ls;
    ls.name = config.name ? config.name : "Ligament";
    ls.bone_a_name = bone_a;
    ls.bone_b_name = bone_b;
    for (int j = 0; j < 3; j++) {
        ls.bone_a_offset[j] = config.tibia_offset[j];
        ls.bone_b_offset[j] = config.talus_offset[j];
        ls.fixed_anchor[j] = config.fixed_anchor[j];
    }
    ls.use_fixed_anchor = config.use_fixed_anchor != 0;
    ls.stiffness = config.stiffness;
    ls.damping = config.damping;
    ls.rest_length = config.rest_length;
    ls.toe_stiffness = config.toe_stiffness;
    ls.toe_region_strain = config.toe_region_strain;

    _ligaments.push_back(std::move(ls));
    return 0;
}

// ---------------------------------------------------------------------------
// Finalize
// ---------------------------------------------------------------------------

int SceneBuilder::finalize() {
    if (_state != SceneBuilderState::Created) {
        setError(_state == SceneBuilderState::Finalized
                 ? "Already finalized" : "No scene created");
        return 1;
    }

    // Init the root node
    sofa::simulation::node::initRoot(_root.get());

    // Resolve typed pointers for all bones
    for (auto& bone : _bones) {
        std::string mo_name = bone.name + "MO";
        bone.mo = dynamic_cast<sofa::core::behavior::MechanicalState<Rigid3Types>*>(
            bone.node->getObject(mo_name));
        if (!bone.mo) {
            setError("Failed to resolve MechanicalObject for bone '" + bone.name + "'");
            return 1;
        }

        if (!bone.is_fixed) {
            bone.ligament_ff = dynamic_cast<
                sofa::component::mechanicalload::ConstantForceField<Rigid3Types>*>(
                bone.node->getObject("LigamentFF"));
            bone.torque_ff = dynamic_cast<
                sofa::component::mechanicalload::ConstantForceField<Rigid3Types>*>(
                bone.node->getObject("TorqueFF"));

            if (!bone.ligament_ff || !bone.torque_ff) {
                setError("Failed to resolve ConstantForceField for bone '" + bone.name + "'");
                return 1;
            }
        }
    }

    // Resolve typed pointers for deformable tissues
    for (auto& ds : _deformables) {
        ds.topo = dynamic_cast<
            sofa::component::topology::container::dynamic::TetrahedronSetTopologyContainer*>(
            ds.node->getObject("TetraTopo"));
        ds.modifier = dynamic_cast<
            sofa::component::topology::container::dynamic::TetrahedronSetTopologyModifier*>(
            ds.node->getObject("TetraModifier"));

        std::string mo_name = ds.name + "MO";
        ds.mo = dynamic_cast<sofa::core::behavior::MechanicalState<sofa::defaulttype::Vec3Types>*>(
            ds.node->getObject(mo_name));

        if (!ds.topo) {
            setError("Failed to resolve TetrahedronSetTopologyContainer for '" + ds.name + "'");
            return 1;
        }
        if (!ds.modifier) {
            setError("Failed to resolve TetrahedronSetTopologyModifier for '" + ds.name + "'");
            return 1;
        }
        if (!ds.mo) {
            setError("Failed to resolve MechanicalObject for deformable '" + ds.name + "'");
            return 1;
        }
    }

    // Auto-compute rest lengths for ligaments that have rest_length == 0
    for (auto& lig : _ligaments) {
        if (lig.rest_length <= 0.0) {
            Vec3 proximal;
            if (lig.use_fixed_anchor) {
                proximal = Vec3(lig.fixed_anchor[0], lig.fixed_anchor[1], lig.fixed_anchor[2]);
            } else {
                const BoneState* ba = findBone(lig.bone_a_name);
                if (ba) {
                    Vec3 bone_pos(ba->initial_position[0], ba->initial_position[1], ba->initial_position[2]);
                    proximal = bone_pos + Vec3(lig.bone_a_offset[0], lig.bone_a_offset[1], lig.bone_a_offset[2]);
                }
            }

            const BoneState* bb = findBone(lig.bone_b_name);
            if (bb) {
                Vec3 bone_pos(bb->initial_position[0], bb->initial_position[1], bb->initial_position[2]);
                Vec3 distal = bone_pos + Vec3(lig.bone_b_offset[0], lig.bone_b_offset[1], lig.bone_b_offset[2]);
                lig.rest_length = (proximal - distal).norm();
            }
        }
    }

    _step_count = 0;
    _last_force = {0, 0, 0};
    _last_torque = {0, 0, 0};
    _state = SceneBuilderState::Finalized;
    return 0;
}

// ---------------------------------------------------------------------------
// Ligament force controller (with bilinear model)
// ---------------------------------------------------------------------------

void SceneBuilder::applyLigamentForces() {
    if (_state != SceneBuilderState::Finalized) return;

    // Collect per-bone forces
    struct BoneForces {
        Vec3 force{0, 0, 0};
        Vec3 torque{0, 0, 0};
    };
    std::map<std::string, BoneForces> bone_forces;

    for (const auto& lig : _ligaments) {
        // Get bone A state (proximal)
        Vec3 proximal;
        Vec3 bone_a_pos(0, 0, 0);
        Quat bone_a_quat;
        bone_a_quat.set(0, 0, 0, 1);

        if (lig.use_fixed_anchor) {
            proximal = Vec3(lig.fixed_anchor[0], lig.fixed_anchor[1], lig.fixed_anchor[2]);
        } else {
            const BoneState* ba = findBone(lig.bone_a_name);
            if (!ba || !ba->mo) continue;

            auto ba_pos_data = ba->mo->read(SOFA_CONST_POSITION);
            const RigidCoord& ba_coord = ba_pos_data->getValue()[0];
            bone_a_pos = ba_coord.getCenter();
            bone_a_quat = ba_coord.getOrientation();

            Vec3 offset(lig.bone_a_offset[0], lig.bone_a_offset[1], lig.bone_a_offset[2]);
            proximal = bone_a_pos + bone_a_quat.rotate(offset);
        }

        // Get bone B state (distal)
        const BoneState* bb = findBone(lig.bone_b_name);
        if (!bb || !bb->mo) continue;

        auto bb_pos_data = bb->mo->read(SOFA_CONST_POSITION);
        const RigidCoord& bb_coord = bb_pos_data->getValue()[0];
        Vec3 bone_b_pos = bb_coord.getCenter();
        Quat bone_b_quat = bb_coord.getOrientation();

        auto bb_vel_data = bb->mo->read(SOFA_CONST_VELOCITY);
        const RigidDeriv& bb_deriv = bb_vel_data->getValue()[0];
        Vec3 bone_b_vel = bb_deriv.getVCenter();

        Vec3 distal_offset(lig.bone_b_offset[0], lig.bone_b_offset[1], lig.bone_b_offset[2]);
        Vec3 distal = bone_b_pos + bone_b_quat.rotate(distal_offset);

        // Spring direction and length
        Vec3 diff = proximal - distal;
        double length = diff.norm();
        if (length < 1e-6) continue;
        Vec3 direction = diff / length;

        double extension = length - lig.rest_length;

        // Tension only — ligaments don't resist compression
        if (extension <= 0.0) continue;

        // Bilinear spring force model
        double force_mag;
        if (lig.toe_region_strain > 0.0) {
            double strain = extension / lig.rest_length;
            double toe_stiff = (lig.toe_stiffness > 0.0) ? lig.toe_stiffness : lig.stiffness;
            if (strain < lig.toe_region_strain) {
                // Toe region
                force_mag = toe_stiff * extension;
            } else {
                // Linear region: continuous at transition
                double toe_ext = lig.toe_region_strain * lig.rest_length;
                force_mag = toe_stiff * toe_ext + lig.stiffness * (extension - toe_ext);
            }
        } else {
            // Legacy linear model
            force_mag = lig.stiffness * extension;
        }

        Vec3 spring_force = direction * force_mag;

        // Velocity-based damping along spring axis
        double vel_along = bone_b_vel * direction; // dot product
        Vec3 damping_force = direction * (-lig.damping * vel_along);

        Vec3 force = spring_force + damping_force;

        // Clamp: ligaments are tension-only, never apply compressive force
        double net_along = force * direction;
        if (net_along < 0.0) force = Vec3(0, 0, 0);

        // Accumulate force on bone B
        bone_forces[lig.bone_b_name].force += force;

        // Torque: r x F where r is from bone B center to attachment
        Vec3 r = distal - bone_b_pos;
        bone_forces[lig.bone_b_name].torque += r.cross(force);
    }

    // Write accumulated forces to each bone's LigamentFF
    for (auto& bone : _bones) {
        if (bone.is_fixed || !bone.ligament_ff) continue;

        auto it = bone_forces.find(bone.name);
        if (it == bone_forces.end()) continue;

        auto* d_forces = bone.ligament_ff->findData("forces");
        if (d_forces) {
            std::ostringstream ss;
            ss << it->second.force[0] << " " << it->second.force[1] << " " << it->second.force[2]
               << " " << it->second.torque[0] << " " << it->second.torque[1] << " " << it->second.torque[2];
            d_forces->read(ss.str());
        }
    }

    // Store total force/torque from first free bone for readback (backward compat)
    BoneState* free_bone = firstFreeBone();
    if (free_bone) {
        auto it = bone_forces.find(free_bone->name);
        if (it != bone_forces.end()) {
            _last_force = {it->second.force[0], it->second.force[1], it->second.force[2]};
            _last_torque = {it->second.torque[0], it->second.torque[1], it->second.torque[2]};
        } else {
            _last_force = {0, 0, 0};
            _last_torque = {0, 0, 0};
        }
    }
}

// ---------------------------------------------------------------------------
// Torque application
// ---------------------------------------------------------------------------

int SceneBuilder::applyTorque(const char* bone_name, float torque_nm, int axis) {
    if (_state != SceneBuilderState::Finalized) {
        setError("Scene not finalized");
        return 1;
    }
    if (axis < 0 || axis > 2) {
        setError("Axis must be 0 (X), 1 (Y), or 2 (Z)");
        return 1;
    }

    BoneState* bone = findBone(bone_name);
    if (!bone) {
        setError(std::string("Bone '") + bone_name + "' not found");
        return 1;
    }
    if (!bone->torque_ff) {
        setError(std::string("Bone '") + bone_name + "' has no TorqueFF (is it fixed?)");
        return 1;
    }

    double torque[3] = {0, 0, 0};
    torque[axis] = static_cast<double>(torque_nm) * 1000.0; // N·m → N·mm

    auto* d_forces = bone->torque_ff->findData("forces");
    if (d_forces) {
        std::ostringstream ss;
        ss << "0 0 0 " << torque[0] << " " << torque[1] << " " << torque[2];
        d_forces->read(ss.str());
    }

    return 0;
}

// ---------------------------------------------------------------------------
// Snapshot
// ---------------------------------------------------------------------------

int SceneBuilder::fillSnapshot(SofaFrameSnapshot* out) const {
    if (_state != SceneBuilderState::Finalized || !out) return 1;

    // Name-based lookup — independent of bone insertion order
    fillBoneFrame("Tibia", &out->tibia);
    fillBoneFrame("Talus", &out->talus);
    fillBoneFrame("Calcaneus", &out->calcaneus);

    // Joint angles
    computeJointAngles(out->joint_angles_deg);

    // Ligament forces
    for (int i = 0; i < 3; i++) {
        out->ligament_force[i] = _last_force[i];
        out->ligament_torque[i] = _last_torque[i];
    }

    out->step_count = _step_count;
    out->step_time_ms = 0.0f; // filled by caller
    out->solver_diverged = 0; // checked by caller

    return 0;
}

void SceneBuilder::fillBoneFrame(const char* name, SofaRigidFrame* frame) const {
    const BoneState* b = findBone(name);
    if (!b || !b->mo) {
        *frame = {};
        return;
    }
    auto data = b->mo->read(SOFA_CONST_POSITION);
    const RigidCoord& c = data->getValue()[0];
    frame->px = c.getCenter()[0];
    frame->py = c.getCenter()[1];
    frame->pz = c.getCenter()[2];
    frame->qx = c.getOrientation()[0];
    frame->qy = c.getOrientation()[1];
    frame->qz = c.getOrientation()[2];
    frame->qw = c.getOrientation()[3];
}

// ---------------------------------------------------------------------------
// Joint angle computation
// ---------------------------------------------------------------------------

void SceneBuilder::computeJointAngles(double angles_deg[3]) const {
    angles_deg[0] = angles_deg[1] = angles_deg[2] = 0.0;

    if (_bones.size() < 2) return;

    const BoneState& b0 = _bones[0];
    const BoneState& b1 = _bones[1];
    if (!b0.mo || !b1.mo) return;

    auto b0_data = b0.mo->read(SOFA_CONST_POSITION);
    const RigidCoord& b0_coord = b0_data->getValue()[0];
    Quat b0_quat = b0_coord.getOrientation();

    auto b1_data = b1.mo->read(SOFA_CONST_POSITION);
    const RigidCoord& b1_coord = b1_data->getValue()[0];
    Quat b1_quat = b1_coord.getOrientation();

    // Relative quaternion
    Quat b0_inv = b0_quat.inverse();
    Quat rel = b0_inv * b1_quat;

    double qx = rel[0], qy = rel[1], qz = rel[2], qw = rel[3];

    // Euler decomposition (XYZ intrinsic = ZYX extrinsic)
    double sinr_cosp = 2.0 * (qw * qx + qy * qz);
    double cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy);
    double roll = std::atan2(sinr_cosp, cosr_cosp);

    double sinp = 2.0 * (qw * qy - qz * qx);
    sinp = std::max(-1.0, std::min(1.0, sinp));
    double pitch = std::asin(sinp);

    double siny_cosp = 2.0 * (qw * qz + qx * qy);
    double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
    double yaw = std::atan2(siny_cosp, cosy_cosp);

    const double RAD2DEG = 180.0 / M_PI;
    angles_deg[0] = roll * RAD2DEG;
    angles_deg[1] = pitch * RAD2DEG;
    angles_deg[2] = yaw * RAD2DEG;
}

// ---------------------------------------------------------------------------
// Deformable tissue addition
// ---------------------------------------------------------------------------

int SceneBuilder::addDeformableTissue(const SofaDeformableConfig& config) {
    if (_state == SceneBuilderState::Empty) {
        setError("No scene created — call createScene() first");
        return 1;
    }
    if (_state == SceneBuilderState::Finalized) {
        setError("Scene already finalized — cannot add deformable tissue");
        return 1;
    }
    if (!config.vertices || config.vertex_count <= 0) {
        setError("Deformable tissue requires vertices");
        return 1;
    }
    if (!config.tetrahedra || config.tetra_count <= 0) {
        setError("Deformable tissue requires tetrahedra");
        return 1;
    }

    namespace sa = sofa::simpleapi;

    std::string name = config.name ? config.name : "Deformable";

    auto tissue_node = sa::createChild(_root, name);

    // Required plugins for FEM + topology
    sa::createObject(tissue_node, "RequiredPlugin", {
        {"pluginName", "Sofa.Component.Topology.Container.Dynamic "
         "Sofa.Component.Topology.Mapping "
         "Sofa.Component.SolidMechanics.FEM.Elastic "
         "Sofa.Component.LinearSolver.Direct"}
    });

    // ODE solver + direct linear solver for FEM
    sa::createObject(tissue_node, "EulerImplicitSolver", {
        {"rayleighStiffness", "0.1"},
        {"rayleighMass", "1.0"},
    });
    sa::createObject(tissue_node, "SparseLDLSolver", {});

    // Build position string from vertex data
    std::ostringstream pos_ss;
    for (int i = 0; i < config.vertex_count; i++) {
        if (i > 0) pos_ss << " ";
        pos_ss << config.vertices[i * 3]
               << " " << config.vertices[i * 3 + 1]
               << " " << config.vertices[i * 3 + 2];
    }

    // Build tetrahedra string
    std::ostringstream tet_ss;
    for (int i = 0; i < config.tetra_count; i++) {
        if (i > 0) tet_ss << " ";
        tet_ss << config.tetrahedra[i * 4]
               << " " << config.tetrahedra[i * 4 + 1]
               << " " << config.tetrahedra[i * 4 + 2]
               << " " << config.tetrahedra[i * 4 + 3];
    }

    // Topology container + modifier
    sa::createObject(tissue_node, "TetrahedronSetTopologyContainer", {
        {"name", "TetraTopo"},
        {"position", pos_ss.str()},
        {"tetrahedra", tet_ss.str()},
    });
    sa::createObject(tissue_node, "TetrahedronSetTopologyModifier", {
        {"name", "TetraModifier"},
    });

    // Mechanical object
    sa::createObject(tissue_node, "MechanicalObject", {
        {"template", "Vec3d"},
        {"name", name + "MO"},
    });

    // Mass
    std::ostringstream density_ss;
    density_ss << config.mass_density;
    sa::createObject(tissue_node, "MeshMatrixMass", {
        {"totalMass", density_ss.str()},
    });

    // FEM force field
    std::ostringstream young_ss, poisson_ss;
    young_ss << config.young_modulus;
    poisson_ss << config.poisson_ratio;
    sa::createObject(tissue_node, "TetrahedronFEMForceField", {
        {"template", "Vec3d"},
        {"youngModulus", young_ss.str()},
        {"poissonRatio", poisson_ss.str()},
        {"method", "large"},
    });

    // Surface extraction sub-node for visualization
    auto surface_node = sa::createChild(tissue_node, name + "Surface");
    sa::createObject(surface_node, "TriangleSetTopologyContainer", {
        {"name", "SurfaceTopo"},
    });
    sa::createObject(surface_node, "TriangleSetTopologyModifier", {});
    sa::createObject(surface_node, "Tetra2TriangleTopologicalMapping", {
        {"input", "@../" + std::string("TetraTopo")},
        {"output", "@SurfaceTopo"},
    });

    // Store deformable state
    DeformableState ds;
    ds.name = name;
    ds.parent_bone = config.parent_bone ? config.parent_bone : "";
    ds.node = tissue_node;
    ds.surface_node = surface_node;
    _deformables.push_back(std::move(ds));

    return 0;
}

// ---------------------------------------------------------------------------
// Resection
// ---------------------------------------------------------------------------

int SceneBuilder::executeResection(const SofaResectionCommand& cmd) {
    if (_state != SceneBuilderState::Finalized) {
        setError("Scene not finalized");
        return 1;
    }

    std::string bone_name = cmd.bone_name ? cmd.bone_name : "";

    // Find the deformable tissue (match by name or parent_bone, or use first one)
    DeformableState* target = nullptr;
    for (auto& d : _deformables) {
        if (!bone_name.empty() && (d.name == bone_name || d.parent_bone == bone_name)) {
            target = &d;
            break;
        }
    }
    if (!target && !_deformables.empty()) {
        target = &_deformables[0];
    }
    if (!target) {
        setError("No deformable tissue found");
        return 1;
    }
    if (!target->topo || !target->modifier || !target->mo) {
        setError("Deformable tissue not properly resolved");
        return 1;
    }

    Vec3 plane_point(cmd.plane_point[0], cmd.plane_point[1], cmd.plane_point[2]);
    Vec3 plane_normal(cmd.plane_normal[0], cmd.plane_normal[1], cmd.plane_normal[2]);
    plane_normal.normalize();

    // Read current positions
    auto pos_data = target->mo->read(SOFA_CONST_POSITION);
    const auto& positions = pos_data->getValue();

    // Get tetrahedra
    const auto& tetras = target->topo->getTetrahedronArray();

    // Find tetrahedra whose centroid is below the plane (dot < 0)
    sofa::type::vector<sofa::Index> to_remove;
    for (sofa::Index i = 0; i < tetras.size(); i++) {
        const auto& tet = tetras[i];

        // Compute centroid (average of 4 vertices)
        Vec3 centroid(0, 0, 0);
        for (int v = 0; v < 4; v++) {
            centroid += positions[tet[v]];
        }
        centroid *= 0.25;

        // Remove if centroid is below the plane
        double dot = (centroid - plane_point) * plane_normal;
        if (dot < 0.0) {
            to_remove.push_back(i);
        }
    }

    if (to_remove.empty()) {
        target->last_removed_count = 0;
        return 0;
    }

    // Remove tetrahedra (sorted descending for safe removal)
    std::sort(to_remove.begin(), to_remove.end(), std::greater<sofa::Index>());
    target->modifier->removeTetrahedra(to_remove, true);

    target->last_removed_count = static_cast<int>(to_remove.size());
    target->topology_dirty = true;

    return 0;
}

int SceneBuilder::getSurfaceMesh(SofaSurfaceMesh* out) const {
    if (!out) {
        return 1;
    }
    if (_deformables.empty()) {
        out->vertex_count = 0;
        out->triangle_count = 0;
        return 0;
    }

    const DeformableState& ds = _deformables[0];
    if (!ds.mo || !ds.surface_node) {
        out->vertex_count = 0;
        out->triangle_count = 0;
        return 0;
    }

    // Get positions from mechanical object
    auto pos_data = ds.mo->read(SOFA_CONST_POSITION);
    const auto& positions = pos_data->getValue();

    // Get surface triangles from surface topology
    auto* surface_topo = ds.surface_node->getObject("SurfaceTopo");
    auto* tri_container = dynamic_cast<
        sofa::component::topology::container::dynamic::TriangleSetTopologyContainer*>(surface_topo);

    if (!tri_container) {
        out->vertex_count = 0;
        out->triangle_count = 0;
        return 0;
    }

    const auto& triangles = tri_container->getTriangleArray();

    // Fill output (capped by caller-provided capacity)
    int vert_cap = out->vertex_count;
    int tri_cap = out->triangle_count;

    int actual_verts = static_cast<int>(positions.size());
    int actual_tris = static_cast<int>(triangles.size());

    out->vertex_count = std::min(actual_verts, vert_cap);
    out->triangle_count = std::min(actual_tris, tri_cap);

    if (out->vertices) {
        for (int i = 0; i < out->vertex_count; i++) {
            out->vertices[i * 3 + 0] = static_cast<float>(positions[i][0]);
            out->vertices[i * 3 + 1] = static_cast<float>(positions[i][1]);
            out->vertices[i * 3 + 2] = static_cast<float>(positions[i][2]);
        }
    }

    if (out->triangles) {
        for (int i = 0; i < out->triangle_count; i++) {
            out->triangles[i * 3 + 0] = static_cast<int>(triangles[i][0]);
            out->triangles[i * 3 + 1] = static_cast<int>(triangles[i][1]);
            out->triangles[i * 3 + 2] = static_cast<int>(triangles[i][2]);
        }
    }

    return 0;
}

bool SceneBuilder::hasTopologyChanged() const {
    for (const auto& d : _deformables) {
        if (d.topology_dirty) return true;
    }
    return false;
}

int SceneBuilder::removedElementCount() const {
    int total = 0;
    for (const auto& d : _deformables) {
        total += d.last_removed_count;
    }
    return total;
}

// ---------------------------------------------------------------------------
// Reset
// ---------------------------------------------------------------------------

void SceneBuilder::reset() {
    _bones.clear();
    _ligaments.clear();
    _deformables.clear();
    _root.reset();
    _step_count = 0;
    _last_force = {0, 0, 0};
    _last_torque = {0, 0, 0};
    _last_error.clear();
    _state = SceneBuilderState::Empty;
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

BoneState* SceneBuilder::findBone(const std::string& name) {
    for (auto& b : _bones) {
        if (b.name == name) return &b;
    }
    return nullptr;
}

const BoneState* SceneBuilder::findBone(const std::string& name) const {
    for (const auto& b : _bones) {
        if (b.name == name) return &b;
    }
    return nullptr;
}

BoneState* SceneBuilder::firstFreeBone() {
    for (auto& b : _bones) {
        if (!b.is_fixed) return &b;
    }
    return nullptr;
}
