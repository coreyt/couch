#ifndef SCENE_BUILDER_H
#define SCENE_BUILDER_H

#include "sofa_ankle_bridge.h"

#include <sofa/simulation/Node.h>
#include <sofa/component/mechanicalload/ConstantForceField.h>
#include <sofa/defaulttype/RigidTypes.h>
#include <sofa/defaulttype/VecTypes.h>
#include <sofa/core/behavior/MechanicalState.h>

#include <string>
#include <vector>
#include <array>
#include <map>

// Internal bone runtime state
struct BoneState {
    std::string name;
    bool is_fixed;
    sofa::simulation::NodeSPtr node;
    sofa::core::behavior::MechanicalState<sofa::defaulttype::Rigid3Types>* mo = nullptr;
    sofa::component::mechanicalload::ConstantForceField<sofa::defaulttype::Rigid3Types>* ligament_ff = nullptr;
    sofa::component::mechanicalload::ConstantForceField<sofa::defaulttype::Rigid3Types>* torque_ff = nullptr;
    std::array<double, 3> initial_position = {0, 0, 0};
};

// Internal ligament runtime state
struct LigamentState2 {
    std::string name;
    std::string bone_a_name;
    std::string bone_b_name;
    std::array<double, 3> bone_a_offset;
    std::array<double, 3> bone_b_offset;
    std::array<double, 3> fixed_anchor;
    bool use_fixed_anchor;
    double stiffness;       // N/mm (linear_stiffness)
    double damping;         // N·s/mm
    double rest_length;     // mm
    double toe_stiffness;   // N/mm, 0 = use stiffness
    double toe_region_strain; // fraction, 0 = pure linear
};

// Forward declarations for SOFA topology types (resolved at finalize)
namespace sofa::component::topology::container::dynamic {
    class TetrahedronSetTopologyContainer;
    class TetrahedronSetTopologyModifier;
}

// Internal deformable tissue runtime state
struct DeformableState {
    std::string name;
    std::string parent_bone;
    sofa::simulation::NodeSPtr node;
    sofa::simulation::NodeSPtr surface_node;
    sofa::component::topology::container::dynamic::TetrahedronSetTopologyContainer* topo = nullptr;
    sofa::component::topology::container::dynamic::TetrahedronSetTopologyModifier* modifier = nullptr;
    sofa::core::behavior::MechanicalState<sofa::defaulttype::Vec3Types>* mo = nullptr;
    bool topology_dirty = false;
    int last_removed_count = 0;
};

enum class SceneBuilderState {
    Empty,
    Created,
    Finalized
};

class SceneBuilder {
public:
    SceneBuilder() = default;

    /// Create the root scene with physics pipeline.
    /// @return 0 on success, non-zero on error.
    int createScene(sofa::simulation::NodeSPtr root, const SofaSceneConfig& config);

    /// Add a rigid bone to the scene. Must be called after createScene, before finalize.
    /// @return 0 on success, non-zero on error.
    int addRigidBone(const SofaRigidBoneConfig& config);

    /// Add a ligament between two bones. Must be called after createScene, before finalize.
    /// @return 0 on success, non-zero on error.
    int addLigament(const SofaLigamentConfig& config);

    /// Add deformable tissue with tetrahedral FEM. Must be called after createScene, before finalize.
    /// @return 0 on success, non-zero on error.
    int addDeformableTissue(const SofaDeformableConfig& config);

    /// Execute a resection: remove tetrahedra whose centroid is below the cut plane.
    /// @return 0 on success, non-zero on error.
    int executeResection(const SofaResectionCommand& cmd);

    /// Extract the current surface mesh from the deformable tissue.
    /// @return 0 on success, non-zero on error.
    int getSurfaceMesh(SofaSurfaceMesh* out) const;

    /// Check if topology has changed since last query.
    bool hasTopologyChanged() const;

    /// Get the number of tetrahedra removed in the last resection.
    int removedElementCount() const;

    /// Finalize the scene: initRoot, resolve typed pointers.
    /// @return 0 on success, non-zero on error.
    int finalize();

    /// Apply ligament forces — called before each animate step.
    void applyLigamentForces();

    /// Apply external torque to a bone.
    /// @param bone_name Name of the bone.
    /// @param torque_nm Torque in N·m.
    /// @param axis 0=X, 1=Y, 2=Z.
    /// @return 0 on success, non-zero on error.
    int applyTorque(const char* bone_name, float torque_nm, int axis);

    /// Fill a snapshot from the current SOFA state.
    /// @return 0 on success, non-zero on error.
    int fillSnapshot(SofaFrameSnapshot* out) const;

    /// Reset all state for reuse.
    void reset();

    SceneBuilderState state() const { return _state; }
    int stepCount() const { return _step_count; }
    void incrementStepCount() { _step_count++; }

    const std::string& lastError() const { return _last_error; }

    // Last computed forces (for readback)
    const std::array<double, 3>& lastForce() const { return _last_force; }
    const std::array<double, 3>& lastTorque() const { return _last_torque; }

    // Find a bone by name
    BoneState* findBone(const std::string& name);
    const BoneState* findBone(const std::string& name) const;

    // Get the first non-fixed bone (for torque application via legacy API)
    BoneState* firstFreeBone();

private:
    SceneBuilderState _state = SceneBuilderState::Empty;
    sofa::simulation::NodeSPtr _root;

    std::vector<BoneState> _bones;
    std::vector<LigamentState2> _ligaments;
    std::vector<DeformableState> _deformables;

    int _step_count = 0;
    std::array<double, 3> _last_force = {0, 0, 0};
    std::array<double, 3> _last_torque = {0, 0, 0};
    std::string _last_error;

    void setError(const std::string& msg) { _last_error = msg; }

    // Fill a single bone's rigid frame from SOFA state
    void fillBoneFrame(const char* name, SofaRigidFrame* frame) const;

    // Joint angle computation from first two bones
    void computeJointAngles(double angles_deg[3]) const;
};

#endif // SCENE_BUILDER_H
