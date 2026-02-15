#include <gtest/gtest.h>
#include "sofa_ankle_bridge.h"
#include <cmath>
#include <cstdlib>
#include <string>

static const char* get_plugin_dir() {
    const char* dir = std::getenv("SOFA_PLUGIN_DIR");
    if (dir) return dir;
    dir = std::getenv("SOFA_ROOT");
    if (dir) {
        static std::string path;
        path = std::string(dir) + "/lib";
        return path.c_str();
    }
    return nullptr;
}

// Zero-gravity scene config for angle tests
static SofaSceneConfig zero_g_config() {
    SofaSceneConfig cfg = {};
    cfg.gravity[0] = 0.0f;
    cfg.gravity[1] = 0.0f;
    cfg.gravity[2] = 0.0f;
    cfg.timestep = 0.001f;
    cfg.constraint_iterations = 200;
    cfg.constraint_tolerance = 1e-4f;
    cfg.rayleigh_stiffness = 0.1f;
    cfg.rayleigh_mass = 1.0f;
    cfg.alarm_distance = 8.0f;
    cfg.contact_distance = 3.0f;
    cfg.friction_coefficient = 0.0f;
    return cfg;
}

class JointAngleTest : public ::testing::Test {
protected:
    void SetUp() override {
        int rc = sofa_bridge_init(get_plugin_dir());
        ASSERT_EQ(rc, 0) << "Init failed: " << sofa_bridge_get_error();
    }
    void TearDown() override {
        sofa_bridge_shutdown();
    }
};

// ---------------------------------------------------------------------------
// 1. Both bones at identity → all angles ≈ 0
// ---------------------------------------------------------------------------
TEST_F(JointAngleTest, RelativeOrientation_IdentityBodies_ReturnsZero) {
    auto cfg = zero_g_config();
    ASSERT_EQ(sofa_scene_create(&cfg), 0) << sofa_bridge_get_error();

    SofaRigidBoneConfig tibia = {};
    tibia.name = "Tibia";
    tibia.position[0] = 0; tibia.position[1] = 0; tibia.position[2] = 0;
    tibia.orientation[0] = 0; tibia.orientation[1] = 0; tibia.orientation[2] = 0; tibia.orientation[3] = 1;
    tibia.mass = 1.0f;
    tibia.is_fixed = 1;
    ASSERT_EQ(sofa_add_rigid_bone(&tibia), 0) << sofa_bridge_get_error();

    SofaRigidBoneConfig talus = {};
    talus.name = "Talus";
    talus.position[0] = 0; talus.position[1] = 0; talus.position[2] = -30;
    talus.orientation[0] = 0; talus.orientation[1] = 0; talus.orientation[2] = 0; talus.orientation[3] = 1;
    talus.mass = 0.1f;
    talus.is_fixed = 0;
    ASSERT_EQ(sofa_add_rigid_bone(&talus), 0) << sofa_bridge_get_error();

    ASSERT_EQ(sofa_scene_finalize(), 0) << sofa_bridge_get_error();

    SofaFrameSnapshot snap = {};
    ASSERT_EQ(sofa_get_frame_snapshot(&snap), 0) << sofa_bridge_get_error();

    EXPECT_NEAR(snap.joint_angles_deg[0], 0.0, 1.0) << "Sagittal should be ~0";
    EXPECT_NEAR(snap.joint_angles_deg[1], 0.0, 1.0) << "Frontal should be ~0";
    EXPECT_NEAR(snap.joint_angles_deg[2], 0.0, 1.0) << "Transverse should be ~0";
}

// ---------------------------------------------------------------------------
// 2. Talus at 20° X-rotation via initial orientation → joint_angles_deg[0] ≈ 20
// ---------------------------------------------------------------------------
TEST_F(JointAngleTest, RelativeOrientation_KnownRotation_ReturnsCorrectEuler) {
    auto cfg = zero_g_config();
    ASSERT_EQ(sofa_scene_create(&cfg), 0) << sofa_bridge_get_error();

    SofaRigidBoneConfig tibia = {};
    tibia.name = "Tibia";
    tibia.position[0] = 0; tibia.position[1] = 0; tibia.position[2] = 0;
    tibia.orientation[0] = 0; tibia.orientation[1] = 0; tibia.orientation[2] = 0; tibia.orientation[3] = 1;
    tibia.mass = 1.0f;
    tibia.is_fixed = 1;
    ASSERT_EQ(sofa_add_rigid_bone(&tibia), 0) << sofa_bridge_get_error();

    // 20° rotation about X: qx = sin(10°), qw = cos(10°)
    const double angle_rad = 20.0 * M_PI / 180.0;
    SofaRigidBoneConfig talus = {};
    talus.name = "Talus";
    talus.position[0] = 0; talus.position[1] = 0; talus.position[2] = -30;
    talus.orientation[0] = (float)std::sin(angle_rad / 2.0);
    talus.orientation[1] = 0;
    talus.orientation[2] = 0;
    talus.orientation[3] = (float)std::cos(angle_rad / 2.0);
    talus.mass = 0.1f;
    talus.is_fixed = 0;
    ASSERT_EQ(sofa_add_rigid_bone(&talus), 0) << sofa_bridge_get_error();

    ASSERT_EQ(sofa_scene_finalize(), 0) << sofa_bridge_get_error();

    SofaFrameSnapshot snap = {};
    ASSERT_EQ(sofa_get_frame_snapshot(&snap), 0) << sofa_bridge_get_error();

    EXPECT_NEAR(snap.joint_angles_deg[0], 20.0, 1.0)
        << "Sagittal angle should be ~20° for a 20° X-rotation";
    EXPECT_NEAR(snap.joint_angles_deg[1], 0.0, 1.0)
        << "Frontal angle should be ~0";
    EXPECT_NEAR(snap.joint_angles_deg[2], 0.0, 1.0)
        << "Transverse angle should be ~0";
}

// ---------------------------------------------------------------------------
// 3. +torque → positive angle, −torque → negative angle
// ---------------------------------------------------------------------------
TEST_F(JointAngleTest, DorsiflexionAxis_MatchesAnatomicalConvention) {
    // Positive torque
    ASSERT_EQ(sofa_scene_create_ankle(0.001f, 0.0f), 0) << sofa_bridge_get_error();
    ASSERT_EQ(sofa_apply_torque(0.005f, 0), 0) << sofa_bridge_get_error();
    for (int i = 0; i < 500; i++) {
        ASSERT_EQ(sofa_step(0.001f), 0) << "Step " << i << ": " << sofa_bridge_get_error();
    }
    SofaFrameSnapshot snap_pos = {};
    ASSERT_EQ(sofa_get_frame_snapshot(&snap_pos), 0);

    // Reset for negative torque
    sofa_bridge_shutdown();
    ASSERT_EQ(sofa_bridge_init(get_plugin_dir()), 0);

    ASSERT_EQ(sofa_scene_create_ankle(0.001f, 0.0f), 0) << sofa_bridge_get_error();
    ASSERT_EQ(sofa_apply_torque(-0.005f, 0), 0) << sofa_bridge_get_error();
    for (int i = 0; i < 500; i++) {
        ASSERT_EQ(sofa_step(0.001f), 0) << "Step " << i << ": " << sofa_bridge_get_error();
    }
    SofaFrameSnapshot snap_neg = {};
    ASSERT_EQ(sofa_get_frame_snapshot(&snap_neg), 0);

    EXPECT_GT(snap_pos.joint_angles_deg[0], 0.0)
        << "Positive torque should produce positive (DF) angle";
    EXPECT_LT(snap_neg.joint_angles_deg[0], 0.0)
        << "Negative torque should produce negative (PF) angle";
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
