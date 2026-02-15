#include <gtest/gtest.h>
#include "sofa_ankle_bridge.h"
#include <cmath>
#include <cstdlib>
#include <chrono>
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

class AnkleSceneTest : public ::testing::Test {
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
// Scene Creation (3 tests)
// ---------------------------------------------------------------------------

// 1. Create ankle scene with defaults succeeds
TEST_F(AnkleSceneTest, CreateAnkleScene_Succeeds) {
    int rc = sofa_scene_create_ankle(0.001f, -9810.0f);
    EXPECT_EQ(rc, 0) << "Error: " << sofa_bridge_get_error();
}

// 2. Initial snapshot readable — talus at (0,0,-30), tibia at origin
TEST_F(AnkleSceneTest, CreateAnkleScene_SnapshotReadable) {
    ASSERT_EQ(sofa_scene_create_ankle(0.001f, -9810.0f), 0)
        << sofa_bridge_get_error();

    SofaFrameSnapshot snap = {};
    ASSERT_EQ(sofa_get_frame_snapshot(&snap), 0) << sofa_bridge_get_error();

    // Tibia at origin
    EXPECT_NEAR(snap.tibia.px, 0.0, 1e-3);
    EXPECT_NEAR(snap.tibia.py, 0.0, 1e-3);
    EXPECT_NEAR(snap.tibia.pz, 0.0, 1e-3);

    // Talus at (0, 0, -30)
    EXPECT_NEAR(snap.talus.px, 0.0, 1e-3);
    EXPECT_NEAR(snap.talus.py, 0.0, 1e-3);
    EXPECT_NEAR(snap.talus.pz, -30.0, 1e-3);

    // Identity quaternions (qw=1)
    EXPECT_NEAR(snap.tibia.qw, 1.0, 1e-3);
    EXPECT_NEAR(snap.talus.qw, 1.0, 1e-3);

    EXPECT_EQ(snap.step_count, 0);
}

// 3. Create with custom ligaments (the _ex variant)
TEST_F(AnkleSceneTest, CreateAnkleSceneEx_CustomLigaments) {
    SofaLigamentConfig custom[4] = {
        {"Custom_ATFL",  {15,10,-14}, {12,8,11}, {0,0,0}, 0, 80.0, 5.0, 0.0},
        {"Custom_PTFL",  {15,-10,-14}, {12,-8,11}, {0,0,0}, 0, 60.0, 5.0, 0.0},
        {"Custom_Dant",  {-12,10,-14}, {-10,8,11}, {0,0,0}, 0, 100.0, 5.0, 0.0},
        {"Custom_Dpost", {-12,-10,-14}, {-10,-8,11}, {0,0,0}, 0, 100.0, 5.0, 0.0},
    };
    int rc = sofa_scene_create_ankle_ex(0.001f, -9810.0f, custom, 4);
    EXPECT_EQ(rc, 0) << "Error: " << sofa_bridge_get_error();
}

// ---------------------------------------------------------------------------
// Physics (3 tests)
// ---------------------------------------------------------------------------

// 4. Tibia (fixed) does not move after 100 steps
TEST_F(AnkleSceneTest, TibiaFixed_DoesNotMove) {
    ASSERT_EQ(sofa_scene_create_ankle(0.001f, -9810.0f), 0)
        << sofa_bridge_get_error();

    for (int i = 0; i < 100; i++) {
        ASSERT_EQ(sofa_step(0.001f), 0) << "Step " << i << ": " << sofa_bridge_get_error();
    }

    SofaFrameSnapshot snap = {};
    ASSERT_EQ(sofa_get_frame_snapshot(&snap), 0);

    EXPECT_NEAR(snap.tibia.px, 0.0, 1e-6);
    EXPECT_NEAR(snap.tibia.py, 0.0, 1e-6);
    EXPECT_NEAR(snap.tibia.pz, 0.0, 1e-6);
    EXPECT_NEAR(snap.tibia.qw, 1.0, 1e-6);
}

// 5. No NaN after 1000 steps with gravity
TEST_F(AnkleSceneTest, Stability_1000Steps_NoNaN) {
    ASSERT_EQ(sofa_scene_create_ankle(0.001f, -9810.0f), 0)
        << sofa_bridge_get_error();

    for (int i = 0; i < 1000; i++) {
        ASSERT_EQ(sofa_step(0.001f), 0) << "Step " << i << ": " << sofa_bridge_get_error();
    }

    SofaFrameSnapshot snap = {};
    ASSERT_EQ(sofa_get_frame_snapshot(&snap), 0);

    EXPECT_FALSE(std::isnan(snap.talus.px));
    EXPECT_FALSE(std::isnan(snap.talus.py));
    EXPECT_FALSE(std::isnan(snap.talus.pz));
    EXPECT_FALSE(std::isnan(snap.talus.qx));
    EXPECT_FALSE(std::isnan(snap.talus.qy));
    EXPECT_FALSE(std::isnan(snap.talus.qz));
    EXPECT_FALSE(std::isnan(snap.talus.qw));
}

// 6. Quaternion stays normalized after 500 steps
TEST_F(AnkleSceneTest, QuaternionNormalized_After500Steps) {
    ASSERT_EQ(sofa_scene_create_ankle(0.001f, -9810.0f), 0)
        << sofa_bridge_get_error();

    for (int i = 0; i < 500; i++) {
        ASSERT_EQ(sofa_step(0.001f), 0) << "Step " << i;
    }

    SofaFrameSnapshot snap = {};
    ASSERT_EQ(sofa_get_frame_snapshot(&snap), 0);

    double norm = std::sqrt(
        snap.talus.qx * snap.talus.qx +
        snap.talus.qy * snap.talus.qy +
        snap.talus.qz * snap.talus.qz +
        snap.talus.qw * snap.talus.qw);
    EXPECT_NEAR(norm, 1.0, 0.01);
}

// ---------------------------------------------------------------------------
// Torque & Angles (3 tests)
// ---------------------------------------------------------------------------

// 7. Applying torque produces rotation (zero-g)
TEST_F(AnkleSceneTest, ApplyTorque_ProducesRotation) {
    ASSERT_EQ(sofa_scene_create_ankle(0.001f, 0.0f), 0)
        << sofa_bridge_get_error();

    // Apply 0.005 N·m (= 5 N·mm) sagittal torque
    ASSERT_EQ(sofa_apply_torque(0.005f, 0), 0) << sofa_bridge_get_error();

    for (int i = 0; i < 1000; i++) {
        ASSERT_EQ(sofa_step(0.001f), 0) << "Step " << i;
    }

    SofaFrameSnapshot snap = {};
    ASSERT_EQ(sofa_get_frame_snapshot(&snap), 0);

    EXPECT_GT(std::abs(snap.joint_angles_deg[0]), 1.0)
        << "Sagittal angle should be > 1 degree after 1000 steps with torque";
}

// 8. Opposite torques produce opposite angles
TEST_F(AnkleSceneTest, OppositeTorques_OppositeAngles) {
    // Positive torque (0.005 N·m = 5 N·mm)
    ASSERT_EQ(sofa_scene_create_ankle(0.001f, 0.0f), 0);
    ASSERT_EQ(sofa_apply_torque(0.005f, 0), 0);
    for (int i = 0; i < 1000; i++) {
        ASSERT_EQ(sofa_step(0.001f), 0);
    }
    SofaFrameSnapshot snap_pos = {};
    ASSERT_EQ(sofa_get_frame_snapshot(&snap_pos), 0);

    sofa_bridge_shutdown();

    // Negative torque (-0.005 N·m = -5 N·mm)
    ASSERT_EQ(sofa_bridge_init(get_plugin_dir()), 0);
    ASSERT_EQ(sofa_scene_create_ankle(0.001f, 0.0f), 0);
    ASSERT_EQ(sofa_apply_torque(-0.005f, 0), 0);
    for (int i = 0; i < 1000; i++) {
        ASSERT_EQ(sofa_step(0.001f), 0);
    }
    SofaFrameSnapshot snap_neg = {};
    ASSERT_EQ(sofa_get_frame_snapshot(&snap_neg), 0);

    // Both angles should be non-zero
    EXPECT_GT(std::abs(snap_pos.joint_angles_deg[0]), 1.0) << "Positive torque should produce angle";
    EXPECT_GT(std::abs(snap_neg.joint_angles_deg[0]), 1.0) << "Negative torque should produce angle";
    // Opposite torques → opposite signs
    EXPECT_LT(snap_pos.joint_angles_deg[0] * snap_neg.joint_angles_deg[0], 0.0)
        << "Angles should have opposite signs: pos=" << snap_pos.joint_angles_deg[0]
        << " neg=" << snap_neg.joint_angles_deg[0];
}

// 9. Initial joint angle near zero (before stepping)
TEST_F(AnkleSceneTest, InitialJointAngle_NearZero) {
    ASSERT_EQ(sofa_scene_create_ankle(0.001f, 0.0f), 0)
        << sofa_bridge_get_error();

    SofaFrameSnapshot snap = {};
    ASSERT_EQ(sofa_get_frame_snapshot(&snap), 0);

    EXPECT_NEAR(snap.joint_angles_deg[0], 0.0, 5.0);
    EXPECT_NEAR(snap.joint_angles_deg[1], 0.0, 5.0);
    EXPECT_NEAR(snap.joint_angles_deg[2], 0.0, 5.0);
}

// ---------------------------------------------------------------------------
// Ligament Restraint (2 tests)
// ---------------------------------------------------------------------------

// 10. Ligaments resist motion (zero-g, talus stays near initial position)
TEST_F(AnkleSceneTest, LigamentsResistMotion) {
    ASSERT_EQ(sofa_scene_create_ankle(0.001f, 0.0f), 0)
        << sofa_bridge_get_error();

    for (int i = 0; i < 200; i++) {
        ASSERT_EQ(sofa_step(0.001f), 0);
    }

    SofaFrameSnapshot snap = {};
    ASSERT_EQ(sofa_get_frame_snapshot(&snap), 0);

    double dx = snap.talus.px;
    double dy = snap.talus.py;
    double dz = snap.talus.pz - (-30.0); // initial z = -30
    double displacement = std::sqrt(dx*dx + dy*dy + dz*dz);

    EXPECT_LT(displacement, 5.0)
        << "Talus should not drift > 5mm from initial position in zero-g with ligaments";
}

// 11. ROM arc in plausible range (10-120 degrees)
TEST_F(AnkleSceneTest, RomArc_InPlausibleRange) {
    // Positive torque sweep (0.005 N·m = 5 N·mm)
    ASSERT_EQ(sofa_scene_create_ankle(0.001f, 0.0f), 0);
    ASSERT_EQ(sofa_apply_torque(0.005f, 0), 0);
    for (int i = 0; i < 2000; i++) {
        ASSERT_EQ(sofa_step(0.001f), 0);
    }
    SofaFrameSnapshot snap_pos = {};
    ASSERT_EQ(sofa_get_frame_snapshot(&snap_pos), 0);

    sofa_bridge_shutdown();

    // Negative torque sweep (-0.005 N·m = -5 N·mm)
    ASSERT_EQ(sofa_bridge_init(get_plugin_dir()), 0);
    ASSERT_EQ(sofa_scene_create_ankle(0.001f, 0.0f), 0);
    ASSERT_EQ(sofa_apply_torque(-0.005f, 0), 0);
    for (int i = 0; i < 2000; i++) {
        ASSERT_EQ(sofa_step(0.001f), 0);
    }
    SofaFrameSnapshot snap_neg = {};
    ASSERT_EQ(sofa_get_frame_snapshot(&snap_neg), 0);

    double total_rom = std::abs(snap_pos.joint_angles_deg[0]) +
                       std::abs(snap_neg.joint_angles_deg[0]);

    std::cout << "[   INFO   ] Positive angle: " << snap_pos.joint_angles_deg[0]
              << " deg, Negative angle: " << snap_neg.joint_angles_deg[0]
              << " deg, Total ROM: " << total_rom << " deg" << std::endl;

    EXPECT_GT(total_rom, 10.0) << "Total ROM should be > 10 degrees";
    EXPECT_LT(total_rom, 120.0) << "Total ROM should be < 120 degrees";
}

// ---------------------------------------------------------------------------
// Performance (1 test)
// ---------------------------------------------------------------------------

// 12. Step timing measurement (informational, not a hard fail)
TEST_F(AnkleSceneTest, StepTiming_Under20ms) {
    ASSERT_EQ(sofa_scene_create_ankle(0.001f, -9810.0f), 0)
        << sofa_bridge_get_error();

    const int num_steps = 100;
    auto start = std::chrono::high_resolution_clock::now();

    for (int i = 0; i < num_steps; i++) {
        ASSERT_EQ(sofa_step(0.001f), 0);
    }

    auto end = std::chrono::high_resolution_clock::now();
    double total_ms = std::chrono::duration<double, std::milli>(end - start).count();
    double avg_ms = total_ms / num_steps;

    std::cout << "[   INFO   ] Average step time: " << avg_ms << " ms ("
              << num_steps << " steps in " << total_ms << " ms)" << std::endl;

    // Soft check — print warning but don't fail
    if (avg_ms > 20.0) {
        std::cout << "[  WARNING ] Step time exceeds 20ms target" << std::endl;
    }
}

// ---------------------------------------------------------------------------
// Lifecycle (2 tests)
// ---------------------------------------------------------------------------

// 13. Full lifecycle: create, step, shutdown, reinit, create, step
TEST_F(AnkleSceneTest, Lifecycle_CreateStepDestroyRecreate) {
    // First cycle
    ASSERT_EQ(sofa_scene_create_ankle(0.001f, -9810.0f), 0)
        << sofa_bridge_get_error();
    for (int i = 0; i < 100; i++) {
        ASSERT_EQ(sofa_step(0.001f), 0) << "Cycle 1 step " << i;
    }

    SofaFrameSnapshot snap1 = {};
    ASSERT_EQ(sofa_get_frame_snapshot(&snap1), 0);
    EXPECT_EQ(snap1.step_count, 100);

    sofa_bridge_shutdown();

    // Second cycle
    ASSERT_EQ(sofa_bridge_init(get_plugin_dir()), 0)
        << "Reinit failed: " << sofa_bridge_get_error();
    ASSERT_EQ(sofa_scene_create_ankle(0.001f, -9810.0f), 0)
        << sofa_bridge_get_error();
    for (int i = 0; i < 100; i++) {
        ASSERT_EQ(sofa_step(0.001f), 0) << "Cycle 2 step " << i;
    }

    SofaFrameSnapshot snap2 = {};
    ASSERT_EQ(sofa_get_frame_snapshot(&snap2), 0);
    EXPECT_EQ(snap2.step_count, 100);
}

// 14. Snapshot and torque fail when no scene is active
TEST_F(AnkleSceneTest, NoScene_FunctionsReturnError) {
    // No scene created — just init
    SofaFrameSnapshot snap = {};
    EXPECT_NE(sofa_get_frame_snapshot(&snap), 0);
    EXPECT_NE(sofa_apply_torque(0.001f, 0), 0);
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
