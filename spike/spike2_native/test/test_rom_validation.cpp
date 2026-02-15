#include <gtest/gtest.h>
#include "sofa_ankle_bridge.h"
#include <cmath>
#include <cstdlib>
#include <string>
#include <iostream>

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

class ROMValidationTest : public ::testing::Test {
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
// 1. +0.005 N·m (= 5 N·mm) → joint_angles_deg[0] > 5°
// ---------------------------------------------------------------------------
TEST_F(ROMValidationTest, PositiveTorque_ProducesPositiveAngle) {
    ASSERT_EQ(sofa_scene_create_ankle(0.001f, 0.0f), 0) << sofa_bridge_get_error();
    ASSERT_EQ(sofa_apply_torque(0.005f, 0), 0) << sofa_bridge_get_error();

    for (int i = 0; i < 1000; i++) {
        ASSERT_EQ(sofa_step(0.001f), 0) << "Step " << i << ": " << sofa_bridge_get_error();
    }

    SofaFrameSnapshot snap = {};
    ASSERT_EQ(sofa_get_frame_snapshot(&snap), 0);

    std::cout << "[   INFO   ] Positive torque angle: " << snap.joint_angles_deg[0] << " deg" << std::endl;

    EXPECT_GT(snap.joint_angles_deg[0], 5.0)
        << "+5 N·mm torque should produce > 5° dorsiflexion";
}

// ---------------------------------------------------------------------------
// 2. -0.005 N·m (= -5 N·mm) → joint_angles_deg[0] < -5°
// ---------------------------------------------------------------------------
TEST_F(ROMValidationTest, NegativeTorque_ProducesNegativeAngle) {
    ASSERT_EQ(sofa_scene_create_ankle(0.001f, 0.0f), 0) << sofa_bridge_get_error();
    ASSERT_EQ(sofa_apply_torque(-0.005f, 0), 0) << sofa_bridge_get_error();

    for (int i = 0; i < 1000; i++) {
        ASSERT_EQ(sofa_step(0.001f), 0) << "Step " << i << ": " << sofa_bridge_get_error();
    }

    SofaFrameSnapshot snap = {};
    ASSERT_EQ(sofa_get_frame_snapshot(&snap), 0);

    std::cout << "[   INFO   ] Negative torque angle: " << snap.joint_angles_deg[0] << " deg" << std::endl;

    EXPECT_LT(snap.joint_angles_deg[0], -5.0)
        << "-5 N·mm torque should produce < -5° (plantarflexion)";
}

// ---------------------------------------------------------------------------
// 3. DF+PF sweep via SceneBuilder API → 10-120°
// ---------------------------------------------------------------------------
TEST_F(ROMValidationTest, FullSweep_TotalArcInPlausibleRange) {
    // Positive torque sweep
    ASSERT_EQ(sofa_scene_create_ankle(0.001f, 0.0f), 0) << sofa_bridge_get_error();
    ASSERT_EQ(sofa_apply_torque(0.005f, 0), 0) << sofa_bridge_get_error();

    for (int i = 0; i < 1000; i++) {
        ASSERT_EQ(sofa_step(0.001f), 0) << "Step " << i;
    }
    SofaFrameSnapshot snap_pos = {};
    ASSERT_EQ(sofa_get_frame_snapshot(&snap_pos), 0);

    sofa_bridge_shutdown();

    // Negative torque sweep
    ASSERT_EQ(sofa_bridge_init(get_plugin_dir()), 0);
    ASSERT_EQ(sofa_scene_create_ankle(0.001f, 0.0f), 0) << sofa_bridge_get_error();
    ASSERT_EQ(sofa_apply_torque(-0.005f, 0), 0) << sofa_bridge_get_error();

    for (int i = 0; i < 1000; i++) {
        ASSERT_EQ(sofa_step(0.001f), 0) << "Step " << i;
    }
    SofaFrameSnapshot snap_neg = {};
    ASSERT_EQ(sofa_get_frame_snapshot(&snap_neg), 0);

    double total_rom = std::abs(snap_pos.joint_angles_deg[0]) +
                       std::abs(snap_neg.joint_angles_deg[0]);

    std::cout << "[   INFO   ] DF: " << snap_pos.joint_angles_deg[0]
              << " deg, PF: " << snap_neg.joint_angles_deg[0]
              << " deg, Total ROM: " << total_rom << " deg" << std::endl;

    EXPECT_GT(total_rom, 10.0) << "Total ROM should be > 10°";
    EXPECT_LT(total_rom, 120.0) << "Total ROM should be < 120°";
}

// ---------------------------------------------------------------------------
// 4. 2× stiffness ligaments → smaller angle than default
// ---------------------------------------------------------------------------
TEST_F(ROMValidationTest, StifferLigaments_ReduceROM) {
    // Default stiffness sweep
    ASSERT_EQ(sofa_scene_create_ankle(0.001f, 0.0f), 0);
    ASSERT_EQ(sofa_apply_torque(0.005f, 0), 0);

    for (int i = 0; i < 500; i++) {
        ASSERT_EQ(sofa_step(0.001f), 0) << "Default step " << i;
    }
    SofaFrameSnapshot snap_default = {};
    ASSERT_EQ(sofa_get_frame_snapshot(&snap_default), 0);
    double default_angle = std::abs(snap_default.joint_angles_deg[0]);

    sofa_bridge_shutdown();

    // 2× stiffness sweep
    ASSERT_EQ(sofa_bridge_init(get_plugin_dir()), 0);

    SofaLigamentConfig stiff_ligs[4] = {
        {"ATFL",  {15,10,-14}, {12,8,11}, {0,0,0}, 0, 140.0, 5.0, 0.0, nullptr, nullptr, 0.0, 0.0},
        {"PTFL",  {15,-10,-14}, {12,-8,11}, {0,0,0}, 0, 100.0, 5.0, 0.0, nullptr, nullptr, 0.0, 0.0},
        {"Deltoid_ant",  {-12,10,-14}, {-10,8,11}, {0,0,0}, 0, 180.0, 5.0, 0.0, nullptr, nullptr, 0.0, 0.0},
        {"Deltoid_post", {-12,-10,-14}, {-10,-8,11}, {0,0,0}, 0, 180.0, 5.0, 0.0, nullptr, nullptr, 0.0, 0.0},
    };
    ASSERT_EQ(sofa_scene_create_ankle_ex(0.001f, 0.0f, stiff_ligs, 4), 0)
        << sofa_bridge_get_error();
    ASSERT_EQ(sofa_apply_torque(0.005f, 0), 0);

    for (int i = 0; i < 500; i++) {
        ASSERT_EQ(sofa_step(0.001f), 0) << "Stiff step " << i;
    }
    SofaFrameSnapshot snap_stiff = {};
    ASSERT_EQ(sofa_get_frame_snapshot(&snap_stiff), 0);
    double stiff_angle = std::abs(snap_stiff.joint_angles_deg[0]);

    std::cout << "[   INFO   ] Default angle: " << default_angle
              << " deg, 2x stiff angle: " << stiff_angle << " deg" << std::endl;

    EXPECT_LT(stiff_angle, default_angle)
        << "Stiffer ligaments should produce smaller angle: stiff="
        << stiff_angle << " default=" << default_angle;
}

// ---------------------------------------------------------------------------
// 5. Angle delta between step 500→1000 < delta between step 0→500
// ---------------------------------------------------------------------------
TEST_F(ROMValidationTest, SweepAngleConverges) {
    ASSERT_EQ(sofa_scene_create_ankle(0.001f, 0.0f), 0) << sofa_bridge_get_error();
    ASSERT_EQ(sofa_apply_torque(0.005f, 0), 0) << sofa_bridge_get_error();

    // Step to 500
    for (int i = 0; i < 500; i++) {
        ASSERT_EQ(sofa_step(0.001f), 0) << "Step " << i;
    }
    SofaFrameSnapshot snap_500 = {};
    ASSERT_EQ(sofa_get_frame_snapshot(&snap_500), 0);

    // Step to 1000
    for (int i = 500; i < 1000; i++) {
        ASSERT_EQ(sofa_step(0.001f), 0) << "Step " << i;
    }
    SofaFrameSnapshot snap_1000 = {};
    ASSERT_EQ(sofa_get_frame_snapshot(&snap_1000), 0);

    double delta_first_half = std::abs(snap_500.joint_angles_deg[0]);  // from 0
    double delta_second_half = std::abs(snap_1000.joint_angles_deg[0] - snap_500.joint_angles_deg[0]);

    std::cout << "[   INFO   ] Angle at 500: " << snap_500.joint_angles_deg[0]
              << " deg, at 1000: " << snap_1000.joint_angles_deg[0]
              << " deg, delta 0-500: " << delta_first_half
              << ", delta 500-1000: " << delta_second_half << std::endl;

    EXPECT_LT(delta_second_half, delta_first_half)
        << "Angle change should decrease (converge) in the second half of the sweep";
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
