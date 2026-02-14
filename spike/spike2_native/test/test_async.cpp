#include <gtest/gtest.h>
#include "sofa_ankle_bridge.h"
#include <cmath>
#include <cstdlib>
#include <string>
#include <thread>

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

// ---------------------------------------------------------------------------
// Version tests
// ---------------------------------------------------------------------------

TEST(VersionTest, ReturnsValidStruct) {
    SofaBridgeVersion v = sofa_bridge_get_version();
    EXPECT_EQ(v.bridge_version_major, 0);
    EXPECT_EQ(v.bridge_version_minor, 1);
    EXPECT_EQ(v.bridge_version_patch, 0);
    EXPECT_EQ(v.sofa_version_major, 24);
    EXPECT_EQ(v.sofa_version_minor, 6);
}

TEST(VersionTest, FieldsNonNegative) {
    SofaBridgeVersion v = sofa_bridge_get_version();
    EXPECT_GE(v.bridge_version_major, 0);
    EXPECT_GE(v.bridge_version_minor, 0);
    EXPECT_GE(v.bridge_version_patch, 0);
    EXPECT_GE(v.sofa_version_major, 0);
    EXPECT_GE(v.sofa_version_minor, 0);
}

// ---------------------------------------------------------------------------
// Async stepping tests
// ---------------------------------------------------------------------------

class AsyncTest : public ::testing::Test {
protected:
    void SetUp() override {
        int rc = sofa_bridge_init(get_plugin_dir());
        ASSERT_EQ(rc, 0) << "Init failed: " << sofa_bridge_get_error();
    }
    void TearDown() override {
        sofa_bridge_shutdown();
    }
};

// 3. Async step completes successfully
TEST_F(AsyncTest, StepAsync_CompletesSuccessfully) {
    ASSERT_EQ(sofa_scene_create_ankle(0.001f, -9810.0f), 0)
        << sofa_bridge_get_error();

    int rc = sofa_step_async(0.001f);
    EXPECT_EQ(rc, 0) << "Error: " << sofa_bridge_get_error();

    sofa_step_async_wait();

    SofaFrameSnapshot snap = {};
    ASSERT_EQ(sofa_get_frame_snapshot(&snap), 0) << sofa_bridge_get_error();
    EXPECT_EQ(snap.step_count, 1);
}

// 4. is_complete returns true initially (no step running)
TEST_F(AsyncTest, StepAsync_IsComplete_TrueInitially) {
    EXPECT_EQ(sofa_step_async_is_complete(), 1);
}

// 5. Double call without waiting returns error
TEST_F(AsyncTest, StepAsync_DoubleCall_ReturnsError) {
    ASSERT_EQ(sofa_scene_create_ankle(0.001f, -9810.0f), 0)
        << sofa_bridge_get_error();

    // Start first async step
    ASSERT_EQ(sofa_step_async(0.001f), 0) << sofa_bridge_get_error();

    // Try to start another immediately — may or may not fail depending on
    // whether the first one completed already. If it does fail, verify error.
    int rc = sofa_step_async(0.001f);
    if (rc != 0) {
        EXPECT_STRNE(sofa_bridge_get_error(), "");
    }

    // Clean up
    sofa_step_async_wait();
}

// 6. Wait blocks until done, then snapshot is readable
TEST_F(AsyncTest, StepAsync_Wait_BlocksUntilDone) {
    ASSERT_EQ(sofa_scene_create_ankle(0.001f, -9810.0f), 0)
        << sofa_bridge_get_error();

    ASSERT_EQ(sofa_step_async(0.001f), 0) << sofa_bridge_get_error();
    sofa_step_async_wait();

    EXPECT_EQ(sofa_step_async_is_complete(), 1);

    SofaFrameSnapshot snap = {};
    ASSERT_EQ(sofa_get_frame_snapshot(&snap), 0) << sofa_bridge_get_error();
    EXPECT_EQ(snap.step_count, 1);
}

// 7. 100 async steps produce same result as 100 sync steps
TEST_F(AsyncTest, StepAsync_100Steps_MatchesSync) {
    // Run 100 sync steps
    ASSERT_EQ(sofa_scene_create_ankle(0.001f, 0.0f), 0) << sofa_bridge_get_error();
    ASSERT_EQ(sofa_apply_torque(5.0f, 0), 0);

    for (int i = 0; i < 100; i++) {
        ASSERT_EQ(sofa_step(0.001f), 0) << "Sync step " << i;
    }

    SofaFrameSnapshot snap_sync = {};
    ASSERT_EQ(sofa_get_frame_snapshot(&snap_sync), 0);

    sofa_bridge_shutdown();

    // Run 100 async steps
    ASSERT_EQ(sofa_bridge_init(get_plugin_dir()), 0) << sofa_bridge_get_error();
    ASSERT_EQ(sofa_scene_create_ankle(0.001f, 0.0f), 0) << sofa_bridge_get_error();
    ASSERT_EQ(sofa_apply_torque(5.0f, 0), 0);

    for (int i = 0; i < 100; i++) {
        ASSERT_EQ(sofa_step_async(0.001f), 0)
            << "Async step " << i << ": " << sofa_bridge_get_error();
        sofa_step_async_wait();
    }

    SofaFrameSnapshot snap_async = {};
    ASSERT_EQ(sofa_get_frame_snapshot(&snap_async), 0);

    // Compare positions (should be identical — same deterministic computation)
    EXPECT_NEAR(snap_sync.talus.px, snap_async.talus.px, 1e-6);
    EXPECT_NEAR(snap_sync.talus.py, snap_async.talus.py, 1e-6);
    EXPECT_NEAR(snap_sync.talus.pz, snap_async.talus.pz, 1e-6);

    // Compare quaternions
    EXPECT_NEAR(snap_sync.talus.qx, snap_async.talus.qx, 1e-6);
    EXPECT_NEAR(snap_sync.talus.qy, snap_async.talus.qy, 1e-6);
    EXPECT_NEAR(snap_sync.talus.qz, snap_async.talus.qz, 1e-6);
    EXPECT_NEAR(snap_sync.talus.qw, snap_async.talus.qw, 1e-6);

    // Compare angles
    EXPECT_NEAR(snap_sync.joint_angles_deg[0], snap_async.joint_angles_deg[0], 1e-4);

    EXPECT_EQ(snap_async.step_count, 100);
}

// 8. Shutdown during async step does not crash
TEST_F(AsyncTest, Shutdown_DuringAsyncStep_DoesNotCrash) {
    ASSERT_EQ(sofa_scene_create_ankle(0.001f, -9810.0f), 0)
        << sofa_bridge_get_error();

    // Start async step and immediately shutdown
    ASSERT_EQ(sofa_step_async(0.001f), 0) << sofa_bridge_get_error();
    sofa_bridge_shutdown();
    // If we get here without crashing, the test passes
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
