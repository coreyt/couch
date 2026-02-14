#include <gtest/gtest.h>
#include "sofa_ankle_bridge.h"
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

class BridgeTest : public ::testing::Test {
protected:
    void TearDown() override {
        sofa_bridge_shutdown();
    }
};

// 1. Init with valid plugin dir succeeds
TEST_F(BridgeTest, InitWithPluginDir) {
    int rc = sofa_bridge_init(get_plugin_dir());
    EXPECT_EQ(rc, 0) << "Error: " << sofa_bridge_get_error();
}

// 2. Init with NULL plugin dir doesn't crash
TEST_F(BridgeTest, InitWithNullPluginDir) {
    int rc = sofa_bridge_init(nullptr);
    EXPECT_EQ(rc, 0) << "Error: " << sofa_bridge_get_error();
}

// 3. Step empty scene succeeds
TEST_F(BridgeTest, StepEmptyScene) {
    ASSERT_EQ(sofa_bridge_init(get_plugin_dir()), 0);
    int rc = sofa_step(0.01f);
    EXPECT_EQ(rc, 0) << "Error: " << sofa_bridge_get_error();
}

// 4. 100 steps in a loop all succeed
TEST_F(BridgeTest, HundredSteps) {
    ASSERT_EQ(sofa_bridge_init(get_plugin_dir()), 0);
    for (int i = 0; i < 100; i++) {
        int rc = sofa_step(0.001f);
        EXPECT_EQ(rc, 0) << "Step " << i << " failed: " << sofa_bridge_get_error();
    }
}

// 5. Step before init returns error
TEST_F(BridgeTest, StepBeforeInit) {
    int rc = sofa_step(0.01f);
    EXPECT_NE(rc, 0);
    EXPECT_STRNE(sofa_bridge_get_error(), "");
}

// 6. Step with dt <= 0 returns error
TEST_F(BridgeTest, StepNegativeDt) {
    ASSERT_EQ(sofa_bridge_init(get_plugin_dir()), 0);
    EXPECT_NE(sofa_step(0.0f), 0);
    EXPECT_NE(sofa_step(-1.0f), 0);
}

// 7. Shutdown + re-init lifecycle works
TEST_F(BridgeTest, ShutdownAndReinit) {
    ASSERT_EQ(sofa_bridge_init(get_plugin_dir()), 0);
    ASSERT_EQ(sofa_step(0.01f), 0);
    sofa_bridge_shutdown();

    int rc = sofa_bridge_init(get_plugin_dir());
    EXPECT_EQ(rc, 0) << "Re-init failed: " << sofa_bridge_get_error();
    EXPECT_EQ(sofa_step(0.01f), 0);
}

// 8. Double init without shutdown returns error
TEST_F(BridgeTest, DoubleInit) {
    ASSERT_EQ(sofa_bridge_init(get_plugin_dir()), 0);
    int rc = sofa_bridge_init(get_plugin_dir());
    EXPECT_NE(rc, 0);
    EXPECT_STRNE(sofa_bridge_get_error(), "");
}

// 9. Double shutdown doesn't crash (idempotent)
TEST_F(BridgeTest, DoubleShutdown) {
    ASSERT_EQ(sofa_bridge_init(get_plugin_dir()), 0);
    sofa_bridge_shutdown();
    sofa_bridge_shutdown();  // Should not crash
}

// 10. Error message populated on failure
TEST_F(BridgeTest, ErrorMessagePopulated) {
    // Before init, error is empty
    EXPECT_STREQ(sofa_bridge_get_error(), "");
    // Trigger an error
    sofa_step(0.01f);
    const char* err = sofa_bridge_get_error();
    EXPECT_STRNE(err, "");
    EXPECT_NE(std::string(err).find("Not initialized"), std::string::npos);
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
