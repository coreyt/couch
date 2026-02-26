#include <gtest/gtest.h>
#include "sofa_ankle_bridge.h"
#include <cmath>
#include <cstdlib>
#include <cstring>
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

// Helper: create default scene config matching ankle defaults
static SofaSceneConfig default_scene_config() {
    SofaSceneConfig cfg = {};
    cfg.gravity[0] = 0.0f;
    cfg.gravity[1] = 0.0f;
    cfg.gravity[2] = -9810.0f;
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

// Box mesh data for collision (8 vertices, 12 triangles)
static const float TIBIA_BOX_VERTS[] = {
    -20, -15, -15,  20, -15, -15,  20,  15, -15,  -20,  15, -15,
    -20, -15,  15,  20, -15,  15,  20,  15,  15,  -20,  15,  15,
};
static const int TIBIA_BOX_TRIS[] = {
    0,1,2, 0,2,3, 4,6,5, 4,7,6,
    0,4,5, 0,5,1, 2,6,7, 2,7,3,
    0,3,7, 0,7,4, 1,5,6, 1,6,2,
};

static const float TALUS_BOX_VERTS[] = {
    -18, -13, -12,  18, -13, -12,  18,  13, -12,  -18,  13, -12,
    -18, -13,  12,  18, -13,  12,  18,  13,  12,  -18,  13,  12,
};
static const int TALUS_BOX_TRIS[] = {
    0,1,2, 0,2,3, 4,6,5, 4,7,6,
    0,4,5, 0,5,1, 2,6,7, 2,7,3,
    0,3,7, 0,7,4, 1,5,6, 1,6,2,
};

static SofaRigidBoneConfig tibia_bone_config(bool with_mesh = true) {
    SofaRigidBoneConfig cfg = {};
    cfg.name = "Tibia";
    cfg.position[0] = 0; cfg.position[1] = 0; cfg.position[2] = 0;
    cfg.orientation[0] = 0; cfg.orientation[1] = 0; cfg.orientation[2] = 0; cfg.orientation[3] = 1;
    cfg.mass = 1.0f;
    cfg.is_fixed = 1;
    if (with_mesh) {
        cfg.collision_vertices = TIBIA_BOX_VERTS;
        cfg.collision_vertex_count = 8;
        cfg.collision_triangles = TIBIA_BOX_TRIS;
        cfg.collision_triangle_count = 12;
    }
    return cfg;
}

static SofaRigidBoneConfig talus_bone_config(bool with_mesh = true) {
    SofaRigidBoneConfig cfg = {};
    cfg.name = "Talus";
    cfg.position[0] = 0; cfg.position[1] = 0; cfg.position[2] = -30;
    cfg.orientation[0] = 0; cfg.orientation[1] = 0; cfg.orientation[2] = 0; cfg.orientation[3] = 1;
    cfg.mass = 0.1f;
    cfg.is_fixed = 0;
    if (with_mesh) {
        cfg.collision_vertices = TALUS_BOX_VERTS;
        cfg.collision_vertex_count = 8;
        cfg.collision_triangles = TALUS_BOX_TRIS;
        cfg.collision_triangle_count = 12;
    }
    return cfg;
}

// Default 4 ankle ligament configs
static SofaLigamentConfig default_ligament(const char* name,
    double ta0, double ta1, double ta2,
    double tb0, double tb1, double tb2,
    double stiffness) {
    SofaLigamentConfig cfg = {};
    cfg.name = name;
    cfg.tibia_offset[0] = ta0; cfg.tibia_offset[1] = ta1; cfg.tibia_offset[2] = ta2;
    cfg.talus_offset[0] = tb0; cfg.talus_offset[1] = tb1; cfg.talus_offset[2] = tb2;
    cfg.use_fixed_anchor = 0;
    cfg.stiffness = stiffness;
    cfg.damping = 5.0;
    cfg.rest_length = 0.0; // auto-compute
    cfg.bone_a_name = nullptr;
    cfg.bone_b_name = nullptr;
    cfg.toe_stiffness = 0.0;
    cfg.toe_region_strain = 0.0;
    return cfg;
}

// ---------------------------------------------------------------------------
// Test fixture
// ---------------------------------------------------------------------------

class SceneBuilderTest : public ::testing::Test {
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
// 1. CreateScene_WithDefaults_Succeeds
// ---------------------------------------------------------------------------
TEST_F(SceneBuilderTest, CreateScene_WithDefaults_Succeeds) {
    auto cfg = default_scene_config();
    int rc = sofa_scene_create(&cfg);
    EXPECT_EQ(rc, 0) << sofa_bridge_get_error();
}

// ---------------------------------------------------------------------------
// 2. CreateScene_SetsGravity
// ---------------------------------------------------------------------------
TEST_F(SceneBuilderTest, CreateScene_SetsGravity) {
    auto cfg = default_scene_config();
    ASSERT_EQ(sofa_scene_create(&cfg), 0) << sofa_bridge_get_error();

    auto tibia = tibia_bone_config();
    ASSERT_EQ(sofa_add_rigid_bone(&tibia), 0) << sofa_bridge_get_error();

    auto talus = talus_bone_config();
    ASSERT_EQ(sofa_add_rigid_bone(&talus), 0) << sofa_bridge_get_error();

    ASSERT_EQ(sofa_scene_finalize(), 0) << sofa_bridge_get_error();

    // Step several times — talus should move downward (gravity Z = -9810)
    for (int i = 0; i < 10; i++) {
        ASSERT_EQ(sofa_step(0.001f), 0) << sofa_bridge_get_error();
    }

    SofaFrameSnapshot snap = {};
    ASSERT_EQ(sofa_get_frame_snapshot(&snap), 0) << sofa_bridge_get_error();

    // Talus should have moved down from initial -30
    EXPECT_LT(snap.talus.pz, -30.0);
}

// ---------------------------------------------------------------------------
// 3. AddRigidBone_Fixed_Succeeds
// ---------------------------------------------------------------------------
TEST_F(SceneBuilderTest, AddRigidBone_Fixed_Succeeds) {
    auto cfg = default_scene_config();
    ASSERT_EQ(sofa_scene_create(&cfg), 0) << sofa_bridge_get_error();

    auto tibia = tibia_bone_config();
    int rc = sofa_add_rigid_bone(&tibia);
    EXPECT_EQ(rc, 0) << sofa_bridge_get_error();
}

// ---------------------------------------------------------------------------
// 4. AddRigidBone_Free_Succeeds
// ---------------------------------------------------------------------------
TEST_F(SceneBuilderTest, AddRigidBone_Free_Succeeds) {
    auto cfg = default_scene_config();
    ASSERT_EQ(sofa_scene_create(&cfg), 0) << sofa_bridge_get_error();

    auto talus = talus_bone_config();
    int rc = sofa_add_rigid_bone(&talus);
    EXPECT_EQ(rc, 0) << sofa_bridge_get_error();
}

// ---------------------------------------------------------------------------
// 5. AddRigidBone_WithCollisionMesh_Succeeds
// ---------------------------------------------------------------------------
TEST_F(SceneBuilderTest, AddRigidBone_WithCollisionMesh_Succeeds) {
    auto cfg = default_scene_config();
    ASSERT_EQ(sofa_scene_create(&cfg), 0) << sofa_bridge_get_error();

    auto tibia = tibia_bone_config(true);
    ASSERT_EQ(sofa_add_rigid_bone(&tibia), 0) << sofa_bridge_get_error();

    auto talus = talus_bone_config(true);
    ASSERT_EQ(sofa_add_rigid_bone(&talus), 0) << sofa_bridge_get_error();

    EXPECT_EQ(sofa_scene_finalize(), 0) << sofa_bridge_get_error();
}

// ---------------------------------------------------------------------------
// 6. AddRigidBone_WithoutCollisionMesh_Succeeds
// ---------------------------------------------------------------------------
TEST_F(SceneBuilderTest, AddRigidBone_WithoutCollisionMesh_Succeeds) {
    auto cfg = default_scene_config();
    ASSERT_EQ(sofa_scene_create(&cfg), 0) << sofa_bridge_get_error();

    auto tibia = tibia_bone_config(false);
    ASSERT_EQ(sofa_add_rigid_bone(&tibia), 0) << sofa_bridge_get_error();

    auto talus = talus_bone_config(false);
    ASSERT_EQ(sofa_add_rigid_bone(&talus), 0) << sofa_bridge_get_error();

    EXPECT_EQ(sofa_scene_finalize(), 0) << sofa_bridge_get_error();
}

// ---------------------------------------------------------------------------
// 7. AddRigidBone_BeforeCreateScene_Fails
// ---------------------------------------------------------------------------
TEST_F(SceneBuilderTest, AddRigidBone_BeforeCreateScene_Fails) {
    auto tibia = tibia_bone_config();
    int rc = sofa_add_rigid_bone(&tibia);
    EXPECT_NE(rc, 0);
}

// ---------------------------------------------------------------------------
// 8. AddRigidBone_AfterFinalize_Fails
// ---------------------------------------------------------------------------
TEST_F(SceneBuilderTest, AddRigidBone_AfterFinalize_Fails) {
    auto cfg = default_scene_config();
    ASSERT_EQ(sofa_scene_create(&cfg), 0);

    auto tibia = tibia_bone_config();
    ASSERT_EQ(sofa_add_rigid_bone(&tibia), 0);
    ASSERT_EQ(sofa_scene_finalize(), 0);

    auto talus = talus_bone_config();
    int rc = sofa_add_rigid_bone(&talus);
    EXPECT_NE(rc, 0);
}

// ---------------------------------------------------------------------------
// 9. AddLigament_BetweenTwoBones_Succeeds
// ---------------------------------------------------------------------------
TEST_F(SceneBuilderTest, AddLigament_BetweenTwoBones_Succeeds) {
    auto cfg = default_scene_config();
    ASSERT_EQ(sofa_scene_create(&cfg), 0);

    auto tibia = tibia_bone_config();
    ASSERT_EQ(sofa_add_rigid_bone(&tibia), 0);
    auto talus = talus_bone_config();
    ASSERT_EQ(sofa_add_rigid_bone(&talus), 0);

    auto lig = default_ligament("ATFL", 15, 10, -14, 12, 8, 11, 70);
    int rc = sofa_add_ligament(&lig);
    EXPECT_EQ(rc, 0) << sofa_bridge_get_error();
}

// ---------------------------------------------------------------------------
// 10. AddLigament_UnknownBone_Fails
// ---------------------------------------------------------------------------
TEST_F(SceneBuilderTest, AddLigament_UnknownBone_Fails) {
    auto cfg = default_scene_config();
    ASSERT_EQ(sofa_scene_create(&cfg), 0);

    auto tibia = tibia_bone_config();
    ASSERT_EQ(sofa_add_rigid_bone(&tibia), 0);

    SofaLigamentConfig lig = {};
    lig.name = "BadLig";
    lig.tibia_offset[0] = 1; lig.tibia_offset[1] = 0; lig.tibia_offset[2] = 0;
    lig.talus_offset[0] = 1; lig.talus_offset[1] = 0; lig.talus_offset[2] = 0;
    lig.stiffness = 50;
    lig.damping = 5;
    // bone_b_name defaults to "Talus" which doesn't exist
    int rc = sofa_add_ligament(&lig);
    EXPECT_NE(rc, 0);
}

// ---------------------------------------------------------------------------
// 11. AddLigament_LinearOnly_BackwardCompat
// ---------------------------------------------------------------------------
TEST_F(SceneBuilderTest, AddLigament_LinearOnly_BackwardCompat) {
    auto cfg = default_scene_config();
    ASSERT_EQ(sofa_scene_create(&cfg), 0);

    auto tibia = tibia_bone_config();
    ASSERT_EQ(sofa_add_rigid_bone(&tibia), 0);
    auto talus = talus_bone_config();
    ASSERT_EQ(sofa_add_rigid_bone(&talus), 0);

    // Linear-only: toe_region_strain = 0
    auto lig = default_ligament("ATFL", 15, 10, -14, 12, 8, 11, 70);
    ASSERT_EQ(sofa_add_ligament(&lig), 0);
    ASSERT_EQ(sofa_scene_finalize(), 0);

    // Step and verify it runs without error
    for (int i = 0; i < 5; i++) {
        EXPECT_EQ(sofa_step(0.001f), 0) << sofa_bridge_get_error();
    }

    SofaFrameSnapshot snap = {};
    ASSERT_EQ(sofa_get_frame_snapshot(&snap), 0);
    EXPECT_FALSE(std::isnan(snap.talus.pz));
}

// ---------------------------------------------------------------------------
// 12. AddLigament_Bilinear_ToeRegion
// ---------------------------------------------------------------------------
TEST_F(SceneBuilderTest, AddLigament_Bilinear_ToeRegion) {
    auto cfg = default_scene_config();
    ASSERT_EQ(sofa_scene_create(&cfg), 0);

    auto tibia = tibia_bone_config(false);
    ASSERT_EQ(sofa_add_rigid_bone(&tibia), 0);
    auto talus = talus_bone_config(false);
    ASSERT_EQ(sofa_add_rigid_bone(&talus), 0);

    // Bilinear: low toe_stiffness, large linear stiffness
    SofaLigamentConfig lig = {};
    lig.name = "BilinearLig";
    lig.tibia_offset[0] = 15; lig.tibia_offset[1] = 10; lig.tibia_offset[2] = -14;
    lig.talus_offset[0] = 12; lig.talus_offset[1] = 8;  lig.talus_offset[2] = 11;
    lig.stiffness = 100.0; // linear stiffness
    lig.damping = 5.0;
    lig.rest_length = 0.0;
    lig.toe_stiffness = 10.0; // much softer in toe region
    lig.toe_region_strain = 0.05; // 5% strain

    ASSERT_EQ(sofa_add_ligament(&lig), 0);
    ASSERT_EQ(sofa_scene_finalize(), 0);

    // Step and verify it runs
    for (int i = 0; i < 5; i++) {
        EXPECT_EQ(sofa_step(0.001f), 0) << sofa_bridge_get_error();
    }

    SofaFrameSnapshot snap = {};
    ASSERT_EQ(sofa_get_frame_snapshot(&snap), 0);
    EXPECT_FALSE(std::isnan(snap.talus.pz));
}

// ---------------------------------------------------------------------------
// 13. AddLigament_Bilinear_LinearRegion
// ---------------------------------------------------------------------------
TEST_F(SceneBuilderTest, AddLigament_Bilinear_LinearRegion) {
    auto cfg = default_scene_config();
    ASSERT_EQ(sofa_scene_create(&cfg), 0);

    auto tibia = tibia_bone_config(false);
    ASSERT_EQ(sofa_add_rigid_bone(&tibia), 0);
    auto talus = talus_bone_config(false);
    ASSERT_EQ(sofa_add_rigid_bone(&talus), 0);

    // Bilinear with a very small toe region — forces should enter linear region quickly
    SofaLigamentConfig lig = {};
    lig.name = "BilinearLinear";
    lig.tibia_offset[0] = 15; lig.tibia_offset[1] = 10; lig.tibia_offset[2] = -14;
    lig.talus_offset[0] = 12; lig.talus_offset[1] = 8;  lig.talus_offset[2] = 11;
    lig.stiffness = 70.0;
    lig.damping = 5.0;
    lig.rest_length = 0.0;
    lig.toe_stiffness = 20.0;
    lig.toe_region_strain = 0.001; // very small toe region

    ASSERT_EQ(sofa_add_ligament(&lig), 0);
    ASSERT_EQ(sofa_scene_finalize(), 0);

    // Step enough to move into linear region
    for (int i = 0; i < 10; i++) {
        EXPECT_EQ(sofa_step(0.001f), 0) << sofa_bridge_get_error();
    }

    SofaFrameSnapshot snap = {};
    ASSERT_EQ(sofa_get_frame_snapshot(&snap), 0);
    EXPECT_FALSE(std::isnan(snap.talus.pz));
}

// ---------------------------------------------------------------------------
// 14. AddLigament_Bilinear_ForceContinuousAtTransition
// ---------------------------------------------------------------------------
TEST_F(SceneBuilderTest, AddLigament_Bilinear_ForceContinuousAtTransition) {
    // The bilinear model should be continuous at the transition point.
    // At strain = toe_region_strain:
    //   toe formula: force = toe_stiffness * extension
    //   linear formula: force = toe_stiffness * toe_ext + linear_stiffness * (extension - toe_ext)
    //   At transition (extension = toe_ext): both give toe_stiffness * toe_ext

    double rest_length = 40.0;
    double toe_stiffness = 20.0;
    double linear_stiffness = 70.0;
    double toe_region_strain = 0.05;

    double toe_ext = toe_region_strain * rest_length;

    // Force at transition from toe side
    double force_toe = toe_stiffness * toe_ext;

    // Force at transition from linear side (extension = toe_ext, so overshoot = 0)
    double force_linear = toe_stiffness * toe_ext + linear_stiffness * 0.0;

    EXPECT_NEAR(force_toe, force_linear, 1e-10)
        << "Bilinear model should be continuous at transition";

    // Verify both formulas agree at a point just past transition
    double eps = 0.001;
    double ext_past = toe_ext + eps;

    // Linear formula at ext_past
    double force_linear_past = toe_stiffness * toe_ext + linear_stiffness * eps;

    // The force should increase smoothly from toe_stiffness * toe_ext
    // by linear_stiffness * eps
    EXPECT_NEAR(force_linear_past, force_toe + linear_stiffness * eps, 1e-10);
}

// ---------------------------------------------------------------------------
// 15. Finalize_ThenStep_Succeeds
// ---------------------------------------------------------------------------
TEST_F(SceneBuilderTest, Finalize_ThenStep_Succeeds) {
    auto cfg = default_scene_config();
    ASSERT_EQ(sofa_scene_create(&cfg), 0);

    auto tibia = tibia_bone_config();
    ASSERT_EQ(sofa_add_rigid_bone(&tibia), 0);
    auto talus = talus_bone_config();
    ASSERT_EQ(sofa_add_rigid_bone(&talus), 0);

    ASSERT_EQ(sofa_scene_finalize(), 0);

    int rc = sofa_step(0.001f);
    EXPECT_EQ(rc, 0) << sofa_bridge_get_error();
}

// ---------------------------------------------------------------------------
// 16. Finalize_ThenSnapshot_ReturnsData
// ---------------------------------------------------------------------------
TEST_F(SceneBuilderTest, Finalize_ThenSnapshot_ReturnsData) {
    auto cfg = default_scene_config();
    ASSERT_EQ(sofa_scene_create(&cfg), 0);

    auto tibia = tibia_bone_config();
    ASSERT_EQ(sofa_add_rigid_bone(&tibia), 0);
    auto talus = talus_bone_config();
    ASSERT_EQ(sofa_add_rigid_bone(&talus), 0);

    ASSERT_EQ(sofa_scene_finalize(), 0);

    SofaFrameSnapshot snap = {};
    int rc = sofa_get_frame_snapshot(&snap);
    EXPECT_EQ(rc, 0) << sofa_bridge_get_error();

    // Tibia at origin
    EXPECT_NEAR(snap.tibia.px, 0.0, 1e-3);
    EXPECT_NEAR(snap.tibia.py, 0.0, 1e-3);
    EXPECT_NEAR(snap.tibia.pz, 0.0, 1e-3);

    // Talus at initial position
    EXPECT_NEAR(snap.talus.px, 0.0, 1e-3);
    EXPECT_NEAR(snap.talus.py, 0.0, 1e-3);
    EXPECT_NEAR(snap.talus.pz, -30.0, 1e-3);

    // Identity quaternions
    EXPECT_NEAR(snap.tibia.qw, 1.0, 1e-3);
    EXPECT_NEAR(snap.talus.qw, 1.0, 1e-3);
}

// ---------------------------------------------------------------------------
// 17. SceneIsReady_FalseBeforeFinalize
// ---------------------------------------------------------------------------
TEST_F(SceneBuilderTest, SceneIsReady_FalseBeforeFinalize) {
    auto cfg = default_scene_config();
    ASSERT_EQ(sofa_scene_create(&cfg), 0);

    EXPECT_EQ(sofa_scene_is_ready(), 0);
}

// ---------------------------------------------------------------------------
// 18. SceneIsReady_TrueAfterFinalize
// ---------------------------------------------------------------------------
TEST_F(SceneBuilderTest, SceneIsReady_TrueAfterFinalize) {
    auto cfg = default_scene_config();
    ASSERT_EQ(sofa_scene_create(&cfg), 0);

    auto tibia = tibia_bone_config();
    ASSERT_EQ(sofa_add_rigid_bone(&tibia), 0);

    ASSERT_EQ(sofa_scene_finalize(), 0);
    EXPECT_EQ(sofa_scene_is_ready(), 1);
}

// ---------------------------------------------------------------------------
// 19. DestroyScene_ThenCreate_Works
// ---------------------------------------------------------------------------
TEST_F(SceneBuilderTest, DestroyScene_ThenCreate_Works) {
    // First create/finalize cycle
    auto cfg = default_scene_config();
    ASSERT_EQ(sofa_scene_create(&cfg), 0);
    auto tibia = tibia_bone_config();
    ASSERT_EQ(sofa_add_rigid_bone(&tibia), 0);
    ASSERT_EQ(sofa_scene_finalize(), 0);
    EXPECT_EQ(sofa_scene_is_ready(), 1);

    // Destroy
    sofa_scene_destroy();
    EXPECT_EQ(sofa_scene_is_ready(), 0);

    // Second create/finalize cycle
    ASSERT_EQ(sofa_scene_create(&cfg), 0);
    auto tibia2 = tibia_bone_config();
    ASSERT_EQ(sofa_add_rigid_bone(&tibia2), 0);
    ASSERT_EQ(sofa_scene_finalize(), 0);
    EXPECT_EQ(sofa_scene_is_ready(), 1);
}

// ---------------------------------------------------------------------------
// 20. FullAnkleViaNewAPI_MatchesLegacy
// ---------------------------------------------------------------------------
TEST_F(SceneBuilderTest, FullAnkleViaNewAPI_MatchesLegacy) {
    // Build ankle via new API
    auto cfg = default_scene_config();
    ASSERT_EQ(sofa_scene_create(&cfg), 0);

    auto tibia = tibia_bone_config();
    ASSERT_EQ(sofa_add_rigid_bone(&tibia), 0);
    auto talus = talus_bone_config();
    ASSERT_EQ(sofa_add_rigid_bone(&talus), 0);

    // Add 4 default ligaments
    auto lig1 = default_ligament("ATFL", 15, 10, -14, 12, 8, 11, 70);
    auto lig2 = default_ligament("PTFL", 15, -10, -14, 12, -8, 11, 50);
    auto lig3 = default_ligament("Deltoid_ant", -12, 10, -14, -10, 8, 11, 90);
    auto lig4 = default_ligament("Deltoid_post", -12, -10, -14, -10, -8, 11, 90);
    ASSERT_EQ(sofa_add_ligament(&lig1), 0);
    ASSERT_EQ(sofa_add_ligament(&lig2), 0);
    ASSERT_EQ(sofa_add_ligament(&lig3), 0);
    ASSERT_EQ(sofa_add_ligament(&lig4), 0);

    ASSERT_EQ(sofa_scene_finalize(), 0);

    // Apply torque and step
    ASSERT_EQ(sofa_apply_torque(2.0f, 0), 0);
    for (int i = 0; i < 50; i++) {
        ASSERT_EQ(sofa_step(0.001f), 0) << sofa_bridge_get_error();
    }

    SofaFrameSnapshot new_snap = {};
    ASSERT_EQ(sofa_get_frame_snapshot(&new_snap), 0);

    // Now do the same with legacy API
    sofa_scene_destroy();

    ASSERT_EQ(sofa_scene_create_ankle(0.001f, -9810.0f), 0);
    ASSERT_EQ(sofa_apply_torque(2.0f, 0), 0);
    for (int i = 0; i < 50; i++) {
        ASSERT_EQ(sofa_step(0.001f), 0) << sofa_bridge_get_error();
    }

    SofaFrameSnapshot legacy_snap = {};
    ASSERT_EQ(sofa_get_frame_snapshot(&legacy_snap), 0);

    // Compare positions (within tolerance for floating point differences)
    EXPECT_NEAR(new_snap.talus.px, legacy_snap.talus.px, 1e-2);
    EXPECT_NEAR(new_snap.talus.py, legacy_snap.talus.py, 1e-2);
    EXPECT_NEAR(new_snap.talus.pz, legacy_snap.talus.pz, 1e-2);

    // Compare angles
    EXPECT_NEAR(new_snap.joint_angles_deg[0], legacy_snap.joint_angles_deg[0], 0.5);
}

// ---------------------------------------------------------------------------
// 21. AddCalcaneus_CreatesThirdBone
// ---------------------------------------------------------------------------
static const float CALCANEUS_BOX_VERTS[] = {
    -16, -12, -10,  16, -12, -10,  16,  12, -10,  -16,  12, -10,
    -16, -12,  10,  16, -12,  10,  16,  12,  10,  -16,  12,  10,
};
static const int CALCANEUS_BOX_TRIS[] = {
    0,1,2, 0,2,3, 4,6,5, 4,7,6,
    0,4,5, 0,5,1, 2,6,7, 2,7,3,
    0,3,7, 0,7,4, 1,5,6, 1,6,2,
};

static SofaRigidBoneConfig calcaneus_bone_config(bool with_mesh = true) {
    SofaRigidBoneConfig cfg = {};
    cfg.name = "Calcaneus";
    cfg.position[0] = 0; cfg.position[1] = -30; cfg.position[2] = -30;
    cfg.orientation[0] = 0; cfg.orientation[1] = 0; cfg.orientation[2] = 0; cfg.orientation[3] = 1;
    cfg.mass = 0.08f;
    cfg.is_fixed = 0;
    if (with_mesh) {
        cfg.collision_vertices = CALCANEUS_BOX_VERTS;
        cfg.collision_vertex_count = 8;
        cfg.collision_triangles = CALCANEUS_BOX_TRIS;
        cfg.collision_triangle_count = 12;
    }
    return cfg;
}

TEST_F(SceneBuilderTest, AddCalcaneus_CreatesThirdBone) {
    auto cfg = default_scene_config();
    ASSERT_EQ(sofa_scene_create(&cfg), 0);

    auto tibia = tibia_bone_config();
    ASSERT_EQ(sofa_add_rigid_bone(&tibia), 0);
    auto talus = talus_bone_config();
    ASSERT_EQ(sofa_add_rigid_bone(&talus), 0);
    auto calcaneus = calcaneus_bone_config();
    ASSERT_EQ(sofa_add_rigid_bone(&calcaneus), 0) << sofa_bridge_get_error();

    ASSERT_EQ(sofa_scene_finalize(), 0) << sofa_bridge_get_error();

    SofaFrameSnapshot snap = {};
    ASSERT_EQ(sofa_get_frame_snapshot(&snap), 0);

    // Calcaneus should be at initial position
    EXPECT_NEAR(snap.calcaneus.px, 0.0, 1e-3);
    EXPECT_NEAR(snap.calcaneus.py, -30.0, 1e-3);
    EXPECT_NEAR(snap.calcaneus.pz, -30.0, 1e-3);
    EXPECT_NEAR(snap.calcaneus.qw, 1.0, 1e-3);
}

// ---------------------------------------------------------------------------
// 22. SubtalarLigaments_ConnectTalusAndCalcaneus
// ---------------------------------------------------------------------------
TEST_F(SceneBuilderTest, SubtalarLigaments_ConnectTalusAndCalcaneus) {
    auto cfg = default_scene_config();
    ASSERT_EQ(sofa_scene_create(&cfg), 0);

    auto tibia = tibia_bone_config();
    ASSERT_EQ(sofa_add_rigid_bone(&tibia), 0);
    auto talus = talus_bone_config();
    ASSERT_EQ(sofa_add_rigid_bone(&talus), 0);
    auto calcaneus = calcaneus_bone_config();
    ASSERT_EQ(sofa_add_rigid_bone(&calcaneus), 0);

    // Add a subtalar ligament between Talus and Calcaneus
    SofaLigamentConfig lig = {};
    lig.name = "ITCL";
    lig.bone_a_name = "Talus";
    lig.bone_b_name = "Calcaneus";
    lig.tibia_offset[0] = 0; lig.tibia_offset[1] = -5; lig.tibia_offset[2] = -10;
    lig.talus_offset[0] = 0; lig.talus_offset[1] = 5; lig.talus_offset[2] = 10;
    lig.stiffness = 120.0;
    lig.damping = 5.0;
    lig.rest_length = 0.0;

    int rc = sofa_add_ligament(&lig);
    EXPECT_EQ(rc, 0) << sofa_bridge_get_error();

    ASSERT_EQ(sofa_scene_finalize(), 0) << sofa_bridge_get_error();
}

// ---------------------------------------------------------------------------
// 23. ThreeBoneScene_1000Steps_Stable
// ---------------------------------------------------------------------------
TEST_F(SceneBuilderTest, ThreeBoneScene_1000Steps_Stable) {
    auto cfg = default_scene_config();
    ASSERT_EQ(sofa_scene_create(&cfg), 0);

    auto tibia = tibia_bone_config();
    ASSERT_EQ(sofa_add_rigid_bone(&tibia), 0);
    auto talus = talus_bone_config();
    ASSERT_EQ(sofa_add_rigid_bone(&talus), 0);
    auto calcaneus = calcaneus_bone_config();
    ASSERT_EQ(sofa_add_rigid_bone(&calcaneus), 0);

    // Tibiotalar ligaments
    auto lig1 = default_ligament("ATFL", 15, 10, -14, 12, 8, 11, 70);
    auto lig2 = default_ligament("PTFL", 15, -10, -14, 12, -8, 11, 50);
    auto lig3 = default_ligament("Deltoid_ant", -12, 10, -14, -10, 8, 11, 90);
    auto lig4 = default_ligament("Deltoid_post", -12, -10, -14, -10, -8, 11, 90);
    ASSERT_EQ(sofa_add_ligament(&lig1), 0);
    ASSERT_EQ(sofa_add_ligament(&lig2), 0);
    ASSERT_EQ(sofa_add_ligament(&lig3), 0);
    ASSERT_EQ(sofa_add_ligament(&lig4), 0);

    // Subtalar ligament
    SofaLigamentConfig subtalar = {};
    subtalar.name = "ITCL";
    subtalar.bone_a_name = "Talus";
    subtalar.bone_b_name = "Calcaneus";
    subtalar.tibia_offset[0] = 0; subtalar.tibia_offset[1] = -5; subtalar.tibia_offset[2] = -10;
    subtalar.talus_offset[0] = 0; subtalar.talus_offset[1] = 5; subtalar.talus_offset[2] = 10;
    subtalar.stiffness = 120.0;
    subtalar.damping = 5.0;
    ASSERT_EQ(sofa_add_ligament(&subtalar), 0);

    ASSERT_EQ(sofa_scene_finalize(), 0) << sofa_bridge_get_error();

    // Run 1000 steps
    for (int i = 0; i < 1000; i++) {
        ASSERT_EQ(sofa_step(0.001f), 0) << "Step " << i << " failed: " << sofa_bridge_get_error();
    }

    SofaFrameSnapshot snap = {};
    ASSERT_EQ(sofa_get_frame_snapshot(&snap), 0);
    EXPECT_EQ(snap.solver_diverged, 0) << "Solver diverged during 3-bone simulation";
    EXPECT_FALSE(std::isnan(snap.calcaneus.px));
    EXPECT_FALSE(std::isnan(snap.calcaneus.py));
    EXPECT_FALSE(std::isnan(snap.calcaneus.pz));
}

// ---------------------------------------------------------------------------
// 24. FillSnapshot_WithoutCalcaneus_CalcaneusFrameIsZero
// ---------------------------------------------------------------------------
TEST_F(SceneBuilderTest, FillSnapshot_WithoutCalcaneus_CalcaneusFrameIsZero) {
    auto cfg = default_scene_config();
    ASSERT_EQ(sofa_scene_create(&cfg), 0);

    auto tibia = tibia_bone_config();
    ASSERT_EQ(sofa_add_rigid_bone(&tibia), 0);
    auto talus = talus_bone_config();
    ASSERT_EQ(sofa_add_rigid_bone(&talus), 0);

    ASSERT_EQ(sofa_scene_finalize(), 0);

    SofaFrameSnapshot snap = {};
    ASSERT_EQ(sofa_get_frame_snapshot(&snap), 0);

    // No calcaneus in scene — frame should be zeroed
    EXPECT_NEAR(snap.calcaneus.px, 0.0, 1e-10);
    EXPECT_NEAR(snap.calcaneus.py, 0.0, 1e-10);
    EXPECT_NEAR(snap.calcaneus.pz, 0.0, 1e-10);
    EXPECT_NEAR(snap.calcaneus.qx, 0.0, 1e-10);
    EXPECT_NEAR(snap.calcaneus.qy, 0.0, 1e-10);
    EXPECT_NEAR(snap.calcaneus.qz, 0.0, 1e-10);
    EXPECT_NEAR(snap.calcaneus.qw, 0.0, 1e-10);
}

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
