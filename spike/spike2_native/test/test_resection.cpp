#include <gtest/gtest.h>
#include "sofa_ankle_bridge.h"
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

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

static SofaSceneConfig default_scene_config() {
    SofaSceneConfig cfg = {};
    cfg.gravity[0] = 0.0f;
    cfg.gravity[1] = 0.0f;
    cfg.gravity[2] = 0.0f; // No gravity for resection tests
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

// Unit cube [0,1]^3 decomposed into 5 tetrahedra
// 8 vertices at corners, standard decomposition
static const float CUBE_VERTICES[] = {
    0, 0, 0,  // 0
    1, 0, 0,  // 1
    1, 1, 0,  // 2
    0, 1, 0,  // 3
    0, 0, 1,  // 4
    1, 0, 1,  // 5
    1, 1, 1,  // 6
    0, 1, 1,  // 7
};
static const int CUBE_VERTEX_COUNT = 8;

// 5-tetrahedra decomposition of unit cube
static const int CUBE_TETRAHEDRA[] = {
    0, 1, 3, 4,  // tet 0
    1, 2, 3, 6,  // tet 1
    1, 3, 4, 6,  // tet 2
    4, 6, 3, 7,  // tet 3
    1, 4, 5, 6,  // tet 4
};
static const int CUBE_TETRA_COUNT = 5;

static SofaDeformableConfig cube_deformable_config() {
    SofaDeformableConfig cfg = {};
    cfg.name = "TestCube";
    cfg.parent_bone = nullptr;
    cfg.vertices = CUBE_VERTICES;
    cfg.vertex_count = CUBE_VERTEX_COUNT;
    cfg.tetrahedra = CUBE_TETRAHEDRA;
    cfg.tetra_count = CUBE_TETRA_COUNT;
    cfg.young_modulus = 1000.0f;     // Low modulus for test stability
    cfg.poisson_ratio = 0.3f;
    cfg.mass_density = 1.0f;
    return cfg;
}

class ResectionTest : public ::testing::Test {
protected:
    void SetUp() override {
        int rc = sofa_bridge_init(get_plugin_dir());
        ASSERT_EQ(rc, 0) << "Init failed: " << sofa_bridge_get_error();
    }

    void TearDown() override {
        sofa_bridge_shutdown();
    }

    // Helper: create scene with deformable cube and finalize
    void createCubeScene() {
        auto cfg = default_scene_config();
        ASSERT_EQ(0, sofa_scene_create(&cfg)) << sofa_bridge_get_error();

        auto dcfg = cube_deformable_config();
        ASSERT_EQ(0, sofa_add_deformable_tissue(&dcfg)) << sofa_bridge_get_error();

        ASSERT_EQ(0, sofa_scene_finalize()) << sofa_bridge_get_error();
    }

    // Helper: create scene with rigid bone + deformable cube and finalize
    void createBoneAndCubeScene() {
        auto cfg = default_scene_config();
        ASSERT_EQ(0, sofa_scene_create(&cfg)) << sofa_bridge_get_error();

        // Add a fixed rigid bone
        SofaRigidBoneConfig bone = {};
        bone.name = "Tibia";
        bone.position[0] = 0; bone.position[1] = 0; bone.position[2] = 0;
        bone.orientation[0] = 0; bone.orientation[1] = 0;
        bone.orientation[2] = 0; bone.orientation[3] = 1;
        bone.mass = 1.0f;
        bone.is_fixed = 1;
        bone.collision_vertices = nullptr;
        bone.collision_vertex_count = 0;
        bone.collision_triangles = nullptr;
        bone.collision_triangle_count = 0;
        ASSERT_EQ(0, sofa_add_rigid_bone(&bone)) << sofa_bridge_get_error();

        auto dcfg = cube_deformable_config();
        ASSERT_EQ(0, sofa_add_deformable_tissue(&dcfg)) << sofa_bridge_get_error();

        ASSERT_EQ(0, sofa_scene_finalize()) << sofa_bridge_get_error();
    }
};

// 1. Centroid-based removal: plane at z=0.5 removes tetras with centroid below
TEST_F(ResectionTest, CentroidBased_RemovesCorrectTetrahedra) {
    createCubeScene();

    // Cut plane at z=0.5, normal pointing up (+z)
    // Tetras with centroid z < 0.5 should be removed
    SofaResectionCommand cmd = {};
    cmd.plane_point[0] = 0.0f;
    cmd.plane_point[1] = 0.0f;
    cmd.plane_point[2] = 0.5f;
    cmd.plane_normal[0] = 0.0f;
    cmd.plane_normal[1] = 0.0f;
    cmd.plane_normal[2] = 1.0f;
    cmd.bone_name = "TestCube";

    int rc = sofa_execute_resection(&cmd);
    EXPECT_EQ(rc, 0) << "Resection failed: " << sofa_bridge_get_error();

    int removed = sofa_get_removed_element_count();
    EXPECT_GT(removed, 0) << "Should have removed some tetrahedra";
    EXPECT_LT(removed, CUBE_TETRA_COUNT) << "Should not have removed all tetrahedra";
}

// 2. Centroid method: verify the boundary is reasonable
TEST_F(ResectionTest, CentroidBased_SmoothCutBoundary) {
    createCubeScene();

    // Cut at z=0.3 — not at midpoint, so different tetras removed
    SofaResectionCommand cmd = {};
    cmd.plane_point[0] = 0.0f;
    cmd.plane_point[1] = 0.0f;
    cmd.plane_point[2] = 0.3f;
    cmd.plane_normal[0] = 0.0f;
    cmd.plane_normal[1] = 0.0f;
    cmd.plane_normal[2] = 1.0f;
    cmd.bone_name = "TestCube";

    int rc = sofa_execute_resection(&cmd);
    EXPECT_EQ(rc, 0) << sofa_bridge_get_error();

    // After resection, surface mesh should still be valid
    std::vector<float> verts(CUBE_VERTEX_COUNT * 3);
    std::vector<int> tris(CUBE_TETRA_COUNT * 4 * 3); // generous capacity
    SofaSurfaceMesh mesh = {};
    mesh.vertices = verts.data();
    mesh.triangles = tris.data();
    mesh.vertex_count = CUBE_VERTEX_COUNT;
    mesh.triangle_count = static_cast<int>(tris.size() / 3);

    rc = sofa_get_surface_mesh(&mesh);
    EXPECT_EQ(rc, 0) << sofa_bridge_get_error();
    // Surface should have some triangles remaining
    EXPECT_GT(mesh.triangle_count, 0) << "Should have remaining surface triangles";
}

// 3. Topology changed flag set after cut
TEST_F(ResectionTest, TopologyChanged_FlagSetAfterCut) {
    createCubeScene();

    EXPECT_EQ(0, sofa_has_topology_changed()) << "No topology change before resection";

    SofaResectionCommand cmd = {};
    cmd.plane_point[0] = 0.5f;
    cmd.plane_point[1] = 0.5f;
    cmd.plane_point[2] = 0.5f;
    cmd.plane_normal[0] = 0.0f;
    cmd.plane_normal[1] = 0.0f;
    cmd.plane_normal[2] = 1.0f;
    cmd.bone_name = "TestCube";

    ASSERT_EQ(0, sofa_execute_resection(&cmd)) << sofa_bridge_get_error();

    EXPECT_EQ(1, sofa_has_topology_changed()) << "Topology should have changed after resection";
}

// 4. Surface mesh changes after cut
TEST_F(ResectionTest, GetSurfaceMesh_ReturnsUpdatedSurface) {
    createCubeScene();

    // Get surface mesh before cut
    std::vector<float> verts_before(CUBE_VERTEX_COUNT * 3);
    std::vector<int> tris_before(100 * 3);
    SofaSurfaceMesh mesh_before = {};
    mesh_before.vertices = verts_before.data();
    mesh_before.triangles = tris_before.data();
    mesh_before.vertex_count = CUBE_VERTEX_COUNT;
    mesh_before.triangle_count = 100;
    ASSERT_EQ(0, sofa_get_surface_mesh(&mesh_before)) << sofa_bridge_get_error();

    int tri_count_before = mesh_before.triangle_count;

    // Execute resection
    SofaResectionCommand cmd = {};
    cmd.plane_point[0] = 0.5f;
    cmd.plane_point[1] = 0.5f;
    cmd.plane_point[2] = 0.5f;
    cmd.plane_normal[0] = 0.0f;
    cmd.plane_normal[1] = 0.0f;
    cmd.plane_normal[2] = 1.0f;
    cmd.bone_name = "TestCube";
    ASSERT_EQ(0, sofa_execute_resection(&cmd)) << sofa_bridge_get_error();

    // Get surface mesh after cut
    std::vector<float> verts_after(CUBE_VERTEX_COUNT * 3);
    std::vector<int> tris_after(100 * 3);
    SofaSurfaceMesh mesh_after = {};
    mesh_after.vertices = verts_after.data();
    mesh_after.triangles = tris_after.data();
    mesh_after.vertex_count = CUBE_VERTEX_COUNT;
    mesh_after.triangle_count = 100;
    ASSERT_EQ(0, sofa_get_surface_mesh(&mesh_after)) << sofa_bridge_get_error();

    // Triangle count should differ after resection
    EXPECT_NE(tri_count_before, mesh_after.triangle_count)
        << "Surface triangle count should change after resection";
}

// 5. Resection preserves boundary conditions (rigid bone still works)
TEST_F(ResectionTest, Resection_PreservesBoundaryConditions) {
    createBoneAndCubeScene();

    // Execute resection
    SofaResectionCommand cmd = {};
    cmd.plane_point[0] = 0.5f;
    cmd.plane_point[1] = 0.5f;
    cmd.plane_point[2] = 0.5f;
    cmd.plane_normal[0] = 0.0f;
    cmd.plane_normal[1] = 0.0f;
    cmd.plane_normal[2] = 1.0f;
    cmd.bone_name = "TestCube";
    ASSERT_EQ(0, sofa_execute_resection(&cmd)) << sofa_bridge_get_error();

    // Rigid bone should still be functional — verify via snapshot
    SofaFrameSnapshot snap = {};
    int rc = sofa_get_frame_snapshot(&snap);
    EXPECT_EQ(rc, 0) << "Snapshot should succeed after resection: " << sofa_bridge_get_error();

    // Tibia (fixed bone) should still be at origin
    EXPECT_NEAR(snap.tibia.px, 0.0, 1.0);
    EXPECT_NEAR(snap.tibia.py, 0.0, 1.0);
    EXPECT_NEAR(snap.tibia.pz, 0.0, 1.0);
}

// 6. Simulation steps without crash after resection
TEST_F(ResectionTest, Resection_CollisionModelRemainsValid) {
    createCubeScene();

    // Execute resection
    SofaResectionCommand cmd = {};
    cmd.plane_point[0] = 0.5f;
    cmd.plane_point[1] = 0.5f;
    cmd.plane_point[2] = 0.5f;
    cmd.plane_normal[0] = 0.0f;
    cmd.plane_normal[1] = 0.0f;
    cmd.plane_normal[2] = 1.0f;
    cmd.bone_name = "TestCube";
    ASSERT_EQ(0, sofa_execute_resection(&cmd)) << sofa_bridge_get_error();

    // Step the simulation multiple times — should not crash
    for (int i = 0; i < 10; i++) {
        int rc = sofa_step(0.001f);
        EXPECT_EQ(rc, 0) << "Step " << i << " failed after resection: " << sofa_bridge_get_error();
    }
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
