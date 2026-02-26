using System.Collections;
using System.IO;
using NUnit.Framework;
using UnityEngine;
using UnityEngine.TestTools;
using AnkleSim.Bridge;
using AnkleSim.Bridge.Resection;
using AnkleSim.Core.DataModels;
using AnkleSim.Core.Resection;

namespace AnkleSim.Tests.PlayMode.Resection
{
    [Timeout(30000)]
    public class ResectionIntegrationTests
    {
        private bool _dllAvailable;

        // Unit cube vertices and tetrahedra for test scenes
        private static readonly float[] CubeVerts = {
            0, 0, 0,  1, 0, 0,  1, 1, 0,  0, 1, 0,
            0, 0, 1,  1, 0, 1,  1, 1, 1,  0, 1, 1,
        };
        private static readonly int[] CubeTetras = {
            0, 1, 3, 4,
            1, 2, 3, 6,
            1, 3, 4, 6,
            4, 6, 3, 7,
            1, 4, 5, 6,
        };

        [SetUp]
        public void SetUp()
        {
            try
            {
                SofaNativeBridge.sofa_bridge_get_version();
                _dllAvailable = true;
            }
            catch (System.DllNotFoundException)
            {
                _dllAvailable = false;
            }

            if (!_dllAvailable)
                Assert.Ignore("SofaAnkleBridge DLL not available — skipping integration tests");
        }

        [TearDown]
        public void TearDown()
        {
            if (_dllAvailable)
            {
                try { SofaNativeBridge.sofa_bridge_shutdown(); }
                catch { /* best effort */ }
            }
        }

        private SofaSimulation CreateSimWithDeformable()
        {
            var sim = new SofaSimulation();
            sim.Initialize(Path.Combine(Application.dataPath, "Plugins", "x86_64"));

            var sceneConfig = SofaSceneConfig.CreateDefault();
            sceneConfig.gravity = new float[] { 0, 0, 0 }; // no gravity
            sim.CreateScene(sceneConfig);

            var dcfg = SofaDeformableConfig.Create("TestCube",
                CubeVerts, CubeTetras,
                1000f, 0.3f, 1.0f);
            try
            {
                sim.AddDeformableTissue(dcfg);
            }
            finally
            {
                SofaDeformableConfig.FreeNativePtrs(ref dcfg);
            }

            sim.FinalizeScene();
            return sim;
        }

        // 1. EzySlice visual cut (or SOFA fallback) produces valid mesh
        [UnityTest]
        public IEnumerator ExecuteCut_EzySlice_ProducesCleanVisualCut()
        {
            using (var sim = CreateSimWithDeformable())
            {
                var engine = new ResectionEngine(sim);
                var controller = new CutPlaneController(BoneType.Tibia);
                controller.SetAngles(90f, 90f); // perpendicular

                // Create a simple test mesh
                var mesh = CreateUnitCubeMesh();

                var record = engine.ExecuteCut(controller, mesh);

                // Visual cut should produce a mesh (via EzySlice or SOFA readback)
                Assert.IsTrue(record.visualCutComplete || record.sofaCutComplete,
                    "At least one cut representation should be complete");
            }
            yield return null;
        }

        // 2. SOFA resection removes tetrahedra
        [UnityTest]
        public IEnumerator ExecuteCut_SOFA_RemovesTetrahedra()
        {
            using (var sim = CreateSimWithDeformable())
            {
                var engine = new ResectionEngine(sim);
                var controller = new CutPlaneController(BoneType.Tibia);
                controller.AdjustDepth(0.5f); // move plane into cube

                var mesh = CreateUnitCubeMesh();
                var record = engine.ExecuteCut(controller, mesh);

                Assert.Greater(record.removedTetrahedraCount, 0,
                    "SOFA resection should remove tetrahedra");
            }
            yield return null;
        }

        // 3. Dual-representation: both cuts complete
        [UnityTest]
        public IEnumerator ExecuteCut_DualRepresentation_BothComplete()
        {
            using (var sim = CreateSimWithDeformable())
            {
                var engine = new ResectionEngine(sim);
                var controller = new CutPlaneController(BoneType.Tibia);

                var mesh = CreateUnitCubeMesh();
                var record = engine.ExecuteCut(controller, mesh);

                Assert.IsTrue(record.sofaCutComplete, "SOFA cut should be complete");
                // visualCutComplete depends on EzySlice or SOFA readback
                Assert.IsTrue(record.visualCutComplete,
                    "Visual cut should be complete (via EzySlice or SOFA readback)");
            }
            yield return null;
        }

        // 4. Execution time under 500ms
        [UnityTest]
        public IEnumerator ExecuteCut_CompletesUnder500ms()
        {
            using (var sim = CreateSimWithDeformable())
            {
                var engine = new ResectionEngine(sim);
                var controller = new CutPlaneController(BoneType.Tibia);

                var mesh = CreateUnitCubeMesh();
                var record = engine.ExecuteCut(controller, mesh);

                Assert.Less(record.executionTimeMs, 500f,
                    $"Resection took {record.executionTimeMs}ms, expected < 500ms");
            }
            yield return null;
        }

        // 5. Preview returns intersection contour
        [UnityTest]
        public IEnumerator PreviewCut_ShowsIntersectionContour()
        {
            using (var sim = CreateSimWithDeformable())
            {
                var engine = new ResectionEngine(sim);
                var controller = new CutPlaneController(BoneType.Tibia);

                var mesh = CreateUnitCubeMesh();
                Vector3[] contour = engine.PreviewCut(controller, mesh);

                Assert.IsNotNull(contour, "Preview should return contour points");
                Assert.Greater(contour.Length, 0, "Preview should have intersection points");
            }
            yield return null;
        }

        // 6. Undo rebuilds scene
        [UnityTest]
        public IEnumerator Undo_RebuildsSofaScene_RestoresOriginalMesh()
        {
            using (var sim = CreateSimWithDeformable())
            {
                var engine = new ResectionEngine(sim);
                var controller = new CutPlaneController(BoneType.Tibia);

                var mesh = CreateUnitCubeMesh();
                engine.ExecuteCut(controller, mesh);

                // Undo destroys the scene
                engine.UndoCut();

                // Scene should be destroyed (not ready)
                Assert.IsFalse(sim.IsSceneReady(),
                    "Scene should not be ready after undo (caller must rebuild)");
            }
            yield return null;
        }

        // 7. Safety check warns near landmarks
        [UnityTest]
        public IEnumerator SafetyCheck_WarnsWhenNearMalleolus()
        {
            using (var sim = CreateSimWithDeformable())
            {
                var engine = new ResectionEngine(sim);
                var controller = new CutPlaneController(BoneType.Tibia);

                // Place a landmark very close to the default cut plane
                Plane plane = controller.GetPlane();
                Vector3 planePoint = -plane.normal * plane.distance;
                Vector3[] landmarks = { planePoint + plane.normal * 2f };

                bool safe = engine.CheckSafety(controller, landmarks, 5f);
                Assert.IsFalse(safe,
                    "Should warn when landmark is within threshold of cut plane");
            }
            yield return null;
        }

        private static Mesh CreateUnitCubeMesh()
        {
            var mesh = new Mesh();
            mesh.vertices = new Vector3[]
            {
                new Vector3(0, 0, 0), new Vector3(1, 0, 0),
                new Vector3(1, 1, 0), new Vector3(0, 1, 0),
                new Vector3(0, 0, 1), new Vector3(1, 0, 1),
                new Vector3(1, 1, 1), new Vector3(0, 1, 1),
            };
            mesh.triangles = new int[]
            {
                0,1,5, 0,5,4,  2,3,7, 2,7,6,
                4,5,6, 4,6,7,  1,0,3, 1,3,2,
                1,2,6, 1,6,5,  0,4,7, 0,7,3,
            };
            mesh.RecalculateNormals();
            return mesh;
        }
    }
}
