using System.Collections;
using NUnit.Framework;
using UnityEngine;
using UnityEngine.TestTools;
using AnkleSim.Bridge;
using AnkleSim.Core.DataModels;
using AnkleSim.Runtime.Anatomy;

namespace AnkleSim.Tests.PlayMode.Visualization
{
    public class BoneVisualizerTests
    {
        private GameObject _rootGO;
        private AnatomyManager _anatomy;
        private BoneVisualizer _visualizer;
        private SofaBridgeComponent _bridge;
        private AnatomyConfig _config;

        [SetUp]
        public void SetUp()
        {
            _rootGO = new GameObject("TestRoot");
            _anatomy = _rootGO.AddComponent<AnatomyManager>();
            _visualizer = _rootGO.AddComponent<BoneVisualizer>();
            _bridge = _rootGO.AddComponent<SofaBridgeComponent>();

            // Wire up serialized fields via reflection (test only)
            var visBridgeField = typeof(BoneVisualizer).GetField("_bridge",
                System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance);
            visBridgeField?.SetValue(_visualizer, _bridge);

            var visAnatomyField = typeof(BoneVisualizer).GetField("_anatomy",
                System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance);
            visAnatomyField?.SetValue(_visualizer, _anatomy);

            _config = ScriptableObject.CreateInstance<AnatomyConfig>();
            _config.boneMeshes = new[]
            {
                new BoneMeshEntry { boneType = BoneType.Tibia, mesh = CreateTestMesh(24) },
                new BoneMeshEntry { boneType = BoneType.Talus, mesh = CreateTestMesh(24) },
                new BoneMeshEntry { boneType = BoneType.Calcaneus, mesh = CreateTestMesh(24) },
                new BoneMeshEntry { boneType = BoneType.Fibula, mesh = CreateTestMesh(24) },
                new BoneMeshEntry { boneType = BoneType.Navicular, mesh = CreateTestMesh(24) },
            };
            _config.ligaments = new LigamentConfig[0];
        }

        [TearDown]
        public void TearDown()
        {
            Object.DestroyImmediate(_rootGO);
            Object.DestroyImmediate(_config);
        }

        [UnityTest]
        public IEnumerator BoneVisualizer_StaticBonesDoNotMove()
        {
            _anatomy.LoadAnatomy(_config);
            yield return null;

            // Record Fibula position (static bone)
            var fibula = _anatomy.GetBoneGameObject(BoneType.Fibula);
            Assert.IsNotNull(fibula);
            var originalPos = fibula.transform.localPosition;

            // Run a frame — BoneVisualizer should not modify static bones
            yield return null;

            Assert.AreEqual(originalPos, fibula.transform.localPosition,
                "Static bone (Fibula) should not be moved by BoneVisualizer");
        }

        [UnityTest]
        public IEnumerator AnatomyManager_28Bones_AllRendered()
        {
            // Create a config with many bones
            var fullConfig = ScriptableObject.CreateInstance<AnatomyConfig>();
            var entries = new BoneMeshEntry[28];
            var boneTypes = System.Enum.GetValues(typeof(BoneType));
            for (int i = 0; i < 28; i++)
            {
                entries[i] = new BoneMeshEntry
                {
                    boneType = (BoneType)boneTypes.GetValue(i),
                    mesh = CreateTestMesh(12)
                };
            }
            fullConfig.boneMeshes = entries;
            fullConfig.ligaments = new LigamentConfig[0];

            _anatomy.LoadAnatomy(fullConfig);
            yield return null;

            int renderedCount = 0;
            foreach (BoneType bt in boneTypes)
            {
                var go = _anatomy.GetBoneGameObject(bt);
                if (go != null)
                {
                    var renderer = go.GetComponent<MeshRenderer>();
                    Assert.IsNotNull(renderer, $"Bone {bt} should have a MeshRenderer");
                    renderedCount++;
                }
            }

            Assert.AreEqual(28, renderedCount, "All 28 bones should be rendered");

            Object.DestroyImmediate(fullConfig);
        }

        [UnityTest]
        public IEnumerator OrbitCamera_FramesBounds_Correctly()
        {
            var cameraGO = new GameObject("TestCamera");
            var cam = cameraGO.AddComponent<Camera>();
            var orbit = cameraGO.AddComponent<AnkleSim.Runtime.UI.OrbitCamera>();

            var bounds = new Bounds(Vector3.zero, new Vector3(100, 100, 100));
            orbit.FrameBounds(bounds);

            yield return null;

            // Camera should be looking toward the bounds center
            float dist = Vector3.Distance(cameraGO.transform.position, bounds.center);
            Assert.Greater(dist, 0f, "Camera should be some distance from center");
            Assert.Less(dist, 1000f, "Camera should not be extremely far away");

            Object.DestroyImmediate(cameraGO);
        }

        private Mesh CreateTestMesh(int vertexCount)
        {
            var mesh = new Mesh();
            var vertices = new Vector3[vertexCount];
            for (int i = 0; i < vertexCount; i++)
            {
                float angle = (2f * Mathf.PI * i) / vertexCount;
                vertices[i] = new Vector3(Mathf.Cos(angle), Mathf.Sin(angle), 0f);
            }
            mesh.vertices = vertices;
            if (vertexCount >= 3)
            {
                var triangles = new int[((vertexCount - 2) * 3)];
                for (int i = 0; i < vertexCount - 2; i++)
                {
                    triangles[i * 3] = 0;
                    triangles[i * 3 + 1] = i + 1;
                    triangles[i * 3 + 2] = i + 2;
                }
                mesh.triangles = triangles;
            }
            mesh.RecalculateBounds();
            return mesh;
        }
    }
}
