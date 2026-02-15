using System.Collections;
using NUnit.Framework;
using UnityEngine;
using UnityEngine.TestTools;
using AnkleSim.Core.DataModels;
using AnkleSim.Runtime.Anatomy;

namespace AnkleSim.Tests.PlayMode.Anatomy
{
    public class AnatomyManagerTests
    {
        private GameObject _managerGO;
        private AnatomyManager _manager;
        private AnatomyConfig _config;

        [SetUp]
        public void SetUp()
        {
            _managerGO = new GameObject("AnatomyManager");
            _manager = _managerGO.AddComponent<AnatomyManager>();

            _config = ScriptableObject.CreateInstance<AnatomyConfig>();
            _config.tibiaMesh = CreateTestMesh("Tibia", 24);
            _config.talusMesh = CreateTestMesh("Talus", 36);
            _config.fibulaMesh = CreateTestMesh("Fibula", 18);
            _config.calcaneusMesh = CreateTestMesh("Calcaneus", 12);
            _config.ligaments = new LigamentConfig[0];
        }

        [TearDown]
        public void TearDown()
        {
            Object.DestroyImmediate(_managerGO);
            Object.DestroyImmediate(_config);
        }

        [UnityTest]
        public IEnumerator LoadAnatomy_CreatesAllBoneGameObjects()
        {
            _manager.LoadAnatomy(_config);
            yield return null;

            foreach (BoneType boneType in System.Enum.GetValues(typeof(BoneType)))
            {
                var go = _manager.GetBoneGameObject(boneType);
                Assert.IsNotNull(go, $"GameObject for {boneType} should exist");
                Assert.AreEqual(boneType.ToString(), go.name);
            }
        }

        [UnityTest]
        public IEnumerator LoadAnatomy_MeshesHaveCorrectVertexCount()
        {
            _manager.LoadAnatomy(_config);
            yield return null;

            Assert.AreEqual(24, _manager.GetBoneMesh(BoneType.Tibia).vertexCount);
            Assert.AreEqual(36, _manager.GetBoneMesh(BoneType.Talus).vertexCount);
            Assert.AreEqual(18, _manager.GetBoneMesh(BoneType.Fibula).vertexCount);
            Assert.AreEqual(12, _manager.GetBoneMesh(BoneType.Calcaneus).vertexCount);
        }

        [UnityTest]
        public IEnumerator SetVisibility_HidesSpecifiedStructure()
        {
            _manager.LoadAnatomy(_config);
            yield return null;

            _manager.SetStructureVisibility("Tibia", false);

            var tibiaGO = _manager.GetBoneGameObject(BoneType.Tibia);
            Assert.IsFalse(tibiaGO.activeSelf);
        }

        [UnityTest]
        public IEnumerator SetVisibility_ShowsSpecifiedStructure()
        {
            _manager.LoadAnatomy(_config);
            yield return null;

            _manager.SetStructureVisibility("Tibia", false);
            _manager.SetStructureVisibility("Tibia", true);

            var tibiaGO = _manager.GetBoneGameObject(BoneType.Tibia);
            Assert.IsTrue(tibiaGO.activeSelf);
        }

        [UnityTest]
        public IEnumerator GetBoneMesh_ReturnsMeshForValidBoneType()
        {
            _manager.LoadAnatomy(_config);
            yield return null;

            var mesh = _manager.GetBoneMesh(BoneType.Talus);
            Assert.IsNotNull(mesh);
            Assert.AreEqual(36, mesh.vertexCount);
        }

        [UnityTest]
        public IEnumerator GetBoneMesh_ReturnsNullForUnloadedBoneType()
        {
            // Load config with only tibia mesh
            var partialConfig = ScriptableObject.CreateInstance<AnatomyConfig>();
            partialConfig.tibiaMesh = CreateTestMesh("Tibia", 24);
            partialConfig.ligaments = new LigamentConfig[0];

            _manager.LoadAnatomy(partialConfig);
            yield return null;

            var mesh = _manager.GetBoneMesh(BoneType.Talus);
            Assert.IsNull(mesh);

            Object.DestroyImmediate(partialConfig);
        }

        private Mesh CreateTestMesh(string name, int vertexCount)
        {
            var mesh = new Mesh { name = name };
            var vertices = new Vector3[vertexCount];
            for (int i = 0; i < vertexCount; i++)
            {
                float angle = (2f * Mathf.PI * i) / vertexCount;
                vertices[i] = new Vector3(Mathf.Cos(angle), Mathf.Sin(angle), 0f);
            }
            mesh.vertices = vertices;

            // Create minimal triangles (need at least 3 verts)
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
