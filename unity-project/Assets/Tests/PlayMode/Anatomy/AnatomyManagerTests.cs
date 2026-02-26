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
            _config.boneMeshes = new[]
            {
                new BoneMeshEntry { boneType = BoneType.Tibia, mesh = CreateTestMesh("Tibia", 24) },
                new BoneMeshEntry { boneType = BoneType.Talus, mesh = CreateTestMesh("Talus", 36) },
                new BoneMeshEntry { boneType = BoneType.Fibula, mesh = CreateTestMesh("Fibula", 18) },
                new BoneMeshEntry { boneType = BoneType.Calcaneus, mesh = CreateTestMesh("Calcaneus", 12) },
            };
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

            // All 4 provided bone types should have GameObjects
            Assert.IsNotNull(_manager.GetBoneGameObject(BoneType.Tibia));
            Assert.IsNotNull(_manager.GetBoneGameObject(BoneType.Talus));
            Assert.IsNotNull(_manager.GetBoneGameObject(BoneType.Fibula));
            Assert.IsNotNull(_manager.GetBoneGameObject(BoneType.Calcaneus));
        }

        [UnityTest]
        public IEnumerator LoadAnatomy_SkipsBoneTypesWithNoMesh()
        {
            // Config only has 4 bones, so 24 others should be null
            _manager.LoadAnatomy(_config);
            yield return null;

            Assert.IsNull(_manager.GetBoneGameObject(BoneType.Navicular));
            Assert.IsNull(_manager.GetBoneGameObject(BoneType.Metatarsal1));
            Assert.IsNull(_manager.GetBoneGameObject(BoneType.DistalPhalanx5));
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
            var partialConfig = ScriptableObject.CreateInstance<AnatomyConfig>();
            partialConfig.boneMeshes = new[]
            {
                new BoneMeshEntry { boneType = BoneType.Tibia, mesh = CreateTestMesh("Tibia", 24) },
            };
            partialConfig.ligaments = new LigamentConfig[0];

            _manager.LoadAnatomy(partialConfig);
            yield return null;

            var mesh = _manager.GetBoneMesh(BoneType.Talus);
            Assert.IsNull(mesh);

            Object.DestroyImmediate(partialConfig);
        }

        [UnityTest]
        public IEnumerator LoadAnatomy_NullConfig_DoesNotThrow()
        {
            _manager.LoadAnatomy(null);
            yield return null;

            Assert.IsNull(_manager.GetBoneGameObject(BoneType.Tibia));
        }

        [UnityTest]
        public IEnumerator LoadAnatomy_Reload_DestroysOldObjects()
        {
            _manager.LoadAnatomy(_config);
            yield return null;

            var firstTibia = _manager.GetBoneGameObject(BoneType.Tibia);
            Assert.IsNotNull(firstTibia);

            var config2 = ScriptableObject.CreateInstance<AnatomyConfig>();
            config2.boneMeshes = new[]
            {
                new BoneMeshEntry { boneType = BoneType.Tibia, mesh = CreateTestMesh("Tibia2", 48) },
                new BoneMeshEntry { boneType = BoneType.Talus, mesh = CreateTestMesh("Talus2", 60) },
                new BoneMeshEntry { boneType = BoneType.Fibula, mesh = CreateTestMesh("Fibula2", 30) },
                new BoneMeshEntry { boneType = BoneType.Calcaneus, mesh = CreateTestMesh("Calcaneus2", 20) },
            };
            config2.ligaments = new LigamentConfig[0];

            _manager.LoadAnatomy(config2);
            yield return null;

            Assert.IsTrue(firstTibia == null);

            var newTibia = _manager.GetBoneGameObject(BoneType.Tibia);
            Assert.IsNotNull(newTibia);
            Assert.AreEqual(48, _manager.GetBoneMesh(BoneType.Tibia).vertexCount);

            Object.DestroyImmediate(config2);
        }

        [UnityTest]
        public IEnumerator LoadAnatomy_CreatesAllProvidedBoneObjects()
        {
            // Create config with many bones
            var fullConfig = ScriptableObject.CreateInstance<AnatomyConfig>();
            fullConfig.boneMeshes = new[]
            {
                new BoneMeshEntry { boneType = BoneType.Tibia, mesh = CreateTestMesh("Tibia", 24) },
                new BoneMeshEntry { boneType = BoneType.Talus, mesh = CreateTestMesh("Talus", 24) },
                new BoneMeshEntry { boneType = BoneType.Calcaneus, mesh = CreateTestMesh("Calcaneus", 24) },
                new BoneMeshEntry { boneType = BoneType.Navicular, mesh = CreateTestMesh("Navicular", 24) },
                new BoneMeshEntry { boneType = BoneType.Cuboid, mesh = CreateTestMesh("Cuboid", 24) },
                new BoneMeshEntry { boneType = BoneType.Metatarsal1, mesh = CreateTestMesh("MT1", 24) },
            };
            fullConfig.ligaments = new LigamentConfig[0];

            _manager.LoadAnatomy(fullConfig);
            yield return null;

            Assert.IsNotNull(_manager.GetBoneGameObject(BoneType.Tibia));
            Assert.IsNotNull(_manager.GetBoneGameObject(BoneType.Talus));
            Assert.IsNotNull(_manager.GetBoneGameObject(BoneType.Calcaneus));
            Assert.IsNotNull(_manager.GetBoneGameObject(BoneType.Navicular));
            Assert.IsNotNull(_manager.GetBoneGameObject(BoneType.Cuboid));
            Assert.IsNotNull(_manager.GetBoneGameObject(BoneType.Metatarsal1));

            Object.DestroyImmediate(fullConfig);
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
