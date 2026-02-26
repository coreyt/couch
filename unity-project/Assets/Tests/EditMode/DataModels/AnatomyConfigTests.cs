using NUnit.Framework;
using UnityEngine;
using AnkleSim.Core.DataModels;
using AnkleSim.Runtime.Anatomy;

namespace AnkleSim.Tests.EditMode.DataModels
{
    public class AnatomyConfigTests
    {
        [Test]
        public void AnatomyConfig_GetMeshForBone_ReturnsCorrectMesh()
        {
            var config = ScriptableObject.CreateInstance<AnatomyConfig>();
            var tibiaMesh = new Mesh { name = "Tibia" };
            var talusMesh = new Mesh { name = "Talus" };
            config.boneMeshes = new[]
            {
                new BoneMeshEntry { boneType = BoneType.Tibia, mesh = tibiaMesh },
                new BoneMeshEntry { boneType = BoneType.Talus, mesh = talusMesh },
            };

            Assert.AreEqual(tibiaMesh, config.GetMeshForBone(BoneType.Tibia));
            Assert.AreEqual(talusMesh, config.GetMeshForBone(BoneType.Talus));

            Object.DestroyImmediate(config);
        }

        [Test]
        public void AnatomyConfig_GetMeshForBone_ReturnsNullForMissingBone()
        {
            var config = ScriptableObject.CreateInstance<AnatomyConfig>();
            config.boneMeshes = new[]
            {
                new BoneMeshEntry { boneType = BoneType.Tibia, mesh = new Mesh() },
            };

            Assert.IsNull(config.GetMeshForBone(BoneType.Calcaneus));
            Assert.IsNull(config.GetMeshForBone(BoneType.Navicular));

            Object.DestroyImmediate(config);
        }
    }
}
