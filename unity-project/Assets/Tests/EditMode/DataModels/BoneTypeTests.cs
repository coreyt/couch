using System;
using System.Linq;
using NUnit.Framework;
using AnkleSim.Core.DataModels;

namespace AnkleSim.Tests.EditMode.DataModels
{
    public class BoneTypeTests
    {
        [Test]
        public void BoneType_HasExpected28Values()
        {
            var values = Enum.GetValues(typeof(BoneType));
            Assert.AreEqual(28, values.Length);
        }

        [Test]
        public void BoneType_IsSimulated_TrueForTibiaTalusCalcaneus()
        {
            Assert.IsTrue(BoneType.Tibia.IsSimulated());
            Assert.IsTrue(BoneType.Talus.IsSimulated());
            Assert.IsTrue(BoneType.Calcaneus.IsSimulated());
        }

        [Test]
        public void BoneType_IsSimulated_FalseForContextBones()
        {
            var contextBones = Enum.GetValues(typeof(BoneType)).Cast<BoneType>()
                .Where(b => b != BoneType.Tibia && b != BoneType.Talus && b != BoneType.Calcaneus);

            foreach (var bone in contextBones)
            {
                Assert.IsFalse(bone.IsSimulated(), $"{bone} should not be simulated");
            }
        }
    }
}
