using NUnit.Framework;
using UnityEngine;
using AnkleSim.Runtime.Anatomy;

namespace AnkleSim.Tests.EditMode.DataModels
{
    public class BoneMaterialTests
    {
        [Test]
        public void BoneMaterial_SimulatedBones_AreOpaque()
        {
            var config = ScriptableObject.CreateInstance<BoneMaterialConfig>();
            var mat = config.CreateSimulatedBoneMaterial();

            // Simulated bone material should have alpha = 1
            Assert.IsNotNull(mat);
            Assert.AreEqual(1.0f, mat.color.a, 0.01f);

            Object.DestroyImmediate(config);
        }

        [Test]
        public void BoneMaterial_ContextBones_AreSemiTransparent()
        {
            var config = ScriptableObject.CreateInstance<BoneMaterialConfig>();
            var mat = config.CreateContextBoneMaterial();

            // Context bone material should have alpha < 1
            Assert.IsNotNull(mat);
            Assert.Less(mat.color.a, 1.0f, "Context bones should be semi-transparent");
            Assert.Greater(mat.color.a, 0.0f, "Context bones should still be visible");

            Object.DestroyImmediate(config);
        }
    }
}
