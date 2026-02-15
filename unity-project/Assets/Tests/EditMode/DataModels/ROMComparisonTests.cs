using NUnit.Framework;
using AnkleSim.Core.DataModels;

namespace AnkleSim.Tests.EditMode.DataModels
{
    public class ROMComparisonTests
    {
        [Test]
        public void Create_CalculatesImprovementsCorrectly()
        {
            var pre = new ROMRecord
            {
                maxDorsiflexion = 10f,
                maxPlantarflexion = 20f,
                totalSagittalArc = 30f
            };
            var post = new ROMRecord
            {
                maxDorsiflexion = 15f,
                maxPlantarflexion = 28f,
                totalSagittalArc = 43f
            };

            var comparison = ROMComparison.Create(pre, post);

            Assert.AreEqual(5f, comparison.dfImprovement, 0.001f);
            Assert.AreEqual(8f, comparison.pfImprovement, 0.001f);
            Assert.AreEqual(13f, comparison.totalArcImprovement, 0.001f);
            Assert.AreSame(pre, comparison.preOp);
            Assert.AreSame(post, comparison.postOp);
        }

        [Test]
        public void Create_RecalculatesTotalArcFromStalValues()
        {
            var pre = new ROMRecord
            {
                maxDorsiflexion = 10f,
                maxPlantarflexion = 20f,
                totalSagittalArc = 0f // intentionally stale
            };
            var post = new ROMRecord
            {
                maxDorsiflexion = 15f,
                maxPlantarflexion = 28f,
                totalSagittalArc = 0f // intentionally stale
            };

            var comparison = ROMComparison.Create(pre, post);

            // After Create(), both records should have recalculated arcs
            Assert.AreEqual(30f, pre.totalSagittalArc, 0.001f);
            Assert.AreEqual(43f, post.totalSagittalArc, 0.001f);
            Assert.AreEqual(13f, comparison.totalArcImprovement, 0.001f);
        }
    }
}
