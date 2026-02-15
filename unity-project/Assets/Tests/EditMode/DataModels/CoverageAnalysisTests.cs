using NUnit.Framework;
using AnkleSim.Core.DataModels;

namespace AnkleSim.Tests.EditMode.DataModels
{
    public class CoverageAnalysisTests
    {
        [Test]
        public void HasOverhang_DetectsOverhangWhenCoverageExceeds100()
        {
            var analysis = new CoverageAnalysis
            {
                tibialCoveragePercent = 105f,
                talarCoveragePercent = 98f
            };

            bool result = analysis.HasOverhang();

            Assert.IsTrue(result);
        }

        [Test]
        public void HasOverhang_ReturnsFalseWhenCoverageBelowThreshold()
        {
            var analysis = new CoverageAnalysis
            {
                tibialCoveragePercent = 95f,
                talarCoveragePercent = 98f
            };

            bool result = analysis.HasOverhang();

            Assert.IsFalse(result);
        }

        [Test]
        public void HasOverhang_DoesNotMutateField()
        {
            var analysis = new CoverageAnalysis
            {
                tibialCoveragePercent = 105f,
                talarCoveragePercent = 98f,
                hasOverhang = false
            };

            analysis.HasOverhang();

            // HasOverhang() should not mutate the hasOverhang field
            Assert.IsFalse(analysis.hasOverhang);
        }
    }
}
