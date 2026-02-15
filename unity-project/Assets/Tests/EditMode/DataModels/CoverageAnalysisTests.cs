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
            Assert.IsTrue(analysis.hasOverhang);
        }
    }
}
