using NUnit.Framework;
using AnkleSim.Core.DataModels;
using AnkleSim.Core.Validation;

namespace AnkleSim.Tests.EditMode.DataModels
{
    public class AlignmentMetricsTests
    {
        [Test]
        public void Validate_AllInRange_ReturnsAcceptable()
        {
            var metrics = new AlignmentMetrics
            {
                tibiotalarAngle = 5f,
                anteriorDistalTibialAngle = 89f,
                posteriorSlope = 3f,
                tibiotalarCongruence = 1f
            };

            AlignmentValidator.Validate(metrics);

            Assert.IsTrue(metrics.isAcceptable);
            Assert.AreEqual(0, metrics.warnings.Count);
        }

        [Test]
        public void Validate_TibiotalarAngle10_ReturnsNotAcceptable()
        {
            var metrics = new AlignmentMetrics
            {
                tibiotalarAngle = 10f,
                anteriorDistalTibialAngle = 89f,
                posteriorSlope = 3f,
                tibiotalarCongruence = 1f
            };

            AlignmentValidator.Validate(metrics);

            Assert.IsFalse(metrics.isAcceptable);
            Assert.AreEqual(1, metrics.warnings.Count);
        }

        [Test]
        public void Validate_ADTAOutsideRange_ReturnsNotAcceptable()
        {
            var metrics = new AlignmentMetrics
            {
                tibiotalarAngle = 5f,
                anteriorDistalTibialAngle = 84f,
                posteriorSlope = 3f,
                tibiotalarCongruence = 1f
            };

            AlignmentValidator.Validate(metrics);

            Assert.IsFalse(metrics.isAcceptable);
            Assert.AreEqual(1, metrics.warnings.Count);
        }

        [Test]
        public void Validate_MultipleViolations_GeneratesWarningPerViolation()
        {
            var metrics = new AlignmentMetrics
            {
                tibiotalarAngle = 12f,
                anteriorDistalTibialAngle = 95f,
                posteriorSlope = 7f,
                tibiotalarCongruence = 3f
            };

            AlignmentValidator.Validate(metrics);

            Assert.IsFalse(metrics.isAcceptable);
            Assert.AreEqual(4, metrics.warnings.Count);
        }
    }
}
