using NUnit.Framework;
using AnkleSim.Core.DataModels;
using AnkleSim.Core.Validation;

namespace AnkleSim.Tests.EditMode.Validation
{
    public class AlignmentValidatorTests
    {
        [Test]
        public void Validate_TibiotalarAngle0_ReturnsAcceptable()
        {
            var metrics = new AlignmentMetrics
            {
                tibiotalarAngle = 0f,
                anteriorDistalTibialAngle = 89f,
                posteriorSlope = 3f,
                tibiotalarCongruence = 1f
            };

            AlignmentValidator.Validate(metrics);

            Assert.IsTrue(metrics.isAcceptable);
            Assert.AreEqual(0, metrics.warnings.Count);
        }

        [Test]
        public void Validate_TibiotalarAngle11_ReturnsWarning()
        {
            var metrics = new AlignmentMetrics
            {
                tibiotalarAngle = 11f,
                anteriorDistalTibialAngle = 89f,
                posteriorSlope = 3f,
                tibiotalarCongruence = 1f
            };

            AlignmentValidator.Validate(metrics);

            Assert.IsFalse(metrics.isAcceptable);
            Assert.IsTrue(metrics.warnings.Exists(w => w.parameter == "Tibiotalar Angle"));
        }

        [Test]
        public void Validate_ADTA89_ReturnsAcceptable()
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
            Assert.IsFalse(metrics.warnings.Exists(w => w.parameter == "ADTA"));
        }

        [Test]
        public void Validate_ADTA85_ReturnsWarning()
        {
            var metrics = new AlignmentMetrics
            {
                tibiotalarAngle = 5f,
                anteriorDistalTibialAngle = 85f,
                posteriorSlope = 3f,
                tibiotalarCongruence = 1f
            };

            AlignmentValidator.Validate(metrics);

            Assert.IsFalse(metrics.isAcceptable);
            Assert.IsTrue(metrics.warnings.Exists(w => w.parameter == "ADTA"));
        }

        [Test]
        public void Validate_PosteriorSlope6_ReturnsWarning()
        {
            var metrics = new AlignmentMetrics
            {
                tibiotalarAngle = 5f,
                anteriorDistalTibialAngle = 89f,
                posteriorSlope = 6f,
                tibiotalarCongruence = 1f
            };

            AlignmentValidator.Validate(metrics);

            Assert.IsFalse(metrics.isAcceptable);
            Assert.IsTrue(metrics.warnings.Exists(w => w.parameter == "Posterior Slope"));
        }

        [Test]
        public void Validate_AllGood_ReturnsNoWarnings()
        {
            var metrics = new AlignmentMetrics
            {
                tibiotalarAngle = 3f,
                anteriorDistalTibialAngle = 90f,
                posteriorSlope = 2f,
                tibiotalarCongruence = 0.5f
            };

            AlignmentValidator.Validate(metrics);

            Assert.IsTrue(metrics.isAcceptable);
            Assert.AreEqual(0, metrics.warnings.Count);
        }

        [Test]
        public void Validate_MultipleViolations_ReturnsAllWarnings()
        {
            var metrics = new AlignmentMetrics
            {
                tibiotalarAngle = 15f,
                anteriorDistalTibialAngle = 80f,
                posteriorSlope = 8f,
                tibiotalarCongruence = 4f
            };

            AlignmentValidator.Validate(metrics);

            Assert.IsFalse(metrics.isAcceptable);
            Assert.AreEqual(4, metrics.warnings.Count);
            Assert.IsTrue(metrics.warnings.Exists(w => w.parameter == "Tibiotalar Angle"));
            Assert.IsTrue(metrics.warnings.Exists(w => w.parameter == "ADTA"));
            Assert.IsTrue(metrics.warnings.Exists(w => w.parameter == "Posterior Slope"));
            Assert.IsTrue(metrics.warnings.Exists(w => w.parameter == "Tibiotalar Congruence"));
        }
    }
}
