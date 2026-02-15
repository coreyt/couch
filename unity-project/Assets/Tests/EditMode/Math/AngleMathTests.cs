using NUnit.Framework;
using AnkleSim.Core.Math;

namespace AnkleSim.Tests.EditMode.Math
{
    public class AngleMathTests
    {
        private const float Tolerance = 0.5f;

        [Test]
        public void CalculateDorsiflexion_At0Degrees_Returns0()
        {
            var angles = new double[] { 0, 0, 0 };
            Assert.AreEqual(0f, AngleMath.Dorsiflexion(angles), 0.001f);
        }

        [Test]
        public void CalculateDorsiflexion_At20Degrees_Returns20()
        {
            var angles = new double[] { 20, 0, 0 };
            Assert.AreEqual(20f, AngleMath.Dorsiflexion(angles), 0.001f);
        }

        [Test]
        public void CalculatePlantarflexion_At50Degrees_Returns50()
        {
            var angles = new double[] { -50, 0, 0 };
            Assert.AreEqual(50f, AngleMath.Plantarflexion(angles), 0.001f);
        }

        [Test]
        public void CalculateTotalArc_20DFPlus50PF_Returns70()
        {
            Assert.AreEqual(70f, AngleMath.TotalArc(20f, 50f), 0.001f);
        }

        [Test]
        public void CalculateInversion_At35Degrees_Returns35()
        {
            var angles = new double[] { 0, 35, 0 };
            Assert.AreEqual(35f, AngleMath.Inversion(angles), 0.001f);
        }

        [Test]
        public void AngleFromQuaternion_IdentityRotation_ReturnsZero()
        {
            var angles = AngleMath.EulerAnglesFromQuaternion(0, 0, 0, 1);

            Assert.AreEqual(0.0, angles[0], Tolerance, "Sagittal should be ~0");
            Assert.AreEqual(0.0, angles[1], Tolerance, "Frontal should be ~0");
            Assert.AreEqual(0.0, angles[2], Tolerance, "Transverse should be ~0");
        }

        [Test]
        public void AngleFromQuaternion_KnownRotation_ReturnsCorrectAngle()
        {
            // 20 degrees about X: qx = sin(10deg), qy = 0, qz = 0, qw = cos(10deg)
            double angle = 20.0 * System.Math.PI / 180.0;
            double qx = System.Math.Sin(angle / 2.0);
            double qw = System.Math.Cos(angle / 2.0);

            var angles = AngleMath.EulerAnglesFromQuaternion(qx, 0, 0, qw);

            Assert.AreEqual(20.0, angles[0], Tolerance, "Sagittal should be ~20 deg");
            Assert.AreEqual(0.0, angles[1], Tolerance, "Frontal should be ~0");
            Assert.AreEqual(0.0, angles[2], Tolerance, "Transverse should be ~0");
        }

        [Test]
        public void RelativeOrientation_TibiaToTalus_DecomposesCorrectly()
        {
            // Tibia at identity, talus at 15 degrees about X
            double angle = 15.0 * System.Math.PI / 180.0;
            double talusQx = System.Math.Sin(angle / 2.0);
            double talusQw = System.Math.Cos(angle / 2.0);

            var (rx, ry, rz, rw) = AngleMath.RelativeOrientation(
                0, 0, 0, 1,               // tibia: identity
                talusQx, 0, 0, talusQw);   // talus: 15 deg about X

            var angles = AngleMath.EulerAnglesFromQuaternion(rx, ry, rz, rw);

            Assert.AreEqual(15.0, angles[0], Tolerance, "Sagittal should be ~15 deg");
            Assert.AreEqual(0.0, angles[1], Tolerance, "Frontal should be ~0");
            Assert.AreEqual(0.0, angles[2], Tolerance, "Transverse should be ~0");
        }
    }
}
