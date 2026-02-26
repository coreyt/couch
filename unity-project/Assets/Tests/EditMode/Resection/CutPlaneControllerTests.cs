using NUnit.Framework;
using UnityEngine;
using AnkleSim.Core.DataModels;
using AnkleSim.Core.Resection;

namespace AnkleSim.Tests.EditMode.Resection
{
    public class CutPlaneControllerTests
    {
        [Test]
        public void CutPlane_DefaultTibial_Perpendicular90Degrees()
        {
            var controller = new CutPlaneController(BoneType.Tibia);
            Assert.AreEqual(90f, controller.CoronalAngleDeg, 0.001f);
        }

        [Test]
        public void CutPlane_SagittalAngle_DefaultsTo89Degrees()
        {
            var controller = new CutPlaneController(BoneType.Tibia);
            Assert.AreEqual(89f, controller.SagittalAngleDeg, 0.001f);
        }

        [Test]
        public void CutPlane_AdjustDepth_MovesPlaneAlongNormal()
        {
            var controller = new CutPlaneController(BoneType.Tibia);
            Plane before = controller.GetPlane();

            controller.AdjustDepth(5f);
            Plane after = controller.GetPlane();

            // Distance from origin should differ by ~5mm
            float distBefore = before.GetDistanceToPoint(Vector3.zero);
            float distAfter = after.GetDistanceToPoint(Vector3.zero);
            Assert.AreNotEqual(distBefore, distAfter, "Plane should have moved");
        }

        [Test]
        public void CutPlane_AdjustDepthBy1mm_AccurateWithin0Point1mm()
        {
            var controller = new CutPlaneController(BoneType.Tibia);
            controller.AdjustDepth(1f);

            Assert.AreEqual(1f, controller.DepthMm, 0.1f);
        }

        [Test]
        public void CutPlane_IntersectsBone_ReturnsTrueForValidPosition()
        {
            var controller = new CutPlaneController(BoneType.Tibia);
            // AABB centered at origin, extends from -5 to +5
            var bounds = new Bounds(Vector3.zero, new Vector3(10f, 10f, 10f));

            Assert.IsTrue(controller.IntersectsBone(bounds));
        }

        [Test]
        public void CutPlane_MissesBody_ReturnsFalseForInvalidPosition()
        {
            var controller = new CutPlaneController(BoneType.Tibia);
            controller.AdjustDepth(100f);

            // AABB far from the plane
            var bounds = new Bounds(new Vector3(-200f, -200f, -200f), new Vector3(1f, 1f, 1f));

            Assert.IsFalse(controller.IntersectsBone(bounds));
        }

        [Test]
        public void VolumeCalculation_KnownCube_ReturnsCorrectVolume()
        {
            // Create a unit cube mesh (1x1x1, centered at origin)
            var mesh = new Mesh();
            mesh.vertices = new Vector3[]
            {
                new Vector3(-0.5f, -0.5f, -0.5f), // 0
                new Vector3( 0.5f, -0.5f, -0.5f), // 1
                new Vector3( 0.5f,  0.5f, -0.5f), // 2
                new Vector3(-0.5f,  0.5f, -0.5f), // 3
                new Vector3(-0.5f, -0.5f,  0.5f), // 4
                new Vector3( 0.5f, -0.5f,  0.5f), // 5
                new Vector3( 0.5f,  0.5f,  0.5f), // 6
                new Vector3(-0.5f,  0.5f,  0.5f), // 7
            };
            mesh.triangles = new int[]
            {
                // Bottom face (y = -0.5)
                0, 1, 5,  0, 5, 4,
                // Top face (y = 0.5)
                2, 3, 7,  2, 7, 6,
                // Front face (z = 0.5)
                4, 5, 6,  4, 6, 7,
                // Back face (z = -0.5)
                1, 0, 3,  1, 3, 2,
                // Right face (x = 0.5)
                1, 2, 6,  1, 6, 5,
                // Left face (x = -0.5)
                0, 4, 7,  0, 7, 3,
            };

            // Plane at y=0, normal pointing up: splits cube in half
            var plane = new Plane(Vector3.up, Vector3.zero);

            float volume = CutPlaneController.EstimateVolume(mesh, plane);

            // Total cube volume = 1.0, half = 0.5
            Assert.AreEqual(0.5f, volume, 0.1f, "Half-cube volume should be ~0.5");
        }
    }
}
