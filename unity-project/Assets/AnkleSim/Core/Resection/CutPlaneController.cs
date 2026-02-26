using UnityEngine;
using AnkleSim.Core.DataModels;

namespace AnkleSim.Core.Resection
{
    public class CutPlaneController
    {
        public BoneType TargetBone { get; private set; }
        public float CoronalAngleDeg { get; private set; }
        public float SagittalAngleDeg { get; private set; }
        public float DepthMm { get; private set; }

        public CutPlaneController(BoneType target)
        {
            TargetBone = target;
            CoronalAngleDeg = 90f;
            SagittalAngleDeg = 89f;
            DepthMm = 0f;
        }

        public Plane GetPlane()
        {
            float coronalRad = CoronalAngleDeg * Mathf.Deg2Rad;
            float sagittalRad = SagittalAngleDeg * Mathf.Deg2Rad;

            // Normal from spherical angles (coronal = polar from Y, sagittal = azimuth from Z)
            Vector3 normal = new Vector3(
                Mathf.Sin(coronalRad) * Mathf.Cos(sagittalRad),
                Mathf.Cos(coronalRad),
                Mathf.Sin(coronalRad) * Mathf.Sin(sagittalRad)
            ).normalized;

            // Plane point offset along normal by depth
            Vector3 point = normal * DepthMm;

            // Unity Plane: defined by inward normal and point on plane
            return new Plane(normal, point);
        }

        public void AdjustDepth(float deltaMm)
        {
            DepthMm += deltaMm;
        }

        public void SetAngles(float coronalDeg, float sagittalDeg)
        {
            CoronalAngleDeg = coronalDeg;
            SagittalAngleDeg = sagittalDeg;
        }

        public bool IntersectsBone(Bounds boneBounds)
        {
            Plane plane = GetPlane();

            // Test all 8 corners of the AABB against the plane.
            // If any corners are on different sides, the plane intersects.
            Vector3 min = boneBounds.min;
            Vector3 max = boneBounds.max;

            bool hasPositive = false;
            bool hasNegative = false;

            for (int i = 0; i < 8; i++)
            {
                Vector3 corner = new Vector3(
                    (i & 1) == 0 ? min.x : max.x,
                    (i & 2) == 0 ? min.y : max.y,
                    (i & 4) == 0 ? min.z : max.z
                );

                float dist = plane.GetDistanceToPoint(corner);
                if (dist > 0f) hasPositive = true;
                else hasNegative = true;

                if (hasPositive && hasNegative) return true;
            }

            return false;
        }

        public static float EstimateVolume(Mesh mesh, Plane plane)
        {
            Vector3[] vertices = mesh.vertices;
            int[] triangles = mesh.triangles;

            // Use a reference point ON the plane as the tetrahedron apex.
            // This eliminates the cap polygon contribution (zero-height tets).
            Vector3 refPoint = -plane.normal * plane.distance;

            float volume = 0f;

            for (int i = 0; i < triangles.Length; i += 3)
            {
                Vector3 v0 = vertices[triangles[i]];
                Vector3 v1 = vertices[triangles[i + 1]];
                Vector3 v2 = vertices[triangles[i + 2]];

                // Clip triangle against plane, keeping the below-plane portion
                var clipped = ClipTriangleBelowPlane(v0, v1, v2, plane);
                if (clipped.Length < 3) continue;

                // Fan-triangulate clipped polygon and sum signed tet volumes
                for (int j = 1; j < clipped.Length - 1; j++)
                {
                    Vector3 a = clipped[0] - refPoint;
                    Vector3 b = clipped[j] - refPoint;
                    Vector3 c = clipped[j + 1] - refPoint;
                    volume += Vector3.Dot(a, Vector3.Cross(b, c)) / 6f;
                }
            }

            return Mathf.Abs(volume);
        }

        // Sutherland-Hodgman clip: keep vertices on the negative side of the plane
        private static Vector3[] ClipTriangleBelowPlane(
            Vector3 v0, Vector3 v1, Vector3 v2, Plane plane)
        {
            Vector3[] poly = { v0, v1, v2 };
            var result = new System.Collections.Generic.List<Vector3>(4);

            for (int i = 0; i < 3; i++)
            {
                Vector3 cur = poly[i];
                Vector3 next = poly[(i + 1) % 3];
                float dCur = plane.GetDistanceToPoint(cur);
                float dNext = plane.GetDistanceToPoint(next);

                if (dCur < 0f)
                    result.Add(cur);

                if ((dCur < 0f) != (dNext < 0f))
                {
                    float t = dCur / (dCur - dNext);
                    result.Add(Vector3.Lerp(cur, next, t));
                }
            }

            return result.ToArray();
        }
    }
}
