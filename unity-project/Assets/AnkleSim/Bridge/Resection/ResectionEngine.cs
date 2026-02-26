using System.Diagnostics;
using UnityEngine;
using AnkleSim.Core.DataModels;
using AnkleSim.Core.Resection;

namespace AnkleSim.Bridge.Resection
{
    public class ResectionEngine
    {
        private readonly SofaSimulation _sim;
        private Mesh _preCutMesh;
        private ResectionRecord _lastRecord;

        public ResectionEngine(SofaSimulation sim)
        {
            _sim = sim;
        }

        public ResectionRecord ExecuteCut(CutPlaneController controller, Mesh visualMesh)
        {
            var sw = Stopwatch.StartNew();

            var record = new ResectionRecord
            {
                targetBone = controller.TargetBone,
                cutPlane = controller.GetPlane(),
                resectionDepth = controller.DepthMm,
                coronalAngle = controller.CoronalAngleDeg,
                sagittalAngle = controller.SagittalAngleDeg,
            };

            // Store pre-cut state
            _preCutMesh = Object.Instantiate(visualMesh);
            record.preCutMesh = _preCutMesh;

            // Estimate volume to be removed
            Plane plane = controller.GetPlane();
            record.volumeRemoved = CutPlaneController.EstimateVolume(visualMesh, plane);

            // Visual cut: use EzySlice if available, else fall back to SOFA surface readback
            Mesh postCutVisual = TryVisualCut(visualMesh, plane);
            record.visualCutComplete = postCutVisual != null;

            // Physics cut via SOFA
            Vector3 planePoint = -plane.normal * plane.distance;
            int removedCount = _sim.ExecuteResection(planePoint, plane.normal,
                controller.TargetBone.ToString());
            record.removedTetrahedraCount = removedCount;
            record.sofaCutComplete = removedCount >= 0;

            // If visual cut failed, use SOFA surface mesh readback
            if (!record.visualCutComplete)
            {
                postCutVisual = _sim.GetSurfaceMesh();
                record.visualCutComplete = postCutVisual != null;
            }

            record.postCutVisualMesh = postCutVisual;

            sw.Stop();
            record.executionTimeMs = (float)sw.Elapsed.TotalMilliseconds;

            _lastRecord = record;
            return record;
        }

        public void UndoCut()
        {
            // Scene rebuild approach (D-06):
            // Caller is responsible for recreating the full scene via
            // DestroyScene + CreateScene + re-add bones/ligaments/tissue + FinalizeScene
            // This method restores the visual mesh state.
            _sim.DestroyScene();

            if (_preCutMesh != null)
            {
                Object.Destroy(_preCutMesh);
                _preCutMesh = null;
            }
            _lastRecord = null;
        }

        public Vector3[] PreviewCut(CutPlaneController controller, Mesh mesh)
        {
            Plane plane = controller.GetPlane();
            Vector3[] vertices = mesh.vertices;
            int[] triangles = mesh.triangles;

            // Find edges that cross the plane and compute intersection points
            var intersections = new System.Collections.Generic.List<Vector3>();

            for (int i = 0; i < triangles.Length; i += 3)
            {
                Vector3 v0 = vertices[triangles[i]];
                Vector3 v1 = vertices[triangles[i + 1]];
                Vector3 v2 = vertices[triangles[i + 2]];

                CheckEdge(plane, v0, v1, intersections);
                CheckEdge(plane, v1, v2, intersections);
                CheckEdge(plane, v2, v0, intersections);
            }

            return intersections.ToArray();
        }

        public bool CheckSafety(CutPlaneController controller, Vector3[] landmarks,
            float minDistanceMm)
        {
            Plane plane = controller.GetPlane();

            foreach (var landmark in landmarks)
            {
                float dist = Mathf.Abs(plane.GetDistanceToPoint(landmark));
                if (dist < minDistanceMm)
                    return false;
            }

            return true;
        }

        private Mesh TryVisualCut(Mesh mesh, Plane plane)
        {
            // EzySlice integration point:
            // When EzySlice is installed to Assets/Plugins/EzySlice/,
            // uncomment the following and add the assembly reference:
            //
            // var hull = mesh.Slice(planePoint, planeNormal);
            // if (hull != null)
            //     return hull.CreateUpperHull();
            //
            // For now, return null to fall back to SOFA surface readback.
            return null;
        }

        private static void CheckEdge(Plane plane, Vector3 a, Vector3 b,
            System.Collections.Generic.List<Vector3> intersections)
        {
            float da = plane.GetDistanceToPoint(a);
            float db = plane.GetDistanceToPoint(b);

            // Points on opposite sides of the plane
            if (da * db < 0f)
            {
                float t = da / (da - db);
                intersections.Add(Vector3.Lerp(a, b, t));
            }
        }
    }
}
