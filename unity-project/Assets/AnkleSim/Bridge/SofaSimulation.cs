using System;
using System.Runtime.InteropServices;
using UnityEngine;

namespace AnkleSim.Bridge
{
    public class SofaSimulation : IDisposable
    {
        private bool _initialized;
        private bool _disposed;

        public SofaBridgeVersion Version { get; private set; }

        public void Initialize(string pluginDir = null)
        {
            if (_initialized)
                throw new SofaBridgeException("Already initialized");

            Version = SofaNativeBridge.sofa_bridge_get_version();

            // Version handshake: major version must match expected
            const int expectedMajor = 0;
            if (Version.bridgeVersionMajor != expectedMajor)
            {
                throw new SofaBridgeException(
                    $"Bridge version mismatch: expected major={expectedMajor}, " +
                    $"got major={Version.bridgeVersionMajor}");
            }

            int rc = SofaNativeBridge.sofa_bridge_init(pluginDir);
            if (rc != 0)
                throw new SofaBridgeException(
                    $"sofa_bridge_init failed: {SofaNativeBridge.GetErrorString()}");

            _initialized = true;
        }

        public void CreateAnkleScene(float dt, float gravityZ)
        {
            CheckInitialized();
            int rc = SofaNativeBridge.sofa_scene_create_ankle(dt, gravityZ);
            if (rc != 0)
                throw new SofaBridgeException(
                    $"sofa_scene_create_ankle failed: {SofaNativeBridge.GetErrorString()}");
        }

        public void CreateScene(SofaSceneConfig config)
        {
            CheckInitialized();
            int rc = SofaNativeBridge.sofa_scene_create(ref config);
            if (rc != 0)
                throw new SofaBridgeException(
                    $"sofa_scene_create failed: {SofaNativeBridge.GetErrorString()}");
        }

        public void DestroyScene()
        {
            CheckInitialized();
            SofaNativeBridge.sofa_scene_destroy();
        }

        public void AddRigidBone(SofaRigidBoneConfig config)
        {
            CheckInitialized();
            int rc = SofaNativeBridge.sofa_add_rigid_bone(ref config);
            if (rc != 0)
                throw new SofaBridgeException(
                    $"sofa_add_rigid_bone failed: {SofaNativeBridge.GetErrorString()}");
        }

        public void AddLigament(SofaLigamentConfig config)
        {
            CheckInitialized();
            int rc = SofaNativeBridge.sofa_add_ligament(ref config);
            if (rc != 0)
                throw new SofaBridgeException(
                    $"sofa_add_ligament failed: {SofaNativeBridge.GetErrorString()}");
        }

        public void FinalizeScene()
        {
            CheckInitialized();
            int rc = SofaNativeBridge.sofa_scene_finalize();
            if (rc != 0)
                throw new SofaBridgeException(
                    $"sofa_scene_finalize failed: {SofaNativeBridge.GetErrorString()}");
        }

        public bool IsSceneReady()
        {
            return SofaNativeBridge.sofa_scene_is_ready() == 1;
        }

        public void Step(float dt)
        {
            CheckInitialized();
            int rc = SofaNativeBridge.sofa_step(dt);
            if (rc != 0)
                throw new SofaBridgeException(
                    $"sofa_step failed: {SofaNativeBridge.GetErrorString()}");
        }

        public void StepAsync(float dt)
        {
            CheckInitialized();
            int rc = SofaNativeBridge.sofa_step_async(dt);
            if (rc != 0)
                throw new SofaBridgeException(
                    $"sofa_step_async failed: {SofaNativeBridge.GetErrorString()}");
        }

        public bool IsStepComplete()
        {
            return SofaNativeBridge.sofa_step_async_is_complete() == 1;
        }

        public void WaitForStep()
        {
            SofaNativeBridge.sofa_step_async_wait();
        }

        public void ApplyTorque(float torqueNm, int axis)
        {
            CheckInitialized();
            int rc = SofaNativeBridge.sofa_apply_torque(torqueNm, axis);
            if (rc != 0)
                throw new SofaBridgeException(
                    $"sofa_apply_torque failed: {SofaNativeBridge.GetErrorString()}");
        }

        public SofaFrameSnapshot GetSnapshot()
        {
            CheckInitialized();
            var snap = new SofaFrameSnapshot();
            int rc = SofaNativeBridge.sofa_get_frame_snapshot(ref snap);
            if (rc != 0)
                throw new SofaBridgeException(
                    $"sofa_get_frame_snapshot failed: {SofaNativeBridge.GetErrorString()}");
            return snap;
        }

        // ---- Deformable tissue + Resection (Sprint 4) ----

        public void AddDeformableTissue(SofaDeformableConfig config)
        {
            CheckInitialized();
            int rc = SofaNativeBridge.sofa_add_deformable_tissue(ref config);
            if (rc != 0)
                throw new SofaBridgeException(
                    $"sofa_add_deformable_tissue failed: {SofaNativeBridge.GetErrorString()}");
        }

        public int ExecuteResection(Vector3 planePoint, Vector3 planeNormal, string boneName)
        {
            CheckInitialized();
            var cmd = SofaResectionCommand.Create(
                new float[] { planePoint.x, planePoint.y, planePoint.z },
                new float[] { planeNormal.x, planeNormal.y, planeNormal.z },
                boneName);
            try
            {
                int rc = SofaNativeBridge.sofa_execute_resection(ref cmd);
                if (rc != 0)
                    throw new SofaBridgeException(
                        $"sofa_execute_resection failed: {SofaNativeBridge.GetErrorString()}");
                return SofaNativeBridge.sofa_get_removed_element_count();
            }
            finally
            {
                SofaResectionCommand.FreeNativePtrs(ref cmd);
            }
        }

        public bool HasTopologyChanged()
        {
            return SofaNativeBridge.sofa_has_topology_changed() == 1;
        }

        public Mesh GetSurfaceMesh(int maxVertices = 10000, int maxTriangles = 20000)
        {
            CheckInitialized();
            var nativeMesh = SofaSurfaceMesh.Create(maxVertices, maxTriangles);
            try
            {
                int rc = SofaNativeBridge.sofa_get_surface_mesh(ref nativeMesh);
                if (rc != 0)
                    throw new SofaBridgeException(
                        $"sofa_get_surface_mesh failed: {SofaNativeBridge.GetErrorString()}");

                if (nativeMesh.vertexCount == 0)
                    return null;

                // Copy native data to managed arrays
                float[] verts = new float[nativeMesh.vertexCount * 3];
                Marshal.Copy(nativeMesh.vertices, verts, 0, verts.Length);

                int[] tris = new int[nativeMesh.triangleCount * 3];
                Marshal.Copy(nativeMesh.triangles, tris, 0, tris.Length);

                // Build Unity mesh
                var mesh = new Mesh();
                var vertices = new Vector3[nativeMesh.vertexCount];
                for (int i = 0; i < nativeMesh.vertexCount; i++)
                {
                    vertices[i] = new Vector3(verts[i * 3], verts[i * 3 + 1], verts[i * 3 + 2]);
                }
                mesh.vertices = vertices;
                mesh.triangles = tris;
                mesh.RecalculateNormals();
                return mesh;
            }
            finally
            {
                SofaSurfaceMesh.FreeNativePtrs(ref nativeMesh);
            }
        }

        public void Dispose()
        {
            if (_disposed) return;
            _disposed = true;

            if (_initialized)
            {
                SofaNativeBridge.sofa_bridge_shutdown();
                _initialized = false;
            }
        }

        private void CheckInitialized()
        {
            if (!_initialized)
                throw new SofaBridgeException("Not initialized — call Initialize() first");
            if (_disposed)
                throw new ObjectDisposedException(nameof(SofaSimulation));
        }
    }
}
