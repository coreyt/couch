using System;

namespace SofaUnityBridge
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
