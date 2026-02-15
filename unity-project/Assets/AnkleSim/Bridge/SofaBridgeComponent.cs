using UnityEngine;

namespace AnkleSim.Bridge
{
    [DefaultExecutionOrder(-100)]
    public class SofaBridgeComponent : MonoBehaviour
    {
        [Tooltip("Path to SOFA plugin directory. Leave empty for auto-detect.")]
        public string pluginDir = "";

        [Tooltip("Simulation timestep in seconds.")]
        public float dt = 0.001f;

        [Tooltip("Gravity along Z axis in mm/s².")]
        public float gravityZ = -9810f;

        private SofaSimulation _sim;
        private bool _asyncStepPending;

        public SofaSimulation Simulation => _sim;
        public SofaFrameSnapshot LatestSnapshot { get; private set; } = CreateIdentitySnapshot();

        private void OnEnable()
        {
            try
            {
                _sim = new SofaSimulation();
                _sim.Initialize(string.IsNullOrEmpty(pluginDir) ? null : pluginDir);
                _sim.CreateAnkleScene(dt, gravityZ);
            }
            catch (System.Exception e)
            {
                Debug.LogError($"[SofaBridge] OnEnable failed: {e.Message}");
                if (_sim != null)
                {
                    _sim.Dispose();
                    _sim = null;
                }
            }
        }

        private void FixedUpdate()
        {
            if (_sim == null) return;

            try
            {
                if (_asyncStepPending)
                {
                    if (!_sim.IsStepComplete()) return;

                    LatestSnapshot = _sim.GetSnapshot();
                    _asyncStepPending = false;
                }

                _sim.StepAsync(dt);
                _asyncStepPending = true;
            }
            catch (SofaBridgeException e)
            {
                Debug.LogError($"[SofaBridge] FixedUpdate failed: {e.Message}");
            }
        }

        private void OnDisable()
        {
            if (_sim != null)
            {
                _sim.Dispose();
                _sim = null;
                _asyncStepPending = false;
            }
        }

        private static SofaFrameSnapshot CreateIdentitySnapshot()
        {
            var snap = new SofaFrameSnapshot
            {
                tibia = new SofaRigidFrame { qw = 1.0 },
                talus = new SofaRigidFrame { qw = 1.0 },
                jointAnglesDeg = new double[3],
                ligamentForce = new double[3],
                ligamentTorque = new double[3],
                stepCount = 0
            };
            return snap;
        }
    }
}
