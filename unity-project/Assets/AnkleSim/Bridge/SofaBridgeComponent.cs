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
        public SofaFrameSnapshot LatestSnapshot { get; private set; }

        private void OnEnable()
        {
            _sim = new SofaSimulation();
            _sim.Initialize(string.IsNullOrEmpty(pluginDir) ? null : pluginDir);
            _sim.CreateAnkleScene(dt, gravityZ);
        }

        private void FixedUpdate()
        {
            if (_sim == null) return;

            if (_asyncStepPending)
            {
                if (!_sim.IsStepComplete()) return;

                LatestSnapshot = _sim.GetSnapshot();
                _asyncStepPending = false;
            }

            _sim.StepAsync(dt);
            _asyncStepPending = true;
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
    }
}
