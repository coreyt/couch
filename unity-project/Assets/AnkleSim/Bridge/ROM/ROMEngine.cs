using System;
using System.Collections.Generic;
using UnityEngine;
using AnkleSim.Core.DataModels;
using AnkleSim.Core.ROM;

namespace AnkleSim.Bridge.ROM
{
    /// <summary>
    /// Orchestrates torque-driven ROM sweeps via the native SOFA bridge.
    /// Calls Step() + GetSnapshot() in a loop, recording angle samples.
    /// </summary>
    public class ROMEngine
    {
        private readonly SofaSimulation _sim;
        private bool _sweeping;
        private float _currentAngle;
        private float _currentTorque;
        private float _maxAngle;
        private int _stepCount;
        private List<Keyframe> _samples;

        public ROMEngine(SofaSimulation sim)
        {
            _sim = sim;
            _samples = new List<Keyframe>();
        }

        public float CurrentAngle => _currentAngle;
        public float CurrentTorque => _currentTorque;
        public bool IsSweeping => _sweeping;

        /// <summary>
        /// Applies torque and begins recording angle samples.
        /// Requires a scene to already be created and finalized.
        /// </summary>
        public void StartSweep(float torqueNm, int axis)
        {
            _sim.ApplyTorque(torqueNm, axis);
            _sweeping = true;
            _currentTorque = torqueNm;
            _maxAngle = 0f;
            _stepCount = 0;
            _samples.Clear();
        }

        /// <summary>
        /// Advances one simulation step and records the angle.
        /// Returns true when sweep is complete (never auto-completes; caller manages loop).
        /// </summary>
        public bool StepSweep(float dt)
        {
            if (!_sweeping) return true;

            _sim.Step(dt);
            _stepCount++;

            var snap = _sim.GetSnapshot();
            _currentAngle = (float)snap.jointAnglesDeg[0];
            float absAngle = Mathf.Abs(_currentAngle);
            if (absAngle > _maxAngle) _maxAngle = absAngle;

            _samples.Add(new Keyframe(_stepCount * dt, _currentAngle));

            return false;
        }

        /// <summary>
        /// Stops recording and returns the max absolute angle observed.
        /// </summary>
        public float StopSweep()
        {
            _sweeping = false;
            float result = _maxAngle;
            _currentTorque = 0f;
            return result;
        }

        /// <summary>
        /// Blocking full ROM measurement: creates scenes, runs DF and PF sweeps,
        /// returns a populated ROMRecord. Manages scene lifecycle internally.
        /// </summary>
        public ROMRecord MeasureROM(ROMSweepConfig config)
        {
            var dfSamples = new List<Keyframe>();
            var pfSamples = new List<Keyframe>();

            // --- Dorsiflexion sweep (positive torque) ---
            _sim.CreateAnkleScene(config.dt, config.gravityZ);
            _sim.ApplyTorque(config.torqueNm, config.sagittalAxis);

            float maxDF = 0f;
            for (int i = 0; i < config.stepsPerDirection; i++)
            {
                _sim.Step(config.dt);
                var snap = _sim.GetSnapshot();
                float angle = (float)snap.jointAnglesDeg[config.sagittalAxis];
                float df = Mathf.Max(0f, angle);
                if (df > maxDF) maxDF = df;
                dfSamples.Add(new Keyframe(i * config.dt, angle));
            }
            _sim.DestroyScene();

            // --- Plantarflexion sweep (negative torque, fresh scene) ---
            _sim.CreateAnkleScene(config.dt, config.gravityZ);
            _sim.ApplyTorque(-config.torqueNm, config.sagittalAxis);

            float maxPF = 0f;
            for (int i = 0; i < config.stepsPerDirection; i++)
            {
                _sim.Step(config.dt);
                var snap = _sim.GetSnapshot();
                float angle = (float)snap.jointAnglesDeg[config.sagittalAxis];
                float pf = Mathf.Max(0f, -angle);
                if (pf > maxPF) maxPF = pf;
                pfSamples.Add(new Keyframe((config.stepsPerDirection + i) * config.dt, angle));
            }
            _sim.DestroyScene();

            // --- Build ROMRecord ---
            var allSamples = new List<Keyframe>();
            allSamples.AddRange(dfSamples);
            allSamples.AddRange(pfSamples);

            var record = new ROMRecord
            {
                maxDorsiflexion = maxDF,
                maxPlantarflexion = maxPF,
                totalSagittalArc = maxDF + maxPF,
                dorsiflexionTorque = config.torqueNm,
                plantarflexionTorque = config.torqueNm,
                torqueCurve = new AnimationCurve(allSamples.ToArray()),
                timestamp = DateTimeOffset.UtcNow.ToUnixTimeMilliseconds() / 1000.0,
            };

            return record;
        }
    }
}
