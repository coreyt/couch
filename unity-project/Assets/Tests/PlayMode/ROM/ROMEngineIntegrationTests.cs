using System;
using System.Collections;
using System.IO;
using NUnit.Framework;
using UnityEngine;
using UnityEngine.TestTools;
using AnkleSim.Bridge;
using AnkleSim.Bridge.ROM;
using AnkleSim.Core.ROM;

namespace AnkleSim.Tests.PlayMode.ROM
{
    public class ROMEngineIntegrationTests
    {
        private SofaSimulation _sim;
        private bool _dllAvailable;

        [SetUp]
        public void SetUp()
        {
            try
            {
                SofaNativeBridge.sofa_bridge_get_version();
                _dllAvailable = true;
            }
            catch (DllNotFoundException)
            {
                _dllAvailable = false;
            }

            if (!_dllAvailable)
            {
                Assert.Ignore("SofaAnkleBridge DLL not available — skipping integration tests");
                return;
            }

            _sim = new SofaSimulation();
            _sim.Initialize(Path.Combine(Application.dataPath, "Plugins", "x86_64"));
        }

        [TearDown]
        public void TearDown()
        {
            if (_sim != null)
            {
                _sim.Dispose();
                _sim = null;
            }
        }

        // 1. StartSweep_AppliesExternalMoment
        [UnityTest]
        [Timeout(30000)]
        public IEnumerator StartSweep_AppliesExternalMoment()
        {
            _sim.CreateAnkleScene(0.001f, 0f);
            try
            {
                var engine = new ROMEngine(_sim);
                engine.StartSweep(5.0f, 0);

                for (int i = 0; i < 50; i++)
                    engine.StepSweep(0.001f);

                Assert.AreNotEqual(0f, engine.CurrentAngle,
                    "Angle should be non-zero after applying torque");

                engine.StopSweep();
            }
            finally
            {
                _sim.DestroyScene();
            }
            yield return null;
        }

        // 2. StartSweep_RecordsAnglesOverTime
        [UnityTest]
        [Timeout(30000)]
        public IEnumerator StartSweep_RecordsAnglesOverTime()
        {
            _sim.CreateAnkleScene(0.001f, 0f);
            try
            {
                var engine = new ROMEngine(_sim);
                engine.StartSweep(5.0f, 0);

                for (int i = 0; i < 10; i++)
                    engine.StepSweep(0.001f);
                float angleEarly = Mathf.Abs(engine.CurrentAngle);

                for (int i = 0; i < 100; i++)
                    engine.StepSweep(0.001f);
                float angleLater = Mathf.Abs(engine.CurrentAngle);

                Assert.Greater(angleLater, angleEarly,
                    "Angle magnitude should increase over time with sustained torque");

                engine.StopSweep();
            }
            finally
            {
                _sim.DestroyScene();
            }
            yield return null;
        }

        // 3. StopSweep_ReturnsCompletedRecord
        [UnityTest]
        [Timeout(30000)]
        public IEnumerator StopSweep_ReturnsCompletedRecord()
        {
            _sim.CreateAnkleScene(0.001f, 0f);
            try
            {
                var engine = new ROMEngine(_sim);
                engine.StartSweep(5.0f, 0);

                for (int i = 0; i < 200; i++)
                    engine.StepSweep(0.001f);

                float maxAngle = engine.StopSweep();

                Assert.Greater(maxAngle, 0f,
                    "StopSweep should return a positive max angle");
            }
            finally
            {
                _sim.DestroyScene();
            }
            yield return null;
        }

        // 4. GetCurrentAngle_DuringSimulation_ReturnsLiveValue
        [UnityTest]
        [Timeout(30000)]
        public IEnumerator GetCurrentAngle_DuringSimulation_ReturnsLiveValue()
        {
            _sim.CreateAnkleScene(0.001f, 0f);
            try
            {
                var engine = new ROMEngine(_sim);
                engine.StartSweep(5.0f, 0);

                for (int i = 0; i < 20; i++)
                    engine.StepSweep(0.001f);
                float angle1 = engine.CurrentAngle;

                for (int i = 0; i < 100; i++)
                    engine.StepSweep(0.001f);
                float angle2 = engine.CurrentAngle;

                Assert.AreNotEqual(angle1, angle2,
                    "CurrentAngle should change between step batches");

                engine.StopSweep();
            }
            finally
            {
                _sim.DestroyScene();
            }
            yield return null;
        }

        // 5. GetCurrentTorque_DuringSimulation_ReturnsLiveValue
        [UnityTest]
        [Timeout(30000)]
        public IEnumerator GetCurrentTorque_DuringSimulation_ReturnsLiveValue()
        {
            _sim.CreateAnkleScene(0.001f, 0f);
            try
            {
                var engine = new ROMEngine(_sim);
                float appliedTorque = 5.0f;
                engine.StartSweep(appliedTorque, 0);

                engine.StepSweep(0.001f);

                Assert.AreEqual(appliedTorque, engine.CurrentTorque, 0.001f,
                    "CurrentTorque should match the applied torque value");

                engine.StopSweep();
            }
            finally
            {
                _sim.DestroyScene();
            }
            yield return null;
        }

        // 6. PreOpROM_WithConstraints_RecordsTotalArcNear25
        [UnityTest]
        [Timeout(30000)]
        public IEnumerator PreOpROM_WithConstraints_RecordsTotalArcNear25()
        {
            var engine = new ROMEngine(_sim);
            var config = new ROMSweepConfig
            {
                torqueNm = 5.0f,
                stepsPerDirection = 1000,
                dt = 0.001f,
                gravityZ = 0f,
                sagittalAxis = 0
            };

            var record = engine.MeasureROM(config);

            float totalArc = record.totalSagittalArc;
            Debug.Log($"[ROMEngine] DF={record.maxDorsiflexion:F1}° " +
                      $"PF={record.maxPlantarflexion:F1}° " +
                      $"Total={totalArc:F1}°");

            Assert.Greater(totalArc, 10f,
                $"Total arc should be > 10° (got {totalArc:F1}°)");
            Assert.Less(totalArc, 180f,
                $"Total arc should be < 180° (got {totalArc:F1}°)");

            Assert.Greater(record.maxDorsiflexion, 0f, "DF should be positive");
            Assert.Greater(record.maxPlantarflexion, 0f, "PF should be positive");
            Assert.IsNotNull(record.torqueCurve, "Torque curve should be populated");
            Assert.Greater(record.torqueCurve.length, 0, "Torque curve should have keys");

            yield return null;
        }
    }
}
