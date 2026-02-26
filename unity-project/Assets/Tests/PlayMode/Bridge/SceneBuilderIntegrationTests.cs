using System;
using System.Collections;
using System.IO;
using System.Runtime.InteropServices;
using NUnit.Framework;
using UnityEngine;
using UnityEngine.TestTools;
using AnkleSim.Bridge;

namespace AnkleSim.Tests.PlayMode.Bridge
{
    public class SceneBuilderIntegrationTests
    {
        private bool _dllAvailable;

        private static string PluginDir =>
            Path.Combine(Application.dataPath, "Plugins", "x86_64");

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
                Assert.Ignore("SofaAnkleBridge DLL not available — skipping integration tests");
        }

        [TearDown]
        public void TearDown()
        {
            if (_dllAvailable)
            {
                try { SofaNativeBridge.sofa_bridge_shutdown(); }
                catch { /* best effort */ }
            }
        }

        // 1. DllLoads_WithoutError
        [UnityTest]
        public IEnumerator DllLoads_WithoutError()
        {
            var version = SofaNativeBridge.sofa_bridge_get_version();
            Assert.AreEqual(0, version.bridgeVersionMajor);
            Assert.GreaterOrEqual(version.bridgeVersionMinor, 3);
            yield return null;
        }

        // 2. VersionCheck_ReturnsValid
        [UnityTest]
        public IEnumerator VersionCheck_ReturnsValid()
        {
            var version = SofaNativeBridge.sofa_bridge_get_version();
            Assert.AreEqual(0, version.bridgeVersionMajor);
            Assert.AreEqual(3, version.bridgeVersionMinor);
            Assert.AreEqual(0, version.bridgeVersionPatch);
            Assert.Greater(version.sofaVersionMajor, 0, "SOFA version major should be positive");
            yield return null;
        }

        // 3. Init_ReturnsSuccess
        [UnityTest]
        public IEnumerator Init_ReturnsSuccess()
        {
            int rc = SofaNativeBridge.sofa_bridge_init(PluginDir);
            Assert.AreEqual(0, rc, $"Init failed: {SofaNativeBridge.GetErrorString()}");
            yield return null;
        }

        // 4. CreateScene_WithDefaults_Succeeds
        [UnityTest]
        public IEnumerator CreateScene_WithDefaults_Succeeds()
        {
            Assert.AreEqual(0, SofaNativeBridge.sofa_bridge_init(PluginDir),
                SofaNativeBridge.GetErrorString());

            var config = SofaSceneConfig.CreateDefault();
            int rc = SofaNativeBridge.sofa_scene_create(ref config);
            Assert.AreEqual(0, rc, $"Create scene failed: {SofaNativeBridge.GetErrorString()}");
            yield return null;
        }

        // 5. AddRigidBone_Succeeds
        [UnityTest]
        public IEnumerator AddRigidBone_Succeeds()
        {
            Assert.AreEqual(0, SofaNativeBridge.sofa_bridge_init(PluginDir));

            var sceneConfig = SofaSceneConfig.CreateDefault();
            Assert.AreEqual(0, SofaNativeBridge.sofa_scene_create(ref sceneConfig));

            var boneConfig = SofaRigidBoneConfig.Create("Tibia",
                new float[] { 0, 0, 0 },
                new float[] { 0, 0, 0, 1 },
                1.0f, true);

            try
            {
                int rc = SofaNativeBridge.sofa_add_rigid_bone(ref boneConfig);
                Assert.AreEqual(0, rc, $"Add bone failed: {SofaNativeBridge.GetErrorString()}");
            }
            finally
            {
                SofaRigidBoneConfig.FreeNativePtrs(ref boneConfig);
            }
            yield return null;
        }

        // 6. AddLigament_Succeeds
        [UnityTest]
        public IEnumerator AddLigament_Succeeds()
        {
            Assert.AreEqual(0, SofaNativeBridge.sofa_bridge_init(PluginDir));

            var sceneConfig = SofaSceneConfig.CreateDefault();
            Assert.AreEqual(0, SofaNativeBridge.sofa_scene_create(ref sceneConfig));

            var tibia = SofaRigidBoneConfig.Create("Tibia",
                new float[] { 0, 0, 0 }, new float[] { 0, 0, 0, 1 }, 1.0f, true);
            var talus = SofaRigidBoneConfig.Create("Talus",
                new float[] { 0, 0, -30 }, new float[] { 0, 0, 0, 1 }, 0.1f, false);

            SofaLigamentConfig[] ligs = null;
            try
            {
                Assert.AreEqual(0, SofaNativeBridge.sofa_add_rigid_bone(ref tibia));
                Assert.AreEqual(0, SofaNativeBridge.sofa_add_rigid_bone(ref talus));

                ligs = new[] {
                    SofaLigamentConfig.Create("ATFL",
                        new double[] { 15, 10, -14 },
                        new double[] { 12, 8, 11 },
                        70, 5, 0)
                };

                int rc = SofaNativeBridge.sofa_add_ligament(ref ligs[0]);
                Assert.AreEqual(0, rc, $"Add ligament failed: {SofaNativeBridge.GetErrorString()}");
            }
            finally
            {
                SofaRigidBoneConfig.FreeNativePtrs(ref tibia);
                SofaRigidBoneConfig.FreeNativePtrs(ref talus);
                if (ligs != null) SofaLigamentConfig.FreeNamePtrs(ligs);
            }
            yield return null;
        }

        // 7. FinalizeScene_Succeeds
        [UnityTest]
        public IEnumerator FinalizeScene_Succeeds()
        {
            Assert.AreEqual(0, SofaNativeBridge.sofa_bridge_init(PluginDir));

            var sceneConfig = SofaSceneConfig.CreateDefault();
            Assert.AreEqual(0, SofaNativeBridge.sofa_scene_create(ref sceneConfig));

            var tibia = SofaRigidBoneConfig.Create("Tibia",
                new float[] { 0, 0, 0 }, new float[] { 0, 0, 0, 1 }, 1.0f, true);
            try
            {
                Assert.AreEqual(0, SofaNativeBridge.sofa_add_rigid_bone(ref tibia));
            }
            finally
            {
                SofaRigidBoneConfig.FreeNativePtrs(ref tibia);
            }

            int rc = SofaNativeBridge.sofa_scene_finalize();
            Assert.AreEqual(0, rc, $"Finalize failed: {SofaNativeBridge.GetErrorString()}");
            Assert.AreEqual(1, SofaNativeBridge.sofa_scene_is_ready());
            yield return null;
        }

        // 8. StepAfterFinalize_Succeeds
        [UnityTest]
        public IEnumerator StepAfterFinalize_Succeeds()
        {
            Assert.AreEqual(0, SofaNativeBridge.sofa_bridge_init(PluginDir));

            var sceneConfig = SofaSceneConfig.CreateDefault();
            Assert.AreEqual(0, SofaNativeBridge.sofa_scene_create(ref sceneConfig));

            var tibia = SofaRigidBoneConfig.Create("Tibia",
                new float[] { 0, 0, 0 }, new float[] { 0, 0, 0, 1 }, 1.0f, true);
            var talus = SofaRigidBoneConfig.Create("Talus",
                new float[] { 0, 0, -30 }, new float[] { 0, 0, 0, 1 }, 0.1f, false);
            try
            {
                Assert.AreEqual(0, SofaNativeBridge.sofa_add_rigid_bone(ref tibia));
                Assert.AreEqual(0, SofaNativeBridge.sofa_add_rigid_bone(ref talus));
            }
            finally
            {
                SofaRigidBoneConfig.FreeNativePtrs(ref tibia);
                SofaRigidBoneConfig.FreeNativePtrs(ref talus);
            }

            Assert.AreEqual(0, SofaNativeBridge.sofa_scene_finalize());

            int rc = SofaNativeBridge.sofa_step(0.001f);
            Assert.AreEqual(0, rc, $"Step failed: {SofaNativeBridge.GetErrorString()}");
            yield return null;
        }

        // 9. GetSnapshot_AfterStep_HasValidData
        [UnityTest]
        public IEnumerator GetSnapshot_AfterStep_HasValidData()
        {
            Assert.AreEqual(0, SofaNativeBridge.sofa_bridge_init(PluginDir));

            var sceneConfig = SofaSceneConfig.CreateDefault();
            Assert.AreEqual(0, SofaNativeBridge.sofa_scene_create(ref sceneConfig));

            var tibia = SofaRigidBoneConfig.Create("Tibia",
                new float[] { 0, 0, 0 }, new float[] { 0, 0, 0, 1 }, 1.0f, true);
            var talus = SofaRigidBoneConfig.Create("Talus",
                new float[] { 0, 0, -30 }, new float[] { 0, 0, 0, 1 }, 0.1f, false);
            try
            {
                Assert.AreEqual(0, SofaNativeBridge.sofa_add_rigid_bone(ref tibia));
                Assert.AreEqual(0, SofaNativeBridge.sofa_add_rigid_bone(ref talus));
            }
            finally
            {
                SofaRigidBoneConfig.FreeNativePtrs(ref tibia);
                SofaRigidBoneConfig.FreeNativePtrs(ref talus);
            }

            Assert.AreEqual(0, SofaNativeBridge.sofa_scene_finalize());
            Assert.AreEqual(0, SofaNativeBridge.sofa_step(0.001f));

            var snap = new SofaFrameSnapshot();
            int rc = SofaNativeBridge.sofa_get_frame_snapshot(ref snap);
            Assert.AreEqual(0, rc, $"Snapshot failed: {SofaNativeBridge.GetErrorString()}");

            Assert.IsFalse(double.IsNaN(snap.tibia.px), "Tibia px should not be NaN");
            Assert.IsFalse(double.IsNaN(snap.talus.px), "Talus px should not be NaN");
            Assert.Greater(snap.stepCount, 0, "Step count should be > 0");
            Assert.AreEqual(0, snap.solverDiverged, "Solver should not have diverged");
            yield return null;
        }

        // 10. LegacyAnkleScene_StillWorks
        [UnityTest]
        public IEnumerator LegacyAnkleScene_StillWorks()
        {
            Assert.AreEqual(0, SofaNativeBridge.sofa_bridge_init(PluginDir));

            int rc = SofaNativeBridge.sofa_scene_create_ankle(0.001f, -9810f);
            Assert.AreEqual(0, rc, $"Legacy ankle scene failed: {SofaNativeBridge.GetErrorString()}");

            Assert.AreEqual(1, SofaNativeBridge.sofa_scene_is_ready());

            Assert.AreEqual(0, SofaNativeBridge.sofa_step(0.001f));

            var snap = new SofaFrameSnapshot();
            Assert.AreEqual(0, SofaNativeBridge.sofa_get_frame_snapshot(ref snap));
            Assert.IsFalse(double.IsNaN(snap.talus.pz));
            yield return null;
        }

        // 11. SceneViaNewAPI_ProducesValidROM
        [UnityTest]
        public IEnumerator SceneViaNewAPI_ProducesValidROM()
        {
            Assert.AreEqual(0, SofaNativeBridge.sofa_bridge_init(PluginDir));

            var sceneConfig = SofaSceneConfig.CreateDefault();
            Assert.AreEqual(0, SofaNativeBridge.sofa_scene_create(ref sceneConfig));

            var tibia = SofaRigidBoneConfig.Create("Tibia",
                new float[] { 0, 0, 0 }, new float[] { 0, 0, 0, 1 }, 1.0f, true);
            var talus = SofaRigidBoneConfig.Create("Talus",
                new float[] { 0, 0, -30 }, new float[] { 0, 0, 0, 1 }, 0.1f, false);

            SofaLigamentConfig[] ligs = null;
            try
            {
                Assert.AreEqual(0, SofaNativeBridge.sofa_add_rigid_bone(ref tibia));
                Assert.AreEqual(0, SofaNativeBridge.sofa_add_rigid_bone(ref talus));

                ligs = new[] {
                    SofaLigamentConfig.Create("ATFL",
                        new double[] { 15, 10, -14 }, new double[] { 12, 8, 11 }, 70, 5, 0),
                    SofaLigamentConfig.Create("PTFL",
                        new double[] { 15, -10, -14 }, new double[] { 12, -8, 11 }, 50, 5, 0),
                    SofaLigamentConfig.Create("Deltoid_ant",
                        new double[] { -12, 10, -14 }, new double[] { -10, 8, 11 }, 90, 5, 0),
                    SofaLigamentConfig.Create("Deltoid_post",
                        new double[] { -12, -10, -14 }, new double[] { -10, -8, 11 }, 90, 5, 0),
                };

                for (int i = 0; i < ligs.Length; i++)
                    Assert.AreEqual(0, SofaNativeBridge.sofa_add_ligament(ref ligs[i]),
                        SofaNativeBridge.GetErrorString());
            }
            finally
            {
                SofaRigidBoneConfig.FreeNativePtrs(ref tibia);
                SofaRigidBoneConfig.FreeNativePtrs(ref talus);
                if (ligs != null) SofaLigamentConfig.FreeNamePtrs(ligs);
            }

            Assert.AreEqual(0, SofaNativeBridge.sofa_scene_finalize());

            // Apply torque and step
            Assert.AreEqual(0, SofaNativeBridge.sofa_apply_torque(2.0f, 0));
            for (int i = 0; i < 50; i++)
                Assert.AreEqual(0, SofaNativeBridge.sofa_step(0.001f),
                    SofaNativeBridge.GetErrorString());

            var snap = new SofaFrameSnapshot();
            Assert.AreEqual(0, SofaNativeBridge.sofa_get_frame_snapshot(ref snap));

            // Should have some non-zero angle from the applied torque
            Assert.AreNotEqual(0.0, snap.jointAnglesDeg[0],
                "Sagittal angle should be non-zero after torque");
            yield return null;
        }

        // 12. AddCalcaneus_Succeeds
        [UnityTest]
        public IEnumerator AddCalcaneus_Succeeds()
        {
            Assert.AreEqual(0, SofaNativeBridge.sofa_bridge_init(PluginDir));

            var sceneConfig = SofaSceneConfig.CreateDefault();
            Assert.AreEqual(0, SofaNativeBridge.sofa_scene_create(ref sceneConfig));

            var tibia = SofaRigidBoneConfig.Create("Tibia",
                new float[] { 0, 0, 0 }, new float[] { 0, 0, 0, 1 }, 1.0f, true);
            var talus = SofaRigidBoneConfig.Create("Talus",
                new float[] { 0, 0, -30 }, new float[] { 0, 0, 0, 1 }, 0.1f, false);
            var calcaneus = SofaRigidBoneConfig.Create("Calcaneus",
                new float[] { 0, -30, -30 }, new float[] { 0, 0, 0, 1 }, 0.08f, false);

            try
            {
                Assert.AreEqual(0, SofaNativeBridge.sofa_add_rigid_bone(ref tibia),
                    SofaNativeBridge.GetErrorString());
                Assert.AreEqual(0, SofaNativeBridge.sofa_add_rigid_bone(ref talus),
                    SofaNativeBridge.GetErrorString());
                Assert.AreEqual(0, SofaNativeBridge.sofa_add_rigid_bone(ref calcaneus),
                    $"Add calcaneus failed: {SofaNativeBridge.GetErrorString()}");
            }
            finally
            {
                SofaRigidBoneConfig.FreeNativePtrs(ref tibia);
                SofaRigidBoneConfig.FreeNativePtrs(ref talus);
                SofaRigidBoneConfig.FreeNativePtrs(ref calcaneus);
            }

            Assert.AreEqual(0, SofaNativeBridge.sofa_scene_finalize(),
                SofaNativeBridge.GetErrorString());
            yield return null;
        }

        // 13. GetSnapshot_CalcaneusFramePopulated
        [UnityTest]
        public IEnumerator GetSnapshot_CalcaneusFramePopulated()
        {
            Assert.AreEqual(0, SofaNativeBridge.sofa_bridge_init(PluginDir));

            var sceneConfig = SofaSceneConfig.CreateDefault();
            Assert.AreEqual(0, SofaNativeBridge.sofa_scene_create(ref sceneConfig));

            var tibia = SofaRigidBoneConfig.Create("Tibia",
                new float[] { 0, 0, 0 }, new float[] { 0, 0, 0, 1 }, 1.0f, true);
            var talus = SofaRigidBoneConfig.Create("Talus",
                new float[] { 0, 0, -30 }, new float[] { 0, 0, 0, 1 }, 0.1f, false);
            var calcaneus = SofaRigidBoneConfig.Create("Calcaneus",
                new float[] { 0, -30, -30 }, new float[] { 0, 0, 0, 1 }, 0.08f, false);

            try
            {
                Assert.AreEqual(0, SofaNativeBridge.sofa_add_rigid_bone(ref tibia));
                Assert.AreEqual(0, SofaNativeBridge.sofa_add_rigid_bone(ref talus));
                Assert.AreEqual(0, SofaNativeBridge.sofa_add_rigid_bone(ref calcaneus));
            }
            finally
            {
                SofaRigidBoneConfig.FreeNativePtrs(ref tibia);
                SofaRigidBoneConfig.FreeNativePtrs(ref talus);
                SofaRigidBoneConfig.FreeNativePtrs(ref calcaneus);
            }

            Assert.AreEqual(0, SofaNativeBridge.sofa_scene_finalize());
            Assert.AreEqual(0, SofaNativeBridge.sofa_step(0.001f));

            var snap = new SofaFrameSnapshot();
            Assert.AreEqual(0, SofaNativeBridge.sofa_get_frame_snapshot(ref snap));

            Assert.IsFalse(double.IsNaN(snap.calcaneus.px), "Calcaneus px should not be NaN");
            Assert.IsFalse(double.IsNaN(snap.calcaneus.py), "Calcaneus py should not be NaN");
            Assert.IsFalse(double.IsNaN(snap.calcaneus.pz), "Calcaneus pz should not be NaN");
            yield return null;
        }

        // 14. SubtalarLigaments_ConstrainMotion
        [UnityTest]
        public IEnumerator SubtalarLigaments_ConstrainMotion()
        {
            Assert.AreEqual(0, SofaNativeBridge.sofa_bridge_init(PluginDir));

            var sceneConfig = SofaSceneConfig.CreateDefault();
            Assert.AreEqual(0, SofaNativeBridge.sofa_scene_create(ref sceneConfig));

            var tibia = SofaRigidBoneConfig.Create("Tibia",
                new float[] { 0, 0, 0 }, new float[] { 0, 0, 0, 1 }, 1.0f, true);
            var talus = SofaRigidBoneConfig.Create("Talus",
                new float[] { 0, 0, -30 }, new float[] { 0, 0, 0, 1 }, 0.1f, false);
            var calcaneus = SofaRigidBoneConfig.Create("Calcaneus",
                new float[] { 0, -30, -30 }, new float[] { 0, 0, 0, 1 }, 0.08f, false);

            SofaLigamentConfig[] ligs = null;
            try
            {
                Assert.AreEqual(0, SofaNativeBridge.sofa_add_rigid_bone(ref tibia));
                Assert.AreEqual(0, SofaNativeBridge.sofa_add_rigid_bone(ref talus));
                Assert.AreEqual(0, SofaNativeBridge.sofa_add_rigid_bone(ref calcaneus));

                // Tibiotalar ligaments
                ligs = new[]
                {
                    SofaLigamentConfig.Create("ATFL",
                        new double[] { 15, 10, -14 }, new double[] { 12, 8, 11 }, 70, 5, 0),
                    SofaLigamentConfig.Create("Deltoid_ant",
                        new double[] { -12, 10, -14 }, new double[] { -10, 8, 11 }, 90, 5, 0),
                    // Subtalar ligament
                    SofaLigamentConfig.Create("ITCL",
                        new double[] { 0, -5, -10 }, new double[] { 0, 5, 10 },
                        120, 5, 0, null, false, "Talus", "Calcaneus"),
                };

                for (int i = 0; i < ligs.Length; i++)
                    Assert.AreEqual(0, SofaNativeBridge.sofa_add_ligament(ref ligs[i]),
                        SofaNativeBridge.GetErrorString());
            }
            finally
            {
                SofaRigidBoneConfig.FreeNativePtrs(ref tibia);
                SofaRigidBoneConfig.FreeNativePtrs(ref talus);
                SofaRigidBoneConfig.FreeNativePtrs(ref calcaneus);
                if (ligs != null) SofaLigamentConfig.FreeNamePtrs(ligs);
            }

            Assert.AreEqual(0, SofaNativeBridge.sofa_scene_finalize());

            // Step multiple times
            for (int i = 0; i < 100; i++)
                Assert.AreEqual(0, SofaNativeBridge.sofa_step(0.001f),
                    SofaNativeBridge.GetErrorString());

            var snap = new SofaFrameSnapshot();
            Assert.AreEqual(0, SofaNativeBridge.sofa_get_frame_snapshot(ref snap));

            // Simulation should not have diverged
            Assert.AreEqual(0, snap.solverDiverged, "Solver should not have diverged");
            Assert.IsFalse(double.IsNaN(snap.calcaneus.pz), "Calcaneus should not be NaN after stepping");
            yield return null;
        }

        // 15. Shutdown_Reinit_CycleWorks
        [UnityTest]
        public IEnumerator Shutdown_Reinit_CycleWorks()
        {
            // First cycle
            Assert.AreEqual(0, SofaNativeBridge.sofa_bridge_init(PluginDir));
            Assert.AreEqual(0, SofaNativeBridge.sofa_scene_create_ankle(0.001f, -9810f));
            Assert.AreEqual(0, SofaNativeBridge.sofa_step(0.001f));
            SofaNativeBridge.sofa_bridge_shutdown();

            yield return null;

            // Second cycle
            Assert.AreEqual(0, SofaNativeBridge.sofa_bridge_init(PluginDir));
            Assert.AreEqual(0, SofaNativeBridge.sofa_scene_create_ankle(0.001f, -9810f));
            Assert.AreEqual(0, SofaNativeBridge.sofa_step(0.001f));

            var snap = new SofaFrameSnapshot();
            Assert.AreEqual(0, SofaNativeBridge.sofa_get_frame_snapshot(ref snap));
            Assert.IsFalse(double.IsNaN(snap.talus.pz));
            yield return null;
        }
    }
}
