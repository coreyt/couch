using UnityEngine;
using AnkleSim.Bridge;
using AnkleSim.Core.DataModels;

namespace AnkleSim.Runtime.Anatomy
{
    public class BoneVisualizer : MonoBehaviour
    {
        [SerializeField] private SofaBridgeComponent _bridge;
        [SerializeField] private AnatomyManager _anatomy;

        void LateUpdate()
        {
            if (_bridge == null || _anatomy == null) return;
            var snapshot = _bridge.LatestSnapshot;

            UpdateBoneTransform(BoneType.Talus, snapshot.talus);
            UpdateBoneTransform(BoneType.Calcaneus, snapshot.calcaneus);
            // Tibia is fixed — no update needed
        }

        private void UpdateBoneTransform(BoneType bone, SofaRigidFrame frame)
        {
            var go = _anatomy.GetBoneGameObject(bone);
            if (go == null) return;

            go.transform.localPosition = new Vector3(
                (float)frame.px, (float)frame.py, (float)frame.pz);
            go.transform.localRotation = new Quaternion(
                (float)frame.qx, (float)frame.qy, (float)frame.qz, (float)frame.qw);
        }
    }
}
