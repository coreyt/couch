using UnityEngine;

namespace AnkleSim.Core.DataModels
{
    [System.Serializable]
    public class ResectionRecord
    {
        public BoneType targetBone;
        public Plane cutPlane;
        public float resectionDepth;
        public float coronalAngle;
        public float sagittalAngle;
        public float volumeRemoved;
        public Mesh preCutMesh;
    }
}
