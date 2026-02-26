using System;
using UnityEngine;
using AnkleSim.Core.DataModels;

namespace AnkleSim.Runtime.Anatomy
{
    [Serializable]
    public struct BoneMeshEntry
    {
        public BoneType boneType;
        public Mesh mesh;
    }

    [CreateAssetMenu(fileName = "AnatomyConfig", menuName = "AnkleSim/Anatomy Config")]
    public class AnatomyConfig : ScriptableObject
    {
        public BoneMeshEntry[] boneMeshes;
        public LigamentConfig[] ligaments;

        public Mesh GetMeshForBone(BoneType boneType)
        {
            if (boneMeshes == null) return null;
            foreach (var entry in boneMeshes)
                if (entry.boneType == boneType)
                    return entry.mesh;
            return null;
        }
    }
}
