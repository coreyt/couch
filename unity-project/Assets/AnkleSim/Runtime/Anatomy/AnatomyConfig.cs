using UnityEngine;
using AnkleSim.Core.DataModels;

namespace AnkleSim.Runtime.Anatomy
{
    [CreateAssetMenu(fileName = "AnatomyConfig", menuName = "AnkleSim/Anatomy Config")]
    public class AnatomyConfig : ScriptableObject
    {
        public Mesh tibiaMesh;
        public Mesh talusMesh;
        public Mesh fibulaMesh;
        public Mesh calcaneusMesh;
        public LigamentConfig[] ligaments;

        public Mesh GetMeshForBone(BoneType boneType)
        {
            switch (boneType)
            {
                case BoneType.Tibia: return tibiaMesh;
                case BoneType.Talus: return talusMesh;
                case BoneType.Fibula: return fibulaMesh;
                case BoneType.Calcaneus: return calcaneusMesh;
                default: return null;
            }
        }
    }
}
