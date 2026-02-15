using UnityEngine;

namespace AnkleSim.Core.DataModels
{
    [System.Serializable]
    public class LigamentConfig
    {
        public string name;
        public Vector3 originPoint;
        public Vector3 insertionPoint;
        public Vector3 fixedAnchor;
        public bool useFixedAnchor;
        public float stiffness;
        public float damping;
        public float restLength;
        public Color displayColor = Color.white;
    }
}
