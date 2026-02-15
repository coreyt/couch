using UnityEngine;

namespace AnkleSim.Core.DataModels
{
    [System.Serializable]
    public class LigamentConfig
    {
        public string name;
        public Vector3 originPoint;
        public Vector3 insertionPoint;
        public float stiffness;
        public float restLength;
        public Color displayColor = Color.white;
    }
}
