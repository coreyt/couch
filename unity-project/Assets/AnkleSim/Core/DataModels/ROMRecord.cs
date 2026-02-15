using UnityEngine;

namespace AnkleSim.Core.DataModels
{
    [System.Serializable]
    public class ROMRecord
    {
        public float maxDorsiflexion;
        public float maxPlantarflexion;
        public float totalSagittalArc;
        public float inversion;
        public float eversion;
        public float dorsiflexionTorque;
        public float plantarflexionTorque;
        public AnimationCurve torqueCurve;
        public double timestamp;

        public void RecalculateTotalArc()
        {
            totalSagittalArc = maxDorsiflexion + maxPlantarflexion;
        }
    }
}
