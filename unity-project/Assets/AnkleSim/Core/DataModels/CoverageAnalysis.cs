using UnityEngine;

namespace AnkleSim.Core.DataModels
{
    [System.Serializable]
    public class CoverageAnalysis
    {
        public float tibialCoveragePercent;
        public float talarCoveragePercent;
        public bool hasOverhang;
        public Vector3[] overhangRegions;
        public float contactArea;
        public float maxGap;

        public bool HasOverhang()
        {
            hasOverhang = tibialCoveragePercent > 100f || talarCoveragePercent > 100f;
            return hasOverhang;
        }
    }
}
