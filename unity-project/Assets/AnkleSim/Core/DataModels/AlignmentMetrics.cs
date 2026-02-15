using System.Collections.Generic;

namespace AnkleSim.Core.DataModels
{
    [System.Serializable]
    public class AlignmentMetrics
    {
        public float tibiotalarAngle;
        public float anteriorDistalTibialAngle;
        public float posteriorSlope;
        public float tibiotalarCongruence;
        public bool isAcceptable;
        public List<AlignmentWarning> warnings = new List<AlignmentWarning>();
    }
}
