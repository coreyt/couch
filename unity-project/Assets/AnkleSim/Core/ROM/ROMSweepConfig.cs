namespace AnkleSim.Core.ROM
{
    [System.Serializable]
    public class ROMSweepConfig
    {
        public float torqueNm = 5.0f;
        public int stepsPerDirection = 1000;
        public float dt = 0.001f;
        public float gravityZ = 0f;
        public int sagittalAxis = 0;
    }
}
