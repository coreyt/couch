namespace AnkleSim.Core.DataModels
{
    [System.Serializable]
    public struct AlignmentWarning
    {
        public string parameter;
        public float value;
        public float threshold;
        public string message;

        public AlignmentWarning(string parameter, float value, float threshold, string message)
        {
            this.parameter = parameter;
            this.value = value;
            this.threshold = threshold;
            this.message = message;
        }
    }
}
