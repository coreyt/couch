namespace AnkleSim.Core.DataModels
{
    [System.Serializable]
    public class ROMComparison
    {
        public ROMRecord preOp;
        public ROMRecord postOp;
        public float dfImprovement;
        public float pfImprovement;
        public float totalArcImprovement;

        public static ROMComparison Create(ROMRecord pre, ROMRecord post)
        {
            return new ROMComparison
            {
                preOp = pre,
                postOp = post,
                dfImprovement = post.maxDorsiflexion - pre.maxDorsiflexion,
                pfImprovement = post.maxPlantarflexion - pre.maxPlantarflexion,
                totalArcImprovement = post.totalSagittalArc - pre.totalSagittalArc
            };
        }
    }
}
