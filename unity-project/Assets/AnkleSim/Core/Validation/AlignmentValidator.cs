using AnkleSim.Core.DataModels;

namespace AnkleSim.Core.Validation
{
    public static class AlignmentValidator
    {
        public const float TibiotalarAngleThreshold = 10f;
        public const float ADTAMin = 86f;
        public const float ADTAMax = 92f;
        public const float PosteriorSlopeThreshold = 5f;
        public const float TibiotalarCongruenceThreshold = 2f;

        public static void Validate(AlignmentMetrics metrics)
        {
            metrics.warnings.Clear();
            metrics.isAcceptable = true;

            if (metrics.tibiotalarAngle >= TibiotalarAngleThreshold)
            {
                metrics.warnings.Add(new AlignmentWarning(
                    "Tibiotalar Angle",
                    metrics.tibiotalarAngle,
                    TibiotalarAngleThreshold,
                    $"Tibiotalar angle {metrics.tibiotalarAngle:F1}° exceeds threshold of {TibiotalarAngleThreshold:F1}°"));
                metrics.isAcceptable = false;
            }

            if (metrics.anteriorDistalTibialAngle < ADTAMin || metrics.anteriorDistalTibialAngle > ADTAMax)
            {
                metrics.warnings.Add(new AlignmentWarning(
                    "ADTA",
                    metrics.anteriorDistalTibialAngle,
                    metrics.anteriorDistalTibialAngle < ADTAMin ? ADTAMin : ADTAMax,
                    $"ADTA {metrics.anteriorDistalTibialAngle:F1}° outside acceptable range [{ADTAMin:F0}°-{ADTAMax:F0}°]"));
                metrics.isAcceptable = false;
            }

            if (metrics.posteriorSlope >= PosteriorSlopeThreshold)
            {
                metrics.warnings.Add(new AlignmentWarning(
                    "Posterior Slope",
                    metrics.posteriorSlope,
                    PosteriorSlopeThreshold,
                    $"Posterior slope {metrics.posteriorSlope:F1}° exceeds threshold of {PosteriorSlopeThreshold:F1}°"));
                metrics.isAcceptable = false;
            }

            if (metrics.tibiotalarCongruence >= TibiotalarCongruenceThreshold)
            {
                metrics.warnings.Add(new AlignmentWarning(
                    "Tibiotalar Congruence",
                    metrics.tibiotalarCongruence,
                    TibiotalarCongruenceThreshold,
                    $"Tibiotalar congruence {metrics.tibiotalarCongruence:F1}° exceeds threshold of {TibiotalarCongruenceThreshold:F1}°"));
                metrics.isAcceptable = false;
            }
        }
    }
}
