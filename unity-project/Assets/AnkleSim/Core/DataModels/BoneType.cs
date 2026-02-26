namespace AnkleSim.Core.DataModels
{
    public enum BoneType
    {
        // Leg (2)
        Tibia, Fibula,
        // Hindfoot (2)
        Talus, Calcaneus,
        // Midfoot (5)
        Navicular, Cuboid,
        MedialCuneiform, IntermediateCuneiform, LateralCuneiform,
        // Forefoot (5)
        Metatarsal1, Metatarsal2, Metatarsal3, Metatarsal4, Metatarsal5,
        // Phalanges - big toe (2)
        ProximalPhalanx1, DistalPhalanx1,
        // Phalanges - toes 2-5 (12)
        ProximalPhalanx2, MiddlePhalanx2, DistalPhalanx2,
        ProximalPhalanx3, MiddlePhalanx3, DistalPhalanx3,
        ProximalPhalanx4, MiddlePhalanx4, DistalPhalanx4,
        ProximalPhalanx5, MiddlePhalanx5, DistalPhalanx5
    }

    public static class BoneTypeExtensions
    {
        /// <summary>
        /// Returns true for bones that participate in SOFA simulation.
        /// </summary>
        public static bool IsSimulated(this BoneType bone)
            => bone == BoneType.Tibia || bone == BoneType.Talus || bone == BoneType.Calcaneus;
    }
}
