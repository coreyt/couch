namespace AnkleSim.Core.Math
{
    /// <summary>
    /// Pure math utility replicating the C++ Euler decomposition (scene_builder.cpp:584-602)
    /// for use in C# without the native DLL.
    /// </summary>
    public static class AngleMath
    {
        private const double Rad2Deg = 180.0 / System.Math.PI;

        /// <summary>
        /// XYZ intrinsic Euler decomposition from quaternion components.
        /// Returns [sagittal, frontal, transverse] in degrees.
        /// Matches scene_builder.cpp computeJointAngles exactly.
        /// </summary>
        public static double[] EulerAnglesFromQuaternion(double qx, double qy, double qz, double qw)
        {
            // Roll (X-axis rotation) = sagittal
            double sinr_cosp = 2.0 * (qw * qx + qy * qz);
            double cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy);
            double roll = System.Math.Atan2(sinr_cosp, cosr_cosp);

            // Pitch (Y-axis rotation) = frontal
            double sinp = 2.0 * (qw * qy - qz * qx);
            sinp = System.Math.Max(-1.0, System.Math.Min(1.0, sinp));
            double pitch = System.Math.Asin(sinp);

            // Yaw (Z-axis rotation) = transverse
            double siny_cosp = 2.0 * (qw * qz + qx * qy);
            double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
            double yaw = System.Math.Atan2(siny_cosp, cosy_cosp);

            return new double[] { roll * Rad2Deg, pitch * Rad2Deg, yaw * Rad2Deg };
        }

        /// <summary>
        /// Computes relative quaternion: tibia_inv * talus.
        /// Returns (qx, qy, qz, qw) of the relative orientation.
        /// </summary>
        public static (double x, double y, double z, double w) RelativeOrientation(
            double tibiaQx, double tibiaQy, double tibiaQz, double tibiaQw,
            double talusQx, double talusQy, double talusQz, double talusQw)
        {
            // Conjugate of tibia quaternion (inverse for unit quaternions)
            double ix = -tibiaQx, iy = -tibiaQy, iz = -tibiaQz, iw = tibiaQw;

            // Hamilton product: tibia_inv * talus
            double rx = iw * talusQx + ix * talusQw + iy * talusQz - iz * talusQy;
            double ry = iw * talusQy - ix * talusQz + iy * talusQw + iz * talusQx;
            double rz = iw * talusQz + ix * talusQy - iy * talusQx + iz * talusQw;
            double rw = iw * talusQw - ix * talusQx - iy * talusQy - iz * talusQz;

            return (rx, ry, rz, rw);
        }

        /// <summary>Positive sagittal angle = dorsiflexion.</summary>
        public static float Dorsiflexion(double[] jointAnglesDeg)
        {
            return (float)System.Math.Max(0, jointAnglesDeg[0]);
        }

        /// <summary>Negative sagittal angle = plantarflexion (returned positive).</summary>
        public static float Plantarflexion(double[] jointAnglesDeg)
        {
            return (float)System.Math.Max(0, -jointAnglesDeg[0]);
        }

        /// <summary>Positive frontal angle = inversion.</summary>
        public static float Inversion(double[] jointAnglesDeg)
        {
            return (float)System.Math.Max(0, jointAnglesDeg[1]);
        }

        /// <summary>Negative frontal angle = eversion (returned positive).</summary>
        public static float Eversion(double[] jointAnglesDeg)
        {
            return (float)System.Math.Max(0, -jointAnglesDeg[1]);
        }

        /// <summary>Total sagittal arc = DF + PF.</summary>
        public static float TotalArc(float df, float pf)
        {
            return df + pf;
        }
    }
}
