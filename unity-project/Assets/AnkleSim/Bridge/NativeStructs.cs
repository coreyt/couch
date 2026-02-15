using System;
using System.Runtime.InteropServices;

namespace AnkleSim.Bridge
{
    [StructLayout(LayoutKind.Sequential)]
    public struct SofaBridgeVersion
    {
        public int bridgeVersionMajor;
        public int bridgeVersionMinor;
        public int bridgeVersionPatch;
        public int sofaVersionMajor;
        public int sofaVersionMinor;
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct SofaRigidFrame
    {
        public double px, py, pz;
        public double qx, qy, qz, qw;
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct SofaLigamentConfig
    {
        public IntPtr name;

        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 3)]
        public double[] tibiaOffset;

        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 3)]
        public double[] talusOffset;

        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 3)]
        public double[] fixedAnchor;

        public int useFixedAnchor;

        public double stiffness;
        public double damping;
        public double restLength;
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct SofaFrameSnapshot
    {
        public SofaRigidFrame tibia;
        public SofaRigidFrame talus;

        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 3)]
        public double[] jointAnglesDeg;

        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 3)]
        public double[] ligamentForce;

        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 3)]
        public double[] ligamentTorque;

        public int stepCount;
    }
}
