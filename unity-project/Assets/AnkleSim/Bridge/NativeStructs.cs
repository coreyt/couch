using System;
using System.Collections.Generic;
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

        // Bilinear fields (Sprint 2)
        public IntPtr boneAName;
        public IntPtr boneBName;
        public double toeStiffness;
        public double toeRegionStrain;

        /// <summary>
        /// Creates a SofaLigamentConfig with a managed name string.
        /// The returned IntPtrs must be freed via FreeNamePtrs() after use.
        /// </summary>
        public static SofaLigamentConfig Create(string ligamentName,
            double[] tibiaOff, double[] talusOff,
            double stiffness, double damping, double restLength,
            double[] fixedAnchor = null, bool useFixed = false,
            string boneAName = null, string boneBName = null,
            double toeStiffness = 0.0, double toeRegionStrain = 0.0)
        {
            return new SofaLigamentConfig
            {
                name = Marshal.StringToHGlobalAnsi(ligamentName),
                tibiaOffset = tibiaOff,
                talusOffset = talusOff,
                fixedAnchor = fixedAnchor ?? new double[3],
                useFixedAnchor = useFixed ? 1 : 0,
                stiffness = stiffness,
                damping = damping,
                restLength = restLength,
                boneAName = boneAName != null ? Marshal.StringToHGlobalAnsi(boneAName) : IntPtr.Zero,
                boneBName = boneBName != null ? Marshal.StringToHGlobalAnsi(boneBName) : IntPtr.Zero,
                toeStiffness = toeStiffness,
                toeRegionStrain = toeRegionStrain
            };
        }

        /// <summary>
        /// Frees all name IntPtrs allocated by Create().
        /// </summary>
        public static void FreeNamePtrs(SofaLigamentConfig[] configs)
        {
            for (int i = 0; i < configs.Length; i++)
            {
                if (configs[i].name != IntPtr.Zero)
                {
                    Marshal.FreeHGlobal(configs[i].name);
                    configs[i].name = IntPtr.Zero;
                }
                if (configs[i].boneAName != IntPtr.Zero)
                {
                    Marshal.FreeHGlobal(configs[i].boneAName);
                    configs[i].boneAName = IntPtr.Zero;
                }
                if (configs[i].boneBName != IntPtr.Zero)
                {
                    Marshal.FreeHGlobal(configs[i].boneBName);
                    configs[i].boneBName = IntPtr.Zero;
                }
            }
        }
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct SofaFrameSnapshot
    {
        public SofaRigidFrame tibia;
        public SofaRigidFrame talus;
        public SofaRigidFrame calcaneus;

        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 3)]
        public double[] jointAnglesDeg;

        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 3)]
        public double[] ligamentForce;

        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 3)]
        public double[] ligamentTorque;

        public float stepTimeMs;
        public int solverDiverged;
        public int stepCount;
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct SofaSceneConfig
    {
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 3)]
        public float[] gravity;

        public float timestep;
        public int constraintIterations;
        public float constraintTolerance;
        public float rayleighStiffness;
        public float rayleighMass;
        public float alarmDistance;
        public float contactDistance;
        public float frictionCoefficient;

        public static SofaSceneConfig CreateDefault()
        {
            return new SofaSceneConfig
            {
                gravity = new float[] { 0f, 0f, -9810f },
                timestep = 0.001f,
                constraintIterations = 200,
                constraintTolerance = 1e-4f,
                rayleighStiffness = 0.1f,
                rayleighMass = 1.0f,
                alarmDistance = 8.0f,
                contactDistance = 3.0f,
                frictionCoefficient = 0.0f
            };
        }
    }

    // ---- Deformable tissue + Resection (Sprint 4) ----

    [StructLayout(LayoutKind.Sequential)]
    public struct SofaDeformableConfig
    {
        public IntPtr name;
        public IntPtr parentBone;
        public IntPtr vertices;     // float* flattened [x,y,z, ...]
        public int vertexCount;
        public IntPtr tetrahedra;   // int* flattened [i0,i1,i2,i3, ...]
        public int tetraCount;
        public float youngModulus;
        public float poissonRatio;
        public float massDensity;

        public static SofaDeformableConfig Create(string tissueName,
            float[] verts, int[] tetras,
            float youngModulus, float poissonRatio, float massDensity,
            string parentBone = null)
        {
            var cfg = new SofaDeformableConfig
            {
                name = Marshal.StringToHGlobalAnsi(tissueName),
                parentBone = parentBone != null ? Marshal.StringToHGlobalAnsi(parentBone) : IntPtr.Zero,
                vertexCount = verts.Length / 3,
                tetraCount = tetras.Length / 4,
                youngModulus = youngModulus,
                poissonRatio = poissonRatio,
                massDensity = massDensity
            };

            cfg.vertices = Marshal.AllocHGlobal(verts.Length * sizeof(float));
            Marshal.Copy(verts, 0, cfg.vertices, verts.Length);

            cfg.tetrahedra = Marshal.AllocHGlobal(tetras.Length * sizeof(int));
            Marshal.Copy(tetras, 0, cfg.tetrahedra, tetras.Length);

            return cfg;
        }

        public static void FreeNativePtrs(ref SofaDeformableConfig cfg)
        {
            if (cfg.name != IntPtr.Zero)
            {
                Marshal.FreeHGlobal(cfg.name);
                cfg.name = IntPtr.Zero;
            }
            if (cfg.parentBone != IntPtr.Zero)
            {
                Marshal.FreeHGlobal(cfg.parentBone);
                cfg.parentBone = IntPtr.Zero;
            }
            if (cfg.vertices != IntPtr.Zero)
            {
                Marshal.FreeHGlobal(cfg.vertices);
                cfg.vertices = IntPtr.Zero;
            }
            if (cfg.tetrahedra != IntPtr.Zero)
            {
                Marshal.FreeHGlobal(cfg.tetrahedra);
                cfg.tetrahedra = IntPtr.Zero;
            }
        }
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct SofaResectionCommand
    {
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 3)]
        public float[] planePoint;

        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 3)]
        public float[] planeNormal;

        public IntPtr boneName;

        public static SofaResectionCommand Create(float[] planePoint, float[] planeNormal,
            string boneName = null)
        {
            return new SofaResectionCommand
            {
                planePoint = planePoint,
                planeNormal = planeNormal,
                boneName = boneName != null ? Marshal.StringToHGlobalAnsi(boneName) : IntPtr.Zero
            };
        }

        public static void FreeNativePtrs(ref SofaResectionCommand cmd)
        {
            if (cmd.boneName != IntPtr.Zero)
            {
                Marshal.FreeHGlobal(cmd.boneName);
                cmd.boneName = IntPtr.Zero;
            }
        }
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct SofaSurfaceMesh
    {
        public IntPtr vertices;     // float* caller-allocated
        public IntPtr triangles;    // int* caller-allocated
        public int vertexCount;     // in/out: capacity in, actual out
        public int triangleCount;   // in/out: capacity in, actual out

        public static SofaSurfaceMesh Create(int maxVertices, int maxTriangles)
        {
            var mesh = new SofaSurfaceMesh
            {
                vertexCount = maxVertices,
                triangleCount = maxTriangles
            };

            mesh.vertices = Marshal.AllocHGlobal(maxVertices * 3 * sizeof(float));
            mesh.triangles = Marshal.AllocHGlobal(maxTriangles * 3 * sizeof(int));

            return mesh;
        }

        public static void FreeNativePtrs(ref SofaSurfaceMesh mesh)
        {
            if (mesh.vertices != IntPtr.Zero)
            {
                Marshal.FreeHGlobal(mesh.vertices);
                mesh.vertices = IntPtr.Zero;
            }
            if (mesh.triangles != IntPtr.Zero)
            {
                Marshal.FreeHGlobal(mesh.triangles);
                mesh.triangles = IntPtr.Zero;
            }
        }
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct SofaRigidBoneConfig
    {
        public IntPtr name;
        public IntPtr collisionVertices;
        public int collisionVertexCount;
        public IntPtr collisionTriangles;
        public int collisionTriangleCount;

        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 3)]
        public float[] position;

        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 4)]
        public float[] orientation;

        public float mass;
        public int isFixed;

        public static SofaRigidBoneConfig Create(string boneName,
            float[] position, float[] orientation,
            float mass, bool isFixed,
            float[] collisionVertices = null, int[] collisionTriangles = null)
        {
            var cfg = new SofaRigidBoneConfig
            {
                name = Marshal.StringToHGlobalAnsi(boneName),
                position = position,
                orientation = orientation,
                mass = mass,
                isFixed = isFixed ? 1 : 0,
                collisionVertices = IntPtr.Zero,
                collisionVertexCount = 0,
                collisionTriangles = IntPtr.Zero,
                collisionTriangleCount = 0
            };

            if (collisionVertices != null && collisionVertices.Length > 0)
            {
                cfg.collisionVertexCount = collisionVertices.Length / 3;
                cfg.collisionVertices = Marshal.AllocHGlobal(collisionVertices.Length * sizeof(float));
                Marshal.Copy(collisionVertices, 0, cfg.collisionVertices, collisionVertices.Length);
            }

            if (collisionTriangles != null && collisionTriangles.Length > 0)
            {
                cfg.collisionTriangleCount = collisionTriangles.Length / 3;
                cfg.collisionTriangles = Marshal.AllocHGlobal(collisionTriangles.Length * sizeof(int));
                Marshal.Copy(collisionTriangles, 0, cfg.collisionTriangles, collisionTriangles.Length);
            }

            return cfg;
        }

        public static void FreeNativePtrs(ref SofaRigidBoneConfig cfg)
        {
            if (cfg.name != IntPtr.Zero)
            {
                Marshal.FreeHGlobal(cfg.name);
                cfg.name = IntPtr.Zero;
            }
            if (cfg.collisionVertices != IntPtr.Zero)
            {
                Marshal.FreeHGlobal(cfg.collisionVertices);
                cfg.collisionVertices = IntPtr.Zero;
            }
            if (cfg.collisionTriangles != IntPtr.Zero)
            {
                Marshal.FreeHGlobal(cfg.collisionTriangles);
                cfg.collisionTriangles = IntPtr.Zero;
            }
        }
    }
}
