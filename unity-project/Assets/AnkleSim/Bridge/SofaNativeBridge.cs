using System;
using System.Runtime.InteropServices;

namespace AnkleSim.Bridge
{
    public static class SofaNativeBridge
    {
        private const string LibName = "SofaAnkleBridge";

        // ---- Version ----

        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        public static extern SofaBridgeVersion sofa_bridge_get_version();

        // ---- Lifecycle ----

        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        public static extern int sofa_bridge_init(
            [MarshalAs(UnmanagedType.LPStr)] string pluginDir);

        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        public static extern int sofa_step(float dt);

        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        public static extern void sofa_bridge_shutdown();

        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        public static extern IntPtr sofa_bridge_get_error();

        // ---- Ankle scene ----

        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        public static extern int sofa_scene_create_ankle(float dt, float gravityZ);

        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        public static extern int sofa_scene_create_ankle_ex(
            float dt, float gravityZ,
            [In] SofaLigamentConfig[] ligaments, int numLigaments);

        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        public static extern int sofa_apply_torque(float torqueNm, int axis);

        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        public static extern int sofa_get_frame_snapshot(ref SofaFrameSnapshot snapshot);

        // ---- Async stepping ----

        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        public static extern int sofa_step_async(float dt);

        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        public static extern int sofa_step_async_is_complete();

        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        public static extern void sofa_step_async_wait();

        // ---- Helpers ----

        public static string GetErrorString()
        {
            IntPtr ptr = sofa_bridge_get_error();
            return ptr == IntPtr.Zero ? string.Empty : Marshal.PtrToStringAnsi(ptr);
        }
    }
}
