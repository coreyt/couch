using UnityEngine;

namespace AnkleSim.Runtime.Anatomy
{
    [CreateAssetMenu(fileName = "BoneMaterialConfig", menuName = "AnkleSim/Bone Material Config")]
    public class BoneMaterialConfig : ScriptableObject
    {
        [Header("Simulated Bones (tibia, talus, calcaneus)")]
        public Color simulatedColor = new Color(0.95f, 0.92f, 0.85f, 1.0f);

        [Header("Context Bones (all others)")]
        public Color contextColor = new Color(0.8f, 0.78f, 0.72f, 0.5f);

        public Material CreateSimulatedBoneMaterial()
        {
            var shader = FindBoneShader();
            if (shader == null) return null;
            var mat = new Material(shader);
            mat.color = simulatedColor;
            return mat;
        }

        public Material CreateContextBoneMaterial()
        {
            var shader = FindBoneShader();
            if (shader == null) return null;
            var mat = new Material(shader);
            mat.color = contextColor;
            SetTransparent(mat);
            return mat;
        }

        private static Shader FindBoneShader()
        {
            var shader = Shader.Find("Universal Render Pipeline/Lit");
            if (shader == null)
                shader = Shader.Find("Standard");
            return shader;
        }

        private static void SetTransparent(Material mat)
        {
            // URP Lit transparency
            if (mat.HasProperty("_Surface"))
            {
                mat.SetFloat("_Surface", 1); // 1 = Transparent
                mat.SetFloat("_Blend", 0);   // Alpha
                mat.SetOverrideTag("RenderType", "Transparent");
                mat.renderQueue = (int)UnityEngine.Rendering.RenderQueue.Transparent;
                mat.EnableKeyword("_SURFACE_TYPE_TRANSPARENT");
                mat.EnableKeyword("_ALPHAPREMULTIPLY_ON");
            }
            // Standard shader fallback
            else if (mat.HasProperty("_Mode"))
            {
                mat.SetFloat("_Mode", 3); // Transparent
                mat.SetInt("_SrcBlend", (int)UnityEngine.Rendering.BlendMode.SrcAlpha);
                mat.SetInt("_DstBlend", (int)UnityEngine.Rendering.BlendMode.OneMinusSrcAlpha);
                mat.SetInt("_ZWrite", 0);
                mat.DisableKeyword("_ALPHATEST_ON");
                mat.EnableKeyword("_ALPHABLEND_ON");
                mat.DisableKeyword("_ALPHAPREMULTIPLY_ON");
                mat.renderQueue = (int)UnityEngine.Rendering.RenderQueue.Transparent;
            }
        }
    }
}
