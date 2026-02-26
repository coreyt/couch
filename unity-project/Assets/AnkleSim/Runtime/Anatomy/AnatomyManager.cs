using System;
using System.Collections.Generic;
using UnityEngine;
using AnkleSim.Core.DataModels;

namespace AnkleSim.Runtime.Anatomy
{
    public class AnatomyManager : MonoBehaviour
    {
        private readonly Dictionary<BoneType, GameObject> _boneObjects = new Dictionary<BoneType, GameObject>();
        private AnatomyConfig _config;

        public void LoadAnatomy(AnatomyConfig config)
        {
            if (config == null) return;

            // Destroy existing bone GameObjects before re-loading
            foreach (var kvp in _boneObjects)
            {
                if (kvp.Value != null)
                    DestroyImmediate(kvp.Value);
            }

            _config = config;
            _boneObjects.Clear();

            foreach (BoneType boneType in Enum.GetValues(typeof(BoneType)))
            {
                Mesh mesh = config.GetMeshForBone(boneType);
                if (mesh == null) continue;

                var boneGO = new GameObject(boneType.ToString());
                boneGO.transform.SetParent(transform);

                var meshFilter = boneGO.AddComponent<MeshFilter>();
                meshFilter.sharedMesh = mesh;

                var meshRenderer = boneGO.AddComponent<MeshRenderer>();
                var material = CreateDefaultMaterial();
                if (material != null)
                    meshRenderer.sharedMaterial = material;

                _boneObjects[boneType] = boneGO;
            }
        }

        public void SetStructureVisibility(string boneName, bool visible)
        {
            if (Enum.TryParse<BoneType>(boneName, true, out var boneType))
            {
                if (_boneObjects.TryGetValue(boneType, out var go))
                {
                    go.SetActive(visible);
                }
            }
        }

        public Mesh GetBoneMesh(BoneType boneType)
        {
            if (_boneObjects.TryGetValue(boneType, out var go))
            {
                var meshFilter = go.GetComponent<MeshFilter>();
                return meshFilter != null ? meshFilter.sharedMesh : null;
            }
            return null;
        }

        public GameObject GetBoneGameObject(BoneType boneType)
        {
            _boneObjects.TryGetValue(boneType, out var go);
            return go;
        }

        public void ApplyMaterialsByRole(BoneMaterialConfig materialConfig)
        {
            if (materialConfig == null) return;

            var simulatedMat = materialConfig.CreateSimulatedBoneMaterial();
            var contextMat = materialConfig.CreateContextBoneMaterial();

            foreach (var kvp in _boneObjects)
            {
                var renderer = kvp.Value.GetComponent<MeshRenderer>();
                if (renderer == null) continue;
                renderer.sharedMaterial = kvp.Key.IsSimulated() ? simulatedMat : contextMat;
            }
        }

        public Bounds GetAnatomyBounds()
        {
            var bounds = new Bounds(transform.position, Vector3.zero);
            bool first = true;

            foreach (var kvp in _boneObjects)
            {
                var renderer = kvp.Value.GetComponent<Renderer>();
                if (renderer == null) continue;

                if (first)
                {
                    bounds = renderer.bounds;
                    first = false;
                }
                else
                {
                    bounds.Encapsulate(renderer.bounds);
                }
            }

            return bounds;
        }

        private Material CreateDefaultMaterial()
        {
            var shader = Shader.Find("Universal Render Pipeline/Lit");
            if (shader == null)
                shader = Shader.Find("Standard");
            if (shader == null)
                return null;
            return new Material(shader);
        }
    }
}
