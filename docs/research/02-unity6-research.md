# Unity 6 for Medical/Surgical Orthopedic Simulation — Technical Report

> **Date:** 2026-02-12
> **Engine Version:** Unity 6 (6000.x series, current LTS: 6.3)
> **Physics Backend:** NVIDIA PhysX 4/5 (Featherstone solver)

---

## Table of Contents

1. [Unity 6 Physics](#1-unity-6-physics)
2. [3D Medical Asset Pipeline](#2-3d-medical-asset-pipeline)
3. [Runtime Mesh Manipulation](#3-runtime-mesh-manipulation)
4. [Rendering: URP vs HDRP](#4-rendering-urp-vs-hdrp)
5. [Plugin Architecture](#5-plugin-architecture)
6. [Unity 6 Features](#6-unity-6-features)
7. [Existing Medical Simulation in Unity](#7-existing-medical-simulation-in-unity)

---

## 1. Unity 6 Physics

### 1.1 PhysX Integration

Unity 6 ships with NVIDIA PhysX (versions 4/5 under the hood). Two solver options are available:

| Solver | Description | Best For |
|--------|-------------|----------|
| **Projected Gauss-Seidel (PGS)** | Default iterative solver | General-purpose gameplay |
| **Temporal Gauss-Seidel (TGS)** | More accurate, better convergence | Industrial/medical simulation, robotics |

Select the solver via `Physics.defaultSolverType` or in **Edit > Project Settings > Physics > Solver Type**.

### 1.2 ArticulationBody for Joints

`ArticulationBody` is the recommended component for building kinematic chains (robotic arms, biomechanical models, prosthetic joints). It uses **Featherstone's algorithm** in **reduced coordinate space**, which guarantees:

- No joint separation or drift
- Stable behavior with long kinematic chains (6+ joints)
- One-to-one DOF mapping (each scalar in the reduced space corresponds to a single physical degree of freedom)

**Joint Types:**

| Joint Type | DOFs | Use Case |
|------------|------|----------|
| `ArticulationJointType.FixedJoint` | 0 | Rigid bone-to-bone fusion |
| `ArticulationJointType.PrismaticJoint` | 1 (linear) | Sliding mechanisms, linear actuators |
| `ArticulationJointType.RevoluteJoint` | 1 (rotational) | Hinge joints (elbow, knee flexion/extension) |
| `ArticulationJointType.SphericalJoint` | Up to 3 (rotational) | Ball-and-socket joints (hip, shoulder, **ankle**) |

**Code — Setting Up an Articulation Chain:**

```csharp
// Root body (e.g., tibia)
ArticulationBody tibiaBody = tibia.AddComponent<ArticulationBody>();
tibiaBody.jointType = ArticulationJointType.FixedJoint; // root is fixed or kinematic

// Child body (e.g., talus — ankle joint)
ArticulationBody talusBody = talus.AddComponent<ArticulationBody>();
talusBody.jointType = ArticulationJointType.SphericalJoint;

// Configure swing/twist limits for ankle ROM
var xDrive = talusBody.xDrive;
xDrive.lowerLimit = -20f;  // dorsiflexion limit (degrees)
xDrive.upperLimit = 50f;   // plantarflexion limit (degrees)
xDrive.stiffness = 500f;   // Nm/rad
xDrive.damping = 50f;      // Nm/(rad/s)
xDrive.forceLimit = float.MaxValue;
talusBody.xDrive = xDrive;

var yDrive = talusBody.yDrive;
yDrive.lowerLimit = -35f;  // inversion limit
yDrive.upperLimit = 25f;   // eversion limit
yDrive.stiffness = 500f;
yDrive.damping = 50f;
talusBody.yDrive = yDrive;
```

### 1.3 ConfigurableJoint for Ankle Simulation

`ConfigurableJoint` provides per-axis control over all 6 DOFs (3 linear, 3 angular). This is useful when you need Rigidbody-based physics rather than ArticulationBody chains.

**The ankle joint has 3 biomechanical DOFs:**
- **Dorsiflexion/Plantarflexion** (~20 deg / ~50 deg) — sagittal plane
- **Inversion/Eversion** (~35 deg / ~25 deg) — frontal plane
- **Internal/External Rotation** (~15 deg / ~15 deg) — transverse plane

**Code — ConfigurableJoint for Ankle:**

```csharp
ConfigurableJoint ankleJoint = talus.AddComponent<ConfigurableJoint>();
ankleJoint.connectedBody = tibiaRigidbody;

// Lock linear motion (ankle is purely rotational)
ankleJoint.xMotion = ConfigurableJointMotion.Locked;
ankleJoint.yMotion = ConfigurableJointMotion.Locked;
ankleJoint.zMotion = ConfigurableJointMotion.Locked;

// Limit angular motion
ankleJoint.angularXMotion = ConfigurableJointMotion.Limited;  // dorsi/plantar
ankleJoint.angularYMotion = ConfigurableJointMotion.Limited;  // inv/eversion
ankleJoint.angularZMotion = ConfigurableJointMotion.Limited;  // int/ext rotation

// Angular X limits (dorsiflexion/plantarflexion)
SoftJointLimit lowAngX = ankleJoint.lowAngularXLimit;
lowAngX.limit = -20f;   // dorsiflexion
ankleJoint.lowAngularXLimit = lowAngX;

SoftJointLimit highAngX = ankleJoint.highAngularXLimit;
highAngX.limit = 50f;   // plantarflexion
ankleJoint.highAngularXLimit = highAngX;

// Angular YZ limit (cone for inversion/eversion + rotation)
SoftJointLimit angYZ = ankleJoint.angularYLimit;
angYZ.limit = 35f;
ankleJoint.angularYLimit = angYZ;
ankleJoint.angularZLimit = angYZ;

// Spring/Damper on angular limits
SoftJointLimitSpring angXSpring = ankleJoint.angularXLimitSpring;
angXSpring.spring = 500f;   // N*m/deg
angXSpring.damper = 50f;    // N*m*s/deg
ankleJoint.angularXLimitSpring = angXSpring;

// Angular Drive (resist/return to neutral)
JointDrive slerpDrive = ankleJoint.slerpDrive;
slerpDrive.positionSpring = 1000f;
slerpDrive.positionDamper = 100f;
slerpDrive.maximumForce = float.MaxValue;
ankleJoint.slerpDrive = slerpDrive;
ankleJoint.rotationDriveMode = RotationDriveMode.Slerp;
ankleJoint.targetRotation = Quaternion.identity;
```

**Drive Formula (both ArticulationBody and ConfigurableJoint):**

```
force = stiffness * (targetPosition - currentPosition) + damping * (targetVelocity - currentVelocity)
```

### 1.4 Force Measurement

#### ArticulationBody Inverse Dynamics API

Unity exposes several force-query methods on `ArticulationBody` for sensor-like behavior:

| Method | Returns |
|--------|---------|
| `GetJointForces(List<float>)` | Total joint forces in reduced coordinate space for the entire hierarchy |
| `GetDriveForces(List<float>)` | Forces applied by each Articulation Drive |
| `GetJointGravityForces(List<float>)` | Gravity compensation forces |
| `GetJointCoriolisCentrifugalForces(List<float>)` | Coriolis/centrifugal forces |
| `GetJointExternalForces(List<float>)` | Forces needed to counteract externally applied forces |
| `GetJointForcesForAcceleration(ArticulationReducedSpace)` | Forces needed to reach a target acceleration |
| `GetDofStartIndices(List<int>)` | Index mapping for per-body DOF data in the force arrays |

**Code — Reading Joint Forces:**

```csharp
ArticulationBody body = GetComponent<ArticulationBody>();
List<float> jointForces = new List<float>();
List<float> driveForces = new List<float>();
List<float> gravityForces = new List<float>();
List<int> dofStarts = new List<int>();

// Query forces for the entire articulation chain
body.GetJointForces(jointForces);
body.GetDriveForces(driveForces);
body.GetJointGravityForces(gravityForces);
body.GetDofStartIndices(dofStarts);

// Access this specific body's force data
int startIdx = dofStarts[body.index];
int dofCount = body.dofCount;

for (int i = 0; i < dofCount; i++)
{
    float totalForce = jointForces[startIdx + i];
    float driveForce = driveForces[startIdx + i];
    float gravForce = gravityForces[startIdx + i];
    Debug.Log($"DOF {i}: total={totalForce:F3}, drive={driveForce:F3}, gravity={gravForce:F3}");
}

// Per-body drive force (shortcut)
ArticulationReducedSpace perBodyDrive = body.driveForce;
```

> **Known Issue:** In earlier Unity versions (2020.x), `jointForce` sometimes returned zero for revolute joints. This has been addressed in Unity 6, but verify with your specific version.

#### Rigidbody/ConfigurableJoint Force Measurement

For Rigidbody-based joints, there is no direct force query API. Instead, measure indirectly:

```csharp
// Option 1: Track force via currentForce on the joint (not directly exposed)
// Option 2: Use the drive formula to compute expected force
float expectedForce = slerpDrive.positionSpring * angleDelta + slerpDrive.positionDamper * angularVelocity;

// Option 3: Measure reaction via Rigidbody
Vector3 measuredForce = rigidBody.GetAccumulatedForce();
Vector3 measuredTorque = rigidBody.GetAccumulatedTorque();
```

---

## 2. 3D Medical Asset Pipeline

### 2.1 DICOM to 3D Mesh Tools

| Tool | License | Strengths | Export Formats |
|------|---------|-----------|----------------|
| **[3D Slicer](https://www.slicer.org/)** | Free/Open Source (BSD) | Full-featured: segmentation (Segment Editor), registration, DICOM browser, Python scripting, 200+ extensions, NVIDIA Clara AI support | STL, OBJ, VTK, PLY, NRRD |
| **[ITK-SNAP](http://www.itksnap.org/)** | Free/Open Source (GPL) | Focused segmentation: Snake active contour, semi-automatic region growing, lightweight UI | STL, VTK, NIfTI |
| **[Mimics](https://www.materialise.com/en/healthcare/mimics-innovation-suite)** | Commercial (FDA-approved) | Gold standard for orthopedics: HU-calibrated thresholding, surgical planning, 3-matic mesh editing | STL, 3MF, VRML |
| **[InVesalius](https://invesalius.github.io/)** | Free/Open Source | Lightweight DICOM-to-3D: threshold segmentation, surface/volume rendering | STL, OBJ, PLY |

### 2.2 Bone Segmentation Workflow

**Step-by-step using 3D Slicer:**

1. **Import DICOM** — File > Add DICOM Data > Import folder of .dcm files
2. **Set Threshold** — In Segment Editor, use "Threshold" effect:
   - **Cortical bone:** ~300-900+ HU (varies by region; diaphysis ~909 HU, epiphysis ~275 HU)
   - **All bone (cortical + cancellous):** ~200-226 HU lower threshold
   - **Common starting point:** 226 HU (lower) to maximum HU (upper)
3. **Region Grow** — Use "Islands" effect to isolate individual bones (e.g., separate tibia from fibula)
4. **Manual Refinement** — Use "Paint" and "3D Brush" tools to correct misclassified voxels
5. **Smoothing** — Apply "Smoothing" effect (median or Gaussian, kernel 1-3mm)
6. **Export Mesh** — Segmentations > Export to files > STL or OBJ
7. **Mesh Cleanup (Blender/MeshLab):**
   - Decimate (reduce from ~500K to ~50K triangles for real-time)
   - Remove non-manifold edges
   - Fill holes
   - Remesh for uniform triangle distribution

**Accuracy benchmarks from literature:**
- PCD-CT scanners: < 0.05 mm mean deviation
- Optimized EID-CT: < 0.2 mm mean deviation
- General orthopedic threshold: < 0.32 mm (well within 0.5 mm surgical guidance threshold)

**Deep Learning Alternative:** U-Net architectures trained on annotated CT datasets can outperform manual thresholding, especially for extremities and joints where bone boundaries are less distinct. MONAI (Medical Open Network for AI) integrates with 3D Slicer and provides pre-trained models.

### 2.3 Mesh Formats for Unity

| Format | Rigging | Animation | PBR Materials | Unity Native | Best Use |
|--------|---------|-----------|---------------|-------------|----------|
| **FBX** | Full | Full | Limited | Yes (preferred) | Primary authoring format; rigged anatomical models |
| **OBJ** | None | None | Basic (.mtl) | Yes | Static anatomy exports from segmentation tools |
| **glTF/GLB** | Supported | Supported | Native PBR | Via packages (UnityGLTF) | Web/AR medical viewers, runtime loading |
| **STL** | None | None | None | Via importers | Direct from segmentation; needs conversion |

**Recommended Pipeline:**

```
CT/MRI (DICOM)
  → 3D Slicer (segment + export STL/OBJ)
    → Blender (cleanup, decimate, rig, export FBX)
      → Unity (import FBX, assign materials, physics)
```

**Code — Runtime glTF Loading (using UnityGLTF):**

```csharp
using UnityGLTF;

async void LoadAnatomyModel(string gltfPath)
{
    var loader = new GLTFSceneImporter(gltfPath, new ImportOptions());
    await loader.LoadSceneAsync();

    // Access loaded mesh
    GameObject root = loader.LastLoadedScene;
    MeshFilter[] meshes = root.GetComponentsInChildren<MeshFilter>();
}
```

---

## 3. Runtime Mesh Manipulation

### 3.1 Boolean/CSG Operations for Bone Resection

Constructive Solid Geometry (CSG) enables union, subtraction, and intersection of 3D meshes at runtime — critical for simulating bone cuts (osteotomy), drilling, and reaming.

**Available Libraries:**

| Library | License | Runtime Support | Cross-Section Fill | Limitations |
|---------|---------|----------------|-------------------|-------------|
| **[EzySlice](https://github.com/DavidArayan/ezy-slice)** | MIT | Yes | Yes (convex only) | Convex cross-section only; no concave fill |
| **[pb_CSG](https://github.com/karl-/pb_CSG)** | MIT | Yes | Union/Subtract/Intersect | Performance degrades with repeated ops |
| **[snikit/CSG](https://github.com/snikit/CSG---3d-boolean-operations-Unity-)** | Open Source | Yes (editor + runtime) | Full boolean ops | Unity 2019.4+ required |
| **ProBuilder Boolean** | Unity Package | Editor only (experimental) | Union/Subtract/Intersect | Not production-ready; editor-time only |
| **[Dynamic Mesh Cutter](https://assetstore.unity.com/packages/tools/modeling/mesh-slicer-59618)** | Commercial | Yes | Yes | Async cutting with thread control |

**Code — EzySlice Plane Cut (Bone Resection):**

```csharp
using EzySlice;

public SlicedHull PerformOsteotomy(GameObject bone, Vector3 cutPosition, Vector3 cutNormal)
{
    // Slice the bone mesh along a plane
    SlicedHull hull = bone.Slice(cutPosition, cutNormal);

    if (hull != null)
    {
        // Generate upper and lower hull GameObjects
        GameObject proximalFragment = hull.CreateUpperHull(bone, crossSectionMaterial);
        GameObject distalFragment = hull.CreateLowerHull(bone, crossSectionMaterial);

        // Add physics to fragments
        proximalFragment.AddComponent<MeshCollider>().convex = true;
        proximalFragment.AddComponent<Rigidbody>();

        distalFragment.AddComponent<MeshCollider>().convex = true;
        distalFragment.AddComponent<Rigidbody>();

        // Destroy original
        Destroy(bone);
    }

    return hull;
}
```

**Code — Manual Plane-Based Mesh Cutting Algorithm:**

```csharp
public static (Mesh positiveSide, Mesh negativeSide) CutMesh(Mesh original, Plane cuttingPlane)
{
    Vector3[] vertices = original.vertices;
    int[] triangles = original.triangles;

    List<Vector3> positiveVerts = new List<Vector3>();
    List<int> positiveTris = new List<int>();
    List<Vector3> negativeVerts = new List<Vector3>();
    List<int> negativeTris = new List<int>();

    for (int i = 0; i < triangles.Length; i += 3)
    {
        Vector3 v0 = vertices[triangles[i]];
        Vector3 v1 = vertices[triangles[i + 1]];
        Vector3 v2 = vertices[triangles[i + 2]];

        bool s0 = cuttingPlane.GetSide(v0);
        bool s1 = cuttingPlane.GetSide(v1);
        bool s2 = cuttingPlane.GetSide(v2);

        if (s0 == s1 && s1 == s2)
        {
            // All vertices on same side — add triangle to that side
            if (s0) AddTriangle(positiveVerts, positiveTris, v0, v1, v2);
            else    AddTriangle(negativeVerts, negativeTris, v0, v1, v2);
        }
        else
        {
            // Triangle intersects plane — split into sub-triangles
            SplitTriangle(cuttingPlane, v0, v1, v2, s0, s1, s2,
                          positiveVerts, positiveTris, negativeVerts, negativeTris);
        }
    }

    Mesh posMesh = BuildMesh(positiveVerts, positiveTris);
    Mesh negMesh = BuildMesh(negativeVerts, negativeTris);
    posMesh.RecalculateNormals();
    negMesh.RecalculateNormals();

    return (posMesh, negMesh);
}
```

### 3.2 Unity Mesh API

**Key Classes and Methods:**

| Class/Method | Purpose |
|-------------|---------|
| `Mesh` | Core mesh data container |
| `Mesh.vertices` / `SetVertices()` | Vertex position data |
| `Mesh.triangles` / `SetTriangles()` | Triangle index data |
| `Mesh.normals` / `SetNormals()` | Per-vertex normals |
| `Mesh.uv` / `SetUVs()` | Texture coordinates |
| `Mesh.RecalculateNormals()` | Auto-compute normals from geometry |
| `Mesh.RecalculateBounds()` | Update bounding box |
| `Mesh.RecalculateTangents()` | Compute tangent vectors |
| `Mesh.Clear()` | Reset all mesh data before rebuilding |
| `MeshCollider.sharedMesh` | Assign mesh to physics collider |

**Advanced Mesh API (for high performance):**

| Method | Purpose |
|--------|---------|
| `Mesh.SetVertexBufferParams()` | Define vertex attribute layout |
| `Mesh.SetVertexBufferData()` | Upload vertex data directly |
| `Mesh.SetIndexBufferParams()` | Define index format (16/32 bit) |
| `Mesh.SetIndexBufferData()` | Upload index data directly |
| `Mesh.SetSubMesh()` | Define sub-mesh regions |
| `Mesh.AcquireReadOnlyMeshData()` | Read mesh data in C# Jobs (Burst-compatible) |
| `Mesh.AllocateWritableMeshData()` | Write mesh data from C# Jobs |
| `Mesh.ApplyAndDisposeWritableMeshData()` | Apply job-written data to mesh |
| `MeshUpdateFlags` | Skip validation for performance |

**Code — Job-Based Mesh Modification (DOTS-compatible):**

```csharp
using Unity.Collections;
using Unity.Jobs;
using UnityEngine;
using UnityEngine.Rendering;

public void ModifyMeshWithJobs(Mesh mesh)
{
    // Read-only access for analysis
    Mesh.MeshDataArray readData = Mesh.AcquireReadOnlyMeshData(mesh);
    Mesh.MeshData inputMesh = readData[0];

    int vertexCount = inputMesh.vertexCount;
    NativeArray<Vector3> vertices = new NativeArray<Vector3>(vertexCount, Allocator.TempJob);
    inputMesh.GetVertices(vertices);

    // Allocate writable output
    Mesh.MeshDataArray writeData = Mesh.AllocateWritableMeshData(1);
    Mesh.MeshData outputMesh = writeData[0];

    outputMesh.SetVertexBufferParams(vertexCount,
        new VertexAttributeDescriptor(VertexAttribute.Position),
        new VertexAttributeDescriptor(VertexAttribute.Normal, stream: 1));

    // ... schedule jobs to modify vertices ...

    Mesh.ApplyAndDisposeWritableMeshData(writeData, mesh);
    mesh.RecalculateNormals();
    mesh.RecalculateBounds();

    vertices.Dispose();
    readData.Dispose();
}
```

### 3.3 Performance Considerations

| Concern | Mitigation |
|---------|------------|
| **Triangle explosion** after repeated cuts | Decimate after each cut; use LOD proxies for physics |
| **Collider rebuild cost** | Use `MeshCollider.sharedMesh` assignment (triggers rebuild); consider convex decomposition |
| **GC allocation** from `mesh.vertices` | Use the advanced API (`GetVertices(NativeArray)`) to avoid managed array allocation |
| **Normal recalculation** | `RecalculateNormals()` is O(n); skip if visual quality is acceptable |
| **Cross-section fill for concave meshes** | EzySlice only supports convex fill; use ear-clipping or Delaunay for concave |
| **Real-time continuous cutting** | Throttle cut operations (e.g., max 1 per frame); use async/threaded cutting (Dynamic Mesh Cutter) |
| **Skinned mesh cutting** | Standard CSG libraries do not support skinned meshes; bake the skinned mesh first via `SkinnedMeshRenderer.BakeMesh()` |

---

## 4. Rendering: URP vs HDRP

### 4.1 Feature Comparison for Medical Visualization

| Feature | URP | HDRP |
|---------|-----|------|
| **Subsurface Scattering** | Limited (custom shaders) | Built-in (skin, soft tissue) |
| **Volumetric Rendering** | Not built-in | Built-in (fog, volumetric lighting) |
| **Ray Tracing** | Not supported | Real-time ray tracing (DXR) |
| **Screen-Space Reflections** | Basic | High quality |
| **PBR Fidelity** | Good | Excellent (physically accurate) |
| **VR/XR Performance** | Excellent (optimized) | Moderate (demanding) |
| **Mobile Support** | Full | Not supported |
| **Standalone VR Headsets** (Quest) | Yes | No |
| **Multi-pass Stereo Rendering** | Optimized (SPI) | Supported |
| **Custom Render Passes** | Scriptable Render Passes | Custom Pass system |
| **Variable Rate Shading** (Unity 6.1+) | Yes | Yes |
| **GPU Resident Drawer** (Unity 6+) | Yes | Yes |
| **Render Graph** (Unity 6+) | Default | Default |

### 4.2 Recommendations

**Choose HDRP when:**
- Photorealistic rendering of bone, cartilage, tissue, and skin is required
- Subsurface scattering is needed (skin rendering, translucent tissue)
- Target is desktop/console workstations
- Surgical planning/patient education on high-end hardware
- Volume rendering of CT/MRI data via custom shaders

**Choose URP when:**
- Cross-platform deployment (mobile, tablet, standalone VR headsets like Meta Quest)
- VR/XR training simulators where 90 fps is mandatory
- Lower hardware requirements for broad accessibility
- Medical apps for tablets or web browsers

**Unity 6 Specific Updates:**
- **Render Graph** is enabled by default in URP (Unity 6+), improving rendering performance
- **GPU Resident Drawer** moves instance rendering to GPU, enabling 1M+ instances at 60 fps
- **Deferred+ Rendering** (URP, Unity 6.1) adds advanced lighting to URP
- URP and HDRP now share the same shader compiler/API (Unity 6.3), moving toward a unified renderer

### 4.3 Custom Medical Shaders

For bone visualization, a custom URP shader can provide adequate quality:

```hlsl
// Bone surface shader — URP Shader Graph or coded
// Key properties:
// - Base albedo from CT-derived density map (whiter = denser cortical bone)
// - Normal map for surface microstructure (trabecular pattern)
// - Ambient occlusion from mesh curvature
// - Smoothness: 0.2-0.4 for dry bone, 0.6-0.8 for wet/surgical field
// - Subsurface (HDRP only): thin bone transmits light (e.g., orbital floor)
```

---

## 5. Plugin Architecture

### 5.1 Unity Native Plugin Interface

Unity supports **native plugins** — compiled C/C++ libraries (.dll on Windows, .so on Linux, .dylib on macOS) loaded at runtime via P/Invoke.

**C++ Side — Plugin Header:**

```cpp
// NativePlugin.h
#pragma once

#ifdef _WIN32
    #define EXPORT __declspec(dllexport)
#else
    #define EXPORT __attribute__((visibility("default")))
#endif

extern "C" {
    // Simple data exchange
    EXPORT float ComputeJointTorque(float angle, float angularVelocity,
                                     float stiffness, float damping);

    // Struct-based data exchange
    struct JointState {
        float position[3];
        float velocity[3];
        float torque[3];
        int   jointId;
    };

    EXPORT void GetJointStates(JointState* outStates, int count);
    EXPORT void SetJointTargets(const JointState* targets, int count);

    // Bulk data (e.g., sensor array from external hardware)
    EXPORT int ReadSensorData(float* buffer, int maxSamples);

    // Initialization / cleanup
    EXPORT bool Initialize(const char* configPath);
    EXPORT void Shutdown();
}
```

**C++ Side — Implementation:**

```cpp
// NativePlugin.cpp
#include "NativePlugin.h"
#include <cmath>

extern "C" {
    EXPORT float ComputeJointTorque(float angle, float angularVelocity,
                                     float stiffness, float damping) {
        return stiffness * (-angle) + damping * (-angularVelocity);
    }

    EXPORT void GetJointStates(JointState* outStates, int count) {
        for (int i = 0; i < count; ++i) {
            // Populate from internal simulation state
            outStates[i].jointId = i;
            outStates[i].position[0] = /* ... */;
            // ...
        }
    }
}
```

**C# Side — Unity Integration:**

```csharp
using System;
using System.Runtime.InteropServices;
using UnityEngine;

public class NativePhysicsBridge : MonoBehaviour
{
    // Struct must match C++ layout exactly
    [StructLayout(LayoutKind.Sequential)]
    public struct JointState
    {
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 3)]
        public float[] position;
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 3)]
        public float[] velocity;
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 3)]
        public float[] torque;
        public int jointId;
    }

    // DllImport — name without extension; Unity resolves per platform
    [DllImport("NativePlugin")]
    private static extern float ComputeJointTorque(float angle, float angVel,
                                                    float stiffness, float damping);

    [DllImport("NativePlugin")]
    private static extern void GetJointStates(
        [In, Out] JointState[] outStates, int count);

    [DllImport("NativePlugin")]
    private static extern void SetJointTargets(
        [In] JointState[] targets, int count);

    [DllImport("NativePlugin")]
    private static extern int ReadSensorData(
        [In, Out] float[] buffer, int maxSamples);

    [DllImport("NativePlugin")]
    private static extern bool Initialize(
        [MarshalAs(UnmanagedType.LPStr)] string configPath);

    [DllImport("NativePlugin")]
    private static extern void Shutdown();

    private JointState[] _states;
    private float[] _sensorBuffer;

    void Start()
    {
        Initialize(Application.dataPath + "/config.json");
        _states = new JointState[6];  // e.g., 6-DOF chain
        _sensorBuffer = new float[1024];
    }

    void FixedUpdate()
    {
        // Read current joint states from native simulation
        GetJointStates(_states, _states.Length);

        // Read sensor data
        int samplesRead = ReadSensorData(_sensorBuffer, _sensorBuffer.Length);

        // Apply to Unity ArticulationBodies...
    }

    void OnDestroy()
    {
        Shutdown();
    }
}
```

### 5.2 DLL / Shared Library Loading

| Platform | Library Extension | Plugin Location |
|----------|------------------|-----------------|
| Windows | `.dll` | `Assets/Plugins/x86_64/` |
| Linux | `.so` | `Assets/Plugins/x86_64/` |
| macOS | `.bundle` / `.dylib` | `Assets/Plugins/` |
| Android | `.so` | `Assets/Plugins/Android/libs/arm64-v8a/` |
| iOS | Static linking (`__Internal`) | `Assets/Plugins/iOS/` |

**Important:** Once loaded, native plugins are **never unloaded** during an editor session. Restart Unity to replace updated DLLs during development.

### 5.3 IPC Options

| Method | Latency | Throughput | Complexity | Best For |
|--------|---------|------------|------------|----------|
| **Native Plugin (DllImport)** | ~1 us (function call) | Highest (in-process) | Medium | Math-heavy computation, hardware drivers |
| **Shared Memory** (`MemoryMappedFile`) | ~1-10 us | Very high (no copy) | High | Large data arrays (sensor streams, images) |
| **ZeroMQ (NetMQ)** | ~50-100 us | High (10K+ msg/s) | Low-Medium | Cross-process pub/sub, req/rep patterns |
| **WebSocket** | ~1-5 ms | Medium | Low | Web clients, dashboard integration |
| **REST/HTTP** | ~5-50 ms | Low | Lowest | Configuration, non-real-time queries |
| **Named Pipes** | ~10-100 us | High | Medium | Local same-machine IPC |
| **TCP Sockets** | ~0.1-1 ms | High | Medium | Cross-machine communication |

### 5.4 ZeroMQ (NetMQ) Integration

NetMQ is the pure C# port of ZeroMQ. Include `AsyncIO.dll` and `NetMQ.dll` in `Assets/Plugins/`.

```csharp
using NetMQ;
using NetMQ.Sockets;
using System.Threading;

public class ZmqDataReceiver : MonoBehaviour
{
    private SubscriberSocket _subscriber;
    private Thread _receiveThread;
    private volatile bool _running;
    private ConcurrentQueue<string> _messageQueue = new ConcurrentQueue<string>();

    void Start()
    {
        _running = true;
        _receiveThread = new Thread(ReceiveLoop);
        _receiveThread.Start();
    }

    void ReceiveLoop()
    {
        using (var subscriber = new SubscriberSocket())
        {
            subscriber.Connect("tcp://localhost:5555");
            subscriber.Subscribe("joint_data");

            while (_running)
            {
                if (subscriber.TryReceiveFrameString(TimeSpan.FromMilliseconds(100), out string topic))
                {
                    string data = subscriber.ReceiveFrameString();
                    _messageQueue.Enqueue(data);
                }
            }
        }
    }

    void Update()
    {
        while (_messageQueue.TryDequeue(out string msg))
        {
            // Parse JSON and apply to simulation
            var jointData = JsonUtility.FromJson<JointDataMessage>(msg);
            ApplyJointData(jointData);
        }
    }

    void OnDestroy()
    {
        _running = false;
        _receiveThread?.Join(1000);
        NetMQConfig.Cleanup();
    }
}
```

> **Known Issue:** Creating multiple ZMQ sockets in Unity can cause editor freezes. Use a single receiver thread and route messages via a concurrent queue.

### 5.5 WebSocket Communication

Using [NativeWebSocket](https://github.com/endel/NativeWebSocket):

```csharp
using NativeWebSocket;
using UnityEngine;

public class WebSocketBridge : MonoBehaviour
{
    WebSocket ws;

    async void Start()
    {
        ws = new WebSocket("ws://localhost:8080/simulation");

        ws.OnOpen += () => Debug.Log("WS Connected");
        ws.OnMessage += (bytes) => {
            string json = System.Text.Encoding.UTF8.GetString(bytes);
            // Process on main thread via queue
        };
        ws.OnError += (e) => Debug.LogError("WS Error: " + e);
        ws.OnClose += (e) => Debug.Log("WS Closed");

        await ws.Connect();
    }

    void Update()
    {
        #if !UNITY_WEBGL || UNITY_EDITOR
        ws?.DispatchMessageQueue();
        #endif
    }

    async void SendCommand(string json)
    {
        if (ws.State == WebSocketState.Open)
            await ws.SendText(json);
    }

    async void OnDestroy()
    {
        if (ws != null) await ws.Close();
    }
}
```

### 5.6 Shared Memory (High-Performance Data Exchange)

```csharp
using System.IO.MemoryMappedFiles;
using System.Runtime.InteropServices;

public class SharedMemoryBridge : MonoBehaviour
{
    [StructLayout(LayoutKind.Sequential)]
    public struct SensorFrame
    {
        public double timestamp;
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 6)]
        public float[] forces;     // Fx, Fy, Fz, Tx, Ty, Tz
        public int frameId;
    }

    private MemoryMappedFile _mmf;
    private MemoryMappedViewAccessor _accessor;

    void Start()
    {
        // Open existing shared memory created by external process
        _mmf = MemoryMappedFile.OpenExisting("OrthoSimSensorData");
        _accessor = _mmf.CreateViewAccessor();
    }

    SensorFrame ReadLatestFrame()
    {
        SensorFrame frame;
        _accessor.Read(0, out frame);
        return frame;
    }

    void FixedUpdate()
    {
        SensorFrame frame = ReadLatestFrame();
        // Apply forces to simulation
        Debug.Log($"Force: [{frame.forces[0]:F2}, {frame.forces[1]:F2}, {frame.forces[2]:F2}]");
    }

    void OnDestroy()
    {
        _accessor?.Dispose();
        _mmf?.Dispose();
    }
}
```

---

## 6. Unity 6 Features

### 6.1 DOTS / ECS

Unity 6.1+ brings ECS to **production-ready status**. The Entity Component System arranges data linearly in memory for cache-friendly access, enabling simulation of millions of entities with minimal overhead.

**Key DOTS Packages:**

| Package | Purpose |
|---------|---------|
| `com.unity.entities` | Core ECS framework |
| `com.unity.entities.graphics` | ECS rendering integration |
| `com.unity.physics` | Stateless DOTS physics (not PhysX) |
| `com.unity.burst` | LLVM-based compiler for C# Jobs |
| `com.unity.collections` | NativeArray, NativeList, etc. |
| `com.unity.jobs` | C# Job System for multi-threaded work |
| `com.unity.mathematics` | SIMD-optimized math library |

**Relevance to Medical Simulation:**
- Process thousands of mesh vertices in parallel (Burst + Jobs)
- Simulate large particle systems (blood, fluid)
- Manage large numbers of implant/instrument entities
- Real-time data processing from sensor arrays

**Code — Burst-Compiled Mesh Processing Job:**

```csharp
using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;

[BurstCompile]
public struct DeformMeshJob : IJobParallelFor
{
    [ReadOnly] public NativeArray<float3> InputVertices;
    public NativeArray<float3> OutputVertices;
    public float3 ToolPosition;
    public float ToolRadius;
    public float DeformStrength;

    public void Execute(int index)
    {
        float3 vertex = InputVertices[index];
        float dist = math.distance(vertex, ToolPosition);

        if (dist < ToolRadius)
        {
            float influence = 1.0f - (dist / ToolRadius);
            float3 direction = math.normalize(vertex - ToolPosition);
            vertex += direction * influence * DeformStrength;
        }

        OutputVertices[index] = vertex;
    }
}

// Usage:
var job = new DeformMeshJob
{
    InputVertices = inputVerts,
    OutputVertices = outputVerts,
    ToolPosition = toolTransform.position,
    ToolRadius = 0.02f,
    DeformStrength = 0.001f
};
JobHandle handle = job.Schedule(inputVerts.Length, 64); // batch size 64
handle.Complete();
```

### 6.2 GPU Compute Shaders

Compute shaders run on the GPU outside the rendering pipeline and are ideal for parallel data processing.

**Code — Compute Shader for Vertex Deformation:**

```hlsl
// BoneDeform.compute
#pragma kernel CSMain

struct VertexData
{
    float3 position;
    float3 normal;
};

RWStructuredBuffer<VertexData> vertices;
float3 toolPosition;
float toolRadius;
float deformAmount;
uint vertexCount;

[numthreads(256, 1, 1)]
void CSMain(uint3 id : SV_DispatchThreadID)
{
    if (id.x >= vertexCount) return;

    VertexData v = vertices[id.x];
    float dist = distance(v.position, toolPosition);

    if (dist < toolRadius)
    {
        float influence = 1.0 - saturate(dist / toolRadius);
        influence = influence * influence; // smooth falloff
        float3 dir = normalize(v.position - toolPosition);
        v.position += dir * influence * deformAmount;
    }

    vertices[id.x] = v;
}
```

**C# Dispatch:**

```csharp
public class GPUMeshDeformer : MonoBehaviour
{
    public ComputeShader deformShader;
    private ComputeBuffer vertexBuffer;

    struct VertexData
    {
        public Vector3 position;
        public Vector3 normal;
    }

    void SetupBuffer(Mesh mesh)
    {
        Vector3[] positions = mesh.vertices;
        Vector3[] normals = mesh.normals;

        VertexData[] data = new VertexData[positions.Length];
        for (int i = 0; i < positions.Length; i++)
        {
            data[i].position = positions[i];
            data[i].normal = normals[i];
        }

        int stride = sizeof(float) * 6;  // 2 x Vector3
        vertexBuffer = new ComputeBuffer(data.Length, stride);
        vertexBuffer.SetData(data);
    }

    void Deform(Vector3 toolPos, float radius, float amount)
    {
        int kernel = deformShader.FindKernel("CSMain");
        deformShader.SetBuffer(kernel, "vertices", vertexBuffer);
        deformShader.SetVector("toolPosition", toolPos);
        deformShader.SetFloat("toolRadius", radius);
        deformShader.SetFloat("deformAmount", amount);
        deformShader.SetInt("vertexCount", vertexBuffer.count);

        int threadGroups = Mathf.CeilToInt(vertexBuffer.count / 256f);
        deformShader.Dispatch(kernel, threadGroups, 1, 1);

        // Read back (expensive — minimize)
        VertexData[] result = new VertexData[vertexBuffer.count];
        vertexBuffer.GetData(result);
        // Apply to mesh...
    }

    void OnDestroy()
    {
        vertexBuffer?.Dispose();
    }
}
```

> **Performance tip:** Avoid CPU readback (`GetData`) every frame. Use `AsyncGPUReadback.Request()` for non-blocking reads, or keep data on GPU if only needed for rendering.

### 6.3 VR/XR Support

Unity 6 provides comprehensive XR support through the **XR Interaction Toolkit** and **AR Foundation**.

**Key XR Packages:**

| Package | Purpose |
|---------|---------|
| `com.unity.xr.management` | XR plugin lifecycle management |
| `com.unity.xr.interaction.toolkit` | Cross-platform VR/AR interaction |
| `com.unity.xr.openxr` | OpenXR runtime support |
| `com.unity.xr.arfoundation` | AR Foundation (mobile AR) |
| `com.unity.xr.hands` | Hand tracking |

**Unity 6 XR Features:**
- **Plug-and-play** support for HoloLens, Magic Leap, Meta Quest, and enterprise headsets
- **World-space UI rendering** for immersive XR interfaces
- **Android XR support** (Unity 6.1+) with face tracking and object tracking (Unity 6.3+)
- **URP compatibility** for XR (single-pass instanced rendering)

**Surgical Simulation XR Considerations:**
- Use **URP** for standalone headsets (Quest 3/Pro) — 72-120 Hz required
- Use **HDRP** for PC-tethered VR (Valve Index, Vive Pro) if visual fidelity is paramount
- Hand tracking enables instrument manipulation without controllers
- Haptic feedback via controller vibration or specialized haptic devices

### 6.4 Addressables

The Addressable Asset System manages runtime loading/unloading of assets by string address, built on top of AssetBundles.

**Medical Simulation Use Cases:**
- Load anatomical models on-demand (patient-specific CT-derived meshes)
- Stream procedure-specific assets (instruments, implants) per surgical scenario
- Remote hosting of large asset libraries (organ systems, pathology variants)
- Reduce initial install size for training applications

**Code — Loading Anatomical Models on Demand:**

```csharp
using UnityEngine;
using UnityEngine.AddressableAssets;
using UnityEngine.ResourceManagement.AsyncOperations;

public class AnatomyLoader : MonoBehaviour
{
    public async void LoadBoneModel(string anatomyAddress)
    {
        // Check download size (for remote assets)
        var sizeHandle = Addressables.GetDownloadSizeAsync(anatomyAddress);
        await sizeHandle.Task;

        if (sizeHandle.Result > 0)
        {
            Debug.Log($"Downloading {sizeHandle.Result / 1024f:F1} KB...");
            var downloadHandle = Addressables.DownloadDependenciesAsync(anatomyAddress);
            while (!downloadHandle.IsDone)
            {
                float progress = downloadHandle.PercentComplete;
                // Update progress bar...
                await System.Threading.Tasks.Task.Yield();
            }
        }

        // Instantiate the model
        var loadHandle = Addressables.InstantiateAsync(anatomyAddress);
        await loadHandle.Task;

        if (loadHandle.Status == AsyncOperationStatus.Succeeded)
        {
            GameObject model = loadHandle.Result;
            // Position, add physics, etc.
        }
    }

    public void UnloadModel(GameObject model)
    {
        // Release to free memory (reference counting)
        Addressables.ReleaseInstance(model);
    }
}
```

**Best Practices:**
- Group assets by procedure/anatomy (load/unload together)
- Use `Addressables.GetDownloadSizeAsync()` to check before downloading
- Always call `Addressables.Release()` / `Addressables.ReleaseInstance()` to prevent memory leaks
- Use the Addressables Event Viewer and Memory Profiler to monitor runtime behavior
- Avoid "Check Duplicate Bundle Dependencies" issues by organizing groups carefully

---

## 7. Existing Medical Simulation in Unity

### 7.1 Open-Source Projects

| Project | Focus | Unity Integration | GitHub / Link |
|---------|-------|-------------------|---------------|
| **[iMSTK](https://www.imstk.org/)** | General surgical simulation (position-based dynamics, collision detection, haptics) | Native Unity plugin (drag-and-drop tools) | [Frontiers paper](https://www.frontiersin.org/journals/virtual-reality/articles/10.3389/frvir.2023.1130156/full) |
| **[SOFA Framework](https://www.sofa-framework.org/)** | Physics-based medical simulation (FEM, collision, soft tissue) | Partial (via plugin/IPC) | [sofa-framework.org](https://www.sofa-framework.org/) |
| **[OpenSurgSim](https://opensurgsim.org/)** | Desktop surgical training simulation | Standalone (C++) | SimQuest |
| **[Imhotep](https://github.com/Meister1593/Imhotep)** | Unity-based medical simulation framework | Built in Unity | GitHub |
| **[Laparoscopic Simulator](https://github.com/grzegorzzajac/laparoscopic-simulator)** | Laparoscopic surgery (OpenCV + Unity) | Built in Unity | GitHub |
| **[Surgery Simulator](https://github.com/zysio1998/surgery-simulator)** | General surgical procedure training | Built in Unity | GitHub |

### 7.2 Commercial Solutions

| Product | Focus | Notes |
|---------|-------|-------|
| **[VirtaMed](https://www.virtamed.com/)** | VR surgical training (arthroscopy, endoscopy) | Built on Unity; specialized hardware + anatomical models |
| **Da Vinci Robot Simulator** | Robotic surgery training | Unity Physics + dVRK integration |
| **Touch Surgery (Digital Surgery)** | Mobile surgical simulation | Unity-based; acquired by Medtronic |
| **FundamentalVR** | VR surgical training with haptics | Unity-based; multi-headset support |
| **Osso VR** | Orthopedic surgical training in VR | Unity-based; FDA De Novo authorized |

### 7.3 iMSTK (Interactive Medical Simulation Toolkit)

iMSTK is the most directly relevant open-source project for orthopedic simulation. Key capabilities:

- **Position-Based Dynamics (PBD)** for soft tissue deformation
- **Continuous Collision Detection** for surgical tool interaction
- **Smooth Particle Hydrodynamics (SPH)** for fluid simulation
- **Integrated Haptics** support
- **Virtual Osteotomy Trainer** — specifically designed for bone cutting simulation (bisagittal split osteotomy)
- Apache 2.0 license
- Unity integration via native plugin with drag-and-drop scene building

### 7.4 Orthopedic-Specific Projects

- **Mixed-Reality HoloLens Orthopedic Trainer** — Hip arthroplasty training using HoloLens + Vuforia SDK + Unity3D. Patient-specific CT-derived 3D models combined with rapid-prototyped synthetic bones for hybrid physical-virtual training.
- **Virtual Osteotomy Trainer (via iMSTK)** — Trains bisagittal split osteotomy (BSSO) for orthognathic surgery, built to improve procedural knowledge and surgical proficiency.

### 7.5 Best Practices for Medical Simulation in Unity

1. **Physics Configuration:**
   - Use TGS solver for stability in joint chains
   - Set fixed timestep to 0.005s or lower (200 Hz) for surgical precision
   - Use ArticulationBody for kinematic chains; ConfigurableJoint for individual constrained bodies
   - Validate force measurements against analytical solutions before trusting inverse dynamics API

2. **Asset Pipeline:**
   - Establish a validated DICOM-to-mesh pipeline early (3D Slicer recommended)
   - Standardize on FBX for Unity import; keep source STL/OBJ files archived
   - Target 50K-100K triangles per anatomical structure for real-time rendering; keep higher-res versions for reference
   - Use LOD groups for complex anatomical scenes

3. **Mesh Operations:**
   - Pre-compute cut planes where possible (guided osteotomy)
   - For freehand cutting, throttle operations and use async cutting
   - Bake skinned meshes before CSG operations
   - Regenerate colliders after mesh modification

4. **Rendering:**
   - Choose render pipeline at project start — migration is costly
   - Use HDRP for photorealism on desktop; URP for VR/mobile
   - Custom shaders for bone density visualization (map HU values to color)
   - Subsurface scattering (HDRP) dramatically improves tissue realism

5. **Performance:**
   - Profile with Unity Profiler and Frame Debugger
   - Use Burst compiler for CPU-bound mesh operations
   - Use compute shaders for GPU-parallel vertex processing
   - Addressables for on-demand asset loading
   - Target 90 fps minimum for VR (11.1 ms frame budget)

6. **External Integration:**
   - Use native plugins for hardware drivers and compute-intensive operations
   - Use ZeroMQ (NetMQ) for cross-process data streaming (Python, external solvers)
   - Use shared memory for high-bandwidth, low-latency sensor data
   - Use WebSockets for web dashboard integration and remote monitoring

7. **Validation:**
   - Compare simulation results against published biomechanical data
   - Implement force/torque logging for post-hoc analysis
   - Record and replay simulation sessions for training assessment
   - Follow IEC 62304 (Medical Device Software Lifecycle) if targeting clinical use

---

## Sources

### Unity Documentation
- [Unity Manual — Introduction to Physics Articulations](https://docs.unity3d.com/Manual/physics-articulations.html)
- [Unity Manual — Articulation Body Component Reference](https://docs.unity3d.com/Manual/class-ArticulationBody.html)
- [Unity Manual — Configurable Joint Component Reference](https://docs.unity3d.com/Manual/class-ConfigurableJoint.html)
- [Unity Manual — Introduction to Joints](https://docs.unity3d.com/Manual/Joints.html)
- [Unity Scripting API — ArticulationBody](https://docs.unity3d.com/ScriptReference/ArticulationBody.html)
- [Unity Scripting API — ArticulationBody.GetJointForces](https://docs.unity3d.com/ScriptReference/ArticulationBody.GetJointForces.html)
- [Unity Scripting API — ArticulationBody.GetDriveForces](https://docs.unity3d.com/6000.0/Documentation/ScriptReference/ArticulationBody.GetDriveForces.html)
- [Unity Scripting API — Mesh](https://docs.unity3d.com/ScriptReference/Mesh.html)
- [Unity Scripting API — ConfigurableJoint](https://docs.unity3d.com/ScriptReference/ConfigurableJoint.html)
- [Unity Manual — Native Plug-ins](https://docs.unity3d.com/Manual/NativePlugins.html)
- [Unity Manual — Compute Shaders](https://docs.unity3d.com/Manual/class-ComputeShader.html)
- [Unity Manual — Model File Formats](https://docs.unity3d.com/Manual/3D-formats.html)
- [Unity Manual — Render Pipeline Feature Comparison](https://docs.unity3d.com/6000.3/Documentation/Manual/render-pipelines-feature-comparison.html)
- [Unity Manual — XR Render Pipeline Compatibility](https://docs.unity3d.com/Manual/xr-render-pipeline-compatibility.html)
- [Unity DOTS Overview](https://unity.com/dots)
- [ProBuilder Boolean Operations](https://docs.unity3d.com/Packages/com.unity.probuilder@6.0/manual/boolean.html)
- [Addressables Documentation](https://docs.unity3d.com/Packages/com.unity.addressables@1.20/manual/LoadingAddressableAssets.html)

### Unity Blog & Learn
- [Prototype Industrial Designs with ArticulationBody](https://blog.unity.com/industry/use-articulation-bodies-to-easily-prototype-industrial-designs-with-realistic-motion-and)
- [Addressables Planning and Best Practices](https://blog.unity.com/engine-platform/addressables-planning-and-best-practices)
- [URP Recipe: Compute Shaders](https://learn.unity.com/tutorial/urp-recipe-compute-shaders)
- [Working with Native Plugins](https://learn.unity.com/tutorial/working-with-native-plugins-2019-3)

### Physics & Simulation
- [PhysX 5.5 Articulations Documentation](https://nvidia-omniverse.github.io/PhysX/physx/5.5.0/docs/Articulations.html)
- [Unity Robotics — Articulations Robot Demo](https://github.com/Unity-Technologies/articulations-robot-demo)
- [Unity Inverse Dynamics Demo](https://github.com/Unity-Technologies/unity-inverse-dynamics-demo)
- [Robotics Simulation in Unity (Medium)](https://medium.com/@jmake/robotics-simulation-in-unity-1986d828abc5)

### Medical Imaging & Segmentation
- [3D Slicer](https://www.slicer.org/)
- [ITK-SNAP](http://www.itksnap.org/)
- [ITK-SNAP vs 3D Slicer Comparison (IMAIOS)](https://www.imaios.com/en/resources/blog/itk-snap-vs-3d-slicer)
- [VTK and Unity Integration (PMC)](https://pmc.ncbi.nlm.nih.gov/articles/PMC6372083/)
- [Bone Segmentation — Effect of Threshold Level (Nature)](https://www.nature.com/articles/s41598-020-64383-9)
- [Deep Learning Bone Segmentation in CBCT (MDPI)](https://www.mdpi.com/2076-3417/14/17/7557)
- [Accuracy of 3D Bone Fracture Models (Springer)](https://link.springer.com/article/10.1007/s10278-024-00998-y)
- [3D Segmentation Software for Hip Surgery (PMC)](https://pmc.ncbi.nlm.nih.gov/articles/PMC9323631/)

### Mesh Cutting & CSG
- [EzySlice — Open Source Mesh Slicer](https://github.com/DavidArayan/ezy-slice)
- [pb_CSG — C# Port of CSG.js](https://github.com/karl-/pb_CSG)
- [CSG 3D Boolean Operations Unity](https://github.com/snikit/CSG---3d-boolean-operations-Unity-)
- [Mesh Slicing in Unity (Medium)](https://medium.com/@hesmeron/mesh-slicing-in-unity-740b21ffdf84)
- [Mesh Cutter (GitHub)](https://github.com/hugoscurti/mesh-cutter)

### Plugin & Communication
- [Integrating Native C/C++ Libraries with Unity (Level Up Coding)](https://levelup.gitconnected.com/integrating-native-c-c-libraries-with-unity-as-plugins-a-step-by-step-guide-17ad70c2e3b4)
- [Unity C++ Native Plugin Examples (Brave New Method)](https://bravenewmethod.com/2017/10/30/unity-c-native-plugin-examples/)
- [ZeroMQ in Unity (UQIDO Tech)](https://tech.uqido.com/2020/09/29/zeromq-in-unity/)
- [Python-Unity Interface with ZeroMQ (Medium)](https://vinnik-dmitry07.medium.com/a-python-unity-interface-with-zeromq-12720d6b7288)
- [NativeWebSocket for Unity](https://github.com/endel/NativeWebSocket)
- [WebSockets in Unity (Ably)](https://ably.com/topic/websockets-unity)
- [Cross-Platform Native Plugins in Unity](https://matiaslavik.wordpress.com/2021/01/21/cross-platform-native-plugins-in-unity/)

### Rendering & Unity 6
- [Unity HDRP vs URP vs Built-in: 2025 Guide](https://canixelarts.com/blog?post_id=64)
- [Unity 6.1 Preview (CG Channel)](https://www.cgchannel.com/2025/03/unity-previews-unity-6-1/)
- [Unity 6.3 LTS (CG Channel)](https://www.cgchannel.com/2025/12/unity-6-3-lts-is-out-see-5-key-features-for-cg-artists/)
- [Unity 2025 Product Roadmap (CG Channel)](https://www.cgchannel.com/2025/03/unity-unveils-its-2025-product-roadmap/)
- [ECS Development Status December 2025](https://discussions.unity.com/t/ecs-development-status-december-2025/1699284)
- [MinimalCompute — Compute Shader Examples](https://github.com/cinight/MinimalCompute)
- [Catlike Coding — Compute Shaders](https://catlikecoding.com/unity/tutorials/basics/compute-shaders/)

### Medical Simulation Projects
- [iMSTK — Interactive Medical Simulation Toolkit (Frontiers)](https://www.frontiersin.org/journals/virtual-reality/articles/10.3389/frvir.2023.1130156/full)
- [SOFA Framework](https://www.sofa-framework.org/)
- [VirtaMed — Unity-Based VR Surgical Training](https://unity.com/resources/virtamed)
- [Da Vinci Robot Simulator in Unity (IEEE)](https://ieeexplore.ieee.org/document/9925319/)
- [Low-Cost Unity VR Laparoscopic Simulator (PMC)](https://pmc.ncbi.nlm.nih.gov/articles/PMC10588702/)
- [Open Source Medical Simulation Projects (Medevel)](https://medevel.com/medical-simulation/)
- [Ankle Exoskeleton 3-DOF Design (MDPI)](https://www.mdpi.com/1424-8220/24/19/6160)
- [Configurable Joints Guide (WireWhiz)](https://wirewhiz.com/the-complete-guide-to-unitys-configurable-joints/)
- [glTF vs FBX Comparison (Threekit)](https://www.threekit.com/blog/gltf-vs-fbx-which-format-should-i-use)
