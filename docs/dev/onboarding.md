# Developer Onboarding Guide

**Last updated:** 2026-02-15
**For codebase version:** main (Sprint 4 complete, Windows DLL deployment operational)

This guide takes you from repository clone to running tests and making your first change. Follow every step in order.

---

## Prerequisites

Install the following exact versions before proceeding. Mismatched versions cause build failures and cryptic runtime errors.

### Required Software

| Component | Version | Purpose | Platform |
|-----------|---------|---------|----------|
| **Unity Editor** | 6000.3.8f1 | Runtime environment | Windows |
| **SOFA Framework** | v24.06.00 (Linux)<br>v25.12.00 (Windows) | Physics simulation library | Both |
| **CMake** | ≥3.16 | Build system | Both |
| **GCC/G++** | ≥9.0 | Linux native plugin build | Linux/WSL2 |
| **Visual Studio 2022** | 17.x | Windows DLL build | Windows |
| **WSL2** | Ubuntu 20.04+ | Development environment | Windows host |

### SOFA Framework Installation

**Linux (WSL2):**
```bash
cd ~/sofa
wget https://github.com/sofa-framework/sofa/releases/download/v24.06.00/SOFA_v24.06.00_Linux.zip
unzip SOFA_v24.06.00_Linux.zip
export SOFA_ROOT=~/sofa/SOFA_v24.06.00_Linux
echo 'export SOFA_ROOT=~/sofa/SOFA_v24.06.00_Linux' >> ~/.bashrc
```

**Windows:**
```powershell
# Download from GitHub releases
# Extract to C:\sofa\SOFA_v25.12.00_Win64
# Set environment variable in PowerShell
[Environment]::SetEnvironmentVariable("SOFA_ROOT", "C:\sofa\SOFA_v25.12.00_Win64", "User")
```

### Unity Installation

Install Unity Hub, then install Unity 6000.3.8f1 with the following modules:
- Windows Build Support (IL2CPP)
- Linux Build Support
- Visual Studio integration

Default install path: `C:\Program Files\Unity\Hub\Editor\6000.3.8f1\`

### Verification

```bash
# From WSL2
echo $SOFA_ROOT
# Expected: /home/<user>/sofa/SOFA_v24.06.00_Linux

cmake --version
# Expected: cmake version 3.16.0 or higher

g++ --version
# Expected: g++ (Ubuntu 9.x or higher)
```

---

## Repository Layout

Clone the repository and review the structure:

```bash
git clone <repository-url> ~/projects/couch
cd ~/projects/couch
```

**Annotated directory tree:**

```
couch/
├── spike/
│   ├── spike1_python/           # Python proof-of-concept (reference only)
│   │   ├── ankle_scene.py       # Original SofaPython3 scene
│   │   └── test_ankle_rom.py    # Clinical ROM validation
│   └── spike2_native/           # ★ C++ native plugin (you will edit this)
│       ├── CMakeLists.txt       # Build configuration
│       ├── CMakePresets.json    # Platform-specific presets
│       ├── include/             # Public C API headers
│       │   └── sofa_ankle_bridge.h
│       ├── src/                 # C++ implementation
│       │   ├── sofa_ankle_bridge.cpp   # API entry points
│       │   ├── scene_builder.cpp       # Scene construction
│       │   ├── ankle_scene.cpp         # Legacy ankle scene
│       │   ├── thread_manager.cpp      # Async stepping
│       │   └── sofa_compat.h           # v24.06/v25.12 compatibility
│       ├── test/                # C++ tests (GTest)
│       └── scripts/             # Deployment scripts
│           ├── collect_sofa_deps.sh          # Linux dependency collection
│           ├── collect_sofa_deps_win.ps1     # Windows dependency collection
│           └── deploy_to_unity.sh            # Copy DLLs to Unity
│
├── unity-project/               # ★ Unity 6 project (you will edit this)
│   ├── Assets/
│   │   ├── AnkleSim/
│   │   │   ├── Core/            # Data models, angle math (AnkleSim.Core)
│   │   │   ├── Bridge/          # P/Invoke, native interop (AnkleSim.Bridge)
│   │   │   └── Runtime/         # MonoBehaviours (AnkleSim.Runtime)
│   │   ├── Plugins/x86_64/      # ★ Native DLLs deployed here (39 files)
│   │   └── Tests/
│   │       ├── EditMode/        # Pure C# tests (37 tests)
│   │       └── PlayMode/        # C++↔C# integration tests (33 tests)
│   └── ProjectSettings/
│
├── docs/
│   ├── research/                # Background reading (clinical TAR, SOFA, Unity)
│   ├── adr/                     # Architecture Decision Records
│   │   ├── 0001-centroid-based-resection.md
│   │   ├── 0002-dual-representation-cutting.md
│   │   └── 0003-undo-via-scene-rebuild.md
│   └── dev/                     # ★ Developer documentation (this file)
│
├── CLAUDE.md                    # Project overview and build quick-reference
└── unity-test.sh                # Headless test runner (WSL2 → Windows Unity)
```

**Directories you will actually touch:**
- `spike/spike2_native/src/` — C++ plugin implementation
- `spike/spike2_native/include/` — C API header
- `unity-project/Assets/AnkleSim/` — C# code
- `unity-project/Assets/Tests/` — Test code
- `unity-project/Assets/Plugins/x86_64/` — Deployed DLLs (via scripts)

---

## Build the Native Plugin

The native plugin must be built separately for Linux (.so) and Windows (.dll). Unity running on Windows requires the Windows DLL.

### Linux Build (.so)

**Purpose:** C++ tests, CI/CD, and reference implementation.

```bash
cd ~/projects/couch/spike/spike2_native

# Verify SOFA_ROOT is set
echo $SOFA_ROOT
# Expected: /home/<user>/sofa/SOFA_v24.06.00_Linux

# Configure with CMake preset
cmake --preset default

# Build
cmake --build build

# Expected output:
# [100%] Built target SofaAnkleBridge
# Output: build/libSofaAnkleBridge.so
```

**Expected files in `build/` directory:**
```
build/
├── libSofaAnkleBridge.so          # Main plugin (links against SOFA)
├── test_bridge                     # Test executables
├── test_ankle_scene
├── test_scene_builder
├── test_joint_angles
├── test_rom_validation
└── test_resection
```

**If build fails:**
- `SOFA_ROOT must be set` → Run `export SOFA_ROOT=~/sofa/SOFA_v24.06.00_Linux`
- `Could not find Sofa.SimpleApi` → Verify SOFA installation path, check `$SOFA_ROOT/lib/cmake/` exists
- Linker errors about SOFA symbols → Ensure SOFA v24.06.00, not v25.x (see `CMakePresets.json:9`)

---

### Windows Build (.dll)

**Purpose:** Unity integration. Unity Editor running on Windows requires a Windows DLL.

**Important:** This build uses SOFA v25.12 (not v24.06). The `sofa_compat.h` header bridges API differences between SOFA versions.

**From WSL2 (invoking Windows tools):**

```bash
cd /mnt/c/projects/couch/spike/spike2_native

# First time: configure (requires Windows SOFA_ROOT set in environment)
cmd.exe /c "cmake --preset windows"

# Build
cmd.exe /c "cmake --build build-win --config Release"

# Expected output:
# Build succeeded.
# Output: build-win\Release\SofaAnkleBridge.dll
```

**Expected files in `build-win/Release/`:**
```
build-win/Release/
├── SofaAnkleBridge.dll            # Main plugin
├── SofaAnkleBridge.lib            # Import library (not deployed to Unity)
└── SofaAnkleBridge.pdb            # Debug symbols (optional)
```

**If build fails:**
- `SOFA_ROOT not set` → Verify Windows environment variable `SOFA_ROOT=C:\sofa\SOFA_v25.12.00_Win64`
- `Could not find Eigen3` → Run `git submodule update --init` to fetch `third_party/eigen3`
- Visual Studio errors → Ensure VS 2022 (17.x) is installed, not VS 2019

**From Windows PowerShell (alternative):**

```powershell
cd C:\projects\couch\spike\spike2_native
cmake --preset windows
cmake --build build-win --config Release
```

---

## Deploy DLLs to Unity

Unity loads native plugins from `Assets/Plugins/x86_64/`. You must copy the bridge DLL and all 38 SOFA dependency DLLs to this directory.

### Collect SOFA Dependencies (Windows)

**First time only:** Collect SOFA DLLs from `%SOFA_ROOT%\bin` and `%SOFA_ROOT%\lib`.

```powershell
# From Windows PowerShell
cd C:\projects\couch\spike\spike2_native

.\scripts\collect_sofa_deps_win.ps1 `
  -SofaRoot "C:\sofa\SOFA_v25.12.00_Win64" `
  -DllPath ".\build-win\Release\SofaAnkleBridge.dll" `
  -OutputDir ".\sofa_deps"
```

**Expected output:**
```
Collected 38 SOFA DLL dependencies to .\sofa_deps
```

**What this script does:**
1. Uses `dumpbin.exe` to walk the dependency tree of `SofaAnkleBridge.dll`
2. Copies all referenced SOFA DLLs (Sofa.Component.*, Sofa.Core.dll, etc.) and third-party DLLs (tinyxml2.dll, tight_inclusion.dll)
3. Stops at Windows system DLLs (msvcrt, kernel32, etc.)

**Verify dependency collection:**
```powershell
ls .\sofa_deps\*.dll | Measure-Object | Select-Object -ExpandProperty Count
# Expected: 38
```

---

### Deploy to Unity Plugins Directory

**From WSL2 (recommended):**

```bash
cd ~/projects/couch/spike/spike2_native

./scripts/deploy_to_unity.sh
```

This script:
1. Copies `build-win/Release/SofaAnkleBridge.dll` to `unity-project/Assets/Plugins/x86_64/`
2. Copies all `sofa_deps/*.dll` to the same directory

**Expected output:**
```
=== Deploy to Unity Plugins ===
Copied: SofaAnkleBridge.dll
Copied: 38 SOFA dependency DLLs

Deploy complete. Total files in unity-project/Assets/Plugins/x86_64:
39
```

**From Windows PowerShell (alternative):**

```powershell
cd C:\projects\couch\spike\spike2_native

# Copy bridge DLL
Copy-Item .\build-win\Release\SofaAnkleBridge.dll `
  C:\projects\couch\unity-project\Assets\Plugins\x86_64\

# Copy SOFA dependencies
Copy-Item .\sofa_deps\*.dll `
  C:\projects\couch\unity-project\Assets\Plugins\x86_64\
```

---

### Verify Deployment

**Critical:** Close Unity Editor before deploying DLLs. Unity locks DLLs on load, preventing overwrites.

```bash
# From WSL2
ls /mnt/c/projects/couch/unity-project/Assets/Plugins/x86_64/*.dll | wc -l
# Expected: 39

# Check for bridge DLL specifically
ls /mnt/c/projects/couch/unity-project/Assets/Plugins/x86_64/SofaAnkleBridge.dll
# Expected: file exists
```

**If deployment fails:**
- `Permission denied` → Close Unity Editor and retry
- Fewer than 39 DLLs → Re-run `collect_sofa_deps_win.ps1`, check for errors
- DLL loads but functions missing → Version mismatch. Rebuild with matching SOFA version.

**Why `Assets/Plugins/x86_64/`?**
Unity's native plugin search path on Windows x64. Other paths (`Plugins/`, `Assets/`, project root) are ignored. See [Unity Manual: Native Plugins](https://docs.unity3d.com/Manual/NativePlugins.html).

**RPATH and dependency loading:**
The bridge DLL has `INSTALL_RPATH` set to `$ORIGIN` (see `CMakeLists.txt:77`), telling Windows to search for SOFA DLLs in the same directory as the bridge DLL. All 39 DLLs must coexist in `Plugins/x86_64/`.

---

## Run Tests

The COUCH project has three test layers. Run them in order to verify your setup.

### 1. C++ Tests (GTest)

**What they test:** Native plugin logic in isolation from Unity. Scene construction, physics stepping, joint angle math, resection algorithm.

**Platform:** Linux (.so build)

**Total:** 72 tests across 8 test executables

```bash
cd ~/projects/couch/spike/spike2_native/build
ctest
```

**Expected output:**
```
Test project /home/user/projects/couch/spike/spike2_native/build
      Start  1: BridgeTests
 1/8  Test  #1: BridgeTests ......................   Passed    0.12 sec
      Start  2: AnkleSceneTests
 2/8  Test  #2: AnkleSceneTests ..................   Passed    2.15 sec
      Start  3: AsyncTests
 3/8  Test  #3: AsyncTests .......................   Passed    0.47 sec
      Start  4: SceneBuilderTests
 4/8  Test  #4: SceneBuilderTests ................   Passed    1.89 sec
      Start  5: TripleBufferTests
 5/8  Test  #5: TripleBufferTests ................   Passed    0.03 sec
      Start  6: JointAngleTests
 6/8  Test  #6: JointAngleTests ..................   Passed    0.92 sec
      Start  7: ROMValidationTests
 7/8  Test  #7: ROMValidationTests ...............   Passed    3.22 sec
      Start  8: ResectionTests
 8/8  Test  #8: ResectionTests ...................   Passed    1.45 sec

100% tests passed, 0 tests failed out of 8

Total Test time (real) =  10.28 sec
```

**Run a single test suite:**
```bash
cd build
./test_scene_builder
```

**If tests fail:**
- `error while loading shared libraries: libSofa.Core.so` → Set `LD_LIBRARY_PATH`: `export LD_LIBRARY_PATH=$SOFA_ROOT/lib:$LD_LIBRARY_PATH`
- Segfault on `sofa_bridge_init()` → SOFA plugin load failure. Check `SOFA_ROOT` points to v24.06, not v25.x
- Test failures in `ROMValidationTests` → Expected; ROM validation tests verify physics against clinical data (may have tolerance issues)

---

### 2. C# EditMode Tests

**What they test:** Pure C# logic that doesn't require SOFA. Data models, angle math, cut plane geometry, alignment metrics.

**Platform:** Windows (via Unity Test Runner)

**Total:** 37 tests across 4 test classes

**Does NOT require native DLLs.** Safe to run even if deployment failed.

```bash
# From WSL2
cd ~/projects/couch
./unity-test.sh EditMode
```

**Expected output:**
```
=== Unity Test Runner ===
Platform:  EditMode
Project:   C:\projects\couch\unity-project

[... Unity log output ...]

All EditMode tests passed.
```

**What the script does:**
1. Invokes Windows Unity.exe from WSL2 via `/mnt/c/Program Files/...`
2. Runs in headless mode (`-batchmode -nographics`)
3. Writes results to `unity-project/TestResults/EditMode-results.xml`
4. Streams log to stdout (`-logFile -`)

**If tests fail:**
- `Could not open Unity project` → Verify bind mount at `/mnt/c/projects/couch/unity-project` (see `MEMORY.md`)
- Compilation errors → Open project in Unity Editor, check Console for C# errors
- Tests fail with NUnit errors → Check `TestResults/EditMode-results.xml` for details

**View results in Unity Editor (alternative):**
1. Open Unity project: `File > Open Project > C:\projects\couch\unity-project`
2. Window > General > Test Runner
3. EditMode tab > Run All

---

### 3. C# PlayMode Tests

**What they test:** C# ↔ C++ integration through the native bridge. P/Invoke calls, scene construction, ROM engine, resection.

**Platform:** Windows (via Unity Test Runner)

**Total:** 33 tests across 4 test classes

**Requires deployed DLLs.** Fails if `Assets/Plugins/x86_64/SofaAnkleBridge.dll` is missing.

```bash
# From WSL2
cd ~/projects/couch
./unity-test.sh PlayMode
```

**Expected output:**
```
=== Unity Test Runner ===
Platform:  PlayMode

[... Unity log output ...]

All PlayMode tests passed.
```

**Critical difference from EditMode:**
PlayMode tests must pass `pluginDir` to `sofa_bridge_init()`. The directory `Assets/Plugins/x86_64/` contains the SOFA plugin DLLs that SOFA's `PluginManager::loadPluginByPath()` must load.

**Example from `SceneBuilderIntegrationTests.cs:16-17`:**
```csharp
private static string PluginDir =>
    Path.Combine(Application.dataPath, "Plugins", "x86_64");
```

**Why `Application.dataPath`?**
In Unity Editor, `Application.dataPath` resolves to `C:\projects\couch\unity-project\Assets`. The full path becomes `C:\...\Assets\Plugins\x86_64\`.

**Why not pass `null`?**
Passing `null` to `sofa_bridge_init()` skips plugin loading. Scene creation will fail with:
```
Error: Component 'ConstantForceField' not found in SOFA object factory.
```

See manifest topic #11 (DLL load order) and topic #24 (plugin dir paths).

**If PlayMode tests fail:**
- `DllNotFoundException: SofaAnkleBridge` → DLL not deployed. Re-run `deploy_to_unity.sh`
- `EntryPointNotFoundException: sofa_scene_create` → Version mismatch. Rebuild Windows DLL.
- Tests ignored with message `"DLL not available"` → Missing DLL (expected behavior; tests skip gracefully)
- Solver divergence errors (`solverDiverged == 1`) → Physics instability. Check SOFA version matches (v25.12 for Windows)

**View results in Unity Editor (alternative):**
1. Open Unity project in Unity Editor
2. Window > General > Test Runner
3. PlayMode tab > Run All
4. Tests execute in Play Mode (scene loads, simulation steps, teardown)

---

## Assembly Definitions and Test Organization

The Unity project is divided into 5 assemblies to enforce clean dependencies and fast compilation.

**Assembly dependency graph:**
```
AnkleSim.Core             (no dependencies)
    ↓
AnkleSim.Bridge           (depends on Core, has P/Invoke)
    ↓
AnkleSim.Runtime          (depends on Core + Bridge, has MonoBehaviours)

AnkleSim.Tests.EditMode   (depends on Core)
AnkleSim.Tests.PlayMode   (depends on Core + Bridge + Runtime)
```

**Key rules:**
- **AnkleSim.Core** is pure C#, no Unity API, no native calls. Safe to reference from any assembly.
- **AnkleSim.Bridge** has `allowUnsafeCode: true` (required for IntPtr manipulation). Contains all P/Invoke declarations in `SofaNativeBridge.cs`.
- **AnkleSim.Runtime** has MonoBehaviours (`SofaBridgeComponent.cs`, `AnatomyManager.cs`). Only this assembly references UnityEngine.
- **Tests.EditMode** references Core only. Cannot reference Bridge (would pull in unsafe code, not allowed in EditMode).
- **Tests.PlayMode** references all three assemblies. Tests P/Invoke calls and MonoBehaviour lifecycle.

**Why this structure?**
Prevents accidental P/Invoke calls from pure data model code. Isolates unsafe code to one assembly. Enables EditMode tests to run without DLLs.

**Assembly definition files:**
- `Assets/AnkleSim/Core/AnkleSim.Core.asmdef`
- `Assets/AnkleSim/Bridge/AnkleSim.Bridge.asmdef`
- `Assets/AnkleSim/Runtime/AnkleSim.Runtime.asmdef`
- `Assets/Tests/EditMode/AnkleSim.Tests.EditMode.asmdef`
- `Assets/Tests/PlayMode/AnkleSim.Tests.PlayMode.asmdef`

**When to use each test type:**

| Test Type | Use When | Example |
|-----------|----------|---------|
| **C++ GTest** | Testing C++ logic in isolation from Unity | Scene builder state machine, ligament force math, resection centroid algorithm |
| **EditMode** | Testing pure C# logic, no DLLs needed | `ROMRecord` validation, `AnkleAngle` math, `CutPlane` geometry |
| **PlayMode** | Testing C# ↔ C++ integration, DLLs required | Scene creation via P/Invoke, ROM engine stepping, resection command execution |

---

## Coordinate System (SOFA vs Unity)

**SOFA uses Z-up, right-handed.** Unity uses Y-up, left-handed.

**No automatic conversion** in the bridge. The native plugin works entirely in SOFA coordinates (Z-up). If you visualize SOFA meshes in Unity, you must apply a coordinate transform:

```csharp
// SOFA position (x, y, z) where z = vertical
Vector3 sofaPos = new Vector3(x, y, z);

// Unity position (x, z, y) where y = vertical
Vector3 unityPos = new Vector3(sofaPos.x, sofaPos.z, sofaPos.y);
```

**Why no automatic conversion?**
Physics and ROM measurements are independent of visualization. Conversion happens only at the rendering layer (future Sprint 7 UI).

See manifest topic #26 (coordinate system).

---

## Make Your First Change

This guided exercise demonstrates the full workflow: modify C API → update C# mirror → rebuild → deploy → test.

**Goal:** Add a new field `float contact_stiffness` to the `SofaSceneConfig` struct.

---

### Step 1: Modify the C Header

**File:** `spike/spike2_native/include/sofa_ankle_bridge.h`

**Add field to `SofaSceneConfig` struct (after line 75):**

```c
typedef struct {
    float gravity[3];
    float timestep;
    int   constraint_iterations;
    float constraint_tolerance;
    float rayleigh_stiffness;
    float rayleigh_mass;
    float alarm_distance;
    float contact_distance;
    float friction_coefficient;
    float contact_stiffness;        // ← NEW: contact response stiffness (N/mm)
} SofaSceneConfig;
```

---

### Step 2: Update the C++ Implementation

**File:** `spike/spike2_native/src/scene_builder.cpp`

**Find `SceneBuilder::createScene()` (around line 50). Update the collision response setup:**

```cpp
// Around line 100-110, where collision response is configured
auto response = simpleapi::createObject(root, "DefaultContactManager");
response->setParameterValue("response", "FrictionContactConstraint");

// Add this line:
response->setParameterValue("contactStiffness", std::to_string(config.contact_stiffness));
```

**Set default value in the config initialization.** Find where `SofaSceneConfig` is initialized (likely in `scene_builder.cpp` or `sofa_ankle_bridge.cpp`):

```cpp
// Add to default config creation
config.contact_stiffness = 100.0f;  // default 100 N/mm
```

---

### Step 3: Update the C# Mirror Struct

**File:** `unity-project/Assets/AnkleSim/Bridge/NativeStructs.cs`

**Add field to `SofaSceneConfig` struct (same order as C struct!):**

```csharp
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
    public float contactStiffness;        // ← NEW: must match C struct order
}
```

**Update `CreateDefault()` factory method:**

```csharp
public static SofaSceneConfig CreateDefault()
{
    return new SofaSceneConfig
    {
        gravity = new float[] { 0, 0, -9810 },
        timestep = 0.01f,
        constraintIterations = 10,
        constraintTolerance = 1e-3f,
        rayleighStiffness = 0.1f,
        rayleighMass = 0.1f,
        alarmDistance = 0.5f,
        contactDistance = 0.1f,
        frictionCoefficient = 0.8f,
        contactStiffness = 100.0f         // ← NEW: default value
    };
}
```

---

### Step 4: Rebuild and Deploy

**Close Unity Editor** (critical — Unity locks DLLs on load).

```bash
# From WSL2

# 1. Rebuild Windows DLL
cd /mnt/c/projects/couch/spike/spike2_native
cmd.exe /c "cmake --build build-win --config Release"

# 2. Deploy to Unity
./scripts/deploy_to_unity.sh

# 3. Rebuild Linux .so (for C++ tests)
cd ~/projects/couch/spike/spike2_native
cmake --build build
```

---

### Step 5: Verify with a Test

**Add a test to verify the field is marshaled correctly.**

**File:** `unity-project/Assets/Tests/PlayMode/Bridge/SceneBuilderIntegrationTests.cs`

**Add test method:**

```csharp
[UnityTest]
public IEnumerator CreateScene_WithCustomContactStiffness_Succeeds()
{
    Assert.AreEqual(0, SofaNativeBridge.sofa_bridge_init(PluginDir));

    var config = SofaSceneConfig.CreateDefault();
    config.contactStiffness = 200.0f;  // Override default

    int rc = SofaNativeBridge.sofa_scene_create(ref config);
    Assert.AreEqual(0, rc, $"Create scene failed: {SofaNativeBridge.GetErrorString()}");

    // If scene creates successfully, the field was marshaled correctly
    yield return null;
}
```

**Run the test:**

```bash
cd ~/projects/couch
./unity-test.sh PlayMode
```

**Expected output:**
```
Test PlayMode.Bridge.SceneBuilderIntegrationTests.CreateScene_WithCustomContactStiffness_Succeeds passed
```

**If test fails:**
- `AccessViolationException` → Struct field order mismatch. Verify C and C# structs have identical field order.
- `Scene creation failed` → C++ implementation error. Check `sofa_bridge_get_error()` message.
- `EntryPointNotFoundException` → DLL not redeployed. Close Unity and re-run `deploy_to_unity.sh`.

---

### Step 6: Commit Your Change

```bash
git add spike/spike2_native/include/sofa_ankle_bridge.h
git add spike/spike2_native/src/scene_builder.cpp
git add unity-project/Assets/AnkleSim/Bridge/NativeStructs.cs
git add unity-project/Assets/Tests/PlayMode/Bridge/SceneBuilderIntegrationTests.cs
git commit -m "feat: add contact_stiffness parameter to SofaSceneConfig"
```

---

## Next Steps

You can now:
- **Build the native plugin** for Linux and Windows
- **Deploy DLLs** to Unity's plugin directory
- **Run all three test layers** (C++ GTest, C# EditMode, C# PlayMode)
- **Modify structs** across the native boundary

**Further reading:**
- [`docs/dev/native-boundary.md`](./native-boundary.md) — Memory ownership, P/Invoke patterns, error handling
- [`docs/dev/sofa-patterns.md`](./sofa-patterns.md) — SOFA scene graph, ligament model, solver stability
- [`docs/dev/testing.md`](./testing.md) — Advanced testing patterns, mocking, test data
- [`docs/dev/api-reference-c.md`](./api-reference-c.md) — Complete C API reference (26 functions)
- [`docs/adr/`](../adr/) — Architecture decisions (resection algorithm, undo strategy, dual-representation cutting)

**Common pitfalls:**
- Forgetting to close Unity before rebuilding DLL → `Permission denied` on deploy
- Mismatched struct field order between C and C# → silent data corruption
- Missing `try/finally` around P/Invoke calls with IntPtr → memory leak
- Passing `null` for `pluginDir` in PlayMode tests → SOFA component not found

**If you get stuck:**
1. Check the error message from `sofa_bridge_get_error()`
2. Search this documentation for the error symptom (see manifest topics for common failure modes)
3. Verify all 39 DLLs are deployed to `Assets/Plugins/x86_64/`
4. Run C++ tests first to isolate whether the problem is in the native plugin or the Unity integration

**Welcome to the COUCH project.** You're ready to develop.
