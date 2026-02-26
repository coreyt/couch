# COUCH — Couch Operates the Unity Connection Handler

## Origin

Extracted from `coreyt/gists-and-scratchpads/unity-sofa-integration/` on 2026-02-14 via `git subtree split`, preserving full commit history. The project outgrew the scratchpad — 120 files, 3.9MB, 12 design docs, a C++ native library with 32 tests, and C# stubs.

## Purpose

Surgical simulation of Total Ankle Replacement (TAR). Unity 6 handles real-time visualization; SOFA Framework handles FEM physics via a custom C++ native plugin (`SofaAnkleBridge`) with a flat C API — no SofaUnity or third-party middleware. Linux builds against SOFA v24.06; Windows builds against SOFA v25.12 (cross-version compatibility via `sofa_compat.h`).

The ankle joint is modeled as an **emergent joint**: tibia (fixed rigid body) + talus (free rigid body) + ligament springs + bone-bone collision. ROM naturally emerges from these constraints rather than from an explicit kinematic joint.

## What exists (completed through Sprint 4.6)

### Spike 1 — Python proof-of-concept (`spike/spike1_python/`)
- SofaPython3 ankle scene with controller-based ligaments (ConstantForceField, NOT StiffSpringForceField)
- 28 bone STL meshes from BodyParts3D (tibia, fibula, talus, calcaneus, navicular, cuboid, 3 cuneiforms, 5 metatarsals, 14 phalanges)
- ROM validation tests (pytest) against clinical ranges (Glazebrook 2008: 22-31 deg)
- Anatomical ligament attachments (anterior + posterior for bidirectional resistance)

### Spike 2 + Sprints 0–4.6 — Native plugin & Unity integration (`spike/spike2_native/`)
- **C++ native plugin** (18 source files): ankle scene, scene builder API, thread manager, resection engine
- **Cross-version compatibility**: `sofa_compat.h` bridges SOFA v24.06 (Linux) and v25.12 (Windows) API differences
- **SofaFrameSnapshot v0.3.0**: tibia + talus + calcaneus frames, name-based bone lookup
- **72 C++ tests** (GTest): build chain, lifecycle, async, scene builder, joint angles, ROM validation, resection
- **Windows DLL**: built via Visual Studio 2022, 39 DLLs deployed to `Assets/Plugins/x86_64/` (bridge + 36 SOFA + 2 third-party)

### Unity project (`unity-project/`)
- **5 assembly definitions**: AnkleSim.Core, AnkleSim.Bridge, AnkleSim.Runtime, Tests.EditMode, Tests.PlayMode
- **25 C# source files**: P/Invoke bridge, data models, ROM engine, resection engine, anatomy manager, bone visualizer, orbit camera
- **14 C# test files**: 9 EditMode + 5 PlayMode
- **44 EditMode tests**: data models, angle math, alignment, cut plane geometry, bone types, materials
- **41 PlayMode tests**: scene builder integration, ROM engine, resection, calcaneus, visualization (all pass against Windows DLL)
- **Test runner**: `unity-test.sh` — headless batch mode from WSL2

### Docs (`docs/`)
- 4 research docs (clinical TAR, Unity 6, SOFA Framework, STL mesh sources)
- 8 design docs (user needs, architecture, requirements, component designs, implementation plans)
- 3 ADRs: centroid-based resection, dual-representation cutting, undo via scene rebuild
- Authoritative plan: `docs/12-unified-implementation-plan.md` (supersedes docs 09-11)
- **Developer documentation suite** (`docs/dev/`) — 7 files, 5,668 lines:
  - `onboarding.md` — zero-to-passing-tests in under an hour
  - `native-boundary.md` — memory ownership, try/finally, struct/function checklists
  - `sofa-patterns.md` — emergent joint, ligaments, solver params, resection
  - `api-reference-c.md` — all 26 C API functions with full contracts
  - `testing.md` — C++ GTest, EditMode, PlayMode patterns + checklists
  - `adr-index.md` — ADR summaries with "when to read" triggers
  - `documentation-manifest.md` — 28-topic blueprint used to generate the suite

### Agent definitions (`.agents/`)
- `code-cartographer.md` — read-only analysis agent that discovers undocumented knowledge (6 danger categories)
- `technical-writer.md` — documentation agent that consumes the manifest and produces developer guides

## Architecture

```
Unity 6 Process
├── C# Managed Layer
│   ├── AnkleSim.Core              ← Data models, angle math, cut plane geometry
│   ├── AnkleSim.Bridge            ← P/Invoke, SofaSimulation, ROM engine, resection engine
│   └── AnkleSim.Runtime           ← MonoBehaviours, AnatomyManager
│
├── Native Plugin (Assets/Plugins/x86_64/SofaAnkleBridge.dll)
│   ├── sofa_ankle_bridge.h        ← Public C API header (26 functions)
│   ├── scene_builder.cpp/h        ← Scene construction: rigid bones, deformable tissue, resection
│   ├── ankle_scene.cpp/h          ← Legacy ankle scene + ligament controller
│   ├── sofa_ankle_bridge.cpp      ← Flat C API (extern "C") + plugin loading
│   ├── thread_manager.cpp/h       ← Background stepping + cancellation
│   └── sofa_compat.h              ← SOFA v24.06/v25.12 compatibility macros
│
└── SOFA Libraries (38 DLLs co-deployed in Assets/Plugins/x86_64/)
```

## SOFA patterns (hard-won lessons)

- **StiffSpringForceField fails across RigidMapping** — use a Python/C++ controller + ConstantForceField instead
- **Write forces in onAnimateBeginEvent**, not direct velocity modification — EulerImplicit overwrites velocities
- **Ligaments are tension-only** — skip force when extension <= 0
- **Ligaments must resist both directions** — need anterior AND posterior attachments for sagittal plane ROM
- **One-step lag with ConstantForceField** — forces computed at step N apply at step N+1; high stiffness (>3x) causes divergence; use damping (~5 N*s/mm)
- **SOFA plugins in Unity require explicit path loading** — `importPlugin()` can't find DLLs in Unity's plugin dir; must use `PluginManager::loadPluginByPath()` with full paths
- **SOFA v25.12 breaking change** — `ConstVecCoordId::position()` deleted; use `sofa::core::vec_id::read_access::position` constexpr (see `sofa_compat.h`)
- **Centroid-based resection** — remove tetrahedra whose centroid is on the cut side; smoother boundary than vertex-based removal (see ADR 0001)

## Build

```bash
# --- Linux (SOFA v24.06) ---
export SOFA_ROOT=~/sofa/SOFA_v24.06.00_Linux
cd spike/spike2_native
cmake --preset default
cmake --build build
cd build && ctest                    # 72 C++ tests

# --- Windows DLL (SOFA v25.12, from WSL) ---
cmd.exe /c "cd C:\projects\couch\spike\spike2_native && cmake --build build-win --config Release"
# Deploy: copy build-win/Release/SofaAnkleBridge.dll → Assets/Plugins/x86_64/

# --- Unity tests (from WSL) ---
./unity-test.sh EditMode             # 44 tests (pure C#, no DLL needed)
./unity-test.sh PlayMode             # 41 tests (requires deployed DLL)

# --- Python tests (spike 1) ---
cd spike/spike1_python
PYTHONPATH=$SOFA_ROOT/plugins/SofaPython3/lib/python3/site-packages pytest
```

## Sprint status (doc 12 sprint plan)

| Sprint | Status | Tests |
|--------|--------|-------|
| Sprint 1 — Core data models, anatomy loading | Done | 30 EditMode |
| Sprint 2 — Scene construction API, rigid bones, ligaments | Done | 12 PlayMode |
| Sprint 3 — ROM engine, angle math, torque-driven sweep | Done | 7 EditMode, 6 PlayMode |
| Sprint 4 — Resection engine (centroid-based FEM + cut plane) | Done | 7 EditMode, 7 PlayMode |
| Windows DLL deployment — cross-platform bridge operational | Done | 33 PlayMode (0 skipped) |
| Sprint 4.5 — Foot bone anatomy, calcaneus simulation | Done | 7 EditMode, 8 PlayMode |
| Sprint 4.6 — Unity bone visualization | Done | — (tests in 4.5 counts) |
| Sprint 5 — Implant system (library, 6-DOF positioning) | Not started | — |
| Sprint 6 — Post-op ROM and comparison engine | Not started | — |
| Sprint 7 — UI and workflow state machine | Not started | — |
| Sprint 8 — E2E integration and biomechanical validation | Not started | — |
| Sprint 9 — Polish, packaging, deployment | Not started | — |

**Test totals**: 72 C++ + 44 EditMode + 41 PlayMode = **157 tests**, all passing.
