# COUCH — Couch Operates the Unity Connection Handler

## Origin

Extracted from `coreyt/gists-and-scratchpads/unity-sofa-integration/` on 2026-02-14 via `git subtree split`, preserving full commit history. The project outgrew the scratchpad — 120 files, 3.9MB, 12 design docs, a C++ native library with 32 tests, and C# stubs.

## Purpose

Surgical simulation of Total Ankle Replacement (TAR). Unity 6 handles real-time visualization; SOFA Framework (v24.06.00) handles FEM physics via a custom C++ native plugin (`SofaAnkleBridge`) with a flat C API — no SofaUnity or third-party middleware.

The ankle joint is modeled as an **emergent joint**: tibia (fixed rigid body) + talus (free rigid body) + ligament springs + bone-bone collision. ROM naturally emerges from these constraints rather than from an explicit kinematic joint.

## What exists (completed spikes + sprint 0)

### Spike 1 — Python proof-of-concept (`spike/spike1_python/`)
- SofaPython3 ankle scene with controller-based ligaments (ConstantForceField, NOT StiffSpringForceField)
- 3 bone STL meshes (tibia, talus, fibula from BodyParts3D)
- ROM validation tests (pytest) against clinical ranges (Glazebrook 2008: 22-31 deg)
- Anatomical ligament attachments (anterior + posterior for bidirectional resistance)

### Spike 2 — C++ build chain (`spike/spike2_native/`)
- CMake project linking against SOFA v24.06.00 pre-built binaries
- Minimal native plugin: `sofa_bridge_init`, `sofa_step`, `sofa_bridge_shutdown`
- Google Test suite validating build chain, lifecycle, and plugin loading
- `collect_sofa_deps.sh` for transitive dependency collection

### Sprint 0 — Native bridge foundation (`spike/spike2_native/`)
- C++ ankle scene ported from Python (spike 3), with ligament controller and behavioral tests
- Version handshake (`SofaBridgeVersion` struct)
- Async stepping with thread manager (background step + cancellation)
- C# P/Invoke stubs: `SofaNativeBridge.cs`, `NativeStructs.cs`, `SofaSimulation.cs`, `SofaBridgeComponent.cs`, `SofaBridgeException.cs`

### Docs (`docs/`)
- 4 research docs (clinical TAR, Unity 6, SOFA Framework, STL mesh sources)
- 8 design docs (user needs, architecture, requirements, component designs, implementation plans)
- Authoritative plan: `docs/12-unified-implementation-plan.md` (supersedes docs 09-11)
- Spike plan: `docs/20260212-vertical-spike-implementation-plan.md`

## Architecture

```
Unity 6 Process
├── C# Managed Layer (AnkleSim.Bridge assembly)
│   ├── SofaNativeBridge.cs        ← P/Invoke declarations
│   ├── NativeStructs.cs           ← C# struct mirrors
│   ├── SofaSimulation.cs          ← High-level API
│   ├── SofaBridgeComponent.cs     ← MonoBehaviour [DefaultExecutionOrder(-100)]
│   └── SofaBridgeException.cs     ← Domain-specific exceptions
│
├── Native Plugin (SofaAnkleBridge.dll/.so)
│   ├── sofa_ankle_bridge.h        ← Public C API header
│   ├── ankle_scene.cpp/h          ← Scene construction + ligament controller
│   ├── sofa_ankle_bridge.cpp      ← Flat C API (extern "C")
│   └── thread_manager.cpp/h       ← Background stepping + cancellation
│
└── SOFA Libraries (linked at build time from SOFA_ROOT)
```

## SOFA patterns (hard-won lessons)

- **StiffSpringForceField fails across RigidMapping** — use a Python/C++ controller + ConstantForceField instead
- **Write forces in onAnimateBeginEvent**, not direct velocity modification — EulerImplicit overwrites velocities
- **Ligaments are tension-only** — skip force when extension <= 0
- **Ligaments must resist both directions** — need anterior AND posterior attachments for sagittal plane ROM
- **One-step lag with ConstantForceField** — forces computed at step N apply at step N+1; high stiffness (>3x) causes divergence; use damping (~5 N*s/mm)

## Build

```bash
# SOFA must be installed
export SOFA_ROOT=~/sofa/SOFA_v24.06.00_Linux

# Build native plugin
cd spike/spike2_native
cmake --preset default
cmake --build build

# Run C++ tests
cd build && ctest

# Run Python tests (spike 1)
cd spike/spike1_python
PYTHONPATH=$SOFA_ROOT/plugins/SofaPython3/lib/python3/site-packages pytest
```

## Next steps (doc 12 sprint plan)

The spikes are done. The next major milestone is creating the Unity 6 project and wiring the native plugin into it. From doc 12's sprint plan:

1. **Sprint 1** — Core data models and anatomy loading in Unity (ROMRecord, AlignmentMetrics, AnatomyManager)
2. **Sprint 2** — SOFA scene construction API (rigid bones, deformable tissue, ligaments) + mesh sync (triple-buffered snapshots, Burst jobs)
3. **Sprint 3** — ROM engine (angle measurement from emergent joint, torque-driven ROM sweep)
4. **Sprint 4** — Resection engine (dual-representation: EzySlice visual + SOFA centroid-based FEM)
5. **Sprint 5** — Implant system (library, 6-DOF positioning, alignment metrics)
6. **Sprint 6** — Post-op ROM and comparison engine
7. **Sprint 7** — UI and workflow state machine
8. **Sprint 8** — E2E integration and biomechanical validation
9. **Sprint 9** — Polish, packaging, deployment

The immediate next step is creating a Unity 6 project in this repo, setting up assembly definitions, deploying the native plugin DLLs to `Assets/Plugins/x86_64/`, and writing the first PlayMode tests that call through P/Invoke.
