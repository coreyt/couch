# ADR-0003: Undo via Scene Rebuild

**Status:** Accepted
**Date:** 2026-02-15
**Sprint:** 4 (Resection Engine)
**Relates to:** doc 12 D-06, doc 08 §Resection Engine

## Context

Users need to undo a bone resection to try a different cut plane. SOFA's topology modification API (`TetrahedronSetTopologyModifier::removeTetrahedra`) permanently removes elements — there is no built-in undo or element re-insertion.

Three undo strategies were considered:

1. **Snapshot/restore**: serialize full SOFA scene state before cut, deserialize to restore.
2. **Scene rebuild**: destroy the entire SOFA scene, recreate from the original configuration.
3. **Element re-insertion**: track removed elements, re-insert them into the topology.

## Decision

Use **scene rebuild** for undo. `ResectionEngine.UndoCut()` calls `SofaSimulation.DestroyScene()`. The caller is responsible for recreating the scene from the original configuration (create scene → add bones → add ligaments → add deformable tissue → finalize).

## Rationale

- **SOFA has no topology undo API**: `removeTetrahedra` is destructive. Re-inserting elements would require maintaining index maps and re-wiring topology connectivity — fragile and error-prone.
- **Scene serialization is complex**: SOFA's XML/JSON serialization doesn't cleanly round-trip dynamic scenes with resolved typed pointers, custom controllers, and runtime state.
- **Scene rebuild is reliable**: the `SceneBuilder` already supports the full create→add→finalize lifecycle. Rebuilding produces a known-good scene identical to the original.
- **Performance is acceptable**: scene construction takes <100ms for typical ankle scenes (2 bones, 4 ligaments, 1 deformable tissue). This is well under the 500ms target for user-facing operations.
- **Matches doc 12 D-06** which specifies this approach.

## Consequences

- Undo discards all simulation state (step count, ligament forces, joint angles). The scene restarts from initial conditions. This is acceptable because undo is a "reset to pre-cut" operation.
- The caller must cache the original scene configuration (bone configs, ligament configs, tissue config) to rebuild. This responsibility lives outside `ResectionEngine`.
- Multiple undo levels are not supported — only the last cut can be undone. Multi-level undo would require a stack of scene configurations, deferred to a future sprint if needed.
- The pre-cut visual mesh is cached by `ResectionEngine` and restored on undo.
