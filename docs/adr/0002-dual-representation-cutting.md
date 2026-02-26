# ADR-0002: Dual-Representation Cutting (EzySlice Visual + SOFA FEM)

**Status:** Accepted
**Date:** 2026-02-15
**Sprint:** 4 (Resection Engine)
**Relates to:** doc 12 §4.1, doc 08 §Resection Engine

## Context

Bone resection needs both a visual representation (for rendering in Unity) and a physics representation (for post-resection ROM simulation in SOFA). These serve different purposes:

- **Visual**: immediate feedback, smooth mesh, rendered with Unity's pipeline.
- **Physics**: tetrahedral FEM mesh in SOFA, drives post-resection biomechanical simulation.

Three architectures were considered:

1. **SOFA-only**: perform the cut in SOFA, read back the surface mesh for Unity rendering.
2. **Unity-only**: perform the cut visually in Unity, approximate the physics effect.
3. **Dual-representation**: cut independently in both systems, keep them synchronized.

## Decision

Use **dual-representation cutting**: EzySlice for instant visual mesh slicing in Unity, SOFA centroid-based tetrahedral removal for physics. The `ResectionEngine` orchestrates both cuts from a single `CutPlaneController` state.

When EzySlice is not available, **fall back to SOFA surface mesh readback** via `Tetra2TriangleTopologicalMapping` → `sofa_get_surface_mesh()` → Unity `Mesh`.

## Rationale

- **EzySlice produces clean visual cuts** with proper UV preservation and smooth edges — better visual quality than SOFA's element-boundary surfaces.
- **SOFA's FEM topology must be modified for accurate post-resection ROM** — a visual-only cut would leave the physics mesh intact, giving incorrect biomechanical results.
- **Decoupling** allows each system to use its optimal cut strategy: smooth polygon clipping for visuals, element removal for FEM.
- **Fallback path** ensures the system works without EzySlice during development — SOFA surface readback provides a functional (if less polished) visual mesh.

## EzySlice Installation

EzySlice (github.com/DavidArayan/ezyslice) does not provide a `package.json` and cannot be imported as a UPM git dependency. It must be manually copied to `Assets/Plugins/EzySlice/`. The `ResectionEngine.TryVisualCut()` method contains the integration hook.

## Consequences

- Two mesh representations must be kept conceptually consistent — same cut plane applied to both. The `CutPlaneController` is the single source of truth for plane geometry.
- Visual and physics cuts may not match at element boundaries (visual is smooth, physics follows tetrahedra). This is acceptable for surgical planning.
- If EzySlice is not installed, `ResectionRecord.visualCutComplete` is still true (via SOFA readback) but the mesh quality is lower.
