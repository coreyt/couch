# Architecture Decision Records Index

This page is your navigation hub for architectural decisions in the COUCH project. ADRs document the "why" behind significant design choices that affect multiple components — not every decision gets an ADR, only the ones that prevent costly rework later.

## What is an ADR?

An Architecture Decision Record (ADR) captures:
- The **context** (forces at play, alternatives considered)
- The **decision** (what we chose and why)
- The **consequences** (tradeoffs we're accepting)

ADRs are not requirements documents or implementation guides. They're the hard-won lessons that explain why the system is shaped the way it is. Read an ADR when you're about to refactor something fundamental and need to understand what assumptions it rests on.

## When to Create a New ADR

Create an ADR when you're about to make a decision that:
1. Affects more than one component or layer (C#/C++/SOFA)
2. Rules out other reasonable approaches (not just a minor implementation detail)
3. Has consequences you'll live with for multiple sprints
4. Required tradeoff analysis among viable options

Do NOT create an ADR for: parameter tuning, local refactoring, bug fixes, or component-specific design.

## ADR Template

All ADRs follow this structure:

```markdown
# ADR-NNNN: [Decision Title]

**Status:** Accepted | Proposed | Superseded
**Date:** YYYY-MM-DD
**Sprint:** N
**Relates to:** doc references, Jira issues

## Context
(Problem statement + alternatives considered)

## Decision
(What we chose)

## Rationale
(Why this choice over alternatives)

## Consequences
(Tradeoffs, what we're accepting, what future work depends on this)
```

---

## ADR-0001: Centroid-Based Tetrahedral Resection

**Summary:** When removing tetrahedra from a FEM mesh to simulate bone resection, classify each tetrahedron by whether its centroid (geometric center) falls on the distal side of the cut plane. This is simpler than exact intersection remeshing and cleaner than vertex-based removal.

**When to read:** You're implementing the resection algorithm, modifying how tetrahedra are selected for removal, or wondering why we don't split tetrahedra at element boundaries.

**Full document:** [ADR-0001](../adr/0001-centroid-based-resection.md)

**Implementation:**
- C++ resection logic: [`spike/spike2_native/src/ankle_scene.cpp`](../../spike/spike2_native/src/ankle_scene.cpp) (centroid calculation and element selection)
- C API: [`spike/spike2_native/include/sofa_ankle_bridge.h:192`](../../spike/spike2_native/include/sofa_ankle_bridge.h#L192) — `sofa_execute_resection()`
- C++ tests: [`spike/spike2_native/test/test_resection.cpp:124–176`](../../spike/spike2_native/test/test_resection.cpp#L124) (centroid-based removal validation)
- C# integration: [`unity-project/Assets/AnkleSim/Bridge/Resection/ResectionEngine.cs:19–65`](../../unity-project/Assets/AnkleSim/Bridge/Resection/ResectionEngine.cs#L19)

**Related docs:** [sofa-patterns.md § Resection](sofa-patterns.md#resection-bone-cutting) — explains the SOFA topology API this decision depends on

---

## ADR-0002: Dual-Representation Cutting (EzySlice Visual + SOFA FEM)

**Summary:** Bone resection requires two independent meshes: a visual mesh (rendered by Unity, smooth polygon clipping via EzySlice) and a physics mesh (FEM tetrahedra in SOFA, element removal). The `ResectionEngine` orchestrates both from a single cut plane.

**When to read:** You're implementing the UI cut plane controller, extending the resection workflow, or wondering why we don't just cut the mesh in SOFA and read it back for rendering.

**Full document:** [ADR-0002](../adr/0002-dual-representation-cutting.md)

**Implementation:**
- Dual-cut orchestration: [`unity-project/Assets/AnkleSim/Bridge/Resection/ResectionEngine.cs:19–65`](../../unity-project/Assets/AnkleSim/Bridge/Resection/ResectionEngine.cs#L19)
- EzySlice integration hook: [`unity-project/Assets/AnkleSim/Bridge/Resection/ResectionEngine.cs:121–133`](../../unity-project/Assets/AnkleSim/Bridge/Resection/ResectionEngine.cs#L121)
- SOFA surface readback fallback: [`unity-project/Assets/AnkleSim/Bridge/Resection/ResectionEngine.cs:51–56`](../../unity-project/Assets/AnkleSim/Bridge/Resection/ResectionEngine.cs#L51)
- C API: [`spike/spike2_native/include/sofa_ankle_bridge.h:205`](../../spike/spike2_native/include/sofa_ankle_bridge.h#L205) — `sofa_get_surface_mesh()`

**Related docs:** [sofa-patterns.md § Resection](sofa-patterns.md#resection-bone-cutting) — explains when and why to use SOFA surface readback

---

## ADR-0003: Undo via Scene Rebuild

**Summary:** When a user undoes a bone resection to try a different cut plane, the system destroys the entire SOFA scene and rebuilds it from the original configuration. This is reliable and simple, trading performance (rebuild takes ~100ms) for maintainability over fragile element re-insertion or serialization approaches.

**When to read:** You're implementing the undo UI, extending the resection workflow with multi-level undo, or modifying the scene lifecycle to understand why we can't just "restore" the removed elements.

**Full document:** [ADR-0003](../adr/0003-undo-via-scene-rebuild.md)

**Implementation:**
- Undo pattern: [`unity-project/Assets/AnkleSim/Bridge/Resection/ResectionEngine.cs:67–81`](../../unity-project/Assets/AnkleSim/Bridge/Resection/ResectionEngine.cs#L67)
- Scene destruction and rebuild: [`unity-project/Assets/AnkleSim/Bridge/SofaSimulation.cs`](../../unity-project/Assets/AnkleSim/Bridge/SofaSimulation.cs) — `DestroyScene()` method
- C API: [`spike/spike2_native/include/sofa_ankle_bridge.h`](../../spike/spike2_native/include/sofa_ankle_bridge.h) — `sofa_scene_destroy()`, `sofa_scene_create()`, etc.

**Related docs:** [native-boundary.md](native-boundary.md) — memory ownership of scene state during rebuild; [sofa-patterns.md § Scene Lifecycle](sofa-patterns.md#scene-construction-lifecycle) — the state machine that makes rebuild possible

---

## Cross-References

- **[sofa-patterns.md](sofa-patterns.md)** — discusses resection patterns, SOFA topology API contracts, and scene lifecycle (prerequisites for understanding all three ADRs)
- **[native-boundary.md](native-boundary.md)** — covers memory ownership when crossing C#/C++ boundary during resection data transfer
- **[doc-12-unified-implementation-plan.md](../../docs/12-unified-implementation-plan.md)** — high-level sprint plan that explains *why* each ADR matters to the sprint it's assigned to
