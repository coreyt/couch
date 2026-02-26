# ADR-0001: Centroid-Based Tetrahedral Resection

**Status:** Accepted
**Date:** 2026-02-15
**Sprint:** 4 (Resection Engine)
**Relates to:** doc 12 §1.10, doc 08 §Resection Engine

## Context

The resection engine needs to remove tetrahedra from a FEM mesh along a cut plane to simulate bone resection in Total Ankle Replacement surgery. Three approaches were considered for deciding which tetrahedra to remove:

1. **Vertex-based**: Remove any tetrahedron with at least one vertex below the plane.
2. **Centroid-based**: Remove any tetrahedron whose centroid (average of 4 vertices) falls below the plane.
3. **Exact intersection**: Split tetrahedra at the plane boundary, creating new elements.

## Decision

Use **centroid-based removal**. For each tetrahedron, compute the centroid as the average of its 4 vertex positions. Remove the tetrahedron if `dot(centroid - plane_point, plane_normal) < 0`.

## Rationale

- **Smoother boundary** than vertex-based: vertex-based removal is too aggressive — a single vertex barely crossing the plane causes the entire element to be removed, producing a jagged cut surface.
- **Much simpler than exact intersection**: splitting tetrahedra at the plane requires remeshing, inserting new vertices, and maintaining element quality — significant complexity for marginal benefit in a surgical planning tool (not intra-operative guidance).
- **Matches SOFA's topology modification API**: `TetrahedronSetTopologyModifier::removeTetrahedra()` accepts a list of element indices to remove. Centroid classification maps directly to this API.
- **Consistent with doc 12 §1.10** which specifies this approach.

## Consequences

- Cut surfaces will not be geometrically smooth at the element boundary — the visual surface follows the remaining tetrahedra faces. This is acceptable for surgical planning where ~1mm resolution is clinically sufficient.
- The `Tetra2TriangleTopologicalMapping` automatically updates the surface triangulation after removal, providing the post-cut visual mesh.
- Undo requires full scene rebuild (see ADR-0003) since SOFA topology modifications are not reversible.

## Build Note

`Sofa.Component.IO.Mesh` was originally planned but requires ZLIB (not available as dev headers in the build environment). It is not needed — meshes are passed programmatically via vertex/tetrahedra arrays, not loaded from files. `Sofa.Component.LinearSolver.Ordering` is a transitive dependency of `Sofa.Component.LinearSolver.Direct` and must be explicitly listed in CMake.
