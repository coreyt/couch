# STL Mesh Sources for Spike 1: Tibia & Talus

> **Date:** 2026-02-12
> **Purpose:** Identify downloadable STL models for the standalone SofaPython3 ankle joint scene

---

## Recommended Source: BodyParts3D (GitHub)

**Best fit for Spike 1.** Individual bone STL files, CC license, anatomically accurate from a standardized whole-body model, reasonable mesh size for simulation.

| Bone | FMA ID | File | Size | Download URL |
|------|--------|------|------|-------------|
| Right tibia | FMA24477 | `FMA24477.stl` | 343 KB | `https://raw.githubusercontent.com/Kevin-Mattheus-Moerman/BodyParts3D/main/assets/BodyParts3D_data/stl/FMA24477.stl` |
| Left tibia | FMA24478 | `FMA24478.stl` | 341 KB | `https://raw.githubusercontent.com/Kevin-Mattheus-Moerman/BodyParts3D/main/assets/BodyParts3D_data/stl/FMA24478.stl` |
| Right talus | FMA24482 | `FMA24482.stl` | 298 KB | `https://raw.githubusercontent.com/Kevin-Mattheus-Moerman/BodyParts3D/main/assets/BodyParts3D_data/stl/FMA24482.stl` |
| Left talus | FMA24483 | `FMA24483.stl` | 296 KB | `https://raw.githubusercontent.com/Kevin-Mattheus-Moerman/BodyParts3D/main/assets/BodyParts3D_data/stl/FMA24483.stl` |

**Repository:** https://github.com/Kevin-Mattheus-Moerman/BodyParts3D

**License:** Creative Commons Attribution-Share Alike 2.1 Japan. Attribution: "BodyParts3D, (c) The Database Center for Life Science licensed under CC Attribution-Share Alike 2.1 Japan"

**Why this source:**
- Separate STL per bone (no need to split a combined ankle mesh)
- ~300 KB files = reasonable polygon count for SOFA contact detection
- Consistent coordinate system across all bones (from same whole-body model)
- Git-hosted = scriptable download, no account required

**Potential issue:** These are full-length bones. The tibia STL is the entire bone, not just the distal (ankle) end. For Spike 1 this is fine — we can use the full bone as a rigid body. For later spikes, may want to trim to just the distal tibia to reduce collision mesh complexity.

---

## Alternative Sources

### 1. Thingiverse — DrGlassDPM Foot & Lower Extremity v2.0

- **URL:** https://www.thingiverse.com/thing:22628
- **Contents:** Separate ankle and knee joint files with fibula, tibia, patella, distal femur, and a "completely revamped talus"
- **License:** CC Attribution 4.0
- **Pros:** Anatomically detailed, separate bone files, designed for articulation
- **Cons:** May need account to download; mesh density unknown; coordinate alignment between files not guaranteed

### 2. embodi3D — CT-Scan-Based Models

- **URLs:**
  - Right foot/ankle: https://www.embodi3d.com/files/file/8910-normal-right-foot-and-ankle-bone-model-3d-printable-stl-file-converted-from-ct-scan/
  - Left foot/ankle: https://www.embodi3d.com/files/file/8582-normal-left-foot-and-ankle-bone-model-3d-printable-stl-file-converted-from-ct-scan/
  - Processed ankle: https://www.embodi3d.com/files/file/56532-3d-ankle-stl-file-processed/
- **License:** Varies per model (check individual pages)
- **Pros:** Real CT scan data = high anatomical accuracy; multiple models available
- **Cons:** Likely combined mesh (all ankle bones in one STL) — would need segmentation; requires free account; mesh may be very dense (designed for 3D printing, not simulation)

### 3. SimTK — Multi-Segment Foot and Ankle Model

- **URL:** https://simtk.org/frs/?group_id=2092
- **Contents:** OpenSim model with geometry files for tibia, talus, calcaneus, midfoot, forefoot, toes
- **Reference:** Maharaj et al. (2021) — validated with biplanar videoradiography
- **Pros:** Research-grade; validated against motion data; includes joint definitions
- **Cons:** Geometry format may be VTP/OBJ (not STL); designed for OpenSim, not SOFA; may need conversion

### 4. University of Bath — Open Ankle Models Project

- **URL:** https://www.bath.ac.uk/projects/open-ankle-models-project/
- **Contents:** Open-source FE ankle model with tibia, talus, calcaneus, fibula, navicular + ligaments + cartilage
- **Reference:** Published at Bone & Joint 2023
- **Pros:** Includes ligament attachment points; FE-ready meshes; includes material properties
- **Cons:** May not be publicly downloadable yet (research project); uses FEBio format, would need conversion

### 5. Sketchfab / Printables — Scanned Specimens

- **Sketchfab tibia:** https://sketchfab.com/3d-models/human-tibia-7c1979d6127749bc80a9d9276d24edcd (Elon University anatomy lab scan)
- **Printables tibia+fibula:** https://www.printables.com/model/180298-human-tibia-and-fibula
- **Pros:** Free; real bone scans
- **Cons:** No matching talus from same specimen; inconsistent coordinate systems; 3D-print-optimized meshes may need decimation

### 6. awesome-biomechanics Dataset (34-Subject Talus)

- **URL:** https://github.com/modenaxe/awesome-biomechanics
- **Contents:** Manually segmented 3D bone geometry models (.STL) from MRI of 34 subjects, including talus geometries
- **Pros:** Statistical shape model data; multiple specimens
- **Cons:** Research dataset — may require data use agreement; talus only (no matching tibia)

---

## Recommendation for Spike 1

**Use BodyParts3D right tibia (`FMA24477.stl`) and right talus (`FMA24482.stl`).**

Rationale:
1. Same coordinate system (both from same whole-body model)
2. Direct download via raw GitHub URLs (no account, no conversion)
3. ~300 KB each = manageable polygon count for SOFA collision detection
4. CC-licensed for research use
5. If mesh is too dense for SOFA contact, can decimate with MeshLab/`trimesh` before loading

### Download Commands

```bash
mkdir -p unity-sofa-integration/spike/spike1_python/meshes
cd unity-sofa-integration/spike/spike1_python/meshes

# Right tibia (distal end articulates with talus)
curl -L -o tibia_right.stl \
  "https://raw.githubusercontent.com/Kevin-Mattheus-Moerman/BodyParts3D/main/assets/BodyParts3D_data/stl/FMA24477.stl"

# Right talus
curl -L -o talus_right.stl \
  "https://raw.githubusercontent.com/Kevin-Mattheus-Moerman/BodyParts3D/main/assets/BodyParts3D_data/stl/FMA24482.stl"
```

### Post-Download Mesh Prep (if needed)

If the full-bone meshes cause performance issues in SOFA contact detection:

```python
# Decimate with trimesh (pip install trimesh)
import trimesh

mesh = trimesh.load('tibia_right.stl')
print(f"Original: {len(mesh.faces)} faces")

# Decimate to ~2000 faces for collision mesh
decimated = mesh.simplify_quadric_decimation(2000)
decimated.export('tibia_right_collision.stl')
print(f"Decimated: {len(decimated.faces)} faces")
```

Use the full mesh for visual rendering and the decimated mesh for collision detection (standard SOFA practice: separate visual/collision topologies).
