#!/usr/bin/env bash
# Download right-foot bone STL files from BodyParts3D (Kevin-Mattheus-Moerman/BodyParts3D)
# All use the same coordinate system as existing tibia/talus/fibula meshes.
#
# Usage: ./scripts/download_foot_meshes.sh
#
# Destination: spike/spike1_python/meshes/
# Existing meshes (not re-downloaded): tibia_right.stl, fibula_right.stl, talus_right.stl

set -euo pipefail

DEST_DIR="$(cd "$(dirname "$0")/.." && pwd)/spike/spike1_python/meshes"
BASE_URL="https://raw.githubusercontent.com/Kevin-Mattheus-Moerman/BodyParts3D/main/assets/BodyParts3D_data/stl"

mkdir -p "$DEST_DIR"

# Map: FMA_ID -> output filename
declare -A BONES=(
    # Hindfoot
    ["FMA24497"]="calcaneus_right.stl"
    # Midfoot
    ["FMA24500"]="navicular_right.stl"
    ["FMA24528"]="cuboid_right.stl"
    ["FMA24521"]="medial_cuneiform_right.stl"
    ["FMA24523"]="intermediate_cuneiform_right.stl"
    ["FMA24525"]="lateral_cuneiform_right.stl"
    # Metatarsals
    ["FMA24507"]="metatarsal_1_right.stl"
    ["FMA24509"]="metatarsal_2_right.stl"
    ["FMA24511"]="metatarsal_3_right.stl"
    ["FMA24513"]="metatarsal_4_right.stl"
    ["FMA24515"]="metatarsal_5_right.stl"
    # Proximal phalanges
    ["FMA43253"]="proximal_phalanx_1_right.stl"
    ["FMA32634"]="proximal_phalanx_2_right.stl"
    ["FMA32636"]="proximal_phalanx_3_right.stl"
    ["FMA32638"]="proximal_phalanx_4_right.stl"
    ["FMA32640"]="proximal_phalanx_5_right.stl"
    # Middle phalanges (no middle phalanx for big toe)
    ["FMA32642"]="middle_phalanx_2_right.stl"
    ["FMA32644"]="middle_phalanx_3_right.stl"
    ["FMA32646"]="middle_phalanx_4_right.stl"
    ["FMA230986"]="middle_phalanx_5_right.stl"
    # Distal phalanges
    ["FMA32650"]="distal_phalanx_1_right.stl"
    ["FMA32652"]="distal_phalanx_2_right.stl"
    ["FMA32654"]="distal_phalanx_3_right.stl"
    ["FMA32656"]="distal_phalanx_4_right.stl"
    ["FMA32658"]="distal_phalanx_5_right.stl"
)

echo "Downloading 25 right-foot bone STLs from BodyParts3D..."
echo "Destination: $DEST_DIR"
echo ""

downloaded=0
skipped=0
failed=0

for fma_id in "${!BONES[@]}"; do
    filename="${BONES[$fma_id]}"
    dest_path="$DEST_DIR/$filename"

    if [ -f "$dest_path" ]; then
        echo "  SKIP  $filename (already exists)"
        skipped=$((skipped + 1))
        continue
    fi

    url="$BASE_URL/$fma_id.stl"
    echo -n "  GET   $filename ($fma_id)... "

    if curl -sfL "$url" -o "$dest_path"; then
        if [ -s "$dest_path" ]; then
            echo "OK ($(wc -c < "$dest_path") bytes)"
            downloaded=$((downloaded + 1))
        else
            echo "FAIL (empty file)"
            rm -f "$dest_path"
            failed=$((failed + 1))
        fi
    else
        echo "FAIL (HTTP error)"
        rm -f "$dest_path"
        failed=$((failed + 1))
    fi
done

echo ""
echo "Done: $downloaded downloaded, $skipped skipped, $failed failed"
echo "Total STL files in $DEST_DIR: $(ls "$DEST_DIR"/*.stl 2>/dev/null | wc -l)"
