#!/usr/bin/env bash
# Collect SOFA transitive dependencies for standalone deployment.
# Usage: ./collect_sofa_deps.sh <libSofaAnkleBridge.so> <output_dir> [test_bridge]
set -euo pipefail

if [[ $# -lt 2 ]]; then
    echo "Usage: $0 <bridge_so> <output_dir> [test_binary]"
    exit 1
fi

BRIDGE_SO="$1"
OUTPUT_DIR="$2"
TEST_BIN="${3:-}"

if [[ ! -f "$BRIDGE_SO" ]]; then
    echo "ERROR: $BRIDGE_SO not found"
    exit 1
fi

mkdir -p "$OUTPUT_DIR"

echo "=== Collecting SOFA dependencies ==="

# Collect all SOFA and tinyxml2 shared libraries from ldd output
collect_deps() {
    local binary="$1"
    ldd "$binary" 2>/dev/null | \
        grep -oP '/\S+' | \
        grep -iE '(libSofa|libtinyxml2|libgtest)' || true
}

# Gather deps from bridge .so
DEPS=$(collect_deps "$BRIDGE_SO")

# Also gather from test binary if provided
if [[ -n "$TEST_BIN" && -f "$TEST_BIN" ]]; then
    DEPS=$(echo "$DEPS"; collect_deps "$TEST_BIN")
fi

# Deduplicate
DEPS=$(echo "$DEPS" | sort -u)

COUNT=0
for dep in $DEPS; do
    if [[ -f "$dep" ]]; then
        # Dereference symlinks
        cp -L "$dep" "$OUTPUT_DIR/"
        basename_dep=$(basename "$dep")
        # Create unversioned symlink if needed (e.g., libSofa.Core.so -> libSofa.Core.so.24.06.00)
        unversioned=$(echo "$basename_dep" | sed -E 's/\.so\.[0-9.]+$/.so/')
        if [[ "$unversioned" != "$basename_dep" && ! -e "$OUTPUT_DIR/$unversioned" ]]; then
            ln -sf "$basename_dep" "$OUTPUT_DIR/$unversioned"
        fi
        COUNT=$((COUNT + 1))
    fi
done

# Copy the bridge .so itself
cp -L "$BRIDGE_SO" "$OUTPUT_DIR/"
COUNT=$((COUNT + 1))

# Copy test binary if provided
if [[ -n "$TEST_BIN" && -f "$TEST_BIN" ]]; then
    cp "$TEST_BIN" "$OUTPUT_DIR/"
    chmod +x "$OUTPUT_DIR/$(basename "$TEST_BIN")"
    COUNT=$((COUNT + 1))
fi

echo ""
echo "=== Verification ==="
WARNINGS=0
for f in "$OUTPUT_DIR"/*.so "$OUTPUT_DIR"/*.so.*; do
    [[ -f "$f" ]] || continue
    NOT_FOUND=$(LD_LIBRARY_PATH="$OUTPUT_DIR" ldd "$f" 2>/dev/null | grep "not found" || true)
    if [[ -n "$NOT_FOUND" ]]; then
        echo "WARNING: $(basename "$f") has missing deps:"
        echo "$NOT_FOUND"
        WARNINGS=$((WARNINGS + 1))
    fi
done

echo ""
echo "=== Report ==="
echo "Files collected: $COUNT"
TOTAL_SIZE=$(du -sh "$OUTPUT_DIR" | cut -f1)
echo "Total size: $TOTAL_SIZE"
if [[ $WARNINGS -gt 0 ]]; then
    echo "WARNINGS: $WARNINGS files have missing dependencies"
    exit 1
else
    echo "All dependencies resolved."
fi
