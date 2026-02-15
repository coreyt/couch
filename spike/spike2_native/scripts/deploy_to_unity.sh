#!/bin/bash
# deploy_to_unity.sh
# Deploys SofaAnkleBridge DLL and SOFA dependencies to Unity Plugins folder.
#
# Usage:
#   ./deploy_to_unity.sh [--build-dir <path>] [--deps-dir <path>]
#
# Defaults:
#   --build-dir  ./build-win/Release  (Windows DLL build output)
#   --deps-dir   ./sofa_deps          (collected SOFA DLLs)

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
NATIVE_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
REPO_ROOT="$(cd "$NATIVE_DIR/../.." && pwd)"
UNITY_PLUGINS="$REPO_ROOT/unity-project/Assets/Plugins/x86_64"

BUILD_DIR="$NATIVE_DIR/build-win/Release"
DEPS_DIR="$NATIVE_DIR/sofa_deps"

# Parse arguments
while [[ $# -gt 0 ]]; do
    case "$1" in
        --build-dir) BUILD_DIR="$2"; shift 2 ;;
        --deps-dir)  DEPS_DIR="$2"; shift 2 ;;
        *) echo "Unknown option: $1"; exit 1 ;;
    esac
done

echo "=== Deploy to Unity Plugins ==="
echo "Build dir:    $BUILD_DIR"
echo "Deps dir:     $DEPS_DIR"
echo "Unity target: $UNITY_PLUGINS"
echo ""

# Create plugins directory
mkdir -p "$UNITY_PLUGINS"

# Copy main DLL
BRIDGE_DLL="$BUILD_DIR/SofaAnkleBridge.dll"
if [[ -f "$BRIDGE_DLL" ]]; then
    cp "$BRIDGE_DLL" "$UNITY_PLUGINS/"
    echo "Copied: SofaAnkleBridge.dll"
else
    echo "ERROR: SofaAnkleBridge.dll not found at $BRIDGE_DLL"
    exit 1
fi

# Copy SOFA dependency DLLs
if [[ -d "$DEPS_DIR" ]]; then
    count=0
    for dll in "$DEPS_DIR"/*.dll; do
        [[ -f "$dll" ]] || continue
        cp "$dll" "$UNITY_PLUGINS/"
        count=$((count + 1))
    done
    echo "Copied: $count SOFA dependency DLLs"
else
    echo "WARNING: SOFA deps directory not found at $DEPS_DIR"
    echo "         Run collect_sofa_deps_win.ps1 first"
fi

echo ""
echo "Deploy complete. Total files in $UNITY_PLUGINS:"
ls -1 "$UNITY_PLUGINS"/*.dll 2>/dev/null | wc -l
