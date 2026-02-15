#!/bin/bash
# unity-test.sh — Run Unity tests from WSL2 in headless batch mode
set -euo pipefail

UNITY="/mnt/c/Program Files/Unity/Hub/Editor/6000.3.8f1/Editor/Unity.exe"
PROJECT_WSL="/mnt/c/projects/couch/unity-project"
PROJECT_WIN=$(wslpath -w "$PROJECT_WSL")
RESULTS_DIR="$PROJECT_WSL/TestResults"

mkdir -p "$RESULTS_DIR"

PLATFORM="${1:-EditMode}"  # EditMode or PlayMode
FILTER="${2:-}"            # optional test filter

NOGRAPHICS=""
if [ "$PLATFORM" = "EditMode" ]; then
  NOGRAPHICS="-nographics"
fi

FILTER_ARG=""
if [ -n "$FILTER" ]; then
  FILTER_ARG="-testFilter $FILTER"
fi

RESULTS_FILE="$(wslpath -w "$RESULTS_DIR")\\${PLATFORM}-results.xml"

echo "=== Unity Test Runner ==="
echo "Platform:  $PLATFORM"
echo "Project:   $PROJECT_WIN"
echo "Results:   $RESULTS_FILE"
echo ""

"$UNITY" \
  -batchmode \
  $NOGRAPHICS \
  -projectPath "$PROJECT_WIN" \
  -runTests \
  -testPlatform "$PLATFORM" \
  -testResults "$RESULTS_FILE" \
  $FILTER_ARG \
  -logFile -

rc=$?
echo ""
echo "---"
if [ $rc -eq 0 ]; then
  echo "All $PLATFORM tests passed."
elif [ $rc -eq 2 ]; then
  echo "Some $PLATFORM tests failed. Check: $RESULTS_DIR/${PLATFORM}-results.xml"
  exit 1
else
  echo "Unity error (exit code $rc). Check log output above."
  exit $rc
fi
