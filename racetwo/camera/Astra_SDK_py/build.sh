#!/usr/bin/env bash
set -euo pipefail

# Usage:
#   ./build.sh
# 可选：ASTRA_ROOT=/path/to/sdk ./build.sh 覆盖自动探测

if [[ -z "${ASTRA_ROOT:-}" ]]; then
  # 尝试在当前目录下自动探测
  for d in ./*; do
    if [[ -d "$d/include/astra" && ( -f "$d/lib/libastra.so" || -f "$d/lib/astra.lib" ) ]]; then
      export ASTRA_ROOT="$(cd "$d" && pwd)"
      echo "[INFO] Auto-detected ASTRA_ROOT=$ASTRA_ROOT"
      break
    fi
  done
fi

if [[ -z "${ASTRA_ROOT:-}" ]]; then
  echo "[ERROR] ASTRA_ROOT is not set and auto-detection failed. Put the SDK folder next to CMakeLists.txt or set ASTRA_ROOT." >&2
  exit 1
fi

BUILD_DIR="build"
mkdir -p "$BUILD_DIR"

cmake -S . -B "$BUILD_DIR" -G "Unix Makefiles" -DASTRA_ROOT="$ASTRA_ROOT"
cmake --build "$BUILD_DIR" --config Release -j

echo "[OK] Built Python module at $BUILD_DIR/astra_py.so"


