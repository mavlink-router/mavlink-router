#!/usr/bin/env bash
#
# Build and run mavlink-router unit tests.
#
# Usage:
#   ./run_tests.sh              # full clean build + test
#   ./run_tests.sh --no-clean   # incremental build + test
#
set -euo pipefail

PROJECT_ROOT="$(cd "$(dirname "$0")" && pwd)"
BUILD_DIR="${PROJECT_ROOT}/builddir-test"
CLEAN=true

for arg in "$@"; do
    case "$arg" in
        --no-clean) CLEAN=false ;;
        *) echo "Unknown option: $arg"; exit 1 ;;
    esac
done

# ── Dependencies check ───────────────────────────────────────────────
for cmd in meson ninja pkg-config; do
    if ! command -v "$cmd" &>/dev/null; then
        echo "Error: '$cmd' is required but not found in PATH." >&2
        exit 1
    fi
done

# ── Configure ────────────────────────────────────────────────────────
if [ "$CLEAN" = true ] && [ -d "$BUILD_DIR" ]; then
    echo "==> Removing previous build directory..."
    rm -rf "$BUILD_DIR"
fi

if [ ! -d "$BUILD_DIR" ]; then
    echo "==> Configuring project (build type: debug)..."
    meson setup "$BUILD_DIR" "$PROJECT_ROOT" \
        --buildtype=debug \
        -Dsystemdsystemunitdir=/tmp/mavlink-router-test-units
else
    echo "==> Reconfiguring existing build directory..."
    meson setup --reconfigure "$BUILD_DIR" "$PROJECT_ROOT"
fi

# ── Build ────────────────────────────────────────────────────────────
echo "==> Building..."
meson compile -C "$BUILD_DIR"

# ── Run tests ────────────────────────────────────────────────────────
echo "==> Running unit tests..."
meson test -C "$BUILD_DIR" --print-errorlogs --verbose

echo ""
echo "==> All tests passed."