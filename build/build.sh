#!/bin/bash
# Configure and build multiview with CMake inside the micromamba env created
# by build/setup-env.sh. No sudo, /opt/multiview or /usr/local/cuda needed.
#
# Usage: build/build.sh [Release|Debug|RelWithDebInfo] [targets...]
#   default: Release multiview-cli multiview-testcases
#
# Extra CMake options can be passed through MULTIVIEW_CMAKE_ARGS (not
# CMAKE_ARGS, which conda compiler activation overwrites), e.g.
#   MULTIVIEW_CMAKE_ARGS="-DMULTIVIEW_WITH_AWS=OFF -DCMAKE_CUDA_ARCHITECTURES=75;86" build/build.sh
#
# Output: multiview/multiview_cpp/build/cmake-<config>/<target>, or set
# MULTIVIEW_BUILD_DIR to keep builds with different options apart.
# Builds with nproc-2 jobs; override with CMAKE_BUILD_PARALLEL_LEVEL.

set -euo pipefail

THIS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SRC_DIR="$(cd "$THIS_DIR/../multiview/multiview_cpp" && pwd)"
ENV_NAME="${MULTIVIEW_ENV:-multiview}"
export MAMBA_ROOT_PREFIX="${MAMBA_ROOT_PREFIX:-$HOME/micromamba}"

CONFIG="Release"
case "${1:-}" in
    Release|Debug|RelWithDebInfo) CONFIG="$1" ; shift ;;
esac
TARGETS=("$@")
[ "${#TARGETS[@]}" = "0" ] && TARGETS=(multiview-cli multiview-testcases)

if [ -z "${CONDA_PREFIX:-}" ] || [ "$(basename "$CONDA_PREFIX")" != "$ENV_NAME" ] ; then
    MAMBA="$(command -v micromamba || echo "$HOME/.local/bin/micromamba")"
    [ -x "$MAMBA" ] || { echo "micromamba not found; run $THIS_DIR/setup-env.sh" ; exit 1 ; }
    set +u # conda activation scripts reference unset variables
    eval "$("$MAMBA" shell hook -s bash)"
    micromamba activate "$ENV_NAME"
    set -u
fi

BUILD_DIR="${MULTIVIEW_BUILD_DIR:-$SRC_DIR/build/cmake-$(echo "$CONFIG" | tr '[:upper:]' '[:lower:]')}"
NPROC="$(nproc)"
export CMAKE_BUILD_PARALLEL_LEVEL="${CMAKE_BUILD_PARALLEL_LEVEL:-$(( NPROC > 3 ? NPROC - 2 : 1 ))}"

# Default to the GPU(s) in this machine; override with MULTIVIEW_CMAKE_ARGS
CUDA_ARCH="native"
if command -v nvidia-smi > /dev/null ; then
    CUDA_ARCH="$(nvidia-smi --query-gpu=compute_cap --format=csv,noheader \
        | tr -d '.' | sort -u | paste -sd ';')"
fi

cmake -S "$SRC_DIR" -B "$BUILD_DIR" -G Ninja \
      -DCMAKE_BUILD_TYPE="$CONFIG" \
      -DCMAKE_PREFIX_PATH="$CONDA_PREFIX" \
      -DCMAKE_CUDA_ARCHITECTURES="${CUDA_ARCH:-native}" \
      ${MULTIVIEW_CMAKE_ARGS:-}
cmake --build "$BUILD_DIR" --target "${TARGETS[@]}"

echo
for T in "${TARGETS[@]}" ; do echo "Built: $BUILD_DIR/$T" ; done
