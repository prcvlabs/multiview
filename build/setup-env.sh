#!/bin/bash
# Create the multiview build environment in user space (no sudo needed).
#
# Installs a static micromamba binary to ~/.local/bin (if not already on
# PATH) and creates the 'multiview' env from build/environment.yml.
#
# Usage: build/setup-env.sh [--update] [--with-tensorrt]
#
#   --update          update an existing env from environment.yml
#   --with-tensorrt   also install TensorRT (for -DMULTIVIEW_WITH_HYPERPOSE=ON)
#                     into <env>/opt/tensorrt: the libraries come from NVIDIA's
#                     tensorrt-cu12-libs wheel (~4.3 GB download, ~1.5 GB kept)
#                     and the headers from github.com/NVIDIA/TensorRT. conda-forge
#                     only packages TensorRT for CUDA 13. Set TRT_WHEEL_FILE to
#                     reuse an already downloaded wheel.
#
# Then build with: build/build.sh

set -euo pipefail

THIS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
UPDATE=0
WITH_TENSORRT=0
for ARG in "$@" ; do
    case "$ARG" in
        --update)        UPDATE=1 ;;
        --with-tensorrt) WITH_TENSORRT=1 ;;
        *) echo "unknown argument: $ARG" ; exit 1 ;;
    esac
done

ENV_NAME="${MULTIVIEW_ENV:-multiview}"
export MAMBA_ROOT_PREFIX="${MAMBA_ROOT_PREFIX:-$HOME/micromamba}"

MAMBA="$(command -v micromamba || true)"
if [ -z "$MAMBA" ] ; then
    MAMBA="$HOME/.local/bin/micromamba"
    if [ ! -x "$MAMBA" ] ; then
        echo "Installing micromamba to $MAMBA"
        mkdir -p "$HOME/.local/bin"
        TMPD="$(mktemp -d)"
        trap 'rm -rf "$TMPD"' EXIT
        curl -fsSL https://micro.mamba.pm/api/micromamba/linux-64/latest \
            | tar -xj -C "$TMPD" bin/micromamba
        mv "$TMPD/bin/micromamba" "$MAMBA"
    fi
fi

if "$MAMBA" env list | awk '{print $1}' | grep -qx "$ENV_NAME" ; then
    if [ "$UPDATE" = "1" ] ; then
        "$MAMBA" install -y -n "$ENV_NAME" -f "$THIS_DIR/environment.yml"
    else
        echo "Environment '$ENV_NAME' already exists (pass --update to update)"
    fi
else
    "$MAMBA" create -y -n "$ENV_NAME" -f "$THIS_DIR/environment.yml"
fi

# TensorRT 10.16 for CUDA 12.x. Patch releases share the API, so the 10.16.0
# headers match the 10.16.1 libraries.
TRT_VERSION=10.16.1.11
TRT_WHEEL="tensorrt_cu12_libs-$TRT_VERSION-py3-none-manylinux_2_28_x86_64.whl"
TRT_WHEEL_SHA256=8e45036efeb964d323231544442a73619201136ccc84392560254cc8f0d516e4
TRT_HEADERS_COMMIT=52399f555c2f80cb690a4a558b604e1a5f227e7c # tag v10.16
TRT_HEADERS="NvInfer.h NvInferImpl.h NvInferLegacyDims.h NvInferPlugin.h
    NvInferPluginBase.h NvInferPluginUtils.h NvInferRuntime.h
    NvInferRuntimeBase.h NvInferRuntimeCommon.h NvInferRuntimePlugin.h
    NvInferVersion.h NvOnnxConfig.h NvOnnxParser.h"

install_tensorrt()
{
    local DEST="$MAMBA_ROOT_PREFIX/envs/$ENV_NAME/opt/tensorrt"
    if [ -f "$DEST/lib/libnvinfer.so.10" ] && [ -f "$DEST/include/NvInfer.h" ] ; then
        echo "TensorRT already installed in $DEST"
        return 0
    fi

    local TMPD
    TMPD="$(mktemp -d)"
    trap 'rm -rf "$TMPD"' RETURN
    local WHEEL="${TRT_WHEEL_FILE:-$TMPD/trt.whl}" # reuse a downloaded wheel
    if [ ! -f "$WHEEL" ] ; then
        echo "Downloading $TRT_WHEEL"
        curl -fL -o "$WHEEL" "https://pypi.nvidia.com/tensorrt-cu12-libs/$TRT_WHEEL"
    fi
    echo "$TRT_WHEEL_SHA256  $WHEEL" | sha256sum -c -

    # Keep the runtime, ONNX parser, plugins and the builder resources for
    # this machine's GPU(s) (plus PTX for anything else)
    local SMS="$(nvidia-smi --query-gpu=compute_cap --format=csv,noheader 2>/dev/null \
        | tr -d '.' | sort -u)"
    local FILES="libnvinfer.so.10 libnvonnxparser.so.10 libnvinfer_plugin.so.10
        libnvinfer_builder_resource_ptx.so.*"
    for SM in $SMS ; do FILES="$FILES libnvinfer_builder_resource_sm$SM.so.*" ; done

    mkdir -p "$DEST/lib" "$DEST/include"
    for F in $FILES ; do
        unzip -q -o -j "$WHEEL" "tensorrt_libs/$F" -d "$DEST/lib" \
            || echo "warning: $F not found in wheel"
    done
    for L in nvinfer nvonnxparser nvinfer_plugin ; do
        ln -sf "lib$L.so.10" "$DEST/lib/lib$L.so"
    done

    local URL="https://raw.githubusercontent.com/NVIDIA/TensorRT/$TRT_HEADERS_COMMIT/include"
    for H in $TRT_HEADERS ; do
        curl -fsSL -o "$DEST/include/$H" "$URL/$H"
    done
    echo "Installed TensorRT $TRT_VERSION to $DEST"
}

[ "$WITH_TENSORRT" = "1" ] && install_tensorrt

if command -v nvidia-smi > /dev/null ; then
    nvidia-smi --query-gpu=name,compute_cap,driver_version --format=csv,noheader
else
    echo "warning: nvidia-smi not found; a CUDA-capable driver is needed to run GPU code"
fi
echo "Done. Next: $THIS_DIR/build.sh"
