#!/bin/bash
# Create the multiview build environment in user space (no sudo needed).
#
# Installs a static micromamba binary to ~/.local/bin (if not already on
# PATH) and creates the 'multiview' env from build/environment.yml.
#
# Usage: build/setup-env.sh [--update]
#
# Then build with: build/build.sh

set -euo pipefail

THIS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
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
    if [ "${1:-}" = "--update" ] ; then
        "$MAMBA" install -y -n "$ENV_NAME" -f "$THIS_DIR/environment.yml"
    else
        echo "Environment '$ENV_NAME' already exists (pass --update to update)"
    fi
else
    "$MAMBA" create -y -n "$ENV_NAME" -f "$THIS_DIR/environment.yml"
fi

if command -v nvidia-smi > /dev/null ; then
    nvidia-smi --query-gpu=name,compute_cap,driver_version --format=csv,noheader
else
    echo "warning: nvidia-smi not found; a CUDA-capable driver is needed to run GPU code"
fi
echo "Done. Next: $THIS_DIR/build.sh"
