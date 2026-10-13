#!/bin/bash
# Entry point of the mvcloud worker image: prepares the cache volume, reports
# the binary and the GPU, then runs the agent with the given arguments
# (`--control ...` and the rest of `python3 -m mvcloud agent`). Any other
# command (`bash`, `multiview-cli ...`, `python3 ...`) is run as is.

set -euo pipefail

mkdir -p "$MULTIVIEW_CACHE_DIR" "$MULTIVIEW_LAZY_S3_DIR" "$MULTIVIEW_HYPERPOSE_DIR/models" \
         /cache/mvcloud /work

case "${1:-}" in
    bash|sh|multiview-cli|python3|download-hyperpose-models.sh) exec "$@" ;;
esac

if ! ls "$MULTIVIEW_HYPERPOSE_DIR"/models/*.onnx > /dev/null 2>&1 ; then
    download-hyperpose-models.sh "$MULTIVIEW_HYPERPOSE_DIR" \
        || echo "entrypoint: no HyperPose models; replay-mode jobs only"
fi
multiview-cli -h 2>&1 | head -n 3 || true
if command -v nvidia-smi > /dev/null ; then
    nvidia-smi --query-gpu=name,compute_cap,driver_version --format=csv,noheader || true
else
    echo "entrypoint: no nvidia-smi (run with --gpus all for HyperPose jobs)"
fi

exec python3 -m mvcloud agent --cli /opt/multiview/bin/multiview-cli \
     --cache /cache/mvcloud --work /work "$@"
