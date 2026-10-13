#!/bin/bash
# T8: the worker image on the CPU. One loc shard of 007 ([0,N), replayed
# poses, --no-stereo) through `mvcloud agent --once` inside the container,
# with no GPU (CUDA_VISIBLE_DEVICES= and use_slic_cuda_if_available false).
#
# Usage: tools/cloud/tests/t8_container.sh [--image mvcloud:dev] [--frames N]
#                                          [--out <dir>] [--cache <dir>]
#
# Asserts: the container exits 0; the shard is done in one attempt; the
# agent logged that it runs without a per-process memory cap; the envelope
# has N frames and equals the first N frames of ref-120.data byte for byte.
# The inputs are copied under <out> (default under ${TMPDIR:-/tmp}), which is
# bind-mounted at the same path: some docker daemons cannot see /home. A
# --cache directory keeps the undistortion maps (regenerated on first use,
# ~2.5 min for 007's 12 sensors on 6 CPUs) and the models between runs.

set -euo pipefail

THIS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$THIS_DIR/../../.." && pwd)"
DATA_D="${MULTIVIEW_TESTCASE_DATA:-$HOME/multiview-data}"
REPLAY_D="${MULTIVIEW_POSE_REPLAY:-$HOME/multiview-data/pose-replay}"
TC=007_site03_sparse_tracks
CLI="$ROOT/multiview/multiview_cpp/build/cmake-release/multiview-cli"
IMAGE=mvcloud:dev
FRAMES=20
OUT=""
CACHE=""

while [ "$#" -gt 0 ] ; do
    case "$1" in
        --image)  shift ; IMAGE="$1" ;;
        --frames) shift ; FRAMES="$1" ;;
        --out)    shift ; OUT="$1" ;;
        --cache)  shift ; CACHE="$1" ;;
        -h|--help) sed -n '2,/^$/p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//' ; exit 0 ;;
        *) echo "t8: unknown argument '$1'" ; exit 1 ;;
    esac
    shift
done

GOLD="$DATA_D/cloud-golden/007"
for f in "$CLI" "$DATA_D/testcases/$TC/manifest.json" "$GOLD/ref-120.data" "$GOLD/params-replay.json" ; do
    [ -e "$f" ] || { echo "t8: missing $f" ; exit 1 ; }
done
[ "$FRAMES" -ge 2 ] && [ "$FRAMES" -le 120 ] || { echo "t8: --frames must be in [2, 120]" ; exit 1 ; }
docker image inspect "$IMAGE" > /dev/null || { echo "t8: no image $IMAGE (tools/cloud/docker/build-image.sh)" ; exit 1 ; }
[ -n "$OUT" ] || OUT="$(mktemp -d "${TMPDIR:-/tmp}/t8-container.XXXXXX")"
OUT="$(mkdir -p "$OUT" && cd "$OUT" && pwd)"
CACHE="${CACHE:-$OUT/cache}"
mkdir -p "$OUT/in/scene" "$OUT/in/poses" "$OUT/work" "$CACHE/home"
CACHE="$(cd "$CACHE" && pwd)"
FAILED=0

check() # check <name> <command...>: records a failure, does not stop
{
    local name="$1" ; shift
    if "$@" ; then echo "t8: PASS $name" ; else echo "t8: FAIL $name" ; FAILED=1 ; fi
}

# ---- Inputs: the scene, the poses of [0,N), params without CUDA SLIC
cp "$DATA_D/testcases/$TC"/{manifest.json,ground-truth.json} "$DATA_D/testcases/$TC"/*.mp4 "$OUT/in/scene/"
for ((t = 0; t < FRAMES; ++t)) ; do
    cp "$REPLAY_D/$TC"/pose-2d_"$(printf %06d "$t")"_*.json "$OUT/in/poses/"
done
sed 's/"use_slic_cuda_if_available"\( *\): true/"use_slic_cuda_if_available"\1: false/' \
    "$GOLD/params-replay.json" > "$OUT/in/params.json"
python3 - "$OUT" "$FRAMES" <<'EOF'
import json, sys
out, n = sys.argv[1], int(sys.argv[2])
json.dump({"scene_id": "007_site03_sparse_tracks", "manifest": f"{out}/in/scene/manifest.json",
           "params": f"{out}/in/params.json", "poses": f"{out}/in/poses",
           "gt": f"{out}/in/scene/ground-truth.json", "start_frame": 0, "n_frames": n,
           "shard_frames": n, "no_stereo": True, "pose_mode": "replay", "max_attempts": 1,
           "retry_delay_s": 0}, open(f"{out}/job.json", "w"), indent=1)
EOF
job="$(PYTHONPATH="$ROOT/tools/cloud/agent" python3 -m mvcloud submit --control "fs://$OUT/root" \
       --cli "$CLI" "$OUT/job.json")"
echo "t8: job $job"

# ---- The container: one shard, no GPU
t0="$(date +%s)"
ret=0
docker run --rm --name "t8-$$" --memory 24g --cpus 6 --user "$(id -u):$(id -g)" \
    -e HOME=/cache/home -e CUDA_VISIBLE_DEVICES= \
    -v "$OUT:$OUT" -v "$CACHE:/cache" -v "$OUT/work:/work" \
    "$IMAGE" --control "fs://$OUT/root" --once > "$OUT/agent.log" 2>&1 || ret=$?
echo "t8: container exit $ret after $(( $(date +%s) - t0 )) s"
check "container exit 0" test "$ret" = 0
check "no memory cap inside the container (logged)" grep -q "no per-process memory cap" "$OUT/agent.log"
check "loc-0000 done in one attempt" \
    python3 "$THIS_DIR/e2e_check.py" shard "$OUT/root" "$job" loc-0000 done done
env="$(find "$OUT/root/blobs" -path '*loc-0000/a1/localization.data' | head -n 1)"
if [ -n "$env" ] ; then
    "$CLI" ldata cat -y --keep "0:$FRAMES" -o "$OUT/ref.data" "$GOLD/ref-120.data" > /dev/null
    check "envelope has $FRAMES frames" \
        python3 -c "import json,sys; sys.exit(json.load(open(sys.argv[1]))['n_frames'] != $FRAMES)" \
        <("$CLI" ldata info "$env")
    check "envelope == ref-120.data [0,$FRAMES) (bytes)" \
        "$CLI" ldata diff --json "$OUT/diff.json" "$env" "$OUT/ref.data"
else
    echo "t8: FAIL no envelope" ; FAILED=1
fi
grep -ao "session [0-9]* ended: .*" "$OUT/agent.log" || true

if [ "$FAILED" = "0" ] ; then echo "t8: PASS (outputs in $OUT)" ; else echo "t8: FAIL (outputs in $OUT)" ; fi
exit "$FAILED"
