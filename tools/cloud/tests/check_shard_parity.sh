#!/bin/bash
# Shard parity on testcase 007: loc shards concatenated must equal one
# --save-loc-data run, and a tracks job over the shards must equal the
# tracks job over that one envelope.
#
# Usage: tools/cloud/tests/check_shard_parity.sh [--frames N] [--shard-frames F]
#                                                [--no-still]
#                                                [--cli <multiview-cli>] [--out <dir>]
#
# Needs the references of make_reference.sh for N frames (default 120) in
# $MULTIVIEW_TESTCASE_DATA/cloud-golden/007. Runs the loc shards
# [0,F), [F,2F), ... (default F = 60) with --save-loc-data --no-tracks, then
#
#   (a) ldata cat of the shards, then ldata diff against ref-N.data: exit 0
#       (every frame byte-equal)
#   (b) a tracks job with one --load-loc-data per shard and --require-aligned:
#       track_results equal to R3's (lab_p included) and to R1's (lab_p nulled)
#   (c) tracks jobs with an unaligned end, or starting before the envelope:
#       refused by --require-aligned
#
# --no-still passes --no-still to the loc shards only: (a) then shows that
# the envelope does not depend on the still (ref-N.data was made with it).
# The tracks job keeps the still, which the pose classifier reads.
#
# Exit status 0 if all hold. Outputs go to <out> (default a new directory
# under ${TMPDIR:-/tmp}), which is kept.

set -euo pipefail

THIS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$THIS_DIR/../../.." && pwd)"
DATA_D="${MULTIVIEW_TESTCASE_DATA:-$HOME/multiview-data}"
TC=007_site03_sparse_tracks
GPU_LOCK=/tmp/multiview-dev-gpu.lock
NICE="${MULTIVIEW_DEV_NICE:-15}"
MEM="${MULTIVIEW_CLOUD_MEM:-24G}"

CLI="$ROOT/multiview/multiview_cpp/build/cmake-release/multiview-cli"
N=120
F=60
OUT=""
NO_STILL=()

while [ "$#" -gt 0 ] ; do
    case "$1" in
        --frames)       shift ; N="$1" ;;
        --shard-frames) shift ; F="$1" ;;
        --no-still)     NO_STILL=(--no-still) ;;
        --cli)          shift ; CLI="$1" ;;
        --out)          shift ; OUT="$1" ;;
        -h|--help) sed -n '2,/^$/p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//' ; exit 0 ;;
        *) echo "check_shard_parity: unknown argument '$1'" ; exit 1 ;;
    esac
    shift
done

GOLD="$DATA_D/cloud-golden/007"
MANIFEST="$DATA_D/testcases/$TC/manifest.json"
GT="$DATA_D/testcases/$TC/ground-truth.json"
REF="$GOLD/ref-$N.data"
for f in "$CLI" "$REF" "$GOLD/params-replay.json" "$GOLD/n$N/R1/pipeline-output.json" \
         "$GOLD/n$N/R3/pipeline-output.json" ; do
    [ -e "$f" ] || { echo "check_shard_parity: missing $f (make_reference.sh --frames $N)" ; exit 1 ; }
done
[ -n "$OUT" ] || OUT="$(mktemp -d "${TMPDIR:-/tmp}/check-shard-parity.XXXXXX")"
mkdir -p "$OUT"

capped_cmd() # capped (systemd scope with a memory cap, nice 15) if present
{
    local capped ; capped="$(command -v capped || echo "$HOME/.local/bin/capped")"
    if [ -x "$capped" ] ; then echo "$capped -m $MEM" ; else echo "nice -n $NICE" ; fi
}

run_pipeline() # run_pipeline <dir> pipeline-args...
{
    local dir="$1" ; shift
    rm -rf "$dir" ; mkdir -p "$dir"
    local ret=0
    # shellcheck disable=SC2046
    flock "$GPU_LOCK" $(capped_cmd) /usr/bin/time -f '%e' -o "$dir/wall-seconds" \
        "$CLI" pipeline -m "$MANIFEST" -p "$GOLD/params-replay.json" -d "$dir" -y \
        --no-stereo --verbosity 1 "$@" \
        > "$dir/stdout" 2> "$dir/stderr" || ret=$?
    echo "check_shard_parity: $(basename "$dir"): exit $ret, $(tail -1 "$dir/wall-seconds" 2>/dev/null || echo "?")s"
    return "$ret"
}

FAILED=0
check() # check <name> <command...>: records a failure, does not stop
{
    local name="$1" ; shift
    if "$@" ; then echo "check_shard_parity: PASS $name" ; else echo "check_shard_parity: FAIL $name" ; FAILED=1 ; fi
}

# ---- Loc shards
shards=()
loads=()
for (( s = 0, i = 0 ; s < N ; s += F, i++ )) ; do
    n=$(( s + F <= N ? F : N - s ))
    d="$OUT/loc-$(printf '%04d' "$i")"
    run_pipeline "$d" --start "$s" --n-frames "$n" --save-loc-data --no-tracks "${NO_STILL[@]}"
    shards+=("$d/localization.data")
    loads+=(--load-loc-data "$d/localization.data")
done

# ---- (a) envelopes
"$CLI" ldata cat -y -o "$OUT/merged.data" --report "$OUT/cat-report.json" "${shards[@]}" > "$OUT/cat.stdout"
check "(a) ldata diff merged ref-$N.data" \
    "$CLI" ldata diff --json "$OUT/diff.json" "$OUT/merged.data" "$REF"

# ---- (b) tracks job over the shards
run_pipeline "$OUT/tracks" "${loads[@]}" --start 0 --n-frames "$N" --require-aligned \
    --pose-classifier pose-svm-lite_v0 --tracks-file "$GT" --pipeline-testcase || FAILED=1
check "(b) tracks job == R3 (lab_p included)" \
    python3 "$THIS_DIR/compare_tracks.py" --keep-lab-p --json "$OUT/tracks-vs-R3.json" \
    "$GOLD/n$N/R3/pipeline-output.json" "$OUT/tracks/pipeline-output.json"
check "(b) tracks job == R1 (lab_p nulled)" \
    python3 "$THIS_DIR/compare_tracks.py" --json "$OUT/tracks-vs-R1.json" \
    "$GOLD/n$N/R1/pipeline-output.json" "$OUT/tracks/pipeline-output.json"

# ---- (c) unaligned tracks jobs are refused before any frame is processed
# (--load-loc-data moves a --start after the envelope's start back to it, so
# the unaligned start is tested on the frames before the second shard)
refused() # refused <name> pipeline-args...
{
    local name="$1" ; shift
    ! run_pipeline "$OUT/$name" "$@" --require-aligned \
        && grep -q 'require-aligned' "$OUT/$name/stdout" "$OUT/$name/stderr"
}
check "(c) unaligned end refused" \
    refused unaligned-end "${loads[@]}" --start 0 --n-frames $(( N - 5 ))
check "(c) start outside the envelope refused" \
    refused outside-envelope --load-loc-data "${shards[1]}" --start $(( F - 10 )) --n-frames 20

echo "check_shard_parity: outputs in $OUT"
exit "$FAILED"
