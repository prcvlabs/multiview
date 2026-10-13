#!/bin/bash
# Reference runs on testcase 007 for the cloud-distributed parity tests.
#
# Usage: tools/cloud/tests/make_reference.sh [--frames N]... [--runs R1,R2,R3]
#                                            [--no-still]
#                                            [--cli <multiview-cli>] [--out <dir>]
#
# For each N (default 120 and 499), all with --no-stereo and the 2D poses
# replayed from $MULTIVIEW_POSE_REPLAY/007_site03_sparse_tracks (the pose
# classifier is only passed where the tracker runs):
#
#   R1  batch run: --tracks-file ground-truth.json --pipeline-testcase
#   R2  loc-shard run over the whole range: --save-loc-data --no-tracks
#   R3  tracks job: --load-loc-data R2/localization.data, scored like R1
#
# Products go to <out>/n<N>/{R1,R2,R3}/ (default out:
# $MULTIVIEW_TESTCASE_DATA/cloud-golden/007), with R2's envelope copied to
# <out>/ref-<N>.data and the T0a check (R3 track_results == R1's with
# fp_info.lab_p nulled) written to <out>/n<N>/t0a.json. Each run's wall time
# and maximum RSS are in its time.txt. Every run takes the GPU lock that
# build/dev.sh uses, runs at nice 15 and is memory-capped.
#
# --no-still passes --no-still to R2 and R3, whose products then go to
# R2-no-still, R3-no-still, ref-<N>-no-still.data and t0a-no-still.json
# (R1 is shared, so --runs defaults to R2,R3). ref-<N>-no-still.data must
# equal ref-<N>.data byte for byte; t0a-no-still.json shows what the still
# changes in a tracks job (the pose classifier's input, hence the pose
# labels, and lab_p).

set -euo pipefail

THIS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$THIS_DIR/../../.." && pwd)"
DATA_D="${MULTIVIEW_TESTCASE_DATA:-$HOME/multiview-data}"
REPLAY_D="${MULTIVIEW_POSE_REPLAY:-$HOME/multiview-data/pose-replay}"
TC=007_site03_sparse_tracks
GPU_LOCK=/tmp/multiview-dev-gpu.lock
NICE="${MULTIVIEW_DEV_NICE:-15}"
MEM="${MULTIVIEW_CLOUD_MEM:-24G}"

CLI="$ROOT/multiview/multiview_cpp/build/cmake-release/multiview-cli"
OUT="$DATA_D/cloud-golden/007"
FRAMES=()
RUNS=""
NO_STILL=()
SUF=""

while [ "$#" -gt 0 ] ; do
    case "$1" in
        --frames) shift ; FRAMES+=("$1") ;;
        --runs)   shift ; RUNS="$1" ;;
        --no-still) NO_STILL=(--no-still) ; SUF="-no-still" ;;
        --cli)    shift ; CLI="$1" ;;
        --out)    shift ; OUT="$1" ;;
        -h|--help) sed -n '2,/^$/p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//' ; exit 0 ;;
        *) echo "make_reference: unknown argument '$1'" ; exit 1 ;;
    esac
    shift
done
[ "${#FRAMES[@]}" -gt 0 ] || FRAMES=(120 499)
if [ -z "$RUNS" ] ; then
    if [ -n "$SUF" ] ; then RUNS="R2,R3" ; else RUNS="R1,R2,R3" ; fi
fi

MANIFEST="$DATA_D/testcases/$TC/manifest.json"
GT="$DATA_D/testcases/$TC/ground-truth.json"
REPLAY="$REPLAY_D/$TC"
[ -x "$CLI" ] || { echo "make_reference: no binary at $CLI (build/dev.sh build)" ; exit 1 ; }
[ -f "$MANIFEST" ] || { echo "make_reference: missing $MANIFEST" ; exit 1 ; }
[ -f "$REPLAY/pose-2d_000000_00.json" ] || { echo "make_reference: no replay files in $REPLAY" ; exit 1 ; }

capped_cmd() # capped (systemd scope with a memory cap, nice 15) if present
{
    local capped ; capped="$(command -v capped || echo "$HOME/.local/bin/capped")"
    if [ -x "$capped" ] ; then echo "$capped -m $MEM" ; else echo "nice -n $NICE" ; fi
}

make_params() # make_params <out-file>: default params with the replay directory set
{
    local tmp ; tmp="$(mktemp -d)"
    "$CLI" dump_default_params -d "$tmp" "$MANIFEST" > /dev/null
    python3 - "$tmp/default-params.json" "$REPLAY" "$1" <<'EOF'
import json, sys
params_file, replay_dir, out_file = sys.argv[1:]
p = json.load(open(params_file))
pose = p["task_params"]["frame_params"]["pose_skeleton_init"]["op_params"]
pose["replay_dir"] = replay_dir
pose["use_hyperpose"] = True
json.dump(p, open(out_file, "w"), indent=1)
EOF
    rm -rf "$tmp"
}

run_pipeline() # run_pipeline <dir> pipeline-args...: logs, wall time and max RSS in <dir>
{
    local dir="$1" ; shift
    rm -rf "$dir" ; mkdir -p "$dir"
    local ret=0
    # shellcheck disable=SC2046
    flock "$GPU_LOCK" $(capped_cmd) /usr/bin/time -v -o "$dir/rusage.txt" \
        "$CLI" pipeline -m "$MANIFEST" -p "$OUT/params-replay.json" -d "$dir" -y \
        --no-stereo --verbosity 1 "$@" \
        > "$dir/stdout" 2> "$dir/stderr" || ret=$?
    # /usr/bin/time runs inside the lock: waiting for the GPU is not counted
    python3 - "$dir/rusage.txt" "$ret" "$*" > "$dir/time.txt" <<'EOF'
import re, sys
text = open(sys.argv[1]).read()
m = re.search(r"Elapsed \(wall clock\) time.*: (?:(\d+):)?(\d+):([\d.]+)", text)
wall = (int(m.group(1) or 0) * 3600 + int(m.group(2)) * 60 + float(m.group(3))) if m else -1.0
m = re.search(r"Maximum resident set size \(kbytes\): (\d+)", text)
print(f"exit {sys.argv[2]}\nwall_seconds {wall:.1f}\nmax_rss_kb {m.group(1) if m else -1}\nargs {sys.argv[3]}")
EOF
    echo "make_reference: $(basename "$(dirname "$dir")")/$(basename "$dir"): exit $ret, $(sed -n 2p "$dir/time.txt")"
    return "$ret"
}

mkdir -p "$OUT"
make_params "$OUT/params-replay.json"

for n in "${FRAMES[@]}" ; do
    d="$OUT/n$n"
    mkdir -p "$d"
    range=(--start 0 --n-frames "$n")
    if [[ ",$RUNS," == *",R1,"* ]] ; then
        run_pipeline "$d/R1" "${range[@]}" --pose-classifier pose-svm-lite_v0 \
            --tracks-file "$GT" --pipeline-testcase
    fi
    if [[ ",$RUNS," == *",R2,"* ]] ; then
        run_pipeline "$d/R2$SUF" "${range[@]}" --save-loc-data --no-tracks "${NO_STILL[@]}"
        cp "$d/R2$SUF/localization.data" "$OUT/ref-$n$SUF.data"
        ls -l "$OUT/ref-$n$SUF.data" | awk -v f="ref-$n$SUF.data" -v n="$n" '{printf "make_reference: %s %d bytes, %.0f bytes/frame\n", f, $5, $5 / n}'
        if [ -n "$SUF" ] && [ -f "$OUT/ref-$n.data" ] ; then
            if cmp -s "$OUT/ref-$n.data" "$OUT/ref-$n$SUF.data" ; then
                echo "make_reference: ref-$n$SUF.data == ref-$n.data (bytes)"
            else
                echo "make_reference: ref-$n$SUF.data DIFFERS from ref-$n.data"
            fi
        fi
    fi
    if [[ ",$RUNS," == *",R3,"* ]] ; then
        run_pipeline "$d/R3$SUF" "${range[@]}" --load-loc-data "$OUT/ref-$n$SUF.data" \
            "${NO_STILL[@]}" --pose-classifier pose-svm-lite_v0 \
            --tracks-file "$GT" --pipeline-testcase
    fi
    if [ -f "$d/R1/pipeline-output.json" ] && [ -f "$d/R3$SUF/pipeline-output.json" ] ; then
        python3 "$THIS_DIR/compare_tracks.py" --json "$d/t0a$SUF.json" \
            "$d/R1/pipeline-output.json" "$d/R3$SUF/pipeline-output.json" || true
    fi
done
