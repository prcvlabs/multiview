#!/bin/bash
# Development loop for multiview: build, unit tests and a pipeline smoke test,
# from any git worktree of the repository. Builds and heavy runs are
# serialised across worktrees with file locks and run at low priority, so
# several branches can iterate on one machine.
#
# Usage: build/dev.sh <command> [args...]
#
#   build [Release|Debug] [--hyperpose] [targets...]
#       Configure (if needed) and build into this worktree's build directory:
#       multiview/multiview_cpp/build/cmake-{release,debug,hyperpose}. Builds from
#       different worktrees wait for each other (one build at a time).
#   test [--hyperpose] [catch-args...]
#       Run multiview-testcases, e.g. `test "[tracks]"` for one tag.
#   smoke [--hyperpose] [--frames N] [--update-reference]
#       Run the pipeline on testcase 007 with --no-stereo and replayed 2D poses
#       (no pose network, little GPU), score it against the ground truth, and
#       compare the metrics with the stored reference. Needs the replay files
#       in $MULTIVIEW_POSE_REPLAY/007_site03_sparse_tracks (see `replay`).
#       A passing run's pipeline-output.json is stored next to the reference
#       (reference-<N>f-output.json) when there is none, or on --update-reference.
#   rt-smoke [--hyperpose] [--frames N]
#       Run `realtime` unpaced, without drops, on the configuration of `smoke`,
#       and require its pipeline-output.json to equal the stored batch output
#       exactly (bin/compare-tracks.py --exact: ids, (x, y, t), fp_info but
#       lab_p), its NDJSON stream to be complete and consistent with it
#       (--stream, --revised), then compare the metrics with the reference.
#   replay [--frames N]
#       Generate replay files for 007 (--no-stereo, like `smoke`) with the
#       HyperPose build: a one-off GPU run over the whole testcase. Note that
#       the public videos have blurred faces, on which HyperPose finds few
#       people (recall 0.15); the poses found on the original videos are the
#       better replay set (F1 0.61), see tools/privacy/README.md.
#   pipeline [--hyperpose] <multiview-cli pipeline args...>
#       Run `multiview-cli pipeline ...` under the GPU lock at low priority.
#   realtime [--hyperpose] <multiview-cli realtime args...>
#       Run `multiview-cli realtime ...` under the GPU lock at low priority.
#   cli [--hyperpose] <multiview-cli args...>
#       Run multiview-cli with any subcommand (no lock).
#   check
#       build + test + smoke + rt-smoke, the pre-commit check for a branch.
#
# Environment:
#   MULTIVIEW_TESTCASE_DATA   default ~/multiview-data  (videos, see fetch-testcases.sh)
#   MULTIVIEW_POSE_REPLAY     default ~/multiview-data/pose-replay
#   MULTIVIEW_HYPERPOSE_DIR   default ~/multiview-data/hyperpose
#   MULTIVIEW_DEV_JOBS        compile jobs, default nproc-2
#   MULTIVIEW_DEV_NICE        niceness, default 15

set -euo pipefail

THIS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$THIS_DIR/.." && pwd)"
SRC_DIR="$ROOT/multiview/multiview_cpp"
DATA_D="${MULTIVIEW_TESTCASE_DATA:-$HOME/multiview-data}"
REPLAY_D="${MULTIVIEW_POSE_REPLAY:-$HOME/multiview-data/pose-replay}"
export MULTIVIEW_HYPERPOSE_DIR="${MULTIVIEW_HYPERPOSE_DIR:-$HOME/multiview-data/hyperpose}"
NICE="${MULTIVIEW_DEV_NICE:-15}"
BUILD_LOCK=/tmp/multiview-dev-build.lock
GPU_LOCK=/tmp/multiview-dev-gpu.lock
SMOKE_TC=007_site03_sparse_tracks

usage() { sed -n '2,46p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'; }

# ------------------------------------------------------------------ helpers

CONFIG=Release
HYPERPOSE=0

parse_variant() # strips [Release|Debug] and --hyperpose from the front of "$@"
{
    REST=()
    while [ "$#" -gt 0 ] ; do
        case "$1" in
            Release|Debug|RelWithDebInfo) CONFIG="$1" ;;
            --hyperpose) HYPERPOSE=1 ;;
            *) REST+=("$1") ;;
        esac
        shift
    done
}

build_dir() # cmake-release, cmake-hyperpose (the README's name), cmake-debug-hyperpose
{
    local cfg ; cfg="$(echo "$CONFIG" | tr '[:upper:]' '[:lower:]')"
    if [ "$HYPERPOSE" = "1" ] ; then
        [ "$cfg" = "release" ] && cfg="hyperpose" || cfg="$cfg-hyperpose"
    fi
    echo "$SRC_DIR/build/cmake-$cfg"
}

cli_bin()  { echo "$(build_dir)/multiview-cli" ; }
test_bin() { echo "$(build_dir)/multiview-testcases" ; }

with_lock() # with_lock <lockfile> <what> command...
{
    local lock="$1" what="$2" ; shift 2
    exec 9>"$lock"
    if ! flock -n 9 ; then
        echo "dev.sh: waiting for the $what lock ($lock)" >&2
        flock 9
    fi
    # Run the command as a child and forward a termination signal to it, so an
    # interrupted dev.sh does not leave an orphan holding the lock (or the GPU)
    "$@" &
    local child=$!
    trap 'kill -TERM $child 2>/dev/null' INT TERM
    wait $child
    local ret=$?
    trap - INT TERM
    flock -u 9
    return $ret
}

ensure_built()
{
    [ -x "$(cli_bin)" ] || { echo "dev.sh: $(cli_bin) is missing; run: build/dev.sh build" ; exit 1 ; }
}

# ------------------------------------------------------------------- build

do_build()
{
    local targets=("$@")
    local args="${MULTIVIEW_CMAKE_ARGS:-}"
    [ "$HYPERPOSE" = "1" ] && args="$args -DMULTIVIEW_WITH_HYPERPOSE=ON"
    local nproc ; nproc="$(nproc)"
    export CMAKE_BUILD_PARALLEL_LEVEL="${MULTIVIEW_DEV_JOBS:-$(( nproc > 3 ? nproc - 2 : 1 ))}"
    export MULTIVIEW_BUILD_DIR ; MULTIVIEW_BUILD_DIR="$(build_dir)"
    export MULTIVIEW_CMAKE_ARGS="$args"
    local start ; start="$(date +%s)"
    with_lock "$BUILD_LOCK" build \
        nice -n "$NICE" "$THIS_DIR/build.sh" "$CONFIG" "${targets[@]}"
    echo "dev.sh: build took $(( $(date +%s) - start ))s  ($(ccache -s 2>/dev/null | grep -m1 'Hits:' || true))"
}

# -------------------------------------------------------------------- test

do_test()
{
    ensure_built
    nice -n "$NICE" "$(test_bin)" "$@"
}

# ------------------------------------------------------------------- smoke
#
# The smoke test processes the first N frames of 007 with the 2D poses
# replayed from disk and --no-stereo, so no pose network runs. The metrics
# (F1, TP, FP, FN, identity switches) are compared with the reference stored
# next to the replay files, with a tolerance for the tracker's randomness.

smoke_params() # smoke_params <replay-dir> <out-file>
{
    local manifest="$DATA_D/testcases/$SMOKE_TC/manifest.json"
    local tmp ; tmp="$(mktemp -d)"
    "$(cli_bin)" dump_default_params -d "$tmp" "$manifest" > /dev/null
    python3 - "$tmp/default-params.json" "$1" "$2" <<'EOF'
import json, sys
params_file, replay_dir, out_file = sys.argv[1:]
# `pipeline -p` reads {"task_params": {"frame_params": ...}, "fowlkes_params": ...}
p = json.load(open(params_file))
tasks = p["task_params"] if "task_params" in p else p
pose = tasks["frame_params"]["pose_skeleton_init"]["op_params"]
pose["replay_dir"] = replay_dir
pose["use_hyperpose"] = True
json.dump(p if "task_params" in p else {"task_params": tasks}, open(out_file, "w"), indent=1)
EOF
    rm -rf "$tmp"
}

extract_metrics() # extract_metrics <stdout> -> json on stdout
{
    python3 - "$1" <<'EOF'
import json, re, sys
text = open(sys.argv[1], errors="replace").read()
text = re.sub(r"\x1b\[[0-9;]*m", "", text)
keys = {"true positives": "tp", "false positives": "fp", "false negatives": "fn",
        "f1": "f1", "total-id-switches": "id_switches", "total-frags": "frags",
        "precision": "precision", "recall": "recall"}
out = {}
for line in text.splitlines():
    m = re.match(r"\|\s*([a-z0-9\- ]+?):\s+([0-9.]+)", line)
    if m and m.group(1).strip() in keys:
        out[keys[m.group(1).strip()]] = float(m.group(2))
m = re.search(r"pipeline-seconds:\s+([0-9.]+)s", text)
if m: out["pipeline_seconds"] = float(m.group(1))
frames = re.findall(r"n-frames:\s+([0-9]+)", text)
if frames: out["n_frames"] = int(frames[-1])
out["success"] = bool(re.search(r"(pipeline|realtime)-execute done\.", text))
print(json.dumps(out, sort_keys=True))
EOF
}

compare_metrics() # compare_metrics <got.json> <reference.json>
{
    python3 - "$1" "$2" <<'EOF'
import json, sys
got, ref = (json.load(open(f)) for f in sys.argv[1:3])
tol = {"f1": 0.03, "precision": 0.05, "recall": 0.05}
bad = []
for k in ("tp", "fp", "fn", "f1", "precision", "recall", "id_switches", "frags"):
    if k not in ref or k not in got: continue
    if k in tol:
        ok = abs(got[k] - ref[k]) <= tol[k]
    else:
        ok = abs(got[k] - ref[k]) <= max(2, 0.05 * ref[k])
    print(f"   {k:12s} got {got[k]:8.3f}   reference {ref[k]:8.3f}   {'ok' if ok else 'DIFFERS'}")
    if not ok: bad.append(k)
if not got.get("success"):
    print("   pipeline did not finish"); bad.append("success")
sys.exit(1 if bad else 0)
EOF
}

do_smoke()
{
    ensure_built
    local frames=120 update=0
    while [ "$#" -gt 0 ] ; do
        case "$1" in
            --frames) shift ; frames="$1" ;;
            --update-reference) update=1 ;;
            *) echo "smoke: unknown argument '$1'" ; exit 1 ;;
        esac
        shift
    done
    local tc="$DATA_D/testcases/$SMOKE_TC" replay="$REPLAY_D/$SMOKE_TC"
    [ -f "$tc/manifest.json" ] || { echo "smoke: fetch the testcase first: build/fetch-testcases.sh 7" ; exit 1 ; }
    [ -f "$replay/pose-2d_000000_00.json" ] || { echo "smoke: no replay files in $replay; run: build/dev.sh replay" ; exit 1 ; }
    local out="${TMPDIR:-/tmp}/multiview-smoke-$(basename "$ROOT")-$$"
    mkdir -p "$out"
    smoke_params "$replay" "$out/smoke-params.json"
    echo "smoke: $SMOKE_TC, $frames frames, pose replay, --no-stereo -> $out"
    local start ; start="$(date +%s)"
    with_lock "$GPU_LOCK" gpu \
        nice -n "$NICE" "$(cli_bin)" pipeline -m "$tc/manifest.json" -p "$out/smoke-params.json" -d "$out" -y \
            --no-stereo --start 0 --n-frames "$frames" --pose-classifier pose-svm-lite_v0 \
            --tracks-file "$tc/ground-truth.json" --pipeline-testcase --verbosity 0 \
            > "$out/stdout" 2> "$out/stderr" || true
    echo "smoke: pipeline took $(( $(date +%s) - start ))s"
    extract_metrics "$out/stdout" > "$out/metrics.json"
    cat "$out/metrics.json"
    local ref="$replay/reference-${frames}f.json"
    if [ "$update" = "1" ] || [ ! -f "$ref" ] ; then
        cp "$out/metrics.json" "$ref"
        echo "smoke: reference written to $ref"
    fi
    if ! compare_metrics "$out/metrics.json" "$ref" ; then
        echo "smoke: FAIL (log: $out/stdout, errors: $out/stderr)"
        return 1
    fi
    # Only a passing run becomes the output that rt-smoke compares against
    local ref_output="$replay/reference-${frames}f-output.json"
    if [ -f "$out/pipeline-output.json" ] && { [ "$update" = "1" ] || [ ! -f "$ref_output" ] ; } ; then
        cp "$out/pipeline-output.json" "$ref_output"
        echo "smoke: reference output written to $ref_output"
    fi
    echo "smoke: PASS (log: $out/stdout)"
}

do_rt_smoke()
{
    ensure_built
    local frames=120
    while [ "$#" -gt 0 ] ; do
        case "$1" in
            --frames) shift ; frames="$1" ;;
            *) echo "rt-smoke: unknown argument '$1'" ; exit 1 ;;
        esac
        shift
    done
    local tc="$DATA_D/testcases/$SMOKE_TC" replay="$REPLAY_D/$SMOKE_TC"
    local ref_output="$replay/reference-${frames}f-output.json"
    [ -f "$tc/manifest.json" ] || { echo "rt-smoke: fetch the testcase first: build/fetch-testcases.sh 7" ; exit 1 ; }
    [ -f "$ref_output" ] || { echo "rt-smoke: no batch output $ref_output; run: build/dev.sh smoke --frames $frames" ; exit 1 ; }
    local out="${TMPDIR:-/tmp}/multiview-rt-smoke-$(basename "$ROOT")-$$"
    mkdir -p "$out"
    smoke_params "$replay" "$out/smoke-params.json"
    echo "rt-smoke: $SMOKE_TC, $frames frames, realtime unpaced without drops -> $out"
    local start ; start="$(date +%s)"
    with_lock "$GPU_LOCK" gpu \
        nice -n "$NICE" "$(cli_bin)" realtime -m "$tc/manifest.json" -p "$out/smoke-params.json" -d "$out" -y \
            --no-stereo --start 0 --n-frames "$frames" --pose-classifier pose-svm-lite_v0 \
            --tracks-file "$tc/ground-truth.json" --pipeline-testcase --verbosity 0 \
            --replay-rate 0 --drop none --slots 2 --out "$out/stream.ndjson" \
            > "$out/stdout" 2> "$out/stderr" || echo "rt-smoke: multiview-cli exited with status $?"
    echo "rt-smoke: realtime took $(( $(date +%s) - start ))s"
    local cmp="$ROOT/bin/compare-tracks.py" failed=0
    [ -f "$out/pipeline-output.json" ] || { echo "rt-smoke: FAIL, no pipeline-output.json (log: $out/stderr)" ; return 1 ; }
    local removed
    removed="$(python3 -c 'import json, sys; s = json.load(open(sys.argv[1])); print(s["raw_tracks"] - s["kept_tracks"])' "$out/realtime-stats.json")"
    python3 "$cmp" --exact "$out/pipeline-output.json" "$ref_output" || failed=1
    python3 "$cmp" --stream "$out/stream.ndjson" --frames "$frames" \
        --output "$out/pipeline-output.json" --removed "$removed" || failed=1
    python3 "$cmp" --revised "$out/stream.ndjson" "$out/pipeline-output-presmooth.json" || failed=1
    cat "$out/stdout" "$out/stderr" > "$out/log"
    extract_metrics "$out/log" > "$out/metrics.json"
    cat "$out/metrics.json"
    compare_metrics "$out/metrics.json" "$replay/reference-${frames}f.json" || failed=1
    if [ "$failed" = "0" ] ; then
        echo "rt-smoke: PASS (log: $out/stderr)"
    else
        echo "rt-smoke: FAIL (log: $out/stderr)"
        return 1
    fi
}

do_replay()
{
    HYPERPOSE=1
    ensure_built
    local frames=-1
    while [ "$#" -gt 0 ] ; do
        case "$1" in
            --frames) shift ; frames="$1" ;;
            *) echo "replay: unknown argument '$1'" ; exit 1 ;;
        esac
        shift
    done
    local tc="$DATA_D/testcases/$SMOKE_TC" replay="$REPLAY_D/$SMOKE_TC"
    mkdir -p "$replay"
    local tmp ; tmp="$(mktemp -d)"
    "$(cli_bin)" dump_default_params -d "$tmp" "$tc/manifest.json" > /dev/null
    python3 - "$tmp/default-params.json" "$replay/replay-params.json" <<'EOF'
import json, sys
p = json.load(open(sys.argv[1]))
tasks = p["task_params"] if "task_params" in p else p
pose = tasks["frame_params"]["pose_skeleton_init"]["op_params"]
pose["feedback"] = True
pose["use_hyperpose"] = True
json.dump(p if "task_params" in p else {"task_params": tasks}, open(sys.argv[2], "w"), indent=1)
EOF
    rm -rf "$tmp"
    echo "replay: writing pose-2d files for $SMOKE_TC to $replay (frames: $frames)"
    with_lock "$GPU_LOCK" gpu \
        nice -n "$NICE" "$(cli_bin)" pipeline -m "$tc/manifest.json" -p "$replay/replay-params.json" -d "$replay" -y \
            --no-stereo --start 0 --n-frames "$frames" --pose-classifier pose-svm-lite_v0 \
            --tracks-file "$tc/ground-truth.json" --pipeline-testcase --verbosity 1 \
            > "$replay/stdout" 2> "$replay/stderr" || echo "replay: multiview-cli exited with status $?"
    echo "replay: $(ls "$replay" | grep -c '^pose-2d_') files; metrics of the HyperPose run:"
    extract_metrics "$replay/stdout"
}

# -------------------------------------------------------------------- main

[ "$#" -gt 0 ] || { usage ; exit 1 ; }
CMD="$1" ; shift
parse_variant "$@"
set -- "${REST[@]+"${REST[@]}"}"

case "$CMD" in
    build)    do_build "$@" ;;
    test)     do_test "$@" ;;
    smoke)    do_smoke "$@" ;;
    rt-smoke) do_rt_smoke "$@" ;;
    replay)   do_replay "$@" ;;
    pipeline) ensure_built ; with_lock "$GPU_LOCK" gpu nice -n "$NICE" "$(cli_bin)" pipeline "$@" ;;
    realtime) ensure_built ; with_lock "$GPU_LOCK" gpu nice -n "$NICE" "$(cli_bin)" realtime "$@" ;;
    cli)      ensure_built ; nice -n "$NICE" "$(cli_bin)" "$@" ;;
    check)    do_build && do_test && do_smoke && do_rt_smoke ;;
    -h|--help|help) usage ;;
    *) echo "dev.sh: unknown command '$CMD'" ; usage ; exit 1 ;;
esac
