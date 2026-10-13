#!/bin/bash
# End-to-end test of the mvcloud agent with the filesystem backend, on
# testcase 007 with --no-stereo and the 2D poses replayed (the PR gate).
#
# Usage: tools/cloud/tests/e2e_fs.sh [--tests T1,T3,T5] [--cli <multiview-cli>]
#                                    [--cli-hyperpose <multiview-cli>]
#                                    [--out <dir>] [--keep]
#
#   T1  [0,120), shard_frames 60: two loc shards and the tracks job through
#       `mvcloud submit` and one `mvcloud agent --drain`. Asserts
#       (a) `ldata cat` of the shards' envelopes, `ldata diff` against
#           ref-120.data: exit 0, every frame byte-identical
#       (b) the tracks job's track_results == R1's (compare_tracks.py,
#           fp_info.lab_p excluded)
#       (c) the tracks job's score.json == `score` of R1
#       (d) every shard has exactly one attempt, done; the done.json files
#           have the same params_md5 and the reference's frame md5s
#       (e) exit status 0 of submit, the agent and every shard
#       (f) no process of any shard's session survives
#   T3  the same with one loc shard over [0,120): zero tolerance
#   T5  idempotency and failure injection (needs T1's root):
#       a re-run agent finds nothing to do; a loc shard whose done.json is
#       deleted and whose envelope is corrupted is re-run, then the tracks
#       job, and (a)-(c) hold again; MVCLOUD_FAULT=kill-after:20 kills one
#       shard once and the job still completes; n_frames 1 and a range past
#       the scene are refused at submit; a tracks job made unaligned through
#       extra_cli_args fails fast (one attempt, fatal, --require-aligned).
#   T4  full-007 parity (not in the default set, ~25 min of GPU): [0,499),
#       shard_frames 250, (a)-(f) against make_reference.sh --frames 499
#   T6  008 HyperPose smoke (not in the default set; the HyperPose build,
#       --cli-hyperpose): [0,149), shard_frames 80, pose_mode hyperpose. A
#       full `--save-loc-data --no-tracks` run in one process is the
#       reference; the job must finish and every HyperPose process must log
#       its TensorRT engine (cache hit or miss); the frame md5s, `ldata diff` and
#       the tracks are recorded, not gated (TensorRT need not be
#       bit-reproducible across processes).
#
# Needs the references of make_reference.sh --frames 120 (and --frames 499
# for T4) in $MULTIVIEW_TESTCASE_DATA/cloud-golden/007. The agent takes the GPU lock
# that build/dev.sh uses and runs each shard at nice 15 under a 24G cap.
# Outputs go to <out> (default a new directory under ${TMPDIR:-/tmp});
# envelopes and caches are deleted at the end unless --keep or a failure.
# Exit status 0 if every assertion holds.

set -euo pipefail

THIS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$THIS_DIR/../../.." && pwd)"
DATA_D="${MULTIVIEW_TESTCASE_DATA:-$HOME/multiview-data}"
REPLAY_D="${MULTIVIEW_POSE_REPLAY:-$HOME/multiview-data/pose-replay}"
TC=007_site03_sparse_tracks
MEM="${MULTIVIEW_CLOUD_MEM:-24G}"
# The HyperPose models for T6, as build/dev.sh finds them
export MULTIVIEW_HYPERPOSE_DIR="${MULTIVIEW_HYPERPOSE_DIR:-$HOME/multiview-data/hyperpose}"

CLI="$ROOT/multiview/multiview_cpp/build/cmake-release/multiview-cli"
HP_CLI="$ROOT/multiview/multiview_cpp/build/cmake-hyperpose/multiview-cli"
TESTS="T1,T3,T5"
OUT=""
KEEP=0

while [ "$#" -gt 0 ] ; do
    case "$1" in
        --tests) shift ; TESTS="$1" ;;
        --cli)   shift ; CLI="$1" ;;
        --cli-hyperpose) shift ; HP_CLI="$1" ;;
        --out)   shift ; OUT="$1" ;;
        --keep)  KEEP=1 ;;
        -h|--help) sed -n '2,/^$/p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//' ; exit 0 ;;
        *) echo "e2e_fs: unknown argument '$1'" ; exit 1 ;;
    esac
    shift
done

GOLD="$DATA_D/cloud-golden/007"
MANIFEST="$DATA_D/testcases/$TC/manifest.json"
GT="$DATA_D/testcases/$TC/ground-truth.json"
REPLAY="$REPLAY_D/$TC"
TC6=008_site04_standing
MANIFEST6="$DATA_D/testcases/$TC6/manifest.json"
NEED=("$CLI" "$MANIFEST" "$GT" "$GOLD/params-replay.json" "$REPLAY/pose-2d_000000_00.json")
for n in 120 $([[ ",$TESTS," == *",T4,"* ]] && echo 499) ; do
    NEED+=("$GOLD/ref-$n.data" "$GOLD/n$n/R1/pipeline-output.json")
done
[[ ",$TESTS," == *",T6,"* ]] && NEED+=("$HP_CLI" "$MANIFEST6")
for f in "${NEED[@]}" ; do
    [ -e "$f" ] || { echo "e2e_fs: missing $f (make_reference.sh --frames 120 [--frames 499])" ; exit 1 ; }
done
[ -n "$OUT" ] || OUT="$(mktemp -d "${TMPDIR:-/tmp}/e2e-fs.XXXXXX")"
mkdir -p "$OUT"
CLI="$(cd "$(dirname "$CLI")" && pwd)/$(basename "$CLI")"
[ -e "$HP_CLI" ] && HP_CLI="$(cd "$(dirname "$HP_CLI")" && pwd)/$(basename "$HP_CLI")"

export PYTHONPATH="$ROOT/tools/cloud/agent${PYTHONPATH:+:$PYTHONPATH}"
CHECK=(python3 "$THIS_DIR/e2e_check.py")
FAILED=0
SIDS="$OUT/sessions.txt"
: > "$SIDS"

check() # check <name> <command...>: records a failure, does not stop
{
    local name="$1" ; shift
    if "$@" ; then echo "e2e_fs: PASS $name" ; else echo "e2e_fs: FAIL $name" ; FAILED=1 ; fi
}

record() # record <name> <command...>: reports the outcome, never fails
{
    local name="$1" ; shift
    if "$@" ; then echo "e2e_fs: RECORD $name: yes" ; else echo "e2e_fs: RECORD $name: no" ; fi
}

make_job() # make_job <file> <json object of overrides>
{
    python3 - "$1" "$2" "$MANIFEST" "$GOLD/params-replay.json" "$REPLAY" "$GT" <<'EOF'
import json, sys
fname, overrides, manifest, params, poses, gt = sys.argv[1:]
job = {"scene_id": "007_site03_sparse_tracks", "manifest": manifest, "params": params,
       "poses": poses, "gt": gt, "start_frame": 0, "n_frames": 120, "shard_frames": 60,
       "no_stereo": True, "pose_mode": "replay", "pose_classifier": "pose-svm-lite_v0",
       "heartbeat_s": 6, "lease_seconds": 120, "progress_timeout_s": 900, "retry_delay_s": 0}
job.update(json.loads(overrides))
json.dump(job, open(fname, "w"), indent=1)
EOF
}

submit() # submit <root> <job.json> [cli]: prints the job id
{
    python3 -m mvcloud submit --control "fs://$1" --cli "${3:-$CLI}" "$2" 2>> "$OUT/submit.log"
}

run_agent() # run_agent <root> <log> [cli]: one agent until there is no work
{
    local ret=0
    python3 -m mvcloud agent --control "fs://$1" --cli "${3:-$CLI}" --cache "$OUT/cache" \
        --work "$OUT/work" --mem "$MEM" --kill-grace 20 --drain 2>> "$2" || ret=$?
    sed -n 's/.*started session \([0-9]*\):.*/\1/p' "$2" >> "$SIDS"
    echo "e2e_fs: agent exit $ret ($(grep -c 'started session' "$2" || true) sessions so far in $(basename "$2"))"
    return "$ret"
}

no_survivors() # no process of any recorded session is alive
{
    local sid alive=0
    for sid in $(sort -u "$SIDS") ; do
        if pgrep -s "$sid" > /dev/null ; then echo "   session $sid: $(pgrep -s "$sid" | tr '\n' ' ')" ; alive=1 ; fi
    done
    return "$alive"
}

references() # references <n>: R1's score and the reference envelope's frame md5s
{
    [ -f "$OUT/ref-$1.info.json" ] && return
    "$CLI" score --calc "$GOLD/n$1/R1/pipeline-output.json" --gt "$GT" \
        --json "$OUT/r1-$1-score.json" > "$OUT/r1-$1-score.stdout"
    "$CLI" ldata info "$GOLD/ref-$1.data" > "$OUT/ref-$1.info.json"
}

assert_job() # assert_job <name> <root> <job_id> <n> [--fresh]: (a)-(d)
{
    local name="$1" root="$2" job="$3" n="$4" fresh="${5:-}"
    local d="$OUT/$name"
    mkdir -p "$d"
    references "$n"
    check "$name (d) shards, attempts, params_md5, frame md5s" \
        "${CHECK[@]}" job "$root" "$job" "$OUT/ref-$n.info.json" "$d" $fresh
    [ -f "$d/envs.txt" ] || { FAILED=1 ; return ; }
    mapfile -t envs < "$d/envs.txt"
    "$CLI" ldata cat -y -o "$d/merged.data" "${envs[@]}" > "$d/cat.stdout" 2>&1 || true
    check "$name (a) ldata diff merged ref-$n.data (bytes)" \
        "$CLI" ldata diff --json "$d/diff.json" "$d/merged.data" "$GOLD/ref-$n.data"
    check "$name (b) tracks job == R1 (lab_p excluded)" \
        python3 "$THIS_DIR/compare_tracks.py" --json "$d/tracks-vs-R1.json" \
            "$GOLD/n$n/R1/pipeline-output.json" "$(cat "$d/tracks.txt")"
    check "$name (c) score.json == score of R1" \
        "${CHECK[@]}" json-equal "$OUT/r1-$n-score.json" "$(cat "$d/score.txt")"
    rm -f "$d/merged.data"
}

run_case() # run_case <name> <overrides> [n]: submit + agent + (a)-(f) on a fresh root
{
    local name="$1" overrides="$2" n="${3:-120}"
    local root="$OUT/$name/root" t0 ; t0="$(date +%s)"
    mkdir -p "$OUT/$name"
    make_job "$OUT/$name/job.json" "$overrides"
    local job ; job="$(submit "$root" "$OUT/$name/job.json")" || { echo "e2e_fs: FAIL $name submit" ; FAILED=1 ; return ; }
    echo "$job" > "$OUT/$name/job-id"
    echo "e2e_fs: $name: job $job"
    check "$name (e) agent exit 0" run_agent "$root" "$OUT/$name/agent.log"
    assert_job "$name" "$root" "$job" "$n" --fresh
    check "$name (f) no process of the shards' sessions survives" no_survivors
    echo "e2e_fs: $name took $(( $(date +%s) - t0 )) s"
}

if [[ ",$TESTS," == *",T1,"* ]] ; then
    run_case T1 '{"shard_frames": 60}'
fi
if [[ ",$TESTS," == *",T3,"* ]] ; then
    run_case T3 '{"shard_frames": 120}'
fi

if [[ ",$TESTS," == *",T5,"* ]] ; then
    t0="$(date +%s)"
    T1_ROOT="$OUT/T1/root"
    [ -f "$OUT/T1/job-id" ] || { echo "e2e_fs: T5 needs T1" ; exit 1 ; }
    job="$(cat "$OUT/T1/job-id")"
    mkdir -p "$OUT/T5"

    # A finished job: a second agent finds nothing to do
    "${CHECK[@]}" snapshot "$T1_ROOT" "$OUT/T5/before.json"
    check "T5 re-run agent exit 0" run_agent "$T1_ROOT" "$OUT/T5/agent-rerun.log"
    check "T5 re-run changes nothing" "${CHECK[@]}" unchanged "$T1_ROOT" "$OUT/T5/before.json"

    # done.json deleted and the envelope corrupted: the shard and the tracks
    # job run again, and the result is the same
    prefix="$("${CHECK[@]}" prefix "$T1_ROOT" "$job" loc-0001)"
    rm "$prefix/done.json"
    dd if=/dev/urandom of="$prefix/localization.data" bs=1 seek=100000 count=64 conv=notrunc status=none
    python3 -m mvcloud sweep --control "fs://$T1_ROOT" | tee "$OUT/T5/sweep.txt"
    check "T5 sweep re-queues loc-0001" grep -q "loc-0001: invalidated: done.json is missing" "$OUT/T5/sweep.txt"
    check "T5 invalidation agent exit 0" run_agent "$T1_ROOT" "$OUT/T5/agent-invalidated.log"
    check "T5 loc-0001 ran twice" "${CHECK[@]}" shard "$T1_ROOT" "$job" loc-0001 done done,done
    check "T5 tracks ran twice" "${CHECK[@]}" shard "$T1_ROOT" "$job" tracks done done,done
    check "T5 loc-0000 ran once" "${CHECK[@]}" shard "$T1_ROOT" "$job" loc-0000 done done
    assert_job T5-invalidated "$T1_ROOT" "$job" 120

    # Fault injection: the first shard's session is killed once
    make_job "$OUT/T5/fault.json" '{"n_frames": 40, "shard_frames": 20}'
    root="$OUT/T5/fault-root"
    fjob="$(submit "$root" "$OUT/T5/fault.json")"
    MVCLOUD_FAULT=kill-after:20 run_agent "$root" "$OUT/T5/agent-fault.log" || FAILED=1
    check "T5 fault: job done" "${CHECK[@]}" job-state "$root" "$fjob" done
    check "T5 fault: loc-0000 killed once, then done" \
        "${CHECK[@]}" shard "$root" "$fjob" loc-0000 done retry,done "fault injection"
    check "T5 fault: no process of the shards' sessions survives" no_survivors

    # Refused at submit
    make_job "$OUT/T5/one-frame.json" '{"n_frames": 1}'
    check "T5 n_frames 1 refused" \
        bash -c "! python3 -m mvcloud submit --control fs://$OUT/T5/refused --cli $CLI $OUT/T5/one-frame.json"
    make_job "$OUT/T5/too-long.json" '{"n_frames": 500}'
    check "T5 range past the 499-frame scene refused" \
        bash -c "! python3 -m mvcloud submit --control fs://$OUT/T5/refused --cli $CLI $OUT/T5/too-long.json"

    # An unaligned tracks job fails fast: one attempt, fatal
    make_job "$OUT/T5/unaligned.json" \
        '{"n_frames": 20, "shard_frames": 20, "extra_cli_args": {"tracks": ["--n-frames", "15"]}}'
    root="$OUT/T5/unaligned-root"
    ujob="$(submit "$root" "$OUT/T5/unaligned.json")"
    run_agent "$root" "$OUT/T5/agent-unaligned.log" || FAILED=1
    check "T5 unaligned: job failed" "${CHECK[@]}" job-state "$root" "$ujob" failed
    check "T5 unaligned: tracks dead after one fatal attempt" \
        "${CHECK[@]}" shard "$root" "$ujob" tracks dead fatal "--require-aligned"
    check "T5 no process of the shards' sessions survives" no_survivors
    echo "e2e_fs: T5 took $(( $(date +%s) - t0 )) s"
fi

if [[ ",$TESTS," == *",T4,"* ]] ; then
    run_case T4 '{"n_frames": 499, "shard_frames": 250}' 499
fi

if [[ ",$TESTS," == *",T6,"* ]] ; then
    t0="$(date +%s)"
    d="$OUT/T6"
    mkdir -p "$d"
    # 008's default params (a params file names its scene's sensors), and
    # those as the agent gives them to a HyperPose loc shard
    "$CLI" dump_default_params -d "$d" "$MANIFEST6" > "$d/dump-params.stdout" 2>&1 || FAILED=1
    python3 -c "
import json, sys
from mvcloud import params
p = params.patch_params(json.load(open(sys.argv[1])), pose_mode='hyperpose')
json.dump(p, open(sys.argv[2], 'w'), indent=1)" "$d/default-params.json" "$d/params-hyperpose.json"

    # The reference: every frame in one process
    t1="$(date +%s)"
    "$ROOT/build/dev.sh" pipeline --hyperpose -m "$MANIFEST6" -p "$d/params-hyperpose.json" \
        -d "$d/ref" -y --start 0 --n-frames 149 --save-loc-data --no-tracks --no-stereo \
        --no-still --verbosity 1 > "$d/ref.stdout" 2>&1 \
        || { echo "e2e_fs: FAIL T6 reference run" ; FAILED=1 ; }
    echo "e2e_fs: T6 reference run took $(( $(date +%s) - t1 )) s (with the wait for the GPU lock)"
    "$CLI" ldata info "$d/ref/localization.data" > "$d/ref.info.json" || FAILED=1

    cat > "$d/job.json" <<EOF6
{"scene_id": "$TC6", "manifest": "$MANIFEST6", "params": "$d/default-params.json",
 "start_frame": 0, "n_frames": 149, "shard_frames": 80, "no_stereo": true,
 "pose_mode": "hyperpose", "pose_classifier": "pose-svm-lite_v0",
 "heartbeat_s": 6, "lease_seconds": 120, "progress_timeout_s": 900, "retry_delay_s": 0}
EOF6
    job="$(submit "$d/root" "$d/job.json" "$HP_CLI")" || { echo "e2e_fs: FAIL T6 submit" ; FAILED=1 ; }
    echo "e2e_fs: T6: job $job"
    check "T6 agent exit 0" run_agent "$d/root" "$d/agent.log" "$HP_CLI"
    check "T6 job done" "${CHECK[@]}" job-state "$d/root" "$job" done
    record "T6 frame md5s == the one-process run's" \
        "${CHECK[@]}" job "$d/root" "$job" "$d/ref.info.json" "$d" --fresh
    if [ -f "$d/envs.txt" ] ; then
        mapfile -t envs < "$d/envs.txt"
        "$CLI" ldata cat -y -o "$d/merged.data" "${envs[@]}" > "$d/cat.stdout" 2>&1 || FAILED=1
        record "T6 ldata diff merged vs the one-process run (bytes)" \
            "$CLI" ldata diff --json "$d/diff.json" "$d/merged.data" "$d/ref/localization.data"
        python3 -c "
import json, sys
o = json.load(open(sys.argv[1]))
print('e2e_fs: T6 tracks job: %d tracks' % len(o['track_results']))
" "$(cat "$d/tracks.txt")" || FAILED=1
        rm -f "$d/merged.data"
    fi
    # The TensorRT engine line of every process that runs HyperPose: the
    # reference and the loc shards (the tracks job reads the poses from the
    # envelopes)
    for f in "$d/ref.stdout" $(find "$d/root/blobs" -path '*/loc-*/stdout.log' | sort) ; do
        echo "$f: $(grep -ao 'TensorRT engine cache [a-z]* .*' "$f" | head -n 1)"
    done | tee "$d/engine-cache.txt"
    check "T6 every process logged its TensorRT engine" \
        bash -c "! grep -q ': $' '$d/engine-cache.txt'"
    check "T6 no process of the shards' sessions survives" no_survivors
    [ "$KEEP" = "0" ] && rm -f "$d/ref/localization.data"
    echo "e2e_fs: T6 took $(( $(date +%s) - t0 )) s"
fi

if [ "$FAILED" = "0" ] && [ "$KEEP" = "0" ] ; then
    rm -rf "$OUT/cache" "$OUT/work" "$OUT"/*/root/blobs "$OUT"/T5/*-root/blobs
fi
if [ "$FAILED" = "0" ] ; then echo "e2e_fs: PASS (outputs in $OUT)" ; else echo "e2e_fs: FAIL (outputs in $OUT)" ; fi
exit "$FAILED"
