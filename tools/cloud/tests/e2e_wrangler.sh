#!/bin/bash
# End-to-end test T2 of the mvcloud Worker under `wrangler dev --env local`
# (D1, R2, Queues and Workflows in Miniflare/workerd), on testcase 007 with
# --no-stereo and the 2D poses replayed.
#
# Usage: tools/cloud/tests/e2e_wrangler.sh [--tests T2,T2c,T2t] [--cli <multiview-cli>]
#                                          [--port <n>] [--out <dir>] [--keep]
#
#   T2   [0,120), shard_frames 60, lease_seconds 30, MULTIPART_PART_BYTES
#        8 MiB: `mvcloud submit --control http://127.0.0.1:<port>` uploads the
#        scene through the Worker and creates the job; agent A
#        (--dispatch api) takes loc-0000 and is killed (SIGKILL) once the
#        shard is under way; agent B reaps A's orphaned session and runs on.
#        Asserts
#        (a) `ldata cat` of the winning envelopes, `ldata diff` against
#            ref-120.data: exit 0, every frame byte-identical
#        (b) the tracks job's track_results == R1's (lab_p excluded)
#        (c) the tracks job's score.json == `score` of R1
#        (d) every shard done; loc-0000's attempts are [expired, done] (the
#            Workflow expired A's lease and attempt 2 won), loc-0001's and the
#            tracks job's [done]; same params_md5; the reference's frame md5s
#        (e) exit status 0 of submit and of agent B
#        (f) no process of any shard's session survives (A's orphan included)
#        (g) the Workflow instance is complete; summary.json exists; no object
#            outside the winning attempts' prefixes; the envelopes (> 8 MiB)
#            went up in parts
#   T2c  cancel during a run (fake multiview-cli, seconds): wrangler_scenarios.py cancel
#   T2t  wall-clock timeout and retry (fake multiview-cli): wrangler_scenarios.py timeout
#
# Needs the mvcloud micromamba env (tools/cloud/environment.yml) with the
# Worker's packages (npm ci in tools/cloud/worker): exit 77 without them.
# Needs make_reference.sh --frames 120 in $MULTIVIEW_TESTCASE_DATA/cloud-golden/007.
# The port is 8787 when it is free, else the first free one from 18787.
# The agents take the GPU lock build/dev.sh uses and run each shard at nice 15
# under a 24G cap. Outputs go to <out> (default a new directory under
# ${TMPDIR:-/tmp}); Miniflare's state, envelopes and caches are deleted at the
# end unless --keep or a failure. Exit status 0 if every assertion holds.

set -euo pipefail

THIS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$THIS_DIR/../../.." && pwd)"
WORKER_D="$ROOT/tools/cloud/worker"
DATA_D="${MULTIVIEW_TESTCASE_DATA:-$HOME/multiview-data}"
REPLAY_D="${MULTIVIEW_POSE_REPLAY:-$HOME/multiview-data/pose-replay}"
TC=007_site03_sparse_tracks
MEM="${MULTIVIEW_CLOUD_MEM:-24G}"
PART_BYTES=$((8 << 20))

CLI="$ROOT/multiview/multiview_cpp/build/cmake-release/multiview-cli"
TESTS="T2,T2c,T2t"
PORT=""
OUT=""
KEEP=0

while [ "$#" -gt 0 ] ; do
    case "$1" in
        --tests) shift ; TESTS="$1" ;;
        --cli)   shift ; CLI="$1" ;;
        --port)  shift ; PORT="$1" ;;
        --out)   shift ; OUT="$1" ;;
        --keep)  KEEP=1 ;;
        -h|--help) sed -n '2,/^$/p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//' ; exit 0 ;;
        *) echo "e2e_wrangler: unknown argument '$1'" ; exit 1 ;;
    esac
    shift
done

# The toolchain: Node 22 and wrangler 4 in the mvcloud env
MAMBA=(micromamba run -n mvcloud)
if ! "${MAMBA[@]}" node --version > /dev/null 2>&1 || [ ! -x "$WORKER_D/node_modules/.bin/wrangler" ] ; then
    echo "e2e_wrangler: SKIP: no mvcloud env or no Worker packages; run"
    echo "    micromamba create -y -f tools/cloud/environment.yml"
    echo "    micromamba run -n mvcloud npm --prefix tools/cloud/worker ci"
    exit 77
fi
export WRANGLER_SEND_METRICS=false

GOLD="$DATA_D/cloud-golden/007"
MANIFEST="$DATA_D/testcases/$TC/manifest.json"
GT="$DATA_D/testcases/$TC/ground-truth.json"
REPLAY="$REPLAY_D/$TC"
if [[ ",$TESTS," == *",T2,"* ]] ; then
    for f in "$CLI" "$MANIFEST" "$GT" "$GOLD/params-replay.json" "$REPLAY/pose-2d_000000_00.json" \
             "$GOLD/ref-120.data" "$GOLD/n120/R1/pipeline-output.json" ; do
        [ -e "$f" ] || { echo "e2e_wrangler: missing $f (make_reference.sh --frames 120)" ; exit 1 ; }
    done
    CLI="$(cd "$(dirname "$CLI")" && pwd)/$(basename "$CLI")"
fi
[ -n "$OUT" ] || OUT="$(mktemp -d "${TMPDIR:-/tmp}/e2e-wrangler.XXXXXX")"
mkdir -p "$OUT"

port_free() { ! (exec 3<> "/dev/tcp/127.0.0.1/$1") 2> /dev/null ; }
if [ -z "$PORT" ] ; then
    PORT=8787
    while ! port_free "$PORT" || ! port_free "$((PORT + 1))" ; do
        [ "$PORT" = "8787" ] && PORT=18787 || PORT=$((PORT + 2))
    done
fi
BASE="http://127.0.0.1:$PORT"
export MVCLOUD_ADMIN_TOKEN="admin-$RANDOM$RANDOM"
export MVCLOUD_WORKER_TOKEN="worker-$RANDOM$RANDOM"
export PYTHONPATH="$ROOT/tools/cloud/agent:$THIS_DIR${PYTHONPATH:+:$PYTHONPATH}"
CHECK=(python3 "$THIS_DIR/e2e_check.py")
FAILED=0
SIDS="$OUT/sessions.txt"
: > "$SIDS"
WRANGLER_SID=""
AGENT_PIDS=()

cleanup()
{
    local p
    for p in "${AGENT_PIDS[@]}" ; do kill -TERM "$p" 2> /dev/null || true ; done
    [ -n "$WRANGLER_SID" ] && pkill -TERM -s "$WRANGLER_SID" 2> /dev/null || true
}
trap cleanup EXIT

check() # check <name> <command...>: records a failure, does not stop
{
    local name="$1" ; shift
    if "$@" ; then echo "e2e_wrangler: PASS $name" ; else echo "e2e_wrangler: FAIL $name" ; FAILED=1 ; fi
}

no_survivors() # no process of any recorded session is alive
{
    local sid alive=0
    for sid in $(sort -u "$SIDS") ; do
        if pgrep -s "$sid" > /dev/null ; then echo "   session $sid: $(pgrep -s "$sid" | tr '\n' ' ')" ; alive=1 ; fi
    done
    return "$alive"
}

sessions() # sessions <agent log>: the shard sessions it started, into $SIDS
{
    sed -n 's/.*started session \([0-9]*\):.*/\1/p' "$1" >> "$SIDS"
}

# ---------------------------------------------------------------- the Worker
echo "e2e_wrangler: wrangler dev --env local on $BASE, state in $OUT/state"
t0="$(date +%s)"
( cd "$WORKER_D" && "${MAMBA[@]}" npx wrangler d1 migrations apply mvcloud --local --env local \
      --persist-to "$OUT/state" ) > "$OUT/migrations.log" 2>&1 \
    || { echo "e2e_wrangler: FAIL migrations (see $OUT/migrations.log)" ; exit 1 ; }
( cd "$WORKER_D" && exec setsid nice -n 15 "${MAMBA[@]}" npx wrangler dev --env local --ip 127.0.0.1 \
      --port "$PORT" --inspector-port "$((PORT + 1))" --persist-to "$OUT/state" \
      --var "MVCLOUD_ADMIN_TOKEN:$MVCLOUD_ADMIN_TOKEN" --var "MVCLOUD_WORKER_TOKEN:$MVCLOUD_WORKER_TOKEN" \
      --var "MULTIPART_PART_BYTES:$PART_BYTES" ) > "$OUT/wrangler.log" 2>&1 &
WRANGLER_SID=$!
for _ in $(seq 1 120) ; do
    curl -sf "$BASE/v1/health" > /dev/null && break
    sleep 1
done
curl -sf "$BASE/v1/health" > /dev/null || { echo "e2e_wrangler: FAIL the Worker did not start (see $OUT/wrangler.log)" ; exit 1 ; }
echo "e2e_wrangler: the Worker is up after $(( $(date +%s) - t0 )) s"

# ---------------------------------------------------------------------- T2
if [[ ",$TESTS," == *",T2,"* ]] ; then
    t0="$(date +%s)"
    d="$OUT/T2"
    mkdir -p "$d"
    python3 - "$d/job.json" "$MANIFEST" "$GOLD/params-replay.json" "$REPLAY" "$GT" <<'EOF'
import json, sys
fname, manifest, params, poses, gt = sys.argv[1:]
json.dump({"scene_id": "007_site03_sparse_tracks", "manifest": manifest, "params": params,
           "poses": poses, "gt": gt, "start_frame": 0, "n_frames": 120, "shard_frames": 60,
           "no_stereo": True, "pose_mode": "replay", "pose_classifier": "pose-svm-lite_v0",
           "heartbeat_s": 6, "lease_seconds": 30, "progress_timeout_s": 900, "retry_delay_s": 0,
           "wait_seconds": 5}, open(fname, "w"), indent=1)
EOF
    t1="$(date +%s)"
    job="$(python3 -m mvcloud submit --control "$BASE" --cli "$CLI" "$d/job.json" 2>> "$d/submit.log")" \
        || { echo "e2e_wrangler: FAIL T2 (e) submit (see $d/submit.log)" ; FAILED=1 ; job="" ; }
    echo "e2e_wrangler: T2: job $job (submit with the uploads took $(( $(date +%s) - t1 )) s)"
fi
if [[ ",$TESTS," == *",T2,"* ]] && [ -n "$job" ] ; then
    agent() # agent <log>: an agent in the background; its pid in $!
    {
        python3 -m mvcloud agent --control "$BASE" --dispatch api --cli "$CLI" --cache "$OUT/cache" \
            --work "$OUT/work" --mem "$MEM" --kill-grace 20 --poll 2 --worker-id "e2e-$(basename "$1" .log)" \
            2>> "$1" &
    }

    # Agent A runs loc-0000 until it is under way, then dies without a word
    agent "$d/agent-A.log"
    A=$! ; AGENT_PIDS+=("$A")
    check "T2 loc-0000 under way on agent A" "${CHECK[@]}" wait-progress "$BASE" "$job" loc-0000 1800
    kill -KILL "$A" 2> /dev/null || true
    wait "$A" 2> /dev/null || true
    sessions "$d/agent-A.log"
    echo "e2e_wrangler: T2: killed agent A (sessions $(tr '\n' ' ' < "$SIDS"))"

    # Agent B reaps A's orphaned session and runs the job to its end
    agent "$d/agent-B.log"
    B=$! ; AGENT_PIDS+=("$B")
    check "T2 job ended" "${CHECK[@]}" wait-job "$BASE" "$job" 3600
    kill -TERM "$B" 2> /dev/null || true
    ret=0 ; wait "$B" || ret=$?
    sessions "$d/agent-B.log"
    check "T2 (e) agent B exit 0" test "$ret" = 0
    check "T2 (f) agent B reaped agent A's orphaned session" grep -q "killed 1 orphaned session" "$d/agent-B.log"

    # (a)-(d) as in e2e_fs.sh, on the objects the Worker serves
    "$CLI" score --calc "$GOLD/n120/R1/pipeline-output.json" --gt "$GT" \
        --json "$OUT/r1-120-score.json" > "$OUT/r1-120-score.stdout"
    "$CLI" ldata info "$GOLD/ref-120.data" > "$OUT/ref-120.info.json"
    check "T2 (d) shards done, params_md5, frame md5s" \
        "${CHECK[@]}" job "$BASE" "$job" "$OUT/ref-120.info.json" "$d"
    check "T2 (d) loc-0000: the expired lease, then attempt 2" \
        "${CHECK[@]}" shard "$BASE" "$job" loc-0000 done expired,done "lease expired"
    check "T2 (d) loc-0001 ran once" "${CHECK[@]}" shard "$BASE" "$job" loc-0001 done done
    check "T2 (d) tracks ran once" "${CHECK[@]}" shard "$BASE" "$job" tracks done done
    if [ -f "$d/envs.txt" ] ; then
        mapfile -t envs < "$d/envs.txt"
        "$CLI" ldata cat -y -o "$d/merged.data" "${envs[@]}" > "$d/cat.stdout" 2>&1 || true
        check "T2 (a) ldata diff merged ref-120.data (bytes)" \
            "$CLI" ldata diff --json "$d/diff.json" "$d/merged.data" "$GOLD/ref-120.data"
        check "T2 (b) tracks job == R1 (lab_p excluded)" \
            python3 "$THIS_DIR/compare_tracks.py" --json "$d/tracks-vs-R1.json" \
                "$GOLD/n120/R1/pipeline-output.json" "$(cat "$d/tracks.txt")"
        check "T2 (c) score.json == score of R1" \
            "${CHECK[@]}" json-equal "$OUT/r1-120-score.json" "$(cat "$d/score.txt")"
        rm -f "$d/merged.data"
    fi
    check "T2 (g) Workflow complete" "${CHECK[@]}" workflow "$BASE" "$job" complete
    check "T2 (g) summary, no losing objects, envelopes in parts" \
        "${CHECK[@]}" objects "$BASE" "$job" "$PART_BYTES"
    check "T2 (f) no process of the shards' sessions survives" no_survivors
    echo "e2e_wrangler: T2 took $(( $(date +%s) - t0 )) s"
fi

# --------------------------------------------------------------- T2c, T2t
if [[ ",$TESTS," == *",T2c,"* ]] ; then
    check "T2c cancel during a run" python3 "$THIS_DIR/wrangler_scenarios.py" cancel "$BASE" "$OUT/T2c"
fi
if [[ ",$TESTS," == *",T2t,"* ]] ; then
    check "T2t wall-clock timeout, retried, then dead" \
        python3 "$THIS_DIR/wrangler_scenarios.py" timeout "$BASE" "$OUT/T2t"
fi

cleanup
WRANGLER_SID=""
AGENT_PIDS=()
if [ "$FAILED" = "0" ] && [ "$KEEP" = "0" ] ; then
    rm -rf "$OUT/state" "$OUT/cache" "$OUT/work" "$OUT"/T2/blobs
fi
if [ "$FAILED" = "0" ] ; then echo "e2e_wrangler: PASS (outputs in $OUT)" ; else echo "e2e_wrangler: FAIL (outputs in $OUT)" ; fi
exit "$FAILED"
