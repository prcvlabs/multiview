#!/usr/bin/env python3
"""Assertions of e2e_fs.sh and e2e_wrangler.sh on the state of a control
plane: ROOT is an fs:// root directory or the Worker's URL (admin token in
$MVCLOUD_ADMIN_TOKEN).

  e2e_check.py job    ROOT JOB_ID REF_INFO OUT_DIR [--fresh]
      The job is done; every shard's winning attempt is its last and is done
      (--fresh: and its only one); every done.json has exit code 0 and the
      same params_md5; the loc shards' frame md5s, in order, are the ones of
      REF_INFO (`ldata info` of the reference envelope). Writes the winning
      envelopes (envs.txt), pipeline-output.json (tracks.txt) and score.json
      (score.txt) paths to OUT_DIR.
  e2e_check.py shard  ROOT JOB_ID SHARD STATE OUTCOMES [ERROR_SUBSTRING]
      The shard's state, its attempts' outcomes (comma-separated) and error.
  e2e_check.py job-state ROOT JOB_ID STATE
  e2e_check.py prefix ROOT JOB_ID SHARD          the winning attempt's blob directory
  e2e_check.py json-equal A B                    two json files hold the same object
  e2e_check.py snapshot ROOT OUT                 every shard row, for `unchanged`
  e2e_check.py unchanged ROOT SNAPSHOT

The Worker only:

  e2e_check.py wait-progress URL JOB_ID SHARD TIMEOUT_S
      Waits until the shard is leased and its agent has reported a progress
      line (the run is under way); prints the lease's worker.
  e2e_check.py wait-job URL JOB_ID TIMEOUT_S     until the job is done, failed or cancelled
  e2e_check.py workflow URL JOB_ID STATUS        the Workflow instance's status
  e2e_check.py objects URL JOB_ID PART_BYTES
      summary.json exists; every object under the job's shards/ is under a
      winning attempt's prefix (finalize deleted the others); every envelope
      larger than PART_BYTES was uploaded in parts (its etag ends in -N).

Exit status 0 when every assertion holds, 1 otherwise.
"""

import json
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "agent"))

from mvcloud import storage  # noqa: E402
from mvcloud.fsbackend import FsControl  # noqa: E402
from mvcloud.httpcontrol import HttpControl  # noqa: E402


def report(ok, what):
    print(f"   {'ok  ' if ok else 'FAIL'} {what}")
    return ok


# --------------------------------------------------------------------- FsSource
class FsSource:
    """Jobs and blobs of an fs:// control root."""

    def __init__(self, root):
        self.control = FsControl(root)

    def blob(self, key, out_dir):
        return self.control.blob_path(key)


# ------------------------------------------------------------------- HttpSource
class HttpSource:
    """Jobs and blobs of the Worker at `base` (admin token from
    $MVCLOUD_ADMIN_TOKEN); blobs are downloaded under <out_dir>/blobs."""

    def __init__(self, base):
        self.base = base.rstrip("/")
        self.control = HttpControl(base, os.environ.get("MVCLOUD_ADMIN_TOKEN"))

    def blob(self, key, out_dir):
        dest = os.path.join(out_dir, "blobs", key)
        if not os.path.isfile(dest):
            storage.fetch(f"{self.base}/v1/blobs/{key}", dest, headers=self.control.auth_())
        return dest


def open_source(root):
    """ROOT is an fs:// root directory or the Worker's http(s):// URL."""
    return HttpSource(root) if root.startswith(("http://", "https://")) else FsSource(root)


def load_job(root, job_id):
    return open_source(root).control.status(job_id)[0]


def check_job(root, job_id, ref_info, out_dir, fresh):
    source = open_source(root)
    job = source.control.status(job_id)[0]
    ok = report(job["state"] == "done", f"job {job_id} is {job['state']}")
    dones = {}
    for r in job["shards"]:
        outcomes = [a["outcome"] for a in r["attempts"]]
        winner = r["output_prefix"] or ""
        good = (r["state"] == "done" and outcomes[-1:] == ["done"]
                and winner.endswith(f"/a{r['attempts'][-1]['attempt']}"))
        if fresh:
            good = good and outcomes == ["done"]
        ok &= report(good, f"{r['id']}: state {r['state']}, attempts {outcomes}, prefix {winner}")
        if r["state"] == "done":
            with open(source.blob(f"{winner}/done.json", out_dir)) as f:
                dones[r["id"]] = done = json.load(f)
            ok &= report(done["exit_code"] == 0, f"{r['id']}: exit code {done['exit_code']}")
    if len(dones) != len(job["shards"]):
        return False

    md5s = {d["params_md5"] for d in dones.values()}
    ok &= report(len(md5s) == 1, f"params_md5 equal across the shards: {sorted(md5s)}")
    loc = [r for r in job["shards"] if r["kind"] == "loc"]
    frame_md5 = [m for r in loc for m in dones[r["id"]]["ldata_info"]["frame_md5"]]
    with open(ref_info) as f:
        ref = json.load(f)
    ok &= report(frame_md5 == ref["frame_md5"],
                 f"done.json frame md5s ({len(frame_md5)}) == the reference's ({len(ref['frame_md5'])})")

    tracks = next(r for r in job["shards"] if r["kind"] == "tracks")
    os.makedirs(out_dir, exist_ok=True)
    with open(os.path.join(out_dir, "envs.txt"), "w") as f:
        f.write("".join(source.blob(f"{r['output_prefix']}/localization.data", out_dir) + "\n"
                        for r in loc))
    with open(os.path.join(out_dir, "tracks.txt"), "w") as f:
        f.write(source.blob(f"{tracks['output_prefix']}/pipeline-output.json", out_dir) + "\n")
    with open(os.path.join(out_dir, "score.txt"), "w") as f:
        f.write(source.blob(f"{tracks['output_prefix']}/score.json", out_dir) + "\n")
    with open(os.path.join(out_dir, "walls.txt"), "w") as f:
        for r in job["shards"]:
            for a in r["attempts"]:
                f.write(f"{r['id']} a{a['attempt']} [{r['start_frame']}, "
                        f"{r['start_frame'] + r['n_frames']}) {a['outcome']} "
                        f"{a['wall_s'] if a['wall_s'] is not None else '-'}\n")
    return ok


def check_shard(root, job_id, shard_id, state, outcomes, error=None):
    r = next(x for x in load_job(root, job_id)["shards"] if x["id"] == shard_id)
    got = [a["outcome"] for a in r["attempts"]]
    ok = report(r["state"] == state, f"{shard_id}: state {r['state']} (expected {state})")
    ok &= report(got == outcomes.split(","), f"{shard_id}: attempts {got} (expected {outcomes})")
    if error is not None:
        errors = [a["error"] or "" for a in r["attempts"]] + [r["error"] or ""]
        ok &= report(any(error in e for e in errors), f"{shard_id}: an error contains '{error}': {errors}")
    return ok


def snapshot(root):
    return {j["id"]: [(r["id"], r["state"], r["attempt"], r["output_prefix"]) for r in j["shards"]]
            for j in FsControl(root).status()}


def wait_for(what, timeout_s, f):
    """Polls f() every 2 s until it returns something true; returns it, or
    None after timeout_s."""
    t_end = time.time() + timeout_s
    while time.time() < t_end:
        try:
            x = f()
        except Exception as e:  # the Worker restarting, say
            print(f"   ({what}: {e})")
            x = None
        if x:
            return x
        time.sleep(2)
    return None


def wait_progress(base, job_id, shard_id, timeout_s):
    def under_way():
        r = next(x for x in load_job(base, job_id)["shards"] if x["id"] == shard_id)
        return r["worker_id"] if r["state"] == "leased" and (r.get("progress") or {}).get("line") else None

    worker = wait_for(f"{shard_id} under way", timeout_s, under_way)
    print(worker or "")
    return report(worker is not None, f"{shard_id} is leased and under way")


def wait_job(base, job_id, timeout_s):
    state = wait_for("job", timeout_s, lambda: (lambda st: st if st in ("done", "failed", "cancelled") else None)(
        load_job(base, job_id)["state"]))
    return report(state is not None, f"job {job_id} ended: {state}")


def check_objects(base, job_id, part_bytes):
    source = HttpSource(base)
    job = load_job(base, job_id)
    status, listing = source.control.call_("GET", f"/v1/blobs?prefix=jobs/{job_id}/")
    objects = listing["objects"] if status == 200 else []
    ok = report(status == 200, f"listed {len(objects)} objects of the job")
    ok &= report(any(o["key"] == f"jobs/{job_id}/summary.json" for o in objects), "summary.json exists")
    winners = [r["output_prefix"] for r in job["shards"] if r["output_prefix"]]
    losers = [o["key"] for o in objects if o["key"].startswith(f"jobs/{job_id}/shards/")
              and not any(o["key"].startswith(f"{w}/") for w in winners)]
    ok &= report(not losers, f"no object outside the winning attempts' prefixes: {losers}")
    envs = [o for o in objects if o["key"].endswith("/localization.data")]
    for o in envs:
        if o["size"] > part_bytes:
            ok &= report("-" in o["etag"], f"{o['key']} ({o['size']} B) went up in parts: etag {o['etag']}")
    return ok


def main(argv):
    cmd = argv[0]
    if cmd == "wait-progress":
        return wait_progress(argv[1], argv[2], argv[3], float(argv[4]))
    if cmd == "wait-job":
        return wait_job(argv[1], argv[2], float(argv[3]))
    if cmd == "workflow":
        wf = load_job(argv[1], argv[2]).get("workflow") or {}
        return report(wf.get("status") == argv[3], f"Workflow {argv[2]}: {wf.get('status')} "
                      f"(expected {argv[3]}), output {wf.get('output')}, error {wf.get('error')}")
    if cmd == "objects":
        return check_objects(argv[1], argv[2], int(argv[3]))
    if cmd == "job":
        return check_job(*argv[1:5], fresh="--fresh" in argv)
    if cmd == "shard":
        return check_shard(*argv[1:7])
    if cmd == "job-state":
        state = load_job(argv[1], argv[2])["state"]
        return report(state == argv[3], f"job {argv[2]} is {state} (expected {argv[3]})")
    if cmd == "prefix":
        r = next(x for x in load_job(argv[1], argv[2])["shards"] if x["id"] == argv[3])
        print(FsControl(argv[1]).blob_path(r["output_prefix"]))
        return True
    if cmd == "json-equal":
        with open(argv[1]) as f, open(argv[2]) as g:
            return report(json.load(f) == json.load(g), f"{argv[1]} == {argv[2]}")
    if cmd == "snapshot":
        with open(argv[2], "w") as f:
            json.dump(snapshot(argv[1]), f)
        return True
    if cmd == "unchanged":
        with open(argv[2]) as f:
            before = json.load(f)
        after = json.loads(json.dumps(snapshot(argv[1])))
        return report(before == after, "no shard changed")
    print(f"e2e_check: unknown command '{cmd}'", file=sys.stderr)
    return False


if __name__ == "__main__":
    sys.exit(0 if main(sys.argv[1:]) else 1)
