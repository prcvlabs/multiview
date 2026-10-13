#!/usr/bin/env python3
"""Assertions of e2e_fs.sh on the state of an fs:// control root.

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

Exit status 0 when every assertion holds, 1 otherwise.
"""

import json
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "agent"))

from mvcloud.fsbackend import FsControl  # noqa: E402


def report(ok, what):
    print(f"   {'ok  ' if ok else 'FAIL'} {what}")
    return ok


def load_job(root, job_id):
    return FsControl(root).status(job_id)[0]


def check_job(root, job_id, ref_info, out_dir, fresh):
    control = FsControl(root)
    job = load_job(root, job_id)
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
            with open(control.blob_path(f"{winner}/done.json")) as f:
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
        f.write("".join(control.blob_path(f"{r['output_prefix']}/localization.data") + "\n"
                        for r in loc))
    with open(os.path.join(out_dir, "tracks.txt"), "w") as f:
        f.write(control.blob_path(f"{tracks['output_prefix']}/pipeline-output.json") + "\n")
    with open(os.path.join(out_dir, "score.txt"), "w") as f:
        f.write(control.blob_path(f"{tracks['output_prefix']}/score.json") + "\n")
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


def main(argv):
    cmd = argv[0]
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
