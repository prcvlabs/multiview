"""The filesystem backend: a job queue in a directory, for one machine and
the tests. Same protocol as the Worker API: leases with tokens and expiry
times, heartbeats, attempts, and done.json completion markers.

    <root>/.lock                            every state change holds this flock
    <root>/jobs/<job_id>/job.json           {id, state, spec, created_at, ...}
    <root>/jobs/<job_id>/shards/<id>.json   one shard row (id: loc-0000.., tracks)
    <root>/blobs/jobs/<job_id>/shards/<id>/a<attempt>/   outputs of an attempt

Shard states: blocked (the tracks job, until every loc shard is done),
pending, leased, failed (retryable; pending again after its delay), done,
dead (out of attempts, or fatal) and cancelled. A job is running, done,
failed or cancelled. Expiry, retries and promotion happen in sweep(), which
pull() runs first, so agents need no other process.
"""

import contextlib
import fcntl
import json
import os
import time
import uuid

from . import plan, storage
from .control import Conflict, HeartbeatResult

k_job_defaults = {
    "start_frame": 0,
    "n_frames": None,  # None: to the end of the scene
    "shard_frames": 300,
    "overlap_frames": 0,
    "still_warmup": 0,
    "no_stereo": True,
    "pose_mode": "replay",
    "pose_classifier": "pose-svm-lite_v0",
    "target_fps": 0.0,
    "extra_cli_args": [],  # a list for every shard, or {"loc": [...], "tracks": [...]}
    "max_attempts": 3,
    "lease_seconds": 1800,
    "heartbeat_s": 60,
    "max_wall_s": {"loc": None, "tracks": None},  # None: plan.wall_budget_s
    "progress_timeout_s": 900,
    "retry_delay_s": 30,  # the n-th retry waits min(600, retry_delay_s * 2^(n-1))
}


# ------------------------------------------------------------------ write_json_
def write_json_(fname, o):
    tmp = f"{fname}.tmp"
    with open(tmp, "w") as f:
        json.dump(o, f, indent=1, sort_keys=True)
    os.replace(tmp, fname)


# ------------------------------------------------------------------- read_json_
def read_json_(fname):
    with open(fname) as f:
        return json.load(f)


# ------------------------------------------------------------------ file_input_
def file_input_(name, path, cache):
    """An input entry for the local file `path`, with its size and md5."""
    path = os.path.abspath(path)
    if not os.path.isfile(path):
        raise plan.PlanError(f"missing input '{path}'")
    return {"name": name, "url": storage.file_url(path), "kind": "file",
            "bytes": os.path.getsize(path), "md5": storage.file_md5(path), "cache": cache}


# -------------------------------------------------------------------- FsControl
class FsControl:
    """Control backend in the directory `root`; `clock` is for the tests."""

    def __init__(self, root, clock=time.time):
        self.root = os.path.abspath(root)
        self.clock = clock
        os.makedirs(os.path.join(self.root, "jobs"), exist_ok=True)
        os.makedirs(os.path.join(self.root, "blobs"), exist_ok=True)

    # -------------------------------------------------------------------- paths
    def job_dir_(self, job_id):
        return os.path.join(self.root, "jobs", job_id)

    def shard_path_(self, job_id, shard_id):
        return os.path.join(self.job_dir_(job_id), "shards", f"{shard_id}.json")

    def blob_path(self, key):
        return os.path.join(self.root, "blobs", key)

    @contextlib.contextmanager
    def locked_(self):
        fd = os.open(os.path.join(self.root, ".lock"), os.O_RDWR | os.O_CREAT, 0o666)
        try:
            fcntl.flock(fd, fcntl.LOCK_EX)
            yield
        finally:
            fcntl.flock(fd, fcntl.LOCK_UN)
            os.close(fd)

    def jobs_(self):
        d = os.path.join(self.root, "jobs")
        jobs = [read_json_(os.path.join(d, j, "job.json")) for j in os.listdir(d)
                if os.path.isfile(os.path.join(d, j, "job.json"))]
        return sorted(jobs, key=lambda j: (j["created_at"], j["id"]))

    def shards_(self, job_id):
        d = os.path.join(self.job_dir_(job_id), "shards")
        rows = [read_json_(os.path.join(d, f)) for f in os.listdir(d) if f.endswith(".json")]
        return sorted(rows, key=lambda r: (r["kind"] != "loc", r["seq"]))

    def save_(self, row):
        row["updated_at"] = self.clock()
        if "kind" in row:
            write_json_(self.shard_path_(row["job_id"], row["id"]), row)
        else:
            write_json_(os.path.join(self.job_dir_(row["id"]), "job.json"), row)

    # ------------------------------------------------------------------- submit
    def submit(self, spec):
        """Plans the job described by `spec` (section 5.7 of the plan, with
        local paths for manifest, params, poses and gt, and the scene's
        total_frames, K and n_cameras filled in) and queues its loc shards.
        Returns the job id; raises plan.PlanError on an invalid job."""
        s = {**k_job_defaults, **spec}
        for key in ("scene_id", "manifest", "params", "total_frames", "K", "n_cameras"):
            if s.get(key) is None:
                raise plan.PlanError(f"the job needs '{key}'")
        if s["n_frames"] is None:
            s["n_frames"] = s["total_frames"] - s["start_frame"]
        if s["pose_mode"] not in ("replay", "hyperpose"):
            raise plan.PlanError(f"unknown pose_mode '{s['pose_mode']}'")
        if s["pose_mode"] == "replay" and not s.get("poses"):
            raise plan.PlanError("pose_mode 'replay' needs 'poses' (a directory or a tar)")
        shards = plan.plan_shards(s["total_frames"], s["start_frame"], s["n_frames"],
                                  s["shard_frames"], s["overlap_frames"], s["K"])
        s["inputs"] = self.scene_inputs_(s)

        now = self.clock()
        job_id = f"{s['scene_id']}-{time.strftime('%Y%m%dT%H%M%S', time.gmtime(now))}-{uuid.uuid4().hex[:6]}"
        os.makedirs(os.path.join(self.job_dir_(job_id), "shards"))
        job = {"id": job_id, "state": "running", "spec": s, "created_at": now, "error": None}

        def row(kind, seq, start, n, keep_start, keep_n, state):
            budget = s["max_wall_s"].get(kind) or plan.wall_budget_s(kind, n, s["pose_mode"])
            return {"id": "tracks" if kind == "tracks" else f"loc-{seq:04d}", "job_id": job_id,
                    "kind": kind, "seq": seq, "start_frame": start, "n_frames": n,
                    "keep_start": keep_start, "keep_n": keep_n, "max_wall_s": budget,
                    "state": state, "attempt": 0, "max_attempts": s["max_attempts"],
                    "worker_id": None, "lease_token": None, "lease_started_at": None,
                    "lease_expires_at": None, "last_heartbeat_at": None, "progress": None,
                    "output_prefix": None, "done": None, "params_md5": None, "error": None,
                    "not_before": 0.0, "attempts": [], "created_at": now}

        with self.locked_():
            for x in shards:
                self.save_(row("loc", x.idx, x.start, x.n, x.keep_start, x.keep_n, "pending"))
            self.save_(row("tracks", 0, s["start_frame"], s["n_frames"], s["start_frame"],
                           s["n_frames"], "blocked"))
            self.save_(job)
        return job_id

    def scene_inputs_(self, s):
        """The scene's input entries: manifest, the videos it names, params,
        poses and ground truth."""
        manifest = os.path.abspath(s["manifest"])
        out = [file_input_("manifest.json", manifest, "scene")]
        for v in read_json_(manifest).get("videos", []):
            out.append(file_input_(v["file"], os.path.join(os.path.dirname(manifest), v["file"]),
                                   "scene"))
        out.append(file_input_("params-in.json", s["params"], "scene"))
        if s.get("poses"):
            poses = os.path.abspath(s["poses"])
            if os.path.isdir(poses):
                out.append({"name": "poses", "url": storage.file_url(poses), "kind": "dir",
                            "bytes": None, "md5": None, "cache": None})
            else:
                out.append({**file_input_("poses.tar", poses, "scene"), "kind": "tar"})
        if s.get("gt"):
            out.append(file_input_("ground-truth.json", s["gt"], "scene"))
        return out

    # -------------------------------------------------------------------- sweep
    def sweep(self, verify=False):
        """Expires leases, schedules retries, promotes the tracks job,
        re-queues done shards whose done.json or outputs are gone (or, with
        `verify`, whose md5s differ), and settles job states. Returns a list
        of what changed."""
        with self.locked_():
            return self.sweep_locked_(verify)

    def sweep_locked_(self, verify):
        now = self.clock()
        events = []
        for job in self.jobs_():
            if job["state"] in ("failed", "cancelled"):
                continue
            rows = self.shards_(job["id"])
            for r in rows:
                if r["state"] == "leased" and r["lease_expires_at"] < now:
                    self.end_attempt_(r, "expired", "lease expired")
                    r["state"] = "pending" if r["attempt"] < r["max_attempts"] else "dead"
                    r["error"] = "lease expired"
                    events.append(f"{job['id']}/{r['id']}: lease expired -> {r['state']}")
                    self.save_(r)
                elif r["state"] == "failed" and r["not_before"] <= now:
                    r["state"] = "pending"
                    self.save_(r)
                elif r["state"] == "done":
                    problem = self.check_done_(r, verify)
                    if problem:
                        r["state"] = "pending"
                        r["error"] = f"invalidated: {problem}"
                        r["output_prefix"] = None
                        r["done"] = None
                        events.append(f"{job['id']}/{r['id']}: {r['error']}")
                        self.save_(r)
            loc = [r for r in rows if r["kind"] == "loc"]
            tracks = next(r for r in rows if r["kind"] == "tracks")
            all_loc_done = all(r["state"] == "done" for r in loc)
            if not all_loc_done and tracks["state"] in ("pending", "failed", "leased", "done"):
                if tracks["state"] == "leased":
                    self.end_attempt_(tracks, "invalidated", "a loc shard was re-queued")
                tracks.update(state="blocked", lease_token=None, output_prefix=None, done=None)
                events.append(f"{job['id']}/tracks: blocked again")
                self.save_(tracks)
            elif all_loc_done and tracks["state"] == "blocked":
                tracks["state"] = "pending"
                events.append(f"{job['id']}/tracks: pending")
                self.save_(tracks)
            state = job["state"]
            if any(r["state"] == "dead" for r in rows):
                state = "failed"
                job["error"] = next(f"{r['id']}: {r['error']}" for r in rows if r["state"] == "dead")
            elif all(r["state"] == "done" for r in rows):
                state = "done"
            else:
                state = "running"
            if state != job["state"]:
                job["state"] = state
                events.append(f"{job['id']}: {state}")
                self.save_(job)
                if state == "done":
                    self.write_summary_(job, rows)
        return events

    def check_done_(self, r, verify):
        """Why the done shard `r` is not done any more, or None."""
        prefix = self.blob_path(r["output_prefix"])
        if not os.path.isfile(os.path.join(prefix, "done.json")):
            return "done.json is missing"
        for o in r["done"]["outputs"]:
            p = os.path.join(prefix, o["name"])
            if not os.path.isfile(p) or os.path.getsize(p) != o["bytes"]:
                return f"{o['name']} is missing or has the wrong size"
            if verify and storage.file_md5(p) != o["md5"]:
                return f"{o['name']} has the wrong md5"
        return None

    def write_summary_(self, job, rows):
        tracks = next(r for r in rows if r["kind"] == "tracks")
        summary = {"job_id": job["id"], "scene_id": job["spec"]["scene_id"],
                   "shards": [{"id": r["id"], "attempt": r["attempt"],
                               "output_prefix": r["output_prefix"]} for r in rows],
                   "score": tracks["done"].get("score"), "n_tracks": tracks["done"].get("n_tracks")}
        key = f"jobs/{job['id']}/summary.json"
        os.makedirs(os.path.dirname(self.blob_path(key)), exist_ok=True)
        write_json_(self.blob_path(key), summary)

    def end_attempt_(self, r, outcome, error, wall_s=None):
        for a in r["attempts"]:
            if a["attempt"] == r["attempt"] and a["finished_at"] is None:
                a.update(finished_at=self.clock(), outcome=outcome, error=error, wall_s=wall_s)

    # --------------------------------------------------------------------- pull
    def pull(self, worker):
        """Leases the oldest job's next pending shard (loc shards first)."""
        with self.locked_():
            self.sweep_locked_(False)
            now = self.clock()
            for job in self.jobs_():
                if job["state"] != "running":
                    continue
                for r in self.shards_(job["id"]):
                    if r["state"] == "pending" and r["not_before"] <= now:
                        return self.lease_(job, r, worker, now)
        return None

    def lease_(self, job, r, worker, now):
        s = job["spec"]
        r["attempt"] += 1
        r.update(state="leased", worker_id=worker["worker_id"], lease_token=uuid.uuid4().hex,
                 lease_started_at=now, lease_expires_at=now + s["lease_seconds"],
                 last_heartbeat_at=None, progress=None, error=None)
        r["attempts"].append({"attempt": r["attempt"], "worker_id": worker["worker_id"],
                              "started_at": now, "finished_at": None, "outcome": None,
                              "error": None, "wall_s": None})
        self.save_(r)

        kind = r["kind"]
        extra = s["extra_cli_args"]
        prefix = f"jobs/{job['id']}/shards/{r['id']}/a{r['attempt']}"
        cli = {"start_frame": r["start_frame"], "n_frames": r["n_frames"], "K": s["K"],
               "no_stereo": s["no_stereo"], "no_still": kind == "loc" and s["no_stereo"],
               "pose_mode": s["pose_mode"],
               "pose_classifier": s["pose_classifier"] if kind == "tracks" else None,
               "still_warmup": s["still_warmup"] if kind == "loc" and not s["no_stereo"] else 0,
               "require_aligned": kind == "tracks", "target_fps": s["target_fps"],
               "extra_args": list(extra.get(kind, []) if isinstance(extra, dict) else extra)}
        if kind == "loc":
            inputs = [i for i in s["inputs"] if i["name"] != "ground-truth.json"]
            names = ["localization.data", "params.json", "run-object.json", "stdout.log",
                     "poses.tar", "done.json"]
            required = {"localization.data", "params.json", "stdout.log", "done.json"}
            keep = []
            params_md5 = None
        else:
            inputs = [i for i in s["inputs"] if i["kind"] == "file"]
            keep = []
            params_md5 = None
            for x in self.shards_(job["id"]):
                if x["kind"] != "loc":
                    continue
                name = f"{x['id']}.data"
                env = next(o for o in x["done"]["outputs"] if o["name"] == "localization.data")
                inputs.append({"name": name, "kind": "file", "bytes": env["bytes"],
                               "md5": env["md5"], "cache": "job",
                               "url": storage.file_url(self.blob_path(
                                   f"{x['output_prefix']}/localization.data"))})
                keep.append({"name": name, "start": x["keep_start"], "n": x["keep_n"],
                             "computed_start": x["start_frame"], "computed_n": x["n_frames"]})
                params_md5 = params_md5 or x["params_md5"]
            names = ["pipeline-output.json", "params.json", "score.json", "overlap.json",
                     "stdout.log", "done.json"]
            required = {"pipeline-output.json", "params.json", "stdout.log", "done.json"}
            if s.get("gt"):
                required.add("score.json")
        return {"shard_id": r["id"], "job_id": job["id"], "attempt": r["attempt"],
                "lease_token": r["lease_token"], "lease_expires_at": r["lease_expires_at"],
                "lease_seconds": s["lease_seconds"], "heartbeat_s": s["heartbeat_s"],
                "max_wall_s": r["max_wall_s"], "progress_timeout_s": s["progress_timeout_s"],
                "kind": kind, "scene_id": s["scene_id"], "total_frames": s["total_frames"],
                "n_cameras": s["n_cameras"], "cli": cli, "inputs": inputs,
                "outputs": [{"name": n, "url": storage.file_url(self.blob_path(f"{prefix}/{n}")),
                             "required": n in required} for n in names],
                "keep": keep, "params_md5": params_md5}

    # ---------------------------------------------------------------- heartbeat
    def current_(self, lease):
        """The shard row of `lease`, and whether the lease is still its own."""
        r = read_json_(self.shard_path_(lease["job_id"], lease["shard_id"]))
        return r, r["state"] == "leased" and r["lease_token"] == lease["lease_token"]

    def heartbeat(self, lease, progress):
        with self.locked_():
            r, current = self.current_(lease)
            now = self.clock()
            if not current or now > r["lease_started_at"] + 1.5 * r["max_wall_s"]:
                return HeartbeatResult.LOST
            job = read_json_(os.path.join(self.job_dir_(r["job_id"]), "job.json"))
            r.update(lease_expires_at=now + job["spec"]["lease_seconds"], last_heartbeat_at=now,
                     progress=progress)
            self.save_(r)
            return HeartbeatResult.OK

    # ----------------------------------------------------------------- complete
    def complete(self, lease, done):
        with self.locked_():
            r, current = self.current_(lease)
            if r["state"] == "cancelled":
                return
            if r["state"] == "done" and r["done"] and r["done"].get("lease_token") == lease["lease_token"]:
                return
            if not current:
                raise Conflict(f"{lease['job_id']}/{lease['shard_id']}: the lease of attempt "
                               f"{lease['attempt']} is not current (state {r['state']})")
            prefix = f"jobs/{r['job_id']}/shards/{r['id']}/a{r['attempt']}"
            names = {o["name"] for o in done["outputs"]}
            for o in lease["outputs"]:
                if o["required"] and o["name"] != "done.json" and o["name"] not in names:
                    raise Conflict(f"{r['id']}: required output {o['name']} is not in done.json")
            for o in done["outputs"] + [{"name": "done.json", "bytes": None}]:
                size = storage.head(self.blob_path(f"{prefix}/{o['name']}"))
                if size is None or (o["bytes"] is not None and size != o["bytes"]):
                    raise Conflict(f"{r['id']}: output {o['name']} is missing or has the wrong size")
            r.update(state="done", output_prefix=prefix, params_md5=done.get("params_md5"),
                     done={**done, "lease_token": lease["lease_token"]}, lease_token=None, error=None)
            self.end_attempt_(r, "done", None, done.get("wall_s"))
            if r["kind"] == "loc":
                others = {x["params_md5"] for x in self.shards_(r["job_id"])
                          if x["kind"] == "loc" and x["state"] == "done" and x["id"] != r["id"]}
                if others and others != {r["params_md5"]}:
                    r.update(state="dead", error=f"params_md5 {r['params_md5']} differs from the "
                             f"other loc shards' {sorted(others)}")
                    r["attempts"][-1]["outcome"] = "fatal"
            self.save_(r)
            self.sweep_locked_(False)

    # --------------------------------------------------------------------- fail
    def fail(self, lease, error, retryable, outcome=None):
        """`outcome` (retry, timeout, fatal, stopped) is recorded in the
        attempt; it defaults to retry or fatal."""
        with self.locked_():
            r, current = self.current_(lease)
            if not current:
                return
            outcome = outcome or ("retry" if retryable else "fatal")
            self.end_attempt_(r, outcome, error)
            r["error"] = error
            r["lease_token"] = None
            if retryable and r["attempt"] < r["max_attempts"]:
                job = read_json_(os.path.join(self.job_dir_(r["job_id"]), "job.json"))
                delay = min(600.0, job["spec"]["retry_delay_s"] * 2 ** (r["attempt"] - 1))
                r.update(state="failed", not_before=self.clock() + delay)
            else:
                r["state"] = "dead"
            self.save_(r)
            self.sweep_locked_(False)

    # ------------------------------------------------------------------- cancel
    def cancel(self, job_id):
        """Cancels the job: its unfinished shards become cancelled, so the
        next heartbeat of a running one returns LOST."""
        with self.locked_():
            for r in self.shards_(job_id):
                if r["state"] in ("blocked", "pending", "leased", "failed"):
                    if r["state"] == "leased":
                        self.end_attempt_(r, "cancelled", "job cancelled")
                    r.update(state="cancelled", lease_token=None)
                    self.save_(r)
            job = read_json_(os.path.join(self.job_dir_(job_id), "job.json"))
            if job["state"] == "running":
                job["state"] = "cancelled"
                self.save_(job)

    # ------------------------------------------------------------------- status
    def status(self, job_id=None):
        """The job (or every job) with its shard rows."""
        with self.locked_():
            jobs = [j for j in self.jobs_() if job_id is None or j["id"] == job_id]
            if job_id is not None and not jobs:
                raise KeyError(f"no job '{job_id}'")
            return [{**j, "shards": self.shards_(j["id"])} for j in jobs]

