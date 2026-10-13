"""The GPU agent: pull a shard, fetch its inputs, run multiview-cli, upload
the outputs and done.json, complete. One shard at a time.

A heartbeat thread extends the lease from the claim until the outputs are
uploaded; a lost lease (or a cancelled job, or SIGTERM to the agent) kills
the child's session, and the attempt is reported as failed.
"""

import json
import logging
import os
import random
import shutil
import signal
import socket
import subprocess
import tarfile
import threading
import time

from . import params, procs, runner, storage
from .control import Conflict, HeartbeatResult

log = logging.getLogger("mvcloud")

k_default_gpu_lock = "/tmp/multiview-dev-gpu.lock"
k_retryable = ("retry", "timeout", "stopped")


class ShardError(Exception):
    """The attempt cannot succeed; `outcome` says whether to retry."""

    def __init__(self, outcome, message):
        super().__init__(message)
        self.outcome = outcome


# -------------------------------------------------------------------- Heartbeat
class Heartbeat(threading.Thread):
    """Extends the lease every heartbeat_s / 3 until finish(). A LOST answer
    sets `lost`; TRANSIENT answers are retried with backoff until the lease
    would have expired, which also counts as lost."""

    def __init__(self, control, lease, clock=time.time):
        super().__init__(name=f"heartbeat-{lease['shard_id']}", daemon=True)
        self.control = control
        self.lease = lease
        self.clock = clock
        self.lost = threading.Event()
        self.progress = {}
        self.n_sent = 0
        self.stop_ = threading.Event()

    def run(self):
        interval = max(0.5, self.lease["heartbeat_s"] / 3.0)
        expires = self.lease["lease_expires_at"]
        wait = interval
        failures = 0
        while not self.stop_.wait(wait):
            try:
                r = self.control.heartbeat(self.lease, dict(self.progress))
            except Exception as e:  # a backend error is transient
                log.warning(f"heartbeat: {e}")
                r = HeartbeatResult.TRANSIENT
            self.n_sent += 1
            if r == HeartbeatResult.OK:
                expires = self.clock() + self.lease["lease_seconds"]
                failures = 0
                wait = interval
            elif r == HeartbeatResult.LOST or self.clock() > expires:
                log.warning(f"{self.lease['job_id']}/{self.lease['shard_id']}: lease lost")
                self.lost.set()
                return
            else:
                failures += 1
                wait = min(60.0, 2.0 ** (failures - 1))

    def finish(self):
        self.stop_.set()
        self.join()


# ------------------------------------------------------------------------ Agent
class Agent:
    def __init__(self, control, *, cli_bin, cache_dir, work_dir, gpu_lock=k_default_gpu_lock,
                 mem=None, kill_grace_s=20.0, worker_id=None, poll_s=15.0, env=None,
                 multiview_cache=None, uploader=None, keep_work=False, clock=time.time):
        self.control = control
        self.cli_bin = os.path.abspath(cli_bin)
        self.cache_dir = os.path.abspath(cache_dir)
        self.work_dir = os.path.abspath(work_dir)
        self.gpu_lock = procs.GpuLock(gpu_lock)
        self.kill_grace_s = kill_grace_s
        self.poll_s = poll_s
        self.uploader = uploader or storage.put
        self.keep_work = keep_work
        self.clock = clock
        self.stop_reason = None
        self.wake_ = threading.Event()
        self.fault_used_ = False
        self.prefix, self.capped = procs.memory_cap_prefix(mem)
        self.env = {k: v for k, v in os.environ.items() if k != "MULTIVIEW_OPENPOSE_DISABLED"}
        self.env.update(env or {})
        if multiview_cache:
            self.env["MULTIVIEW_CACHE_DIR"] = os.path.abspath(multiview_cache)
        self.worker = {"worker_id": worker_id or default_worker_id(), "gpu": gpu_name_(),
                       "version": "mvcloud-agent-1"}

    # ---------------------------------------------------------------------- run
    def run(self, *, once=False, drain=False, install_signals=False):
        """Processes leases until stopped; `once`: at most one; `drain`:
        until there is no work. Returns the number of leases processed."""
        if install_signals:
            signal.signal(signal.SIGTERM, lambda *_: self.stop("agent terminated"))
            signal.signal(signal.SIGINT, lambda *_: self.stop("agent terminated"))
        os.makedirs(self.work_dir, exist_ok=True)
        n = procs.reap_orphans(self.work_dir, grace_s=self.kill_grace_s)
        if n > 0:
            log.warning(f"killed {n} orphaned session(s) under {self.work_dir}")
        n = 0
        while not self.stop_reason:
            lease = self.control.pull(self.worker)
            if lease is None:
                if once or drain:
                    log.info("no work")
                    break
                self.wake_.wait(self.poll_s * random.uniform(0.8, 1.2))
                continue
            self.run_lease(lease)
            n += 1
            if once:
                break
        return n

    def stop(self, reason):
        self.stop_reason = reason
        self.wake_.set()

    # ---------------------------------------------------------------- run_lease
    def run_lease(self, lease):
        """Runs one leased shard to its end and reports it. Returns the
        outcome: ok, retry, timeout, fatal, stopped or conflict."""
        name = f"{lease['job_id']}/{lease['shard_id']} a{lease['attempt']}"
        log.info(f"{name}: leased ({lease['kind']} [{lease['cli']['start_frame']}, "
                 f"{lease['cli']['start_frame'] + lease['cli']['n_frames']}))")
        workdir = os.path.join(self.work_dir, lease["job_id"], lease["shard_id"], f"a{lease['attempt']}")
        shutil.rmtree(workdir, ignore_errors=True)
        os.makedirs(os.path.join(workdir, "in"))
        hb = Heartbeat(self.control, lease, self.clock)

        def should_stop():
            if self.stop_reason:
                return self.stop_reason
            return "lease lost" if hb.lost.is_set() else None

        def on_progress(line):
            hb.progress = {**hb.progress, "line": line, "at": self.clock()}

        def on_lock():
            hb.progress = {"run_started_at": self.clock()}

        started_at = self.clock()
        hb.start()
        try:
            done = self.attempt_(lease, workdir, should_stop, on_progress, on_lock, started_at)
            outcome = "ok"
            error = ""
        except ShardError as e:
            outcome = e.outcome
            error = str(e)
            self.upload_log_(lease, workdir)
        except Exception as e:  # a bug in the agent: let another attempt try
            log.exception(f"{name}: agent error")
            outcome = "retry"
            error = f"agent error: {e}"
        hb.finish()
        if outcome == "ok" and hb.lost.is_set():
            outcome = "stopped"
            error = "lease lost"

        if outcome == "ok":
            try:
                self.control.complete(lease, done)
                log.info(f"{name}: done in {done['wall_s']:.1f} s")
            except Conflict as e:
                log.error(f"{name}: complete refused: {e}")
                outcome = "conflict"
        else:
            log.warning(f"{name}: {error} ({outcome})")
            self.control.fail(lease, error, outcome in k_retryable, outcome=outcome)
        if not self.keep_work:
            shutil.rmtree(workdir, ignore_errors=True)
        return outcome

    # ----------------------------------------------------------------- attempt_
    def attempt_(self, lease, workdir, should_stop, on_progress, on_lock, started_at):
        """Fetch, run, check and upload; returns done.json's object or
        raises ShardError."""
        replay_dir = self.prepare_inputs_(lease, workdir)
        self.write_params_(lease, workdir, replay_dir)
        if lease["kind"] == "loc" and lease["cli"]["pose_mode"] == "replay":
            check_replay_(lease, replay_dir)

        fault = None
        spec = os.environ.get("MVCLOUD_FAULT", "")
        if spec.startswith("kill-after:") and not self.fault_used_:
            self.fault_used_ = True
            fault = float(spec.split(":", 1)[1])
        r = runner.run_shard(lease, workdir, self.cli_bin, env=self.env, prefix=self.prefix,
                             gpu_lock=self.gpu_lock, kill_grace_s=self.kill_grace_s,
                             should_stop=should_stop, on_progress=on_progress, on_lock=on_lock,
                             fault_kill_after_s=fault)
        if r.sid is not None:
            log.info(f"{lease['job_id']}/{lease['shard_id']}: session {r.sid} ended: "
                     f"exit {r.exit_code}, {r.outcome}, {r.wall_s:.1f} s "
                     f"(after {r.lock_wait_s:.1f} s for the GPU lock)")
        if r.outcome != "ok":
            raise ShardError(r.outcome, r.error)

        done = {"shard_id": lease["shard_id"], "job_id": lease["job_id"],
                "attempt": lease["attempt"], "kind": lease["kind"], "exit_code": r.exit_code,
                "started_at": started_at, "wall_s": r.wall_s, "lock_wait_s": r.lock_wait_s,
                "frames": {"start": lease["cli"]["start_frame"], "n": lease["cli"]["n_frames"]},
                "host": {**self.worker, "host": socket.gethostname(), "cli": self.cli_bin,
                         "memory_capped": self.capped}}
        done["params_md5"] = params.params_file_md5(r.outputs["params.json"])
        if lease["kind"] == "loc":
            problem, info = runner.check_product(lease, workdir, self.cli_bin)
            if problem:
                raise ShardError("fatal", problem)
            done["ldata_info"] = info
            if lease["cli"]["pose_mode"] == "hyperpose":
                tar_poses_(workdir, r.outputs)
        else:
            if lease.get("params_md5") and done["params_md5"] != lease["params_md5"]:
                raise ShardError("fatal", f"params_md5 {done['params_md5']} differs from the loc "
                                          f"shards' {lease['params_md5']}")
            with open(r.outputs["pipeline-output.json"]) as f:
                done["n_tracks"] = len(json.load(f).get("track_results", {}).get("tracks", []))
            if "score.json" in r.outputs:
                with open(r.outputs["score.json"]) as f:
                    done["score"] = json.load(f)

        outputs = []
        for o in lease["outputs"]:
            if o["name"] == "done.json":
                continue
            path = r.outputs.get(o["name"])
            if path is None:
                if o["required"]:
                    raise ShardError("retry", f"missing output {o['name']}")
                continue
            self.upload_(o, path)
            outputs.append({"name": o["name"], "bytes": os.path.getsize(path),
                            "md5": storage.file_md5(path)})
            if should_stop():
                raise ShardError("stopped", should_stop())
        done["outputs"] = outputs
        done["finished_at"] = self.clock()
        done_path = os.path.join(workdir, "done.json")
        with open(done_path, "w") as f:
            json.dump(done, f, indent=1, sort_keys=True)
        self.upload_(next(o for o in lease["outputs"] if o["name"] == "done.json"), done_path)
        return done

    def upload_(self, output, path):
        try:
            if output.get("multipart"):
                storage.put_multipart(output["multipart"], path, headers=output.get("headers"))
            else:
                self.uploader(output["url"], path, headers=output.get("headers"))
        except storage.TransferError as e:
            raise ShardError("retry", f"upload of {output['name']}: {e}") from e

    def upload_log_(self, lease, workdir):
        """Best effort: the log of a failed attempt, for diagnosis."""
        path = os.path.join(workdir, "stdout.log")
        o = next((o for o in lease["outputs"] if o["name"] == "stdout.log"), None)
        if o is not None and os.path.isfile(path):
            try:
                self.uploader(o["url"], path, headers=o.get("headers"))
            except storage.TransferError as e:
                log.warning(f"could not upload the log: {e}")

    # ---------------------------------------------------------- prepare_inputs_
    def prepare_inputs_(self, lease, workdir):
        """Fetches every input (verified, cached by md5) and links it into
        in/. Returns the pose replay directory, if there is one."""
        replay_dir = None
        for i in lease["inputs"]:
            kind = i.get("kind", "file")
            try:
                if kind == "dir":
                    path = storage.local_path(i["url"])
                    if path is None or not os.path.isdir(path):
                        raise storage.TransferError(f"{i['url']}: not a local directory")
                    replay_dir = path
                    continue
                cache = os.path.join(self.cache_dir, i.get("cache") or "other")
                local = storage.fetch_cached(i["url"], cache, i["name"], size=i.get("bytes"),
                                             md5=i.get("md5"), headers=i.get("headers"))
            except storage.TransferError as e:
                raise ShardError("fatal", f"input {i['name']}: {e}") from e
            if kind == "tar":
                replay_dir = extract_once_(local)
            else:
                os.symlink(local, os.path.join(workdir, "in", i["name"]))
        return replay_dir

    def write_params_(self, lease, workdir, replay_dir):
        with open(os.path.join(workdir, "in", "params-in.json")) as f:
            p = json.load(f)
        mode = lease["cli"]["pose_mode"]
        if lease["kind"] == "loc" or mode == "hyperpose":
            try:
                p = params.patch_params(p, pose_mode=mode, replay_dir=replay_dir)
            except ValueError as e:
                raise ShardError("fatal", str(e)) from e
        with open(os.path.join(workdir, "params.json"), "w") as f:
            json.dump(p, f, indent=1)


# ---------------------------------------------------------------- check_replay_
def check_replay_(lease, replay_dir):
    """A replay run only WARNs about a missing pose file; here it is fatal."""
    c = lease["cli"]
    for t in range(c["start_frame"], c["start_frame"] + c["n_frames"]):
        for i in range(lease["n_cameras"]):
            fname = os.path.join(replay_dir, f"pose-2d_{t:06d}_{2 * i:02d}.json")
            if not os.path.isfile(fname):
                raise ShardError("fatal", f"pose replay: missing '{fname}'")


# ---------------------------------------------------------------- extract_once_
def extract_once_(tar_path):
    """The directory a (cached) tar is extracted to, extracting it once."""
    dest = f"{tar_path}.d"
    if not os.path.isdir(dest):
        tmp = f"{dest}.tmp{os.getpid()}"
        shutil.rmtree(tmp, ignore_errors=True)
        with tarfile.open(tar_path) as t:
            t.extractall(tmp, filter="data")
        os.replace(tmp, dest)
    return dest


# ------------------------------------------------------------------- tar_poses_
def tar_poses_(workdir, outputs):
    """The 2D poses a HyperPose loc shard wrote, as poses.tar."""
    out = os.path.join(workdir, "out")
    names = sorted(f for f in os.listdir(out) if f.startswith("pose-2d_") and f.endswith(".json"))
    if not names:
        return
    path = os.path.join(workdir, "poses.tar")
    with tarfile.open(path, "w") as t:
        for f in names:
            t.add(os.path.join(out, f), arcname=f)
    outputs["poses.tar"] = path


# -------------------------------------------------------------------- gpu_name_
def gpu_name_():
    try:
        r = subprocess.run(["nvidia-smi", "--query-gpu=name", "--format=csv,noheader"],
                           stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True, timeout=20)
        return r.stdout.splitlines()[0].strip() if r.returncode == 0 and r.stdout else ""
    except (OSError, subprocess.TimeoutExpired):
        return ""


# ------------------------------------------------------------ default_worker_id
def default_worker_id():
    gpu = os.environ.get("CUDA_VISIBLE_DEVICES", "0") or "cpu"
    return f"{socket.gethostname()}-gpu{gpu}"
