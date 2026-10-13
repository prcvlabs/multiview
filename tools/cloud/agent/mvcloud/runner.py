"""Runs one shard: multiview-cli as a child process under a watchdog, and the
classification of how it ended.

Outcomes: "ok"; "retry" (a crash that another attempt may not repeat: the
issue #13 aborts, a missing product); "timeout" (the watchdog killed it;
retried); "fatal" (deterministic: a bad command line, an unaligned range,
missing inputs; the job fails fast); "stopped" (the lease was lost, the job
cancelled or the agent terminated; retried).
"""

import json
import logging
import os
import re
import subprocess
import time
from dataclasses import dataclass, field

from . import procs

log = logging.getLogger("mvcloud")

k_fatal_patterns = (
    "Unused argument",
    "error on command-line",
    "aborting due to previous errors",
    "at least 2 frames required",
    "--require-aligned",
    "failed to save localization data",
    "pose replay: missing",
)
k_retry_patterns = ("FATAL", "Precondition failed", "Resource deadlock avoided", "kBAM")
k_progress_patterns = ("saving loc-data frames", "Track frames")
k_tail_bytes = 4000
k_ansi = re.compile(r"\x1b\[[0-9;]*m")  # the logger's colours


# -------------------------------------------------------------------- RunResult
@dataclass
class RunResult:
    exit_code: int | None  # None when the child was never started
    outcome: str  # ok, retry, timeout, fatal, stopped
    error: str = ""
    wall_s: float = 0.0
    log_tail: str = ""
    sid: int | None = None
    lock_wait_s: float = 0.0  # waiting for the GPU lock, not in wall_s
    outputs: dict = field(default_factory=dict)  # name -> local path


# ---------------------------------------------------------------- classify_exit
def classify_exit(code, log_tail, outputs_present, timed_out):
    """ok, retry, timeout or fatal for a child that exited with `code` (a
    negative code is the signal that killed it)."""
    if timed_out:
        return "timeout"
    if code == 0:
        return "ok" if outputs_present else "retry"
    if any(p in log_tail for p in k_fatal_patterns):
        return "fatal"
    return "retry"  # FATAL / Expects (issue #13), 134, 139, 137, unknown


# -------------------------------------------------------------------- read_tail
def read_tail(path, n=k_tail_bytes):
    try:
        with open(path, "rb") as f:
            f.seek(max(0, os.path.getsize(path) - n))
            return f.read().decode("utf-8", "replace")
    except OSError:
        return ""


# -------------------------------------------------------------------- run_child
def run_child(argv, *, cwd, env, log_path, pid_path, max_wall_s, progress_timeout_s,
              kill_grace_s, should_stop, on_progress=None, fault_kill_after_s=None):
    """Runs `argv` in a new session and watches it: the session is killed
    when it runs longer than `max_wall_s`, prints no progress line for
    `progress_timeout_s`, or should_stop() returns a reason. Returns
    (exit_code, error, wall_s, sid); error is "" when the child exited by
    itself, else it starts with "timeout:" or is the stop reason."""
    t0 = time.monotonic()
    offset = os.path.getsize(log_path) if os.path.exists(log_path) else 0
    proc = procs.start_session(argv, cwd=cwd, env=env, log_path=log_path, pid_path=pid_path)
    log.info(f"started session {proc.pid}: {' '.join(argv)}")
    last_progress = t0
    error = ""
    while True:
        try:
            proc.wait(timeout=0.5)
            break
        except subprocess.TimeoutExpired:
            pass
        now = time.monotonic()
        offset, lines = new_lines_(log_path, offset)
        for line in lines:
            if any(p in line for p in k_progress_patterns):
                last_progress = now
                if on_progress is not None:
                    on_progress(line.strip())
        reason = should_stop()
        if reason:
            error = reason
        elif now - t0 > max_wall_s:
            error = f"timeout: wall {now - t0:.0f} s > {max_wall_s} s"
        elif now - last_progress > progress_timeout_s:
            error = f"timeout: no progress for {now - last_progress:.0f} s"
        elif fault_kill_after_s is not None and now - t0 > fault_kill_after_s:
            error = f"fault injection: killed after {fault_kill_after_s} s"
        if error:
            log.warning(f"killing session {proc.pid}: {error}")
            procs.kill_session(proc.pid, grace_s=kill_grace_s, proc=proc)
            break
    if not error and procs.session_members(proc.pid):
        log.warning(f"session {proc.pid}: the leader exited, killing what it left")
        procs.kill_session(proc.pid, grace_s=kill_grace_s)
    if pid_path is not None and os.path.exists(pid_path):
        os.remove(pid_path)
    return proc.returncode, error, time.monotonic() - t0, proc.pid


# ------------------------------------------------------------------- new_lines_
def new_lines_(path, offset):
    try:
        with open(path, "rb") as f:
            f.seek(offset)
            data = f.read()
    except OSError:
        return offset, []
    end = data.rfind(b"\n") + 1
    return offset + end, data[:end].decode("utf-8", "replace").splitlines()


# ---------------------------------------------------------------- pipeline_argv
def pipeline_argv(lease, workdir, cli_bin):
    """The `multiview-cli pipeline` command line of the shard. Loc shards do
    not get --pose-classifier (only the tracker reads it); --no-still goes to
    no-stereo loc shards only (the tracks job's pose classifier reads the
    still)."""
    c = lease["cli"]
    argv = [cli_bin, "pipeline", "-m", os.path.join(workdir, "in", "manifest.json"),
            "-p", os.path.join(workdir, "params.json"), "-d", os.path.join(workdir, "out"), "-y"]
    if lease["kind"] == "loc":
        argv += ["--start", str(c["start_frame"]), "--n-frames", str(c["n_frames"]),
                 "--save-loc-data", "--no-tracks"]
        if c["no_stereo"]:
            argv += ["--no-stereo"]
        if c["no_still"]:
            argv += ["--no-still"]
        elif c.get("still_warmup", 0) > 0:
            argv += ["--still-warmup", str(c["still_warmup"])]
    else:
        if merged_input_(lease):
            argv += ["--load-loc-data", os.path.join(workdir, "merged.data")]
        else:
            for k in lease["keep"]:
                argv += ["--load-loc-data", os.path.join(workdir, "in", k["name"])]
        argv += ["--start", str(c["start_frame"]), "--n-frames", str(c["n_frames"]),
                 "--require-aligned"]
        if c["no_stereo"]:
            argv += ["--no-stereo"]
        if c.get("pose_classifier"):
            argv += ["--pose-classifier", c["pose_classifier"]]
    if c.get("target_fps", 0.0) > 0.0:
        argv += ["--target-fps", str(c["target_fps"])]
    return argv + ["--verbosity", "1"] + list(c.get("extra_args", []))


# ---------------------------------------------------------------- merged_input_
def merged_input_(lease):
    """True when the tracks job's envelopes overlap, so `ldata cat` must
    merge them by their keep ranges first (repeated --load-loc-data tiles
    them by their start frames)."""
    keep = lease.get("keep") or []
    return any(k["start"] != k.get("computed_start", k["start"]) for k in keep)


# --------------------------------------------------------------------- cat_argv
def cat_argv(lease, workdir, cli_bin):
    argv = [cli_bin, "ldata", "cat", "-y", "--report", os.path.join(workdir, "out", "overlap.json"),
            "-o", os.path.join(workdir, "merged.data")]
    for k in lease["keep"]:
        argv += ["--keep", f"{k['start']}:{k['n']}"]
    return argv + [os.path.join(workdir, "in", k["name"]) for k in lease["keep"]]


# ------------------------------------------------------------------- ldata_info
def ldata_info(cli_bin, fname):
    """`multiview-cli ldata info` of one envelope, as a dict."""
    r = subprocess.run([cli_bin, "ldata", "info", fname], stdout=subprocess.PIPE,
                       stderr=subprocess.PIPE, text=True, timeout=600)
    if r.returncode != 0:
        raise RuntimeError(f"ldata info {fname}: exit {r.returncode}: {(r.stdout + r.stderr)[-500:]}")
    return json.loads(r.stdout)


# ---------------------------------------------------------------- check_product
def check_product(lease, workdir, cli_bin):
    """(error, info) for a loc shard's envelope: error is None when it covers
    exactly the frames of the lease."""
    fname = os.path.join(workdir, "out", "localization.data")
    try:
        info = ldata_info(cli_bin, fname)
    except (RuntimeError, ValueError, OSError, subprocess.TimeoutExpired) as e:
        return f"cannot read the envelope: {e}", None
    c = lease["cli"]
    if info.get("start_frame") != c["start_frame"] or info.get("n_frames") != c["n_frames"]:
        return (f"envelope covers {info.get('n_frames')} frames from {info.get('start_frame')}, "
                f"the lease {c['n_frames']} from {c['start_frame']}"), info
    return None, info


# ---------------------------------------------------------------- product_paths
def product_paths(lease, workdir):
    """name -> local path of every output the run may have produced, except
    done.json (which the agent writes last)."""
    out = os.path.join(workdir, "out")
    names = {"stdout.log": os.path.join(workdir, "stdout.log"),
             "params.json": os.path.join(out, "params.json")}
    if lease["kind"] == "loc":
        names["localization.data"] = os.path.join(out, "localization.data")
        names["run-object.json"] = os.path.join(out, "run-object.json")
        names["poses.tar"] = os.path.join(workdir, "poses.tar")
    else:
        names["pipeline-output.json"] = os.path.join(out, "pipeline-output.json")
        names["score.json"] = os.path.join(out, "score.json")
        names["overlap.json"] = os.path.join(out, "overlap.json")
    return names


# -------------------------------------------------------------------- run_shard
def run_shard(lease, workdir, cli_bin, *, env, prefix, gpu_lock, kill_grace_s, should_stop,
              on_progress=None, on_lock=None, fault_kill_after_s=None):
    """Runs the shard whose inputs are already in `workdir` (in/, params.json)
    and returns a RunResult. The GPU lock is held from before the pipeline
    starts until it has exited; max_wall_s counts from the lock, and
    on_lock() is called when it is taken."""
    log_path = os.path.join(workdir, "stdout.log")
    pid_path = os.path.join(workdir, "child.pid")
    os.makedirs(os.path.join(workdir, "out"), exist_ok=True)
    paths = product_paths(lease, workdir)

    def child(argv, max_wall_s, progress_timeout_s, fault=None):
        return run_child(prefix + argv, cwd=workdir, env=env, log_path=log_path, pid_path=pid_path,
                         max_wall_s=max_wall_s, progress_timeout_s=progress_timeout_s,
                         kill_grace_s=kill_grace_s, should_stop=should_stop,
                         on_progress=on_progress, fault_kill_after_s=fault)

    def result(code, error, wall_s, sid, product=None, outcome=None):
        tail = read_tail(log_path)
        if outcome is None:
            present = product is None or (os.path.isfile(paths[product])
                                          and os.path.getsize(paths[product]) > 0)
            outcome = outcome_of_(code, error, tail, present)
            if outcome != "ok" and not error:
                error = f"exit 0 without {product}" if code == 0 else f"exit {code}: {last_line_(tail)}"
        return RunResult(exit_code=code, outcome=outcome, error=error, wall_s=wall_s,
                         log_tail=tail, sid=sid)

    if lease["kind"] == "tracks" and merged_input_(lease):
        r = result(*child(cat_argv(lease, workdir, cli_bin), 600, 600))
        if r.outcome != "ok":
            r.outcome = "fatal" if r.outcome == "retry" else r.outcome  # inputs are md5-checked
            return r

    t_lock = time.monotonic()
    if not gpu_lock.acquire(should_stop):
        return RunResult(exit_code=None, outcome="stopped", error=should_stop() or "stopped")
    lock_wait_s = time.monotonic() - t_lock
    if on_lock is not None:
        on_lock()
    try:
        main_product = "localization.data" if lease["kind"] == "loc" else "pipeline-output.json"
        r = result(*child(pipeline_argv(lease, workdir, cli_bin), lease["max_wall_s"],
                          lease["progress_timeout_s"], fault_kill_after_s), product=main_product)
    finally:
        gpu_lock.release()
    r.lock_wait_s = lock_wait_s
    if r.outcome != "ok":
        return r

    gt = os.path.join(workdir, "in", "ground-truth.json")
    if lease["kind"] == "tracks" and os.path.exists(gt):
        argv = [cli_bin, "score", "--gt", gt, "--calc", paths["pipeline-output.json"],
                "--json", paths["score.json"]]
        r2 = result(*child(argv, 600, 600), product="score.json")
        if r2.outcome != "ok":
            r2.error = f"score: {r2.error}"
            return r2
    r.log_tail = read_tail(log_path)
    r.outputs = {k: v for k, v in paths.items() if os.path.isfile(v)}
    return r


# ------------------------------------------------------------------ outcome_of_
def outcome_of_(code, error, tail, present):
    if error.startswith("timeout:"):
        return "timeout"
    if error.startswith("fault injection"):
        return "retry"
    if error:
        return "stopped"
    return classify_exit(code, tail, present, False)


# ------------------------------------------------------------------- last_line_
def last_line_(tail):
    """The line of the log tail that says why: the last one with a fatal,
    then a retry pattern, else the last line."""
    lines = [k_ansi.sub("", x).strip() for x in tail.splitlines() if x.strip()]
    for patterns in (k_fatal_patterns, k_retry_patterns):
        hits = [x for x in lines if any(p in x for p in patterns)]
        if hits:
            return hits[-1][:300]
    return lines[-1][:300] if lines else ""
