"""Child processes: the GPU lock, sessions, kills and the memory cap.

The agent itself holds the GPU lock for as long as its child runs (a
forking `flock` wrapper would swallow the kill and orphan the GPU job). The
child starts a new session, so a kill reaches everything it started.
"""

import fcntl
import logging
import os
import signal
import subprocess
import threading
import time

log = logging.getLogger("mvcloud")

k_cli_comm = "multiview-cli"


# ---------------------------------------------------------------------- GpuLock
class GpuLock:
    """An exclusive flock on `path`, the file build/dev.sh locks for GPU runs,
    so the agent and dev runs exclude each other."""

    def __init__(self, path):
        self.path = path
        self.fd_ = None

    def acquire(self, should_stop=None, poll_s=0.5):
        """Waits for the lock. Returns False, without the lock, as soon as
        should_stop() returns something true. The wait is a blocking flock
        in a helper thread, so the agent queues for the GPU like dev.sh's
        `flock` does, instead of polling behind it."""
        fd = os.open(self.path, os.O_RDWR | os.O_CREAT, 0o666)
        got = threading.Event()
        abandoned = threading.Event()
        padlock = threading.Lock()

        def wait_():
            fcntl.flock(fd, fcntl.LOCK_EX)
            with padlock:
                if abandoned.is_set():
                    fcntl.flock(fd, fcntl.LOCK_UN)
                    os.close(fd)
                    return
                got.set()

        threading.Thread(target=wait_, name="gpu-lock", daemon=True).start()
        while not got.wait(poll_s):
            if should_stop is not None and should_stop():
                with padlock:
                    if not got.is_set():
                        abandoned.set()
                        return False
        self.fd_ = fd
        return True

    def release(self):
        if self.fd_ is not None:
            fcntl.flock(self.fd_, fcntl.LOCK_UN)
            os.close(self.fd_)
            self.fd_ = None

    def __enter__(self):
        self.acquire()
        return self

    def __exit__(self, *exc):
        self.release()


# ----------------------------------------------------------------- proc_status_
def proc_status_(pid):
    """(state, pgrp, session) of a process, or None if it is gone."""
    try:
        with open(f"/proc/{pid}/stat") as f:
            stat = f.read()
    except OSError:
        return None
    fields = stat[stat.rfind(")") + 2:].split()
    return fields[0], int(fields[2]), int(fields[3])


# -------------------------------------------------------------- session_members
def session_members(sid):
    """The live (not zombie) processes of session `sid`."""
    out = []
    for name in os.listdir("/proc"):
        if not name.isdigit():
            continue
        st = proc_status_(int(name))
        if st is not None and st[2] == sid and st[0] != "Z":
            out.append(int(name))
    return out


# ---------------------------------------------------------------- start_session
def start_session(argv, *, cwd, env, log_path, pid_path=None):
    """Starts `argv` as the leader of a new session, stdout and stderr
    appended to `log_path`. The session id (the child's pid) goes to
    `pid_path`, for reap_orphans after a crash of the agent."""
    with open(log_path, "ab") as out:
        proc = subprocess.Popen(argv, cwd=cwd, env=env, stdin=subprocess.DEVNULL, stdout=out,
                                stderr=subprocess.STDOUT, start_new_session=True)
    if pid_path is not None:
        with open(pid_path, "w") as f:
            f.write(f"{proc.pid}\n")
    return proc


# ----------------------------------------------------------------- kill_session
def kill_session(sid, *, grace_s, proc=None):
    """SIGTERM to every process of session `sid`, SIGKILL to those still
    there after `grace_s`; then reaps the leader if it is our child."""

    def signal_all(sig):
        for pid in session_members(sid):
            try:
                os.kill(pid, sig)
            except ProcessLookupError:
                pass

    def wait_gone(timeout_s):
        deadline = time.monotonic() + timeout_s
        while True:
            if proc is not None:
                proc.poll()
            if not session_members(sid):
                return True
            if time.monotonic() > deadline:
                return False
            time.sleep(0.1)

    signal_all(signal.SIGTERM)
    if not wait_gone(grace_s):
        log.warning(f"session {sid} still alive {grace_s:.0f} s after SIGTERM: SIGKILL")
        signal_all(signal.SIGKILL)
        wait_gone(10.0)
    if proc is not None:
        try:
            proc.wait(timeout=10.0)
        except subprocess.TimeoutExpired:
            log.error(f"session leader {sid} did not exit after SIGKILL")


# ----------------------------------------------------------------- reap_orphans
def reap_orphans(work_dir, *, grace_s):
    """Kills the sessions named in child.pid files under `work_dir` whose
    leader is still a multiview-cli: children of an agent that died. Returns
    the number of sessions killed."""
    n = 0
    for dirpath, _, files in os.walk(work_dir):
        if "child.pid" not in files:
            continue
        pid_path = os.path.join(dirpath, "child.pid")
        try:
            with open(pid_path) as f:
                sid = int(f.read().strip())
            with open(f"/proc/{sid}/comm") as f:
                comm = f.read().strip()
        except (OSError, ValueError):
            comm = None
        if comm == k_cli_comm:
            log.warning(f"killing orphaned session {sid} ({pid_path})")
            kill_session(sid, grace_s=grace_s)
            n += 1
        os.remove(pid_path)
    return n


# ------------------------------------------------------------ memory_cap_prefix
def memory_cap_prefix(mem, *, probe=None):
    """The command prefix that runs a child at nice 15 with a memory cap of
    `mem` (e.g. "24G"), and whether a cap is applied. The cap needs a systemd
    user session; without one (containers) the child runs uncapped."""
    nice = ["nice", "-n", "15"]
    if not mem:
        return nice, False
    probe = probe or ["systemd-run", "--user", "--scope", "--quiet", "true"]
    try:
        ok = subprocess.run(probe, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                            timeout=30).returncode == 0
    except (OSError, subprocess.TimeoutExpired):
        ok = False
    if not ok:
        log.warning("no per-process memory cap (no systemd user session); "
                    "use the container's --memory")
        return nice, False
    return ["systemd-run", "--user", "--scope", "--quiet", "--collect", "-p", f"MemoryMax={mem}",
            "-p", "MemorySwapMax=0"] + nice + ["ionice", "-c3"], True
