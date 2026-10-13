"""procs: the GPU lock, session kills, orphan reaping, the memory cap probe."""

import os
import subprocess
import tempfile
import time
import unittest

import support  # noqa: F401  (sys.path)
from mvcloud import procs


def alive(pid):
    st = procs.proc_status_(pid)
    return st is not None and st[0] != "Z"


class TestProcs(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.d = self.tmp.name

    def tearDown(self):
        self.tmp.cleanup()

    def test_gpu_lock_excludes(self):
        path = os.path.join(self.d, "gpu.lock")
        a = procs.GpuLock(path)
        self.assertTrue(a.acquire())
        b = procs.GpuLock(path)
        self.assertFalse(b.acquire(should_stop=lambda: "stop", poll_s=0.01))
        r = subprocess.run(["flock", "-n", path, "true"])
        self.assertNotEqual(r.returncode, 0)  # build/dev.sh's flock waits too
        a.release()
        self.assertEqual(subprocess.run(["flock", "-n", path, "true"]).returncode, 0)
        with procs.GpuLock(path):
            self.assertFalse(procs.GpuLock(path).acquire(should_stop=lambda: True, poll_s=0.01))

    def test_kill_session_reaches_grandchildren(self):
        log_path = os.path.join(self.d, "log")
        pid_path = os.path.join(self.d, "child.pid")
        proc = procs.start_session(["bash", "-c", "sleep 60 & echo $! ; wait"], cwd=self.d,
                                   env=os.environ.copy(), log_path=log_path, pid_path=pid_path)
        for _ in range(100):
            if len(procs.session_members(proc.pid)) >= 2:
                break
            time.sleep(0.05)
        members = procs.session_members(proc.pid)
        self.assertEqual(len(members), 2)
        with open(pid_path) as f:
            self.assertEqual(int(f.read()), proc.pid)
        procs.kill_session(proc.pid, grace_s=2.0, proc=proc)
        self.assertEqual(procs.session_members(proc.pid), [])
        self.assertFalse(any(alive(p) for p in members))

    def test_kill_escalates_to_sigkill(self):
        proc = procs.start_session(["bash", "-c", "trap '' TERM ; sleep 60 & wait ; sleep 60"],
                                   cwd=self.d, env=os.environ.copy(),
                                   log_path=os.path.join(self.d, "log"))
        time.sleep(0.3)
        t0 = time.monotonic()
        procs.kill_session(proc.pid, grace_s=0.5, proc=proc)
        self.assertEqual(procs.session_members(proc.pid), [])
        self.assertLess(time.monotonic() - t0, 5.0)

    def test_reap_orphans(self):
        # A stand-in named multiview-cli, as the real binary's comm is
        bin_dir = os.path.join(self.d, "bin")
        os.makedirs(bin_dir)
        os.symlink("/usr/bin/sleep", os.path.join(bin_dir, "multiview-cli"))
        work = os.path.join(self.d, "work")
        a = os.path.join(work, "job", "loc-0000", "a1")
        b = os.path.join(work, "job", "loc-0001", "a1")
        os.makedirs(a)
        os.makedirs(b)
        orphan = subprocess.Popen([os.path.join(bin_dir, "multiview-cli"), "60"],
                                  start_new_session=True)
        with open(os.path.join(a, "child.pid"), "w") as f:
            f.write(f"{orphan.pid}\n")
        gone = subprocess.Popen(["true"], start_new_session=True)
        gone.wait()
        with open(os.path.join(b, "child.pid"), "w") as f:
            f.write(f"{gone.pid}\n")
        other = subprocess.Popen(["sleep", "60"], start_new_session=True)  # not a multiview-cli
        with open(os.path.join(b, "child.pid"), "w") as f:
            f.write(f"{other.pid}\n")
        try:
            self.assertEqual(procs.reap_orphans(work, grace_s=2.0), 1)
            orphan.wait(timeout=5)
            self.assertFalse(alive(orphan.pid))
            self.assertTrue(alive(other.pid))
            self.assertFalse(os.path.exists(os.path.join(a, "child.pid")))
        finally:
            other.kill()
            other.wait()

    def test_memory_cap_prefix(self):
        prefix, capped = procs.memory_cap_prefix("24G", probe=["true"])
        self.assertTrue(capped)
        self.assertEqual(prefix[:3], ["systemd-run", "--user", "--scope"])
        self.assertIn("MemoryMax=24G", prefix)
        self.assertEqual(prefix[-5:], ["nice", "-n", "15", "ionice", "-c3"])
        with self.assertLogs("mvcloud", level="WARNING") as logs:
            prefix, capped = procs.memory_cap_prefix("24G", probe=["false"])
        self.assertEqual((prefix, capped), (["nice", "-n", "15"], False))
        self.assertIn("no per-process memory cap", logs.output[0])
        self.assertEqual(procs.memory_cap_prefix(""), (["nice", "-n", "15"], False))


if __name__ == "__main__":
    unittest.main()
