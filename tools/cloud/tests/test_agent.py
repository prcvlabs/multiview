"""The agent end to end against fake_multiview_cli.py and the fs backend:
command lines, outcomes, done.json, the watchdog, heartbeats, cancel and
SIGTERM, and that no process of a child's session survives."""

import json
import os
import signal
import subprocess
import sys
import tempfile
import threading
import time
import unittest

import support
from mvcloud import runner, storage
from mvcloud.agent import Agent
from mvcloud.control import HeartbeatResult
from mvcloud.fsbackend import FsControl


# --------------------------------------------------------------------- Recorder
class Recorder:
    """A Control that forwards to FsControl, records calls, and can answer
    heartbeats with a script of results."""

    def __init__(self, control, heartbeats=()):
        self.control = control
        self.heartbeats = list(heartbeats)
        self.events = []

    def pull(self, worker):
        return self.control.pull(worker)

    def heartbeat(self, lease, progress):
        self.events.append(("heartbeat", time.monotonic()))
        if self.heartbeats:
            return self.heartbeats.pop(0)
        return self.control.heartbeat(lease, progress)

    def complete(self, lease, done):
        self.events.append(("complete", time.monotonic()))
        return self.control.complete(lease, done)

    def fail(self, lease, error, retryable, outcome=None):
        self.events.append(("fail", time.monotonic()))
        return self.control.fail(lease, error, retryable, outcome=outcome)


def pids_alive(fname):
    if not os.path.exists(fname):
        return []
    with open(fname) as f:
        pids = [int(x) for x in f.read().split()]
    out = []
    for p in pids:
        try:
            with open(f"/proc/{p}/stat") as f:
                if f.read().rsplit(")", 1)[1].split()[0] != "Z":
                    out.append(p)
        except OSError:
            pass
    return out


class TestAgent(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        d = self.tmp.name
        self.root = os.path.join(d, "root")
        self.paths = support.make_scene(os.path.join(d, "scene"))
        self.cli = support.fake_cli(os.path.join(d, "bin"))
        self.argv_log = os.path.join(d, "argv.jsonl")
        self.pids = os.path.join(d, "pids")
        self.control = FsControl(self.root)

    def tearDown(self):
        for p in pids_alive(self.pids):
            os.kill(p, signal.SIGKILL)
        self.tmp.cleanup()

    def agent(self, mode="ok", control=None, **kw):
        env = {"MVCLOUD_FAKE_MODE": mode, "MVCLOUD_FAKE_LOG": self.argv_log,
               "MVCLOUD_FAKE_PIDS": self.pids, "MVCLOUD_FAKE_SECONDS": kw.pop("seconds", "4")}
        d = self.tmp.name
        return Agent(control or self.control, cli_bin=self.cli, cache_dir=os.path.join(d, "cache"),
                     work_dir=os.path.join(d, "work"), gpu_lock=os.path.join(d, "gpu.lock"),
                     mem=None, kill_grace_s=1.0, env=env, **kw)

    def submit(self, **kw):
        return self.control.submit(support.make_spec(self.paths, **kw))

    def shard(self, job_id, shard_id):
        return next(r for r in self.control.status(job_id)[0]["shards"] if r["id"] == shard_id)

    def argvs(self, kind):
        with open(self.argv_log) as f:
            return [x["argv"] for x in map(json.loads, f) if x["kind"] == kind]

    def one_shard(self, mode, **kw):
        """Runs the loc shard of a one-shard job, in a root of its own, once;
        returns (outcome, row)."""
        self.control = FsControl(tempfile.mkdtemp(dir=self.tmp.name))
        job_id = self.submit(n_frames=20, **kw)
        outcome = self.agent(mode).run_lease(self.control.pull({"worker_id": "t"}))
        return outcome, self.shard(job_id, "loc-0000")

    # -------------------------------------------------------------------- tests
    def test_job_end_to_end(self):
        job_id = self.submit()
        self.assertEqual(self.agent().run(drain=True), 3)
        job = self.control.status(job_id)[0]
        self.assertEqual(job["state"], "done")
        for r in job["shards"]:
            self.assertEqual([a["outcome"] for a in r["attempts"]], ["done"])
            self.assertTrue(r["output_prefix"].endswith(f"/{r['id']}/a1"))
            with open(self.control.blob_path(f"{r['output_prefix']}/done.json")) as f:
                done = json.load(f)
            self.assertEqual(done["exit_code"], 0)
            names = {o["name"] for o in done["outputs"]}
            for o in done["outputs"]:
                path = self.control.blob_path(f"{r['output_prefix']}/{o['name']}")
                self.assertEqual(storage.file_md5(path), o["md5"])
            if r["kind"] == "loc":
                self.assertEqual(names, {"localization.data", "params.json", "run-object.json",
                                         "stdout.log"})
                self.assertEqual(done["ldata_info"]["start_frame"], r["start_frame"])
                self.assertEqual(len(done["ldata_info"]["frame_md5"]), 20)
            else:
                self.assertEqual(names, {"pipeline-output.json", "params.json", "score.json",
                                         "stdout.log"})
                self.assertEqual(done["n_tracks"], 1)
                self.assertEqual(done["score"]["f1_jaccard"], 1.0)
        md5s = {r["params_md5"] for r in job["shards"]}
        self.assertEqual(len(md5s), 1)

        loc = self.argvs("loc")
        self.assertEqual(len(loc), 2)
        for argv in loc:
            self.assertIn("--no-still", argv)
            self.assertIn("--no-stereo", argv)
            self.assertIn("--save-loc-data", argv)
            self.assertNotIn("--pose-classifier", argv)
        tracks = self.argvs("tracks")[0]
        self.assertIn("--require-aligned", tracks)
        self.assertEqual(tracks[tracks.index("--pose-classifier") + 1], "pose-svm-lite_v0")
        self.assertNotIn("--no-still", tracks)
        self.assertEqual(tracks.count("--load-loc-data"), 2)
        self.assertEqual(tracks[tracks.index("--n-frames") + 1], "40")
        self.assertFalse(os.listdir(os.path.join(self.tmp.name, "work", job_id, "tracks")))

    def test_rerun_on_a_finished_job_does_nothing(self):
        job_id = self.submit()
        self.agent().run(drain=True)
        before = self.control.status(job_id)
        self.assertEqual(self.agent().run(drain=True), 0)
        self.assertEqual(self.control.status(job_id), before)

    def test_stereo_loc_argv(self):
        lease = {"kind": "loc", "cli": {"start_frame": 20, "n_frames": 20, "no_stereo": False,
                                        "no_still": False, "still_warmup": 20, "extra_args": ["-x"]}}
        argv = runner.pipeline_argv(lease, "/w", "/b/multiview-cli")
        self.assertNotIn("--no-still", argv)
        self.assertNotIn("--no-stereo", argv)
        self.assertEqual(argv[argv.index("--still-warmup") + 1], "20")
        self.assertEqual(argv[-1], "-x")

    def test_outcomes(self):
        # retry_delay_s is 0: a retryable failure is pending again at once
        for mode, outcome, state in [("abort", "retry", "pending"), ("fatal", "retry", "pending"),
                                     ("no-output", "retry", "pending"),
                                     ("unused", "fatal", "dead"), ("savefail", "fatal", "dead"),
                                     ("wrong-range", "fatal", "dead")]:
            with self.subTest(mode=mode):
                got, r = self.one_shard(mode)
                self.assertEqual((got, r["state"], r["attempts"][-1]["outcome"]),
                                 (outcome, state, outcome))
                self.assertTrue(r["error"])

    def test_a_grandchild_left_behind_is_killed(self):
        got, r = self.one_shard("leave-child")
        self.assertEqual((got, r["state"]), ("ok", "done"))
        self.assertEqual(pids_alive(self.pids), [])

    def test_missing_replay_file_is_fatal(self):
        os.remove(os.path.join(self.paths["poses"], "pose-2d_000007_00.json"))
        got, r = self.one_shard("ok")
        self.assertEqual(got, "fatal")
        self.assertIn("pose replay: missing", r["error"])
        self.assertFalse(os.path.exists(self.argv_log))  # nothing ran

    def test_corrupt_input_is_fatal(self):
        job_id = self.submit(n_frames=20)
        lease = self.control.pull({"worker_id": "t"})
        with open(self.paths["params"], "r+") as f:
            f.write("[")  # same size, other bytes
        self.assertEqual(self.agent().run_lease(lease), "fatal")
        self.assertIn("md5", self.shard(job_id, "loc-0000")["error"])

    def test_wall_timeout_kills_the_session(self):
        t0 = time.monotonic()
        got, r = self.one_shard("fork", max_wall_s={"loc": 2, "tracks": None})
        self.assertEqual(got, "timeout")
        self.assertTrue(r["error"].startswith("timeout: wall"))
        self.assertLess(time.monotonic() - t0, 2 + 1.0 + 3.0)
        with open(self.pids) as f:
            self.assertEqual(len(f.read().split()), 2)  # the fake and its grandchild
        self.assertEqual(pids_alive(self.pids), [])

    def test_progress_timeout(self):
        got, r = self.one_shard("progress-then-sleep", progress_timeout_s=2)
        self.assertEqual(got, "timeout")
        self.assertTrue(r["error"].startswith("timeout: no progress"))
        self.assertEqual(pids_alive(self.pids), [])

    def test_lost_lease_kills_the_child(self):
        job_id = self.submit(n_frames=20)
        control = Recorder(self.control, heartbeats=[HeartbeatResult.LOST])
        outcome = self.agent("sleep", control=control).run_lease(self.control.pull({"worker_id": "t"}))
        self.assertEqual(outcome, "stopped")
        r = self.shard(job_id, "loc-0000")
        self.assertEqual((r["state"], r["error"]), ("pending", "lease lost"))
        self.assertEqual(pids_alive(self.pids), [])

    def test_transient_heartbeats_are_retried(self):
        job_id = self.submit(n_frames=20)
        control = Recorder(self.control, heartbeats=[HeartbeatResult.TRANSIENT] * 2)
        a = self.agent("slow", control=control, seconds="5")
        self.assertEqual(a.run_lease(self.control.pull({"worker_id": "t"})), "ok")
        self.assertGreaterEqual(sum(1 for e in control.events if e[0] == "heartbeat"), 3)
        self.assertEqual(self.shard(job_id, "loc-0000")["state"], "done")

    def test_cancel_during_a_run(self):
        job_id = self.submit(n_frames=20)
        lease = self.control.pull({"worker_id": "t"})
        threading.Timer(1.0, self.control.cancel, args=(job_id,)).start()
        t0 = time.monotonic()
        self.assertEqual(self.agent("sleep").run_lease(lease), "stopped")
        self.assertLess(time.monotonic() - t0, 1.0 + 1.0 + 1.0 + 3.0)  # heartbeat_s / 3 + grace
        self.assertEqual(self.shard(job_id, "loc-0000")["state"], "cancelled")
        self.assertEqual(self.control.status(job_id)[0]["state"], "cancelled")
        self.assertEqual(pids_alive(self.pids), [])

    def test_heartbeats_continue_during_a_slow_upload(self):
        job_id = self.submit(n_frames=20)
        control = Recorder(self.control)
        window = []

        def slow_put(url, src, headers=None):
            if url.endswith("/localization.data"):
                window.append(time.monotonic())
                time.sleep(3.0)  # 3 x heartbeat_s / 3
                window.append(time.monotonic())
            storage.put(url, src, headers=headers)

        a = self.agent(control=control, uploader=slow_put)
        self.assertEqual(a.run_lease(self.control.pull({"worker_id": "t"})), "ok")
        beats = [t for e, t in control.events if e == "heartbeat"]
        self.assertGreaterEqual(sum(1 for t in beats if window[0] <= t <= window[1]), 2)
        self.assertEqual(control.events[-1][0], "complete")  # joined before complete
        self.assertEqual(self.shard(job_id, "loc-0000")["state"], "done")

    def test_sigterm_kills_the_session_and_fails_the_attempt(self):
        job_id = self.submit(n_frames=20)
        d = self.tmp.name
        env = {**os.environ, "PYTHONPATH": os.path.join(os.path.dirname(support.k_tests_dir), "agent"),
               "MVCLOUD_FAKE_MODE": "fork", "MVCLOUD_FAKE_PIDS": self.pids}
        agent = subprocess.Popen(
            [sys.executable, "-m", "mvcloud", "agent", "--control", f"fs://{self.root}", "--cli",
             self.cli, "--cache", os.path.join(d, "cache"), "--work", os.path.join(d, "work"),
             "--gpu-lock", os.path.join(d, "gpu.lock"), "--mem", "", "--kill-grace", "1", "--once"],
            env=env, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE, text=True)
        for _ in range(100):
            if len(pids_alive(self.pids)) == 2:
                break
            time.sleep(0.1)
        self.assertEqual(len(pids_alive(self.pids)), 2)
        agent.send_signal(signal.SIGTERM)
        _, err = agent.communicate(timeout=20)
        self.assertEqual(agent.returncode, 0, err)
        self.assertEqual(pids_alive(self.pids), [])
        r = self.shard(job_id, "loc-0000")
        self.assertEqual((r["state"], r["error"]), ("pending", "agent terminated"))
        self.assertEqual(r["attempts"][-1]["outcome"], "stopped")

    def test_fault_injection_kills_once(self):
        job_id = self.submit(n_frames=20)
        os.environ["MVCLOUD_FAULT"] = "kill-after:1"
        try:
            a = self.agent("loc=slow,tracks=ok", seconds="2")
            self.assertEqual(a.run(drain=True), 3)  # killed, retried, then the tracks job
        finally:
            del os.environ["MVCLOUD_FAULT"]
        r = self.shard(job_id, "loc-0000")
        self.assertEqual([x["outcome"] for x in r["attempts"]], ["retry", "done"])
        self.assertTrue(r["attempts"][0]["error"].startswith("fault injection"))
        self.assertEqual(self.control.status(job_id)[0]["state"], "done")

    def test_cli_submit_fills_the_scene_from_scene_info(self):
        spec = support.make_spec(self.paths)
        for key in ("total_frames", "K", "n_cameras"):
            del spec[key]
        job = os.path.join(self.tmp.name, "job.json")
        with open(job, "w") as f:
            json.dump(spec, f)
        env = {**os.environ, "PYTHONPATH": os.path.join(os.path.dirname(support.k_tests_dir), "agent")}
        run = lambda *a: subprocess.run([sys.executable, "-m", "mvcloud", *a], env=env, text=True,
                                        stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        r = run("plan", "--cli", self.cli, job)
        self.assertEqual(r.returncode, 0, r.stderr)
        self.assertIn("[20, 40)", r.stdout)
        r = run("submit", "--control", f"fs://{self.root}", job)
        self.assertEqual(r.returncode, 1)  # no scene numbers and no --cli
        r = run("submit", "--control", f"fs://{self.root}", "--cli", self.cli, job)
        self.assertEqual(r.returncode, 0, r.stderr)
        s = self.control.status(r.stdout.strip())[0]["spec"]
        self.assertEqual((s["total_frames"], s["K"], s["n_cameras"]), (40, 10, 1))
        spec["n_frames"] = 50  # past the 40-frame scene
        with open(job, "w") as f:
            json.dump(spec, f)
        r = run("submit", "--control", f"fs://{self.root}", "--cli", self.cli, job)
        self.assertEqual(r.returncode, 1)
        self.assertIn("outside the scene", r.stderr)


if __name__ == "__main__":
    unittest.main()
