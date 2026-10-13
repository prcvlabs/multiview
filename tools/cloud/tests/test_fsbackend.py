"""fsbackend.FsControl: leases, heartbeats, retries, the tracks job's
promotion, cancel, done.json markers, and two agents leasing at once."""

import multiprocessing
import os
import tempfile
import unittest

import support
from mvcloud import storage
from mvcloud.control import Conflict, HeartbeatResult
from mvcloud.fsbackend import FsControl
from mvcloud.plan import PlanError

k_worker = {"worker_id": "w0", "gpu": "", "version": "test"}


class Clock:
    def __init__(self):
        self.t = 1_000_000.0

    def __call__(self):
        return self.t


# ------------------------------------------------------------------- lease_all_
def lease_all_(root, out):
    """One agent process: leases until there is nothing left."""
    control = FsControl(root)
    got = []
    while True:
        lease = control.pull({**k_worker, "worker_id": f"w{os.getpid()}"})
        if lease is None:
            break
        got.append((lease["shard_id"], lease["attempt"]))
    out.put(got)


class TestFsBackend(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.root = os.path.join(self.tmp.name, "root")
        self.paths = support.make_scene(os.path.join(self.tmp.name, "scene"), n_frames=60)
        self.clock = Clock()
        self.control = FsControl(self.root, clock=self.clock)

    def tearDown(self):
        self.tmp.cleanup()

    def submit(self, **kw):
        kw.setdefault("total_frames", 60)
        kw.setdefault("n_frames", 60)
        return self.control.submit(support.make_spec(self.paths, **kw))

    def shard(self, job_id, shard_id):
        return next(r for r in self.control.status(job_id)[0]["shards"] if r["id"] == shard_id)

    def job_state(self, job_id):
        return self.control.status(job_id)[0]["state"]

    def test_submit_rejects(self):
        with self.assertRaises(PlanError):
            self.submit(n_frames=1)
        with self.assertRaises(PlanError):
            self.submit(n_frames=70)  # past the 60-frame scene
        with self.assertRaises(PlanError):
            self.submit(start_frame=5, n_frames=50)
        self.assertEqual(self.control.status(), [])

    def test_two_agents_never_share_a_shard(self):
        job_id = self.submit()
        self.assertEqual([r["id"] for r in self.control.status(job_id)[0]["shards"]],
                         ["loc-0000", "loc-0001", "loc-0002", "tracks"])
        ctx = multiprocessing.get_context("fork")
        q = ctx.Queue()
        agents = [ctx.Process(target=lease_all_, args=(self.root, q)) for _ in range(2)]
        for p in agents:
            p.start()
        got = q.get(timeout=30) + q.get(timeout=30)
        for p in agents:
            p.join(timeout=30)
        self.assertEqual(sorted(got), [("loc-0000", 1), ("loc-0001", 1), ("loc-0002", 1)])

    def test_heartbeat_extends_and_expiry_releases(self):
        job_id = self.submit()
        lease = self.control.pull(k_worker)
        self.assertEqual((lease["shard_id"], lease["attempt"]), ("loc-0000", 1))
        self.clock.t += 20
        self.assertEqual(self.control.heartbeat(lease, {"line": "x"}), HeartbeatResult.OK)
        self.assertEqual(self.shard(job_id, "loc-0000")["lease_expires_at"], self.clock.t + 30)
        self.clock.t += 31  # past the extended expiry
        leases = [self.control.pull(k_worker) for _ in range(3)]
        self.assertEqual([(x["shard_id"], x["attempt"]) for x in leases],
                         [("loc-0000", 2), ("loc-0001", 1), ("loc-0002", 1)])
        self.assertEqual(self.control.heartbeat(lease, {}), HeartbeatResult.LOST)
        with self.assertRaises(Conflict):
            support.complete_lease(self.control, lease)
        r = self.shard(job_id, "loc-0000")
        self.assertEqual([a["outcome"] for a in r["attempts"]], ["expired", None])

    def test_heartbeat_refused_after_the_wall_budget(self):
        self.submit(max_wall_s={"loc": 100, "tracks": None})
        lease = self.control.pull(k_worker)
        self.clock.t += 1000  # waiting for the GPU lock does not count
        self.assertEqual(self.control.heartbeat(lease, {}), HeartbeatResult.OK)
        progress = {"run_started_at": self.clock.t}
        self.clock.t += 149
        self.assertEqual(self.control.heartbeat(lease, progress), HeartbeatResult.OK)
        self.clock.t += 2  # 1.5 x max_wall_s after the run started
        self.assertEqual(self.control.heartbeat(lease, progress), HeartbeatResult.LOST)

    def test_retries_then_dead(self):
        job_id = self.submit(n_frames=20, shard_frames=20, total_frames=60)
        for attempt in (1, 2, 3):
            lease = self.control.pull(k_worker)
            self.assertEqual((lease["shard_id"], lease["attempt"]), ("loc-0000", attempt))
            self.control.fail(lease, "exit -6: abort", retryable=True)
        self.assertIsNone(self.control.pull(k_worker))
        r = self.shard(job_id, "loc-0000")
        self.assertEqual(r["state"], "dead")
        self.assertEqual([a["outcome"] for a in r["attempts"]], ["retry"] * 3)
        self.assertEqual(self.job_state(job_id), "failed")

    def test_retry_waits_for_its_delay(self):
        self.submit(n_frames=20, shard_frames=20, retry_delay_s=30)
        self.control.fail(self.control.pull(k_worker), "boom", retryable=True)
        self.assertIsNone(self.control.pull(k_worker))
        self.clock.t += 30
        self.assertEqual(self.control.pull(k_worker)["attempt"], 2)

    def test_fatal_fails_fast(self):
        job_id = self.submit()
        self.control.fail(self.control.pull(k_worker), "--require-aligned: start", retryable=False)
        self.assertEqual(self.job_state(job_id), "failed")
        self.assertIsNone(self.control.pull(k_worker))

    def test_tracks_job_after_every_loc_shard(self):
        job_id = self.submit()
        leases = [self.control.pull(k_worker) for _ in range(3)]
        self.assertIsNone(self.control.pull(k_worker))
        for lease in leases[:2]:
            support.complete_lease(self.control, lease)
        self.assertEqual(self.shard(job_id, "tracks")["state"], "blocked")
        self.assertIsNone(self.control.pull(k_worker))
        support.complete_lease(self.control, leases[2])
        tracks = self.control.pull(k_worker)
        self.assertEqual(tracks["kind"], "tracks")
        self.assertEqual(tracks["params_md5"], "p0")
        self.assertEqual([k["name"] for k in tracks["keep"]],
                         ["loc-0000.data", "loc-0001.data", "loc-0002.data"])
        envs = [i for i in tracks["inputs"] if i["cache"] == "job"]
        self.assertEqual(len(envs), 3)
        self.assertTrue(all(storage.file_md5(storage.local_path(i["url"])) == i["md5"] for i in envs))
        self.assertIn("/a1/", envs[0]["url"])  # attempt-scoped prefixes
        self.assertEqual(tracks["cli"]["start_frame"], 0)
        self.assertEqual(tracks["cli"]["n_frames"], 60)
        support.complete_lease(self.control, tracks)
        self.assertEqual(self.job_state(job_id), "done")
        self.assertTrue(os.path.isfile(self.control.blob_path(f"jobs/{job_id}/summary.json")))
        self.assertIsNone(self.control.pull(k_worker))

    def test_complete_checks_outputs(self):
        self.submit()
        lease = self.control.pull(k_worker)
        with self.assertRaises(Conflict):
            support.complete_lease(self.control, lease, outputs={"params.json"})
        done = support.complete_lease(self.control, lease)
        self.control.complete(lease, done)  # a repeated complete is a no-op

    def test_params_md5_mismatch_is_fatal(self):
        job_id = self.submit()
        a = self.control.pull(k_worker)
        b = self.control.pull(k_worker)
        support.complete_lease(self.control, a, params_md5="p0")
        support.complete_lease(self.control, b, params_md5="p1")
        r = self.shard(job_id, "loc-0001")
        self.assertEqual(r["state"], "dead")
        self.assertIn("params_md5", r["error"])
        self.assertEqual(self.job_state(job_id), "failed")

    def test_cancel(self):
        job_id = self.submit()
        leased = self.control.pull(k_worker)
        self.control.cancel(job_id)
        rows = self.control.status(job_id)[0]["shards"]
        self.assertTrue(all(r["state"] == "cancelled" for r in rows))
        self.assertEqual(self.job_state(job_id), "cancelled")
        self.assertEqual(self.control.heartbeat(leased, {}), HeartbeatResult.LOST)
        support.complete_lease(self.control, leased)  # accepted, changes nothing
        self.control.fail(leased, "lease lost", retryable=True)
        self.assertEqual(self.shard(job_id, "loc-0000")["state"], "cancelled")
        self.assertIsNone(self.control.pull(k_worker))

    def test_missing_done_json_requeues(self):
        job_id = self.submit()
        for _ in range(3):
            support.complete_lease(self.control, self.control.pull(k_worker))
        support.complete_lease(self.control, self.control.pull(k_worker))
        self.assertEqual(self.job_state(job_id), "done")
        self.assertEqual(self.control.sweep(), [])  # nothing to do on a finished job
        prefix = self.shard(job_id, "loc-0001")["output_prefix"]
        os.remove(self.control.blob_path(f"{prefix}/done.json"))
        events = self.control.sweep()
        self.assertIn(f"{job_id}/loc-0001: invalidated: done.json is missing", events)
        self.assertEqual(self.shard(job_id, "tracks")["state"], "blocked")
        self.assertEqual(self.job_state(job_id), "running")
        lease = self.control.pull(k_worker)
        self.assertEqual((lease["shard_id"], lease["attempt"]), ("loc-0001", 2))
        self.assertIn("/a2/", lease["outputs"][0]["url"])

    def test_invalidated_shard_gets_a_full_retry_budget(self):
        job_id = self.submit(n_frames=20, shard_frames=20)
        for _ in range(2):
            self.control.fail(self.control.pull(k_worker), "exit -6: abort", retryable=True)
        support.complete_lease(self.control, self.control.pull(k_worker))  # attempt 3 of 3
        support.complete_lease(self.control, self.control.pull(k_worker))  # tracks, attempt 1
        self.assertEqual(self.job_state(job_id), "done")
        prefix = self.shard(job_id, "loc-0000")["output_prefix"]
        os.remove(self.control.blob_path(f"{prefix}/done.json"))
        self.control.sweep()
        self.assertEqual(self.shard(job_id, "tracks")["max_attempts"], 1 + 3)
        for attempt in (4, 5):  # retryable failures of the fresh run
            lease = self.control.pull(k_worker)
            self.assertEqual((lease["shard_id"], lease["attempt"]), ("loc-0000", attempt))
            self.control.fail(lease, "exit -6: abort", retryable=True)
        self.assertEqual(self.shard(job_id, "loc-0000")["state"], "pending")
        self.assertEqual(self.job_state(job_id), "running")
        support.complete_lease(self.control, self.control.pull(k_worker))
        support.complete_lease(self.control, self.control.pull(k_worker))
        self.assertEqual(self.job_state(job_id), "done")

    def test_verify_finds_a_corrupt_output(self):
        job_id = self.submit()
        for _ in range(4):
            support.complete_lease(self.control, self.control.pull(k_worker))
        prefix = self.shard(job_id, "loc-0002")["output_prefix"]
        path = self.control.blob_path(f"{prefix}/localization.data")
        with open(path, "r+b") as f:
            f.write(b"X")  # same size, other bytes
        self.assertEqual(self.control.sweep(), [])
        self.assertIn(f"{job_id}/loc-0002: invalidated: localization.data has the wrong md5",
                      self.control.sweep(verify=True))


if __name__ == "__main__":
    unittest.main()
