"""httpcontrol.HttpControl against a stdlib mock of the Worker API: the
answers the agent maps (204 = no work, 409 = lease lost or refused, 5xx =
transient), the tokens it adds to the Worker's own URLs, and submit's
content-addressed uploads. The Worker itself is tested by vitest
(tools/cloud/worker) and e2e_wrangler.sh."""

import hashlib
import http.server
import json
import os
import tempfile
import threading
import unittest

import support
from mvcloud import storage
from mvcloud.control import Conflict, HeartbeatResult
from mvcloud.httpcontrol import HttpControl, tar_poses


# ---------------------------------------------------------------------- MockApi
class MockApi(http.server.ThreadingHTTPServer):
    """Records every request; answers routes from `script` (path -> list of
    (status, body), consumed in order, the last one repeated) and keeps
    blobs in memory."""

    def __init__(self):
        super().__init__(("127.0.0.1", 0), Handler)
        self.requests = []
        self.script = {}
        self.blobs = {}

    def answer(self, path):
        answers = self.script.get(path, [(404, {"error": "no route"})])
        return answers.pop(0) if len(answers) > 1 else answers[0]


class Handler(http.server.BaseHTTPRequestHandler):
    def log_message(self, *args):
        pass

    def reply_(self, status, body=None, headers=None):
        raw = b"" if body is None else json.dumps(body).encode()
        self.send_response(status)
        for k, v in (headers or {}).items():
            self.send_header(k, v)
        self.send_header("Content-Length", str(len(raw)))
        self.end_headers()
        if self.command != "HEAD":
            self.wfile.write(raw)

    def handle_(self):
        n = int(self.headers.get("Content-Length") or 0)
        body = self.rfile.read(n) if n else b""
        self.server.requests.append({"method": self.command, "path": self.path,
                                     "headers": {k.lower(): v for k, v in self.headers.items()},
                                     "body": body})
        if self.path.startswith("/v1/blobs/"):
            key = self.path[len("/v1/blobs/"):]
            if self.command == "HEAD":
                if key in self.server.blobs:
                    self.send_response(200)
                    self.send_header("Content-Length", str(len(self.server.blobs[key])))
                    self.end_headers()
                else:
                    self.reply_(404)
                return
            if self.command == "PUT":
                self.server.blobs[key] = body
                return self.reply_(200, {"key": key})
        status, obj = self.server.answer(self.path)
        self.reply_(status, obj)

    do_GET = do_POST = do_PUT = do_HEAD = handle_


class TestHttpControl(unittest.TestCase):

    def setUp(self):
        self.api = MockApi()
        threading.Thread(target=self.api.serve_forever, daemon=True).start()
        self.base = f"http://127.0.0.1:{self.api.server_address[1]}"
        self.control = HttpControl(self.base, "wrk")
        self.tmp = tempfile.TemporaryDirectory()

    def tearDown(self):
        self.api.shutdown()
        self.api.server_close()
        self.tmp.cleanup()

    def lease(self):
        return {"job_id": "j", "shard_id": "loc-0000", "lease_token": "tok", "attempt": 1,
                "inputs": [{"name": "manifest.json", "url": f"{self.base}/v1/blobs/scenes/m.json"},
                           {"name": "elsewhere", "url": "https://r2.example/presigned?X-Amz=1"}],
                "outputs": [{"name": "localization.data", "url": f"{self.base}/v1/blobs/jobs/j/x"}]}

    def test_pull(self):
        self.api.script["/v1/pull"] = [(204, None)]
        self.assertIsNone(self.control.pull({"worker_id": "w"}))
        self.api.script["/v1/pull"] = [(200, self.lease())]
        lease = self.control.pull({"worker_id": "w"})
        self.assertEqual(lease["inputs"][0]["headers"], {"Authorization": "Bearer wrk"})
        self.assertNotIn("headers", lease["inputs"][1])  # a presigned URL carries its own
        self.assertEqual(lease["outputs"][0]["headers"],
                         {"Authorization": "Bearer wrk", "x-mvcloud-lease": "tok"})
        self.assertEqual(json.loads(self.api.requests[-1]["body"]), {"worker_id": "w"})
        self.assertEqual(self.api.requests[-1]["headers"]["authorization"], "Bearer wrk")
        self.api.script["/v1/pull"] = [(401, {"error": "token"})]
        self.assertIsNone(self.control.pull({"worker_id": "w"}))  # logged, the agent polls again

    def test_heartbeat(self):
        lease = self.lease()
        for status, expected in [(200, HeartbeatResult.OK), (409, HeartbeatResult.LOST),
                                 (503, HeartbeatResult.TRANSIENT)]:
            self.api.script["/v1/heartbeat"] = [(status, {})]
            self.assertEqual(self.control.heartbeat(lease, {"line": "x"}), expected)
        body = json.loads(self.api.requests[-1]["body"])
        self.assertEqual(body, {"job_id": "j", "shard_id": "loc-0000", "lease_token": "tok",
                                "progress": {"line": "x"}})
        unreachable = HttpControl("http://127.0.0.1:9", "wrk")
        self.assertEqual(unreachable.heartbeat(lease, {}), HeartbeatResult.TRANSIENT)

    def test_complete_and_fail(self):
        lease = self.lease()
        self.api.script["/v1/complete"] = [(503, {}), (200, {"state": "done"})]
        self.control.complete(lease, {"outputs": []})  # retried through the 503
        self.assertEqual(sum(r["path"] == "/v1/complete" for r in self.api.requests), 2)
        for status in (409, 422):
            self.api.script["/v1/complete"] = [(status, {"reason": "x"})]
            with self.assertRaises(Conflict):
                self.control.complete(lease, {"outputs": []})
        self.api.script["/v1/fail"] = [(200, {"noop": True})]
        self.control.fail(lease, "exit 134", True, outcome="retry")
        self.assertEqual(json.loads(self.api.requests[-1]["body"]),
                         {"job_id": "j", "shard_id": "loc-0000", "lease_token": "tok",
                          "error": "exit 134", "retryable": True, "outcome": "retry"})

    def test_submit_uploads_by_md5_once(self):
        paths = support.make_scene(os.path.join(self.tmp.name, "scene"), n_frames=4)
        spec = support.make_spec(paths, n_frames=None)
        admin = HttpControl(self.base, "adm")
        self.api.script["/v1/jobs"] = [(201, {"job_id": "synthetic-1"})]
        self.assertEqual(admin.submit(spec), "synthetic-1")
        posted = json.loads(self.api.requests[-1]["body"])
        md5 = storage.file_md5(paths["manifest"])
        self.assertEqual(posted["manifest_key"], f"scenes/synthetic/{md5}/manifest.json")
        self.assertEqual([v["name"] for v in posted["videos"]], ["cam0.mp4"])
        for k in ("manifest", "params", "poses", "gt"):
            self.assertNotIn(k, posted)  # local paths stay local
        poses = self.api.blobs[posted["poses_key"]]
        self.assertEqual(hashlib.md5(poses).hexdigest(), posted["poses_key"].split("/")[2])
        puts = [r for r in self.api.requests if r["method"] == "PUT"]
        self.assertEqual(len(puts), 5)  # manifest, video, params, poses, gt
        self.assertTrue(all(r["headers"]["x-mvcloud-md5"] == r["path"].split("/")[5] for r in puts))

        self.assertRegex(posted["job_id"], r"^synthetic-\d{8}T\d{6}-[0-9a-f]{6}$")

        self.api.requests.clear()
        self.api.script["/v1/jobs"] = [(503, {}), (201, {"job_id": "synthetic-2"})]
        admin.submit(spec)  # the same files: nothing is uploaded again
        self.assertEqual([r for r in self.api.requests if r["method"] == "PUT"], [])
        posts = [json.loads(r["body"]) for r in self.api.requests if r["path"] == "/v1/jobs"]
        self.assertEqual(len(posts), 2)  # retried through the 503 with the same job id
        self.assertEqual(posts[0]["job_id"], posts[1]["job_id"])

        self.api.script["/v1/jobs"] = [(422, {"error": "at least 2 frames required"})]
        with self.assertRaisesRegex(ValueError, "at least 2 frames required"):
            admin.submit(spec)

    def test_tar_poses_is_deterministic(self):
        paths = support.make_scene(os.path.join(self.tmp.name, "scene"), n_frames=3, n_cameras=2)
        with open(os.path.join(paths["poses"], "README.txt"), "w") as f:
            f.write("not a pose")
        a, b = (os.path.join(self.tmp.name, f"{x}.tar") for x in "ab")
        self.assertEqual(tar_poses(paths["poses"], a), 6)
        os.utime(os.path.join(paths["poses"], "pose-2d_000000_00.json"), (0, 12345))
        tar_poses(paths["poses"], b)
        self.assertEqual(storage.file_md5(a), storage.file_md5(b))

    def test_needs_a_token(self):
        with self.assertRaises(ValueError):
            HttpControl(self.base, None)


if __name__ == "__main__":
    unittest.main()
