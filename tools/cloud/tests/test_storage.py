"""storage: verified, atomic fetches; puts with retries; multipart uploads
against a mock of the Worker's blob proxy."""

import hashlib
import http.server
import json
import os
import tempfile
import threading
import unittest
import urllib.parse

import support  # noqa: F401  (sys.path)
from mvcloud import storage


# ------------------------------------------------------------------- MockServer
class MockHandler(http.server.BaseHTTPRequestHandler):
    def log_message(self, *args):
        pass

    def reply(self, code, body=b"", headers=None):
        self.send_response(code)
        for k, v in (headers or {}).items():
            self.send_header(k, v)
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        if self.command != "HEAD":
            self.wfile.write(body)

    def read_body(self):
        n = int(self.headers.get("Content-Length", 0))
        h = hashlib.md5()
        size = 0
        while size < n:
            block = self.rfile.read(min(1 << 20, n - size))
            h.update(block)
            size += len(block)
        return size, h.hexdigest()

    def do_PUT(self):
        s = self.server
        u = urllib.parse.urlparse(self.path)
        q = dict(urllib.parse.parse_qsl(u.query))
        size, md5 = self.read_body()
        s.calls.append(("PUT", u.path, q, size))
        if u.path == "/flaky" and s.failures_left > 0:
            s.failures_left -= 1
            return self.reply(503)
        if u.path == "/part":
            part = int(q["part"])
            if part == s.fail_part:
                return self.reply(500)
            s.parts[part] = (size, md5)
            return self.reply(200, json.dumps({"etag": f"etag-{part}"}).encode())
        s.objects[u.path] = size
        self.reply(200)

    def do_POST(self):
        s = self.server
        u = urllib.parse.urlparse(self.path)
        n = int(self.headers.get("Content-Length", 0))
        body = self.rfile.read(n)
        s.calls.append(("POST", u.path, dict(urllib.parse.parse_qsl(u.query)), n))
        if u.path == "/init":
            return self.reply(200, json.dumps({"upload_id": "U1"}).encode())
        if u.path == "/complete":
            s.completed = json.loads(body)
            return self.reply(200)
        self.reply(404)

    def do_DELETE(self):
        self.server.calls.append(("DELETE", self.path, {}, 0))
        self.server.aborted = True
        self.reply(204)

    def do_GET(self):
        """Serves s.blobs; s.claimed[path] overrides the Content-Length, a
        body cut short when it is larger."""
        s = self.server
        path = urllib.parse.urlparse(self.path).path
        body = s.blobs.get(path)
        if body is None:
            return self.reply(404)
        self.send_response(200)
        self.send_header("Content-Length", str(s.claimed.get(path, len(body))))
        self.end_headers()
        self.wfile.write(body)
        self.close_connection = True

    def do_HEAD(self):
        size = self.server.objects.get(urllib.parse.urlparse(self.path).path)
        if size is None:
            return self.reply(404)
        self.send_response(200)
        self.send_header("Content-Length", str(size))
        self.end_headers()


def start_server():
    s = http.server.ThreadingHTTPServer(("127.0.0.1", 0), MockHandler)
    s.calls, s.parts, s.objects, s.blobs, s.claimed = [], {}, {}, {}, {}
    s.failures_left, s.fail_part, s.completed, s.aborted = 0, None, None, False
    threading.Thread(target=s.serve_forever, daemon=True).start()
    return s, f"http://127.0.0.1:{s.server_address[1]}"


class TestStorage(unittest.TestCase):
    def setUp(self):
        storage.k_backoff_s = 0.01
        self.tmp = tempfile.TemporaryDirectory()
        self.d = self.tmp.name
        self.src = os.path.join(self.d, "src.bin")
        with open(self.src, "wb") as f:
            f.write(os.urandom(300_000))
        self.md5 = storage.file_md5(self.src)

    def tearDown(self):
        self.tmp.cleanup()

    def test_fetch_verifies(self):
        dest = os.path.join(self.d, "out", "a.bin")
        storage.fetch(storage.file_url(self.src), dest, size=300_000, md5=self.md5)
        self.assertEqual(storage.file_md5(dest), self.md5)
        for kw in ({"size": 299_999}, {"md5": "0" * 32}):
            bad = os.path.join(self.d, "out", "b.bin")
            with self.subTest(kw=kw), self.assertRaises(storage.TransferError):
                storage.fetch(self.src, bad, **kw)
            self.assertFalse(os.path.exists(bad))
            self.assertFalse(os.path.exists(bad + ".part"))
        with self.assertRaises(storage.TransferError):
            storage.fetch(os.path.join(self.d, "missing"), os.path.join(self.d, "c.bin"))

    def test_fetch_cached(self):
        cache = os.path.join(self.d, "cache")
        a = storage.fetch_cached(self.src, cache, "x.bin", size=300_000, md5=self.md5)
        os.remove(self.src)  # a second fetch must come from the cache
        b = storage.fetch_cached(self.src, cache, "x.bin", size=300_000, md5=self.md5)
        self.assertEqual(a, b)
        self.assertEqual(storage.file_md5(b), self.md5)

    def test_fetch_http(self):
        s, base = start_server()
        with open(self.src, "rb") as f:
            s.blobs["/x"] = f.read()
        s.blobs["/cut"] = s.blobs["/x"][:100_000]
        s.claimed["/cut"] = 300_000
        out = os.path.join(self.d, "http")
        try:
            dest = os.path.join(out, "good.bin")
            storage.fetch(base + "/x", dest, size=300_000, md5=self.md5)
            self.assertEqual(storage.file_md5(dest), self.md5)
            for path, kw in [("/x", {"md5": "0" * 32}), ("/cut", {}),
                             ("/cut", {"size": 300_000, "md5": self.md5}), ("/nothing-there", {})]:
                bad = os.path.join(out, "bad.bin")
                with self.subTest(path=path, kw=kw), self.assertRaises(storage.TransferError):
                    storage.fetch(base + path, bad, **kw)
                self.assertFalse(os.path.exists(bad))
            self.assertEqual(os.listdir(out), ["good.bin"])  # no .part left behind
        finally:
            s.shutdown()

    def test_concurrent_fetches_of_one_file(self):
        cache = os.path.join(self.d, "cache")
        errors = []

        def fetch():
            try:
                storage.fetch_cached(self.src, cache, "x.bin", size=300_000, md5=self.md5)
            except storage.TransferError as e:
                errors.append(e)

        threads = [threading.Thread(target=fetch) for _ in range(8)]
        for t in threads:
            t.start()
        for t in threads:
            t.join()
        self.assertEqual(errors, [])
        dest = os.path.join(cache, self.md5[:2], self.md5)
        self.assertEqual(os.listdir(dest), ["x.bin"])
        self.assertEqual(storage.file_md5(os.path.join(dest, "x.bin")), self.md5)

    def test_put_file_url(self):
        dest = os.path.join(self.d, "blobs", "k", "v.bin")
        storage.put(storage.file_url(dest), self.src)
        self.assertEqual(storage.file_md5(dest), self.md5)
        self.assertFalse(os.path.exists(dest + ".tmp"))
        self.assertEqual(storage.head(storage.file_url(dest)), 300_000)
        self.assertIsNone(storage.head(storage.file_url(dest + ".no")))

    def test_put_retries(self):
        s, base = start_server()
        try:
            s.failures_left = 2
            storage.put(base + "/flaky", self.src)
            self.assertEqual([c[0] for c in s.calls], ["PUT"] * 3)
            self.assertEqual(s.objects["/flaky"], 300_000)
            self.assertEqual(storage.head(base + "/flaky"), 300_000)
            self.assertIsNone(storage.head(base + "/absent"))
        finally:
            s.shutdown()

    def spec(self, base):
        return {"init_url": base + "/init", "part_url": base + "/part", "abort_url": base + "/abort",
                "complete_url": base + "/complete", "part_bytes": 64 << 20}

    def test_put_multipart(self):
        big = os.path.join(self.d, "sparse.bin")
        with open(big, "wb") as f:
            f.truncate(200 << 20)
        s, base = start_server()
        try:
            storage.put_multipart(self.spec(base), big)
            sizes = [s.parts[i][0] for i in sorted(s.parts)]
            self.assertEqual(sizes, [64 << 20] * 3 + [8 << 20])
            self.assertEqual(s.completed, {"parts": [{"part": i, "etag": f"etag-{i}"} for i in range(1, 5)]})
            self.assertTrue(all(c[2].get("upload_id") == "U1" for c in s.calls if c[1] == "/part"))
            self.assertFalse(s.aborted)
        finally:
            s.shutdown()

    def test_put_multipart_aborts(self):
        s, base = start_server()
        try:
            s.fail_part = 2
            spec = {**self.spec(base), "part_bytes": 100_000}
            with self.assertRaises(storage.TransferError):
                storage.put_multipart(spec, self.src)
            self.assertTrue(s.aborted)
            self.assertIsNone(s.completed)
        finally:
            s.shutdown()


if __name__ == "__main__":
    unittest.main()
