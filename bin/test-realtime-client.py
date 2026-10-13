#!/usr/bin/env python3
"""Tests for realtime-client.py.

  bin/test-realtime-client.py

Summarises a synthetic stream file, and reads a stream from a fake WebSocket
server that sends a ping, a fragmented message and a close, checking the
client's pong and close reply.
"""

import base64
import hashlib
import importlib.util
import json
import os
import socket
import subprocess
import sys
import tempfile
import threading
import unittest

K_SCRIPT = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        "realtime-client.py")
K_WS_GUID = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11"


# ---------------------------------------------------------------- helpers

def load_client():
    spec = importlib.util.spec_from_file_location("realtime_client", K_SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def server_frame(opcode, payload, fin=True):
    """An unmasked frame, as a server sends it."""
    n = len(payload)
    head = bytes([(0x80 if fin else 0) | opcode])
    if n < 126:
        head += bytes([n])
    else:
        head += bytes([126]) + n.to_bytes(2, "big")
    return head + payload


def read_client_frame(conn):
    """(opcode, payload) of one masked client frame."""
    b0, b1 = conn.recv(2, socket.MSG_WAITALL)
    n = b1 & 0x7F
    if n == 126:
        n = int.from_bytes(conn.recv(2, socket.MSG_WAITALL), "big")
    mask = conn.recv(4, socket.MSG_WAITALL)
    payload = conn.recv(n, socket.MSG_WAITALL) if n else b""
    assert b1 & 0x80, "client frames are masked"
    return b0 & 0x0F, bytes(b ^ mask[i % 4] for i, b in enumerate(payload))


def fake_server(listener, received):
    conn, _ = listener.accept()
    request = b""
    while b"\r\n\r\n" not in request:
        request += conn.recv(4096)
    key = [line.split(":", 1)[1].strip()
           for line in request.decode().split("\r\n")
           if line.lower().startswith("sec-websocket-key:")][0]
    accept = base64.b64encode(
        hashlib.sha1((key + K_WS_GUID).encode()).digest()).decode()
    conn.sendall(("HTTP/1.1 101 Switching Protocols\r\n"
                  "Upgrade: websocket\r\nConnection: Upgrade\r\n"
                  f"Sec-WebSocket-Accept: {accept}\r\n\r\n").encode())
    long_line = json.dumps({"type": "stats", "pad": "x" * 300}).encode()
    conn.sendall(server_frame(0x1, b'{"type":"header"}')
                 + server_frame(0x9, b"ping!")
                 + server_frame(0x1, long_line))
    received.append(read_client_frame(conn))  # the pong
    end = b'{"type":"end","frames":1,"dropped":0}'
    conn.sendall(server_frame(0x1, end[:10], fin=False)
                 + server_frame(0x0, end[10:])
                 + server_frame(0x8, (1001).to_bytes(2, "big")))
    received.append(read_client_frame(conn))  # the close reply
    conn.close()


# ------------------------------------------------------------------ tests

class TestRealtimeClient(unittest.TestCase):

    def test_summary_of_a_file(self):
        lines = [{"type": "header"},
                 {"type": "frame", "frame": 0, "latency_ms": 10.0},
                 {"type": "frame", "frame": 1, "latency_ms": 20.0},
                 {"type": "dropped", "frame": 2, "reason": "no_free_slot"},
                 {"type": "frame", "frame": 3, "latency_ms": 40.0},
                 {"type": "stats", "latency_ms": {"provisional": {
                     "n": 3, "p50": 20.0, "p95": 40.0, "max": 40.0}}},
                 {"type": "end", "frames": 4, "dropped": 1}]
        with tempfile.TemporaryDirectory() as tmp:
            fname = os.path.join(tmp, "stream.ndjson")
            with open(fname, "w") as fp:
                fp.write("".join(json.dumps(o) + "\n" for o in lines))
            p = subprocess.run([sys.executable, K_SCRIPT, fname],
                               capture_output=True, text=True)
        self.assertEqual(p.returncode, 0, p.stderr)
        self.assertIn("lines: 7 (dropped 1, end 1, frame 3, header 1, "
                      "stats 1)", p.stdout)
        self.assertIn("frames: 3 provisional, 1 dropped", p.stdout)
        self.assertIn("provisional latency: n 3, p50 20.0, p95 40.0, "
                      "max 40.0 ms", p.stdout)
        self.assertIn("end: frames 4, dropped 1", p.stdout)

    def test_no_end_fails(self):
        p = subprocess.run([sys.executable, K_SCRIPT, "-"],
                           input='{"type":"header"}\n',
                           capture_output=True, text=True)
        self.assertEqual(p.returncode, 1)
        self.assertIn("end: not received", p.stdout)

    def test_websocket(self):
        client = load_client()
        listener = socket.socket()
        listener.bind(("127.0.0.1", 0))
        listener.listen(1)
        received = []
        server = threading.Thread(target=fake_server,
                                  args=(listener, received))
        server.start()
        port = listener.getsockname()[1]
        connection = client.ws_connect(f"ws://127.0.0.1:{port}/v1/stream")
        messages = list(connection.messages(max_seconds=10.0))
        server.join()
        listener.close()

        self.assertEqual(len(messages), 3)
        self.assertEqual(messages[0], '{"type":"header"}')
        self.assertEqual(json.loads(messages[1])["type"], "stats")
        self.assertEqual(json.loads(messages[2])["type"], "end")
        self.assertEqual(received[0], (0xA, b"ping!"))
        self.assertEqual(received[1][0], 0x8)

    def test_percentile_is_nearest_rank(self):
        client = load_client()
        xs = list(range(1, 1001))
        self.assertEqual(client.percentile(xs, 50), 500)
        self.assertEqual(client.percentile(xs, 95), 950)
        self.assertEqual(client.percentile(xs, 100), 1000)


if __name__ == "__main__":
    unittest.main()
