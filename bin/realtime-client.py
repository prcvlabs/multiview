#!/usr/bin/env python3
"""Read the NDJSON stream of `multiview-cli realtime`, and summarise it.

  realtime-client.py SOURCE [--save FILE] [--print] [--max-seconds S]

SOURCE is a file, '-' for stdin, or the server's WebSocket URL
(ws://HOST:PORT/v1/stream, when realtime runs with --ws-port PORT). The
summary gives the number of lines of each type, the frames processed and
dropped, the provisional latency (`latency_ms` of the `frame` lines) over
the whole stream and over its first and second half, and the latencies of
the last `stats` line.

  --save FILE        Also write every line received to FILE.
  --print            Print every line to stdout; the summary goes to stderr.
  --max-seconds S    Stop reading after S seconds (0: at the end).

Standard library only: the WebSocket client is a minimal RFC 6455 client
(no TLS, no extensions) that answers pings and the server's close.
"""

import argparse
import base64
import hashlib
import json
import math
import os
import socket
import sys
import time
import urllib.parse

K_WS_GUID = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11"
K_OP_CONTINUATION = 0x0
K_OP_TEXT = 0x1
K_OP_BINARY = 0x2
K_OP_CLOSE = 0x8
K_OP_PING = 0x9
K_OP_PONG = 0xA


# -------------------------------------------------------------- websocket

def ws_accept_key(key):
    digest = hashlib.sha1((key + K_WS_GUID).encode()).digest()
    return base64.b64encode(digest).decode()


def encode_frame(opcode, payload, mask=None):
    """One final, masked frame (a client masks every frame)."""
    mask = os.urandom(4) if mask is None else mask
    n = len(payload)
    head = bytes([0x80 | opcode])
    if n < 126:
        head += bytes([0x80 | n])
    elif n < (1 << 16):
        head += bytes([0x80 | 126]) + n.to_bytes(2, "big")
    else:
        head += bytes([0x80 | 127]) + n.to_bytes(8, "big")
    body = bytes(b ^ mask[i % 4] for i, b in enumerate(payload))
    return head + mask + body


class WsConnection:
    """A connected WebSocket client: `messages()` yields text messages."""

    def __init__(self, sock, pending=b""):
        self.sock = sock
        self.buffer = pending
        self.is_closed = False

    def recv_exact(self, n, deadline):
        while len(self.buffer) < n:
            if deadline is not None and time.monotonic() > deadline:
                raise TimeoutError
            try:
                chunk = self.sock.recv(65536)
            except socket.timeout:
                continue
            if not chunk:
                raise ConnectionError("connection closed by the server")
            self.buffer += chunk
        out, self.buffer = self.buffer[:n], self.buffer[n:]
        return out

    def read_frame(self, deadline):
        b0, b1 = self.recv_exact(2, deadline)
        fin, opcode = bool(b0 & 0x80), b0 & 0x0F
        n = b1 & 0x7F
        if n == 126:
            n = int.from_bytes(self.recv_exact(2, deadline), "big")
        elif n == 127:
            n = int.from_bytes(self.recv_exact(8, deadline), "big")
        mask = self.recv_exact(4, deadline) if b1 & 0x80 else None
        payload = self.recv_exact(n, deadline)
        if mask is not None:
            payload = bytes(b ^ mask[i % 4] for i, b in enumerate(payload))
        return fin, opcode, payload

    def send(self, opcode, payload=b""):
        self.sock.sendall(encode_frame(opcode, payload))

    def close(self):
        if not self.is_closed:
            self.is_closed = True
            try:
                self.send(K_OP_CLOSE, (1000).to_bytes(2, "big"))
            except OSError:
                pass

    def messages(self, max_seconds=0.0):
        deadline = time.monotonic() + max_seconds if max_seconds > 0 else None
        parts = []
        while True:
            try:
                fin, opcode, payload = self.read_frame(deadline)
            except TimeoutError:
                self.close()
                return
            except OSError as e:  # closed without a close frame
                print(f"realtime-client: {e}", file=sys.stderr)
                self.is_closed = True
                return
            if opcode == K_OP_PING:
                self.send(K_OP_PONG, payload)
            elif opcode == K_OP_CLOSE:
                self.close()  # the reply
                return
            elif opcode in (K_OP_TEXT, K_OP_BINARY, K_OP_CONTINUATION):
                parts.append(payload)
                if fin:
                    yield b"".join(parts).decode()
                    parts = []


def ws_connect(url, timeout=10.0):
    u = urllib.parse.urlsplit(url)
    if u.scheme != "ws":
        raise ValueError(f"only ws:// URLs are supported, not '{url}'")
    port = u.port or 80
    sock = socket.create_connection((u.hostname, port), timeout=timeout)
    key = base64.b64encode(os.urandom(16)).decode()
    request = (f"GET {u.path or '/'} HTTP/1.1\r\n"
               f"Host: {u.hostname}:{port}\r\n"
               "Upgrade: websocket\r\n"
               "Connection: Upgrade\r\n"
               f"Sec-WebSocket-Key: {key}\r\n"
               "Sec-WebSocket-Version: 13\r\n\r\n")
    sock.sendall(request.encode())

    response = b""
    while b"\r\n\r\n" not in response:
        chunk = sock.recv(4096)
        if not chunk:
            raise ConnectionError("no handshake response")
        response += chunk
    head, pending = response.split(b"\r\n\r\n", 1)
    lines = head.decode("latin-1").split("\r\n")
    if len(lines[0].split()) < 2 or lines[0].split()[1] != "101":
        raise ConnectionError(f"handshake refused: {lines[0]}")
    fields = {}
    for line in lines[1:]:
        name, _, value = line.partition(":")
        fields[name.strip().lower()] = value.strip()
    if fields.get("sec-websocket-accept") != ws_accept_key(key):
        raise ConnectionError("bad Sec-WebSocket-Accept")
    sock.settimeout(0.5)
    return WsConnection(sock, pending)


# ---------------------------------------------------------------- reading

def file_lines(source, max_seconds):
    deadline = time.monotonic() + max_seconds if max_seconds > 0 else None
    fp = sys.stdin if source == "-" else open(source)
    try:
        for line in fp:
            if line.strip():
                yield line.rstrip("\n")
            if deadline is not None and time.monotonic() > deadline:
                return
    finally:
        if fp is not sys.stdin:
            fp.close()


# ---------------------------------------------------------------- summary

def percentile(xs, p):
    """Nearest rank, as calc_percentile in perceive/utils/math.hpp."""
    if not xs:
        return math.nan
    xs = sorted(xs)
    rank = math.ceil(min(max(p, 0.0), 100.0) * len(xs) / 100.0)
    return xs[max(1, rank) - 1]


def latency_text(xs):
    if not xs:
        return "n 0"
    return (f"n {len(xs)}, p50 {percentile(xs, 50):.1f}, "
            f"p95 {percentile(xs, 95):.1f}, max {max(xs):.1f} ms")


class Summary:
    def __init__(self):
        self.counts = {}
        self.latencies = []  # `frame` lines, in stream order
        self.dropped = 0
        self.last_stats = None
        self.end = None
        self.n_bad = 0

    def add(self, line):
        try:
            o = json.loads(line)
        except ValueError:
            self.n_bad += 1
            return
        kind = o.get("type", "?")
        self.counts[kind] = self.counts.get(kind, 0) + 1
        if kind == "frame" and o.get("latency_ms") is not None:
            self.latencies.append(float(o["latency_ms"]))
        elif kind == "dropped":
            self.dropped += 1
        elif kind == "stats":
            self.last_stats = o
        elif kind == "end":
            self.end = o

    def text(self):
        out = []
        n_lines = sum(self.counts.values())
        types = ", ".join(f"{k} {v}" for k, v in sorted(self.counts.items()))
        out.append(f"lines: {n_lines} ({types})"
                   + (f"; {self.n_bad} not JSON" if self.n_bad else ""))
        out.append(f"frames: {len(self.latencies)} provisional, "
                   f"{self.dropped} dropped")
        half = len(self.latencies) // 2
        out.append(f"provisional latency: {latency_text(self.latencies)}")
        out.append(f"  first half:  {latency_text(self.latencies[:half])}")
        out.append(f"  second half: {latency_text(self.latencies[half:])}")
        if self.last_stats is not None:
            for status, o in self.last_stats.get("latency_ms", {}).items():
                out.append(f"last stats, {status} latency: n {o.get('n')}, "
                           f"p50 {o.get('p50')}, p95 {o.get('p95')}, "
                           f"max {o.get('max')} ms")
            out.append(f"last stats, drops: {self.last_stats.get('drops')}")
        if self.end is not None:
            out.append(f"end: frames {self.end.get('frames')}, "
                       f"dropped {self.end.get('dropped')}")
        else:
            out.append("end: not received")
        return "\n".join(out)


# ------------------------------------------------------------------- main

def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument("source", metavar="SOURCE")
    ap.add_argument("--save", metavar="FILE")
    ap.add_argument("--print", action="store_true")
    ap.add_argument("--max-seconds", type=float, default=0.0)
    args = ap.parse_args()

    connection = None
    if args.source.startswith("ws://"):
        connection = ws_connect(args.source)
        lines = connection.messages(args.max_seconds)
    else:
        lines = file_lines(args.source, args.max_seconds)

    summary = Summary()
    save = open(args.save, "w") if args.save else None
    try:
        for line in lines:
            summary.add(line)
            if save:
                save.write(line + "\n")
            if args.print:
                print(line, flush=True)
    except KeyboardInterrupt:
        if connection is not None:
            connection.close()
    finally:
        if save:
            save.close()

    print(summary.text(), file=sys.stderr if args.print else sys.stdout)
    return 0 if summary.end is not None else 1


if __name__ == "__main__":
    sys.exit(main())
