#!/usr/bin/python3
"""A stand-in for multiview-cli in the agent tests: `pipeline`, `ldata
info|cat`, `score` and `scene_info`, with the products' shapes but none of
the work. Envelopes are json: {"start_frame", "n_frames", "frame_md5"}.

MVCLOUD_FAKE_MODE selects how `pipeline` behaves, for every run ("sleep")
or per kind ("loc=sleep,tracks=ok"):

  ok                   write the products and exit 0
  slow                 as ok, after MVCLOUD_FAKE_SECONDS (default 4) of progress lines
  abort                die of SIGABRT (an Expects on a pool thread, issue #13)
  fatal                print FATAL, exit 1
  unused               an unknown argument: exit 1, "aborting due to previous errors"
  savefail             exit 1, "failed to save localization data"
  no-output            exit 0 without products
  wrong-range          an envelope one frame off the requested range
  sleep                sleep forever
  progress-then-sleep  one progress line, then sleep forever
  fork                 start a grandchild (sleep) in the session, then sleep
  leave-child          as ok, leaving a grandchild (sleep) running in the session

MVCLOUD_FAKE_LOG: each pipeline argv is appended to it as a json line.
MVCLOUD_FAKE_PIDS: the pids of the run (and its grandchild) are appended.
"""

import hashlib
import json
import os
import signal
import subprocess
import sys
import time


def arg(argv, name, default=None):
    return argv[argv.index(name) + 1] if name in argv else default


def all_args(argv, name):
    return [argv[i + 1] for i, a in enumerate(argv) if a == name]


def mode_for(kind):
    spec = os.environ.get("MVCLOUD_FAKE_MODE", "ok")
    if "=" not in spec:
        return spec
    modes = dict(x.split("=", 1) for x in spec.split(","))
    return modes.get(kind, "ok")


def note_pid(pid):
    if os.environ.get("MVCLOUD_FAKE_PIDS"):
        with open(os.environ["MVCLOUD_FAKE_PIDS"], "a") as f:
            f.write(f"{pid}\n")


def envelope(start, n):
    md5s = [hashlib.md5(f"frame {t}".encode()).hexdigest() for t in range(start, start + n)]
    return {"start_frame": start, "n_frames": n, "frame_md5": md5s}


def pipeline(argv):
    kind = "tracks" if "--load-loc-data" in argv else "loc"
    mode = mode_for(kind)
    if os.environ.get("MVCLOUD_FAKE_LOG"):
        with open(os.environ["MVCLOUD_FAKE_LOG"], "a") as f:
            f.write(json.dumps({"kind": kind, "argv": argv}) + "\n")
    note_pid(os.getpid())
    out = arg(argv, "-d")
    start = int(arg(argv, "--start", 0))
    n = int(arg(argv, "--n-frames", 0))
    print(f"fake pipeline {kind} [{start}, {start + n}) mode {mode}", flush=True)

    if mode == "abort":
        os.kill(os.getpid(), signal.SIGABRT)
    if mode == "fatal":
        print("FATAL: Precondition failed: labs[i][j] != nullptr", flush=True)
        return 1
    if mode == "unused":
        print("WARN: Unused argument: '--bogus'\nERROR aborting due to previous errors...", flush=True)
        return 1
    if mode == "savefail":
        print("\x1b[31mERROR\x1b[0m failed to save localization data: frame 3 differs\nexiting",
              flush=True)
        return 1
    if mode == "progress-then-sleep":
        print(f"saving loc-data frames [{start:4d}-{start:4d}] - 1.0s per frame", flush=True)
    if mode in ("fork", "leave-child"):
        note_pid(subprocess.Popen(["sleep", "600"]).pid)
    if mode in ("sleep", "progress-then-sleep", "fork"):
        while True:
            time.sleep(60)
    if mode == "slow":
        t_end = time.time() + float(os.environ.get("MVCLOUD_FAKE_SECONDS", "4"))
        while time.time() < t_end:
            print(f"saving loc-data frames [{start:4d}-{start:4d}] - 1.0s per frame", flush=True)
            time.sleep(0.5)
    if mode == "no-output":
        return 0

    os.makedirs(out, exist_ok=True)
    with open(arg(argv, "-p")) as f:
        params = json.load(f)
    with open(os.path.join(out, "params.json"), "w") as f:
        json.dump({"config": {"out_dir": out}, **params}, f)
    if kind == "loc":
        env = envelope(start + (1 if mode == "wrong-range" else 0), n)
        with open(os.path.join(out, "localization.data"), "w") as f:
            json.dump(env, f)
        with open(os.path.join(out, "run-object.json"), "w") as f:
            json.dump({"args": argv}, f)
    else:
        print(f"Track frames {start:04d}-{start + n:04d}, amortized time 0.1s per frame", flush=True)
        with open(os.path.join(out, "pipeline-output.json"), "w") as f:
            json.dump({"track_results": {"tracks": [{"id": 0, "path": []}]}}, f)
    return 0


def ldata(argv):
    if argv[0] == "info":
        with open(argv[1]) as f:
            env = json.load(f)
        print(json.dumps({**env, "file": argv[1]}))
        return 0
    if argv[0] == "cat":
        keeps = [tuple(map(int, k.split(":"))) for k in all_args(argv, "--keep")]
        start, n = keeps[0][0], sum(k[1] for k in keeps)
        with open(arg(argv, "-o"), "w") as f:
            json.dump(envelope(start, n), f)
        return 0
    return 2


def main():
    argv = sys.argv[1:]
    cmd, rest = argv[0], argv[1:]
    if cmd == "pipeline":
        return pipeline(rest)
    if cmd == "ldata":
        return ldata(rest)
    if cmd == "score":
        with open(arg(rest, "--json"), "w") as f:
            json.dump({"tp": 1, "fp": 0, "fn": 0, "f1_jaccard": 1.0}, f)
        return 0
    if cmd == "scene_info":
        info = {"n_frames": int(os.environ.get("MVCLOUD_FAKE_FRAMES", "40")), "K": 10,
                "n_cameras": 1, "n_sensors": 2, "sensor_ids": ["S0", "S1"]}
        with open(arg(rest, "-o"), "w") as f:
            json.dump(info, f)
        return 0
    print(f"fake multiview-cli: unknown command '{cmd}'", file=sys.stderr)
    return 2


if __name__ == "__main__":
    sys.exit(main())
