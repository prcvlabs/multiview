#!/usr/bin/env python3
"""Tests for pose-replay-remap.py.

  bin/test-pose-replay-remap.py

Remaps a synthetic two-frame replay set, and checks the file names, the
rewritten "frame-no", and the exit code when a source file is missing.
"""

import json
import os
import subprocess
import sys
import tempfile
import unittest

K_SCRIPT = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        "pose-replay-remap.py")


# ---------------------------------------------------------------- helpers

def write_json(fname, o):
    with open(fname, "w") as fp:
        json.dump(o, fp)


def read_json(fname):
    with open(fname) as fp:
        return json.load(fp)


def make_replay(src, movie_frames, sensors):
    """One file per movie frame and sensor, each with two poses."""
    os.makedirs(src, exist_ok=True)
    for m in movie_frames:
        for s in sensors:
            poses = [{"frame-no": m, "sensor-no": s, "id": i} for i in range(2)]
            write_json(os.path.join(src, f"pose-2d_{m:06d}_{s:02d}.json"), poses)


def run_remap(mapping, src, dst):
    return subprocess.run([sys.executable, K_SCRIPT, mapping, src, dst],
                          capture_output=True, text=True)


# ------------------------------------------------------------------ tests

class PoseReplayRemapTest(unittest.TestCase):

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.src = os.path.join(self.tmp.name, "src")
        self.dst = os.path.join(self.tmp.name, "dst")
        self.mapping = os.path.join(self.tmp.name, "map.txt")

    def tearDown(self):
        self.tmp.cleanup()

    def test_two_frames(self):
        # Scene frames 0 and 1 show movie frames 0 and 2
        make_replay(self.src, [0, 1, 2], [0, 1])
        with open(self.mapping, "w") as fp:
            fp.write("# k movie-frame\n0 0\n1 2\n")
        result = run_remap(self.mapping, self.src, self.dst)
        self.assertEqual(result.returncode, 0, result.stderr)

        self.assertEqual(sorted(os.listdir(self.dst)),
                         ["pose-2d_000000_00.json", "pose-2d_000000_01.json",
                          "pose-2d_000001_00.json", "pose-2d_000001_01.json"])
        for k, m in [(0, 0), (1, 2)]:
            for s in [0, 1]:
                poses = read_json(
                    os.path.join(self.dst, f"pose-2d_{k:06d}_{s:02d}.json"))
                self.assertEqual(len(poses), 2)
                for i, pose in enumerate(poses):
                    self.assertEqual(pose["frame-no"], k)  # rewritten
                    self.assertEqual(pose["sensor-no"], s)
                    self.assertEqual(pose["id"], i)
        # The source is left as it was
        self.assertEqual(read_json(os.path.join(
            self.src, "pose-2d_000002_00.json"))[0]["frame-no"], 2)

    def test_missing_source_file(self):
        make_replay(self.src, [0, 1], [0])
        with open(self.mapping, "w") as fp:
            fp.write("0 0\n1 3\n")  # movie frame 3 has no file
        result = run_remap(self.mapping, self.src, self.dst)
        self.assertEqual(result.returncode, 1)
        self.assertIn("1 source files missing", result.stderr)
        self.assertEqual(os.listdir(self.dst), ["pose-2d_000000_00.json"])


if __name__ == "__main__":
    unittest.main()
