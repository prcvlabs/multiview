#!/usr/bin/env python3
"""Rename pose replay files for a run at a lower --target-fps.

  pose-replay-remap.py MAPPING SRC DST

Pose replay files are named by scene frame (pose-2d_<frame:06d>_<sensor:02d>.json).
A replay set recorded at the videos' rate cannot be replayed at a lower
--target-fps as it is: scene frame k then shows movie frame m(k). MAPPING is
the output of

  multiview-cli realtime -m MANIFEST --target-fps F --print-frame-mapping

(lines `k m`, `#` comments), and DST receives, for every k and every sensor,
SRC/pose-2d_<m(k)>_<s>.json as DST/pose-2d_<k>_<s>.json, with each pose's
"frame-no" set to k (the tracker looks frames up by it). Exits 1 if a source
file is missing.
"""

import argparse
import json
import os
import re
import sys

K_POSE_FILE = re.compile(r"^pose-2d_(\d{6})_(\d{2})\.json$")


# ---------------------------------------------------------------- loading

def load_mapping(fname):
    mapping = []
    with open(fname) as fp:
        for line in fp:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            k, m = (int(x) for x in line.split())
            if k != len(mapping):
                raise ValueError(f"{fname}: expected scene frame {len(mapping)}, got {k}")
            mapping.append(m)
    return mapping


def sensors_of(src):
    sensors = set()
    for name in os.listdir(src):
        match = K_POSE_FILE.match(name)
        if match:
            sensors.add(int(match.group(2)))
    return sorted(sensors)


# ------------------------------------------------------------------- main

def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument("mapping")
    ap.add_argument("src")
    ap.add_argument("dst")
    args = ap.parse_args()

    mapping = load_mapping(args.mapping)
    sensors = sensors_of(args.src)
    if not sensors:
        print(f"no pose-2d_*.json files in {args.src}", file=sys.stderr)
        return 1
    os.makedirs(args.dst, exist_ok=True)

    n_written = 0
    missing = []
    for k, m in enumerate(mapping):
        for s in sensors:
            src = os.path.join(args.src, f"pose-2d_{m:06d}_{s:02d}.json")
            dst = os.path.join(args.dst, f"pose-2d_{k:06d}_{s:02d}.json")
            if not os.path.isfile(src):
                missing.append(src)
                continue
            with open(src) as fp:
                poses = json.load(fp)
            for pose in poses:
                pose["frame-no"] = k
            if os.path.lexists(dst):
                os.remove(dst)
            with open(dst, "w") as fp:
                json.dump(poses, fp)
            n_written += 1

    print(f"{n_written} files for {len(mapping)} scene frames and {len(sensors)} sensors in {args.dst}")
    if missing:
        print(f"{len(missing)} source files missing, e.g. {missing[0]}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
