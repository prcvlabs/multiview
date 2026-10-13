#!/usr/bin/env python3
"""Compare the track_results of two pipeline-output.json files.

    compare_tracks.py [--keep-lab-p] [--json out.json] <a.json> <b.json>

By default fp_info.lab_p is nulled in both files before comparing: it is the
only output that depends on the per-sensor still, which a tracks job computed
from --load-loc-data accumulates over different frames than a batch run.
Exit status: 0 equal, 1 different, 2 error.
"""

import argparse
import json
import sys


def load_track_results(fname, keep_lab_p):
    with open(fname) as f:
        track_results = json.load(f)["track_results"]
    if not keep_lab_p:
        for track in track_results.get("tracks", []):
            if isinstance(track.get("fp_info"), dict):
                track["fp_info"]["lab_p"] = None
    return track_results


def describe_differences(a, b):
    out = []
    for key in sorted(set(a) | set(b)):
        if key != "tracks" and a.get(key) != b.get(key):
            out.append(f"track_results.{key}: {a.get(key)!r} != {b.get(key)!r}")
    ta, tb = a.get("tracks", []), b.get("tracks", [])
    if len(ta) != len(tb):
        out.append(f"number of tracks: {len(ta)} != {len(tb)}")
    for i, (x, y) in enumerate(zip(ta, tb)):
        for key in sorted(set(x) | set(y)):
            if x.get(key) == y.get(key):
                continue
            if key == "path":
                px, py = x.get("path", []), y.get("path", [])
                first = next((j for j, (u, v) in enumerate(zip(px, py)) if u != v),
                             min(len(px), len(py)))
                out.append(f"track {i} path: {len(px)} vs {len(py)} points, "
                           f"first difference at index {first}")
            else:
                out.append(f"track {i} {key}: {x.get(key)!r} != {y.get(key)!r}")
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("a")
    ap.add_argument("b")
    ap.add_argument("--keep-lab-p", action="store_true",
                    help="compare fp_info.lab_p too")
    ap.add_argument("--json", help="write the result to this file")
    args = ap.parse_args()

    try:
        a = load_track_results(args.a, args.keep_lab_p)
        b = load_track_results(args.b, args.keep_lab_p)
    except (OSError, KeyError, ValueError) as e:
        print(f"compare_tracks: {e}", file=sys.stderr)
        return 2

    diffs = [] if a == b else describe_differences(a, b)
    if a != b and not diffs:
        diffs = ["track_results differ"]
    result = {"a": args.a, "b": args.b, "equal": not diffs,
              "lab_p_nulled": not args.keep_lab_p,
              "n_tracks": [len(a.get("tracks", [])), len(b.get("tracks", []))],
              "differences": diffs}
    if args.json:
        with open(args.json, "w") as f:
            json.dump(result, f, indent=1)
    print(f"compare_tracks: {'EQUAL' if not diffs else 'DIFFERENT'} "
          f"({result['n_tracks'][0]} vs {result['n_tracks'][1]} tracks)")
    for line in diffs:
        print(f"   {line}")
    return 0 if not diffs else 1


if __name__ == "__main__":
    sys.exit(main())
