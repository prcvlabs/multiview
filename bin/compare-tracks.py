#!/usr/bin/env python3
"""Compare the tracks of two pipeline runs, or check a real-time stream.

  compare-tracks.py --exact A.json B.json
      A and B are pipeline-output.json files (or bare track results). Exits 0
      only if both have the same header (frames, start frame, histogram) and
      the same tracks: the same ids, the same (x, y, t) points, and the same
      fp_info fields except lab_p (which depends on the still image). Values
      are compared as written to the files.

  compare-tracks.py --stream stream.ndjson --frames N --output OUT.json
                    [--removed K]
      Checks the NDJSON stream of `multiview-cli realtime` against the
      pipeline-output.json it wrote: header first and end last; every frame
      once, as `frame`, and once per revision (0 tracked, 1 revised) in
      `tracks`; no `dropped` or `window_error` line; end.frames == N; every
      `smoothed` point with t <= final_upto_t of a track in OUT equals OUT's
      point (to the precision OUT is written with), and every point of OUT
      is published as final. Reports the stream ids that are not in OUT
      (tracks that post-processing removed); with --removed, their number
      must be K.

  compare-tracks.py --revised stream.ndjson PRESMOOTH.json
      Scores the revised (revision 1) positions of the stream against the
      post-processed, unsmoothed result: on the ids in PRESMOOTH every point
      must match (Jaccard 1.0). Points of other ids are reported as the
      unconfirmed leak.
"""

import argparse
import json
import math
import sys

K_POS_TOL = 1e-4  # histogram cells; pipeline-output.json has 6 digits
K_GAZE_TOL = 1e-5  # radians


# ---------------------------------------------------------------- loading

def load_track_results(fname):
    with open(fname) as fp:
        data = json.load(fp)
    return data["track_results"] if "track_results" in data else data


def load_stream(fname):
    with open(fname) as fp:
        return [json.loads(line) for line in fp if line.strip()]


def is_nan(x):
    return x is None or (isinstance(x, float) and math.isnan(x))


def same_value(a, b):
    return (is_nan(a) and is_nan(b)) or a == b


def close(a, b, tol):
    if is_nan(a) or is_nan(b):
        return is_nan(a) and is_nan(b)
    return abs(a - b) <= tol


# ------------------------------------------------------------------ exact

HEADER_KEYS = ("frames", "start-frame", "width", "height", "hist-sz",
               "top-left", "frame-duration")


def compare_exact(fname_a, fname_b):
    a, b = load_track_results(fname_a), load_track_results(fname_b)
    errors = []

    for key in HEADER_KEYS:
        if a.get(key) != b.get(key):
            errors.append(f"header '{key}': {a.get(key)} != {b.get(key)}")

    tracks_a = {t["id"]: t for t in a["tracks"]}
    tracks_b = {t["id"]: t for t in b["tracks"]}
    if len(tracks_a) != len(a["tracks"]) or len(tracks_b) != len(b["tracks"]):
        errors.append("duplicate track ids")
    if set(tracks_a) != set(tracks_b):
        errors.append(f"ids differ: {sorted(tracks_a)} != {sorted(tracks_b)}")

    n_points = 0
    n_gaze_diffs = 0
    fp_keys = set()
    for tid in sorted(set(tracks_a) & set(tracks_b)):
        ta, tb = tracks_a[tid], tracks_b[tid]
        pa, pb = ta["path"], tb["path"]
        if len(pa) != len(pb):
            errors.append(f"track {tid}: {len(pa)} points != {len(pb)}")
        for u, v in zip(pa, pb):
            n_points += 1
            if u[0:3] != v[0:3]:
                errors.append(f"track {tid}: point {u[0:3]} != {v[0:3]}")
            if not same_value(u[3], v[3]):
                n_gaze_diffs += 1
        fa, fb = ta.get("fp_info", {}), tb.get("fp_info", {})
        keys = (set(fa) | set(fb)) - {"lab_p"}
        fp_keys |= keys
        for key in sorted(keys):
            if not same_value(fa.get(key), fb.get(key)):
                errors.append(f"track {tid}: fp_info.{key} "
                              f"{fa.get(key)} != {fb.get(key)}")

    lab_p_diffs = sum(
        1 for tid in set(tracks_a) & set(tracks_b)
        if not same_value(tracks_a[tid].get("fp_info", {}).get("lab_p"),
                          tracks_b[tid].get("fp_info", {}).get("lab_p")))

    print(f"A: {fname_a}")
    print(f"B: {fname_b}")
    print(f"tracks: {len(tracks_a)} / {len(tracks_b)}, points compared: "
          f"{n_points}, fp_info fields compared: {sorted(fp_keys)}")
    print(f"gaze differences (not gated): {n_gaze_diffs}; "
          f"lab_p differences (excluded): {lab_p_diffs}")
    for e in errors[:50]:
        print(f"   DIFFERS {e}")
    if len(errors) > 50:
        print(f"   ... and {len(errors) - 50} more")
    print("exact: IDENTICAL" if not errors else
          f"exact: {len(errors)} DIFFERENCES")
    return 0 if not errors else 1


# ----------------------------------------------------------------- stream

def output_points(results):
    """{id: {t: (x, y, gaze)}} in histogram cells"""
    out = {}
    for track in results["tracks"]:
        out[track["id"]] = {p[2]: (p[0], p[1], p[3]) for p in track["path"]}
    return out


def to_cells(header, x, y):
    tl_x, tl_y = header["top_left"]
    hist_sz = header["hist_sz"]
    return (x - tl_x) / hist_sz, (y - tl_y) / hist_sz


def check_stream(fname, n_frames, output_fname, n_removed):
    lines = load_stream(fname)
    errors = []

    if not lines or lines[0].get("type") != "header":
        errors.append("the first line is not a header")
    if not lines or lines[-1].get("type") != "end":
        errors.append("the last line is not an end")
    header = lines[0] if lines else {}
    start = header.get("start_frame", 0)
    end = lines[-1] if lines else {}
    if end.get("frames") != n_frames:
        errors.append(f"end.frames = {end.get('frames')}, not {n_frames}")

    by_type = {}
    for line in lines:
        by_type.setdefault(line["type"], []).append(line)

    frames = [m["frame"] for m in by_type.get("frame", [])]
    if sorted(frames) != list(range(start, start + n_frames)):
        errors.append(f"{len(frames)} `frame` lines, not frames "
                      f"[{start}, {start + n_frames}) once each")
    for kind in ("dropped", "window_error"):
        if by_type.get(kind):
            errors.append(f"{len(by_type[kind])} `{kind}` lines")

    revisions = {0: [], 1: []}
    stream_ids = set()
    for m in by_type.get("tracks", []):
        revisions.setdefault(m["revision"], []).append(m["frame"])
        if m["revision"] == 1:
            stream_ids |= {tp["id"] for tp in m["tracks"]}
    for rev in (0, 1):
        if sorted(revisions[rev]) != list(range(start, start + n_frames)):
            errors.append(f"revision {rev}: {len(revisions[rev])} `tracks` "
                          f"lines, not frames [{start}, {start + n_frames}) "
                          f"once each")

    results = load_track_results(output_fname)
    out = output_points(results)
    n_final = 0
    finals = {}
    for m in by_type.get("smoothed", []):
        stream_ids.add(m["id"])
        if m["id"] not in out:
            continue
        for x, y, t, gaze in m["points"]:
            if t > m["final_upto_t"]:
                continue
            n_final += 1
            if (m["id"], t) in finals:
                errors.append(f"track {m['id']} frame {t} final twice")
            finals[(m["id"], t)] = True
            ref = out[m["id"]].get(t)
            if ref is None:
                errors.append(f"track {m['id']} frame {t} is not in the output")
                continue
            cx, cy = to_cells(header, x, y)
            if not (close(cx, ref[0], K_POS_TOL) and close(cy, ref[1], K_POS_TOL)
                    and close(gaze, ref[2], K_GAZE_TOL)):
                errors.append(f"track {m['id']} frame {t}: smoothed "
                              f"({cx:.5f}, {cy:.5f}, {gaze}) != output {ref}")
    missing = [(tid, t) for tid, pts in out.items() for t in pts
               if (tid, t) not in finals]
    if missing:
        errors.append(f"{len(missing)} output points never published as "
                      f"final, e.g. {missing[:5]}")

    removed = sorted(stream_ids - set(out))
    if set(out) - stream_ids:
        errors.append(f"output ids not in the stream: "
                      f"{sorted(set(out) - stream_ids)}")
    if n_removed is not None and len(removed) != n_removed:
        errors.append(f"{len(removed)} stream ids are not in the output, "
                      f"not {n_removed}")

    stats = by_type.get("stats", [])
    final_ms = stats[-1].get("latency_ms", {}).get("final", {}) if stats else {}

    counts = ", ".join(f"{k} {len(v)}" for k, v in sorted(by_type.items()))
    print(f"stream: {fname}: {len(lines)} lines ({counts})")
    print(f"output: {output_fname}: {len(out)} tracks, "
          f"{sum(len(p) for p in out.values())} points; "
          f"{n_final} final smoothed points checked")
    print(f"stream ids not in the output (removed by post-processing): "
          f"{len(removed)} {removed}")
    if final_ms:
        print(f"final (smoothed) latency ms: p50 {final_ms.get('p50')}, "
              f"p95 {final_ms.get('p95')}, max {final_ms.get('max')}")
    for e in errors[:50]:
        print(f"   FAIL {e}")
    print("stream: OK" if not errors else f"stream: {len(errors)} FAILURES")
    return 0 if not errors else 1


# ---------------------------------------------------------------- revised

def check_revised(fname, presmooth_fname):
    lines = load_stream(fname)
    header = lines[0]
    ref = output_points(load_track_results(presmooth_fname))

    got = {}
    leak = 0
    total = 0
    for m in lines:
        if m.get("type") != "tracks" or m["revision"] != 1:
            continue
        for tp in m["tracks"]:
            total += 1
            if tp["id"] not in ref:
                leak += 1
                continue
            got[(tp["id"], m["frame"])] = to_cells(header, tp["x"], tp["y"])

    ref_keys = {(tid, t) for tid, pts in ref.items() for t in pts}
    matched = 0
    for key in ref_keys & set(got):
        r = ref[key[0]][key[1]]
        if close(got[key][0], r[0], K_POS_TOL) and \
           close(got[key][1], r[1], K_POS_TOL):
            matched += 1
    union = len(ref_keys | set(got))
    jaccard = matched / union if union else 1.0
    print(f"revised stream vs {presmooth_fname}: {len(got)} points on kept "
          f"ids, {len(ref_keys)} reference points, {matched} equal, "
          f"Jaccard {jaccard:.6f}")
    print(f"unconfirmed leak: {leak} of {total} revised points "
          f"({100.0 * leak / total if total else 0.0:.1f} %) belong to ids "
          f"that post-processing removed")
    return 0 if jaccard == 1.0 else 1


# ------------------------------------------------------------------- main

def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument("--exact", nargs=2, metavar=("A", "B"))
    ap.add_argument("--stream", metavar="NDJSON")
    ap.add_argument("--frames", type=int)
    ap.add_argument("--output", metavar="OUT")
    ap.add_argument("--removed", type=int)
    ap.add_argument("--revised", nargs=2, metavar=("NDJSON", "PRESMOOTH"))
    args = ap.parse_args()

    if args.exact:
        return compare_exact(*args.exact)
    if args.stream:
        if args.frames is None or args.output is None:
            ap.error("--stream needs --frames and --output")
        return check_stream(args.stream, args.frames, args.output,
                            args.removed)
    if args.revised:
        return check_revised(*args.revised)
    ap.error("one of --exact, --stream or --revised is required")


if __name__ == "__main__":
    sys.exit(main())
