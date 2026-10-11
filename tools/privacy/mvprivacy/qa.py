"""Checks: container/timing identity of outputs, and the sparse-detection
miss rate.
"""

import json
import subprocess

import cv2
import numpy as np

from .heads import heads_from_faces, heads_from_pose, k_aspect
from .masks import MaskRenderer

k_same = ("width", "height", "r_frame_rate", "avg_frame_rate", "time_base", "pix_fmt",
          "nb_frames", "start_time", "duration")


# ------------------------------------------------------------- stream_info
def stream_info(path):
    s = json.loads(subprocess.check_output(
        ["ffprobe", "-v", "error", "-select_streams", "v:0", "-show_entries",
         "stream=codec_name,profile," + ",".join(k_same) + ":format=format_name",
         "-of", "json", path]))
    pts = subprocess.check_output(
        ["ffprobe", "-v", "error", "-select_streams", "v:0", "-show_entries", "frame=pts", "-of", "csv=p=0", path])
    return s["streams"][0], s["format"], [int(x.strip(b",")) for x in pts.split() if x.strip(b",")]


# ------------------------------------------------------------ verify_format
def verify_format(orig, out):
    """Differences between `out` and `orig` that would break the dataset's
    contract: same geometry, rate, time base, frame count and per-frame pts,
    same container, H.264."""
    sa, fa, pa = stream_info(orig)
    sb, fb, pb = stream_info(out)
    errs = [f"{k}: {sa.get(k)} != {sb.get(k)}" for k in k_same if sa.get(k) != sb.get(k)]
    if fa["format_name"] != fb["format_name"]:
        errs.append(f"container: {fa['format_name']} != {fb['format_name']}")
    if pa != pb:
        errs.append(f"frame pts differ ({len(pa)} vs {len(pb)} frames)")
    if sb["codec_name"] != "h264":
        errs.append(f"codec {sb['codec_name']}")
    return errs


# --------------------------------------------------------------- sparse_eval
def sparse_eval(dets, height, strides=(1, 2, 4, 8), window=3, cover=0.9):
    """`dets` must come from a stride-1 run. Every head found at full rate
    is a reference; a reference is missed at stride N when the masks built
    from every N-th frame cover less than `cover` of its ellipse. Also
    reports the masked area relative to stride 1 (over-masking cost)."""
    if dets.stride != 1:
        raise ValueError("sparse_eval needs detections from a stride-1 run")
    refs = []
    for t in range(dets.nframes):
        for e in range(2):
            for h in heads_from_pose(*dets.pose(t, e)) + heads_from_faces(*dets.faces(t, e)):
                refs.append((t, e, h))
    by_t = {}
    for t, e, h in refs:
        by_t.setdefault(t, []).append((e, h))

    rows = []
    area1 = None
    for n in strides:
        r = MaskRenderer(dets.subsample(n), height, window=window)
        missed = 0
        area = 0
        for t in range(dets.nframes):
            m = r.frame(t)
            area += int(np.count_nonzero(m))
            for e, (cx, cy, rad) in by_t.get(t, []):
                ax, ay = int(round(rad)), int(round(rad * k_aspect))
                x0, y0 = max(0, int(cx) - ax - 1), max(0, int(cy) - ay - 1)
                roi = m[e, y0 : int(cy) + ay + 2, x0 : int(cx) + ax + 2]
                if roi.size == 0:
                    continue
                ref = np.zeros_like(roi)
                cv2.ellipse(ref, (int(round(cx)) - x0, int(round(cy)) - y0), (ax, ay), 0, 0, 360, 255, -1)
                inside = np.count_nonzero(ref)
                if inside and np.count_nonzero(ref & roi) < cover * inside:
                    missed += 1
        area1 = area1 or area
        rows.append(dict(stride=n, references=len(refs), missed=missed,
                         miss_rate=round(missed / max(1, len(refs)), 5), mask_area=round(area / max(1, area1), 3)))
        print(json.dumps(rows[-1]), flush=True)
    return rows
