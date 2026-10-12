"""Activity segments from cached detections, for trimming long idle stretches.

A sampled frame is active when any camera sees a person (pose box above
`conf`) that is not static. Static boxes (mannequins, posters, people
printed on signage) are those whose quantised position recurs in more than
`static_frac` of the sampled frames. Active frames are joined across gaps
shorter than `merge` seconds and padded by `pad` seconds on each side.
Cameras of one scene are passed together so they are cut to the same
ranges and stay synchronised.
"""
import numpy as np

from .detect import Detections

k_cell = 48  # px, quantisation of box centres and sizes for the static test


def person_frames_(dets, conf, static_frac):
    """Per detected frame: True if a non-static person is seen in either eye."""
    times = sorted({t for t, _ in dets.pose_})
    keys = {}
    for (t, eye), (boxes, confs, _) in dets.pose_.items():
        for b, c in zip(boxes, confs):
            if c < conf:
                continue
            cx, cy, w, h = (b[0] + b[2]) / 2, (b[1] + b[3]) / 2, b[2] - b[0], b[3] - b[1]
            k = (eye, int(cx // k_cell), int(cy // k_cell), int(w // k_cell), int(h // k_cell))
            keys.setdefault(t, set()).add(k)
    n = max(1, dets.nframes // dets.stride)
    counts = {}
    for ks in keys.values():
        for k in ks:
            counts[k] = counts.get(k, 0) + 1
    static = {k for k, c in counts.items() if c > static_frac * n}
    active = np.zeros(dets.nframes, bool)
    for t, ks in keys.items():
        if ks - static:
            active[t] = True
    return active, len(static), times


def segments(dets_list, fps, conf=0.5, static_frac=0.5, pad=5.0, merge=20.0, min_active=1.0):
    """[(start, end)] frame ranges (end exclusive) shared by all cameras of a scene."""
    n = min(d.nframes for d in dets_list)
    stride = max(d.stride for d in dets_list)
    active = np.zeros(n, bool)
    for d in dets_list:
        a, _, _ = person_frames_(d, conf, static_frac)
        active |= a[:n]
    # a sampled frame stands for the `stride` frames that follow it
    idx = np.flatnonzero(active)
    if len(idx) == 0:
        return []
    runs, s, e = [], idx[0], idx[0] + stride
    for t in idx[1:]:
        if t - e < merge * fps:
            e = t + stride
        else:
            runs.append((s, e))
            s, e = t, t + stride
    runs.append((s, e))
    out = []
    for s, e in runs:
        if np.count_nonzero(active[s:e]) * stride < min_active * fps:
            continue  # isolated flicker, not a visit
        s, e = max(0, int(s - pad * fps)), min(n, int(e + pad * fps))
        s -= s % stride  # segments start on a detected frame
        if out and s <= out[-1][1]:
            out[-1] = (out[-1][0], e)
        else:
            out.append((s, e))
    return out


def summary(paths, fps, **kw):
    dets = [Detections.load(p) for p in paths]
    segs = segments(dets, fps, **kw)
    n = min(d.nframes for d in dets)
    kept = sum(e - s for s, e in segs)
    statics = [person_frames_(d, kw.get("conf", 0.5), kw.get("static_frac", 0.5))[1] for d in dets]
    return dict(frames=n, kept=kept, segments=[list(map(int, x)) for x in segs], static_boxes=statics)
