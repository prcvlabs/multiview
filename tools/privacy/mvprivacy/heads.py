"""Head regions from detections, and temporal gap filling.

The geometry is the one used for the first dataset release: a head ellipse
from COCO pose keypoints (face points, ear and eye spacing, shoulder width),
a second source from a face detector for small or distant faces, and a
greedy tracker that interpolates heads across missed or skipped frames.
A head is [cx, cy, r] in eye-local pixels; the ellipse is r x 1.12 r.
"""

import numpy as np

k_kp_conf = 0.25  # keypoint confidence threshold
k_gap = 12  # max frames to interpolate a head along a track
k_aspect = 1.12  # ellipse height / width


# ----------------------------------------------------------- heads_from_pose
def heads_from_pose(boxes, confs, kps):
    out = []
    for box, conf, kp in zip(boxes, confs, kps):
        x0, y0, x1, y1 = box
        bw, bh = x1 - x0, y1 - y0
        face = kp[:5]
        fv = face[face[:, 2] > k_kp_conf]
        ls, rs = kp[5], kp[6]
        sh_ok = ls[2] > k_kp_conf and rs[2] > k_kp_conf

        scales = []
        if sh_ok:
            scales.append(0.42 * np.hypot(*(ls[:2] - rs[:2])))
        if face[3, 2] > k_kp_conf and face[4, 2] > k_kp_conf:
            scales.append(0.75 * np.hypot(*(face[3, :2] - face[4, :2])))
        if face[1, 2] > k_kp_conf and face[2, 2] > k_kp_conf:
            scales.append(1.7 * np.hypot(*(face[1, :2] - face[2, :2])))
        bb = 0.17 * min(max(bw, bh), 2.2 * min(bw, bh) + 1)
        r = float(np.clip(np.median(scales), 0.6 * bb, 0.33 * max(bw, bh))) if scales else bb

        if len(fv):
            cx, cy = fv[:, 0].mean(), fv[:, 1].mean()
            if sh_ok:  # nudge toward the neck, the skull is behind the face plane
                mx, my = (ls[:2] + rs[:2]) / 2
                cx, cy = cx + 0.08 * (mx - cx), cy + 0.08 * (my - cy)
        elif sh_ok:
            mid = (ls[:2] + rs[:2]) / 2
            hips = kp[11:13]
            hv = hips[hips[:, 2] > k_kp_conf]
            if len(hv):
                d = mid - hv[:, :2].mean(0)
                d /= np.linalg.norm(d) + 1e-6
            else:
                d = np.array([0.0, -1.0])
            cx, cy = mid + d * 1.1 * r
        else:
            if conf < 0.25:
                continue
            r = 0.22 * min(bw, bh)
            cx, cy = (x0 + x1) / 2, y0 + r
        out.append([float(cx), float(cy), float(np.clip(r * 1.12 + 3, 8, 130))])
    return out


# ---------------------------------------------------------- heads_from_faces
def heads_from_faces(boxes, confs):
    out = []
    for (x0, y0, x1, y1), conf in zip(boxes, confs):
        w, h = x1 - x0, y1 - y0
        if conf < 0.2 or max(w, h) < 4 or max(w, h) > 120:
            continue
        r = float(np.clip(0.5 * max(w, h) * 1.4 + 3, 8, 75))
        out.append([float((x0 + x1) / 2), float((y0 + y1) / 2 - 0.08 * h), r])
    return out


# --------------------------------------------------------------- fill_tracks
def fill_tracks(per_frame, gap=k_gap):
    """per_frame: list over frames of lists of heads. Links heads greedily
    frame to frame and interpolates up to `gap` missing frames along each
    link. Returns a new per-frame list including the interpolated heads."""
    tracks = []  # [last_t, last_head]
    filled = [list(f) for f in per_frame]
    for t, hs in enumerate(per_frame):
        used = set()
        for h in hs:
            best, bd = None, 1e9
            for ti, (tt, th) in enumerate(tracks):
                if ti in used or t - tt > gap + 1:
                    continue
                d = np.hypot(h[0] - th[0], h[1] - th[1])
                lim = 1.2 * max(h[2], th[2]) + 6 * (t - tt)
                if d < lim and d < bd:
                    best, bd = ti, d
            if best is None:
                tracks.append([t, h])
                used.add(len(tracks) - 1)
                continue
            tt, th = tracks[best]
            g = t - tt
            for s in range(1, g):
                a = s / g
                filled[tt + s].append([th[i] * (1 - a) + h[i] * a for i in range(3)])
            tracks[best] = [t, h]
            used.add(best)
    return filled


# --------------------------------------------------------------- head_tracks
def head_tracks(dets, eye):
    """Per-frame gap-filled heads for one eye of a `Detections`."""
    per_frame = [
        heads_from_pose(*dets.pose(t, eye)) + heads_from_faces(*dets.faces(t, eye))
        for t in range(dets.nframes)
    ]
    return fill_tracks(per_frame, gap=max(k_gap, 2 * dets.stride))
