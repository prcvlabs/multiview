"""Pass 2 (render) and the two-pass job runner.

  decode thread -> GPU upload -> mask op on YUV planes -> download -> encode thread
                    masks rasterised by a CPU thread pool, one batch ahead

Detections are cached next to the outputs, so trying another treatment is
one decode and encode per video.
"""

import json
import os
import time
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

import numpy as np
import torch

from .detect import Detections, Detector, detect_video, upload_batch
from .masks import Masker, MaskRenderer, load_static_boxes
from .video import Reader, Writer, probe


# ------------------------------------------------------------- render_video
def render_video(info, dets, dst, renderer, masker, encoder="x264", quality=16,
                 hwaccel=True, batch=16, workers=6, timings=None):
    """Masks every frame of `info` and encodes it to `dst`. Returns the
    number of frames written."""
    if dets.nframes != info.nframes:
        raise RuntimeError(f"{info.path}: {info.nframes} frames, detections cover {dets.nframes}")
    tm = timings if timings is not None else {}
    add = lambda k, dt: tm.__setitem__(k, tm.get(k, 0.0) + dt)
    device = masker.device
    tmp = str(dst) + ".tmp.mp4"
    Path(dst).parent.mkdir(parents=True, exist_ok=True)

    reader = Reader(info, hwaccel=hwaccel)
    writer = Writer(info, tmp, encoder=encoder, quality=quality)
    pool = ThreadPoolExecutor(workers)
    submit = lambda t0, n: [pool.submit(renderer.frame, t) for t in range(t0, min(t0 + n, info.nframes))]
    t = 0
    pending = submit(0, batch)
    try:
        for bufs in reader.batches(batch):
            t0 = time.perf_counter()
            masks = np.stack([f.result() for f in pending[: len(bufs)]])
            pending = submit(t + len(bufs), batch)
            t1 = time.perf_counter()
            y, u, v = upload_batch(bufs, info, device)
            m = torch.from_numpy(masks).to(device, non_blocking=True)
            y, u, v = masker.apply(y, u, v, m)
            out = torch.cat([y.flatten(1), u.flatten(1), v.flatten(1)], 1).cpu().numpy()
            t2 = time.perf_counter()
            for row in out:
                writer.write(row)
            add("rasterise", t1 - t0)
            add("mask", t2 - t1)
            t += len(bufs)
    finally:
        pool.shutdown(cancel_futures=True)
        rc_dec = reader.close()
        rc_enc = writer.close()
    if rc_dec != 0 or rc_enc != 0 or t != info.nframes:
        raise RuntimeError(f"{info.path}: wrote {t} of {info.nframes} frames (decode rc {rc_dec}, encode rc {rc_enc})")
    os.replace(tmp, dst)
    return t


# ---------------------------------------------------------------- run_jobs
def run_jobs(jobs, args):
    """jobs: list of {"src", "dst"[, "dets"][, "static": path][, "static_key"]}.
    Skips finished outputs; re-uses cached detections."""
    detector = None
    for j in jobs:
        if os.path.exists(j["dst"]) and not args.force:
            continue
        info = probe(j["src"])
        dets_path = Path(j.get("dets") or Path(j["dst"]).with_suffix(".dets.npz"))
        t0 = time.perf_counter()
        if dets_path.exists():
            dets = Detections.load(dets_path)
        else:
            if detector is None:
                detector = Detector(args.pose, args.face, imgsz=args.imgsz, batch=args.batch,
                                    engine=args.engine, eye_size=(info.width // 2, info.height))
            dets = detect_video(info, detector, stride=args.stride, hwaccel=args.hwaccel)
            dets_path.parent.mkdir(parents=True, exist_ok=True)
            dets.save(dets_path)
        t1 = time.perf_counter()
        static = load_static_boxes(j.get("static") or args.static, j.get("static_key"))
        renderer = MaskRenderer(dets, info.height, region=args.region, window=args.window)
        masker = Masker(info, op=args.op, pix=args.pix, sigma=args.sigma, static=static)
        n = render_video(info, dets, j["dst"], renderer, masker, encoder=args.encoder,
                         quality=args.quality, hwaccel=args.hwaccel)
        t2 = time.perf_counter()
        print(json.dumps(dict(dst=j["dst"], frames=n, detect_s=round(t1 - t0, 1), render_s=round(t2 - t1, 1),
                              fps=round(n / max(t2 - t0, 1e-6), 1), static=bool(static))), flush=True)
