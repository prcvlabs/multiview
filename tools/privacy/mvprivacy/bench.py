"""Per-stage benchmark on one clip: decode, detect, mask, encode.

Each stage runs on its own over the same frames held in memory, so the
numbers say where the time goes; GPU, NVDEC and NVENC utilisation are
sampled with nvidia-smi while each stage runs.
"""

import json
import os
import subprocess
import tempfile
import time

import numpy as np
import torch

from .detect import Detections, Detector, upload_batch
from .masks import Masker, MaskRenderer
from .video import Reader, Writer, probe


# ---------------------------------------------------------------- GpuSampler
class GpuSampler:
    """Mean GPU / decoder / encoder utilisation while the block runs."""

    def __enter__(self):
        self.proc_ = subprocess.Popen(
            ["nvidia-smi", "--query-gpu=utilization.gpu,utilization.decoder,utilization.encoder",
             "--format=csv,noheader,nounits", "-lms", "200", "-i", "0"],
            stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True)
        return self

    def __exit__(self, *exc):
        self.proc_.terminate()
        out, _ = self.proc_.communicate()
        rows = [[float(x) for x in l.split(",")] for l in out.splitlines() if l.strip()]
        a = np.mean(rows, 0) if rows else [float("nan")] * 3
        self.util = dict(gpu=round(a[0]), nvdec=round(a[1]), nvenc=round(a[2]))
        return False


# ---------------------------------------------------------------------- stage
def stage(results, name, n, fn):
    torch.cuda.synchronize()
    with GpuSampler() as g:
        t0 = time.perf_counter()
        out = fn()
        torch.cuda.synchronize()
        dt = time.perf_counter() - t0
    results.append(dict(stage=name, frames=n, seconds=round(dt, 3), fps=round(n / dt, 1), **g.util))
    print(json.dumps(results[-1]), flush=True)
    return out


# ---------------------------------------------------------------------- bench
def bench(path, frames=900, batch=16, imgsizes=(1280, 640), engine=False, pose="yolo11x-pose.pt",
          face="yolov11l-face.pt", encoders=("nvenc", "x264")):
    info = probe(path)
    n = min(frames, info.nframes) if frames else info.nframes
    res = []
    dev = torch.device("cuda:0")

    stage(res, "decode-cpu", n, lambda: sum(1 for _ in Reader(info, hwaccel=False, limit=n)))
    bufs = stage(res, "decode-nvdec", n, lambda: list(Reader(info, hwaccel=True, limit=n)))
    n = len(bufs)

    def upload():
        for i in range(0, n, batch):
            upload_batch(bufs[i : i + batch], info, dev)

    stage(res, "upload", n, upload)

    dets = None
    for imgsz in imgsizes:
        det = Detector(pose, face, imgsz=imgsz, batch=batch, engine=engine,
                       eye_size=(info.width // 2, info.height))
        det.predict(det.prepare(*upload_batch(bufs[:batch], info, dev)))  # warm-up

        def detect():
            pose_d, face_d = {}, {}
            for i in range(0, n, batch):
                for k, (p, f) in enumerate(det.predict(det.prepare(*upload_batch(bufs[i : i + batch], info, dev)))):
                    pose_d[(i + k // 2, k % 2)] = p
                    face_d[(i + k // 2, k % 2)] = f
            return Detections(n, 1, info.width // 2, pose_d, face_d)

        d = stage(res, f"detect-{imgsz}{'-trt' if engine else ''}", n, detect)
        dets = dets or d
        del det
        torch.cuda.empty_cache()

    renderer = MaskRenderer(dets, info.height)
    masks = stage(res, "rasterise", n, lambda: [renderer.frame(t) for t in range(n)])
    for op in ("blur", "pixelate", "fill"):
        masker = Masker(info, op=op)

        def mask():
            for i in range(0, n, batch):
                y, u, v = upload_batch(bufs[i : i + batch], info, dev)
                m = torch.from_numpy(np.stack(masks[i : i + batch])).to(dev)
                y, u, v = masker.apply(y, u, v, m)
                torch.cat([y.flatten(1), u.flatten(1), v.flatten(1)], 1).cpu()

        stage(res, f"mask-{op}+transfer", n, mask)

    with tempfile.TemporaryDirectory() as tmp:
        for enc in encoders:
            def encode():
                w = Writer(info, os.path.join(tmp, f"{enc}.mp4"), encoder=enc, quality=19 if enc == "nvenc" else 16)
                for b in bufs:
                    w.write(b)
                return w.close()

            stage(res, f"encode-{enc}", n, encode)
    return res
