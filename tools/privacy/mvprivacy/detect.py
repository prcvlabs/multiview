"""Pass 1: sparse person-pose and face detection per stereo eye.

Frames go from the decoder to the GPU as planar YUV, are converted to RGB,
split into the two eyes and resized there, then batched through a YOLO pose
model and a YOLO face model (PyTorch, or TensorRT engines with --engine).
Detections are cached per video in a compact .npz, so mask treatments can
be replayed without detecting again.
"""

import os
import time
import urllib.request
from pathlib import Path

import numpy as np
import torch
import torch.nn.functional as F

from .video import Reader, probe, split_planes

k_model_dir = Path(os.environ.get("MULTIVIEW_PRIVACY_MODELS", Path.home() / ".cache/multiview-privacy/models"))
k_model_urls = {
    "yolo11x-pose.pt": "https://github.com/ultralytics/assets/releases/download/v8.3.0/yolo11x-pose.pt",
    "yolov11l-face.pt": "https://github.com/akanametov/yolo-face/releases/download/1.0.0/yolov11l-face.pt",
}
k_pose_args = dict(conf=0.10, iou=0.6, max_det=60)
k_face_args = dict(conf=0.15, iou=0.5, max_det=100)


# --------------------------------------------------------------- model_path
def model_path(name):
    """Local path of a model weight file, downloading it on first use."""
    p = Path(name)
    if p.exists():
        return str(p)
    p = k_model_dir / name
    if not p.exists() and name in k_model_urls:
        k_model_dir.mkdir(parents=True, exist_ok=True)
        urllib.request.urlretrieve(k_model_urls[name], str(p) + ".part")
        os.rename(str(p) + ".part", p)
    return str(p) if p.exists() else name


# ------------------------------------------------------------------- yuv_rgb
def yuv_to_rgb(y, u, v, full_range=False):
    """Planar YUV uint8 tensors (B,H,W), (B,h,w), (B,h,w) on the GPU -> RGB
    float (B,3,H,W) in [0, 1], BT.601."""
    H, W = y.shape[-2:]
    if u.shape[-2:] != (H, W):
        u = F.interpolate(u[:, None].float(), size=(H, W), mode="nearest")[:, 0]
        v = F.interpolate(v[:, None].float(), size=(H, W), mode="nearest")[:, 0]
    y, u, v = y.float(), u.float() - 128.0, v.float() - 128.0
    if not full_range:
        y = (y - 16.0) * (255.0 / 219.0)
        u = u * (255.0 / 224.0)
        v = v * (255.0 / 224.0)
    r = y + 1.402 * v
    g = y - 0.344136 * u - 0.714136 * v
    b = y + 1.772 * u
    return (torch.stack([r, g, b], 1) / 255.0).clamp_(0.0, 1.0)


# ---------------------------------------------------------------- Detections
class Detections:
    """Pose and face detections of one video, indexed by (frame, eye)."""

    k_empty_pose = (np.zeros((0, 4), np.float32), np.zeros((0,), np.float32), np.zeros((0, 17, 3), np.float32))
    k_empty_face = (np.zeros((0, 4), np.float32), np.zeros((0,), np.float32))

    def __init__(self, nframes, stride, eye_width, pose=None, faces=None, meta=None):
        self.nframes = nframes
        self.stride = stride
        self.eye_width = eye_width
        self.meta = meta or {}
        self.pose_ = pose or {}  # (t, eye) -> (boxes, confs, kps)
        self.faces_ = faces or {}  # (t, eye) -> (boxes, confs)

    def pose(self, t, eye):
        return self.pose_.get((t, eye), self.k_empty_pose)

    def faces(self, t, eye):
        return self.faces_.get((t, eye), self.k_empty_face)

    def detected_frames(self):
        return range(0, self.nframes, self.stride)

    def subsample(self, stride):
        """The detections a run with a larger stride would have produced."""
        if stride % self.stride:
            raise ValueError(f"stride {stride} is not a multiple of {self.stride}")
        keep = lambda d: {k: v for k, v in d.items() if k[0] % stride == 0}
        return Detections(self.nframes, stride, self.eye_width, keep(self.pose_), keep(self.faces_), self.meta)

    def save(self, path):
        def flat(d, n):
            keys = sorted(k for k, v in d.items() if len(v[0]))
            idx = np.array([k for k in keys for _ in range(len(d[k][0]))], np.int32).reshape(-1, 2)
            cols = [np.concatenate([d[k][i] for k in keys]) if keys else self.k_empty_pose[i] for i in range(n)]
            return idx, cols

        pi, (pb, pc, pk) = flat(self.pose_, 3)
        fi, (fb, fc) = flat(self.faces_, 2)
        tmp = str(path) + ".tmp.npz"
        np.savez_compressed(
            tmp, nframes=self.nframes, stride=self.stride, eye_width=self.eye_width, meta=repr(self.meta),
            pose_idx=pi, pose_box=pb, pose_conf=pc, pose_kp=pk,
            face_idx=fi, face_box=fb, face_conf=fc,
        )
        os.replace(tmp, path)

    @classmethod
    def load(cls, path):
        z = np.load(path)

        def group(idx, *cols):
            out = {}
            if not len(idx):
                return out
            keys, starts = np.unique(idx, axis=0, return_index=True)
            order = np.argsort(starts)
            keys, starts = keys[order], starts[order]
            ends = np.append(starts[1:], len(idx))
            for (t, e), a, b in zip(keys, starts, ends):
                out[(int(t), int(e))] = tuple(c[a:b] for c in cols)
            return out

        return cls(
            int(z["nframes"]), int(z["stride"]), int(z["eye_width"]),
            group(z["pose_idx"], z["pose_box"], z["pose_conf"], z["pose_kp"]),
            group(z["face_idx"], z["face_box"], z["face_conf"]),
        )


# ------------------------------------------------------------------ Detector
class Detector:
    """Batched pose + face detection on stereo frames already on the GPU."""

    def __init__(self, pose="yolo11x-pose.pt", face="yolov11l-face.pt", imgsz=1280, batch=32,
                 engine=False, eye_size=(896, 672), device="cuda:0"):
        from ultralytics import YOLO
        from ultralytics.utils import LOGGER

        LOGGER.setLevel(40)
        self.device = torch.device(device)
        self.batch = batch  # stereo frames per batch; the networks see 2x this
        self.eye_w, self.eye_h = eye_size
        self.in_w = imgsz
        self.in_h = int(round(imgsz * self.eye_h / self.eye_w / 32.0)) * 32
        self.engine = engine
        self.pose_ = YOLO(self.weights_(model_path(pose), engine), task="pose")
        self.face_ = YOLO(self.weights_(model_path(face), engine), task="detect")
        self.meta = dict(pose=Path(pose).name, face=Path(face).name, imgsz=imgsz, engine=engine)

    def weights_(self, pt, engine):
        """The .pt itself, or a cached TensorRT engine exported from it."""
        if not engine:
            return pt
        from ultralytics import YOLO

        out = Path(pt).with_name(f"{Path(pt).stem}-b{2 * self.batch}-{self.in_h}x{self.in_w}-fp16.engine")
        if not out.exists():
            src = YOLO(pt).export(format="engine", half=True, batch=2 * self.batch,
                                  imgsz=(self.in_h, self.in_w), device=0, verbose=False)
            os.replace(src, out)
        return str(out)

    # ------------------------------------------------------------------ prepare
    def prepare(self, y, u, v, full_range=False):
        """(B,H,W) planes of side-by-side stereo frames -> (2B,3,in_h,in_w),
        left eyes at even indices."""
        rgb = yuv_to_rgb(y, u, v, full_range)
        B, _, H, W = rgb.shape
        ew = W // 2
        eyes = rgb.view(B, 3, H, 2, ew).permute(0, 3, 1, 2, 4).reshape(2 * B, 3, H, ew)
        return F.interpolate(eyes, size=(self.in_h, self.in_w), mode="bilinear", align_corners=False)

    # ------------------------------------------------------------------ predict
    def predict(self, x):
        """Network input (2B,3,in_h,in_w) -> per eye-image (pose, faces) in
        eye pixels."""
        n = x.shape[0]
        if self.engine and n < 2 * self.batch:  # engines have a static batch
            x = torch.cat([x, x.new_zeros((2 * self.batch - n,) + x.shape[1:])])
        sx, sy = self.in_w / self.eye_w, self.in_h / self.eye_h
        scale = torch.tensor([sx, sy, sx, sy], device=x.device)
        rp = self.pose_.predict(x, half=True, verbose=False, **k_pose_args)[:n]
        rf = self.face_.predict(x, half=True, verbose=False, **k_face_args)[:n]
        out = []
        for p, f in zip(rp, rf):
            kp = p.keypoints.data.clone() if p.keypoints is not None else torch.zeros((0, 17, 3))
            kp[..., 0] /= sx
            kp[..., 1] /= sy
            out.append((
                ((p.boxes.xyxy / scale).cpu().numpy().astype(np.float32),
                 p.boxes.conf.cpu().numpy().astype(np.float32),
                 kp.cpu().numpy().astype(np.float32)),
                ((f.boxes.xyxy / scale).cpu().numpy().astype(np.float32),
                 f.boxes.conf.cpu().numpy().astype(np.float32)),
            ))
        return out


# ------------------------------------------------------------- upload_batch
def upload_batch(bufs, info, device):
    """Raw frame buffers -> (Y, U, V) uint8 tensors on the GPU."""
    planes = [split_planes(b, info) for b in bufs]
    return [torch.from_numpy(np.stack([p[i] for p in planes])).to(device, non_blocking=True) for i in range(3)]


# -------------------------------------------------------------- detect_video
def detect_video(info, detector, stride=4, hwaccel=True, timings=None):
    """Runs the detector on frames 0, stride, 2 stride, ... of a video."""
    pose, faces = {}, {}
    t = 0
    full_range = info.color_range == "pc" or info.pix_fmt.startswith("yuvj")
    tm = timings if timings is not None else {}
    reader = Reader(info, hwaccel=hwaccel, every=stride)
    for bufs in reader.batches(detector.batch):
        t0 = time.perf_counter()
        x = detector.prepare(*upload_batch(bufs, info, detector.device), full_range=full_range)
        t1 = time.perf_counter()
        res = detector.predict(x)
        tm["prepare"] = tm.get("prepare", 0.0) + t1 - t0
        tm["detect"] = tm.get("detect", 0.0) + time.perf_counter() - t1
        for i, (p, f) in enumerate(res):
            k = ((t + i // 2) * stride, i % 2)
            if len(p[0]):
                pose[k] = p
            if len(f[0]):
                faces[k] = f
        t += len(bufs)
    if reader.close() != 0:
        raise RuntimeError(f"decode failed: {info.path}")
    nframes = info.nframes
    if not (t - 1) * stride < nframes <= t * stride:
        nframes = probe(info.path, exact=True).nframes
    if not (t - 1) * stride < nframes <= t * stride:
        raise RuntimeError(f"{info.path}: probed {nframes} frames, decoded {t} at stride {stride}")
    return Detections(nframes, stride, info.width // 2, pose, faces, detector.meta)
