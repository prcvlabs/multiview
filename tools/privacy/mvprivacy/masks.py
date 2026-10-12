"""Pass 2 building blocks: mask rasterisation and GPU mask operators.

Masks are drawn per frame and eye on the CPU (a handful of cv2 ellipses),
uploaded as uint8, and softened on the GPU. Operators work directly on the
YUV planes of each eye: luma at full resolution, chroma at its subsampled
resolution with the block size and sigma scaled to match. Blur and
pixelate are linear, so this equals doing them in RGB up to rounding.
"""

import json

import cv2
import numpy as np
import torch
import torch.nn.functional as F

from .heads import head_tracks, k_aspect

k_ops = ("blur", "pixelate", "fill")
k_regions = ("heads", "bodies")


# ------------------------------------------------------------- static_boxes
def load_static_boxes(path, key=None):
    """Fixed per-camera boxes (signage, screens): {"L": [[x0,y0,x1,y1]...],
    "R": [...]} in eye pixels, or a file of such dicts keyed by `key`."""
    if not path:
        return None
    d = json.load(open(path))
    if key is not None:
        d = d.get(key)
    return d if d and (d.get("L") or d.get("R")) else None


# -------------------------------------------------------------- MaskRenderer
class MaskRenderer:
    """Per-frame binary masks (2, H, eye_w) for a video's detections."""

    def __init__(self, dets, height, region="heads", window=3):
        if region not in k_regions:
            raise ValueError(f"region must be one of {k_regions}")
        self.dets = dets
        self.h = height
        self.w = dets.eye_width
        self.region = region
        self.window = window
        self.heads = [head_tracks(dets, e) for e in range(2)]

    def draw_heads_(self, m, eye, t):
        n = self.dets.nframes
        for s in range(max(0, t - self.window), min(n, t + self.window + 1)):
            for cx, cy, r in self.heads[eye][s]:
                axes = (int(round(r)), int(round(r * k_aspect)))
                cv2.ellipse(m, (int(round(cx)), int(round(cy))), axes, 0, 0, 360, 255, -1)

    def draw_bodies_(self, m, eye, t):
        """Person boxes from the detected frames around t, as filled ellipses."""
        st = self.dets.stride
        lo = max(0, (t - self.window) // st * st)
        hi = min(self.dets.nframes - 1, t + self.window + st)
        for s in range(lo, hi + 1, st):
            for x0, y0, x1, y1 in self.dets.pose(s, eye)[0]:
                c = (int(round((x0 + x1) / 2)), int(round((y0 + y1) / 2)))
                axes = (int(round(0.55 * (x1 - x0))), int(round(0.55 * (y1 - y0))))
                cv2.ellipse(m, c, axes, 0, 0, 360, 255, -1)

    def frame(self, t):
        out = np.zeros((2, self.h, self.w), np.uint8)
        for e in range(2):
            self.draw_heads_(out[e], e, t)
            if self.region == "bodies":
                self.draw_bodies_(out[e], e, t)
        return out

    def count(self, t):
        return sum(len(self.heads[e][t]) for e in range(2))


# --------------------------------------------------------------- gaussian
def gaussian_kernel(sigma, device):
    r = max(1, int(np.ceil(3.0 * sigma)))
    x = torch.arange(-r, r + 1, device=device, dtype=torch.float32)
    k = torch.exp(-0.5 * (x / sigma) ** 2)
    return k / k.sum()


def gaussian_blur(x, sigma):
    """Separable gaussian on (N,C,H,W) float, reflect border."""
    if sigma <= 0:
        return x
    k = gaussian_kernel(sigma, x.device)
    r = (len(k) - 1) // 2
    c = x.shape[1]
    x = F.pad(x, (r, r, 0, 0), mode="reflect" if x.shape[3] > r else "replicate")
    x = F.conv2d(x, k.view(1, 1, 1, -1).expand(c, 1, 1, -1), groups=c)
    x = F.pad(x, (0, 0, r, r), mode="reflect" if x.shape[2] > r else "replicate")
    return F.conv2d(x, k.view(1, 1, -1, 1).expand(c, 1, -1, 1), groups=c)


def pixelate(x, block, smooth=True):
    if block <= 1:
        return x
    H, W = x.shape[-2:]
    small = F.adaptive_avg_pool2d(x, (max(1, H // block), max(1, W // block)))
    if smooth:
        return F.interpolate(small, size=(H, W), mode="bilinear", align_corners=False)
    return F.interpolate(small, size=(H, W), mode="nearest")


# -------------------------------------------------------------------- Masker
class Masker:
    """Applies a mask operator to batches of planar YUV stereo frames on the
    GPU. Defaults reproduce the first release: pixelate 10, gaussian 6 on
    heads; pixelate 10, gaussian 10 on static boxes."""

    def __init__(self, info, op="blur", pix=10, sigma=6.0, static=None, device="cuda:0"):
        if op not in k_ops:
            raise ValueError(f"op must be one of {k_ops}")
        self.op = op
        self.pix = pix
        self.sigma = sigma
        self.sx, self.sy = info.subsampling
        self.h = info.height
        self.ew = info.width // 2
        self.device = torch.device(device)
        self.static_ = self.static_mask_(static) if static else None

    def static_mask_(self, boxes):
        m = np.zeros((2, self.h, self.ew), np.uint8)
        for e, key in enumerate("LR"):
            for x0, y0, x1, y1 in boxes.get(key, []):
                cv2.rectangle(m[e], (int(x0), int(y0)), (int(x1), int(y1)), 255, -1)
        t = torch.from_numpy(m).to(self.device).float().div_(255.0)[:, None]
        return gaussian_blur(t, 4.0).clamp_(0.0, 1.0)

    # ------------------------------------------------------------------ helpers
    def eyes_(self, p):
        """(B,h,w) plane -> (2B,1,h,w/2) float view of the two eyes."""
        B, h, w = p.shape
        return p.view(B, h, 2, w // 2).permute(0, 2, 1, 3).reshape(2 * B, 1, h, w // 2).float()

    def uneyes_(self, e, B):
        _, _, h, w2 = e.shape
        return e.view(B, 2, h, w2).permute(0, 2, 1, 3).reshape(B, h, 2 * w2)

    def soften_(self, m):
        m = gaussian_blur(m, 2.5)
        return torch.where(m > 20.0 / 255.0, (2.0 * m).clamp_(max=1.0), torch.zeros_like(m))

    def treat_(self, x, scale, pix, sigma):
        if self.op == "fill":
            return torch.full_like(x, 128.0)
        if self.op == "pixelate":
            return pixelate(x, max(1, int(round(pix * scale))), smooth=False)
        return gaussian_blur(pixelate(x, max(1, int(round(pix * scale)))), sigma * scale)

    def blend_(self, planes, m, pix, sigma):
        """planes: [(2B,1,h,w) float] Y, U, V; m: (2B,1,H,W) or (2,1,H,W)."""
        out = []
        for i, x in enumerate(planes):
            mm = m
            if i and (self.sx, self.sy) != (1, 1):
                mm = F.avg_pool2d(m, (self.sy, self.sx))
            if mm.shape[0] != x.shape[0]:
                mm = mm.repeat(x.shape[0] // mm.shape[0], 1, 1, 1)
            scale = 1.0 if i == 0 else 1.0 / self.sx
            out.append(torch.lerp(x, self.treat_(x, scale, pix, sigma), mm))
        return out

    # -------------------------------------------------------------------- apply
    def apply(self, y, u, v, masks):
        """y, u, v: (B,h,w) uint8 planes on the GPU; masks: (B,2,H,ew) uint8
        or None. Returns the treated planes as uint8."""
        B = y.shape[0]
        planes = [self.eyes_(p) for p in (y, u, v)]
        if self.static_ is not None:
            op = self.op
            self.op = "blur"
            planes = self.blend_(planes, self.static_, 10, 10.0)
            self.op = op
        if masks is not None and bool(masks.any()):
            m = self.soften_(masks.reshape(2 * B, 1, self.h, self.ew).float().div_(255.0))
            planes = self.blend_(planes, m, self.pix, self.sigma)
        return [self.uneyes_(p.round_().clamp_(0, 255), B).to(torch.uint8) for p in planes]
