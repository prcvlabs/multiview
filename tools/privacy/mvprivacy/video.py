"""Raw planar-YUV video I/O through ffmpeg subprocesses.

Frames travel as planar YUV (yuv420p or yuv444p, whatever the source uses),
so masking can work on the decoder's own planes and the encoder receives the
same layout back: no colour conversion on the masking path.
"""

import json
import queue
import subprocess
import threading
from dataclasses import dataclass
from fractions import Fraction

import numpy as np

k_planar = {"yuv420p": (2, 2), "yuvj420p": (2, 2), "yuv444p": (1, 1), "yuvj444p": (1, 1)}


# ------------------------------------------------------------------------ probe
@dataclass
class VideoInfo:
    path: str
    width: int
    height: int
    pix_fmt: str
    rate: str  # r_frame_rate, e.g. "15/1"
    timescale: int  # 1 / time_base, kept on re-encode
    nframes: int
    profile: str
    color_range: str

    @property
    def fps(self):
        return float(Fraction(self.rate))

    @property
    def layout(self):
        """The raw pixel format used on the pipe."""
        return "yuv444p" if k_planar.get(self.pix_fmt, (2, 2)) == (1, 1) else "yuv420p"

    @property
    def subsampling(self):
        return k_planar[self.layout]

    @property
    def frame_bytes(self):
        sx, sy = self.subsampling
        return self.width * self.height + 2 * (self.width // sx) * (self.height // sy)


def probe(path, count=False):
    """Stream geometry and timing; `count` decodes packet headers when the
    container does not store nb_frames."""
    cmd = ["ffprobe", "-v", "error", "-select_streams", "v:0"]
    if count:
        cmd += ["-count_packets"]
    cmd += [
        "-show_entries",
        "stream=width,height,pix_fmt,r_frame_rate,time_base,nb_frames,nb_read_packets,profile,color_range",
        "-of", "json", path,
    ]
    s = json.loads(subprocess.check_output(cmd))["streams"][0]
    n = s.get("nb_read_packets") or s.get("nb_frames")
    if not n or n == "N/A":
        if not count:
            return probe(path, count=True)
        n = 0
    return VideoInfo(
        path=path,
        width=int(s["width"]),
        height=int(s["height"]),
        pix_fmt=s["pix_fmt"],
        rate=s["r_frame_rate"],
        timescale=Fraction(s["time_base"]).denominator,
        nframes=int(n),
        profile=s.get("profile", ""),
        color_range=s.get("color_range", "tv"),
    )


# ----------------------------------------------------------------------- planes
def split_planes(buf, info):
    """One raw frame (bytes or uint8 array) -> (Y, U, V) views."""
    a = np.frombuffer(buf, np.uint8)
    sx, sy = info.subsampling
    w, h = info.width, info.height
    cw, ch = w // sx, h // sy
    y = a[: w * h].reshape(h, w)
    u = a[w * h : w * h + cw * ch].reshape(ch, cw)
    v = a[w * h + cw * ch :].reshape(ch, cw)
    return y, u, v


# ----------------------------------------------------------------------- Reader
class Reader:
    """Iterates raw frames of a video. `every` > 1 keeps frames 0, N, 2N, ...
    (the decoder still decodes all of them; only the pipe is thinned)."""

    def __init__(self, info, hwaccel=True, every=1, limit=0, threads=4, prefetch=64):
        self.info = info
        self.every = every
        cmd = ["ffmpeg", "-v", "error", "-nostdin", "-threads", str(threads)]
        if hwaccel:
            cmd += ["-hwaccel", "cuda"]
        cmd += ["-i", info.path]
        if every > 1:
            cmd += ["-vf", f"select=not(mod(n\\,{every}))"]
        if limit:
            cmd += ["-frames:v", str(limit)]
        cmd += ["-fps_mode", "passthrough", "-f", "rawvideo", "-pix_fmt", info.layout, "-"]
        self.proc_ = subprocess.Popen(cmd, stdout=subprocess.PIPE, bufsize=info.frame_bytes * 4)
        self.queue_ = queue.Queue(maxsize=prefetch)
        self.thread_ = threading.Thread(target=self.pump_, daemon=True)
        self.thread_.start()

    def pump_(self):
        n = self.info.frame_bytes
        while True:
            buf = self.proc_.stdout.read(n)
            if len(buf) < n:
                break
            self.queue_.put(buf)
        self.queue_.put(None)

    def __iter__(self):
        while True:
            buf = self.queue_.get()
            if buf is None:
                break
            yield buf
        self.close()

    def batches(self, size):
        batch = []
        for buf in self:
            batch.append(buf)
            if len(batch) == size:
                yield batch
                batch = []
        if batch:
            yield batch

    def close(self):
        if self.proc_.poll() is None:
            self.proc_.stdout.close()
            self.proc_.wait()
        return self.proc_.returncode


# ----------------------------------------------------------------------- Writer
def encoder_args(info, encoder, quality):
    """Codec arguments for an H.264 output matching `info`'s pixel format."""
    if encoder == "nvenc":
        profile = "high444p" if info.layout == "yuv444p" else "high"
        return ["-c:v", "h264_nvenc", "-preset", "p6", "-tune", "hq", "-profile:v", profile,
                "-rc", "vbr", "-cq", str(quality), "-b:v", "0"]
    return ["-c:v", "libx264", "-preset", "slow", "-crf", str(quality)]


class Writer:
    """Encodes raw frames to H.264 with the input's frame rate, timescale and
    pixel format, so frame count and per-frame pts match the source."""

    def __init__(self, info, dst, encoder="x264", quality=16, threads=4, queue_size=64):
        cmd = ["ffmpeg", "-v", "error", "-nostdin", "-y",
               "-f", "rawvideo", "-pix_fmt", info.layout, "-s", f"{info.width}x{info.height}",
               "-r", info.rate, "-i", "-", "-threads", str(threads)]
        cmd += encoder_args(info, encoder, quality)
        cmd += ["-pix_fmt", info.layout, "-color_range", info.color_range or "tv",
                "-video_track_timescale", str(info.timescale),
                "-map_metadata", "-1", "-fflags", "+bitexact", "-flags:v", "+bitexact",
                "-movflags", "+faststart", dst]
        self.nframes = 0
        self.proc_ = subprocess.Popen(cmd, stdin=subprocess.PIPE)
        self.queue_ = queue.Queue(maxsize=queue_size)
        self.thread_ = threading.Thread(target=self.pump_, daemon=True)
        self.thread_.start()

    def pump_(self):
        while True:
            buf = self.queue_.get()
            if buf is None:
                break
            self.proc_.stdin.write(buf)

    def write(self, buf):
        self.queue_.put(buf)
        self.nframes += 1

    def close(self):
        self.queue_.put(None)
        self.thread_.join()
        self.proc_.stdin.close()
        return self.proc_.wait()
