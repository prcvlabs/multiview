"""Turning archive material into testcase-format videos.

Testcase format: side-by-side stereo (left | right), H.264 High, yuv420p,
time base 1/15360, one file per camera.

  * PNG pipeline dumps: frames/NNNNNN_camK_S.png, S = 0 left, 1 right, one
    PNG per frame per sensor, in no particular order inside the tarball.
  * Debug renders whose left part is the stereo pair: cropped.
"""

import json
import re
import shutil
import subprocess
from collections import defaultdict
from concurrent.futures import ThreadPoolExecutor
from multiprocessing import Pool
from pathlib import Path

import cv2
import numpy as np

k_png_re = re.compile(r"(\d+)_cam(\d+)_([01])\.png$")
k_timescale = 15360


# ------------------------------------------------------------------- extract
def extract(tarball, dest):
    """Unpacks a .tar.bz with lbzip2 (all cores) when installed, else bzip2."""
    dest = Path(dest)
    dest.mkdir(parents=True, exist_ok=True)
    unzip = "lbzip2" if shutil.which("lbzip2") else "bzip2"
    if unzip == "bzip2":
        print("lbzip2 not found, using single-threaded bzip2 "
              "(micromamba install -c conda-forge lbzip2)", flush=True)
    subprocess.run(["tar", "-x", "-I", unzip, "-f", str(tarball), "-C", str(dest)], check=True)


# ---------------------------------------------------------------- scan_pngs
def scan_pngs(frames_dir):
    """{camera: {frame: {sensor: path}}} for a directory of PNG dumps."""
    cams = defaultdict(lambda: defaultdict(dict))
    for p in Path(frames_dir).iterdir():
        m = k_png_re.search(p.name)
        if m:
            cams[int(m[2])][int(m[1])][int(m[3])] = p
    return cams


# ------------------------------------------------------------- encode_camera
def encode_camera_(job):
    cam, frames, dst, fps, crf, threads = job
    first, last = min(frames), max(frames)
    gaps = {t for t in range(first, last + 1) if len(frames.get(t, {})) < 2}
    t0 = next(t for t in range(first, last + 1) if t not in gaps)
    h, w = cv2.imread(str(frames[t0][0]), cv2.IMREAD_COLOR).shape[:2]

    cmd = ["ffmpeg", "-v", "error", "-nostdin", "-y", "-f", "rawvideo", "-pix_fmt", "bgr24",
           "-s", f"{2 * w}x{h}", "-r", str(fps), "-i", "-", "-threads", str(threads),
           "-c:v", "libx264", "-preset", "slow", "-crf", str(crf), "-pix_fmt", "yuv420p",
           "-color_range", "tv", "-video_track_timescale", str(k_timescale),
           "-map_metadata", "-1", "-fflags", "+bitexact", "-flags:v", "+bitexact",
           "-movflags", "+faststart", str(dst) + ".tmp.mp4"]
    enc = subprocess.Popen(cmd, stdin=subprocess.PIPE)

    def load(t):
        if t in gaps:
            return None
        return np.hstack([cv2.imread(str(frames[t][s]), cv2.IMREAD_COLOR) for s in (0, 1)])

    last_img = load(t0)
    chunk = 8 * threads  # frames decoded ahead; a whole camera would not fit in memory
    with ThreadPoolExecutor(threads) as pool:
        for c0 in range(first, last + 1, chunk):
            for img in pool.map(load, range(c0, min(c0 + chunk, last + 1))):
                img = last_img if img is None else img  # a gap repeats the previous frame
                enc.stdin.write(img.tobytes())
                last_img = img
    enc.stdin.close()
    if enc.wait() != 0:
        raise RuntimeError(f"encode failed: {dst}")
    Path(str(dst) + ".tmp.mp4").rename(dst)
    return dict(camera=cam, file=str(dst), first_frame=first, last_frame=last,
                frames=last - first + 1, repeated_gaps=len(gaps), eye_size=[w, h])


# ---------------------------------------------------------------- ingest_pngs
def ingest_pngs(frames_dir, out_dir, fps=15, crf=14, jobs=3, threads=4, name="cam{cam}.mp4"):
    """One stereo video per camera from a PNG dump. Writes ingest.json with
    frame ranges and the number of missing frames that were repeated."""
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    cams = scan_pngs(frames_dir)
    if not cams:
        raise RuntimeError(f"no NNNNNN_camK_S.png files in {frames_dir}")
    work = [(c, dict(cams[c]), out / name.format(cam=c), fps, crf, threads)
            for c in sorted(cams) if not (out / name.format(cam=c)).exists()]
    with Pool(jobs) as pool:
        for r in pool.imap_unordered(encode_camera_, work):
            print(json.dumps(r), flush=True)
            report = out / "ingest.json"
            done = json.loads(report.read_text()) if report.exists() else {}
            done[str(r["camera"])] = r
            report.write_text(json.dumps(done, indent=1, sort_keys=True))


# ---------------------------------------------------------------------- crop
def crop(src, dst, width=1792, height=672, x=0, y=0, crf=14):
    """Cuts the stereo pair out of a wider render, keeping its frame timing."""
    subprocess.run(
        ["ffmpeg", "-v", "error", "-nostdin", "-y", "-i", str(src), "-vf", f"crop={width}:{height}:{x}:{y}",
         "-fps_mode", "passthrough", "-c:v", "libx264", "-preset", "slow", "-crf", str(crf),
         "-pix_fmt", "yuv420p", "-color_range", "tv", "-video_track_timescale", str(k_timescale),
         "-an", "-map_metadata", "-1", "-fflags", "+bitexact", "-flags:v", "+bitexact",
         "-movflags", "+faststart", str(dst)],
        check=True)
