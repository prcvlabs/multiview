"""What the planner needs to know about a scene, from `multiview-cli
scene_info`: the number of frames, the cameras and sensors, and K.

It is a full scene load (videos, calibration, undistort caches), seconds
with warm caches.
"""

import json
import os
import subprocess
import tempfile


class SceneInfoError(RuntimeError):
    """`multiview-cli scene_info` failed."""


# ------------------------------------------------------------------- scene_info
def scene_info(cli_bin, manifest, *, params=None, no_stereo=False, target_fps=0.0,
               timeout_s=900):
    """The scene_info object of `manifest`: n_frames, n_cameras, n_sensors,
    sensor_ids, scene_fps, frame_duration and K (from `params` if given)."""
    with tempfile.TemporaryDirectory(prefix="mvcloud-scene-info.") as d:
        out = os.path.join(d, "scene-info.json")
        argv = [cli_bin, "scene_info", "-m", manifest, "-o", out]
        if params:
            argv += ["-p", params]
        if no_stereo:
            argv += ["--no-stereo"]
        if target_fps > 0.0:
            argv += ["--target-fps", str(target_fps)]
        try:
            r = subprocess.run(argv, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
                               timeout=timeout_s)
        except (OSError, subprocess.TimeoutExpired) as e:
            raise SceneInfoError(f"{' '.join(argv)}: {e}") from e
        if r.returncode != 0 or not os.path.isfile(out):
            raise SceneInfoError(f"{' '.join(argv)}: exit {r.returncode}: {r.stdout[-1000:]}")
        with open(out) as f:
            return json.load(f)


# ---------------------------------------------------------------- fill_job_spec
def fill_job_spec(spec, info):
    """Copies total_frames, K and n_cameras from a scene_info object into a
    job spec, unless the spec has them."""
    out = dict(spec)
    out.setdefault("total_frames", info["n_frames"])
    out.setdefault("K", info["K"])
    out.setdefault("n_cameras", info["n_cameras"])
    out.setdefault("sensor_ids", info.get("sensor_ids", []))
    return out
