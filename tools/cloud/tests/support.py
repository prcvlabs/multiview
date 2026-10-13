"""Shared set-up of the mvcloud tests: a synthetic scene, a job spec and the
fake multiview-cli. Run the tests with

    python3 -m unittest discover -s tools/cloud/tests        (or pytest)
"""

import json
import logging
import os
import stat
import sys

k_tests_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(k_tests_dir), "agent"))

from mvcloud import storage  # noqa: E402

if not os.environ.get("MVCLOUD_TEST_LOG"):
    logging.getLogger("mvcloud").addHandler(logging.NullHandler())


# ------------------------------------------------------------------- make_scene
def make_scene(d, *, n_frames=40, n_cameras=1):
    """A manifest with one tiny 'video', params, a pose replay directory and
    a ground truth under `d`; returns the paths."""
    os.makedirs(d, exist_ok=True)
    videos = []
    for i in range(n_cameras):
        name = f"cam{i}.mp4"
        with open(os.path.join(d, name), "wb") as f:
            f.write(os.urandom(1000))
        videos.append({"camera": f"C{i}", "file": name})
    paths = {"manifest": os.path.join(d, "manifest.json"), "params": os.path.join(d, "params.json"),
             "poses": os.path.join(d, "poses"), "gt": os.path.join(d, "ground-truth.json")}
    with open(paths["manifest"], "w") as f:
        json.dump({"type": "SceneManifest", "videos": videos}, f)
    with open(paths["params"], "w") as f:
        json.dump({"task_params": {"frame_params": {"pose_skeleton_init": {"op_params": {
            "feedback": False, "replay_dir": "/elsewhere", "use_hyperpose": True}}},
            "tracklet_params": {"max_frames_per_tracklet": 10}}}, f)
    os.makedirs(paths["poses"], exist_ok=True)
    for t in range(n_frames):
        for i in range(n_cameras):
            with open(os.path.join(paths["poses"], f"pose-2d_{t:06d}_{2 * i:02d}.json"), "w") as f:
                f.write("[]")
    with open(paths["gt"], "w") as f:
        json.dump({"track_results": {"tracks": []}}, f)
    return paths


# -------------------------------------------------------------------- make_spec
def make_spec(paths, **kw):
    spec = {"scene_id": "synthetic", "manifest": paths["manifest"], "params": paths["params"],
            "poses": paths["poses"], "gt": paths["gt"], "total_frames": 40, "K": 10,
            "n_cameras": 1, "start_frame": 0, "n_frames": 40, "shard_frames": 20,
            "no_stereo": True, "pose_mode": "replay", "pose_classifier": "pose-svm-lite_v0",
            "heartbeat_s": 3, "lease_seconds": 30, "progress_timeout_s": 30, "retry_delay_s": 0}
    spec.update(kw)
    return spec


# --------------------------------------------------------------------- fake_cli
def fake_cli(d):
    """The fake as an executable named multiview-cli. Its comm is
    multiview-cli, as for the real binary, because the kernel runs the
    script's interpreter directly (#!/usr/bin/python3; through
    /usr/bin/env it would read python3), so reap_orphans takes it for one."""
    src = os.path.join(k_tests_dir, "fake_multiview_cli.py")
    os.chmod(src, os.stat(src).st_mode | stat.S_IXUSR)
    os.makedirs(d, exist_ok=True)
    dst = os.path.join(d, "multiview-cli")
    if not os.path.exists(dst):
        os.symlink(src, dst)
    return dst


# --------------------------------------------------------------- complete_lease
def complete_lease(control, lease, *, params_md5="p0", outputs=None):
    """Writes the lease's required outputs and done.json, then completes it
    (what an agent does, without running anything)."""
    done_outputs = []
    for o in lease["outputs"]:
        if o["name"] == "done.json" or not o["required"]:
            continue
        if outputs is not None and o["name"] not in outputs:
            continue
        path = storage.local_path(o["url"])
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, "w") as f:
            f.write(f"{lease['shard_id']} {o['name']}")
        done_outputs.append({"name": o["name"], "bytes": os.path.getsize(path),
                             "md5": storage.file_md5(path)})
    done = {"shard_id": lease["shard_id"], "attempt": lease["attempt"], "outputs": done_outputs,
            "params_md5": params_md5, "wall_s": 1.0}
    path = storage.local_path(next(o["url"] for o in lease["outputs"] if o["name"] == "done.json"))
    with open(path, "w") as f:
        json.dump(done, f)
    control.complete(lease, done)
    return done
