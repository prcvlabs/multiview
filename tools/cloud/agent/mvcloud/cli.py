"""Command line of the cloud-distributed multiview agent.

  python3 -m mvcloud agent  --control fs:///root --cli multiview-cli [--once | --drain]
  python3 -m mvcloud submit --control fs:///root [--cli multiview-cli] job.json
  python3 -m mvcloud status --control fs:///root [job_id]
  python3 -m mvcloud plan   [--cli multiview-cli] job.json
  python3 -m mvcloud sweep  --control fs:///root [--verify]
  python3 -m mvcloud cancel --control fs:///root job_id

A job (job.json) names a scene and a frame range:

  {"scene_id": "007_site03_sparse_tracks", "manifest": ".../manifest.json",
   "params": ".../params.json", "poses": "<replay dir or .tar>", "gt": ".../ground-truth.json",
   "start_frame": 0, "n_frames": 120, "shard_frames": 60, "no_stereo": true,
   "pose_mode": "replay", "pose_classifier": "pose-svm-lite_v0"}

Relative paths are relative to job.json. `submit` and `plan` fill in the
scene's total_frames, K and n_cameras with `multiview-cli scene_info` unless
the job has them. Other keys and their defaults: fsbackend.k_job_defaults.
"""

import argparse
import json
import logging
import os
import sys

from . import agent, plan, scene
from .control import open_control

log = logging.getLogger("mvcloud")


# -------------------------------------------------------------------- load_job_
def load_job_(fname, cli_bin):
    """The job spec in `fname`, with absolute paths and the scene's numbers."""
    with open(fname) as f:
        spec = json.load(f)
    base = os.path.dirname(os.path.abspath(fname))
    for key in ("manifest", "params", "poses", "gt"):
        if spec.get(key):
            spec[key] = os.path.normpath(os.path.join(base, os.path.expanduser(spec[key])))
    if any(spec.get(k) is None for k in ("total_frames", "K", "n_cameras")):
        if not cli_bin:
            raise plan.PlanError("the job has no total_frames/K/n_cameras: pass --cli to run scene_info")
        info = scene.scene_info(cli_bin, spec["manifest"], params=spec.get("params"),
                                no_stereo=spec.get("no_stereo", True),
                                target_fps=spec.get("target_fps", 0.0))
        spec = scene.fill_job_spec(spec, info)
    return spec


# ------------------------------------------------------------------------ parse
def parse(argv):
    ap = argparse.ArgumentParser(prog="mvcloud", description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("-v", "--verbose", action="store_true")
    sub = ap.add_subparsers(dest="cmd", required=True)

    p = sub.add_parser("agent", help="run shards")
    p.add_argument("--control", required=True, help="fs:///path/to/root")
    p.add_argument("--cli", required=True, help="the multiview-cli binary")
    p.add_argument("--cache", default=os.path.expanduser("~/mvcloud-cache"),
                   help="inputs, cached by md5")
    p.add_argument("--work", default="/tmp/mvcloud-work",
                   help="attempt directories, one per agent (orphans found there are killed)")
    p.add_argument("--gpu-lock", default=agent.k_default_gpu_lock)
    p.add_argument("--mem", default="24G", help="memory cap of a shard; '' for none")
    p.add_argument("--kill-grace", type=float, default=20.0, help="seconds from SIGTERM to SIGKILL")
    p.add_argument("--worker-id")
    p.add_argument("--poll", type=float, default=15.0, help="seconds between pulls when idle")
    p.add_argument("--multiview-cache", help="MULTIVIEW_CACHE_DIR of the children")
    p.add_argument("--keep-work", action="store_true", help="keep attempt directories")
    g = p.add_mutually_exclusive_group()
    g.add_argument("--once", action="store_true", help="process at most one shard")
    g.add_argument("--drain", action="store_true", help="stop when there is no work")

    p = sub.add_parser("submit", help="plan a job and queue its shards")
    p.add_argument("--control", required=True)
    p.add_argument("--cli", help="multiview-cli, for scene_info")
    p.add_argument("job")

    p = sub.add_parser("status", help="jobs and shards as json")
    p.add_argument("--control", required=True)
    p.add_argument("job_id", nargs="?")

    p = sub.add_parser("plan", help="print a job's shards")
    p.add_argument("--cli", help="multiview-cli, for scene_info")
    p.add_argument("job")

    p = sub.add_parser("sweep", help="expire leases, retry, promote the tracks job")
    p.add_argument("--control", required=True)
    p.add_argument("--verify", action="store_true", help="also check the md5 of done outputs")

    p = sub.add_parser("cancel", help="cancel a job")
    p.add_argument("--control", required=True)
    p.add_argument("job_id")
    return ap.parse_args(argv)


# ------------------------------------------------------------------------- main
def main(argv=None):
    args = parse(sys.argv[1:] if argv is None else argv)
    logging.basicConfig(level=logging.DEBUG if args.verbose else logging.INFO,
                        format="%(asctime)s mvcloud %(levelname)s %(message)s", stream=sys.stderr)
    try:
        if args.cmd == "agent":
            a = agent.Agent(open_control(args.control), cli_bin=args.cli, cache_dir=args.cache,
                            work_dir=args.work, gpu_lock=args.gpu_lock, mem=args.mem,
                            kill_grace_s=args.kill_grace, worker_id=args.worker_id,
                            poll_s=args.poll, multiview_cache=args.multiview_cache,
                            keep_work=args.keep_work)
            n = a.run(once=args.once, drain=args.drain, install_signals=True)
            log.info(f"processed {n} shard(s)" + (f"; {a.stop_reason}" if a.stop_reason else ""))
        elif args.cmd == "submit":
            job_id = open_control(args.control).submit(load_job_(args.job, args.cli))
            print(job_id)
        elif args.cmd == "status":
            print(json.dumps(open_control(args.control).status(args.job_id), indent=1))
        elif args.cmd == "plan":
            spec = load_job_(args.job, args.cli)
            start = spec.get("start_frame", 0)
            n = spec.get("n_frames") or spec["total_frames"] - start
            shards = plan.plan_shards(spec["total_frames"], start, n, spec.get("shard_frames", 300),
                                      spec.get("overlap_frames", 0), spec["K"])
            mode = spec.get("pose_mode", "replay")
            print(f"{'shard':>8} {'compute':>14} {'keep':>14} {'max_wall_s':>11}")
            for x in shards:
                print(f"{f'loc-{x.idx:04d}':>8} {f'[{x.start}, {x.start + x.n})':>14} "
                      f"{f'[{x.keep_start}, {x.keep_start + x.keep_n})':>14} "
                      f"{plan.wall_budget_s('loc', x.n, mode):>11}")
            print(f"{'tracks':>8} {f'[{start}, {start + n})':>14} {f'[{start}, {start + n})':>14} "
                  f"{plan.wall_budget_s('tracks', n, mode):>11}")
        elif args.cmd == "sweep":
            for event in open_control(args.control).sweep(verify=args.verify):
                print(event)
        elif args.cmd == "cancel":
            open_control(args.control).cancel(args.job_id)
    except (plan.PlanError, scene.SceneInfoError, KeyError, ValueError, OSError) as e:
        log.error(f"{args.cmd}: {e}")
        return 1
    return 0
