"""Privacy-preserving processing of stereo testcase videos.

  python -m mvprivacy run jobs.json            detect (cached) + mask + encode
  python -m mvprivacy detect SRC... -o DIR      pass 1 only
  python -m mvprivacy render SRC DETS DST       pass 2 only (try another --op)
  python -m mvprivacy bench SRC                 per-stage throughput
  python -m mvprivacy sparse-eval DETS... --height 672
  python -m mvprivacy verify ORIG OUT [ORIG OUT ...]
  python -m mvprivacy extract TARBALL DIR
  python -m mvprivacy ingest-pngs FRAMES_DIR OUT_DIR
  python -m mvprivacy crop SRC DST
"""

import argparse
import json
import sys
from pathlib import Path


# --------------------------------------------------------------- arguments
def add_detect_args(p):
    p.add_argument("--pose", default="yolo11x-pose.pt", help="pose weights (.pt)")
    p.add_argument("--face", default="yolov11l-face.pt", help="face weights (.pt)")
    p.add_argument("--imgsz", type=int, default=1280, help="network input width per eye")
    p.add_argument("--batch", type=int, default=16, help="stereo frames per batch")
    p.add_argument("--stride", type=int, default=2, help="detect every N-th frame")
    p.add_argument("--engine", action="store_true", help="TensorRT FP16 engines (exported once, cached)")


def add_render_args(p):
    p.add_argument("--op", default="blur", choices=("blur", "pixelate", "fill"))
    p.add_argument("--region", default="heads", choices=("heads", "bodies"))
    p.add_argument("--pix", type=int, default=10, help="pixelation block before blurring")
    p.add_argument("--sigma", type=float, default=6.0, help="gaussian sigma")
    p.add_argument("--window", type=int, default=3, help="union masks over +/- frames")
    p.add_argument("--static", help="JSON of fixed boxes {L: [...], R: [...]} (or keyed by --static-key)")
    p.add_argument("--static-key")
    p.add_argument("--encoder", default="x264", choices=("x264", "nvenc"))
    p.add_argument("--quality", type=int, default=16, help="x264 CRF / NVENC CQ")


def parse(argv):
    ap = argparse.ArgumentParser(prog="mvprivacy", description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--no-hwaccel", dest="hwaccel", action="store_false", help="decode on the CPU")
    sub = ap.add_subparsers(dest="cmd", required=True)

    p = sub.add_parser("run", help="both passes over a job list")
    p.add_argument("jobs", help='JSON list of {"src", "dst"[, "dets", "static", "static_key"]}')
    p.add_argument("--force", action="store_true", help="redo existing outputs")
    add_detect_args(p)
    add_render_args(p)

    p = sub.add_parser("detect", help="pass 1: cache detections")
    p.add_argument("src", nargs="+")
    p.add_argument("-o", "--out-dir", required=True)
    add_detect_args(p)

    p = sub.add_parser("render", help="pass 2: mask and encode from cached detections")
    p.add_argument("src")
    p.add_argument("dets")
    p.add_argument("dst")
    add_render_args(p)

    p = sub.add_parser("bench", help="time decode, detect, mask and encode separately")
    p.add_argument("src")
    p.add_argument("--frames", type=int, default=900)
    p.add_argument("--imgsz", default="1280,640", help="comma-separated detector widths")
    p.add_argument("--batch", type=int, default=16)
    p.add_argument("--engine", action="store_true")
    p.add_argument("--pose", default="yolo11x-pose.pt")
    p.add_argument("--face", default="yolov11l-face.pt")
    p.add_argument("-o", "--out", help="write results as JSON")

    p = sub.add_parser("sparse-eval", help="miss rate vs detection stride, from stride-1 detections")
    p.add_argument("dets", nargs="+")
    p.add_argument("--height", type=int, default=672)
    p.add_argument("--strides", default="1,2,3,4,6,8")
    p.add_argument("--window", type=int, default=3)

    p = sub.add_parser("verify", help="output has the original's geometry, timing and frame pts")
    p.add_argument("pairs", nargs="+", help="ORIG OUT [ORIG OUT ...]")

    p = sub.add_parser("extract", help="unpack a PNG-dump tarball")
    p.add_argument("tarball")
    p.add_argument("dest")

    p = sub.add_parser("ingest-pngs", help="PNG dump -> one stereo video per camera")
    p.add_argument("frames_dir")
    p.add_argument("out_dir")
    p.add_argument("--fps", type=int, default=15)
    p.add_argument("--crf", type=int, default=14)
    p.add_argument("--jobs", type=int, default=3, help="cameras encoded at once")
    p.add_argument("--threads", type=int, default=4, help="PNG decode / x264 threads per camera")

    p = sub.add_parser("crop", help="cut the stereo pair out of a wider render")
    p.add_argument("src")
    p.add_argument("dst")
    p.add_argument("--width", type=int, default=1792)
    p.add_argument("--height", type=int, default=672)
    return ap.parse_args(argv)


# --------------------------------------------------------------------- main
def main(argv=None):
    args = parse(argv)

    if args.cmd == "run":
        from .pipeline import run_jobs

        run_jobs(json.load(open(args.jobs)), args)

    elif args.cmd == "detect":
        from .detect import Detector, detect_video
        from .video import probe

        detector = None
        for src in args.src:
            out = Path(args.out_dir) / (Path(src).stem + ".dets.npz")
            if out.exists():
                continue
            info = probe(src)
            if detector is None:
                detector = Detector(args.pose, args.face, imgsz=args.imgsz, batch=args.batch,
                                    engine=args.engine, eye_size=(info.width // 2, info.height))
            tm = {}
            dets = detect_video(info, detector, stride=args.stride, hwaccel=args.hwaccel, timings=tm)
            out.parent.mkdir(parents=True, exist_ok=True)
            dets.save(out)
            print(json.dumps(dict(out=str(out), frames=dets.nframes, **{k: round(v, 2) for k, v in tm.items()})),
                  flush=True)

    elif args.cmd == "render":
        from .detect import Detections
        from .masks import Masker, MaskRenderer, load_static_boxes
        from .pipeline import render_video
        from .video import probe

        info = probe(args.src)
        dets = Detections.load(args.dets)
        static = load_static_boxes(args.static, args.static_key)
        tm = {}
        n = render_video(info, dets, args.dst, MaskRenderer(dets, info.height, args.region, args.window),
                         Masker(info, op=args.op, pix=args.pix, sigma=args.sigma, static=static),
                         encoder=args.encoder, quality=args.quality, hwaccel=args.hwaccel, timings=tm)
        print(json.dumps(dict(dst=args.dst, frames=n, **{k: round(v, 2) for k, v in tm.items()})))

    elif args.cmd == "bench":
        from .bench import bench

        res = bench(args.src, frames=args.frames, batch=args.batch,
                    imgsizes=[int(x) for x in args.imgsz.split(",")], engine=args.engine,
                    pose=args.pose, face=args.face)
        if args.out:
            Path(args.out).write_text(json.dumps(res, indent=1))

    elif args.cmd == "sparse-eval":
        from .detect import Detections
        from .qa import sparse_eval

        strides = [int(x) for x in args.strides.split(",")]
        for p in args.dets:
            print(p, flush=True)
            sparse_eval(Detections.load(p), args.height, strides=strides, window=args.window)

    elif args.cmd == "verify":
        from .qa import verify_format

        if len(args.pairs) % 2:
            sys.exit("verify takes ORIG OUT pairs")
        bad = 0
        for orig, out in zip(args.pairs[::2], args.pairs[1::2]):
            errs = verify_format(orig, out)
            print(("BAD " if errs else "OK  ") + out + ("  " + "; ".join(errs) if errs else ""), flush=True)
            bad += bool(errs)
        sys.exit(1 if bad else 0)

    elif args.cmd == "extract":
        from .ingest import extract

        extract(args.tarball, args.dest)

    elif args.cmd == "ingest-pngs":
        from .ingest import ingest_pngs

        ingest_pngs(args.frames_dir, args.out_dir, fps=args.fps, crf=args.crf, jobs=args.jobs, threads=args.threads)

    elif args.cmd == "crop":
        from .ingest import crop

        crop(args.src, args.dst, width=args.width, height=args.height)


if __name__ == "__main__":
    main()
