# Privacy pipeline for stereo videos

Masks faces (or whole people) in side-by-side stereo videos in the testcase
format, keeping frame count and timing, so recordings can be published.
It reproduces the treatment of the first dataset release (head ellipses from
YOLO11x-pose keypoints plus a YOLOv11l face detector, temporal gap filling,
pixelate + gaussian blur per eye, optional fixed boxes for signage) and is
built for the throughput goals of issue #10.

## Design

Two passes per video:

1. **detect**: decode, keep every N-th frame (`--stride`, default 2),
   convert YUV to RGB, split the eyes and resize on the GPU, then batch both
   eyes through the pose and face networks (PyTorch, or TensorRT FP16
   engines with `--engine`). Detections are cached as `<video>.dets.npz`.
2. **render**: decode, rasterise the head ellipses of frames t-3..t+3
   (gap-filled along tracks) on a CPU thread pool, apply the mask operator on
   the GPU directly on the Y and U/V planes of each eye, encode with NVENC
   (or x264). Frames never change colour space on this path.

Because detections are cached, comparing treatments (`--op blur|pixelate|fill`,
`--region heads|bodies`, `--pix`, `--sigma`) costs one decode and encode per
video.

## License note

This tool calls Ultralytics YOLO and loads its YOLO11 weights, which are
AGPL-3.0, as is the YOLOv11 face model built with it. multiview itself is
Apache-2.0 and does not link or bundle them: `tools/privacy` is a separate
tool that imports `ultralytics` at run time and downloads the weights on
first use. Anyone redistributing or serving it should check the AGPL terms.

## Setup

Python 3.10+ with `torch` (CUDA), `ultralytics`, `opencv-python`, `numpy`,
and `ffmpeg`/`ffprobe` on the `PATH` (NVENC build for the default encoder).
For `--engine`: `tensorrt-cu13` (or `-cu12`, matching torch), `onnx`, `onnxslim`.
Model weights download on first use to `~/.cache/multiview-privacy/models`
(`MULTIVIEW_PRIVACY_MODELS` overrides).

```sh
export PYTHONPATH=$PWD/tools/privacy
python -m mvprivacy --help
```

## Usage

```sh
# both passes over a list of videos; finished outputs are skipped
python -m mvprivacy run jobs.json --stride 2
#   jobs.json: [{"src": "in/C0001.mp4", "dst": "out/C0001.mp4",
#                "static": "masks.json", "static_key": "site04|C0001"}]

# the two passes separately
python -m mvprivacy detect in/*.mp4 -o dets/
python -m mvprivacy render in/C0001.mp4 dets/C0001.dets.npz out/C0001.mp4 --op pixelate

# outputs keep geometry, rate, time base, frame count and per-frame pts
python -m mvprivacy verify in/C0001.mp4 out/C0001.mp4

# per-stage throughput on one clip (add --engine for TensorRT)
python -m mvprivacy bench in/C0001.mp4 --frames 900 --imgsz 1280,640

# sparse detection: detect at stride 1, then simulate larger strides
python -m mvprivacy detect qa/*.mp4 -o dets1/ --stride 1
python -m mvprivacy sparse-eval dets1/*.dets.npz --strides 1,2,4,8
```

Blur strength: `--strength light` (default; gaussian sigma 6, no pixelation),
`release` (pixelate 10 then sigma 6, the first dataset release) or `heavy`
(pixelate 14, sigma 9). `--pix` and `--sigma` override the preset. Static
signage boxes are always pixelated 10, sigma 10.

Static boxes are `{"L": [[x0, y0, x1, y1], ...], "R": [...]}` in eye pixels,
or a file of those keyed by `--static-key`.

Sources should be constant frame rate (the testcase videos and everything
`ingest-pngs` writes are). The encoder writes frames at the source's
`r_frame_rate` and time base; `verify` reports a source whose timestamps
were irregular or whose edit list discarded frames.

## Archive material (issue #6)

```sh
# PNG pipeline dumps: frames/NNNNNN_camK_S.png, S = 0 left, 1 right
python -m mvprivacy extract dump.tar.bz /scratch/dump      # lbzip2 if installed
python -m mvprivacy ingest-pngs /scratch/dump/frames videos/ --fps 15
#   one camK.mp4 per camera + ingest.json (frame ranges, repeated gaps)

# debug renders with the stereo pair on the left
python -m mvprivacy crop render.mp4 stereo.mp4 --width 1792 --height 672
```

Naming, manifests and anonymised site ids are assigned outside this tool.

## Measurements

RTX 3090, Ryzen 5 3600, 600 frames of a 1792x672 15 fps clip, batch 16
stereo frames (32 eye images). Detection with `--engine` (TensorRT 11 FP16):
33.9 fps at 1280 px, 103 fps at 640 px. PyTorch FP16:

| Stage | fps |
|---|---|
| decode, CPU (libavcodec, 4 threads) | 1,009 |
| decode, NVDEC, downloaded through the pipe | 520 |
| upload to GPU | 2,409 |
| detect, pose + face, 1280 px per eye | 14.5 |
| detect, pose + face, 640 px per eye | 55 |
| rasterise masks (CPU) | 2,482 |
| blur on GPU + download | 593 |
| encode, NVENC p6 | 297 |
| encode, x264 slow (first release) | 45 |

Detection dominates; `--stride` divides its cost (stride 4 at 1280 px is
about 58 fps), and TensorRT engines raise the per-frame rate.

## Tracking on anonymised video with the original poses

Blurring faces removes the face keypoints HyperPose anchors skeletons on,
so tracking recall drops on anonymised video (#5). Instead, publish the 2D
poses found on the originals and replay them:

1. Run the pipeline on the original video with
   `task_params.frame_params.pose_skeleton_init.op_params.feedback = true`.
   It writes `pose-2d_<frame>_<sensor>.json` (one array of `Skeleton2D`
   per frame and sensor) into the output directory.
2. Ship those files with the anonymised video, e.g.
   `tar -czf poses-2d.tar.gz pose-2d_*.json` (007: 2,994 files, 1.2 MB).
3. Run the pipeline on the anonymised video with `op_params.replay_dir` set
   to the directory holding the files; no pose network runs.

On 007_site03_sparse_tracks (`--no-stereo`, `pose-svm-lite_v0`), blurred
video with replayed poses scores exactly what the original video does (F1
0.6139, 0 ID switches, identical TP distance and IoU); with HyperPose on the
blurred video, recall was 0.149. Remaining differences in the output are
the colour-appearance scores of the blurred heads.
