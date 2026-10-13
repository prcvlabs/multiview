# mvcloud: sharded multiview runs (issue #14)

`tools/cloud` runs the C++ tracking pipeline over long scenes and many scenes by
cutting each scene into time windows ("loc shards") that any GPU machine can
compute, then running the tracker once per scene over the concatenated
results ("the tracks job"). With `--no-stereo` and replayed 2D poses the
shards' localization data is byte-identical to one batch run's, and the tracks
job's track results and score equal the batch run's except `fp_info.lab_p`,
which depends on the still image (accumulated over different frames); that
equality, not a score against ground truth, is the acceptance test.

| Piece | Where | State |
|---|---|---|
| Loc shards, the tracks job, `ldata`, `score`, `scene_info` | `multiview-cli` (C++) | built, unit-tested, gated by `e2e_fs.sh` |
| Agent and the filesystem control plane | `agent/mvcloud` (Python 3.12, standard library) | built, 71 unit tests, `e2e_fs.sh` |
| S3 client for R2 or MinIO, `s3://` manifests through lazy-s3 | `multiview-cli` (`MULTIVIEW_S3_*`) | built, `[s3]` unit test, `t10_minio.sh` against MinIO |
| Worker image | `docker/` | built here; CPU smoke `t8_container.sh` passes; the GPU path is untested (no nvidia-container-toolkit on the build machine) |
| Cloudflare Worker: API, D1 leases, one Workflow per job, R2 blob proxy, status page | `worker/` (TypeScript), agent `httpcontrol.py` | built; vitest (the D1 protocol on `node:sqlite`), `e2e_wrangler.sh` under `wrangler dev`; never deployed (no account) |
| Queue HTTP pull consumer, presigned R2 URLs, engine upload to R2 | | deferred (M8): need an account; `/v1/pull` on D1 dispatches, blobs go through the Worker |

## 1. Runbook

### Prerequisites

- The `multiview` micromamba env and a Release build: `build/setup-env.sh`,
  then `build/dev.sh build` (and `build/dev.sh build --hyperpose` for live
  poses; `--with-tensorrt` in setup-env.sh).
- Testcase data under `$MULTIVIEW_TESTCASE_DATA` (default `~/multiview-data`):
  `build/fetch-testcases.sh`, and for replay jobs the pose files
  `pose-replay/<scene>/pose-2d_<frame:06d>_<sensor:02d>.json`.
- For HyperPose jobs the ONNX models: `build/download-hyperpose-models.sh`
  (`MULTIVIEW_HYPERPOSE_DIR`). TensorRT engines are cached per GPU type under
  `$MULTIVIEW_CACHE_DIR/trt/`; the first process on a machine builds one
  (60-80 s), later ones load it in seconds.

### A job on the local GPU (filesystem backend)

```
export PYTHONPATH=tools/cloud/agent
CLI=multiview/multiview_cpp/build/cmake-release/multiview-cli   # cmake-hyperpose for pose_mode hyperpose
cat > job.json <<EOF
{"scene_id": "007_site03_sparse_tracks",
 "manifest": "$HOME/multiview-data/testcases/007_site03_sparse_tracks/manifest.json",
 "params": "params.json",
 "poses": "$HOME/multiview-data/pose-replay/007_site03_sparse_tracks",
 "gt": "$HOME/multiview-data/testcases/007_site03_sparse_tracks/ground-truth.json",
 "start_frame": 0, "n_frames": 499, "shard_frames": 250,
 "no_stereo": true, "pose_mode": "replay", "pose_classifier": "pose-svm-lite_v0"}
EOF
python3 -m mvcloud plan   --cli $CLI job.json                 # the shards, without running anything
python3 -m mvcloud submit --control fs://$HOME/mvcloud-root --cli $CLI job.json
python3 -m mvcloud agent  --control fs://$HOME/mvcloud-root --cli $CLI --drain
python3 -m mvcloud status --control fs://$HOME/mvcloud-root <job_id>
```

`params.json` is the scene's params file (`multiview-cli dump_default_params
-d <dir> <manifest>`; it names the scene's sensors, so it is per scene); the
agent sets the pose replay directory or HyperPose in its copy. For HyperPose
jobs export `MULTIVIEW_HYPERPOSE_DIR` for the agent: the children inherit its
environment. The agent takes the GPU lock
that `build/dev.sh` uses (`/tmp/multiview-dev-gpu.lock`), so it queues behind
other GPU work on the machine; it runs each shard in its own session at
nice 15 under a 24 GB cap (`--mem`), heartbeats while it runs, and kills the
session on lease loss, cancel, its wall-clock budget or no progress. Several
agents may share a root (one `--work` directory each). Other commands:
`sweep [--verify]` (expire leases, re-queue shards whose outputs are gone or
corrupt), `cancel <job_id>`. Job keys and defaults: `fsbackend.k_job_defaults`
(`shard_frames` 300, `max_attempts` 3, `lease_seconds` 1800, `heartbeat_s` 60,
`progress_timeout_s` 900).

Outputs: `<root>/blobs/jobs/<job>/shards/<shard>/a<attempt>/` (the winning
attempt is the shard row's `output_prefix`); the tracks job's
`pipeline-output.json` and, with `gt`, `score.json`.

### A job through the Worker (wrangler dev here; Cloudflare when deployed)

The Worker (`tools/cloud/worker`) is the same control plane as the
filesystem backend, on Cloudflare: D1 holds jobs, shards, attempts and
workers; one Workflow instance per job plans it, waits for the loc shards,
promotes the tracks job and finalizes; R2 holds the blobs, which the Worker
proxies (multipart above `MULTIPART_PART_BYTES`, 64 MiB). Its toolchain
lives in its own env:

```
micromamba create -y -f tools/cloud/environment.yml          # Node 22, Python 3.12, pytest
micromamba run -n mvcloud npm --prefix tools/cloud/worker ci
cd tools/cloud/worker
micromamba run -n mvcloud npm test                           # vitest
micromamba run -n mvcloud npx wrangler d1 migrations apply mvcloud --local --env local
micromamba run -n mvcloud npx wrangler dev --env local --port 8787 \
    --var MVCLOUD_ADMIN_TOKEN:<admin> --var MVCLOUD_WORKER_TOKEN:<worker>
```

Then, with the same `job.json` as above:

```
export MVCLOUD_ADMIN_TOKEN=<admin> MVCLOUD_WORKER_TOKEN=<worker>
python3 -m mvcloud submit --control http://127.0.0.1:8787 --cli $CLI job.json   # uploads, then POST /v1/jobs
python3 -m mvcloud agent  --control http://127.0.0.1:8787 --dispatch api --cli $CLI
python3 -m mvcloud status --control http://127.0.0.1:8787 <job_id>
python3 -m mvcloud cancel --control http://127.0.0.1:8787 <job_id>
```

`submit` uploads the manifest, the videos, the params, the replayed poses (as
one tar) and the ground truth through the Worker, content-addressed (section
3), skipping objects already there, and the Worker checks that every input
exists before it accepts the job. Agents poll `/v1/pull` (`--poll`, 15 s
with jitter); a job's tracks job appears when its Workflow sees the last loc
shard done (at once: `complete` wakes it), and expired leases and retries are
swept every `wait_seconds` (300 by default). The status page is
`http://127.0.0.1:8787/?token=<admin>`.

API: `POST /v1/jobs` (admin), `GET /v1/jobs[/<id>]`, `POST /v1/jobs/<id>/cancel`
(admin), `POST /v1/pull | /v1/claim | /v1/heartbeat | /v1/complete | /v1/fail`,
`GET /v1/workers`, `/v1/blobs/<key>` (a worker writes only under its lease's
`a<attempt>/` prefix, named by `x-mvcloud-lease`), `GET /v1/health`. Every
state change of a shard is one conditional D1 `UPDATE` on its state and lease
token; a late or duplicate call changes nothing. Deploying is in the header
of `wrangler.jsonc` (create the D1 database, R2 bucket and Queue, apply the
migration remotely, set the two tokens as secrets); it has not been tried.

### Tests and references

| Command | What | Time here |
|---|---|---|
| `python3 -m unittest discover -s tools/cloud/tests` | agent unit tests against `fake_multiview_cli.py` | 25-35 s |
| `build/dev.sh test "[ldata],[shard],[score],[within-noise],[s3]"` | C++ unit tests | seconds |
| `tools/cloud/tests/make_reference.sh --frames 120` (and `499`) | the batch references R1/R2/R3 in `cloud-golden/007` | 10 min (120), 37 min (499) |
| `tools/cloud/tests/e2e_fs.sh` | the gate: T1 (2 x 60 frames), T3 (1 x 120), T5 (idempotency, faults, refusals) | 45-60 min with another worktree on the GPU |
| `tools/cloud/tests/e2e_fs.sh --tests T4` | full 007, 2 x 250 frames against the 499-frame references | 21 min (1,262 s, of which 310 s waiting for the GPU lock) |
| `tools/cloud/tests/e2e_fs.sh --tests T6` | 008 with live HyperPose, [0,80) + [80,149), against a one-process run | 13 min (801 s, with the one-process reference run) |
| `tools/cloud/tests/t10_minio.sh` | S3 client, storage.py and an `s3://` manifest against MinIO in Docker | 10 min (two 30-frame runs, with GPU lock waits) |
| `tools/cloud/tests/t8_container.sh` | the worker image on the CPU: one 20-frame loc shard through `mvcloud agent --once` | 1 min (after the image build) |
| `tools/cloud/tests/check_shard_parity.sh` | the C++ gate without the agent | 8 min |
| `npm test` in `tools/cloud/worker` (mvcloud env) | vitest: `plan.ts` against `plan-golden.json`, the D1 protocol on the real migration | < 1 s |
| `tools/cloud/tests/e2e_wrangler.sh` | T2: 007 [0,120) through `wrangler dev` with an agent killed mid-shard (lease expiry, attempt 2), multipart envelopes, (a)-(f) of T1; T2c cancel and T2t timeout with the fake CLI | 4.5 min with the GPU free (271 s), 9.5 min behind another worktree (565 s); T2c + T2t 20 s |

### Storage on R2 (or any S3 endpoint)

`multiview-cli` reads `s3://bucket/key` paths through the AWS SDK. Point it
at R2 with:

| Variable | Default | Effect |
|---|---|---|
| `MULTIVIEW_S3_ENDPOINT` | `""`: AWS | e.g. `https://<account>.r2.cloudflarestorage.com` or `http://127.0.0.1:9000`; the scheme comes from the URI; the EC2 metadata service is never asked |
| `MULTIVIEW_S3_REGION` | `""`: `auto` when an endpoint is set | the signing region |
| `MULTIVIEW_S3_PATH_STYLE` | unset: on when an endpoint is set | `1`/`true` or `0`: bucket in the path instead of the host name |
| `MULTIVIEW_S3_PROFILE` | `multiview` | exported as `AWS_PROFILE`; empty keeps the caller's |
| `AWS_ACCESS_KEY_ID`, `AWS_SECRET_ACCESS_KEY` | | an R2 API token pair; read by the SDK, never recorded |

With an endpoint, request and response checksums are computed only when the
API requires them (the SDK's default, `WHEN_SUPPORTED`, adds them wherever an
operation allows, which S3-compatible stores do not all accept). The settings, never the keys, appear in `run-object.json`.

`pipeline -m s3://multiview/scenes/<scene>/manifest.json` reads the manifest
and the videos it names (bare names, resolved next to the manifest) through
the lazy-s3 cache in `MULTIVIEW_LAZY_S3_DIR`: the first run downloads, later
runs read the cache. Calibration assets come from S3 with
`MULTIVIEW_ASSET_STORE=LAZY_S3` and `MULTIVIEW_ASSET_S3_BUCKET`. Known limits:
the cache key is `bucket/key` (not the endpoint, not the ETag), so a changed
object or a second endpoint with the same bucket name needs a fresh cache
directory; `scene_info -m` and the agent's filesystem backend take local
manifests only.

### The container

```
tools/cloud/docker/build-image.sh [-t mvcloud:dev] [--commit HEAD | --worktree] [--cuda-archs "86;89"] [--jobs 4]
docker run --gpus all --memory 24g -v mvcloud-cache:/cache -v /data/root:/data/root \
    mvcloud:dev --control fs:///data/root --drain
```

Two stages: the builder creates the micromamba env with TensorRT
(`setup-env.sh --with-tensorrt`, builder resources for `TRT_SMS`) and builds
`multiview-cli` with HyperPose for `CUDA_ARCHS`; CMake fetches HyperPose and
stdtensor, so the build needs network access. The runtime image keeps the
env's shared libraries, the binary, `data/` and the agent, and its entry point
runs `python3 -m mvcloud agent --cli /opt/multiview/bin/multiview-cli --cache
/cache/mvcloud --work /work "$@"` (any of `bash`, `multiview-cli`, `python3`
as the first argument runs that instead). `/cache` holds the HyperPose models
(downloaded on first start), the TensorRT engines, the undistortion maps and
the lazy-s3 cache: keep it on a volume. There is no systemd user session in a
container, so the agent cannot cap a shard's memory itself (it logs that and
runs the shard uncapped): use `docker run --memory`.

What was tried here (`tools/cloud/tests/t8_container.sh`): the image is
4.8 GB (the pruned micromamba env is 4.7 GB of it) and builds in 16-18
minutes with 4 compile jobs on this machine. The compile runs inside dockerd,
not niced and not under `capped`, so `build-image.sh` passes
`docker build --memory 24g`; the legacy builder (Docker 29 without buildx,
as here) applies it to every `RUN` step, BuildKit does not. Without a GPU (no
nvidia-container-toolkit here), a 20-frame 007 loc shard with replayed
poses and `use_slic_cuda_if_available` false runs in 29.5 s and its
envelope equals the GPU-built reference byte for byte. A fresh `/cache`
first regenerates the 12 undistortion maps of 007 (~2.5 min on 6 CPUs).
The HyperPose path in the container is untested. Of the ONNX models only
`openpose-thin` can still be downloaded by `download-hyperpose-models.sh`;
put the others (the default is `openpose-coco`) in the `/cache/hyperpose/models`
volume or set `HYPERPOSE_MODELS_ARCHIVE`. Docker on this machine cannot
bind-mount paths under `/home`, which is why T8 copies its inputs under
`$TMPDIR`.

## 2. The shard contract

A job is a scene, a frame range `[start, start+n)` and options. `K` is the
tracker's window (`max_frames_per_tracklet`, 10 by default; `scene_info -p`
reports the value the binary will use). The planner requires `start % K == 0`,
`(start + n) % K == 0` or `start + n` equal to the scene's frame count, and
`start + n <= frames`, and cuts the range into loc shards of `shard_frames`
(a last piece under 2 frames is folded into its neighbour).

| | Loc shard | Tracks job |
|---|---|---|
| Command | `pipeline --start S --n-frames N --save-loc-data --no-tracks [--no-stereo] [--no-still]` | `pipeline --load-loc-data e0 --load-loc-data e1 ... --start S0 --n-frames NT --require-aligned [--no-stereo] --pose-classifier pose-svm-lite_v0` |
| Product | `localization.data`: one `LocalizationDataEnvelope` (the existing binary format) | `pipeline-output.json`, `score.json` with ground truth |
| Rules | `--no-still` only with `--no-stereo` (exact: the still never reaches a no-stereo envelope) | never `--no-still`: the pose classifier reads the still (it relabels 98 of 120 path points of a track on 007) |
| Inputs | videos, params, poses (replay) | the envelopes, the videos (the tracker needs LAB images), params, ground truth |
| Checked by | `ldata info` against the lease's range; `done.json` holds per-frame md5s | `--require-aligned` in C++ (also refuses a range outside the envelopes) |

Repeated `--load-loc-data` concatenates envelopes in the order given (they
must tile the range); `multiview-cli ldata cat --keep s:n` merges overlapping
ones. `ldata diff` compares envelopes frame by frame as serialised bytes
(`operator==` is tolerance-based and skips fields, so it is reported but does
not decide). Every output is written atomically (temporary name, then
rename), so a killed shard never leaves a complete-looking file.

Exit classes (`runner.classify_exit`): 0 with every output present is `ok`;
parser errors, missing input, the alignment message, a failed save of the
envelope and a wrong envelope range are `fatal` (no retry, the job fails
fast); everything else, including `FATAL`/`Expects` aborts (issue #13),
signals and the watchdog's kills, is retried up to `max_attempts` with
exponential backoff.

Stereo runs are not exact: the still (a running mean of every sensor's
frames) feeds the stereo branch. `--still-warmup W` decodes and accumulates
`W` frames before `S`; `W = S` reproduces the batch's still. The trade-off
between `W` and accuracy is not measured yet (plan T9); until it is, shard
stereo jobs with `W = S` or not at all.

## 3. R2 layout

Bucket `multiview`, one prefix per environment:

```
scenes/<scene_id>/<md5>/manifest.json      the agent puts each input into its in/ under the name given; the manifest names videos by bare name
scenes/<scene_id>/<md5>/<camera>.mp4
scenes/<scene_id>/<md5>/params.json        a reference run's params.json; the agent sets replay_dir / use_hyperpose
scenes/<scene_id>/<md5>/poses.tar          pose-2d_<frame:06d>_<sensor:02d>.json (replay mode), uncompressed, no times or owners
scenes/<scene_id>/<md5>/ground-truth.json  optional (scoring)
scenes/<scene_id>/scene-info.json          `multiview-cli scene_info -p params.json` at upload time (frames, sensors, K) (planned; not built: `submit` runs scene_info locally)
cache/undistort/<basename>.mdata           inverse_<sensor>_*.mdata, ~40 MB each, so fresh workers skip regenerating them (planned; not built)
engines/<digest>.plan, <digest>.json       TensorRT plans per GPU type (deferred: workers keep them in /cache)
jobs/<job_id>/job.json
jobs/<job_id>/shards/loc-0003/a2/{localization.data, params.json, run-object.json, stdout.log, poses.tar?, done.json}
jobs/<job_id>/shards/tracks/a1/{pipeline-output.json, score.json?, stdout.log, done.json}
jobs/<job_id>/summary.json
```

Scene inputs are content-addressed (`<md5>` of the file; `mvcloud submit`
uploads them with their md5, which R2 verifies), so two jobs with different
params for one scene never overwrite each other's inputs and an unchanged
file is uploaded once. The filesystem backend uses the same `jobs/...` layout
under `<root>/blobs/` and reads scene inputs where they are. Every attempt
writes under its own `a<attempt>/` prefix, so a late or duplicate worker never
overwrites the winner; the Worker's finalize step deletes the losing
attempts' objects.

## 4. Throughput and cost

Measured on one RTX 3090 + Ryzen 5 3600 (shared with another worktree's jobs,
so wall times are upper bounds); 007 has 6 cameras, 008 has 9.

| Work | Per frame | Fixed per process | Source |
|---|---|---|---|
| Loc shard, replayed poses, `--no-stereo --no-still` | 0.72-0.74 s | ~12 s | C1 back-to-back runs; C3 fit of 20/60/120-frame shards |
| Loc shard with the still (needed for stereo) | 1.31 s | ~12 s | C1 |
| Tracks job (fast path, still kept) | 1.03-1.06 s | ~12 s | C2 (120 and 499 frames), C3 |
| Batch run, replayed poses (for comparison) | 1.28 s (499 frames: 638 s) | | C1 golden R1 |
| Loc shard, live HyperPose FP32, 008 (9 cameras) | 1.24 s (149 frames in one process); 1.40 s over the two shards with their fixed costs | ~12 s + 2.4-2.7 s engine load | T6 |
| Tracks job, 008 (9 cameras) | 1.59 s (237 s for 149 frames) | | T6 |
| Full 007 as 2 x 250 + tracks (T4) | loc 0.79-0.83 s, tracks 1.05 s: 929 s of GPU time against 638 s for the batch run | | T4 |
| TensorRT engine: build (cache miss) / load (hit) | | 60-82 s / 2.4-2.7 s | measurements.md; T6 |

Envelopes are 161-199 KB per frame on 007 (80.5 MB for 499 frames) and 497 KB
per frame on 008 (74 MB for 149 frames, 9 cameras); busier scenes will be
larger.

The corpus (~410k camera-frames, ~55k scene-frames at ~7.5 cameras per scene;
derived from the dataset card, not measured) then costs, per pass:

| | GPU-hours | Notes |
|---|---|---|
| Loc shards, replayed poses | ~11 (55k x 0.73 s) + fixed costs ~0.7 (200 shards x 12 s) | poses must exist (an earlier HyperPose pass) |
| Loc shards, live HyperPose FP32 | ~19 (55k x 1.2 s, measured on 008, + fixed costs) | the realistic first pass |
| Tracks jobs | ~16-24 (55k x 1.05 s on 6 cameras to 1.59 s on 9), one per scene, in parallel across machines | LAB conversion runs on the GPU when there is one (the CPU fallback is not timed) |

Two-phase sharding costs more GPU time than one batch run per scene: the
tracks job decodes every frame again and recomputes the still and the LAB
images, so on 007 with replayed poses the shards and the tracks job took
929 s against the batch run's 638 s (T4), and the tracks job alone (525 s)
bounds a scene's wall time from below. Sharding pays for long scenes,
for retries (a crash costs one shard, not a scene) and for spreading the pose
network over machines. For a corpus of short scenes, one shard per scene
(`shard_frames` at least the scene length) keeps the overhead to the tracks
job's second decode; a job kind that runs loc and tracks in one process would
remove it (not built).

On the local 3090 alone a full pass is about 35-45 hours. Rented
RTX 3090s cost about $0.2-0.5 per hour (community clouds; check current
prices), so four of them bring a pass to a working day for roughly
$8-25. R2 storage for ~10 GB of videos and 10-30 GB of envelopes is
under $1 per month at $0.015/GB-month with no egress fees (Cloudflare's list
price when the plan was written; check). Workers Paid ($5/month) covers the
control plane's request and Workflow-step volume for a corpus pass.

FP16 (`"precision": "fp16"` in the HyperPose params) is a small lever for loc
shards: 008 [0,149) in one process took 170 s against 184 s at FP32 (not
back to back, on a shared machine), with 876 against 873 2D detections, because
decoding, LAB and SLIC on the CPU, not the network, dominate a no-stereo frame.
The thin model is not measured.

## 5. The public videos are face-blurred

Every video in the public dataset has blurred faces. HyperPose finds few
people on them (recall 0.15 on 007), so live-pose runs on the public videos
score far below the published numbers, by design. The pose replay set in
`~/multiview-data/pose-replay` was computed on the original videos; with it
007 scores TP 159 / FP 46 / FN 81 (F1 0.556) on 120 frames and TP 609 / FP 83 /
FN 300 (F1 0.614) on 499. That is why every gate here compares shard output
with batch output of the same configuration, never with ground truth, and
why the HyperPose smoke (T6) records its agreement instead of a score.

## 6. Open questions from issue #14, and what this prototype recommends

| Question | Recommendation |
|---|---|
| Throughput (process the corpus once) or a service (anyone uploads)? | Throughput first. The two-phase shard design serves both, but a service adds upload handling, quotas and the licence question below; the corpus pass is the immediate need and takes 35-45 GPU-hours on one 3090 (section 4). |
| Budget for cloud GPUs, or the local machine plus occasional spot? | The local 3090 is the first worker (the agent shares the GPU lock with development work). Rent spot/community GPUs only to shorten a pass; the cost is small (section 4) and the image is meant to run anywhere with `--gpus all` (its GPU path is untested here). |
| R2 primary with Hugging Face as a mirror, or Hugging Face only? | R2 as the working store (S3 API, no egress fees, presigned URLs for workers), Hugging Face as the public mirror and dataset card. The C++ client reaches R2 with `MULTIVIEW_S3_ENDPOINT`. |
| Licensing for a hosted service (AGPL-3.0 Ultralytics models in `tools/privacy`) | No hosted service until the detector licences are settled: besides the AGPL detectors, the OpenPose-derived COCO weights used through HyperPose carry CMU's non-commercial licence (to verify). Running the corpus on our own hardware is unaffected. |
| Does a windowed run score within noise of a full run? | With `--no-stereo` it scores the same, by construction: loc shards share no state, so their localization data is byte-identical to a full run's, and one tracks job runs the unchanged tracker over all frames, so its tracks and score equal the full run's except `fp_info.lab_p`, which depends on the still. Stereo needs the warm-up calibration (T9), not done yet. |
| Crash rate x fan-out | Shards are idempotent and retried (3 attempts), and the agent's watchdog kills a hung child. The crash rate itself is not measured (the plan's T7/T7b runs were not done). One observation: a `FATAL` on a pool thread (a missing HyperPose model) aborted twice and hung once until killed, which is issue #13's exit-from-worker hazard; its fixes stay a prerequisite for a 1,000-shard pass. |
