# Design: sharded and cloud-distributed runs (`tools/cloud`)

This document describes the `cloud-distributed` branch for a reviewer: what
it adds, how the pieces fit, why they are built this way, how they were
tested, and what is not done. It implements the prototype of
[issue #14](https://github.com/prcvlabs/multiview/issues/14) (run multiview
over long scenes and the whole corpus on more than one GPU). The runbook,
with every option, is [`tools/cloud/README.md`](../../tools/cloud/README.md).

## 1. Goal and scope

`multiview-cli pipeline` processes one scene in one process: the per-frame
DAG over every frame, then the tracker over all of them. A 499-frame,
6-camera scene takes about 11 minutes on an RTX 3090; the public corpus is
about 55,000 scene-frames. A crash costs the whole scene, and one machine
does all of it.

This branch cuts a scene's frames into time windows ("loc shards") that any
GPU machine computes independently, and runs the tracker once per scene
("the tracks job") over the concatenated results. A small control plane
hands shards to agents, retries them and records their outputs, either on a
shared filesystem or on Cloudflare (a Worker with D1, R2 and Workflows).

The acceptance criterion is exactness, not speed. With `--no-stereo` and a
fixed set of 2D poses, the shards' localization data must be byte-identical
to one batch run's, and the tracks job's `pipeline-output.json` must equal
the batch run's except `fp_info.lab_p` (section 8). Throughput and cost are
measured and reported, not gated. Stereo sharding is supported but not
calibrated, and nothing is deployed to Cloudflare (section 9).

## 2. Architecture

```
  mvcloud submit job.json
        | plan: aligned job range -> loc shards [S0,S1) [S1,S2) ... + tracks
        v
  +----------------------------------------------+
  | control plane                                |    blobs
  |  fs:///root  : json rows under one flock     |    fs: <root>/blobs/
  |  https://... : Worker, D1 rows, a Workflow   |    R2: through the Worker
  |  jobs, shards, attempts, leases with tokens  |
  +----------------------------------------------+
        ^  pull / heartbeat / complete / fail
        |
  +------------------+   one shard at a time per GPU
  | mvcloud agent    |   GPU lock (flock), own session, nice 15,
  | (Python, stdlib) |   memory cap, wall-clock and progress watchdog
  +------------------+
        | runs
        v
  loc shard k:   multiview-cli pipeline --start Sk --n-frames Nk
                   --save-loc-data --no-tracks [--no-stereo --no-still]
                   -> localization.data (LocalizationDataEnvelope)
                   -> done.json (per-frame md5s, params md5)
                                   |
             all loc shards done   |  (the tracks job is promoted)
                                   v
  tracks job:    multiview-cli pipeline --load-loc-data e0 --load-loc-data e1
                   ... --start S0 --n-frames N --require-aligned
                   --pose-classifier pose-svm-lite_v0
                   -> pipeline-output.json, score.json (with ground truth)
```

### 2.1 Components

| Piece | Where | What it does |
|---|---|---|
| Shard flags | `pipeline` (`cli-args.cpp`, `execute.cpp`) | repeatable `--load-loc-data`, `--require-aligned`, `--no-still`, `--still-warmup`, atomic outputs |
| Envelope operations | `cost-functions/localization/envelope-ops.{hpp,cpp}` | `cat_envelopes` (one merge for `--load-loc-data` and `ldata cat`), byte-level `diff_envelopes`, `envelope_info` |
| Alignment guard | `pipeline/shard-range.{hpp,cpp}` | `shard_alignment_error`: the tracks job's range against K and the loaded envelopes |
| Front-ends | `front-ends/{ldata,score,scene-info}` | `ldata info/cat/diff`, `score` (and `within_noise`), `scene_info` |
| Scoring | `pipeline/test-output.cpp`, `pipeline/noise-gate.{hpp,cpp}` | `TestOutput::to_json`, `within_noise` tolerance gate |
| S3 client | `io/s3.{hpp,cpp}`, `config.cpp` | `MULTIVIEW_S3_ENDPOINT/REGION/PATH_STYLE/PROFILE` for R2 or MinIO; `-m s3://...` manifests |
| Agent | `tools/cloud/agent/mvcloud` | planner, runner, watchdog, storage, the `fs://` backend and the HTTP client of the Worker |
| Worker | `tools/cloud/worker` (TypeScript) | API, D1 schema and lease protocol, one Workflow per job, R2 blob proxy, status page |
| Container | `tools/cloud/docker` | two-stage image: the micromamba env, `multiview-cli` with HyperPose, the agent |
| Tests | `tools/cloud/tests`, `src/testcases/...` | Catch cases, agent unit tests, vitest, end-to-end scripts |

### 2.2 Changes to existing code

All small, and all on the main thread or removing work:

- `save_localization_data` deep-checks the saved file against the frames in
  memory instead of recomputing every frame that has left the DAG's cache
  (a `--save-loc-data` run is 38% faster); its exceptions are caught at the call
  site inside the `noexcept` `run_pipeline`.
- `SceneDescription::InitOptions::accumulate_still` (`--no-still`) skips the
  running still in `get_images_for_frame`.
- In no-stereo runs, `--load-loc-data` computes only the LAB images the
  tracker needs (`load_lab_images_pt_clouds`): no rectification, SLIC or
  disparity. The tasks are requested parents first, so each runs on the
  calling thread; requesting `convert_to_lab` directly hung 3 of 3 runs
  (a pool worker waiting on a job queued behind itself, issue #13).
- The CUDA SLIC context is created on first use, and a CUDA error inside
  `run_f2d` falls back to the CPU SLIC instead of leaving the `noexcept`
  node, so the CUDA build runs on a machine without a GPU.
- `file_put_contents_atomic` (temporary name, then rename) for
  `params.json`, `pipeline-output.json` and the envelope.

No new pool code, no new helping waits, and no `exit()` or `FATAL` on a pool
thread.

## 3. Decisions and why

| Decision | Why | Alternatives rejected |
|---|---|---|
| Two phases: loc shards, then one tracks job per scene | The tracker is a chain over 10-frame windows in which window k rewrites window k-1; running it once over all frames gives batch tracks by construction | Windowed tracking with id stitching across shards (inexact, new code) |
| The shard product is the existing `LocalizationDataEnvelope` | `--save-loc-data` / `--load-loc-data` already exist; no new format | Serialising `Tracks` or tracker state |
| Exactness gated in bytes | `LocalizationData::operator==` has a tolerance and skips `prob_xy`; `ldata diff` serialises each frame and compares bytes, and reports `operator==` only as information | A score against ground truth (the public videos are face-blurred; section 8) |
| Only the tracks job is aligned to K | Loc shards share no state in no-stereo; the tracker's windows start at multiples of K, so the job range must be aligned. The planner checks it and `--require-aligned` refuses it again in C++ | Aligning every shard |
| `--no-still` on no-stereo loc shards only | The still never reaches a no-stereo envelope (measured: byte-identical with and without), and it costs 0.56 s per 6-camera frame. The pose classifier reads the still (`bg_delta_score`), so the tracks job keeps it: with `--no-still` it relabels 98 of 120 path points of a track on 007 | `--no-still` everywhere (as first planned) |
| `--still-warmup W` for stereo shards | The still feeds the stereo floor histogram; decoding W frames before the start rebuilds it; `W = S` reproduces the batch's still | Overlapping shards |
| Python standard library agent, one shard per GPU | Runs on a bare worker and in the container; the shard is a `multiview-cli` process, so a crash (issue #13) costs one attempt | A C++ or third-party agent |
| The agent holds the GPU lock and the child's session | Signals reach the child and every process it forks; the lock is the one `build/dev.sh` uses, so shards queue behind development work | A forking lock wrapper (signals stop at the wrapper) |
| Leases with per-attempt tokens; outputs under `a<attempt>/` | A late or duplicate worker cannot overwrite or complete the winner | Overwriting one output prefix |
| The wall-clock budget counts from the GPU lock | On a shared GPU an agent can wait many minutes for the lock; with the budget counted from the lease, the first end-to-end run lost its lease while it waited | Counting from the lease |
| The Worker API is the agent's only contract; D1 is the authority | The filesystem backend implements the same protocol, so every test of the agent applies to both; a Queue message is a hint, never state | Queue-driven state |
| One Workflow instance per job | Plan, wait, promote, finalize as durable steps; the last loc shard's `complete` promotes the tracks job itself, so a stalled instance does not strand a job | A cron loop over all jobs |

## 4. The shard contract

A job is a scene, a frame range `[start, start + n)` and options. K is the
tracker's window (`max_frames_per_tracklet`, 10 by default; `scene_info -p`
prints the value the binary will use). The planner requires
`start % K == 0`, `(start + n) % K == 0` or `start + n` equal to the
scene's frame count, and `start + n <= frames`, and cuts the range into loc
shards of `shard_frames` (default 300; a last piece under 2 frames joins
its neighbour).

| | Loc shard | Tracks job |
|---|---|---|
| Command | `pipeline --start S --n-frames N --save-loc-data --no-tracks [--no-stereo] [--no-still]` | `pipeline --load-loc-data e0 --load-loc-data e1 ... --start S0 --n-frames N --require-aligned [--no-stereo] --pose-classifier pose-svm-lite_v0` |
| Inputs | videos, params, poses (replay) or the HyperPose models | the envelopes, the videos (the tracker reads LAB images), params, ground truth |
| Product | `localization.data`, `done.json` (per-frame md5s, `params_md5`) | `pipeline-output.json`, `score.json` |
| Checked by | `ldata info` against the lease's range | `--require-aligned` in C++, and the planner |

Exit classes: 0 with every output present is `ok`. Parser errors, missing
input, the alignment message, a failed save and a wrong envelope range are
`fatal` (no retry; the job fails fast). Everything else, including
`FATAL`/`Expects` aborts, signals and the watchdog's kills, is retried up
to `max_attempts` (3) with exponential backoff.

## 5. CLI and configuration

### 5.1 `multiview-cli`

| Command or flag | Effect |
|---|---|
| `pipeline --load-loc-data <f>` (repeatable) | concatenate the envelopes in order; they must tile the range |
| `pipeline --require-aligned` | fail unless the start and end are multiples of K (or the end is the movie's end) and the loaded data covers them |
| `pipeline --no-still` | do not accumulate the still; refused with `--still-warmup` and `--export-training-data`; warns with a pose classifier or without `--no-stereo` |
| `pipeline --still-warmup W` | decode the W frames before `--start` into the still only |
| `pipeline -m s3://.../manifest.json` | the manifest and its videos through the lazy-s3 cache |
| `ldata info <f>...` | json: range, fps, geometry, detections, size, md5, per-frame md5s |
| `ldata cat [--keep s:n]... -o out in...` | concatenate (and trim overlaps); reports overlapping frames compared |
| `ldata diff a b` | exit 0 when the ranges are equal and every frame is byte-equal, 1 when not, 2 on error |
| `score --calc run.json --gt gt.json` | score a `pipeline-output.json` as `--pipeline-testcase` does |
| `score --calc run.json --ref ref.json [--gt] --within-noise` | agreement with a reference run, and a tolerance gate (`--tolerances`, `--boundaries`) |
| `scene_info -m manifest [-p params]` | frames, cameras, sensors, fps and the effective K, as json |

| Environment variable | Default | Effect |
|---|---|---|
| `MULTIVIEW_S3_ENDPOINT` | AWS | e.g. `https://<account>.r2.cloudflarestorage.com` |
| `MULTIVIEW_S3_REGION` | `auto` with an endpoint | signing region |
| `MULTIVIEW_S3_PATH_STYLE` | on with an endpoint | bucket in the path |
| `MULTIVIEW_S3_PROFILE` | `multiview` | exported as `AWS_PROFILE`; empty keeps the caller's |

### 5.2 The agent

`PYTHONPATH=tools/cloud/agent python3 -m mvcloud <command>`:

| Command | Effect |
|---|---|
| `plan --cli <bin> job.json` | print the shards without running anything |
| `submit --control <url> --cli <bin> job.json` | plan the job (running `scene_info` when the job lacks the frame count, K or cameras) and queue it; through the Worker, upload the inputs content-addressed |
| `agent --control <url> --cli <bin> [--once \| --drain]` | lease and run shards; `--mem` (24G), `--kill-grace` (20 s), `--poll` (15 s), `--gpu-lock`, `--work`, `--cache` |
| `status`, `cancel <job>`, `sweep [--verify]` | inspect; cancel (reaches running shards through the heartbeat); expire leases and re-queue shards whose outputs are gone or corrupt |

`<url>` is `fs:///path/to/root` or the Worker's `http(s)://` URL (tokens in
`MVCLOUD_ADMIN_TOKEN` and `MVCLOUD_WORKER_TOKEN`). Job keys and their
defaults are `fsbackend.k_job_defaults`: `shard_frames` 300, `no_stereo`
true, `pose_mode` `replay` or `hyperpose`, `max_attempts` 3,
`lease_seconds` 1800, `heartbeat_s` 60, `progress_timeout_s` 900,
`extra_cli_args`.

### 5.3 The Worker

`POST /v1/jobs`, `GET /v1/jobs[/<id>]`, `POST /v1/jobs/<id>/cancel` (admin
token); `POST /v1/pull | claim | heartbeat | complete | fail`,
`/v1/blobs/<key>`, `GET /v1/workers`, `GET /v1/health` (worker token);
a status page at `/`. Tokens travel only as `Authorization: Bearer`, except
the status page, which moves `?token=` into an HttpOnly cookie. Every shard
transition is one conditional D1 `UPDATE` on (state, lease token, attempt),
checked through `meta.changes`; a retried `pull` returns the lease whose
reply was lost, and `submit` names the job so a retried `POST /v1/jobs`
returns the first one. `wrangler.jsonc` documents the bindings and the
deployment steps.

## 6. How to run and test

Prerequisites: the `multiview` micromamba env and `build/dev.sh build`
(`--hyperpose` for live poses), the testcases from
`build/fetch-testcases.sh`, and for replay jobs the 2D poses under
`$MULTIVIEW_TESTCASE_DATA/pose-replay/<scene>/`. The references are made
once with `tools/cloud/tests/make_reference.sh --frames 120` (and `499`).

A job on the local GPU:

```
export PYTHONPATH=tools/cloud/agent
CLI=multiview/multiview_cpp/build/cmake-release/multiview-cli
python3 -m mvcloud submit --control fs://$HOME/mvcloud-root --cli $CLI job.json
python3 -m mvcloud agent  --control fs://$HOME/mvcloud-root --cli $CLI --drain
python3 -m mvcloud status --control fs://$HOME/mvcloud-root <job_id>
```

`job.json` names the scene, its manifest, a per-scene params file
(`multiview-cli dump_default_params`), the poses, the ground truth and the
range; the README has a complete example.

| Command | What it checks |
|---|---|
| `build/dev.sh check` | build, the Catch suite (new tags `[ldata]`, `[shard]`, `[cli-args]`, `[score]`, `[within-noise]`, `[s3]`, `[fs]`; `[.still]` runs by name, `[s3-live]` needs a bucket), the 120-frame smoke |
| `python3 -m unittest discover -s tools/cloud/tests` | the agent against `fake_multiview_cli.py`: planner, params digest, storage, process control, fs backend, agent loop, HTTP client |
| `npm test` in `tools/cloud/worker` (env `mvcloud`) | vitest: the planner against the Python planner's golden file, the D1 protocol on the real migration, blob cleanup |
| `tools/cloud/tests/e2e_fs.sh` | the gate: T1 (2 x 60 frames), T3 (1 x 120), T5 (idempotency, fault injection, refusals) |
| `e2e_fs.sh --tests T4` / `--tests T6` | full 007 as 2 x 250; 008 with live HyperPose |
| `tools/cloud/tests/e2e_wrangler.sh` | T2: the same job through `wrangler dev`, with an agent killed mid-shard; T2c cancel; T2t timeout |
| `tools/cloud/tests/check_shard_parity.sh` | the C++ gate without the agent |
| `tools/cloud/tests/t10_minio.sh`, `t8_container.sh` | the S3 client against MinIO in Docker; the image on the CPU |

## 7. Measured numbers

One RTX 3090 and a Ryzen 5 3600, shared with another worktree's GPU jobs
during most runs, so wall times (measured inside the GPU lock) are upper
bounds. Testcase 007 has 6 cameras and 499 frames; 008 has 9 cameras and
149 frames. Replayed poses and `--no-stereo` unless stated.

| Work | Per frame | Notes |
|---|---|---|
| Batch run (reference R1) | 1.37 s (120 frames), 1.28 s (499) | 163.9 s, 638.1 s |
| `--save-loc-data` before the deep-check fix | 2.18 s | 120 frames, back to back |
| `--save-loc-data` after the fix | 1.31 s | 120 frames, back to back |
| Loc shard with `--no-still` | 0.72-0.74 s, + ~12 s per process | fit of 20/60/120-frame shards |
| Tracks job before the LAB-only fast path | 1.91 s | 228.6 s for 120 frames |
| Tracks job with the fast path (still kept) | 1.03-1.06 s | 126.4 s (120), 513.5 s (499) |
| Full 007 as 2 x 250 + tracks (T4) | loc 0.79-0.83 s, tracks 1.05 s | 929 s of GPU time against 638 s for the batch |
| Loc shards, live HyperPose FP32, 008 | 1.24 s in one process; 1.40 s over two shards | engine loaded from the cache in 2.4-2.7 s |
| Tracks job, 008 | 1.59 s | 237 s, 8 tracks |
| HyperPose FP16, 008, one process | 1.14 s (170 s) against 1.24 s (184 s) at FP32 | 876 against 873 detections; not back to back |
| CPU-only container, 20-frame loc shard | 29.5 s | the envelope equals the GPU reference byte for byte |
| T2 through `wrangler dev`, GPU free | 273 s end to end | loc 56.9 s and 55.4 s, tracks 129.8 s |

Envelopes are 161-199 KB per frame on 007 and 497 KB per frame on 008.
Two-phase sharding costs about 46% more GPU time than one batch run on 007,
because the tracks job decodes every frame again and recomputes the still
and the LAB images; the tracks job alone (82% of the batch) bounds a
scene's wall time from below. Sharding pays for long scenes, retries and
spreading the pose network over machines. The corpus (about 55,000
scene-frames, derived from the dataset card) is about 35-45 GPU-hours per
pass on one 3090 with live HyperPose; section 4 of the README has the
breakdown and the cost of renting GPUs.

## 8. Parity and exactness results

Every gate compares shard output with batch output of the same
configuration, never with ground truth: the public videos are face-blurred,
so live HyperPose finds few people on them (recall 0.15 on 007). The
replayed poses were computed on the original videos; with them 007 scores
TP 159 / FP 46 / FN 81 (F1 0.556) on 120 frames and TP 609 / FP 83 /
FN 300 (F1 0.614) on 499.

`fp_info.lab_p` is excluded from every tracks comparison: it is computed
from the still, which the tracks stage snapshots while later frames are
still being processed, and in a tracks job the still lacks frame 0, which
`bcam_init` adds in a batch run. It is not an input of the keep/drop score.

| Test | Result |
|---|---|
| T0a: `--load-loc-data` of the batch envelope against the batch run, `lab_p` nulled | EQUAL at 120 frames (2 tracks) and 499 (5 tracks) |
| `--no-still` envelope against the envelope with the still | byte-identical at 120 and 499 frames |
| `--no-still` on the tracks job | not equal: 98 of 120 path points relabelled ('I' for 'W'); hence never used there |
| `check_shard_parity.sh`: [0,60) + [60,120) in C++ | `ldata diff` exit 0, 120/120 frames byte-equal; tracks EQUAL; unaligned end and a start outside the envelopes refused (exit 1, under 4 s) |
| T1 [0,120) as 2 x 60, and T3 as 1 x 120, through the agent (fs backend) | `ldata diff` of the merged envelopes against the batch envelope: identical, 120/120 frames byte-equal; tracks EQUAL to the batch run (2 vs 2, `lab_p` excluded); `score.json` equal; one `done` attempt per shard, one params md5, the reference's 120 frame md5s; no surviving process. T1: loc 54.7 s and 56.3 s, tracks 129.5 s, 249 s end to end; T3: loc 99.6 s, tracks 128.9 s |
| T5: idempotency, corruption, `kill-after:20`, refusals | rerun changes nothing; a corrupted envelope is re-run, then the tracks job, and every T1 check holds again; a killed shard is retried once and the job finishes; `n_frames` 1 and past the scene refused at submit; an unaligned tracks job fails after one fatal attempt |
| T4: full 007, [0,250) + [250,499) | 499/499 frames byte-equal; tracks EQUAL (5 vs 5); score equal: TP 609 / FP 83 / FN 300, F1 0.614, 0 id switches, 17 frags |
| T2: [0,120) through `wrangler dev`, agent A killed mid-shard | 120/120 byte-equal; tracks EQUAL; score equal; loc-0000 attempts `[expired, done]`; agent B reaps A's orphaned session; the Workflow completes with no losing objects left; envelopes over 8 MiB uploaded in parts. T2c (cancel) and T2t (timeout, retried, then dead) pass |
| T6: 008 with live HyperPose FP32, [0,80) + [80,149) | 149/149 frames byte-equal to a one-process run (recorded, not gated: TensorRT need not be reproducible across processes) |
| T10: `-m s3://` through MinIO, 30 frames | byte-equal to the reference, and again from the filled cache |
| `score` of R1 against `--pipeline-testcase` | TP 159 / FP 46 / FN 81, F1 0.5559, 0 / 5 equal; theta bias and stddev differ in the 7th significant digit (gaze is stored with 6) |

Crash rate is not measured. One observation: a `FATAL` on a pool thread (a
missing HyperPose model) aborted twice and hung once for 492 s until the
agent was stopped, which is issue #13; the agent now classifies that
message as fatal.

## 9. Stubbed, deferred and known limits

- **Nothing is deployed.** There is no Cloudflare account: D1, R2, Queues
  and Workflows ran only under `wrangler dev` (Miniflare); production limits,
  the 100 MB request-body limit and Workflow step pricing are not measured.
  The D1 `database_id` in `wrangler.jsonc` is a placeholder.
- **Deferred:** the Queue's HTTP pull consumer and presigned R2 URLs
  (`--dispatch queue` is refused; `/v1/pull` dispatches and the Worker
  proxies blobs); the TensorRT engine upload to R2; `scene-info.json` and
  the undistortion maps in R2 (`submit` runs `scene_info` locally, and a
  fresh worker regenerates 007's 12 maps in about 2.5 minutes); a
  `corpus-push` command.
- **Stereo shards** use `--still-warmup`; how small W can be (plan T9) is
  not measured, so stereo shards should use `W = S` or one shard.
- **The container's GPU path is untested** (no nvidia-container-toolkit
  here); the image is 4.8 GB, and only `openpose-thin` can still be
  downloaded by `download-hyperpose-models.sh`.
- In a container the agent cannot cap a shard's memory itself (no systemd
  user session); use `docker run --memory`.
- The lazy-s3 cache is keyed by `bucket/key`, not by endpoint or ETag.
- The correlated-failure pause exists in the Worker (unit-tested) but not in
  the filesystem backend, and is not exercised end to end.
- Miniflare logs a "code had hung" error when a cancelled job's Workflow
  ends; the instance reports `complete` and later requests succeed. The
  cause is not found.
- The agent's unit tests take 30-40 s, because the watchdog and heartbeat
  cases wait in real time.

## 10. Open questions

| Question (issue #14) | What the prototype suggests |
|---|---|
| Throughput over the corpus, or a service anyone uploads to? | Throughput first; the shard design serves both, but a service adds uploads, quotas and the licence question |
| Local GPU or cloud GPUs? | The local 3090 is the first worker (it shares the GPU lock with development); rented GPUs only to shorten a pass |
| R2, Hugging Face, or both? | R2 as the working store (S3 API, no egress fees), Hugging Face as the public mirror |
| Licences for a hosted service | No hosted service until settled: the privacy tools use AGPL-3.0 detectors, and the OpenPose-derived weights carry a non-commercial licence (to verify) |
| Does a windowed run score within noise of a full run? | With `--no-stereo` it scores the same, by construction and as measured; stereo needs the warm-up calibration |
| Crash rate times fan-out | Shards are idempotent and retried; issue #13's fixes remain a prerequisite for a 1,000-shard pass |
