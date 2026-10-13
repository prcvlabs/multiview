# Design: the real-time pipeline (`multiview-cli realtime`)

This document describes the `realtime` subcommand for a reviewer of the
`realtime-pipeline` branch: what it is, how it is built, why it is built that
way, how it was tested, and what it does not do yet. It implements the
prototype of [issue #2](https://github.com/prcvlabs/multiview/issues/2)
(real-time on-site tracking). The user documentation, with every flag and
the output format, is [`doc/realtime.md`](../realtime.md).

## 1. Goal and scope

`pipeline` processes a whole movie: it runs the per-frame DAG on every
frame, then the tracker over all of them, then writes `pipeline-output.json`.
A site needs positions and track ids while people are still in the room.

`realtime` runs the same per-frame DAG and the same tracker on frames as
they arrive, and publishes every result as soon as it is known, as one JSON
object per line (NDJSON) on a file, stdout, or a WebSocket. The input is
the scene's videos replayed at a wall-clock rate; there is no camera source
yet. The acceptance criterion is parity, not speed: run unpaced and without
drops, `realtime` must write the batch's `pipeline-output.json`, point for
point. Latency and throughput are measured and reported.

## 2. Architecture

```
                                         per-frame DAG (FrameResults),
 paced source        dispatcher          N slots on the thread pool
 +--------------+    +-------------+     +-----------------------------+
 | a decoder    |--->| drop newest |---->| slot 0: copy_sensor_images  |
 | thread per   |ring| or wait     |     |   -> LAB, SLIC, pose,       |
 | camera; one  |    | (own thread)|     |      localization           |
 | scene frame  |    +-------------+     | slot 1: ...                 |
 | per tick     |          |             +-----------------------------+
 +--------------+          | dropped             | results, any order
                           v                     v
                     +----------------------------------+
                     | reorder buffer (own thread):     |---> provisional
                     | frame order, coalesced drop      |     positions
                     | ranges, --missing-timeout        |     ("frame")
                     +----------------------------------+
                                     | FrameRecord (or a gap)
                                     v
                     +----------------------------------+
                     | online tracker (own thread):     |---> "tracks"
                     | batch window code, K = 10 frames |     tracked/revised,
                     | at a time; TrackStream smoothing |     "smoothed",
                     +----------------------------------+     "track_end"
                                     |
                                     v at the end
                       make_tracker_result + finalise_tracks
                       -> pipeline-output.json (as batch)

 all messages -> publisher (one thread) -> file / stdout (never drops)
                                        -> WsServer (Beast, one thread;
                                           per-client queue, drops oldest)
```

### 2.1 Components

| Component | File (under `src/perceive/pipeline/realtime/`) | Role |
|---|---|---|
| `PacedFrameSource` | `paced-frame-source.*` | One `cv::VideoCapture` and decoder thread per camera; puts whole scene frames into a ring at `--replay-rate` (0: as fast as the consumer takes them). Frame offsets come from the manifest through `calc_movie_frame_offset`, so frame `k` is the picture batch reads for frame `k`. |
| `FrameSlotPool`, `SlotPool` | `frame-slot-pool.*`, `slot-pool.hpp` | N copies of `FrameResults` (the per-frame DAG). Each slot is warmed up once (`bcam_init`, `get_xy_mapping`) before the clock starts. A slot is never cancelled: dropping means not dispatching. |
| Images provider | `nodes/copy_sensor_images.*` | `copy_sensor_images` takes its frame from a provider (the source's ring) instead of seeking the scene's videos; batch leaves it unset. |
| `ReorderBuffer` | `reorder-buffer.*` | Puts results back in frame order, keeps drops as coalesced ranges `{from, to, reason}`, and gives up on a frame after `--missing-timeout` (reason `late`). |
| `ProvisionalLocalizer` | `provisional-localizer.*` | Clusters a frame's detections and positions each cluster with the tracker's own estimator; labels it with the nearest live track (`pid`). |
| `OnlineTracker` | `online-tracker.*` | Runs `Tracklet::execute` and the tracks stage, unchanged, on each complete window, on its own thread. Gaps become `LocalizationData::make_empty` frames that borrow the last good frame's LAB images and point clouds. |
| `TrackStream` | `track-stream.*` | Smooths each track's tail with the batch `KzFilter` and publishes the points that can no longer change. |
| `RealtimeStats`, `LatencyStats` | `latency-stats.*` | Rates, queue depths, drops by reason, latency percentiles, the overload rule. |
| `Publisher` | `publisher.*` | Serialises each message once and writes it to every sink, in order; adds a `stats` line every `--stats-period`. |
| `WsServer` | `ws-server.*` | `ws://HOST:PORT/v1/stream`, `GET /v1/health`, `GET /v1/metrics`. Pimpl around Boost.Beast. |
| `RealtimeRun` | `realtime-pipeline.*` | Wires the stages, owns start-up and the stage-ordered shutdown, writes the outputs. |
| Front-end | `src/front-ends/realtime/realtime.cpp` | The subcommand, in the shape of `dump-default-params.cpp`. |

### 2.2 Threads and locks

The new threads are the decoders, the dispatcher, the reorder buffer, the
tracker, the publisher and the server. They use `std::mutex`,
`std::condition_variable` and `std::lock_guard<std::mutex>` only, and
`BoundedQueue` (built on those). No new code path uses the helping
`perceive::lock_guard` or `ParallelJobSet` (issue #13). No pool job blocks
on a realtime stage: the slot's completion callback uses `try_push`, and the
dispatcher's admission rule (`n_busy + loc_q.size() < capacity`) makes sure
the push cannot fail. Pipeline nodes stay `noexcept` and catch everything;
nothing on a pool thread calls `exit()` or `FATAL`.

Shutdown is stage-ordered (source, dispatcher, slots drained, reorder
flushed, tracker finished, publisher, server), so every frame a slot took is
published before the `end` line, and SIGINT drains rather than aborts.

## 3. Decisions, and why

**The tracker is the batch tracker, fed the batch inputs in order.** The
tracks stage is a Markov chain over 10-frame windows in which window k
rewrites window k-1 in place. Any reimplementation would drift from batch;
running the unchanged window code on the same per-frame inputs gives parity
by construction. The end of the tracks stage (`make_tracker_result`,
`finalise_tracks`, `smooth_track`, `calc_fp_info`) is factored out of
`TrackCalculator` so that batch and online use one copy of it.

**Three revisions per frame.** A frame's tracks are published when its
window closes (`tracked`), again after the next window has rewritten them
(`revised`: ids and raw positions are then final, `id_remap` says which ids
changed), and its smoothed points when the smoothing window has passed
(`smoothed`, final for `t <= final_upto_t`). The horizon is read from the
filter (`KzFilter::delta_t()`, 16 frames for the default m = 9, k = 4), not
hard-coded.

**Provisional positions before tracking.** Tracking needs a full window, so
ids arrive a window or more late. Each processed frame first gets
`frame` lines: detection clusters positioned with the tracker's estimator,
with the nearest live track's id within `--match-radius`.

**Slots instead of a parallel DAG per node.** `FrameResults` already
parallelises within a frame. Running N copies side by side overlaps frames
without touching the DAG; a slot is a unit of admission, and a frame that
cannot get a slot is dropped (`--drop newest`) or waited for
(`--drop none`). Measurement made the default one slot (section 6).

**Absolute frame numbers and gap records.** Dropped frames keep their scene
frame number and reach the tracker as frames without evidence
(`LocalizationData::make_empty`), so `t` in the output is the scene frame and
`compare_tracker_output` scores it like batch.

**The still is sampled, not threaded.** In `--no-stereo` only `lab_p` (the
tracks' appearance score) depends on the running still, and adding every
frame makes `copy_sensor_images` cost 745 ms per 9-camera frame instead of
32 ms (T 0.53 instead of 0.82 fps). `copy_sensor_images` adds a frame
when `frame_num % still_sample_period == 0`, up to `still_max_frames`;
batch keeps the defaults (every frame, no limit). This is why `lab_p` is the
one field excluded from parity.

**Slot warm-up instead of sharing scene results between slots.** Each slot
computes `bcam_init` and `get_xy_mapping` once at start-up. Sharing them
would mean changing `shallow_copy_params_result`, a core `TaskNode`
invariant; the warm-up costs seconds, not a design change.

**WebSocket last and small.** NDJSON to a file or stdout is the contract;
the server is one Beast listener behind a Pimpl, with a bounded queue per
client that drops the oldest line, so a slow client never slows the run.

**Rejected:** detecting poses on every second frame (interpolated skeletons
are rejected downstream, so localization has no evidence); renumbering
frames contiguously after drops (the output could not be scored); the
`openpose-thin` model (no faster in the pipeline, worse tracks).

## 4. Changes to existing code

Most of the branch is new files. These edits touch code that `pipeline`
also runs:

| Change | Effect on batch |
|---|---|
| `copy_sensor_images`: images provider, still sampling params | none (defaults: no provider, every frame) |
| `bcam_init` reads the working format instead of decoding frame 0 | `--start 0`: none. `--start N > 0`: frame 0 is no longer added to the still, which changes stereo output and `lab_p` |
| `calc_rectified` is skipped in `--no-stereo` | none (output identical, `lab_p` included); ~22 ms CPU per sensor per frame saved |
| `convert_to_lab` uses each node's own converter | none (output identical); the 9 conversions of a frame run on the GPU side by side |
| `fowlkes`, `tracker-results`, `run-tracklets-algorithm-1`, `tracks-task`: end of the tracks stage shared | none (moved code, byte-equivalent; smoke identical) |
| `LocalizationData::make_empty` | new function |
| `ops.cpp`: guard for a window with no still sample | none at `--start 0` |
| `KzFilter`: window-size comment corrected | none |
| `scene-description`: frame loading split from still accumulation; `seek_frame_(-1)` guard | none |
| `build/dev.sh`: `rt-smoke`, part of `check` | dev loop only |

The rectification skip and the per-node LAB converter speed up `pipeline`
too, and could move to `build-fast-iteration` on their own (`79bda2d`,
`a1b58d1`).

## 5. CLI and configuration

`realtime` accepts every `pipeline` flag and adds: `--replay-rate`,
`--drop {auto,newest,none}`, `--slots` (default 1), `--ring`,
`--still-every` (default 15), `--still-max` (default 900), `--out`,
`--match-radius`, `--stats-period`, `--max-seconds`, `--missing-timeout`,
`--no-provisional`, `--ws-port`, `--print-frame-mapping`. The flags are
parsed by the front-end's own callback into `RealtimeParams`, written under
the top-level `"realtime"` key of `params.json`; `pipeline -p` reads that
file and ignores the key. `pipeline` itself is unchanged.

Live pose detection uses the HyperPose settings of the params file
(`hp_params.model_name`, `hp_params.precision` `fp32` or `fp16`); the
TensorRT engine cache from the base branch makes later starts take ~2 s.

Outputs: the NDJSON stream; `pipeline-output.json` and
`pipeline-output-presmooth.json` as `pipeline` writes them;
`params.json`; `realtime-stats.json` (throughput, latencies, drops, per-node
times). Exit status is non-zero if the tracker failed or the tracks could
not be written.

## 6. Running and testing

Build and gate (the dev loop serialises builds and GPU runs with locks):

```
build/dev.sh build            # or: build/dev.sh build --hyperpose
build/dev.sh test             # all Catch cases; realtime tags below
build/dev.sh check            # build + test + smoke + rt-smoke
python3 bin/test-realtime-client.py
python3 bin/test-pose-replay-remap.py
```

Unit-test tags: `[realtime_source]`, `[realtime_queue]`,
`[realtime_slots]`, `[realtime_reorder]`, `[realtime_provisional]`,
`[realtime_tracker]`, `[realtime_publish]`, `[realtime_metrics]`,
`[realtime_ws]`, `[localization_empty]`, `[kz_filter]`.

`build/dev.sh smoke` runs batch `pipeline` on 120 frames of testcase 007
(6 cameras) with replayed 2D poses and `--no-stereo`, and compares the
ground-truth metrics with the stored reference. `build/dev.sh rt-smoke` runs
`realtime` on the same configuration, unpaced and without drops, and
requires:

- `bin/compare-tracks.py --exact`: `pipeline-output.json` equal to the stored
  batch output in every track id, every `(x, y, t)` and every `fp_info`
  field except `lab_p`;
- `--stream`: every final smoothed point in the stream equals the output,
  each frame appears once, and the ids missing from the output are exactly
  those post-processing removed;
- `--revised`: the revised positions equal the unsmoothed output
  (Jaccard 1.0);
- the same ground-truth metrics as `smoke`.

A paced run with drops, on the smoke configuration (`smoke-params.json` is
written by `build/dev.sh smoke` into its output directory):

```
CLI=multiview/multiview_cpp/build/cmake-release/multiview-cli
TC=~/multiview-data/testcases/007_site03_sparse_tracks
REF=~/multiview-data/pose-replay/007_site03_sparse_tracks/reference-120f-output.json
$CLI realtime -m $TC/manifest.json -p smoke-params.json -d /tmp/rt-D -y \
    --no-stereo --start 0 --n-frames 120 --pose-classifier pose-svm-lite_v0 \
    --tracks-file $REF --pipeline-testcase \
    --replay-rate 1.75 --drop newest --slots 2 --out /tmp/rt-D/stream.ndjson
bin/realtime-client.py /tmp/rt-D/stream.ndjson
```

Scoring with `--tracks-file` set to the batch output measures agreement with
batch rather than with the ground truth.

## 7. Measurements

RTX 3090, Ryzen 5 3600 (12 threads), 62 GB, `--no-stereo`,
`--pose-classifier pose-svm-lite_v0`, 2026-10-13. Another worktree shared
the machine for some runs (about 10 % noise). T is the processed rate of an
unpaced run without drops. Latencies are from a frame's arrival.

**007, 6 cameras, poses replayed (no pose network), 120 frames, 2 slots.**

| Run | Rate | Processed / dropped | Provisional p50 / p95 | Window p50 / p95 | Against batch |
|---|---|---|---|---|---|
| A: unpaced, `--drop none` (rt-smoke) | T = 1.40 fps | 120 / 0 | queued (not a live latency) | - | exact: IDENTICAL |
| B: paced at 0.8 x T | 1.12 fps | 120 / 0 | 756 / 1833 ms | 458 / 884 ms | exact: IDENTICAL; F1 1.0 |
| D: paced above T | 1.75 fps | 95 / 25 (`no_free_slot`) | 1539 / 2730 ms | 494 / 1430 ms | 179 point differences; TP 177 / FP 0 / FN 28, F1 0.863; same 2 kept tracks |
| C: paced at the live rate | 15 fps | 12 / 108 (`no_free_slot`) | 1759 / 3140 ms | 43 / 1031 ms | no kept tracks (F1 0.0) |

In B the later revisions were: tracked p50/p95 5.2 / 9.2 s, revised
13.7 / 18.1 s, final (smoothed) 27.0 / 32.3 s; at 1.12 fps a 10-frame
window takes 9 s to fill, so these are the protocol's frame lags at a low
rate.

**008, 9 cameras, live HyperPose (FP16 unless stated).**

| Slots | T | DAG p50 / p95 | Start-up | RSS |
|---|---|---|---|---|
| 1 | 0.91 fps (FP32 0.80) | 1.03 / 1.76 s | 14-48 s | 6.6-7.0 GB |
| 2 | 1.00 fps | 1.89 / 3.95 s | 48-54 s | 7.9-8.6 GB |
| 4 | 1.04 fps | 3.61 / 6.32 s | 106 s | 11 GB |

Paced at 0.8 x T on 008 (1 slot): 146 processed / 3 dropped, provisional
p50 1.05 s, p95 1.79 s, the same 2 tracks as unpaced. At the live 15 fps:
7-9 processed of 149, no track survives.

Pose inference per 9-image call: ~100 ms alone at FP16 (215 ms FP32), but
462-832 ms inside the pipeline, where it shares the GPU with the per-sensor
LAB conversion (~540 ms) and SLIC (~390 ms) of the frame. Neither half alone
limits the rate.

Tracking quality against the frame rate (007, all 499 frames, replayed
poses remapped to each rate; `realtime` equals `pipeline` at each):

| `--target-fps` | 15 | 7.5 | 5 | 3 | 1.5 |
|---|---|---|---|---|---|
| F1 | 0.614 | 0.599 | 0.608 | 0.435 | 0.300 |

So the prototype runs 9 cameras at about 1 frame per second with about 1 s
of provisional latency when not overloaded. Tracking holds down to 5 fps,
which is about 5.5 times the measured rate. Issue #2's 500 ms provisional
latency target is not met: the latency is one DAG time.

## 8. Parity and exactness

| Check | Result |
|---|---|
| Unpaced, `--drop none`, 2 slots vs batch, 120 frames (rt-smoke) | `exact: IDENTICAL`: 2 tracks, 205 points; ids, `(x, y, t)`, `dist_p`, `duration_p`, `is_true_positive`, `prob_fp_p`, `score` equal; `lab_p` differs on both tracks (excluded) |
| Same, deep JSON compare of `track_results` without `lab_p` | equal (gaze, pose and labels included) |
| Same, `--still-every 1` | IDENTICAL (earlier run) |
| Stream vs output (`--stream`) | OK: 205 final smoothed points equal; 4 ids removed by post-processing = 6 raw - 2 kept |
| Revised stream vs unsmoothed output (`--revised`) | 205 / 205 points equal, Jaccard 1.000000 |
| Paced at 0.8 x T without drops (run B) | IDENTICAL |
| `--target-fps 7.5`, `realtime` vs `pipeline` | identical tracks |
| Batch smoke after all changes | TP 159 / FP 46 / FN 81, F1 0.556, 0 id switches, 5 fragmentations (unchanged) |
| Rectification skip, per-node LAB | batch output identical, `lab_p` included |

`lab_p` differs because the still is sampled every 15th frame instead of
every frame (section 3).

## 9. Not done, stubbed or deferred

- **Cameras.** Input is replayed from files; there is no RTSP or camera
  source, and `--loop` is not implemented.
- **Soak.** RSS over a long run was not measured (it needs `--loop`).
- **Service hardening.** The WebSocket server has no TLS, no
  authentication, no client limit, and binds all interfaces. It was tested
  with local clients only.
- **Stereo.** Without `--no-stereo` the still differs from batch; stereo
  realtime is untested.
- **Engine warm-up.** The TensorRT engine is loaded on the first frame, on
  a pool thread, as in batch. With 2 slots this once pushed frame 0 past
  `--missing-timeout`, and the frame was published as dropped.
- **Slot warm-up** takes 5-50 s per slot and varies from run to run; the
  cause (the `get_xy_mapping` jobs wait rather than compute) is not
  understood.
- **Issue #13.** The tracker thread still enters the existing helping waits
  inside `Tracklet::execute` and the tracks stage, so it can run slot jobs
  while it waits; window times include any it picked up.
- **Full frame-source extraction.** `grab_video_frame_` stays in
  `SceneDescription` and batch still decodes there; the paced source has its
  own decoders. Moving batch onto a shared `FileFrameSource` is deferred.
- **Latency stamping.** `latency_ms` is taken where a message is made, not
  when the publisher writes it; the output queue was empty in every stats
  line measured, but a blocked stdout pipe would not show.
- **Scoring** covers `[start, start + n)`; with `--start > 0` at a frame rate
  other than the ground truth's it is skipped.
- **Not measured:** the pose-level comparison of the 6- and 9-image engine
  profiles; a remote WebSocket client.

## 10. Open questions

1. **A faster detector.** A live service needs about 5 fps on 9 cameras.
   Is issue #10's TensorRT YOLO-pose the path, and does the per-sensor
   LAB + SLIC chain (needed for `lab_p`) get its own GPU budget or a
   cheaper appearance feature?
2. **Smoothing lag.** A point is final 27-34 frames after it arrives
   (about 3.1 s at p95 at 15 fps, with the window computation). A realtime
   KZ profile (m = 5, k = 2, `delta_t` 4) would cut it to under 1.5 s but
   changes the output from batch. Is that acceptable for the live view?
3. **Unconfirmed tracks in the stream.** 27 % of the revised points in the
   parity run belong to ids that post-processing later removes. Should
   clients hide tracks until `confirmed`, or should the stream hold them
   back?
4. **Two levers for the base branch.** Should the rectification skip and
   the per-node LAB converter move to `build-fast-iteration` as their own
   PR, since they speed up batch and the cloud branch as well?
5. **`--start > 0` and the still.** `bcam_init` no longer adds frame 0 to
   the still, which changes stereo batch runs that start later. Keep it, or
   gate it to `realtime`?
