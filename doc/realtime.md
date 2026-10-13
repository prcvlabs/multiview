# Real-time tracking (`multiview-cli realtime`)

`realtime` runs the pipeline on frames as they arrive, instead of on a whole
movie: a paced source replays the scene's videos at a wall-clock rate, each
frame goes through the same per-frame pipeline as `pipeline` (in one of
`--slots` copies of it, side by side), and the results are tracked by the same
tracker, ten frames (one window) at a time. Every result is written as it
becomes known, one JSON object per line (NDJSON). This is the prototype for
[issue #2](https://github.com/prcvlabs/multiview/issues/2): it replays files,
it does not read cameras yet.

Without drops, the output is the batch output: `realtime --replay-rate 0
--drop none` writes the same `pipeline-output.json` as `pipeline`, point for
point, except `lab_p` (the tracks' appearance score), which depends on the
still that `--still-every` samples (`build/dev.sh rt-smoke` checks it on
every `check`, with `bin/compare-tracks.py --exact`, which leaves `lab_p`
out).

**Contents**

1. [How to run it](#1-how-to-run-it)
2. [What it writes](#2-what-it-writes)
3. [How it works](#3-how-it-works)
4. [Measurements](#4-measurements)
5. [Decisions](#5-decisions)
6. [Not done (stubs)](#6-not-done-stubs)

## 1. How to run it

`realtime` takes every `pipeline` flag (`-m`, `-p`, `-d`, `-y`, `--no-stereo`,
`--start`, `--n-frames`, `--target-fps`, `--pose-classifier`, `--tracks-file`,
`--pipeline-testcase`, ...) and these:

| Flag | Default | Meaning |
|---|---|---|
| `--replay-rate <fps>` | scene fps | Wall-clock pacing of the replay. `0` is unpaced: each frame is read when a slot can take it, and nothing is dropped. |
| `--drop <auto\|newest\|none>` | `auto` | When every slot is busy: `newest` drops the frames that waited and takes the newest one; `none` waits. `auto` is `newest` when paced and `none` when unpaced. |
| `--slots <n>` | 1 | Frames in flight (copies of the per-frame pipeline), 1 to 6. |
| `--ring <n>` | slots + 4 | Decoded frames held by the source. |
| `--still-every <n>` | 15 | Add every n-th frame to the running still (the batch adds every frame). |
| `--still-max <n>` | 900 | Stop adding to the still after n frames (0: never). |
| `--out <file\|->` | `-` | NDJSON output; `-` is stdout (all logging goes to stderr). |
| `--match-radius <m>` | 0.6 | Radius for provisional ids, metres. |
| `--stats-period <s>` | 1 | Seconds between `stats` lines. |
| `--max-seconds <s>` | 0 | Stop after this long (0: at the end of the frames). |
| `--missing-timeout <s>` | 5 | Give up waiting for a frame's result (drop newest only). |
| `--no-provisional` | | Do not write `frame` lines. |
| `--ws-port <port>` | 0 | Serve the stream over WebSocket, and the health and metrics endpoints, on this port (0: no server). See below. |
| `--print-frame-mapping` | | Print `k movie-frame` for every scene frame `k`, and exit. |

A run on testcase 007 with the 2D poses replayed from disk (no pose network;
`build/dev.sh smoke` writes such a `smoke-params.json`):

```
CLI=multiview/multiview_cpp/build/cmake-release/multiview-cli
TC=~/multiview-data/testcases/007_site03_sparse_tracks
$CLI realtime -m $TC/manifest.json -p smoke-params.json -d /tmp/rt -y \
    --no-stereo --pose-classifier pose-svm-lite_v0 --n-frames 120 \
    --out /tmp/rt/stream.ndjson
```

With live pose detection, build with HyperPose (`build/dev.sh build
--hyperpose`) and set `task_params.frame_params.pose_skeleton_init.op_params`
in the params file: `use_hyperpose: true`, and in `hp_params` the
`model_name` (`openpose-coco-V2-HW=368x656.onnx` by default, or
`openpose-thin-V2-HW=368x432.onnx`) and `precision` (`fp32` or `fp16`). The
first run for a model, precision and camera count builds a TensorRT engine
(50-80 s for 9 cameras) and caches it in `~/.cache/multiview-data/cache/trt`;
later runs load it in about 2 s.

SIGINT stops the run cleanly (the frames in flight are finished, the tracks
written, `end` published); a second SIGINT exits at once. `--max-seconds`
stops it the same way. At the end the output directory holds
`pipeline-output.json` (as `pipeline` writes it), `pipeline-output-presmooth.json`
(the same before smoothing), `params.json` (with the realtime flags under
`"realtime"`; `pipeline -p` accepts it), `realtime-stats.json` (throughput,
latency percentiles, drops, per-node mean times) and, with `--timings`, one
Gantt chart per slot.

### Serving the stream (`--ws-port`)

With `--ws-port 8765` one listener on all interfaces (no TLS, no
authentication) serves, from scene load to the end of the run:

| Endpoint | Answer |
|---|---|
| `ws://HOST:8765/v1/stream` | Every NDJSON line, one WebSocket text message each, without the newline. A client that connects late gets the `header` line first, then the stream from where it joined. |
| `GET /v1/health` | The `stats` object (below) plus `"status"`: `"ok"`, or `"overloaded"` when more than 20 % of the frames of the last 5 s were dropped, and `"drop_share"`. `{"status": "starting"}` until the slots are warmed up. |
| `GET /v1/metrics` | The same counters in Prometheus text format (`multiview_realtime_*`: frames, fps, queue depths, drops by reason, latency summaries, `overloaded`). |

Each client has its own queue of 256 lines; when a client reads too slowly,
its oldest unsent line is dropped (counted as `ws_dropped` in
`realtime-stats.json`), so a client never slows the run. The file or stdout
sink never drops. At the end the server sends each client its last lines
and closes it (waiting 2 s at most).

`bin/realtime-client.py` reads a stream from a file, stdin or the WebSocket
URL (standard library only), and summarises it: lines per type, frames
processed and dropped, provisional latency over the run and per half, the
last `stats` line's latencies, and the `end` line:

```
bin/realtime-client.py ws://localhost:8765/v1/stream --save /tmp/ws.ndjson
bin/realtime-client.py /tmp/rt/stream.ndjson
curl -s localhost:8765/v1/health
```

### Replaying poses at a lower `--target-fps`

Pose replay files are named by scene frame. At a `--target-fps` below the
videos' rate, scene frame `k` shows movie frame `m(k)`, so a replay set
recorded at the videos' rate has to be renamed first:

```
$CLI realtime -m $TC/manifest.json -d /tmp/map -y --no-stereo --target-fps 7.5 \
    --print-frame-mapping > map-7.5.txt
bin/pose-replay-remap.py map-7.5.txt ~/multiview-data/pose-replay/007_site03_sparse_tracks /tmp/replay-7.5
```

and `replay_dir` set to `/tmp/replay-7.5`. The mapping is the one the batch
uses (`scene_frame_to_movie_frame_mapping`); it is not a fixed stride (at
7.5 fps on 15 fps videos the steps are 1, 2 or 3 movie frames).
`bin/test-pose-replay-remap.py` tests the script.

## 2. What it writes

One JSON object per line, in this order: `header` first, `end` last, `stats`
every `--stats-period`. Positions are metres, `frame` and `t` are absolute
scene frame numbers, `latency_ms` is from the frame's arrival (decoded) to
the line being written.

```
{"type":"header","scene":"site03_scene03","n_cameras":6,"fps":15.0,"hist_sz":0.1,"top_left":[-6.0,-3.0],"w":90,"h":130,"start_frame":0,"K":10,"delta_t":16,"version":"..."}
{"type":"frame","frame":0,"ts":"1970-01-01T00:00:00.000000","latency_ms":1665.2,"n_p2ds":2,"positions":[{"x":-0.4,"y":1.1,"n_detections":2,"score":0.45,"prob_fp":0.30,"pid":-1}]}
{"type":"dropped","frame":61,"reason":"no_free_slot"}
{"type":"tracks","window":0,"frame":0,"revision":0,"status":"tracked","id_remap":[],"tracks":[{"id":0,"x":-0.4,"y":1.1,"gaze":-0.58,"interp":false,"confirmed":false,"score":0.88}]}
{"type":"tracks","window":0,"frame":0,"revision":1,"status":"revised","id_remap":[[7,5]],"tracks":[...]}
{"type":"smoothed","id":0,"final_upto_t":21,"points":[[x,y,t,gaze],...]}
{"type":"track_end","id":5,"last_t":7,"confirmed":false}
{"type":"window_error","window":9,"reason":"..."}
{"type":"stats","uptime_s":12.0,"fps":{...},"frames":{...},"drops":{...},"latency_ms":{"provisional":{"p50":...,"p95":...,"max":...,"n":...},"tracked":{...},"revised":{...},"final":{...}},"queue_depth":{...},"slots_busy":2,"still_samples":8,"rss_mb":4100,"window_ms":{...}}
{"type":"end","frames":120,"dropped":2,"output":"/tmp/rt/pipeline-output.json"}
```

- `frame` (provisional): the frame's detections clustered and positioned with
  the tracker's own estimator, before any window closes; `pid` is the id of
  the nearest live track within `--match-radius`, or -1.
- `dropped`: a frame that was not processed. Reasons: `no_free_slot` (the
  dispatcher took a newer frame), `tracker_behind` (no room downstream),
  `source_behind` (the decoders were late), `decode_error`, `dag_error` (the
  per-frame pipeline returned no result), `late` (the result did not come
  back within `--missing-timeout`). The
  tracker sees a dropped frame as a frame without evidence.
- `tracks`, revision 0 (`tracked`): the frame's tracks when its window
  closes. Revision 1 (`revised`): after the next window has rewritten it;
  ids and raw positions are then final, and `id_remap` lists the ids that
  changed. `confirmed` is the batch's keep/drop rule applied to the track so
  far; only the end of the stream decides it.
- `smoothed`: a track's smoothed points; those with `t <= final_upto_t` are
  final (`final_upto_t` is the last revised frame minus `delta_t`, the
  smoothing filter's half width).
- `track_end`: a track that did not continue into the newest window.
- `window_error`: a window that failed validation and was skipped (none in
  any run so far).

`bin/compare-tracks.py` checks a stream against the run's output
(`--stream`), the revised positions against the unsmoothed output
(`--revised`), and two outputs against each other exactly (`--exact`).

## 3. How it works

```
paced source --> dispatcher --> N slots (FrameResults) --> reorder --> tracker
 (a decoder       (drop newest    per-frame pipeline        (frame       (one window
 thread per       or wait)        on the thread pool)       order)       of K frames
 camera)                                                                 at a time)
                                                               |            |
                                                               v            v
                                                  publisher (one thread) --> file / stdout
                                                                         --> WebSocket server
                                                                             (one thread)
```

- The source decodes each camera on its own thread, sequentially, and puts
  whole scene frames into a ring at the replay rate.
- The dispatcher hands a frame to an idle slot. A slot is a copy of the
  per-frame pipeline (`FrameResults`), warmed up once at start-up. Slots are
  never cancelled; dropping means not dispatching.
- The reorder stage writes frames in order as their results arrive, and
  passes them to the online tracker, which runs the batch tracker's window
  code (`Tracklet::execute` and the tracks stage) on its own thread when a
  window of K = 10 frames is complete. Window k rewrites window k-1 in place,
  which is why `tracked` and `revised` are separate revisions.
- At the end, the tracker concatenates the windows, post-processes and
  smooths them exactly as `pipeline` does, and writes `pipeline-output.json`.
- The running still is sampled every `--still-every` frames inside the
  slots' `copy_sensor_images`, with the batch code. In `--no-stereo` nothing
  but `lab_p` (the tracks' appearance score) depends on the still.
- The reorder stage and the tracker hand their messages to the publisher,
  which serialises each once, writes it to the file (blocking: nothing is
  lost) and hands it to the WebSocket server, and adds a `stats` line every
  `--stats-period`. The server runs Boost.Beast on its own thread; every
  client keeps a read pending, so pings and the close are answered.
- At the end the stages stop in data-flow order (source, dispatcher, slots,
  reorder, tracker, publisher, server), so every frame a slot took is
  published before `end`.
- New threads use `std::mutex` and `std::condition_variable` only: no new
  helping waits (issue #13). The tracker thread still enters the existing
  ones inside `Tracklet::execute` and the tracks stage.

## 4. Measurements

Measured on 2026-10-13 on an RTX 3090 with a Ryzen 5 3600 (12 threads),
`--no-stereo`, `--pose-classifier pose-svm-lite_v0`, while another worktree
shared the machine (expect about 10 % noise). Testcase 008 has 9 cameras and
149 frames; 007 has 6 cameras. "T" is the processed rate of an unpaced run
without drops (`--replay-rate 0 --drop none`); latencies are from a frame's
arrival.

**Pose detection, 9 images per call (HyperPose `run engine`, ms, 1 slot).**
The first four rows ran with the shared LAB converter, the last with the
per-node converter that this branch ships.

| Model, precision | Alone (first call) | In the pipeline, p50 | p90 | TensorRT build (once) | Tracks against FP32 |
|---|---|---|---|---|---|
| openpose-coco 368x656, FP32, shared LAB converter | 215 | 806 | 844 | 50 s | - |
| openpose-coco 368x656, FP16, shared LAB converter | 102 | 462 | 552 | 79 s | same 2 tracks, within 2 cm |
| openpose-thin 368x432, FP32, shared LAB converter | 259 | 440 | 538 | 49 s | one person found 68 frames later, 0.6 m off |
| openpose-thin 368x432, FP16, shared LAB converter | 126 | 435 | 605 | 70 s | as thin FP32 |
| openpose-coco 368x656, FP16, per-node LAB converter | 100 | 832 | 857 | - | as coco FP16 (output identical) |

In the pipeline the network shares the GPU with the per-sensor LAB conversion
and SLIC of the frames in flight, which makes it 4-8 times slower than alone.
The per-node converter lets the nine LAB conversions run on the GPU at once,
which doubles the in-pipeline pose time (FP16 p50 462 -> 832 ms; 942 ms at
2 slots) while T still rises (next table).

**Throughput on 008 (FP16 unless stated).**

| Slots | T, shared LAB converter | T, per-node LAB converter | DAG p50 / p95 (per-node) | Start-up | RSS |
|---|---|---|---|---|---|
| 1 | 0.82 fps (FP32: 0.80) | **0.91 fps** | 1.03 / 1.76 s | 14-48 s | 6.6-7.0 GB |
| 2 | 0.98 fps (FP32: 0.94) | **1.00 fps** | 1.89 / 3.95 s | 48-54 s | 7.9-8.6 GB |
| 4 | 1.06 fps | **1.04 fps** | 3.61 / 6.32 s | 106 s | 11 GB |

Start-up is mostly slot warm-up, which varies from 5 to 50 s from one run to
the next with one or two slots (section 6), so their start-up ranges overlap.

With one slot, each of the 9 sensors spends ~540 ms in `convert_to_lab` and
~390 ms in `run_f2d` (SLIC), side by side, then `localization` takes ~70 ms;
pose (0.5-0.9 s) runs in parallel with them. The detector is not what
limits the rate: with the shared converter FP16 halves the pose time and
raises T by 2-4 %. On 007 (6 cameras, poses replayed, 2 slots) T is 1.56 fps.

**Levers.**

| Lever | Effect | Status |
|---|---|---|
| FP16 (`hp_params.precision`) | pose 806 -> 462 ms in the pipeline (shared LAB converter), 215 -> 102 ms alone; T +2-4 %; tracks within 2 cm | recommended; the default stays `fp32` (shared with `pipeline`) |
| `openpose-thin` 368x432 | no faster than coco FP16 in the pipeline; worse tracks | rejected |
| `--slots` 1 / 2 / 4 | T 0.91 / 1.00 / 1.04, DAG latency 1.0 / 1.9 / 3.6 s | default 1 (section 5) |
| `--still-every` 1 vs 15 | T 0.53 vs 0.82; `copy_sensor_images` 745 vs 32 ms per frame | default 15 |
| No rectification in `--no-stereo` | ~22 ms of CPU per sensor per frame; output identical (`lab_p` included) | done, for `pipeline` too |
| Per-node LAB converter | T +11 % at 1 slot, +2 % at 2; pose 462 -> 832 ms in the pipeline (FP16); output identical | done, for `pipeline` too |
| Skip SLIC in `--no-stereo` | ~0.23 s per frame on 007 | not done: SLIC feeds the appearance patches (`lab_p` 0.935 -> 0.022 in a test) |

The two levers marked "for `pipeline` too" change the batch pipeline (and so
the cloud branch) as well, and belong on `build-fast-iteration`: `79bda2d`
(no rectification in `--no-stereo`) and `a1b58d1` (per-node LAB converter).
The rectification skip is unconditional in `--no-stereo`, with no flag: the
output does not change, `lab_p` included.

**Paced runs on 008 (FP16, per-node LAB, drop newest).**

| Rate | Slots | Processed / dropped | Provisional latency p50 / p95 / max | Kept tracks |
|---|---|---|---|---|
| 15 fps (live) [1] | 1 | 7 / 142 | 1.41 / 3.14 / 3.14 s | 0 |
| 15 fps (live) [1] | 2 | 7 / 142 | 3.21 / 5.27 / 5.27 s | 0 |
| 15 fps (live), shared LAB | 1 | 8 / 141 | 1.40 / 2.81 / 2.81 s | 0 |
| 15 fps (live), shared LAB | 2 | 9 / 140 | 2.45 / 5.09 / 5.09 s | 0 |
| 3 fps (`--target-fps 3`, shared LAB) | 1 | 8 / 23 | 1.48 / 2.53 / 2.53 s | 0 |
| 0.73 fps (0.8 T, slow motion) | 1 | 146 / 3 | 1.05 / 1.79 / 3.21 s | 2 |
| 0.8 fps (0.8 T, slow motion) | 2 | 148 / 1 | 1.06 / 2.69 / 4.16 s | 2 |

[1] These two runs had 6 and 14 `source_behind` drops (the rest are
`no_free_slot`): another worktree's processes ran on the machine outside the
GPU lock and slowed the decoders. The shared-LAB runs at 15 fps, which had
none, are shown below them.

At the live rate one frame in twenty is processed, every tracking window is
mostly gaps, and no track survives. Paced below what the DAG sustains,
almost nothing is dropped, and the provisional latency is one DAG time
(about 1 s at p50). A position becomes final 27 (p50) to 34 (p95) frames
after it arrived, i.e. 1.8-2.3 s at 15 fps plus one window computation
(p50 0.43 s, p95 0.81 s): about 3.1 s at p95. These lags were measured at
0.66 fps (a slow-motion replay) and converted to 15 fps; the windows would
be computed at 15 fps under more load.

The paced runs leave issue #2's 500 ms provisional-latency target unmet: the
latency is one DAG time, p50 1.05 s and p95 1.8-2.7 s.

**Tracking at a reduced rate** (007, the whole 33 s, poses replayed and
remapped, equal to batch):

| `--target-fps` | 15 | 7.5 | 5 | 3 | 1.5 |
|---|---|---|---|---|---|
| F1 | 0.614 | 0.599 | 0.608 | 0.435 | 0.300 |
| ID switches | 0 | 0 | 0 | 0 | 0 |

`realtime` and `pipeline` at `--target-fps 7.5` give identical tracks.

## 5. Decisions

The rate decisions follow the table of the plan, keyed on the 9-camera
measurements:

| Rule | Measured | Outcome |
|---|---|---|
| `--slots`: the N with the best T(N)/N among those with T(N) >= 0.8 x T(4) | T = 0.91 / 1.00 / 1.04; 0.8 x T(4) = 0.83 | **1 slot** (also the lowest latency and memory) |
| `--target-fps`: the largest of 15, 7.5, 5, 3 that is <= 0.8 x T | 0.8 x 0.91 = 0.73 fps | **none**: the default stays the scene rate, and frames the DAG cannot take are dropped |
| Pose p50 per 9-image call <= 66 ms (15 fps), 133 ms (7.5), 200 ms (5) | 832 ms in the pipeline (per-node LAB converter; 462 ms with the shared one), ~100 ms alone (FP16) | not met; 15 fps needs a faster detector (issue #10's TensorRT YOLO-pose) |
| Per-frame chain (LAB + SLIC + localization) <= 400 ms | ~0.8-1.0 s | not met; the chain limits the rate as much as the detector |
| Smoothing lag p95 <= 3 s at 15 fps | 34 frames = 2.27 s + window p95 0.81 s = about 3.1 s (frames counted at 0.66 fps) | at the threshold, slightly over; the shorter KZ filter is the option (below) |
| Provisional latency <= 500 ms | p50 1.05 s, p95 1.8-2.7 s (paced at 0.8 x T) | not met (one DAG time) |

So on this machine the prototype runs 9 cameras at about 1 frame per second,
with about 1 s of provisional latency when it is not overloaded. Tracking
quality holds down to 5 fps (F1 0.61), so a live service needs a sustained
5 fps or better, about 5.5 times the measured rate. Both halves of the frame
have to get there: the pose network about 4.2 times faster in the pipeline
(832 -> 200 ms; at FP16 it is fast enough alone, so most of that is the GPU
it shares), and the per-sensor LAB and SLIC chain about 4 times at one slot.
The
architecture does not depend on the rate: with a faster DAG the same flags
apply, and `--target-fps 5` (which also paces the replay at 5 fps) is the
configuration to try first.

The smoothing lag is at the 3 s limit. The option, if it has to be shorter,
is a realtime `fowlkes_params` profile with `smooth_kz_filter_m: 5` and
`smooth_kz_filter_k: 2`: a 9-frame window (`delta_t` 4 instead of 16), so a
point is final 14-23 frames after it arrived instead of 26-35 (computed, not
measured: at most 1.5 s plus a window at 15 fps). It smooths less, and its
output differs from batch, so it is not the default.

When frames are dropped, the tracker sees them as frames without evidence;
tracks still form if most frames of a window are processed (a run paced at
0.8 x T dropped 1-3 of 149 frames, kept the same 2 tracks, and differed from
the drop-free output in 1-35 points).

## 6. Not done (stubs)

- **Cameras**: input is replayed from the scene's files (`PacedFrameSource`);
  there is no RTSP or camera source, and `--loop` is not implemented.
- **Service hardening**: the WebSocket server has no TLS, no
  authentication and no client limit, and listens on all interfaces; it was
  checked with one or two local clients only.
- **Stereo**: without `--no-stereo` the still, which stereo depends on, is
  sampled differently from batch; stereo realtime is untested.
- **Engine warm-up**: the TensorRT engine is loaded (or built) on the first
  frame, on a pool thread, as in batch. From the cache this adds 0.5-2.3 s to
  the first frame; with 2 slots it once pushed frame 0 past
  `--missing-timeout`, and that frame is then published as dropped with
  reason `late` (`dag_error` in the runs of section 4, which predate the
  `late` reason).
- **Slot warm-up** takes 5-50 s per slot and is not understood (the
  `get_xy_mapping` jobs wait rather than compute).
- **Issue #13**: the tracker thread still enters the existing helping waits
  inside `Tracklet::execute` and the tracks stage, so it can run a slot's
  pool jobs while it waits.
- **Confirmed tracks** are decided on the sequence so far; only the end of
  the stream applies `post_process`.
- **Replayed poses** must carry the scene frame in `frame-no`; a mismatch
  fails a precondition on a pool thread (as in `pipeline`).
- **Not measured**: the soak (RSS over a long run, which needs `--loop`), the
  pose-level comparison of the 6- and 9-image engine profiles, and a remote
  WebSocket client (the local ones received every line, byte for byte as the
  file).
