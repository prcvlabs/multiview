// The lease protocol of db.ts on the real schema (node:sqlite): claims,
// heartbeats and their wall-clock budget, complete with output checks,
// failures and retries, expiry, cancel, the tracks job's promotion and
// lease, and correlated failures. The same cases as test_fsbackend.py.

import { fileURLToPath } from "node:url";
import { beforeEach, describe, expect, it } from "vitest";

import {
  buildLease,
  cancelJob,
  claimShard,
  completeShard,
  correlatedFailure,
  failShard,
  getShard,
  heartbeatShard,
  insertJob,
  jobStatus,
  leasePrefix,
  pendingMessages,
  planJob,
  promoteTracks,
  pullShard,
  sweepJob,
  tally,
} from "../src/db";
import { normalizeSpec } from "../src/spec";
import type { DoneRecord, JobSpec, ShardRow } from "../src/types";
import { SqliteD1 } from "./d1-sqlite";

const k_migration = fileURLToPath(new URL("../migrations/0001_init.sql", import.meta.url));
const k_worker = { worker_id: "host-gpu0", gpu: "RTX 3090", version: "test" };
const k_origin = "http://127.0.0.1:8787";

function spec(overrides: Record<string, unknown> = {}): JobSpec {
  const s = normalizeSpec({
    scene_id: "007_site03_sparse_tracks",
    manifest_key: "scenes/007/m/manifest.json",
    params_key: "scenes/007/p/params.json",
    poses_key: "scenes/007/t/poses.tar",
    gt_key: "scenes/007/g/ground-truth.json",
    videos: [{ name: "C1.mp4", key: "scenes/007/v/C1.mp4" }],
    total_frames: 499,
    K: 10,
    n_cameras: 6,
    n_frames: 120,
    shard_frames: 60,
    lease_seconds: 30,
    heartbeat_s: 6,
    retry_delay_s: 10,
    ...overrides,
  });
  const input = (name: string, key: string, kind: "file" | "tar" = "file") =>
    ({ name, key, kind, bytes: 10, md5: "0".repeat(32), cache: "scene" }) as const;
  return {
    ...s,
    inputs: [
      input("manifest.json", s.manifest_key),
      input("C1.mp4", "scenes/007/v/C1.mp4"),
      input("params-in.json", s.params_key),
      input("poses.tar", s.poses_key!, "tar"),
      input("ground-truth.json", s.gt_key!),
    ],
  };
}

// The outputs of a finished attempt, as the agent lists them in done.json,
// and a bucket that has them (sizes by key).
function finished(row: ShardRow, paramsMd5 = "p".repeat(32)) {
  const prefix = `jobs/${row.job_id}/shards/${row.id}/a${row.attempt}`;
  const names =
    row.kind === "loc"
      ? ["localization.data", "params.json", "stdout.log"]
      : ["pipeline-output.json", "params.json", "stdout.log", "score.json"];
  const done: DoneRecord = {
    outputs: names.map((name) => ({ name, bytes: 100, md5: "m".repeat(32) })),
    params_md5: paramsMd5,
    wall_s: 12.5,
  };
  const sizes = new Map<string, number>([...names, "done.json"].map((n) => [`${prefix}/${n}`, 100]));
  return { done, headSize: async (key: string) => sizes.get(key) ?? null };
}

describe("the lease protocol", () => {
  let d1: SqliteD1;
  let db: D1Database;
  const jobId = "job-1";
  let now = 1000;

  beforeEach(async () => {
    d1 = new SqliteD1(k_migration);
    db = d1.asD1();
    now = 1000;
    await insertJob(db, jobId, spec(), now);
    expect(await planJob(db, jobId, now)).toBe(2);
  });

  async function complete(row: ShardRow, paramsMd5?: string) {
    const { done, headSize } = finished(row, paramsMd5);
    return completeShard(db, jobId, row.id, row.lease_token!, done, headSize, now);
  }

  it("plans once: loc shards pending, the tracks job blocked", async () => {
    expect(await planJob(db, jobId, now)).toBe(2); // the Workflow step may run twice
    const status = (await jobStatus(db, jobId))!;
    expect(status.state).toBe("running");
    const shards = status.shards as Record<string, unknown>[];
    expect(shards.map((s) => [s.id, s.state, s.start_frame, s.n_frames, s.max_wall_s])).toEqual([
      ["loc-0000", "pending", 0, 60, 780],
      ["loc-0001", "pending", 60, 60, 780],
      ["tracks", "blocked", 0, 120, 540],
    ]);
    expect(await pendingMessages(db, jobId, "loc")).toEqual([
      { v: 1, job_id: jobId, shard_id: "loc-0000", attempt: 0 },
      { v: 1, job_id: jobId, shard_id: "loc-0001", attempt: 0 },
    ]);
  });

  it("leases each loc shard once, never the blocked tracks job", async () => {
    const a = await pullShard(db, k_worker, null, now);
    const b = await pullShard(db, { ...k_worker, worker_id: "other" }, null, now);
    expect([a?.id, b?.id]).toEqual(["loc-0000", "loc-0001"]);
    expect(a?.attempt).toBe(1);
    expect(a?.lease_expires_at).toBe(now + 30);
    expect(await pullShard(db, k_worker, null, now)).toBeNull();
    // A stale queue message (attempt 0) cannot claim the leased shard
    expect(await claimShard(db, jobId, "loc-0000", 0, k_worker, now)).toBeNull();
    expect(await leasePrefix(db, a!.lease_token!)).toBe(`jobs/${jobId}/shards/loc-0000/a1`);
  });

  it("builds loc and tracks leases as the agent expects them", async () => {
    const a = (await pullShard(db, k_worker, null, now))!;
    const lease = await buildLease(db, a, spec(), k_origin, 8 << 20);
    expect(lease.cli).toMatchObject({ start_frame: 0, n_frames: 60, no_still: true, require_aligned: false });
    expect(lease.cli.pose_classifier).toBeNull();
    expect(lease.inputs.map((i) => i.name)).toEqual(["manifest.json", "C1.mp4", "params-in.json", "poses.tar"]);
    expect(lease.inputs[0].url).toBe(`${k_origin}/v1/blobs/scenes/007/m/manifest.json`);
    const env = lease.outputs.find((o) => o.name === "localization.data")!;
    expect(env.required).toBe(true);
    expect(env.multipart).toEqual({
      init_url: `${env.url}?uploads`,
      part_url: env.url,
      complete_url: env.url,
      abort_url: env.url,
      part_bytes: 8 << 20,
    });
    expect(lease.outputs.find((o) => o.name === "params.json")!.multipart).toBeNull();

    const b = (await pullShard(db, k_worker, null, now))!;
    expect((await complete(a)).status).toBe(200);
    expect(await promoteTracks(db, jobId, now)).toEqual([]); // loc-0001 is not done yet
    expect((await complete(b)).status).toBe(200);
    expect(await promoteTracks(db, jobId, now)).toEqual([{ v: 1, job_id: jobId, shard_id: "tracks", attempt: 0 }]);
    expect((await jobStatus(db, jobId))!.state).toBe("merging");

    const t = (await pullShard(db, k_worker, null, now))!;
    const tl = await buildLease(db, t, spec(), k_origin, 8 << 20);
    expect(tl.cli).toMatchObject({ no_still: false, require_aligned: true, pose_classifier: "pose-svm-lite_v0" });
    expect(tl.keep).toEqual([
      { name: "loc-0000.data", start: 0, n: 60, computed_start: 0, computed_n: 60 },
      { name: "loc-0001.data", start: 60, n: 60, computed_start: 60, computed_n: 60 },
    ]);
    expect(tl.inputs.map((i) => i.name)).toEqual([
      "manifest.json", "C1.mp4", "params-in.json", "ground-truth.json", "loc-0000.data", "loc-0001.data",
    ]);
    expect(tl.inputs[4].url).toBe(`${k_origin}/v1/blobs/jobs/${jobId}/shards/loc-0000/a1/localization.data`);
    expect(tl.params_md5).toBe("p".repeat(32));
    expect(tl.outputs.find((o) => o.name === "score.json")!.required).toBe(true);
  });

  it("extends a lease on heartbeat and enforces 1.5 x max_wall_s from the run's start", async () => {
    const a = (await pullShard(db, k_worker, null, now))!;
    now = 1010;
    expect(await heartbeatShard(db, jobId, a.id, a.lease_token!, {}, now)).toEqual({ ok: true, lease_expires_at: 1040 });
    expect(await heartbeatShard(db, jobId, a.id, "stale", {}, now)).toMatchObject({ ok: false, reason: "the lease was superseded" });
    now = 2000; // waiting for the GPU lock does not count
    expect((await heartbeatShard(db, jobId, a.id, a.lease_token!, {}, now)).ok).toBe(true);
    expect((await heartbeatShard(db, jobId, a.id, a.lease_token!, { run_started_at: 1 }, now)).ok).toBe(true);
    expect((await getShard(db, jobId, a.id))!.run_started_at).toBe(2000);
    now = 2000 + 1.5 * 780;
    expect((await heartbeatShard(db, jobId, a.id, a.lease_token!, {}, now)).ok).toBe(true);
    now += 1;
    expect(await heartbeatShard(db, jobId, a.id, a.lease_token!, {}, now)).toMatchObject({
      ok: false,
      state: "leased",
      reason: "wall-clock budget exceeded",
    });
  });

  it("completes only with the outputs in place, idempotently, and wakes the Workflow when a kind is done", async () => {
    const a = (await pullShard(db, k_worker, null, now))!;
    const b = (await pullShard(db, k_worker, null, now))!;
    const { done, headSize } = finished(a);
    const missing = async (key: string) => (key.endsWith("/done.json") ? null : headSize(key));
    expect((await completeShard(db, jobId, a.id, a.lease_token!, done, missing, now)).status).toBe(422);
    const wrongSize = { ...done, outputs: done.outputs.map((o) => ({ ...o, bytes: 99 })) };
    expect((await completeShard(db, jobId, a.id, a.lease_token!, wrongSize, headSize, now)).status).toBe(422);
    const noEnvelope = { ...done, outputs: done.outputs.filter((o) => o.name !== "localization.data") };
    expect((await completeShard(db, jobId, a.id, a.lease_token!, noEnvelope, headSize, now)).status).toBe(422);
    expect((await completeShard(db, jobId, a.id, "stale", done, headSize, now)).status).toBe(409);

    const ok = await complete(a);
    expect(ok).toMatchObject({ status: 200, wake: false });
    expect(await complete(a)).toMatchObject({ status: 200, body: { noop: true } });
    expect(await complete(b)).toMatchObject({ status: 200, wake: true });
    expect((await getShard(db, jobId, a.id))!.output_prefix).toBe(`jobs/${jobId}/shards/loc-0000/a1`);
    const status = (await jobStatus(db, jobId))!;
    const attempts = (status.shards as { attempts: { outcome: string; wall_s: number }[] }[])[0].attempts;
    expect(attempts).toMatchObject([{ attempt: 1, outcome: "done", wall_s: 12.5 }]);
  });

  it("kills a loc shard whose params_md5 differs from the others'", async () => {
    const a = (await pullShard(db, k_worker, null, now))!;
    const b = (await pullShard(db, k_worker, null, now))!;
    await complete(a);
    expect(await complete(b, "q".repeat(32))).toMatchObject({ status: 200, body: { state: "dead" }, wake: true });
    expect((await tally(db, jobId, "loc")).dead_error).toContain("params_md5");
  });

  it("retries a retryable failure after its delay, then gives up", async () => {
    for (let attempt = 1; attempt <= 3; ++attempt) {
      const a = (await pullShard(db, k_worker, null, now))!;
      expect([a.id, a.attempt]).toEqual(["loc-0000", attempt]);
      await pullShard(db, k_worker, null, now).then((b) => b && failShard(db, jobId, b.id, b.lease_token!, "x", true, null, now));
      const r = await failShard(db, jobId, a.id, a.lease_token!, "exit 134", true, "retry", now);
      expect(r.body.state).toBe(attempt < 3 ? "failed" : "dead");
      expect(r.wake).toBe(attempt === 3);
      // Not pending before the delay (10 s, doubling), pending after it
      expect(await sweepJob(db, jobId, now)).toEqual([]);
      now += 10 * 2 ** (attempt - 1);
      const msgs = await sweepJob(db, jobId, now);
      if (attempt < 3) expect(msgs.map((m) => m.shard_id)).toContain("loc-0000");
    }
    expect((await getShard(db, jobId, "loc-0000"))!.state).toBe("dead");
    expect(await failShard(db, jobId, "loc-0000", "gone", "late", true, null, now)).toMatchObject({ body: { noop: true } });
  });

  it("expires a lease whose holder stopped heartbeating; the next attempt is 2", async () => {
    const a = (await pullShard(db, k_worker, null, now))!;
    now += 30; // not yet: expiry is strictly after lease_expires_at
    expect(await sweepJob(db, jobId, now)).toEqual([]);
    now += 1;
    expect(await sweepJob(db, jobId, now)).toEqual([{ v: 1, job_id: jobId, shard_id: "loc-0000", attempt: 1 }]);
    expect((await heartbeatShard(db, jobId, a.id, a.lease_token!, {}, now)).ok).toBe(false);
    const again = (await pullShard(db, k_worker, null, now))!;
    expect([again.id, again.attempt]).toEqual(["loc-0000", 2]);
    expect((await complete(a)).status).toBe(409); // the first holder's late complete
    expect((await complete(again)).status).toBe(200);
    const shards = (await jobStatus(db, jobId))!.shards as { id: string; attempts: { outcome: string }[] }[];
    expect(shards[0].attempts.map((x) => x.outcome)).toEqual(["expired", "done"]);
  });

  it("cancels pending and leased shards; late calls of the holder are no-ops", async () => {
    const a = (await pullShard(db, k_worker, null, now))!;
    expect(await cancelJob(db, jobId, now)).toBe(true);
    expect(await cancelJob(db, "no-such-job", now)).toBe(false);
    expect((await heartbeatShard(db, jobId, a.id, a.lease_token!, {}, now))).toMatchObject({ ok: false, state: "cancelled" });
    expect(await complete(a)).toMatchObject({ status: 200, body: { noop: true } });
    expect(await failShard(db, jobId, a.id, a.lease_token!, "cancelled", true, null, now)).toMatchObject({ status: 200 });
    const status = (await jobStatus(db, jobId))!;
    expect(status.state).toBe("cancelled");
    expect((status.shards as { state: string }[]).map((s) => s.state)).toEqual(["cancelled", "cancelled", "cancelled"]);
    expect(await pullShard(db, k_worker, null, now)).toBeNull();
  });

  it("calls three same-class failures of different shards correlated", async () => {
    await insertJob(db, "job-2", spec({ shard_frames: 40 }), now);
    await planJob(db, "job-2", now);
    for (const id of ["loc-0000", "loc-0001", "loc-0002"]) {
      const r = (await claimShard(db, "job-2", id, 0, k_worker, now))!;
      now += 1;
      await failShard(db, "job-2", id, r.lease_token!, "timeout: wall 3120 s", true, "timeout", now);
      if (id !== "loc-0002") expect(await correlatedFailure(db, "job-2")).toBeNull();
    }
    expect(await correlatedFailure(db, "job-2")).toBe("timeout: timeout");
  });
});

describe("normalizeSpec", () => {
  it("refuses what cannot be planned", () => {
    expect(() => spec({ n_frames: 1 })).toThrow("at least 2 frames required");
    expect(() => spec({ n_frames: 500 })).toThrow("outside the scene's 499 frames");
    expect(() => spec({ start_frame: 5 })).toThrow("not a multiple of K = 10");
    expect(() => spec({ poses_key: null })).toThrow("needs 'poses_key'");
    expect(() => spec({ params_key: "../x" })).toThrow("not a valid object key");
    expect(() => spec({ extra_cli_args: { both: [] } })).toThrow("extra_cli_args");
  });

  it("fills the defaults", () => {
    const s = spec({ n_frames: null });
    expect(s.n_frames).toBe(499);
    expect(s.max_attempts).toBe(3);
    expect(s.max_wall_s).toEqual({ loc: null, tracks: null });
    expect(s.wait_seconds).toBe(300);
  });
});
