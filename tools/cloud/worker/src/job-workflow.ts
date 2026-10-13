// One Workflow instance per job (its id is the job id):
//
//   plan -> enqueue the loc shards -> wait for them -> promote and enqueue
//   the tracks job -> wait for it -> finalize (summary.json, delete the
//   losing attempts' objects, job done)
//
// Each wait is a loop of waitForEvent('shard-event') with a timeout, followed
// by a sweep step (expire leases, retry failed shards when their delay has
// passed, tally). `complete` and `fail` send the event only when a kind is
// finished or a shard died, so the steps do not grow with the shard count.
// The Workflow is the only re-enqueuer; D1 is the authority, and the last
// loc shard's `complete` promotes the tracks job itself.
//
// Steps: an instance may take 1,024. The plan and finalize take 4, each kind
// 2 per wait. The timeout is the job's wait_seconds (300 s by default) while
// a shard is leased or failed or the tally moves, and doubles up to
// k_idle_wait_max_s (1 h) while nothing happens: a job queued behind others
// costs 2 steps an hour once its wait has grown (about 10 to get there), a
// running one 2 per wait_seconds, 24 an hour at 300 s. So the instance
// lasts about 42 hours of running shards, plus the time spent queued (an
// agent that leases a shard during a long wait does not wake the instance,
// so a lease lost then is swept at the end of that wait at the latest). An
// instance that errors anyway (out of steps, or a sweep out of retries) does
// not strand the job: GET /v1/jobs/{id}, the status page and the cron
// trigger fail a live job whose instance has errored (failStrandedJob).

import { WorkflowEntrypoint, type WorkflowEvent, type WorkflowStep } from "cloudflare:workers";
import { NonRetryableError } from "cloudflare:workflows";

import { deleteExcept } from "./blobs";
import {
  correlatedFailure,
  failJob,
  finishJob,
  loadJob,
  pendingMessages,
  planJob,
  promoteTracks,
  shardsOf,
  sweepJob,
  tally,
} from "./db";
import type { ShardKind } from "./plan";
import type { DoneRecord, Env, JobParams, ShardMessage } from "./types";

function now_(): number {
  return Date.now() / 1000;
}

// --------------------------------------------------------------------- enqueue
// Queue messages are dispatch only (the /v1/pull dispatcher reads D1), so a
// lost or duplicate message does no harm.
export async function enqueue(env: Env, messages: ShardMessage[]): Promise<number> {
  for (let i = 0; i < messages.length; i += 100)
    await env.SHARDS.sendBatch(messages.slice(i, i + 100).map((body) => ({ body })));
  return messages.length;
}

const k_wake_timeout_ms = 5000;
const k_idle_wait_max_s = 3600;

// ---------------------------------------------------------------- wakeWorkflow
// Wakes the job's Workflow from its wait; it would sweep at the timeout
// anyway, so a failure here only costs time, and the wait for an answer is
// bounded so that a request's waitUntil always settles.
export async function wakeWorkflow(env: Env, jobId: string, kind: string): Promise<void> {
  let timer: ReturnType<typeof setTimeout> | undefined;
  const timeout = new Promise<never>((_, reject) => {
    timer = setTimeout(() => reject(new Error(`no answer in ${k_wake_timeout_ms} ms`)), k_wake_timeout_ms);
  });
  try {
    const instance = await env.JOB_WORKFLOW.get(jobId);
    await Promise.race([instance.sendEvent({ type: "shard-event", payload: { kind } }), timeout]);
  } catch (e) {
    console.warn(`wake ${jobId}: ${String(e)}`);
  } finally {
    if (timer !== undefined) clearTimeout(timer);
  }
}

type SweepResult =
  | { outcome: "running"; done: number; total: number; busy: number; requeued: number }
  | { outcome: "failed" | "cancelled"; error: string | null };

// --------------------------------------------------------------- sweepAndTally
async function sweepAndTally(env: Env, jobId: string, kind: ShardKind): Promise<SweepResult> {
  const requeued = await enqueue(env, await sweepJob(env.DB, jobId, now_()));
  const job = await loadJob(env.DB, jobId);
  if (!job || job.row.state === "cancelled") return { outcome: "cancelled", error: null };
  if (job.row.state === "failed") return { outcome: "failed", error: job.row.error };
  const t = await tally(env.DB, jobId, kind);
  const correlated = await correlatedFailure(env.DB, jobId);
  let error: string | null = null;
  if (t.dead > 0) error = t.dead_error;
  else if (t.params_md5s > 1) error = `the ${kind} shards were computed with ${t.params_md5s} different params_md5`;
  else if (correlated) error = `correlated failures: ${correlated}`;
  if (error !== null) {
    if (await failJob(env.DB, jobId, error, now_())) return { outcome: "failed", error };
    const ended = await loadJob(env.DB, jobId); // cancelled meanwhile
    if (ended?.row.state === "cancelled") return { outcome: "cancelled", error: null };
    return { outcome: "failed", error: ended?.row.error ?? error };
  }
  return { outcome: "running", done: t.done, total: t.total, busy: t.leased + t.failed, requeued };
}

// -------------------------------------------------------------------- finalize
async function finalize(env: Env, jobId: string): Promise<{ state: string; deleted_objects: number }> {
  const job = await loadJob(env.DB, jobId);
  if (!job) throw new NonRetryableError(`no job '${jobId}'`);
  const rows = await shardsOf(env.DB, jobId);
  const tracks = rows.find((r) => r.kind === "tracks");
  const tracksDone = JSON.parse(tracks?.done_json ?? "{}") as DoneRecord;
  const summary = {
    job_id: jobId,
    scene_id: job.spec.scene_id,
    shards: rows.map((r) => ({ id: r.id, attempt: r.attempt, output_prefix: r.output_prefix })),
    score: tracksDone.score ?? null,
    n_tracks: tracksDone.n_tracks ?? null,
    finished_at: now_(),
  };
  await env.BUCKET.put(`jobs/${jobId}/summary.json`, JSON.stringify(summary, null, 1));
  const winners = rows.map((r) => r.output_prefix).filter((p): p is string => p !== null);
  const deleted = await deleteExcept(env.BUCKET, `jobs/${jobId}/shards/`, winners);
  await finishJob(env.DB, jobId, { ...summary, deleted_objects: deleted }, now_());
  return { state: "done", deleted_objects: deleted };
}

// ----------------------------------------------------------------- JobWorkflow
export class JobWorkflow extends WorkflowEntrypoint<Env, JobParams> {
  async run(event: Readonly<WorkflowEvent<JobParams>>, step: WorkflowStep): Promise<unknown> {
    const jobId = event.payload.jobId;
    const plan = await step.do("plan", async () => {
      const nLoc = await planJob(this.env.DB, jobId, now_());
      const job = await loadJob(this.env.DB, jobId);
      return { n_loc: nLoc, wait_seconds: job?.spec.wait_seconds ?? 300 };
    });
    await step.do("enqueue-loc", async () => enqueue(this.env, await pendingMessages(this.env.DB, jobId, "loc")));
    const loc = await this.awaitKind_(step, jobId, "loc", plan.wait_seconds);
    if (loc !== null) return loc;
    await step.do("enqueue-tracks", async () => enqueue(this.env, await promoteTracks(this.env.DB, jobId, now_())));
    const tracks = await this.awaitKind_(step, jobId, "tracks", plan.wait_seconds);
    if (tracks !== null) return tracks;
    return step.do("finalize", async () => finalize(this.env, jobId));
  }

  // Waits until every shard of `kind` is done: null. A cancelled job ends the
  // instance ({state: "cancelled"}); a failed one errors it. The timeout
  // doubles while no shard is leased or failed and the tally stands still
  // (the header's step arithmetic); the results it depends on are the
  // steps', so a replay computes the same timeouts.
  private async awaitKind_(
    step: WorkflowStep,
    jobId: string,
    kind: ShardKind,
    waitSeconds: number,
  ): Promise<{ state: string } | null> {
    let wait = waitSeconds;
    let lastDone = -1;
    for (let i = 0; ; ++i) {
      try {
        await step.waitForEvent(`shard-event-${kind}-${i}`, {
          type: "shard-event",
          timeout: `${wait} seconds`,
        });
      } catch {
        // The timeout: sweep anyway
      }
      const t = await step.do(`sweep-${kind}-${i}`, async () => sweepAndTally(this.env, jobId, kind));
      if (t.outcome === "cancelled") return { state: "cancelled" };
      if (t.outcome !== "running") throw new NonRetryableError(`job ${jobId} failed: ${t.error}`);
      if (t.done === t.total) return null;
      const idle = t.busy === 0 && t.requeued === 0 && t.done === lastDone;
      wait = idle ? Math.max(waitSeconds, Math.min(k_idle_wait_max_s, 2 * wait)) : waitSeconds;
      lastDone = t.done;
    }
  }
}
