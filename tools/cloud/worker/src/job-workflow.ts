// One Workflow instance per job (its id is the job id):
//
//   plan -> enqueue the loc shards -> wait for them -> promote and enqueue
//   the tracks job -> wait for it -> finalize (summary.json, delete the
//   losing attempts' objects, job done)
//
// Each wait is a loop of waitForEvent('shard-event') with a timeout of the
// job's wait_seconds, followed by a sweep step (expire leases, retry failed
// shards when their delay has passed, tally). `complete` and `fail` send the
// event only when a kind is finished or a shard died, so the number of steps
// grows with the job's wall time / wait_seconds, not with its shard count.
// The Workflow is the only re-enqueuer; D1 is the authority.

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

// --------------------------------------------------------------- wakeWorkflow
// Wakes the job's Workflow from its wait; it would sweep at the timeout
// anyway, so a failure here only costs time.
export async function wakeWorkflow(env: Env, jobId: string, kind: string): Promise<void> {
  try {
    const instance = await env.JOB_WORKFLOW.get(jobId);
    await instance.sendEvent({ type: "shard-event", payload: { kind } });
  } catch (e) {
    console.warn(`wake ${jobId}: ${String(e)}`);
  }
}

type SweepResult =
  | { outcome: "running"; done: number; total: number; requeued: number }
  | { outcome: "failed" | "cancelled"; error: string | null };

// ------------------------------------------------------------- sweepAndTally
async function sweepAndTally(env: Env, jobId: string, kind: ShardKind): Promise<SweepResult> {
  const requeued = await enqueue(env, await sweepJob(env.DB, jobId, now_()));
  const job = await loadJob(env.DB, jobId);
  if (!job || job.row.state === "cancelled") return { outcome: "cancelled", error: null };
  if (job.row.state === "failed") return { outcome: "failed", error: job.row.error };
  const t = await tally(env.DB, jobId, kind);
  const correlated = await correlatedFailure(env.DB, jobId);
  const error = t.dead > 0 ? t.dead_error : correlated ? `correlated failures: ${correlated}` : null;
  if (error !== null) {
    await failJob(env.DB, jobId, error, now_());
    return { outcome: "failed", error };
  }
  return { outcome: "running", done: t.done, total: t.total, requeued };
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
  // instance ({state: "cancelled"}); a failed one errors it.
  private async awaitKind_(
    step: WorkflowStep,
    jobId: string,
    kind: ShardKind,
    waitSeconds: number,
  ): Promise<{ state: string } | null> {
    for (let i = 0; ; ++i) {
      try {
        await step.waitForEvent(`shard-event-${kind}-${i}`, {
          type: "shard-event",
          timeout: `${waitSeconds} seconds`,
        });
      } catch {
        // The timeout: sweep anyway
      }
      const t = await step.do(`sweep-${kind}-${i}`, async () => sweepAndTally(this.env, jobId, kind));
      if (t.outcome === "cancelled") return { state: "cancelled" };
      if (t.outcome === "failed") throw new NonRetryableError(`job ${jobId} failed: ${t.error}`);
      if (t.outcome === "running" && t.done === t.total) return null;
    }
  }
}
