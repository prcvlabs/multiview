// deleteExcept, which finalize uses to delete the losing attempts' objects
// (e2e_wrangler.sh T2 kills its first agent before it uploads anything, so
// there it deletes nothing), on an in-memory bucket with paged listings.

import { describe, expect, it } from "vitest";

import { deleteExcept } from "../src/blobs";
import { isBlobKey } from "../src/spec";

function bucket(keys: string[], pageSize: number) {
  const objects = new Set(keys);
  const deleted: string[][] = [];
  return {
    objects,
    deleted,
    async list({ prefix, cursor }: { prefix: string; cursor?: string }) {
      const all = [...objects].filter((k) => k.startsWith(prefix)).sort();
      const from = Number(cursor ?? 0);
      const page = all.slice(from, from + pageSize);
      const truncated = from + pageSize < all.length;
      return { objects: page.map((key) => ({ key })), truncated, cursor: truncated ? String(from + pageSize) : undefined };
    },
    async delete(keys: string[]) {
      deleted.push(keys);
      for (const k of keys) objects.delete(k);
    },
  };
}

describe("deleteExcept", () => {
  it("deletes every object under the prefix outside the winning prefixes", async () => {
    const p = "jobs/j/shards/";
    const b = bucket(
      [
        `${p}loc-0000/a1/stdout.log`,
        `${p}loc-0000/a1/localization.data`,
        `${p}loc-0000/a2/localization.data`,
        `${p}loc-0000/a2/done.json`,
        `${p}loc-0000/a20/done.json`, // a prefix of a2's name, not under it
        `${p}tracks/a1/pipeline-output.json`,
        "jobs/j/summary.json",
      ],
      2,
    );
    const n = await deleteExcept(b as unknown as R2Bucket, p, [`${p}loc-0000/a2`, `${p}tracks/a1`]);
    expect(n).toBe(3);
    expect([...b.objects].sort()).toEqual([
      `${p}loc-0000/a2/done.json`,
      `${p}loc-0000/a2/localization.data`,
      `${p}tracks/a1/pipeline-output.json`,
      "jobs/j/summary.json",
    ]);
  });
});

describe("isBlobKey", () => {
  it("accepts relative keys without empty, '.' or '..' segments", () => {
    expect(isBlobKey("scenes/007/abc/manifest.json")).toBe(true);
    for (const k of ["", "/abs", "a//b", "a/./b", "a/../b", "a/"]) expect(isBlobKey(k)).toBe(false);
  });
});
