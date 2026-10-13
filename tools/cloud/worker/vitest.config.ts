import { defineConfig } from "vitest/config";

// The tests run in Node: plan.ts, spec.ts and db.ts import no Workers
// runtime module, and db.ts runs on node:sqlite (test/d1-sqlite.ts). The
// Worker as a whole is exercised by tools/cloud/tests/e2e_wrangler.sh.
export default defineConfig({
  test: { include: ["test/**/*.test.ts"], environment: "node" },
});
