// A D1Database on node:sqlite, enough for db.ts: prepare/bind/first/all/run
// and batch (one transaction). The schema is the real migration.

import { readFileSync } from "node:fs";
import { DatabaseSync } from "node:sqlite";

type Value = string | number | null;

class Statement {
  constructor(
    private readonly db: DatabaseSync,
    private readonly sql: string,
    private readonly params: Value[] = [],
  ) {}

  bind(...params: Value[]): Statement {
    return new Statement(this.db, this.sql, params);
  }

  firstSync(): Record<string, unknown> | null {
    const row = this.db.prepare(this.sql).get(...this.params);
    return row ? { ...row } : null;
  }

  allSync() {
    return { success: true, meta: { changes: 0 }, results: this.db.prepare(this.sql).all(...this.params).map((r) => ({ ...r })) };
  }

  runSync() {
    const info = this.db.prepare(this.sql).run(...this.params);
    return { success: true, meta: { changes: Number(info.changes) }, results: [] };
  }

  async first() {
    return this.firstSync();
  }

  async all() {
    return this.allSync();
  }

  async run() {
    return this.runSync();
  }
}

export class SqliteD1 {
  readonly db = new DatabaseSync(":memory:");

  constructor(migration: string) {
    this.db.exec(readFileSync(migration, "utf8"));
  }

  prepare(sql: string): Statement {
    return new Statement(this.db, sql);
  }

  async batch(statements: Statement[]) {
    this.db.exec("BEGIN");
    try {
      const results = statements.map((s) => s.runSync());
      this.db.exec("COMMIT");
      return results;
    } catch (e) {
      this.db.exec("ROLLBACK");
      throw e;
    }
  }

  asD1(): D1Database {
    return this as unknown as D1Database;
  }
}
