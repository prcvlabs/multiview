// The blob proxy: /v1/blobs/<key> on the R2 binding, for the harness levels
// that have no presigned URLs (wrangler dev and a Worker without R2 API
// keys). Objects larger than one request (the 100 MB body limit of a Worker)
// go up through R2 multipart uploads, in the protocol of the agent's
// storage.put_multipart:
//
//   GET    /v1/blobs/<key>                         the object (HEAD: its size)
//   PUT    /v1/blobs/<key>                         one request; x-mvcloud-md5 is verified
//   POST   /v1/blobs/<key>?uploads                 -> {upload_id}       (x-mvcloud-md5 kept)
//   PUT    /v1/blobs/<key>?upload_id=U&part=N      -> {etag}             (N from 1)
//   POST   /v1/blobs/<key>?upload_id=U  {parts: [{part, etag}...]}  -> {key, size, etag}
//   DELETE /v1/blobs/<key>?upload_id=U             abort
//   GET    /v1/blobs?prefix=P                      -> {objects: [{key, size, etag, md5}]}
//
// HEAD and GET report the md5 in x-mvcloud-md5 when it is known: the one the
// uploader sent, or the one R2 computed for a single-part object.

import { isBlobKey } from "./spec";

const k_md5_header = "x-mvcloud-md5";

// ------------------------------------------------------------------------ json
export function json(status: number, body: unknown): Response {
  return new Response(JSON.stringify(body), { status, headers: { "content-type": "application/json" } });
}

// ---------------------------------------------------------------------- toHex_
function toHex_(buf: ArrayBuffer | undefined): string | null {
  return buf ? [...new Uint8Array(buf)].map((b) => b.toString(16).padStart(2, "0")).join("") : null;
}

// ------------------------------------------------------------------- objectMd5
export function objectMd5(o: R2Object): string | null {
  return o.customMetadata?.md5 ?? toHex_(o.checksums.md5);
}

// ------------------------------------------------------------------ headObject
// Size and md5 of an object, or null if there is none.
export async function headObject(
  bucket: R2Bucket,
  key: string,
): Promise<{ bytes: number; md5: string | null } | null> {
  const o = await bucket.head(key);
  return o ? { bytes: o.size, md5: objectMd5(o) } : null;
}

// -------------------------------------------------------------- objectHeaders_
function objectHeaders_(o: R2Object): Headers {
  const h = new Headers({ "content-length": String(o.size), etag: o.httpEtag });
  o.writeHttpMetadata(h);
  const md5 = objectMd5(o);
  if (md5) h.set(k_md5_header, md5);
  return h;
}

// ------------------------------------------------------------------- listBlobs
export async function listBlobs(bucket: R2Bucket, prefix: string): Promise<Response> {
  const objects: { key: string; size: number; etag: string; md5: string | null }[] = [];
  let cursor: string | undefined;
  do {
    const page = await bucket.list({ prefix, cursor });
    for (const o of page.objects) objects.push({ key: o.key, size: o.size, etag: o.etag, md5: objectMd5(o) });
    cursor = page.truncated ? page.cursor : undefined;
  } while (cursor);
  return json(200, { objects });
}

// -------------------------------------------------------------------- readBlob
export async function readBlob(request: Request, bucket: R2Bucket, key: string): Promise<Response> {
  if (!isBlobKey(key)) return json(400, { error: `invalid key '${key}'` });
  if (request.method === "HEAD") {
    const o = await bucket.head(key);
    return o ? new Response(null, { headers: objectHeaders_(o) }) : new Response(null, { status: 404 });
  }
  const o = await bucket.get(key);
  if (!o) return json(404, { error: `no object '${key}'` });
  return new Response(o.body, { headers: objectHeaders_(o) });
}

// ------------------------------------------------------------------- writeBlob
// The write routes; the caller has checked that it may write `key`.
export async function writeBlob(request: Request, bucket: R2Bucket, key: string): Promise<Response> {
  if (!isBlobKey(key)) return json(400, { error: `invalid key '${key}'` });
  const q = new URL(request.url).searchParams;
  const uploadId = q.get("upload_id");

  if (request.method === "POST" && q.has("uploads")) {
    // The uploader's md5 of the whole object, kept as metadata (R2 has none
    // for a multipart object); the agent verifies it when it downloads.
    const md5 = request.headers.get(k_md5_header)?.toLowerCase() ?? null;
    const valid = md5 !== null && /^[0-9a-f]{32}$/.test(md5);
    const mp = await bucket.createMultipartUpload(key, valid ? { customMetadata: { md5 } } : undefined);
    return json(200, { upload_id: mp.uploadId });
  }
  if (uploadId !== null) {
    const mp = bucket.resumeMultipartUpload(key, uploadId);
    if (request.method === "PUT") {
      const part = Number(q.get("part"));
      if (!Number.isInteger(part) || part < 1 || part > 10000) return json(400, { error: "part must be 1..10000" });
      if (request.body === null) return json(400, { error: "empty part" });
      const p = await mp.uploadPart(part, request.body);
      return json(200, { etag: p.etag });
    }
    if (request.method === "POST") {
      const body = (await request.json()) as { parts?: { part: number; etag: string }[] };
      if (!Array.isArray(body.parts) || body.parts.length === 0) return json(400, { error: "no parts" });
      const o = await mp.complete(body.parts.map((p) => ({ partNumber: p.part, etag: p.etag })));
      return json(200, { key: o.key, size: o.size, etag: o.etag });
    }
    if (request.method === "DELETE") {
      await mp.abort();
      return json(200, { aborted: uploadId });
    }
  }
  if (request.method === "PUT") {
    const md5 = request.headers.get(k_md5_header)?.toLowerCase() ?? null;
    if (md5 !== null && !/^[0-9a-f]{32}$/.test(md5)) return json(400, { error: `invalid ${k_md5_header}` });
    try {
      const o = await bucket.put(key, request.body ?? "", {
        md5: md5 ?? undefined,
        customMetadata: md5 ? { md5 } : undefined,
      });
      return json(200, { key: o?.key ?? key, size: o?.size ?? 0, etag: o?.etag ?? null });
    } catch (e) {
      return json(md5 ? 400 : 500, { error: `PUT ${key}: ${String(e)}` });
    }
  }
  return json(405, { error: `${request.method} is not a blob write` });
}

// ---------------------------------------------------------------- deleteExcept
// Deletes every object under `prefix` that is not under one of `keep`.
// Returns the number deleted.
export async function deleteExcept(bucket: R2Bucket, prefix: string, keep: string[]): Promise<number> {
  const losers: string[] = [];
  let cursor: string | undefined;
  do {
    const page = await bucket.list({ prefix, cursor });
    for (const o of page.objects) if (!keep.some((k) => o.key.startsWith(`${k}/`))) losers.push(o.key);
    cursor = page.truncated ? page.cursor : undefined;
  } while (cursor);
  for (let i = 0; i < losers.length; i += 1000) await bucket.delete(losers.slice(i, i + 1000));
  return losers.length;
}
