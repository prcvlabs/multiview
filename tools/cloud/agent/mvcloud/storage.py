"""Blob transfer: local paths and file:// URLs now, http(s):// for the
Worker's blob proxy and presigned URLs.

Every download is verified against the size and md5 it was announced with,
and every file appears under its final name only when it is complete.
"""

import hashlib
import json
import os
import shutil
import time
import urllib.error
import urllib.parse
import urllib.request

k_chunk = 1 << 20
k_put_attempts = 5
k_backoff_s = 0.5  # the first retry's delay; doubles each time
k_timeout_s = 60


class TransferError(Exception):
    """A download or upload failed, or its content is not what was announced."""


# ------------------------------------------------------------------- local_path
def local_path(url):
    """The filesystem path of a file:// URL or a bare path; None for any
    other URL."""
    u = urllib.parse.urlparse(str(url))
    if u.scheme == "file":
        return urllib.parse.unquote(u.path)
    if u.scheme == "":
        return str(url)
    return None


# --------------------------------------------------------------------- file_url
def file_url(path):
    return "file://" + urllib.parse.quote(os.path.abspath(path))


# --------------------------------------------------------------------- file_md5
def file_md5(path):
    h = hashlib.md5()
    with open(path, "rb") as f:
        for block in iter(lambda: f.read(k_chunk), b""):
            h.update(block)
    return h.hexdigest()


# ------------------------------------------------------------------------ copy_
def copy_(src, dst):
    """Copies the stream `src` to the file `dst`; returns (bytes, md5)."""
    h = hashlib.md5()
    size = 0
    with open(dst, "wb") as f:
        for block in iter(lambda: src.read(k_chunk), b""):
            h.update(block)
            f.write(block)
            size += len(block)
    return size, h.hexdigest()


# ------------------------------------------------------------------------ fetch
def fetch(url, dest, *, size=None, md5=None, headers=None):
    """Downloads `url` to `dest`, verifying `size` and `md5` when given. The
    data goes to `dest`.part first, so `dest` is either absent or complete."""
    os.makedirs(os.path.dirname(os.path.abspath(dest)), exist_ok=True)
    part = f"{dest}.part"
    try:
        path = local_path(url)
        if path is not None:
            with open(path, "rb") as src:
                got_size, got_md5 = copy_(src, part)
        else:
            request = urllib.request.Request(url, headers=headers or {})
            with urllib.request.urlopen(request, timeout=k_timeout_s) as src:
                got_size, got_md5 = copy_(src, part)
        if size is not None and got_size != size:
            raise TransferError(f"{url}: {got_size} bytes, expected {size}")
        if md5 is not None and got_md5 != md5:
            raise TransferError(f"{url}: md5 {got_md5}, expected {md5}")
        os.replace(part, dest)
    except (OSError, urllib.error.URLError) as e:
        raise TransferError(f"{url}: {e}") from e
    finally:
        if os.path.exists(part):
            os.remove(part)


# ----------------------------------------------------------------- fetch_cached
def fetch_cached(url, cache_dir, name, *, size=None, md5=None, headers=None):
    """Returns a local file with the content of `url`: a cached copy keyed by
    its md5 when the md5 is known, else a fresh download."""
    if md5 is None:
        dest = os.path.join(cache_dir, "nomd5", hashlib.md5(url.encode()).hexdigest(), name)
        fetch(url, dest, size=size, headers=headers)
        return dest
    dest = os.path.join(cache_dir, md5[:2], md5, name)
    if os.path.isfile(dest) and (size is None or os.path.getsize(dest) == size):
        return dest
    fetch(url, dest, size=size, md5=md5, headers=headers)
    return dest


# --------------------------------------------------------------------- request_
def request_(method, url, *, data=None, headers=None, length=None):
    """One HTTP request; returns (status, headers, body)."""
    h = dict(headers or {})
    if length is not None:
        h["Content-Length"] = str(length)
    request = urllib.request.Request(url, data=data, headers=h, method=method)
    with urllib.request.urlopen(request, timeout=k_timeout_s) as r:
        return r.status, dict(r.headers), r.read()


# -------------------------------------------------------------------- retrying_
def retrying_(what, f):
    """Calls f() up to k_put_attempts times with exponential backoff; a
    client error (4xx) is not retried."""
    for i in range(k_put_attempts):
        try:
            return f()
        except (OSError, urllib.error.URLError) as e:
            client_error = isinstance(e, urllib.error.HTTPError) and e.code < 500
            if client_error or i + 1 == k_put_attempts:
                raise TransferError(f"{what}: {e}") from e
            time.sleep(min(30.0, k_backoff_s * 2**i))


# -------------------------------------------------------------------------- put
def put(url, src, *, headers=None):
    """Uploads the file `src` to `url`. A file:// destination is written
    atomically (a temporary name, then a rename)."""
    path = local_path(url)
    if path is not None:
        os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
        tmp = f"{path}.tmp"
        try:
            shutil.copyfile(src, tmp)
            os.replace(tmp, path)
        except OSError as e:
            raise TransferError(f"{url}: {e}") from e
        finally:
            if os.path.exists(tmp):
                os.remove(tmp)
        return

    def put_once():
        with open(src, "rb") as f:
            request_("PUT", url, data=f, headers=headers, length=os.path.getsize(src))

    retrying_(f"PUT {url}", put_once)


# ---------------------------------------------------------------- put_multipart
def put_multipart(spec, src, *, headers=None):
    """Uploads `src` in parts of spec["part_bytes"] (the Worker's proxy for
    objects too large for one request):

        POST init_url                          -> {"upload_id": U}
        PUT  part_url?upload_id=U&part=N       -> {"etag": E}  (N from 1)
        POST complete_url?upload_id=U  {"parts": [{"part": N, "etag": E}...]}

    and DELETE abort_url?upload_id=U when a part fails."""
    part_bytes = int(spec["part_bytes"])
    if part_bytes <= 0:
        raise TransferError(f"part_bytes must be positive, got {part_bytes}")
    json_headers = {**(headers or {}), "Content-Type": "application/json"}
    with_id = lambda u, **q: f"{u}{'&' if '?' in u else '?'}{urllib.parse.urlencode(q)}"

    _, _, body = retrying_("multipart init", lambda: request_("POST", spec["init_url"], data=b"",
                                                              headers=json_headers))
    upload_id = json.loads(body)["upload_id"]
    parts = []
    try:
        with open(src, "rb") as f:
            for number in range(1, 1 + max(1, -(-os.path.getsize(src) // part_bytes))):
                block = f.read(part_bytes)
                url = with_id(spec["part_url"], upload_id=upload_id, part=number)
                _, _, body = retrying_(f"part {number}",
                                       lambda: request_("PUT", url, data=block, headers=headers,
                                                        length=len(block)))
                parts.append({"part": number, "etag": json.loads(body)["etag"]})
        doc = json.dumps({"parts": parts}).encode()
        retrying_("multipart complete",
                  lambda: request_("POST", with_id(spec["complete_url"], upload_id=upload_id),
                                   data=doc, headers=json_headers, length=len(doc)))
    except TransferError:
        try:
            request_("DELETE", with_id(spec["abort_url"], upload_id=upload_id), headers=headers)
        except (OSError, urllib.error.URLError):
            pass
        raise


# ------------------------------------------------------------------------- head
def head(url, *, headers=None):
    """The size of the object at `url`, or None if there is none."""
    path = local_path(url)
    if path is not None:
        return os.path.getsize(path) if os.path.isfile(path) else None
    try:
        _, h, _ = request_("HEAD", url, headers=headers)
    except urllib.error.HTTPError as e:
        if e.code == 404:
            return None
        raise TransferError(f"HEAD {url}: {e}") from e
    except (OSError, urllib.error.URLError) as e:
        raise TransferError(f"HEAD {url}: {e}") from e
    length = h.get("Content-Length")
    return int(length) if length is not None else None
