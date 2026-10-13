#!/bin/bash
# T10: the S3 paths against a MinIO server in Docker, standing in for R2.
#
# Usage: tools/cloud/tests/t10_minio.sh [--frames N] [--port P] [--out <dir>]
#                                       [--keep]
#
#   1. starts MinIO (MINIO_IMAGE, default bitnamilegacy/minio:latest: the
#      minio/minio and quay.io/minio images are no longer pullable without a
#      login) on 127.0.0.1:<port> with the buckets `multiview` (private) and
#      `mvpublic` (anonymous read/write, standing in for presigned URLs)
#   2. uploads testcase 007 (manifest, videos, ground truth) to
#      s3://multiview/scenes/007_site03_sparse_tracks/
#   3. mvcloud.storage put / head / fetch of a 5 MB file on mvpublic over HTTP
#   4. multiview-testcases "[s3-live]": s3_store (in-memory body), s3_exists,
#      s3_load, s3_remove through MULTIVIEW_S3_ENDPOINT
#   5. `pipeline -m s3://multiview/scenes/007.../manifest.json` twice with an
#      empty MULTIVIEW_LAZY_S3_DIR (loc shard of [0,N) with replayed poses):
#      the first run fetches the videos, the second reads the lazy-s3 cache;
#      both envelopes must equal the first N frames of ref-120.data, byte for
#      byte (`ldata diff`)
#
# The container is removed at the end unless --keep. Exit status 0 if every
# step passes.

set -euo pipefail

THIS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$THIS_DIR/../../.." && pwd)"
DATA_D="${MULTIVIEW_TESTCASE_DATA:-$HOME/multiview-data}"
TC=007_site03_sparse_tracks
IMAGE="${MINIO_IMAGE:-bitnamilegacy/minio:latest}"
NAME=mvcloud-t10-minio
CLI="$ROOT/multiview/multiview_cpp/build/cmake-release/multiview-cli"
FRAMES=30
PORT=9000
OUT=""
KEEP=0

while [ "$#" -gt 0 ] ; do
    case "$1" in
        --frames) shift ; FRAMES="$1" ;;
        --port)   shift ; PORT="$1" ;;
        --out)    shift ; OUT="$1" ;;
        --keep)   KEEP=1 ;;
        -h|--help) sed -n '2,/^$/p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//' ; exit 0 ;;
        *) echo "t10: unknown argument '$1'" ; exit 1 ;;
    esac
    shift
done

GOLD="$DATA_D/cloud-golden/007"
for f in "$CLI" "$DATA_D/testcases/$TC/manifest.json" "$GOLD/ref-120.data" "$GOLD/params-replay.json" ; do
    [ -e "$f" ] || { echo "t10: missing $f" ; exit 1 ; }
done
[ "$FRAMES" -ge 2 ] && [ "$FRAMES" -le 120 ] || { echo "t10: --frames must be in [2, 120]" ; exit 1 ; }
[ -n "$OUT" ] || OUT="$(mktemp -d "${TMPDIR:-/tmp}/t10-minio.XXXXXX")"
mkdir -p "$OUT/lazy-s3"
FAILED=0

check() # check <name> <command...>: records a failure, does not stop
{
    local name="$1" ; shift
    if "$@" ; then echo "t10: PASS $name" ; else echo "t10: FAIL $name" ; FAILED=1 ; fi
}

cleanup()
{
    [ "$KEEP" = "1" ] || docker rm -f -v "$NAME" > /dev/null 2>&1 || true
}
trap cleanup EXIT

# ---- 1. MinIO
export AWS_ACCESS_KEY_ID=mvcloud AWS_SECRET_ACCESS_KEY=mvcloud-t10-secret
export MULTIVIEW_S3_ENDPOINT="http://127.0.0.1:$PORT"
docker rm -f -v "$NAME" > /dev/null 2>&1 || true
if ! docker image inspect "$IMAGE" > /dev/null 2>&1 \
        && ! docker pull -q "$IMAGE" > "$OUT/pull.log" 2>&1 ; then
    echo "t10: SKIP: cannot pull $IMAGE ($(tail -n 1 "$OUT/pull.log"))"
    exit 0
fi
docker run -d --name "$NAME" -p "127.0.0.1:$PORT:9000" \
    -e MINIO_ROOT_USER="$AWS_ACCESS_KEY_ID" -e MINIO_ROOT_PASSWORD="$AWS_SECRET_ACCESS_KEY" \
    -e MINIO_DEFAULT_BUCKETS=multiview,mvpublic "$IMAGE" > /dev/null
for _ in $(seq 60) ; do
    curl -fs "$MULTIVIEW_S3_ENDPOINT/minio/health/ready" > /dev/null 2>&1 \
        && docker exec "$NAME" mc ls local/mvpublic > /dev/null 2>&1 && break
    sleep 1
done
docker exec "$NAME" mc anonymous set public local/mvpublic > /dev/null
echo "t10: $(docker exec "$NAME" minio --version | head -n 1) at $MULTIVIEW_S3_ENDPOINT"

# ---- 2. Upload 007
docker exec "$NAME" mkdir -p /tmp/up
for f in manifest.json ground-truth.json $(cd "$DATA_D/testcases/$TC" && ls ./*.mp4) ; do
    docker cp -q "$DATA_D/testcases/$TC/$f" "$NAME:/tmp/up/"
done
docker exec "$NAME" mc cp -q --recursive /tmp/up/ "local/multiview/scenes/$TC/" > /dev/null
docker exec "$NAME" mc ls "local/multiview/scenes/$TC/" | tee "$OUT/bucket.txt"

# ---- 3. mvcloud.storage over HTTP
head -c 5000000 /dev/urandom > "$OUT/blob.bin"
check "storage put / head / fetch on $MULTIVIEW_S3_ENDPOINT/mvpublic" \
    env PYTHONPATH="$ROOT/tools/cloud/agent" python3 -c "
import sys
from mvcloud import storage
src, url = sys.argv[1], sys.argv[2]
assert storage.head(url) is None, 'object exists before put'
storage.put(url, src)
size = storage.head(url)
assert size == 5000000, size
storage.fetch(url, src + '.back', size=size, md5=storage.file_md5(src))
print('   5000000 bytes, md5', storage.file_md5(src + '.back'))
" "$OUT/blob.bin" "$MULTIVIEW_S3_ENDPOINT/mvpublic/t10/blob.bin"

# ---- 4. The C++ client
check "multiview-testcases [s3-live]" \
    env MULTIVIEW_S3_TEST_PREFIX=s3://multiview/catch "$ROOT/build/dev.sh" test "[s3-live]"

# ---- 5. pipeline over an s3:// manifest, through the lazy-s3 cache
"$CLI" ldata cat -y --keep "0:$FRAMES" -o "$OUT/ref.data" "$GOLD/ref-120.data" > /dev/null
for run in 1 2 ; do
    t0="$(date +%s.%N)"
    MULTIVIEW_LAZY_S3_DIR="$OUT/lazy-s3" "$ROOT/build/dev.sh" pipeline \
        -m "s3://multiview/scenes/$TC/manifest.json" -p "$GOLD/params-replay.json" \
        -d "$OUT/run$run" -y --start 0 --n-frames "$FRAMES" --save-loc-data --no-tracks \
        --no-stereo --no-still --verbosity 1 > "$OUT/run$run.log" 2>&1 \
        || { echo "t10: FAIL pipeline run $run (log $OUT/run$run.log)" ; FAILED=1 ; }
    echo "t10: run $run took $(echo "$(date +%s.%N) - $t0" | bc) s (with the wait for the GPU lock)"
    check "run $run envelope == ref-120.data [0,$FRAMES) (bytes)" \
        "$CLI" ldata diff --json "$OUT/diff$run.json" "$OUT/run$run/localization.data" "$OUT/ref.data"
    # size, modification time and name of every file in the lazy-s3 cache
    find "$OUT/lazy-s3" -type f -printf '%s %T@ %P\n' | sort -k3 > "$OUT/lazy-s3-$run.txt"
done
cat "$OUT/lazy-s3-1.txt"
check "run 1 cached the manifest and 6 videos" \
    bash -c "[ \"\$(wc -l < '$OUT/lazy-s3-1.txt')\" = 7 ]"
check "run 2 read the cache (no file rewritten)" cmp -s "$OUT/lazy-s3-1.txt" "$OUT/lazy-s3-2.txt"

if [ "$FAILED" = "0" ] ; then echo "t10: PASS (outputs in $OUT)" ; else echo "t10: FAIL (outputs in $OUT)" ; fi
exit "$FAILED"
