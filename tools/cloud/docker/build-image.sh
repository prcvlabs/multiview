#!/bin/bash
# Builds the mvcloud worker image from a commit (default HEAD) of this
# repository: `git archive` is the build context, so uncommitted changes and
# build directories stay out (--worktree: the files git knows or would add,
# as they are on disk). Takes build/dev.sh's build lock, since the compile
# competes with the worktrees' builds.
#
# Usage: tools/cloud/docker/build-image.sh [-t tag] [--commit rev | --worktree]
#                                          [--cuda-archs "86;89"] [--jobs N]
#   default tag mvcloud:dev, 4 compile jobs; other arguments go to
#   `docker build`. The compile runs inside dockerd, outside nice and capped,
#   so the build is held to --memory 24g (an OOM there must not reach the
#   desktop session).

set -euo pipefail

THIS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$THIS_DIR/../../.." && pwd)"
TAG=mvcloud:dev
COMMIT=HEAD
WORKTREE=0
CUDA_ARCHS="86;89"
JOBS=4
EXTRA=()
while [ "$#" -gt 0 ] ; do
    case "$1" in
        -t)           shift ; TAG="$1" ;;
        --commit)     shift ; COMMIT="$1" ;;
        --worktree)   WORKTREE=1 ;;
        --cuda-archs) shift ; CUDA_ARCHS="$1" ;;
        --jobs)       shift ; JOBS="$1" ;;
        -h|--help)    sed -n '2,/^$/p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//' ; exit 0 ;;
        *)            EXTRA+=("$1") ;;
    esac
    shift
done

VERSION="$(cd "$ROOT" && echo "$(git describe --tags --always "$COMMIT") commit $(git rev-parse "$COMMIT")" | sed 's, ,:,g')"
[ "$WORKTREE" = "1" ] && VERSION="$VERSION:worktree"
echo "build-image: $TAG from $VERSION"

exec 9> /tmp/multiview-dev-build.lock
if ! flock -n 9 ; then
    echo "build-image: waiting for the build lock (/tmp/multiview-dev-build.lock)"
    flock 9
fi
context()
{
    if [ "$WORKTREE" = "1" ] ; then
        (cd "$ROOT" && git ls-files -co --exclude-standard -z | tar -c --null -T -)
    else
        git -C "$ROOT" archive --format=tar "$COMMIT"
    fi
}
context | docker build -f tools/cloud/docker/Dockerfile -t "$TAG" --memory 24g \
        --build-arg CUDA_ARCHS="$CUDA_ARCHS" --build-arg JOBS="$JOBS" \
        --build-arg MULTIVIEW_VERSION="$VERSION" "${EXTRA[@]}" -
