#!/bin/bash
# Download testcase videos from the Hugging Face dataset
# prcvlabs/multiview-retail-tracking. The manifests and ground truth are
# bundled in data/testcases, and are copied next to the videos so that each
# downloaded testcase directory is complete.
#
# Needs the Hugging Face CLI (pip install -U huggingface_hub); no login is
# required.

set -euo pipefail

THIS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUNDLED_D="$(cd "$THIS_DIR/../data/testcases" && pwd)"
REPO_ID="prcvlabs/multiview-retail-tracking"
DEFAULT_D="${MULTIVIEW_TESTCASE_DATA:-$HOME/multiview-data}"

show_help()
{
    cat <<EOF

   Usage: $(basename $0) [OPTIONS...] <id>... | all

      Downloads testcases from the Hugging Face dataset
      '$REPO_ID' into <dir>/testcases/<name>.

      An <id> is a testcase number (e.g., '7' or '007'), or a full
      testcase name (e.g., '007_site03_sparse_tracks'). Pass 'all' to
      download every testcase.

   Options:

      -h|--help      Show help.
      -d <dir>       Download directory. Default is: $DEFAULT_D
                     (or set MULTIVIEW_TESTCASE_DATA)
      -l|--list      List the testcases, and exit.
      --dry-run      Print the download command, but do not run it.

   The dataset is at https://huggingface.co/datasets/$REPO_ID.

EOF
}

list_testcases()
{
    ls "$BUNDLED_D"
}

# ------------------------------------------------------------ Parse Commandline

OUT_D="$DEFAULT_D"
DRY_RUN=0
SHOW_HELP=0
LIST_ONLY=0
HAS_ERROR=0
IDS=()

[ "$#" = "0" ] && SHOW_HELP=1

while [ "$#" -gt "0" ] ; do
    [ "$1" = "-h" ] || [ "$1" = "--help" ] && SHOW_HELP=1 && shift && continue
    [ "$1" = "-l" ] || [ "$1" = "--list" ] && LIST_ONLY=1 && shift && continue
    [ "$1" = "--dry-run" ] && DRY_RUN=1 && shift && continue
    if [ "$1" = "-d" ] ; then
        [ "$#" -lt "2" ] && echo "Option '-d' requires an argument." && exit 1
        OUT_D="$2"
        shift 2
        continue
    fi
    if [ "${1:0:1}" = "-" ] ; then
        echo "Unknown option '$1'."
        HAS_ERROR=1
        shift
        continue
    fi
    IDS+=("$1")
    shift
done

if [ "$SHOW_HELP" = "1" ] ; then
    show_help
    exit 0
fi

if [ "$LIST_ONLY" = "1" ] ; then
    list_testcases
    exit 0
fi

# ----------------------------------------------------------------------- Sanity

# Echos the name of the bundled testcase that matches an id
resolve_id()
{
    local ID="$1"
    if [[ "$ID" =~ ^[0-9]{1,3}$ ]] ; then
        ID="$(printf %03d "$((10#$ID))")"
        list_testcases | grep -E "^${ID}_" || true
    elif [ -d "$BUNDLED_D/$ID" ] && [ "$ID" = "$(basename "$ID")" ] ; then
        echo "$ID"
    fi
}

ALL=0
NAMES=()
for ID in "${IDS[@]}" ; do
    if [ "$ID" = "all" ] ; then
        ALL=1
        NAMES=($(list_testcases))
        break
    fi
    MATCHES=($(resolve_id "$ID"))
    if [ "${#MATCHES[@]}" = "0" ] ; then
        echo "Unknown testcase '$ID'; pass '--list' to list the testcases."
        HAS_ERROR=1
    fi
    NAMES+=("${MATCHES[@]}")
done

if [ "${#NAMES[@]}" = "0" ] && [ "$HAS_ERROR" = "0" ] ; then
    echo "Must specify at least one testcase, or 'all'."
    HAS_ERROR=1
fi

if [ "$HAS_ERROR" = "1" ] ; then
    echo "Aborting."
    exit 1
fi

NAMES=($(printf '%s\n' "${NAMES[@]}" | sort -u))

# --------------------------------------------------------------------- Download

INCLUDES=(--include "testcases/*")
if [ "$ALL" = "0" ] ; then
    INCLUDES=()
    for NAME in "${NAMES[@]}" ; do
        INCLUDES+=(--include "testcases/$NAME/*")
    done
fi

EXCLUDES=()
for F in manifest.json ground-truth.json expected-output.text hash.md5 ; do
    EXCLUDES+=(--exclude "testcases/*/$F")
done

CMD=(hf download "$REPO_ID" --repo-type dataset --local-dir "$OUT_D"
     "${INCLUDES[@]}" "${EXCLUDES[@]}")

if [ "$DRY_RUN" = "1" ] ; then
    printf '%q ' "${CMD[@]}"
    echo
    for NAME in "${NAMES[@]}" ; do
        echo "cp $BUNDLED_D/$NAME/* $OUT_D/testcases/$NAME/"
    done
    exit 0
fi

if ! command -v hf > /dev/null ; then
    echo "Failed to find 'hf'; install it with 'pip install -U huggingface_hub'."
    exit 1
fi

mkdir -p "$OUT_D"
"${CMD[@]}"

for NAME in "${NAMES[@]}" ; do
    if ! ls "$OUT_D/testcases/$NAME"/*.mp4 > /dev/null 2>&1 ; then
        echo "No videos were downloaded for testcase '$NAME'."
        HAS_ERROR=1
        continue
    fi
    cp "$BUNDLED_D/$NAME"/* "$OUT_D/testcases/$NAME/"
done

echo
echo "Testcases are in: $OUT_D/testcases"
exit "$HAS_ERROR"
