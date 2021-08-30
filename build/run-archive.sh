#!/bin/bash

set -e

GSUTIL="/snap/bin/gsutil"

show_help()
{
    cat <<EOF

   Usage: $(basename $0) <tag> [multiview-cli options...]

EOF
}

if [ "$1" = "-h" ] || [ "$1" = "--help" ] ; then
    show_help
    exit 0
fi

if (( $# < 1 )) ; then
    echo "Must have at least 1 argument, pass '-h' for help."
    exit 1
fi

TAG="$1"
shift

# ----------------------------------------------------------------------- Sanity

if [ ! -x "$GSUTIL" ] ; then
    echo "Cannot find $GSUTIL!"
    exit 1
fi

if [[ "$EUID" -ne 0 ]]; then
    echo "Must be root, aborting."
    exit 1
fi

if [ "$TAG" = "" ] ; then
    echo "Exepcted a tag, aborting."
    exit 1
fi

# -------------------------------------------------------- Create temp directory

TMPD="$(mktemp -d "/tmp/$(basename "$0").XXXXXX")"
trap cleanup EXIT
cleanup()
{
    rm -rf "$TMPD"
}

ARCHIVE=multiview-cli_archive_${TAG}.tar.bz2
URL="gs://multiview-deploy/$ARCHIVE"

cd "$TMPD"
gsutil cp "$URL" .

cd /
cat "$TMPD/$ARCHIVE" | bzip2 -dc | tar -xf -

ARCH="$(cat /opt/multiview/etc/arch)"
EXEC="/opt/multiview/$ARCH/bin/multiview-cli"

cp "$EXEC" "$TMPD/"

"$TMPD/multiview-cli" "$@"

