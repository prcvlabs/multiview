#!/bin/bash

# Bash will exit on first error
set -e

PPWD="$(cd "$(dirname "$0")" ; pwd)"
source "$PPWD/install-files/base-scripts/env.sh"

GSUTIL="/snap/bin/gsutil"

# -------------------------------------------------------------------- show-help

show_help()
{
    cat <<EOF

   Usage: $(basename $0) <tag>

      Builds the docker container for multiview on the local machine.

   Examples:

      # Publish tagged release 4.3.0
      ~ > $(basename $0) 4.3.0

EOF
}

for ARG in "$@" ; do
    [ "$ARG" = "--help" ] || [ "$ARG" = "-h" ] && show_help && exit 0
done

# ------------------------------------------------------------------------ Parse

TAG="$1"

# ----------------------------------------------------------------------- Sanity

if [[ "$EUID" -eq 0 ]]; then
    echo "Cannot be root, aborting."
    exit 1
fi

if [ ! -x "$GSUTIL" ] ; then
    echo "Cannot find $GSUTIL!"
    exit 1
fi

if [ "$TAG" = "" ] ; then
    echo "Exepcted a tag, aborting."
    exit 1
fi

# -------------------------------------------------------- Create temp directory

TMPD="$(mktemp -d /tmp/$(basename $0).XXXXXX)"
trap cleanup EXIT
cleanup()
{
    rm -rf "$TMPD"
}

MULTIVIEW_DIR="$TMPD/multiview/multiview/multiview_cpp"

# ----------------------------------------------------------- Checkout multiview

checkout_multiview()
{
    cd "$TMPD"
    git clone -b "$TAG" git@github.com:perceiveinc/multiview.git
}

# -------------------------------------------------------------- Build Multiview

get_version()
{
    cd "$PPWD"
    git describe --tags | head -n 1
}

build_multiview()
{
    checkout_multiview
    rm -rf /tmp/build-${USER}
    cd "$MULTIVIEW_DIR"
    ./run.sh release lto build
}

find_target()
{
    cd "$MULTIVIEW_DIR"
    FILE="$(./run.sh release lto info | grep TARGET | grep build | awk '{ print $4 }')"
    echo "$MULTIVIEW_DIR/$FILE"
}

list_manifest()
{
    # Shared library dependencies
    lddtree "$(find_target)" | awk '{ print $3 }' | grep -E '^/opt' | sort | uniq

    # for `env.sh`
    find /opt/multiview/etc -type f

    # hyperpose models
    find "$PREFIX/opt/hyperpose/models" -type f | sort
    
    # openpose models
    find "$PREFIX/opt/openpose/models" -type f | sort
}

target_archive_name()
{
    echo "multiview-cli_archive_$(get_version).tar.bz2"
}

create_archive()
{
    build_multiview
    local TARGET=$(find_target)

    if [ ! -x "$TARGET" ] ; then
        echo "Failed to find target '$TARGET'"
        exit 1
    fi

    list_manifest | while read L ; do
        D="$TMPD$(dirname "$L")"
        mkdir -p "$D"
        cp "$L" "$D/"
    done

    mkdir -p "$TMPD$PREFIX/bin/"
    cp "$TARGET" "$TMPD$PREFIX/bin/"

    cd "$TMPD"
    local TARGETF="$(target_archive_name)"
    echo "Creating '$TARGETF'"
    tar -c opt | bzip2 -c9 > "$TARGETF"
}

create_archive

echo "Uploading to 'gs://multiview-deploy/'"
$GSUTIL cp "$(target_archive_name)" gs://multiview-deploy/

