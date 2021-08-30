#!/bin/bash

# Bash will exit on first error
set -e

PPWD="$(cd "$(dirname "$0")" ; pwd)"

source "$PPWD/../base-scripts/env.sh"

# ----------------------------------------------------------------- Must be root

if [[ "$EUID" -ne 0 ]]; then
    echo "Must be root, aborting."
    exit 1
fi

# ------------------------------------------------------------ Working directory

TMPD=$(mktemp -d /tmp/$(basename $0).XXXXX)

trap cleanup EXIT
cleanup()
{
    rm -rf $TMPD
}

build_fmt()
{

    FILE=fmt.tar.bz2
    HASH=41864331399473c2778548cb67db028d

    cd $TMPD
    get_install_file "$FILE" "$HASH"
    cat "$FILE" | bzip2 -dc | tar -xf -
    ./build.sh
}   


