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

FILE=eigen-3.3.7.tar.bz2
HASH=ed5243b8e4cf1bd47f0e0079896fa739

cd $TMPD
get_install_file "$FILE" "$HASH"
cat "$FILE" | bzip2 -dc | tar -xf -

rm -rf "$PREFIX/include/Eigen"
mkdir -p "$PREFIX/include"
cp -a eigen-3.3.7/Eigen $PREFIX/include/Eigen


   
