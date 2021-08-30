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

FILE=icu-release-67-1.tar.gz
HASH=84dabe551d25be8ffa007edcb91ff4ae

cd $TMPD
get_install_file "$FILE" "$HASH"
cat "$FILE" | gzip -dc | tar -xf -
cd icu-release-67-1/icu4c/source

export CXXFLAGS="$CXXSTD $CXXFLAGS"

./configure --prefix=$PREFIX --enable-release --enable-shared --enable-static

make -j$(nproc)
make install

exit $?

