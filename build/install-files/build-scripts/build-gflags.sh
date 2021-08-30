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

# git clone -b v2.2.2 https://github.com/gflags/gflags
FILE=gflags.tar.bz2
HASH=0255ddff25fa486512f5428bbf2d288c

cd $TMPD
get_install_file "$FILE" "$HASH"
cat "$FILE" | bzip2 -dc | tar -xf -
mkdir build
cd build

$CMAKE -D CMAKE_C_COMPILER=$CC \
      -D CMAKE_CXX_COMPILER=$CXX \
      -D CMAKE_CXX_FLAGS="$CXXFLAGS" \
      -D CMAKE_SHARED_LINKER_FLAGS="$LDFLAGS $LIBS" \
      -D BUILD_SHARED_LIBS=On \
      -D CMAKE_BUILD_TYPE=Release \
      -D CMAKE_PREFIX_PATH=$PREFIX \
      -D CMAKE_INSTALL_PREFIX:PATH=$PREFIX \
      ../gflags

make -j$(nproc)
sudo make install
