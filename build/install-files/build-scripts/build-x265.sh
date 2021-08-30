#!/bin/bash

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

# hg clone http://hg.videolan.org/x265
# cd x265
# hg update -r 2.9

FILE=x265_v2.9.tar.gz
HASH=2f216dca3861adc4f98faf1f3bc9efe7

cd $TMPD
get_install_file "$FILE" "$HASH"
cat $FILE | gzip -dc | tar -xf -
cd x265_v2.9

mkdir build-dir
cd build-dir

$CMAKE -D CMAKE_C_COMPILER=$CC \
      -D CMAKE_CXX_COMPILER=$CXX \
      -D CMAKE_CXX_FLAGS="$CXXFLAGS" \
      -D CMAKE_SHARED_LINKER_FLAGS="$LDFLAGS $LIBS" \
      -D BUILD_SHARED_LIBS=On \
      -D CMAKE_BUILD_TYPE=Release \
      -D CMAKE_PREFIX_PATH=$PREFIX \
      -D CMAKE_INSTALL_PREFIX:PATH=$PREFIX \
      ../source

make -j$(nproc)
sudo make install
sudo ldconfig

