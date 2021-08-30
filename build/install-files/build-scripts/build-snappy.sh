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

# git clone -b 1.1.7 https://github.com/google/snappy.git
FILE="snappy_1.1.7.tar.gz"
HASH=5a5e36fcd7ad947064f781dedcff9abc

cd $TMPD
get_install_file "$FILE" "$HASH"
cat $FILE | gzip -dc | tar -xf -
cd snappy_1.1.7

mkdir build
cd build

$CMAKE -D CMAKE_C_COMPILER=$CC \
      -D CMAKE_CXX_COMPILER=$CXX \
      -D CMAKE_CXX_FLAGS="-std=c++98 $CXXFLAGS" \
      -D CMAKE_SHARED_LINKER_FLAGS="$LDFLAGS $LIBS" \
      -D BUILD_SHARED_LIBS=On \
      -D CMAKE_BUILD_TYPE=Release \
      -D CMAKE_PREFIX_PATH=$PREFIX \
      -D CMAKE_INSTALL_PREFIX:PATH=$PREFIX \
      ..

make -j$(nproc)
sudo make install
sudo ldconfig

exit $?
