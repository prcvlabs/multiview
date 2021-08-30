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

# git clone -b v4.1.0 https://github.com/assimp/assimp.git
FILE=assimp-v4.1.0.tar.gz
HASH=4cb0e175ba874d0aa700bba9ca3d40a1

cd $TMPD
get_install_file "$FILE" "$HASH"
cat "$FILE" | gzip -dc | tar -xf -
cd assimp-v4.1.0
mkdir build
cd build

$CMAKE -D CMAKE_C_COMPILER=$CC \
      -D CMAKE_CXX_COMPILER=$CXX \
      -D CMAKE_CXX_FLAGS="$CXXFLAGS $LDFLAGS $LIBS -Wno-unused-command-line-argument" \
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


