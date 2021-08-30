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

# git clone -b 1.7.21 https://github.com/aws/aws-sdk-cpp.git
FILE="aws-sdk-cpp_1.8.44.tar.bz2"
HASH=c7128afbcd2747eb96e5e5e9865b65df

cd $TMPD
get_install_file "$FILE" "$HASH"
cat "$FILE" | bzip2 -dc | tar -xf -
mkdir build
cd build

sudo mkdir -p $PREFIX/include/aws/checksums

sudo $CMAKE -D CMAKE_C_COMPILER=$CC \
      -D CMAKE_CXX_COMPILER=$CXX \
      -D CMAKE_CXX_FLAGS="$CXXFLAGS" \
      -D CMAKE_SHARED_LINKER_FLAGS="$LDFLAGS $LIBS" \
      -D CMAKE_EXE_LINKER_FLAGS="$LDFLAGS $LIBS" \
      -D BUILD_ONLY="s3" \
      -D ENABLE_UNITY_BUILD=ON \
      -D BUILD_SHARED_LIBS=ON \
      -D ENABLE_TESTING=OFF \
      -D CMAKE_BUILD_TYPE=Release \
      -D CMAKE_PREFIX_PATH=$PREFIX \
      -D CMAKE_INSTALL_PREFIX:PATH=$PREFIX \
      ../aws-sdk-cpp

sudo make -j$(nproc)
sudo make install
sudo ldconfig

