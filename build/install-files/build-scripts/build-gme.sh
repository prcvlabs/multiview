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

# git clone https://github.com/mcfiredrill/libgme.git
FILE=libgme.tar.gz
HASH=1129f43bf9dd894b00a99f2ab429ffc1

cd $TMPD
get_install_file "$FILE" "$HASH"
cat $FILE | gzip -dc | tar -xf -

cd libgme
mkdir build
cd build

$CMAKE -D CMAKE_C_COMPILER=$CC \
      -D CMAKE_CXX_COMPILER=$CXX \
      -D CMAKE_CXX_FLAGS="-std=c++98 $CXXFLAGS" \
      -D CMAKE_SHARED_LINKER_FLAGS="$LDFLAGS $LIBS" \
      -D CMAKE_BUILD_TYPE=Release \
      -D CMAKE_PREFIX_PATH=$PREFIX \
      -D CMAKE_INSTALL_PREFIX:PATH=$PREFIX \
      ..

make -j$(nproc)
sudo make install
sudo ldconfig

exit $?

