#!/bin/bash

set -e

PPWD="$(cd "$(dirname "$0")" ; pwd)"
source "$PPWD/../base-scripts/env.sh"

# ----------------------------------------------------------------- Must be root

if [[ "$EUID" -ne 0 ]]; then
    echo "Must be root, aborting."
    exit 1
fi

# Doxygen is a special case... it goes in CC_ROOT/bin
unset CC
unset CXX
unset CFLAGS
unset CXXFLAGS
unset LDFLAGS
unset LIBS

# ------------------------------------------------------------ Working directory

TMPD=$(mktemp -d /tmp/$(basename $0).XXXXXX)
trap cleanup EXIT
cleanup()
{
    rm -rf $TMPD
}

cd $TMPD

VERSION=1.8.20
FILE=doxygen-${VERSION}.src.tar.gz
HASH=8729936a843232a66fe970ef65f3c3e4

get_install_file "$FILE" "$HASH"
cat $FILE | gzip -dc | tar -xf -

mkdir build
cd build

export CMAKE_PREFIX_PATH=$CC_ROOT
$CMAKE -D CMAKE_BUILD_TYPE=Release \
       -D CMAKE_CXX_LINKER_FLAGS="-Wl,-rpath,$CC_ROOT/lib," \
       -D english_only=ON \
       -D build_doc=OFF \
       -D build_wizard=ON \
       -D build_search=ON \
       -D build_xmlparser=ON \
       -D use_libclang=ON \
       -D CMAKE_INSTALL_PREFIX:PATH=$CC_ROOT \
       ../doxygen-${VERSION}

make -j$(nproc)
make install

