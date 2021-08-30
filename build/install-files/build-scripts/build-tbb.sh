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

# wget https://github.com/01org/tbb/archive/$VERSION.tar.gz
# https://github.com/oneapi-src/oneTBB.git
FILE="oneTBB_2019_U3.tar.gz"
HASH=19b4025e62af16ac217f00cadf3358cc

cd $TMPD
get_install_file "$FILE" "$HASH"
cat $FILE | gzip -dc | tar -xf -
cd oneTBB_2019_U3

# TBB will not link when built with fPIE
export CXX_ONLY_FLAGS="$(echo "$CXXFLAGS" | sed 's,-fPIE,,g')"
export LDFLAGS="$LDFLAGS $LIBS"

make -j$(nproc)

# manually install
sudo cp -a include/tbb include/serial $PREFIX/include/
sudo cp -f build/linux_*_release/lib*.so* $PREFIX/lib

exit $?
