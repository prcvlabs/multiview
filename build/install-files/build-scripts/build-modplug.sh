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

# wget http://sourceforge.net/projects/modplug-xmms/files/libmodplug/0.8.8.5/libmodplug-0.8.8.5.tar.gz/download

FILE=libmodplug-0.8.8.5.tar.gz
HASH=5f30241db109d647781b784e62ddfaa1

cd $TMPD
get_install_file "$FILE" "$HASH"
cat $FILE | gzip -dc | tar -xf -
cd libmodplug-0.8.8.5

# Remove LTO
export CFLAGS="$(echo "$CFLAGS" | sed 's,-flto=thin,,g')"
export CXXFLAGS="$(echo "$CXXFLAGS" | sed 's,-flto=thin,,g')"
export LDFLAGS="$(echo "$LDFLAGS" | sed 's,-flto=thin,,g') $LIBS"

./configure --prefix=$PREFIX 
make -j$(nproc)
sudo make install
sudo ldconfig

