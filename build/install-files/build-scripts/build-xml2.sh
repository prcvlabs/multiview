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


# git clone -b v2.9.8 https://gitlab.gnome.org/GNOME/libxml2.git
FILE="libxml2_v2.9.8.tar.gz"
HASH=d6d70c71b9aa3cdecbd7c275ff2c5d15

cd $TMPD
get_install_file "$FILE" "$HASH"
cat $FILE | gzip -dc | tar -xf -
cd libxml2_v2.9.8

unset LIBS
unset LDFLAGS
unset CFLAGS
unset CXXFLAGS

./autogen.sh --prefix=$PREFIX --with-pic=yes
make -j$(nproc)
sudo make install
sudo ldconfig
