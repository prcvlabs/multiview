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

# git clone -b v324 https://github.com/cjlin1/libsvm.git
FILE=svm-train-gpu.tar.bz2
HASH=c96d5cbc9a746a7dac65927d7b6f322e

cd $TMPD
get_install_file "$FILE" "$HASH"
cat "$FILE" | bzip2 -dc | tar -xf -
cd svm-train-gpu

unset CC
unset CXX
unset CFLAGS
unset CXXFLAGS
unset LDFLAGS
unset LIBS

make
sudo mkdir -p "$PREFIX/bin"

echo "cp svm-train-gpu $PREFIX/bin/"
sudo cp svm-train-gpu $PREFIX/bin/

sudo rm -rf $PREFIX/bin/svm-train-tools
sudo mkdir -p $PREFIX/bin/svm-train-tools
sudo cp -a tools/* $PREFIX/bin/svm-train-tools/

exit $?

