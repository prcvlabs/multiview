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
FILE=libsvm.tar.bz2
HASH=429f0393926da88b07467281eae378f9

cd $TMPD
get_install_file "$FILE" "$HASH"
cat "$FILE" | bzip2 -dc | tar -xf -
cd libsvm

export CXXFLAGS="$(echo "$CXXFLAGS" | sed 's,-flto=thin,,g')"

make lib
make

sudo mkdir -p "$PREFIX/lib"
sudo mkdir -p "$PREFIX/bin"


sudo cp -f libsvm.so.2 $PREFIX/lib/
sudo cp -f svm-predict $PREFIX/bin/
sudo cp -f svm-scale   $PREFIX/bin/
sudo cp -f svm-train   $PREFIX/bin/

sudo cp -f svm.h       $PREFIX/include

cd $PREFIX/lib
sudo rm -f libsvm.so
sudo ln -s libsvm.so.2 libsvm.so

exit $?

