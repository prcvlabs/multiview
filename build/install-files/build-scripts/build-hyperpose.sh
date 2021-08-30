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

build_install_hyperpose()
{
    # git clone -b 2.1.0  https://github.com/tensorlayer/hyperpose.git
    # local FILE=hyperpose.tar.bz2
    # local HASH=947fa17bba812299b9c64f4cf6d21e91

    # cd $TMPD

    # get_install_file "$FILE" "$HASH"
    # cat "$FILE" | bzip2 -dc | tar -xf -

    git clone https://github.com/tensorlayer/hyperpose.git
    
    mkdir build
    cd build

    $CMAKE -D CMAKE_C_COMPILER=$CC \
           -D CMAKE_CXX_COMPILER=$CXX \
           -D CMAKE_CXX_FLAGS="$CXXFLAGS" \
           -D CMAKE_SHARED_LINKER_FLAGS="$LDFLAGS $LIBS" \
           -D BUILD_SHARED_LIBS=On \
           -D CMAKE_BUILD_TYPE=Release \
           -D CMAKE_PREFIX_PATH=$PREFIX \
           -D CMAKE_INSTALL_PREFIX:PATH=$PREFIX \
           ../hyperpose

    make -j$(nproc)

    sudo cp -f hyperpose-cli $PREFIX/bin/
    sudo cp -f libhelpers.so $PREFIX/lib/
    sudo cp -f libhyperpose.so.2.0 $PREFIX/lib/
    sudo cp -f libstdtracer.so $PREFIX/lib/

    sudo rm -rf $PREFIX/include/hyperpose
    sudo cp -a ../hyperpose/include/hyperpose $PREFIX/include/

    cd $PREFIX/lib/
    sudo rm -f libhyperpose.so
    sudo ln -s libhyperpose.so.2.0 libhyperpose.so

    # TODO: download the models
    sudo mkdir -p $PREFIX/opt/hyperpose
    cd $PREFIX/opt/hyperpose
}

download_models()
{
    local FILE=hyperpose-models.tar.bz2
    local HASH=ac13e4b62e7733198a15eacb8e448501

    cd "$TMPD"
    get_install_file "$FILE" "$HASH"
    cat "$FILE" | bzip2 -dc | tar -xf -

    DESTD="$PREFIX/opt/hyperpose/models"
    echo "Installing models to '$DESTD'"
    rm -rf "$DESTD"
    mkdir -p $PREFIX/opt/hyperpose
    mv models "$DESTD"
}

build_install_hyperpose
download_models

