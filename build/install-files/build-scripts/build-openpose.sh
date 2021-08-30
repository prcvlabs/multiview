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

CLEAN_FILES=1
USE_CUDA=1
while [ "$#" -gt "0" ] ; do
    [ "$1" = "noclean" ] && CLEAN_FILES=0 && shift && continue
    shift
done

TMPD=/tmp/build-${USER}/openpose
if [ "$CLEAN_FILES" = "1" ] ; then
    TMPD=$(mktemp -d /tmp/$(basename $0).XXXXX)
else
    mkdir -p "$TMPD"
fi

trap cleanup EXIT
cleanup()
{
    if [ "$CLEAN_FILES" = "1" ] ; then
        rm -rf $TMPD
    fi
}

cd $TMPD

FILE=openpose-cudnn8.tar.bz2
HASH=fe2837e61b49cbba1d217afb8f163496

get_install_file "$FILE" "$HASH"
cat "$FILE" | bzip2 -dc | tar -xf -

download_models()
{
    OPENPOSE_PREFIX="$PREFIX/opt/openpose"
    mkdir -p "$OPENPOSE_PREFIX"
    cd "$OPENPOSE_PREFIX"
    echo "Downloading latest models"

    local FILE="openpose-models.tar.bz2"
    local HASH=7b7f54880f3db0c2108ec240e89a282d
    get_install_file "$FILE" "$HASH"

    cat openpose-models.tar.bz2 | bzip2 -dc | tar -xf -
    rm -f openpose-models.tar.bz2
}

mkdir $TMPD/build
cd $TMPD/build

# sudo apt-get -y remove libcudnn8

$CMAKE -D CMAKE_C_COMPILER=$CC \
       -D CMAKE_CXX_COMPILER=$CXX \
       -D CMAKE_CXX_FLAGS="$CXXFLAGS $LDFLAGS $LIBS -Wno-unused-value -Wno-unused-function -Wno-unused-const-variable -Wno-unused-command-line-argument" \
       -D Boost_LIBRARIES=$PREFIX/lib \
       -D CMAKE_SHARED_LINKER_FLAGS="$LDFLAGS $LIBS" \
       -D BUILD_SHARED_LIBS=On \
       -D USE_LEVELDB=OFF \
       -D USE_LMDB=OFF \
       -D BUILD_matlab=OFF \
       -D BUILD_python=OFF \
       -D USE_OPENCV=ON \
       -D BLAS=Open \
       -D CUDA_ARCH_NAME=All \
       -D CMAKE_BUILD_TYPE=Release \
       -D BUILD_CAFFE=OFF \
       -D Caffe_INCLUDE_DIRS=$PREFIX/include/caffe \
       -D Caffe_LIBS=$PREFIX/lib/libcaffe.so.1.0.0 \
       -D CMAKE_PREFIX_PATH=$PREFIX \
       -D DOWNLOAD_BODY_25_MODEL=OFF \
       -D DOWNLOAD_BODY_COCO_MODEL=OFF \
       -D DOWNLOAD_BODY_MPI_MODEL=OFF \
       -D DOWNLOAD_FACE_MODEL=OFF \
       -D DOWNLOAD_HAND_MODEL=OFF \
       -D CMAKE_INSTALL_PREFIX:PATH=$PREFIX \
       ../openpose-cudnn8

make -j$(nproc)
sudo make install
sudo ldconfig

sudo apt-get -y install libnvonnxparsers-dev libnvparsers-dev libnvinfer-plugin-dev libnccl2 libnccl-dev libnvinfer-dev uff-converter-tf

download_models

if [ "$CLEAN_FILES" = "1" ] ; then
    sudo rm -rf "$SRCD"
fi

exit $?
