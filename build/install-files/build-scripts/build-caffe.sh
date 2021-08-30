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

FILE=caffe.tar.bz2
HASH=e713cf7e9a0fecbf6e55e1bfd7d2703c

cd $TMPD
get_install_file "$FILE" "$HASH"
cat "$FILE" | bzip2 -dc | tar -xf -
cd caffe

OLD_BINS="set(Caffe_known_gpu_archs \".*\")"
BINS="$(echo $CUDA_ARCH_BINS | sed 's,\.,,g')"
FILE=cmake/Cuda.cmake
ls "$FILE"
if [ ! -f "$FILE.bak" ] ; then
    cp -f "$FILE" "$FILE.bak"
fi
cat "$FILE.bak" \
    | sed "s,$OLD_BINS,set(Caffe_known_gpu_archs \"$BINS\")," > "$FILE"

cat > "include/caffe/util/private-opencv-inc.hpp" <<EOF
#pragma once
#ifdef USE_OPENCV
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#if(CV_MAJOR_VERSION == 4)
#define CV_LOAD_IMAGE_COLOR cv::IMREAD_COLOR
#define CV_LOAD_IMAGE_GRAYSCALE cv::IMREAD_GRAYSCALE
#endif
#endif
EOF

fgrep --recursive -n "opencv2/core/core.hpp" \
    | awk -F: '{ print $1 }' | while read F ; do
    echo "Patching '$F'"
    sed -i 's,"opencv2/core/core.hpp","caffe/util/private-opencv-inc.hpp",' $F
    sed -i 's,<opencv2/core/core.hpp>,"caffe/util/private-opencv-inc.hpp",' $F
done

mkdir build
cd build

sudo apt-get -y remove libcudnn8

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
       -D USE_CUDNN=On \
       -D CPU_ONLY=Off \
       -D USE_NCCL=On \
       -D BUILD_docs=Off \
       -D BLAS=Open \
       -D CMAKE_BUILD_TYPE=Release \
       -D CMAKE_PREFIX_PATH=$PREFIX \
       -D CMAKE_INSTALL_PREFIX:PATH=$PREFIX \
       -D CUDA_ARCH_NAME=All \
       ..

make -j$(nproc)
sudo make install
sudo ldconfig

sudo apt-get -y install libnvonnxparsers-dev libnvparsers-dev libnvinfer-plugin-dev libnccl2 libnccl-dev libnvinfer-dev uff-converter-tf

exit $?



