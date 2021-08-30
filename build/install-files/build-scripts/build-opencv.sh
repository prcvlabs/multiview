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



# git clone -b v4.1.0 https://github.com/assimp/assimp.git
CV_VERSION=4.4.0
FILE=opencv-${CV_VERSION}.tar.bz2
HASH=88f034e6827c7cd076ab02baf6602748

cd $TMPD
get_install_file "$FILE" "$HASH"
cat "$FILE" | bzip2 -dc | tar -xf -

mkdir build
cd build

# -------------------------------------------------------------------- Run Cmake

CMAKE_CUDA_ARCH="-D CUDA_ARCH_BIN=\"$CUDA_ARCH_BINS\" -D CUDA_ARCH_PTX=\"$CUDA_ARCH_BINS\""
CV_CUDA_ARCH="$(echo "$CMAKE_CUDA_ARCH" | tr ' ' ',' | sed 's/-D,/-D /g' | sed 's/,-D/ -D/g')"

ALL_CUDA_OPTS="-D WITH_CUDA=ON -D CUDA_FAST_MATH=0 $CV_CUDA_ARCH"

$CMAKE \
      -D CMAKE_C_COMPILER=$CC \
      -D CMAKE_CXX_COMPILER=$CXX \
      -D CMAKE_CXX_FLAGS="$CXXFLAGS -Wno-unused-value -Wno-unused-function -Wno-unused-const-variable" \
      -D CMAKE_SHARED_LINKER_FLAGS="$LDFLAGS $LIBS" \
      -D CMAKE_BUILD_TYPE=RELEASE \
      -D CMAKE_INSTALL_PREFIX=$PREFIX \
      -D OPENCV_EXTRA_MODULES_PATH=$TMPD/opencv-${CV_VERSION}/opencv_contrib/modules \
      -D BUILD_opencv_legacy=OFF \
      -D BUILD_opencv_apps=OFF \
      -D WITH_TBB=ON \
      -D WITH_IPP=OFF \
      -D BUILD_NEW_PYTHON_SUPPORT=ON \
      -D WITH_V4L=ON \
      -D INSTALL_C_EXAMPLES=OFF \
      -D INSTALL_PYTHON_EXAMPLES=OFF \
      -D BUILD_EXAMPLES=OFF \
      -D WITH_QT=OFF \
      -D WITH_GTK=OFF \
      -D WITH_OPENGL=OFF \
      -D WITH_PROTOBUF=OFF  \
      -D BUILD_TIFF=OFF \
      -D BUILD_opencv_viz=OFF \
      -D WITH_VTK=OFF \
      -D WITH_EIGEN=ON \
      -D BUILD_SHARED_LIBS=ON \
      -D BUILD_TESTS=ON \
      -D ENABLE_PRECOMPILED_HEADERS=OFF \
      -D BUILD_opencv_cudacodec=ON \
      -D BUILD_opencv_perf_stitching=OFF \
      -D BUILD_opencv_gapi=OFF \
      $ALL_CUDA_OPTS \
      $TMPD/opencv-${CV_VERSION}/opencv

make -j$(nproc)
make -j$(nproc)
sudo make install

