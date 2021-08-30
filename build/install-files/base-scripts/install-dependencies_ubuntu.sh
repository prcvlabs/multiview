#!/bin/bash

set -e

DISTRIB_RELEASE=$(cat /etc/lsb-release | grep DISTRIB_RELEASE | awk -F= '{ print $2 }')
if [ "$DISTRIB_RELEASE" = "16.04" ] ; then
    echo "No longer suppoer 16.04"
    exit 1
elif [ "$DISTRIB_RELEASE" = "18.04" ] ; then
    true
elif [ "$DISTRIB_RELEASE" = "20.04" ] ; then
    true
else
    echo "Unsupported distro: '$DISTRIB_RELEASE'"
    exit 1
fi

sudo apt-get -y update
sudo apt-get -y upgrade 
sudo apt-get -y dist-upgrade 
sudo apt-get -y install \
     ant \
     automake \
     bison \
     build-essential \
     checkinstall \
     clang-6.0 \
     clang-format-6.0 \
     cmake \
     curl \
     doxygen \
     ffmpeg \
     flex \
     freeglut3-dev \
     git \
     gnuplot-nox \
     graphviz \
     gzip \
     htop \
     libatlas-base-dev \
     libbz2-dev \
     libcurl4-gnutls-dev \
     libdispatch-dev \
     libedit-dev \
     libeigen3-dev \
     libelf-dev \
     libfontconfig1-dev \
     libfreetype6-dev \
     libgcrypt20-dev \
     libglew-dev \
     libgoogle-glog-dev \
     libhdf5-dev \
     liblapack-dev \
     liblapacke-dev \
     liblmdb-dev \
     libmp3lame-dev \
     libncurses5-dev \
     libopenblas-dev \
     libopenjp2-7-dev \
     libopus-dev \
     libparted-dev \
     libprotobuf-dev \
     libpulse-dev \
     libreadline-dev \
     librsvg2-dev \
     librtmp-dev \
     libsdl2-dev \
     libsdl2-ttf-dev \
     libsoxr-dev \
     libspeex-dev \
     libssh-dev \
     libssl-dev \
     libtheora-dev \
     libtool \
     libunwind-dev \
     libv4l-dev \
     libwavpack-dev \
     libx264-dev \
     libxapian-dev \
     libxvidcore-dev \
     lld-6.0 \
     lzma-dev \
     mercurial \
     nano \
     ninja-build \
     pax-utils \
     pkg-config \
     protobuf-compiler \
     python2.7-dev \
     python3-dev \
     python3-lxml \
     python3-pip \
     python3-six \
     python3-tk \
     qtdeclarative5-dev \
     software-properties-common \
     subversion \
     sudo \
     swig \
     time \
     unzip \
     uuid-dev \
     wget \
     yasm \
     zlib1g-dev

sudo apt-get -y purge libgflags-dev

sudo add-apt-repository -y ppa:ubuntu-toolchain-r/test
sudo apt-get -y update
sudo apt-get -y install libstdc++-9-dev
# libstdc++-10-dev THIS WILL BREAK CUDA!!!

# ---- CUDA packages
# sudo apt-get -y install libnvonnxparsers-dev libnvparsers-dev libnvinfer-plugin-dev libnccl2 libnccl-dev libnvinfer-dev uff-converter-tf

sudo apt-get -y update
sudo apt-get -y upgrade
sudo apt-get -y clean all

exit 0

