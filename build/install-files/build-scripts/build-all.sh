#!/bin/bash

# Bash will exit on first error
set -e

PPWD="$(cd "$(dirname "$0")" ; pwd)"

# ----------------------------------------------------------------- Must be root

if [[ "$EUID" -ne 0 ]]; then
    echo "Must be root, aborting."
    exit 1
fi

# $PPWD/build-imgui.sh

$PPWD/build-eigen.sh
$PPWD/build-icuuc.sh
$PPWD/build-boost.sh
$PPWD/build-assimp.sh
$PPWD/build-awscpp.sh
$PPWD/build-libsvm.sh
$PPWD/build-svm-train-gpu.sh
$PPWD/build-tbb.sh
$PPWD/build-xml2.sh
$PPWD/build-gme.sh
$PPWD/build-modplug.sh
$PPWD/build-snappy.sh
$PPWD/build-x265.sh
$PPWD/build-glog.sh
$PPWD/build-gflags.sh
$PPWD/build-ffmpeg.sh        

$PPWD/patch-cuda-install.sh
$PPWD/build-opencv.sh
$PPWD/build-caffe.sh
$PPWD/build-openpose.sh
$PPWD/build-hyperpose.sh


