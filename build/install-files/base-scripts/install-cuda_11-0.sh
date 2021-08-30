#!/bin/bash

set -e

if [ -f "/.dockerenv" ] ; then
    exit 0
fi

S3_REPO="s3://perceive-multiview/install-files/ubuntu-18.04"

mkdir -p /tmp/cuda-install-dir
cd /tmp/cuda-install-dir

get_it()
{
    if [ ! -f "$1" ] ; then
        aws s3 cp "$S3_REPO/$1" /tmp/cuda-install-dir/
    fi
}

other_debs()
{
cat <<EOF
libcudnn8_8.0.3.33-1+cuda11.0_amd64.deb
libcudnn8-dev_8.0.3.33-1+cuda11.0_amd64.deb
libcudnn8-samples_8.0.3.33-1+cuda11.0_amd64.deb
nv-tensorrt-repo-ubuntu1804-cuda11.0-trt7.1.3.4-ga-20200617_1-1_amd64.deb
EOF
}

# Get the deb files
other_debs | while read F ; do
    echo "$F"
    get_it "$F"
done

other_debs | while read F ; do
    echo "installing '$F'"
    sudo dpkg -i "$F"
done

sudo apt-get update

sudo apt-key add /var/nv-tensorrt-repo-cuda11.0-trt7.1.3.4-ga-20200617/7fa2af80.pub

sudo apt-get update

sudo apt-get -y install libcudnn8 libcudnn8-dev 

# sudo apt remove libcudnn8-dev
sudo apt-get -y install libnvonnxparsers-dev libnvparsers-dev libnvinfer-plugin-dev libnccl2 libnccl-dev libnvinfer-dev uff-converter-tf
