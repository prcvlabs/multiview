#!/bin/bash

set -e

PPWD="$(cd "$(dirname "$0")" ; pwd)"
source "$PPWD/env.sh"

if [ -f "/.dockerenv" ] ; then
    exit 0
fi


CUDA_URL="$INSTALL_FILE_URL/ubuntu-18.04"

mkdir -p /tmp/cuda-install-dir
cd /tmp/cuda-install-dir

get_it()
{
    cd /tmp/cuda-install-dir
    if [ ! -f "$1" ] ; then
        wget -nv "$CUDA_URL/$1"
    fi
}

other_debs()
{
cat <<EOF
libcudnn7_7.6.1.34-1+cuda10.1_amd64.deb
libcudnn7-dev_7.6.1.34-1+cuda10.1_amd64.deb
libcudnn7-doc_7.6.1.34-1+cuda10.1_amd64.deb
nccl-repo-ubuntu1804-2.4.7-ga-cuda10.1_1-1_amd64.deb
nv-tensorrt-repo-ubuntu1804-cuda10.1-trt5.1.5.0-ga-20190427_1-1_amd64.deb
EOF
}

# Get the deb files
get_it cuda-ubuntu1804.pin
get_it cuda-repo-ubuntu1804-10-1-local-10.1.243-418.87.00_1.0-1_amd64.deb
other_debs | while read F ; do get_it "$F" ; done

# Some purging!
sudo dpkg --purge cuda-repo-ubuntu1804
sudo apt-get -y --allow-change-held-packages purge cuda-10-1
sudo dpkg --purge libnvinfer-dev
sudo dpkg --purge libnvinfer5
sudo dpkg --purge libcudnn7-doc
sudo dpkg --purge libcudnn7-dev
sudo dpkg --purge libcudnn7
sudo dpkg --purge libnccl-dev
sudo dpkg --purge libnccl2

# Install cuda base
sudo mv cuda-ubuntu1804.pin /etc/apt/preferences.d/cuda-repository-pin-600
sudo dpkg -i cuda-repo-ubuntu1804-10-1-local-10.1.243-418.87.00_1.0-1_amd64.deb
sudo apt-key add /var/cuda-repo-10-1-local-10.1.243-418.87.00/7fa2af80.pub
sudo apt-get update

sudo apt-get -y install cuda-10-1
# sudo apt-get -y install cuda

# Pin
echo cuda-10-1 hold            | sudo dpkg --set-selections
echo cuda-repo-ubuntu1804 hold | sudo dpkg --set-selections

# Install more cuda libraries
other_debs | while read F ; do
    # Make sure it is installed
    sudo dpkg -i "$F"
done

sudo apt-get -y install libnccl2
sudo apt-get -y install libnccl-dev
sudo apt-get -y install libnvinfer5
sudo apt-get -y install libnvinfer-dev

# More pins
echo libcudnn7 hold            | sudo dpkg --set-selections
echo libcudnn7-dev hold        | sudo dpkg --set-selections
echo libcudnn7-doc hold        | sudo dpkg --set-selections
echo libnccl2 hold             | sudo dpkg --set-selections
echo libnccl-dev hold          | sudo dpkg --set-selections
echo libnvinfer5 hold          | sudo dpkg --set-selections
echo libnvinfer-dev hold       | sudo dpkg --set-selections

# Upgrade should be good
sudo apt-get update
sudo apt-get -y install -f
sudo apt-get -y upgrade

# Ensure /usr/local/cuda is pointing to the right place
sudo rm -f /usr/local/cuda
sudo ln -s /usr/local/cuda-10.1 /usr/local/cuda

