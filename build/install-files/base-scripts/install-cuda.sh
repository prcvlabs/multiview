#!/bin/bash

set -e

# Open a terminal window and type the following three commands
# to get rid of any NVIDIA/CUDA packages you may already have installed:
sudo rm -f /etc/apt/sources.list.d/cuda*
sudo apt-get -y remove --autoremove nvidia-cuda-toolkit || true
sudo apt-get -y remove --autoremove nvidia-* || true

# Purge any remaining NVIDIA configuration files and
# the associated dependencies that they may have been installed with.
sudo apt-get -y purge nvidia* || true
sudo apt-get -y autoremove || true
sudo apt-get -y autoclean

# Remove any existing CUDA folders you may have in /usr/local/
sudo rm -rf /usr/local/cuda*

# Setup your CUDA PPA. (Read here if you’re curious about what a PPA is.)
sudo apt-get update
sudo add-apt-repository -y ppa:graphics-drivers
sudo apt-key adv --fetch-keys http://developer.download.nvidia.com/compute/cuda/repos/ubuntu1804/x86_64/7fa2af80.pub
sudo bash -c 'echo "deb http://developer.download.nvidia.com/compute/cuda/repos/ubuntu1804/x86_64 /" > /etc/apt/sources.list.d/cuda.list'
sudo bash -c 'echo "deb http://developer.download.nvidia.com/compute/machine-learning/repos/ubuntu1804/x86_64 /" > /etc/apt/sources.list.d/cuda_learn.list'


# (b) Install CUDA 10.0 and 11.0 packages
# (10.0 required for uff-converter-tf)
sudo apt-get -y update
sudo apt-get -y install cuda-10-0
sudo apt-get -y install cuda-11-1

# Install TensorRt -- requires AWS. (You can get an NVidia
# account and get the files that way.)
cd /tmp
sudo apt-get -y install awscli
aws s3 cp s3://perceive-multiview/install-files/ubuntu-18.04/nv-tensorrt-repo-ubuntu1804-cuda11.0-trt7.1.3.4-ga-20200617_1-1_amd64.deb .
sudo dpkg -i nv-tensorrt-repo-ubuntu1804-cuda11.0-trt7.1.3.4-ga-20200617_1-1_amd64.deb 
sudo apt-get update
sudo apt-get -y install libcudnn8 libcudnn8-dev 
sudo apt-get -y install libnvonnxparsers-dev libnvparsers-dev libnvinfer-plugin-dev libnccl2 libnccl-dev libnvinfer-dev

# Now install `uff-converter-tf`, which has been removed from CUDA 11.0
cd /tmp
aws s3 cp s3://perceive-multiview/install-files/ubuntu-18.04/nv-tensorrt-repo-ubuntu1804-cuda10.0-trt5.0.2.6-ga-20181009_1-1_amd64.deb .
sudo dpkg -i nv-tensorrt-repo-ubuntu1804-cuda10.0-trt5.0.2.6-ga-20181009_1-1_amd64.deb
sudo apt-get update
sudo apt-get -y install uff-converter-tf

# After installing, we need to add CUDA to our PATH, so
# that the shell knows where to find CUDA. To edit our path,
# open up the ‘.profile’ file using vim.
sudo rm -f /usr/local/cuda
sudo ln -s /usr/local/cuda-11.1 /usr/local/cuda
