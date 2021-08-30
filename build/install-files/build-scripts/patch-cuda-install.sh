#!/bin/bash

# CUDA
# Patch CUDA so that we can use clang 11
FILE=/usr/local/cuda/include/crt/host_config.h
BAK_FILE=/usr/local/cuda/include/crt/host_config.h.bak
if [ ! -f "$BAK_FILE" ] ; then
    cp $FILE $BAK_FILE
fi
cat $BAK_FILE | sed 's,__clang_major__ >= [0-9]\+,__clang_major__ >= 20,' > $FILE

