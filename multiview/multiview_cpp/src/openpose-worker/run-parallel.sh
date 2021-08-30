#!/bin/bash

set -e

cd "$(dirname "$0")"

make -j8

make test &
#make test &
#make test &

wait

