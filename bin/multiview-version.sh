#!/bin/bash

cd "$(dirname "$0")"
echo "$(git describe --tags | head -n 1) $(git log | grep commit | head -n 1)" | sed 's, ,:,g'

