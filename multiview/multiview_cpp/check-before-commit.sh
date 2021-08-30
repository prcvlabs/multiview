#!/bin/bash

set -e

cd "$(dirname "$0")"

IS_DOCKER=0
[ -f "/.dockerenv" ] && IS_DOCKER=1

echo "make sure the pipeline (ie cli) runs"
nice ionice -c3 ./run.sh release pipeline --help

echo "release test cuda"
nice ionice -c3 ./run.sh release test cuda

echo "asan test cuda"
nice ionice -c3 ./run.sh asan test cuda

echo "asan test no-cuda"
nice ionice -c3 ./run.sh asan test no-cuda

echo "release cuda build"
nice ionice -c3 ./run.sh release cuda build

echo "release nocuda build"
nice ionice -c3 ./run.sh release no-cuda build

if ! xset q &>/dev/null; then
    echo "No X server at \$DISPLAY [$DISPLAY]"
    echo "SKIPPING gui build tests"
else
    echo "release gui cuda build"
    nice ionice -c3 ./run.sh release gui cuda build

    echo "release gui no-cuda build"
    nice ionice -c3 ./run.sh release gui no-cuda build
fi

echo
echo "Success"
echo

