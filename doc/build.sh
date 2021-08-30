#!/bin/bash

set -e

PPWD="$(cd "$(dirname "$0")" ; pwd)"

ARCH_D=$(cat /opt/multiview/etc/arch)
CONFIG_F="doxy.config"
EXEC=/opt/multiview/$ARCH_D/opt/cc/bin/doxygen

# ! [ -x "LD_LIBRARY_PATH=$LLVM_D/lib $EXEC" ]  \
#   && echo "Failed to find $EXEC. Please build with: " \
#   && echo "build/install-files/scripts/build-doxygen.sh" \
#   && exit 1

! [ "$(LD_LIBRARY_PATH=$LLVM_D/lib $EXEC -v)" = "1.8.20" ] \
    && echo "Doxygen is wrong version. Please build with " \
    && echo "$PPWD/../build/install-files/scripts/build-doxygen.sh" \
    && exit 1

TMPD=$(mktemp -d /tmp/$(basename $0).XXXXX)
trap cleanup EXIT
cleanup()
{
    rm -rf $TMPD
}

# Replace PROJECT_NUMBER with git information
TAG="$(git describe --tags | head -n 1)"
COMMIT="$(git log | grep commit | head -n 1)"

cat "$PPWD/$CONFIG_F" | sed "s,<PROJECT_VERSION_NUMBER>,$TAG $COMMIT," > $TMPD/doxy-config

cd $PPWD
LD_LIBRARY_PATH=$LLVM_D/lib $EXEC $TMPD/doxy-config

rsync -ac "$PPWD/html/" "$PERCEIVE_DATA/computer-vision/documentation/multiview/html"

