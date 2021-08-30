#!/bin/bash

# Bash will exit on first error
set -e

PPWD="$(cd "$(dirname "$0")" ; pwd)"

VERSION=clang-11.0.0

BUILD_CC=0
INSTALL_CUDA=0
FORCE_CUDA=0
INSTALL_DEBS=1
DEB_STOP=0
BUILD_LIBS=1
PRINT_ONLY=0

DOCKER_MODE=0

# ------------------------------------------------------------------------ Usage

show_help()
{
cat <<EOF

   Usage: $(basename $0) [OPTIONS...]

      Builds and installs dependencies for "multiview". Non C++ dependencies
      are installed to the usual places. All C++ dependencies are built and
      installed somewhere under "/opt/llvm/.../".

   Options:

      --print/-p              Dry run print and exit

      --arch [clang-11.0.0]   Set llvm (clang) version. Default is $VERSION.

      --packages-only         Stop installation after install base packages.
      --no-install-packages   Do not install apt packages. (Assumes installed.)

      --force-cuda-install    Even if it appears that cuda is installed.
      --force-build-compiler  Build compiler (clang) even if it exists.

      --no-build-libs         Do not build supporting libraries.

EOF
}

# -------------------------------------------------- Parse Command-line (if any)

for ARG in "$@" ; do
    [ "$ARG" = "-h" ] || [ "$ARG" = "--help" ] && show_help && exit 0
done

while [ "$#" -gt "0" ] ; do
    ARG="$1"

    [ "$ARG" = "-p" ] || [ "$ARG" = "--print" ] && shift && PRINT_ONLY=1 && continue
    
    [ "$ARG" = "--arch" ] && shift && VERSION="$1" && shift && continue

    [ "$ARG" = "--packages-only" ] && shift && INSTALL_DEBS=1 && DEB_STOP=1 && continue
    [ "$ARG" = "--no-install-packages" ] && shift && INSTALL_DEBS=0 && continue
    [ "$ARG" = "--force-build-compiler" ] && shift && BUILD_CC=1 && continue
    [ "$ARG" = "--no-build-libs" ] && shift && BUILD_LIBS=0 && continue
    [ "$ARG" = "--force-cuda-install" ] && shift && FORCE_CUDA=1 && continue

    echo "Unknown command-line argument: '$ARG', aborting" \
        && exit 1
done

# ----------------------------------------------------------------- Must be root

if [[ "$EUID" -ne 0 ]]; then
    echo "Must be root, aborting."
    exit 1
fi

# ------------------------------------------------------------- Docker bootstrap

if [ ! -x "/usr/bin/sudo" ] ; then
    if [[ "$EUID" -ne 0 ]]; then
        echo "failed to find '/usr/bin/sudo', aborting"
        exit 1
    fi
    apt-get update
    apt-get -y install rlwrap apt-utils
    apt-get -y upgrade
    apt-get -y dist-upgrade    
    apt-get -y install sudo lsb-release
    apt-get -y clean all    
fi

# ------------------------------------------------ Was the NVIDIA driver loaded?

if [ -x /usr/bin/nvidia-smi ] || [ -f "/.dockerenv" ] ; then
    # All is good
    true
else
    echo "Doesn't look like the NVIDIA driver was loaded, aborting"
    exit 1
fi

# ------------------------------------------------------------ Setup Environment

# We do not want to get stuck configuring a keyboard
export ARCH="${VERSION}"

source "$PPWD/base-scripts/env.sh"

# version is "global" for the install configuration
echo -n "$ARCH" > "$OPT_ROOT/etc/arch"

# Do we have an llvm version?
[ ! -f "$OPT_ROOT/etc/arch" ] \
    && echo "failed to find/create file '$OPT_ROOT/etc/arch'" \
    && exit 1

# Docker-mode implies no cuda install
[ "$DOCKER_MODE" = "1" ] && INSTALL_CUDA=0

# ---------------------------------------------- Are we actually installing CUDA
# Don't install cuda if `/usr/local/cuda` symlink exists
if [ -L "/usr/local/cuda" ] || [ "$INSTALL_DEBS" = "0" ] ; then
    INSTALL_CUDA=0
else
    INSTALL_CUDA=1
fi

# Force a cuda-install, if requested
if [ "$FORCE_CUDA" = "1" ] ; then
    INSTALL_CUDA=1
fi

if [ ! -L "/usr/local/cuda" ] && [ "$INSTALL_CUDA" = "0" ] ; then
    echo "cuda doesn't seem to be installed, and isn't going to be installed!"
    exit 1
fi

# -------------------------------------------- Are we actually installing clang?

if [ ! -x "$CC" ] || [ ! -x "$CXX" ] ; then
    BUILD_CC=1
fi

cat <<EOF

   Build Configuration:

      architecture:      $ARCH

      ubuntu:            $([ "$IS_UBUNTU" = "1" ] && echo -n yes || echo -n no)
      fedora:            $([ "$IS_FEDORA" = "1" ] && echo -n yes || echo -n no)

      install cuda:      $([ "$INSTALL_CUDA" = "1" ] && echo -n yes || echo -n no)
      install packages:  $([ "$INSTALL_DEBS" = "1" ] && echo -n yes || echo -n no)
      build-compiler:    $([ "$BUILD_CC" = "1" ] && echo -n yes || echo -n no)
      build-libs:        $([ "$BUILD_LIBS" = "1" ] && echo -n yes || echo -n no)

      docker-mode:       $([ "$DOCKER_MODE" = "1" ] && echo -n yes || echo -n no)

EOF

# -------------------------------------------------------------------------- Run

if [ "$PRINT_ONLY" = "1" ] ; then
    exit 0
fi


if [ "$INSTALL_DEBS" = "1" ] ; then
    "$PPWD/base-scripts/install-dependencies.sh"
fi

if [ "$INSTALL_CUDA" = "1" ] ; then
    "$PPWD/base-scripts/install-cuda_10-1.sh"
fi

if [ "$DEB_STOP" = "1" ] ; then
    exit 0 ;
fi

# Build CC if required
if [ "$BUILD_CC" = "1" ] ; then
    "$PPWD/base-scripts/build-support.sh" force
    "$PPWD/base-scripts/build-cc.sh" "$VERSION"
    "$PPWD/base-scripts/build-doxygen.sh"
fi

# Build LIBS
if [ "$BUILD_LIBS" = "1" ] ; then
    "$PPWD/build-scripts/build-all.sh"
fi
