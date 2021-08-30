#!/bin/dash

cd "$(dirname "$0")"

# ------------------------------------------------------------ Parse Commandline

CLEAN=
VERBOSE=
TARGET=alglib.a
CONFIG=release
TOOLCHAIN=clang-5.0
REGENERATE=0
ANALYZE=
FEEDBACK=-a

while [ "$#" -gt "0" ] ; do
    
    [ "$1" = "gcc" ]       && TOOLCHAIN="gcc-7" && shift && continue
    [ "$1" = "gcc-5" ]     && TOOLCHAIN="gcc-5" && shift && continue
    [ "$1" = "gcc-7" ]     && TOOLCHAIN="gcc-7" && shift && continue
    [ "$1" = "clang" ]     && TOOLCHAIN="clang-5.0" && shift && continue
    [ "$1" = "clang-5" ]   && TOOLCHAIN="clang-5.0" && shift && continue
    [ "$1" = "clang-5.0" ] && TOOLCHAIN="clang-5.0" && shift && continue
    [ "$1" = "clang-lto" ] && TOOLCHAIN="clang-5.0-lto" && shift && continue

    [ "$1" = "debug" ]     && CONFIG=debug && shift && continue
    [ "$1" = "release" ]   && CONFIG=release && shift && continue
    [ "$1" = "asan" ]      && CONFIG=asan && shift && continue
    [ "$1" = "usan" ]      && CONFIG=usan && shift && continue
    [ "$1" = "tsan" ]      && CONFIG=tsan && shift && continue
    [ "$1" = "prof" ]      && CONFIG=prof && shift && continue
    
    [ "$1" = "clean" ]     && CLEAN="-t clean" && shift && continue
    [ "$1" = "verbose" ]   && VERBOSE="-v" && shift && continue
    [ "$1" = "-v" ]        && VERBOSE="-v" && shift && continue
    [ "$1" = "regenerate" ] \
        || [ "$1" = "-r" ] && REGENERATE="1" && shift && continue
    [ "$1" = "analyze" ]   && ANALYZE="scanbuild" && shift && continue
    [ "$1" = "nf" ]        && FEEDBACK="" && shift && continue
    

    shift
    
done

MGN="./ninja-files/machine-generated-ninja"
mkdir -p "$MGN"
NINJA_F="$TARGET-$TOOLCHAIN-$CONFIG.ninja"

[ "$REGENERATE" = "1" ] || [ ! -f "$MGN/$NINJA_F" ] \
    && ./ninja-files/regenerate.sh

[ ! -f "$MGN/$NINJA_F" ] \
    && echo "Failed to find ninja file file '$NINJA_F'" \
    && exit 1

../../multiview_cpp/ninja-files/ninja $FEEDBACK -f "$MGN/$NINJA_F" $VERBOSE $CLEAN $ANALYZE

