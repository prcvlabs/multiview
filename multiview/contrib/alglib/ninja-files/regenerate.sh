#!/bin/dash

# Cd into the root directory
PPWD="$(cd "$(dirname "$0")" ; pwd)"
OUTD=$PPWD/machine-generated-ninja
MAKE1="$PPWD/../../../multiview_cpp/ninja-files/base/make-1-ninja.sh"

cd "$(dirname "$0")/.."
for TOOLCHAIN in gcc-5 gcc-7 clang-5.0 ; do
    for CONFIG in release debug asan usan prof ; do
        for TARGET in alglib.a ; do
            "$MAKE1" $TOOLCHAIN $CONFIG static products/$TOOLCHAIN-$CONFIG/$TARGET cpp/src > \
                     $OUTD/$TARGET-$TOOLCHAIN-$CONFIG.ninja
        done
    done
done

