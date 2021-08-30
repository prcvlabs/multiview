#!/bin/bash

set -e

PPWD="$(cd "$(dirname "$0")" ; pwd)"
source "$PPWD/env.sh"

# Obviously CC, etc. are not going to work before they're built!
unset CC
unset CXX
unset CFLAGS
unset CXXFLAGS
unset LDFLAGS
unset LIBS

ROOT="$CC_ROOT"

NO_CLEANUP=0
if [ "$NO_CLEANUP" = "1" ] ; then
    TMPD=$HOME/TMP/build/clang-gcc-boost
    mkdir -p $TMPD
else
    TMPD=$(mktemp -d /tmp/$(basename $0).XXXXXX)
fi

trap cleanup EXIT
cleanup()
{
    if [ "$NO_CLEANUP" != "1" ] || [ "$DOCKER_MODE" = "1" ] ; then
        rm -rf $TMPD
        rm -f $HOME/user-config.jam
    fi    
}

# ------------------------------------------------------------------------ clang

build_clang()
{
    CLANG_V="$1"
    TAG="$2"
    LLVM_DIR="llvm"

    SRC_D=$TMPD/$LLVM_DIR
    BUILD_D="$TMPD/build-llvm-${TAG}"

    rm -rf "$BUILD_D"
    mkdir -p "$SRC_D"
    mkdir -p "$BUILD_D"

    cd "$SRC_D"

    ! [ -d "llvm-project" ] &&
        git clone https://github.com/llvm/llvm-project.git
    cd llvm-project
    
    git checkout "$CLANG_V"

    cd "$BUILD_D"

    # Using lto=thin is a wee bit slow... linking takes a few hours
    #          -DLLVM_ENABLE_LTO=thin
    
    nice ionice -c3 cmake -G "Ninja" \
         -DLLVM_ENABLE_PROJECTS="clang;clang-tools-extra;libcxx;libcxxabi;libunwind;compiler-rt;lld;polly;lldb" \
         -DCMAKE_BUILD_TYPE=release \
         -DCMAKE_C_COMPILER=clang-6.0 \
         -DCMAKE_CXX_COMPILER=clang++-6.0 \
         -DLLVM_ENABLE_ASSERTIONS=Off \
         -DLIBCXX_ENABLE_STATIC_ABI_LIBRARY=Yes \
         -DLIBCXX_ENABLE_SHARED=YES \
         -DLIBCXX_ENABLE_STATIC=YES \
         -DLIBCXX_ENABLE_FILESYSTEM=YES \
         -DLIBCXX_ENABLE_EXPERIMENTAL_LIBRARY=YES \
         -DLLVM_USE_LINKER=lld-6.0 \
         -DLLVM_BUILD_LLVM_DYLIB=YES \
         -DPYTHON_EXECUTABLE=/usr/bin/python3.6m \
         -DPYTHON_LIBRARY=/usr/lib/python3.6/config-3.6m-x86_64-linux-gnu/libpython3.6m.so \
         -DPYTHON_INCLUDE_DIR=/usr/include/python3.6m \
         -DCURSES_LIBRARY=/usr/lib/x86_64-linux-gnu/libncurses.a \
         -DCURSES_INCLUDE_PATH=/usr/include/ \
         -DCMAKE_INSTALL_PREFIX:PATH=${CC_ROOT} \
         $SRC_D/llvm-project/llvm

    /usr/bin/time -v nice ionice -c3 ninja 2>$BUILD_D/stderr.text | tee $BUILD_D/stdout.text
    ninja install | tee -a $BUILD_D/stdout.text
    cat $BUILD_D/stderr.text   
}

# ------------------------------------------------------------------------ build

VERSION="$1"

if [ "$VERSION" = "llvm-11.0.0" ] || [ "$VERSION" = "clang-11.0.0" ] ; then
    build_clang llvmorg-11.0.0-rc5 11.0.0
elif [ "$VERSION" = "llvm-9.0.1" ] || [ "$VERSION" = "clang-9.0.1" ] ; then
    build_clang llvmorg-9.0.1
else
    echo "Unexpected version: '$1'"
    exit 1
fi

exit 0
