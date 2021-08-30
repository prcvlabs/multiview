#!/bin/bash

set -e

FORCE=0
if [ "$1" = "force" ] ; then
    FORCE=1
fi

PPWD="$(cd "$(dirname "$0")" ; pwd)"
source "$PPWD/env.sh"

# Obviously CC, etc. are not going to work before they're built!
unset CC
unset CXX
unset CFLAGS
unset CXXFLAGS
unset LDFLAGS
unset LIBS

TMPD=$(mktemp -d /tmp/$(basename $0).XXXXXX)
trap cleanup EXIT
cleanup()
{
    rm -rf $TMPD
}

# ------------------------------------------------------------------------ cmake

install_cmake()
{
    # -- The master branch needs a new version of CMAKE.
    cd $TMPD

    # wget https://github.com/Kitware/CMake/releases/download/v3.13.4/cmake-3.13.4.tar.gz
    local FILE="cmake-3.18.1.tar.gz"
    local HASH=bc0d668dba6ec081c664ae95b75540d6
    
    get_install_file "$FILE" "$HASH"
    cat $FILE | gzip -dc | tar -xf -
    
    cd cmake-3.18.1
    ./configure --prefix=$CC_ROOT
    nice ionice -c3 make -j$(expr $(nproc) \* 6 / 5)
    make install
}

# ------------------------------------------------------------------------ ninja

install_ninja()
{
    cd $TMPD

    local FILE=ninja.tar.gz
    local HASH=547a55aa3c0a1889171ab53179bd2650

    get_install_file "$FILE" "$HASH"
    cat $FILE | gzip -dc | tar -xf -
    
    cd ninja
    nice ionice -c3 make -j$(nproc)
    cp ninja $CC_ROOT/bin/
}

# ----------------------------------------------------------------------- mobius

install_mobius()
{
    cd $TMPD

    local FILE=mobius.tar.gz
    local HASH=9ff9437aaa7f3dca3e6abd083aec1d54

    get_install_file "$FILE" "$HASH"
    cat $FILE | gzip -dc | tar -xf -
    
    cd mobius
    nice ionice -c3 make -j$(nproc)
    cp mobius $CC_ROOT/bin/
}

# --------------------------------------------------------------------- valgrind

install_valgrind()
{
    cd $TMPD
    
    local FILE=valgrind-3.16.1.tar.bz2
    local HASH=d1b153f1ab17cf1f311705e7a83ef589

    get_install_file "$FILE" "$HASH"
    cat $FILE | bzip2 -dc | tar -xf -
    cd valgrind-3.16.1

    ./configure --prefix=$CC_ROOT    
    nice ionice -c3 make -j$(nproc)
    make install
}

# ------------------------------------------------------------------------------

cmake_already_installed()
{
    [ ! -x "$CC_ROOT/bin/cmake" ] && return 1
    local N_VERSION="$($CC_ROOT/bin/cmake --version | grep version | awk '{ print $3 }')"
    local COMPARE="$("$PPWD/version-compare.sh" "$N_VERSION" "3.18.1")"
    [ "$COMPARE" = "2" ] && return 1
    return 0
}

ninja_already_installed()
{
    [ ! -x "$CC_ROOT/bin/ninja" ] && return 1
    local N_VERSION="$($CC_ROOT/bin/ninja --version)"
    [ "$N_VERSION" = "1.8.2.git" ] && return 0
    return 1
}

mobius_already_installed()
{
    [ ! -x "$CC_ROOT/bin/mobius" ] && return 1
    return 0
}

do_install()
{
    DESC="$1"
    TEST_FN="$2"
    INSTALL_FN="$3"

    DO_IT=0
    
    if $TEST_FN ; then
        if [ "$FORCE" = "0" ] ; then
            echo "$DESC is already correctly installed, skipping"
        else
            echo "$DESC already installed, but reinstall requested"
            DO_IT=1
        fi
    else
        DO_IT=1
    fi

    if [ "$DO_IT" = "1" ] ; then
        echo "Installing $DESC"
        $INSTALL_FN
        if ! $TEST_FN ; then
            echo "$INSTALL_FN failed! Is something wrong with the install script?"
            exit 1
        fi
    fi
}

# An existing doxygen binary could break everything
rm -f $CC_ROOT/bin/doxygen

install_cmake
install_ninja
install_mobius
install_valgrind
