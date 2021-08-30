#!/bin/bash

# Bash will exit on first error
set -e

PPWD="$(cd "$(dirname "$0")" ; pwd)"
source "$PPWD/../base-scripts/env.sh"

# ----------------------------------------------------------------- Must be root

if [[ "$EUID" -ne 0 ]]; then
    echo "Must be root, aborting."
    exit 1
fi

BOOST_VERSION=1_71_0

TMPD=$(mktemp -d /tmp/$(basename $0).XXXXXX)

trap cleanup EXIT
cleanup()
{
    if [ 1 = 1 ] ; then
        rm -rf $TMPD
        rm -f $HOME/user-config.jam
    else
        echo "Skipping cleanup"
    fi
}

# ------------------------------------------------------------- Get sources code

FILE=boost_${BOOST_VERSION}.tar.gz
HASH=5f521b41b79bf8616582c4a8a2c10177

cd $TMPD
get_install_file "$FILE" "$HASH"
cat "boost_${BOOST_VERSION}.tar.gz" | gzip -dc | tar -xf -
cd boost_${BOOST_VERSION}

# -------------------------------------------------------- Setup user-config.jam

rm -f "$HOME/user-config.jam"
cp tools/build/example/user-config.jam $HOME
if [ "$IS_UBUNTU" = "1" ] ; then
    source /etc/lsb-release
    if [ "$DISTRIB_RELEASE" == "18.04" ] ; then
        echo "using python : 3.6 : /usr/bin/python3 : /usr/include/python3.6m : /usr/lib ;" >> $HOME/user-config.jam
    else
        echo "Only setup fo Ubuntu 18.04 (for now)"
        exit 1
    fi
else
    echo "Not setup!"
    exit 1
fi

cat >> $HOME/user-config.jam <<EOF
using clang 
   : 11.0.0 
   : $CXX 
   : <cxxflags>"$CXXFLAGS" 
     <linkflags>"$LDFLAGS $LIBS" 
   ;
EOF

# ------------------------------------------------------------ bootstrap & build

# This just confuses the boostrap.sh process
unset CC
unset CXX
unset CFLAGS
unset CXXFLAGS
unset LDFLAGS
unset LIBS

BOOST_LIBS="--with-system --with-filesystem --with-graph --with-python --with-date_time --with-math --with-thread"
./bootstrap.sh --prefix=$PREFIX --with-icu=$PREFIX/lib
./b2 -j $(nproc) install toolset=clang cxxstd=2a $BOOST_LIBS

