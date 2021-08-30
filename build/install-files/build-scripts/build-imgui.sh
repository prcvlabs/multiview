#!/bin/bash

set -e

PPWD="$(cd "$(dirname "$0")" ; pwd)"
source "$PPWD/../base-scripts/env.sh"

# ----------------------------------------------------------------- Must be root

if [[ "$EUID" -ne 0 ]]; then
    echo "Must be root, aborting."
    exit 1
fi

# ------------------------------------------------------------ Working directory

TMPD=$(mktemp -d /tmp/$(basename $0).XXXXX)

trap cleanup EXIT
cleanup()
{
    rm -rf $TMPD
}

VERSION=1.78
FILE=imgui_v${VERSION}.tar.bz2
HASH=61d0f687a722e2a56d871ff1f0f881a3

cd $TMPD
get_install_file "$FILE" "$HASH"
cat "$FILE" | bzip2 -dc | tar -xf -

mkdir build

cd "imgui"

compile_it()
{
    L="$1"
    echo "Compiling ${L:2}"
    echo "$CXX -flto=thin $CXXFLAGS -I/usr/include/SDL2 -I./ -I${PREFIX}/include -I/usr/include/freetype2 -o ../build/$(basename "$L" .cpp).o -c \"$L\""
    $CXX -flto=thin $CXXFLAGS -I/usr/include/SDL2 -I./ -I${PREFIX}/include -I/usr/include/freetype2 -o ../build/$(basename "$L" .cpp).o -c "$L"
}

find . -type f -name '*.cpp' | grep -v examples/ | while read L ; do
    compile_it "$L"
done

compile_it examples/imgui_impl_opengl3.cpp
compile_it examples/imgui_impl_sdl.cpp

echo "Linking libimgui.so.${VERSION}"
cd "../build"

OBJECTS="$(find . -type f -name '*.o')"
$CXX -flto=thin -shared -o libimgui.so.${VERSION} $LDFLAGS $LIBS -lSDL2 -lfreetype $OBJECTS

echo "Installing headers to $PREFIX/include"

mkdir -p "$PREFIX/include"

install_header()
{
    cp "$1" "$PREFIX/include/"
}

find ../imgui -type f -name '*.h' | grep -v examples/ | while read L ; do
    install_header "$L"
done
install_header "../imgui/examples/imgui_impl_opengl3.h"
install_header "../imgui/examples/imgui_impl_sdl.h"

echo "Installing library to $PREFIX/lib"
sudo cp -f libimgui.so.${VERSION} $PREFIX/lib/
cd $PREFIX/lib/
sudo rm -f libimgui.so
sudo ln -s libimgui.so.${VERSION} libimgui.so


