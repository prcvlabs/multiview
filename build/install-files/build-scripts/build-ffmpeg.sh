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

# git clone -b release/4.1 https://github.com/FFmpeg/FFmpeg.git
FILE="ffmpeg_4.1.tar.gz"
HASH=2b44b40f1991359dc0741fd360440108

cd $TMPD
get_install_file "$FILE" "$HASH"
cat $FILE | gzip -dc | tar -xf -
cd ffmpeg_4.1

export CPPFLAGS="$CXXFLAGS"
export LDFLAGS="$LDFLAGS $LIBS"

./configure --prefix=$PREFIX \
            --enable-gpl --enable-version3 --enable-nonfree --enable-shared --disable-doc \
            --enable-gmp \
            --enable-libx265 --enable-libxml2 --enable-libxvid --enable-libx264 \
            --enable-libwavpack \
            --enable-gcrypt --enable-gnutls --enable-libmp3lame --enable-libsnappy --enable-librtmp \
            --enable-libpulse --enable-libopus --enable-libopenjpeg --enable-libmodplug \
            --enable-libtheora  --enable-libssh --enable-libspeex --enable-libsoxr \
            --cc=$CC --cxx=$CXX --objcc=$CC --dep-cc=$CC | tee configure-output.text

make -j$(nproc)
sudo make install
sudo ldconfig

exit $?

