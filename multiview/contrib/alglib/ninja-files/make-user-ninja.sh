#!/bin/bash

cd "$(dirname "$0")"

PKG_CONFIGS="eigen3 jsoncpp python3 opencv protobuf"
INCLUDES="-Isrc -isystem../contrib/alglib/include"
EXTRA_FLAGS="-Wno-vla -Wno-unused-parameter $INCLUDES"

DEFINE_GL=
LINK_GL=
pkg-config --list-all | grep OpenGl | grep -q gl \
    && DEFINE_GL="-DUSING_OPENGL" \
    && LINK_GL="-lGL -lGLU"

make_user()
{
    cat <<EOF

# User-config autogenerated by 'ninja-files/$(basename $0)'
cpp_flags  = \$cpp_flags  $EXTRA_FLAGS $(pkg-config --cflags $PKG_CONFIGS) $DEFINE_GL
 


link_flags = \$link_flags -lutil -ldl $(pkg-config --libs $PKG_CONFIGS) $LINK_GL -L/usr/local/lib -lboost_python3 -lboost_filesystem

EOF

}

make_user > user.ninja
