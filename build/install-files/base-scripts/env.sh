#!/bin/bash

# --------------------------------------------------------------------- OPT_ROOT

if [ -z "${OPT_ROOT+set}" ] ; then
    export OPT_ROOT="/opt/multiview"
fi

if [ ! -d "$OPT_ROOT" ] ; then
    sudo mkdir -p "$OPT_ROOT"
    sudo chown "$USER" "$OPT_ROOT"
fi

if [ ! -d "$OPT_ROOT" ] ; then
    echo "Cannot find OPT_ROOT='$OPT_ROOT', aborting"
    exit 1
fi

test_writable()
{
    local F="$OPT_ROOT/.test"
    if [ -f "$F" ] ; then
        rm -f "$F"
        [ -f "$F" ] && return 1
        return 0
    fi

    touch "$F"
    [ ! -f "$F" ] && return 1
    rm -f "$F"
    return 0
}

if [ ! test_writable ] ; then
    echo "User cannot write to '$OPT_ROOT', aborting"
    exit 1
fi

mkdir -p "$OPT_ROOT/etc"    

# ----------------------------------------------------------------- LLVM_VERSION

if [ -z "${ARCH+set}" ] ; then
    if [ ! -f "$OPT_ROOT/etc/arch" ] ; then
        echo "file not found: $OPT_ROOT/etc/arch, aborting"
        exit 1
    else
        export ARCH="$(cat $OPT_ROOT/etc/arch)"
    fi
fi

! [ "$ARCH" = "llvm-11.0.0" ] \
    && ! [ "$ARCH" = "clang-11.0.0" ] \
    && echo "llvm-version must be '11.0.0'" \
    && exit 1

export PREFIX="$OPT_ROOT/$ARCH"

mkdir -p "$PREFIX"
[ ! -d "$PREFIX" ] \
    && echo "could not create PREFIX='$PREFIX'" \
    && exit 1

# --------------------------------------------------------------------- PLATFORM

export IS_UBUNTU=$([ -x /usr/bin/lsb_release ] && lsb_release -a 2>/dev/null | grep -q Ubuntu && echo 1 || echo 0)
export IS_FEDORA=$([ -f /etc/fedora-release ] && echo 1 || echo 0)

! [ "$IS_UBUNTU" = "1" ] && ! [ "$IS_FEDORA" = "1" ] \
    && echo "Failed to determine distribution, aborting" \
    && exit 1
[ "$IS_UBUNTU" = "1" ] && [ "$IS_FEDORA" = "1" ] \
    && echo "Cannot be both Ubuntu and Fedora at the same time!" \
    && exit 1

if [ -f "/.dockerenv" ] ; then
    export DOCKER_MODE=1
else
    export DOCKER_MODE=0
fi

# --------------------------------------------------------------- grabbing files

export INSTALL_FILE_URL="https://perceive-multiview.s3.amazonaws.com/install-files"
export INSTALL_FILE_S3="s3://perceive-multiview/install-files"

get_install_file()
{
    local FILE="$1"
    local HASH="$2"

    local INSTALL_D="/home/zeus/Dropbox/perceive-data/multiview/install-files"
    if [ -f "$INSTALL_D/$FILE" ] ; then
        local H2="$(md5sum "$INSTALL_D/$FILE" | awk '{ print $1 }')"
        if [ "$H2" != "$HASH" ] ; then
            echo "hash mismatch for install file '$INSTALL_D/$FILE': expected '$HASH', but got '$H2'" && \
                exit 1
        fi
        if [ "$(pwd)" != "$INSTALL_D" ] ; then
            cp "$INSTALL_D/$FILE" .
            return 0
        fi
    fi

    if [ "$DOCKER_MODE" = "1" ] ; then
        wget -nv "$INSTALL_FILE_URL/$FILE"
    else
        aws s3 cp "$INSTALL_FILE_S3/$FILE" .
    fi
    
    local H2="$(md5sum "$FILE" | awk '{ print $1 }')"
    [ "$H2" != "$HASH" ] && \
        echo "hash mismatch for install file '$FILE': expected '$HASH', but got '$H2'" && \
        exit 1
    return 0
}

# --------------------------------------------------------------- uniqufy (PATH)

uniquify()
{
    INPUT="$1"
    DELIM=
    (( $# > 1 )) && DELIM="$2"

    if [ "$DELIM" = "" ] ; then
        echo "$(printf "%s" "${INPUT}" | awk -v RS=" " -v ORS=" " '!($0 in a) {a[$0]; print}')"
    else
        RES="$(printf "%s" "${INPUT}" | awk -v RS=$DELIM -v ORS=$DELIM '!($0 in a) {a[$0]; print}')"
        echo "${RES%$DELIM}"    # remove trailing delim
    fi
}

# ------------------------------------------------------------------------------

export CC_ROOT="$PREFIX/opt/cc"
export DEBIAN_FRONTEND=noninteractive
export PATH="$(uniquify "$CC_ROOT/bin:$PATH")"

# ----------------------------------------------------------------------- CXX/CC

export CC="$CC_ROOT/bin/clang"
export CXX="$CC_ROOT/bin/clang++"
export CMAKE="$CC_ROOT/bin/cmake"
export LLD="$CC_ROOT/bin/ld.lld"

export CXXSTD="-std=c++20"

#  -nostdinc++ -isystem$CC_ROOT/include/c++/v1 -isystem$CC_ROOT/include -isystem$PREFIX/include/c++/v1
#   -stdlib=libc++ -L$CC_ROOT/lib -Wl,-rpath,$CC_ROOT/lib -L$PREFIX/lib -Wl,-rpath,$PREFIX/lib

export CFLAGS="-fPIC -O3"
export CXXFLAGS="-fPIC -O3"

export LDFLAGS="-fuse-ld=$LLD -L$PREFIX/lib -Wl,-rpath,$PREFIX/lib"
export LIBS="-lm -pthreads"


export PKG_CONFIG_PATH="$PREFIX/lib/pkgconfig"

#  -gencode=arch=compute_52,code=sm_52 -gencode=arch=compute_61,code=sm_61 -gencode=arch=compute_75,code=sm_75
export CUDA_ARCH_BINS="5.2 6.1 7.5 8.6"

LD_FILE="/etc/ld.so.conf.d/multiview.conf"
if [ ! -f "$LD_FILE" ] || [ "$(cat "$LD_FILE")" != "$CC_ROOT/lib" ] ; then
    sudo echo "$CC_ROOT/lib" > "$LD_FILE"
    sudo ldconfig
fi

# -------------------------------------------------------------------- print-env

print_env()
{
    cat <<EOF

    OPT_ROOT:        $OPT_ROOT  
    ARCH:            $ARCH
    PREFIX:          $PREFIX
    CC_ROOT:         $CC_ROOT

    IS_UBUNTU:       $IS_UBUNTU
    IS_FEDORA:       $IS_FEDORA
    DOCKER_MODE:     $DOCKER_MODE

    DEBIAN_FRONTEND: $DEBIAN_FRONTEND

    CC:              $CC
    CXX:             $CXX

    CXXSTD:          $CXXSTD
    CFLAGS:          $CFLAGS
    CXXFLAGS:        $CXXFLAGS

    LDFLAGS:         $LDFLAGS
    LIBS:            $LIBS

    CUDA_ARCH_BINS:  $CUDA_ARCH_BINS

EOF
}


