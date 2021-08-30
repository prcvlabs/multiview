#!/bin/bash

TMPD=$(mktemp -d /tmp/$(basename $0).XXXXX)
trap cleanup EXIT
cleanup()
{
    rm -rf $TMPD
}

# ------------------------------------------------------------ Setup Environment

PPWD="$(cd "$(dirname "$0")"; pwd)"

# ------------------------------------------------------------ Parse Commandline

TOOLCHAIN="clang-11.0.0"
CLEAN=
VERBOSE=
CONFIG=asan
TARGET_FILE0=multiview-cli
TARGET_FILE="$TARGET_FILE0"

safe_conf()
{
    FILE="$PPWD/../../build/install-files/scripts/$1"
    DEFAULT_VALUE="$2"
    if [ -f "$FILE" ] ; then
        cat "$FILE"
    else
        echo "$DEFAULT_VALUE"
    fi
}

FEEDBACK=-a
NO_BUILD=0
GDB=0
GDB_BREAK=0
NVPROFILE=0
BUILD_ONLY=0
TRICLOPS=0
OPENGL=0
EGL=0
SGM=0
GUI=0
IMGUI=0
INFO_AND_EXIT=0
LTO=0
PROFILE_MEMORY=0
PCL=0
TRACE_MODE=0
TIME_MODE=0
OPENPOSE_SERIAL=0
WITH_OPENPOSE=0
WITH_HYPERPOSE=1
USE_CUDA=1

while [ "$#" -gt "0" ] ; do
    
    # Compiler
    [ "$1" = "llvm-11.0.0" ] && TOOLCHAIN="llvm-11.0.0" && shift && continue
    [ "$1" = "clang-11.0.0" ] && TOOLCHAIN="clang-11.0.0" && shift && continue

    # Configuration
    [ "$1" = "debug" ]     && CONFIG=debug && shift && continue
    [ "$1" = "gdb" ]       && CONFIG=debug && GDB=1 && shift && continue
    [ "$1" = "ns-gdb" ]    && CONFIG=debug && GDB=1 && GDB_BREAK=1 \
        && shift && continue
    [ "$1" = "nvprofile" ]    && NVPROFILE=1 && shift && continue
    [ "$1" = "release" ]   && CONFIG=release && shift && continue
    [ "$1" = "asan" ]      && CONFIG=asan && shift && continue    # sanitizers
    [ "$1" = "usan" ]      && CONFIG=usan && shift && continue
    [ "$1" = "tsan" ]      && CONFIG=tsan && shift && continue
    [ "$1" = "prof" ]      && CONFIG=prof && shift && continue    # microbench
    [ "$1" = "bench" ]     && CONFIG=prof && shift && continue   
    [ "$1" = "analyze" ]   && CONFIG=analyze && shift && continue # analyze    
    
    # Other options
    [ "$1" = "clean" ]     && CLEAN="-t clean" && shift && continue
    [ "$1" = "verbose" ]   && VERBOSE="-v" && shift && continue   # verbose ninja
    [ "$1" = "nf" ]        && FEEDBACK="" && shift && continue    # quiet ninja
    [ "$1" = "mobius" ]    && NO_BUILD="1" && shift && continue   # no ninja
    [ "$1" = "build" ]     && BUILD_ONLY="1" && shift && continue
    [ "$1" = "triclops" ]  && TRICLOPS="1" && shift && continue
    [ "$1" = "notriclops" ] && TRICLOPS="0" && shift && continue
    [ "$1" = "egl" ]       && OPENGL="1" && EGL="1" && shift && continue
    [ "$1" = "sgm" ]       && SGM="1" && USE_CUDA="1" && shift && continue
    [ "$1" = "lto" ]       && LTO="1" && shift && continue
    [ "$1" = "opengl" ]    && OPENGL="1" && shift && continue
    [ "$1" = "info" ]      && INFO_AND_EXIT="1" && shift && continue
    [ "$1" = "print" ]     && INFO_AND_EXIT="1" && shift && continue
    [ "$1" = "prof-mem" ]  && PROFILE_MEMORY="1" && shift && continue
    [ "$1" = "no-prof-mem" ] && PROFILE_MEMORY="0" && shift && continue
    [ "$1" = "trace" ]     && TRACE_MODE="1" && shift && continue
    [ "$1" = "timer" ]     && TIME_MODE="1" && shift && continue
    [ "$1" = "openpose-serial" ] && OPENPOSE_SERIAL="1" && shift && continue
    
    # Target
    [ "$1" = "test" ] \
        && TARGET_FILE=multiview-testcases && PROFILE_MEMORY="1" \
        && shift \
        && continue
    [ "$1" = "gui" ] \
        && TARGET_FILE=multiview-gui \
        && GUI="1" \
        && OPENGL="1" \
        && shift \
        && continue
    [ "$1" = "imgui" ] \
        && TARGET_FILE=multiview-imgui \
        && IMGUI="1" \
        && OPENGL="1" \
        && shift \
        && continue
    [ "$1" = "cli" ]   && TARGET_FILE=multiview-cli && shift && continue
    [ "$1" = "multiview_cpp.so" ] && echo "python bindings removed" && exit 1
    [ "$1" = "multiview_gl_cpp.so" ] && echo "python bindings removed" && exit 1
    
    break
    
done

# ---------------------------------- Don't try to use CUDA if it isn't installed

if [ ! -d "/usr/local/cuda" ] ; then
    echo "Falling to find CUDA installation, aborting."
    exit 1
fi

# ------------------------------------------------------ Setup build environment

export ARCH="$TOOLCHAIN"
source "$PPWD/../../build/install-files/base-scripts/env.sh"

export MOBIUSDIR="$PPWD/project-config/toolchain-configs"
$MOBIUSDIR/set-env.sh "$ARCH"

# ------------------------------------------------------ Set the build directory

UNIQUE_DIR="${TOOLCHAIN}-${CONFIG}"
[ "$BUILD_TESTS" = "1" ] && UNIQUE_DIR="test_${UNIQUE_DIR}"
[ "$LTO" = "1" ] && UNIQUE_DIR="${UNIQUE_DIR}_lto"
[ "$GUI" = "1" ] && UNIQUE_DIR="${UNIQUE_DIR}_gui"

# -------------------------------------------------------------- Build Variables  

export MULTIVIEW_VERSION="$($PPWD/../../bin/multiview-version.sh)"

if [ "$TRICLOPS" = "1" ] && [ -f "/usr/include/triclops/triclops.h" ] ; then
    export TRICLOPS_FLAGS="-DUSE_TRICLOPS -isystem/usr/include/flycapture -isystem/usr/include/triclops"
    export TRICLOPS_LINK="-lflycapture -ltriclops"
else
    export TRICLOPS_FLAGS=""
    export TRICLOPS_LINK=""
fi

# PCL should be present when OpenGL is present
if [ "$PCL" = "1" ] ; then
    export PCL_FLAG="-isystem/usr/include/pcl-1.8"
    export PCL_LINK="-L/usr/lib -lpcl_io_ply -lpcl_recognition -lpcl_stereo -lpcl_sample_consensus -lpcl_segmentation -lpcl_tracking -lpcl_octree -lpcl_keypoints -lpcl_io -lpcl_features -lpcl_surface -lpcl_search -lpcl_registration -lpcl_filters -lpcl_common -lpcl_ml -lpcl_kdtree"
else
    export PCL_FLAG=""
    export PCL_LINK=""    
fi

# no GLU
if [ "$OPENGL" = "1" ] ; then
    export OPENGL_FLAG="-DUSING_OPENGL"
    export OPENGL_LINK="-lX11 -lGL -lGLEW"
else
    export OPENGL_FLAG=""
    export OPENGL_LINK=""
fi

if [ "$EGL" = "1" ] ; then
    export EGL_LINK="-lEGL"
    export EGL_FLAG="-DUSE_EGL"
else
    export EGL_LINK=""
    export EGL_FLAG="-DUSE_GLX"
fi

if [ "$SGM" = "1" ] ; then
    export SGM_LINK="\$local_lib -lsgm"
    export SGM_FLAG="-DUSE_SGM"
else
    export SGM_LINK=""
    export SGM_FLAG=""
fi

if [ "$GUI" = "1" ] ; then
    QT_PACKAGES="Qt5Core Qt5Gui Qt5OpenGL"
    export QT_LINK="$(pkg-config --libs $QT_PACKAGES)"
    export QT_FLAG="-DGUI_BUILD $(pkg-config --cflags $QT_PACKAGES) -Wno-enum-enum-conversion -Wno-deprecated-copy"
else
    export QT_LINK=""
    export QT_FLAG="" 
fi

if [ "$IMGUI" = "1" ] ; then
    export IMGUI_LINK="-lSDL2 -lfreetype"
    export IMGUI_FLAG="-DIMGUI_BUILD -isystem/usr/include/freetype2 -isystem/usr/include/SDL2 -isystemsrc/im-gui/contrib/imgui -Isrc/im-gui"
else
    export IMGUI_LINK=""
    export IMGUI_FLAG="" 
fi

if [ "$PROFILE_MEMORY" = "1" ] ; then
    export PROF_MEM_FLAG="-DPERCEIVE_PROFILE_MEMORY"
else
    export PROF_MEM_FLAG=""
fi

get_branch()
{
    cd "$PPWD"
    git branch | grep \* | sed 's,[*()],,g' | sed 's,[^[:alnum:] -_],,g' \
        | awk '{$1=$1};1' | sed 's,\s,-,g'
}

export TOOLCHAIN=$TOOLCHAIN
export TARGET=$TARGET
export TARGET="build/$UNIQUE_DIR/$TARGET_FILE"
# export BUILDDIR="/tmp/build-$USER/${TOOLCHAIN}-${CONFIG}/\$target"
BRANCH="$(get_branch)"
export BUILDDIR="/tmp/build-$USER/${UNIQUE_DIR}_${BRANCH}/$TARGET_FILE"
export PCHFILE=src/stdinc.hpp
export NINJA_FILE="$(dirname "$TARGET")/build.ninja"

if [ "$TARGET_FILE" = "multiview-testcases" ] ; then
    export SRC_DIRECTORIES="src/perceive src/testcases"
    export LINK_COMMAND=link_exec

elif [ "$TARGET_FILE" = "multiview-cli" ] ; then    
    export SRC_DIRECTORIES="src/perceive src/front-ends"
    export LINK_COMMAND=link_exec

elif [ "$TARGET_FILE" = "multiview-gui" ] ; then
    export SRC_DIRECTORIES="src/perceive src/gui"
    export LINK_COMMAND=link_exec

elif [ "$TARGET_FILE" = "multiview-imgui" ] ; then
    export SRC_DIRECTORIES="src/perceive src/im-gui"
    export LINK_COMMAND=link_exec
    
else
    echo "Unexpected target=$TARGET_FILE, aborting"
    exit 1
fi

[ "$OPENGL" = "1" ] \
    && export SRC_DIRECTORIES="$SRC_DIRECTORIES src/gl"

[ -d "$PPWD/src/cuda" ] \
    && export SRC_DIRECTORIES="$SRC_DIRECTORIES src/cuda"

case $CONFIG in
    release)
        export O_FLAG="-O3"
        export C_FLAGS="\$c_w_flags \$w_user \$f_flags \$r_flags"
        export CPP_FLAGS="\$cpp_std \$w_flags \$w_user \$f_flags \$r_flags"
        export LINK_FLAGS="\$l_flags"
        ;;

    debug)
        export O_FLAG="-O0"
        export C_FLAGS="\$c_w_flags \$w_user \$f_flags \$d_flags \$gdb_flags"
        export CPP_FLAGS="\$cpp_std \$w_flags \$w_user \$f_flags \$d_flags \$gdb_flags"
        export LINK_FLAGS="\$l_flags"
        ;;

    asan)
        export O_FLAG="-O0"
        export C_FLAGS="\$c_w_flags \$w_user \$f_flags \$d_flags \$gdb_flags \$asan_flags"
        export CPP_FLAGS="\$cpp_std \$w_flags \$w_user \$f_flags \$d_flags \$gdb_flags \$asan_flags"        
        export LINK_FLAGS="\$l_flags \$asan_link"
        ;;

    usan)
        export O_FLAG="-O1"
        export C_FLAGS="\$c_w_flags \$w_user \$f_flags \$s_flags \$usan_flags"
        export CPP_FLAGS="\$cpp_std \$w_flags \$w_user \$f_flags \$s_flags \$usan_flags"
        export LINK_FLAGS="\$l_flags \$usan_link"
        ;;

    tsan)
        # https://github.com/google/sanitizers/wiki/ThreadSanitizerCppManual
        # ! [ "$TOOLCHAIN" = "clang-9.0" ] && \
        #     echo "tsan is only supported with clang." && \
        #     exit 1
        export O_FLAG="-O0"
        export C_FLAGS="\$c_w_flags \$w_user \$f_flags \$s_flags \$d_flags \$gdb_flags \$tsan_flags"
        export CPP_FLAGS="\$cpp_std \$w_flags \$w_user \$f_flags \$s_flags \$d_flags \$gdb_flags \$tsan_flags"
        export LINK_FLAGS="\$l_flags \$tsan_link"
        ;;

    prof)
        export O_FLAG="-O2"
        export C_FLAGS="\$c_w_flags \$w_user \$f_flags \$r_flags \$prof_flags"
        export CPP_FLAGS="\$cpp_std \$w_flags \$w_user \$f_flags \$r_flags \$prof_flags"
        export LINK_FLAGS="\$l_flags \$prof_link"
        ;;

    analyze)
        ! [ "$TOOLCHAIN" = "clang-5.0" ] && \
            echo "Static analysis is only supported with clang." && \
            exit 1
        export O_FLAG="-O2"
        export C_FLAGS="\$c_w_flags \$w_user \$f_flags \$r_flags"
        export CPP_FLAGS="\$cpp_std \$w_flags \$w_user \$f_flags \$r_flags"
        export LINK_FLAGS="\$l_flags"
        ;;
    
    *)
        echo "Unexpected config=$CONFIG, aborting"
        exit 1
        ;;
esac

# ---- Add LTO flag
if [ "$LTO" = "1" ] ; then
    export C_FLAGS="-flto=thin $C_FLAGS"
    export CPP_FLAGS="-flto=thin $CPP_FLAGS"
    export LINK_FLAGS="-flto=thin $LINK_FLAGS"
fi

# ---- Add TEST define
if [ "$TARGET_FILE" = "multiview-testcases" ] ; then
    export C_FLAGS="-DTESTCASE_BUILD $C_FLAGS"
    export CPP_FLAGS="-DTESTCASE_BUILD $CPP_FLAGS"
fi

# ---- Add -DWITH_CUDA if we have cuda
export CUDA_GENCODE="$(for F in $CUDA_ARCH_BINS ; do X=$(echo "$F" | sed 's,\.,,') ; echo "-gencode=arch=compute_$X,code=sm_$X" ; done | tr '\n' ' ')"
export C_FLAGS="$C_FLAGS -DWITH_CUDA"
export CPP_FLAGS="$CPP_FLAGS -DWITH_CUDA"

# ---- Link against tirpc on fedora ----
if [ "$IS_FEDORA" = "1" ]; then
    LINK_FLAGS="$LINK_FLAGS -ltirpc"
fi

# ---- What python directories are we using?
if [ -d "/usr/include/x86_64-linux-gnu/python3.5m" ] ; then
    export PYTHON_FLAG="-I/usr/local/lib/python3.5/dist-packages/numpy/core/include -I/usr/include/python3.5m -I/usr/include/x86_64-linux-gnu/python3.5m"
    export PYTHON_LINK="-lpython3.5m -lboost_python35"
elif [ -d "/usr/include/x86_64-linux-gnu/python3.6m" ] ; then
    export PYTHON_FLAG="-I/usr/local/lib/python3.6/dist-packages/numpy/core/include -I/usr/include/python3.6m -I/usr/include/x86_64-linux-gnu/python3.6m"
    export PYTHON_LINK="-lpython3.6m -lboost_python36"
else
    export PYTHON_FLAG=""
    export PYTHON_LINK=""
fi

# ---- Add -DWITH_OPENPOSE if we have cuda
export OPENPOSE_FLAGS=""
export OPENPOSE_LINK=""
if [ "$WITH_OPENPOSE" = "1" ] ; then
    export OPENPOSE_BASE="\$local_base/openpose"
    if [ "$USE_CUDA" = "1" ] ; then
        export OPENPOSE_DIR="\$local_base/openpose/cuda"    
    else
        export OPENPOSE_DIR="\$local_base/openpose/cpu" 
    fi
    
    export OPENPOSE_FLAGS="-DWITH_OPENPOSE -isystem${OPENPOSE_DIR}/include"
    export OPENPOSE_LINK="-L${OPENPOSE_DIR}/lib -Wl,-rpath,${OPENPOSE_DIR}/lib, -lcaffe -lopenpose"
fi

# ---- Add -DWITH_HYPERPOSE...
export HYPERPOSE_FLAGS=""
export HYPERPOSE_LINK=""
if [ "$WITH_HYPERPOSE" = "1" ] ; then
    export HYPERPOSE_DIR="\$local_base/opt/hyperpose"
    export HYPERPOSE_FLAGS="-DWITH_HYPERPOSE"
    export HYPERPOSE_LINK="-lhyperpose"
fi

# ----------------------------------------------------------------------- Action
 
if [ "$INFO_AND_EXIT" = "1" ] ; then
    cat <<EOF

# Build Configuration #
 * CXX            = $CXX
 * TARGET_FILE    = $TARGET_FILE
 * TOOLCHAIN      = $TOOLCHAIN
 * CONFIG         = $CONFIG
 * SRC            = $SRC_DIRECTORIES
 * O_FLAG         = $O_FLAG
 * C_FLAGS        = $C_FLAGS
 * CPP_FLAGS      = $CPP_FLAGS
 * LINK_FLAGS     = $LINK_FLAGS
 * LINK           = $LINK_COMMAND
 * BUILDDIR       = $BUILDDIR
 * NINJA_FILE     = $NINJA_FILE
 * TARGET         = $TARGET

# Other options #
 * CLEAN           = $CLEAN
 * VERBOSE         = $VERBOSE
 * FEEDBACK        = $FEEDBACK
 * NO_BUILD        = $NO_BUILD
 * BUILD_ONLY      = $BUILD_ONLY
 * TRICLOPS        = $TRICLOPS
 * OPENGL          = $OPENGL
 * LTO             = $LTO
 * EGL             = $EGL
 * SGM             = $SGM
 * CUDA            = $USE_CUDA
 * WITH_OPENPOSE   = $WITH_OPENPOSE
 * HYPERPOSE_FLAGS = $HYPERPOSE_FLAGS
 * HYPERPOSE_LINK  = $HYPERPOSE_LINK
 * HYPERPOSE_DIR   = $HYPERPOSE_DIR
 * GUI             = $GUI
 * IMGUI           = $IMGUI
 * PROFILE_MEMORY  = $PROFILE_MEMORY
 * TRACE_MODE      = $TRACE_MODE

EOF
    
    exit 0
fi

cd "$(dirname "$0")"
mkdir -p "$(dirname "$TARGET")"

MOBIUS=$CC_ROOT/bin/mobius
NINJA=$CC_ROOT/bin/ninja

ANALYZE="--analyze -Xanalyzer -analyzer-output=text -Xanalyzer -analyzer-checker=core -Xanalyzer -analyzer-checker=cplusplus -Xanalyzer -analyzer-checker=unix -Xanalyzer -analyzer-checker=alpha.unix -Xanalyzer -analyzer-checker=alpha.core -Xanalyzer -analyzer-checker=alpha.cplusplus"
if [ "$CONFIG" = "analyze" ] ; then    
    $MOBIUS -i project-config/build.mobius | $NINJA -f - -v -t commands | sed 's,nice ionice -c3 ,,' | sed '$ d' | sed '$ d' | sed 's,-MMD -MF /[^ ]*,,' | sed 's,-o.*,,' | sed 's,-include /[^ ]*,,' | sed "s,clang-5.0,clang-5.0 $ANALYZE," | grep src/perceive | grep -v .pb.cc
    exit $?
fi

cat project-config/build.mobius | $MOBIUS - > "$NINJA_FILE"
MOBIUS_RET="$?"                             

! [ "$MOBIUS_RET" = "0" ] && exit $MOBIUS_RET

if [ "$NO_BUILD" = "1" ] ; then
    echo "Ninja file generated: $PPWD/$NINJA_FILE"
    exit "$MOBIUS_RET"
elif ! [ "$MOBIUS_RET" = "0" ] ; then
    exit "$MOBIUS_RET"
fi

# | grep -v x86_64-linux-gnu/libGL.so
nice ionice -c3 $NINJA -f $(expr 10 \* $(nproc) / 10) -f "$NINJA_FILE" $FEEDBACK $VERBOSE $CLEAN | grep -v x86_64-linux-gnu/libGL.so
NINJA_EXIT="${PIPESTATUS[0]}"

! [ "$NINJA_EXIT" = "0" ] && exit "$NINJA_EXIT"

! [ "$CLEAN" = "" ] && exit 0
[ "$BUILD_ONLY" = "1" ] && exit 0

# ---- If we're building the executable (multiview-testcases), then run it

if [ "$TARGET_FILE" = "multiview-testcases" ] || \
       [ "$TARGET_FILE" = "multiview-cli" ] || \
       [ "$TARGET_FILE" = "multiview-gui" ] || \
       [ "$TARGET_FILE" = "multiview-imgui" ]
then
    cd $PPWD
    PRODUCT="$TARGET"

    export MULTIVIEW_CODE="$(cd ../.. ; pwd)"

    if [ "$CONFIG" = "asan" ] || [ "$CONFIG" = "usan" ] || [ "$CONFIG" = "tsan" ] ; then
        P_EXEC="$PRODUCT"
    else
        P_EXEC="$TMPD/$(basename "$PRODUCT")"
        cp -a "$PRODUCT" "$P_EXEC"
    fi

    if [ "$TRACE_MODE" = "1" ] ; then
        export MULTIVIEW_TRACE_MODE=1
    fi

    if [ "$OPENPOSE_SERIAL" = "1" ] ; then
        export MULTIVIEW_OPENPOSE_SERIAL=1
    fi
    
    export GLOG_minloglevel=3
    export GLOG_logtostderr=1

    if [ -f "$HOME/.aws/credentials" ] && [ "$AWS_ACCESS_KEY_ID" = "" ] ; then
        export AWS_ACCESS_KEY_ID="$(cat $HOME/.aws/credentials | grep key_id | cut -d ' ' -f3)"
        export AWS_SECRET_ACCESS_KEY="$(cat $HOME/.aws/credentials | grep secret | cut -d ' ' -f3)"
    fi
    
    if [ "$GDB_BREAK" = "1" ] ; then
        gdb -silent -return-child-result -statistics --args $P_EXEC "$@"
        exit $?
    elif [ "$GDB" = "1" ] ; then
        gdb -ex 'handle SIGFPE stop nopass' -ex run -silent -return-child-result -statistics --args $P_EXEC "$@"
        exit $?
    elif [ "$NVPROFILE" = "1" ] ; then
        exec /usr/local/cuda/bin/nvprof -f -o /tmp/multiview.nvvp $P_EXEC "$@"
    fi
    
    TIME_CMD=""
    [ "$TIME_MODE" = "1" ] && TIME_CMD="/usr/bin/time --verbose"
    export TSAN_OPTIONS="suppressions=$PPWD/project-config/tsan-options"
    export ASAN_OPTIONS="detect_leaks=0,protect_shadow_gap=0"
    nice ionice -c3 $TIME_CMD $P_EXEC "$@"
    exit $?
fi

exit 0

