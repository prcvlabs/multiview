#!/bin/bash

# Bail on an error
set -e

TMPD="$(mktemp -d /tmp/$(basename $0).XXXXXX)"
trap cleanup EXIT
cleanup()
{
    rm -rf "$TMPD"
}

# ------------------------------------------------------------------------- Help

DEFAULT_D="$HOME/TMP/epoch-output"
CONFIG="release"

show_help()
{
    cat <<EOF

   Usage: $(basename $0) [OPTIONS...] <scene> <epoch>

      --fps <number>
      -d <path>            Default is: $DEFAULT_D/<scene>_<epoch>
      --start <number>     Start frame.
      --n-frames <number>  Number of frames to execute.
      <build-mode>         One of debug|gdb|release|asan|usan|tsan
                           The default is $CONFIG.

      gui                  Load the GUI
      trace                Turn on trace-mode.
      build                Build-only.
      setup                Preparse a testcase for setup.

   Example:

      > $(basename $0) glenbrook_2018-11-02_v1 2018-11-02T15:00:00

EOF
}

# ------------------------------------------------------------ Parse Commandline

PPWD="$(cd "$(dirname "$0")"; pwd)"
RUN_SH="$(cd "$PPWD/../multiview/multiview_cpp" ; pwd)/run.sh"
MM_PY="$PPWD/multiview-manifest.py"
TRACE_MODE=0
SHOW_HELP=0
HAS_ERROR=0
FPS=0
START_F=0
OUT_D=""
N_FRAMES=-1
BUILD_ONLY=""
SCENE_NAME=""
EPOCH_NAME=""
SETUP_MODE=0
GUI_MODE=0

[ "$#" = "0" ]  && SHOW_HELP=1

while [ "$#" -gt "0" ] ; do
    [ "$1" = "debug" ]      && CONFIG=debug && shift && continue
    [ "$1" = "gdb" ]        && CONFIG=debug && GDB=1 && shift && continue
    [ "$1" = "release" ]    && CONFIG=release && shift && continue
    [ "$1" = "asan" ]       && CONFIG=asan && shift && continue    # sanitizers
    [ "$1" = "usan" ]       && CONFIG=usan && shift && continue
    [ "$1" = "tsan" ]       && CONFIG=tsan && shift && continue

    [ "$1" = "build" ]      && BUILD_ONLY="build" && shift && continue
    [ "$1" = "trace" ]      && TRACE_MODE=1 && shift && continue
    [ "$1" = "setup" ]      && SETUP_MODE=1 && shift && continue
    [ "$1" = "gui"   ]      && GUI_MODE=1 && shift && continue

    [ "$1" = "--help" ] || [ "$1" = "-h" ] && SHOW_HELP=1 && shift && continue
    [ "$1" = "--fps" ]      && shift && FPS="$1" && shift && continue
    [ "$1" = "--start" ]    && shift && START_F="$1" && shift && continue
    [ "$1" = "--n-frames" ] && shift && N_FRAMES="$1" && shift && continue
    [ "$1" = "-d" ]         && shift && OUT_D="$1" && shift && continue

    if [ "$SCENE_NAME" = "" ] ; then
        SCENE_NAME="$1" && shift && continue
    elif [ "$EPOCH_NAME" = "" ] ; then
        EPOCH_NAME="$1" && shift && continue
    else
        echo "About to set epoch to '$1'; however, it was already set to '$EPOCH_NAME', aborting."
        HAS_ERROR=1
        shift
    fi
done

# ----------------------------------------------------------------------- Sanity

if [ "$SHOW_HELP" = "1" ] ; then
    show_help
    exit 0
fi

if [ ! -f "$RUN_SH" ] ; then
    echo "Failed to find '$RUN_SH'."
    HAS_ERROR=1
fi

if [ "$SCENE_NAME" = "" ] && [ "$BUILD_ONLY" = "" ] ; then
    echo "Must set the scene!"
    HAS_ERROR=1
fi

if [ "$EPOCH_NAME" = "" ] && [ "$BUILD_ONLY" = "" ] ; then
    echo "Must set the epoch!"
    HAS_ERROR=1
fi

if [ ! "$FPS" -eq "$FPS" 2>/dev/null ] ; then
    echo "Spurious frames per second = '$FPS'."
    HAS_ERROR=1
elif [ "$FPS" -lt "0" ] || [ "$FPS" -gt "60" ] ; then
    echo "Spurious frames per second = '$FPS'."
    HAS_ERROR=1
fi

if [ ! "$START_F" -eq "$START_F" 2>/dev/null ] ; then
    echo "Spurious start frame = '$START_F'."
    HAS_ERROR=1
fi

if [ ! "$N_FRAMES" -eq "$N_FRAMES" 2>/dev/null ] ; then
    echo "Spurious --n-frames = '$N_FRAMES'."
    HAS_ERROR=1
fi

if [ ! -x "$MM_PY" ] ; then
    echo "Failed to find script '$MM_PY'"
    HAS_ERROR=1
fi

if [ "$HAS_ERROR" = "1" ] ; then
    echo "Aborting."
    exit 1
fi

# ------------------------------------------------------- Let's get the Manifest

TMP_MANIFEST_F="$TMPD/manifest.json"
if [ "$BUILD_ONLY" != "build" ] ; then
    "$MM_PY" "$SCENE_NAME" "$EPOCH_NAME" > "$TMP_MANIFEST_F"
fi

# ------------------------------------------------------- Setup output directory

D="$(echo "${SCENE_NAME}_e_${EPOCH_NAME}" | sed 's,\:,\-,g')"
if [ "$OUT_D" = "" ] ; then
    OUT_D="$DEFAULT_D/$D"
fi

FPS_S="<default>"
[ ! "$FPS" = "0" ]                 && FPS_S="$FPS"
TRACE_MODE_S=""
[ "$TRACE_MODE" = "1" ]            && TRACE_MODE_S=", trace"
SETUP_S=""
[ "$SETUP_MODE" = "1" ]            && SETUP_S=", setup-mode"
GUI_S=""
[ "$GUI_MODE" = "1" ] && GUI_S=", gui"

cat <<EOF

   Scene:       $SCENE_NAME
   Epoch:       $EPOCH_NAME
   Config:      $CONFIG$TRACE_MODE_S$SETUP_S$GUI_S
   Out Folder:  $OUT_D
   FPS:         $FPS_S

EOF

SETUP_ARG=""
[ "$SETUP_MODE" = "1" ]            && SETUP_ARG="--setup-testcase"

STDOUT="/dev/stdout"
STDERR="/dev/stderr"
MANIFEST_F="$TMP_MANIFEST_F"
if [ "$BUILD_ONLY" != "build" ] ; then
    rm -rf "$OUT_D"
    mkdir -p "$OUT_D"
    MANIFEST_F="$OUT_D/manifest.json"
    cp "$TMP_MANIFEST_F" "$MANIFEST_F"

    STDOUT="$OUT_D/stdout"
    STDERR="$OUT_D/stderr"
fi

DEBUG_VIDEO="--debug-video"
RUN_MODE="pipeline"
GUI_OPTS=""
if [ "$GUI_MODE" = "1" ] ; then
    DEBUG_VIDEO=""
    TRACKS_F=""
    PIPE_TEST=""
    RUN_MODE="gui"
    GUI_OPTS="--developer --annotate-mode --annotations-directory $S3_D/$TESTCASE_NAME"
fi

TRACE_ARG=""
if [ "$TRACE_MODE" = "1" ] ; then
    TRACE_ARG="trace"
fi

{ stdbuf -oL -eL $RUN_SH $CONFIG $BUILD_ONLY $TRACE_ARG $RUN_MODE \
             -m "$MANIFEST_F" \
             -d "$OUT_D" \
             $DEBUG_VIDEO \
             --target-fps "$FPS" \
             --start "$START_F" \
             --n-frames "$N_FRAMES" \
             $SETUP_ARG \
             2>&1 1>&3- | tee "$STDERR" 1>&2 ; } 3>&1 | stdbuf -oL -eL tee "$STDOUT"

RET="$?"

if [ "$RET" = "0" ] && [ "$SETUP_MODE" = "1" ] ; then
    rm -f "$OUT_D/stdout"
    rm -f "$OUT_D/stderr"
    rm -f "$OUT_D/run-object.json"
    echo "" | md5sum | awk '{ print $1 }' > "$OUT_D/hash.md5"
    echo "Testcase setup. Upload command:"
    echo
    echo "   aws s3 cp --recursive \"$OUT_D\" \"s3://perceive-multiview/testcases/$(basename "$OUT_D")\""
    echo
fi

exit $?
