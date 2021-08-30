#!/bin/bash

# ------------------------------------------------------------------------- Help

DEFAULT_D="$HOME/TMP/testcase-output"
DEFAULT_CLASSIFIER="pose-svm-lite_v0"

show_help()
{
    cat <<EOF

   Usage: $(basename $0) [OPTIONS...] <testcase-name>

      The possible testcase names are given below.

   Options:

      -h|--help      Show help.
     
      trace          Turn on "trace" messages.
      build          Build multiview and quit.
      gui            Run the editor GUI for this testcase.

      --fps <number> Frame per second.
      -d <path>      Output directory. Default is: $DEFAULT_D/<testcase-name>     
 
      --download     Download only. (i.e., do not run testcase.)
      -f             Force download, even if checksums match.
      --no-stereo    Only use one sensor per camera.
      --no-out-video The debug video effictively doubles runtime. Maybe you
                     don't want to wait.
      --serial       Run Openpose in "serial" mode, which may use less GPU RAM
      --disable-openpose  Useful in GUI mode, when editing tracks.
      
      --gt-video     Render ground-truth on the video. Makes no sense in GUI mode.
      --export-training-data
   
      --classifier <name>   Default is '$DEFAULT_CLASSIFIER'. Note that
                            the classifier is disabled if you're exporting
                            training data.

   Testcases:

   * 001_B7-short                     Two people walking through envista.
   * 002_glenbrook-gaze               Track 1 person, but gaze is over the shoulder.
   * 003_glenbrook-cluster            People are tightly clustered together.
   * 004_envistalab-clutter           Lots of tracks in a small space. Frames [1000-1500).
   * 005_envistalab-sitting           More envista, with someone sitting. Frames [0-1000).
   * 006_inspired-tracks              Three people, some moving fast. Frames [6500-6900).
   * 007_inspired-sparse-tracks       Two tracks across many cameras. Frames [0-500).
   * 008_cal-closets-standing         Tracks subject to reprojection error issues.
   * 009_cal-closets-stitching        A track that traces through three camera views. Frames [4400-4700).
   * 010_monterey                     Crowds of people, steep angle, far away. Frames [1100-1900).
   * 011_cc-cincinnati-standing       A bunch of people standing around. Frames [700-850).
   * 012_cc-cincinnati-walking        A group of people walking through store. Frames [1200-1600).
   * 013_cc-cincinnati-occlude        Three people in a tight space, with occlusion. Frames [1900-2100).
   * 014_cc-cincinnati-sitting        A group walking to a table, and some sitting. Frames [5300-5650).   
   * 072_cc_carmel_vonnegut_lay_1     .
   * 073_office_bridget_measuring_1   .
   * 074_office_mark_measuring_1      .
   * 075_office_sam_measuring_1       .
   * 076_office_front_lobby_back_1    .
   * 077_office_front_lobby_back_2    .
   * 078_office_next_to_fern_1        .
   * 079_office_side_door_1           .
   * 080_office_front_lobby_front_1   .

EOF
}

# ------------------------------------------------------------ Parse Commandline

PPWD="$(cd "$(dirname "$0")"; pwd)"
RUN_SH="$(cd "$PPWD/../multiview/multiview_cpp" ; pwd)/run.sh"
TESTCASE_NAME=""
CONFIG="release"
TRACE_MODE=0
SHOW_HELP=0
FORCE_OPTION=0
DOWNLOAD_ONLY=0
HAS_ERROR=0
FPS=0
START_F=0
N_FRAMES=-1
OUT_D=""
BUILD_ONLY=""
NO_STEREO="--no-stereo"
EDITOR=0
NO_OUT_VIDEO=0
SETUP_TESTCASE=0
OPENPOSE_SERIAL=""
OPENPOSE_DISABLED=0
RENDER_GT_ONLY=0
EXPORT_TRAINING=0
DEV_MODE=0

[ "$#" = "0" ]  && SHOW_HELP=1

while [ "$#" -gt "0" ] ; do
    [ "$1" = "debug" ]      && CONFIG=debug && shift && continue
    [ "$1" = "gdb" ]        && CONFIG=gdb && GDB=1 && shift && continue
    [ "$1" = "release" ]    && CONFIG=release && shift && continue
    [ "$1" = "asan" ]       && CONFIG=asan && shift && continue    # sanitizers
    [ "$1" = "usan" ]       && CONFIG=usan && shift && continue
    [ "$1" = "tsan" ]       && CONFIG=tsan && shift && continue
    
    [ "$1" = "build" ]      && BUILD_ONLY="build" && shift && continue    
    [ "$1" = "trace" ]      && TRACE_MODE=1 && shift && continue
    [ "$1" = "gui" ]        && EDITOR=1 && shift && continue
    [ "$1" = "dev" ]        && DEV_MODE=1 && shift && continue
    
    [ "$1" = "-f" ]         && FORCE_OPTION=1 && shift && continue
    [ "$1" = "--download" ] && DOWNLOAD_ONLY=1 && shift && continue
    [ "$1" = "--help" ] || [ "$1" = "-h" ] && SHOW_HELP=1 && shift && continue
    [ "$1" = "--fps" ]      && shift && FPS="$1" && shift && continue
    [ "$1" = "--start" ]    && shift && START_F="$1" && shift && continue
    [ "$1" = "--n-frames" ] && shift && N_FRAMES="$1" && shift && continue
    [ "$1" = "-d" ]         && shift && OUT_D="$1" && shift && continue
    [ "$1" = "--no-stereo" ] && shift && NO_STEREO="--no-stereo" && continue
    [ "$1" = "--no-out-video" ] && shift && NO_OUT_VIDEO=1 && continue
    [ "$1" = "--serial" ]   && shift && OPENPOSE_SERIAL="openpose-serial" && continue
    [ "$1" = "--classifier" ] && shift && DEFAULT_CLASSIFIER="$1" && shift && continue
    
    [ "$1" = "--disable-openpose" ] && shift && OPENPOSE_DISABLED="1" && continue
    [ "$1" = "--gt-video" ] && shift && RENDER_GT_ONLY="1" && continue
    [ "$1" = "--export-training-data" ] && shift && EXPORT_TRAINING="1" && continue
    
    if [ "$TESTCASE_NAME" != "" ] && [ "$SHOW_HELP" = "0" ] ; then
        echo "About to set testcase name to '$1'; however, it was already set to '$TESTCASE_NAME', aborting."
        HAS_ERROR=1
    fi
    
    TESTCASE_NAME="$1"
    shift
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

if [ "$TESTCASE_NAME" = "" ] && [ "$BUILD_ONLY" = "" ] ; then
    echo "Must set the testcase name!"
    HAS_ERROR=1
fi

if [ "$TESTCASE_NAME" != "$(basename "$TESTCASE_NAME")" ] ; then
    echo "Spurious testcase name: '$TESTCASE_NAME'."
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

if [ "$HAS_ERROR" = "1" ] ; then
    echo "Aborting."
    exit 1
fi


# -------------------------------------------------------- Download the testcase

if [ "$OUT_D" = "" ] ; then
    OUT_D="$DEFAULT_D/$TESTCASE_NAME"
fi

remote_hash()
{
    local REMOTE_FILE="$1"
    local LOCAL_FILE="$OUT_D/hash.md5"
    aws s3 cp "$REMOTE_FILE" "$LOCAL_FILE" 1>/dev/null 2>/dev/null || return 0
    [ -f "$LOCAL_FILE" ] && cat "$LOCAL_FILE" || return 0
    rm -f "$LOCAL_FILE" 1>/dev/null 2>/dev/null
}

calc_hash()
{
    local D="$1"
    [ ! -d "$D" ] && echo "<missing>" && return 0
    [ ! -f "$D/ground-truth.json" ] && echo "<missing>" && return 0
    md5sum "$D/ground-truth.json" | awk '{ print $1 }'
}

DO_DOWNLOAD=0
S3_D="s3://perceive-multiview/testcases"
CACHE_D="$HOME/.cache/multiview-data/testcases"
TESTCASE_D="$CACHE_D/$TESTCASE_NAME"
HASH_F="hash.md5"

#echo "hashf = $S3_D/$TESTCASE_NAME/$HASH_F"
#echo "REMOTE = $REMOTE_HASH"

REMOTE_HASH_S="$REMOTE_HASH"
[ "$REMOTE_HASH" = "" ]            && REMOTE_HASH_S="<missing>"
DO_DOWNLOAD=0
BUILD_S=""
[ "$BUILD_ONLY" != "" ]            && BUILD_S=", build-only"

[ ! -f "$TESTCASE_D/manifest.json" ]  && DO_DOWNLOAD=1
[ "$FORCE_OPTION" = "1" ]          && DO_DOWNLOAD=1
[ "$TRACE_MODE" = "1" ]            && export MULTIVIEW_TRACE_MODE=1
[ "$BUILD_ONLY" != "" ]            && DO_DOWNLOAD=0
FPS_S="<default>"
[ ! "$FPS" = "0" ]                 && FPS_S="$FPS"
TRACE_MODE_S=""
[ "$TRACE_MODE" = "1" ]            && TRACE_MODE_S=", trace"
NO_STEREO_S=""
[ "$NO_STEREO" = "" ]              && NO_STEREO_S=", no-stereo"
RANGE_S=""
[ "$START_F" = "0" ] && [ "$N_FRAMES" = "-1" ] && RANGE_S="[0..<end>)"
[ "$START_F" != "0" ] && [ "$N_FRAMES" = "-1" ] && RANGE_S="[${START_F}..<end>)"
[ "$START_F" = "0" ] && [ "$N_FRAMES" != "-1" ] && RANGE_S="[0..${N_FRAMES})"
[ "$START_F" != "0" ] && [ "$N_FRAMES" != "-1" ] \
    && RANGE_S="[${START_F}..$(expr ${START_F} + ${N_FRAMES}))"
GUI_S=""
[ "$EDITOR" = "1" ] && GUI_S=", gui"
DEBUG_VIDEO_S=""
if [ "$NO_OUT_VIDEO" = "1" ] && [ "$EDITOR" = "0" ] ; then
    DEBUG_VIDEO_S="Yes"
else
    DEBUG_VIDEO_S="No"
fi
RENDER_GT_ONLY_S=""
if [ "$RENDER_GT_ONLY" = "1" ] ; then
    RENDER_GT_ONLY_S=", render-gt-only"
fi

OP_SERIAL_S=""
if [ "$OPENPOSE_SERIAL" != "" ] ; then
    OP_SERIAL_S=", openpose-serial"
fi

if [ "$(hostname)" = "zeus" ] && [ "$EDITOR" = "1" ] ; then
    OPENPOSE_DISABLED="1"
fi

OPENPOSE_DISABLED_S=""
if [ "$OPENPOSE_DISABLED" = "1" ] ; then
    OPENPOSE_DISABLED_S=", openpose-disabled"
fi

EXPORT_TRAINING_S=""
if [ "$EXPORT_TRAINING" = "1" ] ; then
    EXPORT_TRAINING_S=", export-training-data"
fi

DEV_MODE_S=""
if [ "$DEV_MODE" = "1" ] ; then
    DEV_MODE_S=", dev-mode"
fi


if [ "$REMOTE_HASH" = "" ] && [ "$LOCAL_HASH" = "<missing>" ] ; then
    echo "Failed to find testcase '$TESTCASE_NAME'."
    echo "Pass '-h' or '--help' to get a list of testcases."
    exit 1
fi

echo "Testcase:   '$TESTCASE_NAME'"
echo "Config:      $CONFIG$TRACE_MODE_S$NO_STEREO_S$GUI_S$BUILD_S$OP_SERIAL_S$OPENPOSE_DISABLED_S$RENDER_GT_ONLY_S$EXPORT_TRAINING_S$DEV_MODE_S"
echo "Frames:      $RANGE_S"
echo "FPS:         $FPS_S"
echo "Debug video: $DEBUG_VIDEO_S"
#echo "Local hash:  $LOCAL_HASH"
#echo "Remote hash: $REMOTE_HASH_S"

if [ "$DO_DOWNLOAD" = "1" ] ; then
    echo
    echo "Downloading"
    echo
    mkdir -p "$TESTCASE_D"
    aws s3 cp --recursive "$S3_D/$TESTCASE_NAME" "$TESTCASE_D"    
fi

# ------------------------------------------------------------- run the testcase

if [ "$DOWNLOAD_ONLY" = "1" ] ; then
    exit 0
fi

if [ "$TESTCASE_NAME" != "" ] ; then
    rm -rf "$OUT_D"
    mkdir -p "$OUT_D"
fi

DEBUG_VIDEO="--debug-video"
TRACKS_F="--tracks-file $TESTCASE_D/ground-truth.json"
PIPE_TEST="--pipeline-testcase"
RUN_MODE="pipeline"

if [ "$DO_DOWNLOAD" = "0" ] && [ "$BUILD_ONLY" = "" ] ; then
    echo "Downloading latest ground-truth.json (it may not exist)"
    aws s3 cp "$S3_D/$TESTCASE_NAME/ground-truth.json" "$TESTCASE_D/ground-truth.json"
fi

if [ ! -f "$TESTCASE_D/ground-truth.json" ] ; then
    TRACKS_F=""
    PIPE_TEST=""
fi

GUI_OPTS=""
if [ "$EDITOR" = "1" ] ; then
    DEBUG_VIDEO=""
    TRACKS_F=""
    PIPE_TEST=""
    RUN_MODE="gui"
    GUI_OPTS="--annotations-directory $S3_D/$TESTCASE_NAME"
fi

export MULTIVIEW_OPENPOSE_DISABLED=0
if [ "$OPENPOSE_DISABLED" = "1" ] ; then
    export MULTIVIEW_OPENPOSE_DISABLED=1
fi

RENDER_GT_ONLY_ARG=""
if [ "$RENDER_GT_ONLY" = "1" ] ; then
    RENDER_GT_ONLY_ARG="--video-gt-only"
fi

CLASSIFIER_ARG0="--pose-classifier"
CLASSIFIER_ARG1="$DEFAULT_CLASSIFIER"

EXPORT_TRAINING_DATA_ARG=""
if [ "$EXPORT_TRAINING" = "1" ] ; then
    EXPORT_TRAINING_DATA_ARG="--export-training-data $TESTCASE_NAME"
    CLASSIFIER_ARG0=""
    CLASSIFIER_ARG1=""
fi

DEV_MODE_ARG=""
if [ "$DEV_MODE" = "1" ] ; then
    DEV_MODE_ARG="--developer"
fi

STDOUT="$OUT_D/stdout"
STDERR="$OUT_D/stderr"

{ stdbuf -oL -eL $RUN_SH $CONFIG $BUILD_ONLY $OPENPOSE_SERIAL $RUN_MODE \
             -m "$TESTCASE_D/manifest.json" \
             -d "$OUT_D" \
             $DEBUG_VIDEO \
             $NO_STEREO \
             $TRACKS_F \
             --target-fps "$FPS" \
             --start "$START_F" \
             --n-frames "$N_FRAMES" \
             $CLASSIFIER_ARG0 $CLASSIFIER_ARG1 \
             $PIPE_TEST \
             $RENDER_GT_ONLY_ARG \
             $EXPORT_TRAINING_DATA_ARG \
             $GUI_OPTS \
             $DEV_MODE_ARG \
             2>&1 1>&3- | tee "$STDERR" 1>&2 ; } 3>&1 | stdbuf -oL -eL tee "$STDOUT"

exit $?


