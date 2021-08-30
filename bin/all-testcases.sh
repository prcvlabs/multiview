#!/bin/bash

# ------------------------------------------------------------------------ Setup

TMPD=$(mktemp -d /tmp/"$(basename $0).XXXXX")
trap cleanup EXIT
cleanup()
{
    rm -rf "$TMPD"
}

# ------------------------------------------------------------------------- Help

PPWD="$(cd "$(dirname "$0")" ; pwd)"
DEFAULT_D="$HOME/TMP/testcase-output"

show_help()
{
    cat <<EOF

   Usage: $(basename $0) [OPTIONS...]

      The possible testcase names are given below.

   Options:
     
      -h|--help       Show help.
      --fps <number>  Frame per second.
      -d <path>       Output directory. Default is: $DEFAULT_D  
      --collate       Do not run the testcases, but attempt to collate results.
      --upload        Upload collated results to spreadsheet. Implies --collate
      --print         Do not run the testcases, but print the commands.   

EOF
}

# ------------------------------------------------------------ Parse Commandline

TESTCASE_SH="$PPWD/testcase.sh"

TRACE=""
SHOW_HELP=0
HAS_ERROR=0
FPS=10
OUT_D="$DEFAULT_D"
CONFIG="release"
COLLATE_ONLY=0
PRINT_JOBS=0
DO_UPLOAD=0

while [ "$#" -gt "0" ] ; do

    [ "$1" = "debug" ]      && CONFIG=debug && shift && continue
    [ "$1" = "gdb" ]        && CONFIG=debug && shift && continue
    [ "$1" = "release" ]    && CONFIG=release && shift && continue
    [ "$1" = "asan" ]       && CONFIG=asan && shift && continue    # sanitizers
    [ "$1" = "usan" ]       && CONFIG=usan && shift && continue
    [ "$1" = "tsan" ]       && CONFIG=tsan && shift && continue
    
    [ "$1" = "trace" ]      && shift && TRACE="trace" && continue
    [ "$1" = "--fps" ]      && shift && FPS="$1" && shift && continue
    [ "$1" = "-d" ]         && shift && OUT_D="$1" && shift && continue
    [ "$1" = "-h" ]         && shift && SHOW_HELP=1 && continue
    [ "$1" = "--help" ]     && shift && SHOW_HELP=1 && continue
    [ "$1" = "--collate" ]  && shift && COLLATE_ONLY=1 && continue
    [ "$1" = "--upload" ]    && shift && DO_UPLOAD=1 && COLLATE_ONLY=1 && continue
    [ "$1" = "--print" ]    && shift && PRINT_JOBS=1 && continue
    [ "$1" = "--print-only" ] && shift && PRINT_JOBS=1 && continue
    
    echo "Unknown argument '$1'"
    HAS_ERROR=1
    shift
done

# ----------------------------------------------------------------------- Sanity

if [ "$SHOW_HELP" = "1" ] ; then
    show_help
    exit 0
fi

if [ ! -f "$TESTCASE_SH" ] ; then
    echo "Failed to find '$TESTCASE_SH'."
    HAS_ERROR=1
fi

if [ ! "$FPS" -eq "$FPS" 2>/dev/null ] ; then
    echo "Spurious frames per second = '$FPS'."
    HAS_ERROR=1
elif [ "$FPS" -lt "0" ] || [ "$FPS" -gt "60" ] ; then
    echo "Spurious frames per second = '$FPS'."
    HAS_ERROR=1
fi

if [ "$OUT_D" = "" ] ; then
    echo "Output directory must be specified!"
    HAS_ERROR=1
fi

if [ "$HAS_ERROR" = "1" ] ; then
    echo "Aborting."
    exit 1
fi

[ "$OUT_D" = "$DEFAULT_D" ] && OUT_D="$DEFAULT_D/fps=${FPS}"

# -------------------------------------------------------------------------- Run

get_commit()
{
    cd "$PPWD"
    git log -1 | grep commit | awk '{ print $2 }' | head -n 1
}

get_tag()
{
    git describe --exact-match --tags $(git log -n1 --pretty='%h') 2>/dev/null \
        && return 0
    git describe --tags
}

get_testcase_d()
{
    echo "$OUT_D/${1}_fps=${FPS}"
}

NOW="$(date '+%Y-%m-%d %H:%M:%S')"
OUT_F="${OUT_D}/run_$(get_tag)_fps=${FPS}_results.text"

"$PPWD/testcase.sh"  | grep '*' | awk '{ print $2 }' | while read L ; do
    echo stdbuf -oL -eL nice ionice -c3 "$PPWD/testcase.sh" $TRACE $CONFIG -f --no-stereo -d "$(get_testcase_d "$L")" --fps $FPS $L
done > "$TMPD/cmds"

if [ "$PRINT_JOBS" = "1" ] ; then
    cat "$TMPD/cmds"
    exit 0
fi

if [ "$COLLATE_ONLY" = "0" ] && [ "$PRINT_JOBS" = "0" ] ; then
    cat "$TMPD/cmds" | bash
fi

# ---------------------------------------------------------------------- Collate

has_stdout()
{
    local STDOUT="$(get_testcase_d "$1")/stdout"
    [ -f "$STDOUT" ] && return 0
    return 1
}

has_ground_truth()
{
    local STDOUT="$(get_testcase_d "$1")/stdout"
    cat "$STDOUT" | grep "tracks_filename" | grep -q "ground-truth.json" \
        && return 0
    return 1
}

get_status()
{
    local STDOUT="$(get_testcase_d "$1")/stdout"
    local STDERR="$(get_testcase_d "$1")/stderr"
    cat "$STDOUT" "$STDERR" | grep -E "(FATAL)|(ERROR)" | \
        tail -n 1 | cut -d " " -f 3-
}

is_success()
{
    local STDOUT="$(get_testcase_d "$1")/stdout"
    [ ! -f "$STDOUT" ] && return 1
    tail -n 10 "$STDOUT" | grep -q 'pipeline-execute done' && return 0
    return 1
}

get_field()
{
    local STDOUT="$(get_testcase_d "$1")/stdout"
    if [ -f "$STDOUT" ] && tail -n 10 "$STDOUT" | grep -q 'pipeline-execute done' ; then
        ARG="$3"
        cat "$STDOUT" | grep "$2" | tail -n 1 | awk "{ print \$${ARG} }" \
            | tr '\n' ' ' | sed -e 's,^\s*,,g' | sed -e 's,\s*$,,g'
    else
        echo "*"
    fi    
}

get_n_frames()
{
    get_field $1 'n-frames:' 2
}

get_n_cams()
{
    get_field $1 'number of cameras:' 4
}

get_true_positives()
{
    get_field $1 'true positives:' 4
}

get_false_positives()
{
    get_field $1 'false positives:' 4
}

get_false_negatives()
{
    get_field $1 'false negatives:' 4
}

get_f1()
{
    get_field $1 'f1:' 3
}

get_total_id_switches()
{
    get_field $1 'total-id-switches:' 3
}

get_total_frags()
{
    get_field $1 'total-frags:' 3
}

get_total_tp_distance()
{
    get_field $1 'total-tp-distances:' 3
}

get_total_tp_iou()
{
    get_field $1 'total-tp-iou:' 3
}

get_total_mostly_tracked()
{
    get_field $1 'total-mostly-tracked:' 3
}

get_total_partly_tracked()
{
    get_field $1 'total-partly-tracked:' 3
}

get_total_mostly_lost()
{
    get_field $1 'total-mostly-lost:' 3
}

get_gt_tracks()
{
    get_field $1 'gt-tracks:' 4
}

get_detected_tracks()
{
    get_field $1 'detected tracks:' 5
}

get_gaze_proportion()
{
    get_field $1 'proportion with gaze:' 5
}

get_average_theta_bias()
{
    get_field $1 'average theta bias:' 5
}

get_average_theta_error()
{
    get_field $1 'theta error stddev:' 5
}

get_runtime()
{
    get_field $1 'total-seconds:' 2 | sed 's,s,,g'
}

get_per_cam_frame()
{
    get_field $1 'seconds per (frame/cam):' 4 | sed 's,s,,g'
}

print_error_line()
{
    printf "| %-30s %-81s |\n" "$1" "$2"
}

print_output_line()
{
    local L="$1"
        
    NFRAMES="$(get_n_frames "$L")"

    TP="$(get_true_positives "$L")"
    FP="$(get_false_positives "$L")"
    FN="$(get_false_negatives "$L")"

    ID="$(get_total_id_switches "$L")"
    FRG="$(get_total_frags "$L")"

    DIST="$(get_total_tp_distance "$L")"
    IOU="$(get_total_tp_iou "$L")"

    MT="$(get_total_mostly_tracked "$L")"
    PT="$(get_total_partly_tracked "$L")"
    ML="$(get_total_mostly_lost "$L")"

    echo "$TP"   >> "$TMPD/tp"
    echo "$FP"   >> "$TMPD/fp"
    echo "$FN"   >> "$TMPD/fn"
    echo "$DIST" >> "$TMPD/dist"
    echo "$IOU"  >> "$TMPD/iou"
    echo "$ID"   >> "$TMPD/id"
    echo "$FRG"  >> "$TMPD/frag"
    echo "$MT"   >> "$TMPD/mt"
    echo "$PT"   >> "$TMPD/pt"
    echo "$ML"   >> "$TMPD/ml"
    
    
    TPR="*****"
    PPV="*****"
    F1="*****"
    PR=" ****"
    TB=" *****"
    TE=" *****"
    if is_success "$L" ; then
        if [ "$TP" -eq "$TP" 2>/dev/null ] && [ "$FN" -eq "$FN" 2>/dev/null ] ; then
            TPR="$(printf %5.3f $(echo "scale=5; $TP / ($TP + $FN)" | bc))"
            PPV="$(printf %5.3f $(echo "scale=5; $TP / ($TP + $FP)" | bc))"

            echo "scale=5; $TPR * $NFRAMES" | bc >> "$TMPD/tpr"
            echo "scale=5; $PPV * $NFRAMES" | bc >> "$TMPD/ppv"
            echo "$NFRAMES"                      >> "$TMPD/n-successful-frames"
        fi
        F1="$(printf %5.3f $(get_f1 "$L"))"
        PR="$(printf %4.2f $(get_gaze_proportion "$L"))"
        TB="$(printf %+6.1f $(get_average_theta_bias "$L"))"
        TE="$(printf %+6.1f $(get_average_theta_error "$L"))"
    fi
    
    GT="$(get_gt_tracks "$L")"
    DD="$(get_detected_tracks "$L")"

    NCAMS="$(get_n_cams "$L")"
    RUNTIME="$(get_runtime "$L")"
    PER_CAM_FRAME="$(get_per_cam_frame "$L")"
    
    # Save the total number of frames for later
    echo "$NFRAMES" >> "$TMPD/n-frames"
    echo "$(echo "scale=5; $PER_CAM_FRAME * $NFRAMES" | bc)" >> "$TMPD/per-cam"

    #        Test   Pec Rec F1 #Tracks MT  PT  ML
    printf "| %-30s %s  %s  %s %4s %3s %3s %3s %3s  %s %s  %s  %4d  %3d %7.1fs |\n" "$L" "$PPV" "$TPR" "$F1" "$GT" "$DD" "$MT" "$PT" "$ML" "$PR" "$TB" "$TE" "$NFRAMES" "$NCAMS"  "$RUNTIME"
}

# --- This is the actual loop that generates the table

BANNER_LINE="+ ---------------------------------------------------------------------------------------------------------------- +"

echo "" > "$OUT_F"
echo "$BANNER_LINE"  >> "$OUT_F"
echo "| @see \$PERCEIVE_DATA/computer-vision/documentation/multiview/html/de/d22/measuring_tracker_accuracy.html          |" >> "$OUT_F"
echo "$BANNER_LINE"  >> "$OUT_F"
printf "| %-26s %4s %4s   %4s  %-6s  MT  PT  ML  \\u03B8-%%  \\u03B8-bias   \\u03B8-err Frams Cams  Runtime |\n" "Testcase" "Precision" "Recall" "F1" "#Tracks" >> "$OUT_F"
echo "$BANNER_LINE" >> "$OUT_F"

"$PPWD/testcase.sh"  | grep '*' | awk '{ print $2 }' | while read L ; do
    if ! has_stdout "$L" ; then
        print_error_line "$L" "job hasn't run"
        if [ "$DO_UPLOAD" = "1" ] ; then            
            touch "$TMPD/has-error"
        fi
    elif ! has_ground_truth "$L" ; then
        print_error_line "$L" "no ground-truth annotations available"
    elif ! is_success "$L" ; then
        print_error_line "$L" "$(get_status "$L")"
    else
        print_output_line "$L"
    fi
done >> "$OUT_F"

TOT_FRAMES=$(expr $(cat "$TMPD/n-frames" | tr '\n' '+' | sed 's,+$,,' | sed 's,+, + ,g'))

TOT_SUCCESSFUL_FRAMES=$(expr $(cat "$TMPD/n-successful-frames" | tr '\n' '+' | sed 's,+$,,' | sed 's,+, + ,g'))

PER_CAM_FRAME="$(echo "scale=5;  ($(cat "$TMPD/per-cam" | tr '\n' '+' | sed 's,+$,,' | sed 's,+, + ,g')) / $TOT_FRAMES" | bc)"
PER_CAM_FRAME_S="$(printf %5.3fs "$PER_CAM_FRAME")"

AVG_PPV="$(echo "scale=5;  ($(cat "$TMPD/ppv" | tr '\n' '+' | sed 's,+$,,' | sed 's,+, + ,g')) / $TOT_SUCCESSFUL_FRAMES" | bc)"
AVG_PPV_S="$(printf %5.3f "$AVG_PPV")"

AVG_TPR="$(echo "scale=5;  ($(cat "$TMPD/tpr" | tr '\n' '+' | sed 's,+$,,' | sed 's,+, + ,g')) / $TOT_SUCCESSFUL_FRAMES" | bc)"
AVG_TPR_S="$(printf %5.3f "$AVG_TPR")"

TOT_TP="$(echo "scale=5; ($(cat "$TMPD/tp" | tr '\n' '+' | sed 's,+$,,' | sed 's,+, + ,g'))" | bc)"
TOT_FP="$(echo "scale=5; ($(cat "$TMPD/fp" | tr '\n' '+' | sed 's,+$,,' | sed 's,+, + ,g'))" | bc)"
TOT_FN="$(echo "scale=5; ($(cat "$TMPD/fn" | tr '\n' '+' | sed 's,+$,,' | sed 's,+, + ,g'))" | bc)"

SUM_DIST="$(echo "scale=5; ($(cat "$TMPD/dist" | tr '\n' '+' | sed 's,+$,,' | sed 's,+, + ,g'))" | bc)"
TOT_ID="$(echo "scale=5; ($(cat "$TMPD/id" | tr '\n' '+' | sed 's,+$,,' | sed 's,+, + ,g'))" | bc)"
TOT_FRAG="$(echo "scale=5; ($(cat "$TMPD/frag" | tr '\n' '+' | sed 's,+$,,' | sed 's,+, + ,g'))" | bc)"
TOT_MT="$(echo "scale=5; ($(cat "$TMPD/mt" | tr '\n' '+' | sed 's,+$,,' | sed 's,+, + ,g'))" | bc)"
TOT_PT="$(echo "scale=5; ($(cat "$TMPD/pt" | tr '\n' '+' | sed 's,+$,,' | sed 's,+, + ,g'))" | bc)"
TOT_ML="$(echo "scale=5; ($(cat "$TMPD/ml" | tr '\n' '+' | sed 's,+$,,' | sed 's,+, + ,g'))" | bc)"
TOT_TRACKS="$(echo "scale=5; $TOT_MT + $TOT_PT + $TOT_ML" | bc)"

MOTP="$(echo "scale=5; ($(cat "$TMPD/dist" | tr '\n' '+' | sed 's,+$,,' | sed 's,+, + ,g')) / $TOT_TP" | bc)"
MOTP_S="$(printf %5.3f "$MOTP")m"

MOTA="$(echo "scale=5; 100.0 * (1.0 - ($TOT_FN + $TOT_FP + $TOT_ID) / ($TOT_TP + $TOT_FN))" | bc)"
MOTA_S="$(printf %.2f%% "$MOTA")"

MODP="$(echo "scale=5; ($(cat "$TMPD/iou" | tr '\n' '+' | sed 's,+$,,' | sed 's,+, + ,g')) / $TOT_TP" | bc)"
MODP_S="$(printf %5.3f "$MODP")m^2"

MODA="$(echo "scale=5; 100.0 * (1.0 - ($TOT_FN + $TOT_FP) / ($TOT_TP + $TOT_FN))" | bc)"
MODA_S="$(printf %.2f%% "$MODA")"

MT="$(echo "scale=5; 100.0 * ($TOT_MT / $TOT_TRACKS)" | bc)"
MT_S="$(printf %.2f%% "$MT")"
PT="$(echo "scale=5; 100.0 * ($TOT_PT / $TOT_TRACKS)" | bc)"
PT_S="$(printf %.2f%% "$PT")"
ML="$(echo "scale=5; 100.0 * ($TOT_ML / $TOT_TRACKS)" | bc)"
ML_S="$(printf %.2f%% "$ML")"

SZ="-87s"
echo "$BANNER_LINE" >> "$OUT_F"
printf "| Date:            %$SZ         |\n" "$NOW"             >> "$OUT_F"
printf "| Tag:             %$SZ         |\n" "$(get_tag)"       >> "$OUT_F"
printf "| Commit:          %$SZ         |\n" "$(get_commit)"    >> "$OUT_F"
printf "| FPS:             %$SZ         |\n" "$FPS"             >> "$OUT_F"
printf "| Frames:          %$SZ         |\n" "$TOT_FRAMES"      >> "$OUT_F"
printf "| Speed:           %$SZ         |\n" "$PER_CAM_FRAME_S" >> "$OUT_F"
printf "| Avg-precision:   %$SZ         |\n" "$AVG_PPV_S"       >> "$OUT_F"
printf "| Avg-recall:      %$SZ         |\n" "$AVG_TPR_S"       >> "$OUT_F"
printf "| MOTP:            %$SZ         |\n" "$MOTP_S"          >> "$OUT_F"
printf "| MOTA:            %$SZ         |\n" "$MOTA_S"          >> "$OUT_F"
printf "| MODP:            %$SZ         |\n" "$MODP_S"          >> "$OUT_F"
printf "| MODA:            %$SZ         |\n" "$MODA_S"          >> "$OUT_F"
printf "| Mostly Tracked:  %$SZ         |\n" "$MT_S"            >> "$OUT_F"
printf "| Partly Tracked:  %$SZ         |\n" "$PT_S"            >> "$OUT_F"
printf "| Mostly Lost:     %$SZ         |\n" "$ML_S"            >> "$OUT_F"
printf "| ID Switch:       %$SZ         |\n" "$TOT_ID"          >> "$OUT_F"
printf "| Track Frags:     %$SZ         |\n" "$TOT_FRAG"        >> "$OUT_F"

echo "$BANNER_LINE" >> "$OUT_F"
echo "" >> "$OUT_F"

cat "$OUT_F"

echo "Output saved to: '$OUT_F'"
echo

if [ "$DO_UPLOAD" = "1" ] ; then
    echo "Starting upload..."
    if [ -f "$TMPD/has-error" ] ; then
        echo "Some jobs haven't been run, aborting --upload"
        exit 1
    fi
    "$PPWD/lib/upload-to-testing-review-spreadsheet.py" "$OUT_F"
    RET="$?"
    if [ "$RET" = "0" ] ; then
        echo "Done!"
    else
        echo "<ERROR> Operation failed =("
        exit 1
    fi
fi
