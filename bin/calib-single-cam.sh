#!/bin/bash

set -e

# ------------------------------------------------------------------------- Help

show_help()
{
    cat <<EOF

   Usage: $(basename $0) <camera-id> <left-sensor> <right-sensor>

      If required, downloads calibration images from s3 into
      \$PERCEIVE_DATA. Then performs calibration on all images
      except those in a directory named '/bad/'. Calibration
      is then coped to the correct place in \$PERCEIVE_DATA.

   Example:
   
      # Calibrate camera 'C0001036_v5'
      > $(basename $0) C0001036_v5 STR00051 STR00081

EOF
}

# ------------------------------------------------------------- Unpack Arguments

for ARG in "$@" ; do
    [ "$ARG" = "-h" ] || [ "$ARG" = "--help" ] && show_help && exit 0
done

[ ! "$#" = "3" ] \
    && echo "Expected 3 arguments, but got $#, aborting" \
    && exit 1

CAM="$1"
SENSOR0="$2"
SENSOR1="$3"
OPTS=cuda

CAMD="$PERCEIVE_DATA/multiview/calibration-data/cameras"
RUN_SH="$MULTIVIEW_CODE/multiview/multiview_cpp/run.sh"
OUTDIR="/tmp/zzz-output_phase-stereo_${CAM}"

# ------------------------------------------------------ Sanity checks

! [ -d "$MULTIVIEW_CODE" ] && \
    echo "Could not locate MULTIVIEW_CODE directory '$MULTIVIEW_CODE'" && \
    exit 1

! [ -x "$RUN_SH" ] && \
    echo "Failed to find run.sh script: $RUN_SH" && \
    exit 1

! [ -d "$PERCEIVE_DATA" ] && \
    echo "Failed to find \$PERCEIVE_DATA=\"$PERCEIVE_DATA\"" && \
    exit 1

# --------------------------------------------------------------- download files

download_images()
{    
    CAM="$1"

    if ! [ -d "$CAMD/$CAM" ] ; then
          mkdir -p "$CAMD/$CAM"
          aws s3 cp --recursive \
              "s3://perceive-multiview/calibration-data/cameras/$CAM" \
              "$CAMD/$CAM"
    fi
}

# -------------------------------------------------------------- download sensor

download_sensor()
{    
    SENSOR="$1"

    F="$PERCEIVE_DATA/multiview/calibration/sensors/$SENSOR.json"
    if [ ! -f "$F" ] ; then
        mkdir -p "$PERCEIVE_DATA/multiview/calibration/sensors/"
        aws s3 cp \
            "s3://perceive-multiview/calibration/sensors/$SENSOR.json" \
            "$F"
    fi
}

# ------------------------------------------------------------------- Let's Run!

process_cam()
{
    CAM="$1"
    SENSOR0="$2"
    SENSOR1="$3"

    echo "Processing camera $CAM, with sensors [$SENSOR0, $SENSOR1]"
    
    download_images "$CAM"
    download_sensor "$SENSOR0"
    download_sensor "$SENSOR1"
    
    IN=$(find "$CAMD/$CAM" -type f -name '*.jpg' | grep -v bad | sort)
    
    echo "Setting output directory to: '$OUTDIR'"
    
    mkdir -p "$OUTDIR"
    rm -f $OUTDIR/*.png
    nice ionice -c3 $RUN_SH $OPTS release phase_stereo_calib \
         -d "$OUTDIR" \
         -s0 $SENSOR0 -s1 $SENSOR1 \
         --cam-id "$CAM" \
         --aruco-cube kyle-cube \
         $IN \
         2>&1 | tee $OUTDIR/log.text \
        && cp $OUTDIR/$CAM.json $PERCEIVE_DATA/multiview/calibration/binocular/$CAM.json \
        && return 0
    return 1
}

make_composite_image()
{
    cd $OUTDIR
    N=$(find . -name "xxx_qt_*cam0_*.png" -type f | wc -l)
    
    I=0
    while (( $I < $N )) ; do
        convert xxx_qt_${I}-cam0_FINAL.png -resize 640x480 ${I}_l.png
        convert xxx_qt_${I}-cam1_FINAL.png -resize 640x480 ${I}_r.png
        convert +append ${I}_l.png ${I}_r.png ${I}_lr.png
        rm -f ${I}_l.png
        rm -f ${I}_r.png
        I=$(expr $I + 1)
    done

    convert -append $(find . -name "*_lr.png" -type f | sort) 000_composite.png
    rm *_lr.png

    echo "Composite image saved to: $(pwd)/000_composite.png"
}

nice ionice -c3 $RUN_SH $OPTS release build

process_cam "$CAM" "$SENSOR0" "$SENSOR1" \
            && cat <<EOF

Calibration SUCCESSFUL!!!

This is the upload command:
     
     > aws s3 cp $OUTDIR/$CAM.json s3://perceive-multiview/calibration/binocular/$CAM.json
EOF

exit $?
