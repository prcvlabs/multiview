#!/bin/bash

set -e

TMPD="$(mktemp -d /tmp/$(basename $0).XXXXXX)"
trap cleanup EXIT
cleanup()
{
    rm -rf "$TMPD"
}

OUTD=""
SENSOR_ID=""
ALLOW_OVERWRITE=0
VERSION=""

[ "$MULTIVIEW_ASSET_S3_BUCKET" = "" ] && \
    MULTIVIEW_ASSET_S3_BUCKET="s3://perceive-multiview"

S3_BASE="$MULTIVIEW_ASSET_S3_BUCKET/calibration-data/sensors"
S3_ASSET_D="$MULTIVIEW_ASSET_S3_BUCKET/calibration/sensors"

# ------------------------------------------------------------------------- Help

show_help()
{
    cat <<EOF

   Usage: $(basename $0) [OPTIONS...] 

      -s <sensor-id> 
      -v <integer>
      -y             Allow overwrite.
      -d <dirname>   Output directory. Default is /tmp/\$SENSOR_ID.

   Example:
   
      # Calibrate sensor 'STR00044'
      > $(basename $0) -s STR00044 -v 1

EOF
}

# -------------------------------------------------------------- Parse Arguments

for ARG in "$@" ; do
    [ "$ARG" = "-h" ] || [ "$ARG" = "--help" ] && show_help && exit 0
done

while [ "$#" -gt "0" ] ; do
    ARG="$1"
    shift
    [ "$ARG" = "-d" ] && OUTD="$1" && shift && continue
    [ "$ARG" = "-s" ] && SENSOR_ID="$1" && shift && continue
    [ "$ARG" = "-y" ] && ALLOW_OVERWRITE=1 && continue
    [ "$ARG" = "-v" ] && VERSION="$1" && shift && continue
    echo "Unknown arguments: '$ARG'"
    exit 1
done

# ------------------------------------------------------------- Unpack Arguments

[ -z ${MULTIVIEW_CODE+x} ] && [ ! -z ${PERCEIVE_CODE+x} ] \
    && MULTIVIEW_CODE="$PERCEIVE_CODE"

RUN_SH="$MULTIVIEW_CODE/multiview/multiview_cpp/run.sh"

[ "$OUTD" = "" ] && OUTD="/tmp/$SENSOR_ID"

# ---------------------------------------------------------------- Sanity checks

! [ -d "$MULTIVIEW_CODE" ] && \
    echo "Could not locate MULTIVIEW_CODE directory '$MULTIVIEW_CODE'" && \
    exit 1

! [ -x "$RUN_SH" ] && \
    echo "Failed to find run.sh script: $RUN_SH" && \
    exit 1

[ "$SENSOR_ID" = "" ] && \
    echo "Must specify a sensor!" && \
    exit 1

! [ "$VERSION" -eq "$VERSION" 2>/dev/null ] && \
    echo "Must specify an integer for the version number: ${VERSION}" && \
    exit 1

(( "$VERSION" < 0 )) && \
    echo "Version number = ${VERSION} must be greater than 0" && \
    exit 1

[ -d "$OUTD" ] && [ "$ALLOW_OVERWRITE" = "0" ] && \
    echo "Cowardly refusing to overwrite output directory '$OUTD'" && \
    exit 1

OUT_FILE_BASE=$S3_ASSET_D/${SENSOR_ID}_v${VERSION}_
aws s3 ls "$OUT_FILE_BASE" > /dev/null  && \
    echo -n "Asset '$OUT_FILE_BASE' already exists on s3; " && \
    echo "please choose another version number." && \
    aws s3 ls "$OUT_FILE_BASE" && \
    exit 1

# ------------------------------------------------------------------------ Setup

[ -d "$OUTD" ] && [ "$ALLOW_OVERWRITE" = "1" ] && rm -rf "$OUTD"
mkdir -p "$OUTD"

# -------------------------------------------------- what as that manifest file?

aws s3 ls "$S3_BASE/$SENSOR_ID" | awk '{ print $2 }' \
    | sed 's,/$,,' > "$TMPD/possible"

[ "$(cat "$TMPD/possible" | wc -l)" = "0" ] && \
    echo "failed to find '$S3_BASE/$SENSOR_ID', aborting" && \
    exit 1

[ "$(cat "$TMPD/possible" | wc -l)" -gt "1" ] && \
    echo "ERROR, found multiple matches to '$SENSOR_ID'" && \
    cat "$TMPD/possible" | sed 's,^,   ,' && \
    exit 1

S3_URI="$S3_BASE/$(cat $TMPD/possible)/manifest.text"

MANIFEST_F="$S3_URI"

nice ionice -c3 "$RUN_SH" release no-cuda distortion_calib \
     -s "$SENSOR_ID" -d "$OUTD" -y -o "$OUTD/$SENSOR_ID.json" "$MANIFEST_F"

OUT_FILE="${SENSOR_ID}_v${VERSION}_$(md5sum "$OUTD/$SENSOR_ID.json" | awk '{ print $1 }').json"
cp "$OUTD/$SENSOR_ID.json" "$TMPD/${OUT_FILE}"

aws s3 cp "$TMPD/${OUT_FILE}" $S3_ASSET_D/

cat <<EOF

   Results written to: $OUTD
   Uploaded to $S3_ASSET_D/$OUT_FILE

EOF

