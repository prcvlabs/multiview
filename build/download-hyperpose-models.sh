#!/bin/bash
# Download the HyperPose ONNX models that multiview knows about
# (see src/perceive/cost-functions/pose-skeleton/params.hpp).
#
# Usage: build/download-hyperpose-models.sh [DIR]
#   DIR defaults to $MULTIVIEW_HYPERPOSE_DIR, else ~/multiview-data/hyperpose.
#   Models go to DIR/models; run multiview with MULTIVIEW_HYPERPOSE_DIR=DIR.
#
# The models are hosted on Google Drive by the HyperPose project
# (hyperpose/scripts/downloader.py). Some links have gone dead upstream;
# those are reported and skipped.

set -euo pipefail

DEST="${1:-${MULTIVIEW_HYPERPOSE_DIR:-$HOME/multiview-data/hyperpose}}"
mkdir -p "$DEST/models"

# name  google-drive-id  md5
MODELS="
openpose-thin-V2-HW=368x432.onnx 1xqXNFPJgsSjgv-AWdqnobcpRmdIu42eh 65e26d62fd71dc0047c4c319fa3d9096
openpose-coco-V2-HW=368x656.onnx 15A0SQyPlU2W-Btcf6Ngi6DY0_1CY50d7 9f422740c7d41d93d6fe16408b0274ef
lopps-resnet50-V2-HW=368x432.onnx 1tb8jnXkoiscfr-ZVydAALg7dtUwAKdEd a6ba26d505c8150d9bf01950143d51d3
ppn-resnet50-V2-HW=384x384.onnx 1qMSipZ5_QMyRuNQ7ux5isNxwr678ctwG 0d1df2e61c0f550185d562ec67a5f2ca
"

N_OK=0
while read -r NAME ID MD5 ; do
    [ -z "$NAME" ] && continue
    F="$DEST/models/$NAME"
    if [ -f "$F" ] && echo "$MD5  $F" | md5sum -c --status - ; then
        echo "have $NAME"
        N_OK=$(( N_OK + 1 ))
        continue
    fi
    URL="https://drive.usercontent.google.com/download?id=$ID&export=download&confirm=t"
    if curl -fsL -o "$F.part" "$URL" && echo "$MD5  $F.part" | md5sum -c --status - ; then
        mv "$F.part" "$F"
        echo "downloaded $NAME ($(du -h "$F" | cut -f1))"
        N_OK=$(( N_OK + 1 ))
    else
        rm -f "$F.part"
        echo "warning: $NAME is not available from Google Drive, skipping"
    fi
done <<< "$MODELS"

[ "$N_OK" = "0" ] && { echo "no models could be downloaded" ; exit 1 ; }
echo "Models are in $DEST/models; set MULTIVIEW_HYPERPOSE_DIR=$DEST"
