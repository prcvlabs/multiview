
Generating Presentation Videos           {#generating_presentation_videos}
==============================

The following script shows how to run `multiview` to generate presentaion videos.

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.sh}
D="$(cd "$(dirname "$0")" ; pwd)"
PIPELINE="$PERCEIVE_CODE/multiview/multiview_cpp/run.sh release pipeline"
VIDEO="path/to/manifest.json"
FOWLKES_ONLY=0

# Command (1): Generates (and saves) stats and fowlkes data. Also runs the tracker.
if (( $FOWLKES_ONLY == 0 )) || ! [ -f $D/fowlkes.data ] ; then
    $PIPELINE -y -o $D/out.json \
              -d $D \
              --generate-stats $D/stats.data \
              --create-video \
              --present-video \
              --gen-fowlkes $D/fowlkes.data \
              -p $D/params.json \
              $VIDEO
fi

# Command (2): Runs the tracker from pre-saved `stats` and `fowlkes` data
(( $FOWLKES_ONLY == 1 )) &&
   $PIPELINE -y -o $D/out.json \
             -d $D \
             -s $D/stats.data \
             --create-video \
             --use-fowlkes $D/fowlkes.data \
             -p $D/params.json \
             $VIDEO
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Command (1) takes a long time. Generating the _stats_ and _fowlkes data_ involves
creating a histogram for every frame of the video. Currently this takes 3 or so
seconds per frame. (We'll soon have realtime speeds.)

Command (2) is very fast for short videos. The two stage process is useful
because you may want to tweak the tracker parameters, and rerun the second
command. Information on the tracker parameters are given in [Fowlkes Tracker](@ref fowlkes_tracker).

## Working Example Script ##

Go run `$PERCEIVE_DATA/computer-vision/presentation-stuff/B2/zap.sh`.
