
Stereo Camera Calibration           {#stereo_camera_calibration}
=========================

These procedures work with the _*Kyle Aruco Cube*_.
If we build a new cube, then we'll need to make minor changes to both the code and the calibration procedures.

## Capture Some Photos ##

 * Position the ArucoCube on the floor or a table.
 * Stand about 6m away with your camera.
 * Make sure that three faces of the ArucoCube are visible in both sensors of the camera.
 * Take a photo.
 * Now move about 1m toward the cube, and then take a few steps obliquely, so that the cube will appear rotated from the previous photo.
 * Take another photo.
 * Continue until you have _*four photos*_ of the ArucoCube, from 3-6m away. Each photo must be from a different angle. (They don't have to be radically different.) 

## Run Stereo Calibrate ##

Assuming that your photos are called "img0.jpg", "img1.jpg", "img2.jpg", "img3.jpg".

```
RUN_SH="$PERCEIVE_CODE/multiview/multiview_cpp/run.sh"
OUTDIR="/tmp/output_phase-stereo"
SENSOR0=STR00042
SENSOR1=STR00041
OUTPUT_CAM=c42_example_v21

$RUN_SH no-cuda release phase_stereo_calib -d "$OUTDIR" \
        -s0 $SENSOR0 -s1 $SENSOR1 \
        --cam-id "$OUTPUT_CAM" \
        img0.jpg img1.jpg img2.jpg img3.jpg
```

A bunch of output images will appear in $OUTDIR.
It look at them when multiview finishes.

There will also be an output file `$OUTDIR/$OUTPUT_CAM.json`. 
If you are happy with the results, then copy this file to:

```
cp $OUTDIR/$OUTPUT_CAM.json $PERCEIVE_DATA/multiview/calibration/binocular/
```

And copy to aws:

```
aws s3 cp $OUTDIR/$OUTPUT_CAM.json s3://perceive-multiview/calibration/binocular/$OUTPUT_CAM.json
```


