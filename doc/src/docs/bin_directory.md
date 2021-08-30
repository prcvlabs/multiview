
## Camera Calibration

The following scripts are used for calibrating cameras:

| Script                         | Description                                           |
|--------------------------------|-------------------------------------------------------|
| `bin/find-corners.py`          | Runs OpenCV's corner detection on an image or video.  |
| `bin/render-corners.py`        | Draws detected corners, possibly making a video file. |
| `bin/distortion-calibration.py`| Estimate distortion parameters.                       |
| `bin/intrinsic-calibration.py` | Estimate intrinsic camera parameters.                 |
| `bin/stereo-calibration.py`    | Calibrates two cameras relative to each other.        |

Each of these scripts has a `--help` switch that gives detailed usage information _and_
usage examples.

```bash
 > bin/find-corners.py --help
```

TODO, all of this is to be moved to the calibration HOWTO

A calibration workflow might look as follows:

```bash
# First take some video footage of a calibration grid. Ideally use the
# large 15x10 grid, lay it flat on the floor, and then start capturing.
# Move the camera aroudn to _paint_ the grid onto all areas of the camera
# sensor, and at many different angles and distances.
#
# Say you capture two 10min videos called 01.h264 and 02.h264

# Find corners. This is slow.
./find-corners.py --dual --nx 15 --ny 10 -o 01_corners.json 01.h264
./find-corners.py --dual --nx 15 --ny 10 -o 02_corners.json 02.h264

# Find distortion parameters
./distortion-calibration.py --feedback -o distort.json 01_corners.json 02_corners.json

# Find stereo calibration
./stereo-calibration.py TBA
```

#### A Note on Distortion versus Intrinsic Calibration

Distortion calibration should only be used on distorted images.
If the input video is not distorted, then calibration becomes
under-determined, and the results unpredictable. Note that
distortion calibration involves finding both the distortion parameters
_and_ the intrinsic parameters.

Intrinsic calibration should only be used on undistorted images.
(Many cameras automatically undistort images in the firmware.)
If intrinsic calibration is done on distorted images, then the
algorithm will likely be confused by the poor match between
the calibration grid model, and the positions of corners detected
in the input images. This results will be poor.

## Guis

So far there is only one GUI, which is a work in progress.

| Gui                            | Description                                           |
|--------------------------------|-------------------------------------------------------|
| `gui/camera-tweaker.py`        | GUI for viewing and tweaking calibration.             |



