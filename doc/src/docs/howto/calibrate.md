
# Calibration Howto

There are two parts to calibration:

 * Intrinsic calibration -- the parameters "internal" to the camera.
 * Extrinsic calibration -- locating the camera (rotation and translation) relative some coordinate system.

The intrinsic calibration either be a "distortion" calibration `A` matrix (if the camera produces distorted images)
or a vanilla "intrinsic" calibration `K` matrix (if the camera already produces undistorted images.
The `A` matrix takes an image pixel, and transforms it to a ray in 3D emanating from the camera center.
The _inverse_ `K` matrix does the same.

## Calibrating the Cameras ##

Intrinsic calibration is done from a video sequence of a _flat_ calibration grid. (We use a checkerboard.)
Under the hood, some math is used to figure out what `A` or `K` matrix could have been used to generate the
observed images of the calibration grid.
This process is time-consuming, and finicky.
To get good results, it is important that _the calibration video "paints" the entire vieable area of the camera with
the grid at many different orientations and distances_.

### Calibration Video Naming Conventions ###

Calibration videos use the following format:

{CAMERA_ID}-{LENS}\_calibrate\_{VIDEO_NUMBER}\_{LENGTH}\_{USER}.h264

 * The CAMERA_ID should be read from the camera "cat /cat/serial_number".
 * LENS is a short piece of text that identifies the lens type.
 * Calibration jobs can involve multiple videos. If there's a series of videos
(they don't have to be in order), then VIDEO_NUMBER gives a way to
distinguish between them. If there's just one video, set VIDEO_NUMBER=0
 * LENGTH should be the length of the video. For example, LENGTH=5min.
 * USER is the user who made the video. (Not the user account on the raspberry pi.) This way someone knows who made the video.

For example: `C000000035-WA_calibrate_01_5min_amichaux.h264`, where `WA` is the lens type (wide-angle).

### Collecting Calibration Videos ###

 * `ssh` onto the desired camera.
 * Capture 5min of video like so:
```bash
raspivid -v -w 1792 -h 672 -t $(expr 5 \* 60000) -3d sbs -o \
   $(cat /etc/serial_number)-{LENS}_calibrate_{VIDEO_NUMBER}_5min_{USER}.h264
```
 * Copy the videos to `{PERCEIVE_DATA}/calibration/calibration_videos/{CAMERA_ID}-{LENS}/

Collect 25min of video for each camera.

## Distorting and Undistorting

#### Distorting and undistorting single points

See code sample: `samples/test_rational_distort_undistort_point.py`

```python
#include "samples/test_rational_distort_undistort_point.py"
```

#### How to undistort/distort whole images, including resizing

See code sample: `samples/test_rational_distort_redistort.py`

```python
#include "samples/test_rational_distort_redistort.py"
```

#### About the rational distortion model

The rational distortion model takes 18 parameters which form the rows
of a 3x6 matrix called `A`.  The `A` matrix takes a pixel location
(call it `x`) in a *distorted* image, and calculates the incident ray
(normalized co-ordinate) in 3D that generated that pixel. Put another
way, when the camera images a point `X` in the real world, a ray is
generated that joins `X` to the focal point of the camera. Call this
ray `x_u`. However, the lens *bends* the ray, so that when it hits the
camera image sensor, the affected pixel is no longer on the ray
through the `X` and the camera center. Matrix `A` takes the pixel `x`,
and tells you the ray `x_d` that "generated" the pixel.

```
x_d = A x
```

Note that `x` represents a pixel location, but the equation `A x`
requires `x` to be 6D column vector. If the pixel location is `[i, j]`
(i.e., x-coordinate and y-coordiate `j`) then `x = [i^2 j^2 i j
1]^t`. This makes each row of `A` into the equation for a conic.


