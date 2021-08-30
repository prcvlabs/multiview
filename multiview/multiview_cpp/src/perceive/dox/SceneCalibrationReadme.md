
Scene Calibration Readme                {#scene_calibration_readme}
========================

# About #

This README is for information relating to extrinsic calibration procedures.
Everything has been checked on multiview version 4.0-50-gb8444bce:commit:b8444bcea2989214a548b04e73cf759e4d8f60df.

 * Author: Aaron Michaux
 * Date: 2019-10-22

# Getting and Running Multiview #

There's a version of multiview that runs inside a docker container, and is available for work on EC2. This is (currently) a tagged release version.
There's also a `computer-vision-dev_v2` AMI which can be used.

When checking out multiview, be aware that git lets you check out specific tags:

```
git clone -b <tag> git@github.com:perceiveinc/multiview.git
```

And allows you to reset to particular commits:

```
git clone git@github.com:perceiveinc/multiview.git
cd multiview
git reset --hard b8444bcea2989214a548b04e73cf759e4d8f60df
```

## Installing Multiview ##

Multiview has a flexible and sophisticated system for downloading, building, and installing all its dependencies. This is required on a new machine.
Sometimes the build scripts need to be rerun... in cases where library dependencies have been updated.

```
~ > $PERCEIVE_CODE/build/install-files/install.sh --help

   Usage: install.sh [OPTIONS...]

      Builds and installs dependencies for "multiview". Non C++ dependencies
      are installed to the usual places. All C++ dependencies are built and
      installed somewhere under "/opt/llvm/.../".

   Options:

      --version [6.0|7.0|8.0|9.0] Set llvm (clang) version. Default is 6.0.
      --stdc++  [0|1]         Set the c++ standard library. Default is 1.
      --cuda    [0|1]         Use CUDA. Default is 0.
      --no-build-clang        Do not build clang. (Assumes clang previously built.)
      --no-install-cuda       Do not install cuda. (Assumes it has already been done.)
```

You typically run them as follows:

```
sudo $PERCEIVE_CODE/build/install-files/install.sh --version 6.0 --cuda 1 --stdc++ 1
```

There's no need to rebuild clang, or reinstall cuda, if this script has been run at least once _with the correct version_. _*We use version 6.0 currently*_, because of idiosyncrasies with opencv and the cuda compiler. We may move to clang 10.0 in the future. (Because of new features like modules.) That's why the `--version` flag is still active.

## Running Multiview ##

The run script is `$PERCEIVE_CODE/multiview/multiview_cpp/run.sh`. It currently assumes that the install script has been run. _The install script can be made lazy (fast) and idempotent_, meaning that it can be run as part of the run-script. This will solve _any_ issues with dependencies not being built correctly. (This would be achieved by checksuming the build scripts, and installing the checksums with the libraries. When a checksum differs, the build-script is re-run, as well as all follow-on build scripts.)

The run script manages three different products in a large number of different built configurations. The products are:

 * multiview-cli: `$RUN_SH`
 * multiview-gui: `$RUN_SH gui`
 * multiview-testcases: `$RUN_SH test`

The build configurations are simply stacked on as command-line arguments. Typically you only want to run things in `release` mode, but the other options are `asan`, `tsan`, `usan`, and `debug`.

To build and run the cli (command line interface) in release mode:

```
 ~ > $RUN_SH release --help
 
  Usage: multiview-cli [-h] [runs...]

      Run can be one of:

      aruco_like
      cam_pos_optimize
      distortion_calib
      dump_default_params
      face_stitch
      fowlkes_test
      graphcut
      motion_test
      online_stereo_calib
      phase_camera_extrinsic
      phase_stereo_calib
      pipeline
      plane_set_calib
      point_in_triangle_test
      rbf_field
      refine_stereo_calib
      render_floor_map
      run_scraps
      s3
      scene_maker
      sprite_main
      stereo_calib
      stereo_cam_pos_opt
      transitive_rigid_motion
      undistort
```

These are all the cli "programs" in this version of multiview. Each of them has (or should have) a `--help` switch.

# Creating a Scene From Scratch #

The cli front-ends of interest are:

 * phase_camera_extrinsic: Position a single camera relative to an aruco cube.
 * scene_maker: A partly complete program that takes a collection of camera positions, and creates a combined scene file.
 * render_floor_map: A simple program that outputs the floor grid, as well as distorted images with the grid rendered on them.

## Individual extrinsic camera positioning ##

```
 ~ > $PERCEIVE_CODE/multiview/multiview_cpp/run.sh release phase_camera_extrinsic --help

   Usage: phase_camera_extrinsic [OPTIONS...] <image-filenames>*

      -d  <dirname>         Output directory. Default is '/tmp'.

      --cam <camera-id>     Id of the input camera. Something like 'C000042_v1'
      --dense-et0
      --pointwise           Use pointwise method. (Ie., don't use more accurate
                            dense method.) This is useful when the ArucoCube
                            has specular highlights that are uneven between
                            the left and right images.
      --fast-distort        Faster distort method that clips edges of images.
      --full-distort        Distorted/Undistort the full image.
 
```

At this stage note the `--fast-distort` and `--full-distort` switches. In general you will want to use the `--fast-distort` switch, because [1] it is faster, and [2] it restricts detection to well-calibrated regions of the image.

Run `phase_camera_extrinsic` on each aruco image for the entire scene.

```
RUN_SH="$PERCEIVE_CODE/multiview/multiview_cpp/run.sh"
CAM="C0001023_v1"

$RUN_SH release phase_camera_extrinsic --pointwise --dense-et0 \
            -d "/path/to/output/directory" --cam "$CAM" "/path/to/image"
            
# etc...
```

## Making the Provisional Scene ##

The scene-maker front end is as follows:

``` bash
 ~ > $PERCEIVE_CODE/multiview/multiview_cpp/run.sh release scene_maker --help

   Usage: scene_maker [OPTIONS...] [--pos[*]? <camera-id> <view> <position-json-file> <image-file>?]+

      The camera-id with the '*' is the reference camera. If no reference
      is given, then the first camera is taken as the reference camera.

      Image files are *REQUIRED* when doing optimization.

   Options:
 
      -d  <dirname>         Output directory. Default is '/tmp'.

      --scene-id <name>     Mandatory.
      --do-opt              Optimize output. (Default.)
      --no-opt              Do not optimize output.

   Examples:

      > scene_maker --scene-id MyNewScene                              \
           --pos  A  c42_v21       /tmp/C42A/positions.json   \
           --pos  B  c42_v21       /tmp/C42B/positions.json   \
           --pos* A  C0001021_v1   /tmp/C1021A/positions.json \
           --pos  B  C0001021_v1   /tmp/C1021B/positions.json \
           --pos  A  C0001024_v2   /tmp/C1024A/positions.json \
           --pos  B  C0001024_v2   /tmp/C1024B/positions.json

   TODO:

      * Fix issue #40, so that we can run load a scene before calculating stats.
      * Automagically load up a scene, and estimate the bounding plane.
      * Automagically ensure that GUAPI has the correct records in it.

```

This will create a _provisional_ scene file in the output directory.
So, if you do something like this:

``` bash
 ~ > $RUN_SH release scene_maker -d $OUTD --scene-id $SCENE_ID <other arguments>
```

Then you can copy the output into `$PERCEIVE_DATA` as follows:

``` bash
 ~ > DESTF=$PERCEIVE_DATA/multiview/calibration/scenes/$SCENE_ID.json
 ~ > ! [ -f "$DESTF" ] && cp $OUTD/$SCENE_ID.json $DESTF
```

## Testing the Scene ##

Please the generated scene file in `$PERCEIVE_DATA`, and make sure that the environment variable `PERCEIVE_STORE="PERCEIVE_DATA"`. When you run multiview, it will now draw files from `$PERCEIVE_DATA` instead of s3. _This means that you may need to download camera files from s3.

Now download an epoch from `s3`. Multiview has a script just for this purpose:

``` bash
 ~ > $PERCEIVE_CODE/bin/multiview-manifest.py --help
 Usage: multiview-manifest.py [-d <outdir>] <scene> <epoch>

   Examples:

      # Print out the manifest for this scene+epoch
      > multiview-manifest.py glenbrook_2018-11-02_v1 2018-11-02T15:00:00

      # Set up directory 'data' with a manifest and videos, ready for pipeline
      > multiview-manifest.py -d data glenbrook_2018-11-02_v1 2018-11-02T15:00:00
```

(You must install GUAPI for this to work.)

Okay, so download an epoch as follows:

``` bash
 ~ > $PERCEIVE_CODE/bin/multiview-manifest.py -d /path/to/epoch/directory <my-scene-name> 2019-10-01T15:10:00
```

This will create the manifest file in the specified directory, and also download all of the video files.

(The relevant `hardware/scene` file must be in GUAPI.)

Now you can load the scene in multiview as follows:

``` bash
 ~ > $RUN_SH release cuda gui --no-stats --start 10 -d /tmp/out --on-the-fly-still -m /path/to/manifest.json
```

(The no-cuda gui is currently segfaulting, and this is a *bug* that will be fixed shortly.)

Click on:

 * "Disparity" to see disparity maps.
 * The "3d visualizer" to see the point cloud.
 * The "Floor Hist" tab, to see the floor histogram.

### Disparity Check ###

Click on the disparity tab.

Cycle through the cameras by using the Ctrl key. (See the View menu for hints.)

The disparities should look robust.

### Point Cloud Check ###

Turn to the 3d Visualizer, and turn on "Draw Axis", and "Draw xy".
Click the individual checkboxes "Cam 1... 10" to turn on the point clouds for individual cameras.

### Histogram Check ###

Everything comes together in the histogram. Click on "Floor Hist".

## Finishing the Scene ##

We're not done yet. The scene file doesn't know how big it is. That is, the scene file must be edited, and a `"hist-aabb"` key added, that tells multiview where the boundaries of the scene are. Aabb stands for "axis-aligned bounding box", it is an axis-aligned 2D rectangle. The coordinates are entered "[left, top, right, bottom]", just like rectangles in typical 2D graphics software. 

``` json
{
    "type":                  "SceneDescriptionInfo",
    "store-id":               X,
    "scene-key":              "some-scene-key",
    "hist-aabb":             [-8.0, 8.0, 4.0, -6.0],
    ...
    
```

This is a bounding rectangle of "-8.0 <= X <= 4.0" and "-6.0 <= Y <= 8.0".

You should be able to determine the bounds by counting the "xy" squares in the point cloud check above.

## Outputting the Composite Floor Image with Grid ##

The cli has a simple front-end for outputting a composite floor image, with a grid rendered on it. It also outputs distorted images, with the floor grid rendered on them.

``` bash
 ~ > $RUN_SH release render_floor_map --help

   Usage: render_floor_map [OPTIONS...]

      -m <filename>      Manifest filename
      -p <filename>      Parameters file to use. Does not need to be specified.
      -d <dirname>       Directory to save products to. Default is '/tmp'.

      --verbose          Output additional diagnostics.
```

Use it as follows:

``` bash
 ~ > $RUN_SH release render_floor_map -m /path/to/manifest.json -d /path/to/output/directory
```

The output directory will contain `zzz-kolor-floor.png`, as well as a set of distorted images -- one for each camera -- with the floor grid rendered on them.

# Installing the Scene #

When you are completely satisfied with the scene, it can be copied onto `s3`.

# Afterword and Improvements #

There are obviously a lot of rough edges here. Here are some possible improvements:

 * The scene-maker should check for, and possibly create the `hardware/scene` record in GUAPI.
 * The scene-maker can guess the initial floor histogram by doing some elementary clustering analysis. The results will need to be checked, but it should work almost all the time.
 * The scene-maker is currently unaware of the store-id, and sets it to '0' in the scene.json file.
 * The scene-maker annoyingly adds version numbers to the scene. (i.e., like `_v1`.)
 * The scene-maker should be able to figure out if the pipeline has calculated any results on the epoch. If it has, it should _then_ add or increment the version number. The scene-maker could then automagically upload and modify scenes on s3/perceive-data, so long as GUAPI says that no results exist.
 * Data input to the scene-maker is awkward... although pretty simple with the right bash script setup. (For hints see: `$PERCEIVE_DATA/multiview/calibration-data/scenes/california-closets_2019-09-27/run-calibration.sh`.)
 * Data input could be made simpler by codifying file positions. We could then have a process that automagically detects and updates `phase_camera_extrinsic` outputs. The scene-maker, in turn, could have a ncurses interface that lists the possible aruco-position files, and lets you select which ones to use. This could be run live during installation.
 * If we apply the clustering solution above for the `hist-aabb`, then the above ncurses interface could live upload a completed scene within minutes. _It would still need checking visually_, but this would be aided by everything being on `s3`. We only need a way for the multiview gui to be able to list and select _scenes_ and _epoches_ for a given scene. That way a technician can just have the GUI open, and when the scene is available, and an epochs' worth of data is uploaded, then can then click on the epoch, and view it directly.


