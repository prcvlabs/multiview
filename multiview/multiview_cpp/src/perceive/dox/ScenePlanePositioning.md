
Scene Plane Positioning           {#scene_plane_positioning}
=======================

# Turn the input image into a video #

The new version should just take still images as input. (One per camera.)

```
cd /tmp
mkdir z
cd z

# We're resizing 'cause the optimization procedure gets real slow on those massive images.
convert $PERCEIVE_DATA/computer-vision/experiments/camera_positioning/fischer_experiment_1/fisher-pos2.jpg -resize 2048x768 001.png

# Create 119 more images
I=2 ; while (( $I < 120 )) ; do cp 001.png $(printf %03d.png $I) ; ((I++)) ; done

# Create a video from those images
ffmpeg -i %03d.png -c:v libx264 -crf 20 fisher-1_c42.mp4

# Move the video back in place.
mv fisher-1_c42.mp4 $PERCEIVE_DATA/computer-vision/experiments/camera_positioning/fischer_experiment_1/

# Clean up, like a good citizen
cd /tmp
rm -rf /tmp/z
```

# (*) Set up the manifest file (*)

The new version should need a manifest file. But this is for loading tweaker.

```
cd $PERCEIVE_DATA/computer-vision/experiments/camera_positioning/fischer_experiment_1/
cat > manifest.json <<EOF
{
   "store": 0,
   "scene-key": "fischer_1_2019-02-12_v2",
   "epoch": "2018-11-02T15:00:00",
   "videos": [
      {
         "camera": "c42_v13",
         "file": "fisher-1_c42.mp4",
         "timestamp": "2018-11-02T15:00:00.000000+0000",
         "duration": 599.937
      }
   ]
}
```

Note a few things. (1), a few things in this file must be there, but they're irrelevant: "store: -1", "epoch", "timestamp", and "duration". (2) The "scene-key" is used to find the scene, and "file" is used to find the input video file.

# (*) Set up the scene file (*)

The scene file describes the cameras in the room and where they are. They are stored on `s3`, and also in `$PERCEIVE_DATA/multiview/calibration/scenes/`. The point of the external calibration is to create a new scene file. Currently we need a scene file as input, but we really only need TWO input values: (1) the scene key (i.e., `fischer_1_2019-02-12_v1`), and the array of bcams (i.e., `["c42_v13"]`). Thus, the new version should not require a scene file as _input_... just the scene's key, and the list of cameras. It still produces a scene as _output_.

```
cd $PERCEIVE_DATA/multiview/calibration/scenes/
cat > fischer_1_2019-02-12_v1.json <<EOF
{
    "type":             "SceneDescriptionInfo",
    "store-id":         0,
    "scene-key":        "fischer_1_2019-02-12_v1",
    "bcam-keys":        ["c42_v13"],
    "bcam-transforms":
    [
        {
            "translation": [0, 0, 0],
            "rotation":    [0, 0, 0, 1],
            "scale":        1.0
        }
    ]
}
EOF

aws s3 cp fischer_1_2019-02-12_v1.json s3://perceive-multiview/calibration/scenes/fischer_1_2019-02-12_v1.json
```

# (*) About those Cameras (*)

This example has a single camera: `c42_v13`. This directs multivew to load the camera calibration from `$PERCEIVE_DATA/multiview/calibration/binocular/c42_v13.json`, or from `s3://perceive-multiview/calibration/binocular/c42_v13.json`, depending on whether the environment variable `PERCEIVE_STORE` is set to `PERCEIVE_DATA` or `S3`.

This calibration should already be done. If it is not done, your program will fail to find the camera file. If this is the case, it should give you a nice error message.

# (*) Run the Aruco External Calibration (*)

We need to find an approximate calibration that positions the aruco cube. In particular, we need to find the rotation such that the aruco cube is aligned with the XYZ axes. Otherwise none of the steps below will work. I will ponder if there's a way to get rid of this step. But worst case, we'll have to have an image of the aruco cube (or some corner of a pillar or a room where three orthogonal planes meet), and the use will have to select some points, and an optimization procedure should do a rough estimate of the camera positions.

```
# Create the folder for the point-picking GUI files
cd $PERCEIVE_DATA/computer-vision/experiments/camera_positioning/fischer_experiment_1/
mkdir fischer_experiment_1_cam-position-files
cd fischer_experiment_1_cam-position-files

# Split the input image for the point-picking GUI
FILE="../fisher-pos2.jpg"

# Get image files width and height
WH=$(convert "$FILE" -print "%w %h" /dev/null 2>/dev/null)
W=$(expr $(echo "$WH" | awk '{ print $1 }') / 2 2>/dev/null)
H=$(echo "$WH" | awk '{ print $2 }')

# Split out the images
convert "$FILE" -crop ${W}x${H}+0+0 "STR00042.png"
convert "$FILE" -crop ${W}x${H}+${W}+0 "STR00041.png"

# Set up the initial calibration file
cat > fischer_experiment_1_cam-pos-measurements.json <<EOF
{
    "type": "EstimateCamExtrinsicInfo",
    "scene-key": "fischer_1_2019-02-12_v1",
    "ref-index": 0,

    "global-world-coords": {
        "F1": [ 0.000, 0.034, 0.034],
        "F2": [ 0.000, 0.522, 0.034],
        "F3": [ 0.000, 0.522, 0.522],
        "F4": [ 0.000, 0.034, 0.522],
        
        "B1": [ 0.562, 0.034, 0.034],
        "B2": [ 0.562, 0.522, 0.034],
        "B3": [ 0.562, 0.522, 0.522],
        "B4": [ 0.562, 0.034, 0.522],

        "E1": [ 0.522, 0.000, 0.034],
        "E2": [ 0.034, 0.000, 0.034],
        "E3": [ 0.034, 0.000, 0.522],
        "E4": [ 0.522, 0.000, 0.522],

        "W1": [ 0.522, 0.562, 0.034],
        "W2": [ 0.034, 0.562, 0.034],
        "W3": [ 0.034, 0.562, 0.522],
        "W4": [ 0.522, 0.562, 0.522],

        "T1": [ 0.034, 0.034, 0.562],
        "T2": [ 0.034, 0.522, 0.562],
        "T3": [ 0.522, 0.522, 0.562],
        "T4": [ 0.522, 0.034, 0.562]        
    },

    "planes": {
        "front":  [1, 0, 0, 0.000],
        "back":   [1, 0, 0,-0.562],
        "east":   [0, 1, 0, 0.000],
        "west":   [0, 1, 0,-0.562],
        "top":    [0, 0, 1,-0.562]
    },

    "sensor-info": [
        {"sensor-id":"STR00041",
         "image-format":[2592,1944],
         "world-coords":[],
         "image-coords":[],
         "estimated-center":[4,1.5,2],
         "image-file":"$PERCEIVE_DATA/computer-vision/experiments/camera_positioning/fischer_experiment_1/fischer_experiment_1_cam-position-files/STR00041.png"},

        {"sensor-id":"STR00042",
         "image-format":[2592,1944],         
         "world-coords":[],
         "image-coords":[],
         "estimated-center":[4,1.5,2],
         "image-file":"$PERCEIVE_DATA/computer-vision/experiments/camera_positioning/fischer_experiment_1/fischer_experiment_1_cam-position-files/STR00042.png"}            
    ]
}
EOF

# Upload to s3
aws s3 cp "STR00041.png" "s3://perceive-images/rotation/fischer_experiment_1/STR00041.png"
aws s3 cp "STR00042.png" "s3://perceive-images/rotation/fischer_experiment_1/STR00042.png"
aws s3 cp "fischer_experiment_1_cam-pos-measurements.json" "s3://perceive-images/rotation/fischer_experiment_1/fischer_experiment_1_cam-pos-measurements.json"
```

# Load the pick-picker GUI
(1) Go to `https://tools.perceiveinc.com/rotation/add-store`
(2) Add store "fischer_experiment_1"
(3) Go to `https://tools.perceiveinc.com/rotation/pixel`
(4) Select some pixels. The Aruco cube has a very particular orientation. See the image "$PERCEIVE_DATA/computer-vision/experiments/camera_positioning/fischer_experiment_1/ARUCO-orientation.png".

I saved the output to:

$PERCEIVE_DATA/computer-vision/experiments/camera_positioning/fischer_experiment_1/fischer_experiment_1_aruco-measurements.json

# Run extrinsic camera positioning

```
cd $PERCEIVE_CODE/multiview/multiview_cpp
IN_FILE="$PERCEIVE_DATA/computer-vision/experiments/camera_positioning/fischer_experiment_1/fischer_experiment_1_aruco-measurements.json"

./run.sh release no-cuda stereo_cam_pos_opt -o $PERCEIVE_DATA/multiview/calibration/scenes/fischer_1_2019-02-12_v2.json "$IN_FILE"

# Patch the verion number of the scene
sed -i 's,fischer_1_2019-02-12_v1,fischer_1_2019-02-12_v2,g' $PERCEIVE_DATA/multiview/calibration/scenes/fischer_1_2019-02-12_v2.json
sed -i 's,^\s*"aruco-result-key.*$,,' $PERCEIVE_DATA/multiview/calibration/scenes/fischer_1_2019-02-12_v2.json

aws s3 cp $PERCEIVE_DATA/multiview/calibration/scenes/fischer_1_2019-02-12_v2.json s3://perceive-multiview/calibration/scenes/fischer_1_2019-02-12_v2.json
```

You can now review the optimization files to check that this worked. The blue and red crosses should align on the labeled corners of the aruco cube.

```
# The individual sensors are optimized:
feh /tmp/one-cam-opt_STR00042.png
feh /tmp/one-cam-opt_STR00041.png

# The combined 3D optimization of the corner points,
# as seen from each images points of view
feh /tmp/stereo-cam-opt_et-opt_STR00042.png
feh /tmp/stereo-cam-opt_et-opt_STR00041.png
```

NOTE: I had some trouble with the point-picker scrambling the order of the outputs. If the calibration is no good, then this is likely the culprit. You can tell by looking at the epipolar errors at the top of the output. I appended a message about it to the output.

# (*) IMPORTANT NOTE (*)

There is some problem with the results... it must be finding a degenerate solution. The steps below will not look right, but they are correct. I will update the steps above when I've found a solution.

# (*) Load Tweaker (*)

```
MANIFEST=$PERCEIVE_DATA/computer-vision/experiments/camera_positioning/fischer_experiment_1/manifest.json
cd $PERCEIVE_CODE
gui/camera-tweaker.py $MANIFEST
```

Tweaker should load, and see the rectified image and point cloud.

# (*) About SLIC (*)

SLIC generates the superpixels. It is a _deterministic_ algorithm; however, it is sensitive to the inputs. Thus if you change 1 pixel in an image, the entire SLIC result will change. This is intentional. In a way, it's sorta like what a pseudo-number generator does.

Below you'll be selecting super-pixels, and associating them with planes. The superpixels are identified by an integer. (They're ordered: superpixel 1..N.) If you modify the input image, or change the superpixel parameters, then your preselected superpixels will appear to be randomly shuffled around the image!!!

When writing the new software, you'll want to give the user the option of adjusting the two input parameters: "Superpixel size", and "Compactness". Go to the "Slic" radio button, and start adjusting the sliders and see what happens.

Before associting a single superpixel to a plane, the user will have to settle on (and finalize) their SLIC parameters. Fewer super pixels means less work (less to select); however, we never want to select superpixels that bleed outside of the relevant planes.

I set size to 201, and compactness to 15.

# (*) Creating Planes (*)

Click on the "Planes" radio button.

## Load/Save ##

Locate and notice the "load/save" buttons. They load/save your plane selection to:

$PERCEIVE_DATA/TMP/plane-set-selected.json

These buttons are DANGEROUS. Obviously the application you create should work differently. Ideally you want the user to identify the file they are saving/loading to/from. Then, that file (ideally) should keep a complete list of undo/redo. So everything that the user does to modify the file with the GUI should be some sort of COMMAND object, and the file saves a sequence of COMMANDS. (Thus, the COMMANDS must be serializable.) The COMMANDS mutate the "model" (in the MVC sense). Also, for every command, you should be able to "unapply" it. If you design your commands in this way, then you'll be able to implement undo/redo, and save the information to file, and thus, the save/load buttons will not be dangerous.

## Creating the Planes ##

Look at the image, and note the following planes:

(1) The floor. This is a "Z" plane. (Meaning that the normal runs down the Z axis.)
(2) The wood part of the calibration table. This is a "Z" plane.
(3) Top of the aruco box. A "Z" plane.
(4) The back (grey) wall. An "X" plane.

Click the "add cp" button to create these planes.


