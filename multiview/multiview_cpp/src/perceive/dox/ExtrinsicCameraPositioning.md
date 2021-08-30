
Extrinsic Camera Positioning           {#extrinsic_camera_positioning}
============================

These procedures work with the _*Kyle Aruco Cube*_.
If we build a new cube, then we'll need to make minor changes to both the code and the calibration procedures.

# Summary #

The point of extrinsic camera positioning is to find the Euclidean Transformations (rotation and translation) for all the cameras in a scene.
A scene is something like "pandora-gleenbrook". There are 1 or more cameras, and we need the positions for each of them.

# Collecting Input Data #

Physically place the cameras in the scene. Then place the ArucoCube in some place where it's 3-6 meters from each camera, and visible to each camera. (What if this is impossible? See note below.) Then take an image from each camera. One per camera. These are the input images.

# Run the Optimization #

Say there are three cameras, and the images are "C1001.png", "C1011.png", and "C1020.png". You need to run the code below _for each image_.

```
CAM="C1001_v1"
RUN_SH="$PERCEIVE_CODE/multiview/multiview_cpp/run.sh"
OUTDIR="/tmp/output_${CAM}"
mkdir -p "$OUTDIR"
$RUN_SH no-cuda release phase_camera_extrinsic --pointwise \
        -d "$OUTDIR" --cam "$CAM" --dense-et0 c1001.png
```

The output is all in `$OUTDIR`. 
Inspect all the images  `$OUTDIR/zzz_*_FINAL.png`. 
The result is in the file `$OUTDIR/positions.json`.

# Save the Results #

The results need to be copied into the scene file. 
If the scene is called "fischer-2019-03-01_v1", then the scene file is at the path: `$PERCEIVE_DATA/multiview/calibration/scenes/fischer-2019-03-01_v1.json`.
On s3, the path is: `s3://perceive-multiview/calibration/scenes/fischer-2019-03-01_v1.json`.

The cameras and euclidean transforms are stored in two different keys:

```
"bcam-keys":             ["c1001_v1", "c1011_v1", "c1020_v1"],
"bcam-transforms":       [{...}, {...}, {...}]
```

You need to copy the output from `$OUTDIR/positions.json` into the correct position in the scene file.

# What is the "Kyle Arubo Cube"? #

The "Kyle Aruco Cube" is a Aruco Cube that has been carefully measured. The details of this are stored in the header file: `$PERCEIVE_CODE/multiview/multiview_cpp/src/perceive/calibration/aruco-cube.hpp`

The origin sits at the intersection of the FRONT, BOTTOM, and EAST faces of the Aruco Cube. A small mark has been made on the three faces close to that corner. See the Aruco Cube docks to match pictures of the faces. [https://docs.google.com/document/d/1xaKrIOwItwW6szzPPmGfJ5gqsGzolg6K3WyRxBHg_jc/edit?pli=1#heading=h.ut2khdnlrrkx](https://docs.google.com/document/d/1xaKrIOwItwW6szzPPmGfJ5gqsGzolg6K3WyRxBHg_jc/edit?pli=1#heading=h.ut2khdnlrrkx).

# Moving Origin -- The Global Transformation #

By default, the origin is the origin of the Aruco Cube. However, that's not always convenient. For example, multiview uses an "axis-aligned" bounding box for the entire scene, and thus it makes sense to align the walls of a room with the X and Y axis. Unfortunately, this _could_ restricts how the Aruco Cube is placed. 

In the spirit of flexibility, the scene file can have a "global-transformation". As the name suggests, this gives a global rigid body transformation for the entire scene. The global transformation rotates and translates the entire set of scene cameras as one.

And example "global-transformation" is:


```
"global-transformation":
{
    "translation": [0.024408, 0.198505, 0.000000],
    "rotation": [0.000000, 0.000000, -0.908805, 0.417221],
    "scale": 1.000000
}
```

# Note: What Happens if a Camera Cannot see the Aruco Cube #

Image we have three cameras, {X, A, and B}, and it's impossible to place the cube such that all three can see it at once. To handle this situation, we must be able to place the cube such that a pair of cameras can see it at the same time.

### Scenario ###

The Aruco Cube is placed in _two_ positions. Cameras 'X' and 'A' can see the cube at position 1. Cameras 'A' and 'B' and see the cube as position 2. This gives _four_ positions: \f$X_1, A_1, A_2, B_2\f$. In this example, we want to use position 1 as the origin. Therefore, we don't have to do anything with camera 'X' and 'A', we already know where they are. What we _do have to do_, is calculate position \f$B_1\f$, which would be the position of the origin relative to camera 'B'. This can be done by "traversing" the arrows: \f$B_2 A_2^{-1} A_1\f$. 

```

Cameras:  X       A       B
           \     / \     /
            \   /   \   /
Positions:   [1]     [2]

```


That is, if we start \f$B_2\f$, we move to [2], then \f$A\f$, and [1], by apply (in order) transformations \f$B_2\f$, then \f$A_2^{-1}\f$, then \f$A_1\f$.

\f[B_1 = B2 A_2^{-1} A_1\f]

These linear transformations are implemented in `$PERCEIVE_CODE/multiview/multiview_cpp/src/perceive/geometry/euclidean-transform.hpp`. 

This can be calculated using the program 'transitive_rigid_motion' as follows:

```
RUN_SH="$PERCEIVE_CODE/multiview/multiview_cpp/run.sh"
OPTS="release no-cuda"
$RUN_SH $OPTS transitive_rigid_motion -y -d /tmp "$CAM_A1" "$CAM_A2" "$CAM_B2"
```
