
Scene Creation Notes           {#scene_creation_notes}
====================

We're using the calibration files in: `$PERCEIVE_DATA/computer-vision/experiments/multi-camera-positioning/fischer-1/`

# Steps to create a new scene

 * Install the cameras on site.
 * Take photos of the Aruco Cube. (Currently we only have the "Kyle Cube".)
 * Run the "extrinsic calibration" procedure to produce a scene file.

# Workflow

Inputs
 
 * The store (but not necessarily the scene) is set up on GUAPI.
 * We have a set of images of Aruco Boxes
 * We run the calibration command-line, which also creates and updates
   the scene.
   
# Multiview-GUAPI interface

 * Multiview uses `system()`, to call a python command-line interface.
 * Interface
   - Get a list of all the stores.
   - Does a scene exist?
   - What versions are available for a given scene?
   - What are the cameras and start/end date for a given scene?
   - How many videos exist for a given scene
   - Create a scene.
   - Update a scene, including associating cameras, but only if zero videos exist.
   - Create and edit the associated scene json file, including uploading it to S3.

# The "Scene Manager"

We need to be able to:

 * Create/Update/Delete scenes (respecting version numbering.)
   - We can only delete scenes if they are *UNUSED*
   - If a scene is *USED*, then the version number is incremented.
     (i.e., we never overwrite or delete a *USED* scene.)
 * Run a calibration to get the camera positioning
 



