# The PRCV multiview project

multiview is a large project which combines several components of a people tracking system. It consists of the following functionality:  
* Camera calibration for stereo and monocular cameras
* Video I/O and decoding
* Object detection
* 3D Reconstruction
* Pose Detection through OpenPose or HyperPose
* Vector space tracking with several algorithm options
* GUI for debugging 3D data
* Command line interface
* Custom geometry library

Over time we will split this library into components.

## Building multiview

See `build/install-files/README.md` but the basic instructions are these:
```
sudo base-scripts/install-cuda.sh
sudo build-scripts/build-all.sh
```
The project currently builds with:  
* Ubuntu 18.04
* CUDA 11.2
* OpenCV 4.4

and has been tested on:
* AMD Ryzen 5 3600 6-Core
* Nvidia GeForce RTX 3060 (CUDA Capability 86)

The build scripts will attempt to download from S3. This obviously won't work and instead you can get the files [here](https://www.dropbox.com/sh/775n3sblawqinqt/AACalq6fX_ASjk2XS8GufmGna?dl=0) and edit the scripts. Sorry about this - it's one of the first things we are fixing.

## Running testcases

The following scenarios are available to test:  
   * 001_B7-short                     -- Two people walking through envista.
   * 002_jewelry-gaze               -- Track 1 person, but gaze is over the shoulder.
   * 003_jewelry-cluster            -- People are tightly clustered together.
   * 004_lobby-clutter           -- Lots of tracks in a small space. Frames [1000-1500).
   * 005_lobby-sitting           -- More lobby, with someone sitting. Frames [0-1000).
   * 006_retail-tracks           -- Three people, some moving fast. Frames [6500-6900).
   * 007_retail-sparse-tracks       -- Two tracks across many cameras. Frames [0-500).
   * 008_showroom1-standing         -- Tracks subject to reprojection error issues.
   * 009_showroom1-stitching        -- A track that traces through three camera views. Frame  [4400-4700).
   * 010_acquarium                     -- Crowds of people, steep angle, far away. Frames [1100-1900).
   * 011_showroom2-standing       -- A bunch of people standing around. Frames [700-850).
   * 012_showroom2-walking        -- A group of people walking through store. Frames [1200-1600).
   * 013_showroom2-occlude        -- Three people in a tight space, with occlusion. Frames [1900-2100).
   * 014_showroom2-sitting        -- A group walking to a table, and some sitting. Frames [5300-5650). 

Testcases can be run via `./bin/testcase.sh`. One test case has data available today which can be downloaded [here](https://www.dropbox.com/sh/btpqu9p9nv68txj/AACajIaTwKjXO7vSe4CrJiROa?dl=0). Like the build scripts we need to reorganize the data to make them all available.

## Running multiview

To compile and then run the CLI, use `./multiview/multiview_cpp/run.sh`

Selected options are below. Use `-h` to see more.

      blur_faces                   Blur faces on an input video
      classifier                   trains classifiers using either an rtree or svm.
      distortion_calib             Distortion calibration.
      phase_camera_extrinsic       Method for positioning cameras in 3D space.
      pipeline                     The multiview pipeline!
      position_scene_cameras       global camera position optimization.
      render_floor_map             Create a floor map from the 3D point cloud.
      sprite_main                  front-end for viewing 3D models using the `assimp` library.

## Demos

For demo videos and other media see the [PRCV website](https://prcvlabs.org).

## Contributors

This project was developed over the course of several years by [Aaron Michaux](https://pageofswords.net/press/) and is being maintained by PRCV Labs.
