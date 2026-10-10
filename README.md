# The PRCV multiview project

multiview was designed and written by [Aaron Michaux](https://github.com/aaron-michaux)
over several years at Perceive. It is maintained by PRCV Labs (Everett Berry),
who open sourced it; the 2026 build modernization was contributed with help
from Claude.

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

multiview builds entirely in user space: no `sudo`, no `/opt/multiview`
toolchain, no `/usr/local/cuda`. All dependencies (GCC 13, CUDA 12.8,
CMake/Ninja, OpenCV 4.12 + contrib, Boost, Eigen, ...) come from conda-forge
via a [micromamba](https://mamba.readthedocs.io/en/latest/user_guide/micromamba.html)
environment described in [`build/environment.yml`](build/environment.yml).

Tested with:
* Ubuntu 24.04 (x86_64)
* NVIDIA GeForce RTX 3090 (compute capability 8.6), driver 595
* CUDA 12.8, GCC 13.4, OpenCV 4.12, Boost 1.86

### Prerequisites

* Linux x86_64 with `curl`, `bzip2`, `git` and about 6 GB of free disk space.
* For GPU code: an NVIDIA driver that supports CUDA 12.8 (driver >= 570).
  Check with `nvidia-smi`. The CUDA toolkit itself is installed into the
  environment; a system CUDA install is not needed. Without a GPU everything
  still builds, and the CUDA code paths fall back to the CPU (GPU-only
  testcases are skipped).

### Build

```
git clone https://github.com/prcvlabs/multiview.git
cd multiview
build/setup-env.sh     # installs micromamba to ~/.local/bin, creates the 'multiview' env
build/build.sh         # Release build of multiview-cli and multiview-testcases
```

`build/build.sh` compiles the CUDA kernels for the GPU(s) found by
`nvidia-smi` (sm_86 on an RTX 30-series). Binaries end up in
`multiview/multiview_cpp/build/cmake-release/`. Other forms:

```
build/build.sh Debug multiview-testcases                     # build type, targets
MULTIVIEW_CMAKE_ARGS="-DCMAKE_CUDA_ARCHITECTURES=75;86" build/build.sh  # pick CUDA archs
MULTIVIEW_CMAKE_ARGS="-DMULTIVIEW_WITH_AWS=OFF" build/build.sh          # no AWS SDK
MULTIVIEW_BUILD_DIR=$PWD/build-hp MULTIVIEW_CMAKE_ARGS="-DMULTIVIEW_WITH_HYPERPOSE=ON" build/build.sh  # separate build dir
```

To use CMake directly, activate the environment
(`eval "$(micromamba shell hook -s bash)" && micromamba activate multiview`)
and configure `multiview/multiview_cpp/CMakeLists.txt` as usual.

### Build options

| CMake option | Default | Effect |
|---|---|---|
| `MULTIVIEW_WITH_CUDA` | `ON` | Build the CUDA kernels in `src/cuda` (RGB to LAB, SLIC superpixels); defines `WITH_CUDA`. |
| `MULTIVIEW_WITH_AWS` | `ON` | Link the AWS SDK for `s3://` assets. When `OFF`, `MULTIVIEW_NO_AWS` is defined and S3 access raises an error. |
| `MULTIVIEW_WITH_LIBSGM` | `ON` with CUDA | Build `contrib/libSGM` and enable the GPU `CUDA_SGM` disparity strategy (`USE_SGM`). |
| `MULTIVIEW_WITH_HYPERPOSE` | `OFF` | HyperPose + TensorRT 10 pose detection (`WITH_HYPERPOSE`). Needs `build/setup-env.sh --with-tensorrt` (TensorRT into `<env>/opt/tensorrt`, or set `TENSORRT_ROOT`) and models from `build/download-hyperpose-models.sh` (read from `$MULTIVIEW_HYPERPOSE_DIR/models`); HyperPose is fetched at a pinned commit during configure. |
| `MULTIVIEW_WITH_CCACHE` | `ON` | Use `ccache` (in `environment.yml`) as the compiler launcher when it is found. |
| `MULTIVIEW_PROFILE_MEMORY` | `OFF` | Allocation tracking (`PERCEIVE_PROFILE_MEMORY`), for all targets. |
| `CMAKE_CUDA_ARCHITECTURES` | `native` | GPU architectures to compile for. |

Rectification (`calc_rectified`) uses a native CUDA remap kernel whenever
`MULTIVIEW_WITH_CUDA` is on. `WITH_OPENCV_CUDA` (GPU StereoBM/SGBM) is turned
on automatically if your OpenCV has the `cudastereo` module; the conda-forge
OpenCV does not, so those run on the CPU.

These are **not** supported by the CMake build: OpenPose/Caffe pose
detection, Triclops, PCL, OpenGL and the GUIs
(`multiview-gui`, `multiview-imgui`). They still need the legacy build below.

### Run the unit tests

```
multiview/multiview_cpp/build/cmake-release/multiview-testcases           # everything
multiview/multiview_cpp/build/cmake-release/multiview-testcases "[cuda_slic],[cuda_argb_to_lab]"  # GPU tests
multiview/multiview_cpp/build/cmake-release/multiview-testcases --list-tests
```

The unit tests need no external data.

### Legacy build

The original build (`multiview/multiview_cpp/run.sh` + `mobius`) expects a
custom clang-11 toolchain in `/opt/multiview`, CUDA in `/usr/local/cuda`,
and is set up with `sudo` from a
[Dropbox bundle of install files](https://www.dropbox.com/sh/775n3sblawqinqt/AACalq6fX_ASjk2XS8GufmGna?dl=0)
(2.5 GB) using `build/install-files/`. It was last tested on Ubuntu 18.04,
CUDA 11.2 and OpenCV 4.4; see `build/install-files/README.md`. It is
the only way to build the pose detection and GUI targets.

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

The CLI is `multiview/multiview_cpp/build/cmake-release/multiview-cli`
(`run.sh` in the legacy build). Run `multiview-cli -h` for the list of
commands, and `multiview-cli <command> -h` for help on each one, e.g.:

```
CLI=multiview/multiview_cpp/build/cmake-release/multiview-cli
$CLI -h
$CLI undistort -h
$CLI dump_default_params -d /tmp/params /path/to/manifest.json
```

Commands that load a scene (`pipeline`, `dump_default_params`, ...) need its
calibration data from the asset store (`MULTIVIEW_ASSET_STORE`, default
`LAZY_S3` with bucket `s3://perceive-multiview`). That bucket is no longer
available, so these commands currently fail with a `NoSuchBucket` error
unless you provide the calibration files yourself in a local store
(`MULTIVIEW_ASSET_STORE=MULTIVIEW_ASSET_DIR` and `MULTIVIEW_ASSET_DIR=/abs/path`,
using the same layout as the bucket, e.g. `calibration/scenes/<scene-key>.json`).

Selected commands are below.

      blur_faces                   Blur faces on an input video
      classifier                   trains classifiers using either an rtree or svm.
      distortion_calib             Distortion calibration.
      phase_camera_extrinsic       Method for positioning cameras in 3D space.
      pipeline                     The multiview pipeline!
      position_scene_cameras       global camera position optimization.
      render_floor_map             Create a floor map from the 3D point cloud.
      sprite_main                  front-end for viewing 3D models using the `assimp` library.

## Tour of the library

See [doc/TOUR.md](doc/TOUR.md) for an overview of the code base.

## Demos

For demo videos and other media see the [PRCV website](https://prcvlabs.org).

## Developing

In VS Code, go to Settings > Watcher Exclude and add `**/contrib/**` and `**/doc/**` to enable file watching. These directories either never change or are automatically generated.

## Contributors

multiview was designed and written by [Aaron Michaux](https://github.com/aaron-michaux),
its author, over several years at Perceive. It is maintained by PRCV Labs
(Everett Berry). The 2026 build modernization (user-space environment, CMake
build, CUDA 12 / OpenCV 4.12 support) was contributed with help from Claude.
