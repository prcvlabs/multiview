# multiview data

Calibration data and testcase metadata for multiview. This directory is the
default asset store (`MULTIVIEW_ASSET_DIR`), so scenes load without any
setup. The videos, and the larger assets, are in the Hugging Face dataset
[prcvlabs/multiview-retail-tracking](https://huggingface.co/datasets/prcvlabs/multiview-retail-tracking).

## Layout

```
calibration/
   scenes/                          scene descriptions, by scene key
   binocular/                       stereo camera calibrations
   sensors/                         sensor distortion models
   aruco-cubes/                     aruco calibration cube geometry
   aruco-results/                   aruco calibration results
   plane-data/                      scene plane data
   camera-positioning-annotations/  annotations for camera positioning
   cad-models/                      few-triangle CAD models (*.stl)
   classifiers/                     pose-svm classifiers (*.ml.gz)
testcases/<NNN_siteNN_...>/
   manifest.json                    the scene key and video files
   ground-truth.json                annotated tracks (where available)
   expected-output.text             expected tracker accuracy
   hash.md5                         md5 of ground-truth.json
```

The Hugging Face dataset has the same layout. It also has each testcase's
videos and `annotation-data.json`, the rtree pose classifiers and the
many-triangle CAD models. It is freely downloadable; fetch testcases with:

```
build/fetch-testcases.sh 7          # one testcase, into ~/multiview-data/testcases
build/fetch-testcases.sh all        # every testcase
build/fetch-testcases.sh --list
```

## Anonymization

The data was recorded at retail stores and offices. Sites are named
`site01` to `site12`, and scenes `siteNN_sceneNN`. Recording dates have
been removed: timestamps start at `2000-01-01`, or `1970-01-01` where none
was recorded.

## License

The data is licensed under
[CC BY-NC 4.0](https://creativecommons.org/licenses/by-nc/4.0/). The code
in this repository is licensed under the Apache License 2.0 (see
[LICENSE](../LICENSE)).
