# A tour of multiview

`multiview` is a C++ computer-vision library written by **[Aaron Michaux](https://github.com/aaron-michaux)** over several years for Perceive, a company that tracked people in retail stores. Its job was to take synchronized video from several calibrated stereo (or mono) cameras mounted in a store and work out where each shopper was on the floor, frame by frame, and then join those positions into tracks. Getting there needs most of classic multi-view geometry: intrinsic and extrinsic calibration, distortion models, rectification and disparity, point clouds, superpixels, 3D lifting of pose skeletons, floor-plane histograms, and combinatorial tracking. Some of the code wraps OpenCV, alglib, Boost, or vendored libraries, but a large share is the author's own work. This tour is for people who want to understand the codebase or learn from those implementations.

All paths below are relative to the repository root. Most of the C++ code is under `multiview/multiview_cpp/src/` (shortened to `src/` from here on).

## 1. How data flows through the pipeline

The runtime is a dataflow graph of tasks. `TaskNode` (`src/perceive/pipeline/pipeline-task.hpp`) implements the generic directed-graph concept in `src/perceive/utils/dag-node.hpp`. Each node publishes an immutable `shared_ptr<const Result>`. A result that depends on another result holds a `shared_ptr` to it, so memory is freed automatically when nothing references it any more, and threads can share results without extra locking. The nodes live in `src/perceive/pipeline/nodes/`. They are wired together in `FrameResults::init_` (`src/perceive/pipeline/frame-results.cpp`), which runs once per frame, and `MovieResults` (`src/perceive/pipeline/movie-results.hpp`) adds the tasks that span many frames.

The per-frame graph, as wired in the code:

1. **`load_scene_description`**: loads the `SceneDescription` (cameras, `BinocularCameraInfo`, world transforms, histogram bounds). Then `copy_sensor_images` and `input_images[sensor]` read the video frames.
2. **`convert_to_lab[sensor]`**: converts RGB to CIE-LAB. **`run_f2d[sensor]`** computes 2D features (Scharr/Harris gradients and SLIC superpixels, on CPU or CUDA).
3. **`bcam_init[camera]` → `get_xy_mapping[sensor]` → `calc_rectified[sensor]`**: builds undistort+rectify maps from the distortion model and stereo extrinsics, then remaps each image. `calc_calibration_mask` marks the region where calibration is valid.
4. **`disp_init[camera]` → `disp_map_update[camera]`**: computes disparity on the rectified pair and triangulates it into a `PointCloud`. `create_slic_lookup[sensor]` maps superpixel labels into rectified space.
5. **`pose_skeleton_init`**: runs a 2D pose detector (OpenPose or HyperPose helpers in `src/perceive/cost-functions/pose-skeleton/`) on the sensor images. This runs **in parallel** with the stereo branch, not after it.
6. **`floor_hist`**: projects every camera's point cloud onto the floor plane as a height-filtered occupancy histogram, using the SLIC lookups and LAB images.
7. **`localization`**: combines the floor histogram, the 2D skeletons (lifted to 3D), and point clouds into per-frame `LocalizationData`, which is a set of foreground/background probability fields and person hypotheses.

Across frames, `MovieResults` runs `movie_task` (which collects per-frame localization), then **`tracklet_task`** (short windows of frames), then **`tracks_task`** (complete tracks). Outputs are written as `FowlkesResult` JSON (`src/perceive/cost-functions/fowlkes/`), and optionally as rendered debug or presentation movies (`src/perceive/movie/`).

Two corrections to the simple "video → calibration → … → tracking" story. Calibration happens **offline**: separate CLI commands produce the distortion models and extrinsics that the pipeline loads. And pose detection is a **sibling branch** of stereo that meets it only at localization. Scenes can also be "no-stereo" (`is_no_stereo()`), in which case localization relies only on the 2D poses.

## 2. Directory map

| Path | Contents |
|---|---|
| `src/perceive/geometry/` | Vectors, quaternions/rotations (`rotation.hpp`), `EuclideanTransform`, planes/lines/polygons, plane fitting, splines and RBF surfaces, spatial index, sparse histogram cells |
| `src/perceive/geometry/projective/` | Distortion model, caching undistort inverse, `DistortedCamera`, `BinocularCamera`, triangulation, back-projection helpers |
| `src/perceive/geometry/skeleton/` | 2D skeletons, bones, 3D skeleton fitting, interpolation, skeleton-to-skeleton affinity |
| `src/perceive/calibration/` | Homography, K, F/E estimation; ArUco cube; phase-based stereo and extrinsic calibration; plane-set calibration; `position-scene-cameras` |
| `src/perceive/cost-functions/` | Disparity strategies, 2D features, SLIC-3D, floor histogram, localization, pose-skeleton, tracklets, tracks, classifier, photo-consistency |
| `src/perceive/optimization/` | Hungarian, golden-section, compass search, Nelder–Mead and LM front-ends, min-cut graph, BFS, DAG shortest path, KZ filter |
| `src/perceive/graphics/` | Image containers, packed LAB, Canny, distance transform, integral images, Bresenham, colour sets, heat maps |
| `src/perceive/pipeline/` | Task DAG, per-frame and per-movie results, pipeline nodes, tracker driver |
| `src/perceive/scene/` | Scene description, manifests, annotations, pose annotations |
| `src/perceive/io/` | JSON I/O, `struct-meta` reflection, S3/lazy-S3 assets, versioned resources, IEEE754/XDR packing |
| `src/perceive/movie/` | ffmpeg I/O, debug and presentation movie rendering, floor-grid drawing |
| `src/perceive/utils/` | Containers, threads, spin-lock, md5/base64, timestamps, CUDA spec, OpenCV/Eigen helpers |
| `src/perceive/dox/` | Author's design notes (intrinsic and stereo calibration, extrinsic positioning, tracker, Fowlkes output) |
| `src/front-ends/` | Subcommands of `multiview-cli` (see §4) |
| `src/cuda/perceive/graphics/` | CUDA kernels for SLIC and RGB→LAB, plus a zero-copy remapper |
| `src/testcases/` | Catch-based unit tests (see §5) |
| `src/gui/` | Qt GUI with tabs for frames, disparity, floor histogram, camera positioning, and a GL viewer |

Vendored code (skipped here) is in `src/perceive/contrib/`: Catch, CTRE, fmt, hopscotch-map, jsoncpp, nelmin, range-v3, the reference SLIC, and Triclops config. More is in `multiview/contrib/`: alglib, libSGM, EllipseFit, fast bilateral-space stereo, mosya_conics, static-math, and an assimp sample.

## 3. Implementations worth reading

**1. Rational polynomial distortion model.** `src/perceive/geometry/projective/polynomial-model.hpp` (`PolynomialModel<K>`, with `DistortionModel` as the instantiated alias). The model does not use the OpenCV radial/tangential form. It maps conditioned distorted coordinates to undistorted ones using a full bivariate polynomial of degree K: a `2 × (K+1)(K+2)/2` coefficient matrix `A_`, plus a radial centre and scale used to condition the data. Because it is fitted by linear least squares (`LLSLineData`), it can model severe fisheye and wide-angle lenses that a few radial terms cannot. It also stores the calibrated region's convex hull, so callers know where the model can be trusted.

**2. Caching undistort inverse.** `src/perceive/geometry/projective/caching-undistort-inverse.{hpp,cpp}` (`CachingUndistortInverse`). The polynomial model goes from distorted to undistorted, but projecting a 3D point needs the inverse direction. This class samples the inverse into an empirical field (`EmpiricalFieldF`), keyed by an MD5 of the model parameters. It persists the field to a cache store and reloads it later (`cu_store().load_cu`). On load it runs a fidelity check, `distort_fidelity_check`, and throws if the cached file no longer matches. It also has an unfinished set of local rectilinear approximations (`ApproxK`, with eight samples along axis and diagonal lines) intended for O(1) approximate distort/undistort. The `cache_undistort_regen` subcommand rebuilds these caches.

**3. Projective camera primitives.** `src/perceive/geometry/projective/distorted-camera.hpp` (`DistortedCamera`, `project_to_distorted`, `backproject_ray`, `epipolar_line_in_undistorted_coords`, `plane_ray_intersect`, `transfer_distorted`, `triangulate`). Together with `triangulation.hpp` (`triangulate`, `intersect_rays`, `intersect_rays_2`), this is a compact vocabulary for reasoning with lens distortion included: back-project a pixel to a ray, intersect the ray with a plane, transfer a point between cameras through a plane, triangulate. `back-project-kite.hpp` adds a closed-form solution for the planar rectangle (a "kite") that projects onto a given image quad. It is used to bootstrap ArUco face poses.

**4. Homography, K, F and E estimation.** `src/perceive/calibration/find-homography.hpp` (`estimate_homography_LLS`, `H_to_r1r2t`, `complete_homography`), `find-K.hpp` (`estimate_K_Zhang`, `refine_K`), and `find-F.hpp` (`longuet_higgins`, `estimate_F`, `estimate_E`, `condition_E`, `estimate_Rt_from_E`, `xFl_lFx`). These are textbook algorithms written out directly: DLT homographies, Zhang's closed-form intrinsics from homographies, the eight-point algorithm with rank-2 conditioning, and recovering R and t from an essential matrix. Each has a cost functor for nonlinear refinement. They are good reference implementations to read alongside Hartley & Zisserman.

**5. Rotations and rigid transforms.** `src/perceive/geometry/rotation.hpp`, `quaternion.hpp`, `euclidean-transform.hpp`. This is a complete set of conversions between quaternion, axis-angle, "spherical axis-angle" (saa), Rodrigues vector, and 3×3 matrix. It also includes `average_rotations`, `look_at`, and `transform_between` (absolute orientation between two point sets). `pack_et_6df` turns a transform into the 6 parameters the optimizers work on. The saa parameterization, used throughout calibration, avoids gimbal lock and the redundant fourth parameter of a quaternion.

**6. ArUco-cube calibration target.** `src/perceive/calibration/aruco-cube.{hpp,cpp}` (`ArucoCube`, `FaceMeasurement`, `estimate_init_et`, `dense_refine_init_et`, `dense_error_fun`). The calibration target is a physical cube with four ArUco markers on each face. Its pose is first estimated from corner detections. It is then refined *densely*: each face's texture is rendered into the image through the distortion model, and the LAB difference against the photo is minimized, switching between LM and Nelder–Mead. Using photometric error rather than only corner error makes the result robust to a few bad marker detections.

**7. Global camera positioning.** `src/perceive/calibration/position-scene-cameras/` (`optimize_n_way` in `helpers.cpp`, `parse_manifest_file` in `manifest-data.cpp`). This is a small bundle adjustment. The unknowns are every camera's pose and every cube placement's pose. The manifest builds a camera-to-camera constraint graph and uses `breadth_first_search` (`src/perceive/optimization/breadth-first-search.hpp`) to chain initial estimates across it, then optimizes all poses jointly. The result is written out as a scene. A related path, `run_phase_position_calibration` in `src/perceive/calibration/phase-position.{hpp,cpp}`, positions a single stereo head with the cube, optionally estimating the baseline and K as well.

**8. Stereo rectification and the `BinocularCamera`.** `src/perceive/geometry/projective/binocular-camera.{hpp,cpp}` (`BinocularCameraInfo`, `BinocularCamera::get_mapxy`, `rectify_ideal`, `project_to_rectified`, `rectified_to_ray`, `well_calibrated_region`). This builds rectification homographies for a stereo pair that combine the custom distortion model with stereo extrinsics. It produces per-sensor remap tables and the inverse mappings from rectified pixels back to rays. Disparity itself uses a **strategy pattern** in `src/perceive/cost-functions/disparity/` (`Disparity::DisparityMethod`). The strategies are OpenCV StereoBM/SGBM, CUDA semi-global matching via vendored libSGM, and Triclops. The interesting part is the interface and the rectification around it, not the matchers.

**9. Superpixels: CUDA SLIC and SLIC-3D planes.** `src/cuda/perceive/graphics/slic_impl.cu` (`kern_init_clusters`, `kern_assign_pixels`, `kern_calc_cluster_centers`, `kern_enforce_connectivity`) and `slic.hpp` (`cuda::slic::SLIC::calc_labels`). This is a from-scratch GPU SLIC. Cluster centres sit on a grid, pixels are assigned by combined LAB and spatial distance, centres are recomputed, and a separate pass enforces connectivity. Loops are written with a grid-stride helper, `strided_loop_2d`. The CPU fallback is the vendored reference SLIC (`src/perceive/cost-functions/features-2d/run-slic.cpp` selects between the two). `src/perceive/cost-functions/slic-3d/slic-3d.hpp` (`set_p3s_to_slic`, `score_slic_p3`) then fits a 3D plane to each superpixel and scores it by photo-consistency between the two views.

**10. Colour science: packed LAB and RGB→LAB on the GPU.** `src/perceive/graphics/LAB.hpp` (`LAB`, `cie1976_distance`) and `src/cuda/perceive/graphics/rgb-to-lab-kernel.cu` (`rgb_to_xyz`, `xyz_to_lab`, `image_rgb_to_lab`). `LAB` packs a colour into one `uint32_t` (8 bits of L and two 11-bit a/b fields), so a LAB image costs the same memory as ARGB and equality is a single integer compare. The CUDA kernel is a clean, readable sRGB → XYZ (D65) → CIE-LAB conversion, including the gamma linearization and the CIE epsilon/kappa piecewise function.

**11. Plane fitting and geometric fitting.** `src/perceive/geometry/fitting-planes.hpp` (`fit_plane`, `intersection_of_3_planes`) and `src/perceive/calibration/plane-set/` (`calib-plane-set`, `cps-operations`, `phase-plane-data`). This covers least-squares planes through point sets, and a plane-set calibration that fits labelled scene planes (floor, walls) to disparity and superpixel data, with Qt support in `plane-edit-widget`. `smooth-point-cloud.hpp` (`smooth_pointcloud`) cleans up point clouds before histogramming.

**12. Floor histogram.** `src/perceive/cost-functions/floor-histogram.{hpp,cpp}` (`FloorHistogram`, `Params`, `CameraInfo::slic_to_hist_xy`, `project_to_histogram`) and `src/perceive/geometry/sparse-hist-cell.hpp`. Every 3D point between `hist_min_z` and `hist_max_z` is binned into a 2D grid on the floor (10 cm cells by default), with optional distance weighting (`WGT_NORM`, `WGT_QUADRANCE`) and Gaussian smoothing. This turns noisy multi-camera stereo into a single top-down "where is mass" image. Each camera also keeps a map from superpixel label to histogram cell, so image regions and floor cells can be linked in either direction.

**13. Pose skeleton 3D lifting.** `src/perceive/geometry/skeleton/skeleton-3d.{hpp,cpp}` (`Skeleton3D::fit_3d_ret`, `shape_score`) and `src/perceive/cost-functions/pose-skeleton/pose-3d.hpp` (`Pose3D`, `make_pose_3d`). This recovers a 3D figure from one camera's 2D keypoints using a human-proportion model (`src/perceive/geometry/human-heights.hpp`). As the comments in `fit_3d_ret` describe, the torso and legs are recovered with 4 degrees of freedom (3 for the torso plane, 1 for hip rotation) by minimizing reprojection error. The head adds 3 more (distance to nose and two gaze angles), and the arms are computed directly. The result gives a floor position, a height, and a gaze direction per detection.

**14. Tracking: XYT graphs, DAG shortest paths, and Hungarian matching.** `src/perceive/cost-functions/tracks/ops.{hpp,cpp}` (`ComputationData`, `Edge::{T0,T1,INTERPOLATION}`, `topological_sort`, `dag_all_pairs_shortest_path`), `src/perceive/cost-functions/tracklets/`, and `src/perceive/optimization/hungarian-algorithm.{hpp,cpp}` (`hungarian_algorithm`, `MunkresData`). Detections become nodes in a graph over floor position and time (x, y, t). Edges carry costs from speed priors, LAB appearance, and false-positive probabilities, and can link the same frame, the next frame, or skip ahead to a later frame via interpolation. Tracks are pulled out with topological sorting and shortest paths on the DAG, and frame-to-frame or tracklet-to-tracklet joins use a templated Munkres solver. The Munkres solver is a direct implementation of Pilgrim's tutorial, padded to a square matrix. `src/perceive/optimization/shortest-path.hpp` (`shortest_path_sparse`) is a generic sparse DAG shortest path.

**15. Optimization toolbox.** `src/perceive/optimization/` contains `golden_section_search`, `compass_search` (derivative-free pattern search), the `KzFilter` (Kolmogorov–Zurbenko smoothing, used to smooth tracks), and `NonConvexOptimizer`. `levenberg_marquardt` and `nelder_mead` share a common C-style `fn/n/start/xmin` signature but are thin front-ends over vendored alglib and nelmin. `MinCutGraph` wraps Boost's Boykov–Kolmogorov max-flow behind a pimpl. Because every optimizer has the same calling convention, calibration code can switch methods with a single flag, as the ArUco refinement does.

**16. Infrastructure worth borrowing.** `src/perceive/io/struct-meta.hpp` (`MetaCompatible`, `MemberMetaData`) is a lightweight reflection system. Each `Params` struct lists its members once and gets JSON read/write, equality, and printing for free; it underpins `dump-default-params` and the pipeline's parameter files. Also worth a look are `src/perceive/utils/dag-node.hpp` (concept-based graph operations), `src/perceive/io/ieee754-packing.hpp` (portable float serialization via bitcast), the self-contained adaptive-threshold Canny in `src/perceive/graphics/canny.cpp`, and the distance transform in `src/perceive/graphics/distance-transform.cpp`.

## 4. CLI front-ends

`multiview-cli` (`src/front-ends/main.cpp`) dispatches its first argument to a registered `run_main`. The descriptions below are the subcommands' own `brief()` strings, lightly edited.

| Subcommand | Description |
|---|---|
| `pipeline` | Runs the full multiview pipeline over a scene or movie |
| `distortion_calib` | Intrinsic distortion calibration (polynomial model) |
| `cache_undistort_regen` | Regenerates the cached undistort-inverse fields |
| `undistort` | Undistorts single image files |
| `phase_stereo_calib` | Stereo calibration of a binocular head |
| `phase_camera_extrinsic` | Positions a camera in 3D using the ArUco cube (`run_phase_position_calibration`) |
| `position_scene_cameras` | Optimizes all camera positions in a scene together |
| `plane_set_calib` | Plane-set fitting for 3D scene planes (the author's own brief admits uncertainty) |
| `render_floor_map` | Creates a floor map from the 3D point cloud |
| `dump_default_params` | Writes the default `params.json` |
| `blur_faces` | Blurs faces in an input video |
| `training_data` | Imports training-data JSON and prints a flat array |
| `classifier` | Trains classifiers (random trees or SVM) |
| `sprite_main` | Viewer for 3D models using `assimp` |
| `run_scraps` | Scratchpad for running small pieces of code |

## 5. Tests

`src/testcases/main.cpp` builds a Catch runner (`CATCH_CONFIG_PREFIX_ALL`, so tests use `CATCH_TEST_CASE`) into the `multiview-testcases` binary. The test folders mirror the library:

- `geometry/`: distortion model, rotations, Euclidean transforms, lines and planes, polygons, ray–sphere intersection, mirror symmetry, normalization, and the `UV-on-same-p3`/`V-from-U` solvers.
- `optimization/`: Hungarian, graph-cut, BFS, shortest path, KZ filter.
- `graphics/`: LAB and CUDA LAB, CUDA SLIC, integral images, image containers, colour sets.
- `cost-functions/`: ArUco cube, plane-set and plane-phase calibration, skeleton-3D, OpenPose, gaze, localization data, track ops.
- `pipeline/`: graph structure, serialization, memory leaks, an example pipeline.
- `file-io/`, `utils/`, `scene/`, `movie/`, `cuda/`: serialization, struct-meta, base64, md5, timestamps, the DAG node, StereoBM on CUDA, and so on.

Many tests need scene assets or a GPU. They work best as runnable usage examples, not as a hermetic suite.

## 6. Suggested reading order

1. **Background notes.** Start with `src/perceive/dox/EuclideanTransformsAndCameraMatrix.md`, `IntrinsicCalibration.md`, and `Tracker.md` to learn the author's conventions and vocabulary.
2. **Geometry basics.** Read `geometry/vector.hpp`, `rotation.hpp`, and `euclidean-transform.hpp`. Everything else is built on these.
3. **The camera model.** Read `projective/polynomial-model.hpp`, then `caching-undistort-inverse.hpp`, then `distorted-camera.hpp`. Together they explain how a pixel becomes a ray in this codebase.
4. **Calibration.** Read `calibration/find-homography.hpp`, `find-K.hpp`, and `find-F.hpp`, then `aruco-cube.hpp` and `position-scene-cameras/helpers.cpp`, to see how cameras are calibrated and placed in the world.
5. **The pipeline skeleton.** Read `pipeline/pipeline-task.hpp` (the comment block on result lifetimes), then `frame-results.cpp` (`init_`) for the actual graph.
6. **Stereo into floor.** Follow `binocular-camera.hpp` to `nodes/disp_map_update.cpp` to `floor-histogram.hpp`, and look at the CUDA SLIC and LAB kernels along the way.
7. **People.** Read `skeleton/skeleton-3d.cpp` (`fit_3d_ret`) and `localization/localization-data.hpp`.
8. **Tracking.** Read `tracks/ops.cpp` with `optimization/hungarian-algorithm.cpp` alongside it, then `fowlkes/` for the output format.
