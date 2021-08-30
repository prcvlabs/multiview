
#pragma once

#include "manifest-data.hpp"

namespace perceive::calibration::position_scene_cameras
{
cv::Mat render_pointwise(const ArucoCube& ac,
                         const cv::Mat& image,
                         const CachingUndistortInverse& cu,
                         const vector<ArucoCube::FaceDetection>& detects,
                         const EuclideanTransform& cam_et, // Cam->World
                         const EuclideanTransform& cube_et // Cube->World
                         ) noexcept;

cv::Mat render_dense(const ArucoCube& ac,
                     const cv::Mat& image,
                     const CachingUndistortInverse& cu,
                     const vector<ArucoCube::FaceDetection>& detects,
                     const array<cv::Mat, 6>& face_ims,
                     const EuclideanTransform& cam_et, // Cam->World
                     const EuclideanTransform& cube_et // Cube->World
                     ) noexcept;

std::pair<vector<EuclideanTransform>, bool>
optimize_n_way(const string_view scene_id,
               const ManifestData& mdata,
               const string_view outdir);

} // namespace perceive::calibration::position_scene_cameras
