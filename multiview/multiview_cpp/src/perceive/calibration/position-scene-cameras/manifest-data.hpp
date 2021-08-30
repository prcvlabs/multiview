
#pragma once

#include "config.hpp"

#include "perceive/calibration/aruco-cube.hpp"
#include "perceive/geometry/projective/binocular-camera.hpp"
#include "perceive/geometry/projective/caching-undistort-inverse.hpp"

namespace perceive::calibration::position_scene_cameras
{
struct CamImageData
{
   string name                              = {}; // position-camera_id
   string image_fname                       = {};
   string camera_id                         = {};
   cv::Mat raw_image                        = {}; // left and right image
   cv::Mat image                            = {}; // Left image only
   cv::Mat undistorted                      = {}; // if `image`
   EuclideanTransform init_et               = {}; // initial LOCAL estimate
   vector<ArucoCube::FaceDetection> detects = {}; // pointwise detections
   size_t position_index                    = 0;
   size_t cam_index                         = 0;
};

struct CamCamConstraints
{
   OrderedPair cam_indices       = {}; // two cameras
   vector<int> cam_image_indices = {}; // the cdat indices they're related by
};

struct PositionInfo
{
   string position_name             = ""s;
   EuclideanTransform init_cube_pos = {}; // global transformation for the cube
   vector<int> cdats                = {}; // CamImageData indices in mdata
   EuclideanTransform init_et       = {}; // scene-GLOBAL, World->cube
};

struct CamInfo
{
   string camera_id              = ""s;
   BinocularCameraInfo bcam_info = {};
   Point2 working_format         = Point2{0, 0};
   CachingUndistortInverse cu    = {}; // left sensor only
   array<cv::Mat, 2> mapxys      = {}; // left sensor only
   vector<int> positions         = {}; // all the positions for a given camera
   vector<int> cdat_path         = {}; // to reference camera
   EuclideanTransform init_et    = {}; // scene-GLOBAL, World->cam
};

struct ManifestData
{
   ArucoCube ac;
   array<cv::Mat, 6> face_ims       = {}; // face images of aruco cubes
   unsigned undistorted_w           = 0;
   unsigned undistorted_h           = 0;
   Matrix3r K                       = {}; // for undistorted image
   size_t default_position_index    = 0;
   vector<PositionInfo> positions   = {}; // every position
   vector<CamImageData> data        = {}; // every camera-position pair
   vector<CamInfo> cam_infos        = {}; // every camera
   vector<array<cv::Mat, 2>> mapxys = {}; // for undistorted

   // maybe not use this (???)
   std::unordered_map<OrderedPair, CamCamConstraints> constraints = {};

   // Returns bcam_infos.size() if camera-id doesn't exist
   size_t lookup_position(const string_view position_name) const noexcept;
   size_t lookup_camera_index(const string_view camera_id) const noexcept;
   vector<std::pair<int, int>> get_cam_cam_constraints() const noexcept;
   int n_cameras() const noexcept { return int(cam_infos.size()); }
   int n_positions() const noexcept { return int(positions.size()); }
};

ManifestData parse_manifest_file(const Config& config) noexcept(false);

void cdat_fn_test(const ManifestData& mdata);

} // namespace perceive::calibration::position_scene_cameras
