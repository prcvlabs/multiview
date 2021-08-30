
#pragma once

#include "phase-plane-data.hpp"
#include "slic-data.hpp"

#include "perceive/geometry/euclidean-transform.hpp"
#include "perceive/geometry/projective/binocular-camera.hpp"
#include "perceive/scene/scene-description.hpp"

namespace perceive::calibration
{
// ---------

struct PhasePlaneOptData
{
   ParallelJobSet pjobs;

   string scene_key{""s};
   vector<PhasePlaneData::PlaneInfo> p3s;
   vector<PhasePlaneData::RayPlanePlaneInfo> rpps;
   vector<Point2> rpp_bcam_sensor; // bcam_infos[x.x].M[x.y]

   vector<EuclideanTransform> et;
   vector<const BinocularCameraInfo*> bcam_infos;
   vector<array<CachingUndistortInverse, 2>> cus;
   vector<array<LABImage, 2>> stills;
   vector<array<ARGBImage, 2>> argb_stills;
   vector<array<SlicData, 2>> slic_data;
   vector<array<cv::Mat, 2>> cv_ims;

   // auto x = sensor_lookup["STR..."]
   // Then x.x is the bcam_info, and x.y is the sensor index in bcam
   std::unordered_map<string, Point2> sensor_lookup;

   // Working data
   vector<array<cv::Mat, 2>> cv_dsts;
   vector<array<cv::Mat, 2>> mapxs, mapys;

   // masks.size() == p3s.size()
   // masks[i].size() == bcam_infos.size()
   vector<vector<array<BinaryImage, 2>>> masks; // for each camera/plane
   vector<vector<array<int, 2>>> mask_counters;

   // all_masks.size() == bcam_infos.size()
   // combines all masks together for a single camera
   vector<array<IntImage, 2>> all_masks;

   real image_match_score_weight = 10.0; // compared to reproj error

   int n_cams() const noexcept { return int(bcam_infos.size()); }
   int n_p3s() const noexcept { return int(p3s.size()); }
   int n_rpps() const noexcept { return int(rpps.size()); }

   // slic_datas: one per camera in 'scene_desc'
   void init(const SceneDescription& scene_desc,
             const PhasePlaneData& data) noexcept;

   real error(const string& outdir = "/tmp"s,
              const bool feedback  = false) noexcept;

   cv::Mat make_ref_disparity_map(const int cam_no,
                                  const string& outdir) noexcept;
};

// ---------

ARGBImage make_phase_plane_image(
    const BinocularCameraInfo& bcam_info,
    const EuclideanTransform& bcam_et,
    const array<CachingUndistortInverse, 2>& cu,
    const vector<const SlicData*>& slic,     // sensor images to convolve
    const Plane& p3,                         // plane of interest
    const vector<std::vector<int>>& selected // slic indices, one per sensor
    ) noexcept;

// ---------

// NOTE:
// if 'et_inout.size()' == 0, then positions are read from scene_desc
real optimize_cameras(const SceneDescription& scene_desc,
                      const PhasePlaneData& data,
                      const vector<array<SlicData*, 2>>& slic_datas,
                      const bool use_nelder_mead, // true is a good bet
                      vector<EuclideanTransform>& et_inout,
                      vector<string>& cam_ids_out,
                      const string& outdir,
                      const bool feedback) noexcept;

real optimize_p3s(const SceneDescription& scene_desc,
                  const PhasePlaneData& phase_plane_data,
                  const vector<array<SlicData*, 2>>& slic_datas,
                  const bool use_nelder_mead, // true is a good bet
                  vector<EuclideanTransform>& et_out,
                  vector<string>& cam_ids_out,
                  PhasePlaneData& opt_out,
                  const string& outdir,
                  const bool feedback) noexcept;

real optimize_all(const SceneDescription& scene_desc,
                  const PhasePlaneData& data,
                  const vector<array<SlicData*, 2>>& slic_datas,
                  const bool use_nelder_mead, // true is a good bet
                  vector<EuclideanTransform>& et_inout,
                  vector<string>& cam_ids_out,
                  PhasePlaneData& data_out,
                  const string& outdir,
                  const bool feedback) noexcept;

} // namespace perceive::calibration
