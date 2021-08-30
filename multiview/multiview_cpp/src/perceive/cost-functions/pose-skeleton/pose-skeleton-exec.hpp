
#pragma once

#include "params.hpp"

#include "perceive/foundation.hpp"
#include "perceive/geometry/euclidean-transform.hpp"
#include "perceive/geometry/projective/caching-undistort-inverse.hpp"
#include "perceive/geometry/projective/distorted-camera.hpp"
#include "perceive/geometry/skeleton/skeleton-2d.hpp"
#include "perceive/io/json-io.hpp"
#include "perceive/io/struct-meta.hpp"
#include "perceive/scene/scene-description.hpp"

namespace perceive
{
// Class for executing OpenPose
class PoseSkeletonExec final
{
 private:
   struct Pimpl;
   unique_ptr<Pimpl> pimpl_;

 public:
   // using PoseModel = Skeleton2D::PoseModel;

   struct Result final
   {
    private:
      // Use _another_ pimpl??
      struct Pimpl;
      unique_ptr<Pimpl> pimpl_;

      friend class PoseSkeletonExec;

    public:
      Result(int n_sensors = 0, int frame_no = 0);
      Result(Result&&);
      Result(const Result&) = delete;
      ~Result();
      Result& operator=(Result&&);
      Result& operator=(const Result&) = delete;

      int frame_no() const noexcept;
      unsigned size() const noexcept; // n sensor images
      bool valid_index(int ind) const noexcept;
      vector<unsigned> sensors() const noexcept;

      // Detected poses for a given sensor image... one per scene-sensor
      const vector<shared_ptr<const Skeleton2D>>& pose(int ind) const noexcept;

      float seconds{0.0}; // execution time
   };

   // -------------------------------------------------- Construction/Assignment
   PoseSkeletonExec();
   PoseSkeletonExec(const PoseSkeletonExec&)     = delete;
   PoseSkeletonExec(PoseSkeletonExec&&) noexcept = default;
   ~PoseSkeletonExec();
   PoseSkeletonExec& operator=(const PoseSkeletonExec&) = delete;
   PoseSkeletonExec& operator=(PoseSkeletonExec&&) noexcept = default;

   // ---------------------------------------------------------------------- Run
 private:
   Result run_(const SceneDescription* scene_desc_ptr,
               const int frame_no,
               const vector<cv::Mat>& images,
               const pose_skeleton::Params& p,
               const string_view outdir,
               std::function<bool()> is_cancelled) noexcept;

 public:
   Result run(const SceneDescription& scene_desc,
              const int frame_no,
              const vector<cv::Mat>& images,
              const pose_skeleton::Params& p,
              const string_view outdir,
              std::function<bool()> is_cancelled) noexcept;

   Result run(const cv::Mat& im,
              const pose_skeleton::Params& p,
              const string_view outdir) noexcept;
};

static_assert(is_nothrow_move_assign<PoseSkeletonExec>(), "must be nothrow");

//

} // namespace perceive

// C++ API call
// #include <openpose/pose/poseParameters.hpp>
// const auto& poseBodyPartMappingBody25
//     = getPoseBodyPartMapping(PoseModel::BODY_25);
// const auto& poseBodyPartMappingCoco
//     = getPoseBodyPartMapping(PoseModel::COCO_18);
// const auto& poseBodyPartMappingMpi
//     = getPoseBodyPartMapping(perceive::PoseSkeletonExec::PoseModel::MPI_15);

// --------------------------------------------------------------------- BODY_25
// @see
// https://github.com/CMU-Perceptual-Computing-Lab/openpose/blob/master/doc/output.md
// Result for BODY_25 (25 body parts consisting of COCO + foot)
// const std::map<unsigned int, std::string> POSE_BODY_25_BODY_PARTS {
//     {0,  "Nose"},
//     {1,  "Neck"},
//     {2,  "RShoulder"},
//     {3,  "RElbow"},
//     {4,  "RWrist"},
//     {5,  "LShoulder"},
//     {6,  "LElbow"},
//     {7,  "LWrist"},
//     {8,  "MidHip"},
//     {9,  "RHip"},
//     {10, "RKnee"},
//     {11, "RAnkle"},
//     {12, "LHip"},
//     {13, "LKnee"},
//     {14, "LAnkle"},
//     {15, "REye"},
//     {16, "LEye"},
//     {17, "REar"},
//     {18, "LEar"},
//     {19, "LBigToe"},
//     {20, "LSmallToe"},
//     {21, "LHeel"},
//     {22, "RBigToe"},
//     {23, "RSmallToe"},
//     {24, "RHeel"},
//     {25, "Background"}
// };
