
#pragma once

#include "perceive/foundation.hpp"
#include "perceive/io/json-io.hpp"
#include "perceive/io/struct-meta.hpp"

namespace perceive
{
// Class for executing OpenPose
class OpExec final
{
 private:
   struct Pimpl;
   unique_ptr<Pimpl> pimpl_;

 public:
   enum RenderMode : int { NONE = 0, CPU, GPU };
   enum PoseModel : int { BODY_25, COCO_18, MPI_15, MPI_15_4 };

   static RenderMode int_to_render_mode(int) noexcept;
   static PoseModel int_to_pose_model(int) noexcept;

   struct Params final : public MetaCompatible
   {
      virtual ~Params() {}
      const vector<MemberMetaData>& meta_data() const noexcept override;

      bool feedback{false};        //!< Prints feedback
      bool single_threaded{false}; //!< Cannot turn off once it's turned on

      // Pose parameters
      bool pose_enabled{true};
      bool openpose_serial{false};
      Point2 net_input_size{656, 368}; //!< Higher=slower. Must be div by 16.
      int gpu_start{0};                //!< First GPU index to use
      int num_gpus{-1};                //!< -1 means all GPUS
      int scales_num{1};               //!< think n-layers in an image pyramid
      float scale_gap{0.15f};          //!< think layer-gap in image pyramid
      RenderMode render_mode{GPU};     //!< GPU gracefully falls back to CPU
      PoseModel pose_model{BODY_25};   //!< faster versus better
      bool bg_heatmap{false};          //!< Background heatmap
      bool body_heatmap{false};        //!<
      bool PAF_heatmap{false};         //!< part_affinity_field
      bool add_part_candidates{false}; //!<
      float render_threshold{0.05f};   //!< confidence threshold for rendering
      int max_n_people{-1};            //!< -1 for unlimited
      bool maximize_recall{false};     //!< increase true and false positives
   };

   struct Keypoint final
   {
      int32_t part = 0; // depends on the model
      float x      = 0.0;
      float y      = 0.0;
      float score  = 0.0;

      bool operator==(const Keypoint& o) const noexcept;
      bool operator!=(const Keypoint& o) const noexcept;

      const Vector2f xy() const noexcept { return Vector2f{x, y}; }
   };

   struct Pose final
   {
      PoseModel model{BODY_25};
      vector<Keypoint> keypoints;
      real theta{0.0}; // gaze angle
      real score{0.0};

      bool operator==(const Pose& o) const noexcept;
      bool operator!=(const Pose& o) const noexcept;

      Json::Value to_json() const noexcept;
      void read(const Json::Value&) noexcept(false);
      string to_json_str() const noexcept;

      // These functions return "NAN" keypoints weren't detected/don't exist
      Vector2 nose() const noexcept;
      Vector2 neck() const noexcept;
      Vector2 pelvis() const noexcept;
      Vector2 l_eye() const noexcept;       // eye
      Vector2 r_eye() const noexcept;       //
      Vector2 l_ear() const noexcept;       // ear
      Vector2 r_ear() const noexcept;       //
      Vector2 l_shoulder() const noexcept;  // shoulder
      Vector2 r_shoulder() const noexcept;  //
      Vector2 l_elbow() const noexcept;     // elbow
      Vector2 r_elbow() const noexcept;     //
      Vector2 l_wrist() const noexcept;     // wrist
      Vector2 r_wrist() const noexcept;     //
      Vector2 l_hip() const noexcept;       // hip
      Vector2 r_hip() const noexcept;       //
      Vector2 l_knee() const noexcept;      // knee
      Vector2 r_knee() const noexcept;      //
      Vector2 l_ankle() const noexcept;     // ankle
      Vector2 r_ankle() const noexcept;     //
      Vector2 l_heal() const noexcept;      // heal
      Vector2 r_heal() const noexcept;      //
      Vector2 l_big_toe() const noexcept;   // big toe
      Vector2 r_big_toe() const noexcept;   //
      Vector2 l_small_toe() const noexcept; // small toe
      Vector2 r_small_toe() const noexcept; //

      Vector2 feet() const noexcept; // average of left and right ankle/feet
   };

   // -------------------------------------------------- Construction/Assignment
   OpExec();
   OpExec(const OpExec&)     = delete;
   OpExec(OpExec&&) noexcept = default;
   ~OpExec();
   OpExec& operator=(const OpExec&) = delete;
   OpExec& operator=(OpExec&&) noexcept = default;

   // ---------------------------------------------------------------------- Run
   vector<Pose> run(const cv::Mat& image, const Params& p) noexcept;

   friend const char* str(const OpExec::RenderMode) noexcept;
   friend const char* str(const OpExec::PoseModel) noexcept;
   friend string str(const vector<Pose>&) noexcept;
};

static_assert(is_nothrow_move_assign<OpExec>(), "must be nothrow");

} // namespace perceive

// C++ API call
// #include <openpose/pose/poseParameters.hpp>
// const auto& poseBodyPartMappingBody25
//     = getPoseBodyPartMapping(PoseModel::BODY_25);
// const auto& poseBodyPartMappingCoco
//     = getPoseBodyPartMapping(PoseModel::COCO_18);
// const auto& poseBodyPartMappingMpi
//     = getPoseBodyPartMapping(perceive::OpExec::PoseModel::MPI_15);

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
