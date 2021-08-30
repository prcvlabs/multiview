
#pragma once

#include "perceive/foundation.hpp"
#include "perceive/geometry/euclidean-transform.hpp"
#include "perceive/geometry/projective/caching-undistort-inverse.hpp"
#include "perceive/geometry/projective/distorted-camera.hpp"
#include "perceive/geometry/skeleton/skeleton-2d.hpp"
#include "perceive/io/json-io.hpp"
#include "perceive/io/struct-meta.hpp"
#include "perceive/scene/scene-description.hpp"

namespace perceive::pose_skeleton
{
// ------------------------------------------------------------------ RenderMode

enum class RenderMode : int { NONE = 0, CPU, GPU };

RenderMode int_to_render_mode(int) noexcept;
const char* str(const RenderMode) noexcept;

// -------------------------------------------------------------- OpenposeParams

struct OpenposeParams final : public MetaCompatible
{
   virtual ~OpenposeParams() {}
   const vector<MemberMetaData>& meta_data() const noexcept override;

   bool feedback{false};        //!< Prints feedback
   bool single_threaded{false}; //!< Cannot turn off once it's turned on

   // Only use sensor0
   bool only_one_sensor_per_camera{true};
   int image_patch_size{7}; //!< That is a WxH box around each keypoint

   // Pose parameters
   bool pose_enabled{true};
   bool openpose_serial{false};
   // Point2 net_input_size{656, 368}; //!< Higher=slower. Must be div by 16.
   Point2 net_input_size{960, 720}; //!< Higher=slower. Must be div by 16.
   int gpu_start{0};                //!< First GPU index to use
   int num_gpus{-1};                //!< -1 means all GPUS
   int scales_num{1};               //!< think n-layers in an image pyramid
   float scale_gap{0.15f};          //!< think layer-gap in image pyramid
   RenderMode render_mode{
       RenderMode::NONE}; //!< GPU gracefully falls back to CPU
   Skeleton2D::PoseModel pose_model{
       Skeleton2D::BODY_25};        //!< faster versus better
   bool bg_heatmap{false};          //!< Background heatmap
   bool body_heatmap{false};        //!<
   bool PAF_heatmap{false};         //!< part_affinity_field
   bool add_part_candidates{false}; //!<
   float render_threshold{0.05f};   //!< confidence threshold for rendering
   int max_n_people{-1};            //!< -1 for unlimited
   bool maximize_recall{false};     //!< increase true and false positives
};

// ------------------------------------------------------------- HyperposeParams
// Models:
// "lopps-resnet50-V2-HW=368x432.onnx"
// "openpose-coco-V2-HW=368x656.onnx"
// "openpose-thin-V2-HW=368x432.onnx"
// "ppn-resnet50-V2-HW=384x384.onnx"
// "TinyVGG-V1-HW=256x384.uff"
//
constexpr array<string_view, 4> hyperpose_models
    = {{"lopps-resnet50-V2-HW=368x432.onnx",
        "openpose-coco-V2-HW=368x656.onnx",
        "openpose-thin-V2-HW=368x432.onnx",
        "ppn-resnet50-V2-HW=384x384.onnx"}};

struct HyperposeParams final : public MetaCompatible
{
   virtual ~HyperposeParams() {}
   const vector<MemberMetaData>& meta_data() const noexcept override;

   string model_name         = "openpose-coco-V2-HW=368x656.onnx"s;
   Point2 network_resolution = {-1, -1}; // extract resolution from name

   bool feedback = false;
};

// ---------------------------------------------------------------------- Params

struct Params final : public MetaCompatible
{
   virtual ~Params() {}
   const vector<MemberMetaData>& meta_data() const noexcept override;

   bool use_hyperpose = false;
   OpenposeParams op_params;
   HyperposeParams hp_params;

   bool feedback = false;
};

} // namespace perceive::pose_skeleton

namespace perceive
{
META_READ_WRITE_LOAD_SAVE(pose_skeleton::Params)
}
