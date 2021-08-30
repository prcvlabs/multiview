
#pragma once

#include "perceive/scene/scene-description.hpp"

#include "nodes/nodes.hpp"

namespace perceive::pipeline
{
class FrameResults
{
 private:
   std::function<void(const TaskNode* task, real seconds)> on_task_finished_;
   void init_(shared_ptr<const SceneDescription> scene_desc, bool feedback);
   std::unordered_map<string, const TaskNode*> task_lookup_;

 public:
   CUSTOM_NEW_DELETE(FrameResults)

   // -------------------------------------------------------------------- Tasks
   load_scene_description::Task load_scene_description;
   movie_stats::Task movie_stats; // For LOADING stats
   copy_sensor_images::Task copy_sensor_images;
   pose_skeleton_init::Task pose_skeleton_init;

   // gui_sensor_images::Task gui_sensor_images;
   // openpose_registration::Task openpose_registration;

   vector<unique_ptr<input_images::Task>> input_images;
   vector<unique_ptr<convert_to_lab::Task>> convert_to_lab;
   vector<unique_ptr<bcam_init::Task>> bcam_init;
   vector<unique_ptr<run_f2d::Task>> run_f2d;

   vector<unique_ptr<calc_calibration_mask::Task>> calc_calibration_mask;
   vector<unique_ptr<calc_rectified::Task>> calc_rectified;
   vector<unique_ptr<get_xy_mapping::Task>> get_xy_mapping;

   vector<unique_ptr<create_slic_lookup::Task>> create_slic_lookup;
   vector<unique_ptr<disp_init::Task>> disp_init;
   vector<unique_ptr<disp_map_update::Task>> disp_map_update; // 1 per camera

   floor_hist::Task floor_hist;
   localization::Task localization;

   // ------------------------------------------------------------- Construction
   FrameResults(shared_ptr<const SceneDescription> scene_desc, bool feedback);
   FrameResults(const FrameResults&);
   FrameResults(FrameResults&&) = delete;
   ~FrameResults()              = default;

   FrameResults& operator=(const FrameResults&);
   FrameResults& operator=(FrameResults&&) = delete;
   bool operator==(const FrameResults& o) const noexcept;
   bool operator!=(const FrameResults& o) const noexcept;

   void output_dot_graph(const std::string& outdir) noexcept;
   void set_frame(int frame_number) noexcept;
   void set_feedback(bool feedback) noexcept;
   void set_output_dir(const std::string& out_dir);
   void set_task_finished_callback(
       std::function<void(const TaskNode* task, real seconds)> f) noexcept;

   void process_all_tasks() noexcept;

   Json::Value params_to_json() const noexcept;
   bool read_params(const Json::Value& o, const string_view path) noexcept;
};

} // namespace perceive::pipeline
