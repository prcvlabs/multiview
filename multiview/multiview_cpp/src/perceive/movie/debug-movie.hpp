
#pragma once

#include "perceive/cost-functions/localization/localization-data.hpp"
#include "perceive/pipeline/cli-args.hpp"

namespace perceive
{
struct PipelineOutput;
struct LocalizationData;
class Tracklet;
class Tracks;
struct Track;
struct FowlkesResult;
struct SceneDescription;
} // namespace perceive

namespace perceive::pipeline
{
class MovieResults;
struct TestOutput;
} // namespace perceive::pipeline

namespace perceive::tracks
{
struct ComputationData;
}

namespace perceive::movie
{
// ---------------------------------------------- functions for making filenames
//
string make_cam_fname(const string_view directory,
                      const int frame_no,
                      const int cam_ind,
                      const int sensor_num = 0) noexcept;

string make_loc_fname(const string_view directory, const int frame_no) noexcept;

string make_tracklet_fname(const string_view directory,
                           const int frame_no) noexcept;

string make_tracks_fname(const string_view directory,
                         const int frame_no) noexcept;

string make_composite_fname(const string_view directory,
                            const int frame_no) noexcept;

// ---------------------------------------------- make debug movies (and params)
//
struct DebugMovieParams
{
   int w                              = 1024;
   int h                              = 768;
   bool render_tracks_on_raw_video    = true;
   bool render_grid_on_raw_video      = true;
   bool render_pose                   = true;  // true
   bool render_p2d_hist_on_raw_video  = false; // true
   bool p2ds_on_localization          = true;
   bool grid_on_localization          = true;
   bool blur_faces                    = true;
   bool render_ground_truth           = false;
   bool render_false_positives        = false;
   bool render_ground_truth_only      = false;
   bool render_height_boxes           = true;
   real person_radius                 = 0.2;
   bool render_plausible_heights_line = false; // true
   bool render_p2d_cylinders          = true;
   bool print_timings                 = false;

   DebugMovieParams(int w_ = 1024, int h_ = 768);
   DebugMovieParams(const pipeline::CliArgs& args);
   DebugMovieParams(const DebugMovieParams&) = default;
   DebugMovieParams(DebugMovieParams&&)      = default;
   ~DebugMovieParams()                       = default;
   DebugMovieParams& operator=(const DebugMovieParams&) = default;
   DebugMovieParams& operator=(DebugMovieParams&&) = default;
};

struct RenderSensorTimings
{
   int total_p2ds                    = 0;
   real blur_faces_s                 = 0.0;
   real render_grid_s                = 0.0;
   real render_p2d_hists_s           = 0.0;
   real render_heights_line_s        = 0.0;
   real render_p2d_cylinders_s       = 0.0;
   real render_pose_skeleton_s       = 0.0;
   real render_tracks_on_raw_video_s = 0.0;

   RenderSensorTimings& operator+=(const RenderSensorTimings&) noexcept;
};

// pair: <pose-labels, timings>
// std::pair<vector<std::pair<Point2, string>>, RenderSensorTimings>
// render_sensor(const SceneDescription* scene_desc,
//               const DebugMovieParams& params,
//               const LocalizationData::Params& loc_params,
//               const LocalizationData* loc_dat,
//               const FowlkesResult* fowlkes_result_ptr, // could be nullptr
//               const int frame_no,
//               const int sensor_no,
//               ARGBImage& argb);

void make_debug_movie(
    const SceneDescription* scene_desc,
    const DebugMovieParams& params,
    const LocalizationData::Params& loc_params,
    const int start_frame,
    const int n_frames,
    std::function<const LocalizationData*(int frame_no)> get_loc,
    std::function<const Tracks*(int frame_no)> get_tracks,
    std::function<const tracks::ComputationData*(int frame_no)> get_comp_data,
    const FowlkesResult* fowlkes_result,            // could be nullptr
    const pipeline::TestOutput* gt_compare_ret_ptr, // could be nullptr
    const string_view working_dir,
    const string_view out_movie_name) noexcept;

void make_debug_movie(
    const PipelineOutput& pipeline_output,
    const FowlkesResult& fowlkes_result,
    const pipeline::TestOutput* gt_compare_ptr, // could be nullptr
    pipeline::MovieResults* movie,
    const DebugMovieParams& params) noexcept;

} // namespace perceive::movie
