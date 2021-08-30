
#pragma once

#include "perceive/io/json-io.hpp"
#include "perceive/io/struct-meta.hpp"

namespace perceive
{
struct LocalizationDataEnvelope;
}

namespace perceive::pipeline
{
struct CliArgs : public MetaCompatible
{
   CUSTOM_NEW_DELETE(CliArgs)

   virtual ~CliArgs() {}
   const vector<MemberMetaData>& meta_data() const noexcept override;

   // string argv0{""s}; // argv[0] passed on commandline
   string version{k_version};
   bool show_help{false};
   bool has_error{false};

   Json::Value pipeline_params{Json::nullValue};
   Json::Value manifest_params{Json::nullValue};
   bool has_pipeline_params() const noexcept;
   bool has_manifest_params() const noexcept;

   // If a manifest file is specifed on the command line, then
   // this field holds the abolute path of the directory name of that file.
   string manifest_opt_search_path = ""s;

   string stats_filename{""s};
   string tracks_filename{""s};
   Json::Value in_stats_filenames{Json::arrayValue};
   bool allow_overwrite{false};
   bool generate_stats{false};
   bool no_stats{true};
   bool generate_tracks{true};
   int verbosity{1}; // 0, 1, 2
   string outdir{"/tmp"s};
   int start_frame_no{0};
   int n_frames_to_process{-1};
   real target_fps{0.0}; // does not attempt to drop frames if set to 0.0
   string pipeline_output_filename{"pipeline-output.json"};
   bool generate_timings{false};
   bool save_localization_data{false};
   string localization_data_fname{""s};
   bool developer_features{false};
   bool on_the_fly_still{false};
   bool colour_floor_hist{false};
   string annotations_directory{""s}; // used by annotation system
   bool annotate_mode{false};
   bool no_stereo{false};
   bool export_training_data{false};
   string export_training_data_prefix
       = ""s; // should be set to the testcase name
   string pose_classifier{""s};
   string tpfp_classifier{""s};

   // Video output
   bool debug_video_output                = false;
   bool presentation_video_output         = false;
   int output_video_width                 = 2000;
   int output_video_height                = 1500;
   bool output_video_render_tracks        = true;
   bool output_video_render_pose          = false; // true
   bool output_video_p2ds_on_localization = true;
   bool output_video_blur_faces           = true;
   bool output_video_ground_truth_only    = false;

   // Localization data has to be loaded in order to
   // set the frame-rate, and start/end frame
   shared_ptr<const LocalizationDataEnvelope> ldata_env_ptr;

   // Running pipeline testcases
   bool run_pipeline_testcase{false};

   bool setup_testcase{false};

   bool outputting_video() const noexcept
   {
      return debug_video_output or presentation_video_output;
   }

   unsigned n_in_stats_fnames() const noexcept;
   void push_in_stats_fname(const string_view) noexcept;
   string in_stats_fname(const unsigned ind) const noexcept;

   // We're loading precalulcated stats from file somewhere
   bool is_loading_stats() const noexcept;

   // We need to generate stats before any run
   bool is_generating_stats_before_run() const noexcept;

   friend string str(const CliArgs& o) noexcept { return o.to_string(); }

   string multiview_run_object_str() const noexcept;
};

void show_help(string argv0) noexcept;

// @param callback Used whenever an unknown parameter is encountered.
//                 Returns TRUE if the parameter(s) are handled successfully.
CliArgs
parse_command_line(int argc,
                   char** argv,
                   std::function<bool(int argc, char** argv, int& i)> callback
                   = nullptr) noexcept;

} // namespace perceive::pipeline

namespace perceive
{
META_READ_WRITE_LOAD_SAVE(pipeline::CliArgs)

} // namespace perceive
