
#include "app-state.hh"

#include "interface/main-window.hh"
#include "perceive/utils/file-system.hpp"

#define This AppState

using namespace perceive;

This::This()
    : QObject(nullptr)
{}

This::~This() = default;

struct AppStateManager
{
   std::mutex padlock_;
   unique_ptr<AppState> state_{make_unique<AppState>()};
   std::atomic<bool> is_disposed_{false};

   AppState* get() noexcept
   {
      if(is_disposed_) {
         // This is not a 100% fail-safe check
         FATAL(format("call to 'get' after dispose"));
      }
      return state_.get();
   }
   void dispose() noexcept
   {
      state_.release();
      is_disposed_ = true;
   }
};

AppStateManager& app_state_manager()
{
   static AppStateManager manager;
   return manager;
}

void This::dispose_instance() { app_state_manager().dispose(); }
AppState* This::instance() noexcept { return app_state_manager().get(); }
AppState* app_state() noexcept { return AppState::instance(); }

// --------------------------------------------------------------------- getters

shared_ptr<const SceneDescription> This::scene_desc() const noexcept
{
   return scene_desc_;
}

This::FrameResults* This::frame_results() noexcept
{
   return (movie_results()) ? movie_results()->frame_results.get() : nullptr;
}

const This::FrameResults* This::frame_results() const noexcept
{
   return (movie_results()) ? movie_results()->frame_results.get() : nullptr;
}

This::MovieResults* This::movie_results() noexcept
{
   return movie_results_ptr_.get();
}

const This::MovieResults* This::movie_results() const noexcept
{
   return movie_results_ptr_.get();
}

// ------------------------------------------------------------------ initialize

bool This::initialize(AppConfig in_config) noexcept
{
   bool has_error = false;
   config_        = std::move(in_config);

   set_dev_mode(config_.config.developer_features);

   // Initialize movie params
   if(!movie_params_.init(config_.config, false)) { return false; }

   // (*) ---- Load manifest file
   if(!config_.config.has_manifest_params()) {
      cout << format("must specify a manifest file!") << endl;
      return false;
   }

   return !has_error;
}

// ------------------------------------------------------------------

int This::frame_no() const noexcept
{
   if(frame_results() == nullptr) return -1;
   return frame_results()->copy_sensor_images.params().frame_num;
}

int This::n_frames() const noexcept
{
   if(scene_desc_) return scene_desc_->n_frames();
   return -1;
}

bool This::has_current_frame() const noexcept
{
   const auto fn = frame_no();
   const auto n  = n_frames();
   return fn >= 0 && fn < n;
}

void This::seek_frame(int frame_no) noexcept
{
   if(main_window_ == nullptr or frame_results() == nullptr) return;
   if(frame_no < 0) frame_no = 0;
   if(frame_no >= n_frames()) frame_no = n_frames() - 1;
   if(frame_no >= 0 && frame_no < n_frames()) {
      auto p      = frame_results()->copy_sensor_images.params();
      p.frame_num = frame_no;
      frame_results()->copy_sensor_images.set_params(p);
      main_window_->on_update_video_frame();
   }
}

unsigned This::n_cameras() const noexcept { return n_cameras_; }

unsigned This::current_camera() const noexcept { return cam_no_; }

bool This::has_current_camera() const noexcept
{
   return current_camera() < n_cameras();
}

unsigned This::n_sensors() const noexcept { return n_sensors_; }

int This::current_sensor_index() const noexcept
{
   return sensor_index(current_camera(), current_sensor_pos());
}

unsigned This::current_sensor_pos() const noexcept // not the index
{
   return sensor_pos_;
}

bool This::has_current_sensor() const noexcept
{
   return current_sensor_index() >= 0;
}

int This::sensor_index(int cam_num, int sensor_pos) const noexcept
{
   auto s = scene_desc();
   return s ? s->sensor_lookup(cam_num, sensor_pos) : -1;
}

// ------------------------------------------------------------------ initialize

void This::open_manifest_file(Json::Value manifest_data) noexcept
{
   using namespace perceive;

   auto thunk = [this, manifest_data]() {
      try {
         movie_params_.config.manifest_params = manifest_data;

         if(!movie_params_.load_manifest()) {
            LOG_ERR(format("failed to load manifest data"));
            return;
         }

         INFO(format("Loaded Scene: {:s}\n\n",
                     movie_params_.scene_desc->scene_info.to_string()));

         { // Create the pipeline
            movie_results_ptr_ = make_unique<MovieResults>(movie_params_);
            frame_results()->output_dot_graph(config().config.outdir);
         }

         if(movie_params_.config.generate_stats) {
            WARN(format("movie-stats deprecated -- not doing anything."));
         }

         { // Adjust the number of tracks objects to store
         }

         auto calc_sensor_counts = [&]() {
            vector<int> ret;
            ret.resize(movie_params_.scene_desc->n_cameras());
            for(auto i = 0u; i < ret.size(); ++i)
               ret[i] = movie_params_.scene_desc->n_sensors_for(i);
            return ret;
         };
         sensor_counts_ = calc_sensor_counts();

         n_cameras_ = movie_params_.scene_desc->n_cameras();
         n_sensors_ = movie_params_.scene_desc->n_sensors();

         scene_desc_ = movie_params_.scene_desc;

         // Output the used parameters
         const string params_fname
             = format("{:s}/params-at-load.json", config().config.outdir);
         {
            std::stringstream ss{""};
            ss << movie_params_.params.to_json() << endl;
            file_put_contents(params_fname, ss.str());
         }

         INFO(format(R"V0G0N(Created pipeline
 * saving task-graph to '{:s}'
 * saving initial parameters to '{:s}'
{:s})V0G0N",
                     config().config.outdir,
                     params_fname,
                     ""));

         on_set_video_frame(config().config.start_frame_no);

         emit this->open_finished();

      } catch(std::exception& e) {
         LOG_ERR(
             format("error initializing scene-description: {:s}", e.what()));
      }
   };

   schedule(thunk);
}

// ---------------------------------------------------------- on set video frame

void This::on_set_video_frame(int frame_no) noexcept
{
   if(frame_results() == nullptr) return;
   auto& task = frame_results()->copy_sensor_images;

   auto p      = task.params();
   p.frame_num = frame_no;
   task.set_params(p);
   emit video_frame_changed(frame_no);
}

// ---------------------------------------------------------- set current camera

void This::set_current_camera(unsigned in_cam_no) noexcept
{
   if(n_cameras() == 0) return;

   const int cam_no = std::clamp(int(in_cam_no), 0, int(n_cameras()) - 1);
   if(cam_no >= 0 and int(current_camera()) != cam_no) {
      cam_no_ = unsigned(cam_no);
      emit camera_changed(cam_no);
   }
}

// ------------------------------------------------------ set current sensor pos

void This::set_current_sensor_pos(unsigned in_sensor_pos) noexcept
{
   if(n_cameras() == 0) return;
   Expects(current_camera() < sensor_counts().size());
   const int count      = sensor_counts()[current_camera()];
   const int sensor_pos = std::clamp(int(in_sensor_pos), 0, count - 1);
   if(sensor_pos >= 0 and sensor_pos != int(current_sensor_pos())) {
      sensor_pos_ = unsigned(sensor_pos);
      emit sensor_pos_changed(sensor_pos);
   }
}

// -------------------------------------------- number of sensors on each camera

const vector<int>& This::sensor_counts() const noexcept
{
   return sensor_counts_;
}

// ---------------------------------------------------------------- set dev mode

void This::set_dev_mode(bool val) noexcept
{
   dev_mode_ = val;
   emit this->dev_mode_set(val);
}

// ------------------------------------------------------------- set to camera N

void This::set_to_camera0() noexcept { set_current_camera(0); }
void This::set_to_camera1() noexcept { set_current_camera(1); }
void This::set_to_camera2() noexcept { set_current_camera(2); }
void This::set_to_camera3() noexcept { set_current_camera(3); }
void This::set_to_camera4() noexcept { set_current_camera(4); }
void This::set_to_camera5() noexcept { set_current_camera(5); }
void This::set_to_camera6() noexcept { set_current_camera(6); }
void This::set_to_camera7() noexcept { set_current_camera(7); }
void This::set_to_camera8() noexcept { set_current_camera(8); }
void This::set_to_camera9() noexcept { set_current_camera(9); }
