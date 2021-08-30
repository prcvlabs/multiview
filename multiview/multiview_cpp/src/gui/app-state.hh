
#pragma once

#include <atomic>
#include <mutex>

#include <QObject>

#include "cmd-line.hpp"

#include "perceive/calibration/plane-set/phase-plane-data.hpp"
#include "perceive/cost-functions/movie-stats/movie-stats-file.hpp"
#include "perceive/io/json-io.hpp"
#include "perceive/scene/scene-description-info.hpp"
#include "perceive/scene/scene-description.hpp"

#include "perceive/pipeline/cli-args.hpp"
#include "perceive/pipeline/frame-results.hpp"
#include "perceive/pipeline/movie-results.hpp"

class MainWindow;

/// A singleton that contains the global gui state.
/// It is is "thread-compatible".
/// QObject so that it can participate in signals and slots.
class AppState final : public QObject
{
   Q_OBJECT

 public:
   using AppConfig        = perceive::gui::AppConfig;
   using FrameResults     = perceive::pipeline::FrameResults;
   using MovieResults     = perceive::pipeline::MovieResults;
   using SceneDescription = perceive::SceneDescription;
   using MovieParams      = perceive::pipeline::MovieResults::Params;

 private:
   AppConfig config_; //!< The command line..
   MainWindow* main_window_{nullptr};

   unsigned cam_no_{0};
   unsigned sensor_pos_{0};
   unsigned n_cameras_{0};
   unsigned n_sensors_{0};
   std::shared_ptr<const SceneDescription> scene_desc_{nullptr};
   MovieParams movie_params_;
   bool dev_mode_{false};

   std::atomic<bool> quit_flag_{false}; //!< Set to TRUE to signal quit
   std::vector<int> sensor_counts_;

   std::unique_ptr<MovieResults> movie_results_ptr_{nullptr};

 public:
   std::shared_ptr<const SceneDescription> scene_desc() const noexcept;
   FrameResults* frame_results() noexcept;
   MovieResults* movie_results() noexcept;
   const FrameResults* frame_results() const noexcept;
   const MovieResults* movie_results() const noexcept;
   const MovieParams& movie_params() const noexcept { return movie_params_; }

   AppState();
   AppState(const AppState&) = delete;
   AppState(AppState&&)      = delete;
   virtual ~AppState();
   AppState& operator=(const AppState&) = delete;
   AppState& operator=(AppState&&) = delete;

   static AppState* instance() noexcept; //!< Singleton
   static void dispose_instance();       //!< Deletes the singlton instance.
   bool initialize(AppConfig config) noexcept; //!< Loads manifest file, etc.

   const AppConfig& config() const noexcept { return config_; }

   void set_main_window(MainWindow* main_window) noexcept
   {
      main_window_ = main_window;
   }

   int frame_no() const noexcept; // Frame number, -1 if none.
   int n_frames() const noexcept; // -1 if no video
   bool has_current_frame() const noexcept;
   void seek_frame(int frame_no) noexcept;

   // Camera
   unsigned n_cameras() const noexcept;
   unsigned current_camera() const noexcept;
   bool has_current_camera() const noexcept;

   // Sensor
   unsigned n_sensors() const noexcept;
   int current_sensor_index() const noexcept;    // index in scene-desc
   unsigned current_sensor_pos() const noexcept; // pos in stereo cam
   bool has_current_sensor() const noexcept;
   // Returns the sensor-index (scene-description order) for
   // the sensor in @sensor_pos of the current camera. If
   // there's no such sensor, or no current camera, then returns -1
   int sensor_index(int cam_num, int sensor_pos) const noexcept;

   void set_quit_flag() noexcept { quit_flag_ = true; }

   const std::vector<int>& sensor_counts() const noexcept;

   void set_dev_mode(bool val) noexcept;
   bool dev_mode() const noexcept { return dev_mode_; }

 signals:

   void open_finished(); // Must be connected as a QueuedConnection
   void camera_changed(unsigned);
   void sensor_pos_changed(unsigned);
   void dev_mode_set(bool);
   void video_frame_changed(int);

 public slots:

   void open_manifest_file(Json::Value data) noexcept;
   void on_set_video_frame(int frame_no) noexcept;
   void set_current_camera(unsigned cam_no) noexcept;
   void set_current_sensor_pos(unsigned sensor_pos) noexcept;

   void set_to_camera0() noexcept;
   void set_to_camera1() noexcept;
   void set_to_camera2() noexcept;
   void set_to_camera3() noexcept;
   void set_to_camera4() noexcept;
   void set_to_camera5() noexcept;
   void set_to_camera6() noexcept;
   void set_to_camera7() noexcept;
   void set_to_camera8() noexcept;
   void set_to_camera9() noexcept;
};

AppState* app_state() noexcept;
