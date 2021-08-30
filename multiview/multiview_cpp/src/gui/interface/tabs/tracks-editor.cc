
#include "stdinc.hpp"

#include <QDir>
#include <QInputDialog>
#include <QMouseEvent>

#include "tracks-editor.hh"

#include "gui/app-state.hh"
#include "gui/interface/interface-helpers.hpp"
#include "gui/qt-helpers.hpp"
#include "gui/widgets/flow-layout.hpp"
#include "gui/widgets/image-viewer-2.hh"
#include "gui/widgets/labeled-slider.hh"

#include "perceive/cost-functions/tracks/labeled-track-point.hpp"
#include "perceive/io/lazy-s3.hpp"
#include "perceive/movie/movie-layout.hpp"
#include "perceive/movie/movie-utils.hpp"
#include "perceive/pipeline/pipeline-output.hpp"
#include "perceive/scene/annotations.hpp"

#define This TracksEditor

using namespace perceive;

// ---------------------------------------------------------------- instructions

static QLabel* make_instructions_label()
{
   QLabel* l = new QLabel{};
   l->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
   l->setFixedHeight(575);
   QFont font("Arial", 8);
   l->setFont(font);
   l->setWordWrap(true);
   l->setText(to_qstr(R"V0G0N(
<table>
<tr><th width=150>Key</th><th>Description</th></tr>
<tr><td>Left Click:  </td>
    <td>To select. Shift+Left-Click to drag a track-point.</td></tr>
<tr><td>Arrow Keys:  </td>
    <td>Move the selected track.</td></tr>
<tr><td>Ctrl+Right Click: </td>
    <td>Update the view-direction of the selected track-point.</td></tr>
<tr><td>Ctrl+I             </td>
    <td>Edit the track-id.</td></tr>
<tr><td>Ctrl+V:            </td>
    <td>Remove the view-direction of the selected track-point.</td></tr>
<tr><td colspan="2"><hr/></td></tr>
<tr><td>Ctrl+Left Click:   </td>
    <td>Create a new track-point under the cursor.</td></tr>
<tr><td>Ctrl+N:            </td>
    <td>Create a new track-point from the selected track point in 
        the previous frame.</td></tr>
<tr><td>Del:          </td>
    <td>Delete the selected track-point.</td></tr>
<tr><td colspan="2"><hr/></td></tr>
<tr><td>Letter sets/unsets pose:</td>
    <td><span style="text-decoration: underline">N</span>one, 
        <span style="text-decoration: underline">S</span>tand, 
        <span style="text-decoration: underline">W</span>alk,<br/>
        S<span style="text-decoration: underline">i</span>t, 
        <span style="text-decoration: underline">L</span>ay, 
        <span style="text-decoration: underline">P</span>hone, 
        <span style="text-decoration: underline">O</span>ther
    </td></tr>

<tr><td colspan="2"><hr/></td></tr>
<tr><td>Ctrl+Left/Right Arrow</td><td>Rotate floor view.</td></tr>
<tr><td>Ctrl+Plus/Minus</td><td>Zoom floor view in/out.</td></tr>

<tr><td colspan="2"><hr/></td></tr>
<tr><td>PageUp/PageDown</td><td>Move to next/previous track.</td></tr>
<tr><td>Home/End</td><td>Move to first/last track.</td></tr>

<tr><td colspan="2"><hr/></td></tr>
<tr><td>Ctrl+Z, Ctrl+R   </td><td>Undo/Redo</td></tr>
<tr><td colspan="2"><hr/></td></tr>
<tr><td>Ctrl+[1-9]:      </td><td>Select camera view.</td></tr>
<tr><td>Ctrl+{ / }:      </td><td>Back/forward one movie frame.</td></tr>
<tr><td>Ctrl+Shift+{ / }: </td><td>Jump 10 frames back/forward.</td></tr>
<tr><td>Ctrl+J:          </td><td>Jump to frame number.</tr></tr>
<tr><td colspan="2"><hr/></td></tr>
<tr><td>Alt+C:             </td>
    <td>Check output... results written to stdout</td></tr>
<tr><td>Alt+S, or Ctrl+S:   </td>
    <td>Save to output file.</td></tr>
</table>
)V0G0N"));
   return l;
}

static int inverse_rotation(int val)
{
   switch(val) {
   case 0: return 0;
   case 1: return 3;
   case 2: return 2;
   case 3: return 1;
   }
   FATAL("logic error");
   return 0;
}

static Vector2
apply_rotation(const int rotation, Vector2 X, const real w, const real h)
{
   switch(rotation) {
   case 0: return X;                                     // no rotation
   case 1: return Vector2{X.y, w - X.x - 1.0};           // 90 clockwise
   case 2: return Vector2{w - X.x - 1.0, h - X.y - 1.0}; // 180
   case 3: return Vector2{h - X.y - 1.0, X.x};           // 270 clockwise
   default: FATAL("logic error");
   }
   return Vector2{};
}

// ----------------------------------------------------------------------- pimpl

using Tracks = perceive::Tracks;

struct This::Pimpl
{
   QWidget* main_window{nullptr};
   This* parent{nullptr};

   QWidget* cpanel{nullptr};
   ImageViewer2* image_viewer = nullptr;
   ImageViewer2* frame_viewer = nullptr;
   QCheckBox* cb_view_prev    = nullptr;
   QCheckBox* cb_view_next    = nullptr;
   QPushButton* btn_check     = nullptr;
   QPushButton* btn_save      = nullptr;
   QPushButton* btn_publish   = nullptr;

   QPushButton* btn_rot_hist_left  = nullptr;
   QPushButton* btn_rot_hist_right = nullptr;
   QPushButton* btn_zoom_hist_in   = nullptr;
   QPushButton* btn_zoom_hist_out  = nullptr;
   QWidget* cam_label_widget       = nullptr;
   vector<QLabel*> cam_labels      = {};

   QPushButton* btn_arrow_left  = nullptr;
   QPushButton* btn_arrow_right = nullptr;
   QPushButton* btn_arrow_up    = nullptr;
   QPushButton* btn_arrow_down  = nullptr;
   QPushButton* btn_trail       = nullptr;

   shared_ptr<QImage> qim_ptr0 = nullptr;
   shared_ptr<QImage> qim_ptr1 = nullptr;

   shared_ptr<const pipeline::movie::Result> movie_result_ptr = nullptr;

   struct CachedResult
   {
      int64_t sequence_no = -1;
      shared_ptr<const pipeline::localization::Result> loc_ret_ptr;

      int frame_no() const noexcept
      {
         return loc_ret_ptr == nullptr ? -1 : loc_ret_ptr->frame_no();
      }

      auto image_ret() const noexcept
      {
         return loc_ret_ptr == nullptr
                    ? nullptr
                    : loc_ret_ptr->pose_skeleton_result->copy_images_result;
      }
   };
   SpinLock cache_padlock;
   size_t n_cached            = 0;
   size_t max_frames_to_cache = 1000;
   int64_t next_cache_sequence_no{0};
   vector<CachedResult> cached_results;

   std::atomic<bool> thread_is_done{false};

   Point2 hist_wh = Point2{0, 0};

   AnnotationData annotation_data;
   string annotations_directory                = ""s;
   bool perform_annotation_data_backup_on_save = false;
   bool perform_ground_truth_backup_on_save    = true;
   bool loaded_annotation_data = false; // as opposed to loaded ground-truth

   int trail_op_frame0          = -1;
   PoseAnnotation trail_op_pose = PoseAnnotation::NONE;
   int trail_op_track_id        = -1;

   perceive::movie::MovieUtils mutils;
   int image_h                      = 0; // set
   int image_w                      = 0;
   static constexpr int max_zoom    = 40;
   static constexpr int zoom_factor = 20;
   int zoom_clicks = 0; // Every button click increments/decrements this value

   int hist_rotation = 0;

   const LabeledTrackPoint* selected_trackpoint() const noexcept
   {
      return annotation_data.selected_trackpoint();
   }

   const vector<LabeledTrackPoint>& track_points() const noexcept
   {
      return annotation_data.state().ltps;
   }

   // ---- Mouse Parameters ---- //
   bool mouse_is_panning    = false;
   Point2 mouse_down_pos    = Point2{0, 0};
   Point2 pan_trackpoint_xy = Point2{0, 0};

   // --------------------------------------------------------------- contructor
   //
   Pimpl(QWidget* in_main_window, This* in_parent)
       : main_window(in_main_window)
       , parent(in_parent)
   {
      qim_ptr0 = make_shared<QImage>();
      qim_ptr1 = make_shared<QImage>();
   }

   ~Pimpl() { thread_is_done.store(true, std::memory_order_relaxed); }

   void make_ui() noexcept;
   void finish_ui() noexcept;

   void set_camera_labels_to_scene() noexcept
   {
      const auto scene_desc = app_state()->scene_desc();
      const int n_cameras   = scene_desc->n_cameras();
      Expects(n_cameras >= 0);
      const int max_cols = 5;

      // Create the widgets
      QGridLayout* layout
          = dynamic_cast<QGridLayout*>(cam_label_widget->layout());
      while(cam_labels.size() < size_t(n_cameras)) {
         const int row = int(cam_labels.size() / max_cols);
         const int col = int(cam_labels.size() % max_cols);
         cam_labels.push_back(new QLabel{});
         cam_labels.back()->installEventFilter(main_window);
         layout->addWidget(cam_labels.back(), row, col);
      }

      for(int i = 0; i < n_cameras; ++i)
         cam_labels[i]->setText(
             to_qstr(scene_desc->bcam_infos.at(i).camera_id));
   }

   void set_camera_labels_highlights()
   {
      const auto scene_desc = app_state()->scene_desc();
      const int n_cameras   = scene_desc->n_cameras();

      if(cam_labels.size() != size_t(n_cameras)) set_camera_labels_to_scene();

      for(auto label_ptr : cam_labels)
         label_ptr->setStyleSheet(
             "QLabel { background-color: none; color: black; margin: 4px; }");

      if(app_state()->has_current_camera()) {
         const auto ind = app_state()->current_camera();
         Expects(ind < cam_labels.size());
         cam_labels[ind]->setStyleSheet(
             "QLabel { background-color: red; color: white; margin: 4px; }");
      }
   }

   // -------------------------------------------------- set localization result
   //
   void set_mutils_from_loc_data(const LocalizationData& ldat) noexcept
   {
      mutils.hist_w      = ldat.loc_hist.width;
      mutils.hist_h      = ldat.loc_hist.height;
      mutils.hist_bounds = ldat.bounds;
      mutils.hist_sz     = ldat.hist_sz;
      mutils.do_rotate   = false;
   }

   // ---------------------------------------------------------------- mtask_ptr

   auto mtask_cptr() noexcept
   {
      Expects(app_state());
      return !app_state()->movie_results()
                 ? nullptr
                 : &(app_state()->movie_results()->movie_task);
   }

   auto mtask_ptr() noexcept
   {
      return const_cast<pipeline::movie::Task*>(mtask_cptr());
   }

   void set_mtask_pointer(shared_ptr<const pipeline::movie::Result> x)
   {
      if(x.get() != movie_result_ptr.get()) { movie_result_ptr = x; }
   }

   // --------------------------------------------------------------- poke tasks
   // Get tasks to recalculate
   auto poke_tasks(int in_frame_no                           = -1,
                   int in_sensor_no                          = -1,
                   std::function<void()> completion_function = nullptr)
   {
      Expects(app_state());

      const auto now = tick();

      shared_ptr<const LABImage> slab         = nullptr;
      shared_ptr<const LocalizationData> ldat = nullptr;

      auto mtask = mtask_ptr();
      if(mtask == nullptr) return std::make_pair(slab, ldat);

      // Figure out which frame-no we're looking at
      const int n_frames = app_state()->n_frames();
      const int frame_no = (in_frame_no >= 0 && in_frame_no < n_frames)
                               ? in_frame_no
                               : app_state()->frame_no();
      Expects(n_frames >= 0);
      if(frame_no < 0 || frame_no >= n_frames)
         return std::make_pair(slab, ldat);

      // Figure out which sensor we're looking at
      const int n_sensors = app_state()->n_sensors();
      const int sensor_no = (in_sensor_no >= 0 && in_sensor_no < n_sensors)
                                ? in_sensor_no
                                : app_state()->current_sensor_index();
      Expects(n_sensors >= 0);
      if(sensor_no < 0 || sensor_no >= n_sensors)
         return std::make_pair(slab, ldat);

      // We should only need to get this once...
      auto mdat = mtask->try_get_casted_result();
      if(mdat == nullptr) {
         schedule([mtask, frame_no, sensor_no, this]() {
            mtask->result([this, frame_no, sensor_no](auto x) {
               // Just hold onto the shared-ptr instance
               movie_result_ptr = x;
               if(x) poke_tasks(frame_no, sensor_no);
            });
         });
         return std::make_pair(slab, ldat); // and we're outta here
      }

      // Okay, let's see if we can try-get our results...
      slab = mdat->try_get_sensor_lab_shared_ptr(frame_no, sensor_no);
      ldat = mdat->try_get_localization_result(frame_no);

      if(ldat != nullptr && app_state()->frame_no() == frame_no) {
         // Paranoid, but make sure this is correct
         set_mutils_from_loc_data(*ldat);
      }

      if(slab == nullptr || ldat == nullptr) {
         // Okay, we got to schedule a task
         schedule([frame_no, sensor_no, this, mdat, completion_function]() {
            auto ldat = mdat->localization_result(frame_no);
            // INFO(format("cached frame {}", frame_no));
            auto slab
                = mdat->try_get_sensor_lab_shared_ptr(frame_no, sensor_no);
            if(ldat != nullptr && slab != nullptr) {
               if(app_state()->frame_no() == ldat->frame_no) {
                  set_mutils_from_loc_data(*ldat);
                  emit parent->data_updated();
               }
               if(completion_function) { completion_function(); }
            }
         });
      }

      if(ldat != nullptr && completion_function) { completion_function(); }

      return std::make_pair(slab, ldat);
   }

   void async_fill_movie_task(const int frame_no)
   {
      return; // Disabled
      using namespace std::chrono_literals;
      const int n_frames = app_state()->n_frames();
      if(frame_no < 0 || frame_no >= n_frames) return;
      if(thread_is_done.load(std::memory_order_relaxed)) return;

      auto get_ptr = [&]() {
         auto ptr = movie_result_ptr;
         while(!ptr) {
            try {
               std::this_thread::sleep_for(100ms);
            } catch(std::exception& e) {
               //
            }
            ptr = movie_result_ptr;
         }
         return ptr;
      };

      auto ptr = get_ptr();
      if(ptr->n_cached_frames() >= ptr->max_frames_to_cache()) return;
      if(thread_is_done.load(std::memory_order_relaxed)) return;

      schedule([frame_no, this]() {
         poke_tasks(frame_no + 1, -1, [frame_no, this]() {
            async_fill_movie_task(frame_no + 1);
         });
      });
   }

   // ---------------------------------------------------- widgets to/from model
   //
   bool widgets_to_model()
   {
      AppState& app = *app_state();
      bool ret      = false;
      {
#define SET_IT(x) p.tracklet_params.x = slider_##x->value()
#undef SET_IT
      }
      if(ret) { emit parent->on_redraw(); }
      return ret;
   }

   void model_to_widgets()
   {
      const AppState& app = *app_state();

      {
#define GET_IT(x) block_signal_set_slider(slider_##x, p.tracklet_params.x)
#undef GET_IT
      }
   }

   // ---------------------------------------------------- update selected
   //
   bool update_selected(int new_idx)
   {
      return annotation_data.select_trackpoint(new_idx);
   }

   bool update_selected_trackpoint_xy(Point2 new_xy)
   {
      return annotation_data.update_selected_trackpoint_xy(new_xy);
   }

   bool update_track_theta(Vector2 hist_xy)
   {
      const auto dxy = hist_xy - to_vec2(selected_trackpoint()->tp.xy());
      return update_track_theta(atan2(dxy.y, dxy.x));
   }

   bool update_track_theta(real theta)
   {
      return annotation_data.update_selected_trackpoint_theta(theta);
   }

   bool remove_track_theta()
   {
      return annotation_data.update_selected_trackpoint_theta(NAN);
   }

   bool update_track_id(int track_id)
   {
      return annotation_data.update_selected_track_id(track_id);
   }

   bool set_selected_pose(const PoseAnnotation pose)
   {
      const auto ptr = selected_trackpoint();
      if(ptr != nullptr) {
         const auto new_pose
             = (pose == ptr->tp.pose ? PoseAnnotation::NONE : pose);
         return annotation_data.update_selected_pose(new_pose);
      }
      return false;
   }

   bool delete_selected_track()
   {
      return annotation_data.delete_selected_track();
   }

   bool create_trackpoint(Vector2 hist_pos)
   {
      const auto frame_no = app_state()->frame_no();
      return annotation_data.create_and_select_trackpoint(to_pt2(hist_pos),
                                                          frame_no);
   }

   bool create_and_select_ltp(const LabeledTrackPoint& ltp)
   {
      return annotation_data.create_and_select_ltp(ltp);
   }

   bool move_all_trackpoints(const Point2 dxy) noexcept
   {
      return annotation_data.update_pos_all_tps(dxy);
   }

   bool apply_trail_operation() noexcept
   {
      {
         const auto selected = annotation_data.selected_trackpoint();
         if(trail_op_frame0 == -1) {
            if(selected == nullptr) {
               WARN(format("'trail-op' frame0 not set, because there's no "
                           "selected track-point"));
            } else if(selected->track_id < 0) {
               WARN(format("'trail-op' frame0 not set, because there's no "
                           "track-id for the selected track"));
            } else if(selected->tp.pose == PoseAnnotation::NONE) {
               WARN(format("'trail-op' frame0 not set, because there's no "
                           "pose for the selected track."));
            } else {
               trail_op_frame0   = app_state()->frame_no();
               trail_op_track_id = selected->track_id;
               trail_op_pose     = selected->tp.pose;
               INFO(
                   format("Set 'trail-op': to [frame={}, track-id={}, pose={}]",
                          trail_op_frame0,
                          trail_op_track_id,
                          str(trail_op_pose)));
            }
            return false;
         }
      }

      int frame0 = trail_op_frame0;
      int frame1 = app_state()->frame_no();
      if(frame0 > frame1) std::swap(frame0, frame1);
      const auto target_N = frame1 - frame0 + 1;
      const auto track_id = trail_op_track_id;
      const auto pose     = trail_op_pose;

      {
         INFO(format("'trail-op' read as [frame={}, track-id={}, pose={}]",
                     trail_op_frame0,
                     trail_op_track_id,
                     str(trail_op_pose)));
      }

      if(frame0 == frame1) {
         WARN(format("'trail-operation' works between a selected track in some "
                     "_other_ frame, and the current frame."));
         return false;
      }

      { // unset
         trail_op_frame0 = -1;
         INFO(format("'trail-op' read frame0 as {}, and RESET", frame0));
      }

      // This is the track-id we're interested in
      if(track_id < 0) {
         WARN(format("invalid track-id for selected track-point"));
         return false;
      }

      // Pose of interest
      if(pose == PoseAnnotation::NONE) {
         WARN(format("pose not set for selected track-point (frame {})",
                     frame0));
         return false;
      }

      // These are the track-ids of interest
      auto calc_indices = [&]() {
         const auto& ltps = annotation_data.state().ltps;
         vector<size_t> indices;
         indices.reserve(target_N);
         for(size_t i = 0; i < ltps.size(); ++i) {
            const auto& ltp = ltps[i];
            if(ltp.track_id == track_id && ltp.tp.t >= frame0
               && ltp.tp.t <= frame1)
               indices.push_back(i);
         }
         std::sort(
             begin(indices), end(indices), [&](const auto& A, const auto& B) {
                return ltps.at(A).tp.t < ltps.at(B).tp.t;
             });
         return indices;
      };
      const auto indices = calc_indices();

      if(indices.size() != size_t(target_N)) {
         LOG_ERR(
             format("trail operation failed: could not find a continuous "
                    "sequence of {} track-points (inclusive) for track-id {}",
                    target_N,
                    track_id));
         return false;
      }

      // Build the composite command here
      vector<AnnotationData::Command> cmds;
      {
         const auto& ltps = annotation_data.state().ltps;
         cmds.reserve(indices.size() * 2);
         for(const auto ind : indices) {
            if(ltps.at(ind).tp.pose != pose) {
               cmds.push_back(annotation_data.make_select_trackpoint(int(ind)));
               cmds.push_back(annotation_data.make_update_selected_pose(pose));
            }
         }
      }

      const auto ret = annotation_data.apply_commands(cmds);

      if(ret == true) {
         INFO(format("success: set track-id {} frames [{}..{}] to pose {}",
                     track_id,
                     frame0,
                     frame1,
                     str(pose)));
      } else {
         INFO(format("failed for some reason"));
      }

      return ret;
   }

   void undo()
   {
      annotation_data.undo();
      emit parent->data_updated();
   }

   void redo()
   {
      annotation_data.redo();
      emit parent->data_updated();
   }

   void reset_edit_data()
   {
      FATAL("kBAM!"); // do I create a special "undo"???
   }

   // ------------------------------------------------ move next-prev trackpoint
   //
   enum class MoveDirection : int { NONE, NEXT, PREV, BEGIN, END };
   void move_to_trackpoint(const MoveDirection md)
   {
      const auto& ltps    = annotation_data.state().ltps;
      const int old_index = annotation_data.state().selected_index;
      const int frame_no  = app_state()->frame_no();

      int first_id = -1;
      int next_id  = -1;
      int prev_id  = -1;
      int last_id  = -1;

      for(int i = 0; i < int(ltps.size()); ++i) {
         if(ltps[i].tp.t != frame_no) continue;
         if(first_id == -1) first_id = i;
         if(i < old_index) prev_id = i;
         if(i > old_index && next_id == -1) next_id = i;
         last_id = i;
      }

      if(prev_id == -1 && old_index == -1) prev_id = last_id;  // wraps around
      if(next_id == -1 && old_index == -1) next_id = first_id; // for clarity

      switch(md) {
      case MoveDirection::NONE: update_selected(-1); break;
      case MoveDirection::PREV: update_selected(prev_id); break;
      case MoveDirection::NEXT: update_selected(next_id); break;
      case MoveDirection::BEGIN: update_selected(first_id); break;
      case MoveDirection::END: update_selected(last_id); break;
      }

      if(old_index != annotation_data.state().selected_index)
         emit parent->data_updated();
   }

   // -------------------------------------------------------- process arrow key
   //
   bool process_arrow_key(const Point2 dxy)
   {
      if(selected_trackpoint()) {
         Point2 rot_xy = dxy;
         switch(hist_rotation) {
         case 0: break; // nothing
         case 1: rot_xy = Point2(dxy.y, -dxy.x); break;
         case 2: rot_xy = -dxy; break;
         case 3: rot_xy = Point2(-dxy.y, dxy.x); break;
         default: FATAL("logic error");
         }

         Point2 new_xy = selected_trackpoint()->tp.rounded_xy() + rot_xy;
         if(update_selected_trackpoint_xy(new_xy)) emit parent->data_updated();
      }
      return true;
   }

   // ---------------------------------------------------------- histogram-point
   //
   real metres_to_image_pixels(const real m) const
   {
      real n_cells  = m / mutils.hist_sz;
      real n_pixels = n_cells * real(image_w) / real(mutils.hist_w);
      return n_pixels;
   }

   real image_pixels_to_hist_cells(const real n_pixels) const
   {
      return n_pixels * real(mutils.hist_w) / real(image_w);
   }

   Vector2 image_point_to_hist_point(const Vector2& image_point) const
   {
      if(image_w < 1 or image_h < 1) return Vector2{0.0, 0.0};

      const real w  = (hist_rotation % 2 == 0) ? image_w : image_h;
      const real h  = (hist_rotation % 2 == 0) ? image_h : image_w;
      const real mw = (hist_rotation % 2 == 0) ? mutils.hist_w : mutils.hist_h;
      const real mh = (hist_rotation % 2 == 0) ? mutils.hist_h : mutils.hist_w;

      const Vector2 pt = apply_rotation(hist_rotation, image_point, w, h);
      const auto ret   = Vector2(mw * pt.x / w, mh * pt.y / h);

      if(false) {
         TRACE(format("[{}x{}] :: {} -> {} -> {}",
                      w,
                      h,
                      str(image_point),
                      str(pt),
                      str(ret)));
         this->image_point(to_pt2(ret));
         cout << endl << endl;
      }

      return ret;
   }

   Vector2 mouse_point_to_histogram_point(const Point2& mouse_point) const
   {
      const Vector2 X = image_viewer->mouse_to_qim(mouse_point);
      return image_point_to_hist_point(X);
   }

   Point2 image_point(const Point2 hist_point, bool do_rot = true) const
   {
      if(image_w < 1 or image_h < 1) return Point2{0, 0};

      const real w  = (hist_rotation % 2 == 0) ? image_w : image_h;
      const real h  = (hist_rotation % 2 == 0) ? image_h : image_w;
      const real mw = (hist_rotation % 2 == 0) ? mutils.hist_w : mutils.hist_h;
      const real mh = (hist_rotation % 2 == 0) ? mutils.hist_h : mutils.hist_w;

      const Vector2 H
          = Vector2(hist_point.x * real(image_w) / real(mutils.hist_w),
                    hist_point.y * real(image_h) / real(mutils.hist_h));

      Vector2 X = (do_rot == false)
                      ? H
                      : apply_rotation(
                          inverse_rotation(hist_rotation), H, image_w, image_h);

      if(false) {
         TRACE(format("[{}x{}] -> [{}x{}] :: {} -> {} -> {}",
                      mw,
                      mh,
                      w,
                      h,
                      str(hist_point),
                      str(H),
                      str(X)));
      }

      return to_pt2(X);
   }

   Point2 mouse_point(const Point2 hist_point) const
   {
      return to_pt2(
          image_viewer->qim_to_mouse(to_vec2(image_point(hist_point))));
   }

   int track_point_under_hist_pos(const Point2 hist_pos)
   {
      const auto frame_no = app_state()->frame_no();
      int best_fit        = -1;
      int best_dist       = std::numeric_limits<int>::max();
      auto& tps           = track_points();

      for(auto ii = begin(tps); ii != end(tps); ++ii) {
         auto& ltp = *ii;
         if(ltp.tp.t == frame_no) {
            const auto fit_dist = (ltp.tp.rounded_xy() - hist_pos).quadrance();
            if(fit_dist < best_dist) {
               best_dist = fit_dist;
               best_fit  = int(std::distance(begin(tps), ii));
            }
         }
      }

      const real dist = sqrt(real(best_dist)) * mutils.hist_sz;

      return (dist <= 1.1 * mutils.p3d_radius) ? best_fit : -1;
   }

   // ----------------------------------------------------------- on-mouse-press
   //
   bool on_mouse_press(QMouseEvent* event)
   {
      const Point2 mouse_pos(event->x(), event->y());
      const bool is_left_btn  = event->buttons() & Qt::LeftButton;
      const bool is_right_btn = event->buttons() & Qt::RightButton;
      const bool shift_down   = event->modifiers() & Qt::ShiftModifier;
      const bool ctrl_down    = event->modifiers() & Qt::ControlModifier;
      const bool alt_down     = event->modifiers() & Qt::AltModifier;
      const bool meta_down    = event->modifiers() & Qt::MetaModifier;

      parent->setFocus();
      const Vector2 hist_pos = mouse_point_to_histogram_point(mouse_pos);

      if(is_left_btn and ctrl_down) { // Create and select a new trackpoint
         if(create_trackpoint(hist_pos)) emit parent->data_updated();

      } else if(is_left_btn) { // Click, select, and drag
         int selected_index = track_point_under_hist_pos(to_pt2(hist_pos));
         const bool do_emit = update_selected(selected_index);

         if(selected_trackpoint()) { start_pan(mouse_pos); }
         if(do_emit) emit parent->data_updated();

      } else if(is_right_btn and ctrl_down) { // Select, rotate view
         int selected_index = track_point_under_hist_pos(to_pt2(hist_pos));
         if(selected_trackpoint() == nullptr) update_selected(selected_index);

         if(selected_trackpoint()) {
            start_pan(mouse_pos);
            update_track_theta(hist_pos);
         }
         emit parent->data_updated();

      } else { // Deselect
         if(selected_trackpoint()) {
            update_selected(-1);
            emit parent->data_updated();
         }
      }

      return false;
   }

   // ------------------------------------------------------------ on-mouse-move
   //
   bool on_mouse_move(QMouseEvent* event)
   {
      if(!mouse_is_panning) return false;

      const bool is_left_btn  = event->buttons() & Qt::LeftButton;
      const bool is_right_btn = event->buttons() & Qt::RightButton;
      const bool shift_down   = event->modifiers() & Qt::ShiftModifier;
      const bool ctrl_down    = event->modifiers() & Qt::ControlModifier;

      if(is_left_btn && shift_down && selected_trackpoint() != nullptr) {
         Point2 mouse_pos = Point2(event->x(), event->y());
         Point2 new_xy    = to_pt2(mouse_point_to_histogram_point(mouse_pos));
         update_selected_trackpoint_xy(new_xy);
         emit parent->data_updated();
         return true;

      } else if(is_right_btn && ctrl_down && selected_trackpoint() != nullptr) {
         Point2 mouse_pos = Point2(event->x(), event->y());
         Vector2 hist_xy  = mouse_point_to_histogram_point(mouse_pos);
         update_track_theta(hist_xy);
         emit parent->data_updated();
         return true;
      }

      stop_pan();

      return false;
   }

   // --------------------------------------------------------- on-mouse-release
   //
   bool on_mouse_release(QMouseEvent* event)
   {
      stop_pan();
      return false;
   }

   // ------------------------------------------------------------------ panning
   //
   void start_pan(Point2 pos)
   {
      Expects(selected_trackpoint());
      mouse_down_pos    = pos;
      mouse_is_panning  = true;
      pan_trackpoint_xy = selected_trackpoint()->tp.rounded_xy();
   }

   void stop_pan() { mouse_is_panning = false; }

   // ------------------------------------------------------------------ zooming
   //
   bool update_zoom(int delta) noexcept
   {
      const bool old_zoom_clicks = zoom_clicks;
      zoom_clicks += delta;
      if(std::fabs(zoom_clicks) > zoom_factor)
         zoom_clicks = (zoom_clicks < 0 ? -1 : 1) * zoom_factor;
      annotation_data.set_hist_zoom_level(zoom_clicks);
      return zoom_clicks != old_zoom_clicks;
   }

   std::pair<int, int> get_hist_size() noexcept
   {
      const real w0 = mutils.hist_w * 6;
      const real h0 = mutils.hist_h * 6;

      const real abs_zoom
          = real(zoom_factor + std::fabs(zoom_clicks)) / zoom_factor;
      const real zoom = (zoom_clicks < 0) ? (1.0 / abs_zoom) : abs_zoom;

      return movie::fitting_rect(
          w0 * zoom, h0 * zoom, mutils.hist_bounds.aspect_ratio());
   }

   // ----------------------------------------------------------------- rotating
   //
   bool update_rotate(int delta) noexcept
   {
      int old_hist_rotation = hist_rotation;
      hist_rotation         = (4 + hist_rotation + delta) % 4;
      annotation_data.set_hist_rotation(hist_rotation);
      return old_hist_rotation != hist_rotation;
   }

   // ------------------------------------------------------------- on-key-press
   //
   bool on_key_press(QKeyEvent* ev)
   {
      const int frame_no = app_state()->frame_no();

      const bool no_modifiers = ev->modifiers() == Qt::NoModifier;
      const bool alt_down     = ev->modifiers() == Qt::AltModifier;
      const bool ctrl_down    = (ev->modifiers() == Qt::ControlModifier);
      const bool shift_down   = (ev->modifiers() == Qt::ShiftModifier);
      const bool ctrl_shift_down
          = ev->modifiers() == (Qt::ShiftModifier | Qt::ControlModifier);
      const int key = ev->key();

      // TRACE(format("key = 0x{:x}, ctrl={}, shift={}", key, ));
      if(key == Qt::Key_Left && ctrl_down) {
         parent->on_btn_hist_rot_left();
      } else if(key == Qt::Key_Right && ctrl_down) {
         parent->on_btn_hist_rot_right();
      } else if((key == Qt::Key_Plus) || key == Qt::Key_Equal) {
         parent->on_btn_hist_zoom_in();
      } else if(key == Qt::Key_Minus) {
         parent->on_btn_hist_zoom_out();
      } else if(key == Qt::Key_PageUp && no_modifiers) {
         move_to_trackpoint(MoveDirection::NEXT);
      } else if(key == Qt::Key_PageDown && no_modifiers) {
         move_to_trackpoint(MoveDirection::PREV);
      } else if(key == Qt::Key_Home && no_modifiers) {
         move_to_trackpoint(MoveDirection::BEGIN);
      } else if(key == Qt::Key_End && no_modifiers) {
         move_to_trackpoint(MoveDirection::END);
      } else if(key == Qt::Key_Left && no_modifiers) {
         process_arrow_key({-1, 0});
      } else if(key == Qt::Key_Right && no_modifiers) {
         process_arrow_key({1, 0});
      } else if(key == Qt::Key_Up && no_modifiers) {
         process_arrow_key({0, -1});
      } else if(key == Qt::Key_Down && no_modifiers) {
         process_arrow_key({0, 1});
      } else if(key == Qt::Key_BraceLeft && ctrl_shift_down) {
         app_state()->seek_frame(frame_no - 10);
      } else if(key == Qt::Key_BraceRight && ctrl_shift_down) {
         app_state()->seek_frame(frame_no + 10);
      } else if(key == Qt::Key_BracketLeft && ctrl_down) {
         app_state()->seek_frame(frame_no - 1);
      } else if(key == Qt::Key_BracketRight && ctrl_down) {
         app_state()->seek_frame(frame_no + 1);
      } else if((key == Qt::Key_N) && ctrl_down) {
         const auto selected = selected_trackpoint();
         if(selected && selected->tp.t + 1 == frame_no) {
            auto ltp = *selected;
            ltp.tp.t = frame_no;
            create_and_select_ltp(ltp);
            Expects(selected_trackpoint());
            emit parent->data_updated();
         }
      } else if((key == Qt::Key_V) && ctrl_down) {
         if(remove_track_theta()) emit parent->data_updated();
      } else if(key == Qt::Key_Delete && no_modifiers) {
         if(delete_selected_track()) emit parent->data_updated();
      } else if((key == Qt::Key_Z) && ctrl_down) {
         undo();
         emit parent->data_updated();
      } else if((key == Qt::Key_R) && ctrl_down) {
         redo();
         emit parent->data_updated();
      } else if((key == Qt::Key_I) && ctrl_down) {
         if(selected_trackpoint()) {
            bool ok              = false;
            int current_track_id = selected_trackpoint()->track_id;

            int track_id = QInputDialog::getInt(parent,
                                                tr("Input 'track id'"),
                                                tr("Track Id:"),
                                                current_track_id,
                                                -1,
                                                10000,
                                                1,
                                                &ok);

            if(ok) {
               if(update_track_id(track_id)) emit parent->data_updated();
            }
         }
      } else if(key == Qt::Key_J && ctrl_down) {
         bool ok          = false;
         int new_frame_no = QInputDialog::getInt(parent,
                                                 tr("Input Frame Number"),
                                                 tr("frame number:"),
                                                 app_state()->frame_no(),
                                                 0,
                                                 10000,
                                                 1,
                                                 &ok);

         if(ok) { app_state()->seek_frame(new_frame_no); }
      } else if((key == Qt::Key_S) && (ctrl_down || alt_down)) {
         parent->on_save_tracks_file();
      } else if((key == Qt::Key_C) && (alt_down == true)) {
         parent->on_check_tracks_file();
      } else if(key == Qt::Key_Period && no_modifiers == true) {
         if(set_selected_pose(PoseAnnotation::NONE))
            emit parent->data_updated();
      } else if(key == Qt::Key_N && no_modifiers == true) {
         if(set_selected_pose(PoseAnnotation::NONE))
            emit parent->data_updated();
      } else if(key == Qt::Key_S && no_modifiers == true) {
         if(set_selected_pose(PoseAnnotation::STAND))
            emit parent->data_updated();
      } else if(key == Qt::Key_W && no_modifiers == true) {
         if(set_selected_pose(PoseAnnotation::WALK))
            emit parent->data_updated();
      } else if(key == Qt::Key_I && no_modifiers == true) {
         if(set_selected_pose(PoseAnnotation::SIT)) emit parent->data_updated();
      } else if(key == Qt::Key_L && no_modifiers == true) {
         if(set_selected_pose(PoseAnnotation::LAY)) emit parent->data_updated();
      } else if(key == Qt::Key_P && no_modifiers == true) {
         if(set_selected_pose(PoseAnnotation::PHONE))
            emit parent->data_updated();
      } else if(key == Qt::Key_O && no_modifiers == true) {
         if(set_selected_pose(PoseAnnotation::OTHER))
            emit parent->data_updated();
      } else {
         return false; // we did NOT handle the keystroke
      }
      return true; // we did something
   }
};

// --------------------------------------------------------------------- make ui

void This::Pimpl::make_ui() noexcept
{
   auto scene_desc = app_state()->scene_desc();

   // ---- (*) ---- Create widgets
   { // cpanel widgets
      btn_check   = new QPushButton("&Check");
      btn_save    = new QPushButton("&Save");
      btn_publish = new QPushButton("&Publish");

      cb_view_prev = new QCheckBox{};
      cb_view_next = new QCheckBox{};

      btn_rot_hist_left
          = new QPushButton{QIcon{to_qstr(":/icons/rotate-left.png")}, ""};
      btn_rot_hist_right
          = new QPushButton{QIcon{to_qstr(":/icons/rotate-right.png")}, ""};
      btn_zoom_hist_in
          = new QPushButton{QIcon{to_qstr(":/icons/zoom-in.png")}, ""};
      btn_zoom_hist_out
          = new QPushButton{QIcon{to_qstr(":/icons/zoom-out.png")}, ""};

      btn_arrow_left
          = new QPushButton{QIcon{to_qstr(":/icons/arrow-left.png")}, ""};
      btn_arrow_right
          = new QPushButton{QIcon{to_qstr(":/icons/arrow-right.png")}, ""};
      btn_arrow_up
          = new QPushButton{QIcon{to_qstr(":/icons/arrow-up.png")}, ""};
      btn_arrow_down
          = new QPushButton{QIcon{to_qstr(":/icons/arrow-down.png")}, ""};
      btn_trail = new QPushButton{QIcon{to_qstr(":/icons/trail.png")}, ""};

      btn_check->installEventFilter(main_window);
      btn_save->installEventFilter(main_window);
      btn_publish->installEventFilter(main_window);
      cb_view_prev->installEventFilter(main_window);
      cb_view_next->installEventFilter(main_window);
      btn_rot_hist_left->installEventFilter(main_window);
      btn_rot_hist_right->installEventFilter(main_window);
      btn_zoom_hist_in->installEventFilter(main_window);
      btn_zoom_hist_out->installEventFilter(main_window);
      btn_arrow_up->installEventFilter(main_window);
      btn_arrow_down->installEventFilter(main_window);
      btn_arrow_left->installEventFilter(main_window);
      btn_arrow_right->installEventFilter(main_window);
      btn_trail->installEventFilter(main_window);
   }

   { // cpanel
      auto make_cbs = [&]() {
         auto layout = new QHBoxLayout{};
         layout->addWidget(new_label("View Previous:"));
         layout->addWidget(cb_view_prev);
         layout->addWidget(new_label("View Next:"));
         layout->addWidget(cb_view_next);
         auto wgt = new QWidget{};
         wgt->setLayout(layout);
         return wgt;
      };

      auto make_controls_wgt = [&]() {
         auto layout = new QFormLayout{};
         layout->addRow(make_cbs());
         layout->addRow(make_hline());
         layout->addRow(make_instructions_label());

         auto wgt = new QWidget{};
         wgt->setLayout(layout);
         return wgt;
      };

      auto make_arrows_wgt = [&]() {
         auto layout = new QHBoxLayout{};
         if(app_state()->dev_mode()) {
            layout->addWidget(btn_arrow_left);
            layout->addWidget(btn_arrow_right);
            layout->addWidget(btn_arrow_up);
            layout->addWidget(btn_arrow_down);
         }
         layout->addWidget(btn_trail);
         layout->addWidget(make_horizontal_expander());
         auto wgt = new QWidget{};
         wgt->setLayout(layout);
         return wgt;
      };

      QVBoxLayout* layout = new QVBoxLayout{};
      layout->addWidget(make_controls_wgt());
      layout->addWidget(make_arrows_wgt());
      layout->addWidget(btn_check);
      layout->addWidget(btn_save);
      layout->addWidget(btn_publish);

      cpanel = new QWidget{};
      cpanel->setFixedWidth(app_state()->config().cpanel_width);
      cpanel->setStyleSheet("background-color: white;");
      cpanel->setLayout(layout);
   }

   { // image viewer (for seeing the video)
      image_viewer = new ImageViewer2{};
      image_viewer->setSizePolicy(QSizePolicy::Expanding,
                                  QSizePolicy::Expanding);
      image_viewer->setMinimumSize(QSize(256, 256));
      image_viewer->setStyleSheet("background-color: blue;");
      image_viewer->set_pixel_indicator(false);
      image_viewer->is_pannable = false;
      image_viewer->is_zoomable = false;
      image_viewer->set_offset(Point2(0, 0));
      image_viewer->set_mouse_tracking(true);

      image_viewer->on_mouse_press = [this](QMouseEvent* event) -> bool {
         return on_mouse_press(event);
      };
      image_viewer->on_mouse_move
          = [this](QMouseEvent* event) -> bool { return on_mouse_move(event); };
      image_viewer->on_mouse_release = [this](QMouseEvent* event) -> bool {
         return on_mouse_release(event);
      };

      image_viewer->installEventFilter(main_window);
   }

   {
      frame_viewer = new ImageViewer2{};
      frame_viewer->setSizePolicy(QSizePolicy::Expanding,
                                  QSizePolicy::Expanding);
      frame_viewer->setMinimumSize(QSize(256, 256));
      frame_viewer->setStyleSheet("background-color: blue;");
      frame_viewer->set_pixel_indicator(false);
      frame_viewer->set_offset(Point2(0, 0));

      frame_viewer->installEventFilter(main_window);
   }

   auto make_hist_viewer_controls = [&]() {
      auto layout = new QHBoxLayout{};
      layout->addWidget(btn_rot_hist_left);
      layout->addWidget(btn_rot_hist_right);
      layout->addWidget(btn_zoom_hist_in);
      layout->addWidget(btn_zoom_hist_out);
      auto wgt = new QWidget{};
      wgt->setLayout(layout);
      return wgt;
   };

   auto make_frame_viewer_labels = [&]() {
      auto layout = new QGridLayout{};
      cam_labels.push_back(new QLabel{to_qstr("")});
      layout->addWidget(cam_labels[0], 0, 0);
      cam_labels[0]->installEventFilter(main_window);

      cam_label_widget = new QWidget{};
      auto wgt         = cam_label_widget;
      wgt->setLayout(layout);
      return wgt;
   };

   auto make_viewer_layout = [&]() {
      auto layout = new QGridLayout{};
      layout->setSpacing(4);
      layout->addWidget(make_hist_viewer_controls(), 0, 0, Qt::AlignLeft);
      layout->addWidget(make_frame_viewer_labels(), 0, 1, Qt::AlignLeft);
      layout->addWidget(image_viewer, 1, 0);
      layout->addWidget(frame_viewer, 1, 1);
      auto wgt = new QWidget{};
      wgt->setLayout(layout);
      return wgt;
   };

   QHBoxLayout* layout = new QHBoxLayout{};
   layout->addWidget(cpanel);
   layout->addWidget(make_viewer_layout());
   parent->setLayout(layout);

   // ---- (*) ---- Initialize
   model_to_widgets();

   // ---- (*) ---- Wiring
   connect(app_state(),
           SIGNAL(open_finished()),
           parent,
           SLOT(model_to_widgets()),
           Qt::QueuedConnection);

   connect(app_state(),
           SIGNAL(open_finished()),
           parent,
           SLOT(init_tracker_file()),
           Qt::QueuedConnection);

   connect(app_state(),
           SIGNAL(open_finished()),
           parent,
           SLOT(finish_ui()),
           Qt::QueuedConnection);

   connect(app_state(),
           SIGNAL(video_frame_changed(int)),
           parent,
           SLOT(on_video_frame_changed(int)),
           Qt::QueuedConnection);

   connect(app_state(),
           SIGNAL(camera_changed(unsigned)),
           parent,
           SLOT(on_camera_changed()),
           Qt::QueuedConnection);

   connect(btn_rot_hist_left,
           SIGNAL(pressed()),
           parent,
           SLOT(on_btn_hist_rot_left()));

   connect(btn_rot_hist_right,
           SIGNAL(pressed()),
           parent,
           SLOT(on_btn_hist_rot_right()));

   connect(btn_zoom_hist_in,
           SIGNAL(pressed()),
           parent,
           SLOT(on_btn_hist_zoom_in()));

   connect(btn_zoom_hist_out,
           SIGNAL(pressed()),
           parent,
           SLOT(on_btn_hist_zoom_out()));

   connect(btn_arrow_up, SIGNAL(pressed()), parent, SLOT(on_btn_arrow_up()));
   connect(
       btn_arrow_down, SIGNAL(pressed()), parent, SLOT(on_btn_arrow_down()));
   connect(
       btn_arrow_left, SIGNAL(pressed()), parent, SLOT(on_btn_arrow_left()));
   connect(
       btn_arrow_right, SIGNAL(pressed()), parent, SLOT(on_btn_arrow_right()));
   connect(btn_trail, SIGNAL(pressed()), parent, SLOT(on_btn_trail()));

   connect(parent,
           SIGNAL(data_updated()),
           parent,
           SLOT(on_redraw()),
           Qt::QueuedConnection);

   connect(btn_check, SIGNAL(pressed()), parent, SLOT(on_check_tracks_file()));
   connect(btn_save, SIGNAL(pressed()), parent, SLOT(on_save_tracks_file()));
   connect(
       btn_publish, SIGNAL(pressed()), parent, SLOT(on_publish_ground_truth()));

#define CONNECT_SLIDER(source, slot)                                        \
   {                                                                        \
      connect(source, SIGNAL(value_changed(double)), parent, SLOT(slot())); \
   }
#define CONNECT_RB(source, slot)                                    \
   {                                                                \
      connect(source, SIGNAL(toggled(bool)), parent, SLOT(slot())); \
   }
#define CONNECT_CB(source, slot)                                    \
   {                                                                \
      connect(source, SIGNAL(toggled(bool)), parent, SLOT(slot())); \
   }

   CONNECT_CB(cb_view_prev, on_redraw);
   CONNECT_CB(cb_view_next, on_redraw);

#undef CONNECT_SLIDER
#undef CONNECT_RB
#undef CONNECT_CB
}

void This::Pimpl::finish_ui() noexcept
{
   set_camera_labels_to_scene();
   set_camera_labels_highlights();

   schedule([this]() {
      Expects(app_state());
      int frame_no = 0;
      if(app_state()->has_current_frame()) frame_no = app_state()->frame_no();
      async_fill_movie_task(frame_no);
   });
}

// ---------------------------------------------------------------- Construction

This::This(QWidget* parent)
    : QWidget(parent)
    , pimpl_(new Pimpl(parent, this))
{
   pimpl_->make_ui();
}

This::~This() { delete pimpl_; }

// ---------------------------------------------------------- handle_key_pressed
//
bool This::handle_key_pressed(QKeyEvent* ev)
{
   return pimpl_->on_key_press(ev);
}

// ------------------------------------------------------- widgets to/from model
//
bool This::widgets_to_model() { return pimpl_->widgets_to_model(); }

void This::model_to_widgets() { pimpl_->model_to_widgets(); }

// ----------------------------------------------------------- init tracker-file
//
void This::init_tracker_file()
{
   auto& P = *pimpl_;
   INFO("init-tracker-file");

   auto app_ptr = app_state();
   Expects(app_ptr != nullptr);

   const auto& cli         = app_ptr->config().config;
   P.annotations_directory = cli.annotations_directory;
   if(P.annotations_directory == ""s) P.annotations_directory = "/tmp"s;
   const bool is_s3 = is_s3_uri(P.annotations_directory);

   /**
    * Loading Protocol
    *
    * + Look for `annotation-fname`, if we find it, we open it.
    *   We also spawn a thread that copies the annotation-fname to a backup
    *   file. If we fail to find it, then we open `ground-truth`.
    */
   auto attempt_load = [&](const string& fname, string& raw_data) -> bool {
      try {
         if(is_s3)
            s3_load(fname, raw_data);
         else
            file_get_contents(fname, raw_data);
         return true;
      } catch(std::exception& e) {
         WARN(format("failed to load '{}', falling back to ground-truth",
                     fname));
      }
      return false;
   };

   auto attempt_load_annotations_data = [&]() -> bool {
      const auto fname = get_annotation_fname(P.annotations_directory);
      string raw_data;
      if(!attempt_load(fname, raw_data)) return false;
      const bool ret = P.annotation_data.load_annotation_data(raw_data);
      if(ret) {
         INFO(format("loaded annotations from '{}'", fname));
      } else {
         WARN(format("failed to read annotation-data from file '{}'", fname));
      }
      return ret;
   };

   auto attempt_load_ground_truth = [&]() -> bool {
      const auto fname = get_ground_truth_fname(P.annotations_directory);
      string raw_data;
      if(!attempt_load(fname, raw_data)) return false;
      const bool ret = P.annotation_data.load_ground_truth(raw_data);
      if(ret) {
         INFO(format("loaded ground-truth from '{}'", fname));
      } else {
         WARN(format("failed to read ground-truth from file '{}'", fname));
      }
      return ret;
   };

   if(attempt_load_annotations_data()) {
      P.perform_annotation_data_backup_on_save = true;
      P.update_zoom(P.annotation_data.hist_zoom_level());
      P.update_rotate(P.annotation_data.hist_rotation());
   } else {
      attempt_load_ground_truth();
   }

   P.perform_ground_truth_backup_on_save = true;

   emit data_updated();
}

// -------------------------------------------------------- on-check tracks-file
//
void This::on_check_tracks_file()
{
   bool is_good = check_trackpoints_as_tracks(pimpl_->track_points());
   if(is_good) {
      INFO("checks done, annotation is GOOD!");
   } else {
      INFO("checks done, fix errors before publishing.");
   }
}

// --------------------------------------------------------- on-save tracks-file
//
void This::on_save_tracks_file()
{
   auto& P = *pimpl_;

   const bool is_s3 = is_s3_uri(P.annotations_directory);
   const auto fname = get_annotation_fname(P.annotations_directory);

   if(P.perform_annotation_data_backup_on_save) {
      // Copy annotation data to backup.
      const auto back_fname
          = get_annotation_backup_fname(P.annotations_directory);

      const auto now = tick();
      INFO(format("backing up:\n   {}\n   ->\n   {}\n\n", fname, back_fname));
      try {
         string raw;
         if(is_s3) {
            s3_load(fname, raw);
            s3_store(back_fname, raw);
         } else {
            file_get_contents(fname, raw);
            file_put_contents(back_fname, raw);
         }
         INFO(format("done: {}ms", ms_tock_s(now)));
      } catch(std::exception& e) {
         LOG_ERR(format(
             "failed to backup:\n   {}\n   ->\n   {}\n\n", fname, back_fname));
      }

      P.perform_annotation_data_backup_on_save = false;
   }

   try {
      if(is_s3) {
         s3_store(fname, P.annotation_data.export_annotation_data());
      } else {
         file_put_contents(fname, P.annotation_data.export_annotation_data());
      }
      INFO(format("annotations data saved to '{}'", fname));
   } catch(std::exception& e) {
      LOG_ERR(format(
          "failed to save annotation data to '{}': {}", fname, e.what()));
   }
}

// ----------------------------------------------------- on-publish ground-truth
//
void This::on_publish_ground_truth()
{
   auto& P = *pimpl_;

   if(!check_trackpoints_as_tracks(P.track_points())) {
      LOG_ERR(format(
          "Cowardly refusing to export ground-truth: please fix errors"));
      return;
   }

   shared_ptr<const LABImage> slab;
   shared_ptr<const LocalizationData> ldat;
   std::tie(slab, ldat) = P.poke_tasks();
   if(ldat == nullptr) {
      LOG_ERR(format("NOTHING published, because localization computation is "
                     "not complete. Just wait a bit, and try again."));
      return;
   }

   auto get_ground_truth_json = [&]() {
      const auto scene_desc = app_state()->scene_desc();

      PipelineOutput o;
      FowlkesResult fowlkes_ret;
      fowlkes_ret.frames         = scene_desc->n_frames();
      fowlkes_ret.frame_duration = scene_desc->frame_duration();
      fowlkes_ret.w              = ldat->loc_hist.width;
      fowlkes_ret.h              = ldat->loc_hist.height;
      fowlkes_ret.hist_sz        = ldat->hist_sz;
      fowlkes_ret.top_left       = ldat->bounds.top_left();
      fowlkes_ret.tracks         = trackpoints_to_tracks(P.track_points());
      o.config                   = app_state()->movie_params().config;
      o.params                   = app_state()->movie_params().params.to_json();
      o.track_results            = fowlkes_ret.to_json();
      Json::Value json;
      write(o, json);
      Json::StyledWriter writer;
      const string raw = writer.write(json);
      return raw;
   };

   get_ground_truth_json();

   const bool is_s3 = is_s3_uri(P.annotations_directory);
   const auto fname = get_ground_truth_fname(P.annotations_directory);

   if(P.perform_ground_truth_backup_on_save) {
      const auto back_fname
          = get_ground_truth_backup_fname(P.annotations_directory);

      const auto now = tick();
      INFO(format("backing up:\n   {}\n   ->\n   {}\n\n", fname, back_fname));
      try {
         string raw;
         if(is_s3) {
            s3_load(fname, raw);
            s3_store(back_fname, raw);
         } else {
            file_get_contents(fname, raw);
            file_put_contents(back_fname, raw);
         }
         INFO(format("done: {}ms", ms_tock_s(now)));
      } catch(std::exception& e) {
         LOG_ERR(format(
             "failed to backup:\n   {}\n   ->\n   {}\n\n", fname, back_fname));
      }

      P.perform_ground_truth_backup_on_save = false;
   }

   { // Here we actually create the envelope and save
      const string raw = get_ground_truth_json();

      try {
         if(!is_s3) {
            file_put_contents(fname, raw);
         } else {
            const auto& testcases_dir = multiview_testcase_cache_dir();
            const auto testcase_name  = basename(P.annotations_directory);
            const auto testcase_dir
                = format("{}/{}", testcases_dir, testcase_name);

            if(!is_directory(testcases_dir)) {
               throw std::runtime_error(
                   format("cannot file cached testcase directory '{}'",
                          testcases_dir));
            }

            const auto hash_fname = format("{}/hash.md5", testcase_dir);

            INFO(format("writing ground-truth into s3 cache directory"));
            const auto cache_fname
                = format("{}/{}", testcase_dir, basename(fname));
            file_put_contents(cache_fname, raw);
            INFO(format("wrote ground-truth to cache file '{}'", cache_fname));

            const string hash = md5(raw);
            file_put_contents(hash_fname, hash);
            INFO(format("wrote {} to cache file '{}'", hash, hash_fname));

            const string hash_s3_fname = format(
                "{}/{}/hash.md5", multiview_testcase_s3_dir(), testcase_name);

            s3_store(fname, raw); // save the ground truth
            s3_store(hash_s3_fname, hash);

            INFO(format("hash published to '{}'", hash_s3_fname));
         }

         INFO(format("ground-truth published to '{}'", fname));

      } catch(std::exception& e) {
         LOG_ERR(format(
             "failed to save ground-truth to '{}': {}", fname, e.what()));
      }
   }
}

// ------------------------------------------------------ on video frame changed

void This::on_video_frame_changed(int frame_no)
{
   auto& P = *pimpl_;
   if(!P.selected_trackpoint()) return;
   if(int(P.selected_trackpoint()->tp.t) == frame_no) return;

   // Find the track for the current frame number...
   const int track_id = P.selected_trackpoint()->track_id;

   auto ii = std::find_if(
       begin(P.track_points()), end(P.track_points()), [&](const auto& ltp) {
          return (ltp.track_id == track_id) and (ltp.tp.t == frame_no);
       });

   int idx = (ii == end(P.track_points()))
                 ? -1
                 : int(std::distance(begin(P.track_points()), ii));

   P.update_selected(idx);
}

// ------------------------------------------------------------------- finish-ui
// Called when scene is loaded
void This::finish_ui() { pimpl_->finish_ui(); }

void This::on_camera_changed()
{
   auto& P = *pimpl_;
   P.set_camera_labels_highlights();
   on_redraw(); // finish with a redraw
}

// ----------------------------------------------------------- histogram buttons

void This::on_btn_hist_rot_left()
{
   if(pimpl_->update_rotate(-1)) on_redraw();
}

void This::on_btn_hist_rot_right()
{
   if(pimpl_->update_rotate(1)) on_redraw();
}

void This::on_btn_hist_zoom_in()
{
   if(pimpl_->update_zoom(1)) on_redraw();
}

void This::on_btn_hist_zoom_out()
{
   if(pimpl_->update_zoom(-1)) on_redraw();
}

// --------------------------------------------------------------- arrow buttons

void This::on_btn_arrow_up()
{
   if(pimpl_->move_all_trackpoints(Point2{0, 1})) emit data_updated();
}

void This::on_btn_arrow_down()
{
   if(pimpl_->move_all_trackpoints(Point2{0, -1})) emit data_updated();
}

void This::on_btn_arrow_left()
{
   if(pimpl_->move_all_trackpoints(Point2{-1, 0})) emit data_updated();
}

void This::on_btn_arrow_right()
{
   if(pimpl_->move_all_trackpoints(Point2{1, 0})) emit data_updated();
}

void This::on_btn_trail()
{
   if(pimpl_->apply_trail_operation()) emit data_updated();
}

// ------------------------------------------------------------------ arrow keys

void This::keyPressEvent(QKeyEvent* ev) { pimpl_->on_key_press(ev); }

// ------------------------------------------------------------------- on-redraw

void This::on_redraw()
{
   auto& P = *pimpl_;

   auto set_qimage_to_null = [&]() {
      P.image_viewer->set_qimage(nullptr);
      P.frame_viewer->set_qimage(nullptr);
   };
   auto app_ptr = app_state();

   if(app_ptr == nullptr) return set_qimage_to_null();
   if(app_ptr->frame_results() == nullptr) return set_qimage_to_null();
   if(!app_ptr->has_current_sensor() || !app_ptr->has_current_frame())
      return set_qimage_to_null();

   const auto scene_desc    = app_ptr->scene_desc();
   const unsigned sensor_no = app_ptr->current_sensor_index();
   const auto frame_no      = app_ptr->frame_no();

   shared_ptr<const LABImage> slab;
   shared_ptr<const LocalizationData> ldat;
   std::tie(slab, ldat) = P.poke_tasks(frame_no, sensor_no);

   if(!slab || !ldat) {
      set_qimage_to_null();
      return;
   }

   // Get the latest results... (thread-safe)... will be nullptr if
   // we're currently calculating

   auto make_hist_im = [&](ImageViewer2* viewer) { // Draw the output
      std::tie(P.image_w, P.image_h) = P.get_hist_size();

      const int w = P.image_w;
      const int h = P.image_h;

      cv::Mat hist_im(h, w, CV_8UC3);

      cv::Mat lhist = argb_to_cv(grey16_im_to_argb(ldat->loc_hist));
      cv::resize(lhist, hist_im, cv::Size(w, h));

      P.mutils.render_grid(hist_im, P.mutils.hist_bounds);

      const bool is_prev = P.cb_view_prev->isChecked();
      const bool is_next = P.cb_view_next->isChecked();

      const int sel_id = (P.selected_trackpoint() == nullptr)
                             ? -1
                             : P.selected_trackpoint()->track_id;

      for(const auto& ltp : P.track_points()) {
         if(ltp.tp.t == frame_no) {
            P.mutils.render_tp(hist_im, ltp.track_id, ltp.tp);
            const bool is_highlighted = (&ltp == P.selected_trackpoint());
            if(is_highlighted) {
               outline_circle(
                   hist_im,
                   to_vec2(P.image_point(ltp.tp.rounded_xy(), false)),
                   k_green,
                   P.metres_to_image_pixels(P.mutils.p3d_radius));
            }
         } else if(((sel_id >= 0 && ltp.track_id == sel_id) || (sel_id == -1))
                   && (ltp.tp.t + 1 == frame_no) && is_prev) {
            outline_circle(hist_im,
                           to_vec2(P.image_point(ltp.tp.rounded_xy(), false)),
                           k_white,
                           P.metres_to_image_pixels(P.mutils.p3d_radius));
         } else if(((sel_id >= 0 && ltp.track_id == sel_id) || (sel_id == -1))
                   && (ltp.tp.t == frame_no + 1) && is_next) {
            outline_circle(hist_im,
                           to_vec2(P.image_point(ltp.tp.rounded_xy(), false)),
                           k_black,
                           P.metres_to_image_pixels(P.mutils.p3d_radius));
         }
      }

      // Now rotate... hist_im
      switch(P.hist_rotation) {
      case 0: break; // no rotation
      case 1: cv::rotate(hist_im, hist_im, cv::ROTATE_90_CLOCKWISE); break;
      case 2: cv::rotate(hist_im, hist_im, cv::ROTATE_180); break;
      case 3:
         cv::rotate(hist_im, hist_im, cv::ROTATE_90_COUNTERCLOCKWISE);
         break;
      }

      to_qimage(hist_im, *P.qim_ptr0);
      viewer->set_qimage(P.qim_ptr0);
      viewer->setFixedWidth(hist_im.cols);
   };

   auto make_frame_im = [&](ImageViewer2* viewer) {
      Expects(slab != nullptr);
      ARGBImage argb = LAB_im_to_argb(*slab);

      if(unsigned(sensor_no) < ldat->p2ds.size()) {
         movie::render_poses(argb, sensor_no, ldat->p2ds[sensor_no]);
         const DistortedCamera& dcam = scene_desc->dcam(sensor_no);
         for(const auto& ltp : P.track_points()) {
            if(ltp.tp.t == frame_no) {
               P.mutils.render_trackpoint(
                   argb, dcam, ltp.track_id, ltp.tp, true);
            }
         }
      }

      to_qimage(argb, *P.qim_ptr1);
      viewer->set_qimage(P.qim_ptr1);
   };

   make_hist_im(P.image_viewer);
   make_frame_im(P.frame_viewer);
}
