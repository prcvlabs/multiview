#include "stdinc.hpp"

#include <QDir>
#include <QInputDialog>
#include <QMessageBox>
#include <QMouseEvent>

#include "camera-positioner.hh"

#include "gui/app-state.hh"
#include "gui/interface/interface-helpers.hpp"
#include "gui/qt-helpers.hpp"
#include "gui/widgets/image-viewer-2.hh"
#include "gui/widgets/labeled-slider.hh"

#include "perceive/calibration/scene-calib-data.hpp"
#include "perceive/graphics/bresenham.hpp"
#include "perceive/graphics/tiny-string.hpp"
#include "perceive/io/json-io.hpp"
#include "perceive/io/lazy-s3.hpp"
#include "perceive/io/perceive-assets.hpp"
#include "perceive/utils/opencv-helpers.hpp"

#define This CameraPositioner

using namespace perceive;

static QLabel* make_instructions_label()
{
   QLabel* l = new QLabel{};
   l->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
   l->setFixedHeight(350);
   QFont font("Arial", 9);
   l->setFont(font);
   l->setWordWrap(true);
   l->setText(to_qstr(R"V0G0N(
<table>
<tr><th width=150>Key</th><th>Description</th></tr>

<tr><td>Ctrl+[1-9]:        </td>
    <td>Select camera view.</td></tr>

<tr><td>Right Click:  </td>
    <td>Select something.</td></tr>
<tr><td>Ctrl+Left Click:  </td>
    <td>Create a new Labelled Point.</td></tr>
<tr><td>Ctrl+Left Click (Drag):  </td>
    <td>Start creating a new Labelled Line.</td></tr>

<tr><td>PageUp/PageDown:  </td>
    <td>Select the next/previous line or point.</td></tr>

<tr><td>Ctrl+I             </td>
    <td>Edit the label-id.</td></tr>

<tr><td>F                  </td>
    <td>Set/remove floor status.</td></tr>
<tr><td>X                  </td>
    <td>Set/remove direction to X.</td></tr>
<tr><td>Y                  </td>
    <td>Set/remove direction to Y.</td></tr>
<tr><td>Z                  </td>
    <td>Set/remove direction to Z.</td></tr>
<tr><td>O                  </td>
    <td>Set/remove origin point status.</tr></tr>

<tr><td>Ctrl+Arrow Keys:  </td>
    <td>Move selected.</td></tr>

<tr><td>Ctrl+Del             </td>
    <td>Delete selected.</td></tr>

<tr><td>Alt+C:             </td>
    <td>Check output... results written to stdout</td></tr>
<tr><td>Alt+S, or Ctrl+S:   </td>
    <td>Save to output file.</td></tr>
</table>
)V0G0N"));
   return l;
}

// A timestamp string without colons
static string timestamp_fname_str(const Timestamp& ts)
{
   string out = ts.to_string();
   for(auto& c : out)
      if(c == ':') c = '-';
   return out;
}

static string remove_all_timestamps(const string_view in)
{
   string out = in.data();
   while(true) {
      const string extracted = extract_timestamp(out);
      if(extracted.empty()) break; // we're done
      const auto pos = out.find(extracted);
      Expects(pos != string::npos);
      Expects(extracted.size() > 0);
      out.erase(pos, extracted.size());
   }
   return out;
}

static string annotation_filename(const SceneDescription& scene_desc)
{
   return format("{}/cam-pos-annotations_{}.json",
                 multiview_camera_pos_annotation_dir(),
                 scene_desc.scene_info.scene_key);
}

static string annotation_backup_filename(const SceneDescription& scene_desc)
{
   return format("{}/zz-backup_cam-pos-annotations_{}_{}.json",
                 multiview_camera_pos_annotation_dir(),
                 scene_desc.scene_info.scene_key,
                 timestamp_fname_str(Timestamp::now()));
}

// ----------------------------------------------------------------------- pimpl

using Tracks = perceive::Tracks;

enum class CalibType : int { NONE, POINT, LINE_A, LINE_B };

struct This::Pimpl
{
   QWidget* main_window{nullptr};
   This* parent{nullptr};

   QWidget* cpanel{nullptr};
   ImageViewer2* image_viewer{nullptr};

   QRadioButton* rb_show_no_error       = nullptr;
   QRadioButton* rb_show_selected_error = nullptr;
   QRadioButton* rb_show_all_errors     = nullptr;
   QComboBox* combo_ref_cam             = nullptr;

   QPushButton* btn_optimize = nullptr;
   QPushButton* btn_check    = nullptr;
   QPushButton* btn_reset    = nullptr;
   QPushButton* btn_revert   = nullptr;
   QPushButton* btn_save     = nullptr;
   QPushButton* btn_publish  = nullptr;

   shared_ptr<QImage> qim_ptr{nullptr};

   shared_ptr<const pipeline::copy_sensor_images::Result> image_result_ptr;

   calibration::CalibrationInfo annotation_data;
   string annotations_directory                = ""s;
   bool perform_annotation_data_backup_on_save = false;
   bool loaded_annotation_data = false; // as opposed to loaded ground-truth

   // ---- What is Selected ---- //
   CalibType selected_type_  = CalibType::NONE;
   int selected_point_index_ = -1;
   int selected_line_index_  = -1;

   // ---- Mouse Parameters ---- //
   bool mouse_is_panning         = false;
   Vector2f mouse_down_image_pos = Vector2f{0.0f, 0.0f}; // image coordinates
   Vector2f mouse_move_image_pos = Vector2f{0.0f, 0.0f};
   Point2 pan_trackpoint_xy      = Point2{0, 0};

   // ---- Our optimized data ---- //
   vector<DistortedCamera> dcams; // one per camera, left sensor
   using opt_data_type                      = vector<EuclideanTransform>;
   shared_ptr<const opt_data_type> opt_data = nullptr;

   // --------------------------------------------------------------- contructor
   //
   Pimpl(QWidget* in_main_window, This* in_parent)
       : main_window(in_main_window)
       , parent(in_parent)
       , qim_ptr(make_shared<QImage>())
   {
      Expects(main_window != nullptr);
   }

   void make_ui() noexcept;

   // -------------------------------------------------- set localization result

   void set_image_result(
       shared_ptr<const pipeline::copy_sensor_images::Result> ptr) noexcept
   {
      if(image_result_ptr.get() != ptr.get()) {
         image_result_ptr = ptr;
         emit parent->data_updated();
      }
   }

   // ---------------------------------------------------------------- ltask_ptr

   auto ftask_cptr() noexcept
   {
      Expects(app_state());
      return !app_state()->frame_results()
                 ? nullptr
                 : &(app_state()->frame_results()->copy_sensor_images);
   }

   auto ftask_ptr() noexcept
   {
      return const_cast<pipeline::copy_sensor_images::Task*>(ftask_cptr());
   }

   // --------------------------------------------------------------- poke tasks
   // Get tasks to recalculate
   auto poke_tasks()
   {
      Expects(app_state());
      Expects(app_state()->frame_results());

      auto ftask = ftask_ptr();
      auto o     = this;

      {
         schedule([ftask, o]() {
            ftask->result([o](auto x) { o->set_image_result(x); });
         });
      }

      return ftask->try_get_casted_result();
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

   // ------------- find a point/line with a specific label in a specific camera
   //
   calibration::LabelledPoint* find_point(const int camera_no, const int label)
   {
      if(label == -1) return nullptr;
      auto ii
          = std::find_if(begin(annotation_data.points),
                         end(annotation_data.points),
                         [&](const auto& o) {
                            return o.label == label && o.camera_no == camera_no;
                         });
      return (ii == end(annotation_data.points)) ? nullptr
                                                 : std::addressof(*ii);
   }

   calibration::LabelledLine* find_line(const int camera_no, const int label)
   {
      if(label == -1) return nullptr;
      auto ii
          = std::find_if(begin(annotation_data.lines),
                         end(annotation_data.lines),
                         [&](const auto& o) {
                            return o.label == label && o.camera_no == camera_no;
                         });
      return (ii == end(annotation_data.lines)) ? nullptr : std::addressof(*ii);
   }

   // --------------------------------------------------------- current selected
   //
   int selected_index() const noexcept
   {
      if(selected_type_ == CalibType::POINT
         && size_t(selected_point_index_) < annotation_data.points.size())
         return selected_point_index_;
      else if((selected_type_ == CalibType::LINE_A
               || selected_type_ == CalibType::LINE_B)
              && size_t(selected_line_index_) < annotation_data.lines.size())
         return selected_line_index_;
      return -1;
   }

   calibration::LabelledPoint* selected_point() noexcept
   {
      int ind = selected_index();
      if(selected_type_ == CalibType::POINT
         && size_t(ind) < annotation_data.points.size()
         && annotation_data.points[ind].camera_no == current_camera_no())
         return &annotation_data.points.at(ind);
      return nullptr;
   }

   calibration::LabelledLine* selected_line() noexcept
   {
      int ind = selected_index();
      if((selected_type_ == CalibType::LINE_A
          || selected_type_ == CalibType::LINE_B)
         && size_t(ind) < annotation_data.lines.size()
         && annotation_data.lines[ind].camera_no == current_camera_no())
         return &annotation_data.lines[ind];
      return nullptr;
   }

   const calibration::LabelledPoint* selected_point() const noexcept
   {
      return const_cast<This::Pimpl*>(this)->selected_point();
   }

   const calibration::LabelledLine* selected_line() const noexcept
   {
      return const_cast<This::Pimpl*>(this)->selected_line();
   }

   void select_point(int ind) noexcept
   {
      if(size_t(ind) >= annotation_data.points.size()) ind = -1;
      selected_point_index_ = ind;
      selected_type_        = (ind >= 0) ? CalibType::POINT : CalibType::NONE;
      emit parent->data_updated();
   }

   void select_line(int ind, bool end) noexcept
   {
      if(size_t(ind) >= annotation_data.lines.size()) ind = -1;
      selected_line_index_ = ind;
      selected_type_       = (ind == -1)     ? CalibType::NONE
                             : (end == true) ? CalibType::LINE_B
                                             : CalibType::LINE_A;
      emit parent->data_updated();
   }

   bool is_selected(const calibration::LabelledPoint& o) const noexcept
   {
      return &o == selected_point();
   }

   bool is_selected(const calibration::LabelledLine& o) const noexcept
   {
      return &o == selected_line();
   }

   void iterate_lpoint(bool is_next) noexcept
   {
      const int n_points = int(annotation_data.points.size());
      const int n_lines  = int(annotation_data.lines.size());

      if(selected_point()) { // Attempt to get next/prev point
         // Attempt to go to the next/prev point
         select_point(selected_point_index_ + (is_next ? 1 : -1));

         // We've dropped off the of selectable points
         if(!selected_point() && n_lines > 0) {
            // We've gone off the end of points
            if(is_next) {
               select_line(0, false);
            } else {
               select_line(n_lines - 1, true);
            }
         }
      } else if(selected_line()) { // Attempt to get next/prev line
         // Sometime we change the selected endpoint of the line
         if(is_next && selected_type_ == CalibType::LINE_A) {
            select_line(selected_line_index_, true);
         } else if(is_next && selected_type_ == CalibType::LINE_B) {
            select_line(selected_line_index_ + 1, false);
         } else if(!is_next && selected_type_ == CalibType::LINE_A) {
            select_line(selected_line_index_ - 1, true);
         } else if(!is_next && selected_type_ == CalibType::LINE_B) {
            select_line(selected_line_index_, false);
         } else {
            FATAL("logic error");
         }

         // We've dropped off the of selectable lines
         if(!selected_line() && n_points > 0) {
            if(is_next) {
               select_point(0);
            } else {
               select_point(n_points - 1);
            }
         }
      } else if(n_points > 0) { // Attempt to select first/last point
         if(is_next) {
            select_point(0);
         } else {
            select_point(n_points - 1);
         }
      } else if(n_lines > 0) { // Attempt to select first/last line
         if(is_next) {
            select_line(0, false);
         } else {
            select_line(n_lines - 1, true);
         }
      } else { // Deselect (should be a redundant operation)
         selected_type_        = CalibType::NONE;
         selected_point_index_ = -1;
         selected_line_index_  = -1;
         emit parent->data_updated();
      }
   }

   // --------------------------------------------------------------- select pos
   //
   void select_nearest_annotation(const Vector2f pos)
   {
      CalibType best_type = CalibType::NONE;
      int best_index      = -1;
      float best_dist     = std::numeric_limits<float>::max();

      auto run_test = [&](const auto& x, const CalibType type, const int i) {
         const auto dist = (pos - x).quadrance();
         if(dist < best_dist) {
            best_dist  = dist;
            best_index = i;
            best_type  = type;
         }
      };

      auto test_point = [&](const auto& o) {};

      for(size_t i = 0; i < annotation_data.points.size(); ++i) {
         const auto& o = annotation_data.points[i];
         run_test(o.x, CalibType::POINT, int(i));
      }

      for(size_t i = 0; i < annotation_data.lines.size(); ++i) {
         const auto& o = annotation_data.lines[i];
         run_test(o.x0, CalibType::LINE_A, int(i));
         run_test(o.x1, CalibType::LINE_B, int(i));
      }

      if(best_dist < square(15.0f)) { // ya gotta be at least 15 pixels close!
         if(best_type == CalibType::POINT)
            select_point(best_index);
         else if(best_type == CalibType::LINE_A)
            select_line(best_index, false);
         else if(best_type == CalibType::LINE_B)
            select_line(best_index, true);
         else
            FATAL("logic error");
      }
   }

   void create_and_select_point(const Vector2f pos)
   {
      calibration::LabelledPoint o;
      o.camera_no = current_camera_no();
      o.x         = pos;
      if(!is_valid_camera_no(o.camera_no)) {
         LOG_ERR(format(
             "couldn't determine camera number; scene probably not loaded. "
             "Detected {}, but expected a number in the range [{}..{})",
             o.camera_no,
             0,
             count_cameras()));
      } else if(!calc_image_bounds(o.camera_no).contains(to_pt2(o.x))) {
         LOG_ERR(format("attempt to create out-of-bounds point {}.", str(o.x)));
      } else {
         annotation_data.points.push_back(o);
         select_point(int(annotation_data.points.size()) - 1);
         emit parent->data_updated();
      }
   }

   void create_and_select_line(const Vector2f A, const Vector2f B)
   {
      const int camera_no     = current_camera_no();
      const bool camera_valid = is_valid_camera_no(camera_no);
      const AABBi bounds
          = !camera_valid ? AABBi{} : calc_image_bounds(camera_no);
      if(!camera_valid) {
         LOG_ERR(format(
             "couldn't determine camera number; scene probably not loaded."));
      } else if(!bounds.contains(to_pt2(A)) || !bounds.contains(to_pt2(B))) {
         LOG_ERR(format(
             "attempt to create out-of-bounds line {}--{}.", str(A), str(B)));
      } else {
         calibration::LabelledLine o;
         o.camera_no = camera_no;
         o.x0        = A;
         o.x1        = B;
         annotation_data.lines.push_back(o);
         select_line(int(annotation_data.lines.size()) - 1, true);
         emit parent->data_updated();
      }
   }

   // ------------------------------------------------------------- Mutate state
   //
   void update_selected_label_id() noexcept
   {
      auto update_id = [&](auto& o) {
         bool ok           = false;
         int current_label = o.label;
         int new_label     = QInputDialog::getInt(parent,
                                              tr("Input 'label'"),
                                              tr("Label:"),
                                              current_label,
                                              -1,
                                              10000,
                                              1,
                                              &ok);

         if(ok) {
            Expects(new_label >= -1);
            o.label = new_label;
            emit parent->data_updated();
         }
      };

      if(selected_point())
         update_id(*selected_point());
      else if(selected_line())
         update_id(*selected_line());
   }

   void toggle_floor_state() noexcept
   {
      if(selected_point()) {
         auto ptr = selected_point();
         if(ptr->is_origin)
            ptr->on_floor = true;
         else
            ptr->on_floor = !ptr->on_floor;
         emit parent->data_updated();
      } else if(selected_line()) {
         selected_line()->on_floor = !selected_line()->on_floor;
         emit parent->data_updated();
      }
   }

   void set_line_direction(int val) noexcept
   {
      const auto axis = calibration::LabelledLine::Axis(val);
      if((axis != calibration::LabelledLine::X_AXIS)
         && (axis != calibration::LabelledLine::Y_AXIS)
         && (axis != calibration::LabelledLine::Z_AXIS)) {
         LOG_ERR(format("should never happend"));
         return;
      }

      auto ll = selected_line();
      if(ll) {
         ll->axis = (ll->axis == axis) ? calibration::LabelledLine::NONE : axis;
         emit parent->data_updated();
      }
   }

   void toggle_is_origin() noexcept
   {
      auto ptr = selected_point();
      if(ptr) {
         ptr->is_origin = !ptr->is_origin;
         if(ptr->is_origin) ptr->on_floor = true;
         emit parent->data_updated();
      }
   }

   void move_selected(Point2 dxy) noexcept
   {
      auto update_xy = [&](const int camera_no, auto& x) {
         using Tref = decltype(x);
         using T    = typename std::remove_reference<Tref>::type;
         if(!is_valid_camera_no(camera_no)) {
            WARN(format("camera-no = {}, shouldn't happen", camera_no));
            return false;
         }
         const AABBi bounds = calc_image_bounds(camera_no);
         auto new_x         = x + T(dxy.x, dxy.y);
         if((new_x != x) && bounds.contains(to_pt2(new_x))) {
            x = T(new_x.x, new_x.y);
            emit parent->data_updated();
         }
         return true;
      };

      if(selected_point()) {
         update_xy(selected_point()->camera_no, selected_point()->x);
      } else if(selected_line() && (selected_type_ == CalibType::LINE_A)) {
         update_xy(selected_line()->camera_no, selected_line()->x0);
      } else if(selected_line() && (selected_type_ == CalibType::LINE_B)) {
         update_xy(selected_line()->camera_no, selected_line()->x1);
      }
   }

   void delete_selected() noexcept
   {
      bool is_updated = false;
      const int ind   = selected_index();
      if(selected_point()) {
         auto ii = std::next(begin(annotation_data.points), ind);
         Expects(std::addressof(*ii) == selected_point());
         annotation_data.points.erase(ii);
         select_point(ind);
         is_updated = true;
      } else if(selected_line()) {
         auto ii = std::next(begin(annotation_data.lines), ind);
         Expects(std::addressof(*ii) == selected_line());
         annotation_data.lines.erase(ii);
         select_line(ind, false);
         is_updated = true;
      }
      if(is_updated) { emit parent->data_updated(); }
   }

   // ----------------------------------------------------
   //

   Point2 mouse_point(const Point2 xy) const
   {
      return to_pt2(image_viewer->qim_to_mouse(to_vec2(xy)));
   }

   // ----------------------------------------------------------- on-mouse-press
   //
   bool on_mouse_press(QMouseEvent* event)
   {
      parent->setFocus();
      const auto mouse_pos    = Point2(event->x(), event->y());
      const auto image_pos    = to_vec2f(image_viewer->mouse_to_qim(mouse_pos));
      const bool no_modifiers = event->modifiers() == Qt::NoModifier;
      const bool is_left_btn  = event->buttons() == Qt::LeftButton;
      const bool is_right_btn = event->buttons() == Qt::RightButton;
      const bool shift_down   = event->modifiers() == Qt::ShiftModifier;
      const bool ctrl_down    = event->modifiers() == Qt::ControlModifier;

      if((is_left_btn || is_right_btn) && no_modifiers && !mouse_is_panning) {
         select_nearest_annotation(image_pos);
      } else if(is_left_btn && ctrl_down && !mouse_is_panning) {
         // Start create line under cursor
         start_pan(image_pos);
      } else if(mouse_is_panning) {
         // Finish create line under cursor
         stop_pan();
      }

      return false;
   }

   bool on_mouse_move(QMouseEvent* event)
   {
      if(!mouse_is_panning) return false;
      const auto mouse_pos = Point2(event->x(), event->y());
      mouse_move_image_pos = to_vec2f(image_viewer->mouse_to_qim(mouse_pos));
      emit parent->data_updated();
      return false;
   }

   bool on_mouse_release(QMouseEvent* event)
   {
      const bool ctrl_down = event->modifiers() == Qt::ControlModifier;
      if(!ctrl_down) stop_pan();

      if(mouse_is_panning) {
         stop_pan();
         const auto mouse_pos = Point2(event->x(), event->y());
         const auto image_pos = to_vec2f(image_viewer->mouse_to_qim(mouse_pos));
         const auto dist      = (mouse_down_image_pos - image_pos).norm();
         if(dist > 20.0f)
            create_and_select_line(mouse_down_image_pos, image_pos);
         else if(dist < 2.0f)
            create_and_select_point(mouse_down_image_pos);
      }
      return false;
   }

   // ------------------------------------------------------------------ panning
   //
   void start_pan(Vector2f pos)
   {
      mouse_down_image_pos = pos;
      mouse_is_panning     = true;
   }

   void stop_pan() { mouse_is_panning = false; }

   // ------------------------------------------------------------- on-key-press
   //
   bool on_key_press(QKeyEvent* ev)
   {
      const int frame_no      = app_state()->frame_no();
      const bool no_modifiers = ev->modifiers() == Qt::NoModifier;
      const bool alt_down     = ev->modifiers() == Qt::AltModifier;
      const bool ctrl_down    = ev->modifiers() == Qt::ControlModifier;
      const bool shift_down   = ev->modifiers() == Qt::ShiftModifier;
      const int key           = ev->key();

      // Arrow keys
      if(no_modifiers && key == Qt::Key_PageUp)
         iterate_lpoint(true);
      else if(no_modifiers && key == Qt::Key_PageDown)
         iterate_lpoint(false);
      else if(no_modifiers && key == Qt::Key_F)
         toggle_floor_state();
      else if(ctrl_down && key == Qt::Key_I)
         update_selected_label_id();
      else if(no_modifiers && key == Qt::Key_F)
         toggle_floor_state();
      else if(no_modifiers && key == Qt::Key_X)
         set_line_direction(1);
      else if(no_modifiers && key == Qt::Key_Y)
         set_line_direction(2);
      else if(no_modifiers && key == Qt::Key_Z)
         set_line_direction(3);
      else if(no_modifiers && key == Qt::Key_O)
         toggle_is_origin();
      else if(ctrl_down && key == Qt::Key_Left)
         move_selected({-1, 0});
      else if(ctrl_down && key == Qt::Key_Right)
         move_selected({1, 0});
      else if(ctrl_down && key == Qt::Key_Up)
         move_selected({0, -1});
      else if(ctrl_down && key == Qt::Key_Down)
         move_selected({0, 1});
      else if(ctrl_down && key == Qt::Key_Delete)
         delete_selected();
      else if(alt_down && key == Qt::Key_C)
         check_annotation_data();
      else if((alt_down || ctrl_down) && key == Qt::Key_S)
         save_annotation_data();
      else if(key == Qt::Key_Escape)
         select_point(-1);
      else
         return false; // we didn't handle the key event

      return true; // we DID handle the key event
   }

   // ----------------------------------------------------
   //
   int count_cameras() const noexcept
   {
      if(!app_state() || !app_state()->scene_desc()) return 0;
      return app_state()->scene_desc()->n_cameras();
   }

   int current_camera_no() const noexcept
   {
      if(!app_state() || !app_state()->scene_desc()) return -1;
      return app_state()->current_camera();
   }

   bool is_valid_camera_no(const int camera_no) const
   {
      if(!app_state() || !app_state()->scene_desc()) return false;
      const auto scene_desc = app_state()->scene_desc();
      return (camera_no >= 0) && (camera_no < scene_desc->n_cameras());
   }

   AABBi calc_image_bounds(const int camera_no) const
   {
      if(!app_state() || !app_state()->scene_desc()) return {};
      const auto scene_desc = app_state()->scene_desc();
      Expects(camera_no >= 0 && camera_no < scene_desc->n_cameras());
      const int sensor_id = scene_desc->sensor_lookup(camera_no, 0);
      Expects(sensor_id >= 0 && sensor_id < scene_desc->n_sensors());
      return image_bounds(scene_desc->sensor_image.at(sensor_id));
   }

   bool check_annotation_data()
   {
      if(!app_state() || !app_state()->scene_desc()) {
         LOG_ERR(format("app-data not yet loaded"));
         return false;
      }

      const auto scene_desc = app_state()->scene_desc();
      const int n_cameras   = scene_desc->n_cameras();

      vector<AABBi> bounds(n_cameras);
      {
         int counter = 0;
         std::generate(begin(bounds), end(bounds), [&]() {
            return calc_image_bounds(counter++);
         });
      }

      string err_msg = annotation_data.validate_calibration(n_cameras, bounds);
      bool has_error = !err_msg.empty();

      if(has_error) {
         LOG_ERR(format("problems found in annotation data:"));
         cout << indent(err_msg, 3) << endl;
      } else {
         INFO(format("annotation data checks passed!"));
      }

      return !has_error;
   }

   // ---------------------------------------------------- reset-annotation-data
   //
   bool reset_annotation_data()
   {
      annotation_data = calibration::CalibrationInfo{};
      emit parent->data_updated();
      return true;
   }

   // --------------------------------------------------------------- init-dcams
   //
   void init_dcams()
   {
      Expects(app_state());
      Expects(app_state()->scene_desc());
      const auto scene_desc = app_state()->scene_desc();
      const int n_cameras   = scene_desc->n_cameras();
      const int n_sensors   = scene_desc->n_sensors();
      dcams.resize(n_cameras);
      for(auto i = 0; i < n_cameras; ++i) {
         const int sensor_no = scene_desc->sensor_lookup(i, 0);
         Expects(size_t(sensor_no) < size_t(n_sensors));
         dcams[i] = scene_desc->dcam(sensor_no);
      }
   }

   void init_dcam_combo()
   {
      Expects(app_state());
      Expects(app_state()->scene_desc());
      const auto scene_desc = app_state()->scene_desc();
      const int n_cameras   = scene_desc->n_cameras();
      combo_ref_cam->clear();
      for(auto i = 0; i < n_cameras; ++i) {
         const auto& bcam_info = scene_desc->bcam_infos[i];
         combo_ref_cam->addItem(
             to_qstr(format("#{} :: {}", i, bcam_info.camera_id)));
      }
   }

   bool update_dcam_positions()
   {
      const auto opt_data_copy = opt_data; // COPY the shared pointer
      if(!opt_data_copy) return false;
      if(dcams.size() == 0) return false; // init_dcams has never been called
      Expects(dcams.size() == opt_data_copy->size());
      for(size_t i = 0; i < dcams.size(); ++i) {
         std::tie(dcams[i].C, dcams[i].q)
             = make_camera_extrinsics(opt_data_copy->at(i));
      }
      return true;
   }

   // ----------------------------------------------------- load-annotation-data
   //
   bool load_annotation_data()
   {
      if(!app_state() || !app_state()->scene_desc()) {
         LOG_ERR(format("app-data not yet loaded"));
         return false;
      }

      const auto fname = annotation_filename(*app_state()->scene_desc());
      try {
         string raw_data;
         s3_load(fname, raw_data);
         if(!annotation_data.read(parse_json(raw_data))) {
            LOG_ERR(format("failed to read data from '{}'", fname));
            return false;
         }
         INFO(format("annotations loaded from {}", fname));
      } catch(std::exception& e) {
         LOG_ERR(format("failed to load/read '{}': {}", fname, e.what()));
         return false;
      }

      if(!loaded_annotation_data) {
         loaded_annotation_data                 = true;
         perform_annotation_data_backup_on_save = true;
      }

      emit parent->data_updated();
      return true;
   }

   // ----------------------------------------------------- save-annotation-data
   //
   bool save_annotation_data()
   {
      if(!app_state() || !app_state()->scene_desc()) {
         LOG_ERR(format("app-data not yet loaded"));
         return false;
      }

      const auto fname = annotation_filename(*app_state()->scene_desc());

      // Attempt to perform backup is scheduled
      if(perform_annotation_data_backup_on_save) {
         const auto backup_fname
             = annotation_backup_filename(*app_state()->scene_desc());
         INFO(format("backing up\n   {}\n   ->\n   {}", fname, backup_fname));

         try {
            string raw_data;
            s3_load(fname, raw_data);
            s3_store(backup_fname, raw_data);
            INFO(format("success"));
         } catch(std::exception& e) {
            WARN(format("backup failed (may not be a problem): {}", e.what()));
         }

         perform_annotation_data_backup_on_save = false;
      }

      try {
         INFO(format("saving to\n   {}", fname));
         Json::StyledWriter writer;
         string raw_data = writer.write(annotation_data.to_json());
         s3_store(fname, raw_data);
         INFO(format("success"));
      } catch(std::exception& e) {
         LOG_ERR(format("save operation failed: {}", e.what()));
         return false;
      }

      return true;
   }

   // -------------------------------------------------- publish-annotation-data
   //
   void publish_annotation_data()
   {
      if(!app_state() || !app_state()->scene_desc()) {
         LOG_ERR(format("app-data not yet loaded"));
         return;
      }

      if(!check_annotation_data()) {
         LOG_ERR(format("annotation data needs to be checked"));
         return;
      }

      SceneDescriptionInfo info = app_state()->scene_desc()->scene_info;
      info.scene_key            = format("{}_o_{}",
                              remove_all_timestamps(info.scene_key),
                              timestamp_fname_str(Timestamp::now()));

      // Now update all the bcam-transforms
      LOG_ERR(format("TODO, update all the bcam-transforms"));

      try {
         const auto fname
             = resolve_key(AssetType::SCENE_DESCRIPTION, info.scene_key);
         INFO(format("saving new scene {} to {}", info.scene_key, fname));
         store(info, info.scene_key);
         INFO(format("success"));
      } catch(std::exception& e) {
         LOG_ERR(format("failed: {}", e.what()));
      }
   }
};

// --------------------------------------------------------------------- make ui

void This::Pimpl::make_ui() noexcept
{
   auto scene_desc = app_state()->scene_desc();

   // ---- (*) ---- Create widgets
   { // cpanel widgets
      rb_show_no_error       = new QRadioButton("None");
      rb_show_selected_error = new QRadioButton("Selected");
      rb_show_all_errors     = new QRadioButton("All");

      combo_ref_cam = new QComboBox();

      btn_optimize = new QPushButton("&Optimize");
      btn_check    = new QPushButton("&Check");
      btn_reset    = new QPushButton("Reset");
      btn_revert   = new QPushButton("Revert");
      btn_save     = new QPushButton("&Save");
      btn_publish  = new QPushButton("&Publish");

      rb_show_no_error->installEventFilter(main_window);
      rb_show_selected_error->installEventFilter(main_window);
      rb_show_all_errors->installEventFilter(main_window);
      combo_ref_cam->installEventFilter(main_window);
      btn_optimize->installEventFilter(main_window);
      btn_check->installEventFilter(main_window);
      btn_reset->installEventFilter(main_window);
      btn_revert->installEventFilter(main_window);
      btn_save->installEventFilter(main_window);
      btn_publish->installEventFilter(main_window);
   }

   { // cpanel
      auto make_rb_group = [&]() {
         QHBoxLayout* layout = new QHBoxLayout;
         auto wgt            = new QWidget{};
         layout->addWidget(rb_show_no_error);
         layout->addWidget(rb_show_selected_error);
         layout->addWidget(rb_show_all_errors);
         layout->addStretch(0);
         wgt->setLayout(layout);
         wgt->installEventFilter(main_window);
         return wgt;
      };

      auto make_group_box = [&]() {
         QVBoxLayout* layout = new QVBoxLayout;
         auto wgt            = new QGroupBox("Show Errors");
         layout->addWidget(make_rb_group());
         layout->addWidget(combo_ref_cam);
         wgt->setLayout(layout);
         wgt->installEventFilter(main_window);
         return wgt;
      };

      auto make_controls_wgt = [&]() {
         QFormLayout* layout = new QFormLayout{};
         auto wgt            = new QWidget{};
         layout->addRow(make_group_box());
         layout->addRow(make_instructions_label());
         wgt->setLayout(layout);
         wgt->installEventFilter(main_window);
         return wgt;
      };

      QVBoxLayout* layout = new QVBoxLayout{};
      layout->addWidget(make_controls_wgt());
      // layout->addWidget(make_vertical_expander());
      layout->addWidget(btn_reset);
      layout->addWidget(btn_revert);
      layout->addWidget(btn_check);
      layout->addWidget(btn_save);
      layout->addWidget(btn_optimize);
      layout->addWidget(btn_publish);

      cpanel = new QWidget{};
      cpanel->setFixedWidth(app_state()->config().cpanel_width);
      cpanel->setStyleSheet("background-color: white;");
      cpanel->setLayout(layout);
      cpanel->installEventFilter(main_window);
   }

   { // image viewer (for seeing the video)
      image_viewer = new ImageViewer2{};
      image_viewer->setSizePolicy(QSizePolicy::Expanding,
                                  QSizePolicy::Expanding);
      image_viewer->setStyleSheet("background-color: blue;");
      image_viewer->set_pixel_indicator(false);
      image_viewer->is_pannable = true;
      image_viewer->is_zoomable = true;
      image_viewer->set_offset(Point2(8, 8));
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

   auto make_viewer_layout = [&]() {
      QVBoxLayout* layout = new QVBoxLayout{};
      layout->addWidget(image_viewer);
      auto wgt = new QWidget{};
      wgt->setLayout(layout);
      wgt->installEventFilter(main_window);
      return wgt;
   };

   QHBoxLayout* layout = new QHBoxLayout{};
   layout->addWidget(cpanel);
   layout->addWidget(make_viewer_layout());
   parent->setLayout(layout);
   parent->installEventFilter(main_window);

   // ---- (*) ---- Initialize
   model_to_widgets();
   rb_show_all_errors->setChecked(true);

   // ---- (*) ---- Wiring
   connect(app_state(),
           SIGNAL(open_finished()),
           parent,
           SLOT(model_to_widgets()),
           Qt::QueuedConnection);

   connect(app_state(),
           SIGNAL(open_finished()),
           parent,
           SLOT(on_load()),
           Qt::QueuedConnection);

   connect(app_state(),
           SIGNAL(video_frame_changed(int)),
           parent,
           SLOT(on_redraw()),
           Qt::QueuedConnection);

   connect(app_state(),
           SIGNAL(camera_changed(unsigned)),
           parent,
           SLOT(on_redraw()),
           Qt::QueuedConnection);

   connect(parent,
           SIGNAL(data_updated()),
           parent,
           SLOT(on_redraw()),
           Qt::QueuedConnection);

   connect(btn_optimize, SIGNAL(pressed()), parent, SLOT(on_optimize()));
   connect(btn_check, SIGNAL(pressed()), parent, SLOT(on_check()));
   connect(btn_reset, SIGNAL(pressed()), parent, SLOT(on_reset()));
   connect(btn_revert, SIGNAL(pressed()), parent, SLOT(on_revert()));
   connect(btn_save, SIGNAL(pressed()), parent, SLOT(on_save()));
   connect(btn_publish, SIGNAL(pressed()), parent, SLOT(on_publish()));

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

   CONNECT_RB(rb_show_no_error, on_redraw);
   CONNECT_RB(rb_show_selected_error, on_redraw);
   CONNECT_RB(rb_show_all_errors, on_redraw);

   connect(combo_ref_cam,
           SIGNAL(currentIndexChanged(int)),
           parent,
           SLOT(on_redraw()));

#undef CONNECT_SLIDER
#undef CONNECT_RB
#undef CONNECT_CB
}

// ---------------------------------------------------------------- Construction

This::This(QWidget* parent)
    : QWidget(parent)
    , pimpl_(new Pimpl(parent, this))
{
   Expects(parent != nullptr);
   pimpl_->make_ui();
}

This::~This() { delete pimpl_; }

bool This::widgets_to_model() { return pimpl_->widgets_to_model(); }

void This::model_to_widgets() { pimpl_->model_to_widgets(); }

// ------------------------------------------------------------- key press event

void This::keyPressEvent(QKeyEvent* ev) {}

// -------------------------------------------------------------------- on-check
//
void This::on_optimize() { LOG_ERR(format("TBA")); }

// -------------------------------------------------------------------- on-check
//
void This::on_check() { pimpl_->check_annotation_data(); }

// -------------------------------------------------------------------- on-reset
//
void This::on_reset()
{
   QMessageBox msgBox;
   msgBox.setWindowTitle("Careful!");
   msgBox.setText("Are you sure you want to reset?");
   msgBox.setStandardButtons(QMessageBox::Yes);
   msgBox.addButton(QMessageBox::No);
   msgBox.setDefaultButton(QMessageBox::No);
   if(msgBox.exec() == QMessageBox::Yes) {
      INFO(format("resetting..."));
      pimpl_->reset_annotation_data();
   }
}

// --------------------------------------------------------- on-load tracks-file
//
void This::on_load()
{
   pimpl_->init_dcams();
   pimpl_->init_dcam_combo();
   pimpl_->load_annotation_data();
}

// --------------------------------------------------------- on-save tracks-file
//
void This::on_save() { pimpl_->save_annotation_data(); }

// ------------------------------------------------------- on-revert tracks-file
//
void This::on_revert()
{
   QMessageBox msgBox;
   msgBox.setWindowTitle("Careful!");
   msgBox.setText("Are you sure you want to revert?");
   msgBox.setStandardButtons(QMessageBox::Yes);
   msgBox.addButton(QMessageBox::No);
   msgBox.setDefaultButton(QMessageBox::No);
   if(msgBox.exec() == QMessageBox::Yes) {
      INFO(format("reloading..."));
      pimpl_->load_annotation_data();
   }
}

// ----------------------------------------------------- on-publish ground-truth
//
void This::on_publish() { pimpl_->publish_annotation_data(); }

// ---------------------------------------------------------- handle_key_pressed
//
bool This::handle_key_pressed(QKeyEvent* ev)
{
   return pimpl_->on_key_press(ev);
}

// --------------------------------------------------------------- epipolar-line

static void debug_point(const DistortedCamera& dcam0,
                        const DistortedCamera& dcam1,
                        const Vector3 X)
{
   const auto d0    = project_to_distorted(dcam0, X); // distorted
   const auto d1    = project_to_distorted(dcam1, X);
   const auto u0    = dcam0.cu.undistort(d0);
   const auto u1    = dcam1.cu.undistort(d1);
   const auto ray0  = backproject_ray(dcam0, d0);
   const auto ray1  = backproject_ray(dcam1, d1);
   const auto thet0 = std::fabs(dot(ray0, (dcam0.C - X).normalised()));
   const auto thet1 = std::fabs(dot(ray1, (dcam1.C - X).normalised()));
   const auto ll0   = epipolar_line_in_undistorted_coords(dcam1, dcam0, u1);
   const auto ll1   = epipolar_line_in_undistorted_coords(dcam0, dcam1, u0);
   const auto l0    = project_point_line(ll0, u0);
   const auto l1    = project_point_line(ll1, u1);
   const auto e0    = dcam0.cu.distort(l0);
   const auto e1    = dcam1.cu.distort(l1);

   TRACE(format(R"V0G0N(
DEBUG POINT

   workingfmr : {}, {}
   X          : {}
   d0, d1     : {}, {}
   u0, u1     : {}, {}
   ray0, ray1 : {}, {}   ({}, {})
   ll0, ll1   : {}, {}
   l0, l1     : {}, {}
   e0, e1     : {}, {}   ({}, {})

)V0G0N",
                str(dcam0.cu.working_format()),
                str(dcam1.cu.working_format()),
                str(X),
                str(d0),
                str(d1),
                str(u0),
                str(u1),
                str(ray0),
                str(ray1),
                thet0,
                thet1,
                str(ll0),
                str(ll1),
                str(l0),
                str(l1),
                str(e0),
                str(e1),
                (e0 - d0).norm(),
                (e1 - d1).norm()));
}

// ------------------------------------------------------------------- on-redraw
//
void This::on_redraw()
{
   auto& P                     = *pimpl_;
   const auto& annotation_data = P.annotation_data;

   auto app_ptr            = app_state();
   auto set_qimage_to_null = [&]() { P.image_viewer->set_qimage(nullptr); };

   if(app_ptr == nullptr || app_ptr->frame_results() == nullptr
      || app_state()->scene_desc() == nullptr)
      return set_qimage_to_null();

   shared_ptr<const pipeline::copy_sensor_images::Result> fdat = P.poke_tasks();
   auto scene_desc        = app_state()->scene_desc();
   const auto frame_no    = app_ptr->frame_no();
   const int sensor_no    = app_ptr->current_sensor_index();
   const int camera_no    = app_ptr->current_camera();
   const int other_cam_no = P.combo_ref_cam->currentIndex();
   P.update_dcam_positions();

   if(fdat == nullptr) return set_qimage_to_null();
   if(fdat->images.frame_no != frame_no) return set_qimage_to_null();
   if(size_t(sensor_no) >= fdat->images.sensor_images.size())
      return set_qimage_to_null();
   if(size_t(camera_no) >= P.dcams.size()) return set_qimage_to_null();

   ARGBImage argb      = cv_to_argb(fdat->images.sensor_images[sensor_no]);
   const Point2 offset = Point2{-4, 7};

   const DistortedCamera* dcam0 = &P.dcams.at(camera_no);
   const DistortedCamera* dcam1 = (size_t(other_cam_no) < P.dcams.size())
                                      ? &P.dcams.at(other_cam_no)
                                      : nullptr;
   const bool dcam1_okay        = (dcam1 != nullptr) && (dcam1 != dcam0);

   const bool no_errors = P.rb_show_no_error->isChecked();
   const bool selected_errors
       = P.rb_show_selected_error->isChecked() && dcam1_okay;
   const bool all_errors = P.rb_show_all_errors->isChecked() && dcam1_okay;

   { // Draw the camera number onto the bottom-right corner
      if(size_t(camera_no) < scene_desc->bcam_infos.size()) {
         const auto& bcam_info = scene_desc->bcam_infos.at(camera_no);
         const string s    = format("{} :: {}", camera_no, bcam_info.camera_id);
         const Point2 dims = render_tiny_dimensions(s);
         const Point2 offset = Point2{-4, -2};
         const Point2 pos
             = Point2{int(argb.width) - dims.x, int(argb.height) - dims.y};
         render_string(argb, s, pos + offset, k_cyan, k_black);
      }
   }

   auto kolour = [&](const auto& o) {
      return (o.label == -1) ? k_black : colour_set_4(o.label);
   };

   auto render_point = [&](const auto& o) { // Render points
      if(o.camera_no != camera_no) return;
      const bool o_is_selected = P.is_selected(o);
      const Point2 x           = to_pt2(o.x);
      const auto k             = kolour(o);

      if((selected_errors && o_is_selected) || all_errors) {
         auto u = P.find_point(other_cam_no, o.label);
         if(u != nullptr) {
            Expects(u->label == o.label);
            Expects(u->camera_no == other_cam_no);

            const auto x0  = to_vec2(o.x);
            const auto x1  = to_vec2(u->x);
            const auto u0  = to_vec2(dcam0->cu.undistort(x0));
            const auto u1  = to_vec2(dcam1->cu.undistort(x1));
            const auto ll0 = epipolar_line_in_undistorted_coords(
                *dcam1, *dcam0, to_vec2(u1));
            const auto m0 = project_point_line(ll0, u0);

            bresenham(to_vec2(x0),
                      dcam0->cu.distort(m0),
                      argb.bounds(),
                      [&](int x, int y) { set(argb, x, y, k_red); });
         }
      }

      if(o.on_floor) {
         draw_cross(argb, x, k, 5);
      } else {
         draw_square(argb, x, k, 5);
         set(argb, x, k);
      }

      if(o.is_origin) {
         outline_circle(argb, to_vec2(o.x), k_yellow, 7);
         outline_circle(argb, to_vec2(o.x), k_yellow, 6);
         outline_circle(argb, to_vec2(o.x), k, 6);
      }

      const auto back_k = (o_is_selected ? k_red : k_black);
      render_string(argb, format("{}", o.label), x + offset, k_yellow, back_k);
   };

   auto render_line = [&](const auto& o) { // Render lines
      if(o.camera_no != camera_no) return;
      const bool o_is_selected = P.is_selected(o);
      const auto x0            = to_vec2(o.x0);
      const auto x1            = to_vec2(o.x1);
      const auto k             = kolour(o);
      if(o.on_floor) {
         plot_line_AA(x0, x1, argb.bounds(), [&](int x, int y, float a) {
            if(!argb.in_bounds(x, y)) return;
            argb(x, y) = blend(k, argb(x, y), a);
         });
      } else {
         bresenham(
             x0, x1, argb.bounds(), [&](int x, int y) { set(argb, x, y, k); });
      }
      const auto back_k = (o_is_selected ? k_red : k_black);
      render_string(argb,
                    format("{},{}", o.label, str(o.axis)),
                    to_pt2(x0) + offset,
                    k_yellow,
                    back_k);

      if(o_is_selected) {
         const bool end = (P.selected_type_ == CalibType::LINE_B);
         const auto x   = (end == true) ? x1 : x0;
         outline_circle(argb, x, k_white, 7);
         outline_circle(argb, x, k_white, 6);
         outline_circle(argb, x, k_red, 6);
      }

   };

   for(const auto& point : annotation_data.points)
      if(!P.is_selected(point)) render_point(point);
   for(const auto& line : annotation_data.lines)
      if(!P.is_selected(line)) render_line(line);

   if(P.selected_point()) render_point(*P.selected_point());
   if(P.selected_line()) render_line(*P.selected_line());

   if(P.mouse_is_panning) {
      bresenham(to_vec2(P.mouse_down_image_pos),
                to_vec2(P.mouse_move_image_pos),
                argb.bounds(),
                [&](int x, int y) {
                   if(argb.in_bounds(x, y))
                      argb(x, y) = invert_colour(argb(x, y));
                });
   }

   to_qimage(argb, *P.qim_ptr);
   P.image_viewer->set_qimage(P.qim_ptr);
}
