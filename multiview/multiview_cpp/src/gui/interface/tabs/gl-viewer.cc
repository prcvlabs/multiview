
#include "stdinc.hpp"

#include <QTabWidget>

#include "gl-viewer.hh"

#include "gui/app-state.hh"
#include "gui/gl/gui-render-options.hpp"
#include "gui/gl/gui-render.hpp"
#include "gui/qt-helpers.hpp"
#include "gui/widgets/gl-visualizer.hh"
#include "gui/widgets/labeled-slider.hh"

#define This GlViewer

using namespace perceive;

// ----------------------------------------------------------------------- pimpl

struct This::Pimpl
{
   This* parent{nullptr};
   QWidget* cpanel{nullptr};
   GLVisualizer* gl_visualizer{nullptr};

   //@{ cpanel widgets
   QCheckBox* cb_image_colors{nullptr};
   QCheckBox* cb_draw_roi{nullptr};
   QCheckBox* cb_draw_floor_image{nullptr};
   QCheckBox* cb_draw_clip_to_aabb{nullptr};
   QCheckBox* cb_draw_C_update{nullptr};
   QCheckBox* cb_skeleton_debug{nullptr};

   QCheckBox* cb_draw_cams{nullptr};
   QCheckBox* cb_draw_pose3d{nullptr};
   QCheckBox* cb_draw_loaded_tracks{nullptr};
   QCheckBox* cb_draw_axis{nullptr};
   QCheckBox* cb_draw_grid_xy{nullptr};
   QCheckBox* cb_draw_grid_xz{nullptr};
   QCheckBox* cb_draw_grid_yz{nullptr};
   QCheckBox* cb_draw_fine_grid_xy{nullptr};
   QCheckBox* cb_do_capture{nullptr};
   QCheckBox* cb_do_capture_movie{nullptr};
   QCheckBox* cb_do_rotate{nullptr};
   LabeledSlider* slider_helicopter_theta{nullptr};
   LabeledSlider* slider_rotation_speed{nullptr};
   LabeledSlider* slider_global_dx{nullptr};
   LabeledSlider* slider_global_dy{nullptr};
   LabeledSlider* slider_rotation_z{nullptr};
   QPushButton* button_show_transform{nullptr};
   LabeledSlider* slider_pan_x{nullptr};
   LabeledSlider* slider_pan_y{nullptr};
   LabeledSlider* slider_pan_z{nullptr};
   LabeledSlider* slider_C_x{nullptr}; // center
   LabeledSlider* slider_C_y{nullptr};
   LabeledSlider* slider_C_z{nullptr};
   LabeledSlider* slider_saa_x{nullptr}; // rotation
   LabeledSlider* slider_saa_y{nullptr};
   LabeledSlider* slider_saa_z{nullptr};
   vector<QCheckBox*> cb_pt_clouds;

   // --- Skeleton Debug Controls --- //
   QComboBox* combo_p2d_select{nullptr};
   LabeledSlider* slider_p2d_select{nullptr};
   LabeledSlider* slider_p3d_scale{nullptr};

   QWidget* wgt_skeleton_debug_ctrls = nullptr;
   QWidget* wgt_gl_ctrls             = nullptr;
   //@}

   bool in_movie_capture_op{false};
   int capture_counter = 0;
   bool capture_ready_flag{false};

   GuiRenderOptions opts;
   shared_ptr<const pipeline::floor_hist::Result> result_ptr;
   shared_ptr<const pipeline::localization::Result> op_result_ptr;

   vector<Point2> combo_p2d_select_lookup;

   Pimpl(This* in_parent)
       : parent(in_parent)
   {}
   void make_ui() noexcept;

   void set_result(shared_ptr<const pipeline::floor_hist::Result> ptr) noexcept
   {
      if(result_ptr.get() != ptr.get()) {
         result_ptr = ptr;
         emit parent->data_updated();
      }
   }

   void
   set_op_result(shared_ptr<const pipeline::localization::Result> ptr) noexcept
   {
      if(op_result_ptr.get() != ptr.get()) {
         op_result_ptr = ptr;
         emit parent->data_updated();
      }
   }

   pipeline::floor_hist::Task* get_task_ptr(AppState& app)
   {
      return const_cast<pipeline::floor_hist::Task*>(get_task_cptr(app));
   }

   pipeline::localization::Task* get_op_task_ptr(AppState& app)
   {
      return const_cast<pipeline::localization::Task*>(get_op_task_cptr(app));
   }

   const pipeline::floor_hist::Task* get_task_cptr(const AppState& app)
   {
      if(!app.frame_results()) return nullptr;
      return &(app.frame_results()->floor_hist);
   }

   const pipeline::localization::Task* get_op_task_cptr(const AppState& app)
   {
      if(!app.frame_results()) return nullptr;
      return &(app.frame_results()->localization);
   }

   void draw_gl() noexcept
   {
      widgets_to_model();
      auto app_ptr      = app_state();
      auto scene_desc   = app_ptr->scene_desc();
      const AABB bounds = (scene_desc == nullptr)
                              ? AABB::nan()
                              : scene_desc->scene_info.hist_bounds;

      auto get_loc_ret
          = [&]() -> shared_ptr<const pipeline::localization::Result> {
         auto frame_ptr = app_ptr->frame_results();
         return !frame_ptr ? nullptr
                           : frame_ptr->localization.try_get_casted_result();
      };
      const auto loc_ptr = get_loc_ret();

      gui_render_point_cloud(opts, (loc_ptr ? loc_ptr.get() : nullptr), bounds);

      if(in_movie_capture_op and loc_ptr and !capture_ready_flag) {
         capture_ready_flag = true; // prevents multiple calls in event loop
         gl_visualizer->setCapture(true, capture_counter++);
      }
   }

   void model_to_widgets() noexcept
   {
      block_signal_set_checked(cb_image_colors, opts.use_image_colors);
      block_signal_set_checked(cb_draw_roi, opts.draw_roi);
      block_signal_set_checked(cb_draw_floor_image, opts.draw_floor_image);
      block_signal_set_checked(cb_draw_cams, opts.draw_cams);
      block_signal_set_checked(cb_draw_pose3d, opts.draw_pose3d);
      block_signal_set_checked(cb_draw_loaded_tracks, opts.draw_loaded_tracks);
      block_signal_set_checked(cb_draw_axis, opts.draw_axis);
      block_signal_set_checked(cb_draw_clip_to_aabb, opts.clip_point_cloud);
      block_signal_set_checked(cb_draw_C_update, opts.live_C_update);
      block_signal_set_checked(cb_skeleton_debug, opts.skeleton_debug);
      block_signal_set_checked(cb_draw_grid_xy, opts.draw_grid_xy);
      block_signal_set_checked(cb_draw_grid_xy, opts.draw_grid_xz);
      block_signal_set_checked(cb_draw_grid_xy, opts.draw_grid_yz);
      block_signal_set_checked(cb_draw_fine_grid_xy, opts.draw_fine_grid_xy);
      //
      block_signal_set_checked(cb_do_rotate, opts.do_rotate);
      block_signal_set_checked(cb_do_capture, opts.do_capture);
      block_signal_set_checked(cb_do_capture_movie, opts.do_capture_movie);
      block_signal_set_slider(slider_rotation_speed, opts.rotation_speed);
      block_signal_set_slider(slider_helicopter_theta,
                              to_degrees(opts.helicopter_theta));
      //
      for(auto cb : cb_pt_clouds) block_signal_set_checked(cb, false);
      for(auto x : opts.cam_pts_cloud_ind)
         if(unsigned(x.x) < cb_pt_clouds.size())
            block_signal_set_checked(cb_pt_clouds[x.x], true);
      //
      block_signal_set_slider(slider_global_dx, opts.dxy.x);
      block_signal_set_slider(slider_global_dy, opts.dxy.y);
      block_signal_set_slider(slider_rotation_z, to_degrees(opts.rotation_z));

      //
      block_signal_set_slider(slider_pan_x, opts.pan_xyz.x);
      block_signal_set_slider(slider_pan_y, opts.pan_xyz.y);
      block_signal_set_slider(slider_pan_z, opts.pan_xyz.z);

      //
      block_signal_set_slider(slider_C_x, opts.C.x);
      block_signal_set_slider(slider_C_y, opts.C.y);
      block_signal_set_slider(slider_C_z, opts.C.z);
      block_signal_set_slider(slider_saa_x, opts.saa.x);
      block_signal_set_slider(slider_saa_y, opts.saa.y);
      block_signal_set_slider(slider_saa_z, opts.saa.z);

      { // --- Skeleton Debug ---
         const auto ii = std::find_if(cbegin(combo_p2d_select_lookup),
                                      cend(combo_p2d_select_lookup),
                                      [&](const Point2& p) {
                                         return (p.x == opts.p2d_sensor)
                                                && (p.y == opts.p2d_index);
                                      });

         if(ii == cend(combo_p2d_select_lookup)) {
            if(combo_p2d_select->count() > 0)
               block_signal_set_combobox(combo_p2d_select, 0);
            else
               block_signal_set_combobox(combo_p2d_select, -1);
         } else {
            block_signal_set_combobox(
                combo_p2d_select,
                int(std::distance(cbegin(combo_p2d_select_lookup), ii)));
         }
      }

      block_signal_set_slider(slider_p3d_scale, opts.p3d_height);
   }

   void widgets_to_model() noexcept
   {
      opts.use_image_colors   = cb_image_colors->isChecked();
      opts.draw_roi           = cb_draw_roi->isChecked();
      opts.draw_floor_image   = cb_draw_floor_image->isChecked();
      opts.draw_cams          = cb_draw_cams->isChecked();
      opts.draw_pose3d        = cb_draw_pose3d->isChecked();
      opts.draw_loaded_tracks = cb_draw_loaded_tracks->isChecked();
      opts.draw_axis          = cb_draw_axis->isChecked();
      opts.clip_point_cloud   = cb_draw_clip_to_aabb->isChecked();
      opts.live_C_update      = cb_draw_C_update->isChecked();
      opts.skeleton_debug     = cb_skeleton_debug->isChecked();
      opts.draw_grid_xy       = cb_draw_grid_xy->isChecked();
      opts.draw_grid_xz       = cb_draw_grid_xz->isChecked();
      opts.draw_grid_yz       = cb_draw_grid_yz->isChecked();
      opts.draw_fine_grid_xy  = cb_draw_fine_grid_xy->isChecked();
      //
      opts.do_rotate        = cb_do_rotate->isChecked();
      opts.do_capture       = cb_do_capture->isChecked();
      opts.do_capture_movie = cb_do_capture_movie->isChecked();
      opts.helicopter_theta = to_radians(slider_helicopter_theta->value());
      opts.rotation_speed   = slider_rotation_speed->value();
      //
      opts.cam_pts_cloud_ind.clear();
      for(auto i = 0u; i < cb_pt_clouds.size(); ++i)
         if(cb_pt_clouds[i]->isChecked())
            opts.cam_pts_cloud_ind.emplace_back(int(i), 0);

      gl_visualizer->setHelicopterRotateInclination(opts.helicopter_theta);
      gl_visualizer->setRotationSpeed(opts.rotation_speed);
      if(gl_visualizer->isRotating() != opts.do_rotate)
         gl_visualizer->setRotating(opts.do_rotate);

      if(!opts.do_capture_movie) {
         capture_ready_flag  = false;
         in_movie_capture_op = false;
         capture_counter     = 0;
      }

      if(gl_visualizer->isCapturing() and !opts.do_capture
         and !opts.do_capture_movie) {
         gl_visualizer->setCapture(false);
      } else if(!gl_visualizer->isCapturing() and opts.do_capture) {
         gl_visualizer->captureOneRevolution();
      } else if(!gl_visualizer->isCapturing() and opts.do_capture_movie
                and !in_movie_capture_op) {
         capture_ready_flag  = false;
         in_movie_capture_op = true;
         capture_counter     = 1;
         gl_visualizer->setCapture(true, capture_counter++);
      }

      //
      opts.dxy.x      = slider_global_dx->value();
      opts.dxy.y      = slider_global_dy->value();
      opts.rotation_z = to_radians(slider_rotation_z->value());

      //
      opts.pan_xyz.x = slider_pan_x->value();
      opts.pan_xyz.y = slider_pan_y->value();
      opts.pan_xyz.z = slider_pan_z->value();

      //
      opts.C.x   = slider_C_x->value();
      opts.C.y   = slider_C_y->value();
      opts.C.z   = slider_C_z->value();
      opts.saa.x = slider_saa_x->value();
      opts.saa.y = slider_saa_y->value();
      opts.saa.z = slider_saa_z->value();

      { // --- Skeleton Debug ---
         const auto idx = combo_p2d_select->currentIndex();
         if(size_t(idx) < combo_p2d_select_lookup.size()) {
            opts.p2d_sensor = combo_p2d_select_lookup[idx].x;
            opts.p2d_index  = combo_p2d_select_lookup[idx].y;
         } else {
            opts.p2d_sensor = -1;
            opts.p2d_index  = -1;
         }
         parent->update_aux_skeleton();
      }

      opts.p3d_height = slider_p3d_scale->value();
   }

   void set_cpanel_parts_visibility() noexcept
   {
      const bool is_skeleton_debug = cb_skeleton_debug->isChecked();
      wgt_gl_ctrls->setVisible(!is_skeleton_debug);
      wgt_skeleton_debug_ctrls->setVisible(is_skeleton_debug);
   }

   void update_widgets() noexcept
   {
      static const perceive::pipeline::pose_skeleton_init::Result*
          op_init_ret_ptr
          = nullptr;

      auto combo             = combo_p2d_select;
      const bool was_blocked = combo->blockSignals(true);
      bool changed           = false;
      if(op_result_ptr == nullptr) { // clear the combo, if not cleared
         op_init_ret_ptr = nullptr;
         auto none_qs    = to_qstr("none");
         if(combo->count() != 1 || combo->itemText(0) != none_qs
            || combo->currentIndex() != 0) {
            combo->clear();
            combo->addItem(none_qs);
            combo->setCurrentIndex(0);
            changed = true;
         }
      } else if(op_init_ret_ptr != op_result_ptr->pose_skeleton_result.get()) {
         // populate the combo, if required
         op_init_ret_ptr     = op_result_ptr->pose_skeleton_result.get();
         const auto& op_ret  = op_init_ret_ptr->op;
         const int n_sensors = int(op_ret.sensors().size());

         combo->clear();
         combo_p2d_select_lookup.clear();
         { // add the 'none' item
            combo_p2d_select_lookup.emplace_back(-1, -1);
            combo->addItem(to_qstr("none"));
         }
         int counter = 0;
         for(auto sensor_no = 0; sensor_no < n_sensors; ++sensor_no) {
            const auto& pose_vec = op_ret.pose(sensor_no);
            for(auto j = 0u; j < pose_vec.size(); ++j) {
               combo_p2d_select_lookup.emplace_back(sensor_no, j); // save
               combo->addItem(to_qstr(format("{:02d}-{:02d}", sensor_no, j)));
            }
         }
         combo->setCurrentIndex(0);

         changed = true;
      }
      combo->blockSignals(was_blocked);

      if(changed && !was_blocked) { widgets_to_model(); }
   }
};

// --------------------------------------------------------------------- make ui

void This::Pimpl::make_ui() noexcept
{
   auto scene_desc = app_state()->scene_desc();

   // Cpanel | viewer

   // ---- (*) ---- Create widgets
   {
      gl_visualizer = new GLVisualizer{nullptr};
      gl_visualizer->setObjectName("3d Visualizer");
      gl_visualizer->onDrawThunk = [this]() { this->draw_gl(); };
   }

   { // cpanel widgets
      cb_image_colors       = new QCheckBox{};
      cb_draw_roi           = new QCheckBox{};
      cb_draw_floor_image   = new QCheckBox{};
      cb_draw_cams          = new QCheckBox{};
      cb_draw_pose3d        = new QCheckBox{};
      cb_draw_loaded_tracks = new QCheckBox{};
      cb_draw_clip_to_aabb  = new QCheckBox{};
      cb_draw_C_update      = new QCheckBox{};
      cb_skeleton_debug     = new QCheckBox{};
      cb_draw_axis          = new QCheckBox{};
      cb_draw_grid_xy       = new QCheckBox{};
      cb_draw_grid_xz       = new QCheckBox{};
      cb_draw_grid_yz       = new QCheckBox{};
      cb_draw_fine_grid_xy  = new QCheckBox{};
      //
      cb_do_capture       = new QCheckBox{};
      cb_do_capture_movie = new QCheckBox{};
      cb_do_rotate        = new QCheckBox{};
      slider_rotation_speed
          = new LabeledSlider(0.1, 300.0, (300.0 - 0.1) * 10.0 + 1);
      slider_helicopter_theta
          = new LabeledSlider(0.0, 90.0, (90.0 - 0.0) * 10.0 + 1);
      //
      cb_pt_clouds.resize(10);
      for(auto i = 0u; i < cb_pt_clouds.size(); ++i)
         cb_pt_clouds[i] = new QCheckBox{};
      //
      slider_global_dx      = new LabeledSlider{-25.0, 25.0, 601};
      slider_global_dy      = new LabeledSlider{-25.0, 25.0, 601};
      slider_rotation_z     = new LabeledSlider{0.0, 360, 360 * 2, true};
      button_show_transform = new QPushButton{"Print"};

      //
      slider_pan_x = new LabeledSlider{-25.0, 25.0, 601};
      slider_pan_y = new LabeledSlider{-25.0, 25.0, 601};
      slider_pan_z = new LabeledSlider{-3.0, 3.0, 601};

      //
      slider_C_x   = new LabeledSlider{-30.0, 30.0, 1201};
      slider_C_y   = new LabeledSlider{-30.0, 30.0, 1201};
      slider_C_z   = new LabeledSlider{-30.0, 30.0, 1201};
      slider_saa_x = new LabeledSlider{-M_PI, M_PI, 601};
      slider_saa_y = new LabeledSlider{-M_PI, M_PI, 601};
      slider_saa_z = new LabeledSlider{-M_PI, M_PI, 601};

      // Skeleton debug
      combo_p2d_select  = new QComboBox{};
      slider_p2d_select = new LabeledSlider{0.0, 511.0, 512};
      slider_p3d_scale  = new LabeledSlider{0.1, 2.5, 100};
   }

   { // cpanel

      auto make_base_ctrls = [&]() {
         QFormLayout* layout = new QFormLayout{};
         auto make_opt_grid  = [&]() {
            auto layout = new QGridLayout{};

            auto add_to_grid
                = [&](int row, int col, const char* label, QWidget* widget) {
                     layout->addWidget(
                         new_label(label), row, col + 0, Qt::AlignRight);
                     layout->addWidget(widget, row, col + 1, Qt::AlignLeft);
                  };

            add_to_grid(0, 0, "Draw Cams:", cb_draw_cams);
            add_to_grid(1, 0, "Draw Rois:", cb_draw_roi);
            add_to_grid(2, 0, "Draw Floor Image:", cb_draw_floor_image);
            add_to_grid(3, 0, "Clip Point Cloud:", cb_draw_clip_to_aabb);

            add_to_grid(0, 2, "Draw Pose3D:", cb_draw_pose3d);
            add_to_grid(1, 2, "Draw Loaded:", cb_draw_loaded_tracks);
            add_to_grid(2, 2, "Live C Update:", cb_draw_C_update);
            add_to_grid(3, 2, "Skeleton Debug:", cb_skeleton_debug);

            auto wgt = new QWidget{};
            wgt->setLayout(layout);
            return wgt;
         };

         auto make_grid_wgt = [&]() {
            auto layout = new QHBoxLayout{};
            layout->addWidget(cb_draw_grid_xy);
            layout->addWidget(cb_draw_grid_xz);
            layout->addWidget(cb_draw_grid_yz);
            layout->addWidget(cb_draw_fine_grid_xy);
            auto wgt = new QWidget{};
            wgt->setLayout(layout);
            return wgt;
         };

         layout->addRow(make_opt_grid());
         layout->addRow(make_hline());
         layout->addRow("Draw Axis:", cb_draw_axis);
         layout->addRow("Draw [xy, xz, yz; fine]:", make_grid_wgt());
         layout->addRow(make_hline());
         layout->addRow("Rotate:", cb_do_rotate);
         layout->addRow("Rotation Speed:", slider_rotation_speed);
         layout->addRow("Helicopter Theta:", slider_helicopter_theta);
         layout->addRow("Capture One Revolution:", cb_do_capture);
         layout->addRow("Capture Movie", cb_do_capture_movie);
         auto wgt = new QWidget{};
         wgt->setLayout(layout);
         wgt->setObjectName("GL");
         return wgt;
      };

      auto make_gl_ctrls = [&]() {
         auto layout = new QFormLayout{};
         // Point-cloud checkboxes
         auto make_pt_clouds_cb = [&](int start_ind, int N) {
            auto layout = new QHBoxLayout{};
            for(auto i = start_ind; i < start_ind + N; ++i) {
               if(i == start_ind) {
                  layout->addWidget(
                      new QLabel(to_qstr(format("Cam {}", (i + 1)))));
               } else {
                  layout->addWidget(new QLabel(to_qstr(format("{}", (i + 1)))));
               }
               layout->addWidget(cb_pt_clouds[i]);
            }
            auto wgt = new QWidget{};
            wgt->setLayout(layout);
            return wgt;
         };
         layout->addRow(make_pt_clouds_cb(0, 5));
         layout->addRow(make_pt_clouds_cb(5, 5));
         layout->addRow(make_hline());

         // Global transformations
         auto layout_global_transformation_widgets = [&]() {
            auto layout = new QHBoxLayout{};
            layout->addWidget(slider_rotation_z);
            {
               auto layout2 = new QFormLayout{};
               layout2->addRow("dx", slider_global_dx);
               layout2->addRow("dy", slider_global_dy);
               layout2->addWidget(button_show_transform);
               auto wgt2 = new QWidget{};
               wgt2->setLayout(layout2);
               layout->addWidget(wgt2);
            }
            auto wgt = new QWidget{};
            wgt->setLayout(layout);
            return wgt;
         };
         layout->addRow(layout_global_transformation_widgets());
         layout->addRow(make_hline());

         auto make_pan_widget = [&](const char* label, QWidget* slider) {
            auto layout = new QHBoxLayout{};
            layout->addWidget(new_label(label));
            layout->addWidget(slider);
            auto wgt = new QWidget{};
            wgt->setLayout(layout);
            return wgt;
         };
         layout->addRow(make_pan_widget("Pan-x", slider_pan_x));
         layout->addRow(make_pan_widget("Pan-y", slider_pan_y));
         layout->addRow(make_pan_widget("Pan-z", slider_pan_z));
         layout->addRow(make_hline());

         layout->addRow(make_pan_widget("C.x", slider_C_x));
         layout->addRow(make_pan_widget("C.y", slider_C_y));
         layout->addRow(make_pan_widget("C.z", slider_C_z));
         layout->addRow(make_pan_widget("t.i", slider_saa_x));
         layout->addRow(make_pan_widget("t.a", slider_saa_y));
         layout->addRow(make_pan_widget("t.r", slider_saa_z));
         layout->addRow(make_vertical_expander());
         wgt_gl_ctrls = new QWidget{};
         wgt_gl_ctrls->setFixedWidth(app_state()->config().cpanel_width - 8);
         wgt_gl_ctrls->setStyleSheet("background-color: white;");
         wgt_gl_ctrls->setLayout(layout);
         wgt_gl_ctrls->setObjectName("View");
         return wgt_gl_ctrls;
      };

      auto make_skeleton_debug_ctrls = [&]() {
         auto layout = new QFormLayout{};

         layout->addRow("Skeleton2D:", combo_p2d_select);
         layout->addRow("Select:", slider_p2d_select);
         layout->addRow("p3d size:", slider_p3d_scale);
         layout->addRow(make_vertical_expander());

         wgt_skeleton_debug_ctrls = new QWidget{}; //
         wgt_skeleton_debug_ctrls->setFixedWidth(
             app_state()->config().cpanel_width - 8);
         wgt_skeleton_debug_ctrls->setStyleSheet("background-color: white;");
         wgt_skeleton_debug_ctrls->setLayout(layout);
         wgt_skeleton_debug_ctrls->setObjectName("Skeleton");
         return wgt_skeleton_debug_ctrls;
      };

      auto tab_widget = new QTabWidget;
      auto add_it     = [&](auto w) { tab_widget->addTab(w, w->objectName()); };
      add_it(make_base_ctrls());
      add_it(make_gl_ctrls());
      add_it(make_skeleton_debug_ctrls());

      QFormLayout* layout = new QFormLayout{};
      layout->addRow(tab_widget);

      // Finish widget
      cpanel = new QWidget{};
      cpanel->setFixedWidth(app_state()->config().cpanel_width);
      cpanel->setStyleSheet("background-color: white;");
      cpanel->setLayout(layout);
   }

   QHBoxLayout* layout = new QHBoxLayout{};
   layout->addWidget(cpanel);
   layout->addWidget(gl_visualizer);
   parent->setLayout(layout);

   // ---- (*) ---- Initialize
   update_widgets();
   model_to_widgets();
   widgets_to_model();
   set_cpanel_parts_visibility();

   // ---- (*) ---- Wiring
   connect(parent,
           SIGNAL(data_updated()),
           parent,
           SLOT(on_redraw()),
           Qt::QueuedConnection);

   connect(parent,
           SIGNAL(data_updated()),
           parent,
           SLOT(update_widgets()),
           Qt::QueuedConnection);

   connect(parent,
           SIGNAL(data_updated()),
           parent,
           SLOT(update_aux_skeleton()),
           Qt::QueuedConnection);

   connect(gl_visualizer,
           SIGNAL(oneRevolutionCaptured()),
           parent,
           SLOT(handle_one_revolution_captured()));

   connect(gl_visualizer,
           SIGNAL(oneFrameCaptured()),
           parent,
           SLOT(handle_one_frame_captured()));

   connect(button_show_transform,
           SIGNAL(pressed()),
           parent,
           SLOT(print_global_transform()));

#define CONNECT_TEXT(source, slot)                                         \
   {                                                                       \
      connect(source, SIGNAL(textChanged(QString)), parent, SLOT(slot())); \
   }

#define CONNECT_CB(source, slot)                                        \
   {                                                                    \
      connect(source, SIGNAL(stateChanged(int)), parent, SLOT(slot())); \
   }

#define CONNECT_SLIDER(source, slot)                                        \
   {                                                                        \
      connect(source, SIGNAL(value_changed(double)), parent, SLOT(slot())); \
   }

#define CONNECT_SLIDER(source, slot)                                        \
   {                                                                        \
      connect(source, SIGNAL(value_changed(double)), parent, SLOT(slot())); \
   }

   CONNECT_CB(cb_image_colors, widgets_to_model);
   CONNECT_CB(cb_draw_cams, widgets_to_model);
   CONNECT_CB(cb_draw_pose3d, widgets_to_model);
   CONNECT_CB(cb_draw_loaded_tracks, widgets_to_model);
   CONNECT_CB(cb_draw_roi, widgets_to_model);
   CONNECT_CB(cb_draw_floor_image, widgets_to_model);
   CONNECT_CB(cb_draw_clip_to_aabb, widgets_to_model);
   CONNECT_CB(cb_draw_C_update, widgets_to_model);
   CONNECT_CB(cb_skeleton_debug, widgets_to_model);
   CONNECT_CB(cb_skeleton_debug, set_cpanel_parts_visibility);
   CONNECT_CB(cb_draw_axis, widgets_to_model);
   CONNECT_CB(cb_draw_grid_xy, widgets_to_model);
   CONNECT_CB(cb_draw_grid_xz, widgets_to_model);
   CONNECT_CB(cb_draw_grid_yz, widgets_to_model);
   CONNECT_CB(cb_draw_fine_grid_xy, widgets_to_model);
   CONNECT_CB(cb_do_capture, widgets_to_model);
   CONNECT_CB(cb_do_capture_movie, widgets_to_model);
   CONNECT_CB(cb_do_rotate, widgets_to_model);
   CONNECT_SLIDER(slider_rotation_speed, widgets_to_model);
   CONNECT_SLIDER(slider_helicopter_theta, widgets_to_model);
   for(auto cb : cb_pt_clouds) CONNECT_CB(cb, widgets_to_model);
   CONNECT_SLIDER(slider_global_dx, widgets_to_model);
   CONNECT_SLIDER(slider_global_dy, widgets_to_model);
   CONNECT_SLIDER(slider_rotation_z, widgets_to_model);
   CONNECT_SLIDER(slider_pan_x, widgets_to_model);
   CONNECT_SLIDER(slider_pan_y, widgets_to_model);
   CONNECT_SLIDER(slider_pan_z, widgets_to_model);
   CONNECT_SLIDER(slider_C_x, widgets_to_model);
   CONNECT_SLIDER(slider_C_y, widgets_to_model);
   CONNECT_SLIDER(slider_C_z, widgets_to_model);
   CONNECT_SLIDER(slider_saa_x, widgets_to_model);
   CONNECT_SLIDER(slider_saa_y, widgets_to_model);
   CONNECT_SLIDER(slider_saa_z, widgets_to_model);

   CONNECT_SLIDER(slider_p2d_select, widgets_to_model);
   CONNECT_SLIDER(slider_p3d_scale, widgets_to_model);
   connect(combo_p2d_select,
           SIGNAL(currentIndexChanged(int)),
           parent,
           SLOT(widgets_to_model()));

#undef CONNECT_TEXT
#undef CONNECT_CB
#undef CONNECT_SLIDER
}

// ---------------------------------------------------------------- Construction

This::This(QWidget* parent)
    : QWidget(parent)
    , pimpl_(new Pimpl(this))
{
   pimpl_->make_ui();
}

This::~This() { delete pimpl_; }

// --------------------------------------------------------- get/set render-opts

void This::set_render_opts(const GuiRenderOptions& opts)
{
   pimpl_->opts = opts;
   model_to_widgets();
}

const GuiRenderOptions& This::render_opts() const noexcept
{
   return pimpl_->opts;
}

// ------------------------------------------------------------------- on-redraw

void This::on_redraw()
{
   auto& P = *pimpl_;

   set_cpanel_parts_visibility();

   auto task    = P.get_task_ptr(*app_state());
   auto op_task = P.get_op_task_ptr(*app_state());
   if(task == nullptr) { return; }
   if(op_task == nullptr) { return; }

   // Poke the task to get a result (threaded)
   schedule([task, this]() {
      task->result([this](auto x) { this->pimpl_->set_result(x); });
   });

   schedule([op_task, this]() {
      op_task->result([this](auto x) { this->pimpl_->set_op_result(x); });
   });
}

// ------------------------------------------------------- model to/from widgets

void This::model_to_widgets() noexcept { pimpl_->model_to_widgets(); }

void This::widgets_to_model() noexcept { pimpl_->widgets_to_model(); }

void This::set_cpanel_parts_visibility() noexcept
{
   pimpl_->set_cpanel_parts_visibility();
}

void This::update_widgets() noexcept { pimpl_->update_widgets(); }

void This::handle_one_revolution_captured() noexcept
{
   pimpl_->cb_do_capture->setChecked(false); // Triggers widgets to model
}

void This::handle_one_frame_captured() noexcept
{
   auto& P = *pimpl_;
   if(!P.in_movie_capture_op) return;

   auto app = app_state();
   if(!app) return;

   const auto n_frames = app->n_frames();
   const auto frame_no = app->frame_no();
   if(frame_no + 1 == n_frames or !P.in_movie_capture_op) {
      P.cb_do_capture_movie->setChecked(false);
      return; // we're done.
   }

   P.gl_visualizer->setCapture(false);
   P.capture_ready_flag = false; // we're ready for another please!
   app->seek_frame(frame_no + 1);
}

void This::print_global_transform() noexcept
{
   auto scene_desc = app_state()->scene_desc();

   if(!scene_desc) {
      WARN(format("Cowardly refusing to print anything, because the scene has "
                  "not yet been loaded"));
   } else {
      INFO(format("Global Transformation is:"));
      const auto et = scene_desc->scene_info.global_transformation
                      * pimpl_->opts.make_global_et();
      cout << et.to_json_str() << endl;

      // Apply the transformation to each camera...
      const auto n_cams = scene_desc->scene_info.bcam_transforms.size();
      Expects(n_cams == scene_desc->scene_info.bcam_keys.size());
      for(auto i = 0u; i < n_cams; ++i) {
         const auto& e0  = scene_desc->scene_info.bcam_transforms[i];
         const auto& key = scene_desc->scene_info.bcam_keys[i];
         cout << json_encode(key) << " " << (e0 * et).to_json_str() << endl;
      }
      cout << "." << endl;
   }
}

void This::update_aux_skeleton() noexcept
{
   //
}
