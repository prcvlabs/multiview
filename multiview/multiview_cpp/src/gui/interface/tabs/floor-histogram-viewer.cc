
#include "stdinc.hpp"

#include "floor-histogram-viewer.hh"

#include "gui/app-state.hh"
#include "gui/interface/interface-helpers.hpp"
#include "gui/qt-helpers.hpp"
#include "gui/widgets/image-viewer-2.hh"
#include "gui/widgets/labeled-slider.hh"

#include "perceive/movie/movie-utils.hpp"
#include "perceive/movie/render-tracks.hpp"

#define This FloorHistogramViewer

using namespace perceive;

// ----------------------------------------------------------------------- pimpl

struct This::Pimpl
{
   This* parent{nullptr};
   QWidget* cpanel{nullptr};
   ImageViewer2* image_viewer{nullptr};
   shared_ptr<QImage> qim_ptr{nullptr};

   //@{ cpanel widgets
   QCheckBox* cb_view_roi{nullptr};
   QCheckBox* cb_view_grid{nullptr};
   QCheckBox* cb_op_primacy{nullptr};
   QRadioButton* rb_frame{nullptr};
   QRadioButton* rb_motion{nullptr};
   QRadioButton* rb_prob_bg{nullptr};
   QRadioButton* rb_prob_fg_openpose{nullptr};
   QRadioButton* rb_prob_fg_3d{nullptr};
   QRadioButton* rb_prob_fg{nullptr};
   QRadioButton* rb_op_labels{nullptr};
   QRadioButton* rb_heights{nullptr};
   QRadioButton* rb_localization{nullptr};
   QWidget* panel_frame{nullptr};
   QWidget* panel_motion{nullptr};
   QWidget* panel_fg_openpose{nullptr};
   QWidget* panel_bgfg_3d{nullptr};
   QWidget* panel_fg{nullptr};
   QWidget* panel_localization{nullptr};

   QPushButton* btn_regen_movie_stats{nullptr};

   // model-widgets
   // Motion
   LabeledSlider* slider_use_median{nullptr};
   LabeledSlider* slider_n_deviations{nullptr};

   LabeledSlider* slider_hist_sz{nullptr};
   LabeledSlider* slider_min_z{nullptr};
   LabeledSlider* slider_max_z{nullptr};
   LabeledSlider* slider_min_count{nullptr};
   LabeledSlider* slider_max_count{nullptr};
   LabeledSlider* slider_gaussian{nullptr};
   LabeledSlider* slider_slic_wgt_norm{nullptr};
   LabeledSlider* slider_slic_still_weights{nullptr};

   LabeledSlider* slider_person_diameter{nullptr};
   LabeledSlider* slider_background_median{nullptr};
   LabeledSlider* slider_background_stddev{nullptr};
   LabeledSlider* slider_track_median{nullptr};
   LabeledSlider* slider_track_stddev{nullptr};
   LabeledSlider* slider_prob_bg_max{nullptr};

   LabeledSlider* slider_localization_min{nullptr};
   LabeledSlider* slider_localization_max{nullptr};
   LabeledSlider* slider_filter_radius{nullptr};

   LabeledSlider* slider_prob_shift{nullptr};
   LabeledSlider* slider_foot_dist_radius_factor{nullptr};
   LabeledSlider* slider_skeleton_disparity_weight{nullptr};
   QCheckBox* cb_apply_smooth_mhist_to_p3ds{nullptr};
   //@}

   pipeline::floor_hist::Params params;
   shared_ptr<const pipeline::floor_hist::Result> floor_hist_result_ptr;
   shared_ptr<const pipeline::movie_stats::Result> movie_stats_result_ptr;
   shared_ptr<const pipeline::localization::Result> localization_result_ptr;

   bool regen_is_running{false};

   Pimpl(This* in_parent)
       : parent(in_parent)
   {
      qim_ptr = make_shared<QImage>();
   }
   void make_ui() noexcept;

   // ---------------------------------------------------- set floor hist result
   void set_floor_hist_result(
       shared_ptr<const pipeline::floor_hist::Result> ptr) noexcept
   {
      if(floor_hist_result_ptr.get() != ptr.get()) {
         floor_hist_result_ptr = ptr;
         emit parent->data_updated();
      }
   }

   // --------------------------------------------------- set movie stats result
   void set_movie_stats_result(
       shared_ptr<const pipeline::movie_stats::Result> ptr) noexcept
   {
      if(movie_stats_result_ptr.get() != ptr.get()) {
         movie_stats_result_ptr = ptr;
         emit parent->data_updated();
      }
   }

   void set_localization_result(
       shared_ptr<const pipeline::localization::Result> ptr) noexcept
   {
      if(localization_result_ptr.get() != ptr.get()) {
         localization_result_ptr = ptr;
         emit parent->data_updated();
      }
   }

   // ---------------------------------------------------------------- htask_ptr
   auto htask_cptr() noexcept
   {
      return !app_state()->frame_results()
                 ? nullptr
                 : &(app_state()->frame_results()->floor_hist);
   }

   auto htask_ptr() noexcept
   {
      return const_cast<pipeline::floor_hist::Task*>(htask_cptr());
   }

   // ---------------------------------------------------------------- mtask_ptr
   auto mtask_cptr() noexcept
   {
      return !app_state()->frame_results()
                 ? nullptr
                 : &(app_state()->frame_results()->movie_stats);
   }

   auto mtask_ptr() noexcept
   {
      return const_cast<pipeline::movie_stats::Task*>(mtask_cptr());
   }

   // ---------------------------------------------------------------- ltask_ptr
   auto ltask_cptr() noexcept
   {
      return !app_state()->frame_results()
                 ? nullptr
                 : &(app_state()->frame_results()->localization);
   }

   auto ltask_ptr() noexcept
   {
      return const_cast<pipeline::localization::Task*>(ltask_cptr());
   }

   // --------------------------------------------------------------- poke tasks
   // Get tasks to recalculate
   auto poke_tasks()
   {
      Expects(app_state()->frame_results());

      auto htask = htask_ptr();
      auto mtask = mtask_ptr();
      auto ltask = ltask_ptr();

      { // floor-hist
         schedule([htask, this]() {
            htask->result([this](auto x) { this->set_floor_hist_result(x); });
         });
      }

      { // movie-stats
         schedule([mtask, this]() {
            mtask->result([this](auto x) { this->set_movie_stats_result(x); });
         });
      }

      { // localization
         schedule([ltask, this]() {
            ltask->result([this](auto x) { this->set_localization_result(x); });
         });
      }

      return std::make_tuple(htask->try_get_casted_result(),
                             mtask->try_get_casted_result(),
                             ltask->try_get_casted_result());
   }

   // --------------------------------------------------------- widgets to model
   bool widgets_to_model(AppState& app)
   {
      auto do_htask = [&]() -> bool {
         auto task = htask_ptr();
         if(task == nullptr) return false;
         auto params = task->params(); // copy
         auto& p     = params.hist_params;

         auto wm = (slider_slic_wgt_norm->value() == 0.0)
                       ? FloorHistogram::WGT_NONE
                       : (slider_slic_wgt_norm->value() == 1.0)
                             ? FloorHistogram::WGT_NORM
                             : FloorHistogram::WGT_QUADRANCE;

         p.hist_sz        = slider_hist_sz->value();
         p.hist_min_z     = slider_min_z->value();
         p.hist_max_z     = slider_max_z->value();
         p.min_hist_count = slider_min_count->value();
         p.max_hist_count = slider_max_count->value();
         p.gaussian_sigma = slider_gaussian->value();
         p.weighting      = wm;
         p.apply_slic_still_weights
             = (0.0 != slider_slic_still_weights->value());

         return task->set_params(params);
      };

      auto do_ltask = [&]() -> bool {
         auto task = ltask_ptr();
         if(task == nullptr) return false;
         auto params = task->params(); // copy
         auto& p     = params.localization_params;

         p.use_median = (0.0 != slider_use_median->value());
#define READ_IT(x) p.x = slider_##x->value()
         READ_IT(n_deviations);
         READ_IT(person_diameter);
         READ_IT(background_median);
         READ_IT(background_stddev);
         READ_IT(track_median);
         READ_IT(track_stddev);
         READ_IT(prob_bg_max);
         p.prefer_openpose_method = cb_op_primacy->isChecked();
         READ_IT(localization_min);
         READ_IT(localization_max);
         READ_IT(filter_radius);
         READ_IT(prob_shift);
         p.p3d_params.foot_dist_radius_factor
             = slider_foot_dist_radius_factor->value();
         p.p3d_params.skeleton_disparity_weight
             = slider_skeleton_disparity_weight->value();
         p.apply_smooth_mhist_to_p3ds
             = cb_apply_smooth_mhist_to_p3ds->isChecked();

#undef READ_IT

         return task->set_params(params);
      };

      bool ret = false;
      if(do_htask()) ret = true;
      if(do_ltask()) ret = true;
      if(ret) { emit parent->on_redraw(); }
      return ret;
   }

   // --------------------------------------------------------- model to widgets
   void model_to_widgets(const AppState& app)
   {
      show_hide_cpanel_parts();

      auto do_htask = [&]() {
         auto task = htask_cptr();
         if(task == nullptr) return;
         const auto& p = task->params().hist_params;
         block_signal_set_slider(slider_hist_sz, p.hist_sz);
         block_signal_set_slider(slider_min_z, p.hist_min_z);
         block_signal_set_slider(slider_max_z, p.hist_max_z);
         block_signal_set_slider(slider_min_count, p.min_hist_count);
         block_signal_set_slider(slider_max_count, p.max_hist_count);
         block_signal_set_slider(slider_gaussian, p.gaussian_sigma);
         block_signal_set_slider(slider_slic_wgt_norm, p.weighting);
         block_signal_set_slider(slider_slic_still_weights,
                                 p.apply_slic_still_weights);
      };

      auto do_ltask = [&]() {
         auto task = ltask_cptr();
         if(task == nullptr) return;
         const auto& p = task->params().localization_params;

#define SET_IT(x) block_signal_set_slider(slider_##x, p.x)
         // Motion
         SET_IT(use_median);
         SET_IT(n_deviations);

         // Background/Foreground 3d
         SET_IT(person_diameter);
         SET_IT(background_median);
         SET_IT(background_stddev);
         SET_IT(track_median);
         SET_IT(track_stddev);
         SET_IT(prob_bg_max);

         // Localization
         block_signal_set_checked(cb_op_primacy, p.prefer_openpose_method);
         SET_IT(localization_min);
         SET_IT(localization_max);
         SET_IT(filter_radius);

         SET_IT(prob_shift);

         block_signal_set_slider(slider_foot_dist_radius_factor,
                                 p.p3d_params.foot_dist_radius_factor);
         block_signal_set_slider(slider_skeleton_disparity_weight,
                                 p.p3d_params.skeleton_disparity_weight);
         block_signal_set_checked(cb_apply_smooth_mhist_to_p3ds,
                                  p.apply_smooth_mhist_to_p3ds);
#undef SET_IT
      };

      do_htask();
      do_ltask();
   }

   // --------------------------------------------------- show-hide cpanel parts
   void show_hide_cpanel_parts()
   {
      panel_frame->setVisible(rb_frame->isChecked());
      panel_motion->setVisible(rb_motion->isChecked());
      panel_localization->setVisible(rb_localization->isChecked());
      panel_bgfg_3d->setVisible(rb_prob_bg->isChecked()
                                or rb_prob_fg_3d->isChecked());
      panel_fg_openpose->setVisible(rb_prob_fg_openpose->isChecked());
      panel_fg->setVisible(rb_prob_fg->isChecked());
   }
};

// --------------------------------------------------------------------- make ui

void This::Pimpl::make_ui() noexcept
{
   // auto scene_desc = app_state()->scene_desc();

   // Cpanel | viewer

   // ---- (*) ---- Create widgets
   {
      // cpanel view widgets
      cb_view_roi           = new QCheckBox("See ROI");
      cb_view_grid          = new QCheckBox("Grid");
      cb_op_primacy         = new QCheckBox("OP Algorithm");
      rb_frame              = new QRadioButton("Frame");
      rb_motion             = new QRadioButton("Motion");
      rb_localization       = new QRadioButton("Localization");
      rb_op_labels          = new QRadioButton("OP Labels");
      rb_heights            = new QRadioButton("Heights");
      rb_prob_bg            = new QRadioButton("Prob(BG)");
      rb_prob_fg_openpose   = new QRadioButton("Prob(OP)");
      rb_prob_fg_3d         = new QRadioButton("Prob(3d)");
      rb_prob_fg            = new QRadioButton("Prob(FG)");
      btn_regen_movie_stats = new QPushButton("Regen Movie Stats");

      cb_view_roi->setChecked(false);
      cb_view_grid->setChecked(true);
      rb_frame->setChecked(true);

      // cpanel widgets
      slider_hist_sz = new LabeledSlider(0.01, 1.00, (1.00 - 0.01) * 100 + 1);
      slider_min_z   = new LabeledSlider(0.00, 5.00, (5.00 - 0.00) * 100 + 1);
      slider_max_z   = new LabeledSlider(0.00, 5.00, (5.00 - 0.00) * 100 + 1);
      slider_min_count
          = new LabeledSlider(0.00, 1000.0, (1000.0 - 0.0) * 1000 + 1);
      slider_max_count
          = new LabeledSlider(0.00, 9999.0, (9999.0 - 0.0) * 1000 + 1);
      slider_gaussian = new LabeledSlider(0.0, 10.0, (10.0 - 0.0) * 100 + 1);
      slider_slic_wgt_norm      = new LabeledSlider(0.0, 2.0, 3);
      slider_slic_still_weights = new LabeledSlider(0.0, 1.0, 2);

      // Motion
      slider_use_median = new LabeledSlider(0.0, 1.0, 2);
      slider_n_deviations
          = new LabeledSlider(0.01, 10.0, (10.0 - 0.01) * 100 + 1);

      // Foreground/Background 3d
      slider_person_diameter   = new LabeledSlider(0.01, 2.0, 200);
      slider_background_median = new LabeledSlider(-200.0, 200.0, 401);
      slider_background_stddev = new LabeledSlider(1.00, 100.0, 100);
      slider_track_median      = new LabeledSlider(0.0, 400.0, 401);
      slider_track_stddev      = new LabeledSlider(1.00, 100.0, 100);
      slider_prob_bg_max       = new LabeledSlider(0.0, 1.0, 101);

      // Localization
      slider_localization_min          = new LabeledSlider(-40.0, 40.0, 801);
      slider_localization_max          = new LabeledSlider(-40.0, 40.0, 801);
      slider_filter_radius             = new LabeledSlider(0.01, 2.00, 200);
      slider_prob_shift                = new LabeledSlider(0.0, 1.0, 101);
      slider_foot_dist_radius_factor   = new LabeledSlider(0.0, 4.0, 401);
      slider_skeleton_disparity_weight = new LabeledSlider(0.0, 4.0, 401);
      cb_apply_smooth_mhist_to_p3ds    = new QCheckBox{};
   }

   { // cpanel
      auto make_cb_widget = [&]() {
         auto layout = new QHBoxLayout{};
         layout->addWidget(make_horizontal_expander(8));
         layout->addWidget(cb_view_roi);
         layout->addWidget(cb_view_grid);
         auto wgt = new QWidget{};
         wgt->setLayout(layout);
         return wgt;
      };

      auto make_rb_widget = [&]() {
         const auto align = Qt::AlignLeft;
         auto layout      = new QGridLayout{};
         layout->setContentsMargins(8, 2, 4, 2);
         layout->addWidget(rb_frame, 0, 0, align);
         layout->addWidget(rb_motion, 0, 1, align);
         layout->addWidget(rb_prob_bg, 0, 2, align);
         layout->addWidget(rb_prob_fg_openpose, 1, 0, align);
         layout->addWidget(rb_prob_fg_3d, 1, 1, align);
         layout->addWidget(rb_prob_fg, 1, 2, align);
         layout->addWidget(rb_heights, 2, 0, align);
         layout->addWidget(rb_op_labels, 2, 1, align);
         layout->addWidget(rb_localization, 2, 2, align);
         auto wgt = new QWidget{};
         wgt->setLayout(layout);
         return wgt;
      };

      auto make_panel_frame = [&]() {
         QFormLayout* layout = new QFormLayout{};
         layout->addRow(new_header_label("Frame"));
         layout->addRow("Hist Size:", slider_hist_sz);
         layout->addRow("Min Z Cutoff:", slider_min_z);
         layout->addRow("Max Z Cutoff:", slider_max_z);
         layout->addRow("Cell Min-Count:", slider_min_count);
         layout->addRow("Cell Max-Count:", slider_max_count);
         layout->addRow("Gaussian Sigma:", slider_gaussian);
         layout->addRow("Weight Method;", slider_slic_wgt_norm);
         layout->addRow("Use Still Weights:", slider_slic_still_weights);
         auto wgt = new QWidget{};
         wgt->setLayout(layout);
         panel_frame = wgt;
         return wgt;
      };

      auto make_motion_frame = [&]() {
         QFormLayout* layout = new QFormLayout{};
         layout->addRow(new_header_label("Motion"));
         layout->addRow("Uses Median:", slider_use_median);
         layout->addRow("N Deviations Spread:", slider_n_deviations);
         auto wgt = new QWidget{};
         wgt->setLayout(layout);
         panel_motion = wgt;
         return wgt;
      };

      auto make_prob_fg_openpose_frame = [&]() {
         QFormLayout* layout = new QFormLayout{};
         layout->addRow(new_header_label("Probability Foreground::Openpose"));
         auto wgt = new QWidget{};
         wgt->setLayout(layout);
         panel_fg_openpose = wgt;
         return wgt;
      };

      auto make_prob_bgfg_3d_frame = [&]() {
         QFormLayout* layout = new QFormLayout{};
         layout->addRow(
             new_header_label("Probability Background/Foreground::3d"));
         layout->addRow("Track Diameter", slider_person_diameter);
         layout->addRow("Background Median", slider_background_median);
         layout->addRow("Background Stddev", slider_background_stddev);
         layout->addRow("Track Median", slider_track_median);
         layout->addRow("Track Stddev", slider_track_stddev);
         layout->addRow("Prob(is-bg) Max", slider_prob_bg_max);
         auto wgt = new QWidget{};
         wgt->setLayout(layout);
         panel_bgfg_3d = wgt;
         return wgt;
      };

      auto make_prob_fg_frame = [&]() {
         QFormLayout* layout = new QFormLayout{};
         layout->addRow(new_header_label("Probability Foreground"));
         auto wgt = new QWidget{};
         wgt->setLayout(layout);
         panel_fg = wgt;
         return wgt;
      };

      auto make_localization_frame = [&]() {
         QFormLayout* layout = new QFormLayout{};
         layout->addRow(new_header_label("Localization"));
         layout->addRow("OP Algorithm:", cb_op_primacy);
         layout->addRow("Foot Dist Factor:", slider_foot_dist_radius_factor);
         layout->addRow("Skeleton Weight:", slider_skeleton_disparity_weight);
         layout->addRow("Apply mhist to p3ds:", cb_apply_smooth_mhist_to_p3ds);
         layout->addRow("OP Neutral:", slider_prob_shift);
         layout->addRow("Min Score:", slider_localization_min);
         layout->addRow("Max Score:", slider_localization_max);
         layout->addRow("Filter Radius:", slider_filter_radius);
         auto wgt = new QWidget{};
         wgt->setLayout(layout);
         panel_localization = wgt;
         return wgt;
      };

      auto layout = new QVBoxLayout{};

      layout->addWidget(make_cb_widget());
      layout->addWidget(make_rb_widget());
      layout->addWidget(make_hline());
      layout->addWidget(make_panel_frame());
      layout->addWidget(make_motion_frame());
      layout->addWidget(make_prob_fg_openpose_frame());
      layout->addWidget(make_prob_bgfg_3d_frame());
      layout->addWidget(make_prob_fg_frame());
      layout->addWidget(make_localization_frame());
      layout->addWidget(make_vertical_expander());
      layout->addWidget(btn_regen_movie_stats);

      cpanel = new QWidget{};
      cpanel->setFixedWidth(app_state()->config().cpanel_width);
      cpanel->setStyleSheet("background-color: white;");
      cpanel->setLayout(layout);
   }

   { // image viewer (for seeing the video)
      image_viewer = new ImageViewer2{};
      image_viewer->setSizePolicy(QSizePolicy::Expanding,
                                  QSizePolicy::Expanding);
      image_viewer->setStyleSheet("background-color: blue;");
   }

   QHBoxLayout* layout = new QHBoxLayout{};
   layout->addWidget(cpanel);
   layout->addWidget(image_viewer);
   parent->setLayout(layout);

   // ---- (*) ---- Initialize
   model_to_widgets(*app_state());

   // ---- (*) ---- Wiring
   connect(app_state(),
           SIGNAL(open_finished()),
           parent,
           SLOT(model_to_widgets()),
           Qt::QueuedConnection);

   connect(parent,
           SIGNAL(data_updated()),
           parent,
           SLOT(on_redraw()),
           Qt::QueuedConnection);

#define CONNECT_TEXT(source, slot)                                         \
   {                                                                       \
      connect(source, SIGNAL(textChanged(QString)), parent, SLOT(slot())); \
   }

#define CONNECT_CB(source, slot)                                        \
   {                                                                    \
      connect(source, SIGNAL(stateChanged(int)), parent, SLOT(slot())); \
   }

#define CONNECT_RB(source, slot)                                    \
   {                                                                \
      connect(source, SIGNAL(toggled(bool)), parent, SLOT(slot())); \
   }

#define CONNECT_SLIDER(source, slot)                                        \
   {                                                                        \
      connect(source, SIGNAL(value_changed(double)), parent, SLOT(slot())); \
   }

   CONNECT_CB(cb_view_roi, on_redraw);
   CONNECT_CB(cb_view_grid, on_redraw);
   CONNECT_CB(cb_op_primacy, widgets_to_model);
   CONNECT_RB(rb_frame, on_rb_clicked);
   CONNECT_RB(rb_motion, on_rb_clicked);
   CONNECT_RB(rb_localization, on_rb_clicked);
   CONNECT_RB(rb_op_labels, on_rb_clicked);
   CONNECT_RB(rb_heights, on_rb_clicked);
   CONNECT_RB(rb_prob_bg, on_rb_clicked);
   CONNECT_RB(rb_prob_fg_openpose, on_rb_clicked);
   CONNECT_RB(rb_prob_fg_3d, on_rb_clicked);
   CONNECT_RB(rb_prob_fg, on_rb_clicked);

   CONNECT_SLIDER(slider_hist_sz, widgets_to_model);
   CONNECT_SLIDER(slider_min_z, widgets_to_model);
   CONNECT_SLIDER(slider_max_z, widgets_to_model);
   CONNECT_SLIDER(slider_min_count, widgets_to_model);
   CONNECT_SLIDER(slider_max_count, widgets_to_model);
   CONNECT_SLIDER(slider_gaussian, widgets_to_model);
   CONNECT_SLIDER(slider_slic_wgt_norm, widgets_to_model);
   CONNECT_SLIDER(slider_slic_still_weights, widgets_to_model);

   // Motion
   CONNECT_SLIDER(slider_use_median, widgets_to_model);
   CONNECT_SLIDER(slider_n_deviations, widgets_to_model);

   // Background/Foreground 3d
   CONNECT_SLIDER(slider_person_diameter, widgets_to_model);
   CONNECT_SLIDER(slider_background_median, widgets_to_model);
   CONNECT_SLIDER(slider_background_stddev, widgets_to_model);
   CONNECT_SLIDER(slider_track_median, widgets_to_model);
   CONNECT_SLIDER(slider_track_stddev, widgets_to_model);
   CONNECT_SLIDER(slider_prob_bg_max, widgets_to_model);

   // Localization
   CONNECT_SLIDER(slider_foot_dist_radius_factor, widgets_to_model);
   CONNECT_SLIDER(slider_skeleton_disparity_weight, widgets_to_model);
   CONNECT_SLIDER(slider_prob_shift, widgets_to_model);
   CONNECT_CB(cb_apply_smooth_mhist_to_p3ds, widgets_to_model);
   CONNECT_SLIDER(slider_localization_min, widgets_to_model);
   CONNECT_SLIDER(slider_localization_max, widgets_to_model);
   CONNECT_SLIDER(slider_filter_radius, widgets_to_model);

   // Regenerate Movie Stats button
   connect(btn_regen_movie_stats,
           SIGNAL(pressed()),
           parent,
           SLOT(on_regen_movie_stats_clicked()));

#undef CONNECT_TEXT
#undef CONNECT_CB
#undef CONNECT_RB
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

// ------------------------------------------------------------------- Functions

void This::set_qimage(const shared_ptr<const QImage> qim)
{
   get_image_viewer()->set_qimage(qim);
}

ImageViewer2* This::get_image_viewer() noexcept { return pimpl_->image_viewer; }

// ------------------------------------------------------------------- on-redraw

void This::on_redraw()
{
   auto& P = *pimpl_;

   if(!app_state()->frame_results()) {
      set_qimage(nullptr);
      return;
   }

   Expects(app_state()->frame_results());
   const auto [hdat, mdat, ldat] = P.poke_tasks();

   if(!ldat) {
      set_qimage(nullptr);
      return;
   }

   const bool see_roi = P.cb_view_roi->isChecked();
   bool see_grid      = P.cb_view_grid->isChecked();

   const bool is_frame        = P.rb_frame->isChecked();
   const bool is_motion       = P.rb_motion->isChecked();
   const bool is_bg           = P.rb_prob_bg->isChecked();
   const bool is_fg_op        = P.rb_prob_fg_openpose->isChecked();
   const bool is_fg_3d        = P.rb_prob_fg_3d->isChecked();
   const bool is_fg           = P.rb_prob_fg->isChecked();
   const bool is_labels       = P.rb_op_labels->isChecked();
   const bool is_heights      = P.rb_heights->isChecked();
   const bool is_localization = P.rb_localization->isChecked();

   const unsigned hist_w = ldat->data.loc_hist.width;
   const unsigned hist_h = ldat->data.loc_hist.height;
   const AABB bounds     = ldat->data.bounds;
   const real hist_sz    = ldat->data.hist_sz;
   const auto mutils     = movie::MovieUtils{hist_w, hist_h, bounds, hist_sz};

   const unsigned w = 512;
   const unsigned h = unsigned(real(hist_h) * real(w) / real(hist_w));

   ARGBImage argb;
   if(is_frame and hdat != nullptr) {
      // Histogram
      argb = hdat->hist.make_image(false);
   } else if(is_motion and ldat != nullptr) {
      // Motion view
      const auto fim = ldat->calc_motion_hist();
      argb           = grey_to_colour(make_hist_image(fim));
   } else if(is_localization and ldat != nullptr) {
      // Localization
      const auto fim = grey16_to_float_im(
          ldat->data.loc_hist, ldat->data.loc_min, ldat->data.loc_max);

      // Colourizes
      cv::Mat small = argb_to_cv(float_im_positive_negative_argb(
          fim, ldat->data.loc_min, ldat->data.loc_max));

      // Stretch
      cv::Mat im;
      cv::resize(small, im, cv::Size(w, h));

      // cv::Mat im = cv::Mat(h, w, CV_8UC3);
      // im.setTo(rgb_to_vec3b(k_white));
      mutils.render_p2ds(im, ldat->data, false, false, false, false);
      if(see_grid) {
         see_grid = false;
         mutils.render_grid(im, bounds);
      }
      argb = cv_to_argb(im);

   } else if(is_labels and ldat != nullptr) {
      cv::Mat im = cv::Mat(h, w, CV_8UC3);
      im.setTo(rgb_to_vec3b(k_white));
      mutils.render_p2ds(im, ldat->data, true, false);
      if(see_grid) {
         see_grid = false;
         mutils.render_grid(im, bounds);
      }
      argb = cv_to_argb(im);
   } else if(is_heights and ldat != nullptr) {
      // OP Heights
      // argb = grey16_im_to_argb(ldat->data.heights);
   } else if(is_bg and ldat != nullptr) {
      // Background view
      const auto fim = ldat->calc_prob_bg();
      argb           = float_im_to_argb(fim, 0.0, 1.0, true);
   } else if(is_fg_op and ldat != nullptr) {
      // Openpose view
      const auto [fim, him, p3ds] = ldat->calc_prob_fg_openpose();
      argb                        = float_im_to_argb(fim, 0.0, 1.0, true);
   } else if(is_fg_3d and ldat != nullptr) {
      // 3d view
      const auto fim = ldat->calc_prob_fg_3d();
      argb           = float_im_to_argb(fim, 0.0, 1.0, true);
   } else if(is_fg and ldat != nullptr) {
      // Foreground view
      const auto fim = ldat->calc_prob_fg();
      argb           = float_im_to_argb(fim, 0.0, 1.0, true);
   } else {
      set_qimage(nullptr);
      return;
   }

   if(see_grid and hdat) {
      const auto& bounds = hdat->hist.bounds;
      const auto hist_sz = hdat->hist.hist_sz;
      draw_gridlines(argb, bounds, hist_sz, k_orange, 0.5);
   }

   { // Draw the entrance zone
      if(hdat != nullptr) {
         const auto scene_desc = app_state()->scene_desc();
         if(see_roi and scene_desc) {
            const auto& zone = scene_desc->scene_info.entrance_zone;
            if(zone.size() > 3)
               draw_entrance_region(
                   argb, zone, hdat->hist, hdat->hist.hist_sz, k_red);
         }
      }
   }

   // Okay, we have a result... let's display it
   to_qimage(argb, *P.qim_ptr);
   set_qimage(P.qim_ptr);
}

bool This::widgets_to_model() { return pimpl_->widgets_to_model(*app_state()); }

void This::model_to_widgets() { pimpl_->model_to_widgets(*app_state()); }

void This::on_rb_clicked()
{
   pimpl_->show_hide_cpanel_parts();
   on_redraw();
}

void This::on_regen_movie_stats_clicked()
{
   auto& P  = *pimpl_;
   auto ptr = app_state()->movie_results();

   auto do_cancel = [pimpl = this->pimpl_]() {
      pimpl->regen_is_running = false; // should cancel
      pimpl->btn_regen_movie_stats->setText("Regen Movie Stats");
   };

   if(P.regen_is_running) {
      do_cancel();
   } else if(ptr != nullptr) {
      P.regen_is_running = true;
      P.btn_regen_movie_stats->setText("Cancel Regeneration");
      ptr->force_update_movie_task_params();
      auto f = [pimpl = pimpl_]() { return !pimpl->regen_is_running; };
      schedule(
          [ptr, f, do_cancel]() { ptr->regenerate_movie_stats(f, do_cancel); });
   }
}
