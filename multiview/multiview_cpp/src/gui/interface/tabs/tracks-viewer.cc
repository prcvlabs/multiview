
#include "stdinc.hpp"

#include <QTabWidget>

#include "tracks-viewer.hh"

#include "gui/app-state.hh"
#include "gui/interface/interface-helpers.hpp"
#include "gui/qt-helpers.hpp"
#include "gui/widgets/image-viewer-2.hh"
#include "gui/widgets/labeled-slider.hh"

#include "perceive/movie/movie-utils.hpp"
#include "perceive/movie/render-tracks.hpp"

#define This TracksViewer

using namespace perceive;

// ----------------------------------------------------------------------- pimpl

using TrackletTask   = perceive::pipeline::tracklet::Task;
using TrackletResult = perceive::pipeline::tracklet::Result;

using TracksTask   = perceive::pipeline::tracks::Task;
using TracksResult = perceive::pipeline::tracks::Result;

struct This::Pimpl
{
   This* parent{nullptr};
   QWidget* cpanel{nullptr};
   ImageViewer2* image_viewer{nullptr};
   shared_ptr<QImage> qim_ptr{nullptr};
   shared_ptr<const TracksResult> tracks_ret_ptr{nullptr};

   QRadioButton* rb_view_localizations{nullptr};
   QRadioButton* rb_view_xyts{nullptr};
   QRadioButton* rb_view_tracks{nullptr};

   QCheckBox* cb_loc_labels{nullptr};

   // Tracklet::Params
   LabeledSlider* slider_max_frames_per_tracklet{nullptr};
   LabeledSlider* slider_frame_t_interp_window{nullptr};
   LabeledSlider* slider_speed_median{nullptr};
   LabeledSlider* slider_speed_stddev{nullptr};
   LabeledSlider* slider_speed_cap{nullptr};
   LabeledSlider* slider_noise_factor{nullptr};
   LabeledSlider* slider_projective_distance_threshold{nullptr};

   // Tracks::Params // false_positive_weight_vector
   LabeledSlider* slider_fixed_window_prev_tracklet{nullptr};
   LabeledSlider* slider_fp_weight_0{nullptr};
   LabeledSlider* slider_fp_weight_1{nullptr};
   LabeledSlider* slider_fp_weight_2{nullptr};
   LabeledSlider* slider_fp_weight_3{nullptr};
   LabeledSlider* slider_hungarian_score_threshold{nullptr};
   LabeledSlider* slider_edge_d0_delta{nullptr};
   LabeledSlider* slider_edge_d0_scale{nullptr};
   LabeledSlider* slider_edge_d1_delta{nullptr};
   LabeledSlider* slider_edge_d1_scale{nullptr};
   LabeledSlider* slider_merge_threshold{nullptr};

   QPushButton* btn_generate_all_tracks{nullptr};

   int calc_idx                                         = -1;
   pipeline::tracks::Result::CompDataEnvelope comp_data = {};

   bool trackgen_is_running = false;

   Pimpl(This* in_parent)
       : parent(in_parent)
   {
      qim_ptr = make_shared<QImage>();
   }

   void make_ui() noexcept;

   // ------------------------------------------------------------ get-comp-data
   //
   const perceive::tracks::ComputationData*
   get_comp_data(const pipeline::tracks::Result* tracks_ret, int frame_no)
   {
      const auto idx = tracks_ret->frame_no_to_track_object_idx(frame_no);
      if(idx != calc_idx) {
         calc_idx  = idx;
         comp_data = tracks_ret->calc_comp_data(calc_idx);
      }
      return comp_data.data_ptr.get();
   }

   // ------------------------------------------------------------------ setters
   //
   void set_qimage(const shared_ptr<const QImage> qim)
   {
      image_viewer->set_qimage(qim);
   }

   // -------------------------------------------------------- tracklet_task_ptr
   //
   TrackletTask* tracklet_task_ptr(AppState& app)
   {
      return const_cast<TrackletTask*>(tracklet_task_cptr(app));
   }

   const TrackletTask* tracklet_task_cptr(const AppState& app)
   {
      if(!app.movie_results()) return nullptr;
      const auto& movie_ret = *app.movie_results();
      return &(app.movie_results()->tracklet_task);
   }

   // ---------------------------------------------------------- tracks_task_ptr
   //
   TracksTask* tracks_task_ptr(AppState& app)
   {
      return const_cast<TracksTask*>(tracks_task_cptr(app));
   }

   const TracksTask* tracks_task_cptr(const AppState& app)
   {
      if(!app.movie_results()) return nullptr;
      const auto& movie_ret = *app.movie_results();
      return &(app.movie_results()->tracks_task);
   }

   void set_tracks_result(shared_ptr<const TracksResult> ptr) noexcept
   {
      if(tracks_ret_ptr.get() != ptr.get()) {
         tracks_ret_ptr = ptr;
         emit parent->data_updated();
      }
   }

   // --------------------------------------------------------------- poke tasks
   // Get tasks to recalculate
   shared_ptr<const TracksResult> poke_tasks()
   {
      Expects(app_state());
      auto task_ptr = tracks_task_ptr(*app_state());
      if(!task_ptr) return nullptr;
      schedule([task_ptr, this]() {
         task_ptr->result([this](auto x) { this->set_tracks_result(x); });
      });
      return task_ptr->try_get_casted_result();
   }

   // ---------------------------------------------------- widgets to/from model
   //
   bool widgets_to_model()
   {
      AppState& app = *app_state();
      bool ret      = false;
      {
         {
            auto task_ptr = tracklet_task_ptr(app);
            if(task_ptr == nullptr) return false;
            pipeline::tracklet::Params p = task_ptr->params(); // copy
#define SET_IT(x) p.tracklet_params.x = slider_##x->value()
            p.tracklet_params.max_frames_per_tracklet
                = slider_max_frames_per_tracklet->value();
            SET_IT(max_frames_per_tracklet);
            SET_IT(frame_t_interp_window);
            SET_IT(speed_median);
            SET_IT(speed_stddev);
            SET_IT(speed_cap);
            SET_IT(noise_factor);
            SET_IT(projective_distance_threshold);
            if(task_ptr->set_params(p)) ret = true;
#undef SET_IT
         }

         {
            auto task_ptr              = tracks_task_ptr(app);
            pipeline::tracks::Params p = task_ptr->params(); // copy
#define SET_IT(x) p.tracks_params.x = slider_##x->value()
            SET_IT(fixed_window_prev_tracklet);
            SET_IT(hungarian_score_threshold);
            p.tracks_params.false_positive_weight_vector[0]
                = slider_fp_weight_0->value();
            p.tracks_params.false_positive_weight_vector[1]
                = slider_fp_weight_1->value();
            p.tracks_params.false_positive_weight_vector[2]
                = slider_fp_weight_2->value();
            p.tracks_params.false_positive_weight_vector[3]
                = slider_fp_weight_3->value();
            SET_IT(edge_d0_delta);
            SET_IT(edge_d0_scale);
            SET_IT(edge_d1_delta);
            SET_IT(edge_d1_scale);
            SET_IT(merge_threshold);
            if(task_ptr->set_params(p)) ret = true;
#undef SET_IT
         }
      }
      if(ret) { emit parent->on_redraw(); }
      return ret;
   }

   void model_to_widgets()
   {
      const AppState& app = *app_state();

      {
         auto task_ptr = tracklet_task_cptr(app);
         if(task_ptr != nullptr) {
            const auto& p = task_ptr->params();
#define GET_IT(x) block_signal_set_slider(slider_##x, p.tracklet_params.x)
            block_signal_set_slider(slider_max_frames_per_tracklet,
                                    p.tracklet_params.max_frames_per_tracklet);
            GET_IT(max_frames_per_tracklet);
            GET_IT(frame_t_interp_window);
            GET_IT(speed_median);
            GET_IT(speed_stddev);
            GET_IT(speed_cap);
            GET_IT(noise_factor);
            GET_IT(projective_distance_threshold);
#undef GET_IT
         }
      }

      {
         auto task_ptr = tracks_task_cptr(app);
         if(task_ptr != nullptr) {
            const auto& p = task_ptr->params();
#define GET_IT(x) block_signal_set_slider(slider_##x, p.tracks_params.x)
            GET_IT(fixed_window_prev_tracklet);
            GET_IT(hungarian_score_threshold);
            block_signal_set_slider(
                slider_fp_weight_0,
                p.tracks_params.false_positive_weight_vector[0]);
            block_signal_set_slider(
                slider_fp_weight_1,
                p.tracks_params.false_positive_weight_vector[1]);
            block_signal_set_slider(
                slider_fp_weight_2,
                p.tracks_params.false_positive_weight_vector[2]);
            block_signal_set_slider(
                slider_fp_weight_3,
                p.tracks_params.false_positive_weight_vector[3]);
            GET_IT(edge_d0_delta);
            GET_IT(edge_d0_scale);
            GET_IT(edge_d1_delta);
            GET_IT(edge_d1_scale);
            GET_IT(merge_threshold);
#undef GET_IT
         }
      }
   }
};

// --------------------------------------------------------------------- make ui

void This::Pimpl::make_ui() noexcept
{
   auto scene_desc = app_state()->scene_desc();

   // ---- (*) ---- Create widgets
   { // cpanel widgets
      rb_view_localizations = new QRadioButton("Localizations");
      rb_view_xyts          = new QRadioButton("XYTs");
      rb_view_tracks        = new QRadioButton("Tracks");

      cb_loc_labels = new QCheckBox("Loc Labels");

      // Tracklets
      slider_max_frames_per_tracklet       = new LabeledSlider(1.0, 100.0, 100);
      slider_frame_t_interp_window         = new LabeledSlider(1.0, 20.0, 20);
      slider_speed_median                  = new LabeledSlider(0.5, 4.0, 36);
      slider_speed_stddev                  = new LabeledSlider(0.5, 4.0, 36);
      slider_speed_cap                     = new LabeledSlider(0.5, 8.0, 76);
      slider_noise_factor                  = new LabeledSlider(0.0, 1.0, 21);
      slider_projective_distance_threshold = new LabeledSlider(0.0, 120.0, 121);

      // Tracks
      slider_fixed_window_prev_tracklet = new LabeledSlider(0.0, 20.0, 21);
      slider_fp_weight_0                = new LabeledSlider(0.0, 1.0, 101);
      slider_fp_weight_1                = new LabeledSlider(0.0, 1.0, 101);
      slider_fp_weight_2                = new LabeledSlider(0.0, 1.0, 101);
      slider_fp_weight_3                = new LabeledSlider(0.0, 1.0, 101);
      slider_hungarian_score_threshold  = new LabeledSlider(0.0, 1.0, 101);
      slider_edge_d0_delta              = new LabeledSlider(-2.0, 1.0, 101);
      slider_edge_d0_scale              = new LabeledSlider(-10.0, 10.0, 101);
      slider_edge_d1_delta              = new LabeledSlider(-2.0, 1.0, 101);
      slider_edge_d1_scale              = new LabeledSlider(-10.0, 10.0, 101);
      slider_merge_threshold            = new LabeledSlider(-2.0, 2.0, 201);

      btn_generate_all_tracks = new QPushButton("Generate All Tracklets");

      rb_view_tracks->setChecked(true);
   }

   { // cpanel
      auto make_rb_panel = [&]() {
         auto layout = new QHBoxLayout{};
         layout->addWidget(rb_view_localizations);
         layout->addWidget(rb_view_xyts);
         layout->addWidget(rb_view_tracks);
         auto wgt = new QWidget{};
         wgt->setLayout(layout);
         return wgt;
      };

      auto make_tracklet_edit_wgt = [&]() {
         QFormLayout* layout = new QFormLayout{};
         layout->addRow(new_header_label("Tracklet Params"));
         layout->addRow("Max Frames per Tracklet",
                        slider_max_frames_per_tracklet);
         layout->addRow("Interp Window", slider_frame_t_interp_window);
         layout->addRow("Speed Median", slider_speed_median);
         layout->addRow("Speed Stddev", slider_speed_stddev);
         layout->addRow("Speed Cap", slider_speed_cap);
         layout->addRow("Noise Factor", slider_noise_factor);
         layout->addRow("Proj Distance Thres.",
                        slider_projective_distance_threshold);
         auto wgt = new QWidget{};
         wgt->setLayout(layout);
         wgt->setObjectName("Tracklets");
         return wgt;
      };

      auto make_track_edit_wgt = [&]() {
         QFormLayout* layout = new QFormLayout{};
         layout->addRow(new_header_label("Tracks Params"));
         layout->addRow("Fixed Window", slider_fixed_window_prev_tracklet);
         layout->addRow("p(fp).p2d-score", slider_fp_weight_0);
         layout->addRow("p(fp).cloud in height", slider_fp_weight_1);
         layout->addRow("p(fp).inbounds height", slider_fp_weight_2);
         layout->addRow("p(fp).p2d-shape", slider_fp_weight_3);
         layout->addRow("Hungarian Thres.", slider_hungarian_score_threshold);
         layout->addRow("Edge delta d0", slider_edge_d0_delta);
         layout->addRow("Edge scale d0", slider_edge_d0_scale);
         layout->addRow("Edge delta d1", slider_edge_d1_delta);
         layout->addRow("Edge scale d1", slider_edge_d1_scale);
         layout->addRow("Merge Threshold", slider_merge_threshold);
         auto wgt = new QWidget{};
         wgt->setLayout(layout);
         wgt->setObjectName("Tracks");
         return wgt;
      };

      auto make_controls_wgt = [&]() {
         QFormLayout* layout = new QFormLayout{};

         layout->addRow(make_rb_panel());
         layout->addRow(cb_loc_labels);

         auto tab_widget = new QTabWidget;
         auto add_it = [&](auto w) { tab_widget->addTab(w, w->objectName()); };
         add_it(make_tracklet_edit_wgt());
         add_it(make_track_edit_wgt());
         layout->addRow(tab_widget);

         auto wgt = new QWidget{};
         wgt->setLayout(layout);
         return wgt;
      };

      QVBoxLayout* layout = new QVBoxLayout{};
      layout->addWidget(make_controls_wgt());
      layout->addWidget(make_vertical_expander());
      layout->addWidget(btn_generate_all_tracks);

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
   model_to_widgets();

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

   connect(btn_generate_all_tracks,
           SIGNAL(pressed()),
           parent,
           SLOT(on_generate_all_tracks()));

#define CONNECT_SLIDER(source, slot)                                        \
   {                                                                        \
      connect(source, SIGNAL(value_changed(double)), parent, SLOT(slot())); \
   }
#define CONNECT_RB(source, slot)                                    \
   {                                                                \
      connect(source, SIGNAL(toggled(bool)), parent, SLOT(slot())); \
   }
#define CONNECT_CB(source, slot)                                        \
   {                                                                    \
      connect(source, SIGNAL(stateChanged(int)), parent, SLOT(slot())); \
   }

   CONNECT_RB(rb_view_localizations, on_rb_clicked);
   CONNECT_RB(rb_view_xyts, on_rb_clicked);
   CONNECT_RB(rb_view_tracks, on_rb_clicked);

   CONNECT_CB(cb_loc_labels, on_redraw);

   CONNECT_SLIDER(slider_max_frames_per_tracklet, widgets_to_model);
   CONNECT_SLIDER(slider_frame_t_interp_window, widgets_to_model);
   CONNECT_SLIDER(slider_speed_median, widgets_to_model);
   CONNECT_SLIDER(slider_speed_stddev, widgets_to_model);
   CONNECT_SLIDER(slider_speed_cap, widgets_to_model);
   CONNECT_SLIDER(slider_noise_factor, widgets_to_model);
   CONNECT_SLIDER(slider_projective_distance_threshold, widgets_to_model);

   CONNECT_SLIDER(slider_fixed_window_prev_tracklet, widgets_to_model);
   CONNECT_SLIDER(slider_fp_weight_0, widgets_to_model);
   CONNECT_SLIDER(slider_fp_weight_1, widgets_to_model);
   CONNECT_SLIDER(slider_fp_weight_2, widgets_to_model);
   CONNECT_SLIDER(slider_fp_weight_3, widgets_to_model);
   CONNECT_SLIDER(slider_hungarian_score_threshold, widgets_to_model);
   CONNECT_SLIDER(slider_edge_d0_delta, widgets_to_model);
   CONNECT_SLIDER(slider_edge_d0_scale, widgets_to_model);
   CONNECT_SLIDER(slider_edge_d1_delta, widgets_to_model);
   CONNECT_SLIDER(slider_edge_d1_scale, widgets_to_model);
   CONNECT_SLIDER(slider_merge_threshold, widgets_to_model);

#undef CONNECT_SLIDER
#undef CONNECT_RB
#undef CONNECT_CB
}

// ---------------------------------------------------------------- Construction

This::This(QWidget* parent)
    : QWidget(parent)
    , pimpl_(new Pimpl(this))
{
   pimpl_->make_ui();
}

This::~This() { delete pimpl_; }

// ------------------------------------------------------------------- on-redraw

void This::on_redraw()
{
   auto& P = *pimpl_;

   auto set_qimage_to_null = [&]() { P.set_qimage(nullptr); };

   auto app_ptr = app_state();
   if(app_ptr == nullptr) return set_qimage_to_null();

   // Get the latest results... (thread-safe)... will be nullptr if
   // we're currently calculating
   auto tracks_dat = P.poke_tasks();

   if(!tracks_dat) return set_qimage_to_null();

   const bool view_localizations = P.rb_view_localizations->isChecked();
   const bool view_xyts          = P.rb_view_xyts->isChecked();
   const bool view_tracks        = P.rb_view_tracks->isChecked();

   const bool loc_labels = P.cb_loc_labels->isChecked();

   const auto frame_no = app_ptr->frame_no();
   const auto localization_ptr
       = tracks_dat->movie_ret().localization_result(frame_no);
   const auto tracklet_ptr = tracks_dat->tracklets().tracklets(frame_no);
   if(localization_ptr == nullptr) return set_qimage_to_null();
   if(tracklet_ptr == nullptr) return set_qimage_to_null();

   const unsigned hist_w = localization_ptr->loc_hist.width;
   const unsigned hist_h = localization_ptr->loc_hist.height;
   const AABB bounds     = localization_ptr->bounds;
   const real hist_sz    = localization_ptr->hist_sz;
   const auto mutils     = movie::MovieUtils{hist_w, hist_h, bounds, hist_sz};

   const unsigned w = 800;
   const unsigned h = unsigned(real(hist_h) * real(w) / real(hist_w));

   cv::Mat im = cv::Mat(h, w, CV_8UC3);
   im.setTo(rgb_to_vec3b(k_white));

   if(view_localizations) {
      mutils.render_p2ds(im, *localization_ptr, !loc_labels, loc_labels);
   } else if(view_xyts) {
      const auto tracks_ptr = tracks_dat->tracks(frame_no);
      auto comp_ptr         = P.get_comp_data(tracks_dat.get(), frame_no);
      if(comp_ptr == nullptr) return set_qimage_to_null();
      Expects(comp_ptr->start_frame <= frame_no);
      Expects(comp_ptr->end_frame >= frame_no);
      Expects(unsigned(frame_no - comp_ptr->start_frame)
              < comp_ptr->frame_dat.size());
      const auto& frame = comp_ptr->frame_dat[frame_no - comp_ptr->start_frame];

      int counter = 0;
      for(const auto& xyt : frame.nodes)
         mutils.render_XYT(im, xyt.x(), xyt.y(), xyt.gaze_theta(), counter++);
   } else if(view_tracks) {
      const auto tracks_ptr = tracks_dat->tracks(frame_no);
      if(tracks_ptr == nullptr) return set_qimage_to_null();
      mutils.render_track(im, frame_no, tracks_ptr->seqs);
   } else {
      FATAL("kBAM!");
   }

   { // Draw the frame number
      const string label = format("{}", frame_no);
      const Point2 dims  = render_tiny_dimensions(label);
      const Point2 pos   = {int(w) - dims.x - 3, int(h) - dims.y - 2};
      render_string(im, label, pos, k_black, k_white);
   }
   mutils.render_grid(im, bounds);

   ARGBImage argb = cv_to_argb(im);

   // Okay, we have a result... let's display it
   to_qimage(argb, *P.qim_ptr);
   P.set_qimage(P.qim_ptr);
}

bool This::widgets_to_model() { return pimpl_->widgets_to_model(); }

void This::model_to_widgets() { pimpl_->model_to_widgets(); }

void This::on_rb_clicked() { on_redraw(); }

void This::on_generate_all_tracks()
{
   auto& P       = *pimpl_;
   auto task_ptr = P.tracklet_task_ptr(*app_state());

   auto do_cancel = [pimpl = this->pimpl_]() {
      pimpl->trackgen_is_running = false; // should cancel
      pimpl->btn_generate_all_tracks->setText("Generate All Tracklets");
   };

   if(P.trackgen_is_running) {
      do_cancel();
   } else if(task_ptr != nullptr) {
      P.trackgen_is_running = true;
      P.btn_generate_all_tracks->setText("Cancel Tracklet Generation");
      auto ret = task_ptr->get_result_synchronized();
      schedule([ret]() {
         WARN(format("functionality has been disabled"));
#ifdef DO_NOT_BUILD_THIS
         const auto n_frames = ret->n_frames();
         for(auto i = 0; i < n_frames and pimpl->trackgen_is_running; ++i)
            ret->tracklets(i);

         // Save all localization data to disk
         if(pimpl->trackgen_is_running) {
            const auto& outdir = app_state()->config().config.outdir;
            auto fname = format("{:s}/localization-and-tracklets.data", outdir);
            INFO(format("saving localization and tracklet data to '{:s}'",
                        fname));
            auto tracklet_ret = app_state()
                                    ->movie_results()
                                    ->tracklet_task.get_result_synchronized();
            tracklet_ret->save_localization_and_tracklet_data(fname);
            INFO(format("DONE!"));
            for(auto i = 0; i < tracklet_ret->n_tracklet_objects(); ++i)
               cout << tracklet_ret->tracklets(i)->to_string() << endl;
         }

         do_cancel();
#endif
      });
   }
}
