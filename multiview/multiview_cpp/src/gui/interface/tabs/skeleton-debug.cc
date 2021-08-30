

#include "stdinc.hpp"

#include "skeleton-debug.hh"

#include "gui/app-state.hh"
#include "gui/qt-helpers.hpp"
#include "gui/widgets/image-viewer-2.hh"
#include "gui/widgets/labeled-slider.hh"

#define This SkeletonDebug

using namespace perceive;

// ----------------------------------------------------------------------- pimpl

struct This::Pimpl
{
   This* parent{nullptr};
   QWidget* cpanel{nullptr};
   ImageViewer2* image_viewer{nullptr};
   shared_ptr<QImage> qim_ptr{nullptr};

   //@{ cpanel widgets
   QComboBox* combo_disp_method{nullptr};
   LabeledSlider* slider_presmooth_sigma{nullptr};

   QWidget* cpanel_bm_wgt{nullptr};
   LabeledSlider* slider_SAD_window_size{nullptr};
   LabeledSlider* slider_12_max_diff{nullptr};
   LabeledSlider* slider_min_disparity{nullptr};
   LabeledSlider* slider_num_disparities{nullptr};
   LabeledSlider* slider_spek_window_size{nullptr};
   LabeledSlider* slider_spek_range{nullptr};
   LabeledSlider* slider_prefilter_cap{nullptr};
   LabeledSlider* slider_prefilter_size{nullptr};
   LabeledSlider* slider_prefilter_type{nullptr};
   LabeledSlider* slider_block_size{nullptr};
   LabeledSlider* slider_threshold{nullptr};
   LabeledSlider* slider_unique_ratio{nullptr};

   QWidget* cpanel_sgm_wgt{nullptr};
   LabeledSlider* slider_sgm_use_grey16{nullptr};
   //@}

   pipeline::disp_init::Params params;
   shared_ptr<const pipeline::disp_init::Result> result_ptr;

   Pimpl(This* in_parent)
       : parent(in_parent)
   {
      qim_ptr = make_shared<QImage>();
   }
   void make_ui() noexcept;

   void set_result(shared_ptr<const pipeline::disp_init::Result> ptr) noexcept
   {
      if(result_ptr.get() != ptr.get()) {
         result_ptr = ptr;
         emit parent->data_updated();
      }
   }

   pipeline::disp_init::Task* get_task_ptr(AppState& app)
   {
      return const_cast<pipeline::disp_init::Task*>(get_task_cptr(app));
   }

   const pipeline::disp_init::Task* get_task_cptr(const AppState& app)
   {
      if(!app.frame_results() or !app.has_current_camera()) return nullptr;
      const auto& pipeline = *app.frame_results();
      const auto cam_num   = app_state()->current_camera();
      if(cam_num >= pipeline.disp_init.size()) return nullptr;
      return pipeline.disp_init[cam_num].get();
   }

   bool widgets_to_model(AppState& app)
   {
      auto task_ptr = get_task_ptr(app);
      if(task_ptr == nullptr) return false;
      auto params = task_ptr->params(); // copy
      auto& p     = params.disp_params;
      auto& bmp   = p.stereobm_params;
      auto& sgm   = p.sgm_params;

      p.disparity_method
          = to_disparity_method(combo_disp_method->currentIndex());
      p.presmooth_sigma = slider_presmooth_sigma->value();

      bmp.bm_SAD_window_size     = slider_SAD_window_size->value();
      bmp.bm_12_max_diff         = slider_12_max_diff->value();
      bmp.bm_min_disparity       = slider_min_disparity->value();
      bmp.bm_num_disparities     = slider_num_disparities->value();
      bmp.bm_speckle_window_size = slider_spek_window_size->value();
      bmp.bm_speckle_range       = slider_spek_range->value();
      bmp.bm_prefilter_cap       = slider_prefilter_cap->value();
      bmp.bm_prefilter_size      = slider_prefilter_size->value();
      bmp.bm_prefilter_type      = slider_prefilter_type->value();
      bmp.bm_block_size          = slider_block_size->value();
      bmp.bm_texture_threshold   = slider_threshold->value();
      bmp.bm_uniqueness_ratio    = slider_unique_ratio->value();

      sgm.use_16bit_greyscale = bool(slider_sgm_use_grey16->value());

      if(false) {
         auto p2 = task_ptr->params().disp_params; // copy
         INFO(format("HERE, and method = {:s}/{:s}, diff = {:s}",
                     str(p.disparity_method),
                     str(p2.disparity_method),
                     str(p2 == p)));
      }

      const bool ret = task_ptr->set_params(params);

      if(false) {
         WARN(format("ret = {:s}, method = {:s}",
                     str(ret),
                     str(task_ptr->params().disp_params.disparity_method)));
      }

      if(ret) { emit parent->on_redraw(); }
      return ret;
   }

   void model_to_widgets(const AppState& app)
   {
      auto task_ptr = get_task_cptr(app);
      if(task_ptr == nullptr) return;
      const auto& params = task_ptr->params(); // copy
      const auto& p      = params.disp_params;
      const auto& bmp    = p.stereobm_params;
      const auto& sgm    = p.sgm_params;

      block_signal_set_combobox(combo_disp_method, int(p.disparity_method));
      block_signal_set_slider(slider_presmooth_sigma, p.presmooth_sigma);
      block_signal_set_slider(slider_SAD_window_size, bmp.bm_SAD_window_size);
      block_signal_set_slider(slider_12_max_diff, bmp.bm_12_max_diff);
      block_signal_set_slider(slider_min_disparity, bmp.bm_min_disparity);
      block_signal_set_slider(slider_num_disparities, bmp.bm_num_disparities);
      block_signal_set_slider(slider_spek_window_size,
                              bmp.bm_speckle_window_size);
      block_signal_set_slider(slider_spek_range, bmp.bm_speckle_range);
      block_signal_set_slider(slider_prefilter_cap, bmp.bm_prefilter_cap);
      block_signal_set_slider(slider_prefilter_size, bmp.bm_prefilter_size);
      block_signal_set_slider(slider_prefilter_type, bmp.bm_prefilter_type);
      block_signal_set_slider(slider_block_size, bmp.bm_block_size);
      block_signal_set_slider(slider_threshold, bmp.bm_texture_threshold);
      block_signal_set_slider(slider_unique_ratio, bmp.bm_uniqueness_ratio);

      block_signal_set_slider(slider_sgm_use_grey16, sgm.use_16bit_greyscale);
   }

   void show_hide_widgets(const Disparity::DisparityMethod method)
   {
      const bool is_stereobm    = (method == Disparity::STEREO_BM);
      const bool is_stereo_sgbm = (method == Disparity::STEREO_SGBM);
      const bool is_triclops    = (method == Disparity::TRICLOPS);
      const bool is_sgm         = (method == Disparity::CUDA_SGM);

      cpanel_bm_wgt->setVisible(is_stereobm);
   }
};

// --------------------------------------------------------------------- make ui

void This::Pimpl::make_ui() noexcept
{
   auto scene_desc = app_state()->scene_desc();

   // Cpanel | viewer

   // ---- (*) ---- Create widgets
   {
      const int min_SAD = 7, max_SAD = 51;
      const int min_nd = 16, max_nd = 1024;
      const int min_pf = 5, max_pf = 51;

      // cpanel widgets
      combo_disp_method      = new QComboBox{};
      slider_presmooth_sigma = new LabeledSlider{0.0, 10.0, 1001};

      slider_SAD_window_size
          = new LabeledSlider(min_SAD, max_SAD, (max_SAD - min_SAD) / 2 + 1);
      slider_12_max_diff   = new LabeledSlider(0, 1000, 1001);
      slider_min_disparity = new LabeledSlider(0, 100, 101);
      slider_num_disparities
          = new LabeledSlider(min_nd, max_nd, (max_nd - min_nd) / 2 + 1);
      slider_spek_window_size = new LabeledSlider(0, 1000, 1001);
      slider_spek_range       = new LabeledSlider(0, 1000, 1001);
      slider_prefilter_cap    = new LabeledSlider(0, 63, 64);
      slider_prefilter_size
          = new LabeledSlider(min_pf, max_pf, (max_pf - min_pf) / 2 + 1);
      slider_prefilter_type = new LabeledSlider(0, 1, 2);
      slider_block_size     = new LabeledSlider(0, 100, 101);
      slider_threshold      = new LabeledSlider(0, 5000, 5001);
      slider_unique_ratio   = new LabeledSlider(0, 100, 101);

      slider_sgm_use_grey16 = new LabeledSlider(0, 1, 2);

      { // set up the method combobox
         auto max_disp = unsigned(Disparity::CUDA_SGM);
         for(auto i = 0u; i <= max_disp; ++i)
            combo_disp_method->addItem(
                to_qstr(str(Disparity::DisparityMethod(i))));
      }
   }

   auto make_cpanel_top = [&]() {
      QFormLayout* layout = new QFormLayout{};
      layout->addRow("Method", combo_disp_method);
      layout->addRow("Presmooth Sigma", slider_presmooth_sigma);
      auto wgt = new QWidget{};
      wgt->setFixedWidth(app_state()->config().cpanel_width);
      wgt->setStyleSheet("background-color: white;");
      wgt->setLayout(layout);
      return wgt;
   };

   auto make_cpanel_bm_wgt = [&]() {
      QFormLayout* layout = new QFormLayout{};
      layout->addRow("SAD Window Size", slider_SAD_window_size);
      layout->addRow("12 Max Diff", slider_12_max_diff);
      layout->addRow("Min Disparity", slider_min_disparity);
      layout->addRow("Num Disparities", slider_num_disparities);
      layout->addRow("Spec Window Size", slider_spek_window_size);
      layout->addRow("Spec Range", slider_spek_range);
      layout->addRow("Prefilter Cap", slider_prefilter_cap);
      layout->addRow("Prefilter Size", slider_prefilter_size);
      layout->addRow("Prefilter Type", slider_prefilter_type);
      layout->addRow("Block Size", slider_block_size);
      layout->addRow("Threshold", slider_threshold);
      layout->addRow("Unique Ratio", slider_unique_ratio);
      cpanel_bm_wgt = new QWidget{};
      cpanel_bm_wgt->setFixedWidth(app_state()->config().cpanel_width);
      cpanel_bm_wgt->setStyleSheet("background-color: white;");
      cpanel_bm_wgt->setLayout(layout);
      return cpanel_bm_wgt;
   };

   auto make_cpanel_sgm_wgt = [&]() {
      QFormLayout* layout = new QFormLayout{};
      layout->addRow("16bit Greyscale", slider_sgm_use_grey16);
      cpanel_sgm_wgt = new QWidget{};
      cpanel_sgm_wgt->setFixedWidth(app_state()->config().cpanel_width);
      cpanel_sgm_wgt->setStyleSheet("background-color: white;");
      cpanel_sgm_wgt->setLayout(layout);
      return cpanel_sgm_wgt;
   };

   {
      QVBoxLayout* layout = new QVBoxLayout{};

      layout->addWidget(make_cpanel_top());
      layout->addWidget(make_cpanel_bm_wgt());
      layout->addWidget(make_cpanel_sgm_wgt());
      layout->addWidget(make_vertical_expander());

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

#define CONNECT_SLIDER(source, slot)                                        \
   {                                                                        \
      connect(source, SIGNAL(value_changed(double)), parent, SLOT(slot())); \
   }

#define CONNECT_COMBO(source, slot)                                            \
   {                                                                           \
      connect(source, SIGNAL(currentIndexChanged(int)), parent, SLOT(slot())); \
   }

   CONNECT_COMBO(combo_disp_method, widgets_to_model);
   CONNECT_SLIDER(slider_presmooth_sigma, widgets_to_model);
   CONNECT_SLIDER(slider_SAD_window_size, widgets_to_model);
   CONNECT_SLIDER(slider_12_max_diff, widgets_to_model);
   CONNECT_SLIDER(slider_min_disparity, widgets_to_model);
   CONNECT_SLIDER(slider_num_disparities, widgets_to_model);
   CONNECT_SLIDER(slider_spek_window_size, widgets_to_model);
   CONNECT_SLIDER(slider_spek_range, widgets_to_model);
   CONNECT_SLIDER(slider_prefilter_cap, widgets_to_model);
   CONNECT_SLIDER(slider_prefilter_size, widgets_to_model);
   CONNECT_SLIDER(slider_prefilter_type, widgets_to_model);
   CONNECT_SLIDER(slider_block_size, widgets_to_model);
   CONNECT_SLIDER(slider_threshold, widgets_to_model);
   CONNECT_SLIDER(slider_unique_ratio, widgets_to_model);

   CONNECT_SLIDER(slider_sgm_use_grey16, widgets_to_model);
   //@}

#undef CONNECT_TEXT
#undef CONNECT_CB
#undef CONNECT_SLIDER
#undef CONNECT_COMBO
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

   auto task = P.get_task_ptr(*app_state());
   if(task == nullptr) {
      set_qimage(nullptr);
      return;
   }

   P.show_hide_widgets(task->params().disp_params.disparity_method);

   // Poke the task to get a result (threaded)
   schedule([task, this]() {
      task->result([this](auto x) { this->pimpl_->set_result(x); });
   });

   // Get the latest result... (thread-safe)... will be nullptr if
   // we're currently calculating
   auto dat = task->try_get_casted_result();

   if(!dat) {
      set_qimage(nullptr);
      return;
   }

   // Okay, we have a result... let's display it
   to_qimage(dat->disparity_image[0], *P.qim_ptr);
   set_qimage(P.qim_ptr);
}

bool This::widgets_to_model() { return pimpl_->widgets_to_model(*app_state()); }

void This::model_to_widgets() { pimpl_->model_to_widgets(*app_state()); }
