
#include "stdinc.hpp"

#include "slic-viewer.hh"

#include "gui/app-state.hh"
#include "gui/interface/interface-helpers.hpp"
#include "gui/qt-helpers.hpp"
#include "gui/widgets/image-viewer-2.hh"
#include "gui/widgets/labeled-slider.hh"
#include "gui/widgets/sensor-selector.hh"

#define This SlicViewer

using namespace perceive;

// ----------------------------------------------------------------------- pimpl

struct This::Pimpl
{
   This* parent{nullptr};
   QWidget* cpanel{nullptr};
   ImageViewer2* image_viewer{nullptr};
   shared_ptr<QImage> qim_ptr{nullptr};

   //@{ cpanel widgets
   SensorSelector* sensor_selector = nullptr;
   QCheckBox* cb_use_cuda          = nullptr;
   QCheckBox* cb_edge_enhance      = nullptr;
   LabeledSlider* slider_superpixel_size{nullptr};
   LabeledSlider* slider_compactness{nullptr};

   QRadioButton* rb_still_score  = nullptr;
   QRadioButton* rb_segmentation = nullptr;
   //@}

   pipeline::run_f2d::Params params;
   shared_ptr<const pipeline::run_f2d::Result> result_ptr;

   Pimpl(This* in_parent)
       : parent(in_parent)
   {
      qim_ptr = make_shared<QImage>();
   }
   void make_ui() noexcept;

   void
   set_result(shared_ptr<const pipeline::run_f2d::Result> ptr) noexcept
   {
      if(result_ptr.get() != ptr.get()) {
         result_ptr = ptr;
         emit parent->data_updated();
      }
   }

   pipeline::run_f2d::Task* get_task_ptr(AppState& app)
   {
      return const_cast<pipeline::run_f2d::Task*>(get_task_cptr(app));
   }

   const pipeline::run_f2d::Task* get_task_cptr(const AppState& app)
   {
      if(!app.frame_results() or !app.has_current_camera()) return nullptr;
      // const unsigned cam_num    = app.current_camera();
      // const unsigned sensor_num = app.current_sensor_pos();
      const auto sensor_ind = app.current_sensor_index();
      const string name     = format("run_f2d[{}]", sensor_ind);
      const auto& pipeline  = *app.frame_results();
      // Get the correct run-f2d
      auto ii = std::find_if(cbegin(pipeline.run_f2d),
                             cend(pipeline.run_f2d),
                             [&](auto& x) { return x->taskname() == name; });

      if(ii == cend(pipeline.run_f2d)) {
         WARN(format("failed to find run_f2d for cam={}, sensor={}",
                     app.current_camera(),
                     app.current_sensor_pos()));
         return nullptr;
      }

      return ii->get();
   }

   bool widgets_to_model(AppState& app)
   {
      auto task_ptr = get_task_ptr(app);
      if(task_ptr == nullptr) return false;
      pipeline::run_f2d::Params p      = task_ptr->params(); // copy
      p.f2d.use_slic_cuda_if_available = cb_use_cuda->isChecked();
      p.f2d.slic_edge_enhance          = cb_edge_enhance->isChecked();
      p.f2d.superpixel_size            = slider_superpixel_size->value();
      p.f2d.compactness                = slider_compactness->value();
      const bool ret                   = task_ptr->set_params(p);
      if(ret) { emit parent->on_redraw(); }
      return ret;
   }

   void model_to_widgets(const AppState& app)
   {
      auto task_ptr = get_task_cptr(app);
      if(task_ptr == nullptr) return;
      const auto& p = task_ptr->params();

      block_signal_set_checked(cb_use_cuda, p.f2d.use_slic_cuda_if_available);
      block_signal_set_checked(cb_edge_enhance, p.f2d.slic_edge_enhance);
      block_signal_set_slider(slider_superpixel_size, p.f2d.superpixel_size);
      block_signal_set_slider(slider_compactness, p.f2d.compactness);
   }
};

// --------------------------------------------------------------------- make ui

void This::Pimpl::make_ui() noexcept
{
   auto scene_desc = app_state()->scene_desc();

   // Cpanel | viewer

   // ---- (*) ---- Create widgets
   { // cpanel widgets
      sensor_selector        = make_sensor_selector();
      cb_use_cuda            = new QCheckBox{};
      cb_edge_enhance        = new QCheckBox{};
      slider_superpixel_size = new LabeledSlider(50.0, 2000.0, 2000 - 50 + 1);
      slider_compactness = new LabeledSlider(0.1, 20.0, (20.0 - 0.1) * 10 + 1);

      rb_segmentation = new QRadioButton{"Segmentation"};
      rb_still_score  = new QRadioButton{"Still score"};
      rb_still_score->setChecked(true);
   }

   { // cpanel
      auto make_view_rbs = [&]() {
         auto layout = new QHBoxLayout{};
         layout->addWidget(rb_segmentation);
         layout->addWidget(rb_still_score);
         auto wgt = new QWidget{};
         wgt->setLayout(layout);
         return wgt;
      };

      QFormLayout* layout = new QFormLayout{};

      layout->addRow(sensor_selector);
      layout->addRow(make_view_rbs());
      layout->addRow(make_hline());
      layout->addRow("Cuda:", cb_use_cuda);
      layout->addRow("Edge Enhance:", cb_edge_enhance);
      layout->addRow("Size:", slider_superpixel_size);
      layout->addRow("Compactness:", slider_compactness);

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

   connect(app_state(),
           SIGNAL(camera_changed(unsigned)),
           parent,
           SLOT(on_redraw()));

   connect(app_state(),
           SIGNAL(sensor_pos_changed(unsigned)),
           parent,
           SLOT(on_redraw()));

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

   CONNECT_CB(cb_use_cuda, widgets_to_model);
   CONNECT_CB(cb_edge_enhance, widgets_to_model);
   CONNECT_SLIDER(slider_superpixel_size, widgets_to_model);
   CONNECT_SLIDER(slider_compactness, widgets_to_model);

   CONNECT_RB(rb_segmentation, on_redraw);
   CONNECT_RB(rb_still_score, on_redraw);

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

   auto task = P.get_task_ptr(*app_state());
   if(task == nullptr) {
      set_qimage(nullptr);
      return;
   }

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
   if(P.rb_segmentation->isChecked())
      to_qimage(make_slic_labeled_image(dat->f2d, false), *P.qim_ptr);
   else
      to_qimage(make_slic_labeled_image(dat->f2d, true), *P.qim_ptr);
   set_qimage(P.qim_ptr);
}

bool This::widgets_to_model() { return pimpl_->widgets_to_model(*app_state()); }

void This::model_to_widgets() { pimpl_->model_to_widgets(*app_state()); }
