
#include "stdinc.hpp"

#include "openpose-viewer.hh"

#include "gui/app-state.hh"
#include "gui/interface/interface-helpers.hpp"
#include "gui/qt-helpers.hpp"
#include "gui/widgets/image-viewer-2.hh"
#include "gui/widgets/labeled-slider.hh"
#include "perceive/cost-functions/pose-skeleton/pose-skeleton-exec.hpp"
#include "perceive/geometry/skeleton/2d-helpers.hpp"

#define This OpenposeViewer

using namespace perceive;

// ----------------------------------------------------------------------- pimpl

struct This::Pimpl
{
   This* parent{nullptr};
   QWidget* cpanel{nullptr};
   ImageViewer2* image_viewer{nullptr};
   shared_ptr<QImage> qim_ptr{nullptr};

   //@{ cpanel widgets
   SensorSelector* sensor_selector  = nullptr;
   QComboBox* combo_model_select    = nullptr;
   LabeledSlider* slider_net_size_x = nullptr;
   LabeledSlider* slider_net_size_y = nullptr;
   QLabel* label_runtime            = nullptr;
   //@}

   perceive::pose_skeleton::OpenposeParams params;
   shared_ptr<const pipeline::pose_skeleton_init::Result> otask_result_ptr;
   shared_ptr<const pipeline::localization::Result> ltask_result_ptr;
   shared_ptr<const perceive::SceneDescription> scene_desc;

   Pimpl(This* in_parent)
       : parent(in_parent)
   {
      qim_ptr = make_shared<QImage>();
   }
   void make_ui() noexcept;

   int current_sensor_index() const noexcept
   {
      if(!scene_desc or !app_state()->frame_results()
         or !app_state()->has_current_camera())
         return -1;
      const auto cam_num    = app_state()->current_camera();
      const auto sensor_num = 0;
      return scene_desc->sensor_lookup(cam_num, sensor_num);
   }

   void set_otask_result(
       shared_ptr<const pipeline::pose_skeleton_init::Result> ptr) noexcept
   {
      if(otask_result_ptr.get() != ptr.get()) {
         otask_result_ptr = ptr;
         emit parent->data_updated();
      }
   }

   void set_ltask_result(
       shared_ptr<const pipeline::localization::Result> ptr) noexcept
   {
      if(ltask_result_ptr.get() != ptr.get()) {
         ltask_result_ptr = ptr;
         emit parent->data_updated();
      }
   }

   // -------------------------------------------------------- openpose task ptr
   pipeline::pose_skeleton_init::Task* get_task_ptr(AppState& app)
   {
      return const_cast<pipeline::pose_skeleton_init::Task*>(
          get_task_cptr(app));
   }

   const pipeline::pose_skeleton_init::Task* get_task_cptr(const AppState& app)
   {
      return !app_state()->frame_results()
                 ? nullptr
                 : &(app_state()->frame_results()->pose_skeleton_init);
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
   std::tuple<shared_ptr<const pipeline::pose_skeleton_init::Result>,
              shared_ptr<const pipeline::localization::Result>>
   poke_tasks()
   {
      if(!app_state()->frame_results()) return {};

      auto otask = get_task_ptr(*app_state());
      auto ltask = ltask_ptr();

      { // openpose
         schedule([otask, this]() {
            otask->result([this](auto x) { set_otask_result(x); });
         });
      }

      { // localization
         schedule([ltask, this]() {
            ltask->result([this](auto x) { set_ltask_result(x); });
         });
      }

      return std::make_tuple(otask->try_get_casted_result(),
                             ltask->try_get_casted_result());
   }

   bool widgets_to_model()
   {
      auto task_ptr = get_task_ptr(*app_state());
      if(task_ptr == nullptr) return false;
      auto params = task_ptr->params(); // copy
      auto& p     = params.op_params.hp_params;

      // widget_to_value(slider_net_size_x, p.network_resolution.x);
      // widget_to_value(slider_net_size_y, p.network_resolution.y);

      // pose_skeleton::hyperpose_models
      const auto model_idx = size_t(combo_model_select->currentIndex());
      Expects(model_idx < pose_skeleton::hyperpose_models.size());
      p.model_name = string(pose_skeleton::hyperpose_models.at(model_idx));

      const bool ret = task_ptr->set_params(params);
      if(ret) { emit parent->on_redraw(); }
      return ret;
   }

   void model_to_widgets()
   {
      auto task_ptr = get_task_cptr(*app_state());
      if(task_ptr == nullptr) return;
      const auto& params = task_ptr->params(); // copy
      const auto& p      = params.op_params.hp_params;

      const auto ii = std::find(cbegin(pose_skeleton::hyperpose_models),
                                cend(pose_skeleton::hyperpose_models),
                                p.model_name);
      Expects(ii != cend(pose_skeleton::hyperpose_models));
      block_signal_set_combobox(
          combo_model_select,
          int(std::distance(cbegin(pose_skeleton::hyperpose_models), ii)));

      // block_signal_set(slider_net_size_x, p.network_resolution.x);
      // block_signal_set(slider_net_size_y, p.network_resolution.y);
   }
};

// --------------------------------------------------------------------- make ui

void This::Pimpl::make_ui() noexcept
{
   auto scene_desc = app_state()->scene_desc();

   // Cpanel | viewer

   // ---- (*) ---- Create widgets
   {
      sensor_selector    = make_sensor_selector();
      combo_model_select = new QComboBox{};
      slider_net_size_x  = new LabeledSlider(32, 800, (800 - 32) / 16 + 1);
      slider_net_size_y  = new LabeledSlider(32, 800, (800 - 32) / 16 + 1);
      label_runtime      = new QLabel{};

      for(const auto name : pose_skeleton::hyperpose_models)
         combo_model_select->addItem(to_qstr(name));
   }

   { // cpanel
      QFormLayout* layout = new QFormLayout{};

      layout->addRow(sensor_selector);
      layout->addRow(make_hline());
      layout->addRow("Model:", combo_model_select);
      layout->addRow("Net Size X:", slider_net_size_x);
      layout->addRow("Net Size Y:", slider_net_size_y);
      layout->addRow("Execution time:", label_runtime);

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
           SLOT(on_open_finished()),
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

   connect(combo_model_select,
           SIGNAL(currentIndexChanged(int)),
           parent,
           SLOT(widgets_to_model()));
   CONNECT_SLIDER(slider_net_size_x, widgets_to_model);
   CONNECT_SLIDER(slider_net_size_y, widgets_to_model);

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

   const auto scene_desc   = app_state()->scene_desc();
   const auto [odat, ldat] = P.poke_tasks();

   if(!odat or !ldat or !scene_desc) {
      set_qimage(nullptr);
      return;
   }

   const int sensor_ind = app_state()->current_sensor_index();
   const unsigned size  = unsigned(ldat->data.p2ds.size());
   if(unsigned(sensor_ind) >= size) {
      set_qimage(nullptr);
      return;
   }

   const auto s = odat->op.seconds;
   P.label_runtime->setText(
       to_qstr(format("{:5.3f}s, or {:6.2f} Hz", s, (1.0 / s))));

   Expects(odat->copy_images_result != nullptr);
   Expects(unsigned(sensor_ind)
           < odat->copy_images_result->images.sensor_images.size());
   cv::Mat im = odat->copy_images_result->images.sensor_images[sensor_ind];
   auto argb  = cv_to_argb(im);

   for(const auto& info : ldat->data.p2ds[sensor_ind]) {
      const auto p2d_ptr = info.p2d_ptr.get();
      Expects(p2d_ptr != nullptr);

      const auto kolour = colour_set_4(info.id);
      const auto& dcam  = scene_desc->dcam(sensor_ind);
      const auto& cu    = scene_desc->cu(sensor_ind);
      const auto& et    = scene_desc->sensor_transforms[sensor_ind];

      render_pose(argb, *p2d_ptr);

      plot_skeleton_3d(dcam,
                       p2d_ptr->best_3d_result(),
                       fNAN,
                       argb.bounds(),
                       [&](int x, int y, float alpha) {
                          if(argb.in_bounds(x, y))
                             argb(x, y) = blend(kolour, argb(x, y), alpha);
                       });

      auto render_base = [&](const Vector3f& X, bool stipple) {
         const auto radius = human::k_adult_male_radius;
         const auto Z_axis = Vector3f{0.0f, 0.0f, 1.0f};
         const auto Xf     = Vector3f(X.x, X.y, 0.0f);

         if(Xf.is_finite()) {
            int counter = 0;
            project_circle(dcam,
                           Xf,
                           radius,
                           Z_axis,
                           false,
                           argb.bounds(),
                           [&](int x, int y, float alpha) {
                              if(stipple && ++counter % 3 != 0) return;
                              if(argb.in_bounds(x, y))
                                 argb(x, y) = blend(kolour, argb(x, y), alpha);
                           });
         }
      };
      // const auto X  = p2d_ptr->best_3d_result().X();
      const auto Xc = p2d_ptr->best_3d_result().Xs_centre();
      // render_base(X, true);
      render_base(Xc, true);
   }

   // Okay, we have a result... let's display it
   to_qimage(argb, *P.qim_ptr);
   set_qimage(P.qim_ptr);
}

bool This::widgets_to_model() { return pimpl_->widgets_to_model(); }

void This::model_to_widgets() { pimpl_->model_to_widgets(); }

void This::on_open_finished()
{
   pimpl_->scene_desc = app_state()->scene_desc();
   model_to_widgets();
   on_redraw();
}
