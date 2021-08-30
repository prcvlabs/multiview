
#include "stdinc.hpp"

#include "frame-viewer.hh"

#include "gui/app-state.hh"
#include "gui/interface/interface-helpers.hpp"
#include "gui/qt-helpers.hpp"
#include "gui/widgets/image-viewer-2.hh"

#define This FrameViewer

using namespace perceive;

// ----------------------------------------------------------------------- pimpl

struct This::Pimpl
{
   FrameViewer* parent{nullptr};

   QWidget* cpanel{nullptr};
   ImageViewer2* image_viewer{nullptr};
   shared_ptr<QImage> qim_ptr{nullptr};

   SensorSelector* sensor_selector = nullptr;
   QCheckBox* cb_floor_grid{nullptr};
   QRadioButton* rb_show_distorted{nullptr};
   QRadioButton* rb_show_still{nullptr};
   QRadioButton* rb_show_rectified{nullptr};

   shared_ptr<const pipeline::copy_sensor_images::Result> result_ptr;
   shared_ptr<const pipeline::floor_hist::Result> fhist_ret_ptr;

   Pimpl(FrameViewer* in_parent)
       : parent(in_parent)
   {
      qim_ptr = make_shared<QImage>();
   }
   void make_ui() noexcept;

   // --- Copy sensor images result
   void set_result(
       shared_ptr<const pipeline::copy_sensor_images::Result> ptr) noexcept
   {
      if(result_ptr.get() != ptr.get()) {
         result_ptr = ptr;
         emit parent->data_updated();
      }
   }

   pipeline::copy_sensor_images::Task* get_task_ptr(AppState& app)
   {
      return const_cast<pipeline::copy_sensor_images::Task*>(
          get_task_cptr(app));
   }

   const pipeline::copy_sensor_images::Task* get_task_cptr(const AppState& app)
   {
      if(!app.frame_results()) return nullptr;
      return &(app.frame_results()->copy_sensor_images);
   }

   // --- Floor-hist result
   void
   set_fhist_result(shared_ptr<const pipeline::floor_hist::Result> ptr) noexcept
   {
      if(fhist_ret_ptr.get() != ptr.get()) {
         fhist_ret_ptr = ptr;
         emit parent->data_updated();
      }
   }

   pipeline::floor_hist::Task* get_fhist_task_ptr(const AppState& app)
   {
      return const_cast<pipeline::floor_hist::Task*>(get_fhist_task_cptr(app));
   }

   const pipeline::floor_hist::Task* get_fhist_task_cptr(const AppState& app)
   {
      if(!app.frame_results()) return nullptr;
      return &(app.frame_results()->floor_hist);
   }
};

// --------------------------------------------------------------------- make ui

void This::Pimpl::make_ui() noexcept
{
   auto scene_desc    = app_state()->scene_desc();
   const int n_frames = !scene_desc ? 1 : scene_desc->n_frames();

   // Cpanel | video
   //        --------------

   // ---- (*) ---- Create widgets
   { // cpanel widgets
      sensor_selector   = make_sensor_selector();
      cb_floor_grid     = new QCheckBox{"Floor Grid"};
      rb_show_distorted = new QRadioButton{"Distorted"};
      rb_show_still     = new QRadioButton{"Still"};
      rb_show_rectified = new QRadioButton{"Rectified"};
   }

   QGroupBox* gb = new QGroupBox("View");
   { // collect radio buttons into a group box
      QVBoxLayout* vbox = new QVBoxLayout;
      vbox->addWidget(rb_show_distorted);
      vbox->addWidget(rb_show_still);
      vbox->addWidget(rb_show_rectified);
      vbox->addStretch(1);
      gb->setLayout(vbox);
   }

   { // cpanel
      QFormLayout* layout = new QFormLayout{};

      layout->addRow(sensor_selector);
      layout->addRow(cb_floor_grid);
      layout->addRow(gb);

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
      image_viewer->set_pixel_indicator(true);
   }

   QWidget* right_panel = new QWidget{};
   {
      QVBoxLayout* layout = new QVBoxLayout{};
      layout->addWidget(image_viewer);
      right_panel->setLayout(layout);
   }

   QHBoxLayout* layout = new QHBoxLayout{};
   layout->addWidget(cpanel);
   layout->addWidget(right_panel);
   parent->setLayout(layout);

   // ---- (*) ---- Initialize
   // cb_sensor->setChecked(true);
   rb_show_distorted->setChecked(true);
   cb_floor_grid->setChecked(true);

   // ---- (*) ---- Wiring
   connect(app_state(),
           SIGNAL(camera_changed(unsigned)),
           parent,
           SLOT(on_redraw()));

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

#define CONNECT_SLIDER(source, slot)                                    \
   {                                                                    \
      connect(source, SIGNAL(valueChanged(int)), parent, SLOT(slot())); \
   }

   CONNECT_CB(cb_floor_grid, on_redraw);
   CONNECT_RB(rb_show_distorted, on_redraw);
   CONNECT_RB(rb_show_still, on_redraw);
   CONNECT_RB(rb_show_rectified, on_redraw);

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

// ------------------------------------------------------------- Getters/Setters

void This::set_qimage(const shared_ptr<const QImage> qim)
{
   get_image_viewer()->set_qimage(qim);
}

ImageViewer2* This::get_image_viewer() noexcept { return pimpl_->image_viewer; }

This::view_e This::view() const noexcept
{
   if(pimpl_->rb_show_distorted->isChecked()) return DISTORTED_IMAGE;
   if(pimpl_->rb_show_still->isChecked()) return STILL_IMAGE;
   return RECTIFIED_IMAGE;
}

void This::set_view(view_e v) noexcept
{
   switch(v) {
   case DISTORTED_IMAGE: pimpl_->rb_show_distorted->setChecked(true);
   case STILL_IMAGE: pimpl_->rb_show_still->setChecked(true);
   case RECTIFIED_IMAGE: pimpl_->rb_show_rectified->setChecked(true);
   }
   on_redraw();
}

// ------------------------------------------------------------------- on-redraw

void This::on_redraw()
{
   auto& P         = *pimpl_;
   auto scene_desc = app_state()->scene_desc();

   // P.ensure_correct_rb_selected();
   // const int sensor_id = this->sensor_id();
   const auto sensor_id = app_state()->current_sensor_index();
   const auto sensor_id_okay
       = sensor_id >= 0 and sensor_id < scene_desc->n_sensors();

   const bool draw_grid = P.cb_floor_grid->isChecked();

   auto finish_with_maybe_grid = [&](const cv::Mat& cv_im) {
      Expects(unsigned(sensor_id) < scene_desc->sensor_transforms.size());
      ARGBImage im0    = cv_to_argb(cv_im);
      const auto& aabb = scene_desc->scene_info.hist_bounds;
      const auto& cu   = scene_desc->cu(sensor_id);
      const auto& et   = scene_desc->sensor_transforms[sensor_id];
      if(draw_grid) {
         const auto& grid = scene_desc->floor_grid(sensor_id);
         Expects(grid.height == im0.height);
         Expects(grid.width == im0.width);
         const uint32_t mask_colour = 0xff000000u;
         for(auto y = 0u; y < grid.height; ++y)
            for(auto x = 0u; x < grid.width; ++x)
               if(grid(x, y) != mask_colour) im0(x, y) = grid(x, y);
      }

      return im0;
   };

   auto draw_distorted = [&]() {
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

      const auto im
          = finish_with_maybe_grid(dat->images.sensor_images[sensor_id]);
      to_qimage(im, *P.qim_ptr);
      set_qimage(P.qim_ptr);
   };

   auto draw_still = [&]() {
      const auto im = finish_with_maybe_grid(
          argb_to_cv(LAB_im_to_argb(scene_desc->sensor_LAB_still(sensor_id))));

      to_qimage(im, *P.qim_ptr);
      set_qimage(P.qim_ptr);
   };

   auto draw_rectified = [&]() {
      auto task = P.get_fhist_task_ptr(*app_state());
      if(task == nullptr) {
         set_qimage(nullptr);
         return;
      }

      schedule([task, this]() {
         task->result([this](auto x) { this->pimpl_->set_fhist_result(x); });
      });

      // Get the latest result... (thread-safe)... will be nullptr if
      // we're currently calculating
      auto dat = task->try_get_casted_result();

      if(!dat) {
         set_qimage(nullptr);
         return;
      }

      const auto [cam_ind, sen_ind] = scene_desc->bcam_lookup2(sensor_id);
      auto rect_ret_ptr             = dat->rect_result(cam_ind, sen_ind);
      if(rect_ret_ptr == nullptr) {
         WARN(format("failed to get rectified image result!"));
         set_qimage(nullptr);
      } else {
         to_qimage(rect_ret_ptr->rectified, *P.qim_ptr);
         set_qimage(P.qim_ptr);
      }
   };

   if(!sensor_id_okay) {
      set_qimage(nullptr);
   } else {
      switch(this->view()) {
      case DISTORTED_IMAGE: draw_distorted(); break;
      case STILL_IMAGE: draw_still(); break;
      case RECTIFIED_IMAGE: draw_rectified(); break;
      }
   }
}
