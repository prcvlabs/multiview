
#include "stdinc.hpp"

#include "sensor-selector.hh"

#include "labeled-slider.hh"

#include "gui/interface/interface-helpers.hpp"
#include "gui/qt-helpers.hpp"

#include <utility>

#include <QFormLayout>
#include <QVBoxLayout>

#define This SensorSelector

using namespace perceive;

static QString to_qstr(string_view s) noexcept
{
   return QString::fromUtf8(s.data());
}

// ----------------------------------------------------------------------- pimpl

struct This::Pimpl
{
   This* parent{nullptr};

   QFormLayout* layout{nullptr};
   LabeledSlider* slider_camera_pos{nullptr};
   LabeledSlider* slider_sensor_pos{nullptr};
   std::vector<int> camera_data;
   bool signals_blocked = false;

   Pimpl(This* in_parent)
       : parent(in_parent)
   {}
   void make_ui() noexcept;

   std::pair<int, int> value() const noexcept
   {
      return std::make_pair<int, int>(slider_camera_pos->value(),
                                      slider_sensor_pos->value());
   }

   bool block_signals(bool val) noexcept
   {
      auto ret        = signals_blocked;
      signals_blocked = val;
      return ret;
   }

   void internal_block_signals(bool val) noexcept
   {
      slider_camera_pos->blockSignals(val);
      slider_sensor_pos->blockSignals(val);
   }

   void set_camera_label() noexcept
   {
      internal_block_signals(true);
      slider_camera_pos->set_value(slider_camera_pos->value());
      slider_sensor_pos->set_value(slider_sensor_pos->value());
      internal_block_signals(false);
   }

   void synchronize_widgets() noexcept
   {
      internal_block_signals(true);
      const int n_cams = int(camera_data.size());

      if(n_cams == 0) {
         slider_camera_pos->init(0, 0, 0);
         slider_sensor_pos->init(0, 0, 0);
      } else {
         if(int(slider_camera_pos->n_steps()) != n_cams)
            slider_camera_pos->init(0, std::max(0, n_cams - 1), n_cams);
         const int cam_num0 = int(slider_camera_pos->value());
         const int cam_num  = (unsigned(cam_num0) < camera_data.size())
                                 ? cam_num0
                                 : int(camera_data.size() - 1);

         Expects(unsigned(cam_num) < camera_data.size());
         // INFO(format("{} in [{:s}]",
         //             cam_num,
         //             implode(begin(camera_data), end(camera_data), ", ")));
         const int n_sensors = camera_data[cam_num];
         if(int(slider_sensor_pos->n_steps()) != n_sensors)
            slider_sensor_pos->init(0, std::max(0, n_sensors - 1), n_sensors);
         slider_camera_pos->set_value(slider_camera_pos->value());
         slider_sensor_pos->set_value(slider_sensor_pos->value());
      }
      internal_block_signals(false);
   }

   void set_value(std::pair<int, int> value) noexcept
   {
      bool has_changed = (value.first != int(slider_camera_pos->value()))
                         or (value.second != int(slider_sensor_pos->value()));
      internal_block_signals(true);
      slider_camera_pos->set_value(value.first);
      slider_sensor_pos->set_value(value.second);
      internal_block_signals(false);
      if(!signals_blocked and has_changed) parent->do_changed_();
   }
};

// --------------------------------------------------------------------- make ui

void This::Pimpl::make_ui() noexcept
{
   { // Create widgets
      slider_camera_pos = new LabeledSlider(0, 0, 0);
      slider_sensor_pos = new LabeledSlider(0, 0, 0);

      slider_camera_pos->get_label()->setFixedWidth(100);
      slider_sensor_pos->get_label()->setFixedWidth(100);

      slider_camera_pos->label_render_fun = [this](double val) -> string {
         return this->parent->set_camera_label(int(val));
      };

      slider_sensor_pos->label_render_fun = [this](double val) -> string {
         int cam_num = int(this->slider_camera_pos->value());
         return this->parent->set_sensor_label(cam_num, int(val));
      };

      slider_camera_pos->set_value(0);
      slider_sensor_pos->set_value(0);
   }

   { // layout
      layout = new QFormLayout{};
      layout->addRow("Camera: ", slider_camera_pos);
      layout->addRow("Sensor: ", slider_sensor_pos);

      parent->setLayout(layout);
   }

   { // init
      camera_data.clear();
      synchronize_widgets();
   }

   { // wiring
      connect(slider_camera_pos,
              SIGNAL(value_changed(double)),
              parent,
              SLOT(on_value_camera_changed_()));

      connect(slider_sensor_pos,
              SIGNAL(value_changed(double)),
              parent,
              SLOT(on_value_sensor_changed_()));
   }
}

// ---------------------------------------------------------------- construction

This::This(QWidget* parent)
    : QWidget(parent)
    , pimpl_(new Pimpl(this))
{
   set_camera_label = [](int x) {
      // WARN("set-camera-label function not set.");
      return ""s;
   };
   set_sensor_label = [](int x, int y) {
      // WARN("set-sensor-label function not set.");
      return ""s;
   };
   pimpl_->make_ui();
}

This::~This() { delete pimpl_; }

// ----------------------------------------------------------------------- value

std::pair<int, int> This::value() const noexcept { return pimpl_->value(); }

void This::set_value(std::pair<int, int> value) noexcept
{
   pimpl_->set_value(value);
}

void This::set_value(int x, int y) noexcept
{
   pimpl_->set_value(std::make_pair<int, int>(int(x), int(y)));
}

bool This::block_signals(bool val) noexcept
{
   return pimpl_->block_signals(val);
}

// ------------------------------------------------------------------------ init

void This::init_cameras(const std::vector<int>& val) noexcept
{
   pimpl_->camera_data = val;
   pimpl_->synchronize_widgets();
}

// ---------------------------------------------------- set-min-form-label-width

void This::set_min_form_label_width(int width)
{
   finish_form_layout(pimpl_->layout, width);
   // for_each_qform_label(pimpl_->layout, [width](QWidget* wgt) {
   //    wgt->setMinimumWidth(width);
   //    dynamic_cast<QLabel*>(wgt)->setAlignment(Qt::AlignRight
   //                                             | Qt::AlignVCenter);
   // });
}

// ----------------------------------------------------------------------- slots

void This::on_redraw() noexcept { pimpl_->set_camera_label(); }

void This::set_camera(unsigned val) noexcept
{
   set_value(val, pimpl_->value().second);
}

void This::set_sensor_pos(unsigned val) noexcept
{
   set_value(pimpl_->value().first, val);
}

void This::on_load() noexcept
{
   if(on_load_callback) on_load_callback();
}

void This::on_value_camera_changed_()
{
   pimpl_->synchronize_widgets();
   pimpl_->set_camera_label();
   do_changed_();
}

void This::on_value_sensor_changed_() { do_changed_(); }

void This::do_changed_()
{
   auto value
       = std::make_pair<int, int>(int(pimpl_->slider_camera_pos->value()),
                                  int(pimpl_->slider_sensor_pos->value()));
   if(value_changed_callback) value_changed_callback(value);
   if(!pimpl_->signals_blocked) emit value_changed(value);
}
