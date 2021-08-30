
#include "interface-helpers.hpp"

#include <QObject>

#include "gui/app-state.hh"

namespace perceive
{
// -------------------------------------------------------- make sensor selector
//
SensorSelector* make_sensor_selector() noexcept
{
   auto sensor_selector = new SensorSelector{};

   sensor_selector->set_camera_label = [](int x) -> string {
      const unsigned cam_num = app_state()->current_camera();
      const auto ss          = app_state()->scene_desc();
      if(ss) {
         const auto& keys = ss->scene_info.bcam_keys;
         if(cam_num < keys.size()) return keys[cam_num];
      }
      return "No camera"s;
   };

   sensor_selector->set_sensor_label = [](int x, int y) {
      const auto ss = app_state()->scene_desc();
      if(ss) {
         const auto ind = ss->sensor_lookup(x, y);
         if(ind >= 0) {
            Expects(unsigned(ind) < ss->sensor_ids.size());
            return ss->sensor_ids[ind];
         }
      }
      return "<invalid>"s;
   };

   sensor_selector->value_changed_callback
       = [sensor_selector](std::pair<int, int> xy) {
            const auto cn = xy.first;
            const auto sn = xy.second;
            if(int(app_state()->current_camera()) != cn)
               app_state()->set_current_camera(cn);
            if(int(app_state()->current_sensor_pos()) != sn)
               app_state()->set_current_sensor_pos(sn);
            sensor_selector->on_redraw();
         };

   sensor_selector->on_load_callback = [sensor_selector]() {
      sensor_selector->init_cameras(app_state()->sensor_counts());
   };

   sensor_selector->set_min_form_label_width(
       app_state()->config().cpanel_form_label_min_width);

   QObject::connect(app_state(),
                    SIGNAL(camera_changed(unsigned)),
                    sensor_selector,
                    SLOT(set_camera(unsigned)));

   QObject::connect(app_state(),
                    SIGNAL(sensor_pos_changed(unsigned)),
                    sensor_selector,
                    SLOT(set_sensor_pos(unsigned)));

   QObject::connect(app_state(),
                    SIGNAL(open_finished()),
                    sensor_selector,
                    SLOT(on_load()),
                    Qt::QueuedConnection);

   return sensor_selector;
}

// ---------------------------------------------------------- finish form layout
//
void finish_form_layout(QFormLayout* layout, const int width) noexcept
{
   for_each_qform_label(layout, [width](QWidget* wgt) {
      wgt->setMinimumWidth(width);
      dynamic_cast<QLabel*>(wgt)->setAlignment(Qt::AlignRight
                                               | Qt::AlignVCenter);
   });
}

} // namespace perceive
