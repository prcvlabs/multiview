
#pragma once

#include <functional>
#include <string>

#include <QLabel>
#include <QWidget>

class SensorSelector : public QWidget
{
   Q_OBJECT

 private:
   struct Pimpl;
   Pimpl* pimpl_{nullptr};

 public:
   SensorSelector(QWidget* parent = nullptr);
   SensorSelector(const SensorSelector&) = delete;
   SensorSelector(SensorSelector&&)      = delete;
   virtual ~SensorSelector();

   SensorSelector& operator=(const SensorSelector&) = delete;
   SensorSelector& operator=(SensorSelector&&) = delete;

   std::pair<int, int> value() const noexcept;
   void set_value(std::pair<int, int> value) noexcept;
   void set_value(int camera, int sensor) noexcept;
   bool block_signals(bool val) noexcept;

   // Pass in the number of sensors for each camera
   void init_cameras(const std::vector<int>&) noexcept;

   void set_min_form_label_width(int width);

   // Callbacks
   std::function<std::string(int)> set_camera_label;
   std::function<std::string(int, int)> set_sensor_label;
   std::function<void(std::pair<int, int>)> value_changed_callback;
   std::function<void()> on_load_callback;

 signals:
   void value_changed(std::pair<int, int> value);

 public slots:
   void on_redraw() noexcept;
   void set_camera(unsigned) noexcept;
   void set_sensor_pos(unsigned) noexcept;
   void on_load() noexcept; // just calls 'on-load-callback'

 private slots:
   void on_value_camera_changed_();
   void on_value_sensor_changed_();
   void do_changed_();
};
