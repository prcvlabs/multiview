
#pragma once

#include <memory>

#include <QImage>
#include <QWidget>

class ImageViewer2;

class FrameViewer final : public QWidget
{
   Q_OBJECT

 private:
   struct Pimpl;
   Pimpl* pimpl_{nullptr};

 public:
   FrameViewer(QWidget* parent = nullptr);
   FrameViewer(const FrameViewer&) = delete;
   FrameViewer(FrameViewer&&)      = delete;
   virtual ~FrameViewer();

   FrameViewer& operator=(const FrameViewer&) = delete;
   FrameViewer& operator=(FrameViewer&&) = delete;

   // Getters/setters
   void set_qimage(const std::shared_ptr<const QImage> qim);
   ImageViewer2* get_image_viewer() noexcept;

   // int sensor_id() const noexcept; // That we're viewing
   // bool is_left() const noexcept;  // Sensor is a left-image

   enum view_e : int { DISTORTED_IMAGE = 0, STILL_IMAGE, RECTIFIED_IMAGE };
   view_e view() const noexcept; //
   void set_view(view_e) noexcept;

 signals:
   void data_updated(); // Must be caught as a queued connection

 public slots:
   void on_redraw(); //!< Force a redraw

 private slots:
};
