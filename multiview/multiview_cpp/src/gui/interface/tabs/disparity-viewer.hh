
#pragma once

#include <memory>

#include <QImage>
#include <QWidget>

class ImageViewer2;

class DisparityViewer final : public QWidget
{
   Q_OBJECT

 private:
   struct Pimpl;
   Pimpl* pimpl_{nullptr};

 public:
   DisparityViewer(QWidget* parent = nullptr);
   DisparityViewer(const DisparityViewer&) = delete;
   DisparityViewer(DisparityViewer&&)      = delete;
   virtual ~DisparityViewer();

   DisparityViewer& operator=(const DisparityViewer&) = delete;
   DisparityViewer& operator=(DisparityViewer&&) = delete;

   void set_qimage(const std::shared_ptr<const QImage> qim);
   ImageViewer2* get_image_viewer() noexcept;

 signals:
   void data_updated(); // Must be caught as a queued connection

 public slots:
   void on_redraw(); //!< Force a redraw
   bool widgets_to_model();
   void model_to_widgets();
};
