
#pragma once

#include <memory>

#include <QImage>
#include <QWidget>

class ImageViewer2;

class FloorHistogramViewer final : public QWidget
{
   Q_OBJECT

 private:
   struct Pimpl;
   Pimpl* pimpl_{nullptr};

 public:
   FloorHistogramViewer(QWidget* parent = nullptr);
   FloorHistogramViewer(const FloorHistogramViewer&) = delete;
   FloorHistogramViewer(FloorHistogramViewer&&)      = delete;
   virtual ~FloorHistogramViewer();

   FloorHistogramViewer& operator=(const FloorHistogramViewer&) = delete;
   FloorHistogramViewer& operator=(FloorHistogramViewer&&) = delete;

   void set_qimage(const std::shared_ptr<const QImage> qim);
   ImageViewer2* get_image_viewer() noexcept;

 signals:
   void data_updated(); // Must be caught as a queued connection

 public slots:
   void on_redraw(); //!< Force a redraw
   bool widgets_to_model();
   void model_to_widgets();

 private slots:
   void on_rb_clicked();
   void on_regen_movie_stats_clicked();
};
