
#pragma once

#include <memory>

#include <QImage>
#include <QWidget>

class ImageViewer2;
class AppState;

class TracksViewer final : public QWidget
{
   Q_OBJECT

 private:
   struct Pimpl;
   Pimpl* pimpl_{nullptr};

 public:
   TracksViewer(QWidget* parent = nullptr);
   TracksViewer(const TracksViewer&) = delete;
   TracksViewer(TracksViewer&&)      = delete;
   virtual ~TracksViewer();

   TracksViewer& operator=(const TracksViewer&) = delete;
   TracksViewer& operator=(TracksViewer&&) = delete;

 signals:
   void data_updated(); // Must be caught as a queued connection

 public slots:
   void on_redraw(); //!< Force a redraw
   bool widgets_to_model();
   void model_to_widgets();

 private slots:
   void on_rb_clicked();
   void on_generate_all_tracks();
};
