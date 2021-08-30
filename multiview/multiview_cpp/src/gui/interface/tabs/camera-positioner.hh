
#pragma once

#include <memory>

#include <QImage>
#include <QWidget>

class ImageViewer2;
class AppState;

class CameraPositioner final : public QWidget
{
   Q_OBJECT

 private:
   struct Pimpl;
   Pimpl* pimpl_{nullptr};

 public:
   CameraPositioner(QWidget* parent);
   CameraPositioner(const CameraPositioner&) = delete;
   CameraPositioner(CameraPositioner&&)      = delete;
   virtual ~CameraPositioner();

   CameraPositioner& operator=(const CameraPositioner&) = delete;
   CameraPositioner& operator=(CameraPositioner&&) = delete;

   bool handle_key_pressed(QKeyEvent*); // return TRUE iff the event was handled

 signals:
   void data_updated(); // Must be caught as a queued connection

 public slots:
   void on_redraw(); //!< Force a redraw
   bool widgets_to_model();
   void model_to_widgets();

   void on_optimize();
   void on_check();
   void on_load();
   void on_reset();
   void on_save();
   void on_revert();
   void on_publish();

 protected:
   void keyPressEvent(QKeyEvent*) override;
};
