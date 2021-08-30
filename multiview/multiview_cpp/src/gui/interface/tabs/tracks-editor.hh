
#pragma once

#include <memory>

#include <QImage>
#include <QWidget>

class ImageViewer2;
class AppState;

//
// + The Pimpl stores a tracks-output internally
// + Tracks output is set when the scene is loaded
// + Saving always saves to the same filename %s/annotation-output%s.json
//   - %s is the output directory (in cli-args)
//   - %s is a suffix to ensure that an annotation isn't overwritten
// + Update the way tracks are rendered.
// * You can select tracks and: [delete, drag, arrow-key-move, set-id]
// * You can create a new track
// * All tracks are rendered directly onto frame videos
//
class TracksEditor final : public QWidget
{
   Q_OBJECT

 private:
   struct Pimpl;
   Pimpl* pimpl_{nullptr};

 public:
   TracksEditor(QWidget* parent);
   TracksEditor(const TracksEditor&) = delete;
   TracksEditor(TracksEditor&&)      = delete;
   virtual ~TracksEditor();

   TracksEditor& operator=(const TracksEditor&) = delete;
   TracksEditor& operator=(TracksEditor&&) = delete;

   bool handle_key_pressed(QKeyEvent*); // return TRUE iff the event was handled

 signals:
   void data_updated(); // Must be caught as a queued connection

 public slots:
   void on_redraw(); //!< Force a redraw
   bool widgets_to_model();
   void model_to_widgets();
   void on_check_tracks_file();
   void on_save_tracks_file();
   void on_publish_ground_truth();

 private slots:
   void init_tracker_file(); // called when app-state.open_finished
   void on_video_frame_changed(int frame_no);
   void finish_ui();
   void on_camera_changed();

   void on_btn_hist_rot_left();
   void on_btn_hist_rot_right();
   void on_btn_hist_zoom_in();
   void on_btn_hist_zoom_out();

   void on_btn_arrow_up();
   void on_btn_arrow_down();
   void on_btn_arrow_left();
   void on_btn_arrow_right();
   void on_btn_trail();

 protected:
   void keyPressEvent(QKeyEvent*) override;
};
