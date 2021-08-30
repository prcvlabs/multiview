
#pragma once

#include <memory>

#include <QMainWindow>

class MainWindow final : public QMainWindow
{
   Q_OBJECT

 private:
   struct Pimpl;
   std::unique_ptr<Pimpl> pimpl_;

 public:
   MainWindow(QWidget* parent = nullptr);
   MainWindow(const MainWindow&) = delete;
   MainWindow(MainWindow&&)      = delete;
   virtual ~MainWindow();
   MainWindow& operator=(const MainWindow&) = delete;
   MainWindow& operator=(MainWindow&&) = delete;

 signals:

   void signal_taskmanager_done();
   void signal_video_frame_changed(int);

 public slots:

   // Application-state
   void save_state();
   void restore_state();

   // Synchronize model and state
   void widgets_to_model();
   void model_to_widgets();

   // The worker thread has finished a cycle
   void on_taskmanager_done();
   // void update_gui();

   // Video frame-number control
   void on_update_video_frame();

 private slots:

   void on_redraw(); // receives app-states open-finished...
   void on_open_finished();
   void on_update_frame_slider(int frame_no);
   void on_dev_mode_set(bool is_dev_mode); //

   // Action Slots
   void on_open_action();
   void on_quit_action();

   void on_switch_dev_mode();

   // Tab control
   void on_detach_tab();
   void on_attach_tab();
   void on_tab_changed();
   void on_next_tab_action();
   void on_prev_tab_action();

   bool eventFilter(QObject*, QEvent*) override;

 protected:
   void keyPressEvent(QKeyEvent*) override;
   void closeEvent(QCloseEvent*) override;
   void resizeEvent(QResizeEvent*) override;
};
