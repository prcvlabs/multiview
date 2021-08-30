
#include "stdinc.hpp"

#include "main-window.hh"

#include <deque>

#include <QAction>
#include <QApplication>
#include <QIcon>
#include <QKeyEvent>
#include <QKeySequence>
#include <QMenuBar>
#include <QSettings>
#include <QShortcut>
#include <QTabWidget>

#include "gui/app-state.hh"
#include "gui/gl/gui-render-options.hpp"
#include "gui/interface/tabs/camera-positioner.hh"
#include "gui/interface/tabs/disparity-viewer.hh"
#include "gui/interface/tabs/floor-histogram-viewer.hh"
#include "gui/interface/tabs/frame-viewer.hh"
#include "gui/interface/tabs/gl-viewer.hh"
#include "gui/interface/tabs/openpose-viewer.hh"
#include "gui/interface/tabs/skeleton-debug.hh"
#include "gui/interface/tabs/slic-viewer.hh"
#include "gui/interface/tabs/tracks-editor.hh"
#include "gui/interface/tabs/tracks-viewer.hh"
#include "gui/qt-helpers.hpp"
#include "gui/widgets/generic-q-event-filter.hh"
#include "perceive/utils/file-system.hpp"

#define This MainWindow

using namespace perceive;

// ----------------------------------------------------------------------- Pimpl

struct This::Pimpl
{
   MainWindow* main_window = nullptr;
   std::deque<QWidget*> detached_widgets;

   // Menus
   QMenuBar* menu_bar = nullptr;
   QMenu* file_menu   = nullptr;
   QMenu* view_menu   = nullptr;

   // Actions
   QAction* open_action            = nullptr;
   QAction* quit_action            = nullptr;
   QAction* prev_tab_action        = nullptr;
   QAction* next_tab_action        = nullptr;
   QAction* attach_tab_action      = nullptr;
   QAction* detach_tab_action      = nullptr;
   QAction* switch_dev_mode_action = nullptr;

   vector<QAction*> select_camera_actions;

   // Widgets
   QTabWidget* tab_widget        = nullptr;
   QLabel* frame_no_slider_label = nullptr;
   QSlider* frame_no_slider      = nullptr;

   // Tab-widgets
   GlViewer* gl_viewer                        = nullptr;
   FrameViewer* frame_viewer_widget           = nullptr;
   SlicViewer* slic_viewer_widget             = nullptr;
   DisparityViewer* disparity_viewer_widget   = nullptr;
   FloorHistogramViewer* floor_hist_widget    = nullptr;
   OpenposeViewer* openpose_viewer_widget     = nullptr;
   TracksViewer* tracks_viewer_widget         = nullptr;
   TracksEditor* tracks_editor_widget         = nullptr;
   CameraPositioner* camera_positioner_widget = nullptr;
   SkeletonDebug* skeleton_widget             = nullptr;

   bool ui_is_created = false;

   // Construction
   Pimpl(MainWindow* main_window_)
       : main_window(main_window_)
   {
      Expects(main_window != nullptr);
   }

   void make_ui();
   void make_actions();
   void make_menubar();

   void set_slider_label()
   {
      auto scene_desc = app_state()->scene_desc();
      if(scene_desc != nullptr) {
         const int val      = frame_no_slider->value();
         const int n_frames = scene_desc->n_frames();
         frame_no_slider_label->setText(
             format("{:4d}/{}", val, n_frames).c_str());
      } else {
         frame_no_slider_label->setText("no video");
      }
   }
};

// --------------------------------------------------------------------- make-ui

void This::Pimpl::make_ui()
{
   { // Set window icon
      main_window->setWindowTitle(to_qstr("multiview :: gui"));
   }

   auto make_slider_widget = [&]() { // main-slider with label
      frame_no_slider_label= new QLabel{};
      frame_no_slider_label->setFixedWidth(75); // pixels for label

      frame_no_slider      = new QSlider{};
      frame_no_slider->setTickInterval(15 * 10);
      frame_no_slider->setTickPosition(QSlider::TicksBelow);
      frame_no_slider->setRange(1, 1 + 100);
      frame_no_slider->setOrientation(Qt::Horizontal);

      auto layout          = new QHBoxLayout{};
      layout->addWidget(frame_no_slider_label);
      layout->addWidget(frame_no_slider);

      auto wgt             = new QWidget{};
      wgt->setLayout(layout);
      return wgt;
   };

   { // -- (*) -- individual tabs
      gl_viewer = new GlViewer{nullptr};
      gl_viewer->setObjectName("3d Visualizer");

      frame_viewer_widget = new FrameViewer{};
      frame_viewer_widget->setObjectName("Frame");

      slic_viewer_widget = new SlicViewer{};
      slic_viewer_widget->setObjectName("SLIC");

      disparity_viewer_widget = new DisparityViewer{};
      disparity_viewer_widget->setObjectName("Disparity");

      openpose_viewer_widget = new OpenposeViewer{};
      openpose_viewer_widget->setObjectName("Openpose");

      floor_hist_widget = new FloorHistogramViewer{};
      floor_hist_widget->setObjectName("Floor Hist");

      camera_positioner_widget = new CameraPositioner{main_window};
      camera_positioner_widget->setObjectName("Cam Positioner");

      tracks_viewer_widget = new TracksViewer{};
      tracks_viewer_widget->setObjectName("Tracks");

      tracks_editor_widget = new TracksEditor{main_window};
      tracks_editor_widget->setObjectName("Tracks Editor");

      skeleton_widget = new SkeletonDebug{};
      skeleton_widget->setObjectName("Skeleton Debug");
   }

   { // -- (*) -- the tab-widget
      tab_widget = new QTabWidget;
      tab_widget->setMovable(true);
      tab_widget->installEventFilter(main_window);

      auto add_it = [&](auto w) { tab_widget->addTab(w, w->objectName()); };
      add_it(frame_viewer_widget);
      add_it(camera_positioner_widget);
      add_it(slic_viewer_widget);
      add_it(disparity_viewer_widget);
      add_it(gl_viewer);
      add_it(openpose_viewer_widget);
      add_it(floor_hist_widget);
      add_it(tracks_editor_widget);

      {
         Expects(app_state());
         if(!app_state()->config().config.annotate_mode) {
            add_it(tracks_viewer_widget);
         } else {
            INFO("`--annotate-mode` is set, so NOT attaching "
                 "tracks-viewer-widget");
         }
      }
   }

   { // -- (*) -- pull everything together in main widget
      auto* layout = new QVBoxLayout;
      layout->addWidget(tab_widget);
      layout->addWidget(make_slider_widget());
      auto* wgt = new QWidget;
      wgt->setLayout(layout);
      main_window->setCentralWidget(wgt);
   }

   { // -- (*) -- Actions
      make_actions();
   }

   { // -- (*) -- menu bars
      make_menubar();
   }

   { // -- (*) -- Initialize
      set_slider_label();
      main_window->model_to_widgets();
      main_window->widgets_to_model(); // so everything is consistent always

      // The gl-viewer needs to know where 'aux' is...
      // gl_viewer->get_aux
      //     = [w = this->plane_opt_widget]() { return &w->aux_render_data(); };
   }

   // -- (*) -- Wiring
   connect(app_state(),
           SIGNAL(camera_changed(unsigned)),
           main_window,
           SLOT(on_redraw()),
           Qt::QueuedConnection);

   connect(app_state(),
           SIGNAL(open_finished()),
           main_window,
           SLOT(on_open_finished()),
           Qt::QueuedConnection);

   connect(main_window,
           SIGNAL(signal_video_frame_changed(int)),
           app_state(),
           SLOT(on_set_video_frame(int)));

   connect(frame_no_slider,
           SIGNAL(valueChanged(int)),
           main_window,
           SLOT(on_update_frame_slider(int)));

   connect(tab_widget,
           SIGNAL(currentChanged(int)),
           main_window,
           SLOT(on_tab_changed()));

   connect(main_window,
           SIGNAL(signal_taskmanager_done()),
           main_window,
           SLOT(on_taskmanager_done()),
           Qt::QueuedConnection);

   connect(app_state(),
           SIGNAL(dev_mode_set(bool)),
           main_window,
           SLOT(on_dev_mode_set(bool)),
           Qt::QueuedConnection);

   // -- (*) -- Finalize Tabs
   auto outfit_tab = [&](int idx) { // Add 'attach' to each child widget
      QWidget* wgt = tab_widget->widget(idx);

      QAction* action= new QAction(wgt);
      action->setShortcut(QKeySequence(QString("Ctrl+A")));
      wgt->addAction(action);
      connect(action, SIGNAL(triggered()), main_window, SLOT(on_attach_tab()));

      // Intercept close events -- so that tabs are reattached properly
      auto f       = [=](QObject* obj, QEvent* event) {
         if(event->type() == QEvent::Close) {
            event->ignore();
            main_window->on_attach_tab();
            return true;
         }
         return false;
      };
      wgt->installEventFilter(new GenericQEventFilter(wgt, f));
   };

   for(int idx = 0; idx < tab_widget->count(); ++idx) outfit_tab(idx);

   // Switch to "dev-mode" or "technician gui"
   main_window->on_dev_mode_set(app_state()->dev_mode());

   ui_is_created = true;
}

// ---------------------------------------------------------------- make actions

void This::Pimpl::make_actions()
{
   auto make_action = [&](const string_view name,
                          const vector<const char*> shortcuts,
                          const auto f) {
      auto action = new QAction(tr(name.data()), main_window);
      QList<QKeySequence> l;
      for(const auto s : shortcuts) l.append(QKeySequence(QString(s)));
      action->setShortcuts(l);
      // action->setStatusTip(tr(status_tip.data()));
      connect(action, &QAction::triggered, main_window, f);
      return action;
   };

   switch_dev_mode_action = make_action(
       "Switch Dev Mode", {"Ctrl+M"}, &MainWindow::on_switch_dev_mode);

   open_action = make_action("&Open", {"Ctrl+O"}, &MainWindow::on_open_action);
   quit_action = make_action("E&xit", {"Ctrl+Q"}, &MainWindow::on_quit_action);
   prev_tab_action = make_action(
       "Pre&v Tab", {"Alt+Left"}, &MainWindow::on_prev_tab_action);
   next_tab_action = make_action(
       "&Next Tab", {"Alt+Right"}, &MainWindow::on_next_tab_action);

   attach_tab_action = make_action(
       "Re&attach Tab", {"Ctrl+A", "Ctrl+11"}, &MainWindow::on_attach_tab);
   detach_tab_action = make_action(
       "&Detatch Tab", {"Ctrl+D", "Ctrl+12"}, &MainWindow::on_detach_tab);

   { // "Select Camera" actions
      auto make_tab_action = [&](int i, const string_view shortcut) {
         QAction* action
             = new QAction(format("Select Camera &{}", i).c_str(), main_window);
         action->setShortcut(QKeySequence(QString(shortcut.data())));
         main_window->addAction(action);
         return action;
      };

#define SELECT_CAMERA(i, f)                                         \
   {                                                                \
      QAction* action = make_tab_action(i, format("Ctrl+{}", i));   \
      select_camera_actions.push_back(action);                      \
      connect(action, SIGNAL(triggered()), app_state(), SLOT(f())); \
   }

      SELECT_CAMERA(1, set_to_camera0);
      SELECT_CAMERA(2, set_to_camera1);
      SELECT_CAMERA(3, set_to_camera2);
      SELECT_CAMERA(4, set_to_camera3);
      SELECT_CAMERA(5, set_to_camera4);
      SELECT_CAMERA(6, set_to_camera5);
      SELECT_CAMERA(7, set_to_camera6);
      SELECT_CAMERA(8, set_to_camera7);
      SELECT_CAMERA(9, set_to_camera8);
      SELECT_CAMERA(0, set_to_camera9);
#undef SELECT_CAMERA
   }
}

// ---------------------------------------------------------------- make menubar

void This::Pimpl::make_menubar()
{
   menu_bar = new QMenuBar(main_window);
   main_window->setMenuBar(menu_bar);

   file_menu = menu_bar->addMenu(tr("&File"));
   file_menu->addAction(open_action);
   file_menu->addSeparator();
   file_menu->addAction(quit_action);

   view_menu = menu_bar->addMenu(tr("&View"));
   view_menu->addAction(attach_tab_action);
   view_menu->addAction(detach_tab_action);
   view_menu->addSeparator();
   for(auto action : select_camera_actions) view_menu->addAction(action);
   view_menu->addSeparator();
   view_menu->addAction(switch_dev_mode_action);
}

// ---------------------------------------------------------------- Construction

This::This(QWidget* parent)
    : QMainWindow(parent)
    , pimpl_(make_unique<Pimpl>(this))
{
   pimpl_->make_ui();
   restore_state(); // Restore-state window state from user config
}

This::~This() = default;

// ------------------------------------------------------------ widgets to model

void This::widgets_to_model() {}

// ------------------------------------------------------------ model to widgets

void This::model_to_widgets() {}

// ------------------------------------------------------------------ save state

void This::save_state()
{
   int tab_idx = pimpl_->tab_widget->currentIndex();
   QSettings settings;
   if(!isMaximized() && !isFullScreen() && !isMinimized() && isVisible()) {
      settings.setValue("MULTIVIEW_GUI_3D__POS_main_viewer", saveGeometry());
      settings.setValue("MULTIVIEW_GUI_3D__STATE_main_viewer", saveState());
      settings.setValue("MULTIVIEW_GUI_3D__TAB_main_viewer", tab_idx);
      settings.setValue(
          "MULTIVIEW_GUI_render_opts_gl_viewer",
          to_qstr(pimpl_->gl_viewer->render_opts().to_json_string()));
   }
}

// --------------------------------------------------------------- restore state

void This::restore_state()
{
   QSettings settings;
   restoreGeometry(
       settings.value("MULTIVIEW_GUI_3D__POS_main_viewer").toByteArray());
   restoreState(
       settings.value("MULTIVIEW_GUI_3D__STATE_main_viewer").toByteArray());
   try {
      int n_tabs  = pimpl_->tab_widget->count();
      int tab_idx = settings.value("MULTIVIEW_GUI_3D__TAB_main_viewer").toInt();
      if(n_tabs > 0 && tab_idx >= 0)
         pimpl_->tab_widget->setCurrentIndex(tab_idx % n_tabs);
   } catch(...) {
      // Don't worry about it =0
   }

   try {
      string s = from_qstr(
          settings.value("MULTIVIEW_GUI_render_opts_gl_viewer").toString());
      GuiRenderOptions opts;
      read(opts, s);
      pimpl_->gl_viewer->set_render_opts(opts);
   } catch(...) {
      // Don't worry about it =0
   }
}

// --------------------------------------------------------- on taskmanager done

void This::on_taskmanager_done()
{
   // auto& app = AppData::instance();
   // app.tasks.taskmanager_done();
   // if(app.cmd_args.render_animation) {
   //    // We want to capture the animation before quitting.
   // } else if(app.cmd_args.process_and_quit) {
   //    this->close();
   // }
}

// ----------------------------------------------------------------- tab control
//
void This::on_next_tab_action()
{
   int idx = pimpl_->tab_widget->currentIndex() + 1;
   pimpl_->tab_widget->setCurrentIndex(idx % pimpl_->tab_widget->count());
}

void This::on_prev_tab_action()
{
   int idx = pimpl_->tab_widget->currentIndex() - 1;
   if(idx < 0) idx = pimpl_->tab_widget->count() - 1;
   pimpl_->tab_widget->setCurrentIndex(idx);
}

void This::on_detach_tab()
{
   if(pimpl_->tab_widget->count() <= 1) return;
   int idx = pimpl_->tab_widget->currentIndex();
   if(idx < 0 && idx >= pimpl_->tab_widget->count()) return; // out of bounds
   QWidget* wgt = pimpl_->tab_widget->widget(idx);
   if(wgt == nullptr) return;

   auto tab_text = pimpl_->tab_widget->tabText(idx);
   pimpl_->tab_widget->removeTab(pimpl_->tab_widget->currentIndex());
   wgt->setParent(nullptr);
   wgt->setVisible(true);
   wgt->setFocus();

   pimpl_->detached_widgets.push_back(wgt);
}

void This::on_attach_tab()
{
   while(pimpl_->detached_widgets.size() > 0) {
      auto* wgt = pimpl_->detached_widgets.front();
      pimpl_->detached_widgets.pop_front();
      pimpl_->tab_widget->addTab(wgt, wgt->objectName());
   }
}

void This::on_tab_changed()
{
   // Makes sure the the tab is updated
   on_redraw();
}

// ---------------------------------------------------------- on set video frame

void This::on_update_video_frame()
{
   auto& P           = *pimpl_;
   auto state        = app_state();
   auto scene_desc   = state->scene_desc();
   auto pipeline_ptr = state->frame_results();

   auto was_blocked = P.frame_no_slider->blockSignals(true);

   if(scene_desc == nullptr or pipeline_ptr == nullptr) {
      // nothing connected
      P.frame_no_slider->setRange(0, 0);
      P.frame_no_slider->setValue(0);
   } else {
      const int n_frames = scene_desc->n_frames();
      const int current_frame_no
          = pipeline_ptr->copy_sensor_images.params().frame_num;

      if(P.frame_no_slider->maximum() + 1 != n_frames)
         P.frame_no_slider->setRange(0, n_frames - 1);
      if(P.frame_no_slider->value() != current_frame_no)
         P.frame_no_slider->setValue(current_frame_no);

      on_redraw();
   }

   P.set_slider_label();
   P.frame_no_slider->blockSignals(was_blocked);
}

void This::on_update_frame_slider(int frame_no)
{
   auto& P           = *pimpl_;
   auto scene_desc   = app_state()->scene_desc();
   auto pipeline_ptr = app_state()->frame_results();

   if(scene_desc == nullptr or pipeline_ptr == nullptr) {
      // nothing connected
   } else {
      auto p = pipeline_ptr->copy_sensor_images.params();

      const int current_frame_no = p.frame_num;
      const int n_frames         = scene_desc->n_frames();

      const int val = P.frame_no_slider->value();

      if(val != current_frame_no and val >= 0 and val < n_frames) {
         P.set_slider_label();
         p.frame_num = val;
         pipeline_ptr->copy_sensor_images.set_params(p);
         on_redraw();
         emit signal_video_frame_changed(val);
      }
   }
}

// ------------------------------------------------------------ on open finished

void This::on_redraw()
{
   auto& P = *pimpl_;
   P.set_slider_label();

   QWidget* wgt = P.tab_widget->currentWidget();
   if(wgt == nullptr) return;

   if(wgt == P.frame_viewer_widget)
      P.frame_viewer_widget->on_redraw();
   else if(wgt == P.slic_viewer_widget)
      P.slic_viewer_widget->on_redraw();
   else if(wgt == P.disparity_viewer_widget)
      P.disparity_viewer_widget->on_redraw();
   else if(wgt == P.gl_viewer)
      P.gl_viewer->on_redraw();
   else if(wgt == P.floor_hist_widget)
      P.floor_hist_widget->on_redraw();
   else if(wgt == P.openpose_viewer_widget)
      P.openpose_viewer_widget->on_redraw();
   else if(wgt == P.tracks_viewer_widget)
      P.tracks_viewer_widget->on_redraw();
   else if(wgt == P.camera_positioner_widget)
      P.camera_positioner_widget->on_redraw();
   else if(wgt == P.tracks_editor_widget)
      P.tracks_editor_widget->on_redraw();
   else if(wgt == P.skeleton_widget)
      P.skeleton_widget->on_redraw();
   else
      WARN(format("tab widget does not have entry in 'on-update-tab'"));
}

// ------------------------------------------------------------ on open finished

void This::on_open_finished()
{
   on_update_video_frame();
   on_redraw();
}

// ---------------------------------------------------------------- event filter

bool This::eventFilter(QObject* obj, QEvent* ev)
{
   auto& P = *pimpl_;
   if(!P.ui_is_created) return false;

   QWidget* wgt = P.tab_widget->currentWidget();
   if(ev->type() == QEvent::KeyPress) {
      QKeyEvent* key_event = static_cast<QKeyEvent*>(ev);
      if(wgt == P.camera_positioner_widget)
         return P.camera_positioner_widget->handle_key_pressed(key_event);
      else if(wgt == P.tracks_editor_widget) {
         return P.tracks_editor_widget->handle_key_pressed(key_event);
      }
   }
   return false;
}

// -------------------------------------------------------------- keypress event

void This::keyPressEvent(QKeyEvent* ev) { eventFilter(this, ev); }

// ----------------------------------------------------------------- close event

void This::closeEvent(QCloseEvent* event) { on_quit_action(); }

// ---------------------------------------------------------------- resize event

void This::resizeEvent(QResizeEvent*) {}

// --------------------------------------------------------------------- actions

void This::on_open_action() { INFO(format("open action")); }

void This::on_quit_action()
{
   app_state()->set_quit_flag();
   if(app_state()->frame_results() != nullptr) {
      const string params_fname = format("{:s}/params-at-quit.json",
                                         app_state()->config().config.outdir);
      {
         std::stringstream ss{""};
         ss << app_state()->frame_results()->params_to_json() << endl;
         file_put_contents(params_fname, ss.str());
      }
   }
   save_state();
   QApplication::quit();
}

void This::on_switch_dev_mode()
{
   app_state()->set_dev_mode(!app_state()->dev_mode());
}

void This::on_dev_mode_set(bool is_dev_mode)
{
   INFO(format("SWITCHING DEV MODE to {:s}", str(is_dev_mode)));
}
