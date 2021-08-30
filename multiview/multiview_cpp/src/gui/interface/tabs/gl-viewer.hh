
#pragma once

#include <memory>

#include <QImage>
#include <QWidget>

class ImageViewer2;

namespace perceive
{
struct GuiRenderOptions;
};

class GlViewer final : public QWidget
{
   Q_OBJECT

 private:
   struct Pimpl;
   Pimpl* pimpl_{nullptr};

 public:
   GlViewer(QWidget* parent = nullptr);
   GlViewer(const GlViewer&) = delete;
   GlViewer(GlViewer&&)      = delete;
   virtual ~GlViewer();

   GlViewer& operator=(const GlViewer&) = delete;
   GlViewer& operator=(GlViewer&&) = delete;

   void set_render_opts(const perceive::GuiRenderOptions& opts);
   const perceive::GuiRenderOptions& render_opts() const noexcept;

 signals:
   void data_updated(); // Must be caught as a queued connection

 public slots:
   void on_redraw(); //!< Force a redraw
   void model_to_widgets() noexcept;
   void widgets_to_model() noexcept;
   void set_cpanel_parts_visibility() noexcept;
   void update_widgets() noexcept; //!< Update (e.g.) the p2d combo box
   void handle_one_revolution_captured() noexcept;
   void handle_one_frame_captured() noexcept;
   void print_global_transform() noexcept;
   void update_aux_skeleton() noexcept;
};
