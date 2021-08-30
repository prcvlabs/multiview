
#include "stdinc.hpp"

#include "image-viewer-2.hh"

#include <QMouseEvent>
#include <QPaintEvent>
#include <QPainter>
#include <QTimer>
#include <QWheelEvent>

#define This ImageViewer2

using namespace perceive;

struct This::Pimpl
{
 public:
   ImageViewer2& parent;
   shared_ptr<const QImage> qim{nullptr};

   double zoom{1.0}; // 1.0 is 100%

   bool mouse_is_panning{false};
   bool mouse_tracking_true{false};
   Point2 mouse_down_pos{0, 0};
   Point2 offset{0, 0};
   bool is_pixel_indicator{false};
   Point2 pixel_indicator_pos{0, 0};
   bool draw_pixel_indicator{true};

   QTimer* timer{nullptr};

   Pimpl(ImageViewer2& parent_)
       : parent(parent_)
       , timer(new QTimer{&parent})
   {
      connect(
          timer, SIGNAL(timeout()), &parent, SLOT(on_process_clear_image()));
   }

   void start_pan(Point2 pos)
   {
      draw_pixel_indicator
          = is_pixel_indicator && (mouse_down_pos != (pos - offset));

      mouse_down_pos   = pos - offset;
      mouse_is_panning = true;
      parent.setMouseTracking(mouse_tracking_true);
      if(draw_pixel_indicator) {
         const auto x = to_pt2(parent.mouse_to_qim(pos));
         cout << format("PIXEL [{}, {}]", x.x, x.y) << endl;
         pixel_indicator_pos = x;
         parent.update();
      }
   }

   void stop_pan()
   {
      parent.setMouseTracking(mouse_tracking_true);
      mouse_is_panning = false;
      parent.update();
   }
};

static bool dummy_mouse_press_event(QMouseEvent* event) { return false; }

This::This(QWidget* parent)
    : QWidget(parent)
    , pimpl_(new Pimpl(*this))
{
   on_mouse_press   = dummy_mouse_press_event;
   on_mouse_move    = dummy_mouse_press_event;
   on_mouse_release = dummy_mouse_press_event;
}

This::~This() { delete pimpl_; }

// --------------------------------------------------------------------- Getters

double This::zoom() const { return pimpl_->zoom; }

Point2 This::offset() const { return pimpl_->offset; }

void This::set_offset(const Point2& pos)
{
   pimpl_->offset = pos;
   update();
}

Vector2 This::mouse_to_qim(const Point2& pos) const
{
   return (to_vec2(pos) - to_vec2(pimpl_->offset)) / pimpl_->zoom;
}

Vector2 This::qim_to_mouse(const Vector2& qim_pos) const
{
   return pimpl_->zoom * qim_pos + to_vec2(pimpl_->offset);
}

bool This::is_pixel_indicator() const noexcept
{
   return pimpl_->is_pixel_indicator;
}

void This::set_pixel_indicator(bool val) noexcept
{
   auto& P = *pimpl_;
   if(val) { // Enable the indicator
      P.is_pixel_indicator = true;
      P.parent.setMouseTracking(true);

   } else {
      P.parent.setMouseTracking(P.mouse_tracking_true);
   }
}

bool This::is_mouse_tracking() const noexcept
{
   return pimpl_->mouse_tracking_true;
}

void This::set_mouse_tracking(bool val) noexcept
{
   pimpl_->mouse_tracking_true = val;
   setMouseTracking(val);
}

// ------------------------------------------------------------------ set-qimage

void This::set_qimage(const shared_ptr<const QImage> qim,
                      const unsigned clear_image_delay_ms)
{
   if(qim == nullptr and clear_image_delay_ms > 0) {
      pimpl_->timer->setSingleShot(true);
      pimpl_->timer->setInterval(clear_image_delay_ms);
      pimpl_->timer->start();
   } else {
      pimpl_->timer->stop();
      pimpl_->qim = qim;
      update();
   }
}

// ----------------------------------------------------------- reset offset/zoom

void This::reset_offset_zoom()
{
   auto& P  = *pimpl_;
   P.zoom   = 1.0;
   P.offset = Point2(0, 0);
   update();
}

// ----------------------------------------------------------------------- mouse

void This::mousePressEvent(QMouseEvent* event)
{
   auto& P = *pimpl_;

   if(on_mouse_press && on_mouse_press(event)) return;

   if(P.qim == nullptr) return; // nothing to do

   const Point2 pos(event->x(), event->y());
   const bool is_left_btn = event->buttons() & Qt::LeftButton;
   const bool shift_down  = event->modifiers() & Qt::ShiftModifier;
   const bool ctrl_down   = event->modifiers() & Qt::ControlModifier;
   const bool alt_down    = event->modifiers() & Qt::AltModifier;
   const bool meta_down   = event->modifiers() & Qt::MetaModifier;
   const bool no_modifier
       = !shift_down && !ctrl_down && !alt_down && !meta_down;

   // Did we initiate a pan?
   if(is_left_btn && no_modifier && is_pannable) {
      P.start_pan(pos);
      return;
   }
}

void This::mouseMoveEvent(QMouseEvent* event)
{
   auto& P = *pimpl_;

   if(on_mouse_move(event)) return;
   if(!P.mouse_is_panning) return; // should never happen

   const Point2 pos(event->x(), event->y());
   const bool shift_down = event->modifiers() & Qt::ShiftModifier;
   const bool ctrl_down  = event->modifiers() & Qt::ControlModifier;
   const bool alt_down   = event->modifiers() & Qt::AltModifier;
   const bool meta_down  = event->modifiers() & Qt::MetaModifier;
   const bool no_modifier
       = !shift_down && !ctrl_down && !alt_down && !meta_down;

   if(!no_modifier)
      P.stop_pan();
   else
      P.offset = pos - P.mouse_down_pos;

   update();
}

void This::mouseReleaseEvent(QMouseEvent* event)
{
   auto& P = *pimpl_;
   if(on_mouse_release(event)) return;
   if(P.mouse_is_panning) P.stop_pan();
}

// ----------------------------------------------------------------------- wheel

void This::wheelEvent(QWheelEvent* event)
{
   auto& P = *pimpl_;

   if(!is_zoomable) return;

   // Make sure there is no panning going on
   P.stop_pan();

   const Point2 pos(event->x(), event->y());
   const bool ctrl_down = event->modifiers() & Qt::ControlModifier;
   const double delta   = event->delta(); // how must we've scrolled

   const double scale    = 0.05 / 120.0;
   const double min_zoom = 0.1;
   const double max_zoom = 10.0;

   const double old_zoom = P.zoom;
   const Vector2 old_pos = mouse_to_qim(pos);

   P.zoom *= (1.0 + delta * scale);
   if(P.zoom < min_zoom) P.zoom = min_zoom;
   if(P.zoom > max_zoom) P.zoom = max_zoom;

   if(false) {
      // INFO(format("Wheel: ctrl={:s}, delta={}, zoom={}",
      //             str(ctrl_down), delta, P.zoom));
   }

   // Figure out the new offset...
   if(old_zoom != P.zoom) {
      // Change offset such that 'mouse-to-image' gives 'old-pos'
      double off_x = double(pos.x) - (P.zoom * old_pos.x);
      double off_y = double(pos.y) - (P.zoom * old_pos.y);
      P.offset     = Point2(off_x + 0.49, off_y + 0.49);
   }

   // Update if necessary
   if(old_zoom != P.zoom) update();
}

// ----------------------------------------------------------------- paint-event

void This::paintEvent(QPaintEvent* event)
{
   auto& P        = *pimpl_;
   auto clip_rect = event->rect();

   shared_ptr<const QImage> qim{P.qim};

   if(qim == nullptr) return; // nothing to paint!

   QPainter painter(this);
   if(qim != nullptr) {
      int total_w = qim->width();
      int total_h = qim->height();

      // According to our zoom, what is the target rect?
      QRect target_rect(P.offset.x,
                        P.offset.y,
                        ceil(total_w * P.zoom),
                        ceil(total_h * P.zoom));

      // qim ready for action
      painter.drawImage(target_rect, *qim, qim->rect());

      if(P.draw_pixel_indicator) {
         const auto x = P.pixel_indicator_pos;
         const auto u = to_vec2(x) * P.zoom + to_vec2(P.offset);
         const int sz = std::max(1, int(floor(P.zoom)));

         const auto k = QColor(0, 255, 255, 255);
         painter.fillRect(u.x, u.y, sz, sz, QBrush(k));

         painter.setPen(k);
         painter.setFont(QFont("Arial", 11));
         painter.drawText(
             QPoint(u.x + 4 + sz, u.y + 4 + sz / 2),
             QString::fromUtf8(format("[{}, {}]", x.x, x.y).c_str()));
      }
   }

   if(post_paint_func) post_paint_func(event, painter);

   painter.end();
}

// ------------------------------------------------------ on-process-clear-image

void This::on_process_clear_image() noexcept { set_qimage(nullptr, 0); }
