
#pragma once

#include <memory>

#include <QImage>
#include <QWidget>

#include "stdinc.hpp"

#include "perceive/geometry/vector.hpp"

class ImageViewer2 final : public QWidget
{
   Q_OBJECT

 private:
   struct Pimpl;
   Pimpl* pimpl_{nullptr};

 public:
   using Point2  = perceive::Point2;
   using Vector2 = perceive::Vector2;

   ImageViewer2(QWidget* parent = nullptr);
   ImageViewer2(const ImageViewer2&) = delete;
   ImageViewer2(ImageViewer2&&)      = delete;
   virtual ~ImageViewer2();

   ImageViewer2& operator=(const ImageViewer2&) = delete;
   ImageViewer2& operator=(ImageViewer2&&) = delete;

   // Sets the image. Will clear the image if 'nullptr' is passed.
   // If @clear_image_delay_ms is non-zero, then a QTimer is used
   // to delay setting the image. The timer is automatically canceled
   // if a non-null qim is passed in a subseqent call. This prevents
   // flicker.
   void set_qimage(const std::shared_ptr<const QImage> qim,
                   const unsigned clear_image_delay_ms = 200);

   // Reset offset and zoom
   void reset_offset_zoom();
   double zoom() const;   // current zoom
   Point2 offset() const; // current drag offset
   void set_offset(const Point2& pos);

   // converts a widget mouse pos into 'qim' image co-ordinates
   Vector2 mouse_to_qim(const Point2& pos) const;
   Vector2 qim_to_mouse(const Vector2& qim_pos) const;

   // If true, then can click and drag the image around
   bool is_pannable{true};
   bool is_zoomable{true};

   // Show the pixels under the cursor
   bool is_pixel_indicator() const noexcept;
   void set_pixel_indicator(bool val) noexcept;

   // If true, then mouse panning is set
   bool is_mouse_tracking() const noexcept;
   void set_mouse_tracking(bool val) noexcept;

   // Return TRUE from this function to prevent parent mouse-press-event firing
   std::function<bool(QMouseEvent* event)> on_mouse_press;
   std::function<bool(QMouseEvent* event)> on_mouse_move;
   std::function<bool(QMouseEvent* event)> on_mouse_release;
   std::function<void(QPaintEvent* event, QPainter& painter)> post_paint_func;

 protected:
   // Make sure you understand what the 'virtual' keyword does
   virtual void paintEvent(QPaintEvent* event);
   // virtual void mouseDoubleClickEvent(QMouseEvent * event);
   virtual void mouseMoveEvent(QMouseEvent* event);
   virtual void mousePressEvent(QMouseEvent* event);
   virtual void mouseReleaseEvent(QMouseEvent* event);
   virtual void wheelEvent(QWheelEvent* event);

 private slots:
   void on_process_clear_image() noexcept;
};

/**
 *
// Declare your widget
ImageViewer2 * viewer;

// This is how I initialized it
viewer = new ImageViewer2;
viewer->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

// Example of seting the function pointers
viewer->on_mouse_press = [=] (QMouseEvent * event) -> bool {
    // Logic for handling mouse press on image-viewer
    return false; // return true to prevent image viewer from processing press
};

viewer->post_paint_func = [=] (QPaintEvent * event, QPainter& painter) {
    painter.drawLine(0, 0, 30, 50);
};

// You will probably have some function "update-gui" that is called
// when you have a frame ready to draw.
// In that function...

   QImage  * qim = nullptr;

void update_gui()
{
   // this is the with and height of the frame you rendered
   int total_w, total_h;

   // Ensure the 'qim' is the same size as your frame
   if(qim == nullptr ||
       qim->width() != total_w || qim->height() != total_h) {
        delete qim;
        qim = new QImage(total_w, total_h, QImage::Format_ARGB32);
    }

    // Now block copy your data into the QImage
    for(int y = 0; y < total_h; ++y) {
       uint32_t * src = (uint32_t *) frame->row(y); // ARGB
       uint32_t * dst = (uint32_t *) qim->scanLine(y);
       for(uint x = 0; x < im.width; ++x) {
          dst[x] = (0xff000000 | src[x]);
       }
    }

    // Now save the image to make sure it is okay
    qim->save("frame.png");

    // Make sure qim "sticks around" after function finishes
    // For example, make 'qim' a global variable
    viewer->set_qimage(qim);
}

*/
