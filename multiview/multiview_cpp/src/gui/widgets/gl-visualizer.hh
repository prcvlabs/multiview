
#pragma once

#include <QColor>
#include <QOpenGLWidget>
#include <QPoint>

#include <functional>
#include <memory>

class GLVisualizer final : public QOpenGLWidget
{
   Q_OBJECT

 private:
   struct Pimpl;
   std::unique_ptr<Pimpl> pimpl_;

 private:
   void make_ui();

   GLVisualizer(const GLVisualizer&) = delete;
   void operator=(const GLVisualizer&) = delete;

   void emit_oneRevolutionCaptured();

 public:
   enum PanMode : int { MODE_XY_XZ = 0, MODE_HELICOPTER };

   GLVisualizer(QWidget* parent = nullptr);
   virtual ~GLVisualizer();

   QSize minimumSizeHint() const override;
   QSize sizeHint() const override;

   std::function<void()> onDrawThunk;
   std::function<void()> onDrawThunk2;
   bool draw_gl_axis;
   bool draw_3d_cube;
   bool capture_frames;
   bool rotating;
   double helicopter_rotation_inclination;
   PanMode pan_mode = MODE_HELICOPTER;

   void print() const;

 public slots:

   void setXRotation(int angle);
   void setYRotation(int angle);
   void setZRotation(int angle);
   void setZoom(double scale);

   void zoomIn();
   void zoomOut();
   double getZoom();

   void setScale(double dx, double dy, double dz);

   void setRotTheta(double rot = std::numeric_limits<double>::quiet_NaN());

   void setCapture(bool capture, int frame_counter_begin = 0);
   void captureOneRevolution();

   void setDrawAxis(bool draw);
   void setDraw3dCube(bool draw);
   void setRotating(bool rotate);
   void setRotateDelta(double delta);
   void setRotateUp(double x, double y, double z);

   bool isRotating() const;
   bool isCapturing() const;

   void setRotationSpeed(double secs_per_revolusion);

   // Rotation spirals above target.
   // If 0.0, then rotation orbits the target.
   void setHelicopterRotateInclination(double value);

   void setEye(double x, double y, double z);
   void setCentre(double x, double y, double z);

 signals:

   void xRotationChanged(int angle);
   void yRotationChanged(int angle);
   void zRotationChanged(int angle);
   void zoomChanged(double scale);
   void oneFrameCaptured();
   void oneRevolutionCaptured();

 protected:
   void initializeGL() override;
   void paintGL() override;
   void resizeGL(int width, int height) override;

   virtual void mousePressEvent(QMouseEvent* event) override;
   virtual void mouseReleaseEvent(QMouseEvent* event) override;
   virtual void mouseMoveEvent(QMouseEvent* event) override;
   virtual void keyPressEvent(QKeyEvent* event) override;
   virtual void wheelEvent(QWheelEvent* event) override;
};
