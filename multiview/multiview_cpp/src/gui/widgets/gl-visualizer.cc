
#include "gl-visualizer.hh"

#include "gl/gl-utils.hpp"
#include "perceive/geometry/vector-3.hpp"
#include "perceive/utils/file-system.hpp"

#include <sys/stat.h>
#include <sys/time.h>
#include <unistd.h>

#include <QMouseEvent>
#include <QOpenGLPaintDevice>
#include <QOpenGLWidget>
#include <QPixmap>
#include <QTimer>

#include <GL/gl.h>
#include <GL/glu.h>

#include <Eigen/Dense>

#define This GLVisualizer

using Eigen::Vector3d;
using std::string;
using namespace perceive;

static void glColour(const Vector3& k) { glColor3d(k.x, k.y, k.z); }

static void glColour(uint32_t k)
{
   glColor3d(((k >> 16) & 0xff) / 255.0,
             ((k >> 8) & 0xff) / 255.0,
             ((k >> 0) & 0xff) / 255.0);
}
static void glVertex(const Vector3& X) { glVertex3dv(X.ptr()); }

static int64_t milli_clock()
{
   struct timeval tv;
   gettimeofday(&tv, NULL);
   return int64_t(1000) * int64_t(tv.tv_sec) + int64_t(tv.tv_usec) / 1000;
}

struct This::Pimpl
{
   Pimpl(This* parent_)
       : parent(parent_)
       , capture_counter(0)
       , animationTimer(new QTimer)
       , xRot(0)
       , yRot(0)
       , zRot(0)
       , lastPos()
       , translate(0, 0, 0)
       , scale(1, 1, 1)
       , fov(25.0)
       , znear(0.5)
       , zfar(5000.0)
       , eye(-5, 0, 5)
       , centre(0, 0, 0)
       , up(0, 0, 1)
       , delta(0.0)
       , camera_speed(64.0)
       , // 64s for one revolution
       time0(0)
       , last_delta(0)
       , did_capture(false)
       , capture_one_rev(false)
   {}

   This* parent;

   uint capture_counter;

   QTimer* animationTimer;

   // Mouse control changes viewing angle
   int xRot;
   int yRot;
   int zRot;
   QPoint lastPos;
   bool mouseIsDown = false;

   // Translate and scale object
   Vector3d translate;
   Vector3d scale; // in the x, y, and z dimensions

   double fov; // options().info.fov() * 180.0 / M_PI;
   double znear;
   double zfar;

   Vector3d eye, centre, up{0.0, 0.0, 1.0};

   Vector3d centre_delta{0.0, 0.0, 0.0};
   double theta_delta{0.0};
   double helicopter_theta_delta{0.0};

   double rot_theta{NAN};

   double delta;
   double camera_speed; // 1/Hz... i.e, num of seconds per complete revolution
   int64_t time0;
   int64_t last_delta;

   bool did_capture;
   bool capture_one_rev;

   double radius() const { return (eye - centre).norm(); }
   void set_radius(double r)
   {
      Vector3d n = eye - centre;
      eye        = centre + r * n / n.norm();
   }

   void set_up(double x, double y, double z)
   {
      double norm = sqrt(x * x + y * y + z * z);
      up(0)       = x / norm;
      up(1)       = y / norm;
      up(2)       = z / norm;
   }

   double calc_delta()
   {
      auto time_now            = milli_clock();
      last_delta               = time_now - time0;
      const double ellapsed_ms = last_delta;
      const double speed       = camera_speed; // 4s for one revolution
      delta                    = fmod(ellapsed_ms * 0.001, speed) / speed;
      assert(delta >= 0.0 && delta <= 1.0);

      if(capture_one_rev && ellapsed_ms * 0.001 >= camera_speed)
         parent->emit_oneRevolutionCaptured();

      return delta;
   }
};

void glVertex3(const Vector3d& X) { glVertex3d(X(0), X(1), X(2)); }

void glDrawAxis(double line_len = 2.0, double line_width = 4.0)
{
   glPushMatrix();
   glScaled(line_len, line_len, line_len);
   Vector3d axis_origin(0.0, 0.0, 0.0);
   glLineWidth(line_width);
   glBegin(GL_LINES);
   glColor3d(1, 0, 0);
   glVertex3(axis_origin);
   glVertex3(axis_origin + Vector3d(1.0, 0.0, 0.0));
   glColor3d(0, 1, 0);
   glVertex3(axis_origin);
   glVertex3(axis_origin + Vector3d(0.0, 1.0, 0.0));
   glColor3d(0, 0, 1);
   glVertex3(axis_origin);
   glVertex3(axis_origin + Vector3d(0.0, 0.0, 1.0));
   glEnd();
   glLineWidth(1.0);
   glPopMatrix();
}

void glDrawNiceCube()
{
   glBegin(GL_QUADS);              // Draw A Quad
   glColor3f(0.0f, 1.0f, 0.0f);    // Set The Color To Green
   glVertex3f(1.0f, 1.0f, -1.0f);  // Top Right Of The Quad (Top)
   glVertex3f(-1.0f, 1.0f, -1.0f); // Top Left Of The Quad (Top)
   glVertex3f(-1.0f, 1.0f, 1.0f);  // Bottom Left Of The Quad (Top)
   glVertex3f(1.0f, 1.0f, 1.0f);   // Bottom Right Of The Quad (Top)

   glColor3f(1.0f, 0.5f, 0.0f);     // Set The Color To Orange
   glVertex3f(1.0f, -1.0f, 1.0f);   // Top Right Of The Quad (Botm)
   glVertex3f(-1.0f, -1.0f, 1.0f);  // Top Left Of The Quad (Botm)
   glVertex3f(-1.0f, -1.0f, -1.0f); // Bottom Left Of The Quad (Botm)
   glVertex3f(1.0f, -1.0f, -1.0f);  // Bottom Right Of The Quad (Botm)

   glColor3f(1.0f, 0.0f, 0.0f);    // Set The Color To Red
   glVertex3f(1.0f, 1.0f, 1.0f);   // Top Right Of The Quad (Front)
   glVertex3f(-1.0f, 1.0f, 1.0f);  // Top Left Of The Quad (Front)
   glVertex3f(-1.0f, -1.0f, 1.0f); // Bottom Left Of The Quad (Front)
   glVertex3f(1.0f, -1.0f, 1.0f);  // Bottom Right Of The Quad (Front)

   glColor3f(1.0f, 1.0f, 0.0f);     // Set The Color To Yellow
   glVertex3f(1.0f, -1.0f, -1.0f);  // Bottom Left Of The Quad (Back)
   glVertex3f(-1.0f, -1.0f, -1.0f); // Bottom Right Of The Quad (Back)
   glVertex3f(-1.0f, 1.0f, -1.0f);  // Top Right Of The Quad (Back)
   glVertex3f(1.0f, 1.0f, -1.0f);   // Top Left Of The Quad (Back)

   glColor3f(0.0f, 0.0f, 1.0f);     // Set The Color To Blue
   glVertex3f(-1.0f, 1.0f, 1.0f);   // Top Right Of The Quad (Left)
   glVertex3f(-1.0f, 1.0f, -1.0f);  // Top Left Of The Quad (Left)
   glVertex3f(-1.0f, -1.0f, -1.0f); // Bottom Left Of The Quad (Left)
   glVertex3f(-1.0f, -1.0f, 1.0f);  // Bottom Right Of The Quad (Left)

   glColor3f(1.0f, 0.0f, 1.0f);    // Set The Color To Violet
   glVertex3f(1.0f, 1.0f, -1.0f);  // Top Right Of The Quad (Right)
   glVertex3f(1.0f, 1.0f, 1.0f);   // Top Left Of The Quad (Right)
   glVertex3f(1.0f, -1.0f, 1.0f);  // Bottom Left Of The Quad (Right)
   glVertex3f(1.0f, -1.0f, -1.0f); // Bottom Right Of The Quad (Right)
   glEnd();                        // Done Drawing The Quad
}

inline double deg2rad(double deg) { return deg * M_PI / 180.0; }

This::This(QWidget* parent)
    : QOpenGLWidget(parent)
    , pimpl_(make_unique<Pimpl>(this))
{
   onDrawThunk                     = []() {};
   draw_gl_axis                    = false;
   draw_3d_cube                    = false;
   capture_frames                  = false;
   rotating                        = false;
   helicopter_rotation_inclination = 0.0;

   make_ui();

   auto animationTimer = pimpl_->animationTimer;
   animationTimer->setSingleShot(false);
   connect(animationTimer, SIGNAL(timeout()), this, SLOT(update()));
   animationTimer->start(1000.0 / 33.0); // i.e. 33 frames per second
   setRotating(false);
}

This::~This()
{
   if(pimpl_->did_capture)
      INFO("convert -delay 10 -loop 0 /tmp/gl-capture/*.png "
           "$HOME/animaion.gif");
}

QSize This::minimumSizeHint() const { return QSize(50, 50); }

QSize This::sizeHint() const { return QSize(400, 400); }

void This::make_ui() {}

static void qNormalizeAngle(int& angle)
{
   while(angle < 0) angle += 360 * 16;
   while(angle > 360 * 16) angle -= 360 * 16;
}

bool This::isRotating() const { return rotating; }

bool This::isCapturing() const { return capture_frames; }

void This::setXRotation(int angle)
{
   qNormalizeAngle(angle);
   if(angle != pimpl_->xRot) {
      pimpl_->xRot = angle;
      emit xRotationChanged(angle);
   }
}

void This::setYRotation(int angle)
{
   qNormalizeAngle(angle);
   if(angle != pimpl_->yRot) {
      pimpl_->yRot = angle;
      emit yRotationChanged(angle);
   }
}

void This::setZRotation(int angle)
{
   qNormalizeAngle(angle);
   if(angle != pimpl_->zRot) {
      pimpl_->zRot = angle;
      emit zRotationChanged(angle);
   }
}

void This::setZoom(double scalev)
{
   const double min_scale = 0.001;
   const double max_scale = 1000.0;
   if(scalev < min_scale) {
      printf("WARNING: scale (%g) less than floor (%g), using floor\n",
             scalev,
             min_scale);
      scalev = min_scale;
   }
   if(scalev > max_scale) {
      printf("WARNING: scale (%g) greater than ceil (%g), using ceil\n",
             scalev,
             max_scale);
      scalev = max_scale;
   }
   pimpl_->scale = (pimpl_->scale / pimpl_->scale.norm()) * scalev;

   emit(zoomChanged(scalev));
}

void This::setScale(double dx, double dy, double dz)
{
   pimpl_->scale(0) = dx;
   pimpl_->scale(1) = dy;
   pimpl_->scale(2) = dz;
}

void This::setRotTheta(double rot) { pimpl_->rot_theta = rot; }

void This::setCapture(bool capture, int frame_counter_begin)
{
   if(capture && !capture_frames) pimpl_->capture_counter = frame_counter_begin;
   capture_frames = capture;
}

void This::setDrawAxis(bool draw) { draw_gl_axis = draw; }

void This::setDraw3dCube(bool draw) { draw_3d_cube = draw; }

void This::setRotating(bool rotate)
{
   if(rotate && !rotating) pimpl_->time0 = milli_clock();
   rotating = rotate;
}

void This::setRotateDelta(double delta) { pimpl_->delta = delta; }

void This::setHelicopterRotateInclination(double value)
{
   helicopter_rotation_inclination = value;
}

void This::setRotateUp(double x, double y, double z)
{
   pimpl_->set_up(x, y, z);
}

void This::setRotationSpeed(double secs_per_revolusion)
{
   pimpl_->camera_speed = secs_per_revolusion;
}

void This::setEye(double x, double y, double z)
{
   pimpl_->eye(0) = x;
   pimpl_->eye(1) = y;
   pimpl_->eye(2) = z;
}

void This::setCentre(double x, double y, double z)
{
   auto& P     = *pimpl_;
   auto vec    = P.eye - P.centre;
   P.centre(0) = x;
   P.centre(1) = y;
   P.centre(2) = z;
   P.eye       = P.centre + vec;
}

void This::zoomIn() { setZoom(pimpl_->scale.norm() * 1.1); }

void This::zoomOut() { setZoom(pimpl_->scale.norm() * 0.9); }

double This::getZoom() { return pimpl_->scale.norm(); }

void This::captureOneRevolution()
{
   setRotating(true);
   setCapture(true);
   pimpl_->capture_one_rev = true;
}

void This::initializeGL()
{
   glShadeModel(GL_SMOOTH);
   glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
   glClearDepth(1.0f);
   glEnable(GL_DEPTH_TEST);
   glDepthFunc(GL_LEQUAL);
   glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);
   glHint(GL_POINT_SMOOTH_HINT, GL_NICEST);
   glEnable(GL_BLEND);
   glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
   glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_TRUE);
   glDisable(GL_CULL_FACE);
}

void This::paintGL()
{
   static bool first_run = true;
   if(first_run) {
      initializeGL();
      first_run = false;
   }

   glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

   glLoadIdentity();

   { // Locate the current eye position...
      Pimpl& P              = *pimpl_;
      const Vector3d centre = P.centre + P.centre_delta;
      const double radius   = (centre - P.eye).norm() * P.scale.norm();
      Vector3d at           = (centre - P.eye);
      at /= at.norm();
      Vector3d up = P.up;
      up /= up.norm();
      Vector3d yaxis  = at.cross(up);
      Vector3d xaxis  = up.cross(yaxis);
      Vector3d& zaxis = up;

      if(rotating) P.calc_delta();
      const double theta0 = (std::isfinite(P.rot_theta))
                                ? P.rot_theta
                                : 2.0 * (P.delta - 0.5) * M_PI; // 2pi radians
      const double theta  = theta0 + P.theta_delta;

      auto normalize_theta = [&](double theta) { // [-pi..pi)
         if(theta >= M_PI) return std::fmod(theta + M_PI, 2.0 * M_PI) - M_PI;
         if(theta < -M_PI) return std::fmod(theta - M_PI, 2.0 * M_PI) + M_PI;
         return theta;
      };

      const double inclination = std::fabs(normalize_theta(
          helicopter_rotation_inclination + P.helicopter_theta_delta));

      const double sin_inc = sin(inclination);
      const double cos_inc = cos(inclination);
      const double sin_azi = sin(theta);
      const double cos_azi = cos(theta);

      Vector3d eye = centre                               //
                     + xaxis * radius * sin_inc * cos_azi //
                     + yaxis * radius * sin_inc * sin_azi //
                     + zaxis * radius * cos_inc;          //

      if(true) {
         glu_look_at(eye(0) - centre(0),
                     eye(1) - centre(1),
                     eye(2) - centre(2),
                     0.0,
                     0.0,
                     0.0,
                     up(0),
                     up(1),
                     up(2));

         // Change the rotation, keeping the centre constant
         glRotated(double(pimpl_->xRot) / 16.0, 1.0, 0.0, 0.0);
         glRotated(double(pimpl_->yRot) / 16.0, 0.0, 1.0, 0.0);
         glRotated(double(pimpl_->zRot) / 16.0, 0.0, 0.0, 1.0);

         // Now translate "away" from the centre
         glTranslated(-centre(0), -centre(1), -centre(2));
      } else {
         glu_look_at(eye(0),
                     eye(1),
                     eye(2),
                     centre(0),
                     centre(1),
                     centre(2),
                     up(0),
                     up(1),
                     up(2));
      }
   }

   // -- Draw the axis
   if(draw_gl_axis) glDrawAxis();

   // Draw a quad -- testing
   if(draw_3d_cube) glDrawNiceCube();

   if(onDrawThunk) onDrawThunk();

   // Save a frame to disk (if requested)
   if(capture_frames) {
      const uint buflen = 2048;

      static uint capture_counter = 0;
      static int t0               = 0;
      static int frame_counter    = 0;
      static char buffer[buflen];
      static uint8_t* pixels = nullptr;
      static uint pixels_sz  = 0;

      capture_counter = (pimpl_->capture_counter)++;
      // convert -delay 10 -loop 0 inputfiles*.png animaion.gif

      string outdir = "/tmp/gl-capture";
      {
         if(!is_directory(outdir) && !is_readable_file(outdir)) {
            mkdir(outdir.c_str(), 0700);
         }

         // Process a screen capture
         auto sz       = size();
         auto w        = sz.width();
         auto h        = sz.height();
         uint n_pixels = 2 * 4 * w * h;
         if(pixels_sz < n_pixels) {
            pixels_sz = n_pixels;
            delete[] pixels;
            pixels = new uint8_t[pixels_sz];
         }

         glReadPixels(0, 0, w, h, GL_BGRA, GL_UNSIGNED_BYTE, pixels);

         // flip the y-axis of the output buffer
         uint bytes    = 4 * w;
         uint8_t* pix2 = pixels + 4 * w * h;
         for(int y = 0; y < h; ++y)
            memcpy(&pix2[(h - y - 1) * bytes], &pixels[y * bytes], bytes);

         snprintf(buffer, buflen, "/tmp/gl-capture/%06d.png", capture_counter);

         // Save the image
         QImage im(pix2, w, h, QImage::Format_ARGB32);
         im.save(buffer);

         printf("capture %s\n", buffer);

         pimpl_->did_capture = true;

         emit oneFrameCaptured();
      }
   }

   if(onDrawThunk2) onDrawThunk2();
}

void This::resizeGL(int width, int height)
{
   auto fov   = pimpl_->fov;
   auto znear = pimpl_->znear;
   auto zfar  = pimpl_->zfar;

   // Protect against a divide by zero
   if(height == 0) height = 1;

   // Setup our viewport.
   glViewport(0, 0, GLint(width), GLint(height));

   // change to the projection matrix and set our viewing volume.
   glMatrixMode(GL_PROJECTION);
   glLoadIdentity();
   glu_perspective(fov, float(width) / float(height), znear, zfar);

   // Make sure we're chaning the model view and not the projection
   glMatrixMode(GL_MODELVIEW);
   glLoadIdentity();
}

void This::mousePressEvent(QMouseEvent* event)
{
   pimpl_->lastPos     = event->pos();
   pimpl_->mouseIsDown = true;
}

void This::mouseReleaseEvent(QMouseEvent* event)
{
   pimpl_->mouseIsDown = false;
}

void This::mouseMoveEvent(QMouseEvent* event)
{
   auto& P = *pimpl_;

   const int dx = event->x() - pimpl_->lastPos.x();
   const int dy = event->y() - pimpl_->lastPos.y();

   const bool is_left_btn   = event->buttons() & Qt::LeftButton;
   const bool is_right_btn  = event->buttons() & Qt::RightButton;
   const bool is_shift_down = event->modifiers() & Qt::ShiftModifier;

   if(pan_mode == MODE_XY_XZ) {
      if(is_right_btn) {
         setXRotation(P.xRot + 8 * dy);
         setYRotation(P.yRot + 8 * dx);
      } else if(is_left_btn) {
         setXRotation(P.xRot + 8 * dy);
         setZRotation(P.zRot + 8 * dx);
      }
   } else {
      if(is_shift_down and is_left_btn) {
         P.centre_delta(0) += 0.02 * dx;
         P.centre_delta(1) += 0.02 * dy;
      } else if(is_shift_down and is_right_btn) {
         P.centre_delta(2) += 0.02 * dy;
      } else if(!is_shift_down and is_left_btn) {
         P.theta_delta += to_radians(0.1 * dx);
      } else if(!is_shift_down and is_right_btn) {
         P.helicopter_theta_delta += to_radians(0.1 * dy);
      }
   }

   pimpl_->lastPos = event->pos();
}

void This::keyPressEvent(QKeyEvent* event) {}

void This::wheelEvent(QWheelEvent* event)
{
   auto d_angle = event->angleDelta();
   if(d_angle.y() < 0) zoomIn();
   if(d_angle.y() > 0) zoomOut();
}

void This::emit_oneRevolutionCaptured()
{
   setCapture(false);
   setRotating(false);
   pimpl_->capture_one_rev = false;
   emit oneRevolutionCaptured();
}

void This::print() const
{
   const auto& P = *pimpl_;
   printf("Rot    = { %d, %d, %d }\n", P.xRot, P.yRot, P.zRot);
   printf("Centre = {%f, %f, %f}\n", P.centre(0), P.centre(1), P.centre(2));
   printf("Eye    = {%f, %f, %f}\n", P.eye(0), P.eye(1), P.eye(2));
   printf("Up     = {%f, %f, %f}\n", P.up(0), P.up(1), P.up(2));
   printf("Scale  = {%f, %f, %f}\n", P.scale(0), P.scale(1), P.scale(2));
   printf("Transl = {%f, %f, %f}\n",
          P.translate(0),
          P.translate(1),
          P.translate(2));
   printf("\n\n");
}
