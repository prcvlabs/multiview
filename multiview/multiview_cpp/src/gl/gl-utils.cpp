
#include "gl-error.hpp"
#include "gl-utils.hpp"
#include "platform/glew-bridge.hpp"

#include "perceive/foundation.hpp"
#include "perceive/geometry/rotation.hpp"
#include "perceive/geometry/vector.hpp"
#include "perceive/graphics/colour-set.hpp"
#include "perceive/graphics/image-container.hpp"
#include "perceive/graphics/tiny-string.hpp"
#include "perceive/utils/spin-lock.hpp"
#include "perceive/utils/threads.hpp"

#include <GL/gl.h>
// #include <GL/glu.h>

namespace perceive
{
// --------------------------------------------------------------- Gl Print Info

void gl_print_info()
{
   const GLubyte* renderer = glGetString(GL_RENDERER);
   const GLubyte* version  = glGetString(GL_VERSION);
   printf("Renderer: %s\n", renderer);
   printf("OpenGL version %s\n", version);
}

// -------------------------------------------------------------- Stock Gl Setup

void stock_gl_setup(unsigned viewport_w, unsigned viewport_h)
{
   check_and_warn_on_gl_error("before stock-gl-setup");

   // ---- GL Setup ----
   glEnable(GL_BLEND);
   glEnable(GL_POLYGON_SMOOTH);
   glEnable(GL_DEPTH_TEST);
   glEnable(GL_CULL_FACE);
   glEnable(GL_NORMALIZE);
   glEnable(GL_LIGHT0);

   glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_TRUE);

   glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);
   glHint(GL_POINT_SMOOTH_HINT, GL_NICEST);
   glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
   glHint(GL_POLYGON_SMOOTH_HINT, GL_NICEST);

   glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

   glShadeModel(GL_SMOOTH);
   glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
   glDepthFunc(GL_LESS);

   glViewport(0, 0, GLsizei(viewport_w), GLsizei(viewport_h));

   glPixelStorei(GL_UNPACK_ALIGNMENT, 4);
   glPixelStorei(GL_PACK_ALIGNMENT, 4);

   check_and_warn_on_gl_error("after stock-gl-setup");
}

// ------------------------------------------------------------------- GluLookAt

static void normalize(float v[3])
{
   float r;

   r = std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
   if(r == 0.0f) return;

   v[0] /= r;
   v[1] /= r;
   v[2] /= r;
}

static void cross(float v1[3], float v2[3], float result[3])
{
   result[0] = v1[1] * v2[2] - v1[2] * v2[1];
   result[1] = v1[2] * v2[0] - v1[0] * v2[2];
   result[2] = v1[0] * v2[1] - v1[1] * v2[0];
}

static void __gluMakeIdentityf(GLfloat m[16])
{
   m[0 + 4 * 0] = 1;
   m[0 + 4 * 1] = 0;
   m[0 + 4 * 2] = 0;
   m[0 + 4 * 3] = 0;
   m[1 + 4 * 0] = 0;
   m[1 + 4 * 1] = 1;
   m[1 + 4 * 2] = 0;
   m[1 + 4 * 3] = 0;
   m[2 + 4 * 0] = 0;
   m[2 + 4 * 1] = 0;
   m[2 + 4 * 2] = 1;
   m[2 + 4 * 3] = 0;
   m[3 + 4 * 0] = 0;
   m[3 + 4 * 1] = 0;
   m[3 + 4 * 2] = 0;
   m[3 + 4 * 3] = 1;
}

static void __gluMakeIdentityd(GLdouble m[16])
{
   m[0 + 4 * 0] = 1;
   m[0 + 4 * 1] = 0;
   m[0 + 4 * 2] = 0;
   m[0 + 4 * 3] = 0;
   m[1 + 4 * 0] = 0;
   m[1 + 4 * 1] = 1;
   m[1 + 4 * 2] = 0;
   m[1 + 4 * 3] = 0;
   m[2 + 4 * 0] = 0;
   m[2 + 4 * 1] = 0;
   m[2 + 4 * 2] = 1;
   m[2 + 4 * 3] = 0;
   m[3 + 4 * 0] = 0;
   m[3 + 4 * 1] = 0;
   m[3 + 4 * 2] = 0;
   m[3 + 4 * 3] = 1;
}

void glu_look_at(GLdouble eyex,
                 GLdouble eyey,
                 GLdouble eyez,
                 GLdouble centerx,
                 GLdouble centery,
                 GLdouble centerz,
                 GLdouble upx,
                 GLdouble upy,
                 GLdouble upz)
{
   float forward[3], side[3], up[3];
   GLfloat m[4][4];

   forward[0] = float(centerx - eyex);
   forward[1] = float(centery - eyey);
   forward[2] = float(centerz - eyez);

   up[0] = float(upx);
   up[1] = float(upy);
   up[2] = float(upz);

   normalize(forward);

   /* Side = forward x up */
   cross(forward, up, side);
   normalize(side);

   /* Recompute up as: up = side x forward */
   cross(side, forward, up);

   __gluMakeIdentityf(&m[0][0]);
   m[0][0] = side[0];
   m[1][0] = side[1];
   m[2][0] = side[2];

   m[0][1] = up[0];
   m[1][1] = up[1];
   m[2][1] = up[2];

   m[0][2] = -forward[0];
   m[1][2] = -forward[1];
   m[2][2] = -forward[2];

   glMultMatrixf(&m[0][0]);
   glTranslated(-eyex, -eyey, -eyez);
}

void glu_perspective(double fovy, double aspect, double zNear, double zFar)
{
   GLdouble m[4][4];
   double sine, cotangent, deltaZ;
   double radians = fovy / 2 * M_PI / 180;

   deltaZ = zFar - zNear;
   sine   = sin(radians);
   if((deltaZ == 0) || (sine == 0) || (aspect == 0)) { return; }
   cotangent = cos(radians) / sine;

   __gluMakeIdentityd(&m[0][0]);
   m[0][0] = cotangent / aspect;
   m[1][1] = cotangent;
   m[2][2] = -(zFar + zNear) / deltaZ;
   m[2][3] = -1;
   m[3][2] = -2 * zNear * zFar / deltaZ;
   m[3][3] = 0;
   glMultMatrixd(&m[0][0]);
}

// -------------------------------------------------------- shader program specs

Point3 glsl_workgroup_count()
{ // max global (total) work group size
   Point3 x;
   glGetIntegeri_v(GL_MAX_COMPUTE_WORK_GROUP_COUNT, 0, &x[0]);
   glGetIntegeri_v(GL_MAX_COMPUTE_WORK_GROUP_COUNT, 1, &x[1]);
   glGetIntegeri_v(GL_MAX_COMPUTE_WORK_GROUP_COUNT, 2, &x[2]);
   return x;
}

Point3 glsl_workgroup_size()
{ // max local (in one shader) work group sizes
   Point3 x;
   glGetIntegeri_v(GL_MAX_COMPUTE_WORK_GROUP_SIZE, 0, &x[0]);
   glGetIntegeri_v(GL_MAX_COMPUTE_WORK_GROUP_SIZE, 1, &x[1]);
   glGetIntegeri_v(GL_MAX_COMPUTE_WORK_GROUP_SIZE, 2, &x[2]);
   return x;
}

int glsl_workgroup_invocations()
{ // max local work group invocations
   GLint work_grp_inv = 0;
   glGetIntegerv(GL_MAX_COMPUTE_WORK_GROUP_INVOCATIONS, &work_grp_inv);
   return work_grp_inv;
}

int glsl_max_image_units()
{ // max local work group invocations
   GLint val = 0;
   glGetIntegerv(GL_MAX_IMAGE_UNITS, &val);
   return val;
}

// --------------------------------------------------------- Read Current Matrix
// The returned 'R' is row-major (GL is col-major)
void gl_read_modelview_matrix(Matrix4r& R) noexcept
{
   GLdouble M[16];
   glGetDoublev(GL_MODELVIEW_MATRIX, M);
   for(auto i = 0; i < 4; ++i)
      for(auto j = 0; j < 4; ++j) R(i, j) = M[i + j * 4];
}

void gl_read_projection_matrix(Matrix4r& R) noexcept
{
   GLdouble M[16];
   glGetDoublev(GL_PROJECTION_MATRIX, M);
   for(auto i = 0; i < 4; ++i)
      for(auto j = 0; j < 4; ++j) R(i, j) = M[i + j * 4];
}

void gl_mult_matrix(const Matrix4r& R) noexcept // R is row-major
{
   GLdouble M[16];
   for(auto i = 0; i < 4; ++i)
      for(auto j = 0; j < 4; ++j) M[i + j * 4] = R(i, j);
   glMultMatrixd(M);
}

// ----------------------------------------------- EuclideanTransform MultMatrix

void gl_mult_matrix(const EuclideanTransform& e) noexcept
{
   gl_mult_matrix(make_transform_matrix(e));
}

// ------------------------------------- Debug the current model/view/projection

void gl_debug_world_to_screen(const Vector3& W) noexcept
{
   Matrix4r P, M;
   gl_read_modelview_matrix(M);
   gl_read_projection_matrix(P);
   const auto Wr     = Vector4r(W(0), W(1), W(2), 1.0);
   const Vector4r Er = M * Wr;
   const Vector4r Cr = P * Er;
   const auto NDC    = Vector3r(Cr(0) / Cr(3), Cr(1) / Cr(3), Cr(2) / Cr(3));
   GLint m_viewport[4];
   glGetIntegerv(GL_VIEWPORT, m_viewport);
   const auto x = (NDC(0) + 1.0) * 0.5 * m_viewport[2] + m_viewport[0];
   const auto y = (NDC(1) + 1.0) * 0.5 * m_viewport[3] + m_viewport[1];
   cout << format("World  = {:s}", str(to_vec4(Wr))) << endl;
   cout << format("Eye    = {:s}", str(to_vec4(Er))) << endl;
   cout << format("Clip   = {:s}", str(to_vec4(Cr))) << endl;
   cout << format("NDC    = {:s}", str(to_vec3(NDC))) << endl;
   cout << format("screen = [{}, {}]", x, y) << endl;
   cout << endl;
}

// ------------------------------------------- Extract the current Camera Center

Vector3 extract_cam_center(const Matrix4r& R) noexcept
{
   Matrix4r R_inv = R.inverse();
   Vector3r X     = R_inv.block(0, 3, 3, 1); // Was amazing bug to slip through
   X /= R(3, 3);
   return to_vec3(X);
}

Vector3 extract_cam_center_from_current_model_view() noexcept
{
   Matrix4r R;
   gl_read_modelview_matrix(R);
   return extract_cam_center(R);
}

// -------------------------------------------------------------- Gl Render Text

Vector2 gl_render_text_sz(const string& s)
{
   const auto xoff = 2.0 / 5.0;
   auto width      = xoff + real(s.size()) * 2.0;
   auto height     = 6.0;
   return Vector2(width * 1.0 / 4.0, height * 1.0 / 6.0);
}

void gl_render_text(const string& s)
{
   static bool is_init      = false;
   static GLuint tex_id     = 0;
   static const int sz      = 16;
   static const int font_sz = 7;
   static const int scale   = 1;
   static ARGBImage bitmap;

   if(!is_init) {
      // Create the texture
      const auto side = sz * font_sz * scale;
      bitmap.resize(side, side, side);
      bitmap.fill(0xffffffffu);
      char s[2];
      s[1] = '\0';
      for(auto c = 1; c < 128; ++c) {
         s[0]           = char(c);
         const auto col = c % sz;
         const auto row = c / sz;
         const auto px  = scale * font_sz * col;
         const auto py  = scale * font_sz * row;
         const auto ox  = 0;
         const auto oy  = 1;
         uint32_t k     = 0x7f0000ffu;
         render_tiny(s, [&](int x, int y) {
            for(auto dx = 0; dx < scale; ++dx)
               for(auto dy = 0; dy < scale; ++dy)
                  bitmap(px + x + dx + ox, py + y + dy + oy) = k;
         });
      }

      glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
      glGenTextures(1, &tex_id);
      glBindTexture(GL_TEXTURE_2D, tex_id);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
      glTexImage2D(GL_TEXTURE_2D,
                   0,
                   GL_RGBA,
                   scale * sz * font_sz,
                   scale * sz * font_sz,
                   0,
                   GL_RGBA,
                   GL_UNSIGNED_BYTE,
                   bitmap.ptr(0));

      is_init = true;
   }

   glEnable(GL_TEXTURE_2D);
   glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_DECAL);
   glBindTexture(GL_TEXTURE_2D, tex_id);

   glPushMatrix();
   glScaled(1.0 / 4.0, 1.0 / 6.0, 1.0);
   glBegin(GL_QUADS);
   const Vector3f n(0.0, 0.0, 1.0);
   auto counter    = 0;
   const auto xoff = 2.0f / 5.0f;

   auto width  = xoff + float(s.size()) * 2.0f;
   auto height = 6.0f;

   { // leading white space (xoff wide)
      float x0     = 0.0f;
      float x1     = x0 + xoff;
      float y0     = 0.0;
      float y1     = height;
      const auto d = float(1.0 / real(sz)); // delta
      const auto x = 0;
      const auto y = 0;
      const auto s = d;
      glTexCoord2f(x, y);
      glNormal3fv(n.ptr());
      glVertex3f(x0, y0, 0.0);
      glTexCoord2f(x, y + d);
      glNormal3fv(n.ptr());
      glVertex3f(x0, y1, 0.0);
      glTexCoord2f(x + s, y + d);
      glNormal3fv(n.ptr());
      glVertex3f(x1, y1, 0.0);
      glTexCoord2f(x + s, y);
      glNormal3fv(n.ptr());
      glVertex3f(x1, y0, 0.0);
   }

   for(auto i = 0u; i < s.size(); ++i) { // each character
      auto c          = std::isprint(s[i]) ? s[i] : ' ';
      const auto last = i + 1 == s.size();

      float x0 = float(counter++) * 2.0f + xoff;
      float x1 = x0 + (last ? 2.0f : 2.0f);
      float y0 = 0.0;
      float y1 = height;

      // What is the texture coord of this character??
      const auto d = float(1.0 / real(sz)); // delta
      const auto x = d * float(c % sz);
      const auto y = d * float(c / sz);
      const auto s = d * 2.2f / 4.0f;

      glTexCoord2f(x, y);
      glNormal3fv(n.ptr());
      glVertex3f(x0, y0, 0.0);
      glTexCoord2f(x, y + d);
      glNormal3fv(n.ptr());
      glVertex3f(x0, y1, 0.0);
      glTexCoord2f(x + s, y + d);
      glNormal3fv(n.ptr());
      glVertex3f(x1, y1, 0.0);
      glTexCoord2f(x + s, y);
      glNormal3fv(n.ptr());
      glVertex3f(x1, y0, 0.0);
   }
   glEnd();
   glPopMatrix();

   glDisable(GL_TEXTURE_2D);
}

// ----------------------------------------------------------------- Render Cube

void gl_render_cube(const Vector3& center, const Vector3& dims) noexcept
{
   const auto& C = center;
   const auto& D = dims;
   glPushMatrix();
   glTranslated(C.x, C.y, C.z);
   glBegin(GL_QUADS);
   glVertex3d(-D.x, -D.y, D.z); // Top
   glVertex3d(D.x, -D.y, D.z);
   glVertex3d(D.x, D.y, D.z);
   glVertex3d(-D.x, D.y, D.z);
   glVertex3d(-D.x, -D.y, -D.z); // Bottom
   glVertex3d(-D.x, D.y, -D.z);
   glVertex3d(D.x, D.y, -D.z);
   glVertex3d(D.x, -D.y, -D.z);
   glEnd();
   glPopMatrix();
}

// --------------------------------------------------------------- Render Sphere

void gl_render_sphere(const Vector3& center,
                      const real radius,
                      const unsigned n_divisions)
{
   const auto& C = center;
   glPushMatrix();
   glTranslated(C.x, C.y, C.z);
   glScaled(radius, radius, radius);

   auto draw_face = [&](const auto& f) {
      auto p3 = Plane(f[0], f[1], f[2]);
      auto n  = p3.xyz();
      glNormal3dv(n.ptr());
      glVertex3dv(f[0].ptr());
      glNormal3dv(n.ptr());
      glVertex3dv(f[1].ptr());
      glNormal3dv(n.ptr());
      glVertex3dv(f[2].ptr());
   };

   glEnable(GL_LIGHTING);
   glBegin(GL_TRIANGLES);
   glColor4d(0.4, 0.4, 0.4, 1.0);
   for(const auto& f : make_octohedron(n_divisions)) draw_face(f);
   glEnd();
   glDisable(GL_LIGHTING);

   glPopMatrix();
}

// --------------------------------------------------------------- Render Circle

void gl_render_circle(const Vector3& center,
                      const real radius,
                      const unsigned n_divisions) noexcept
{
   if(n_divisions < 2) return;
   const auto& C = center;
   glPushMatrix();
   glBegin(GL_TRIANGLE_FAN);
   glVertex3dv(C.ptr());
   for(auto i = 0u; i <= n_divisions; ++i) {
      const real theta = 2.0 * M_PI * real(i) / real(n_divisions);
      const real x     = cos(theta) * radius;
      const real y     = sin(theta) * radius;
      glVertex3d(C.x + x, C.y + y, C.z);
   }
   glEnd();
   glPopMatrix();
}

void gl_render_circle(const Vector3& C,
                      const Vector3& N,
                      const real r,
                      const unsigned n_divisions) noexcept
{
   auto q    = Quaternion::between_vectors(Vector3(0, 0, 1), N);
   Vector3 X = q.rotate(Vector3(1, 0, 0));
   Vector3 Y = q.rotate(Vector3(0, 1, 0));

   const auto n = n_divisions;
   real step    = 2.0 * M_PI / real(n);

   glBegin(GL_LINE_LOOP);
   for(auto i = 0u; i < n; ++i) {
      const auto dxy = Vector2(cos(i * step), sin(i * step));
      const auto U   = C + r * (dxy.x * X + dxy.y * Y);
      glVertex3d(U.x, U.y, U.z);
   }
   glEnd();
}

// ------------------------------------------------------------- render cylinder

// void gl_render_cylinder(const Cylinder& Cy,
//                         const unsigned in_n_divisions,
//                         const uint32_t kolour,
//                         const float alpha) noexcept
// {
//    // Theta
//    constexpr unsigned n_divisions = 17;
//    std::array<float, n_divisions + 1> thetas;
//    for(auto i = 0u; i < thetas.size(); ++i)
//       thetas[i] = 2.0f * float(M_PI) * float(i) / float(n_divisions);

//    // GlFragment
//    vector<GlFragment> frags;
//    frags.reserve(2 * n_divisions + 2 * n_divisions);

//    { // Transparent fragments
//       const Vector4f k4 = kolour_to_vector4f(kolour, alpha);
//       const auto dz0    = Vector3f{0.0f, 0.0f, 0.0f};
//       const auto dz1    = Vector3f{0.0f, 0.0f, float(Cy.height)};
//       const auto C      = to_vec3f(Cy.X);
//       for(auto i = 0u; i < n_divisions; ++i) {
//          const real theta0 = thetas[i + 0];
//          const real theta1 = thetas[i + 1];
//          const float x0    = cos(theta0) * Cy.radius;
//          const float y0    = sin(theta0) * Cy.radius;
//          const float x1    = cos(theta1) * Cy.radius;
//          const float y1    = sin(theta1) * Cy.radius;

//          const auto A  = Vector3f{C.x + x0, C.y + y0, 0.0f};
//          const auto B  = Vector3f{C.x + x1, C.y + y1, 0.0f};
//          const auto B1 = B + (C - B).normalised() * float(Cy.radius) * 0.05f;
//          const auto A1 = A + (C - A).normalised() * float(Cy.radius) * 0.05f;

//          // The circle at the top and bottom
//          // frags.emplace_back(A + dz0, C + dz0, B + dz0, k4);
//          // frags.emplace_back(A + dz1, C + dz1, B + dz1, k4);
//          frags.emplace_back(A + dz1, B1 + dz1, B + dz1, k4);
//          frags.emplace_back(A + dz1, A1 + dz1, B1 + dz1, k4);

//          //
//          frags.emplace_back(A + dz0, A + dz1, B + dz0, k4);
//          // frags.emplace_back(B + dz0, A + dz1, B + dz1, k4);
//       }
//    }

//    { // The thick signifier on the ground
//       const Vector3 k3 = kolour_to_vector3(kolour);
//       glEnable(GL_LINE_SMOOTH);
//       glLineWidth(3.0f);

//       const auto draw_circle = [&](const auto& C, float radius) {
//          glBegin(GL_LINE_LOOP);
//          for(const auto& t : thetas)
//             glVertex3f(C.x + cos(t) * radius, C.y + sin(t) * radius, C.z);
//          glEnd();
//       };

//       const auto& p3 = Cy.p3; //

//       const auto Zaxis = Vector3f{0.0, 0.0, 1.0};
//       const auto Yaxis = cross(Zaxis, to_vec3f(p3.xyz())).normalised();
//       const auto Xaxis = cross(Yaxis, Zaxis);

//       const auto F  = to_vec3f(Cy.X) + Xaxis * float(Cy.radius);
//       const auto S0 = to_vec3f(Cy.X) + Yaxis * float(Cy.radius);
//       const auto S1 = to_vec3f(Cy.X) - Yaxis * float(Cy.radius);

//       glBegin(GL_LINES);
//       glVertex3dv(Cy.X.ptr());
//       glVertex3fv(F.ptr());
//       glVertex3dv(Cy.X.ptr());
//       glVertex3fv(S0.ptr());
//       glVertex3dv(Cy.X.ptr());
//       glVertex3fv(S1.ptr());
//       glEnd();

//       glColor3dv(k3.ptr());
//       draw_circle(Cy.X, Cy.radius * 1.0f);
//       draw_circle(Cy.X, Cy.radius * 0.5f);
//    }

//    global_frag_buffer.push(begin(frags), end(frags));
// }

// ------------------------------------------------------------- Make Octohedron

vector<array<Vector3, 3>> make_octohedron(const unsigned n_divisions)
{
   using Face = array<Vector3, 3>;

   // Every triangle gets subdivided by four
   auto subdivide = [&](const vector<Face>& figure) -> vector<Face> {
      vector<Face> X;
      X.reserve(figure.size() * 4);
      for(const auto& f : figure) {
         Vector3 v0 = f[0];
         Vector3 v1 = f[1];
         Vector3 v2 = f[2];
         Vector3 v3 = (f[0] + f[1]).normalized();
         Vector3 v4 = (f[1] + f[2]).normalized();
         Vector3 v5 = (f[2] + f[0]).normalized();
         X.push_back({{v0, v3, v5}});
         X.push_back({{v3, v1, v4}});
         X.push_back({{v4, v2, v5}});
         X.push_back({{v5, v3, v4}});
      }
      return X;
   };

   const real s2 = sqrt(0.5);
   const auto Z0 = Vector3(0.0, 0.0, 1.0);
   const auto Z1 = Vector3(0.0, 0.0, -1.0);
   const auto A0 = Vector3(s2, s2, 0.0);
   const auto A1 = Vector3(-s2, s2, 0.0);
   const auto A2 = Vector3(-s2, -s2, 0.0);
   const auto A3 = Vector3(s2, -s2, 0.0);

   vector<Face> figure;
   figure.push_back({{A0, A1, Z0}});
   figure.push_back({{A1, A2, Z0}});
   figure.push_back({{A2, A3, Z0}});
   figure.push_back({{A3, A0, Z0}});
   figure.push_back({{A1, A0, Z1}});
   figure.push_back({{A2, A1, Z1}});
   figure.push_back({{A3, A2, Z1}});
   figure.push_back({{A0, A3, Z1}});

   for(auto i = 0u; i < n_divisions; ++i)
      figure = subdivide(figure); //    N  ==>  4N

   return figure;
}

vector<array<Vector3, 3>> make_hemioctohedron(const unsigned n_divisions)
{
   using Face = array<Vector3, 3>;

   // Every triangle gets subdivided by four
   auto subdivide = [&](const vector<Face>& figure) -> vector<Face> {
      vector<Face> X;
      X.reserve(figure.size() * 4);
      for(const auto& f : figure) {
         Vector3 v0 = f[0];
         Vector3 v1 = f[1];
         Vector3 v2 = f[2];
         Vector3 v3 = (f[0] + f[1]).normalized();
         Vector3 v4 = (f[1] + f[2]).normalized();
         Vector3 v5 = (f[2] + f[0]).normalized();
         X.push_back({{v0, v3, v5}});
         X.push_back({{v3, v1, v4}});
         X.push_back({{v4, v2, v5}});
         X.push_back({{v5, v3, v4}});
      }
      return X;
   };

   const real s2 = sqrt(0.5);
   const auto Z0 = Vector3(0.0, 0.0, 1.0);
   const auto A0 = Vector3(s2, s2, 0.0);
   const auto A1 = Vector3(-s2, s2, 0.0);
   const auto A2 = Vector3(-s2, -s2, 0.0);
   const auto A3 = Vector3(s2, -s2, 0.0);

   vector<Face> figure;
   figure.push_back({{A0, A1, Z0}});
   figure.push_back({{A1, A2, Z0}});
   figure.push_back({{A2, A3, Z0}});
   figure.push_back({{A3, A0, Z0}});

   for(auto i = 0u; i < n_divisions; ++i)
      figure = subdivide(figure); //    N  ==>  4N

   return figure;
}

void flush_transparent_fragments()
{
   global_frag_buffer().sort_render_and_flush();
}

} // namespace perceive
