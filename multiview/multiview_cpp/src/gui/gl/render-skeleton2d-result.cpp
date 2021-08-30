
#include "render-skeleton2d-result.hpp"

#include "perceive/geometry/euclidean-transform.hpp"
#include "perceive/geometry/skeleton/2d-helpers.hpp"

#ifdef USING_OPENGL
#include "gl/gl-utils.hpp"
#include <GL/gl.h>
#include <GL/glu.h>
#endif

namespace perceive
{
#ifndef USING_OPENGL
void gl_render_cylinder_result(const Skeleton2D::CylinderResult& cy_ret,
                               const int p2d_ind,
                               const uint32_t kolour,
                               const float alpha)
{
   Expects(false);
}

void gl_render_3d_skeleton(const Skeleton3D& p3d, const float height)
{
   Expects(false);
}

void gl_render_3d_cylinder(const Skeleton3D& p3d,
                           const float height,
                           const int p2d_ind,
                           const uint32_t kolour,
                           const float alpha)
{
   Expects(false);
}

#else
// --------------------------------------------------- gl-render-cylinder-result
//
void gl_render_cylinder_result(const Skeleton2D::CylinderResult& cy_ret,
                               const int p2d_ind,
                               const uint32_t kolour,
                               const float alpha)
{
   // Theta
   constexpr unsigned n_divisions = 17;
   std::array<float, n_divisions + 1> thetas;
   for(auto i = 0u; i < thetas.size(); ++i)
      thetas[i] = 2.0f * float(M_PI) * float(i) / float(n_divisions);

   const auto& Cy = cy_ret.Cy;

   // GlFragment
   vector<GlFragment> frags;
   frags.reserve(2 * n_divisions + 2 * n_divisions);

   { // Transparent fragments
      const Vector4f k4 = kolour_to_vector4f(kolour, alpha);

      glEnable(GL_LINE_SMOOTH);
      glLineWidth(1.0f);
      glBegin(GL_LINES);
      glColor4fv(k4.ptr());

      const auto dz0 = Vector3f{0.0f, 0.0f, 0.0f};
      const auto dz1 = Vector3f{0.0f, 0.0f, float(Cy.height) * 1.00f};
      const auto dz2 = Vector3f{0.0f, 0.0f, float(Cy.height) * 0.20f};
      const auto C   = to_vec3f(Cy.X);
      for(auto i = 0u; i < n_divisions; ++i) {
         const real theta0 = thetas[i + 0];
         const real theta1 = thetas[i + 1];
         const float x0    = cos(theta0) * Cy.radius;
         const float y0    = sin(theta0) * Cy.radius;
         const float x1    = cos(theta1) * Cy.radius;
         const float y1    = sin(theta1) * Cy.radius;

         const auto A  = Vector3f{C.x + x0, C.y + y0, 0.0f};
         const auto B  = Vector3f{C.x + x1, C.y + y1, 0.0f};
         const auto C  = 0.5f * (A + B);
         const auto B1 = B + (C - B).normalised() * float(Cy.radius) * 0.05f;
         const auto A1 = A + (C - A).normalised() * float(Cy.radius) * 0.05f;

         // The circle at the top and bottom
         // frags.emplace_back(A + dz0, C + dz0, B + dz0, k4);
         // frags.emplace_back(A + dz1, C + dz1, B + dz1, k4);
         frags.emplace_back(A + dz1, B1 + dz1, B + dz1, k4, false);
         frags.emplace_back(A + dz1, A1 + dz1, B1 + dz1, k4, false);

         //
         frags.emplace_back(A + dz0, C + dz2, B + dz0, k4, false);

         glVertex3f(C.x, C.y, dz1.z);
         glVertex3f(C.x, C.y, dz2.z);
      }

      glEnd();
   }

   { // The thick signifier on the ground
      const Vector3 k3 = kolour_to_vector3(kolour);
      const Vector3 kg = kolour_to_vector3(k_dim_gray);

      glEnable(GL_LINE_SMOOTH);
      glLineWidth(3.0f);

      const auto draw_circle = [&](const auto& C, float radius) {
         glBegin(GL_LINE_LOOP);
         for(const auto& t : thetas)
            glVertex3f(C.x + cos(t) * radius, C.y + sin(t) * radius, C.z);
         glEnd();
      };

      const auto& p3 = cy_ret.p3; //

      const auto Zaxis = Vector3{0.0, 0.0, 1.0};
      const auto Yaxis = cross(Zaxis, to_vec3(p3.xyz())).normalised();
      const auto Xaxis = cross(Yaxis, Zaxis);

      const auto F  = to_vec3(Cy.X) + Xaxis * real(Cy.radius);
      const auto S0 = to_vec3(Cy.X) + Yaxis * real(Cy.radius);
      const auto S1 = to_vec3(Cy.X) - Yaxis * real(Cy.radius);

      if(false) {
         WARN(format("INFO"));
         cout << format("p3.xyz = {}", str(p3.xyz())) << endl;
         cout << format("X = {}", str(Xaxis)) << endl;
         cout << format("Y = {}", str(Yaxis)) << endl;
         cout << format("Z = {}", str(Zaxis)) << endl;
         cout << endl;
      }

      glColor3dv(k3.ptr());
      glBegin(GL_LINES);
      glVertex3dv(Cy.X.ptr());
      glVertex3dv(F.ptr());
      // glVertex3dv(S0.ptr());
      // glVertex3dv(S1.ptr());
      glEnd();

      glColor3dv(k3.ptr());
      draw_circle(Cy.X, Cy.radius * 1.0f);
      draw_circle(Cy.X, Cy.radius * 0.5f);
      draw_circle(Cy.X, Cy.radius * 0.05f);

      glLineWidth(1.0f);
   }

   global_frag_buffer().push(begin(frags), end(frags));
}

// ------------------------------------------------------- gl-render-3d-skeleton
//
void gl_render_3d_skeleton(const Skeleton3D& p3d,
                           const float height,
                           const real theta,
                           const uint32_t kolour)
{
   const auto& bones = skeleton::get_p2d_bones();
   const auto Xs     = p3d.calc_Xs(height);
   const auto Xc     = p3d.Xs_centre();
   const auto X0     = Vector3(Xc.x, Xc.y, 0.0);
   const auto D      = Vector3(cos(theta), sin(theta), 0.0);
   const auto gaze   = X0 + 0.4 * D;

   glEnable(GL_LINE_SMOOTH);
   glLineWidth(4.0f);

   {
      const Vector3 k = kolour_to_vector3(kolour);
      glColor3dv(k.ptr());
      gl_render_circle(to_vec3(X0), Vector3(0, 0, 1), 0.02, 32);
   }

   glBegin(GL_LINES);
   glVertex3dv(X0.ptr());
   glVertex3dv(gaze.ptr());

   {
      const Vector3 X0 = to_vec3(p3d.X());

      const Vector3 kx      = kolour_to_vector3(k_red);
      const Vector3 ky      = kolour_to_vector3(k_green);
      const Vector3 kz      = kolour_to_vector3(k_blue);
      const Vector3 forward = X0 + 0.4 * to_vec3(p3d.forward_n());
      const Vector3 up      = X0 + 0.4 * to_vec3(p3d.up_n());
      const Vector3 left    = X0 + 0.4 * to_vec3(p3d.left_n());

      glColor3dv(kx.ptr());
      glVertex3dv(X0.ptr());
      glVertex3dv(forward.ptr());

      glColor3dv(ky.ptr());
      glVertex3dv(X0.ptr());
      glVertex3dv(left.ptr());

      glColor3dv(kz.ptr());
      glVertex3dv(X0.ptr());
      glVertex3dv(up.ptr());
   }

   for(const auto& bone : bones) {
      if(Xs[int(bone.kp0)].is_finite() && Xs[int(bone.kp1)].is_finite()) {
         const Vector3 k = kolour_to_vector3(bone.kolour);
         glColor3dv(k.ptr());
         glVertex3fv(Xs[int(bone.kp0)].ptr());
         glVertex3fv(Xs[int(bone.kp1)].ptr());
      }
   }
   glEnd();

   glLineWidth(1.0f);
}

// ------------------------------------------------------- gl-render-3d-cylinder
//
void gl_render_3d_cylinder(const Skeleton3D& p3d,
                           const float height,
                           const int p2d_ind,
                           const uint32_t kolour,
                           const float alpha)
{
   //
   // Theta
   constexpr unsigned n_divisions = 17;
   std::array<float, n_divisions + 1> thetas;
   for(auto i = 0u; i < thetas.size(); ++i)
      thetas[i] = 2.0f * float(M_PI) * float(i) / float(n_divisions);

   const float radius        = p3d.radius(height);
   const Vector3f X          = p3d.X(height);
   const Vector3f& up_n      = p3d.up_n();
   const Vector3f& forward_n = p3d.forward_n();
   const Vector3f left_n     = p3d.left_n();

   // GlFragment
   vector<GlFragment> frags;
   frags.reserve(2 * n_divisions + 2 * n_divisions);

   // We have to translate/rotate everything
   const auto q
       = QuaternionF::between_vectors(Vector3f{0.0f, 0.0f, 1.0f}, up_n);
   const auto et = EuclideanTransformF{X, q, 1.0f};

   auto push_frag = [&](const Vector3f& A,
                        const Vector3f& B,
                        const Vector3f& C,
                        const Vector4f& k) {
      frags.emplace_back(
          et.apply(A), et.apply(B), et.apply(C), k, !C.is_finite());
   };

   { // Transparent fragments
      const Vector4f k4 = kolour_to_vector4f(kolour, alpha);
      const auto dz0    = Vector3f{0.0f, 0.0f, 0.0f};
      const auto dz1    = Vector3f{0.0f, 0.0f, float(height) * 1.00f};
      const auto dz2    = Vector3f{0.0f, 0.0f, float(height) * 0.05f};
      const auto C      = Vector3f{0.0f, 0.0f, 0.0f};
      for(auto i = 0u; i < n_divisions; ++i) {
         const real theta0 = thetas[i + 0];
         const real theta1 = thetas[i + 1];
         const float x0    = cos(theta0) * radius;
         const float y0    = sin(theta0) * radius;
         const float x1    = cos(theta1) * radius;
         const float y1    = sin(theta1) * radius;

         const auto A  = Vector3f{C.x + x0, C.y + y0, 0.0f};
         const auto B  = Vector3f{C.x + x1, C.y + y1, 0.0f};
         const auto D  = 0.5f * (A + B);
         const auto B1 = B + (C - B).normalised() * float(radius) * 0.05f;
         const auto A1 = A + (C - A).normalised() * float(radius) * 0.05f;

         // The circle at the top and bottom
         push_frag(A + dz1, B1 + dz1, B + dz1, k4);
         push_frag(A + dz1, A1 + dz1, B1 + dz1, k4);
         push_frag(A + dz0, D + dz2, B + dz0, k4);
         push_frag(D + dz2, D + dz1, Vector3f::nan(), k4); // makes a line
      }
   }

   // push all frags
   global_frag_buffer().push(begin(frags), end(frags));
}

#endif

} // namespace perceive
