
#include "gl-projection.hpp"
#include "perceive/foundation.hpp"
#include "perceive/geometry/rotation.hpp"
#include "perceive/geometry/vector.hpp"

#include <GL/gl.h>
#include <GL/glu.h>

#define This GlProjection

namespace perceive
{
const EuclideanTransform This::et0
    = EuclideanTransform(Vector3{0.0, 0.0, 0.0},
                         Vector4{0.0, 1.0, 0.0, M_PI},
                         1.0);

void This::init(real x_,
                real y_,
                real w_,
                real h_,
                real hfov_radians_,
                real z_near_,
                real z_far_) noexcept
{
   x      = x_;
   y      = y_;
   w      = w_;
   h      = h_;
   aspect = w_ / h_;
   hfov   = hfov_radians_;

   aspect_inv = h_ / w_;
   f          = 1.0 / tan(hfov_radians_ * 0.5);
   f_inv      = tan(hfov_radians_ * 0.5);
   z_near     = z_near_;
   z_far      = z_far_;
   z_n_plus_f = z_near_ + z_far_;
   z_n_m_f    = z_near_ - z_far_;
   z_2nf      = 2.0 * z_near_ * z_far_;
}

void This::debug_world_to_screen(const Vector3& W) const noexcept
{
   cout << format("World     = {:s}", str(W)) << endl;
   cout << format("Eye       = {:s}", str(world_to_eye(W))) << endl;
   const auto R = world_to_ray(W);
   cout << format("Ray       = {:s}", str(R)) << endl;
   cout << format("Clip      = {:s}", str(world_to_clip(W))) << endl;
   cout << format("NDC       = {:s}", str(world_to_NDC(W))) << endl;
   const auto NDC = world_to_NDC(W);
   const auto dbz = 0.5 * (NDC.z + 1.0);
   const auto z   = convert_z(dbz);
   cout << format("depth-buf = {}", dbz) << endl;
   cout << format("conv-z    = {}", convert_z(dbz)) << endl;
   const auto S = world_to_screen(W);
   cout << format("screen    = [{}, {}]", S.x, S.y) << endl;
   cout << format("s-to-NDC  = {:s}", str(screen_to_NDC(S))) << endl;
   cout << format("s-to-ray  = {:s}", str(screen_to_ray(S))) << endl;
   cout << format("s-to-eye  = {:s}", str(screen_to_eye(S, z))) << endl;
   cout << format("s-to-W    = {:s}", str(screen_to_world(S, z))) << endl;
   cout << format("ray-to-s  = {:s}", str(eye_to_screen(R))) << endl;
   const auto S2 = flip_y(S);
   cout << format("regS      = [{}, {}]", S2.x, S2.y) << endl;
   const auto RN = regular_screen_to_normalized_coord(S2);
   cout << format("regS-to-N = {:s}", str(RN)) << endl;
   const auto RE = regular_screen_to_eye(S2, z);
   cout << format("regS-to-E = {:s}", str(RE)) << endl;
   cout << format("E-to-regS = {:s}", str(eye_to_regular_screen(RE))) << endl;
}

Matrix4r This::projection_matrix() const noexcept
{
   Matrix4r M = Matrix4r::Zero();
   M(0, 0)    = f * aspect_inv;
   M(1, 1)    = f;
   M(2, 2)    = (z_far + z_near) / (z_near - z_far);
   M(2, 3)    = 2.0 * z_far * z_near / (z_near - z_far);
   M(3, 2)    = -1.0;
   return M;
}

void This::gl_projection_apply() const noexcept
{
   glViewport(GLint(x), GLint(y), GLsizei(w), GLsizei(h));

   glMatrixMode(GL_PROJECTION);
   glLoadIdentity();
   gl_mult_matrix(projection_matrix());
   glMatrixMode(GL_MODELVIEW);
}

void This::gl_modelview_apply() const noexcept
{
   glMatrixMode(GL_MODELVIEW);
   glLoadIdentity();
   gl_mult_matrix(gl_et_);
}

} // namespace perceive
