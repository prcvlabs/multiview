
#include "distorted-camera.hpp"
#include "triangulation.hpp"

#include "perceive/geometry/human-heights.hpp"
#include "perceive/graphics/bresenham.hpp"

#define This DistortedCamera

namespace perceive
{
DistortedCamera make_distorted_camera(const EuclideanTransform& et,
                                      const CachingUndistortInverse& cu,
                                      const unsigned w,
                                      const unsigned h)
{
   DistortedCamera cam;
   std::tie(cam.C, cam.q) = make_camera_extrinsics(et);
   cam.cu                 = cu;
   cam.cu.set_working_format(w, h);
   cam.w = int(w);
   cam.h = int(h);
   return cam;
}

EuclideanTransform This::et() const noexcept
{
   return dcam_Cq_to_euclidean_transform(C, q);
}

string DistortedCamera::to_string() const noexcept
{
   return format("dcam({}), wh={}x{}, C={}, q={}",
                 cu.M().sensor_id(),
                 w,
                 h,
                 str(C),
                 q.to_readable_str());
}

std::pair<DistortedCamera, DistortedCamera>
make_distorted_camera_pair(const BinocularCameraInfo& bcam_info,
                           const EuclideanTransform& et0,
                           const unsigned w,
                           const unsigned h)
{
   auto et1 = bcam_info.make_et1(et0);

   auto make_it = [&](int i) {
      DistortedCamera cam;
      std::tie(cam.C, cam.q) = make_camera_extrinsics(i == 0 ? et0 : et1);
      cam.cu.init(bcam_info.M[size_t(i)]);
      cam.cu.set_working_format(w, h);
      cam.w = int(w);
      cam.h = int(h);
      return cam;
   };

   return std::make_pair(make_it(0), make_it(1));
}

std::pair<DistortedCamera, DistortedCamera>
make_distorted_camera_pair(const BinocularCamera& bcam,
                           const EuclideanTransform& et0,
                           const unsigned w,
                           const unsigned h)
{
   auto et1 = bcam.make_et1(et0);

   auto make_it = [&](int i) {
      DistortedCamera cam;
      std::tie(cam.C, cam.q) = make_camera_extrinsics(i == 0 ? et0 : et1);
      cam.cu                 = bcam.cu(i);
      cam.cu.set_working_format(w, h);
      cam.w = int(w);
      cam.h = int(h);
      return cam;
   };

   return std::make_pair(make_it(0), make_it(1));
}

// -------------------------------------- euclidean-transform to/from extrinsics
// Convert EuclideanTransforms to/from
// camera extrinsics
std::pair<Vector3, Quaternion>
make_camera_extrinsics(const EuclideanTransform& et) noexcept
{
   return std::make_pair(et.translation, et.rotation.inverse());
}

EuclideanTransform dcam_Cq_to_euclidean_transform(const Vector3& C,
                                                  const Quaternion& q) noexcept
{
   EuclideanTransform et;
   et.translation = C;
   et.rotation    = q.inverse();
   et.scale       = 1.0;
   return et;
}

// ---------------------------------------------------------- triangulate points

Vector3 triangulate(const array<DistortedCamera, 2>& dcams,
                    const Vector2& u,
                    const Vector2& v,
                    const bool feedback) noexcept
{
   return triangulate(dcams[0], dcams[1], u, v, feedback);
}

Vector3 triangulate(const DistortedCamera& dcam0,
                    const DistortedCamera& dcam1,
                    const Vector2& u,
                    const Vector2& v,
                    const bool feedback) noexcept
{
   const auto ray_0 = project_to_ray(dcam0, u);
   const auto ray_1 = project_to_ray(dcam1, v);
   const auto X     = intersect_rays_2(dcam0.C, ray_0, dcam1.C, ray_1);
   if(feedback) {
      const auto err
          = intersect_rays_2_plane_err(dcam0.C, ray_0, dcam1.C, ray_1);
      INFO(" * dcam triangulate feedback *");
      cout << format(
          "   e0  = {}, {}\n", str(dcam0.C), dcam0.q.to_readable_str());
      cout << format(
          "   e1  = {}, {}\n", str(dcam1.C), dcam1.q.to_readable_str());
      cout << format("   u   = {}", str(u)) << endl;
      cout << format("   v   = {}", str(v)) << endl;
      cout << format("   r0  = {}", str(ray_0)) << endl;
      cout << format("   r1  = {}", str(ray_1)) << endl;
      cout << format("   err = {}", err) << endl;
      cout << format("   X   = {}", str(X)) << endl;
      cout << endl;
   }
   return X;
}

// --------------------------------------------------------------- plot-cylinder
//
void plot_cylinder(const DistortedCamera& dcam,
                   const Cylinder& Cy,
                   const AABB& bounds,
                   std::function<void(int, int)> f)
{
   constexpr int n_segments = 40;

   auto draw_circle = [&](const Vector3& C, const real radius) {
      const auto dx = Vector3(1.0, 0.0, 0.0);
      const auto dy = Vector3(0.0, 1.0, 0.0);
      auto calc_pos = [&](const real theta) -> Vector3 {
         return C + radius * cos(theta) * dx + radius * sin(theta) * dy;
      };

      Vector2 l = Vector2(bounds.right, dNAN), r = Vector2(bounds.left, dNAN);

      const auto dt = 2.0 * M_PI / real(n_segments - 1);
      real theta    = 0.0;
      Vector2 c0    = project_to_distorted(dcam, calc_pos(theta));
      for(int i = 1; i < n_segments; ++i) {
         Vector2 c1 = project_to_distorted(dcam, calc_pos(dt * real(i)));
         bresenham(c0, c1, bounds, f);
         if(c0.x < l.x) l = c0;
         if(c0.x > r.x) r = c0;
         c0 = c1;
      }

      return std::pair<Vector2, Vector2>(l, r);
   };

   const auto dz     = Vector3(0.0, 0.0, Cy.height);
   const auto [a, b] = draw_circle(Cy.X, Cy.radius);
   const auto [c, d] = draw_circle(Cy.X + dz, Cy.radius);

   bresenham(a, c, bounds, f);
   bresenham(b, d, bounds, f);
}

void plot_human_cylinder(const DistortedCamera& dcam,
                         const Cylinder& Cy,
                         const AABB& bounds,
                         std::function<void(int, int)> f)
{
   plot_cylinder(dcam, Cy, bounds, f);

   constexpr int n_segments = 40;

   auto draw_circle = [&](const Vector3& C, const real radius) {
      const auto dx = Vector3(1.0, 0.0, 0.0);
      const auto dy = Vector3(0.0, 1.0, 0.0);
      auto calc_pos = [&](const real theta) -> Vector3 {
         return C + radius * cos(theta) * dx + radius * sin(theta) * dy;
      };

      const auto dt = 2.0 * M_PI / real(n_segments - 1);
      real theta    = 0.0;
      Vector2 c0    = project_to_distorted(dcam, calc_pos(theta));
      for(int i = 1; i < n_segments; ++i) {
         Vector2 c1 = project_to_distorted(dcam, calc_pos(dt * real(i)));
         bresenham(c0, c1, bounds, f);
         c0 = c1;
      }
   };

   const auto dz = Vector3(0.0, 0.0, Cy.height);

   // draw_circle(Cy.X + dz * human::k_foot_height_ratio, Cy.radius);
   // draw_circle(Cy.X + dz * human::k_knee_height_ratio, Cy.radius);
   // draw_circle(Cy.X + dz * human::k_pelvis_height_ratio, Cy.radius);
   draw_circle(Cy.X + dz * human::k_height_ratio_shoulder, Cy.radius);
   // draw_circle(Cy.X + dz * human::k_eye_height_ratio, Cy.radius);
}

// Returns two points where projected circle would be occluded if it
// were a cylinder
std::pair<Vector2f, Vector2f>
project_circle(const DistortedCamera& dcam,
               const Vector3f& X,      // Circle centre
               const float radius,     // Circle radius
               const Vector3f& up_n,   // Normal to circle plane
               const bool semi_circle, // to point of view
               const AABB& im_bounds,
               std::function<void(int, int, float)> f) noexcept
{
   Expects(X.is_finite());
   Expects(up_n.is_finite());

   constexpr unsigned n_divisions = 21;
   constexpr float z              = 0.0f;

   std::array<Vector2f, n_divisions + 1> xs;

   const auto q
       = Quaternion::between_vectors(Vector3{0.0, 0.0, 1.0}, to_vec3(up_n));
   const auto et = EuclideanTransform{to_vec3(X), q, 1.0};

   auto make_X = [&](const float theta, const float z) -> Vector3f {
      return Vector3f(std::cos(theta) * radius, std::sin(theta) * radius, z);
   };

   auto calc_p3 = [&](const float z) {
      Plane p3; // the transformed plane...
      p3.xyz() = (dcam.C - to_vec3(X)).normalised();
      p3.d()   = -dot(p3.xyz(), to_vec3(X + z * up_n));
      return to_vec4f(et.inverse().apply_to_plane(p3));
   };

   auto make_edge = [&](const float theta, const float z) -> Vector2f {
      const auto X = make_X(theta, z);
      return to_vec2f(project_to_distorted(dcam, et.apply(to_vec3(X))));
   };

   const auto p3 = calc_p3(z);
   Expects(p3.is_finite());
   const auto theta0 = std::atan2(p3.xyz().x, -p3.xyz().y);
   const bool sign   = p3.side(make_X(theta0 + float(0.5 * M_PI), z)) > 0.0f;
   const auto theta1 = theta0 + (sign ? 1.0f : -1.0f) * float(M_PI);
   Expects(std::isfinite(theta0));
   Expects(std::isfinite(theta1));

   const auto dt = (!semi_circle ? float(2.0 * M_PI) : theta1 - theta0)
                   / float(n_divisions);
   auto t = theta0;
   for(auto i = 0u; i <= n_divisions; ++i, t += dt) xs[i] = make_edge(t, z);

   for(auto i = 0u; i < n_divisions; ++i)
      plot_line_AA(to_vec2(xs[i + 0]), to_vec2(xs[i + 1]), im_bounds, f);

   return {to_vec2f(make_edge(theta0, z)), to_vec2f(make_edge(theta1, z))};
}

// -------------------------------------------------------------------- plot-box
//
void plot_box(const DistortedCamera& dcam,
              const Box& box,
              const AABB& bounds,
              std::function<void(int, int)> f,
              const uint32_t stipple)
{
   int counter      = 0;
   auto render_line = [&](const Vector3& A, const Vector3& B) {
      const auto a = project_to_distorted(dcam, A);
      const auto b = project_to_distorted(dcam, B);
      int counter  = 0;
      bresenham(a, b, bounds, [&](int x, int y) {
         int pos = counter++;
         if(counter == 32) counter = 0; // wrap
         if(!bounds.contains(x, y)) return;
         if((stipple & (1 << pos)) != 0) f(x, y);
      });
   };

   const auto sz = box.base.size();
   const auto H  = Vector3(0.0, 0.0, box.height);

   // Draw the base
   for(size_t i = 0; i < sz; ++i)
      render_line(box.base[(i + 0) % sz], box.base[(i + 1) % sz]);

   // Draw the sides
   for(size_t i = 0; i < sz; ++i) render_line(box.base[i], box.base[i] + H);

   // Draw the top
   for(size_t i = 0; i < sz; ++i)
      render_line(box.base[(i + 0) % sz] + H, box.base[(i + 1) % sz] + H);
}

vector<Vector2> box_hull(const DistortedCamera& dcam, const Box& box)
{
   Expects(box.base.size() == 4);
   const auto sz = box.base.size();
   const auto H  = Vector3(0.0, 0.0, box.height);

   std::array<Vector2, 8> points;
   size_t counter = 0;
   auto process   = [&](const Vector3& X) {
      points[counter++] = project_to_distorted(dcam, X);
   };

   for(size_t i = 0; i < sz; ++i) process(box.base[i]);     // base
   for(size_t i = 0; i < sz; ++i) process(box.base[i] + H); // top

   vector<Vector2> out;
   andrews_convex_hull(cbegin(points), cend(points), out);
   return out;
}

// ------------------------------------------------------------------- plot-axis
//
void plot_axis(ARGBImage& argb, const DistortedCamera& dcam) noexcept
{
   auto project
       = [&](const Vector3& X) { return project_to_distorted(dcam, X); };

   auto project_render_line
       = [&](const Vector3& A, const Vector3& B, const uint32_t k) {
            const auto a = project(A);
            const auto b = project(B);
            plot_line_AA(a, b, argb.bounds(), [&](int x, int y, float a) {
               if(argb.in_bounds(x, y)) argb(x, y) = blend(k, argb(x, y), a);
            });
         };

   const Vector3 O = Vector3(0, 0, 0);
   project_render_line(O, Vector3(1, 0, 0), k_red);
   project_render_line(O, Vector3(0, 1, 0), k_green);
   project_render_line(O, Vector3(0, 0, 1), k_blue);

   const Vector3 o = project(O);
   const Point2 p  = to_pt2(homgen_P2_to_R2(o));
   fill_circle(argb, p, k_black, 7);
   fill_circle(argb, p, k_yellow, 5);
   draw_cross(argb, p, k_red, 3);
}

// ------------------------------------------------------------------ debug-dcam
//
static const DistortedCamera* debug_dcam_ptr_ = nullptr;

const DistortedCamera* get_debug_dcam() noexcept { return debug_dcam_ptr_; }

void set_debug_dcam(const DistortedCamera* ptr) noexcept
{
   debug_dcam_ptr_ = ptr;
}

} // namespace perceive
