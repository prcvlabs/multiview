
#pragma once

#include "gl-error.hpp"
#include "gl-fragment-buffer.hpp"

#include "perceive/foundation.hpp"
#include "perceive/geometry/cylinder.hpp"
#include "perceive/geometry/vector.hpp"

namespace perceive
{
void stock_gl_setup(unsigned viewport_w, unsigned viewport_h);
void gl_print_info();

// Glu-look-at
void glu_look_at(double eyeX,
                 double eyeY,
                 double eyeZ,
                 double centerX,
                 double centerY,
                 double centerZ,
                 double upX,
                 double upY,
                 double upZ);

void glu_perspective(double fovy, double aspect, double zNear, double zFar);

// Shader program specs
Point3 glsl_workgroup_count();
Point3 glsl_workgroup_size();
int glsl_workgroup_invocations();
int glsl_max_image_units();

// The returned 'R' is row-major (GL is col-major)
void gl_read_projection_matrix(Matrix4r& R) noexcept;
void gl_read_modelview_matrix(Matrix4r& R) noexcept;
void gl_mult_matrix(const Matrix4r& R) noexcept; // R is row-major
void gl_mult_matrix(const EuclideanTransform& e) noexcept;

void gl_debug_world_to_screen(const Vector3& W) noexcept;

Vector3 extract_cam_center(const Matrix4r& R) noexcept;
Vector3 extract_cam_center_from_current_model_view() noexcept;

// Not thread-safe: call from opengl rendering context.
// Binds the font texture, and outputs a set of texture/vertices
// Texture is rendered on the XY plane (normals +Z)
// starting at (0, 0), and going towards (x, 0).
// Each character is 1.0x1.0 unit in size (square)
void gl_render_text(const string& s);
Vector2 gl_render_text_sz(const string& s);

void gl_render_cube(const Vector3& center, const Vector3& dims) noexcept;

// nfaces = 8 * 4 * (1 + n_divisions)
void gl_render_sphere(const Vector3& center,
                      const real radius,
                      const unsigned n_divisions);

void gl_render_circle(const Vector3& center,
                      const real radius,
                      const unsigned n_divisions) noexcept;

void gl_render_circle(const Vector3& C,
                      const Vector3& N,
                      const real r,
                      const unsigned n_divisions) noexcept;

// void gl_render_cylinder(const Cylinder& Cy,
//                         const unsigned n_divisions,
//                         const uint32_t kolour,
//                         const float alpha) noexcept;

vector<array<Vector3, 3>> make_octohedron(const unsigned n_divisions);
vector<array<Vector3, 3>> make_hemioctohedron(const unsigned n_divisions);

void flush_transparent_fragments();

} // namespace perceive
