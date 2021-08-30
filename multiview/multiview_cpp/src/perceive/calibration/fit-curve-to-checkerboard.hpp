
#pragma once

#include <opencv2/core.hpp>

#include "perceive/foundation.hpp"
#include "perceive/geometry.hpp"
#include "perceive/graphics/image-container.hpp"

namespace perceive::calibration
{
/**
 * Optimizes the spline parameters based on the (gradient) field.
 * Specifically, it finds the spline parameters that maximize the
 * dot product between evenly sampled gradients on the spline,
 * and the gradient in the field.
 *
 * The funciton 'accurate-spline-fit' has a visualizer for this
 */
real optimize_spline_on_field(const Field& field, Spline2d& spline,
                              const bool opt_endpoints, const bool feedback);

vector<Spline2d>
find_initial_splines(const unsigned nx, const unsigned ny, const Vector6r r1r2t,
                     std::function<Vector2(const Vector2& x)> distort,
                     const real rho = 0.4, const unsigned m_factor = 15);

vector<Spline2d> find_initial_splines(const unsigned nx, const unsigned ny,
                                      const vector<Vector3r>& Ds,
                                      const real rho, const unsigned m_factor);

void accurate_spline_fit(const cv::Mat& im, // distorted
                         vector<Spline2d>& splines_in_out, const bool feedback,
                         const unsigned refits = 6, const real max_blur = 16.0,
                         const unsigned max_blur_sz = 31u, const real rho = 0.4,
                         const unsigned m_factor = 15);

void accurate_spline_fit(const string& image_filename, // distorted
                         vector<Spline2d>& splines_in_out, const bool feedback,
                         const unsigned refits = 6, const real max_blur = 16.0,
                         const unsigned max_blur_sz = 31u, const real rho = 0.4,
                         const unsigned m_factor = 15);

ARGBImage render_splines(const cv::Mat& im, const vector<Spline2d>& splines,
                         int dxy = 1);

// ------------------------------------------------------------------- find-grid

vector<Vector3r> find_nx_ny_grid(const unsigned nx, const unsigned ny,
                                 const cv::Mat& image,
                                 const bool fast_check = false);

ARGBImage render_detected_corner_grid(const vector<Vector3r>& corners,
                                      const cv::Mat& image);

vector<Vector3r> find_nx_ny_grid(const unsigned n, // the image number
                                 const unsigned nx, const unsigned ny,
                                 const string& image_filename,
                                 const string& out_dir);

// What is the average size (pixel distance) of each edge in the grid?
real average_grid_size(const unsigned nx, const unsigned ny,
                       const vector<Vector3r>& corners);

} // namespace perceive::calibration
