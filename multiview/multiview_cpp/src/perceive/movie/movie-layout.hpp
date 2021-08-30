
#pragma once

#include "perceive/graphics/image-container.hpp"

namespace perceive::movie
{
// A movie as N videos, and a histogram of dimensions '3'
struct MovieLayout
{
   bool is_init = false;
   int w        = 0;
   int h        = 0;
   vector<AABBi> movie_bounds;
   vector<AABBi> hist_bounds;
   bool rotate_hists{false};

   CUSTOM_NEW_DELETE(MovieLayout)

   string to_string() const noexcept;
   ARGBImage debug_image() const noexcept;

   friend string str(const MovieLayout& ml) noexcept { return ml.to_string(); }
};

MovieLayout make_movie_layout(const int w, // Target movie width
                              const int h, // and height
                              const int margin,
                              const int padding,
                              const int n_movies, // Number of input movies
                              const int movie_w,  // Movie width/height
                              const int movie_h,
                              const int n_hists, // Number of input histograms
                              const int hist_w,  // Histogram width/height
                              const int hist_h) noexcept;

// Returns {w, h} with aspect `aspect`, and `w` <= `max_width`, etc...
std::pair<int, int> fitting_rect(const int max_width,
                                 const int max_height,
                                 const real aspect) noexcept;

} // namespace perceive::movie
