
#include "movie-layout.hpp"

#include "perceive/graphics/bresenham.hpp"
#include "perceive/graphics/colour-set.hpp"
#include "perceive/graphics/tiny-string.hpp"

#define This MovieLayout

namespace perceive::movie
{
// ------------------------------------------------------------------- to-string

string This::to_string() const noexcept
{
   const auto in_bounds = [&](const AABBi& o) -> bool {
      return o.left >= 0 and o.top >= 0 and o.right <= w and o.bottom <= h;
   };

   const auto f = [&](const AABBi& o) -> string {
      return format("[{}, {}, {}, {}]{}",
                    o.left,
                    o.top,
                    o.right,
                    o.bottom,
                    (in_bounds(o) ? "" : " OUT OF BOUNDS"));
   };

   const string movie_s
       = implode(cbegin(movie_bounds), cend(movie_bounds), "\n", f);
   const string hist_s
       = implode(cbegin(hist_bounds), cend(hist_bounds), "\n", f);

   return format(R"V0G0N(
MovieLayout:
   init     =  {}
   rot-hist =  {}
   wxh      = [{}x{}]
   Movies:{}{}
   Hists:{}{}
{})V0G0N",
                 str(is_init),
                 str(rotate_hists),
                 w,
                 h,
                 (movie_s.size() > 0 ? "\n" : ""),
                 indent(movie_s, 6),
                 (hist_s.size() > 0 ? "\n" : ""),
                 indent(hist_s, 6),
                 "");
}

// ----------------------------------------------------------- make-movie-layout

ARGBImage This::debug_image() const noexcept
{
   ARGBImage im;
   // if(!is_init) return im;
   im.resize(unsigned(w), unsigned(h));
   im.fill(k_light_gray);

   auto draw_bounds = [&](const string_view label, const AABBi& o, uint32_t k) {
      const Point2 tl = o.top_left();
      const Point2 tr = tl + Point2(o.width() - 1, 0);
      const Point2 bl = tl + Point2(0, o.height() - 1);
      const Point2 br = bl + Point2(o.width() - 1, 0);

      auto f = [&](int x, int y) {
         if(im.in_bounds(x, y)) im(x, y) = k;
      };

      for(auto y = tl.y; y < bl.y; ++y)
         for(auto x = tl.x; x < tr.x; ++x)
            if(im.in_bounds(x, y)) im(x, y) = k_white;

      bresenham(tl, tr, f);
      bresenham(tr, br, f);
      bresenham(br, bl, f);
      bresenham(bl, tl, f);

      const auto c  = o.center();
      const auto wh = render_tiny_dimensions(label.data());
      render_string(im, label, c + wh / 2, k, k_white);
   };

   for(auto i = 0u; i < movie_bounds.size(); ++i)
      draw_bounds(format("m{}", i), movie_bounds[i], k_red);
   for(auto i = 0u; i < hist_bounds.size(); ++i)
      draw_bounds(format("h{}", i), hist_bounds[i], k_black);
   return im;
}

// ----------------------------------------------------------- make-movie-layout

MovieLayout make_movie_layout(const int w, // Target movie width
                              const int h, // and height
                              const int margin,
                              const int padding,
                              const int n_movies, // Number of input movies
                              const int movie_w,  // Movie width/height
                              const int movie_h,
                              const int n_hists, // Number of input histograms
                              const int hist_w,  // Histogram width/height
                              const int hist_h) noexcept
{
   MovieLayout ml;
   Expects(w > 0);
   Expects(h > 0);

   ml.w = w;
   ml.h = h;

   { // -- (*) -- MOVIES
      // All the movies appear in multiple rows at the top half of the screen
      // Set the number of rows so that the movies are as boxy as possible.
      const int t0      = margin;
      const int b0      = (h / 2);
      const int l0      = margin;
      const int r0      = w - margin;
      const int draw_w  = (r0 - l0); // The width of the drawing area
      const int draw_h  = (b0 - t0); // ibid height
      const real aspect = real(movie_w) / real(movie_h);

      auto calc_for_n_rows = [&](const int n_rows) -> vector<AABBi> {
         const int n_cols = int_div2(n_movies, n_rows);
         Expects(n_rows * n_cols >= n_movies);
         const int max_w = draw_w / n_cols;
         const int max_h = draw_h / n_rows;
         int w0          = movie_w;
         int h0          = movie_h;
         if(w0 > max_w) {
            w0 = max_w;
            h0 = int(max_w / aspect);
         }
         if(h0 > max_h) {
            w0 = int(max_h * aspect);
            h0 = max_h;
         }
         const int actual_w       = w0 * n_cols;
         const int actual_h       = h0 * n_rows;
         const int extra_offset_w = (draw_w - actual_w) / 2;
         const int extra_offset_h = (draw_h - actual_h) / 2;
         // INFO(format("MAX"));
         // cout << format("   AABBi  = [{}, {}, {}, {}]", l0, t0, r0, b0) <<
         // endl; cout << format("   rowcol = {} x {}", n_rows, n_cols) << endl;
         // cout << format("   max wh = [{} x {}]", max_w, max_h) << endl;

         auto calc_aabb = [&](const int row, const int col) -> AABBi {
            AABBi o;
            o.left   = t0 + (col * w0) + extra_offset_w;
            o.top    = l0 + (row * h0) + extra_offset_h;
            o.right  = o.left + w0;
            o.bottom = o.top + h0;
            return o;
         };
         vector<AABBi> out((size_t(n_movies)));
         for(auto i = 0; i < n_movies; ++i)
            out[size_t(i)] = calc_aabb(i / n_cols, i % n_cols);
         return out;
      };

      auto area_for_aabbs = [&](const vector<AABBi>& aabbs) -> int {
         return std::accumulate(
             cbegin(aabbs), cend(aabbs), 0, [&](const int a, const AABBi& o) {
                return a + o.area();
             });
      };

      // Just try 1-5 rows. Seriously that should be good enough.
      auto best_ret = calc_for_n_rows(1);
      for(int i = 2; i <= 5; ++i) {
         auto ret = calc_for_n_rows(i);
         if(area_for_aabbs(ret) > area_for_aabbs(best_ret))
            best_ret = std::move(ret);
      }
      ml.movie_bounds = std::move(best_ret);
   }

   { // -- (*) -- HISTOGRAMS
      // All the histograms appear in a row in the bottom half of the screen
      const int t0     = (h / 2) + padding;
      const int b0     = h - padding;
      const int l0     = margin;
      const int r0     = w - margin;
      const int draw_w = (r0 - l0); // The width of the drawing area
      const int draw_h = (b0 - t0); // ibid height
      const int max_w  = (draw_w - (n_hists - 1) * padding) / n_hists;
      const int max_h  = draw_h;

      int w0 = 0, h0 = 0;
      bool do_rotate = false;
      {
         const real aspect = real(hist_w) / real(hist_h);
         do_rotate         = aspect > 1.0;
         w0                = (do_rotate) ? max_h : max_w;
         h0                = (do_rotate) ? max_w : max_h;
         if(w0 > max_w) {
            w0 = max_w;
            h0 = (do_rotate) ? int(max_w * aspect) : int(max_w / aspect);
         }
         if(h0 > max_h) {
            w0 = (do_rotate) ? int(max_h / aspect) : int(max_h * aspect);
            h0 = max_h;
         }
      }

      const int actual_w       = w0 * n_hists + (n_hists - 1) * padding;
      const int actual_h       = h0;
      const int extra_offset_w = (draw_w - actual_w) / 2;
      const int extra_offset_h = (draw_h - actual_h) / 2;

      auto calc_hist = [&](int i) {
         const int offset = i * padding;
         AABBi aabb;
         aabb.top    = t0 + extra_offset_h;
         aabb.bottom = aabb.top + h0;
         aabb.left   = l0 + offset + (i * w0) + extra_offset_w;
         aabb.right  = aabb.left + w0;
         return aabb;
      };

      ml.hist_bounds.resize(size_t(n_hists));
      ml.rotate_hists = do_rotate;
      for(auto i = 0; i < n_hists; ++i)
         ml.hist_bounds[size_t(i)] = calc_hist(i);
   }

   // BOUNDS CHECK
   // If any AABB is out of bounds, then set 'is-init' to FALSE
   const auto in_bounds = [&](const AABBi& o) -> bool {
      return o.left >= 0 and o.top >= 0 and o.right <= w and o.bottom <= h;
   };

   ml.is_init
       = std::all_of(cbegin(ml.movie_bounds), cend(ml.movie_bounds), in_bounds)
         and std::all_of(
             cbegin(ml.hist_bounds), cend(ml.hist_bounds), in_bounds);

   return ml;
}

// ---------------------------------------------------------------- fitting-rect
//
std::pair<int, int> fitting_rect(const int max_width,
                                 const int max_height,
                                 const real aspect // w/h
                                 ) noexcept
{
   Expects(std::isfinite(aspect));
   Expects(aspect > 0.0);

   if(max_width == 0 || max_height == 0) return {1, 1};

   Expects(max_width >= 1);
   Expects(max_height >= 1);

   std::pair<int, int> o;
   const int w = int(max_height * aspect);
   if(w <= max_width)
      o = {w, max_height};
   else
      o = {max_width, max_width / aspect};

   // WARN(format("aspect = {}, {{}x{}}  ==>  {{}x{}} ({})",
   //             aspect,
   //             max_width,
   //             max_height,
   //             o.first,
   //             o.second,
   //             real(o.first) / real(o.second)));

   Expects(o.first <= max_width);
   Expects(o.second <= max_height);

   return o;
}

} // namespace perceive::movie
