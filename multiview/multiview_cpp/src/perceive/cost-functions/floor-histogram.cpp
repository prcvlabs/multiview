
#include "floor-histogram.hpp"

#include "perceive/cost-functions/localization/localization-data.hpp"
#include "perceive/cost-functions/movie-stats/movie-stats-file.hpp"
#include "perceive/graphics/bresenham.hpp"
#include "perceive/graphics/colour-set.hpp"
#include "perceive/graphics/tiny-string.hpp"

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc.hpp>

#define This FloorHistogram

namespace perceive
{
// ------------------------------------------------------ ensure floor-hist dims
//
static std::pair<int, int>
ensure_floor_hist_dims(FloatImage& dist_im,
                       const AABB& aabb,
                       const real hist_sz,
                       const float default_value) noexcept
{
   const auto hist_sz_inv = 1.0 / hist_sz;
   const int h = int(ceil((aabb.bottom - aabb.top) * hist_sz_inv) + 1e-9);
   const int w = int(ceil((aabb.right - aabb.left) * hist_sz_inv) + 1e-9);

   if(dist_im.width != unsigned(w) or dist_im.height != unsigned(h)) {
      dist_im.resize(unsigned(w), unsigned(h));
      dist_im.fill(default_value);
   }

   return std::pair<int, int>{w, h};
}

// ------------------------------------------------------------------ meta-data
const vector<MemberMetaData>& FloorHistogram::Params::meta_data() const noexcept
{
#define ThisParams FloorHistogram::Params
   auto make_meta = []() {
      vector<MemberMetaData> m;

      m.push_back(MAKE_META(ThisParams, FLOAT, hist_sz, true));
      m.push_back(MAKE_META(ThisParams, FLOAT, hist_min_z, true));
      m.push_back(MAKE_META(ThisParams, FLOAT, hist_max_z, true));
      m.push_back(MAKE_META(ThisParams, FLOAT, min_hist_count, true));
      m.push_back(MAKE_META(ThisParams, FLOAT, max_hist_count, true));
      m.push_back(MAKE_META(ThisParams, FLOAT, gaussian_sigma, true));
      m.push_back(MAKE_META(ThisParams, FLOAT, super_pixel_min_z, true));

      m.push_back({meta_type::STRING,
                   "weighting"s,
                   true,
                   [](const void* ptr) -> std::any {
                      const auto& o = *reinterpret_cast<const ThisParams*>(ptr);
                      return std::any(string(str(o.weighting)));
                   },
                   [](void* ptr, const std::any& x) -> void {
                      auto& o         = *reinterpret_cast<ThisParams*>(ptr);
                      const string& s = std::any_cast<const string>(x);
                      o.weighting     = to_floor_hist_weighting_method(s);
                   }});

      m.push_back(MAKE_META(ThisParams, BOOL, apply_slic_still_weights, true));

      m.push_back({meta_type::JSON_VALUE,
                   "skip_cameras"s,
                   true,
                   [](const void* ptr) -> std::any {
                      const auto& o = *reinterpret_cast<const ThisParams*>(ptr);
                      Json::Value z{Json::arrayValue};
                      z.resize(unsigned(o.skip_cameras.size()));
                      for(auto i = 0u; i < z.size(); ++i)
                         z[i] = Json::Value(o.skip_cameras[i]);
                      return std::any(z);
                   },
                   [](void* ptr, const std::any& x) -> void {
                      auto& o = *reinterpret_cast<ThisParams*>(ptr);
                      const Json::Value& z
                          = std::any_cast<const Json::Value>(x);
                      if(z.type() != Json::arrayValue)
                         throw std::runtime_error("expected Json::array");
                      o.skip_cameras.resize(z.size());
                      for(auto i = 0u; i < z.size(); ++i)
                         o.skip_cameras[i] = unsigned(z[i].asInt());
                   }});

      // m.push_back(MAKE_META(ThisParams, Vector2 bbox_min, true));
      // m.push_back(MAKE_META(ThisParams, Vector2 bbox_max, true));
      m.push_back(MAKE_META(ThisParams, BOOL, use_median, true));
      m.push_back(MAKE_META(ThisParams, FLOAT, n_spread, true));
      m.push_back(MAKE_META(ThisParams, BOOL, color_histogram, true));
      m.push_back(MAKE_META(ThisParams, FLOAT, color_hist_sz, true));

      return m;
   };
   static vector<MemberMetaData> meta_ = make_meta();
   return meta_;
#undef ThisParams
}

// ------------------------------------------------------------------- calculate
//
bool This::calculate(const SceneDescription& scene_desc,
                     const Params& p,
                     AABB aabb,
                     unsigned n_cams,
                     std::vector<const BinocularCameraInfo*>& bcams,
                     std::vector<const EuclideanTransform*>& transforms,
                     std::vector<const PointCloud*>& point_clouds,
                     std::vector<const cv::Mat*>& rect_images,
                     std::vector<const IntImage*>& slic_lookups,
                     std::vector<const ImageFeatures2d*>& cam_sensor0_features,
                     std::vector<Vector3>& translations,
                     FloorHistogram& floor_hist,
                     std::function<bool()> is_canceled,
                     const bool feedback,
                     const string_view outdir)
{
   ParallelJobSet pjobs;

   if(is_canceled()) return false;

   if(!aabb.is_finite() || aabb.area() <= 0.0) {
      FATAL(format("Cannot calculate floor histogram, because scene bounding "
                   "box does not have finite area: bounds = {}",
                   str(aabb)));
   }

   Expects(aabb.is_finite());

   floor_hist.hist_sz       = real(p.hist_sz);
   floor_hist.color_hist_sz = real(p.color_hist_sz);

   const auto hist_sz            = p.hist_sz;
   const auto color_hist_sz      = p.color_hist_sz;
   const auto min_z              = p.hist_min_z;
   const auto max_z              = p.hist_max_z;
   const auto do_color_histogram = p.color_histogram;
   const auto is_trace_mode      = multiview_trace_mode();

   const auto hist_sz_inv       = 1.0 / real(hist_sz);
   const auto color_hist_sz_inv = 1.0 / real(color_hist_sz);

   // Now restrict to the bounding box, if set
   if(p.bbox_min.is_finite() and p.bbox_max.is_finite()
      and p.bbox_min != p.bbox_max) {
      AABB tmp = AABB::minmax();
      tmp.union_point(p.bbox_min.x, p.bbox_min.y);
      tmp.union_point(p.bbox_max.x, p.bbox_max.y);
      AABB tmp2 = intersection(aabb, tmp);
      aabb      = tmp2;
   }

   // LOG_ERR(format("features.size() = {}, ptclds = {}, n-cams = {}",
   //                features.size(),
   //                point_clouds.size(),
   //                n_cams));

   floor_hist.bounds = aabb;

   auto is_okay = [&](const Vector3& X) -> bool {
      if(X.z < real(min_z)) return false;
      if(X.z > real(max_z)) return false;
      return true;
   };

   // int h = ceil((aabb.bottom - aabb.top) * hist_sz_inv) + 1e-9;
   // int w = ceil((aabb.right - aabb.left) * hist_sz_inv) + 1e-9;

   auto& hist        = floor_hist.hist;
   const auto [w, h] = ensure_floor_hist_dims(hist, aabb, real(hist_sz), 0.0f);

   // Ensure we have the histograms for each camera
   auto& camera_info = floor_hist.camera_info;
   camera_info.resize(n_cams);

   int ch = int(ceil((aabb.bottom - aabb.top) * color_hist_sz_inv) + 1e-9);
   int cw = int(ceil((aabb.right - aabb.left) * color_hist_sz_inv) + 1e-9);

   // TRACE(format("aabb = {}, wh = [{}x{}]", str(aabb), w, h));

   Vec3fImage lab_hist;
   IntImage lab_counter;
   if(do_color_histogram) {
      lab_hist.resize(cw, ch);
      lab_hist.fill(Vector3f(0.0, 0.0, 0.0));
      lab_counter.resize(cw, ch);
      lab_counter.fill(0);
   }

   // auto& motion = floor_hist.motion;
   // motion.resize(w, h, w);
   // motion.fill(0.0f);

   auto process_camera_i = [&](unsigned cam_num) {
      auto& cam_hist = camera_info[cam_num].hist;
      cam_hist.resize(hist.width, hist.height, hist.width);
      cam_hist.fill(0.0f);

      if(scene_desc.is_no_stereo()) return;

      const BinocularCameraInfo& bcam{*bcams[cam_num]};
      const EuclideanTransform& et{*transforms[cam_num]};
      const PointCloud& point_cloud{*point_clouds[cam_num]};
      const IntImage& slic_lookup{*slic_lookups[cam_num]};
      const ImageFeatures2d& f2d{*cam_sensor0_features[cam_num]};
      const Vector3 C{translations[cam_num]};
      const int n_slic_labels = int(f2d.slic_numlabels);

      auto& spixel_xy = camera_info[cam_num].slic_to_hist_xy;
      spixel_xy.resize(size_t(n_slic_labels), SparseHistCell{{-1, -1}, 0});

      auto ii
          = std::find(cbegin(p.skip_cameras), cend(p.skip_cameras), cam_num);
      if(ii != cend(p.skip_cameras)) {
         INFO(format("skipping {}", bcam.camera_id));
         return;
      }

      // We calculate the centre of gravity for each super-pixel
      // but only for 3D points above `super_pixel_min_z`
      vector<Vector4f> spixel_Cs(size_t(n_slic_labels),
                                 Vector4f{0.0f, 0.0f, 0.0f, 0.0f});

      const auto& Xs   = point_cloud.Xs;
      const auto& xy   = point_cloud.xy;
      const unsigned N = unsigned(Xs.size());

      auto calc_weight
          = [&](const Vector3& X, const Point2& x) -> std::pair<int, float> {
         auto wgt = p.weighting == WGT_NONE   ? 1.0
                    : p.weighting == WGT_NORM ? (X - C).norm()
                                              : (X - C).quadrance();

         int label = slic_lookup.in_bounds(x) ? slic_lookup(x) : -1;

         if(p.apply_slic_still_weights and label >= 0) {
            if(label < int(f2d.slic_info.size())) {
               wgt *= 1.0 + 10.0 * f2d.slic_info[size_t(label)].still_score;
            } else {
               if(is_trace_mode) {
                  WARN(format("point x = [{}, {}] has no slic lookup for "
                              "cam# {}!",
                              x.x,
                              x.y,
                              cam_num));
               }
            }
         }
         return std::pair<int, float>(label, wgt);
      };

      for(unsigned i = 0; i < N; ++i) {
         // Project 'X' onto the floor
         const auto& q = xy[i];

         const auto& X = Xs[i];
         const auto Y  = et.apply(X);

         const auto Z = floor_hist.project_to_histogram(Y, real(p.hist_sz));
         const auto x = int(std::round(Z.x));
         const auto y = int(std::round(Z.y));

         if(is_okay(Y) && cam_hist.in_bounds(x, y)) {
            const auto [label, weight] = calc_weight(X, q);
            cam_hist(x, y) += weight;
            if(label >= 0 && Y.z >= real(p.super_pixel_min_z)) {
               spixel_Cs[size_t(label)]
                   += Vector4f(to_vec3f(Y) * float(weight),
                               spixel_Cs[size_t(label)].w + weight);
            }
         }
      }

      { // Now find the average centroid for each super-pixels
         for(auto label = 0u; label < spixel_Cs.size(); ++label) {
            const auto& sC = spixel_Cs[size_t(label)];
            if(sC.w == 0.0f) continue; // Nothing for this superpixel
            const Vector3 C = to_vec3(Vector3f(sC.x, sC.y, sC.z)) / real(sC.w);
            const auto Z = floor_hist.project_to_histogram(C, real(p.hist_sz));
            spixel_xy[size_t(label)].xy    = to_pt2(Z.round());
            spixel_xy[size_t(label)].count = std::round(sC.w);
         }
      }
   };

   // Now iterate over each point cloud, writing to the histogram
   for(unsigned cam_num = 0; cam_num < n_cams; ++cam_num) {
      pjobs.schedule(
          [&process_camera_i, cam_num]() { process_camera_i(cam_num); });
   }
   pjobs.execute();

   { // Now sum into `hist`
      hist.fill(0.0f);
      for(unsigned cam_num = 0; cam_num < n_cams; ++cam_num)
         hist += camera_info[cam_num].hist;
   }

   if(do_color_histogram && !scene_desc.is_no_stereo()) {
      auto apply_point_cloud = [&](const EuclideanTransform& et,
                                   const cv::Mat& rect_image,
                                   const PointCloud& point_cloud) {
         const auto& Xs = point_cloud.Xs;
         const auto& xy = point_cloud.xy;
         const size_t N = Xs.size();

         for(size_t i = 0; i < N; ++i) {
            const auto Y = et.apply(Xs[i]);
            const auto Z
                = floor_hist.project_to_histogram(Y, real(p.color_hist_sz));
            const auto rounded_Z = to_pt2(Z.round());
            if(!lab_hist.in_bounds(rounded_Z.x, rounded_Z.y)) continue;
            const auto& q = xy[i];

            const unsigned ref_w = unsigned(rect_image.cols);
            const unsigned ref_h = unsigned(rect_image.rows);
            if(unsigned(q.x) < ref_w and unsigned(q.y) < ref_h) {
               auto color = rect_image.at<cv::Vec3b>(cv::Point(q.x, q.y));
               lab_hist(rounded_Z.x, rounded_Z.y)
                   += to_vec3f(rgb_to_lab(vec3b_to_rgb(color)));
               lab_counter(rounded_Z.x, rounded_Z.y) += 1;
            }
         }
      };

      { // Colourize the floor
         for(size_t cam_num = 0; cam_num < n_cams; ++cam_num) {
            auto ii = std::find(
                cbegin(p.skip_cameras), cend(p.skip_cameras), cam_num);
            if(ii != cend(p.skip_cameras)) continue;

            const EuclideanTransform& et = *transforms[cam_num];
            const cv::Mat& rect_image    = *rect_images[cam_num];
            apply_point_cloud(et, rect_image, *point_clouds[cam_num]);
         }
      }

      { // Finalize the color histogram here
         auto& chist = floor_hist.color_hist;
         chist.resize(cw, ch);
         chist.fill(k_black);

         // Average that histogram...
         for(auto y = 0; y < ch; ++y) {
            for(auto x = 0; x < cw; ++x) {
               if(lab_counter(x, y) == 0) {
                  //
               } else {
                  chist(x, y) = lab_to_kolour(lab_hist(x, y)
                                              / float(lab_counter(x, y)));
               }
            }
         }

         // Draw the gridlines and AXIS
         draw_gridlines(chist, aabb, real(p.color_hist_sz), k_yellow);
         const auto O  = floor_hist.project_to_histogram(Vector3(0.0, 0.0, 0.0),
                                                        real(p.color_hist_sz));
         const auto X_ = floor_hist.project_to_histogram(Vector3(0.5, 0.0, 0.0),
                                                         real(p.color_hist_sz));
         const auto Y_ = floor_hist.project_to_histogram(Vector3(0.0, 0.5, 0.0),
                                                         real(p.color_hist_sz));
         bresenham(O, X_, [&](int x, int y) {
            if(chist.in_bounds(x, y)) chist(x, y) = k_red;
         });
         bresenham(O, Y_, [&](int x, int y) {
            if(chist.in_bounds(x, y)) chist(x, y) = k_green;
         });

         // Draw the cameras
         auto draw_cam_on_floor_hist = [&](const string& camera_id,
                                           const EuclideanTransform& et) {
            const auto chz = p.color_hist_sz;
            auto proj_f    = [&](const Vector3& Y) {
               const auto tl
                   = Vector2(floor_hist.bounds.left, floor_hist.bounds.top);
               return project_to_histogram_f(Y, tl, real(p.color_hist_sz));
            };

            const auto c   = proj_f(et.apply(Vector3(0, 0, 0)));
            const auto d   = proj_f(et.apply(Vector3(0, 0, 10.0)));
            const auto dc  = (d - c).normalised();
            const auto len = 0.3 / real(chz);
            bresenham(c, c + len * dc, [&](int x, int y) {
               set(chist, x, y, k_lawn_green);
            });

            set(chist, int(c.x), int(c.y), k_red);

            const auto sz = render_tiny_dimensions(camera_id);
            const auto off
                = Vector2{-sz.x * 0.5, (dc.y < 0.0 ? 4.0 : -2.0 - sz.y)};
            render_string(chist, camera_id, to_pt2(c + off), k_yellow, k_black);
         };
         for(unsigned cam_num = 0; cam_num < n_cams; ++cam_num) {
            const BinocularCameraInfo& bcam = *bcams[cam_num];
            const EuclideanTransform& et    = *transforms[cam_num];
            draw_cam_on_floor_hist(bcam.camera_id, et);
         }

         // And... SAVE
         const auto fname = format("{}/zzz-kolor-floor.png", outdir);
         chist.save(fname);
         INFO(format("color floor histogram saved to: '{}'", fname));
      }
   }

   { // Apply Guassian smoothing if applicable
      if(std::isfinite(p.gaussian_sigma) and p.gaussian_sigma > 0.0f) {
         int blur_sz = int(std::ceil(p.gaussian_sigma) * 4.0f);

         if(blur_sz % 2 == 0) blur_sz += 1;
         const auto sigma = real(p.gaussian_sigma);
         auto sz          = cv::Size(blur_sz, blur_sz);
         cv::Mat grey     = float_im_to_cv(hist);
         cv::GaussianBlur(grey, grey, sz, sigma, sigma, cv::BORDER_DEFAULT);

         cv_to_float_im(grey, hist);
      }
   }

   { // Clip the histogram values
      for(auto y = 0u; y < hist.height; ++y) {
         for(auto x = 0u; x < hist.width; ++x) {
            auto val = hist(x, y);
            if(val < p.min_hist_count) val = 0.0;
            if(val > p.max_hist_count) val = p.max_hist_count;
            hist(x, y) = val;
         }
      }
   }

   if(false) {
      const auto [min_val, max_val] = hist.minmax();
      FATAL(format("minmax = {}, {}", min_val, max_val));
   }

   return !is_canceled();
}

// ------------------------------------------------------- make-motion-histogram
//
FloatImage This::make_motion_histogram(const Params& p, const FloatImage& hist)
{
   return perceive::make_motion_histogram(hist, p.use_median, real(p.n_spread));
}

// ------------------------------------------------------------------ make-image
//
GreyImage make_hist_image(const FloatImage& hist)
{
   GreyImage g;

   int h                         = int(hist.height);
   int w                         = int(hist.width);
   const auto [min_val, max_val] = hist.minmax();
   const auto inv                = 1.0f / std::min(max_val, 2000.0f);

   g.resize(w, h);

   for(int y = 0; y < h; ++y) {
      const float* h_ptr = hist.ptr(y);
      auto* dst          = g.ptr(y);
      for(int x = 0; x < w; ++x) {
         const auto h = 1.0f - (*h_ptr++ * inv);
         *dst++       = uint8_t(h * 255.0f);
      }
   }

   return g;
}

// ------------------------------------------------------------------ make-image
//
ARGBImage This::make_image(const bool gridlines) const
{
   ARGBImage im;

   int h                         = int(hist.height);
   int w                         = int(hist.width);
   const auto [min_val, max_val] = hist.minmax();
   const auto inv                = 1.0f / std::min(max_val, 2000.0f);

   im.resize(w, h);

   for(int y = 0; y < h; ++y) {
      // const float* m_ptr = motion.ptr(y);
      const float* h_ptr = hist.ptr(y);
      uint32_t* dst      = im.ptr(y);
      for(int x = 0; x < w; ++x) {
         const real h = real(1.0f - (*h_ptr++ * inv));
         *dst++       = heat_map(h, HeatMap::BONE);
      }
   }

   if(gridlines) draw_gridlines(im, bounds, hist_sz, k_orange, 0.5);

   return im;
}

// ---------------------------------------------------------- make-entrance-zone
//
BinaryImage make_entrance_zone(const vector<Vector2>& region,
                               const Vector2& top_left, // in meters
                               const unsigned w,        // width and height
                               const unsigned h,        // of histogram
                               const real hist_sz)      // histogram cell size
{
   BinaryImage out;
   out.resize(w, h, w);
   out.fill(false);
   find_entrance_region(region, top_left, w, h, hist_sz, [&](int x, int y) {
      if(out.in_bounds(x, y)) out(x, y) = true;
   });
   return out;
}

// -------------------------------------------------------- find-entrance-region
//
void find_entrance_region(const vector<Vector2>& region,
                          const Vector2& top_left, // in meters
                          const unsigned w,        // width and height
                          const unsigned h,        // of histogram
                          const real hist_sz,
                          std::function<void(int, int)> f)
{
   const auto N = region.size();
   vector<Vector2> projected_region(N);

   std::transform(cbegin(region),
                  cend(region),
                  begin(projected_region),
                  [&](const auto& X) -> Vector2 {
                     const auto Y = Vector3(X.x, X.y, 0.0);
                     auto p       = FloorHistogram::project_to_histogram(
                         Y, top_left, hist_sz);
                     return to_vec2(p);
                  });

   points_in_polygon(cbegin(projected_region),
                     cend(projected_region),
                     [&](const auto& p) { f(int(p.x), int(p.y)); });
}

// -------------------------------------------------------- draw-entrance-region
//
void draw_entrance_region(ARGBImage& im,
                          const vector<Vector2>& region,
                          const Vector2 top_left,
                          const unsigned width,
                          const unsigned height,
                          const real hist_sz,
                          const uint32_t kolour,
                          const real alpha)
{
   find_entrance_region(
       region, top_left, width, height, hist_sz, [&](int x, int y) {
          if(im.in_bounds(x, y))
             im(x, y) = blend(im(x, y), kolour, float(alpha));
       });
}

void draw_entrance_region(ARGBImage& im,
                          const vector<Vector2>& region,
                          const FloorHistogram& floor_hist,
                          const real hist_sz,
                          const uint32_t kolour,
                          const real alpha)
{
   draw_entrance_region(im,
                        region,
                        Vector2(floor_hist.bounds.left, floor_hist.bounds.top),
                        floor_hist.hist.width,
                        floor_hist.hist.height,
                        hist_sz,
                        kolour,
                        alpha);
}

// -------------------------------------------------------------- draw-gridlines
//
void draw_gridlines(ARGBImage& im,
                    const AABB& bounds,
                    const real hist_sz,
                    const uint32_t kolour,
                    const real alpha)
{
   const auto tl = Vector2(bounds.left, bounds.top);
   AABB b2       = bounds;
   b2.left       = floor(b2.left);
   b2.right      = ceil(b2.right);
   b2.top        = floor(b2.top);
   b2.bottom     = ceil(b2.bottom);

   if(b2.right < b2.left) std::swap(b2.right, b2.left);
   if(b2.bottom < b2.top) std::swap(b2.top, b2.bottom);

   auto proj = [&](real x, real y) {
      return FloorHistogram::project_to_histogram(
          Vector3(x, y, 0.0), tl, hist_sz);
   };

   for(auto y = b2.top - 1.0; y < b2.bottom; y += 1.0)
      bresenham(proj(b2.left, y), proj(b2.right, y), [&](int x, int y) {
         if(im.in_bounds(x, y) and x % 2 == 0)
            im(x, y) = blend(im(x, y), kolour, 0.5);
      });
   for(auto x = b2.left - 1.0; x < b2.right; x += 1.0)
      bresenham(proj(x, b2.top), proj(x, b2.bottom), [&](int x, int y) {
         if(im.in_bounds(x, y) and y % 2 == 0)
            im(x, y) = blend(im(x, y), kolour, 0.5);
      });

   // Draw the origin
   const auto o = to_pt2(proj(0.0, 0.0).round());
   draw_square(im, o, k_red, 2, 0.25);

   // Draw axes
   const auto axis_len = 5;
   for(auto dx = 0; dx < axis_len; ++dx) set(im, o.x + dx, o.y, k_red);
   for(auto dy = 0; dy < axis_len; ++dy) set(im, o.x, o.y + dy, k_green);
   set(im, o, k_blue); // z-axis
}

string str(FloorHistogram::WeightingMethod x) noexcept
{
   switch(x) {
   case FloorHistogram::WGT_NONE: return "WGT_NONE"s;
   case FloorHistogram::WGT_NORM: return "WGT_NORM"s;
   case FloorHistogram::WGT_QUADRANCE: return "WGT_QUADRANCE"s;
   }
   Expects(false);
   return ""s;
}

FloorHistogram::WeightingMethod
to_floor_hist_weighting_method(const string_view s) noexcept(false)
{
   if(s == "WGT_NONE"s) return FloorHistogram::WGT_NONE;
   if(s == "WGT_NORM"s) return FloorHistogram::WGT_NORM;
   if(s == "WGT_QUADRANCE"s) return FloorHistogram::WGT_QUADRANCE;
   throw std::runtime_error(format("bad weighting method value: '{}'", s));
}

// ------------------------------------------- update floor-hist cam-p3 distance
//
void update_floor_hist_cam_p3_distance(
    FloatImage& dist_im,
    const AABB& bounds,
    const real hist_sz,
    const EuclideanTransform& cam_et) noexcept
{
   const auto [w, h]
       = ensure_floor_hist_dims(dist_im, bounds, hist_sz, float(NAN));

   auto process_xy = [&](int x, int y) {
      // Get the location on 'p3'
      // Get the ray to C.
      // Encode the distance
      // Update the floor image
   };

   for(auto y = 0; y < h; ++y)
      for(auto x = 0; x < w; ++x) process_xy(x, y);
}

} // namespace perceive
