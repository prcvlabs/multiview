
#include "localization-data.hpp"

#include "calc-prob-p2d-false-positive.hpp"

#include "perceive/calibration/training-data/training-data-point.hpp"
#include "perceive/geometry/human-heights.hpp"
#include "perceive/geometry/skeleton/2d-helpers.hpp"
#include "perceive/graphics/bresenham.hpp"
#include "perceive/io/fp-io.hpp"

#define This LocalizationData

namespace perceive
{
const vector<MemberMetaData>& This::Params::meta_data() const noexcept
{
#define ThisParams LocalizationData::Params
   auto make_meta = []() {
      vector<MemberMetaData> m;
      m.push_back(MAKE_META(ThisParams, BOOL, use_median, true));
      m.push_back(MAKE_META(ThisParams, REAL, n_deviations, true));
      m.push_back(MAKE_META(ThisParams, REAL, theta_stddev_degrees, true));
      m.push_back(MAKE_META(ThisParams, REAL, person_diameter, true));
      m.push_back(MAKE_META(ThisParams, REAL, background_median, true));
      m.push_back(MAKE_META(ThisParams, REAL, background_stddev, true));
      m.push_back(MAKE_META(ThisParams, REAL, track_median, true));
      m.push_back(MAKE_META(ThisParams, REAL, track_stddev, true));
      m.push_back(MAKE_META(ThisParams, REAL, prob_bg_max, true));
      m.push_back(MAKE_META(ThisParams, REAL, dist_weight_factor, true));
      m.push_back(MAKE_META(ThisParams, REAL, localization_min, true));
      m.push_back(MAKE_META(ThisParams, REAL, localization_max, true));
      m.push_back(MAKE_META(ThisParams, REAL, filter_radius, true));
      m.push_back(
          MAKE_META(ThisParams, VECTOR4F, false_positive_weight_vector, true));
      m.push_back(MAKE_META(ThisParams, UNSIGNED, pose2d_patch_w, true));
      m.push_back(MAKE_META(ThisParams, UNSIGNED, pose2d_patch_h, true));
      m.push_back(MAKE_META(
          ThisParams, FLOAT, extrinsic_calibration_error_factor, true));
      m.push_back(MAKE_META(ThisParams, BOOL, prefer_openpose_method, true));
      m.push_back(MAKE_META(ThisParams, REAL, prob_shift, true));
      m.push_back(MAKE_META(ThisParams, REAL, prob_scale, true));
      m.push_back(MAKE_META(ThisParams, REAL, zero_value, true));
      m.push_back(MAKE_META(ThisParams, COMPATIBLE_OBJECT, p3d_params, true));
      m.push_back(
          MAKE_META(ThisParams, BOOL, apply_smooth_mhist_to_p3ds, true));
      return m;
   };
   static vector<MemberMetaData> meta_ = make_meta();
   return meta_;
#undef ThisParams
}

// ------------------------------------------------------------------ operator==
//

bool This::operator==(const LocalizationData& o) const noexcept
{
   auto test_it = [&](const char* name, auto& lhs, auto& rhs) {
      const bool ret = (lhs == rhs);
      // if(!ret) { INFO(format("{} was not equal", name)); }
      return ret;
   };

#define TEST(x) (test_it(#x, x, o.x))
   return TEST(loc_hist) and TEST(loc_min) and TEST(loc_max) and TEST(hist_sz)
          and TEST(bounds) and TEST(p2ds);
#undef TEST
}

bool This::operator!=(const LocalizationData& o) const noexcept
{
   return !(*this == o);
}

bool This::in_bounds(int x, int y) const noexcept
{
   return loc_hist.in_bounds(x, y);
}

float This::hist_cost(int x, int y) const noexcept
{
   if(!loc_hist.in_bounds(x, y)) return loc_max;
   constexpr float max_val
       = float(std::numeric_limits<Grey16Image::value_type>::max());
   constexpr float max_val_inv = 1.0f / max_val;
   return scale_01(float(loc_hist(x, y)) * max_val_inv);
}

Vector2 This::project_to_hist(const Vector3& X) const noexcept
{
   return FloorHistogram::project_to_histogram_f(X, bounds.top_left(), hist_sz);
}

// ------------------------------------------------- Read/write to/from a stream
//
void This::read(FILE* fp) noexcept(false)
{
   auto read_version = [&](FILE* fp) {
      int version = 0;

      string version_s;
      load_str(fp, version_s);
      auto pos = version_s.rfind('_');
      if(pos == string::npos)
         throw std::runtime_error(
             "failed to load verion number from localization stream");
      return atoi(&version_s[pos + 1]);
   };

   const int version = read_version(fp);
   if(version != 3) {
      FATAL(format("wrong version number in localization-data::read. Read {}, "
                   "but expected {}",
                   version,
                   3));
   }

   read_image_container(fp, loc_hist);
   load_float(fp, loc_min);
   load_float(fp, loc_max);
   load_real(fp, hist_sz);
   load_int(fp, frame_no);
   for(auto i = 0; i < 4; ++i) load_real(fp, bounds[i]);

   unsigned p2ds_sz = 0;
   load_uint(fp, p2ds_sz);
   p2ds.resize(p2ds_sz);
   for(auto& p2d_vec : p2ds) load_vec(fp, p2d_vec, read_skeleton_2d_info);
}

void This::write(FILE* fp) const noexcept
{
   save_str(fp, "version_3"s);
   write_image_container(fp, loc_hist);
   save_float(fp, loc_min);
   save_float(fp, loc_max);
   save_real(fp, hist_sz);
   save_int(fp, frame_no);
   for(auto i = 0; i < 4; ++i) save_real(fp, bounds[i]);

   save_uint(fp, unsigned(p2ds.size()));
   for(const auto& p2d_vec : p2ds)
      save_vec(fp, cbegin(p2d_vec), end(p2d_vec), write_skeleton_2d_info);
}

// --------------------------------------------------- make_radius_filtered_hist
//
static FloatImage make_radius_filtered_hist(const FloatImage& hist,
                                            const real in_radius,
                                            const real hist_sz) noexcept
{
   const auto& im         = hist;
   const int h            = int(im.height);
   const int w            = int(im.width);
   const float radius_inv = 1.0f / (0.5f * float(in_radius));
   const int radius       = int(std::ceil(in_radius / hist_sz));

   FloatImage ret(im.width, im.height);

   auto process_xy = [&](const int x0, const int y0, vector<float>& xy_s) {
      xy_s.clear();
      for(int dy = -radius; dy <= radius; ++dy)
         for(int dx = -radius; dx <= radius; ++dx)
            if(square(dx) + square(dy) <= square(radius))
               if(im.in_bounds(x0 + dx, y0 + dy)) {
                  const real cell = real(im(x0 + dx, y0 + dy));
                  const auto dist
                      = std::sqrt(square(dx) + square(dy)) * hist_sz;
                  const auto wght = 1.0 - dist * real(radius_inv);
                  if(wght >= 0.0) xy_s.push_back(float(wght * wght * cell));
               }

      const auto N = std::distance(begin(xy_s), end(xy_s));
      return (N == 0) ? 0.0 : calc_average(begin(xy_s), end(xy_s));
   };

   auto process_y = [&](int y) {
      vector<float> xy_s;
      for(auto x = 0; x < w; ++x) ret(x, y) = float(process_xy(x, y, xy_s));
   };

   ParallelJobSet pjobs;
   for(auto y = 0; y < h; ++y)
      pjobs.schedule([y, &process_y]() { process_y(y); });
   pjobs.execute();

   return ret;
}

static FloatImage make_radius_filtered_hist(const FloatImage& motion_hist,
                                            const LocalizationData::Params& p,
                                            const real hist_sz) noexcept
{
   return make_radius_filtered_hist(
       motion_hist, 0.5 * p.person_diameter, hist_sz);
}

// ------------------------------------------------------- make-motion-histogram
//
FloatImage make_motion_histogram(const FloatImage& hist,
                                 const bool use_median,
                                 const real n_deviations) noexcept
{
   const auto w = hist.width;
   const auto h = hist.height;

   FloatImage motion;
   motion.resize(w, h, w);
   motion.fill(0.0f);

   for(auto y = 0u; y < hist.height; ++y) {
      auto hist_ptr   = hist.row_ptr(y);
      auto motion_ptr = motion.row_ptr(y);
      for(auto x = 0u; x < hist.width; ++x) {
         const auto val = *hist_ptr++;
         if(use_median) {
            // const auto diff  = val - s.median;
            // const auto thres = n_deviations * s.absdev;
            // *motion_ptr++    = (diff > thres) ? diff : 0.0f;
            WARN("use-median no longer implemented");
         }

         { // average
            const auto diff  = val;
            const auto thres = n_deviations;
            *motion_ptr++    = (real(diff) > thres) ? diff : 0.0;
         }
      }
   }

   return motion;
}

// ----------------------------------------------------------- make bg histogram

static FloatImage make_bg_histogram_(const FloatImage& radius_hist,
                                     const LocalizationData::Params& p,
                                     const real hist_sz) noexcept
{
   int counter = 0;
   const int h = int(radius_hist.height);
   const int w = int(radius_hist.width);
   FloatImage ret(radius_hist.width, radius_hist.height);
   for(auto y = 0; y < h; ++y) {
      for(auto x = 0; x < w; ++x) {
         const auto p_bg_z = (real(radius_hist(x, y)) - p.background_median)
                             / p.background_stddev;
         ret(x, y) = float(std::min(p.prob_bg_max, phi_function(-p_bg_z)));
      }
   }
   return ret;
}

FloatImage make_bg_histogram(const FloatImage& motion_hist,
                             const LocalizationData::Params& p,
                             const real hist_sz) noexcept
{
   FloatImage rhist = make_radius_filtered_hist(motion_hist, p, hist_sz);
   return make_bg_histogram_(rhist, p, hist_sz);
}

// -------------------------------------------------------- make fg 3d histogram

static FloatImage make_fg_3d_histogram_(const FloatImage& radius_hist,
                                        const LocalizationData::Params& p,
                                        const real hist_sz) noexcept
{
   const int h = int(radius_hist.height);
   const int w = int(radius_hist.width);
   FloatImage ret(radius_hist.width, radius_hist.height);

   auto process_y = [&](int y) {
      for(auto x = 0; x < w; ++x) {
         const auto p_fg_z
             = (real(radius_hist(x, y)) - p.track_median) / p.track_stddev;
         ret(x, y) = float(phi_function(p_fg_z));
      }
   };

   ParallelJobSet pjobs;
   for(auto y = 0; y < h; ++y)
      pjobs.schedule([&process_y, y]() { process_y(y); });
   pjobs.execute();
   return ret;
}

FloatImage make_fg_3d_histogram(const FloatImage& motion_hist,
                                const LocalizationData::Params& p,
                                const real hist_sz) noexcept
{
   FloatImage rhist = make_radius_filtered_hist(motion_hist, p, hist_sz);
   return make_fg_3d_histogram_(rhist, p, hist_sz);
};

// ----------------------------------------------------------- make fg histogram

FloatImage make_fg_histogram(const FloatImage& fg0,
                             const FloatImage& fg1,
                             const LocalizationData::Params& p) noexcept
{
   Expects(fg0.width == fg1.width);
   Expects(fg0.height == fg1.height);

   const int w = int(fg0.width);
   const int h = int(fg0.height);

   FloatImage fg;
   fg.resize(fg0.width, fg0.height);

   for(int y = 0; y < h; ++y)
      for(int x = 0; x < w; ++x)
         fg(x, y) = std::max(fg0(x, y), 0.10f) * std::max(fg1(x, y), 0.10f);

   return fg;
}

FloatImage make_fg_histogram(const FloatImage& motion_hist,
                             const LocalizationData::Params& p,
                             const real hist_sz) noexcept
{
   FloatImage fg0 = make_fg_3d_histogram(motion_hist, p, hist_sz);
   FloatImage fg1 = fg0;
   return make_fg_histogram(fg0, fg1, p);
}

// -------------------------------------------------- make-fg openpose histogram

static std::tuple<FloatImage, FloatImage, vector<vector<Skeleton2DInfo>>>
make_fg_openpose_histogram_(
    const SceneDescription& scene_desc,
    const PoseSkeletonExec::Result& openpose_ret,
    const FloorHistogram& floor_hist,
    std::function<const PointCloud*(int sensor_no)> in_get_point_cloud,
    std::function<const ImageFeatures2d*(int sensor_no)> in_get_f2d,
    std::function<const BinocularCamera*(int cam_no)> in_get_bcam,
    const FloatImage& motion_hist,
    const LocalizationData::Params& p) noexcept
{
   // -- (*) -- Setup
   ParallelJobSet pjobs;

   const int n_sensors           = scene_desc.n_sensors();
   const int w                   = int(floor_hist.hist.width);
   const int h                   = int(floor_hist.hist.height);
   const AABB bounds             = floor_hist.bounds;
   const Vector2 top_left        = bounds.top_left();
   const float hist_sz           = float(floor_hist.hist_sz);
   const real dist_weight_factor = p.dist_weight_factor;
   const bool op_primacy         = true;

   vector<const PointCloud*> pt_clouds((size_t(scene_desc.n_sensors())));
   {
      for(auto i = 0; i < scene_desc.n_sensors(); ++i)
         pt_clouds[size_t(i)] = in_get_point_cloud(i);
   }
   auto get_point_cloud = [&pt_clouds](int sensor_no) -> const PointCloud* {
      Expects(std::size_t(sensor_no) < pt_clouds.size());
      return pt_clouds[size_t(sensor_no)];
   };

   vector<const ImageFeatures2d*> f2ds((size_t(scene_desc.n_sensors())));
   {
      for(auto i = 0; i < scene_desc.n_sensors(); ++i)
         f2ds[size_t(i)] = in_get_f2d(i);
   }
   auto get_f2d = [&f2ds](int sensor_no) -> const ImageFeatures2d* {
      Expects(size_t(sensor_no) < f2ds.size());
      return f2ds[size_t(sensor_no)];
   };

   vector<const BinocularCamera*> bcams((size_t(scene_desc.n_cameras())));
   {
      for(auto i = 0; i < scene_desc.n_cameras(); ++i)
         bcams[size_t(i)] = in_get_bcam(i);
   }
   auto get_bcam = [&bcams](int cam_no) -> const BinocularCamera* {
      Expects(size_t(cam_no) < bcams.size());
      return bcams[size_t(cam_no)];
   };

   FloatImage out, out_heights;

   out.resize(w, h);
   out.fill(0.0f);

   out_heights.resize(w, h);
   out_heights.fill(0.0f);

   auto project_to_hist = [&](const Vector3& X) -> Vector2 {
      return FloorHistogram::project_to_histogram(X, top_left, real(hist_sz));
   };

   // using Keypoint = Skeleton2D::Keypoint;

   // -- (*) -- Outputs

   SpinLock padlock;
   vector<vector<Skeleton2DInfo>> s_p2ds((size_t(n_sensors)));
   {
      int id = 0;
      for(auto i = 0; i < n_sensors; ++i) {
         const int sensor_pos = scene_desc.sensor_position(i);
         if(sensor_pos == 0) {
            s_p2ds[size_t(i)].resize(openpose_ret.pose(i).size());
            s_p2ds[size_t(i)].shrink_to_fit();
            for(auto& info : s_p2ds[size_t(i)]) info.id = id++;
         } else {
            s_p2ds[size_t(i)].clear();
         }
      }
   }

   const auto inv_theta_stddev = 1.0 / to_radians(p.theta_stddev_degrees);

   FloatImage out0, out1; // keep the top two results per sensor
   out0.resize(w, h);
   out1.resize(w, h);
   out0.fill(0.0f);
   out1.fill(0.0f);

   // -- (*) -- Execution -- (*) --

   // -- (*) -- Process a given p2d detection (sensor-num, p2d-idx)
   // Creates the sparse histogram for a given p2d
   auto process_hist_ij = [&](int sensor_num, int p2d_idx, int id) {
      const auto& p2d_vec           = openpose_ret.pose(sensor_num);
      const auto cam_num_sensor_pos = scene_desc.bcam_lookup(sensor_num);
      const int cam_num             = cam_num_sensor_pos.x;
      const int sensor_pos          = cam_num_sensor_pos.y;
      if(sensor_pos != 0) return; // nothing to do
      const bool has_pt_cloud = (get_point_cloud(sensor_num) != nullptr);
      const bool f2d_empty    = get_f2d(sensor_num)->is_empty;

      const auto& et = scene_desc.cam_transforms[size_t(cam_num)];
      Expects(p2d_idx >= 0 && p2d_idx < int(p2d_vec.size()));
      const auto& bcam            = *get_bcam(cam_num);
      const IntImage& slic_labels = get_f2d(sensor_num)->slic_labels;
      const LABImage& lab_im      = get_f2d(sensor_num)->slic_im_LAB;
      Expects(unsigned(cam_num) < floor_hist.camera_info.size());
      const auto& cam_info = floor_hist.camera_info[size_t(cam_num)];
      const auto& dcam     = scene_desc.dcam(sensor_num);

      auto& p2d_ptr         = p2d_vec[size_t(p2d_idx)];
      const Skeleton2D& p2d = *p2d_ptr;
      vector<SparseHistCell> s_hist;

      s_hist.reserve(128);
      if(has_pt_cloud) {
         const auto& ptcloud = *get_point_cloud(sensor_num);
         trace_skeleton(p2d, [&](const Point2& in_x) {
            const auto x
                = to_pt2(bcam.rectify_distorted(sensor_pos, to_vec2(in_x)));
            if(!ptcloud.lookup.in_bounds(x)) return;
            const auto ind = ptcloud.lookup(x);
            if(ind < 0) return;
            const auto& camX = ptcloud.Xs[size_t(ind)];
            const Vector3 X  = et.apply(camX);
            if(X.z < 0.25) return; // TODO, put the parameter in
            const auto xy = to_pt2(project_to_hist(X).round());
            if(!out.in_bounds(xy)) return;
            const auto weight = (camX - et.translation).norm();
            auto ii           = std::find_if(begin(s_hist),
                                   end(s_hist),
                                   [&](const auto& o) { return o.xy == xy; });
            if(ii == end(s_hist)) {
               s_hist.emplace_back(xy, float(weight));
            } else {
               ii->count += float(weight);
            }
         });
      }

      s_hist.shrink_to_fit();

      const auto prob_fp
          = calc_prob_p2d_false_positive(p,
                                         p2d,
                                         &s_hist,
                                         bounds,
                                         real(hist_sz),
                                         dcam,
                                         p.false_positive_weight_vector);

      auto& p2d_info = s_p2ds[size_t(sensor_num)][size_t(p2d_idx)];
      p2d_info.init(id,
                    p2d_vec[size_t(p2d_idx)],
                    lab_im,
                    int(p.pose2d_patch_w),
                    int(p.pose2d_patch_h),
                    std::move(s_hist),
                    prob_fp);
      Expects(p2d_info.p2d_ptr);
      Expects(p2d_info.p2d_ptr->is_interpolation() == false);
   };

   // -- (*) -- Process Sensor
   // For each sensor
   // + Fill out `shist` which gives the spares hist for a given sensor
   //   - We just aggregate the sparse-scores for each p2d spare hist.
   //   - Data comes from skeleton (2d) reading off of the point cloud.
   // + Fill out 's_out`
   //   - Comes from `hist-cell-plausibility(p2d, dcam, X)` for each
   //     histogram cell. Score is weighted by p2d
   auto process_sensor = [&](int sensor_num) {
      auto& p2d_infos  = s_p2ds[size_t(sensor_num)];
      const auto& dcam = scene_desc.dcam(sensor_num);
      FloatImage s_out;
      s_out.resize(w, h);
      s_out.fill(0.0f);

      FloatImage shist;
      shist.resize(w, h);

      for(auto p2d_idx = 0u; p2d_idx < p2d_infos.size(); ++p2d_idx) {
         auto& p2d_info                       = p2d_infos[p2d_idx];
         const auto& p2d_ptr                  = p2d_info.p2d_ptr;
         const vector<SparseHistCell>& s_hist = p2d_info.hist;

         p2d_info.prob_xy.resize(w, h);

         const float min_swgt = 0.15f;
         shist.fill(min_swgt); // This is a float image

         {
            const auto sum = sparse_histcell_sum(s_hist);
            for(const auto& cell : s_hist) {
               const auto wgt0 = float(cell.count / sum);
               const auto wgt1 = std::isfinite(wgt0) ? wgt0 : 0.0f;
               const auto wgt  = std::clamp<float>(wgt1, 0.0f, 1.0f);
               Expects(shist.in_bounds(cell.xy));
               shist(cell.xy) = wgt;
            }
         }

         for(auto y = 0; y < h; ++y) {
            for(auto x = 0; x < w; ++x) {
               const auto X = FloorHistogram::unproject_hist_xyz(
                   Point2(x, y), top_left, real(hist_sz));
               const auto hcp = hist_cell_plausibility(p, *p2d_ptr, dcam, X);
               const auto prob_fp = p2d_info.prob_fp;
               const auto prob    = std::clamp<float>(
                   hcp.probability() * (1.0f - prob_fp), 0.0f, 1.0f);
               p2d_info.set_xy(x, y, prob); // Store this value
               const float score = std::clamp<float>(
                   min_swgt + 0.75f * (shist(x, y) + prob), 0.0f, 1.0f);
               s_out(x, y) = std::max(s_out(x, y), score);
            }
         }
      }

      {
         lock_guard lock(padlock);
         for(auto y = 0; y < h; ++y) {
            for(auto x = 0; x < w; ++x) {
               float& val  = s_out(x, y);
               float& val0 = out0(x, y);
               float& val1 = out1(x, y);
               if(val > val1) std::swap(val, val1);
               if(val1 > val0) std::swap(val1, val0);
            }
         }
      }
   };

   {
      Expects(n_sensors <= int(openpose_ret.size()));
      int id = 0;
      for(auto sensor_ind = 0; sensor_ind < n_sensors; ++sensor_ind) {
         for(auto j = 0u; j < s_p2ds[size_t(sensor_ind)].size(); ++j) {
            pjobs.schedule([sensor_ind, j, id, &process_hist_ij]() {
               process_hist_ij(sensor_ind, int(j), id);
            });
            ++id;
         }
      }
      pjobs.execute();

      for(auto sensor_ind = 0; sensor_ind < n_sensors; ++sensor_ind) {
         pjobs.schedule(
             [sensor_ind, &process_sensor]() { process_sensor(sensor_ind); });
      }
      pjobs.execute();

      // Combine into 'out(x, y)' by multiplying the max 2 probabilities for
      // each cell, but using 'prior' as a floor

      // What is the average `out1` value?
      const auto [min_out1, max_out1] = out1.minmax();
      const auto av_out1_val = 0.33f * std::min<float>(float(max_out1), 1.0f);

      auto process_row = [&](int y) {
         for(auto x = 0; x < w; ++x)
            out(x, y) = logish_prob(out0(x, y)
                                    * std::max<float>(av_out1_val, out1(x, y)));
      };

      for(auto y = 0; y < h; ++y)
         pjobs.schedule([y, &process_row]() { process_row(y); });
      pjobs.execute();
   }

   // And rescale
   if(false) {
      const auto [minval, maxval] = out.minmax();
      for(auto y = 0; y < h; ++y)
         for(auto x = 0; x < w; ++x) out(x, y) /= maxval;
   }

   return std::make_tuple(out, out_heights, s_p2ds);
} // namespace perceive

std::tuple<FloatImage, FloatImage, vector<vector<Skeleton2DInfo>>>
make_fg_openpose_histogram(
    const SceneDescription& scene_desc,
    const PoseSkeletonExec::Result& openpose_ret,
    const FloorHistogram& floor_hist,
    std::function<const PointCloud*(int sensor_no)> get_point_cloud,
    std::function<const ImageFeatures2d*(int sensor_no)> get_f2d,
    std::function<const BinocularCamera*(int cam_no)> get_bcam,
    const FloatImage& motion_hist,
    const LocalizationData::Params& p) noexcept
{
   auto ret = make_fg_openpose_histogram_(scene_desc,
                                          openpose_ret,
                                          floor_hist,
                                          get_point_cloud,
                                          get_f2d,
                                          get_bcam,
                                          motion_hist,
                                          p);
   return ret;
}

// ------------------------------------------------------ make-fowlkes histogram

FloatImage make_fowlkes_histogram(const FloatImage& bg,
                                  const FloatImage& fg,
                                  const LocalizationData::Params& p,
                                  const real hist_sz) noexcept
{
   const float min_val = float(p.loc_min());
   const float max_val = float(p.loc_max());
   const int radius    = int(std::ceil(p.filter_radius / hist_sz));

   Expects(bg.width == fg.width);
   Expects(bg.height == fg.height);

   const int h = int(bg.height);
   const int w = int(bg.width);
   FloatImage p_fg_bg(bg.width, bg.height);

   {
      //
      // Map [0..1] to [max-val..min-val]
      //
      const auto range = max_val - min_val;
      auto map_prob
          = [&](float p) -> float { return (1.0f - p) * range + min_val; };

      for(auto y = 0; y < h; ++y)
         for(auto x = 0; x < w; ++x) p_fg_bg(x, y) = map_prob(fg(x, y));
   }

   // Apply convolution filter
   auto calc_filter_value = [&](int x, int y) {
      if(radius < 2) return p_fg_bg(x, y);
      int counter = 0;
      float sum   = 0.0f;
      for(int dy = -radius; dy <= radius; ++dy)
         for(int dx = -radius; dx <= radius; ++dx) {
            Point2 p(x + dx, y + dy);
            if(p_fg_bg.in_bounds(p)) {
               sum += p_fg_bg(p);
               counter++;
            }
         }
      Expects(counter > 0);
      return sum / float(counter);
   };

   FloatImage ret((unsigned(w)), (unsigned(h)));
   for(auto y = 0; y < h; ++y)
      for(auto x = 0; x < w; ++x) ret(x, y) = calc_filter_value(x, y);

   if(false) { // Clamp values
      for(auto y = 0; y < h; ++y) {
         for(auto x = 0; x < w; ++x) {
            const auto val = std::clamp(ret(x, y), min_val, max_val);
            ret(x, y)      = std::isfinite(val) ? val : max_val;
         }
      }
   }

   return ret;
}

// ------------------------------------------------- load/save localization data

LocalizationDataEnvelope load_localization_data(FILE* fp) noexcept
{
   Expects(fp);

   LocalizationDataEnvelope ret;
   unsigned start_frame = 0, n_frames = 0;
   real frame_duration = 0.0;
   try {
      load_uint(fp, start_frame);
      load_uint(fp, n_frames);
      load_real(fp, frame_duration);
      ret.start_frame    = start_frame;
      ret.frame_duration = frame_duration;
      ret.loc_data.resize(n_frames);
      for(auto i = 0u; i < n_frames; ++i) ret.loc_data[i].read(fp);
   } catch(std::exception& e) {
      FATAL(format("exception reading localization data: {}", e.what()));
   }

   if(!std::isfinite(ret.frame_duration))
      FATAL(format(
          "expected finite frame-duration in localization-data, aborting"));

   return ret;
}

LocalizationDataEnvelope
load_localization_data(const string_view filename) noexcept(false)
{
   FILE* fp = fopen(filename.data(), "r");
   if(fp == nullptr)
      throw std::runtime_error(
          format("failed to open '{}' for reading", filename));
   auto ret = load_localization_data(fp);
   fclose(fp);
   return ret;
}

void save_localization_data(
    FILE* fp,
    const unsigned start_frame,
    const unsigned n_frames,
    const real frame_duration,
    std::function<const LocalizationData*(unsigned)> get_loc,
    const bool verbose) noexcept
{
   Expects(fp);

   save_uint(fp, start_frame);
   save_uint(fp, n_frames);
   save_real(fp, frame_duration);

   auto now    = tick();
   int counter = 0;

   for(auto i = 0u; i < n_frames; ++i) {
      const auto t = start_frame + i;
      // INFO(format("get frame {}", t));
      auto loc_ptr = get_loc(t);
      Expects(loc_ptr);
      loc_ptr->write(fp);

      ++counter;
      if(verbose && (t % 20 == 0)) {
         const auto s  = tock(now);
         const auto t0 = int(t) - counter + 1;
         cout << format("saving loc-data frames [{:4d}-{:4d}] - {}s "
                        "per frame",
                        t0,
                        t,
                        s / counter)
              << endl;
         now     = tick();
         counter = 0;
      }
   }
}

void save_localization_data(
    const string_view filename,
    const unsigned start_frame,
    const unsigned n_frames,
    const real frame_duration,
    std::function<const LocalizationData*(unsigned)> get_loc,
    const bool verbose) noexcept(false)
{
   FILE* fp = fopen(filename.data(), "w");
   if(fp == nullptr)
      throw std::runtime_error(
          format("failed to open '{}' for writing", filename));

   save_localization_data(
       fp, start_frame, n_frames, frame_duration, get_loc, verbose);
   fclose(fp);

   if(true) {
      INFO(format("performing deep-check on ldat save operation"));
      cout << format("   start-frame    = {}", start_frame) << endl;
      cout << format("   n_frames       = {}", n_frames) << endl;
      cout << format("   frame_duration = {}", frame_duration) << endl;
      cout << format("   fps            = {}", 1.0 / frame_duration) << endl;

      // Let's check that this actually worked
      LocalizationDataEnvelope ldat = load_localization_data(filename);

      Expects(start_frame == ldat.start_frame);
      Expects(n_frames == ldat.loc_data.size());
      Expects(is_close(frame_duration, ldat.frame_duration));
      unsigned t = start_frame;
      for(const auto& ldat : ldat.loc_data) {
         const auto odat_ptr = get_loc(t);
         Expects(odat_ptr != nullptr);
         if(ldat == *odat_ptr) {
            // all good
         } else {
            LOG_ERR(format("discrepancy when loading from {}", t));
            cout << "original-data: " << endl;
            cout << indent(odat_ptr->to_string(), 3) << endl;
            cout << "loaded-data: " << endl;
            cout << indent(ldat.to_string(), 3) << endl;
            FATAL("aborting");
         }
         ++t;
      }
      cout << format("   DONE") << endl;
   }
}

// ------------------------------------------------------------------- Calculate

LocalizationData
This::calculate(const SceneDescription& scene_desc,
                const PoseSkeletonExec::Result* pose_ret_ptr,
                const FloorHistogram& floor_hist,
                std::function<const PointCloud*(int sensor_no)> get_point_cloud,
                std::function<const ImageFeatures2d*(int sensor_no)> get_f2d,
                std::function<const BinocularCamera*(int cam_no)> get_bcam,
                const LocalizationData::Params& p,
                const bool feedback,
                const int frame_no) noexcept
{
   const int w        = int(floor_hist.hist.width);
   const int h        = int(floor_hist.hist.height);
   const auto hist_sz = floor_hist.hist_sz;

   const auto& hist = floor_hist.hist;
   const auto motion_hist
       = make_motion_histogram(hist, p.use_median, p.n_deviations);
   const auto rhist = make_radius_filtered_hist(motion_hist, p, hist_sz);
   const auto bg    = make_bg_histogram_(rhist, p, hist_sz);
   const auto fg0   = make_fg_3d_histogram_(rhist, p, hist_sz);

   FloatImage fg1, heights;
   vector<vector<Skeleton2DInfo>> p2d_infos;

   if(pose_ret_ptr == nullptr) {
      fg1 = fg0;
      heights.resize(fg1.width, fg1.height);
      heights.fill(NAN);
   } else {
      std::tie(fg1, heights, p2d_infos)
          = make_fg_openpose_histogram_(scene_desc,
                                        *pose_ret_ptr,
                                        floor_hist,
                                        get_point_cloud,
                                        get_f2d,
                                        get_bcam,
                                        fg0,
                                        p);
   }

   Expects(int(p2d_infos.size()) == scene_desc.n_sensors());

   LocalizationData dat;
   dat.frame_no = frame_no;
   dat.hist_sz  = floor_hist.hist_sz;
   dat.bounds   = floor_hist.bounds;
   dat.loc_min  = float(p.loc_min());
   dat.loc_max  = float(p.loc_max());

   FloatImage fhist;
   if(p.prefer_openpose_method) {
      const float pmid  = float(p.prob_shift);
      fhist             = fg1;
      const float maxv  = float(p.loc_max());
      const float minv  = float(p.loc_min());
      const float range = maxv - minv;
      const float mid   = 0.5f * range + minv;
      const float low   = float(p.loc_min());
      const float* end  = fhist.end();
      for(float* ii = fhist.begin(); ii != end; ++ii) {
         if(*ii == 0.0f) {
            *ii = 0.5f * (1.0f - float(p.zero_value)) * range + low;
         } else {
            auto x = float(p.prob_scale) * (*ii - pmid);
            if(x < float(p.zero_value)) x = float(p.zero_value);
            *ii = 0.5f * (1.0f - x) * range + low;
         }
      }
   } else {
      fhist = fg1;
   }
   dat.loc_hist
       = float_im_to_grey16(fhist, float(p.loc_min()), float(p.loc_max()));

   {
      dat.p2ds = std::move(p2d_infos);
   }

   Expects(dat.invariant_check());

   if(feedback)
      INFO(format(
          "localization[{}] memory usage is {}", frame_no, dat.memory_usage()));

   return dat;
}

string This::to_string() const noexcept
{
   auto hash_s = [&](const auto& im) -> size_t {
      return sdbm_hash(im.begin(), unsigned(im.n_bytes()));
   };

   const auto tot_p2ds = std::accumulate(
       cbegin(p2ds), cend(p2ds), size_t(0), [&](size_t x, auto& y) {
          return x + y.size();
       });

   return format(
       R"V0G0N(
LocalizationData {{
   loc-min/max:    [{}, {}]
   hist-sz:         {}
   bounds:         [{}, {}, {}, {}]
   p2ds:            {},
   loc-hist:       [{}, {}], hash={:08x}
}}
{})V0G0N",
       loc_min,
       loc_max,
       hist_sz,
       bounds.left,
       bounds.top,
       bounds.right,
       bounds.bottom,
       tot_p2ds,
       loc_hist.width,
       loc_hist.height,
       hash_s(loc_hist),
       "");
}

string str(const LocalizationData& dat) noexcept { return dat.to_string(); }

// --------------------------------------------------------- approx-memory-usage
//
size_t This::memory_usage() const noexcept
{
   Expects(p2ds.size() <= p2ds.capacity());

   auto sizeof_p2d_vec = [&](const vector<Skeleton2DInfo>& info_vec) {
      return vector_memory_usage(
          info_vec, [&](auto& info) { return info.memory_usage(); });
   };

   return sizeof(This) + loc_hist.memory_usage() - sizeof(Int16Image)
          - sizeof(Grey16Image) + vector_memory_usage(p2ds, sizeof_p2d_vec);
}

// --------------------------------------------------------- approx-memory-usage
//
bool This::invariant_check() const noexcept
{
   { // Paranoid: ensure there's no interpolations
      for(const auto& sensor_p2ds : p2ds)
         for(const auto& pinfo : sensor_p2ds) {
            if(!pinfo.p2d_ptr) {
               WARN(format("p2d_ptr was null"));
               return false;
            }
            if(pinfo.p2d_ptr->is_interpolation()) {
               WARN(format("found interpolation!"));
               return false;
            }
         }
   }

   return true;
}

} // namespace perceive
