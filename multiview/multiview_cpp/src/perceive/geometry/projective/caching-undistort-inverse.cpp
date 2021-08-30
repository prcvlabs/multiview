
#include "caching-undistort-inverse.hpp"

#include "perceive/graphics/bresenham.hpp"
#include "perceive/io/fp-io.hpp"
#include "perceive/io/ieee754-packing.hpp"
#include "perceive/io/json-io.hpp"
#include "perceive/io/perceive-assets.hpp"
#include "perceive/io/xdr.hpp"
#include "perceive/utils/create-cv-remap.hpp"
#include "perceive/utils/file-system.hpp"
#include "perceive/utils/md5.hpp"

#include <boost/endian/conversion.hpp>

#define This CachingUndistortInverse

namespace perceive
{
static void get_cached_mapxy(const unsigned w,
                             const unsigned h,
                             const Matrix3r& H_in,
                             const CachingUndistortInverse& cu,
                             const Matrix3r& H_out,
                             cv::Mat& mapx,
                             cv::Mat& mapy,
                             const bool feedback = false);

// -----------------------------------------------------------------------------
// --                        CachingUndistortInverse                          --
// -----------------------------------------------------------------------------

namespace detail
{
   struct CachingUndistortInverseStore
   {
    private:
      mutable std::mutex padlock_;
      std::unordered_map<string, CachingUndistortInverse> lookup_;

    public:
      bool store_cu(const string& key,
                    const CachingUndistortInverse& cu) noexcept
      {
         lock_guard<decltype(padlock_)> lock(padlock_);
         lookup_.insert_or_assign(key, cu);
         return true;
      }

      bool load_cu(const string& key,
                   CachingUndistortInverse& cu) const noexcept
      {
         lock_guard<decltype(padlock_)> lock(padlock_);
         const auto ii      = lookup_.find(key);
         const auto success = ii != cend(lookup_);
         if(success) cu = ii->second;
         return success;
      }
   };

   static CachingUndistortInverseStore& cu_store() noexcept
   {
      static CachingUndistortInverseStore store_;
      return store_;
   }
} // namespace detail

// --------------------------------------------------------------------- ApproxK
//
Vector2 This::ApproxK::distort(const Vector2& U) const noexcept
{
   return homgen_P2_to_R2(to_vec3(K_inv_ * Vector3r(U.x, U.y, 1.0)));
}

Vector2 This::ApproxK::undistort(const Vector2& D) const noexcept
{
   return homgen_P2_to_R2(to_vec3(K_ * Vector3r(D.x, D.y, 1.0)));
}

const This::ApproxK* This::nearest_to_undistorted(const Vector2& U) const
{
   auto ii = rng::smallest_elem(
       approx_Ks_, std::less<real>(), [&U](const auto& ak) -> real {
          return (U - ak.Uc_).quadrance();
       });
   return (ii == cend(approx_Ks_)) ? nullptr : &(*ii);
}

const This::ApproxK* This::nearest_to_distorted(const Vector2& D) const
{
   auto ii = rng::smallest_elem(
       approx_Ks_, std::less<real>(), [&D](const auto& ak) {
          return (D - ak.Dc_).quadrance();
       });
   return (ii == cend(approx_Ks_)) ? nullptr : &(*ii);
}

Vector2 This::approx_distort(const Vector2& U) const noexcept
{
   const auto ak_ptr = nearest_to_undistorted(U);
   return (ak_ptr == nullptr) ? Vector2::nan() : ak_ptr->distort(U);
}

Vector2 This::approx_undistort(const Vector2& D) const noexcept
{
   const auto ak_ptr = nearest_to_distorted(D);
   return (ak_ptr == nullptr) ? Vector2::nan() : ak_ptr->undistort(D);
}

// -------------------------------------------------------------- init-approx-Ks
//
void This::init_approx_Ks_()
{
   if(is_K_) {
      // No need to create approx_Ks, when we're already just using 'K'
      approx_Ks_.clear();
      return;
   }

   const auto& M    = M_;
   const auto& hull = M.calib_hull();
   const auto w     = M.calib_format().x;
   const auto h     = M.calib_format().y;
   const auto C     = Vector2(0.5 * w, 0.5 * h);

   array<Vector3, 4> lls; // lines that we intersect the hull with.
   lls[0] = to_homgen_line(C, C + Vector2(1.0, 0.0));  // horizontal
   lls[1] = to_homgen_line(C, C + Vector2(0.0, 1.0));  // vertical
   lls[2] = to_homgen_line(C, C + Vector2(1.0, -1.0)); // diagonal
   lls[3] = to_homgen_line(C, C + Vector2(-1.0, 1.0)); // diagonal

   // Find all the points of intersection between the hull and these 4 lines.
   // *SHOULD* be 8 points
   vector<Vector2> isects;
   isects.reserve(8);
   for(const auto& ll : lls) {
      vector<Vector2> tmp;
      tmp.reserve(2);
      hull_line_intersections(cbegin(hull), cend(hull), ll, tmp);
      isects.insert(end(isects), cbegin(tmp), cend(tmp));
   }

   // Initialize our result object
   auto init_aK = [&](const Vector2& D0) {
      const auto CD = (D0 - C);      // vector from C -> D0
      const auto D  = C + 0.80 * CD; // move 'D0' towards the center

      ApproxK ak;
      ak.Dc_ = D;
      ak.Uc_ = undistort(D);

      Matrix3r K = Matrix3r::Identity();
      // Such that Us[i] = K * Ds[i]
      // TODO, this has to be completed
      // calibration::estimate_homography_LLS(Ds, Us, K);
      ak.K_     = K;
      ak.K_inv_ = K.inverse();

      return ak;
   };

   approx_Ks_.resize(isects.size());
   std::transform(cbegin(isects), cend(isects), begin(approx_Ks_), init_aK);
}

// ------------------------------------------------------------------------ init
//
void This::init(AABB aabb, const DistortionModel& M, EmpiricalFieldF&& field)
{
   M_ = M;
   M_.finalize();
   is_K_     = false;
   K_        = Matrix3r::Identity();
   K_inv_    = K_.inverse();
   aabb_     = aabb;
   A_str_    = M.to_json_string();
   field_    = make_shared<EmpiricalFieldF>(std::move(field));
   calib_fmt = Vector2(M.calib_format().x, M.calib_format().y);
   set_working_format(M.working_format().x, M.working_format().y);
   init_approx_Ks_();
}

// ------------------------------------------------------------------------ init
//
void This::init(const Matrix3r& K)
{
   is_K_  = true;
   K_     = K;
   K_inv_ = K.inverse();
   field_ = make_shared<EmpiricalFieldF>();
   init_approx_Ks_();
}

// ------------------------------------------------------------------------ init
//
void This::init(const DistortionModel& M,
                const bool force_recalc,
                const AABBi aabb_in,
                const bool default_full_image,
                const float pixel_size_in,
                const bool feedback)
{
   if(!default_full_image) WARN(format("ignoring full-image=false setting"));

   const bool use_format  = true;
   const auto use_fmt_str = use_format ? "x"s : "o"s;

   const AABBi aabb       = aabb_in;
   const float pixel_size = pixel_size_in;

   const auto gen_aabb = aabb.area() == 0;
   const auto aabb_s
       = format("{}x{}x{}x{}", aabb.left, aabb.top, aabb.right, aabb.bottom);

   const auto A_str = M.to_json_string();
   // INFO(format("A-str = {}", A_str));

   const auto M_checksum1 = MD5(A_str).finalize().hexdigest();
   Expects(M_checksum1.size() == 32);
   const auto M_checksum
       = format("{}-{}", M_checksum1.substr(0, 16), M_checksum1.substr(16, 32));

   const auto key = format("inverse_{}_{}_{:4.2f}_{}",
                           M.sensor_id(),
                           (gen_aabb ? use_fmt_str : aabb_s),
                           pixel_size,
                           M_checksum);

   TRACE(format("key = {}, force = {}", key, str(force_recalc)));
   if(!force_recalc and detail::cu_store().load_cu(key, *this))
      return; // we're done

   bool loaded = false;

   const bool try_load
       = !force_recalc and data_source_exists(AssetType::CACHE_FILE, key);

   if(try_load) {
      // load it
      TRACE(format("try load: {}", key));

      try {
         fetch(*this, key);

         // Check
         DistortionModel M0;
         M0.from_json(str_to_json(A_str_));
         MatrixXr A    = M.A().array() - M0.A().array();
         const auto sq = A.squaredNorm();
         if(sq > 1e-6) {
            LOG_ERR("cache-file dirty: printing diagnostics");
            cout << "MA = " << endl << M.A() << endl << endl;
            cout << "A0 = " << endl << M0.A() << endl << endl;
            cout << "A  = " << endl << A << endl << endl;
            cout << "sq = " << str_precise(sq) << endl;
            throw std::runtime_error(format("cached file is dirty"));
         } else {
            TRACE(format("GOOD: sq = {}", str_precise(sq)));
         }

         loaded = true;
         if(feedback) {
            const auto err = distort_fidelity_check(*this, false);
            INFO(format("loaded '{}', with fidelity {}", key, err));
         } else if(multiview_trace_mode()) {
            const auto err = distort_fidelity_check(*this, true);
            TRACE(format("loaded '{}', with fidelity {}", key, err));
            // TRACE(format("aabb:  '{}'", aabb_.to_string()));
            // TRACE(format("scale: '{}'", str(scale_)));
            // cout << format("wxh    = {}x{}",
            //                field_->field.width,
            //                field_->field.height)
            //      << endl;
            // cout << format("ppt    = {}", str(field_->ppt)) << endl;
            // cout << format("fxy    = {}", str(field_->fxy)) << endl;
            // cout << format("domain = {}", str(field_->domain)) << endl;
            // auto print_coord = [&](int x, int y) {
            //    cout << format("[{}x{}] = U = {} -> D = {}",
            //                   x,
            //                   y,
            //                   str(field_->inv_transform(Vector2f(x, y))),
            //                   str(field_->field(x, y)))
            //         << endl;
            // };
            // print_coord(0, 0);
            // print_coord(field_->field.width - 1, field_->field.height - 1);
            // cout << endl;
         }
      } catch(std::exception& e) {
         WARN(format("failed to load cached-undistort: {}. (file-key = {})",
                     e.what(),
                     key));
      }
   }

   this->set_working_format(M.calib_format().x, M.calib_format().y);

   if(!loaded) {
      reinit(M, aabb, pixel_size, use_format, key); // calculate it
      store(*this, key);

      { // Sanity check...
         try {
            CachingUndistortInverse cu;
            fetch(cu, key);
            INFO(format("sanity check passed for cu file {}", key));
         } catch(std::exception& e) {
            FATAL(format("This is insane: just stored and reloaded and it "
                         "failed: {}",
                         e.what()));
         }
      }
   }

   this->set_working_format(M.working_format().x, M.working_format().y);
   detail::cu_store().store_cu(key, *this);

   if(false) { // Compare distort...
      const auto U  = M.undistort(Vector2(300, 200));
      const auto D1 = M.distort(U);
      const auto D2 = this->distort(U);
      cout << format("{}: U = {}, |{} - {}| = {}",
                     M.sensor_id(),
                     str(U),
                     str(D1),
                     str(D2),
                     (D1 - D2).norm())
           << endl;
   }
}

// ---------------------------------------------------------------------- reinit
//
void This::reinit(const DistortionModel& in_M,
                  const AABBi distorted_aabb,
                  const float pixel_size,
                  const bool use_format,
                  const string& key)
{
   const auto gen_aabb = (distorted_aabb.area() == 0);

   EmpiricalFieldF field;

   auto M = in_M;
   {
      M.finalize();
      const auto p = M.calib_format();
      M.set_working_format(unsigned(p.x), unsigned(p.y));
   }

   auto AABB_report = [](const string_view name, const AABB& aabb) {
      INFO(format("AABB report: {}", name));
      cout << format(R"V0G0N(
   [{}, {}, {}, {}]
   wxh = [{} x {}]
   area = {}
)V0G0N",
                     aabb.left,
                     aabb.top,
                     aabb.right,
                     aabb.bottom,
                     aabb.width(),
                     aabb.height(),
                     aabb.area());
   };

   AABB u_aabb = AABB::minmax();
   if(gen_aabb) {
      // The rectangle of the distorted image
      const auto d_format_i = M.calib_format();
      const auto d_format   = Vector2(d_format_i.x, d_format_i.y);
      Expects(d_format_i.x == 2592);
      Expects(d_format_i.y == 1944);

      const auto [K, K_err] = M.estimate_homography();

      if(false) {
         INFO(format("Distortiong K estimate is:"));
         cout << K << endl << endl;
         cout << format("K-err = {}", K_err) << endl << endl;
      }

      array<Vector2, 4> format_rect{{{0.0, 0.0},
                                     {d_format.x, 0.0},
                                     {d_format.x, d_format.y},
                                     {0.0, d_format.y}}};

      auto cr_aabb = M.calib_region();
      array<Vector2, 4> calib_rect{{{cr_aabb.left, cr_aabb.top},
                                    {cr_aabb.right, cr_aabb.top},
                                    {cr_aabb.right, cr_aabb.bottom},
                                    {cr_aabb.left, cr_aabb.bottom}}};

      auto f_aabb = AABB::minmax();
      for(const auto& d : format_rect) f_aabb.union_point(M.undistort(d));

      auto c_aabb = AABB::minmax();
      for(const auto& d : calib_rect) c_aabb.union_point(M.undistort(d));

      u_aabb = (use_format) ? f_aabb : c_aabb;

      // // Create the "large" u_aabb
      // // if(use_format)
      // //    for(const auto& d : format_rect)
      // u_aabb.union_point(M.undistort(d));
      // // else
      // //    for(const auto& d : calib_rect)
      // u_aabb.union_point(M.undistort(d));

      // // cout << format("large u-aabb = {}", str(u_aabb)) << endl;

      // LOG_ERR(format("M = {}", M.sensor_id()));
      // LOG_ERR(
      //     format("large c-aabb = {}, f-aabb = {}", str(c_aabb),
      //     str(f_aabb)));

      if(false) {
         const auto R = M.radial_epipole();
         cout << str(M) << endl;
         cout << format("WORKING FORMAT = {}", str(M.working_format())) << endl;
         AABB uc_aabb = AABB::minmax();
         for(const auto& x : M.calib_hull()) {
            const auto y = 0.90 * (x - R) + R;
            cout << format("x = {}   ==>   u = {}", str(y), str(M.undistort(y)))
                 << endl;
            uc_aabb.union_point(M.undistort(y));
         }

         AABB_report("f_aabb", f_aabb);
         AABB_report("c_aabb", c_aabb);
         AABB_report("uc_aabb", uc_aabb);
      }

      // LOG_ERR(format("uc-aabb = {}", str(uc_aabb)));

      if(false) {
         // Now shrink u_aabb to be the inscribed rectangle
         auto process = [&](const Vector2& p, int side) {
            if(side == 0 && p.y < u_aabb.top) u_aabb.top = p.y;
            if(side == 1 && p.x > u_aabb.right) u_aabb.right = p.x;
            if(side == 2 && p.y > u_aabb.bottom) u_aabb.bottom = p.y;
            if(side == 3 && p.x < u_aabb.left) u_aabb.left = p.x;
         };

         for(size_t side = 0; side < 4; ++side) {
            auto f = [&](int x, int y) {
               process(M.undistort(Vector2(x, y)), int(side));
            };
            if(use_format)
               bresenham(format_rect[side], format_rect[(side + 1) % 4], f);
            else
               bresenham(calib_rect[side], calib_rect[(side + 1) % 4], f);
         }
      }

      // cout << format("final u-aabb = {}", str(u_aabb)) << endl;

   } else {
      // Trace the edge of the distorted AABB, and make the AABB in the
      // undistorted region
      array<Point2, 4> quad{
          {Point2(distorted_aabb.left, distorted_aabb.bottom),
           Point2(distorted_aabb.left, distorted_aabb.top),
           Point2(distorted_aabb.right, distorted_aabb.top),
           Point2(distorted_aabb.right, distorted_aabb.bottom)}};

      auto process_xy = [&](int x, int y) {
         auto U = M.undistort(Vector2(x, y));
         u_aabb.union_point(U);
      };

      bresenham(quad[0], quad[1], process_xy);
      bresenham(quad[1], quad[2], process_xy);
      bresenham(quad[2], quad[3], process_xy);
      bresenham(quad[3], quad[0], process_xy);
   }

   // What is the granularity?
   const auto distort_w = (distorted_aabb.width() == 0)
                              ? real(M.calib_format().x)
                              : distorted_aabb.width();
   const unsigned w     = unsigned(std::ceil(distort_w / real(pixel_size)));
   const unsigned h     = unsigned(std::ceil(w / u_aabb.aspect_ratio()));
   field.ppt            = Vector2f(float(u_aabb.left), float(u_aabb.top));
   field.fxy            = to_vec2f(
       Vector2(u_aabb.width() / real(w - 1), u_aabb.height() / real(h - 1)));

   auto undist = [&](const Vector2f& U) -> Vector2f {
      auto D = M.distort(to_vec2(U));
      return to_vec2f(D);
   };

   // FATAL(format("u_aabb = {}", str(u_aabb)));

   static std::mutex padlock;
   {
      lock_guard<decltype(padlock)> lock(padlock);

      if(true) {
         INFO("Regenerating cached-distort function");
         cout << format("sensor = {}", M.sensor_id()) << endl;
         cout << format("aabb   = {} (gen = {})", str(u_aabb), str(gen_aabb))
              << endl;
         cout << format("u-wh   = {}, {}", u_aabb.width(), u_aabb.height())
              << endl;
         cout << format("aspect = {}", u_aabb.aspect_ratio()) << endl;
         cout << format("wxh    = {}x{}", w, h) << endl;
         cout << format("ppt    = {}", str(field.ppt)) << endl;
         cout << format("fxy    = {}", str(field.fxy)) << endl;
      }

      field.init(undist, u_aabb, w, true, true);
   }

   init(u_aabb, M, std::move(field));

   {
      const float err = float(distort_fidelity_check(*this, true));
      if(err > 2.0f * pixel_size) {
         // auto print_coord = [&](int x, int y) {
         //    INFO(format("[{}x{}] = U = {} -> D = {}",
         //                x,
         //                y,
         //                str(field_->inv_transform(Vector2f(x, y))),
         //                str(this->field_->field(x, y))));
         // };
         // print_coord(0, 0);
         // print_coord(w - 1, h - 1);

         FATAL(format(
             "Distortion fidelity is TERRIBLE, this should never happen."));
      }
   }
}

// ------------------------------------------------------ distort-fidelity-check
//
real This::distort_fidelity_check(const CachingUndistortInverse& in_cu,
                                  const bool feedback) noexcept
{
   // -- (*) -- We need to set the currect working format
   const Point2 native_format = in_cu.M().calib_format();
   CachingUndistortInverse cu_;
   const CachingUndistortInverse* cu_ptr = &in_cu;
   if((in_cu.working_format() - to_vec2(native_format)).norm() > 1e-5) {
      cu_ = in_cu;
      cu_.set_working_format(native_format.x, native_format.y);
      cu_ptr = &cu_;
   }
   Expects(cu_ptr != nullptr);
   const CachingUndistortInverse& cu = *cu_ptr;

   // -- (*) -- Get a set of 5 points
   array<Vector2f, 5> Ds;
   array<Vector2f, 5> Us;
   array<Vector2f, 5> Vs;
   array<real, 5> DV_err;

   const Vector2f C = 0.5f * to_vec2f(Point2(native_format.x, native_format.y));

   Ds[0] = C;
   Ds[1] = C + 0.25f * to_vec2f(Point2(native_format.x, native_format.y));
   Ds[2] = C + 0.25f * to_vec2f(Point2(native_format.x, -native_format.y));
   Ds[3] = C + 0.25f * to_vec2f(Point2(-native_format.x, native_format.y));
   Ds[4] = C + 0.25f * to_vec2f(Point2(-native_format.x, -native_format.y));

   std::transform(cbegin(Ds), cend(Ds), begin(Us), [&](const auto& D) {
      return to_vec2f(cu.undistort(to_vec2(D)));
   });

   std::transform(cbegin(Us), cend(Us), begin(Vs), [&](const auto& U) {
      return to_vec2f(cu.fast_distort(to_vec2(U)));
   });

   std::transform(cbegin(Ds),
                  cend(Ds),
                  cbegin(Vs),
                  begin(DV_err),
                  [](const Vector2f& a, const Vector2f& b) -> real {
                     return real((a - b).norm());
                  });

   const auto largest_itr = rng::smallest_elem(DV_err, std::greater<real>());
   Expects(largest_itr != cend(DV_err));

   if(feedback) {
      TRACE(format("Fidelity report for cu {}", cu.M().sensor_id()));
      const auto max_ind = std::distance(cbegin(DV_err), largest_itr);
      for(auto i = 0u; i < Ds.size(); ++i) {
         const char label = (i == max_ind) ? '*' : ' ';
         cout << format(" {:c} err = {} :: {} -> {} -> {} (slow-error "
                        "= {})",
                        label,
                        DV_err[i],
                        str(Ds[i]),
                        str(Us[i]),
                        str(Vs[i]),
                        (Ds[i] - cu.slow_distort(Us[i])).norm())
              << endl;
      }
   }

   return *largest_itr;
}

// ------------------------------------------------------------------- Load/Save
//
static const string k_caching_undistort_magic
    = "MagicCachingUndistortInverse_v6";

void load(CachingUndistortInverse& data, const string& fname) noexcept(false)
{
   using deleter_t = std::function<void(FILE*)>;
   unique_ptr<FILE, deleter_t> fp{fopen(fname.c_str(), "rb"),
                                  [](FILE* fp) { fclose(fp); }};

   if(!fp)
      throw std::runtime_error(
          format("failed to open '{}' for reading", fname));
   read(data, fp.get());
}

void save(const CachingUndistortInverse& data,
          const string& fname) noexcept(false)
{
   using deleter_t = std::function<void(FILE*)>;
   unique_ptr<FILE, deleter_t> fp{fopen(fname.c_str(), "wb"),
                                  [](FILE* fp) { fclose(fp); }};

   if(!fp)
      throw std::runtime_error(
          format("failed to open '{}' for writing", fname));
   write(data, fp.get());
}

void read(CachingUndistortInverse& data, FILE* fp) noexcept(false)
{
   string magic;
   load_str(fp, magic);

   if(magic != k_caching_undistort_magic)
      throw std::runtime_error(format("failed to read magic number"));

   AABB aabb;
   auto w = 0u;
   auto h = 0u;
   EmpiricalFieldF field;
   string A_str;
   string M_json_str;
   Point2 native_format{0, 0};
   Vector2 epipole{0.0, 0.0};
   real scale = 0.0;
   DistortionModel M;

   vector<Vector2f> Ds;
   vector<Vector2f> Us;
   vector<Vector2f> Vs;
   vector<float> dist;

   // Load the model
   load_str(fp, M_json_str);
   read(M, M_json_str);

   load_uint(fp, w);
   load_uint(fp, h);
   load_vec2f(fp, field.ppt);
   load_vec2f(fp, field.fxy);

   double val = 0.0;
   load_real(fp, aabb.left);
   load_real(fp, aabb.top);
   load_real(fp, aabb.right);
   load_real(fp, aabb.bottom);

   { // load the underlying model
      load_str(fp, A_str);
      load_int(fp, native_format.x);
      load_int(fp, native_format.y);
      load_real(fp, epipole.x);
      load_real(fp, epipole.y);
      load_real(fp, scale);
   }

   { // Let's load some results from the field.
      load_vec(fp, Ds, load_vec2f);
      load_vec(fp, Us, load_vec2f);
      load_vec(fp, Vs, load_vec2f);
      load_vec(fp, dist, load_float);
   }

   string hash = ""s;
   load_str(fp, hash);

   vector<uint32_t> buffer(2 * h * w);
   const auto n = fread(&buffer[0], sizeof(uint32_t), buffer.size(), fp);
   if(n != buffer.size())
      throw std::runtime_error(format(
          "read {} elements, but expected to read {}", n, buffer.size()));

   const auto calc_hash = md5(&buffer[0], buffer.size() * sizeof(uint32_t));
   if(hash != calc_hash) {
      throw std::runtime_error(
          format("hash sum failed: got {}, but expected {}", calc_hash, hash));
   }

   std::atomic<int> nfinite_counter{0};
   field.field.resize(w, h, w);
   auto& ff         = field.field;
   auto process_row = [w, &ff, &buffer, &nfinite_counter](int y) {
      size_t pos = size_t(y) * size_t(w) * 2;
      for(auto x = 0u; x < w; ++x) {
         Vector2f val;
         val.x = unpack_f32(boost::endian::little_to_native(buffer[pos++]));
         val.y = unpack_f32(boost::endian::little_to_native(buffer[pos++]));
         ff.pixels[x + size_t(y) * ff.row_stride] = val;
         if(!val.is_finite()) nfinite_counter++;
      }
   };

   ParallelJobSet pjobs;
   for(auto y = 0u; y < field.field.height; ++y)
      pjobs.schedule([&process_row, y]() { process_row(int(y)); });
   pjobs.execute();

   if(nfinite_counter > 0) {
      WARN(format("loaded {} non-finite field values", nfinite_counter));
   }

   if(false) {
      INFO("READ");
      cout << k_caching_undistort_magic << endl;
      // cout << data.A_str() << endl;
      cout << format("wxh  = [{}x{}]", w, h) << endl;
      cout << format("aabb = {}", str(aabb)) << endl;
      cout << format("ppt  = {}", str(field.ppt)) << endl;
      cout << format("fxy  = {}", str(field.fxy)) << endl;
      for(auto y = 0u; y < field.field.height; ++y) {
         for(auto x = 0u; x < field.field.width; ++x) {
            const auto& v = field.field(x, y);
            if(x == 10 and y == 10)
               printf("[%5.2f, %5.2f]\n", real(v.x), real(v.y));
         }
      }
      cout << endl;
   }

   data.init(aabb, M, std::move(field));

   {
      // CachingUndistortInverse cu_raw(data);
      CachingUndistortInverse& cu_raw = data;
      cu_raw.set_working_format(native_format.x, native_format.y);

      // WARN(format("cu-raw.id = {}", M.sensor_id()));

      auto fast_distort_2 = [&](const Vector2f& x) {
         auto f = cu_raw.field_->evaluate(x, true);
         auto X = to_vec2f(
             Vector2(real(f.x) * cu_raw.scale_.x, real(f.y) * cu_raw.scale_.y));
         cout << format(" >>> x = {}", str(x)) << endl;
         cout << format(" >>> f = {}", str(f)) << endl;
         cout << format(" >>> X = {}", str(X)) << endl;
         return X;
      };

      auto check_for_error = [&](const bool report) {
         bool has_error = false;
         {
            if(report) {
               INFO(format("LOAD REPORT, N = {}", Ds.size()));
               cout << format("native-format = {}", str(native_format)) << endl;
            }
            for(auto i = 0u; i < Ds.size(); ++i) {
               const Vector2f& D = Ds[i];
               const Vector2f U  = cu_raw.undistort(D);
               const Vector2f Ds = cu_raw.slow_distort(U);
               // const Vector2f Df = fast_distort_2(U);
               const Vector2f Df = cu_raw.fast_distort(U);
               const bool pass
                   = (D - Ds).norm() < 2.0f and (D - Df).norm() < 2.0f;
               if(!has_error) has_error = !pass;

               if(report) {
                  cout << format("  #{}... D = {}", i, str(D)) << endl;
                  cout << format("         U = |{} - {}| = {}\n",
                                 str(U),
                                 str(Us[i]),
                                 (U - Us[i]).norm());
                  cout << format("         Ds = |{} - {}| = {}\n",
                                 str(D),
                                 str(Ds),
                                 (D - Ds).norm());
                  cout << format("         Df = |{} - {}| = {}\n",
                                 str(D),
                                 str(Df),
                                 (D - Df).norm());
                  cout << format("      pass = {}\n", str(pass));

                  cout << endl;
               }
            }
         }
         return has_error;
      };

      const bool has_error = check_for_error(false);
      if(has_error) {
         WARN(format("Internal file-testcase failed in cache-undistort file, "
                     "sensor-id = '{}'",
                     M.sensor_id()));
         check_for_error(true);
         throw std::runtime_error("internal file-testcase failed");
      }
   }
}

void write(const CachingUndistortInverse& data, FILE* fp) noexcept(false)
{
   const auto& field = data.field();

   save_str(fp, k_caching_undistort_magic);

   // Save the model
   save_str(fp, data.M().to_json_string());

   save_uint(fp, field.field.width);
   save_uint(fp, field.field.height);
   save_vec2f(fp, field.ppt);
   save_vec2f(fp, field.fxy);

   save_real(fp, data.aabb().left);
   save_real(fp, data.aabb().top);
   save_real(fp, data.aabb().right);
   save_real(fp, data.aabb().bottom);

   const Point2 native_format = data.M().calib_format();
   const Vector2 epipole      = data.M().radial_epipole();
   const real scale           = data.M().scale();

   { // Save something about the underlying model
      save_str(fp, data.A_str());
      save_int(fp, native_format.x);
      save_int(fp, native_format.y);
      save_real(fp, epipole.x);
      save_real(fp, epipole.y);
      save_real(fp, scale);
   }

   { // Let's save some results from the field.
      // (The files stores it's own test-cases.)
      CachingUndistortInverse cu_raw(data);
      cu_raw.set_working_format(native_format.x, native_format.y);

      vector<Vector2f> Ds(5);
      vector<Vector2f> Us(5);
      vector<Vector2f> Vs(5);
      vector<float> dist(5);

      const Vector2f C
          = 0.5f * to_vec2f(Point2(native_format.x, native_format.y));

      Ds[0] = C;
      Ds[1] = C + 0.25f * to_vec2f(Point2(native_format.x, native_format.y));
      Ds[2] = C + 0.25f * to_vec2f(Point2(native_format.x, -native_format.y));
      Ds[3] = C + 0.25f * to_vec2f(Point2(-native_format.x, native_format.y));
      Ds[4] = C + 0.25f * to_vec2f(Point2(-native_format.x, -native_format.y));

      std::transform(cbegin(Ds), cend(Ds), begin(Us), [&](const auto& D) {
         return to_vec2f(cu_raw.undistort(to_vec2(D)));
      });

      std::transform(cbegin(Us), cend(Us), begin(Vs), [&](const auto& U) {
         return to_vec2f(cu_raw.fast_distort(to_vec2(U)));
      });

      for(auto i = 0u; i < Ds.size(); ++i) dist[i] = (Ds[i] - Vs[i]).norm();

      // Save the field values.
      save_vec(fp, cbegin(Ds), cend(Ds), save_vec2f);
      save_vec(fp, cbegin(Us), cend(Us), save_vec2f);
      save_vec(fp, cbegin(Vs), cend(Vs), save_vec2f);
      save_vec(fp, cbegin(dist), cend(dist), save_float);

      if(false) {
         INFO(format("SAVE REPORT, N = {}", Ds.size()));
         cout << format("native-format = {}", str(native_format)) << endl;
         for(auto i = 0u; i < Ds.size(); ++i) {
            cout << format("  #{}  |{} - {}| = {}, via {}",
                           i,
                           str(Ds[i]),
                           str(Vs[i]),
                           dist[i],
                           str(Us[i]))
                 << endl;
         }
         cout << format("   field(10, 10) = {}", str(field.field(10, 10)))
              << endl;
      }
   }

   vector<uint32_t> buffer(2 * field.field.height * field.field.width);
   size_t pos = 0;
   for(auto y = 0u; y < field.field.height; ++y) {
      for(auto x = 0u; x < field.field.width; ++x) {
         const Vector2f xy = field.field(x, y);
         buffer[pos++]     = boost::endian::native_to_little(pack_f32(xy.x));
         buffer[pos++]     = boost::endian::native_to_little(pack_f32(xy.y));
      }
   }

   const auto hash = md5(&buffer[0], buffer.size() * sizeof(uint32_t));
   save_str(fp, hash);

   const auto n = fwrite(&buffer[0], sizeof(uint32_t), buffer.size(), fp);
   if(n != buffer.size())
      throw std::runtime_error(format(
          "wrote {} elements, but expected to write {}", n, buffer.size()));

   if(false) {
      INFO("WRITE");
      cout << k_caching_undistort_magic << endl;
      //  cout << data.A_str() << endl;
      cout << format("wxh  = [{}x{}]", field.field.width, field.field.height)
           << endl;
      cout << format("aabb = {}", str(data.aabb())) << endl;
      cout << format("ppt  = {}", str(field.ppt)) << endl;
      cout << format("fxy  = {}", str(field.fxy)) << endl;
      for(auto y = 0u; y < field.field.height; ++y) {
         for(auto x = 0u; x < field.field.width; ++x) {
            const auto& v = field.field(x, y);
            printf("[%5.2f, %5.2f]  ", real(v.x), real(v.y));
         }
         cout << endl;
      }
   }
}

// -----------------------------------------------------------------------------
// --                       CachingUndistortInverse XDR                       --
// -----------------------------------------------------------------------------

void write_xdr(const CachingUndistortInverse& data, FILE* fp) noexcept(false)
{
   XdrWrapper xdrs(fp, XDR_ENCODE);
   const auto& field = data.field();
   unsigned w = field.field.width, h = field.field.height;

   xdr_encode(xdrs, k_caching_undistort_magic);
   xdr_encode(xdrs, data.aabb());
   xdr_encode(xdrs, data.A_str());
   xdr_encode(xdrs, w);
   xdr_encode(xdrs, h);
   xdr_encode(xdrs, field.ppt);
   xdr_encode(xdrs, field.fxy);
   for(auto y = 0u; y < h; ++y)
      for(auto x = 0u; x < w; ++x) xdr_encode(xdrs, field.field(x, y));

   for(auto y = 0u; y < field.field.height; ++y) {
      for(auto x = 0u; x < field.field.width; ++x) {
         const auto& v = field.field(x, y);
         printf("{%5.2f, %5.2f}  ", real(v.x), real(v.y));
      }
      cout << endl;
   }
}

void save_xdr(const CachingUndistortInverse& data,
              const string& fname) noexcept(false)
{
   using deleter_t = std::function<void(FILE*)>;
   unique_ptr<FILE, deleter_t> fp{fopen(fname.c_str(), "wb"),
                                  [](FILE* fp) { fclose(fp); }};
   if(!fp)
      throw std::runtime_error(
          format("failed to open '{}' for writing", fname));
   write_xdr(data, fp.get());
}

void read_xdr(CachingUndistortInverse& data, FILE* fp) noexcept(false)
{
   FATAL(format("funcitonality is now broken."));
   XdrWrapper xdrs(fp, XDR_DECODE);
   string magic;
   string A_str;
   unsigned w = 0u, h = 0u;
   AABB aabb;
   EmpiricalFieldF field;

   xdr_decode(xdrs, magic);
   if(magic != k_caching_undistort_magic)
      throw std::runtime_error(format("failed to read magic number"));

   xdr_decode(xdrs, aabb);
   xdr_decode(xdrs, A_str);
   xdr_decode(xdrs, w);
   xdr_decode(xdrs, h);
   xdr_decode(xdrs, field.ppt);
   xdr_decode(xdrs, field.fxy);

   field.field.resize(w, h, w);
   for(auto y = 0u; y < h; ++y)
      for(auto x = 0u; x < w; ++x) {
         Vector2f f;
         xdr_decode(xdrs, f);
         field.field(x, y) = f;
      }

   DistortionModel M;
   data.init(aabb, M, std::move(field));
}

void load_xdr(CachingUndistortInverse& data,
              const string& fname) noexcept(false)
{
   using deleter_t = std::function<void(FILE*)>;
   unique_ptr<FILE, deleter_t> fp{fopen(fname.c_str(), "rb"),
                                  [](FILE* fp) { fclose(fp); }};
   if(!fp)
      throw std::runtime_error(
          format("failed to open '{}' for reading", fname));
   read_xdr(data, fp.get());
}

// ---------------------------------------------------------------- Cached MapXY
//
static void get_cached_mapxy(const unsigned w,
                             const unsigned h,
                             const Matrix3r& H_in,
                             const CachingUndistortInverse& cu,
                             const Matrix3r& H_out,
                             cv::Mat& mapx,
                             cv::Mat& mapy,
                             const bool feedback)
{
   static std::unordered_map<string, array<cv::Mat, 2>> mapxy_dict;
   static std::mutex padlock;

   // Calculate the key
   auto str_it = [&](const Matrix3r& H) -> string {
      MD5 md5;
      for(auto r = 0; r < 3; ++r) {
         for(auto c = 0; c < 3; ++c) {
            auto v = H(r, c);
            md5.update(&v);
         }
      }
      md5.finalize();
      return md5.hexdigest();
   };
   const string key   = format("cached-mapxy_CU={}_{}x{}_Hin={}_Hout={}",
                             cu.A_str(),
                             w,
                             h,
                             str_it(H_in),
                             str_it(H_out));
   const string fname = format("{}/{}.cv", multiview_cache_dir(), key);

   { // (*) Have we already loaded mapx and mapy into memory?
      lock_guard<decltype(padlock)> lock(padlock);
      auto ii = mapxy_dict.find(key);
      if(ii != mapxy_dict.end()) {
         if(feedback) INFO(format("loading cached mapxy({}) from memory", key));
         mapx = ii->second[0];
         mapy = ii->second[1];
         return;
      }
   }

   // Function to save mapxy in the cache
   auto cache_mapxy = [&]() {
      lock_guard<decltype(padlock)> lock(padlock);
      mapxy_dict[key] = array<cv::Mat, 2>{{mapx, mapy}};
   };

   { // (*) Does mapxy exist on disk?
      if(is_regular_file(fname)) {
         if(feedback) INFO(format("loading cached mapxy({}) from disk", key));

         try {
            cv::FileStorage fs(fname.c_str(), cv::FileStorage::READ);
            fs["mapx"] >> mapx;
            fs["mapy"] >> mapy;
            cache_mapxy(); // Save to cache
            return;
         } catch(...) {
            FATAL(format("Input/ouput error"));
         }
      }
   }

   { // (*) We must calculate mapyxy, boo!
      if(feedback) INFO(format("recalculating cached mapxy({})", key));

      auto f = [&](const Vector2& x) { return cu.distort(x); };
      create_cv_remap(w, h, H_in, f, H_out, mapx, mapy);
      cache_mapxy(); // save to cache

      // Now write the files for next time...
      try {
         if(feedback) INFO(format("saving cached mapxy({}) to disk", key));
         cv::FileStorage fs(fname.c_str(), cv::FileStorage::WRITE);
         fs << "mapx" << mapx;
         fs << "mapy" << mapy;
      } catch(...) {
         FATAL(format("Input/output error"));
      }
   }
}

} // namespace perceive
