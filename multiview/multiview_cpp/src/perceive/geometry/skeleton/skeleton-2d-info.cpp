
#include "skeleton-2d-info.hpp"

#include "p2d-affinity.hpp"
#include "perceive/cost-functions/floor-histogram.hpp"
#include "perceive/io/fp-io.hpp"

#define This Skeleton2DInfo

namespace perceive
{
void This::init(int id,
                shared_ptr<const Skeleton2D> p2d_ptr,
                const LABImage& lab_im,
                int patch_w,
                int patch_h,
                vector<SparseHistCell>&& hist,
                float prob_fp)
{
   this->id         = id;
   this->p2d_ptr    = p2d_ptr;
   this->patches    = p2d_ptr->make_image_patches(patch_w, patch_h, lab_im);
   this->hist       = std::move(hist);
   this->hist_total = sparse_histcell_sum(hist);
   this->prob_fp    = prob_fp;
}

bool This::operator==(const This& o) const noexcept
{
   auto test_it = [&](const char* name, auto& lhs, auto& rhs) {
      const bool ret = (lhs == rhs);
      // if(!ret) { INFO(format("{} was not equal", name)); }
      return ret;
   };

   auto test_p2d = [&]() -> bool {
      if((p2d_ptr == nullptr) && (o.p2d_ptr == nullptr)) return true;
      if((p2d_ptr == nullptr) || (o.p2d_ptr == nullptr)) return false;
      string out;
      auto ret = p2d_ptr->test_eq(*o.p2d_ptr, out, false);
      if(false && !ret) {
         p2d_ptr->test_eq(*o.p2d_ptr, out, true);
         LOG_ERR(format("ERROR:"));
         cout << out << endl;
      }
      return ret;
   };

#define TEST(x) (test_it(#x, x, o.x))
   return TEST(id) && TEST(patches) && TEST(hist) && TEST(hist_total)
          && TEST(prob_fp) && test_p2d();
#undef TEST

   return this->p2d_ptr == o.p2d_ptr;
}

bool This::operator!=(const This& o) const noexcept { return !(*this == o); }

size_t This::memory_usage() const noexcept
{
   auto sizeof_labimage = [&](const LABImage& im) { return im.memory_usage(); };

   return sizeof(Skeleton2DInfo) + p2d_ptr->memory_usage()
          + hist.capacity() * sizeof(SparseHistCell)
          + vector_memory_usage(patches, sizeof_labimage)
          + prob_xy.memory_usage();
}

// -------------------------------------------------------------------------- xy
//

void This::set_xy(int x, int y, float val) noexcept
{
   if(prob_xy.in_bounds(x, y)) { prob_xy(x, y) = val; }
}

float This::xy(int x, int y) const noexcept
{
   if(!prob_xy.in_bounds(x, y)) return fNAN;
   return prob_xy(x, y);
}

float This::xy(const Vector2f& X,
               real hist_sz,
               const AABB& bounds) const noexcept
{
   const auto Y   = Vector2{(real(X.x) - bounds.left) / hist_sz + 1e-9,
                          (real(X.y) - bounds.top) / hist_sz + 1e-9};
   const Point2 y = to_pt2(Y.round());
   return xy(y.x, y.y);
}

float This::xy(const Vector3f& X,
               real hist_sz,
               const AABB& bounds) const noexcept
{
   return xy(Vector2f(X.x, X.y), hist_sz, bounds);
}

// ----------------------------------------------------------- patch-still-score
//
std::pair<float, float> patch_still_score(const vector<LABImage>& patches,
                                          const Skeleton2D& p2d,
                                          const LABImage& still) noexcept
{
   Expects(patches.size() > 0);
   const auto patch_w = patches[0].width;
   const auto patch_h = patches[0].height;
   const auto still_patches
       = p2d.make_image_patches(int(patch_w), int(patch_h), still);
   const float patch_score = calc_lab_patch_score(still_patches, patches);
   return {patch_score, lab_score_to_lab_p(patch_score)};
}

// -----------------------------------------------------------------------------
//
float lab_score_to_lab_p(const float lab_score) noexcept
{
   const float lab_z = float((real(lab_score) - cie1976_mean) / cie1976_stddev);
   const auto lab_cdf = phi_function(lab_z - 1.0f);
   return std::clamp<float>(lab_cdf, 0.0f, 1.0f);
}

std::pair<float, float> calc_lab_score(const Skeleton2DInfo& a,
                                       const Skeleton2DInfo& b) noexcept
{
   const float patch_score = calc_lab_patch_score(a.patches, b.patches);

   return {patch_score, lab_score_to_lab_p(patch_score)};
}

// ------------------------------------------------- read/write skeleton-2d-info
//
void write_skeleton_2d_info(FILE* fp, const Skeleton2DInfo& info)
{
   Expects(info.p2d_ptr != nullptr);

   const string p2d_s = info.p2d_ptr->to_json_str();

   save_int(fp, info.id);
   save_str(fp, p2d_s);
   save_vec(fp,
            cbegin(info.patches),
            cend(info.patches),
            write_image_container<LAB>);
   save_vec(fp, cbegin(info.hist), cend(info.hist), write_sparse_hist_cell);
   save_float(fp, info.hist_total);
   save_float(fp, info.prob_fp);
   write_image_container(fp, info.prob_xy);

   { // SANITY
      Skeleton2D op2d;
      try {
         const auto node = parse_json(p2d_s);
         op2d.read(node);
      } catch(std::exception& e) {
         LOG_ERR(format("failed to write/read/eq p2d: {}", e.what()));
         cout << p2d_s << endl << endl;
         FATAL("kBAM!");
      }

      string s;
      if(!info.p2d_ptr->test_eq(op2d, s, false)) {
         LOG_ERR(format("p2d != op2d!"));
         info.p2d_ptr->test_eq(op2d, s, true);
         cout << s << endl;
         cout << p2d_s << endl;
         FATAL("kBAM!");
      }
   }
}

void read_skeleton_2d_info(FILE* fp, Skeleton2DInfo& info)
{
   string p2d_s = ""s;

   load_int(fp, info.id);
   load_str(fp, p2d_s);
   load_vec(fp, info.patches, read_image_container<LAB>);
   load_vec(fp, info.hist, load_sparse_hist_cell);
   load_float(fp, info.hist_total);
   load_float(fp, info.prob_fp);
   read_image_container(fp, info.prob_xy);

   try {
      Skeleton2D s2d;
      s2d.read(parse_json(p2d_s));
      info.p2d_ptr = make_shared<Skeleton2D>(std::move(s2d));
   } catch(std::exception& e) {
      FATAL(format("failed to read Skeleton2D data from file: {}", e.what()));
   }
}

// -------------------------------------------------------------------------- xy
//
Skeleton2DInfoPosition position_p2d_infos(const SceneDescription* scene_desc,
                                          const Skeleton2DInfo** infos,
                                          const size_t n_infos,
                                          const real hist_sz,
                                          const AABB& bounds) noexcept
{
   if(n_infos == 0) return {};

   const auto calc = position_p2d_infos_calculator(
       scene_desc, infos, n_infos, hist_sz, bounds);
   auto s1 = calc.score();
   return s1;

   {
      Skeleton2DInfoPosition out;
      out.count = int(n_infos);
      out.score = 0.0f;

      auto process_cell = [&](int x, int y) {
         float val = 1.0f;
         for(int i = 0; i < out.count; ++i) {
            Expects(infos[i]->prob_xy.in_bounds(x, y));
            val *= infos[i]->xy(x, y);
         }
         Expects(val >= 0.0f && val <= 1.0f);

         if(val > out.score) {
            out.score = val;
            out.xy    = Point2{x, y};
         }

         return val;
      };

      auto to_hist_xy = [&](const auto& X) {
         const auto Y = Vector2{(real(X.x) - bounds.left) / hist_sz + 1e-9,
                                (real(X.y) - bounds.top) / hist_sz + 1e-9};
         return to_pt2(Y.round());
      };

      auto hist_X = [&](int ind) -> Point2 {
         Expects(size_t(ind) < n_infos);
         return to_hist_xy(infos[ind]->p2d_ptr->best_3d_result().Xs_centre());
      };

      // search for 1 metre around the position of `x0`
      const int dxy = std::max<int>(int(std::round(0.5 / hist_sz)), 2);
      const auto x0 = hist_X(0);
      FloatImage mat;
      mat.resize(calc.mat.width, calc.mat.height);
      Expects(int(mat.width) == 2 * dxy + 1);
      Expects(mat.height == mat.width);

      const int xinit = x0.x - dxy;
      const int yinit = x0.y - dxy;
      for(int y = yinit; y <= x0.y + dxy; ++y)
         for(int x = xinit; x <= x0.x + dxy; ++x)
            if(infos[0]->prob_xy.in_bounds(x, y))
               mat(x - xinit, y - yinit) = process_cell(x, y);

      TRACE(format(R"V0G0N(
s1:   {},  x0: {}
out:  {},  x0: {}
)V0G0N",
                   s1.to_string(),
                   calc.x0.to_string(),
                   out.to_string(),
                   x0.to_string()));

      return out;
   }
}

Skeleton2DInfoPositionCalculator
position_p2d_infos_calculator(const SceneDescription* scene_desc,
                              const Skeleton2DInfo** infos,
                              const size_t n_infos,
                              const real hist_sz,
                              const AABB& bounds) noexcept
{
   Expects(n_infos > 0);
   Expects(scene_desc != nullptr);

   // search for 1 metre around the position of `x0`
   Skeleton2DInfoPositionCalculator out;
   out.scene_desc = scene_desc;
   out.count      = int(n_infos);
   out.bounds     = bounds;
   out.hist_sz    = hist_sz;
   out.x0         = out.hist_X(infos[0]);
   const int dxy  = out.dxy();
   out.mat.resize(unsigned(2 * dxy + 1), unsigned(2 * dxy + 1));
   out.mat.fill(1.0f); // The multiplicative identity

   for(int i = 0; i < out.count; ++i) { out.add_skel_info(infos[i]); }

   return out;
}

void Skeleton2DInfoPositionCalculator::add_skel_info(
    const Skeleton2DInfo* info) noexcept
{
   Expects(info != nullptr);
   const int dxy_ = dxy();

   for(auto y = 0; y < int(mat.height); ++y) {
      for(auto x = 0; x < int(mat.width); ++x) {
         const Point2 xy = x0 + Point2{x - dxy_, y - dxy_};
         if(!info->prob_xy.in_bounds(xy)) {
            mat(x, y) = 0.0f;
         } else {
            mat(x, y) *= info->xy(xy.x, xy.y);
            Expects(mat(x, y) >= 0.0f && mat(x, y) <= 1.0f);
         }
      }
   }
}

Skeleton2DInfoPosition Skeleton2DInfoPositionCalculator::score(
    const Skeleton2DInfo* info) const noexcept
{
   if(count == 0) return {};

   Skeleton2DInfoPosition out;
   out.count = this->count + (info == nullptr ? 0 : 1);
   out.score = 0.0f;
   out.xy    = Point2{-1, -1};

   const int dxy_ = dxy();
   for(auto y = 0; y < int(mat.height); ++y) {
      for(auto x = 0; x < int(mat.width); ++x) {
         const Point2 xy = x0 + Point2{x - dxy_, y - dxy_};
         if(info != nullptr && !info->prob_xy.in_bounds(xy)) continue;
         const float val
             = (info == nullptr ? 1.0f : info->xy(xy.x, xy.y)) * mat(x, y);
         Expects(val >= 0.0f && val <= 1.0f);
         if(val > out.score) {
            out.score = val;
            out.xy    = xy;
         }
      }
   }

   return out;
}

} // namespace perceive
