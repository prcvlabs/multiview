
#pragma once

#include "perceive/geometry/sparse-hist-cell.hpp"
#include "perceive/graphics/image-container.hpp"
#include "skeleton-2d.hpp"

namespace perceive
{
struct Skeleton2DInfo
{
   int id = -1; // unique only to the localization data
   shared_ptr<const Skeleton2D> p2d_ptr;
   vector<LABImage> patches;
   vector<SparseHistCell> hist;
   float hist_total = 0.0f; // sum(hist.count)
   float prob_fp    = 1.0f;
   FloatImage prob_xy; // includes terms for `hist`, and `prob-fp`

   void init(int id,
             shared_ptr<const Skeleton2D> p2d_ptr,
             const LABImage& lab_im,
             int patch_w,
             int patch_h,
             vector<SparseHistCell>&& hist,
             float prob_fp);

   bool operator==(const Skeleton2DInfo& o) const noexcept;
   bool operator!=(const Skeleton2DInfo& o) const noexcept;
   size_t memory_usage() const noexcept;

   // `xy` and `set_xy` are in histogram coordinates
   void set_xy(int x, int y, float val) noexcept;
   float xy(int x, int y) const noexcept;

   // The following functions use metric space
   float xy(const Vector2f& X, real hist_sz, const AABB& bounds) const noexcept;
   float xy(const Vector3f& X, real hist_sz, const AABB& bounds) const noexcept;
};

// {patch-score, score}, where score in is [0..1], with lower better
std::pair<float, float> patch_still_score(const vector<LABImage>& patches,
                                          const Skeleton2D& p2d,
                                          const LABImage& still) noexcept;

// Converts a "patch score" to an lab-p
float lab_score_to_lab_p(const float lab_score) noexcept;

// {patch-score, score}, where score in is [0..1], with lower better
std::pair<float, float> calc_lab_score(const Skeleton2DInfo& a,
                                       const Skeleton2DInfo& b) noexcept;

// Takes a set of patches, and averages them
void write_skeleton_2d_info(FILE* fp, const Skeleton2DInfo& info);
void read_skeleton_2d_info(FILE* fp, Skeleton2DInfo& info);

struct Skeleton2DInfoPosition
{
   int count   = 0;        // number of skeletons
   Point2 xy   = {-1, -1}; // histogram location for maximal score
   float score = fNAN;     // maximal score: higher is better

   string to_string() const noexcept
   {
      return format(
          "[count={}, xy={{{}, {}}}, score={}", count, xy.x, xy.y, score);
   }

   friend string str(const Skeleton2DInfoPosition& o) noexcept
   {
      return o.to_string();
   }
};

// `infos` is an array of pointers
Skeleton2DInfoPosition position_p2d_infos(const SceneDescription* scene_desc,
                                          const Skeleton2DInfo** infos,
                                          const size_t n_infos,
                                          const real hist_sz,
                                          const AABB& bounds) noexcept;

struct Skeleton2DInfoPositionCalculator
{
   FloatImage mat                     = {};
   const SceneDescription* scene_desc = nullptr;
   int count                          = 0;        // number of skeletons
   Point2 x0                          = {-1, -1}; // for further calculations
   AABB bounds                        = {};
   real hist_sz                       = dNAN;

   void add_skel_info(const Skeleton2DInfo*) noexcept;

   int dxy() const noexcept
   {
      return std::max<int>(int(std::round(0.5 / hist_sz)), 2);
   }

   template<typename T> auto to_hist_xy(const T& X) const noexcept
   {
      return to_pt2(Vector2{(real(X.x) - bounds.left) / hist_sz + 1e-9,
                            (real(X.y) - bounds.top) / hist_sz + 1e-9}
                        .round());
   }

   Point2 hist_X(const Skeleton2DInfo* info_ptr) const noexcept
   {
      return to_hist_xy(info_ptr->p2d_ptr->best_3d_result().Xs_centre());
   }

   Skeleton2DInfoPosition score(const Skeleton2DInfo* info
                                = nullptr) const noexcept;
};

// `infos` is an array of pointers
Skeleton2DInfoPositionCalculator
position_p2d_infos_calculator(const SceneDescription* scene_desc,
                              const Skeleton2DInfo** infos,
                              const size_t n_infos,
                              const real hist_sz,
                              const AABB& bounds) noexcept;

} // namespace perceive
