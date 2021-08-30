
#pragma once

#include "skeleton-2d.hpp"

namespace perceive
{
// Affinity of a p2d in one sensor, with that in another sensor.
struct P2DAffinity
{
   int l_idx             = -1;
   int r_idx             = -1;
   float lab_patch_score = fNAN; // How well do color patches match?

   P2DAffinity()                   = default;
   P2DAffinity(const P2DAffinity&) = default;
   P2DAffinity(P2DAffinity&&)      = default;
   ~P2DAffinity()                  = default;
   P2DAffinity& operator=(const P2DAffinity&) = default;
   P2DAffinity& operator=(P2DAffinity&&) = default;

   P2DAffinity(int l_idx_, int r_idx_, float p_score_ = fNAN) noexcept;
};

float calc_lab_patch_score(const vector<LABImage>& patches_a,
                           const vector<LABImage>& patches_b) noexcept;

// Memoize the P2DAffinity scores for the cartisian product of
// Skeleton2Ds in `p2ds_`
struct AffinityMemo
{
 private:
   bool is_symmetric_ = false;
   unsigned n_idx0s_  = 0;
   unsigned n_idx1s_  = 0; // (is_symmetric) implies (n_idx0s == n_idx1s)

   using IdxRet = std::pair<const Skeleton2D*, const vector<LABImage>*>;
   std::function<IdxRet(unsigned idx)> get_idx0_ = nullptr;
   std::function<IdxRet(unsigned idx)> get_idx1_ = nullptr;

   mutable vector<P2DAffinity> data_;

 public:
   // Symmetric affinities (i.e., intra-frame)
   static unsigned calc_index(const int N, int idx0, int idx1) noexcept;

   void init_symmetric(const unsigned N,
                       std::function<IdxRet(unsigned idx)> get_idx) noexcept;
   void init(const unsigned n_idx0s,
             const unsigned n_idx1s,
             std::function<IdxRet(unsigned idx)> get_idx0,
             std::function<IdxRet(unsigned idx)> get_idx1) noexcept;
   void force_calc_all() noexcept; // in parallel

   P2DAffinity affinity(unsigned l_idx, unsigned r_idx) const noexcept;

   size_t memory_usage() const noexcept;
};

} // namespace perceive
