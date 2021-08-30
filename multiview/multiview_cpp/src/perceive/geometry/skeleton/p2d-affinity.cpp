
#include "p2d-affinity.hpp"

#include "perceive/cost-functions/floor-histogram.hpp"

#define This P2DAffinity

namespace perceive
{
// ---------------------------------------------------------------- Construction
//
This::P2DAffinity(int l_idx_, int r_idx_, float p_score) noexcept
    : l_idx(l_idx_)
    , r_idx(r_idx_)
    , lab_patch_score(p_score)
{}

// ------------------------------------------------------------- LAB Patch Score
//
float calc_lab_patch_score(const vector<LABImage>& patches_a,
                           const vector<LABImage>& patches_b) noexcept
{
   auto product = [](const LABImage& A, const LABImage& B) {
      Expects(A.width * A.height > 0);
      return dot_product(A, B) / float(A.width * A.height);
   };

   Expects(patches_a.size() == patches_b.size());
   auto ret = 0.0f;
   for(auto i = 0u; i < patches_a.size(); ++i)
      ret += product(patches_a[i], patches_b[i]);
   const float av_dot_lab = ret / float(patches_a.size());
   return av_dot_lab;
}

// ------------------------------------------------------------- LAB Patch Score
//
unsigned AffinityMemo::calc_index(const int N, int idx0, int idx1) noexcept
{
   if(idx1 > idx0) std::swap(idx0, idx1);
   const int z = (N - idx0);
   return unsigned((N * N + N) / 2 - (z * N - (z * z - z) / 2) + idx1);
}

void AffinityMemo::init_symmetric(
    const unsigned N,
    std::function<IdxRet(unsigned idx)> get_idx) noexcept
{
   is_symmetric_ = true;
   n_idx0s_      = N;
   get_idx0_     = std::move(get_idx);
   data_.resize((N * N + N) / 2);
   data_.shrink_to_fit();
}

void AffinityMemo::init(const unsigned n_idx0s,
                        const unsigned n_idx1s,
                        std::function<IdxRet(unsigned idx)> get_idx0,
                        std::function<IdxRet(unsigned idx)> get_idx1) noexcept
{
   is_symmetric_ = false;
   n_idx0s_      = n_idx0s;
   n_idx1s_      = n_idx1s;
   get_idx0_     = std::move(get_idx0);
   get_idx1_     = std::move(get_idx1);
   data_.resize(n_idx0s * n_idx1s);
   data_.shrink_to_fit();
}

void AffinityMemo::force_calc_all() noexcept
{
   auto process_i
       = [&](unsigned l_idx, unsigned r_idx) { affinity(l_idx, r_idx); };

   ParallelJobSet pjobs;
   if(is_symmetric_) {
      for(auto i = 0u; i < n_idx0s_; ++i)
         for(auto j = i; j < n_idx0s_; ++j)
            pjobs.schedule([i, j, &process_i]() { process_i(i, j); });
   } else {
      for(auto i = 0u; i < n_idx0s_; ++i)
         for(auto j = 0u; j < n_idx1s_; ++j)
            pjobs.schedule([i, j, &process_i]() { process_i(i, j); });
   }
   pjobs.execute();

   if(is_symmetric_) { // Sanity check... everything was calculated
      const auto N = n_idx0s_;
      for(auto i = 0u; i < N; ++i) {
         for(auto j = 0u; j < N; ++j) {
            auto& aff = data_[calc_index(int(N), int(i), int(j))];
            Expects(aff.l_idx == int(std::min(i, j)));
            Expects(aff.r_idx == int(std::max(i, j)));
         }
      }
   }
}

P2DAffinity AffinityMemo::affinity(unsigned l_idx,
                                   unsigned r_idx) const noexcept
{
   Expects(data_.size() > 0);
   Expects(l_idx < n_idx0s_ && r_idx < n_idx1s_);

   auto get_idx = [&]() -> unsigned {
      if(is_symmetric_) {
         const int N        = int(n_idx0s_);
         const unsigned idx = calc_index(N, int(l_idx), int(r_idx));
         const auto sz      = (N * N + N) / 2;
         Expects(data_.size() == unsigned(sz));
         return idx;
      }
      return l_idx * n_idx1s_ + r_idx;
   };

   const auto idx = get_idx();
   Expects(idx < data_.size());

   auto& aff = data_[idx];
   if(aff.l_idx == -1) { // synchronization doesn't matter: this is a memo
      aff.l_idx = int(l_idx);
      aff.r_idx = int(r_idx);
      if(is_symmetric_ && aff.l_idx > aff.r_idx)
         std::swap(aff.l_idx, aff.r_idx);

      if(is_symmetric_ && l_idx == r_idx) {
         aff.lab_patch_score = 0.0f;
      } else {
         const auto [p2d0_ptr, labs0_ptr] = get_idx0_(l_idx);
         const auto [p2d1_ptr, labs1_ptr]
             = is_symmetric_ ? get_idx0_(r_idx) : get_idx1_(r_idx);

         aff.lab_patch_score = calc_lab_patch_score(*labs0_ptr, *labs1_ptr);
      }
   }

   return aff;
}

size_t AffinityMemo::memory_usage() const noexcept
{
   return sizeof(P2DAffinity) * data_.capacity() + sizeof(AffinityMemo);
}

} // namespace perceive
