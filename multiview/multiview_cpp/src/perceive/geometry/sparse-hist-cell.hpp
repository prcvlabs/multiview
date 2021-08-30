
#pragma once

#include "perceive/geometry/vector.hpp"

namespace perceive
{
struct SparseHistCell
{
   Point2 xy   = {-1, -1};
   float count = 0.0f;

   SparseHistCell(Point2 xy_ = {-1, -1}, float count_ = 0.0f) noexcept
       : xy(xy_)
       , count(count_)
   {}
   SparseHistCell(const SparseHistCell&) = default;
   SparseHistCell(SparseHistCell&&)      = default;
   ~SparseHistCell()                     = default;
   SparseHistCell& operator=(const SparseHistCell&) = default;
   SparseHistCell& operator=(SparseHistCell&&) = default;

   bool operator==(const SparseHistCell& o) const noexcept
   {
      return xy == o.xy && count == o.count;
   }

   bool operator!=(const SparseHistCell& o) const noexcept
   {
      return !(*this == o);
   }

   string to_string() const noexcept;
   friend string str(const SparseHistCell& o) noexcept { return o.to_string(); }
};

inline float
sparse_histcell_sum(const vector<SparseHistCell>& cell_vec) noexcept
{
   float ret = 0.0f;
   for(const auto& cell : cell_vec) ret += cell.count;
   return ret;
}

void write_sparse_hist_cell(FILE* fp, const SparseHistCell& cell);
void load_sparse_hist_cell(FILE* fp, SparseHistCell& cell);

} // namespace perceive
