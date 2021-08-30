
#include "sparse-hist-cell.hpp"

#include "perceive/io/fp-io.hpp"

#define This SparseHistCell

namespace perceive
{
string This::to_string() const noexcept
{
   return format("[({}, {}), count = {}]", xy.x, xy.y, count);
}

void write_sparse_hist_cell(FILE* fp, const SparseHistCell& cell)
{
   save_int(fp, cell.xy.x);
   save_int(fp, cell.xy.y);
   save_float(fp, cell.count);
}

void load_sparse_hist_cell(FILE* fp, SparseHistCell& cell)
{
   load_int(fp, cell.xy.x);
   load_int(fp, cell.xy.y);
   load_float(fp, cell.count);
}

} // namespace perceive
