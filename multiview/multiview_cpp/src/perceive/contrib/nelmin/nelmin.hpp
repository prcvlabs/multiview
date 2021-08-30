
#include <functional>

namespace nelmin
{
void nelmin(std::function<double(const double*)> fn,
            int n,
            double start[],
            double xmin[],
            double* ynewlo,
            double reqmin,
            double step[],
            int konvge,
            int kcount,
            int max_restarts,
            int* icount,
            int* numres,
            int* ifault);

void set_nelder_mead_feedback(bool value);
} // namespace nelmin
