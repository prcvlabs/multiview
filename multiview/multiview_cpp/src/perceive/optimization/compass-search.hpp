
#pragma once

#include <functional>

namespace perceive
{
/**
 * fn,             function to be minimized
 * n,              number of variables
 * start[n],       start-point.
 * xmin[n],        (output) the output estimate that minimizes 'fn'
 * ynewlo,         (output) same as fn(&xmin[0])
 * smallest_step,  the smallest step size that is allowed
 * initial_step,   the starting stepsize
 * kcount,         maximum number of times to evaluate 'fn'
 * icount,         (output) number of function iterations used
 */
void compass_search(std::function<double(const double*)> fn, int n,
                    double start[], double xmin[], double& ynewlo,
                    double smallest_step, double initial_step, int kcount,
                    int& icount);

} // namespace perceive
