
#pragma once

#include <functional>
#include <string.h>

namespace perceive
{
/**
 * fn,       function to be minimized
 * n,        number of variables
 * start[n], start-point.
 * xmin[n],  (output) the output estimate that minimizes 'fn'
 * ynewlo,   (output) same as fn(&xmin[0])
 * reqmin,   terminating limit for variance of 'fn' values
 * step[n],  size and shape of initial simple
 * konvge,   convergence check carried out every KONVGE interations
 * kcount,   maximum number of times to evaluate 'fn'
 * max_rest  maximum number of restarts. -1 for infinite
 * icount,   (output) number of function iterations used
 * numres,   (output) number of restarts
 * ifault,   (output) error indicator:
 *               0, no errors detected
 *               1, REQMIN, N, or KONVGE has an illegal value
 *               2, iteration terminated because KCOUNT was exceeded
 *               3, iteration terminated because REQMIN hit
 *               4, iteration terminated because MAX-RESTARTS
 */
void nelder_mead(std::function<double(const double*)> fn,
                 unsigned n,
                 double start[],
                 double xmin[],
                 double& ynewlo,
                 double reqmin,
                 double step[],
                 int konvge,
                 int kcount,
                 int& icount,
                 int& numres,
                 int& ifault);

void nelder_mead(std::function<float(const float*)> fn,
                 unsigned n,
                 float start[],
                 float xmin[],
                 float& ynewlo,
                 float reqmin,
                 float step[],
                 int konvge,
                 int kcount,
                 int& icount,
                 int& numres,
                 int& ifault);

void nelder_mead(std::function<double(const double*)> fn,
                 unsigned n,
                 double start[],
                 double xmin[],
                 double& ynewlo,
                 double reqmin,
                 double step[],
                 int konvge,
                 int kcount,
                 int max_restarts,
                 int& icount,
                 int& numres,
                 int& ifault);

void nelder_mead(std::function<float(const float*)> fn,
                 unsigned n,
                 float start[],
                 float xmin[],
                 float& ynewlo,
                 float reqmin,
                 float step[],
                 int konvge,
                 int kcount,
                 int max_restarts,
                 int& icount,
                 int& numres,
                 int& ifault);

// Returns textual description of fault (i.e., exit) code
const char* nelder_mead_fault_str(int ifault);

} // namespace perceive
