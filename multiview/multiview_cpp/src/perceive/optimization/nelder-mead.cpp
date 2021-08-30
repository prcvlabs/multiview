
#include "nelder-mead.hpp"

#include <functional>
#include <limits>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <vector>

#include "nelmin/nelmin.hpp"

/******************************************************************************/

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
 * icount,   (output) number of function iterations used
 * numres,   (output) number of restarts
 * ifault,   (output) error indicator:
 *               0, no errors detected
 *               1, REQMIN, N, or KONVGE has an illegal value
 *               2, iteration terminated because KCOUNT was exceeded
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
                 int& ifault)
{
   nelder_mead(fn,
               n,
               start,
               xmin,
               ynewlo,
               reqmin,
               step,
               konvge,
               kcount,
               -1,
               icount,
               numres,
               ifault);
}

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
                 int& ifault)
{
   nelder_mead(fn,
               n,
               start,
               xmin,
               ynewlo,
               reqmin,
               step,
               konvge,
               kcount,
               -1,
               icount,
               numres,
               ifault);
}

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
                 int& ifault)
{
   std::vector<double> start_d(n);
   memcpy(&start_d[0], start, n * sizeof(double));

   nelmin::nelmin(fn,
                  int(n),
                  &start_d[0],
                  xmin,
                  &ynewlo,
                  reqmin,
                  step,
                  konvge,
                  kcount,
                  max_restarts,
                  &icount,
                  &numres,
                  &ifault);
}

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
                 int& ifault)
{
   std::vector<float> buffer(n);
   std::vector<double> start_d(n);
   std::vector<double> xmin_d(n);
   std::vector<double> step_d(n);

   for(unsigned i = 0; i < n; ++i) start_d[i] = double(start[i]);
   for(unsigned i = 0; i < n; ++i) xmin_d[i] = double(xmin[i]);
   for(unsigned i = 0; i < n; ++i) step_d[i] = double(step[i]);

   auto cost_fn = [&](const double* X) -> double {
      for(auto& val : buffer) val = float(*X++);
      return double(fn(&buffer[0]));
   };

   double ynewlo_d{0.0};

   nelmin::nelmin(cost_fn,
                  int(n),
                  &start_d[0],
                  &xmin_d[0],
                  &ynewlo_d,
                  double(reqmin),
                  &step_d[0],
                  konvge,
                  kcount,
                  max_restarts,
                  &icount,
                  &numres,
                  &ifault);

   for(unsigned i = 0; i < n; ++i) xmin[i] = float(xmin_d[i]);
   ynewlo = float(ynewlo_d);
}

const char* nelder_mead_fault_str(int ifault)
{
   switch(ifault) {
   case 0: return "success";
   case 1: return "REQMIN, N, or KONVGE had an illegal value";
   case 2: return "Iteration terminated because KCOUNT was exceeded";
   case 3: return "Iteration terminated because REQMIN reached";
   case 4: return "Iteration terminated because MAX-RESTARTS reached";
   default: return "Unknown ifault condition";
   }
}

} // namespace perceive
