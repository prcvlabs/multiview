
#include <functional>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "compass-search.hpp"

static inline void r8vec_copy(int n, const double a1[], double a2[])
{
   memcpy(a2, a1, size_t(n) * sizeof(double));
}

/******************************************************************************/

static void compass_search_c(std::function<double(const double*)> fn,
                             int m,
                             double start[],
                             double xmin[],
                             double* fx,
                             double delta_tol,
                             double delta_init,
                             int k_max,
                             int* k)

/******************************************************************************/
/*
  Purpose:

  COMPASS_SEARCH carries out a direct search minimization algorithm.

  Licensing:

     This code is distributed under the GNU LGPL license.

  Modified:

     14 January 2012 (John Burkardt)
     14 September 2017 (Aaron Michaux)

  Author:

     John Burkardt (Original author)

  Reference:

     Tamara Kolda, Robert Michael Lewis, Virginia Torczon,
     Optimization by Direct Search: New Perspectives on Some Classical
     and Modern Methods,
     SIAM Review,
     Volume 45, Number 3, 2003, pages 385-482.

  Parameters:

  Input, double FN ( int m, double x[] ), the name of
  a function which evaluates the function to be minimized.

  Input,  int M,             the number of variables.
  Input,  double X0[M],      a starting estimate for the minimizer.
  Output, double xmin[M],    the estimated minimizer.
  Input,  double DELTA_TOL,  the smallest step size that is allowed.
  Input,  double DELTA_INIT, the starting stepsize.
  Input,  int K_MAX,         the maximum number of steps allowed.
  Output, double *FX,        the function value at X.
  Output, int *K,            the number of steps taken.
*/
{
   int decrease;
   double delta;
   double fxd;
   int i;
   int ii;
   double s;
   double* x0 = &start[0];
   double* x  = &xmin[0];

   std::vector<double> xd;
   xd.resize(size_t(m));

   *k = 0;
   r8vec_copy(m, x0, x);
   *fx = fn(x);

   if(delta_tol <= 0) {
      fprintf(stderr, "\n");
      fprintf(stderr, "COMPASS_SEARCH - Fatal error!\n");
      fprintf(stderr, "  DELTA_TOL <= 0.0.\n");
      fprintf(stderr, "  DELTA_TOL = %g\n", delta_tol);
      exit(1);
   }

   if(delta_init <= delta_tol) {
      fprintf(stderr, "\n");
      fprintf(stderr, "COMPASS_SEARCH - Fatal error!\n");
      fprintf(stderr, "  DELTA_INIT < DELTA_TOL.\n");
      fprintf(stderr, "  DELTA_INIT = %g\n", delta_init);
      fprintf(stderr, "  DELTA_TOL = %g\n", delta_tol);
      exit(1);
   }

   delta = delta_init;

   while(*k < k_max) {
      *k = *k + 1;
      /*
        For each coordinate direction I, seek a lower function value
        by increasing or decreasing X(I) by DELTA.
      */
      decrease = 0;
      s        = +1.0;
      i        = 0;

      for(ii = 1; ii <= 2 * m; ii++) {
         r8vec_copy(m, x, &xd[0]);
         xd[size_t(i)] = xd[size_t(i)] + s * delta;
         fxd           = fn(&xd[0]);
         /*
           As soon as a decrease is noticed, accept the new point.
         */
         if(fxd < *fx) {
            r8vec_copy(m, &xd[0], x);
            *fx      = fxd;
            decrease = 1;
            break;
         }

         s = -s;
         if(s == +1.0) { i = i + 1; }
      }
      /*
        If no decrease occurred, reduce DELTA.
      */
      if(!decrease) {
         delta = delta / 2.0;
         if(delta < delta_tol) { break; }
      }
   }
}

namespace perceive
{
void compass_search(std::function<double(const double*)> fn,
                    int m,
                    double start[],
                    double xmin[],
                    double& fx,
                    double delta_tol,
                    double delta_init,
                    int k_max,
                    int& k)
{
   return compass_search_c(
       fn, m, start, xmin, &fx, delta_tol, delta_init, k_max, &k);
}

} // namespace perceive
