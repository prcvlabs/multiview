
#include <alglib/optimization.h>
#include <functional>
#include <limits>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "perceive/foundation.hpp"

#include "levenberg-marquardt.hpp"

using namespace alglib;

namespace perceive
{
static void opt_fun(const real_1d_array& x, real_1d_array& fi, void* ptr)
{
   using cost_fun_t = std::function<double(const double*)>;
   fi[0] = (static_cast<cost_fun_t*>(ptr))->operator()(x.getcontent());
}

// ---------------------------------------- Levenberg-Marquardt (double version)

void levenberg_marquardt(std::function<double(const double*)> fn,
                         unsigned n,
                         double start[],
                         double xmin[],
                         double reqmin,
                         double diffstep, // 0.0001
                         int kcount,
                         int& icount,
                         int& ifault)
{
   levenberg_marquardt(
       fn, n, start, xmin, reqmin, diffstep, 1, kcount, icount, ifault);
   // real_1d_array x;
   // minlmstate state;
   // minlmreport rep;

   // icount = ifault = 0;

   // // Copy in start point
   // x.setlength(n);
   // for(unsigned i = 0; i < n; ++i)
   //     x[i] = start[i];

   // minlmcreatev(n, 1, x, diffstep, state);           // Initialize state
   // minlmsetcond(state, reqmin, kcount);              // constraints
   // alglib::minlmoptimize(state, opt_fun, NULL, &fn); // Optimize
   // minlmresults(state, x, rep);                      // Get results

   // // Unpack
   // for(unsigned i = 0; i < n; ++i)
   //     xmin[i] = x[i];

   // icount = rep.iterationscount;
   // ifault = rep.terminationtype;
}

// ----------------------------------------- Levenberg-Marquardt (float version)

void levenberg_marquardt(std::function<float(const float*)> fn,
                         unsigned n,
                         float start[],
                         float xmin[],
                         float reqmin,
                         float diffstep, // 0.0001
                         int kcount,
                         int& icount,
                         int& ifault)
{
   std::vector<float> buffer(n);
   std::vector<double> start_d(n);
   std::vector<double> xmin_d(n);

   for(unsigned i = 0; i < n; ++i) start_d[i] = double(start[i]);
   for(unsigned i = 0; i < n; ++i) xmin_d[i] = double(xmin[i]);

   auto cost_fn = [&](const double* X) -> double {
      for(auto& val : buffer) val = float(*X++);
      return double(fn(&buffer[0]));
   };

   levenberg_marquardt(cost_fn,
                       n,
                       &start_d[0],
                       &xmin_d[0],
                       double(reqmin),
                       double(diffstep),
                       kcount,
                       icount,
                       ifault);

   for(unsigned i = 0; i < n; ++i) xmin[i] = float(xmin_d[i]);
}

// --------------------------------------------------------------- With restarts

void levenberg_marquardt(std::function<double(const double*)> fn,
                         unsigned n,
                         double start[],
                         double xmin[],
                         double reqmin,
                         double diffstep,
                         int numres,
                         int kcount,
                         int& icount,
                         int& ifault)
{
   real_1d_array x;
   minlmstate state;
   minlmreport rep;
   unsigned final_count = 0;

   icount = ifault = 0;

   double best_val = std::numeric_limits<double>::max();
   std::function<double(const double*)> fn2z = [&](const double* X) {
      auto val = fn(X);
      if(val < best_val) {
         best_val = val;
         memcpy(&xmin[0], X, sizeof(double) * n);
      }
      return val;
   };

   // Copy in start point
   x.setlength(n);
   for(unsigned i = 0; i < n; ++i) x[i] = start[i];

   try {
      minlmcreatev(n, 1, x, diffstep, state);             // Initialize state
      minlmsetcond(state, reqmin, kcount);                // constraints
      alglib::minlmoptimize(state, opt_fun, NULL, &fn2z); // Optimize
      minlmresults(state, x, rep);                        // Get results

      // Unpack
      icount += rep.iterationscount;
      for(unsigned i = 0; i < n; ++i) xmin[i] = x[i];

      for(int itr = 1; itr < numres && rep.terminationtype == 2; ++itr) {
         // cout << "------------------------------------------------------"
         //      << endl;
         for(unsigned i = 0; i < n; ++i) x[i] = xmin[i];
         diffstep *= 0.1;
         minlmcreatev(n, 1, x, diffstep, state);             // Initialize state
         minlmsetcond(state, reqmin, kcount);                // constraints
         alglib::minlmoptimize(state, opt_fun, NULL, &fn2z); // Optimize
         minlmresults(state, x, rep);
         icount += rep.iterationscount;
         for(unsigned i = 0; i < n; ++i) xmin[i] = x[i];
      }

      ifault = int(rep.terminationtype);
   } catch(alglib::ap_error& e) {
      ifault = -9;
      LOG_ERR(format("Exception in levenberg-marquardt: {}", e.msg));
   } catch(std::exception& e) {
      ifault = -9;
      LOG_ERR(format("Exception in levenberg-marquardt: {}", e.what()));
   } catch(...) {
      ifault = -9;
   }
}

void levenberg_marquardt(std::function<float(const float*)> fn,
                         unsigned n,
                         float start[],
                         float xmin[],
                         float reqmin,
                         float diffstep,
                         int numres, // diffstep *= 0.1 between each restart
                         int kcount,
                         int& icount,
                         int& ifault)
{
   std::vector<float> buffer(n);
   std::vector<double> start_d(n);
   std::vector<double> xmin_d(n);

   for(unsigned i = 0; i < n; ++i) start_d[i] = double(start[i]);
   for(unsigned i = 0; i < n; ++i) xmin_d[i] = double(xmin[i]);

   auto cost_fn = [&](const double* X) -> double {
      for(auto& val : buffer) val = float(*X++);
      return double(fn(&buffer[0]));
   };

   levenberg_marquardt(cost_fn,
                       n,
                       &start_d[0],
                       &xmin_d[0],
                       double(reqmin),
                       double(diffstep),
                       numres,
                       kcount,
                       icount,
                       ifault);

   for(unsigned i = 0; i < n; ++i) xmin[i] = float(xmin_d[i]);
}

// ----------------------------------------------- Levenberg-Marquardt fault-str

const char* levenberg_marquardt_fault_str(int ifault)
{
   switch(ifault) {
   case -9: return "exception";
   case -8: return "NAN/INF value detected either in function or Jacobian";
   case -7: return "derivative correctness check failed";
   case -5: return "inappropriate solver was used";
   case -3: return "constraints are inconsistent";
   case 2: return "relative step is no more than EpsX.";
   case 5: return "MaxIts steps was taken";
   case 7: return "stopping conditions forbid further improvement";
   case 8: return "terminated by  user via MinLMRequestTermination()";
   default: return "Unknown ifault condition";
   }
}

} // namespace perceive
