
#include "non-convex-optimization.hpp"

#include "perceive/optimization/levenberg-marquardt.hpp"
#include "perceive/optimization/nelder-mead.hpp"

#define This NonConvexOptimizer

namespace perceive
{
// -------------------------------------------------------------------- optimize
//
float This::optimize(std::function<float(const float*)> cost_function)
{
   Expects(n_params() >= 0 && n_params() < k_max_n_params);

   std::array<float, k_max_n_params> out;
   float best_score = std::numeric_limits<float>::max();
   auto cost_fn     = [&](const float* X) {
      const auto score = cost_function(X);
      Expects(std::isfinite(score));

      if(score < best_score) {
         std::copy(X, X + n_params(), begin(out));
         best_score = score;
      }

      return score;
   };

   const auto now = tick();

   ystartlo_ = cost_fn(&start_[0]);

   if(use_nelder_mead()) {
      nelder_mead(cost_fn,
                  unsigned(n_params()),
                  &start_[0],
                  &xmin_[0],
                  ynewlo_,
                  reqmin(),
                  &step_[0],
                  recursive_invocations(), // maximum number of restarts
                  kcount_,
                  icount_,
                  numres_,
                  ifault_);
   } else {
      levenberg_marquardt(cost_fn,                 // cost function
                          unsigned(n_params()),    // n parameters
                          &start_[0],              // start
                          &xmin_[0],               // xmin
                          reqmin(),                // required minimum variance
                          diffstep(),              // initial difference step
                          recursive_invocations(), // recursive invocations
                          kcount_,                 // max interations
                          icount_,                 // output count
                          ifault_);                // output fault
   }

   seconds_ = float(tock(now));
   ynewlo_  = best_score;
   std::copy(begin(out), std::next(begin(out), n_params()), begin(xmin_));
   return best_score;
}

// ------------------------------------------------------------------ ifault-str
//
const string_view This::ifault_str() const noexcept
{
   return (use_nelder_mead()) ? nelder_mead_fault_str(ifault())
                              : levenberg_marquardt_fault_str(ifault());
}

// ------------------------------------------------------------- feedback-string
//
string This::feedback_string(const bool time_in_milliseconds) const
    noexcept(false)
{
   return format(R"V0G0N({}
   method               {:s}
   n-params             {:d}
   counter              {}
   iterations           {}
   fault-code           {}
   fault-string         {}
   initial-score        {}
   final-score          {}
   time                 {}
   init-X              [{}]
   min-X               [{}]
)V0G0N",
                 feedback_title(),
                 method_str(),
                 n_params(),
                 counter_,
                 icount(),
                 ifault(),
                 ifault_str(),
                 ystartlo(),
                 ynewlo(),
                 (time_in_milliseconds
                      ? trim_copy(format("{:7.3f}ms", 1000.0f * seconds()))
                      : format("{}s", seconds())),
                 implode(&start_[0], &start_[size_t(n_params())], ", "),
                 implode(&xmin_[0], &xmin_[size_t(n_params())], ", "));
}

} // namespace perceive
