
#pragma once

namespace perceive
{
class NonConvexOptimizer
{
 private:
   static constexpr int k_max_n_params = 20;

   bool use_nelder_mead_                    = true;
   std::array<float, k_max_n_params> start_ = {};
   std::array<float, k_max_n_params> step_  = {};
   std::array<float, k_max_n_params> xmin_  = {};
   int n_params_                            = 0;
   float ynewlo_                            = fNAN;
   float ystartlo_                          = fNAN;
   float reqmin_                            = 1e-7f;
   float diffstep_                          = 1.0f;
   int recursive_invocations_               = 1;
   int kcount_                              = 1000; // max interations
   int icount_                              = 0;
   int numres_                              = 0;
   int ifault_                              = 0;
   float seconds_                           = 0.0f;
   string feedback_title_                   = "Non-convex Optimizer Feedback:";
   int counter_                             = 0;

 public:
   // Optimize!
   float optimize(std::function<float(const float*)> cost_function);

   // Getters
   int n_params() const noexcept { return n_params_; }
   bool use_nelder_mead() const noexcept { return use_nelder_mead_; }
   const float* start() const noexcept { return &start_[0]; }
   const float* step() const noexcept { return &step_[0]; }
   const float* xmin() const noexcept { return &xmin_[0]; }
   float ynewlo() const noexcept { return ynewlo_; }
   float ystartlo() const noexcept { return ystartlo_; }
   float reqmin() const noexcept { return reqmin_; }
   float diffstep() const noexcept { return diffstep_; }
   int recursive_invocations() const noexcept { return recursive_invocations_; }
   int kcount() const noexcept { return kcount_; }
   int icount() const noexcept { return icount_; }
   int numres() const noexcept { return numres_; }
   int ifault() const noexcept { return ifault_; }
   float seconds() const noexcept { return seconds_; }
   const string_view feedback_title() const noexcept { return feedback_title_; }
   const string_view method_str() const noexcept
   {
      return use_nelder_mead() ? "nelder-mead" : "levenberg-marquardt";
   }
   const string_view ifault_str() const noexcept;
   string feedback_string(const bool time_in_milliseconds) const
       noexcept(false);

   // Setters
   float* start() noexcept { return &start_[0]; }
   float* step() noexcept { return &step_[0]; }
   void set_use_nelder_mead(bool val) noexcept { use_nelder_mead_ = val; }
   void set_n_params(int val) noexcept
   {
      Expects(val >= 0 && val < k_max_n_params);
      n_params_ = val;
   }
   template<typename InputIt> void set_start(InputIt ii)
   {
      std::copy(ii, std::next(ii, n_params()), begin(start_));
   }
   template<typename InputIt> void set_step(InputIt ii)
   {
      std::copy(ii, std::next(ii, n_params()), begin(step_));
   }
   void set_reqmin(float val) noexcept { reqmin_ = val; }
   void set_diffstep(float val) noexcept { diffstep_ = val; }
   void set_recursive_invocations(int val) noexcept
   {
      recursive_invocations_ = val;
   }
   void set_kcount(int val) noexcept { kcount_ = val; }
   void set_feedback_title(const string_view val) { feedback_title_ = val; }

   template<typename InputIt1, typename InputIt2>
   void init_nelder_mead(const string_view feedback_title,
                         int n_params,
                         int kcount,
                         float reqmin,
                         InputIt1 start_ii,
                         InputIt2 step_ii) noexcept
   {
      set_feedback_title(feedback_title);
      set_n_params(n_params);
      set_kcount(kcount);
      set_reqmin(reqmin);
      set_start(start_ii);
      set_step(step_ii);
   }

   template<typename InputIt>
   void init_levenberg(const string_view feedback_title,
                       int n_params,
                       int kcount,
                       float reqmin,
                       float diffstep,
                       int n_recursive_invocations,
                       InputIt start_ii) noexcept
   {
      set_feedback_title(feedback_title);
      set_n_params(n_params);
      set_kcount(kcount);
      set_reqmin(reqmin);
      set_diffstep(diffstep);
      set_recursive_invocations(n_recursive_invocations);
      set_start(start_ii);
   }
};

} // namespace perceive
