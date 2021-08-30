
#include "n-way-camera.hpp"

#include "perceive/optimization/levenberg-marquardt.hpp"
#include "perceive/optimization/nelder-mead.hpp"

namespace perceive::calibration::n_way
{
// --------------------------------------------------------- for output purposes

static const auto g_info = "\x1b[37m\u261b\x1b[0m"s;

static void
print2(const string& glyph, const string& msg, const bool print_newline = true)
{
   cout << format("   {} {}\x1b[0m", glyph, msg);
   if(print_newline)
      cout << endl;
   else
      cout.flush();
}

// ------------------------------------------------------ calc-constraint-counts
//
static vector<std::pair<int, int>>
calc_constraint_counts(const int n_cameras,
                       const vector<std::pair<int, int>>& edges)
{
   vector<std::pair<int, int>> o((size_t(n_cameras)));
   for(auto i = 0; i < n_cameras; ++i) o[size_t(i)] = std::make_pair(i, 0);
   for(const auto& [a, b] : edges) {
      o[size_t(a)].second++; // record these cameras
      o[size_t(b)].second++; // are constrained by each other
   }
   std::sort(begin(o), end(o), [](const auto& A, const auto& B) {
      return A.second > B.second; // Most constrained first
   });
   return o;
}

// ----------------------------------------------------------------- run-cam-opt
// optimize a sub-set of cameras
static Result
run_cam_opt(const int n_cameras,
            const vector<EuclideanTransform>& in_ets,
            const vector<std::pair<int, int>>& constraints,
            const vector<std::pair<int, int>>& edges,
            const int start_cam_index,
            const int end_cam_index,
            cost_fun_type& cost_fun,
            std::function<bool(real score, int counter)>& end_condition,
            const bool use_nelder_mead,
            const bool feedback)
{
   const int n_cams_to_optimize   = end_cam_index - start_cam_index;
   const size_t n_params          = size_t(6 * n_cams_to_optimize);
   vector<EuclideanTransform> ets = in_ets;

   Expects(ranges::all_of(edges, [&](const auto a) -> bool {
      return unsigned(a.first) < unsigned(n_cameras)
             && unsigned(a.second) < unsigned(n_cameras);
   }));

   if(feedback) {
      const auto s = format(" optimizing {} cameras", n_cams_to_optimize);
      const int dash_count = 70 - int(s.size());
      Expects(dash_count >= 0);
      print2(g_info, format("{}{}", string(size_t(dash_count), '-'), s));
   }

   auto create_marks = [&]() {
      vector<bool> marks(size_t(n_cameras), false);
      for(auto ind = start_cam_index; ind < end_cam_index; ++ind) {
         const int cam_ind = constraints[size_t(ind)].first;
         marks.at(size_t(cam_ind))
             = true; // We're going to optimize this camera
      }
      return marks;
   };
   const auto marks = create_marks();

   if(false) {
      INFO(format("constraints = [{}]",
                  rng::implode(constraints, ", ", [&](const auto ij) {
                     return format("{{{}, {}}}", ij.first, ij.second);
                  })));
      INFO(format("marks      = [{}]",
                  rng::implode(marks, ", ", [](bool x) { return str(x); })));
   }

   auto pack = [&](real* X) {
      for(auto ind = start_cam_index; ind < end_cam_index; ++ind) {
         const int cam_ind = constraints[size_t(ind)].first;
         pack_et_6df(ets[size_t(cam_ind)], X);
         X += 6;
      }
   };

   auto unpack = [&](const real* X) {
      for(auto ind = start_cam_index; ind < end_cam_index; ++ind) {
         const int cam_ind    = constraints[size_t(ind)].first;
         ets[size_t(cam_ind)] = unpack_et_6df(X);
         X += 6;
      }
   };

   auto rel_performance                = dNAN;
   auto counter                        = 0;
   auto best_error                     = std::numeric_limits<real>::max();
   bool end_condition_met              = false;
   vector<EuclideanTransform> best_ets = in_ets;

   auto cost_fn = [&](const real* X) -> real {
      if(end_condition_met) return dNAN;
      unpack(X);

      real err  = 0.0;
      int count = 0;
      for(const auto& [a_ind, b_ind] : edges) {
         if(!marks[size_t(a_ind)] || !marks[size_t(b_ind)]) continue;
         err += cost_fun({a_ind, ets.at(size_t(a_ind))},
                         {b_ind, ets.at(size_t(b_ind))});
         ++count;
      }
      Expects(count > 0);
      err /= real(count);

      if(counter > 0 && end_condition(err, counter)) {
         end_condition_met = true;
         return std::numeric_limits<real>::max();
      }

      if(err < best_error) {
         const auto delta_real = err - best_error;
         const auto delta_gradiant
             = (counter == 0) ? 0.0 : delta_real / real(counter);

         best_error = err;
         best_ets   = ets;

         if(feedback) {
            print2(g_info, format("#{:5d} - {:10.8f}", counter, best_error));
         }
      }

      ++counter;

      return err;
   };

   vector<real> start(n_params, 0.0);
   vector<real> step(n_params, one_degree());
   vector<real> xmin(n_params, 0.0);
   real ynewlo   = dNAN;
   real ystartlo = dNAN;
   real reqmin   = 1e-7;
   real diffstep = 0.1;
   int kcount    = 100000; // max interations
   int icount = 0, numres = 0, ifault = 0;
   const char* method = use_nelder_mead ? "nelder-mead" : "levenberg-marquardt";

   pack(&start[0]);
   ystartlo = cost_fn(&start[0]);

   if(!use_nelder_mead) {
      levenberg_marquardt(cost_fn,
                          unsigned(n_params),
                          &start[0],
                          &xmin[0],
                          reqmin,
                          diffstep,
                          10,
                          kcount,
                          icount,
                          ifault);
   } else {
      nelder_mead(cost_fn,
                  unsigned(n_params),
                  &start[0],
                  &xmin[0],
                  ynewlo,
                  reqmin,
                  &step[0],
                  10,
                  10 * kcount,
                  icount,
                  numres,
                  ifault);
   }

   if(end_condition_met) ifault = 0;

   if(feedback) {
      const auto fault_msg = (end_condition_met) ? "end condition met"
                             : (use_nelder_mead)
                                 ? nelder_mead_fault_str(ifault)
                                 : levenberg_marquardt_fault_str(ifault);

      cout << format("Feedback for optimizing {}", n_cams_to_optimize) << endl;
      cout << format("   method:              {}", method) << endl;
      cout << format("   iterations:          {}", icount) << endl;
      cout << format("   evaluations:         {}", counter) << endl;
      cout << format("   fault-code:          {}", ifault) << endl;
      cout << format("   fault-string:        {}", fault_msg) << endl;
      cout << endl;
      cout << format("   initial-score:       {}", ystartlo) << endl;
      cout << format("   final-score:         {}", best_error) << endl;
      cout << endl;
   }

   // Marshal the result
   Result ret;
   ret.min_val = best_error;
   ret.ets     = best_ets;
   return ret;
}

// ---------------------------------------------------------- n-way-cam-optimize

Result
n_way_cam_optimize(const int n_cameras,
                   const vector<EuclideanTransform>& ets,
                   const vector<std::pair<int, int>>& edges,
                   cost_fun_type cost_fun,
                   std::function<bool(real score, int counter)> end_condition,
                   const bool use_nelder_mead,
                   const bool feedback)
{
   { // Sanity
      Expects(n_cameras >= 0);
      Expects(size_t(n_cameras) == ets.size());

      // Make sure ALL edges are valid camera indices
      Expects(std::all_of(cbegin(edges), cend(edges), [&](const auto& o) {
         return size_t(o.first) < size_t(n_cameras)
                && size_t(o.second) < size_t(n_cameras);
      }));
   }

   { // Degenerate case
      Result ret;
      if(n_cameras == 0) return ret;
      if(n_cameras == 1) {
         ret.min_val = 0.0;
         ret.ets     = ets;
         return ret;
      }
   }

   // Figure out which cameras are most constrainted
   const auto constraints = calc_constraint_counts(n_cameras, edges);

   // Start with the most constrained cameras
   Result ret;
   ret.ets = ets;

   Expects(constraints.size() >= 2);
   for(auto i = 1; i < n_cameras; ++i)
      ret = run_cam_opt(n_cameras,
                        ret.ets,
                        constraints,
                        edges,
                        0,
                        i + 1,
                        cost_fun,
                        end_condition,
                        use_nelder_mead,
                        feedback);

   return ret;
}

} // namespace perceive::calibration::n_way
