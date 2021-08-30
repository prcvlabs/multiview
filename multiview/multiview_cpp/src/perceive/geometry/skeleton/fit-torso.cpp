
#include "fit-torso.hpp"
#include "skeleton-3d.hpp"

#include "perceive/geometry/fitting-planes.hpp"
#include "perceive/optimization/levenberg-marquardt.hpp"
#include "perceive/optimization/nelder-mead.hpp"
#include "perceive/utils/eigen-helpers.hpp"

#ifdef DO_NOT_COMPILE

namespace perceive
{
using namespace skeleton;

template<typename InputIt> auto calc_eigen_vectors(InputIt begin, InputIt end)
{
   using T = typename std::iterator_traits<InputIt>::value_type;

   const auto N = std::distance(begin, end);
   if(N == 0) return vector<T>{};

   const auto& T0 = *begin;
   const int C    = T0.size();
   if(N < C) return vector<T>{};

   MatrixXr A(N, C);
   int pos = 0;
   for(auto ii = begin; ii != end; ++ii, ++pos)
      for(auto i = 0; i < C; ++i) A(pos, i) = ii->operator[](i);
   Expects(pos == N);

   MatrixXr At  = A.transpose();
   MatrixXr AtA = At * A;
   Expects(AtA.rows() == C);
   Expects(AtA.cols() == C);

   MatrixXr U, V;
   VectorXr D;
   svd_UDV(AtA, U, D, V); // Don't forget that this has a sign ambiguity

   {
      static std::mutex padlock_;
      lock_guard lock(padlock_);
      cout << format("AtA = \n") << AtA << endl << endl;
      cout << format("U   = \n") << U << endl << endl;
      cout << format("D   = \n") << D.transpose() << endl << endl;
      cout << format("V   = \n") << V << endl << endl;
   }

   vector<T> out(C);
   for(auto i = 0; i < C; ++i) {
      for(auto j = 0; j < C; ++j) out[i](j) = V(i, j); // or is it V(j, i)
      out[i].normalise();
   }

   return out;
}

// ------------------------------------------------------- get torso orientation
//
static Plane4f get_torso_orientation(
    const Vector3f& C,     // camera center
    const Skeleton2D& p2d, // 2d skeleton has rays
    const Plane4f& p3,
    // const std::array<Vector3f, skeleton::k_n_keypoints>& solXs,
    const bool feedback)
{
   // Let's check that the plane normal points in the direction the person
   // is looking
   if(false) {
      std::vector<Vector3f> Xs;
      Xs.reserve(p2d.rays().size());
      for(const auto& ray : p2d.rays())
         if(ray.is_finite())
            Xs.push_back(plane_ray_intersection(p3, C, C + ray));
      vector<Vector3f> evs = calc_eigen_vectors(begin(Xs), end(Xs));
      if(evs.size() == 0) return p3; // not enough solutions
      Expects(evs.size() == 3);
      const bool z_good
          = is_close<float>(std::fabs(dot(p3.xyz(), evs[2])), 1.0f);
      if(!z_good) {
         LOG_ERR(format("SVD did not return expected eigen-vectors"));
         for(auto i = 0; i < 3; ++i) {
            cout << format("    dot({}, {}) = {}",
                           str(p3.xyz()),
                           str(evs[i]),
                           dot(p3.xyz(), evs[i]))
                 << endl;
         }
         Expects(false);
      }

      Expects(std::fabs(dot(evs[2], p3.xyz())) - 1.0f < 1e-4f);
      const Vector3f Yaxis = evs[0]; // largest eigenvector should be up
   }

   std::array<skeleton::KeypointName, 6> l_kps
       = {skeleton::KeypointName::L_SHOULDER,
          skeleton::KeypointName::L_ELBOW,
          skeleton::KeypointName::L_WRIST,
          skeleton::KeypointName::L_HIP,
          skeleton::KeypointName::L_KNEE,
          skeleton::KeypointName::L_ANKLE};

   std::array<skeleton::KeypointName, 6> r_kps
       = {skeleton::KeypointName::R_SHOULDER,
          skeleton::KeypointName::R_ELBOW,
          skeleton::KeypointName::R_WRIST,
          skeleton::KeypointName::R_HIP,
          skeleton::KeypointName::R_KNEE,
          skeleton::KeypointName::R_ANKLE};

   const auto average_kps = [&](const auto& kp_list) {
      Vector3f X  = {0.0f, 0.0f, 0.0f};
      int counter = 0;
      for(const auto kp : kp_list) {
         const auto& ray = p2d.rays()[int(kp)];
         if(!ray.is_finite()) continue;
         X += plane_ray_intersection(p3, C, C + ray);
         ++counter;
      }
      return (counter == 0) ? Vector3f::nan() : X / float(counter);
   };

   const auto L     = average_kps(l_kps);
   const auto R     = average_kps(r_kps);
   const auto Xaxis = (L - R).normalised();
   const auto Yaxis = cross(p3.xyz(), Xaxis);

   if(Yaxis.z < 0.0f) return -p3; // Flip!
   return p3;

   // const auto Z0 = cross(Xaxis, Yaxis).normalised();
   // if(dot(Z0, p3.xyz()) < 0.0f) return -p3; // Flip!
   // return p3;
}

// ------------------------------------------------------------------- fit-torso
//
constexpr std::array<skeleton::KeypointName, 6> k_keypoints
    = {skeleton::KeypointName::NOTCH,
       skeleton::KeypointName::PELVIS,
       skeleton::KeypointName::L_SHOULDER,
       skeleton::KeypointName::R_SHOULDER,
       skeleton::KeypointName::L_HIP,
       skeleton::KeypointName::R_HIP};

struct StaticData
{
   std::array<Bone, 5> bone_labels;
   std::array<Point2, 5> bone_kps;

   StaticData()
   {
      bone_labels[0] = get_bone(Bone::L_COLLAR);
      bone_labels[1] = get_bone(Bone::R_COLLAR);
      bone_labels[2] = get_bone(Bone::SPINE);
      bone_labels[3] = get_bone(Bone::L_PELVIS);
      bone_labels[4] = get_bone(Bone::R_PELVIS);
      for(auto i = 0; i < 5; ++i) {
         auto get_idx = [&](skeleton::KeypointName kp) -> int {
            auto ii = std::find(cbegin(k_keypoints), cend(k_keypoints), kp);
            Expects(ii != cend(k_keypoints));
            return std::distance(cbegin(k_keypoints), ii);
         };
         const auto& l = bone_labels[i];
         bone_kps[i]   = Point2(get_idx(l.kp0), get_idx(l.kp1));
      }
   }
};

// ------------------------------------------------------------------- fit-torso
//
static Plane4f fit_torso_(const Vector3f& C,     // camera center
                          const Skeleton2D& p2d, // 2d skeleton has rays
                          const float height,    // scales reconstruction
                          std::array<Vector3f, skeleton::k_n_keypoints>& solXs,
                          const bool feedback) noexcept
{
   // The normal has two degrees of freedom... we should
   // already have at least one point on the plane.

   static StaticData static_data;
   const auto& keypoints   = k_keypoints;
   const auto& bone_labels = static_data.bone_labels;
   const auto& bone_kps    = static_data.bone_kps;
   const float unit_size   = height / Bone::adult_height;

   std::array<Vector3f, 6> rays;
   std::transform(cbegin(keypoints),
                  cend(keypoints),
                  begin(rays),
                  [&p2d](auto kp) { return p2d.rays()[int(kp)]; });

   std::array<Vector3f, 6> sols;
   std::transform(cbegin(keypoints),
                  cend(keypoints),
                  begin(sols),
                  [&solXs](auto kp) { return solXs[int(kp)]; });

   auto print_current_solution = [&]() {
      sync_write([&]() {
         cout << format("current solution") << endl;
         for(auto i = 0u; i < sols.size(); ++i) {
            cout << format("sol[{}] = {}, finite = {}",
                           i,
                           str(sols[i]),
                           str(sols[i].is_finite()))
                 << endl;
         }
      });
   };
   if(false) { print_current_solution(); }

   const int n_solved
       = std::accumulate(cbegin(sols), cend(sols), 0, [](int n, const auto& X) {
            return n + (X.is_finite() ? 1 : 0);
         });

   auto set_sol_from_p3 = [&](const Vector4f& p3) {
      for(auto i = 0u; i < rays.size(); ++i)
         sols[i] = plane_ray_intersection(p3, C, C + rays[i]);
   };

   auto finalize_result = [&]() {
      for(auto i = 0u; i < sols.size(); ++i) solXs[int(keypoints[i])] = sols[i];
   };

   // WARN(format("n-solved = {}", n_solved));

   if(n_solved == 0) return Plane4f::nan(); // nothing to do
   if(n_solved >= 3) {
      std::vector<Vector3> Xs;
      Xs.reserve(n_solved);
      for(const auto& X : sols)
         if(X.is_finite()) Xs.push_back(to_vec3(X));
      const auto p3 = to_vec4f(fit_plane(Xs));
      if(p3.is_finite()) set_sol_from_p3(p3);
      finalize_result();
      return p3;
   }

   { // 2D result space
      Expects(n_solved == 1 || n_solved == 2);

      std::array<int, 2> Xsols = {0, 0};
      int pos                  = 0;
      for(auto i = 0u; i < sols.size(); ++i) {
         const auto& X = sols[i];
         Expects(unsigned(pos) < Xsols.size());
         if(X.is_finite()) Xsols[pos++] = i;
      }
      Expects(pos > 0);

      const auto n_params = (n_solved == 1) ? 2 : 1;
      const auto& X0      = sols[Xsols[0]];
      const auto& X1      = sols[Xsols[1]];
      Vector4f p3, best_p3;

      Vector3f Xaxis = {1.0f, 0.0f, 0.0f};
      Vector3f Yaxis = {0.0f, 1.0f, 0.0f};
      const Vector3f XX
          = (n_params == 1) ? (X1 - X0).normalized() : Vector3f::nan();
      if(n_params == 1) {
         Quaternion q
             = Quaternion::between_vectors(Vector3{0, 0, 1}, to_vec3(XX));
         Xaxis            = to_vec3f(q.rotate(Vector3(1.0, 0.0, 0.0)));
         Yaxis            = to_vec3f(q.rotate(Vector3(0.0, 1.0, 0.0)));
         const auto& ray0 = Xsols[0];
         const auto Y     = rays[Xsols[0]].cross(XX).normalised();
         p3.xyz()         = XX.cross(Y);
      } else {
         p3.xyz() = rays[Xsols[0]];
      }

      p3.d() = -dot(p3.xyz(), X0);
      if(n_params == 1) Expects(std::fabs(p3.side(X1)) < 1e-5f);

      // Okay, invoke levenberg-marquardt on a 1D/2D problem
      auto pack = [&](float* X) {
         if(n_params == 1) {
            auto cos_theta
                = std::clamp<float>(dot(p3.xyz(), Xaxis), -1.0f, 1.0f);
            X[0] = acos(cos_theta);
         } else {
            Vector3f s = cartesian_to_spherical(p3.xyz());
            X[0]       = s.x; // inclination
            X[1]       = s.y; // azimuth
         }
      };

      auto unpack = [&](const float* X) {
         if(n_params == 1) {
            p3.xyz() = cos(*X) * Xaxis + sin(*X) * Yaxis;
            p3.d()   = -dot(p3.xyz(), X0);
            Expects(std::fabs(p3.side(X1)) < 1e-5f);
         } else {
            p3.xyz() = spherical_to_cartesian(X[0], X[1], 1.0f);
            p3.d()   = -dot(p3.xyz(), X0);
         }
         set_sol_from_p3(p3);
      };

      int counter  = 0;
      int best_err = std::numeric_limits<float>::max();
      auto fn      = [&](const float* X) {
         unpack(X);

         float err = 0.0f;
         int count = 0;

         // score the bones here
         for(auto i = 0u; i < bone_labels.size(); ++i) {
            const auto len  = bone_labels[i].proportion * unit_size;
            const auto& P   = sols[bone_kps[i].x];
            const auto& Q   = sols[bone_kps[i].y];
            const auto dist = (P - Q).norm();
            if(std::isfinite(dist)) {
               const auto diff = square(dist - len);
               err += diff;
               ++count;
            }
         }

         Expects(count > 0);
         err /= float(count);

         if(err < best_err) {
            best_err = err;
            best_p3  = p3;
            if(feedback) {
               // Print something nice here
               cout << format("  #{: 5d}, score = {}", counter, best_err)
                    << endl;
            }
         }

         ++counter;
         return err;
      };

      const bool use_nelder_mead = false;
      vector<float> start(n_params);
      vector<float> xmin(n_params);
      float ynewlo   = NAN;
      float ystartlo = NAN;
      float reqmin   = 1e-7f;
      float diffstep = one_degree();
      int kcount     = 1000; // max interations
      int icount = 0, numres = 0, ifault = 0;
      const char* method = nullptr;
      const auto now     = tick();

      pack(&start[0]);
      ystartlo = fn(&start[0]);

      if(!use_nelder_mead) {
         method = "levenberg-marquardt";
         levenberg_marquardt(fn,        // cost function
                             n_params,  // n parameters
                             &start[0], // start
                             &xmin[0],  // xmin
                             reqmin,    // required minimum variance
                             diffstep,  // initial difference step
                             5,         // recursive invocations
                             kcount,    // max interations
                             icount,    // output count
                             ifault);   // output fault
         ynewlo = fn(&xmin[0]);
      } else {
         method = "nelder-mead";

         vector<float> step(n_params);
         std::fill(begin(step), end(step), one_degree());
         nelder_mead(fn,
                     n_params,
                     &start[0],
                     &xmin[0],
                     ynewlo,
                     reqmin,
                     &step[0],
                     10,
                     20 * kcount,
                     icount,
                     numres,
                     ifault);
      }

      unpack(&xmin[0]);
      finalize_result();

      if(feedback) {
         sync_write([&]() {
            cout << format("Feedback for finding torso p3") << endl;
            cout << format("   method:               {}", method) << endl;
            cout << format("   n-params:             {}", n_params) << endl;
            cout << format("   counter:              {}", counter) << endl;
            cout << format("   iterations:           {}", icount) << endl;
            cout << format("   fault-code:           {}", ifault) << endl;
            auto msg = (use_nelder_mead)
                           ? nelder_mead_fault_str(ifault)
                           : levenberg_marquardt_fault_str(ifault);
            cout << format("   fault-string:         {}", msg) << endl;
            cout << endl;
            cout << format("   initial-score:        {}", ystartlo) << endl;
            cout << format("   final-score:          {}", ynewlo) << endl;
            cout << format("   time:                 {}ms", 1000.0 * tock(now))
                 << endl;
            cout << endl;
            cout << endl;
         });
      }

      return best_p3;
   }

   return Plane4f::nan();
}

Plane4f fit_torso(const Vector3f& C,     // camera center
                  const Skeleton2D& p2d, // 2d skeleton has rays
                  const float height,    // scales reconstruction
                  std::array<Vector3f, skeleton::k_n_keypoints>& solXs,
                  const bool feedback) noexcept
{
   const auto p3 = fit_torso_(C, p2d, height, solXs, feedback);
   return get_torso_orientation(C, p2d, p3, feedback);
}

// ------------------------------------------------------------------- fit-torso
//
Plane4f fit_torso(const Vector3f& C,
                  const Skeleton2D& p2d,
                  const bool feedback) noexcept
{
   Expects(C.is_finite());
   const auto Cy_ret = p2d.best_cylinder();
   const auto height = Cy_ret.Cy.height;
   Expects(Cy_ret.Cy.X.is_finite());

   // We need to set some points on that skeleton, based on height
   const auto hip_height      = height * human::k_height_ratio_hip;
   const auto shoulder_height = height * human::k_height_ratio_shoulder;

   std::array<Vector3f, skeleton::k_n_keypoints> solXs;
   std::fill(begin(solXs), end(solXs), Vector3f::nan());

   for(const auto kp : k_keypoints) {
      const auto& ray = p2d.rays()[int(kp)];
      if(ray.is_finite()) {
         const Vector3f X = to_vec3f(Cy_ret.Cy.X);
         Plane4f p3       = Plane4f{ray, -dot(X, ray)}; // We need some plane
         solXs[int(kp)]   = plane_ray_intersection(p3, C, C + ray);
         Expects(solXs[int(kp)].is_finite());
         break;
      }
   }

   // Get that plane
   return fit_torso(C, p2d, height, solXs, feedback);
}

} // namespace perceive

#endif
