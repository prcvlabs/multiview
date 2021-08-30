
#pragma once

#include <iostream>
#include <utility>

#include "bicubic-spline.hpp"
#include "perceive/foundation.hpp"
#include "perceive/graphics/image-container.hpp"
#include "perceive/utils/string-utils.hpp"
#include "perceive/utils/threads.hpp"
#include "perceive/utils/tick-tock.hpp"

namespace perceive
{
// f00 --- f01
//  |       |
//  |       |
// f10 --- f11
inline Vector2 bilinear_interpolate(double dx,
                                    double dy,
                                    const Vector2& f00,
                                    const Vector2& f10,
                                    const Vector2& f01,
                                    const Vector2& f11)
{
   auto u0 = f00 + dx * (f01 - f00);
   auto u1 = f10 + dx * (f11 - f10);
   return u0 + dy * (u1 - u0);
}

inline real
linear_interpolate_1d(real dx, real dy, real f00, real f10, real f01, real f11)
{
   return dy * (f00 + f10) * 0.25 + (1.0 - dy) * (f01 + f11) * 0.25
          + dx * (f00 + f01) * 0.25 + (1.0 - dx) * (f10 + f11) * 0.25;
}

template<typename T> class EmpiricalFieldT
{
 public:
   using Vector2t = Vector2T<T>;
   using FieldT   = ImageContainerT<Vector2t>;

   CUSTOM_NEW_DELETE(EmpiricalFieldT)

   EmpiricalFieldT()                       = default;
   EmpiricalFieldT(const EmpiricalFieldT&) = default;
   EmpiricalFieldT(EmpiricalFieldT&&)      = default;
   ~EmpiricalFieldT()                      = default;

   EmpiricalFieldT& operator=(const EmpiricalFieldT&) = default;
   EmpiricalFieldT& operator=(EmpiricalFieldT&&) = default;

   FieldT field; // the actual field

   // PPT and FXY transform point 'X' into the grid-space of the stored field
   AABB domain;
   Vector2t ppt; // To translate points
   Vector2t fxy; // To scale in the x and y directions

   size_t n_bytes() const noexcept { return 4 * sizeof(T) + field.n_bytes(); }

   void init(std::function<Vector2T<T>(const Vector2T<T>&)> f,
             const AABB domain_,
             const unsigned field_width,
             const bool feedback = false,
             const bool parallel = false)
   {
      domain = domain_;
      Expects(domain.left <= domain.right);
      Expects(domain.top <= domain.bottom);
      const unsigned w = field_width;
      const unsigned h = unsigned(std::ceil(w / domain.aspect_ratio()));
      ppt              = Vector2t(T(domain.left), T(domain.top));
      fxy              = Vector2t(T(domain.width()) / T(w - 1),
                     T(domain.height()) / T(h - 1));
      field.resize(w, h, w);

      auto process_y = [&](unsigned y) {
         auto now = tick();
         for(auto x = 0u; x < w; ++x) {
            auto U      = inv_transform(Vector2t(T(x), T(y)));
            auto D      = f(U);
            field(x, y) = Vector2t(D.x, D.y);

            if(false && y == 0 && x == 0) {
               LOG_ERR(
                   format("[{}x{}] :: U = {} -> D = {}", x, y, str(U), str(D)));
            }
         }
         if(feedback and (y + 1) % 20 == 0)
            sync_write([&]() {
               cout << format(
                   "init empirical-field: {}/{}, {}s", y, h, tock(now))
                    << endl;
            });
      };

      if(parallel) {
         ParallelJobSet jobs;
         for(auto y = 0u; y < h; ++y)
            jobs.schedule([process_y, y]() { process_y(y); });
         jobs.execute();
      } else {
         for(auto y = 0u; y < h; ++y) process_y(y);
      }
   }

   string to_string() const noexcept
   {
      return format(R"V0G0N(
EmpiricalField:
   Domain:         {}
   ppt:            {}
   fxy:            {}
   w*h*sizeof(T) = {} x {} x {} = {}k
)V0G0N",
                    str(domain),
                    str(ppt),
                    str(fxy),
                    field.width,
                    field.height,
                    sizeof(T),
                    field.n_bytes() / 1024.0);
   }

   // domain goes to 'field lookup' space
   Vector2t transform(const Vector2t& X) const noexcept
   {
      return Vector2t((X.x - ppt.x) / fxy.x, (X.y - ppt.y) / fxy.y);
   }

   // 'field lookup' space to domain
   Vector2t inv_transform(const Vector2t& X) const noexcept
   {
      return Vector2t(X.x * fxy.x + ppt.x, X.y * fxy.y + ppt.y);
   }

   // domain to range
   Vector2t evaluate(const Vector2t& Y,
                     const bool feedback = false) const noexcept
   {
      assert(Y.is_finite());
      auto X        = transform(Y);
      const auto& f = field;
      T dx          = X.x - std::floor(X.x);
      T dy          = X.y - std::floor(X.y);

      if(!(dx >= T(0.0) and dx <= T(1.0)) or !(dy >= T(0.0) and dy <= T(1.0))) {
         FATAL(format("Y = {}, dxdy = [{}, {}]", str(Y), dx, dy));
         Expects(false);
      }

      const unsigned col = unsigned(std::round(X.x - dx));
      const unsigned row = unsigned(std::round(X.y - dy));

      const bool out_of_bounds = (row >= f.height - 1 || col >= f.width - 1);

      if(feedback) {
         INFO(format("field::evaluate"));
         cout << format("  ppt  = {}", str(ppt)) << endl;
         cout << format("  fxy  = {}", str(fxy)) << endl;
         cout << format("  ----------") << endl;
         cout << format("    Y  =  {}", str(Y)) << endl;
         cout << format("    X  =  {}", str(X)) << endl;
         cout << format("  dxy  = ({}, {})", dx, dy) << endl;
         cout << format("  row  =  {}/{}", row, f.height) << endl;
         cout << format("  col  =  {}/{}", col, f.width) << endl;
         cout << format("  oob  =  {}", str(out_of_bounds)) << endl;

         if(!out_of_bounds) {
            const auto& e0 = f(col + 0, row + 0);
            const auto& e1 = f(col + 0, row + 1);
            const auto& e2 = f(col + 1, row + 0);
            const auto& e3 = f(col + 1, row + 1);

            const auto u0 = e0 + dx * (e2 - e0);
            const auto u1 = e1 + dx * (e3 - e1);

            const auto ret = u0 + dy * (u1 - u0);

            cout << format("   e0  =  {}", str(e0)) << endl;
            cout << format("   e1  =  {}", str(e1)) << endl;
            cout << format("   e2  =  {}", str(e2)) << endl;
            cout << format("   e3  =  {}", str(e3)) << endl;
            cout << format("   ---------") << endl;
            cout << format("   u0  =  {}", str(u0)) << endl;
            cout << format("   u1  =  {}", str(u1)) << endl;
            cout << format("   ---------") << endl;
            cout << format("  out  =  {}", str(ret)) << endl;
            cout << endl;
         }
      }

      if(out_of_bounds) { return Vector2t::nan(); }

      const auto& e0 = f(col + 0, row + 0);
      const auto& e1 = f(col + 0, row + 1);
      const auto& e2 = f(col + 1, row + 0);
      const auto& e3 = f(col + 1, row + 1);

      const auto u0 = e0 + dx * (e2 - e0);
      const auto u1 = e1 + dx * (e3 - e1);
      return u0 + dy * (u1 - u0);
   }

   Vector2t evaluate(T x, T y) const noexcept
   {
      return evaluate(Vector2t(x, y));
   }

   friend std::string str(const EmpiricalFieldT<T>& field) noexcept
   {
      return field.to_string();
   }
};

using EmpiricalField  = EmpiricalFieldT<real>;
using EmpiricalFieldF = EmpiricalFieldT<float>;

/*
    RBase    -  RBase parameter, RBase>0
    NLayers  -  NLayers parameter, NLayers>0, recommended value  to  start
                with - about 5.
    LambdaNS -  >=0, nonlinearity penalty coefficient, negative values are
                not allowed. This parameter adds controllable smoothing to
                the problem, which may reduce noise. Specification of non-
                zero lambda means that in addition to fitting error solver
                will  also  minimize   LambdaNS*|S''(x)|^2  (appropriately
                generalized to multiple dimensions.

                Specification of exactly zero value means that no  penalty
                is added  (we  do  not  even  evaluate  matrix  of  second
                derivatives which is necessary for smoothing).

                Calculation of nonlinearity penalty is costly - it results
                in  several-fold  increase  of  model  construction  time.
                Evaluation time remains the same.

                Optimal  lambda  is  problem-dependent and requires  trial
                and  error.  Good  value to  start  from  is  1e-5...1e-6,
                which corresponds to slightly noticeable smoothing  of the
                function.  Value  1e-2  usually  means  that  quite  heavy
                smoothing is applied.

TUNING ALGORITHM

In order to use this algorithm you have to choose three parameters:
* initial radius RBase
* number of layers in the model NLayers
* penalty coefficient LambdaNS

Initial radius is easy to choose - you can pick any number  several  times
larger  than  the  average  distance between points. Algorithm won't break
down if you choose radius which is too large (model construction time will
increase, but model will be built correctly).

Choose such number of layers that RLast=RBase/2^(NLayers-1)  (radius  used
by  the  last  layer)  will  be  smaller than the typical distance between
points.  In  case  model  error  is  too large, you can increase number of
layers.  Having  more  layers  will make model construction and evaluation
proportionally slower, but it will allow you to have model which precisely
fits your data. From the other side, if you want to  suppress  noise,  you
can DECREASE number of layers to make your model less flexible (or specify
non-zero LambdaNS).

TYPICAL ERRORS

1. Using too small number of layers - RBF models with large radius are not
   flexible enough to reproduce small variations in the  target  function.
   You  need  many  layers  with  different radii, from large to small, in
   order to have good model.

2. Using  initial  radius  which  is  too  small.  You will get model with
   "holes" in the areas which are too far away from interpolation centers.
   However, algorithm will work correctly (and quickly) in this case.
*/

/* TO USE bicubic
    BicubicSpline mapx, mapy;
    mapx.init(aabb, spline_cols, spline_rows);
    mapy.init(aabb, spline_cols, spline_rows);

    for(unsigned j = 0; j < spline_rows; ++j) {
        for(unsigned i = 0; i < spline_cols; ++i) {
            Vector2 p = mapx.position(i, j);
            mapx.X(i, j) = calc_f_dx_dy_dxy(p, 0);
            mapy.X(i, j) = calc_f_dx_dy_dxy(p, 1);
        }
    }

    mapx.finalize();
    mapy.finalize();

    field.init_bicubic(mapx, mapy);
*/

class RBFField
{
 private:
   class Pimpl;
   unique_ptr<Pimpl> pimpl_;

 public:
   // Construction
   RBFField();
   RBFField(const RBFField&);
   RBFField(RBFField&&);
   ~RBFField();
   RBFField& operator=(const RBFField&);
   RBFField& operator=(RBFField&&);

   // Returns the number of seconds...
   void init(const vector<Vector2>& src, // Field maps src to dst
             const vector<Vector2>& dst,
             double rbase,
             unsigned layers,
             double smoothing);

   void init_qnn(const vector<Vector2>& src, // Field maps src to dst
                 const vector<Vector2>& dst,
                 double q = 1.0,
                 double z = 5.0);

   void init_bicubic(const BicubicSpline& x_map, const BicubicSpline& y_map);

   void init_bicubic(const AABB region,
                     const real grid_sz,
                     std::function<Vector2(const Vector2& x)> func,
                     const real dxy_step = dNAN);

   const BicubicSpline& x_map() const;
   const BicubicSpline& y_map() const;

   // This we the AABB used to create points in 'init'
   const AABB& src_aabb() const;

   // Fills the passed field with values
   void fill_field(EmpiricalField& f, unsigned w, unsigned h) const;
   Vector2 evaluate(const Vector2& x) const;

   void serialize(std::ostream& s_out) const;
   void unserialize(std::istream& s_in);

   std::string to_string() const;

   // For perturbing the field...
   // unsigned
};

} // namespace perceive
