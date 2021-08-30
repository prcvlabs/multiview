
#include "bicubic-spline.hpp"
#include "perceive/utils/md5.hpp"
#include "rbf-field.hpp"

#define This BicubicSpline

namespace perceive
{
void This::init(const AABB& bounds, unsigned n_cols, unsigned n_rows)
{
   if(n_cols < 2 || n_rows < 2)
      throw std::runtime_error("Must have n_cols and n_rows >= 2");

   bounds_ = bounds;
   grid_.resize(n_cols, n_rows);
   A_.resize(n_cols - 1, n_rows - 1);
   dx_     = bounds.width() / double(n_cols - 1);
   dy_     = bounds.height() / double(n_rows - 1);
   dx_inv_ = 1.0 / dx_;
   dy_inv_ = 1.0 / dy_;

   grid_.zero();
   finalize();
}

// 0, 1, 2, 3
// ., x, y,xy
static Matrix4r evaluate_A(const Vector4r& f00,
                           const Vector4r& f10,
                           const Vector4r& f01,
                           const Vector4r& f11)
{
   Matrix4r X, F, Y;

   X << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, -3., 3.0, -2., -1., 2.0, -2.,
       1.0, 1.0;

   F << f00(0), f01(0), f00(2), f01(2), f10(0), f11(0), f10(2), f11(2), f00(1),
       f01(1), f00(3), f01(3), f10(1), f11(1), f10(3), f11(3);

   if(false) {
      cout << "f00 = " << f00.transpose() << endl;
      cout << "f01 = " << f01.transpose() << endl;
      cout << "f10 = " << f10.transpose() << endl;
      cout << "f11 = " << f11.transpose() << endl;
      cout << "F = " << endl << F << endl << endl << endl << endl;
   }

   Y << 1.0, 0.0, -3., 2.0, 0.0, 0.0, 3.0, -2.0, 0.0, 1.0, -2., 1.0, 0.0, 0.0,
       -1., 1.0;

   Matrix4r FY = F * Y;
   return X * FY;
}

// MUST CALL THIS after modifying the grid
void This::finalize()
{
   for(int y = 0; y < int(A_.height); ++y)
      for(int x = 0; x < int(A_.width); ++x)
         A_(x, y)
             = evaluate_A(X(x, y), X(x + 1, y), X(x, y + 1), X(x + 1, y + 1));
}

real This::evaluate(real x, real y) const
{
   if(A_.width < 1 || A_.height < 1)
      throw std::runtime_error("Not initialized");

   if(!bounds_.contains(x, y)) {
      WARN(format("{{}, {}} not in {}", x, y, bounds_.to_string()));
      return dNAN;
   }

   auto pos_x = (x - bounds_.left) * dx_inv_;
   auto pos_y = (y - bounds_.top) * dy_inv_;

   real px = floor(pos_x);
   real py = floor(pos_y);

   if(x == bounds_.right) {
      px    = A_.width - 1.0;
      pos_x = 1.0 + px;
   }
   if(y == bounds_.bottom) {
      py    = A_.height - 1.0;
      pos_y = 1.0 + py;
   }

   auto p0 = Point2(int(px), int(py));
   if(!A_.in_bounds(p0)) FATAL("Logic Error");

   real dx = pos_x - px;
   real dy = pos_y - py;

   assert(dx >= 0.0);
   assert(dy >= 0.0);
   assert(dx <= 1.0);
   assert(dy <= 1.0);

   if(false) { // grid evaluate
      auto f00 = X(p0.x + 0, p0.y + 0)(0);
      auto f10 = X(p0.x + 1, p0.y + 0)(0);
      auto f01 = X(p0.x + 0, p0.y + 1)(0);
      auto f11 = X(p0.x + 1, p0.y + 1)(0);
      return linear_interpolate_1d(dx, dy, f00, f10, f01, f11);
   }

   Vector4r X;
   Vector4r Y;

   X(0) = 1.0; // X
   X(1) = dx;
   X(2) = X(1) * dx;
   X(3) = X(2) * dx;

   Y(0) = 1.0; // Y
   Y(1) = dy;
   Y(2) = Y(1) * dy;
   Y(3) = Y(2) * dy;

   if(false) {
      Matrix4r R1 = Matrix4r::Zero();
      Matrix4r R2 = Matrix4r::Zero();
      R1 << 1, 0, 0, 0, 1, 1, 1, 1, 0, 1, 0, 0, 0, 1, 2, 3;
      R2 << 1, 1, 0, 0, 0, 1, 1, 1, 0, 1, 0, 2, 0, 1, 0, 3;

      INFO(format("feedback on mapxy evaluate [{}, {}]", x, y));
      cout << "p0 = " << str(p0) << endl;
      cout << format("dxdy = {}, {}", dx, dy) << endl;
      cout << "X = " << X.transpose() << endl;
      cout << "Y = " << Y.transpose() << endl;
      cout << "A = " << endl << A_(p0) << endl;
      cout << "F = " << endl << (R1 * A_(p0) * R2) << endl;
      cout << endl << endl;
   }

   Vector4r AY = A_(p0) * Y;
   return X.dot(AY);
}

Point2 This::top_left(real x, real y) const
{
   Point2 o;
   o.x = clamp<int>(
       int(std::floor((x - bounds_.left) * dx_inv_)), 0, int(width()) - 2);
   o.y = clamp<int>(
       int(std::floor((y - bounds_.top) * dy_inv_)), 0, int(height()) - 2);
   return o;
}

std::string This::to_string() const
{
   std::stringstream ss("");
   serialize(ss);

   std::stringstream out("");

   out << format(R"V0G0N(
Bicublic Spline, bounds={}, dxdy=[{}, {}], weights-hash={}
)V0G0N",
                 bounds().to_string(),
                 dx(),
                 dy(),
                 md5(ss.str()));

   if(A_.height * A_.width < 16) {
      for(auto y = 0u; y < A_.height; ++y) {
         for(auto x = 0u; x < A_.width; ++x) {
            out << format("----------------------- [{}x{}]", x, y) << endl;
            out << format(" f(x)    = {}", grid_(x, y)(0)) << endl;
            out << format(" f_x(x)  = {}", grid_(x, y)(1)) << endl;
            out << format(" f_y(x)  = {}", grid_(x, y)(2)) << endl;
            out << format(" f_xy(x) = {}", grid_(x, y)(3)) << endl;
            out << " A = " << endl << A_(x, y) << endl << endl;
         }
      }
   }

   return out.str();
}

// void This::write(real * Xs) const
// {
//     for(unsigned i = 0; i < 4; ++i)
//         *Xs++ = bounds_[i];
//     for(unsigned y = 0; y < height(); ++y)
//         for(unsigned x = 0; x < width(); ++x)
//             for(unsigned k = 0; k < 4; ++k)
//                 *Xs++ = grid_(x, y)(k);
// }

// void This::read(const real * Xs)
// {
//     for(unsigned i = 0; i < 4; ++i)
//         bounds_[i] = *Xs++;
//     for(unsigned y = 0; y < height(); ++y)
//         for(unsigned x = 0; x < width(); ++x)
//             for(unsigned k = 0; k < 4; ++k)
//                 grid_(x, y)(k) = *Xs++;
//     finalize();
// }

void This::serialize(std::ostream& out) const
{
   const auto& map = *this;
   for(int i = 0; i < 4; ++i) out << " " << map.bounds()[i] << " ";
   out << map.width() << " ";
   out << map.height() << " ";

   for(const auto& X : grid_)
      for(unsigned i = 0; i < 4; ++i) out << X(i) << " ";
   for(const auto& X : A_)
      for(unsigned i = 0; i < 4; ++i)
         for(unsigned j = 0; j < 4; ++j) out << X(i, j) << " ";
}

void This::unserialize(std::istream& in)
{
   auto& map = *this;
   AABB bounds;
   unsigned n_cols, n_rows, n_params;
   for(int i = 0; i < 4; ++i) in >> bounds[i];
   in >> n_cols;
   in >> n_rows;
   map.init(bounds, n_cols, n_rows);

   for(auto& X : grid_)
      for(unsigned i = 0; i < 4; ++i) in >> X(i);
   for(auto& X : A_)
      for(unsigned i = 0; i < 4; ++i)
         for(unsigned j = 0; j < 4; ++j) in >> X(i, j);
}

} // namespace perceive
