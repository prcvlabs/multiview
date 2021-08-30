
#pragma once

#include <Eigen/Dense>

#include "perceive/foundation.hpp"
#include "perceive/utils/sdbm-hash.hpp"
#include "vector-2.hpp"
#include "vector-3.hpp"

namespace perceive
{
// An "axis-aligned" bounding box
#pragma pack(push, 1)
template<typename T> class AABBT
{
 public:
   CUSTOM_NEW_DELETE(AABBT)

   using value_type = T;

   T left{T(0.0)}, top{T(0.0)}, right{T(0.0)}, bottom{T(0.0)};

   AABBT() noexcept             = default;
   AABBT(const AABBT&) noexcept = default;
   AABBT(AABBT&&) noexcept      = default;
   ~AABBT()                     = default;
   AABBT& operator=(const AABBT&) noexcept = default;
   AABBT& operator=(AABBT&&) noexcept = default;

   AABBT(T in_left, T in_top, T in_right, T in_bottom)
   noexcept
       : left(in_left)
       , top(in_top)
       , right(in_right)
       , bottom(in_bottom)
   {}
   AABBT(Vector2T<T> lefttop, Vector2T<T> rightbottom)
   noexcept
       : left(lefttop.x)
       , top(lefttop.y)
       , right(rightbottom.x)
       , bottom(rightbottom.y)
   {}

   template<class Iterator> AABBT(Iterator first, Iterator last) noexcept
   {
      *this = minmax();
      union_with(first, last);
   }

   static AABBT nan() noexcept { return AABBT(T(NAN), T(NAN), T(NAN), T(NAN)); }
   static AABBT minmax() noexcept
   {
      auto mmax = std::numeric_limits<T>::max();
      auto mmin = std::numeric_limits<T>::lowest();
      return AABBT<T>(mmax, mmax, mmin, mmin);
   }

   bool is_finite() const noexcept
   {
      return true && std::isfinite(left) && std::isfinite(top)
             && std::isfinite(right) && std::isfinite(bottom);
   }

   // Union a point with the AABB
   void union_point(T x, T y) noexcept
   {
      if(std::isfinite(x)) {
         if(x < left) left = x;
         if(x > right) right = x;
      }
      if(std::isfinite(y)) {
         if(y < top) top = y;
         if(y > bottom) bottom = y;
      }
   }

   void union_point(const Vector2T<T>& p) noexcept { union_point(p.x, p.y); }
   void union_point(const T p[2]) noexcept { union_point(p[0], p[1]); }

   bool contains(T x, T y) const noexcept
   {
      if constexpr(std::is_floating_point<T>::value) {
         return is_close_between(x, left, right)
                and is_close_between(y, top, bottom);
      }
      return (x >= left) && (x <= right) && (y >= top) && (y <= bottom);
   }
   bool contains(const Vector2T<T>& p) const noexcept
   {
      return contains(p.x, p.y);
   }
   bool contains(const T p[2]) const noexcept { return contains(p[0], p[1]); }

   template<class Iterator>
   void union_with(Iterator first, Iterator last) noexcept
   {
      while(first != last) union_point(*first++);
   }

   T w() const noexcept
   {
      return right - left;
      if constexpr(std::is_integral_v<T>)
         return abs(right - left);
      else
         return fabs(right - left);
   }
   T h() const noexcept
   {
      return bottom - top;
      if constexpr(std::is_integral_v<T>)
         return abs(bottom - top);
      else
         return fabs(bottom - top);
   }
   T width() const noexcept { return w(); }
   T height() const noexcept { return h(); }

   T area() const noexcept { return w() * h(); }

   AABBT& grow(T inc) noexcept
   {
      left -= inc;
      top -= inc;
      right += inc;
      bottom += inc;
      return *this;
   }

   static AABBT intersection(const AABBT& a, const AABBT& b) noexcept
   {
      AABBT<T> c = a;

      c.left   = std::max(a.left, b.left);
      c.right  = std::min(a.right, b.right);
      c.top    = std::max(a.top, b.top);
      c.bottom = std::min(a.bottom, b.bottom);

      if(c.w() < 0) {
         c.left  = T(0.5 * double(c.left + c.right));
         c.right = c.left;
      }

      if(c.h() < 0) {
         c.top    = T(0.5 * double(c.top + c.bottom));
         c.bottom = c.top;
      }

      return c;
   }

   static T intersection_area(const AABBT& a, const AABBT& b) noexcept
   {
      AABBT<T> c = a;

      c.left   = std::max(a.left, b.left);
      c.right  = std::min(a.right, b.right);
      c.top    = std::max(a.top, b.top);
      c.bottom = std::min(a.bottom, b.bottom);

      T w = c.w();
      T h = c.h();

      return (w > 0.0 && h > 0.0) ? (w * h) : T(0.0);
   }

   // Raster order in images has a "flipped" y-axis
   AABBT reflect_raster() const noexcept
   {
      AABBT o = *this;
      std::swap(o.top, o.bottom);
      return o;
   }

   T intersection_area(const AABBT& o) const noexcept
   {
      return intersection_area(*this, o);
   }

   static T union_area(const AABBT& a, const AABBT& b) noexcept
   {
      return a.area() + b.area() - intersection_area(a, b);
   }
   T union_area(const AABBT& o) const noexcept { return union_area(*this, o); }

   Vector2T<T> centre() const noexcept
   {
      return Vector2T<T>(T(left + 0.5 * (right - left)),
                         T(top + 0.5 * (bottom - top)));
   }
   Vector2T<T> center() const noexcept { return centre(); }

   real aspect_ratio() const noexcept { return real(width()) / real(height()); }

   // Scale the AABB by some size
   AABBT& operator*=(T scalar) noexcept
   {
      Vector2T<T> c = centre();
      T w           = this->w() * scalar * 0.5;
      T h           = this->h() * scalar * 0.5;
      left          = c.x - w;
      right         = c.x + w;
      top           = c.y - h;
      bottom        = c.y + h;
      return *this;
   }

   AABBT& operator/=(T scalar) noexcept { return *this *= (1.0 / scalar); }
   AABBT operator*(T scalar) const noexcept
   {
      AABBT res(*this);
      res *= scalar;
      return res;
   }
   AABBT operator/(T scalar) const noexcept
   {
      AABBT res(*this);
      res /= scalar;
      return res;
   }

   Vector2T<T> left_top() const noexcept { return Vector2T<T>(left, top); }
   Vector2T<T> top_left() const noexcept { return Vector2T<T>(left, top); }
   Vector2T<T> right_bottom() const noexcept
   {
      return Vector2T<T>(right, bottom);
   }
   Vector2T<T> bottom_right() const noexcept
   {
      return Vector2T<T>(right, bottom);
   }

   Vector2T<T> corner(int index) const noexcept
   {
      switch(index) {
      case 0: return Vector2T<T>(left, bottom);
      case 1: return Vector2T<T>(left, top);
      case 2: return Vector2T<T>(right, top);
      case 3: return Vector2T<T>(right, bottom);
      }
      FATAL("kBAM!");
      return {};
   }

   std::array<Vector2T<T>, 4> to_array_polygon() const noexcept
   {
      std::array<Vector2T<T>, 4> X;
      X[0] = Vector2T<T>(left, top);
      X[1] = Vector2T<T>(right, top);
      X[2] = Vector2T<T>(right, bottom);
      X[3] = Vector2T<T>(left, bottom);
      return X;
   }

   vector<Vector2T<T>> to_polygon() const noexcept(false)
   {
      vector<Vector2T<T>> X(4);
      X[0] = Vector2T<T>(left, top);
      X[1] = Vector2T<T>(right, top);
      X[2] = Vector2T<T>(right, bottom);
      X[3] = Vector2T<T>(left, bottom);
      return X;
   }

   Vector3T<T> left_line() const noexcept
   {
      T l = this->left, t = this->top, r = this->right, b = this->bottom;
      return to_homgen_line(Vector3T<T>(l, t, 1), Vector3T<T>(l, b, 1));
   }

   Vector3T<T> right_line() const noexcept
   {
      T l = this->left, t = this->top, r = this->right, b = this->bottom;
      return to_homgen_line(Vector3T<T>(r, t, 1), Vector3T<T>(r, b, 1));
   }

   Vector3T<T> top_line() const noexcept
   {
      T l = this->left, t = this->top, r = this->right, b = this->bottom;
      return to_homgen_line(Vector3T<T>(l, t, 1), Vector3T<T>(r, t, 1));
   }

   Vector3T<T> bottom_line() const noexcept
   {
      T l = this->left, t = this->top, r = this->right, b = this->bottom;
      return to_homgen_line(Vector3T<T>(l, b, 1), Vector3T<T>(r, b, 1));
   }

   Vector3T<T> line(int ind) const noexcept
   {
      switch(ind) {
      case 0: return left_line();
      case 1: return top_line();
      case 2: return right_line();
      case 3: return bottom_line();
      }
      return left_line();
   }

   bool operator!=(const AABBT<T>& rhs) const noexcept
   {
      return !(*this == rhs);
   }
   bool operator==(const AABBT<T>& rhs) const noexcept
   {
      return is_close(left, rhs.left) and is_close(top, rhs.top)
             and is_close(right, rhs.right) and is_close(bottom, rhs.bottom);
   }

   AABBT& set_to(const T& l, const T& t, const T& r, const T& b) noexcept
   {
      left   = l;
      top    = t;
      right  = r;
      bottom = b;
      return *this;
   }
   AABBT& set_to(T a[4]) noexcept
   {
      set_to(a[0], a[1], a[2], a[3]);
      return *this;
   }

   T* copy_to(T a[4]) const noexcept
   {
      a[0] = left;
      a[1] = top;
      a[2] = right;
      a[3] = bottom;
      return a;
   }

   T* ptr() noexcept { return &left; }
   const T* ptr() const noexcept { return &left; }
   T& operator[](int idx) noexcept
   {
#ifdef DEBUG_BUILD
      assert(idx >= 0 && idx < 4);
#endif
      return ptr()[idx];
   }

   const T& operator[](int idx) const noexcept
   {
#ifdef DEBUG_BUILD
      assert(idx >= 0 && idx < 4);
#endif
      return ptr()[idx];
   }

   T& operator()(int idx) noexcept
   {
#ifdef DEBUG_BUILD
      assert(idx >= 0 && idx < 4);
#endif
      return ptr()[idx];
   }
   const T& operator()(int idx) const noexcept
   {
#ifdef DEBUG_BUILD
      assert(idx >= 0 && idx < 4);
#endif
      return ptr()[idx];
   }

   unsigned size() const noexcept { return 4; }

   std::string to_string(const char* fmt = "[{} {} {} {}]") const
   {
      return format(fmt, left, top, right, bottom);
   }

   std::string to_str() const { return to_string(); }

   size_t hash() const noexcept { return sdbm_hash(ptr(), sizeof(T) * size()); }

   friend std::string str(const AABBT<T>& aabb) { return aabb.to_string(); }
};
#pragma pack(pop)

template<typename T>
AABBT<T> intersection(const AABBT<T>& a, const AABBT<T>& b) noexcept
{
   return AABBT<T>::intersection(a, b);
}

template<typename T>
T intersection_area(const AABBT<T>& a, const AABBT<T>& b) noexcept
{
   return AABBT<T>::intersection_area(a, b);
}

template<typename T> T union_area(const AABBT<T>& a, const AABBT<T>& b) noexcept
{
   return AABBT<T>::union_area(a, b);
}

// Scalar multiples
template<typename T> AABBT<T> operator*(float a, const AABBT<T>& v) noexcept
{
   return v * a;
}
template<typename T> AABBT<T> operator/(float a, const AABBT<T>& v) noexcept
{
   return v / a;
}
template<typename T> AABBT<T> operator*(double a, const AABBT<T>& v) noexcept
{
   return v * a;
}
template<typename T> AABBT<T> operator/(double a, const AABBT<T>& v) noexcept
{
   return v / a;
}

// Intersection of homogeneous line ('line') with AABB
template<typename T>
std::pair<Vector2T<T>, Vector2T<T>>
intersect(const AABBT<T>& aabb, const Vector3T<T>& in_line) noexcept
{
   array<Vector3T<T>, 4> lines;
   array<Vector3T<T>, 4> isects;
   array<bool, 4> bounded;
   std::pair<Vector2T<T>, Vector2T<T>> ret
       = {Vector2T<T>::nan(), Vector2T<T>::nan()};

   const auto line = in_line.normalised_line();

   { // What are the lines of the AABB?, the order is [left, top, right, bot]
      int counter = 0;
      std::generate(
          begin(lines), end(lines), [&]() { return aabb.line(counter++); });
   }

   { // Where does 'line' intersect with each line of the AABB?
      std::transform(
          cbegin(lines), cend(lines), begin(isects), [&](const auto& ll) {
             return cross(line, ll).normalise_point();
          });
   }

   { // Are inserections on the border of the AABB?
     // Point 'X' is between lines 'l1' and 'l2'
      auto is_between = [&](const Vector3T<T>& X,
                            const Vector3T<T>& l1,
                            const Vector3T<T>& l2) -> bool {
         return X.is_finite() && (dot(X, l1) < 0.0) == (dot(X, l2) < 0.0);
      };

      bounded[0] = is_close_between(isects[0].y, aabb.top, aabb.bottom);
      bounded[1] = is_close_between(isects[1].x, aabb.left, aabb.right);
      bounded[2] = is_close_between(isects[2].y, aabb.top, aabb.bottom);
      bounded[3] = is_close_between(isects[3].x, aabb.left, aabb.right);
   }

   unsigned pos     = 0;
   auto push_result = [&](const Vector3T<T>& X) {
      if(pos < 2) {
         auto& p = (pos++ == 0) ? ret.first : ret.second;
         p(0)    = X(0);
         p(1)    = X(1);
      } else {
         const auto v  = Vector2T<T>(X(0), X(1));
         const auto q0 = (ret.first - ret.second).quadrance();
         const auto q1 = (ret.first - v).quadrance();
         if(q1 > q0) ret.second = v;
      }
   };

   for(size_t i = 0; i < 4; ++i)
      if(bounded[i]) push_result(isects[i]);

   if(false) {
      INFO("FEEDBACK");
      cout << format("AABB  = {}", str(aabb)) << endl;
      cout << format("line  = {}", str(line)) << endl;
      for(auto i = 0; i < 4; ++i) {
         cout << format(" {:c} ll = {}, isect = {{}, {}}",
                        (bounded[size_t(i)] ? '*' : ' '),
                        str(lines[size_t(i)]),
                        isects[size_t(i)].x,
                        isects[size_t(i)].y)
              << endl;
      }
      cout << format("out   = {{}, {}} -> {{}, {}}",
                     ret.first.x,
                     ret.first.y,
                     ret.second.x,
                     ret.second.y)
           << endl
           << endl;
   }

   return ret;
}

template<typename T>
std::pair<Vector2T<T>, Vector2T<T>> intersect(const AABBT<T>& aabb,
                                              const Vector2T<T>& A,
                                              const Vector2T<T>& B) noexcept
{
   const bool A_in = aabb.contains(A);
   const bool B_in = aabb.contains(B);

   if(A_in and B_in) return {A, B};

   const auto ll     = to_homgen_line(A, B);
   const auto [U, V] = intersect(aabb, ll);

   auto orthogonal_line_at = [&](const auto& U) -> auto
   {
      return Vector3T<T>(-ll.y, ll.x, ll.y * U.x - ll.x * U.y);
   };

   auto is_between_A_B = [&](const auto& U) -> bool {
      const auto tt = orthogonal_line_at(U);
      const auto da = tt.x * A.x + tt.y * A.y + tt.z;
      const auto db = tt.x * B.x + tt.y * B.y + tt.z;
      return std::signbit(da) != std::signbit(db);
   };

   auto subline
       = [&](const auto& X, const auto& Y, const auto& U, const auto& V) {
            return is_between_A_B(U) ? U : V;
         };

   if(A_in)
      return {A, subline(A, B, U, V)};
   else if(B_in)
      return {subline(B, A, U, V), B};
   if(is_between_A_B(U) and is_between_A_B(V)) return {U, V};
   return {Vector2T<T>::nan(), Vector2T<T>::nan()};
}

// The returned homography maps points in the 'src' AABB to the 'dst' AABB
template<typename T>
inline Eigen::Matrix3d rescale_homography(const AABBT<T>& src,
                                          const AABBT<T>& dst) noexcept
{
   Eigen::Matrix3d H = Eigen::Matrix3d::Identity();
   H(0, 0)           = dst.width() / src.width();
   H(1, 1)           = dst.height() / src.height();
   H(0, 2)           = dst.left - H(0, 0) * src.left;
   H(1, 2)           = dst.top - H(1, 1) * src.top;

   // auto tryit = [&] (Vector2 s, Vector2 d) {
   //     Vector3r x = H * Vector3r(s.x, s.y, 1.0);
   //     x /= x(2);
   //     cout << format("{} => {} ({}, {})",
   //                    s.to_string(), d.to_string(), x(0), x(1))
   //     << endl;
   // };

   // tryit(src.top_left(), dst.top_left());
   // tryit(src.right_bottom(), dst.right_bottom());

   return H;
}

template<typename T>
inline bool in_bounding_box(const Vector3T<T>& X,
                            const Vector3T<T>& A,
                            const Vector3T<T>& B)
{
   for(auto i = 0; i < 3; ++i)
      if(X(i) < A(i)) return false;
   for(auto i = 0; i < 3; ++i)
      if(X(i) > B(i)) return false;
   return true;
};

} // namespace perceive
