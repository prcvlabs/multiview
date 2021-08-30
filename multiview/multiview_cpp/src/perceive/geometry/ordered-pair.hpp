
#pragma once

#include "perceive/foundation.hpp"
#include "perceive/utils/sdbm-hash.hpp"
#include "vector-2.hpp"
#include <cmath>

namespace perceive
{
// ----------------------------------------------------------------- OrderedPair
#pragma pack(push, 1)
template<typename T> class OrderedPairT
{
 public:
   using value_type = T;

 private:
   T x_{T(0.0)}, y_{T(0.0)};

 public:
   OrderedPairT() = default;
   OrderedPairT(T x, T y) { set(x, y); }
   OrderedPairT(const OrderedPairT&) = default;
   OrderedPairT(OrderedPairT&&)      = default;
   ~OrderedPairT()                   = default;
   OrderedPairT& operator=(const OrderedPairT& v) = default;
   OrderedPairT& operator=(OrderedPairT&& v) = default;

   T x() const { return x_; }
   T y() const { return y_; }

   void set(T x, T y)
   {
      if(x <= y) {
         x_ = x;
         y_ = y;
      } else {
         x_ = y;
         y_ = x;
      }
   }

   void set_x(T x_) { set(x_, y()); }
   void set_y(T y_) { set(x(), y_); }

   bool has(T val) const noexcept { return x_ == val || y_ == val; }
   auto other(T val) const noexcept
   {
      Expects(has(val));
      return (x_ == val) ? y_ : x_;
   }

   unsigned size() const { return 2; }

   bool operator==(const OrderedPairT& rhs) const
   {
      return x_ == rhs.x_ && y_ == rhs.y_;
   }
   bool operator!=(const OrderedPairT& rhs) const { return !(*this == rhs); }

   bool operator<(const OrderedPairT& rhs) const
   {
      return x_ == rhs.x_ ? y_ < rhs.y_ : x_ < rhs.x_;
   }
   bool operator<=(const OrderedPairT& rhs) const
   {
      return x_ == rhs.x_ ? y_ <= rhs.y_ : x_ <= rhs.x_;
   }
   bool operator>(const OrderedPairT& rhs) const { return !(*this <= rhs); }
   bool operator>=(const OrderedPairT& rhs) const { return !(*this < rhs); }

   std::string to_string(const char* fmt = "[{} {}]") const
   {
      return format(fmt, x(), y());
   }
   std::string to_str() const { return format("[{}, {}]", x(), y()); }

   size_t hash() const { return sdbm_hash(&x_, sizeof(T) * size()); }
};
#pragma pack(pop)

// String shim
template<typename T> std::string str(const OrderedPairT<T>& v)
{
   return v.to_string();
}
template<typename T>
std::ostream& operator<<(std::ostream& out, const OrderedPairT<T>& v)
{
   out << v.to_string();
   return out;
}

} // namespace perceive
