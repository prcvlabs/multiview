
#pragma once

namespace perceive
{
template<typename T> struct Line1DT
{
   static_assert(std::is_arithmetic<T>::value);

   T a = T(0);
   T b = T(0);

   Line1DT() = default;
   Line1DT(T a_, T b_);
   Line1DT(const Line1DT&) = default;
   Line1DT(Line1DT&&)      = default;
   ~Line1DT()              = default;
   Line1DT& operator=(const Line1DT&) = default;
   Line1DT& operator=(Line1DT&&) = default;

   bool operator==(const Line1DT& o) const noexcept;
   bool operator!=(const Line1DT& o) const noexcept;
   bool operator<(const Line1DT& o) const noexcept;
   bool operator<=(const Line1DT& o) const noexcept;
   bool operator>(const Line1DT& o) const noexcept;
   bool operator>=(const Line1DT& o) const noexcept;

   T length() const noexcept { return b - a; }

   bool empty() const noexcept { return a == b; }
   Line1DT& normalise() noexcept;
   Line1DT normalised() const noexcept;

   T front() const noexcept { return a; }
   T back() const noexcept { return b; }

   string to_string() const noexcept;
};

template<typename T>
Line1DT<T> union_1d(const Line1DT<T>& A, const Line1DT<T>& B) noexcept;

template<typename T>
Line1DT<T> intersection_1d(const Line1DT<T>& A, const Line1DT<T>& B) noexcept;

template<typename T>
Line1DT<T> difference_1d(const Line1DT<T>& A, const Line1DT<T>& B) noexcept;

using Line1D  = Line1DT<real>;
using Line1Df = Line1DT<float>;
using Line1Di = Line1DT<int>;

template<typename T> string str(const Line1DT<T>& ll) noexcept
{
   return ll.to_string();
}

//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
// -------------------------------------------------------------- implementation

template<typename T>
Line1DT<T>::Line1DT(T a_, T b_)
    : a(a_)
    , b(b_)
{}

template<typename T>
bool Line1DT<T>::operator==(const Line1DT<T>& o) const noexcept
{
   return (a == o.a) and (b == o.b);
}

template<typename T>
bool Line1DT<T>::operator!=(const Line1DT<T>& o) const noexcept
{
   return !(*this == o);
}

template<typename T> bool Line1DT<T>::operator<(const Line1DT& o) const noexcept
{
   if(a < o.a and b < o.b) return true;
   if(a > o.a and b > o.b) return false;
   if(a == o.a) return b < o.b;
   return a < o.a;
}

template<typename T>
bool Line1DT<T>::operator<=(const Line1DT& o) const noexcept
{
   return (*this < o) or (*this == 0);
}

template<typename T> bool Line1DT<T>::operator>(const Line1DT& o) const noexcept
{
   return !(*this <= o);
}

template<typename T>
bool Line1DT<T>::operator>=(const Line1DT& o) const noexcept
{
   return !(this < o);
}

template<typename T> Line1DT<T>& Line1DT<T>::normalise() noexcept
{
   using std::swap;
   if(a > b) swap(a, b);
   return *this;
}

template<typename T> Line1DT<T> Line1DT<T>::normalised() const noexcept
{
   auto ret = Line1DT<T>(*this);
   ret.normalise();
   return ret;
}

template<typename T> string Line1DT<T>::to_string() const noexcept
{
   return format("[{}, {}]", a, b);
}

template<typename T>
Line1DT<T> union_1d(const Line1DT<T>& A, const Line1DT<T>& B) noexcept
{
   return Line1DT(std::min(A.a, B.a), std::max(A.b, B.b));
}

template<typename T>
Line1DT<T> intersection_1d(const Line1DT<T>& A, const Line1DT<T>& B) noexcept
{
   const auto a = A.normalised();
   const auto b = B.normalised();
   auto ret     = Line1DT(std::max(a.a, b.a), std::min(a.b, b.b));
   if(ret.length() < T(0.0)) ret.a = ret.b;
   return ret;
}

template<typename T>
Line1DT<T> difference_1d(const Line1DT<T>& A, const Line1DT<T>& B) noexcept
{
   if(B.empty()) return A;
   const auto a = A.normalised();
   const auto b = B.normalised();
   if((b.a >= a.b) or (b.b <= a.a)) return a; // 'b' BEFORE or AFTER 'a'
   if(b.a > a.a) return Line1DT(a.a, b.a);
   return Line1DT(std::min(a.b, b.b), a.b);
}

} // namespace perceive
