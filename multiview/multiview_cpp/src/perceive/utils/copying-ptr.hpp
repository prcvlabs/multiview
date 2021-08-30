
#pragma once

#include <memory>

namespace nonstd
{
namespace detail
{
   template<typename T, typename F> class copyer : public T
   {
    public:
      copyer(T&& t)
          : T(std::forward(t))
      {}
      copyer(copyer&& o) = default;
      copyer(copyer& o)
          : T(F()(o)) // Call `F` to make a copy
      {}

      copyer& operator=(copyer&& o) = default;
      copyer& operator              =(const copyer& o)
      {
         using std::swap;        // `*this` unchanged if `copyer(o)`
         swap(*this, copyer(o)); // throws and exception
         return *this;
      }
   };

   template<typename T> struct CopyF
   {
      std::unique_ptr<T> operator()(const std::unique_ptr<T>& o)
      {
         return std::make_unique<T>(*o);
      }
   };

} // namespace detail

template<typename T>
using copying_ptr = detail::copyer<std::unique_ptr<T>, detail::CopyF<T>>;

} // namespace nonstd
