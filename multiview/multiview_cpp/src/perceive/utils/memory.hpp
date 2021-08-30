
#pragma once

#include <memory>
#include <new>

// --------------------------------------------------------------------- Bitcast
//
template<typename To,
         typename From,
         typename = std::enable_if<(sizeof(To) == sizeof(From))
                                   && std::is_trivially_copyable<To>::value
                                   && std::is_trivially_copyable<From>::value>>
To bit_cast(const From& src) noexcept
{
   To dst;
   std::memcpy(&dst, &src, sizeof(To)); // The compiler optimizes this away
   return dst;
}

// ----------------------------------------------------------- memory-is-aligned
//
template<typename T> constexpr bool memory_is_aligned(const void* ptr) noexcept
{
   const std::size_t align  = std::alignment_of<T>::value;
   const std::uintptr_t val = bit_cast<std::uintptr_t, const void*>(ptr);
   return val % align == 0;
}

// -------------------------------------------------------- Allocation Functions
//
namespace perceive::detail
{
void* allocate(std::size_t sz) noexcept(false); // throws std::bad_alloc
void deallocate(void* ptr, std::size_t sz) noexcept;

std::size_t allocated() noexcept;

// These functions store a 'tag' before the allocation
void* default_new(std::size_t sz, std::size_t al);
void default_del(void* ptr);

} // namespace perceive::detail

// ------------------------------------------------------------------- Auxiliary
//
namespace perceive
{
std::string pp_mem_sz(std::size_t bytes) noexcept;
void pp_atexit_mem_usage() noexcept;
} // namespace perceive

// ------------------------------------------------------------------ MAllocator
//
namespace perceive
{
template<class T> struct MAllocator
{
   typedef T value_type;

   MAllocator() = default;
   template<class U> constexpr MAllocator(const MAllocator<U>&) noexcept {}

   [[nodiscard]] T* allocate(std::size_t n) noexcept(false)
   {
      static constexpr std::size_t max_n = std::size_t(-1) / sizeof(T);
      if(n > max_n) throw std::bad_alloc();
      return static_cast<T*>(detail::allocate(n * sizeof(T)));
   }

   void deallocate(T* p, std::size_t sz) noexcept { detail::deallocate(p, sz); }
};
} // namespace perceive

template<class T, class U>
bool operator==(const perceive::MAllocator<T>&, const perceive::MAllocator<U>&)
{
   return true;
}
template<class T, class U>
bool operator!=(const perceive::MAllocator<T>&, const perceive::MAllocator<U>&)
{
   return false;
}

// ------------------------------------------------------- Customize Make-shared

// We need a custom "make-shared" so that we can track allocations
namespace perceive
{
// #ifdef PERCEIVE_PROFILE_MEMORY
// template<class T, class... Args> std::shared_ptr<T> make_shared(Args&&...
// args)
// {
//    return std::allocate_shared<T>(MAllocator<T>{},
//    std::forward<Args>(args)...);
// }
// #else
// using std::make_shared;
// using std::make_unique;
// #endif

// template<class _Alloc, class... _Args>
// static shared_ptr<_Tp> allocate_shared(const _Alloc& __a, _Args&&... __args);

} // namespace perceive

#ifdef PERCEIVE_PROFILE_MEMORY

// Custom new/delete operators are used to track memory usage
// void* operator new(std::size_t sz);
// void* operator new[](std::size_t sz);
// void* operator new(std::size_t sz, std::align_val_t al);
// void* operator new[](std::size_t sz, std::align_val_t al);
// void operator delete(void* ptr) throw();
// void operator delete[](void* ptr) throw();

#define CUSTOM_NEW_DELETE(clazz)                                              \
   static void* operator new(std::size_t sz) { return detail::allocate(sz); } \
   static void* operator new[](std::size_t sz)                                \
   {                                                                          \
      return detail::allocate(sz);                                            \
   }                                                                          \
   static void operator delete(void* ptr, std::size_t sz)                     \
   {                                                                          \
      detail::deallocate(ptr, sz);                                            \
   }                                                                          \
   static void operator delete[](void* ptr, std::size_t sz)                   \
   {                                                                          \
      detail::deallocate(ptr, sz);                                            \
   }

#else
#define CUSTOM_NEW_DELETE(clazz)
#endif
