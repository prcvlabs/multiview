
#pragma once

#include <chrono>
#include <functional>
#include <iostream>

namespace perceive
{
inline std::chrono::time_point<std::chrono::steady_clock> tick() noexcept
{
   return std::chrono::steady_clock::now();
}

inline double
tock(const std::chrono::time_point<std::chrono::steady_clock>& whence) noexcept
{
   using ss = std::chrono::duration<double, std::ratio<1, 1>>;
   return std::chrono::duration_cast<ss>(tick() - whence).count();
}

inline int ms_tock(
    const std::chrono::time_point<std::chrono::steady_clock>& whence) noexcept
{
   return int(std::round(tock(whence) * 1000.0));
}

inline double ms_tock_f(
    const std::chrono::time_point<std::chrono::steady_clock>& whence) noexcept
{
   return std::round(tock(whence) * 1000000.0) * 0.001;
}

inline string ms_tock_s(
    const std::chrono::time_point<std::chrono::steady_clock>& whence) noexcept
{
   return format("{:7.3f}", std::round(tock(whence) * 1000000.0) * 0.001);
}

inline double time_thunk(std::function<void(void)> f)
{
   auto now = tick();
   f();
   return tock(now);
}

inline double profile_thunk(std::function<void(void)> f,
                            unsigned iterations,
                            const char* message)
{
   auto s = 0.0;
   if(iterations == 0) {
      auto now         = tick();
      unsigned counter = 0;
      while((s = tock(now)) < 1.0) {
         f();
         ++counter;
      }
      s = s / double(counter);
   } else {
      auto now = tick();
      for(unsigned i = 0; i < iterations; ++i) f();
      s = tock(now) / double(iterations);
   }
   if(message != nullptr)
      std::cout << message << " - " << s << "s" << std::endl;
   return s;
}

#ifdef BENCHMARK
// Turn off all type information optimization for passed pointer
inline void escape_optimizer(void* p)
{
   // 'asm volatile' tells comipler that this assembly code
   // has unknowable side effects
   asm volatile("" : : "g"(p) : "memory");
}

// Tell optimizer that all memory is modified
inline void clobber_optimizer() { asm volatile("" : : : "memory"); }
#endif

} // namespace perceive
