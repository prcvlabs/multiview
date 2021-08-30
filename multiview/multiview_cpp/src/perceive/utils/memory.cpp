
#include "memory.hpp"

#include <cstdlib>

static constexpr std::size_t k_one_kilo = 1024 * 1;
static constexpr std::size_t k_one_meg  = 1024 * k_one_kilo;
static constexpr std::size_t k_one_gig  = 1024 * k_one_meg;

static constexpr double k_one_kilo_inv = 1.0 / k_one_kilo;
static constexpr double k_one_meg_inv  = 1.0 / k_one_meg;
static constexpr double k_one_gig_inv  = 1.0 / k_one_gig;

namespace perceive::detail
{
// using perceive::pp_mem_sz;

static array<char, 60> pp_mem_sz_(std::size_t bytes)
{
   array<char, 60> ret;

   int written = 0;
   if(bytes < k_one_kilo)
      written = snprintf(&ret[0], ret.size(), "%d bytes", int(bytes));
   else if(bytes < k_one_meg)
      written = snprintf(
          &ret[0], ret.size(), "%g k", double(bytes) * k_one_kilo_inv);
   else if(bytes < k_one_gig)
      written = snprintf(
          &ret[0], ret.size(), "%g MiB", double(bytes) * k_one_meg_inv);
   else
      written = snprintf(
          &ret[0], ret.size(), "%g GiB", double(bytes) * k_one_gig_inv);

   Expects(written < int(ret.size()) - 1);
   return ret;
}

struct Allocator
{
   std::atomic<std::size_t> allocated_{0};

   static Allocator& instance()
   {
      static std::atomic<Allocator*> instance_{nullptr};
      static std::mutex padlock;

      auto tmp = instance_.load(std::memory_order_acquire);
      if(tmp == nullptr) {
         std::lock_guard<std::mutex> lock(padlock);
         tmp = instance_.load(std::memory_order_relaxed);
         if(tmp == nullptr) {
            auto ptr = std::malloc(sizeof(Allocator));
            tmp      = new(ptr) Allocator{};
            instance_.store(tmp, std::memory_order_release);
         }
      }

      return *tmp;
   }

   static void at_exit_handler() {}

   Allocator() { std::atexit(at_exit_handler); }

   void* allocate(std::size_t sz) noexcept(false)
   {
      if(sz > k_one_gig) {
         auto s = pp_mem_sz_(sz);
         fprintf(stderr, "WARNING: attempting to allocate %s", &s[0]);
      }

      auto ret = std::malloc(sz);
      if(ret == nullptr) throw std::bad_alloc{};
      allocated_ += sz;

      if(false) {
         auto s = pp_mem_sz_(sz);
         fprintf(stdout, "  ALLOCATE -> %s\n", &s[0]);
      }

      return ret;
   }

   void deallocate(void* ptr, std::size_t sz) noexcept
   {
      if(ptr == nullptr) return;
      std::free(ptr);
      allocated_ -= sz;

      if(false) {
         auto s = pp_mem_sz_(sz);
         fprintf(stdout, "DEALLOCATE -> %s\n", &s[0]);
      }
   }

   std::size_t allocated() noexcept
   {
      auto sz = allocated_.load(std::memory_order_relaxed);
      return sz;
   }
};

static Allocator& instance() { return Allocator::instance(); }

// throws std::bad_alloc
void* allocate(std::size_t sz) noexcept(false)
{
   return instance().allocate(sz);
}

void deallocate(void* ptr, std::size_t sz) noexcept
{
   instance().deallocate(ptr, sz);
}

std::size_t allocated() noexcept { return instance().allocated(); }

void* default_new(std::size_t sz, std::size_t al)
{
   std::size_t offset = sizeof(std::size_t);
   if(al != 0) {
      fprintf(stderr, "alignment = %d not implemented\n", int(al));
      exit(1);
   }

   auto ptr0 = static_cast<char*>(instance().allocate(sz + offset));
   if(ptr0 == nullptr) return nullptr;
   *reinterpret_cast<std::size_t*>(ptr0) = sz + offset;
   auto ptr                              = ptr0 + offset;
   if(!(al == 0 or (reinterpret_cast<std::size_t>(ptr) % al) == 0)) {
      fprintf(stderr, "alignmentment error\n");
      exit(1);
   }
   return ptr;
}

void default_del(void* ptr)
{
   if(ptr == nullptr) return; // we're done

   // Find the tag
   std::size_t* tag_ptr = static_cast<std::size_t*>(ptr);
   --tag_ptr;
   while(*tag_ptr == 0) --tag_ptr;
   instance().deallocate(tag_ptr, *tag_ptr);
}

} // namespace perceive::detail

namespace perceive
{
string pp_mem_sz(std::size_t bytes) noexcept
{
   auto s = detail::pp_mem_sz_(bytes);
   return string(&s[0]);
}

void pp_atexit_mem_usage() noexcept
{
#ifdef PERCEIVE_PROFILE_MEMORY
   cout << format("ATEXIT memory usage: {}", pp_mem_sz(detail::allocated()))
        << endl;
#endif
}

} // namespace perceive

// -------------------------------------------------------- Overriden new/delete
//
#ifdef zPERCEIVE_PROFILE_MEMORY
// SOMETHING ELSE IS TAGGING POINTERS ARGGG!!!
void* operator new(std::size_t sz)
{
   return perceive::detail::default_new(sz, 0);
}

void* operator new[](std::size_t sz)
{
   return perceive::detail::default_new(sz, 0);
}

void* operator new(std::size_t sz, std::align_val_t al)
{
   return perceive::detail::default_new(sz, static_cast<std::size_t>(al));
}

void* operator new[](std::size_t sz, std::align_val_t al)
{
   return perceive::detail::default_new(sz, static_cast<std::size_t>(al));
}

void operator delete(void* ptr) throw()
{
   return perceive::detail::default_del(ptr);
}

void operator delete[](void* ptr) throw()
{
   return perceive::detail::default_del(ptr);
}
#endif
