
#include "gl-fragment.hpp"
#include "perceive/geometry/vector.hpp"
#include "perceive/utils/spin-lock.hpp"

namespace perceive
{
struct GlFragmentBuffer
{
 private:
   vector<GlFragment> frags_;
   SpinLock padlock_;

 public:
   static GlFragmentBuffer& instance();

   void push(const GlFragment& frag)
   {
      lock_guard lock(padlock_);
      frags_.push_back(frag);
   }

   template<typename InputIt> void push(InputIt begin, InputIt end)
   {
      lock_guard lock(padlock_);
      frags_.reserve(frags_.size() + std::distance(begin, end));
      frags_.insert(frags_.end(), begin, end);
   }

   vector<GlFragment> get_and_flush();
   void sort_render_and_flush();
};

inline GlFragmentBuffer& global_frag_buffer()
{
   return GlFragmentBuffer::instance();
}

} // namespace perceive
