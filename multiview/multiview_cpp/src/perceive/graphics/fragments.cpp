
#include "fragments.hpp"

#define This Fragments

namespace perceive::graphics
{
void This::push_fragment(Fragment f) { fragments_.push_back(f); }

void This::render(const DistortedCamera& dcam, ARGBImage& im)
{
   // z-order fragments
   const auto Cf = to_vec3f(dcam.C);

   std::sort(begin(fragments_),
             end(fragments_),
             [&](const auto& A, const auto& B) -> bool {
                return (A.centre() - Cf).quadrance()
                       > (B.centre() - Cf).quadrance();
             });
}

} // namespace perceive::graphics
