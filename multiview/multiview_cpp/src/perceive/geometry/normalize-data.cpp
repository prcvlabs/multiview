
#include "normalize-data.hpp"

#include "perceive/utils/eigen-helpers.hpp"

namespace perceive
{
// SVD on 3d data
// For best results, center and scale the data first.

std::pair<Vector3f, Vector3f> rectify_3d_vec(vector<Vector3f>& data)
{
   Vector3f major_axis = {0.0f, 0.0f, 0.0f};
   Vector3f minor_axis = {0.0f, 0.0f, 0.0f};

   vector<Vector3f> finite;
   finite.reserve(data.size());
   for(const auto& X : data)
      if(is_finite(X)) finite.push_back(X);

   if(finite.size() >= 3) {
      const auto udv = svd_3d_UDV(cbegin(finite), cend(finite));

      // cout << str(udv) << endl;
      const auto q = udv.rot_vec();
      for(auto& X : data) X = to_vec3f(q.apply(to_vec3(X)));
   }

   return {major_axis, minor_axis};
}

} // namespace perceive
