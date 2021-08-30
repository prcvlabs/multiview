
#include "smooth-point-cloud.hpp"

#include "perceive/utils/file-system.hpp"

namespace perceive
{
static const string exec = "/home/zeus/Dropbox/TMP/pcl/a.out"s;

// ----------------------------------------------------------- smooth pointcloud
//
PointCloud smooth_pointcloud(const PointCloud& in_cloud,
                             const SmoothPointCloudParams& params) noexcept
{
   { // sanity check
      if(!is_regular_file(exec)) {
         LOG_ERR(format(
             "smoothing executable '{}' not found; skipping smoothing step.",
             exec));
         return in_cloud;
      }
   }

   vector<char> buffer;
   float* raw_ptr = nullptr;
   bool has_error = false;

   { // set up buffer
      buffer.resize(in_cloud.N() * sizeof(float) * 3);
      raw_ptr = reinterpret_cast<float*>(&buffer[0]);
   }

   { // Save pointcloud to buffer
      float* cursor = raw_ptr;
      for(const auto& X : in_cloud.Xs) {
         *cursor++ = float(X.x);
         *cursor++ = float(X.y);
         *cursor++ = float(X.z);
      }
   }

   {
      string tmpd = ""s;
      try { // Make call
         tmpd = make_temp_directory("/tmp/smooth-ptXXXXXX");

         const string in_fname  = format("{}/in.data", tmpd);
         const string out_fname = format("{}/out.data", tmpd);
         const string cmd       = format("{} -i {} -o {} --polynomial-order "
                                   "{} --search_radius {}",
                                   exec,
                                   in_fname,
                                   out_fname,
                                   params.polynomial_order,
                                   params.search_radius);

         file_put_contents(in_fname, buffer);
         int ret = std::system(cmd.c_str());
         if(ret == 0) {
            file_get_contents(out_fname, buffer);
         } else {
            LOG_ERR(format("failed to run '{}'", cmd));
            has_error = true;
         }
      } catch(std::exception& e) {
         LOG_ERR(format("smooth point-cloud failed: {}", e.what()));
         has_error = true;
      }

      try {
         if(!tmpd.empty()) remove_all(tmpd);
      } catch(std::exception& e) {
         LOG_ERR(format("failed to delete tmp directory: '{}'", tmpd));
      }
   }

   PointCloud o;

   if(!has_error) {
      const size_t N = buffer.size() / (4 * sizeof(float));

      // INFO(format("HERE, N={}, sz = {}", N, buffer.size()));

      o.Xs.resize(N);
      o.xy.resize(N);
      o.lookup.resize(in_cloud.lookup.width, in_cloud.lookup.height);
      o.lookup.fill(-1);
      o.C = Vector3(0.0, 0.0, 0.0);

      //      INFO("K");
      float* cursor = reinterpret_cast<float*>(&buffer[0]);
      for(size_t pos = 0; pos < N; ++pos) {
         auto& X        = o.Xs[pos];
         X.x            = real(*cursor++);
         X.y            = real(*cursor++);
         X.z            = real(*cursor++);
         uint32_t label = *reinterpret_cast<uint32_t*>(cursor++);

         Expects(label < in_cloud.N());
         // INFO(format("label = {}/{}", label, in_cloud.N()));
         o.xy[pos] = in_cloud.xy[label];
         // INFO(format("xy = {}, lookup = [{}, {}]",
         //             str(o.xy[pos]),
         //             o.lookup.width,
         //             o.lookup.height));
         o.lookup(o.xy[pos]) = int(pos);
         // INFO("K");

         o.C += X;
      }
      // INFO("K");

      o.C /= real(N);
   } else {
      o = in_cloud;
   }

   return o;
}

} // namespace perceive
