
#include "perceive/cost-functions/features-2d/features-2d.hpp"
#include "point-cloud.hpp"
#include "projective/binocular-camera.hpp"

#define This PointCloud

namespace perceive
{
void This::init(const cv::Mat& disparity,
                const bool left_is_reference,
                const BinocularCamera& bino_cam)
{
   const unsigned w = unsigned(disparity.cols);
   const unsigned h = unsigned(disparity.rows);

   Xs.clear();
   xy.clear();

   C = Vector3(0, 0, 0);

   lookup.resize(w, h);

   Vector3 X;

   auto process_row = [&](const unsigned y) {
      auto row_ptr = reinterpret_cast<const float*>(disparity.ptr(int(y)));
      auto int_ptr = lookup.ptr(y);
      for(unsigned x = 0; x < w; ++x) {
         int_ptr[x] = -1; // i.e., no label
         auto dx    = row_ptr[x];

         if(false and (Vector2(x, y) - Vector2(235, 205)).norm() < 1.1) {
            auto x1 = double(x);
            auto x2 = double(x) - double(dx);
            X       = bino_cam.solve3d(x1, x2, double(y));
            INFO(format("bcam = {}, xxy = [{}, {}, {}] => X = {}",
                        bino_cam.camera_id(),
                        x1,
                        x2,
                        y,
                        str(X)));
         }

         if(!std::isfinite(dx)) continue;
         if(dx == 0.0f) continue;

         if(left_is_reference) {
            auto x1 = double(x);
            auto x2 = double(x) - double(dx);
            X       = bino_cam.solve3d(x1, x2, double(y));
         } else {
            // TODO, check this
            auto x1 = double(x) + double(dx);
            auto x2 = double(x);
            X       = bino_cam.solve3d(x1, x2, double(y));
         }

         C += X;

         int_ptr[x] = int(Xs.size());
         xy.push_back(Point2(int(x), int(y)));
         Xs.push_back(X);
      }
   };

   for(unsigned y = 0; y < h; ++y) process_row(y);

   C /= double(Xs.size());
   // labels.resize(Xs.size());
   // std::fill(labels.begin(), labels.end(), k_label_out_of_range);
}

// ---------------------------------------------------------------- memory-usage
//
size_t This::memory_usage() const noexcept
{
   return sizeof(This) + lookup.memory_usage()
          + (sizeof(Vector3) * Xs.capacity())
          + (sizeof(Point2) * xy.capacity());
}

// ------------------------------------------------------------ make-depth-image
//
GreyImage make_depth_image(const PointCloud& pt) noexcept(false)
{
   FloatImage g;
   g.resize(pt.lookup.width, pt.lookup.height);

   for(auto y = 0u; y < g.height; ++y) {
      for(auto x = 0u; x < g.width; ++x) {
         const int ind = pt.lookup(x, y);
         if(ind == -1) {
            g(x, y) = fNAN;
         } else {
            g(x, y) = float(pt.Xs[size_t(ind)].norm());
         }
      }
   }

   return float_im_to_grey(g);
}

ARGBImage make_depth_heatmap(const PointCloud& pt) noexcept(false)
{
   FloatImage g;
   g.resize(pt.lookup.width, pt.lookup.height);

   for(auto y = 0u; y < g.height; ++y) {
      for(auto x = 0u; x < g.width; ++x) {
         const int ind = pt.lookup(x, y);
         if(ind == -1) {
            g(x, y) = fNAN;
         } else {
            g(x, y) = float(pt.Xs[size_t(ind)].norm());
         }
      }
   }

   return float_im_to_argb(g);
}

} // namespace perceive
