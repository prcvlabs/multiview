
#include "create-cv-remap.hpp"
#include "stdinc.hpp"

#include "perceive/utils/threads.hpp"
#include "perceive/utils/tick-tock.hpp"

namespace perceive
{
static void create_cv_remap_impl(const unsigned w,
                                 const unsigned h,
                                 const Matrix3r& H_in,
                                 std::function<Vector2(const Vector2& x)> f,
                                 const Matrix3r& H_out,
                                 cv::Mat& mapx,
                                 cv::Mat& mapy,
                                 ParallelJobSet* pjobs,
                                 const bool feedback) noexcept
{
   auto sz = cv::Size(int(w), int(h));
   Expects(sz.height == signed(h));
   Expects(sz.width == signed(w));

   auto setup_map = [&](cv::Mat& map) {
      if(map.cols != signed(w) or map.rows != signed(h)
         or map.type() != CV_32FC1) {
         map.create(sz, CV_32FC1);
      }
   };
   setup_map(mapx);
   setup_map(mapy);

   Matrix3r H_inv = H_out.inverse();

   auto process_xy = [&](unsigned x, unsigned y) -> Vector3r {
      Vector3r X_    = H_inv * Vector3r(x, y, 1.0);
      Vector3r X_hat = normalized_P2(X_);
      auto D_hat     = f(Vector2(X_hat(0), X_hat(1)));
      Vector3r D     = normalized_P2(H_in * Vector3r(D_hat(0), D_hat(1), 1.0));
      return D;
   };

   auto now       = tick();
   auto process_y = [&](unsigned y) {
      if(pjobs == nullptr and feedback and (y % 50 == 0))
         cout << format("remap :: y = {:3d}/{:3d} :: {}s", y, h, tock(now))
              << endl;
      float* m1f = mapx.ptr<float>(int(y));
      float* m2f = mapy.ptr<float>(int(y));
      for(unsigned x = 0; x < w; ++x) {
         Vector3r D = process_xy(x, y);
         *m1f++     = float(D(0));
         *m2f++     = float(D(1));
      }
   };

   auto s = time_thunk([&]() {
      if(pjobs != nullptr) {
         for(auto y = 0u; y < h; ++y)
            pjobs->schedule([&process_y, y]() { process_y(y); });
         pjobs->execute();
      } else {
         for(auto y = 0u; y < h; ++y) process_y(y);
      }
   });

   if(feedback) cout << format("create-cv-remap: {}s", s) << endl;
}

void create_cv_remap(const unsigned w,
                     const unsigned h,
                     const Matrix3r& H_in,
                     std::function<Vector2(const Vector2& x)> f,
                     const Matrix3r& H_out,
                     cv::Mat& mapx,
                     cv::Mat& mapy,
                     const bool feedback) noexcept
{
   create_cv_remap_impl(w, h, H_in, f, H_out, mapx, mapy, nullptr, feedback);
}

void create_cv_remap_threaded(const unsigned w,
                              const unsigned h,
                              const Matrix3r& H_in,
                              std::function<Vector2(const Vector2& x)> f,
                              const Matrix3r& H_out,
                              cv::Mat& mapx,
                              cv::Mat& mapy,
                              ParallelJobSet& job_set,
                              const bool feedback) noexcept
{
   create_cv_remap_impl(w, h, H_in, f, H_out, mapx, mapy, &job_set, feedback);
}

} // namespace perceive
