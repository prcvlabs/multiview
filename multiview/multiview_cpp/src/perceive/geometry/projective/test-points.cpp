
#include "stdinc.hpp"
#include "test-points.hpp"

#include "perceive/graphics/colour-set.hpp"
#include "perceive/graphics/image-container.hpp"

namespace perceive
{
void test_distorted_bino_pair(const BinocularCameraInfo& bcam_info,
                              const BinocularCamera& bcam,
                              const cv::Mat in_distorted[2],
                              const cv::Mat in_rectified[2],
                              const vector<Vector2>& d0s,
                              const vector<Vector2>& d1s)
{
   Expects(d0s.size() == d1s.size());
   const auto N = d0s.size();

   // Distroted w and h
   auto distorted_w = in_distorted[0].cols;
   auto distorted_h = in_distorted[0].rows;
   auto scale       = in_distorted[0].cols / real(in_rectified[0].cols);

   // What is the rectification
   array<Quaternion, 2> Qs;
   array<Matrix3r, 2> Rs;
   {
      auto rect_q = bcam.rectify_rotation();
      auto cam1_q = bcam.q(); // CAM1 world to cam.
      Qs[0]       = rect_q;   // rectification for cam0
      Qs[1]       = rect_q * cam1_q.inverse();
      for(size_t i = 0; i < 2; ++i) Rs[i] = quaternion_to_rot3x3(Qs[i]);
   }

   // The rectification intrinsic parameters
   array<Matrix3r, 2> Ks;
   for(size_t i = 0; i < 2; ++i) Ks[i] = bcam.Kn(int(i));
   array<Matrix3r, 2> KRs;
   for(size_t i = 0; i < 2; ++i) KRs[i] = Ks[i] * Rs[i];

   // INFO("KRs");
   // for(auto i = 0; i < 2; ++i) {
   //    cout << "KR = " << endl << KRs[i] << endl << endl;
   //    cout << "__ = " << endl << bcam.KR(i) << endl << endl;
   // }

   cout << bcam_info.to_string() << endl;

   // Extract M
   array<DistortionModel, 2> M;
   for(size_t i = 0; i < 2; ++i) {
      M[i] = bcam.model(int(i));
      M[i].set_working_format(unsigned(distorted_w), unsigned(distorted_h));
      M[i].finalize();
   }

   array<ARGBImage, 2> im_d; // distorted
   array<ARGBImage, 2> im_r; // rectified
   for(size_t i = 0; i < 2; ++i) {
      im_d[i] = cv_to_argb(in_distorted[i]);
      im_r[i] = cv_to_argb(in_rectified[i]);
   }

   // Copy distorted
   array<vector<Vector2>, 2> d01s;
   {
      d01s[0] = d0s;
      d01s[1] = d1s;
      Ensures(d01s[0].size() == d01s[1].size());
   }

   // Calculate undistorted coords
   array<vector<Vector2>, 2> u01s;
   for(auto cam = 0u; cam < 2; ++cam) {
      std::transform(begin(d01s[cam]),
                     end(d01s[cam]),
                     back_inserter(u01s[cam]),
                     [&](auto D) { return M[cam].undistort(D); });
      Ensures(d01s[cam].size() == u01s[cam].size());
   }

   // Calculate rectified coords
   array<vector<Vector2>, 2> r01s;
   for(auto cam = 0u; cam < 2; ++cam) {
      std::transform(begin(u01s[cam]),
                     end(u01s[cam]),
                     back_inserter(r01s[cam]),
                     [&](auto& U) {
                        Vector3r r = KRs[cam] * Vector3r(U.x, U.y, 1.0);
                        return homgen_P2_to_R2(to_vec3(r));
                     });
      Ensures(r01s[cam].size() == u01s[cam].size());
   }

   // Calcualte recitifed coords using mapxy
   cv::Mat mapx[2];
   cv::Mat mapy[2];
   bcam.get_mapxy(mapx, mapy);
   array<vector<Vector2>, 2> s01s;
   LOG_ERR(format("[{}x{}]", mapx[0].cols, mapx[0].rows));
   LOG_ERR(format("[{}x{}]", mapy[0].cols, mapy[0].rows));
   LOG_ERR(format("[{}x{}]", mapx[1].cols, mapx[1].rows));
   LOG_ERR(format("[{}x{}]", mapy[1].cols, mapy[1].rows));

   for(size_t i = 0; i < 2; ++i) {
      LOG_ERR(format("M  = {}", str(M[i].working_format())));
      LOG_ERR(format("M  = {}", str(bcam.cu(int(i)).working_format())));
      LOG_ERR(format("cu = {}", str(bcam.model(int(i)).working_format())));
   }

   for(auto cam = 0u; cam < 2; ++cam) {
      std::transform(begin(d01s[cam]),
                     end(d01s[cam]),
                     back_inserter(s01s[cam]),
                     [&](auto& Dv) {
                        Point2 D  = to_pt2(Dv / scale);
                        Vector2 R = Vector2::nan();
                        if(D.x < 0 or D.x >= mapx[cam].cols) return R;
                        if(D.y < 0 or D.y >= mapx[cam].rows) return R;
                        if(D.x < 0 or D.x >= mapy[cam].cols) return R;
                        if(D.y < 0 or D.y >= mapy[cam].rows) return R;
                        R.x = real(mapx[cam].at<float>(D.y, D.x));
                        R.y = real(mapy[cam].at<float>(D.y, D.x));
                        return R;
                     });
      Ensures(d01s[cam].size() == s01s[cam].size());
   }

   // Draw crosses onto distorted
   for(auto cam = 0u; cam < 2; ++cam)
      for(auto i = 0u; i < N; ++i)
         draw_cross(im_d[cam], to_pt2(d01s[cam][i]), k_red, 5);

   // Draw crosses onto rectified
   for(auto cam = 0u; cam < 2; ++cam)
      for(auto& x : r01s[cam]) draw_cross(im_r[cam], to_pt2(x), k_red, 5);

   // Draw crosses onto rectified
   for(auto cam = 0u; cam < 2; ++cam)
      for(auto& x : s01s[cam]) draw_cross(im_r[cam], to_pt2(x), k_yellow, 5);

   // Print some information
   for(auto i = 0u; i < N; ++i) {
      const auto& d0 = d01s[0][i];
      const auto& d1 = d01s[1][i];
      const auto& u0 = u01s[0][i];
      const auto& u1 = u01s[1][i];
      const auto& r0 = r01s[0][i];
      const auto& r1 = r01s[1][i];

      auto delta_y = r01s[0][i].y - r01s[1][i].y;

      auto X = bcam.solve3d_from_distorted(d0, d1);
      auto Y = bcam_info.solve3d_from_distorted(d0, d1);
      cout << format("\u0394 = {} * [{}, {}] => [{}, {}] => [{}, "
                     "{}] => ({} :: {}) ({} :: {})",
                     fabs(delta_y),
                     str(d0),
                     str(d1),
                     str(u0),
                     str(u1),
                     str(r0),
                     str(r1),
                     str(X),
                     X.norm(),
                     str(Y),
                     Y.norm())
           << endl;
   }

   for(size_t i = 0; i < 2; ++i) {
      im_d[i].save(format("/tmp/zzz_distorted_{}.png", i));
      im_r[i].save(format("/tmp/zzz_rectified_{}.png", i));
   }

   // FATAL("kBAM!");
}

void test_distorted_bino_pair(const BinocularCameraInfo& bcam_info,
                              const BinocularCamera& bcam,
                              const cv::Mat in_distorted[2],
                              const cv::Mat in_rectified[2],
                              const vector<std::pair<Vector2, Vector2>>& d01)
{
   vector<Vector2> d0, d1;

   std::transform(begin(d01), end(d01), back_inserter(d0), [](auto& a) {
      return a.first;
   });
   std::transform(begin(d01), end(d01), back_inserter(d1), [](auto& b) {
      return b.second;
   });

   test_distorted_bino_pair(
       bcam_info, bcam, in_distorted, in_rectified, d0, d1);
}

} // namespace perceive
