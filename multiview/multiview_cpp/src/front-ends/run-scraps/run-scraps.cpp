
#include "run-scraps-inc.hpp"
#include "stdinc.hpp"

#include "perceive/foundation.hpp"
#include "perceive/geometry.hpp"
#include "perceive/geometry/projective/binocular-camera.hpp"
#include "perceive/geometry/projective/triangulation.hpp"
#include "perceive/io/perceive-assets.hpp"
#include "perceive/scene/aruco-result-info.hpp"
#include "perceive/utils/create-cv-remap.hpp"
#include "perceive/utils/file-system.hpp"
#include "perceive/utils/math.hpp"

#include "perceive/optimization/levenberg-marquardt.hpp"
#include "perceive/optimization/nelder-mead.hpp"

#include "perceive/calibration/camera-extrinsic.hpp"

namespace perceive::run_scraps
{
static const string k_dmodel_str = R"V0G0N(
{
   "model": "polynomial<8>",
   "sensor-id": "debug-sensor",
   "format": [2592, 1944],
   "center": [1.270465286109240651058e+03, 9.044933392516943513328e+02],
   "scale":  2.089579639688679459780e-03,
   "calib-region": [161.497, 147.487, 2476.07, 1815.58],
   "A":
[[6.693961037584688841817e-06, -1.798717425666003445935e-05, 2.678064042140884063598e-06, -4.230369989887007966048e-05, 1.046475530620599378451e-05, -3.979807582729682135722e-05, 2.097894086810020105628e-05, -2.632946978106285793042e-05, 1.720026909017623260173e-05, 3.990524206968744727608e-05, 1.045129808649671477195e-05, 9.335525387624949399112e-05, 2.509864060731576557162e-05, 8.968666648340704612014e-05, -6.602098759442764572269e-06, 6.272634380700742220277e-05, -1.606810598695217499540e-05, -1.089135851169058060404e-04, 2.460934052573062998451e-04, -9.664117108435280566192e-05, 3.722761180032809185531e-04, -1.585450611196327456998e-04, 2.284341813395098300110e-04, -1.287423833746454467780e-04, -4.161096841520791830837e-04, -1.550030271104659105458e-04, -7.187909681122059283101e-04, -8.799618925823182377655e-05, -4.760775963542083002999e-04, 8.406143274568804576660e-05, 5.093626470035227723349e-04, -9.957193510193862451274e-04, 5.418940177880646538733e-04, -8.233735041470180426870e-04, 3.514931001964900249279e-04, 1.680985540027847384426e-03, 5.567889969193072063208e-04, 1.650983683773925465854e-03, -1.082540646521767437049e-04, 1.831057139241867902715e-03, -1.301066549066687977332e-03, -4.516866869628361966238e-04, 3.545373609867331632373e-01, -1.956792191505688210257e-03, -1.002852246094259984488e-02],
 [-3.478597812631276011558e-06, 6.892152958340561965669e-06, -1.959567950023687786998e-05, 2.745433014203556114363e-06, -5.651755592730693661526e-05, 1.459431476532176098423e-05, -5.840371390853613163888e-05, 2.128303599532761837304e-05, -4.349095715962996225312e-06, 4.564595964492252841427e-06, 2.499414140213880261045e-05, 5.611435194571796946164e-06, 1.075567510017788836005e-04, 2.117825283853391575561e-05, 1.604468504366033429053e-04, -3.223719106604239215130e-06, 1.146574741495718155471e-04, 5.057252231636656615230e-05, -9.831523749136432573092e-05, 3.021537281635088651940e-04, -1.033331914972810804815e-04, 4.839675727495613977158e-04, -1.703950433212238300268e-04, 8.976795523501363005536e-05, -4.365098207012666755786e-05, -3.149518238261968770819e-04, -1.007227277972041513954e-04, -1.018879130293869322149e-03, -7.240808636577774487497e-05, -9.097439569349403259979e-04, -2.591185247318381892553e-04, 4.344298890999428769573e-04, -1.257735653707170919366e-03, 5.357492025587300664424e-04, -5.547262950531088587791e-04, 1.149388180365876633432e-04, 1.693481953589478367084e-03, 3.989642703526276247723e-04, 2.690977333319816838797e-03, 5.290231284106818206159e-04, 1.930647191342105967182e-03, -1.384037915223037401580e-03, 2.019327148612831690500e-03, 3.539743190128664118710e-01, -2.311312211501305100003e-02]],
   "calib-hull": [[161.497, 1813.95], [161.554, 1559.89], [161.63, 1433.3], [162.311, 1306.32], [163.842, 1053.09], [164.655, 927.736], [166.441, 802.364], [168.903, 677.43], [178.532, 287.632], [183.369, 152.546], [343.628, 149.087], [482.388, 148.127], [621.372, 147.487], [1448.5, 150.359], [1988.49, 153.715], [2120.74, 155.966], [2251.58, 158.199], [2336.42, 159.783], [2427.12, 177.966], [2445.45, 274.778], [2449.52, 397.676], [2457.53, 647.061], [2461.07, 772.458], [2466.96, 1025.17], [2472.04, 1279.39], [2473.85, 1406.65], [2476.07, 1766.82], [2358.86, 1771.7], [2240.38, 1776.57], [2120.31, 1780.56], [1999.39, 1784.34], [1876.81, 1787.64], [1753.82, 1790.74], [1002.29, 1809.63], [875.085, 1811.93], [773.285, 1813.3], [699.519, 1813.95], [623.572, 1814.33], [433.582, 1815.19], [285.847, 1815.58], [161.497, 1813.95]]
}
)V0G0N";

static DistortionModel load_dmodel(int w, int h) noexcept
{
   DistortionModel dmodel;
   read(dmodel, k_dmodel_str);
   dmodel.set_working_format(unsigned(w), unsigned(h));
   return dmodel;
}

void test_remap()
{
   INFO("TESTING REMAP");

   const int w = 360, h = 240;
   // const int w = 2592, h = 1944;
   const DistortionModel dmodel = load_dmodel(w, h);

   const auto D0 = Vector2(w * 0.5, h * 0.5);
   const auto U0 = dmodel.undistort(D0);
   const auto D1 = dmodel.distort(U0);

   const auto now = tick();
   CachingUndistortInverse cu(dmodel);
   cu.set_working_format(w, h);
   const auto s = tock(now);

   if(true) {
      INFO("REPORT");
      cout << format("D0 = {{}, {}}", D0.x, D0.y) << endl;
      cout << format("U0 = {{}, {}}", U0.x, U0.y) << endl;
      cout << format("D1 = {{}, {}}", D1.x, D1.y) << endl;

      INFO(format("cu-load took {}s", s));
      cout << format("cu.fmt = {:s}", str(cu.working_format())) << endl;
      cout << format("|D0 - D1| :: {:s}", vec_feedback(D0, D1)) << endl;
      cout << format("|U0 - uu| :: {:s}\n", vec_feedback(U0, cu.undistort(D0)));
      cout << format("|D1 - dd| :: {:s}\n", vec_feedback(D1, cu.distort(U0)));
   }

   Expects((D0 - D1).norm() < 1e-6);
   Expects((U0 - cu.undistort(D0)).norm() < 1e-6);
   Expects((D1 - cu.distort(U0)).norm() < 1e-6);

   ParallelJobSet pjobs;
   cv::Mat mapx, mapy;
   static const Matrix3r H = Matrix3r::Identity();
   Matrix3r K              = Matrix3r::Identity();
   const real hfov         = to_radians(80.0);
   K(0, 0) = K(1, 1) = 0.5 * real(h) / atan(0.5 * hfov);
   K(0, 2)           = w * 0.5;
   K(1, 2)           = h * 0.5;

   auto f = [&](const Vector2& x) -> Vector2 { return cu.distort(x); };
   create_cv_remap_threaded(w, h, H, f, K, mapx, mapy, pjobs, true);
}

// ----------------------------------------------------------------------- brief

string brief() noexcept
{
   return "A front end for running niggly bits of code.";
}

// -------------------------------------------------------------------- run-main
//
int run_main(int argc, char** argv)
{
   INFO("HELLO!!! Going to run some code...");

   // [1] is remap doing the same thing
   // [2] is undistort doing the same thing

   test_remap();

   bool success = true;
   return success ? EXIT_SUCCESS : EXIT_FAILURE;
}

} // namespace perceive::run_scraps
