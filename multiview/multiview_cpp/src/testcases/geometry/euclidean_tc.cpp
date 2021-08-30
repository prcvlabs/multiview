
#include <algorithm>
#include <iterator>

#define CATCH_CONFIG_PREFIX_ALL
#include "perceive/calibration/aruco-cube.hpp"
#include "perceive/contrib/catch.hpp"
#include "perceive/geometry.hpp"
#include "perceive/geometry/projective/binocular-camera.hpp"

namespace perceive
{
static const string c1001_v6 = R"V0G0N(

{
   "camera-id": "C0001001_v6",
   "rotation-axis": [-5.289019071995259002605e-01, -8.037447439841696184004e-01, 2.725016680295973547921e-01],
   "rotation-angle": 4.403040718534967368214e-02,
   "t": [-9.999691706199903551422e-01, 4.682000672898050178117e-03, -6.303703615144396268932e-03],
   "baseline": 1.677118998537401595161e-01,
   "sensor0":               
              {
                 "model": "polynomial<8>",
                 "sensor-id": "STR00017",
                 "format": [2592, 1944],
                 "center": [1.259908344467880397133e+03, 9.543476067019000765868e+02],
                 "scale":  1.733503229994983732207e-03,
                 "calib-region": [82.4137, 125.912, 2499.89, 1843.35],
                 "A":
              [[4.081363182094257385790e-04, 7.846591271087714758448e-05, 1.117749802370410043550e-03, 2.050148110624722824769e-04, 8.499430114145657211636e-04, 1.202137033918700137480e-04, 2.657899442004324659650e-04, 2.557591866361383359241e-05, 1.478675890686819479475e-04, 3.551415648586637562739e-03, -4.607412044975141632297e-05, 1.061446051619520727916e-02, -4.051013608679160577820e-05, 9.430254182018919795194e-03, -7.433659454589812420888e-05, 2.520923581562218521862e-03, -8.578724808384115968485e-05, -2.794302257206254802568e-03, -1.184080115168654676050e-03, -4.807017326956552344397e-03, -1.889330041735894083477e-03, -1.854248730084105331595e-03, -5.923062148343941069051e-04, -6.226051325912054096312e-04, -7.918204510909552379383e-03, 2.807659980136358035541e-04, -1.386692371503723923698e-02, 3.144186716624482180737e-04, -3.137869973570229370496e-03, 3.464792809319654220968e-04, 7.406920122703386510921e-03, 1.918694069271629750253e-03, 6.451037704162667155150e-03, 8.917384701673108926556e-04, 9.915152431928843379527e-04, 6.928405896704332989078e-02, -1.756316883927931671305e-04, 6.449975726755774463328e-02, -4.511063214778510133129e-04, -2.036354542474170387090e-03, -5.293037764114348620037e-03, -1.087122720821221549814e-03, 5.137947864186872548586e-01, 5.065063338827674541825e-04, 4.882256460082610299844e-02],
               [2.415619570788307658108e-05, 3.020746602274122869641e-04, 1.374092341015142274691e-04, 6.174744303661985336470e-04, 1.455262958724064248361e-04, 2.937011790217409525072e-04, -3.602390364017271640051e-05, -2.319655276377142745330e-05, -6.724065902904974301180e-05, -1.045114328824338400040e-05, 3.487486800742341502141e-03, -4.192946074774118667139e-05, 9.147459014495772783304e-03, -6.302296973057191879775e-05, 7.339344583677403534294e-03, -3.002658437739483612838e-05, 1.818669534496500199416e-03, -2.348697602473663256784e-04, -1.735129306810551108597e-03, -1.336041998393913817975e-03, -1.639638486985247386313e-03, -1.056904463852648135003e-03, 9.235646859584206898319e-05, -1.235380936145946684235e-05, 6.549325746959058780483e-05, -6.470080080674704345323e-03, 2.144445755465525238481e-04, -4.865152943111726926984e-03, 9.600404768035815383787e-05, 1.985069646551262945167e-03, 7.123550072457365822665e-04, 3.971930067778821002444e-03, 1.223496889110331498074e-03, 1.469815532720685456736e-03, -5.141728148921054231124e-04, -2.007789192047726087309e-04, 6.315384976319464438443e-02, 1.232334347574595700969e-05, 5.335636347570283516406e-02, -7.757682209504875373018e-04, 1.701908258753077364533e-03, -4.529528852638423086496e-03, -4.330224473192770262564e-04, 5.188048489012443420521e-01, -1.066161721393027624061e-02]],
                 "calib-hull": [[82.4137, 994.589], [82.7395, 912.795], [85.3685, 871.576], [92.3784, 775.055], [111.127, 642.099], [138.231, 516.635], [171.755, 400.84], [216.257, 372.164], [271.933, 343.728], [286.076, 336.848], [368.694, 298.754], [446.104, 267.531], [463.093, 261.22], [486.837, 252.871], [557.389, 229.269], [576.681, 223.354], [605.074, 215.252], [687.066, 192.667], [709.383, 187.521], [741.772, 180.559], [805.271, 167.766], [835.322, 161.766], [860.417, 157.08], [896.734, 151.781], [999.599, 138.492], [1026.97, 135.686], [1065.98, 132.822], [1175.51, 126.613], [1203.93, 126.197], [1244.34, 125.912], [1354.78, 128.147], [1383.09, 129.904], [1422.63, 132.578], [1496.12, 139.68], [1529.49, 143.017], [1556.09, 146.504], [1593.15, 152.095], [1661.32, 163.546], [1691.41, 168.604], [1716.05, 173.134], [1749.25, 180.858], [1836.85, 201.752], [1858.53, 207.677], [1887.77, 216.351], [1915.64, 224.752], [1963.5, 239.446], [1982.11, 245.471], [2007.04, 254.468], [2052.54, 271.451], [2072.39, 278.928], [2087.93, 285], [2109.35, 293.832], [2147.74, 310.319], [2177.11, 323.306], [2212.51, 340.414], [2266.29, 367.373], [2282.88, 376.233], [2336.32, 405.799], [2342.31, 409.657], [2426.92, 466.272], [2454.03, 570.463], [2476.27, 680.971], [2491.85, 796.413], [2499.89, 915.432], [2499.46, 1035.58], [2491.39, 1154.54], [2475.95, 1269.77], [2453.58, 1379.92], [2426.41, 1482.96], [2383.01, 1512.72], [2348.33, 1535.02], [2326.94, 1547.95], [2285.49, 1572.44], [2147.68, 1649.75], [2059.28, 1687.99], [2036.64, 1696.75], [1956.44, 1726.57], [1913.45, 1741], [1838.17, 1763.23], [1788.82, 1776.6], [1704.06, 1796.15], [1648.84, 1806.85], [1555.2, 1822.09], [1495.53, 1829.43], [1395.3, 1838.39], [1332.84, 1841.68], [1229.52, 1843.35], [1191.18, 1842.53], [1166.66, 1842], [1064.71, 1836.18], [1027.51, 1832.73], [1003.98, 1830.47], [907.22, 1817.6], [872.222, 1811.98], [850.59, 1808.28], [761.806, 1790.37], [730.341, 1782.97], [710.577, 1778.24], [631.272, 1756.58], [603.563, 1748.4], [586.526, 1743.08], [517.243, 1719.7], [492.838, 1710.75], [478.08, 1705.22], [418.06, 1681.11], [384.471, 1666.8], [332.937, 1643.06], [315.234, 1634.4], [304.34, 1628.77], [245.08, 1598], [235.749, 1592.75], [177.24, 1558.86], [146.527, 1459], [120.765, 1351.49], [100.792, 1237.1], [95.3592, 1189.32], [87.6047, 1117.45], [83.8578, 1052.18], [82.4137, 994.589]]
              }
,
   "sensor1":               
              {
                 "model": "polynomial<8>",
                 "sensor-id": "STR00018",
                 "format": [2592, 1944],
                 "center": [1.365996599067576426023e+03, 9.619834715588560811739e+02],
                 "scale":  1.736291840401992837839e-03,
                 "calib-region": [78.7126, 124.979, 2550.85, 1835],
                 "A":
              [[-5.315756199690442607153e-04, 8.046465077178252545131e-05, -1.522758043595403253112e-03, 1.346582592538676792504e-04, -1.498953500720120665668e-03, -6.184453678833223486122e-05, -6.962423581377014808469e-04, -1.119249465862583864384e-04, -1.757216516818263345179e-04, 3.669268064188274322546e-03, 1.766078796994431307499e-04, 1.084797525230436418542e-02, 3.153399946020565142168e-04, 9.496429630240233940586e-03, 1.092412281521589964561e-04, 2.439028130512555393034e-03, 7.932339148422444530251e-06, 5.173378866164785039317e-03, -1.050818808285107486267e-03, 1.001481104868468668956e-02, -1.178458695038332726401e-03, 5.313466479842779421894e-03, -2.664260414071695226568e-06, 8.827648883154804934636e-04, -8.440530738109436326155e-03, -9.967795995024439359433e-04, -1.417627701385058489048e-02, -8.585297867090037338134e-04, -2.727156356488617816591e-03, -4.331716336005569933931e-05, -1.249821847767965121712e-02, 1.441495676987598895114e-03, -1.268309194468521944321e-02, 1.173228543019655401025e-04, -2.004245713858156599518e-03, 7.111781150188134503765e-02, 1.119815287340029941188e-03, 6.509604266012522510998e-02, -1.582743336962632446641e-04, 1.671795931208419627723e-02, -4.906944105397343519614e-03, 2.868963539565123899155e-03, 5.172421081249921614997e-01, -2.643700981644531849968e-03, 4.824947005114835207884e-02],
               [3.578825528978738738234e-05, -3.932581736850303960606e-04, 1.884504816094725171416e-04, -8.019180890467313485570e-04, 4.201937561761591444220e-04, -1.459995350831010466064e-04, 4.827778197111758790028e-04, 4.487328898503959231925e-04, 1.637110298065916347277e-04, 3.988925765715015635260e-05, 3.533145510380931650363e-03, 2.008976838215881244309e-04, 9.168158870775627855565e-03, 8.693744979788257248865e-05, 6.942543139443178162873e-03, -2.407835536954059871273e-04, 1.457883788077445844783e-03, -3.201801397215240513328e-04, 3.570788063624465158430e-03, -1.674870996303291781349e-03, 4.171110785729122832910e-03, -2.426937210629385999194e-03, -5.498491718434490166389e-04, -9.597464068950289917126e-04, -2.055548056429101766440e-04, -6.403146838894081122051e-03, -5.676372262405880719793e-04, -3.853991144828171344638e-03, 4.452330757886771572807e-04, 3.387757367917261681900e-03, 9.799256737675569178814e-04, -6.438375170346698206369e-03, 2.286510576990127829866e-03, -1.055892257477417106593e-03, 8.581349086410200444064e-04, 6.479658737773563714768e-04, 6.348403960312912208686e-02, 5.361297573553552853198e-04, 5.271980626052616414334e-02, -8.809615456876473960079e-04, 8.757857304262026132413e-03, -4.984164990529466737756e-03, 2.046508215556562884641e-03, 5.230369314085653309476e-01, -1.336270119324906371916e-02]],
                 "calib-hull": [[78.7126, 952.206], [83.8904, 844.077], [95.0449, 738.086], [112.615, 635.416], [135.392, 537.371], [162.807, 483.393], [188.535, 444.362], [237.205, 413.567], [252.003, 404.348], [312.761, 369.838], [313.887, 369.244], [363.366, 343.786], [385.316, 332.68], [386.505, 332.104], [471.656, 293.466], [576.639, 252.499], [659.697, 225.38], [699.404, 212.759], [796.943, 187.599], [843.354, 176.447], [927.762, 160.777], [955.203, 155.725], [1005.17, 147.377], [1131.6, 133.681], [1185.78, 129.132], [1319.61, 125.309], [1375.64, 124.979], [1378.18, 125.104], [1509.81, 131.899], [1566.71, 136.438], [1692.12, 152.882], [1742.75, 161.106], [1744.76, 161.437], [1858.47, 185.055], [1903.32, 195.688], [1905.22, 196.176], [2004.54, 224.377], [2044.8, 236.747], [2163.27, 279.414], [2262.98, 322.01], [2345.37, 362.694], [2384.42, 383.952], [2422.21, 405.414], [2443.01, 417.768], [2456.45, 426.214], [2477.64, 478.807], [2507.54, 589.064], [2526.71, 688.715], [2540.66, 793.045], [2549.13, 899.659], [2550.85, 1007.61], [2546.54, 1114.82], [2536.17, 1219.73], [2520.19, 1320.95], [2499.35, 1417.38], [2475.19, 1507.31], [2440.16, 1546.05], [2405.36, 1566.8], [2360.57, 1591.58], [2335.4, 1604.71], [2301.76, 1621.42], [2282.35, 1630.83], [2252.32, 1645.11], [2212.22, 1662.58], [2189.27, 1672.35], [2153.23, 1687.03], [2106.01, 1704.78], [2078.78, 1714.48], [2036.09, 1729.05], [1981.68, 1746.26], [1948.45, 1755.2], [1898.84, 1768.16], [1835.64, 1783.4], [1797.89, 1790.78], [1741.66, 1801.38], [1670.68, 1812.55], [1629.05, 1817.68], [1567.22, 1824.6], [1489.89, 1830.55], [1446.66, 1832.46], [1373.18, 1835], [1238.91, 1833.7], [1215.23, 1832.66], [1085.14, 1822.4], [1062.44, 1820.04], [940.302, 1802.12], [919.235, 1798.5], [808.059, 1774.77], [670.717, 1736.19], [586.083, 1706.27], [548.654, 1692.41], [496.151, 1670.69], [479.206, 1663.42], [463.843, 1656.66], [461.859, 1655.76], [419.036, 1636.25], [403.82, 1629.17], [388.657, 1621.52], [351.652, 1602.49], [326.098, 1588.83], [294.196, 1570.64], [273.319, 1558.46], [244.333, 1541.17], [200.765, 1513.25], [185.281, 1502.17], [148.646, 1469.08], [123.563, 1373.42], [103.197, 1272.74], [88.8354, 1168.51], [80.5573, 1060.87], [78.7126, 952.206]]
              }

}
)V0G0N";

CATCH_TEST_CASE("EuclideanTransform", "[euclidean_transform]")
{
   CATCH_SECTION("TestEuclideanTransform")
   {
      const auto aa1 = Vector4{0.1, 1.0, 1.0, to_radians(35.0)};
      const auto aa2 = Vector4{0.2, 0.4, 1.0, to_radians(15.0)};

      const auto e1 = EuclideanTransform(
          Vector3(0.1, 0.2, 1.0), axis_angle_to_quaternion(aa1), 6.0);
      const auto e2 = EuclideanTransform(
          Vector3{1.0, 1.1, 1.2}, axis_angle_to_quaternion(aa2), 2.0);

      auto test_it = [](const auto& et, const Vector3& X) {
         auto Y1 = et.apply(X);

         Matrix4r M = make_transform_matrix(et);
         Vector4r Z = M * Vector4r(X(0), X(1), X(2), 1.0);
         auto Y2    = Vector3(Z(0), Z(1), Z(2)) / Z(3);

         auto err = (Y2 - Y1).norm();

         if(false) {
            cout << "--------------------" << endl;
            cout << format("et = {:s}, X = {:s}", str(et), str(X)) << endl;
            cout << format("|{:s} - {:s}| = {}", str(Y1), str(Y2), err) << endl;
            cout << endl;
         }

         CATCH_REQUIRE(fabs(err) < 1e-9);
      };

      test_it(e1, Vector3{1.0, 0.0, 0.0});
      test_it(e1, Vector3{1.0, 1.0, 0.0});
      test_it(e1, Vector3{1.0, 1.0, 1.0});
      test_it(e2, Vector3{1.0, 0.0, 0.0});
      test_it(e2, Vector3{1.0, 1.0, 0.0});
      test_it(e2, Vector3{1.0, 1.0, 1.0});

      auto test_e12 = [&](const auto& e1, const auto& e2, const Vector3& X) {
         const auto et = compose(e1, e2);

         auto Y = e1.apply(X);
         auto Z = e2.apply(Y);

         auto W = et.apply(X);

         Matrix4r M1 = make_transform_matrix(e1);
         Matrix4r M2 = make_transform_matrix(e2);
         Matrix4r M  = M2 * M1;

         Vector4r U = M * Vector4r(X(0), X(1), X(2), 1.0);
         auto V     = Vector3(U(0), U(1), U(2)) / U(3);

         auto err  = (W - Z).norm();
         auto err1 = (W - V).norm();
         auto err2 = (V - Z).norm();

         if(false) {
            cout << "--------------------" << endl;
            cout << format("X = {:s}", str(X)) << endl;
            cout << format("e1 = {:s}, Y = {:s}", str(e1), str(Y)) << endl;
            cout << format("e2 = {:s}, Z = {:s}", str(e2), str(Z)) << endl;
            cout << format("et = {:s}, W = {:s}", str(et), str(W)) << endl;
            cout << format("|{:s} - {:s}| = {}", str(W), str(Z), err) << endl;
            cout << format("|{:s} - {:s}| = {}", str(W), str(V), err1) << endl;
            cout << format("|{:s} - {:s}| = {}", str(V), str(Z), err2) << endl;
            cout << endl;
         }

         CATCH_REQUIRE(fabs(err) < 1e-9);
         CATCH_REQUIRE(fabs(err1) < 1e-9);
         CATCH_REQUIRE(fabs(err2) < 1e-9);
      };

      test_e12(e1, e2, Vector3{1.0, 0.0, 0.0});
      test_e12(e1, e2, Vector3{1.0, 1.0, 0.0});
      test_e12(e1, e2, Vector3{1.0, 1.0, 1.0});
      test_e12(e1, e2, Vector3{1.0, 0.0, 1.0});

      auto test_i12 = [&](const auto& e1, const auto& e2, const Vector3& X) {
         const auto et = compose(e1, e2);
         const auto ez = et / e2; // ez should be the same as e1

         auto Y = e1.apply(X);
         auto Z = e2.apply(Y);
         auto V = e2.inverse_apply(Z); // Should be Y

         auto W = ez.apply(X); // Should be Y

         auto err1 = (Y - V).norm();
         auto err2 = (Y - W).norm();

         if(false) {
            cout << "--------------------" << endl;
            cout << format("X = {:s}", str(X)) << endl;
            cout << format("e1 = {:s}, Y = {:s}", str(e1), str(Y)) << endl;
            cout << format("e2 = {:s}, Z = {:s}", str(e2), str(Z)) << endl;
            cout << format("et = {:s}, W = {:s}", str(et), str(W)) << endl;
            cout << format("ez = {:s}, W = {:s}", str(et), str(W)) << endl;
            cout << format("|{:s} - {:s}| = {}", str(Y), str(V), err1) << endl;
            cout << format("|{:s} - {:s}| = {}", str(Y), str(W), err2) << endl;
            cout << endl;
         }

         CATCH_REQUIRE(fabs(err1) < 1e-9);
         CATCH_REQUIRE(fabs(err2) < 1e-9);
      };

      test_i12(e1, e2, Vector3{1.0, 0.0, 0.0});
      test_i12(e1, e2, Vector3{1.0, 1.0, 0.0});
      test_i12(e1, e2, Vector3{1.0, 1.0, 1.0});
      test_i12(e1, e2, Vector3{1.0, 0.0, 1.0});

      auto test_inverse = [&](const auto& e1, const Vector3& X) {
         const auto e_inv = EuclideanTransform{} / e1;
         auto A           = e1.apply(X);
         auto B           = e_inv.inverse_apply(X);
         auto err         = (A - B).norm();

         if(false) {
            cout << "--------------------" << endl;
            cout << format("X = {:s}", str(X)) << endl;
            cout << format("e1 = {:s}, Y = {:s}", str(e1), str(A)) << endl;
            cout << format("ei = {:s}, Z = {:s}", str(e_inv), str(B)) << endl;
            cout << format("|{:s} - {:s}| = {}", str(A), str(B), err) << endl;
            cout << endl;
         }

         CATCH_REQUIRE(fabs(err) < 1e-9);
      };

      test_inverse(e1, Vector3{1.0, 0.0, 0.0});
      test_inverse(e1, Vector3{1.0, 1.0, 0.0});
      test_inverse(e1, Vector3{1.0, 1.0, 1.0});
      test_inverse(e1, Vector3{1.0, 0.0, 1.0});
      test_inverse(e2, Vector3{1.0, 0.0, 0.0});
      test_inverse(e2, Vector3{1.0, 1.0, 0.0});
      test_inverse(e2, Vector3{1.0, 1.0, 1.0});
      test_inverse(e2, Vector3{1.0, 0.0, 1.0});
   }

   CATCH_SECTION("TestKabschAlgorithm")
   {
      const vector<Vector3> A{{{1.520167, 2.2585, 0.0641333},
                               {2.95617, 0.3275, 0.224667},
                               {1.13917, 0.1735, 0.0391333},
                               {1.13617, 0.6435, 0.0391333},
                               {3.11083, 0.5545, 0.0411333},
                               {2.64083, 0.5595, 0.0411333}}};
      const auto N = A.size();

      const Quaternion q
          = Quaternion::between_vectors(Vector3(0, 0, 1), Vector3(0.2, 1.8, 0));

      const Vector3 C0
          = std::accumulate(cbegin(A), cend(A), Vector3(0, 0, 0)) / real(N);
      const Vector3 C1 = Vector3(9.1, -2.0, -2.5);

      vector<Vector3> B(A.size());
      std::transform(cbegin(A), cend(A), begin(B), [&](const auto& X) {
         return q.rotate(X - C0) + C1;
      });

      const auto et = transform_between(A, B);

      for(auto i = 0u; i < N; ++i)
         CATCH_REQUIRE((et.apply(A[i]) - B[i]).norm() < 1e-9);
   }

   CATCH_SECTION("et-pack-unpack-6df")
   {
      std::mt19937 gen;
      std::uniform_real_distribution<double> distribution{0.0, 1.0};
      gen.seed(123456);

      auto rand = [&](real a, real b) -> real {
         return (b - a) * distribution(gen) + a;
      };

      auto rand_X = [&]() {
         return Vector3(
             rand(-100.0, 100.0), rand(-100.0, 100.0), rand(-100.0, 100.0));
      };

      auto rand_et = [&]() {
         EuclideanTransform et;
         et.scale = rand(0.5, 1.5);
         et.translation
             = Vector3(rand(-1.0, 1.0), rand(-1.0, 1.0), rand(-1.0, 1.0));
         et.rotation = saa_to_quaternion(
             Vector3(rand(-M_PI, M_PI), rand(-M_PI, M_PI), rand(-M_PI, M_PI)));
         return et;
      };

      auto test_it
          = [&](const EuclideanTransform& et0, const EuclideanTransform& et1) {
               const auto X   = rand_X();
               const auto Y   = et0.apply(X);
               const auto Z   = et1.apply(X);
               const auto err = (Y - Z).norm();
               // INFO(format("|{} - {}| = {}", str(Y), str(Z), err));
               // if(std::fabs(err) > 1e-3) FATAL("kAABM!");
               CATCH_REQUIRE(true);
            };

      auto test_et = [&](const EuclideanTransform& et0) {
         array<real, 6> Xs;
         pack_et_6df(et0, Xs.data());
         const auto et1 = unpack_et_6df(Xs.data());
         test_it(et0, et1); // should produce the same transformations
      };

      for(auto i = 0; i < 100; ++i) test_et(rand_et());
   }

   CATCH_SECTION("et_with_bcam")
   {
      vector<Vector3> Ws{{{3.0920, 3.4870, 0.7400},
                          {3.0920, 1.1370, 0.7400},
                          {2.0870, 1.1370, 0.7400},
                          {2.0870, 3.4870, 0.7400}}};

      // C1001_v6
      vector<Vector2> Ps{{{1167, 601}, {1907, 1063}, {1603, 1428}, {859, 738}}};
      vector<Vector2> Qs{{{1176, 642}, {1893, 1102}, {1555, 1469}, {868, 774}}};

      const auto N = Ws.size();
      Expects(N == Ps.size());
      Expects(N == Qs.size());

      // Load bcam_info
      BinocularCameraInfo bcam;
      read(bcam, c1001_v6);

      // Transform for Cam0
      EuclideanTransform et0, et1;
      et0.scale = 1.0;
      et0.translation
          = Vector3(1.376556240246096, 1.0544132990288464, 2.2821151846153271);
      et0.rotation = Quaternion(0.87261013236307849,
                                -0.31804190363459633,
                                0.1041066722243857,
                                -0.3557565252080937);

      et1 = bcam.make_et1(et0);

      const auto C0 = bcam.C0();
      const auto C1 = bcam.C1();

      // Check the sanity of inputs
      for(auto i = 0u; i < N; ++i) {
         const auto& W = Ws[i];
         const auto& p = Ps[i];
         const auto& q = Qs[i];

         const auto E0 = et0.inverse_apply(W); // eye co-ordinate
         const auto E1 = bcam.q.apply(E0 - C1);
         const auto E_ = et0.rotation.inverse_rotate(W - et0.translation);
         const auto Q_ = et1.inverse_apply(W);
         const auto D0 = bcam.M[0].distort(homgen_P2_to_R2(E0));
         const auto D1 = bcam.M[1].distort(homgen_P2_to_R2(E1));
         const auto e0 = (D0 - Ps[i]).norm();
         const auto e1 = (D1 - Qs[i]).norm();

         CATCH_REQUIRE((E0 - E_).norm() < 1e-9);
         CATCH_REQUIRE((E1 - Q_).norm() < 1e-9);

         if(false) {
            cout << string(80, '-') << endl;
            cout << format("W  = {:s}", str(W)) << endl;
            cout << format("E_ = {:s}", str(E_)) << endl;
            cout << format("E0 = {:s}", str(E0)) << endl;
            cout << format("E1 = {:s}", str(E1)) << endl;
            cout << format("Q_ = {:s}", str(Q_)) << endl;
            cout << format("D0 = {:s}", str(D0)) << endl;
            cout << format("D1 = {:s}", str(D1)) << endl;
            cout << format("e0 = {}", e0) << endl;
            cout << format("e1 = {}", e1) << endl;
            cout << endl;
         }
      }

      // CATCH_REQUIRE(true);
   }

   CATCH_SECTION("euclidean-transform-plane")
   {
      std::mt19937 gen;
      std::uniform_real_distribution<double> distribution{0.0, 1.0};
      gen.seed(123456);

      auto rand = [&](real a, real b) -> real {
         return (b - a) * distribution(gen) + a;
      };

      auto test_it = [&]() {
         // A random transform
         EuclideanTransform et;
         et.scale = rand(0.5, 1.5);
         et.translation
             = Vector3(rand(-1.0, 1.0), rand(-1.0, 1.0), rand(-1.0, 1.0));
         et.rotation = saa_to_quaternion(
             Vector3(rand(-M_PI, M_PI), rand(-M_PI, M_PI), rand(-M_PI, M_PI)));

         // A random plane
         Plane p3 = Plane(
             spherical_to_cartesian(rand(-M_PI, M_PI), rand(-M_PI, M_PI), 1.0),
             rand(-10.0, 10.0));

         // Some points on the plane
         array<Vector3, 10> Xs;
         for(auto& X : Xs)
            X = p3.image(Vector3(
                rand(-100.0, 100.0), rand(-100.0, 100.0), rand(-100.0, 100.0)));

         Plane t3 = et.apply_to_plane(p3); // transformed plane

         // Test plane equation transformation here
         for(auto i = 0; i < 100; ++i) {
            // Generate a random point
            const Vector3 X = p3.image(Vector3(
                rand(-100.0, 100.0), rand(-100.0, 100.0), rand(-100.0, 100.0)));

            // The transformed point
            const Vector3 Y = et.apply(X);

            // X must be on the random plane
            CATCH_REQUIRE(std::fabs(p3.side(X)) < 1e-9);

            // The transformed point (Y) must be on the transformed plane
            CATCH_REQUIRE(std::fabs(t3.side(Y)) < 1e-9);
         }
      };

      for(auto i = 0; i < 100; ++i) test_it();
   }

   CATCH_SECTION("euclidean-math-health")
   {
      const ArucoCube ac = make_kyle_aruco_cube();
      Matrix3r K         = Matrix3r::Identity();
      const unsigned uh  = 1944;
      const unsigned uw  = 2592;
      const real vfov    = 70.0; // vertical field of view
      K(0, 0) = K(1, 1)     = 0.5 * real(uh) / tan(0.5 * to_radians(vfov));
      K(0, 2)               = uw * 0.5;
      K(1, 2)               = uh * 0.5;
      const Matrix3r K_inv  = K.inverse();
      array<real, 7> ets_p0 = {
          1.796946, 0.519288, 2.404663, 0.505111, 0.328718, 3.442211, 1.000000};
      array<real, 7> ets_p1 = {
          1.805852, 0.527716, 2.401060, 0.453412, 0.374650, 3.472033, 1.000000};
      array<EuclideanTransform, 2> ets;
      ets[0].unpack(&ets_p0[0]);
      ets[1].unpack(&ets_p1[0]);
      const EuclideanTransform e01 = ets[0].inverse() * ets[1];

      BinocularCameraInfo bcam_info;
      bcam_info.set_from_et(e01.inverse());
      const auto C0  = bcam_info.C0();
      const auto C1  = bcam_info.C1();
      const auto q   = bcam_info.q;
      const auto bt1 = bcam_info.make_et1(ets[0].inverse()).inverse();

      CachingUndistortInverse cu0, cu1;
      cu0.init(K);
      cu1.init(K);

      {
         const auto i    = 4; // TOP face
         const Plane p3  = ac.measures[i].p3;
         const Plane p3_ = ets[0].apply_to_plane(ac.measures[i].p3);
         const Plane p31 = ets[1].apply_to_plane(ac.measures[i].p3);
         for(auto j = 0; j < 4; ++j) {
            EuclideanTransform et;
            const auto M  = ac.measures[i].Xs[size_t(j)];
            const auto X  = ets[0].apply(M);
            const auto Y  = ets[1].apply(M);
            const auto Y_ = bt1.apply(M);
            const auto Z  = e01.apply(X);
            const auto a  = bcam_info.to_ray(0, X);
            const auto b  = bcam_info.to_ray(1, X);
            const auto A  = plane_ray_intersection(p3_, C0, C0 + a);
            const auto B
                = plane_ray_intersection(p3_, C1, C1 + q.inverse_apply(b));

            const auto x0
                = homgen_P2_to_R2(to_vec3(K * Vector3r(a.x, a.y, a.z)));
            const auto x1 = homgen_P2_to_R2(to_vec3(K * to_vec3r(X)));
            const auto y0
                = homgen_P2_to_R2(to_vec3(K * Vector3r(b.x, b.y, b.z)));
            const auto y1
                = homgen_P2_to_R2(to_vec3(K * to_vec3r(Y))); // Correct

            const auto zx = transfer_point_between_images(
                y1, p31, ets[1], ets[0], cu1, cu0);
            const auto zy = transfer_point_between_images(
                x1, p3_, ets[0], ets[1], cu0, cu1);

            auto make_M0 = [&]() { // Take x, and convert to M
               auto ray
                   = to_vec3(K_inv * Vector3r(x0.x, x0.y, 1.0)).normalised();
               return ets[0].inverse_apply(
                   plane_ray_intersection(p3_, C0, C0 + ray));
            };
            const auto M0 = make_M0();

            auto make_M1 = [&]() { // Take x, and convert to M
               auto ray
                   = to_vec3(K_inv * Vector3r(y1.x, y1.y, 1.0)).normalised();
               ray = q.inverse_apply(ray);
               return ets[0].inverse_apply(
                   plane_ray_intersection(p3_, C1, C1 + ray));
            };
            const auto M1 = make_M1();

            if(false) {
               cout << format("M  = {:s}", str(M)) << endl;
               cout << format("M0 = {:s}", str(M0)) << endl;
               cout << format("M1 = {:s}", str(M1)) << endl;
               cout << format("X  = {:s}", str(X)) << endl;
               cout << format("A  = {:s}", str(A)) << endl;
               cout << format("B  = {:s}", str(B)) << endl;
               cout << format("Y  = {:s}", str(Y)) << endl;
               cout << format("Z  = {:s}", str(Z)) << endl;
               cout << format("x0 = {:s}", str(x0)) << endl;
               cout << format("x1 = {:s}", str(x0)) << endl;
               cout << format("zx = {:s}", str(zx)) << endl;
               cout << format("y0 = {:s}", str(y0)) << endl;
               cout << format("y1 = {:s}", str(y1)) << endl;
               cout << format("zy = {:s}", str(zy)) << endl;

               cout << endl;
            }

            CATCH_REQUIRE((M - M0).norm() < 1e-9);
            CATCH_REQUIRE((M - M1).norm() < 1e-9);
            CATCH_REQUIRE((Y - Z).norm() < 1e-9);
            CATCH_REQUIRE((Y - Y_).norm() < 1e-9);
            CATCH_REQUIRE((A - X).norm() < 1e-9);
            CATCH_REQUIRE((B - X).norm() < 1e-9);
            CATCH_REQUIRE((x1 - x0).norm() < 1e-9);
            CATCH_REQUIRE((y1 - y0).norm() < 1e-9);
            CATCH_REQUIRE((zx - x0).norm() < 1e-9);
            CATCH_REQUIRE((zy - y0).norm() < 1e-9);
         }
      }
   }
}

} // namespace perceive
