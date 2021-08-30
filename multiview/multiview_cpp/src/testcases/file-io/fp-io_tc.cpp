
#include <algorithm>
#include <iterator>

#define CATCH_CONFIG_PREFIX_ALL
#include "perceive/contrib/catch.hpp"
#include "perceive/io/fp-io.hpp"
#include "perceive/utils/file-system.hpp"

namespace perceive
{
template<typename T> void test_save_load_T(vector<char>& buffer, T x)
{
   T y{};
   FILE* fp = nullptr;

   fp = fmemopen(&buffer[0], buffer.size(), "w");
   save_fp(fp, x);
   fclose(fp);

   // if constexpr(std::is_same<T, int32_t>::value) {
   //    INFO(format("INTO REPORT: {}", x));
   //    cout << str(&buffer[0], 5) << endl << endl;
   // }

   fp = fmemopen(&buffer[0], buffer.size(), "r");
   load_fp(fp, y);
   fclose(fp);

   if constexpr(std::is_floating_point<T>::value) {
      if(std::isnan(x)) {
         CATCH_REQUIRE(std::isnan(y));
      } else {
         CATCH_REQUIRE(x == y);
      }
   } else {
      CATCH_REQUIRE(x == y);
   }
}

template<typename T> void test_fd_T(vector<char>& buffer)
{
   //
   test_save_load_T<T>(buffer, T(0.0));
   test_save_load_T<T>(buffer, T(-0.0));
   test_save_load_T<T>(buffer, T(1.0));
   test_save_load_T<T>(buffer, std::numeric_limits<T>::lowest());
   test_save_load_T<T>(buffer, std::numeric_limits<T>::max());
   test_save_load_T<T>(buffer, std::numeric_limits<T>::min());
   test_save_load_T<T>(buffer, std::numeric_limits<T>::infinity());
   test_save_load_T<T>(buffer, std::numeric_limits<T>::quiet_NaN());
}

template<typename T> void test_save_load_fp(vector<char>& buffer)
{
   T x{};
   test_save_load_T<T>(buffer, x);
}

CATCH_TEST_CASE("FpIo", "[fp-io]")
{
   CATCH_SECTION("fp-io")
   {
      vector<char> buffer;
      buffer.resize(1024);

      test_save_load_fp<bool>(buffer);
      test_save_load_fp<char>(buffer);
      test_save_load_fp<int32_t>(buffer);
      test_save_load_fp<uint32_t>(buffer);
      test_save_load_fp<float>(buffer);
      test_save_load_fp<double>(buffer);
      test_save_load_fp<real>(buffer);
      test_save_load_fp<string>(buffer);
      test_save_load_fp<Vector3>(buffer);
      test_save_load_fp<Vector2f>(buffer);
      test_save_load_fp<Vector3f>(buffer);

      test_save_load_T<bool>(buffer, true);
      test_save_load_T<bool>(buffer, false);
      for(int i = std::numeric_limits<char>::lowest();
          i <= int(std::numeric_limits<char>::max());
          ++i) {
         test_save_load_T<char>(buffer, char(i));
         test_save_load_T<int32_t>(buffer, int32_t(i));
         test_save_load_T<uint32_t>(buffer, uint32_t(i));
      }

      test_fd_T<float>(buffer);
      test_fd_T<double>(buffer);
      test_fd_T<real>(buffer);

      test_save_load_T<array<double, 6>>(buffer,
                                         {0.0, 1.0, 2.0, 3.0, 4.0, 5.0});

      test_save_load_T<Vector2r>(buffer, Vector2r{1.0, 2.0});
      test_save_load_T<Vector3r>(buffer, Vector3r{1.0, 2.0, 3.0});

      {
         Vector6r X{};
         X << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0;
         test_save_load_T<Vector6r>(buffer, X);
      }

      {
         Matrix3r X{};
         X << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0;
         test_save_load_T<Matrix3r>(buffer, X);
      }

      {
         MatrixXr X{};
         for(auto r = 0; r < 5; ++r) {
            for(auto c = 0; c < 5; ++c) {
               X.resize(r, c);

               for(auto y = 0; y < r; ++y)
                  for(auto x = 0; x < c; ++x) X(y, x) = (y + 1) * (x + 1);

               test_save_load_T<MatrixXr>(buffer, X);
            }
         }
      }
   }
}

} // namespace perceive
