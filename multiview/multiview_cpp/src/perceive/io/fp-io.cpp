
#include "fp-io.hpp"
// #include "perceive/utils/md5.hpp"

#include <cfloat>
#include <stdio.h>

#include <boost/endian/conversion.hpp>

#include "ieee754-packing.hpp"

namespace perceive
{
static const unsigned k_max_str_bytes = 10 * 1024 * 1024; // 10 Megs

enum class SaveType : uint8_t {
   BOOL = 0, //  0
   CHAR,     //  1
   INT32,    //  2
   UINT32,   //  3
   FLOAT,    //  4
   DOUBLE,   //  5
   STRING,   //  6
   VEC3,     //  7
   VEC_2F,   //  8
   VEC_3F,   //  9
   ARR6,     // 10
   VEC_2R,   // 11
   VEC_3R,   // 12
   VEC_6R,   // 13
   MAT_3R,   // 14
   MAT_XR,   // 15
   SPLINE_2D // 16
};

static void save_type(FILE* fp, SaveType type)
{
   const auto sz = fwrite(&type, sizeof(SaveType), 1, fp);
   if(sz != 1) throw std::runtime_error("failed to write save-type");
}

static void check_type(FILE* fp, SaveType type)
{
   int val = fgetc(fp);
   if(int(type) != val)
      throw std::runtime_error(format("failed to read FILE*, type-check "
                                      "failed. Loaded {}, but expected {}",
                                      val,
                                      int(type)));
}

void save_bool(FILE* fp, bool val)
{
   save_type(fp, SaveType::BOOL);
   char o        = val ? '1' : '0';
   const auto sz = fwrite(&o, sizeof(char), 1, fp);
   if(sz != 1) throw std::runtime_error("failed to write char");
}

void save_char(FILE* fp, char val)
{
   save_type(fp, SaveType::CHAR);
   const auto sz = fwrite(&val, sizeof(char), 1, fp);
   if(sz != 1) throw std::runtime_error("failed to write char");
}

template<typename T> void save_int_T(SaveType type, FILE* fp, T val)
{
   save_type(fp, type);
   const T packed = boost::endian::native_to_little(val);
   const auto sz  = fwrite(&packed, sizeof(T), 1, fp);
   if(sz != 1) throw std::runtime_error(format("write error to FILE*"));
}

void save_int(FILE* fp, int32_t val) { save_int_T(SaveType::INT32, fp, val); }

void save_uint(FILE* fp, uint32_t val)
{
   save_int_T(SaveType::UINT32, fp, val);
}

template<typename T> void save_fd(FILE* fp, T val)
{
   if constexpr(std::is_same<T, float>::value) {
      save_type(fp, SaveType::FLOAT);
      const uint32_t packed = boost::endian::native_to_little(pack_f32(val));
      const auto sz         = fwrite(&packed, sizeof(packed), 1, fp);
      if(sz != 1) throw std::runtime_error(format("write error to FILE*"));
   } else if constexpr(std::is_same<T, double>::value) {
      save_type(fp, SaveType::DOUBLE);
      const uint64_t packed = boost::endian::native_to_little(pack_f64(val));
      const auto sz         = fwrite(&packed, sizeof(packed), 1, fp);
      if(sz != 1) throw std::runtime_error(format("write error to FILE*"));
   } else {
      FATAL(format("logic error"));
   }
}

void save_float(FILE* fp, float val) { save_fd(fp, val); }
void save_double(FILE* fp, double val) { save_fd(fp, val); }
void save_real(FILE* fp, real val) { save_fd(fp, val); }

void save_str(FILE* fp, const std::string& val)
{
   save_type(fp, SaveType::STRING);
   unsigned len = unsigned(val.size());
   if(len > k_max_str_bytes)
      throw std::runtime_error(
          format("cowardly refusing to save string with {} characters", len));

   save_uint(fp, len);
   auto sz = fwrite(val.data(), 1, len, fp);
   Expects(len == sz);
}

void save_vector3(FILE* fp, const Vector3& G)
{
   save_type(fp, SaveType::VEC3);
   for(unsigned i = 0; i < G.size(); ++i) save_real(fp, G[int(i)]);
}

void save_vec2f(FILE* fp, const Vector2f& G)
{
   save_type(fp, SaveType::VEC_2F);
   for(auto i = 0u; i < G.size(); ++i) save_float(fp, G[int(i)]);
}

void save_vec3f(FILE* fp, const Vector3f& G)
{
   save_type(fp, SaveType::VEC_3F);
   for(auto i = 0u; i < G.size(); ++i) save_float(fp, G[int(i)]);
}

void save_arr6(FILE* fp, const array<double, 6>& arr)
{
   save_type(fp, SaveType::ARR6);
   for(const auto& X : arr) save_real(fp, X);
}

template<typename Mat> void save_mat_T(SaveType type, FILE* fp, const Mat& G)
{
   save_type(fp, type);
   save_uint(fp, unsigned(G.rows()));
   save_uint(fp, unsigned(G.cols()));
   for(unsigned row = 0; row < G.rows(); ++row)
      for(unsigned col = 0; col < G.cols(); ++col) save_real(fp, G(row, col));
}

void save_vec2(FILE* fp, const Vector2r& G)
{
   save_mat_T(SaveType::VEC_2R, fp, G);
}

void save_vec3(FILE* fp, const Vector3r& G)
{
   save_mat_T(SaveType::VEC_3R, fp, G);
}

void save_vec6(FILE* fp, const Vector6r& G)
{
   save_mat_T(SaveType::VEC_6R, fp, G);
}

void save_mat3r(FILE* fp, const Matrix3r& G)
{
   save_mat_T(SaveType::MAT_3R, fp, G);
}

void save_matXr(FILE* fp, const MatrixXr& G)
{
   save_mat_T(SaveType::MAT_XR, fp, G);
}

void save_spline(FILE* fp, const Spline2d& spline)
{
   save_type(fp, SaveType::SPLINE_2D);
   auto coeffs = spline.unpack();
   save_uint(fp, unsigned(coeffs.size()));
   save_real(fp, spline.range_hint().x);
   save_real(fp, spline.range_hint().y);
   for(const auto& coeff : coeffs) {
      for(const auto& x : coeff) {
         save_real(fp, x);
         // cout << format("{:10.6f}", x) << ", ";
      }
      // cout << endl;
   }
}

// -----------------------------------------------------------------------------

void load_bool(FILE* fp, bool& val)
{
   check_type(fp, SaveType::BOOL);
   const int x = fgetc(fp);
   if(x == EOF) throw std::runtime_error("failed to read char");
   val = (x == '1');
}

void load_char(FILE* fp, char& val)
{
   check_type(fp, SaveType::CHAR);
   const int x = fgetc(fp);
   if(x == EOF) throw std::runtime_error("Failed to read char");
   val = static_cast<char>(x);
}

template<typename T> void load_int_T(SaveType type, FILE* fp, T& val)
{
   check_type(fp, type);
   T in          = 0;
   const auto sz = fread(&in, sizeof(T), 1, fp);
   if(sz != 1) throw std::runtime_error(format("read error reading FILE*"));
   val = boost::endian::little_to_native(in);
}

void load_int(FILE* fp, int32_t& val) { load_int_T(SaveType::INT32, fp, val); }

void load_uint(FILE* fp, uint32_t& val)
{
   load_int_T(SaveType::UINT32, fp, val);
}

template<typename T> void load_fd(FILE* fp, T& val)
{
   if constexpr(std::is_same<T, float>::value) {
      uint32_t in = 0;
      load_int_T(SaveType::FLOAT, fp, in);
      val = unpack_f32(boost::endian::little_to_native(in));
   } else if constexpr(std::is_same<T, double>::value) {
      uint64_t in = 0;
      load_int_T(SaveType::DOUBLE, fp, in);
      val = unpack_f64(boost::endian::little_to_native(in));
   } else {
      FATAL("logic error");
   }
}

void load_float(FILE* fp, float& val) { load_fd(fp, val); }
void load_double(FILE* fp, double& val) { load_fd(fp, val); }
void load_real(FILE* fp, real& val) { load_fd(fp, val); }

void load_str(FILE* fp, std::string& val)
{
   check_type(fp, SaveType::STRING);
   const unsigned max_str_bytes = 10 * 1024 * 1024; // 10 Megs
   unsigned len                 = 0;
   load_uint(fp, len);
   if(len > k_max_str_bytes)
      throw std::runtime_error(
          format("cowardly refusing to create string with {} characters", len));

   val.resize(len);
   auto sz = fread(&val[0], 1, len, fp);

   if(sz != len)
      throw std::runtime_error(
          format("Expected {} values, but got {}", len, sz));
}

void load_vector3(FILE* fp, Vector3& G)
{
   check_type(fp, SaveType::VEC3);
   for(unsigned i = 0; i < G.size(); ++i) load_real(fp, G[int(i)]);
}

void load_vec2f(FILE* fp, Vector2f& G)
{
   check_type(fp, SaveType::VEC_2F);
   for(auto i = 0u; i < G.size(); ++i) load_float(fp, G[int(i)]);
}

void load_vec3f(FILE* fp, Vector3f& G)
{
   check_type(fp, SaveType::VEC_3F);
   for(auto i = 0u; i < G.size(); ++i) load_float(fp, G[int(i)]);
}

void load_arr6(FILE* fp, array<double, 6>& arr)
{
   check_type(fp, SaveType::ARR6);
   for(auto& X : arr) load_real(fp, X);
}

template<typename Mat> void load_mat_T(SaveType type, FILE* fp, Mat& G)
{
   check_type(fp, type);
   unsigned n_rows = 0, n_cols = 0;
   load_uint(fp, n_rows);
   load_uint(fp, n_cols);

   if(G.rows() != int(n_rows) or G.cols() != int(n_cols)) {
      // Attempt a resize
      G.resize(n_rows, n_cols);
   }

   double val = 0.0;
   for(unsigned row = 0; row < G.rows(); ++row)
      for(unsigned col = 0; col < G.cols(); ++col) {
         load_real(fp, val);
         G(row, col) = val;
      }
}

void load_vec2(FILE* fp, Vector2r& G) { load_mat_T(SaveType::VEC_2R, fp, G); }

void load_vec3(FILE* fp, Vector3r& G) { load_mat_T(SaveType::VEC_3R, fp, G); }

void load_vec6(FILE* fp, Vector6r& G) { load_mat_T(SaveType::VEC_6R, fp, G); }

void load_mat3r(FILE* fp, Matrix3r& G) { load_mat_T(SaveType::MAT_3R, fp, G); }

void load_matXr(FILE* fp, MatrixXr& G) { load_mat_T(SaveType::MAT_XR, fp, G); }

void load_spline(FILE* fp, Spline2d& spline)
{
   check_type(fp, SaveType::SPLINE_2D);
   unsigned N = 0;
   load_uint(fp, N);
   vector<array<double, 12>> coeffs(N);
   load_real(fp, spline.range_hint().x);
   load_real(fp, spline.range_hint().y);
   for(auto& coeff : coeffs) {
      for(auto& x : coeff) {
         load_real(fp, x);
         // cout << format("{:10.6f}", x) << ", ";
      }
      // cout << endl;
   }
   spline.init(coeffs);
}

// ----------------------------------------------------------------- legacy-load

void legacy_load_uint(FILE* fp, unsigned& val)
{
   if(fscanf(fp, "%u\n", &val) != 1)
      throw std::runtime_error("Failed to read unsigned value");
}

void legacy_load_real(FILE* fp, real& o)
{
   double val = 0.0;
   if(fscanf(fp, "%lf\n", &val) != 1)
      throw std::runtime_error("Failed to read float value");
   o = val;
}

void legacy_load_str(FILE* fp, std::string& val)
{
   unsigned len = 0;
   auto pos     = ftell(fp);
   legacy_load_uint(fp, len);

   // We only want to consume _ONE_ whitespace character (GRR!!!)
   fseek(fp, pos, SEEK_SET);
   while(!feof(fp))
      if(fgetc(fp) == '\n') break; // Good, only 1 '\n' consumed

   val.resize(len);
   auto sz = fread(&val[0], 1, len, fp);

   if(sz != len)
      throw std::runtime_error(
          format("Expected {} values, but got {}", len, sz));
}

} // namespace perceive
