
#pragma once

#include "perceive/foundation.hpp"
#include "perceive/geometry/splines/spline-2d.hpp"
#include "perceive/geometry/vector.hpp"
#include "perceive/graphics/image-container.hpp"

namespace perceive
{
// ------------------------------------------------------------------------ save

void save_bool(FILE* fp, bool val);
void save_char(FILE* fp, char val);
void save_int(FILE* fp, int32_t val);
void save_uint(FILE* fp, uint32_t val);
void save_float(FILE* fp, float val);
void save_double(FILE* fp, double val);
void save_real(FILE* fp, real val);
void save_str(FILE* fp, const std::string& val);
void save_vector3(FILE* fp, const Vector3& G);
void save_vec2f(FILE* fp, const Vector2f& G);
void save_vec3f(FILE* fp, const Vector3f& G);
void save_arr6(FILE* fp, const array<double, 6>& arr);
void save_vec2(FILE* fp, const Vector2r& G);
void save_vec3(FILE* fp, const Vector3r& G);
void save_vec6(FILE* fp, const Vector6r& G);
void save_mat3r(FILE* fp, const Matrix3r& G);
void save_matXr(FILE* fp, const MatrixXr& G);
void save_spline(FILE* fp, const Spline2d& spline);

template<typename Itr, typename F>
void save_vec(FILE* fp, Itr begin, Itr end, F f)
{
   save_uint(fp, unsigned(std::distance(begin, end)));
   for(auto ii = begin; ii != end; ++ii) f(fp, *ii);
}

template<typename T> void save_fp(FILE* fp, const T& x)
{
   if constexpr(std::is_same<T, bool>::value)
      save_bool(fp, x);
   else if constexpr(std::is_same<T, char>::value)
      save_char(fp, x);
   else if constexpr(std::is_same<T, int32_t>::value)
      save_int(fp, x);
   else if constexpr(std::is_same<T, uint32_t>::value)
      save_uint(fp, x);
   else if constexpr(std::is_same<T, float>::value)
      save_float(fp, x);
   else if constexpr(std::is_same<T, double>::value)
      save_double(fp, x);
   else if constexpr(std::is_same<T, string>::value)
      save_str(fp, x);
   else if constexpr(std::is_same<T, Vector3>::value)
      save_vector3(fp, x);
   else if constexpr(std::is_same<T, Vector2f>::value)
      save_vec2f(fp, x);
   else if constexpr(std::is_same<T, Vector3f>::value)
      save_vec3f(fp, x);
   else if constexpr(std::is_same<T, array<double, 6>>::value)
      save_arr6(fp, x);
   else if constexpr(std::is_same<T, Vector2r>::value)
      save_vec2(fp, x);
   else if constexpr(std::is_same<T, Vector3r>::value)
      save_vec3(fp, x);
   else if constexpr(std::is_same<T, Vector6r>::value)
      save_vec6(fp, x);
   else if constexpr(std::is_same<T, Matrix3r>::value)
      save_mat3r(fp, x);
   else if constexpr(std::is_same<T, MatrixXr>::value)
      save_matXr(fp, x);
   else if constexpr(std::is_same<T, Spline2d>::value)
      save_spline(fp, x);
   else
      throw std::logic_error("invalid type in save-fp");
}

// ------------------------------------------------------------------------ load

void load_bool(FILE* fp, bool& val);
void load_char(FILE* fp, char& val);
void load_int(FILE* fp, int32_t& val);
void load_uint(FILE* fp, uint32_t& val);
void load_float(FILE* fp, float& val);
void load_double(FILE* fp, double& val);
void load_real(FILE* fp, real& val);
void load_str(FILE* fp, std::string& val);
void load_vector3(FILE* fp, Vector3& G);
void load_vec2f(FILE* fp, Vector2f& G);
void load_vec3f(FILE* fp, Vector3f& G);
void load_arr6(FILE* fp, array<double, 6>& arr);
void load_vec2(FILE* fp, Vector2r& G);
void load_vec3(FILE* fp, Vector3r& G);
void load_vec6(FILE* fp, Vector6r& G);
void load_mat3r(FILE* fp, Matrix3r& G);
void load_matXr(FILE* fp, MatrixXr& G);
void load_spline(FILE* fp, Spline2d& spline);

template<typename T, typename F> void load_vec(FILE* fp, vector<T>& vec, F f)
{
   unsigned N = 0;
   load_uint(fp, N);
   vec.resize(N);
   for(auto& G : vec) f(fp, G);
}

template<typename T> void load_fp(FILE* fp, T& x) noexcept(false)
{
   if constexpr(std::is_same<T, bool>::value)
      load_bool(fp, x);
   else if constexpr(std::is_same<T, char>::value)
      load_char(fp, x);
   else if constexpr(std::is_same<T, int32_t>::value)
      load_int(fp, x);
   else if constexpr(std::is_same<T, uint32_t>::value)
      load_uint(fp, x);
   else if constexpr(std::is_same<T, float>::value)
      load_float(fp, x);
   else if constexpr(std::is_same<T, double>::value)
      load_double(fp, x);
   else if constexpr(std::is_same<T, string>::value)
      load_str(fp, x);
   else if constexpr(std::is_same<T, Vector3>::value)
      load_vector3(fp, x);
   else if constexpr(std::is_same<T, Vector2f>::value)
      load_vec2f(fp, x);
   else if constexpr(std::is_same<T, Vector3f>::value)
      load_vec3f(fp, x);
   else if constexpr(std::is_same<T, array<double, 6>>::value)
      load_arr6(fp, x);
   else if constexpr(std::is_same<T, Vector2r>::value)
      load_vec2(fp, x);
   else if constexpr(std::is_same<T, Vector3r>::value)
      load_vec3(fp, x);
   else if constexpr(std::is_same<T, Vector6r>::value)
      load_vec6(fp, x);
   else if constexpr(std::is_same<T, Matrix3r>::value)
      load_mat3r(fp, x);
   else if constexpr(std::is_same<T, MatrixXr>::value)
      load_matXr(fp, x);
   else if constexpr(std::is_same<T, Spline2d>::value)
      load_spline(fp, x);
   else
      throw std::logic_error("invalid type in load-fp");
}

// -----------------------------------------------------------------------------
//
template<typename T>
inline void write_image_container(FILE* fp, const ImageContainerT<T>& im)
{
   save_uint(fp, im.width);
   save_uint(fp, im.height);
   save_uint(fp, im.row_stride);
   auto ret = fwrite(im.data(), 1, im.n_bytes(), fp);
   if(ret != im.n_bytes())
      FATAL(format("write failed: {} != {}", ret, im.n_bytes()));
}

template<typename T>
inline void read_image_container(FILE* fp, ImageContainerT<T>& im)
{
   unsigned w = 0, h = 0, stride = 0;
   load_uint(fp, w);
   load_uint(fp, h);
   load_uint(fp, stride);
   im.resize(w, h, stride);
   auto ret = fread(im.data(), 1, im.n_bytes(), fp);
   if(ret != im.n_bytes())
      FATAL(format("read failed: {} != {}", ret, im.n_bytes()));
}

// ----------------------------------------------------------------- legacy-load

void legacy_load_uint(FILE* fp, unsigned& val);
void legacy_load_real(FILE* fp, real& val);
void legacy_load_str(FILE* fp, std::string& val);

} // namespace perceive
