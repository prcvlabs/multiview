
#pragma once

#include "json/json.h"

#include "perceive/geometry.hpp"
#include "perceive/geometry/projective/camera.hpp"
#include "perceive/utils/timestamp.hpp"

namespace perceive
{
void json_load(const Json::Value& node, bool&);
void json_load(const Json::Value& node, int16_t&);
void json_load(const Json::Value& node, int&);
void json_load(const Json::Value& node, unsigned&);
void json_load(const Json::Value& node, float&);
void json_load(const Json::Value& node, double&);
void json_load(const Json::Value& node, std::string&);
void json_load(const Json::Value& node, OrderedPair&);
void json_load(const Json::Value& node, AABB&);
void json_load(const Json::Value& node, Point2&);
void json_load(const Json::Value& node, Point3&);
void json_load(const Json::Value& node, Vector2f&);
void json_load(const Json::Value& node, Vector3f&);
void json_load(const Json::Value& node, Vector4f&);
void json_load(const Json::Value& node, Vector2&);
void json_load(const Json::Value& node, Vector3&);
void json_load(const Json::Value& node, Vector4&);
void json_load(const Json::Value& node, Quaternion&);
// void json_load(const Json::Value& node, Camera&);
void json_load(const Json::Value& node, Matrix3r&);
void json_load(const Json::Value& node, MatrixXr&);
void json_load(const Json::Value& node, EuclideanTransform&);
void json_load(const Json::Value& node, Timestamp&);

double load_numeric(const Json::Value& elem) noexcept(false);

template<typename T>
inline void json_load(const Json::Value& node, vector<T>& o)
{
   if(!node.isArray()) throw std::runtime_error("expected an array");
   o.resize(node.size());
   for(auto i = 0u; i < o.size(); ++i) json_load(node[i], o[i]);
}

template<typename T>
inline void
json_load_t(const Json::Value& node,
            vector<T>& o,
            std::function<void(const Json::Value& node, T& value)> f)
{
   if(!node.isArray()) throw std::runtime_error("expected an array");
   o.resize(node.size());
   for(auto i = 0u; i < o.size(); ++i) f(node[i], o[i]);
}

Json::Value json_save(const bool&);
Json::Value json_save(const int16_t&);
Json::Value json_save(const int&);
Json::Value json_save(const unsigned&);
Json::Value json_save(const float&);
Json::Value json_save(const double&);
Json::Value json_save(const string&);
Json::Value json_save(const OrderedPair&);
Json::Value json_save(const AABB&);
Json::Value json_save(const Point2&);
Json::Value json_save(const Vector2f&);
Json::Value json_save(const Vector3f&);
Json::Value json_save(const Vector4f&);
Json::Value json_save(const Vector2&);
Json::Value json_save(const Vector3&);
Json::Value json_save(const Vector4&);
Json::Value json_save(const Quaternion&);
Json::Value json_save(const Matrix3r&);
Json::Value json_save(const MatrixXr&);
Json::Value json_save(const EuclideanTransform&);
Json::Value json_save(const Timestamp&);

template<typename InputIt>
inline Json::Value json_save(InputIt cbegin, InputIt cend)
{
   Json::Value x{Json::arrayValue};
   x.resize(unsigned(std::distance(cbegin, cend)));
   for(auto i = 0u; i < x.size(); ++i) x[i] = json_save(*cbegin++);
   return x;
}

template<typename InputIt>
inline Json::Value
json_save_t(InputIt cbegin,
            InputIt cend,
            std::function<Json::Value(
                typename std::iterator_traits<InputIt>::reference&)> f)
{
   Json::Value x{Json::arrayValue};
   x.resize(unsigned(std::distance(cbegin, cend)));
   for(auto i = 0u; i < x.size(); ++i) x[i] = f(*cbegin++);
   return x;
}

template<typename T> inline string json_encode(const T& o)
{
   std::stringstream ss("");
   ss << json_save(o);
   return ss.str();
}

inline string json_encode_str(const string& s) { return json_encode(s); }

/**
 * For example,
 *
 *      unsigned width{10};
 *      node["width"] = width;
 *      // ...
 *      json_load(get_key(node, "width"), width);
 *
 */
Json::Value get_key(const Json::Value& node, const char* key);

inline Json::Value get_key(const Json::Value& node, const string_view& key)
{
   return get_key(node, key.data());
}

inline bool has_key(const Json::Value& node, const string_view& key)
{
   if(node.type() != Json::objectValue) return false;
   return node.isMember(key.data());
}

Json::Value str_to_json(const string& json) noexcept(false);
Json::Value parse_json(const string& s) noexcept(false); // that-is throws

// RETURN true if parsing was successful
bool parse_json(const string& s, Json::Value& val) noexcept;

template<typename T>
inline T json_load_key(const Json::Value& node,
                       const string_view key,
                       const string_view operation) noexcept(false)
{
   if(!has_key(node, key))
      throw std::runtime_error(
          format("failed to find key '{}' while {}", key, operation));
   T value;
   try {
      json_load(get_key(node, key), value);
   } catch(std::runtime_error& e) {
      throw std::runtime_error(format(
          "error loading key '{}' while {}: {}", key, operation, e.what()));
   }
   return value;
}

template<typename T>
inline bool json_try_load_key(T& result,
                              const Json::Value& node,
                              const string_view key,
                              const string_view operation,
                              const bool warn_on_fail = true) noexcept
{
   if(!has_key(node, key)) {
      if(warn_on_fail)
         WARN(format("failed to find key '{}' while {}", key, operation));
      return false;
   }
   try {
      json_load(get_key(node, key), result);
      return true;
   } catch(std::runtime_error& e) {
      if(warn_on_fail)
         WARN(format(
             "error loading key '{}' while {}: {}", key, operation, e.what()));
   }
   return false;
}

template<typename T>
inline bool
json_try_load_t(vector<T>& o,
                const Json::Value& node,
                const string_view key,
                std::function<void(const Json::Value& node, T& value)> f,
                const string_view operation,
                const bool warn_on_fail = true)
{
   bool success = false;
   if(!has_key(node, key)) {
      if(warn_on_fail)
         WARN(format("failed to find key '{}' while {}", key, operation));
      return success;
   }

   try {
      vector<T> z;
      json_load_t(get_key(node, key), z, f);
      o       = std::move(z);
      success = true;
   } catch(std::exception& e) {
      if(warn_on_fail)
         WARN(format(
             "error loading key '{}' while {}: {}", key, operation, e.what()));
   }
   return success;
}

template<typename T> bool test_json_serialization(const T& A) noexcept
{
   thread_local bool run_test = true;
   if(!run_test) return true;
   run_test = false; // Prevents recursive call

   bool ret = false;
   try {
      Json::Value serialized = A.to_json();
      T B;
      B.read(serialized); // run_test prevents recursive call here
      ret = (A == B);
   } catch(...) {
      ret = false;
   }

   run_test = true; // Allow future tests
   return ret;
}

inline string str(const Json::Value& o) noexcept
{
   std::stringstream ss{""s};
   ss << o;
   return ss.str();
}

} // namespace perceive
