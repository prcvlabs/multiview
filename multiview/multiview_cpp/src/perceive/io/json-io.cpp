
#include "json-io.hpp"
#include "perceive/geometry.hpp"
#include "perceive/geometry/projective/camera.hpp"
#include "perceive/utils/file-system.hpp"

namespace perceive
{
// ------------------------------------------------------------------------ Load

double load_numeric(const Json::Value& elem) noexcept(false)
{
   if(elem.isNull())
      return dNAN;
   else if(!elem.isNumeric())
      throw std::runtime_error(format("expected numeric value"));
   return elem.asDouble();
}

template<typename T> void loadT(const Json::Value& node, T* v, unsigned size)
{
   if(!node.isArray()) throw std::runtime_error("expecting JSON array value");
   if(node.size() != size)
      throw std::runtime_error(format("incorrect number of elements in array "
                                      "value. Expected {}, but got {}.",
                                      size,
                                      node.size()));

   for(unsigned i = 0; i < size; ++i) v[i] = T(load_numeric(node[i]));
}

void json_load(const Json::Value& node, OrderedPair& v)
{
   Vector2 u;
   loadT(node, u.ptr(), u.size());
   v.set(int(std::round(u.x)), int(std::round(u.y)));
}
void json_load(const Json::Value& node, AABB& v)
{
   loadT(node, v.ptr(), v.size());
}
void json_load(const Json::Value& node, Point2& v)
{
   loadT(node, v.ptr(), v.size());
}
void json_load(const Json::Value& node, Point3& v)
{
   loadT(node, v.ptr(), v.size());
}
void json_load(const Json::Value& node, Vector2& v)
{
   loadT(node, v.ptr(), v.size());
}
void json_load(const Json::Value& node, Vector3& v)
{
   loadT(node, v.ptr(), v.size());
}
void json_load(const Json::Value& node, Vector4& v)
{
   loadT(node, v.ptr(), v.size());
}
void json_load(const Json::Value& node, Vector2f& v)
{
   loadT(node, v.ptr(), v.size());
}
void json_load(const Json::Value& node, Vector3f& v)
{
   loadT(node, v.ptr(), v.size());
}
void json_load(const Json::Value& node, Vector4f& v)
{
   loadT(node, v.ptr(), v.size());
}
void json_load(const Json::Value& node, Quaternion& v)
{
   loadT(node, v.ptr(), v.size());
}

void json_load(const Json::Value& node, bool& v)
{
   if(!node.isBool()) throw std::runtime_error("expected boolean value");
   v = node.asBool();
}
void json_load(const Json::Value& node, int16_t& v)
{
   int val = 0;
   json_load(node, val);
   if(val > std::numeric_limits<int16_t>::max()
      or val < std::numeric_limits<int16_t>::lowest())
      throw std::runtime_error(format("int16_t value {} out of bounds", val));
   v = static_cast<int16_t>(val);
}
void json_load(const Json::Value& node, int& v)
{
   if(!node.isInt()) throw std::runtime_error("expected integer value");
   v = node.asInt();
}
void json_load(const Json::Value& node, unsigned& v)
{
   if(!node.isUInt()) throw std::runtime_error("expected unsigned value");
   v = node.asUInt();
}
void json_load(const Json::Value& node, float& v)
{
   v = float(load_numeric(node));
}
void json_load(const Json::Value& node, double& v) { v = load_numeric(node); }
void json_load(const Json::Value& node, std::string& v) { v = node.asString(); }

void json_load(const Json::Value& node, Timestamp& o)
{
   std::string s;
   json_load(node, s);
   o = Timestamp(s);
}

// ----------------------------------------------------------------- Load Camera

Json::Value get_key(const Json::Value& node, const char* key)
{
   if(!node.isMember(key))
      throw std::runtime_error(format("array key '{}' missing", key));
   return node[key];
}

void json_load(const Json::Value& node, Matrix3r& A)
{
   MatrixXr A0;
   json_load(node, A0);
   if(A0.rows() == A0.cols() && A0.rows() == 3) {
      for(unsigned r = 0; r < 3; ++r)
         for(unsigned c = 0; c < 3; ++c) A(r, c) = A0(r, c);
   } else {
      throw std::runtime_error(
          format("expected a 3x3 matrix but got {}x{}", A0.rows(), A0.cols()));
   }
}

void json_load(const Json::Value& node, MatrixXr& A)
{
   if(!node.isArray()) throw std::runtime_error("expecting JSON array value");

   const unsigned n_rows = node.size();
   if(n_rows == 0) throw std::runtime_error("expected at least one row!");

   auto count_cols = [&](unsigned row_ind) {
      const Json::Value& row = node[row_ind];
      if(!row.isArray())
         throw std::runtime_error("expecting JSON array of arrays!");
      return row.size();
   };

   const unsigned n_cols = count_cols(0);
   A                     = MatrixXr::Zero(n_rows, n_cols);

   for(unsigned r = 0; r < n_rows; ++r) {
      if(count_cols(r) != n_cols)
         throw std::runtime_error("exepcted JSON array of arrays where "
                                  "every array has the same size.");
      for(unsigned c = 0; c < n_cols; ++c) A(r, c) = load_numeric(node[r][c]);
   }
}

void json_load(const Json::Value& node, EuclideanTransform& et)
{
   et = EuclideanTransform(); // Zero data

   try {
      json_load(get_key(node, "translation"), et.translation);
   } catch(std::runtime_error& e) {
      // ignore
   }

   try {
      json_load(get_key(node, "rotation"), et.rotation);
   } catch(std::runtime_error& e) {
      // ignore
   }

   auto scale_key = node["scale"];
   et.scale       = scale_key.isNull() ? 1.0 : load_numeric(scale_key);
}

// ------------------------------------------------------------------------ Save

template<typename T> Json::Value saveT(const T* v, unsigned size)
{
   Json::Value X(Json::arrayValue);
   X.resize(size);
   for(unsigned i = 0; i < size; ++i) X[i] = real(v[i]);
   return X;
}

Json::Value json_save(const bool& x)
{
   auto z = Json::Value(x);
   Expects(z.type() == Json::booleanValue);
   return z;
}
Json::Value json_save(const int16_t& x) { return Json::Value(int(x)); }
Json::Value json_save(const int& x) { return Json::Value(x); }
Json::Value json_save(const unsigned& x) { return Json::Value(x); }
Json::Value json_save(const float& x)
{
   return (std::isfinite(x)) ? Json::Value(double(x))
                             : Json::Value(Json::nullValue);
}
Json::Value json_save(const double& x)
{
   return (std::isfinite(x)) ? Json::Value(x) : Json::Value(Json::nullValue);
}

Json::Value json_save(const OrderedPair& v)
{
   assert(v.size() == 2);
   OrderedPair::value_type P[2] = {v.x(), v.y()};
   return saveT(P, 2);
}
Json::Value json_save(const AABB& v) { return saveT(v.ptr(), v.size()); }
Json::Value json_save(const Point2& v) { return saveT(v.ptr(), v.size()); }
Json::Value json_save(const Vector2f& v) { return saveT(v.ptr(), v.size()); }
Json::Value json_save(const Vector3f& v) { return saveT(v.ptr(), v.size()); }
Json::Value json_save(const Vector4f& v) { return saveT(v.ptr(), v.size()); }
Json::Value json_save(const Vector2& v) { return saveT(v.ptr(), v.size()); }
Json::Value json_save(const Vector3& v) { return saveT(v.ptr(), v.size()); }
Json::Value json_save(const Vector4& v) { return saveT(v.ptr(), v.size()); }
Json::Value json_save(const Quaternion& v) { return saveT(v.ptr(), v.size()); }

Json::Value json_save(const Timestamp& v) { return json_save(str(v)); }

template<typename T> Json::Value json_save_T(const T& A)
{
   unsigned N = unsigned(A.rows() * A.cols());
   Json::Value X(Json::arrayValue);
   X.resize(unsigned(A.rows()));

   for(unsigned r = 0; r < A.rows(); ++r) {
      X[r] = Json::Value(Json::arrayValue);
      X[r].resize(unsigned(A.cols()));
      for(unsigned c = 0; c < A.cols(); ++c) X[r][c] = A(r, c);
   }
   return X;
}

Json::Value json_save(const Matrix3r& A) { return json_save_T(A); }
Json::Value json_save(const MatrixXr& A) { return json_save_T(A); }

// --------------------------------------------------------- Euclidean Transform

Json::Value json_save(const EuclideanTransform& et)
{
   Json::Value node(Json::objectValue);
   node["translation"] = json_save(et.translation);
   node["rotation"]    = json_save(et.rotation);
   node["scale"]       = Json::Value(et.scale);
   return node;
}

// ---------------------------------------------------------------------- String

Json::Value json_save(const string& s)
{
   Json::Value node(Json::stringValue);
   node = s;
   return node;
}

// ----------------------------------------------------------------- Str to JSON

Json::Value str_to_json(const string& json) noexcept(false)
{
   return parse_json(json);
}

// ------------------------------------------------------------------ Parse JSON

Json::Value parse_json(const std::string& s) noexcept(false)
{
   static thread_local Json::CharReaderBuilder json_parser_builder_;
   auto reader
       = unique_ptr<Json::CharReader>(json_parser_builder_.newCharReader());

   Json::Value root;
   std::string err = "";

   try {
      reader->parse(s.data(), s.data() + s.size(), &root, &err);
      if(err != "") err = format("error reading JSON data: {}", err);
   } catch(std::runtime_error& e) {
      err = format("error reading JSON data: {}", e.what());
   } catch(std::exception& e) {
      // JSON parse error
      err = format("parse error reading JSON");
   } catch(...) {
      err = format("unknown error reading JSON");
   }

   // if(err != "") { cout << s << endl << endl; }

   if(err != "") throw std::runtime_error(err);
   return root;
}

bool parse_json(const string& s, Json::Value& val) noexcept
{
   try {
      val = parse_json(s);
      return true;
   } catch(std::exception&) {
      // do nothing
   }
   return false;
}

} // namespace perceive
