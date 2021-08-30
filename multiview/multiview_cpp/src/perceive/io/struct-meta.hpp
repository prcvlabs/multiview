
#pragma once

#include <any>

#include "perceive/io/json-io.hpp"
#include "perceive/utils/file-system.hpp"
#include "perceive/utils/string-utils.hpp"
#include "json/json.h"

/**
 * Example Usage

// Some "vanilla" type
struct TypeA
{
   int x   = -1;
   bool y  = false;
   float z = 1.0f;

   // static function returns meta data for type
   static const vector<MemberMetaData>& meta_data() noexcept;
};

//
// In cpp file
//
const vector<MemberMetaData>& TypeA::meta_data() noexcept
{
   auto make_it = []() {
      vector<MemberMetaData> M;
      M.push_back(MAKE_META(TypeA, INT, x, true));
      M.push_back(MAKE_META(TypeA, BOOL, y, true));
      M.push_back(MAKE_META(TypeA, FLOAT, z, true));
      return M;
   };
   static vector<MemberMetaData> meta = make_it();
   return meta;
}

//
// Now we have:
//
void some_fun()
{
   TypeA x;
   Json::Value o = x.to_json();
   cout << str(x) << endl;
   x.read(o);
}

// @see `struct-meta_tc.cpp` for more examples

*/

namespace perceive
{
// ------------------------------------------------------------- Meta Compatible
//
enum class meta_type : int {
   BOOL = 0,
   INT,
   UNSIGNED,
   FLOAT,
   REAL,
   TIMESTAMP,
   STRING,
   JSON_VALUE,
   AABB,
   VECTOR2,
   VECTOR3,
   VECTOR4,
   VECTOR4F,
   COMPATIBLE_OBJECT
};

struct MetaCompatible;

// -------------------------------------------------------------- MemberMetaData
//
struct MemberMetaData
{
   using get_fun = std::function<std::any(const void*)>;
   using set_fun = std::function<void(void*, const std::any&)>;

   CUSTOM_NEW_DELETE(MemberMetaData)

 private:
   template<typename T1, typename T2> inline int offset_of(T1 T2::*member)
   {
      if constexpr(std::is_literal_type<T2>::value) {
         constexpr T2 object{};
         return int(size_t(&(object.*member)) - size_t(&object));
      } else {
         static T2 object{};
         return int(size_t(&(object.*member)) - size_t(&object));
      }
   }

   meta_type type_       = meta_type::BOOL;
   int offset_           = -1;
   get_fun getter_       = nullptr; // Uses 'offset' if nullptr
   set_fun setter_       = nullptr; // Uses 'offset' if nullptr
   std::string name_     = ""s;
   bool important_in_eq_ = true;

   const void* get_ptr_(const void* o) const noexcept;
   void* set_ptr_(void* o) const noexcept;

   MemberMetaData(meta_type t,
                  string member_name,
                  bool eq,
                  int off    = -1,
                  get_fun gf = nullptr,
                  set_fun sf = nullptr) noexcept;

 public:
   MemberMetaData(meta_type t, string name, bool eq, get_fun gf, set_fun sf)
       : MemberMetaData(t, name, eq, -1, gf, sf)
   {}

   template<typename T1, typename T2>
   MemberMetaData(meta_type t,
                  string name,
                  bool eq,
                  T1 T2::*member,
                  get_fun gf = nullptr,
                  set_fun sf = nullptr)
       : MemberMetaData(t, name, eq, offset_of(member), gf, sf)
   {
#ifdef DEBUG_BUILD
      // Prefer std::any over 'offset', for type safty
      if(getter_ == nullptr and type() != meta_type::COMPATIBLE_OBJECT) {
         getter_ = [this, member](const void* o) -> std::any {
            const T2* x = reinterpret_cast<const T2*>(o);
            try {
               return std::any(x->*member);
            } catch(std::bad_any_cast& e) {
               LOG_ERR(format("bad any cast reading '{}'", this->name()));
               throw e;
            }
         };
      }

      if(setter_ == nullptr and type() != meta_type::COMPATIBLE_OBJECT) {
         setter_ = [this, member](void* o, const std::any& val) {
            T2* x = reinterpret_cast<T2*>(o);
            try {
               x->*member = std::any_cast<T1>(val);
            } catch(std::bad_any_cast& e) {
               LOG_ERR(format("bad any cast writing to '{}'", this->name()));
               throw e;
            }
         };
      }
#endif
   }

   // Getters
   meta_type type() const noexcept { return type_; }
   const string& name() const noexcept { return name_; }
   bool important_in_eq() const noexcept { return important_in_eq_; }

   // Operations
   bool eq(const void* u, const void* v) const noexcept(false);
   std::ostream& to_stream(const void* x, std::ostream& ss, int indent) const
       noexcept(false);
   Json::Value to_json(const void* u) const noexcept(false);
   void read(void* x, const Json::Value& o) const noexcept(false);
   void read2(void* x,
              const Json::Value& o,
              const string_view path,
              const bool print_warnings) const noexcept(false);
};

#define MAKE_META(clazz, type, member, eq)            \
   {                                                  \
      meta_type::type, #member##s, eq, &clazz::member \
   }

// ------------------------------------------------------------- Meta Compatible
//
struct MetaCompatible
{
 public:
   virtual ~MetaCompatible() {}

   virtual const vector<MemberMetaData>& meta_data() const noexcept
   {
      Expects(false);
      static vector<MemberMetaData> meta;
      return meta;
   }

   bool operator==(const MetaCompatible& o) const noexcept;
   bool operator!=(const MetaCompatible& o) const noexcept;
   std::ostream& to_stream(std::ostream& ss, int indent = 0) const noexcept;
   string to_string(int indent = 0) const noexcept;
   string to_json_string(int indent = 0) const noexcept;
   Json::Value to_json() const noexcept;
   void read(const Json::Value& val) noexcept(false);

   void read_with_defaults(const Json::Value& val,
                           const MetaCompatible* defaults,
                           const bool print_warnings = true,
                           const string_view path    = ""s);

   // read_with_defaults<Params>(json_object);
   template<typename T>
   void read_with_defaults(const Json::Value& val,
                           const bool print_warnings = true,
                           const string_view path    = ""s)
   {
      T defaults;
      read_with_defaults(val, &defaults, print_warnings, path);
   }

   friend string str(const MetaCompatible& o) noexcept { return o.to_string(); }
};

// ---------------------------------------------------- Getting Member Meta Data
//

namespace detail
{
   bool eq_with_meta(const vector<MemberMetaData>& meta_data,
                     const void* a,
                     const void* b) noexcept;

   std::ostream& to_stream_with_meta(std::ostream& out,
                                     const vector<MemberMetaData>& meta_data,
                                     const void* o,
                                     int indent) noexcept;

   Json::Value to_json_with_meta(const vector<MemberMetaData>& meta_data,
                                 const void* o) noexcept;

   void read_json_with_meta(const vector<MemberMetaData>& meta_data,
                            void* x,
                            const Json::Value& o) noexcept(false);

} // namespace detail

template<typename T> bool eq_with_meta(const T& a, const T& b) noexcept
{
   if constexpr(std::is_base_of<T, MetaCompatible>::value) {
      return a == b;
   } else {
      return detail::eq_with_meta(T::meta_data(), &a, &b);
   }
}

template<typename T>
std::ostream&
to_stream_with_meta(std::ostream& ss, const T& o, int indent = 0) noexcept
{
   if constexpr(std::is_base_of<T, MetaCompatible>::value) {
      return o.to_stream(ss, indent);
   } else {
      return detail::to_stream_with_meta(ss, T::meta_data(), &o, indent);
   }
}

template<typename T> string str_with_meta(const T& o, int indent = 0) noexcept
{
   std::stringstream ss{""};
   to_stream_with_meta(ss, o, indent);
   return ss.str();
}

template<typename T> Json::Value to_json_with_meta(const T& o) noexcept
{
   if constexpr(std::is_base_of<T, MetaCompatible>::value) {
      return o.to_json();
   } else {
      return detail::to_json_with_meta(T::meta_data(), &o);
   }
}

template<typename T>
void read_json_with_meta(const T& x, Json::Value& o) noexcept(false)
{
   if constexpr(std::is_base_of<T, MetaCompatible>::value) {
      x.read(o);
   } else {
      detail::read_json_with_meta(T::meta_data(), &x, o);
   }
}

#define META_READ_WRITE_LOAD_SAVE(TYPE_)                                    \
   inline void read(TYPE_& data, const Json::Value& node) noexcept(false)   \
   {                                                                        \
      TYPE_ defaults;                                                       \
      data.read_with_defaults(node, &defaults);                             \
   }                                                                        \
   inline void read(TYPE_& data, const std::string& in) noexcept(false)     \
   {                                                                        \
      read(data, parse_json(in));                                           \
   }                                                                        \
   inline void write(const TYPE_& data, std::string& out) noexcept(false)   \
   {                                                                        \
      out = data.to_json_string();                                          \
   }                                                                        \
   inline void write(const TYPE_& data, Json::Value& node) noexcept(false)  \
   {                                                                        \
      node = data.to_json();                                                \
   }                                                                        \
   inline void load(TYPE_& data, const string& fname) noexcept(false)       \
   {                                                                        \
      read(data, file_get_contents(fname));                                 \
   }                                                                        \
   inline void save(const TYPE_& data, const string& fname) noexcept(false) \
   {                                                                        \
      std::string s;                                                        \
      write(data, s);                                                       \
      file_put_contents(fname, s);                                          \
   }

} // namespace perceive
