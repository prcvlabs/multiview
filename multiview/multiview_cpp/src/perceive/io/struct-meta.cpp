
#include "struct-meta.hpp"

#include "perceive/io/json-io.hpp"
#include "perceive/utils/math.hpp"

#define This MemberMetaData

namespace perceive
{
const void* This::get_ptr_(const void* o) const noexcept
{
   return static_cast<const char*>(o) + offset_;
}

void* This::set_ptr_(void* o) const noexcept
{
   return static_cast<char*>(o) + offset_;
}

// ---------------------------------------------------------------- construction
//
This::This(meta_type t,
           std::string member_name,
           bool eq,
           int off,
           get_fun gf,
           set_fun sf) noexcept
    : type_(t)
    , offset_(off)
    , getter_(std::move(gf))
    , setter_(std::move(sf))
    , name_(std::move(member_name))
    , important_in_eq_(eq)
{
   Expects(getter_ != nullptr or offset_ >= 0);
   Expects(setter_ != nullptr or offset_ >= 0);
}

// -------------------------------------------------------------------------- eq
//
bool This::eq(const void* u, const void* v) const noexcept(false)
{
   if(!important_in_eq_) return true;
#define CASE_1(TYPE, t)                                            \
   case meta_type::TYPE:                                           \
      return (getter_) ? (std::any_cast<t>(getter_(u))             \
                          == std::any_cast<t>(getter_(v)))         \
                       : (*reinterpret_cast<const t*>(get_ptr_(u)) \
                          == *reinterpret_cast<const t*>(get_ptr_(v)))

#define CASE_2(TYPE, t)                                                        \
   case meta_type::TYPE:                                                       \
      return (getter_) ? is_close<t>(std::any_cast<t>(getter_(u)),             \
                                     std::any_cast<t>(getter_(v)))             \
                       : is_close<t>(*reinterpret_cast<const t*>(get_ptr_(u)), \
                                     *reinterpret_cast<const t*>(get_ptr_(v)))

   switch(type_) {
      CASE_1(BOOL, bool);
      CASE_1(INT, int);
      CASE_1(UNSIGNED, unsigned);
      CASE_2(FLOAT, float);
      CASE_2(REAL, real);
      CASE_1(TIMESTAMP, Timestamp);
      CASE_1(STRING, string);
      CASE_1(JSON_VALUE, Json::Value);
      CASE_1(AABB, AABB);
      CASE_1(VECTOR2, Vector2);
      CASE_1(VECTOR3, Vector3);
      CASE_1(VECTOR4, Vector4);
      CASE_1(VECTOR4F, Vector4f);
   case meta_type::COMPATIBLE_OBJECT:
      if(getter_) {
         return std::any_cast<const MetaCompatible&>(getter_(u))
                == std::any_cast<const MetaCompatible&>(getter_(v));
      }
      return reinterpret_cast<const MetaCompatible*>(get_ptr_(u))
          ->
          operator==(*reinterpret_cast<const MetaCompatible*>(get_ptr_(v)));
   }

#undef CASE_1
#undef CASE_2
}

// ------------------------------------------------------------------- to_stream
//
std::ostream& This::to_stream(const void* x, std::ostream& ss, int indent) const
    noexcept(false)
{
#define CASE_1(TYPE, t)                                               \
   case meta_type::TYPE:                                              \
      ss << ((getter_) ? (std::any_cast<const t>(getter_(x)))         \
                       : (*reinterpret_cast<const t*>(get_ptr_(x)))); \
      break

#define CASE_2(TYPE, t)                                                   \
   case meta_type::TYPE:                                                  \
      ss << json_encode(                                                  \
          str(((getter_) ? (std::any_cast<const t>(getter_(x)))           \
                         : (*reinterpret_cast<const t*>(get_ptr_(x)))))); \
      break

#define CASE_3(TYPE, t)                                                  \
   case meta_type::TYPE:                                                 \
      ss << str((getter_) ? (std::any_cast<const t>(getter_(x)))         \
                          : (*reinterpret_cast<const t*>(get_ptr_(x)))); \
      break

#ifdef DEBUG_BUILD
   try {
#endif
      switch(type_) {
         CASE_3(BOOL, bool);
         CASE_1(INT, int);
         CASE_1(UNSIGNED, unsigned);
         CASE_1(FLOAT, float);
         CASE_1(REAL, real);
         CASE_2(TIMESTAMP, Timestamp);
         CASE_2(STRING, string);
         CASE_1(JSON_VALUE, Json::Value);
         CASE_2(AABB, AABB);
         CASE_2(VECTOR2, Vector2);
         CASE_2(VECTOR3, Vector3);
         CASE_2(VECTOR4, Vector4);
         CASE_2(VECTOR4F, Vector4f);
      case meta_type::COMPATIBLE_OBJECT:
         if(getter_) {
            std::any_cast<const MetaCompatible&>(getter_(x))
                .to_stream(ss, indent);
         } else {
            reinterpret_cast<const MetaCompatible*>(get_ptr_(x))
                ->to_stream(ss, indent);
         }
         break;
      }
#ifdef DEBUG_BUILD
   } catch(std::bad_any_cast& e) {
      LOG_ERR(format("bad any cast at member '{}'", name()));
      throw e;
   }
#endif

#undef CASE_1
#undef CASE_2
   return ss;
}

// --------------------------------------------------------------------- to_json
//
template<typename T> Json::Value save_T(const T& x) { return json_save(x); }

Json::Value This::to_json(const void* x) const noexcept(false)
{
   int val = -2;
   Json::Value o;
#define CASE(TYPE, t)                                               \
   case meta_type::TYPE:                                            \
      return save_T<t>(((getter_)                                   \
                            ? (std::any_cast<const t&>(getter_(x))) \
                            : (*reinterpret_cast<const t*>(get_ptr_(x)))))

   switch(type_) {
      CASE(BOOL, bool);
      CASE(INT, int);
      CASE(UNSIGNED, unsigned);
      CASE(FLOAT, float);
      CASE(REAL, real);
      CASE(TIMESTAMP, Timestamp);
      CASE(STRING, string);
      CASE(AABB, AABB);
      CASE(VECTOR2, Vector2);
      CASE(VECTOR3, Vector3);
      CASE(VECTOR4, Vector4);
      CASE(VECTOR4F, Vector4f);
   case meta_type::JSON_VALUE:
      if(getter_) {
         return std::any_cast<Json::Value>(getter_(x));
      } else {
         return *reinterpret_cast<const Json::Value*>(get_ptr_(x));
      }
   case meta_type::COMPATIBLE_OBJECT:
      if(getter_) {
         return std::any_cast<const MetaCompatible&>(getter_(x)).to_json();
      } else {
         return reinterpret_cast<const MetaCompatible*>(get_ptr_(x))->to_json();
      }
      break;
   }
#undef CASE
   Expects(false);
   return Json::Value{Json::nullValue};
}

// ------------------------------------------------------------------- read_json
//
template<typename T> T load_T(const Json::Value& node)
{
   T x;
   json_load(node, x);
   return x;
}

void This::read(void* x, const Json::Value& o) const noexcept(false)
{
   return read2(x, o, ""s, false);
#define CASE(TYPE, t)                                      \
   case meta_type::TYPE:                                   \
      if(setter_)                                          \
         setter_(x, std::any(load_T<t>(o)));               \
      else                                                 \
         json_load(o, *reinterpret_cast<t*>(set_ptr_(x))); \
      break;

   switch(type_) {
      CASE(BOOL, bool);
      CASE(INT, int);
      CASE(UNSIGNED, unsigned);
      CASE(FLOAT, float);
      CASE(REAL, real);
      CASE(TIMESTAMP, Timestamp);
      CASE(STRING, string);
      CASE(AABB, AABB);
      CASE(VECTOR2, Vector2);
      CASE(VECTOR3, Vector3);
      CASE(VECTOR4, Vector4);
      CASE(VECTOR4F, Vector4f);
   case meta_type::JSON_VALUE:
      if(setter_)
         setter_(x, std::any(o));
      else
         reinterpret_cast<Json::Value*>(set_ptr_(x))->operator=(o);
      break;
   case meta_type::COMPATIBLE_OBJECT:
      if(setter_) { Expects(false); }
      reinterpret_cast<MetaCompatible*>(set_ptr_(x))->read(o);
      break;
   }
#undef CASE
}

void This::read2(void* x,
                 const Json::Value& o,
                 const string_view path,
                 const bool print_warnings) const noexcept(false)
{
#define CASE(TYPE, t)                                      \
   case meta_type::TYPE:                                   \
      if(setter_)                                          \
         setter_(x, std::any(load_T<t>(o)));               \
      else                                                 \
         json_load(o, *reinterpret_cast<t*>(set_ptr_(x))); \
      break;

   switch(type_) {
      CASE(BOOL, bool);
      CASE(INT, int);
      CASE(UNSIGNED, unsigned);
      CASE(FLOAT, float);
      CASE(REAL, real);
      CASE(TIMESTAMP, Timestamp);
      CASE(STRING, string);
      CASE(AABB, AABB);
      CASE(VECTOR2, Vector2);
      CASE(VECTOR3, Vector3);
      CASE(VECTOR4, Vector4);
      CASE(VECTOR4F, Vector4f);
   case meta_type::JSON_VALUE:
      if(setter_)
         setter_(x, std::any(o));
      else
         reinterpret_cast<Json::Value*>(set_ptr_(x))->operator=(o);
      break;
   case meta_type::COMPATIBLE_OBJECT:
      if(setter_) { Expects(false); }
      reinterpret_cast<MetaCompatible*>(set_ptr_(x))
          ->read_with_defaults(o, nullptr, print_warnings, path);
      break;
   }

#undef CASE
}

// ----------------------------------------------------- MetaCompatible::to-json
//
namespace detail
{
   bool eq_with_meta(const vector<MemberMetaData>& meta_data,
                     const void* a,
                     const void* b) noexcept
   {
      if(a == b) return true;
      try {
         for(const auto& m : meta_data)
            if(!m.eq(a, b)) return false;
         return true;
      } catch(...) {}
      return false;
   }

   std::ostream& to_stream_with_meta(std::ostream& ss,
                                     const vector<MemberMetaData>& meta_data,
                                     const void* o,
                                     int indent) noexcept
   {
      auto do_indent = [&](int val) {
         for(auto i = 0; i < (3 * val); ++i) ss << ' ';
      };

      ss << "{";
      bool first = true;
      for(const auto& m : meta_data) {
         if(first)
            first = false;
         else
            ss << ',';
         ss << '\n';
         do_indent(indent + 1);
         ss << '"' << m.name() << "\": ";
         m.to_stream(o, ss, indent + 1);
      }
      if(!first) {
         ss << '\n';
         do_indent(indent);
      }
      ss << "}";
      return ss;
   }

   Json::Value to_json_with_meta(const vector<MemberMetaData>& meta_data,
                                 const void* x) noexcept
   {
      try {
         Json::Value o{Json::objectValue};
         for(const auto& m : meta_data) {
            Expects(!has_key(o, m.name()));
            o[m.name()] = m.to_json(x);
         }
         return o;
      } catch(...) {}
      return Json::Value{Json::nullValue};
   }

   void read_json_with_meta(const vector<MemberMetaData>& meta_data,
                            void* x,
                            const Json::Value& o) noexcept(false)
   {
      for(const auto& m : meta_data) {
         if(!has_key(o, m.name()))
            throw std::runtime_error(
                format("failed to find key '{}'", m.name()));
         m.read(x, o[m.name()]);
      }
   }

   void
   read_json_with_meta_and_defaults(const vector<MemberMetaData>& meta_data,
                                    void* x,
                                    const Json::Value& o,
                                    const void* defaults,
                                    const string_view path,
                                    const bool print_warnings)
   {
      auto get_path = [&](const string_view name) {
         const char* delim = (path.size() == 0) ? "" : ".";
         return format("{}{}{}", path, delim, name);
      };

      for(const auto& m : meta_data) {
         bool use_default = false;
         bool has_error   = false;
         try {
            if(!has_key(o, m.name())) {
               has_error = true;
            } else {
               if(print_warnings) {
                  try {
                     m.read2(
                         x, o[m.name()], get_path(m.name()), print_warnings);
                  } catch(std::exception& e) {
                     has_error = true;
                  }
               } else {
                  m.read(x, o[m.name()]);
               }
            }
         } catch(std::exception& e) {
            has_error = true;
            if(!defaults) throw e;
         }

         if(has_error and defaults != nullptr) {
            auto val = m.to_json(defaults);
            m.read(x, val);
         }

         if(has_error and print_warnings and !k_is_testcase_build) {
            cout << format("{}WARNING{}: using default value for '{}'",
                           ANSI_COLOUR_YELLOW,
                           ANSI_COLOUR_RESET,
                           get_path(m.name()))
                 << endl;
         }
      }
   }

} // namespace detail

// -------------------------------------------------- MetaCompatible::operator==
//
bool MetaCompatible::operator==(const MetaCompatible& o) const noexcept
{
   if(this == &o) return true;
   const auto& A = meta_data();
   const auto& B = o.meta_data();
   if(&A != &B) return false;
   return detail::eq_with_meta(A, this, &o);
}

bool MetaCompatible::operator!=(const MetaCompatible& o) const noexcept
{
   return !(*this == o);
}

std::ostream& MetaCompatible::to_stream(std::ostream& ss, int indent) const
    noexcept
{
   return detail::to_stream_with_meta(ss, this->meta_data(), this, indent);
}

string MetaCompatible::to_json_string(int indent) const noexcept
{
   std::stringstream ss{""};
   to_stream(ss, indent);
   return ss.str();
}

string MetaCompatible::to_string(int indent) const noexcept
{
   return to_json_string();
}

Json::Value MetaCompatible::to_json() const noexcept
{
   return detail::to_json_with_meta(this->meta_data(), this);
}

void MetaCompatible::read(const Json::Value& val) noexcept(false)
{
   detail::read_json_with_meta(this->meta_data(), this, val);
}

void MetaCompatible::read_with_defaults(const Json::Value& val,
                                        const MetaCompatible* defaults,
                                        const bool print_warnings,
                                        const string_view path)
{
   detail::read_json_with_meta_and_defaults(
       this->meta_data(), this, val, defaults, path, print_warnings);
}

} // namespace perceive
