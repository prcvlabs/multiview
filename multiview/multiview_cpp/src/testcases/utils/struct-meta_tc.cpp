
#include <any>
#include <set>

#define CATCH_CONFIG_PREFIX_ALL
#include "perceive/contrib/catch.hpp"

#include "perceive/io/struct-meta.hpp"

static const bool feedback = false;

namespace perceive
{
struct TypeA
{
   int x   = -1;
   bool y  = false;
   float z = 1.0f;
   std::set<string> roles;
   static const vector<MemberMetaData>& meta_data() noexcept;
};

const vector<MemberMetaData>& TypeA::meta_data() noexcept
{
   auto make_it = []() {
      vector<MemberMetaData> M;
      M.push_back(MAKE_META(TypeA, INT, x, true));
      M.push_back(MAKE_META(TypeA, BOOL, y, true));
      M.push_back(MAKE_META(TypeA, FLOAT, z, true));
      M.emplace_back(meta_type::STRING,
                     "roles"s,
                     true,
                     [](const void* p) -> std::any {
                        auto& o = *reinterpret_cast<const TypeA*>(p);
                        return std::any(
                            implode(cbegin(o.roles), cend(o.roles), " "));
                     },
                     [](void* p, const std::any& x) -> void {
                        auto& o = *reinterpret_cast<TypeA*>(p);
                        std::stringstream ss(std::any_cast<string>(x));
                        o.roles.clear();
                        string val; // tokenize into 'val'
                        while(getline(ss, val, ' ')) {
                           trim(val);
                           if(!val.empty()) o.roles.insert(val);
                        }
                     });
      return M;
   };
   static vector<MemberMetaData> meta = make_it();
   return meta;
}

struct TypeB : public MetaCompatible
{
   int x   = -1;
   bool y  = false;
   float z = 1.0f;

   const vector<MemberMetaData>& meta_data() const noexcept override
   {
      auto make_it = []() {
         vector<MemberMetaData> M;
         M.emplace_back(meta_type::INT, "x", true, &TypeB::x);
         M.emplace_back(meta_type::BOOL, "y", true, &TypeB::y);
         M.emplace_back(meta_type::FLOAT, "z", true, &TypeB::z);
         return M;
      };
      static vector<MemberMetaData> meta = make_it();
      return meta;
   }
};

struct TypeC
{
   int x   = -1;
   bool y  = false;
   float z = 1.0f;
   TypeB blah;

   static const vector<MemberMetaData>& meta_data() noexcept
   {
      auto make_it = []() {
         vector<MemberMetaData> M;
         M.emplace_back(meta_type::INT, "x", true, &TypeC::x);
         M.emplace_back(meta_type::BOOL, "y", true, &TypeC::y);
         M.emplace_back(meta_type::FLOAT, "z", true, &TypeC::z);
         M.push_back(MAKE_META(TypeC, COMPATIBLE_OBJECT, blah, true));
         return M;
      };
      static vector<MemberMetaData> meta = make_it();
      return meta;
   }
};

CATCH_TEST_CASE("StructMeta", "[struct_meta]")
{
   CATCH_SECTION("struct-meta")
   {
      { //
         TypeA a, b;
         a.roles.insert("hello");
         a.roles.insert("world");
         b.roles.insert("world");
         b.roles.insert("hello");
         CATCH_REQUIRE(eq_with_meta(a, b));

         // cout << str_with_meta(a) << endl;
      }

      {
         TypeB a, b;
         CATCH_REQUIRE(a == b);

         // cout << str(a) << endl;

         auto json = a.to_json();
         // cout << "json = " << json << endl;
         json["x"] = 10;
         b.read(json);
         // cout << str(b) << endl;
      }

      {
         TypeC a, b;
         // CATCH_REQUIRE(eq_with_meta(a, b));

         // cout << str_with_meta(a) << endl;
      }
   }
}

} // namespace perceive
