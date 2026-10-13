
#define CATCH_CONFIG_PREFIX_ALL
#include "perceive/contrib/catch.hpp"

#include "perceive/cost-functions/pose-skeleton/engine-cache.hpp"
#include "perceive/utils/file-system.hpp"
#include "perceive/utils/md5.hpp"

#include <filesystem>

namespace perceive
{
// --------------------------------------------------------------- make-test-key
//
static EngineKey make_test_key() noexcept
{
   EngineKey key;
   key.model_md5      = md5("onnx model"s);
   key.max_batch_size = 6;
   key.input_h        = 368;
   key.input_w        = 656;
   key.precision      = "fp32"s;
   key.trt_version    = 101600;
   key.gpu_name       = "NVIDIA GeForce RTX 3090"s;
   key.compute_major  = 8;
   key.compute_minor  = 6;
   return key;
}

// ------------------------------------------------------------- make-other-keys
// Keys that differ from `make_test_key()` in one field each
static vector<EngineKey> make_other_keys() noexcept
{
   vector<EngineKey> out;
   auto push_key = [&](std::function<void(EngineKey&)> f) {
      EngineKey key = make_test_key();
      f(key);
      out.push_back(key);
   };
   push_key([](EngineKey& o) { o.model_md5 = md5("other onnx model"s); });
   push_key([](EngineKey& o) { o.max_batch_size = 12; });
   push_key([](EngineKey& o) { o.input_h = 384; });
   push_key([](EngineKey& o) { o.input_w = 432; });
   push_key([](EngineKey& o) { o.precision = "fp16"s; });
   push_key([](EngineKey& o) { o.trt_version = 101500; });
   push_key([](EngineKey& o) { o.gpu_name = "NVIDIA GeForce RTX 4090"s; });
   push_key([](EngineKey& o) { o.compute_major = 9; });
   push_key([](EngineKey& o) { o.compute_minor = 9; });
   return out;
}

// ------------------------------------------------------ Hyperpose Engine Cache
// The key, the sidecar check, and the files, without TensorRT or a GPU
CATCH_TEST_CASE("HyperposeEngineCache", "[hyperpose-cache]")
{
   CATCH_SECTION("engine-key")
   {
      const auto key    = make_test_key();
      const auto digest = key.digest();
      CATCH_REQUIRE(extract_md5_hexdigest(digest) == digest);
      CATCH_REQUIRE(make_test_key().digest() == digest);

      const auto others = make_other_keys();
      CATCH_REQUIRE(others.size() == key.meta_data().size());
      for(const auto& o : others) CATCH_REQUIRE(o.digest() != digest);
   }

   CATCH_SECTION("engine-sidecar")
   {
      const auto key      = make_test_key();
      const auto plan_md5 = md5("plan"s);
      const auto sidecar  = make_engine_sidecar(key, plan_md5);

      CATCH_REQUIRE(engine_sidecar_matches(sidecar, key, plan_md5));
      CATCH_REQUIRE(!engine_sidecar_matches(sidecar, key, md5("other plan"s)));
      for(const auto& o : make_other_keys())
         CATCH_REQUIRE(!engine_sidecar_matches(sidecar, o, plan_md5));

      auto edit_sidecar = [&](std::function<void(Json::Value&)> f) {
         auto o = parse_json(sidecar);
         f(o);
         std::stringstream ss{""};
         ss << o;
         return ss.str();
      };

      for(const auto& m : key.meta_data()) {
         const auto s
             = edit_sidecar([&](auto& o) { o.removeMember(m.name()); });
         CATCH_REQUIRE(!engine_sidecar_matches(s, key, plan_md5));
      }

      const auto s0 = edit_sidecar([](auto& o) { o.removeMember("plan_md5"); });
      const auto s1 = edit_sidecar([](auto& o) { o["max_batch_size"] = "6"; });
      CATCH_REQUIRE(!engine_sidecar_matches(s0, key, plan_md5));
      CATCH_REQUIRE(!engine_sidecar_matches(s1, key, plan_md5));

      // Malformed json
      const auto half = sidecar.substr(0, sidecar.size() / 2);
      CATCH_REQUIRE(!engine_sidecar_matches(half, key, plan_md5));
      CATCH_REQUIRE(!engine_sidecar_matches(""s, key, plan_md5));
      CATCH_REQUIRE(!engine_sidecar_matches("[]"s, key, plan_md5));
   }

   CATCH_SECTION("engine-cache")
   {
      namespace fs = std::filesystem;

      const auto dir = make_temp_directory("/tmp/engine-cache_tc.XXXXXX");
      const auto key = make_test_key();
      auto save
          = [](const string& fname) { file_put_contents(fname, "plan"s); };
      auto n_files = [&]() {
         return std::distance(fs::directory_iterator(dir),
                              fs::directory_iterator{});
      };

      CATCH_REQUIRE(find_cached_engine(dir, key) == ""s);
      const auto plan_fname = cache_engine(dir, key, save);
      CATCH_REQUIRE(plan_fname == format("{}/{}.plan", dir, key.digest()));
      CATCH_REQUIRE(file_get_contents(plan_fname) == "plan"s);
      CATCH_REQUIRE(n_files() == 2); // the plan and its sidecar
      CATCH_REQUIRE(find_cached_engine(dir, key) == plan_fname);
      for(const auto& o : make_other_keys())
         CATCH_REQUIRE(find_cached_engine(dir, o) == ""s);

      // A plan that does not match its sidecar
      file_put_contents(plan_fname, "corrupt plan"s);
      CATCH_REQUIRE(find_cached_engine(dir, key) == ""s);

      // Nothing is cached when the plan is not saved
      CATCH_REQUIRE(cache_engine(dir, key, [](const string&) {}) == ""s);
      CATCH_REQUIRE(n_files() == 2);
      CATCH_REQUIRE(find_cached_engine(dir, key) == ""s);

      CATCH_REQUIRE(cache_engine(dir, key, save) == plan_fname);
      CATCH_REQUIRE(find_cached_engine(dir, key) == plan_fname);

      remove_all(dir);
   }
}

} // namespace perceive
