
#define CATCH_CONFIG_PREFIX_ALL

#include "perceive/contrib/catch.hpp"
#include "perceive/io/json-io.hpp"
#include "perceive/io/s3.hpp"
#include "perceive/utils/md5.hpp"

namespace perceive
{
// ------------------------------------------------------------- s3-settings-for
//
CATCH_TEST_CASE("S3SettingsFor", "[s3]")
{
   CATCH_SECTION("r2 endpoint")
   {
      const auto s
          = s3_settings_for("https://acct.r2.cloudflarestorage.com", "", true);
      CATCH_REQUIRE(s.endpoint == "https://acct.r2.cloudflarestorage.com");
      CATCH_REQUIRE(s.region == "auto");
      CATCH_REQUIRE(s.use_https);
      CATCH_REQUIRE(s.path_style);
      CATCH_REQUIRE(s.checksums_when_required);
      CATCH_REQUIRE(s.disable_imds);
   }

   CATCH_SECTION("minio endpoint")
   {
      const auto s
          = s3_settings_for("http://127.0.0.1:9000", "us-east-1", false);
      CATCH_REQUIRE(s.endpoint == "http://127.0.0.1:9000");
      CATCH_REQUIRE(s.region == "us-east-1");
      CATCH_REQUIRE(!s.use_https);
      CATCH_REQUIRE(!s.path_style);
      CATCH_REQUIRE(s.checksums_when_required);
      CATCH_REQUIRE(s.disable_imds);
   }

   CATCH_SECTION("aws defaults")
   {
      const auto s = s3_settings_for("", "", false);
      CATCH_REQUIRE(s.endpoint.empty());
      CATCH_REQUIRE(s.region.empty());
      CATCH_REQUIRE(!s.use_https);
      CATCH_REQUIRE(!s.path_style);
      CATCH_REQUIRE(!s.checksums_when_required);
      CATCH_REQUIRE(!s.disable_imds);
   }
}

// ---------------------------------------------------- s3-environment-variables
//
CATCH_TEST_CASE("S3EnvironmentVariables", "[s3]")
{
   const Json::Value original = parse_json(environment_json_str());

   auto path_style_with
       = [&](const string_view endpoint, const string_view path_style) {
            Json::Value o                = original;
            o["MULTIVIEW_S3_ENDPOINT"]   = string(endpoint);
            o["MULTIVIEW_S3_PATH_STYLE"] = string(path_style);
            set_environment_variables(o);
            const bool ret = multiview_s3_path_style();
            set_environment_variables(original);
            return ret;
         };

   CATCH_SECTION("path style defaults to on with an endpoint")
   {
      CATCH_REQUIRE(path_style_with("http://127.0.0.1:9000", ""));
      CATCH_REQUIRE(!path_style_with("http://127.0.0.1:9000", "0"));
      CATCH_REQUIRE(path_style_with("http://127.0.0.1:9000", "true"));
      CATCH_REQUIRE(!path_style_with("", ""));
      CATCH_REQUIRE(path_style_with("", "1"));
   }

   CATCH_SECTION("registered, without credentials")
   {
      const auto s = environment_json_str();
      for(const auto key : {"MULTIVIEW_S3_ENDPOINT",
                            "MULTIVIEW_S3_REGION",
                            "MULTIVIEW_S3_PATH_STYLE",
                            "MULTIVIEW_S3_PROFILE"})
         CATCH_REQUIRE(original.isMember(key));
      CATCH_REQUIRE(s.find("AWS_") == string::npos);
   }
}

// ------------------------------------------------------------------ s3-live-io
//
// Needs a bucket: MULTIVIEW_S3_TEST_PREFIX=s3://<bucket>/<prefix>, with the
// endpoint in MULTIVIEW_S3_ENDPOINT and the keys in AWS_ACCESS_KEY_ID and
// AWS_SECRET_ACCESS_KEY (a MinIO container will do).
CATCH_TEST_CASE("S3LiveIO", "[s3-live]")
{
   const char* prefix = getenv("MULTIVIEW_S3_TEST_PREFIX");
   if(prefix == nullptr || !is_s3_uri(prefix)) {
      WARN("MULTIVIEW_S3_TEST_PREFIX is not set, skipping the s3 round trip");
      return;
   }

   std::mt19937 gen(42);
   std::uniform_int_distribution<int> dist(0, 255);
   vector<char> in(3 * 1024 * 1024 + 17);
   for(auto& c : in) c = char(dist(gen));

   const auto uri = format("{}/s3-live-io.bin", prefix);
   s3_store(uri, in);
   CATCH_REQUIRE(s3_exists(uri));

   vector<char> out;
   s3_load(uri, out);
   CATCH_REQUIRE(out.size() == in.size());
   CATCH_REQUIRE(md5(out.data(), out.size()) == md5(in.data(), in.size()));

   s3_remove(uri);
   CATCH_REQUIRE(!s3_exists(uri));
}

} // namespace perceive
