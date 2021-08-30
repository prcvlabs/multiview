
#include "s3.hpp"
#include "stdinc.hpp"

#include "perceive/utils/file-system.hpp"

#include <deque>
#include <filesystem>
#include <iterator>
#include <mutex>
#include <regex>
#include <stdlib.h>

#include <aws/core/Aws.h>
#include <aws/s3/S3Client.h>
#include <aws/s3/model/DeleteObjectRequest.h>
#include <aws/s3/model/GetObjectAclRequest.h>
#include <aws/s3/model/GetObjectRequest.h>
#include <aws/s3/model/HeadObjectRequest.h>
#include <aws/s3/model/ListObjectsRequest.h>
#include <aws/s3/model/PutObjectRequest.h>

// export AWS_PROFILE=multiview
// aws s3 ls s3://perceive-calibration
// aws s3 cp README.md s3://perceive-calibration/
// aws s3 cp s3://perceive-calibration/README.md ~

namespace perceive
{
static std::once_flag ensure_aws_flag;
static Aws::SDKOptions options;
static std::regex s3_path_re;

static void aws_destroy() { Aws::ShutdownAPI(options); }

static void aws_init()
{
   if(setenv("AWS_PROFILE", "multiview", 1) != 0)
      FATAL(format("Failed to setenv('AWS_PROFILE', 'multiview')"));

   s3_path_re = std::regex("s3://([^/]*)/(.*)");

   // options.loggingOptions.logLevel = Aws::Utils::Logging::LogLevel::Info;
   Aws::InitAPI(options);
   std::atexit(aws_destroy);
}

static std::pair<Aws::String, Aws::String>
to_bucket_key(const string_view s3_path)
{
   std::smatch pieces_match;
   string s3_path_s = s3_path.data();
   if(!std::regex_match(s3_path_s, pieces_match, s3_path_re))
      throw std::runtime_error(format("failed to parse s3 '{}'", s3_path));
   Expects(pieces_match.size() == 3);

   Aws::String bucket_name = pieces_match[1].str().c_str();
   Aws::String key_name    = pieces_match[2].str().c_str();

   return std::pair<Aws::String, Aws::String>(bucket_name, key_name);
}

// ----------------------------------------------------------------- Ensure Init

void ensure_aws_is_init() { std::call_once(ensure_aws_flag, aws_init); }

// --------------------------------------------------------------------- s3 Load

void s3_load(const string_view s3_path, std::vector<char>& out) noexcept(false)
{
   ensure_aws_is_init();
   // parse s3_path
   auto [bucket_name, key_name] = to_bucket_key(s3_path);

   Aws::S3::S3Client s3_client;
   Aws::S3::Model::GetObjectRequest request;
   request.WithBucket(bucket_name).WithKey(key_name);

   auto outcome = s3_client.GetObject(request);

   // INFO(format("s3-path = '{}'", s3_path));

   if(outcome.IsSuccess()) {
      using ibuf_itr = std::istreambuf_iterator<char>;
      auto ii        = ibuf_itr{outcome.GetResult().GetBody()};
      out.clear();
      out.insert(out.end(), ii, ibuf_itr{});
   } else {
      // std::stringstream ss("");
      auto msg = format("GetObject error retrieving '{}': {} {}",
                        s3_path,
                        outcome.GetError().GetExceptionName(),
                        outcome.GetError().GetMessage());
      // ss << "GetObject error: " << outcome.GetError().GetExceptionName() << "
      // "
      //    << outcome.GetError().GetMessage();
      // auto msg = ss.str();
      throw std::runtime_error(msg);
   }
}

void s3_load(const string_view s3_path, std::string& out) noexcept(false)
{
   std::vector<char> v;
   s3_load(s3_path, v);
   out.resize(v.size());
   std::copy(v.begin(), v.end(), out.begin());
}

// -------------------------------------------------------------------- s3 Store

void s3_store(const string_view s3_path, const std::vector<char>& in)
{
   ensure_aws_is_init();

   // parse s3_path
   auto [bucket_name, key_name] = to_bucket_key(s3_path);

   Aws::S3::S3Client s3_client;
   Aws::S3::Model::PutObjectRequest request;
   request.WithBucket(bucket_name).WithKey(key_name);

   // Cannot figure out how to read input into an object-request.
   // Don't have time to figure it out. So this can be improved
   // to be in-memory
   char buffer[256];
   snprintf(buffer, 256, "/tmp/multiview-s3-out.XXXXXX");
   // {
   //    int desc = mkstemp(buffer);
   //    close(desc);
   // }

   // Use RAII to ensure temporary file is deleted
   struct FileUnlinker
   {
      const char* fname_{nullptr};
      FileUnlinker(const char* fname)
          : fname_(fname)
      {}
      ~FileUnlinker()
      {
         if(fname_) delete_file(fname_);
      }
   };
   FileUnlinker funlinker(buffer);

   // Copy input data to the temporary file
   std::ofstream ofs(buffer, std::ios::out | std::ios::binary);
   std::copy(cbegin(in), cend(in), std::ostreambuf_iterator<char>(ofs));
   ofs.close();

   // Finish PutObjectRequest
   auto fdat = Aws::MakeShared<Aws::FStream>(
       "PutObjStream", buffer, std::ios_base::in | std::ios_base::binary);
   request.SetBody(fdat);

   auto outcome = s3_client.PutObject(request);

   if(outcome.IsSuccess()) {
      // We're good
   } else {
      std::stringstream ss("");
      ss << "SetObject error: " << outcome.GetError().GetExceptionName() << " "
         << outcome.GetError().GetMessage();
      auto msg = ss.str();
      throw std::runtime_error(msg);
   }
}

void s3_store(const string_view s3_path, const string& in) noexcept(false)
{
   vector<char> dat(in.size());
   std::copy(cbegin(in), cend(in), begin(dat));
   s3_store(s3_path, dat);
}

// ---------------------------------------------------------------------- Exists

bool s3_exists(const string_view s3_path) noexcept(false)
{
   ensure_aws_is_init();

   // parse s3_path
   auto [bucket_name, key_name] = to_bucket_key(s3_path);

   Aws::S3::S3Client s3_client;

   {
      Aws::S3::Model::HeadObjectRequest request;
      request.WithBucket(bucket_name).WithKey(key_name);

      auto outcome = s3_client.HeadObject(request);

      if(outcome.IsSuccess()) {
         return true;
      } else {
         return false;
      }
   }
}

// ---------------------------------------------------------------------- Remove

void s3_remove(const string_view s3_path) noexcept(false)
{
   ensure_aws_is_init();

   // parse s3_path
   auto [bucket_name, key_name] = to_bucket_key(s3_path);

   Aws::S3::S3Client s3_client;
   Aws::S3::Model::DeleteObjectRequest request;
   request.WithBucket(bucket_name).WithKey(key_name);

   auto outcome = s3_client.DeleteObject(request);

   if(outcome.IsSuccess()) {
      // std::cout << "Done!" << std::endl;

   } else {
      std::stringstream ss("");
      ss << "DeleteObjects error: " << outcome.GetError().GetExceptionName()
         << " " << outcome.GetError().GetMessage();
      auto msg = ss.str();
      throw std::runtime_error(msg);
   }
}

// ------------------------------------------------------------------- is s3 uri

bool is_s3_uri(const string_view uri) noexcept
{
   return starts_with(uri, "s3://"s);
}

// ------------------------------------------------------------------ s3 dirname

string s3_dirname(const string_view uri) noexcept
{
   static const char* prefix = "s3://";
   if(!is_s3_uri(uri)) return dirname(uri);

   namespace fs = std::filesystem;
   const auto s
       = fs::path(std::string(&uri[strlen(prefix)])).parent_path().string();

   return format("{}{}", prefix, s);
}

} // namespace perceive
