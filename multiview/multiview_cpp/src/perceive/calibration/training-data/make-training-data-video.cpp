
#include "make-training-data-video.hpp"

#include "perceive/graphics/image-container.hpp"
#include "perceive/movie/ffmpeg.hpp"
#include "perceive/utils/file-system.hpp"

#include <filesystem>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace perceive
{
static void
recursive_directory_pass(string_view directory,
                         std::function<void(string_view)> f) noexcept
{
   namespace fs = std::filesystem;
   const string dir(begin(directory), end(directory));
   const auto opts = fs::directory_options::follow_directory_symlink;
   for(auto& file : fs::recursive_directory_iterator(dir, opts))
      if(fs::is_regular_file(file)) f(file.path().string());
}

void finish_training_data_export(string_view directory,
                                 real frame_rate,
                                 string_view outfile_base)
{
   constexpr auto k_png_ext  = string_view(".png");
   constexpr auto k_text_ext = string_view(".text");

   vector<string> png_fnames;
   vector<string> text_fnames;

   recursive_directory_pass(directory, [&](string_view fname) {
      if(ends_with(fname, k_png_ext))
         png_fnames.emplace_back(begin(fname), end(fname));
      else if(ends_with(fname, k_text_ext))
         text_fnames.emplace_back(begin(fname), end(fname));
   });

   std::sort(begin(png_fnames), end(png_fnames));
   std::sort(begin(text_fnames), end(text_fnames));

   if(png_fnames.size() != text_fnames.size()) {
      FATAL(format("found {} png files, and {} text files!",
                   png_fnames.size(),
                   text_fnames.size()));
   }

   if(png_fnames.size() == 0) return; // nothing to do

   const auto argb0       = ARGBImage::load(png_fnames.front());
   const auto width       = int(argb0.width);
   const auto height      = int(argb0.height);
   const auto outfile_mp4 = format("{}/{}.mp4", directory, outfile_base);
   const auto outfile_data
       = format("{}/{}.annotations", directory, outfile_base);

   try {
      auto encoder = movie::StreamingFFMpegEncoder::create(
          outfile_mp4, width, height, frame_rate);
      for(const auto& fname : png_fnames)
         encoder.push_frame(cv::imread(fname, cv::IMREAD_COLOR));
      encoder.close();
   } catch(std::exception& e) {
      FATAL(format("exception creating training mp4: {}", e.what()));
   }

   try {
      std::stringstream ss{""};
      for(const auto& fname : text_fnames)
         ss << trim_copy(file_get_contents(fname)) << '\n';
      file_put_contents(outfile_data, ss.str());
   } catch(std::exception& e) {
      FATAL(format("exception creating annotation file: {}", e.what()));
   }

   // Remove all files
   for(const auto& fname : png_fnames) delete_file(fname);
   for(const auto& fname : text_fnames) delete_file(fname);
}

} // namespace perceive
