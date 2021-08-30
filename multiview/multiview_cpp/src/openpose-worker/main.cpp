
#include <memory>
#include <stdio.h>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

#include "op-inc.hpp"

#include "perceive/utils/cuda-spec.hpp"

using std::cerr;
using std::cout;
using std::endl;
using std::string;
using std::vector;

using perceive::format;
using perceive::tick;
using perceive::tock;

// ----------------------------------------------------------- tutorial-thread-1
//

struct OpData
{
   vector<cv::Mat> ims;
   vector<string> fnames;

   OpData(vector<string> in_fnames)
   {
      fnames = std::move(in_fnames);
      ims.resize(fnames.size());
      INFO(format("loading {} pngs", fnames.size()));
      std::transform(cbegin(fnames),
                     cend(fnames),
                     begin(ims),
                     [&](const auto& s) { return cv::imread(s); });

      if(false) {
         int counter = 0;
         for(const auto& im : ims)
            cv::imwrite(format("/tmp/image-{:2d}.png", counter++), im);
      }
   }

   size_t size() const noexcept { return fnames.size(); }
};

// ---------------------------------------------------------------------- run it
//
double run_it(const vector<string>& fnames)
{
   printf("openpose thingy\n");
   OpData dat(fnames);
   ::perceive::OpExec op;
   ::perceive::OpExec::Params params;

   decltype(tick()) now = tick();
   for(auto i = 0u; i < dat.size(); ++i) {
      if(i == 1) now = tick(); // Skip the first frame
      const auto ret = op.run(dat.ims[i], params);
      // cout << str(ret) << endl;
   }

   const double seconds = tock(now);

   return (dat.size() > 1) ? (seconds / (dat.size() - 1)) : seconds;
}

// ------------------------------------------------------------------------ main
//
int main(int argc, char** argv)
{
   printf("Hello World!\n");

   ::perceive::cuda::init_cuda();
   ::perceive::cuda::print_cuda_report();

   vector<string> fnames;
   for(int i = 1; i < argc; ++i) fnames.push_back(argv[i]);

   const auto now         = ::perceive::tick();
   const auto per_frame_s = run_it(fnames);
   const auto seconds     = ::perceive::tock(now);

   printf(R"V0G0N(

  DONE 

  N             = {}
  total-seconds = {}ms
  per-frame     = {}ms 

)V0G0N",
          int(fnames.size()),
          double(seconds * 1000.0),
          double(per_frame_s * 1000.0));

   return 0;
}
