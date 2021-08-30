
#include "results_example.hpp"

#include "perceive/utils/file-system.hpp"

#define This FrameResults

namespace perceive::stereobm
{
This::This(const std::string& left_image, const std::string& right_image)
{
   // ---- NOTES ----
   // (*) We can load/save parameters from a combined JSON file
   // (*) Currently no plans for changing how 'subscribe' behaviour
   //     works. That means that you must manually update this
   //     graph everytime

   { // -------------------------------------------------------- Load image task
      const std::string* stereo_image[2] = {&left_image, &right_image};
      for(unsigned i = 0; i < 2; ++i) {
         input_image[i].set_taskname(format("load-image_[{}]", i));
         load_image::Params p;
         p.filename = *stereo_image[i];
         input_image[i].set_params(p);
      }
   }

   { // ---------------------------------------------------- Stereo BM CPU task
      stereobm_cpu.set_taskname(format("stereobm_cpu"));
      stereobm::Params p;
      p.implementation = stereobm::Params::CPU;
      stereobm_cpu.set_params(p);
      for(auto i = 0u; i < 2; ++i) { stereobm_cpu.subscribe(&input_image[i]); }
   }

   { // --------------------------------------------------- Stereo BM CUDA task
      stereobm_cuda.set_taskname(format("stereobm_cuda"));
      stereobm::Params p;
      p.implementation = stereobm::Params::CUDA;
      stereobm_cuda.set_params(p);
      // enforce that this runs after stereobm_cpu
      stereobm_cuda.subscribe(&stereobm_cpu);
      for(auto i = 0u; i < 2; ++i) { stereobm_cuda.subscribe(&input_image[i]); }
   }

   { // ------------------------------------------------------- Output dot-graph
      string dot_source = dag::to_dot_graph<TaskNode>(&input_image[0]);
      auto fname        = format("{:s}/task-graph.dot", outdir);
      file_put_contents(fname, dot_source);
      auto command = format("dot -Tpng {:s} > {:s}/task-graph.png", fname, outdir);
      if(system(command.c_str()) != 0) {
         LOG_ERR(
             format("error executing 'dot' command:\n\n   {:s}:\n\n Do you have "
                    "'dot' installed (i.e., graphviz)?",
                    command));
      }
   }

   { // ---------------------------------------------------------- Sanity checks
      if(dag::has_cycle<TaskNode>(&input_image[0])) {
         FATAL(format("cycle detected in FrameResults task graph!"));
      }
   }
}

} // namespace perceive::stereobm
