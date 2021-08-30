
#include "results_example.hpp"

#include "perceive/utils/file-system.hpp"

#define This FrameResults

namespace perceive::example
{
This::This()
{
   // ---- NOTES ----
   // (*) We can load/save parameters from a combined JSON file
   // (*) Currently no plans for changing how 'subscribe' behaviour
   //     works. That means that you must manually update this
   //     graph everytime

   { // -------------------------------------------------------- Load image task
      input_image.set_taskname("input-image");

      load_image::Params p;
      p.filename = "";
      input_image.set_params(p);
   }

   { // ------------------------------------------------------ Split image tasks
      for(auto i = 0u; i < 2; ++i) {
         split_image[i].set_taskname(format("split-image_[{}]", i));

         split_image::Params p;
         p.is_left_image = (i == 0) ? true : false;
         split_image[i].set_params(p);
         split_image[i].subscribe(&input_image);
      }
   }

   { // ------------------------------------------------------- Superpixel tasks
      for(auto i = 0u; i < 2; ++i) {
         superpixels[i].set_taskname(format("superpixels_[{}]", i));
         superpixels[i].subscribe(&split_image[i]);

         // This is not necessary; however, the testcase wants to
         // ensure that this task isn't "double-started"
         superpixels[i].subscribe(&input_image);
      }
   }

   { // ------------------------------------------------------- Output dot-graph
      string dot_source = dag::to_dot_graph<TaskNode>(&input_image);
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
      if(dag::has_cycle<TaskNode>(&input_image)) {
         FATAL(format("cycle detected in FrameResults task graph!"));
      }
   }
}

} // namespace perceive::example
