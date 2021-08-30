
#include "frame-results.hpp"

#include "perceive/pipeline/pipeline-task.hpp"
#include "perceive/utils/cuda-spec.hpp"
#include "perceive/utils/file-system.hpp"

#define This FrameResults

namespace perceive::pipeline
{
// -------------------------------------------------------------- traverse nodes

template<typename ResultType, typename Visitor>
static void traverse_nodes(ResultType& r, const Visitor& v)
{
   v(r.load_scene_description);
   v(r.copy_sensor_images);
   v(r.pose_skeleton_init);

   for(auto& node : r.input_images) { v(*node); }
   for(auto& node : r.convert_to_lab) { v(*node); }
   for(auto& node : r.bcam_init) { v(*node); }
   for(auto& node : r.run_f2d) { v(*node); }
   for(auto& node : r.create_slic_lookup) { v(*node); }
   for(auto& node : r.calc_calibration_mask) { v(*node); }
   for(auto& node : r.calc_rectified) { v(*node); }
   for(auto& node : r.get_xy_mapping) { v(*node); }
   for(auto& node : r.disp_init) { v(*node); }
   for(auto& node : r.disp_map_update) { v(*node); }
   // for(auto& node : r.slic_3d_init) { v(*node); }

   v(r.floor_hist);
   v(r.localization);
   v(r.movie_stats);
}

template<typename ResultType, typename Visitor>
static void ctraverse_nodes(const ResultType& r, const Visitor& v)
{
   traverse_nodes(*const_cast<ResultType*>(&r), v);
}

// ------------------------------------------------------------------------ init
//
void This::init_(shared_ptr<const SceneDescription> scene_desc, bool feedback)
{
   Expects(scene_desc);

   const unsigned n_cameras{unsigned(scene_desc->n_cameras())};
   const unsigned n_sensors{unsigned(scene_desc->n_sensors())};

   // subsribe left node to all right nodes
   auto one_to_many = [](auto& left_node, auto& right_nodes) {
      for(auto& right : right_nodes) { left_node.subscribe(right.get()); }
   };

   // subscribe all left nodes to right node
   auto many_to_one = [](auto& left_nodes, auto& right_node) {
      for(auto& left : left_nodes) { left->subscribe(&right_node); }
   };

   // Subscribe all left_nodes to all right_nodes
   auto many_to_many = [&](auto& left_nodes, auto& right_nodes) {
      for(auto& left : left_nodes) { one_to_many(*left, right_nodes); }
   };

   // Subscribe left_nodes[i] to right_nodes[i]
   auto per_camera = [&](auto& left_nodes, auto& right_nodes) {
      Expects(n_cameras == left_nodes.size());
      Expects(n_cameras == right_nodes.size());
      for(unsigned i = 0; i < n_cameras; ++i) {
         left_nodes[i]->subscribe(right_nodes[i].get());
      }
   };

   auto set_names = [](auto& nodes, const std::string& prefix) {
      int i = 0;
      for(auto& node : nodes) {
         node->set_taskname(format("{}[{}]", prefix, i));
         ++i;
      }
   };

   // Setup params
   {
      load_scene_description::Params p;
      p.scene_desc = scene_desc;
      load_scene_description.set_params(p);
   }

   {
      floor_hist::Params p;
      floor_hist.set_params(p);
   }

   {
      copy_sensor_images::Params p;
      p.frame_num = 0;
      copy_sensor_images.set_params(p);
   }

   for(unsigned sensor{0}; sensor < n_sensors; ++sensor) {
      input_images::Params p;
      p.sensor_index = sensor;
      input_images.emplace_back(make_unique<input_images::Task>())
          ->set_params(p);
   }

   for(unsigned cam{0}; cam < n_cameras; ++cam) {
      bcam_init::Params p;
      p.cam_num = cam;
      bcam_init.emplace_back(make_unique<bcam_init::Task>())->set_params(p);
   }

   for(unsigned sensor{0}; sensor < n_sensors; ++sensor) {
      convert_to_lab::Params p;
      p.sensor_index = sensor;
      auto& node{
          convert_to_lab.emplace_back(make_unique<convert_to_lab::Task>())};
      node->set_params(p);
      node->set_taskname(format("convert_to_lab[{}]", sensor));
   }

   for(unsigned sensor{0}; sensor < n_sensors; ++sensor) {
      run_f2d::Params p;
      p.sensor_index = sensor;
      auto& node{run_f2d.emplace_back(make_unique<run_f2d::Task>())};
      node->set_params(p);
      node->set_taskname(format("run_f2d[{}]", sensor));
   }

   for(unsigned sensor{0}; sensor < n_sensors; ++sensor) {
      calc_calibration_mask::Params p;
      p.sensor_index = sensor;
      auto& node{calc_calibration_mask.emplace_back(
          make_unique<calc_calibration_mask::Task>())};
      node->set_params(p);
      node->set_taskname(format("calc_calibration_mask[{}]", sensor));
   }

   for(unsigned sensor{0}; sensor < n_sensors; ++sensor) {
      calc_rectified::Params p;
      p.sensor_index = sensor;
      auto& node{
          calc_rectified.emplace_back(make_unique<calc_rectified::Task>())};
      node->set_params(p);
      node->set_taskname(format("calc_rectified[{}]", sensor));
   }

   for(unsigned sensor{0}; sensor < n_sensors; ++sensor) {
      get_xy_mapping::Params p;
      p.sensor_index = sensor;
      auto& node{
          get_xy_mapping.emplace_back(make_unique<get_xy_mapping::Task>())};
      node->set_params(p);
      node->set_taskname(format("get_xy_mapping[{}]", sensor));
   }

   for(unsigned sensor{0}; sensor < n_sensors; ++sensor) {
      create_slic_lookup::Params p;
      p.sensor_index = sensor;
      auto& node{create_slic_lookup.emplace_back(
          make_unique<create_slic_lookup::Task>())};
      node->set_params(p);
      node->set_taskname(format("create_slic_lookup[{}]", sensor));
   }

   for(unsigned cam{0}; cam < n_cameras; ++cam) {
      disp_init::Params p;
      p.cam_num = cam;
      disp_init.emplace_back(make_unique<disp_init::Task>())->set_params(p);
   }

   for(unsigned cam{0}; cam < n_cameras; ++cam) {
      disp_map_update::Params p;
      p.cam_num = cam;
      disp_map_update.emplace_back(make_unique<disp_map_update::Task>())
          ->set_params(p);
   }

   // for(unsigned cam{0}; cam < n_cameras; ++cam) {
   //    slic_3d_init::Params p;
   //    p.cam_num = cam;
   //    slic_3d_init.emplace_back(make_unique<slic_3d_init::Task>())
   //        ->set_params(p);
   // }

   // Set task  names
   load_scene_description.set_taskname("load_scene_description");
   movie_stats.set_taskname("movie_stats");
   copy_sensor_images.set_taskname("copy_sensor_images");
   pose_skeleton_init.set_taskname("pose_skeleton_init");
   set_names(input_images, "input_images");
   set_names(bcam_init, "bcam_init");
   // set_names(slic_3d_init, "slic_3d_init");
   set_names(disp_init, "disp_init");
   set_names(disp_map_update, "disp_map_update");
   floor_hist.set_taskname("floor_hist");
   localization.set_taskname("localization");

   // gui_sensor_images.set_taskname("gui_sensor_images");
   // openpose_registration.set_taskname("openpose_registration");

   // Set up dependencies
   copy_sensor_images.subscribe(&load_scene_description);
   pose_skeleton_init.subscribe(&copy_sensor_images);
   many_to_one(bcam_init, load_scene_description);
   many_to_one(input_images, copy_sensor_images);

   for(auto sensor = 0u; sensor < n_sensors; ++sensor) {
      auto indices = scene_desc->bcam_lookup(int(sensor));
      auto camera  = indices[0];

      convert_to_lab[sensor]->subscribe(input_images[sensor].get());
      run_f2d[sensor]->subscribe(convert_to_lab[sensor].get());
      run_f2d[sensor]->subscribe(input_images[sensor].get());
      if(0 == sensor) {
         disp_map_update[size_t(camera)]->subscribe(run_f2d[sensor].get());
      }
      // slic_3d_init[camera]->subscribe(run_f2d[sensor].get());
      disp_map_update[size_t(camera)]->subscribe(
          create_slic_lookup[sensor].get());
      floor_hist.subscribe(create_slic_lookup[sensor].get());
      floor_hist.subscribe(convert_to_lab[sensor].get());
      create_slic_lookup[sensor]->subscribe(run_f2d[sensor].get());

      get_xy_mapping[sensor]->subscribe(bcam_init[size_t(camera)].get());
      get_xy_mapping[sensor]->subscribe(&load_scene_description);
      calc_calibration_mask[sensor]->subscribe(input_images[sensor].get());
      calc_rectified[sensor]->subscribe(input_images[sensor].get());
      calc_rectified[sensor]->subscribe(get_xy_mapping[sensor].get());
      create_slic_lookup[sensor]->subscribe(get_xy_mapping[sensor].get());
      disp_init[size_t(camera)]->subscribe(calc_rectified[sensor].get());
      disp_map_update[size_t(camera)]->subscribe(run_f2d[sensor].get());
   }

   // gui_sensor_images->subscribe(&load_scene_description);
   // one_to_many(gui_sensor_images, input_images);
   // one_to_many(gui_sensor_images, calc_rectified);

   // per_camera(slic_3d_init, bcam_init);

   per_camera(disp_init, bcam_init);
   per_camera(disp_map_update, bcam_init);
   per_camera(disp_map_update, disp_init);
   one_to_many(floor_hist, disp_map_update);
   one_to_many(floor_hist, run_f2d);

   movie_stats.subscribe(&load_scene_description);

   localization.subscribe(&load_scene_description);
   localization.subscribe(&pose_skeleton_init);
   // localization.subscribe(&movie_stats);
   localization.subscribe(&floor_hist);

   // Add on-timer callbacks to each task-node
   traverse_nodes(*this, [&](TaskNode& node) {
      node.set_timing_callback([this](const TaskNode* task, real seconds) {
         if(this->on_task_finished_) this->on_task_finished_(task, seconds);
      });
   });

   // Create the 'lookup'
   traverse_nodes(*this, [&](TaskNode& node) {
      Expects(task_lookup_.count(node.taskname()) == 0);
      task_lookup_[node.taskname()] = &node;
   });

   set_feedback(feedback);
}

// ---------------------------------------------------------------- Construction
//
This::FrameResults(shared_ptr<const SceneDescription> scene_desc, bool feedback)
{
   init_(scene_desc, feedback);
}

This::FrameResults(const FrameResults& o)
{
   const auto pp = o.load_scene_description.params();
   init_(pp.scene_desc, false);
   *this = o;
}

FrameResults& This::operator=(const FrameResults& o)
{
   if(this == &o) return *this;

   on_task_finished_ = [](const TaskNode*, real) {};

   traverse_nodes(*this, [&](TaskNode& node) {
      auto ii = o.task_lookup_.find(node.taskname());
      Expects(ii != end(o.task_lookup_));
      node.shallow_copy_params_result(*ii->second);
   });

   return *this;
}

// ------------------------------------------------------------------ operator==
//
bool This::operator==(const FrameResults& o) const noexcept
{
   auto same = true;
   ctraverse_nodes(*this, [&](TaskNode& node) {
      auto ii = o.task_lookup_.find(node.taskname());
      Expects(ii != end(o.task_lookup_));
      if(same) same = node.params_equal(*(ii->second));
   });
   return same;
}

bool This::operator!=(const FrameResults& o) const noexcept
{
   return !(*this == o);
}

// ------------------------------------------------------------ output dot graph

void This::output_dot_graph(const std::string& outdir) noexcept
{
   // string dot_source{dag::to_dot_graph<TaskNode>(&copy_sensor_images)};
   string dot_source{dag::to_dot_graph<TaskNode>(input_images[0].get())};
   auto fname{format("{}/pipeline-graph.dot", outdir)};
   file_put_contents(fname, dot_source);
   auto command{format("dot -Tpng {} > {}/pipeline-graph.png", fname, outdir)};
   if(system(command.c_str()) != 0) {
      LOG_ERR(format("error executing 'dot' command:\n\n   {}:\n\n Do you have "
                     "'dot' installed (i.e., graphviz)?",
                     command));
   }
}

// ------------------------------------------------------------------- set frame

void This::set_frame(int frame_number) noexcept
{
   copy_sensor_images::Params p{copy_sensor_images.params()};
   if(p.frame_num != unsigned(frame_number)) {
      p.frame_num = unsigned(frame_number);
      copy_sensor_images.set_params(p);
   }
}

// ---------------------------------------------------------------- set feedback

void This::set_feedback(bool feedback) noexcept
{
   traverse_nodes(*this, [&](auto& node) {
      auto p{node.params()};
      p.feedback = feedback;
      node.set_params(p);
   });
}

// -------------------------------------------------------------- set output dir

void This::set_output_dir(const std::string& out_dir)
{
   traverse_nodes(*this, [&](auto& node) {
      auto p{node.params()};
      p.out_dir = out_dir;
      node.set_params(p);
   });
}

// -------------------------------------------------- set task-finished callback

void This::set_task_finished_callback(
    std::function<void(const TaskNode* task, real seconds)> f) noexcept
{
   on_task_finished_ = std::move(f);
}

// ----------------------------------------------------------- process all tasks

void This::process_all_tasks() noexcept
{
   traverse_nodes(*this, [&](auto& node) { node.calc_result([&](auto x) {}); });
}

// -------------------------------------------------------------- params to json

Json::Value This::params_to_json() const noexcept
{
   auto root{Json::Value{Json::objectValue}};

   traverse_nodes(*this, [&](auto& node) {
      auto name{node.taskname()};
      root[name] = node.params().to_json();
   });

   return root;
}

// ----------------------------------------------------------------- read params

bool This::read_params(const Json::Value& o, const string_view path) noexcept
{
   bool has_error = false;
   traverse_nodes(*this, [&](auto& node) {
      auto name{node.taskname()};
      auto params = node.params();
      decltype(params) defaults;

      if(!has_key(o, name)) {
         cout << format("{}WARNING{}: using default value for '{}.{}'",
                        ANSI_COLOUR_YELLOW,
                        ANSI_COLOUR_RESET,
                        path,
                        name)
              << endl;
         node.set_params(defaults);
      } else {
         try {
            const auto path_s = format("{}.{}", path, name);
            params.read_with_defaults(o[name], &defaults, true, path_s);
            node.set_params(params);
         } catch(std::exception& e) {
            LOG_ERR(format(
                "exception reading {} params: {}", node.taskname(), e.what()));
            // has_error = true;
         }
      }
   });
   return !has_error;
}
} // namespace perceive::pipeline
