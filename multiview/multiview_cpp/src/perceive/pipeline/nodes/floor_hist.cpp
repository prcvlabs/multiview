
#include "floor_hist.hpp"

#include "perceive/movie/draw-floor-grid.hpp"

namespace perceive::pipeline::floor_hist
{
// ---------------------------------------------------------------------- Params
const vector<MemberMetaData>& Params::meta_data() const noexcept
{
#define ThisParams Params
   auto make_meta = []() {
      vector<MemberMetaData> m;
      m.push_back(MAKE_META(ThisParams, BOOL, feedback, false));
      m.push_back(MAKE_META(ThisParams, STRING, out_dir, false));
      m.push_back(MAKE_META(ThisParams, COMPATIBLE_OBJECT, hist_params, true));
      return m;
   };
   static vector<MemberMetaData> meta_ = make_meta();
   return meta_;
#undef ThisParams
}

template<typename ResultType>
void static get_cam_results(const Task::RunData& data,
                            const std::string& prefix,
                            const unsigned n_cams,
                            std::vector<shared_ptr<const ResultType>>& out)
{
   out.clear();
   for(unsigned cam_num{0}; cam_num < n_cams; ++cam_num) {
      auto result{
          data.match_result<ResultType>(format("{}[{}]", prefix, cam_num))};
      if(!result) {
         throw std::runtime_error(format("{}[{}] not found", prefix, cam_num));
      }
      out.push_back(result);
   }
}

// --------------------------------------------------------------------- Execute
shared_ptr<const Result>
Task::execute(const RunData& data,
              const Params& params,
              std::function<bool()> is_cancelled) const noexcept
{
   const int ref_image{0};
   const auto disparity_ind{0};

   auto ret_ptr     = new Result{};
   auto ret         = shared_ptr<Result>(ret_ptr);
   ret->hist_params = params.hist_params;

   auto first_f2d = data.match_result<run_f2d::Result>("run_f2d[0]");
   if(!first_f2d) {
      LOG_ERR("Couldn't find dependency: run_f2d[0]");
      return nullptr;
   }
   auto& scene_desc{first_f2d->scene_desc()};
   const auto n_cams{scene_desc.n_cameras()};
   const auto n_sensors{scene_desc.n_sensors()};

   try {
      const size_t sz_n_sensors = size_t(n_sensors);
      ret->f2d_result.resize(sz_n_sensors);
      ret->slic_lookup_result.resize(sz_n_sensors);
      ret->lab_result.resize(sz_n_sensors);

      for(size_t sensor = 0; sensor < sz_n_sensors; ++sensor) {
         auto f2d_result{
             data.match_result<run_f2d::Result>(format("run_f2d[{}]", sensor))};
         if(!f2d_result) {
            throw std::runtime_error(format("run_f2d[{}] not found", sensor));
         }
         ret->f2d_result[sensor] = f2d_result;

         auto slic_result{data.match_result<create_slic_lookup::Result>(
             format("create_slic_lookup[{}]", sensor))};
         if(!slic_result) {
            throw std::runtime_error(
                format("create_slic_lookup[{}] not found", sensor));
         }
         ret->slic_lookup_result[sensor] = slic_result;

         auto lab_result{data.match_result<convert_to_lab::Result>(
             format("convert_to_lab[{}]", sensor))};
         if(!lab_result) {
            throw std::runtime_error(
                format("convert_to_lab[{}] not found", sensor));
         }
         ret->lab_result[sensor] = lab_result;
      }

      get_cam_results<disp_map_update::Result>(
          data, "disp_map_update", unsigned(n_cams), ret->disp_result);

      AABB aabb = scene_desc.scene_info.hist_bounds;
      if(!aabb.is_finite()) {
         WARN(format("no histogram set in the scene-description"));
         aabb = AABB{-5.0, -5.0, 5.0, 5.0};
      }

      std::vector<const BinocularCameraInfo*> bcams;
      std::vector<const EuclideanTransform*> transforms;
      std::vector<const PointCloud*> point_clouds;
      std::vector<const cv::Mat*> rect_images;
      std::vector<const IntImage*> slic_lookups;
      std::vector<const ImageFeatures2d*> features;
      std::vector<Vector3> translations;

      for(size_t cam_num = 0; cam_num < size_t(n_cams); ++cam_num) {
         bcams.push_back(&scene_desc.bcam_infos[cam_num]);
         transforms.push_back(&scene_desc.cam_transforms[cam_num]);
         point_clouds.push_back(ret->disp_result[cam_num]->point_cloud.get());

         const auto slic_ind
             = scene_desc.sensor_lookup(int(cam_num), disparity_ind);
         slic_lookups.push_back(
             &ret->slic_lookup_result[size_t(slic_ind)]->slic_lookup);
         const auto f2d_ind = scene_desc.sensor_lookup(int(cam_num), ref_image);
         features.push_back(&ret->f2d_result[size_t(f2d_ind)]->f2d);
         translations.push_back(scene_desc.cam_transforms[cam_num].translation);
         rect_images.push_back(&(ret->disp_result[cam_num]
                                     ->disp_result->rectified_result[ref_image]
                                     ->rectified));
      }

      const auto success = FloorHistogram::calculate(scene_desc,
                                                     params.hist_params,
                                                     aabb,
                                                     unsigned(n_cams),
                                                     bcams,
                                                     transforms,
                                                     point_clouds,
                                                     rect_images,
                                                     slic_lookups,
                                                     features,
                                                     translations,
                                                     ret->hist,
                                                     is_cancelled,
                                                     params.feedback,
                                                     params.out_dir);

      if(success and !is_cancelled() and params.hist_params.color_histogram) {
         for(auto cam_num = 0; cam_num < scene_desc.n_cameras(); ++cam_num) {
            const int sensor_no = scene_desc.sensor_lookup(cam_num, 0);
            const auto& sensor_im
                = ret->f2d_result[size_t(sensor_no)]->image_result->raw;
            if(sensor_im.empty()) continue;
            const auto im = draw_floor_grid(
                cv_to_argb(sensor_im),
                scene_desc.scene_info.hist_bounds,
                scene_desc.cu(sensor_no),
                scene_desc.sensor_transforms[size_t(sensor_no)]);
            im.save(format("{}/{}_{}_{}.png",
                           params.out_dir,
                           scene_desc.scene_info.scene_key,
                           scene_desc.scene_info.bcam_keys[size_t(cam_num)],
                           scene_desc.sensor_ids[size_t(sensor_no)]));
         }
      }

      if(!success || is_cancelled()) { return nullptr; }
   } catch(const std::runtime_error& e) {
      LOG_ERR(e.what());
      return nullptr;
   }

   return ret;
}
} // namespace perceive::pipeline::floor_hist
