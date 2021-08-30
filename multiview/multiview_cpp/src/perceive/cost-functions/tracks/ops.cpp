
#include "ops.hpp"

#include "perceive/calibration/training-data/training-data-point.hpp"
#include "perceive/cost-functions/classifier/classifier-registry.hpp"
#include "perceive/cost-functions/localization/calc-prob-p2d-false-positive.hpp"
#include "perceive/cost-functions/localization/localization-data.hpp"
#include "perceive/cost-functions/tracklets/tracklet-exec.hpp"
#include "perceive/cost-functions/tracks/localization-memo.hpp"
#include "perceive/cost-functions/tracks/track-ops.hpp"
#include "perceive/geometry/skeleton/2d-helpers.hpp"
#include "perceive/geometry/skeleton/interpolate.hpp"
#include "perceive/graphics/LAB.hpp"
#include "perceive/graphics/bresenham.hpp"
#include "perceive/graphics/colour-set.hpp"
#include "perceive/graphics/image-container.hpp"
#include "perceive/graphics/tiny-string.hpp"
#include "perceive/io/fp-io.hpp"
#include "perceive/optimization/hungarian-algorithm.hpp"
#include "perceive/optimization/min-cut-graph.hpp"
#include "perceive/optimization/shortest-path.hpp"
#include "perceive/utils/file-system.hpp"
#include "perceive/utils/spin-lock.hpp"

#define This ComputationData

namespace perceive::tracks
{
using NodeSequence = tracks::NodeSequence;

static string print_node(const This::XYTNode* o) { return o->to_string(); }

static string print_edge(const This::XYTEdge* e)
{
   return format("{{w={:+5.3f} ({}, {}, {}) -> ({}, {}, {})}}",
                 e->weight,
                 e->src->t(),
                 e->src->frame_id(),
                 e->src->inchoate_id(),
                 e->dst->t(),
                 e->dst->frame_id(),
                 e->dst->inchoate_id());

   return format("[{}/{}/{}, {}/{}/{}, w={}]",
                 e->src->t(),
                 e->src->frame_id(),
                 e->src->inchoate_id(),
                 e->dst->t(),
                 e->dst->frame_id(),
                 e->dst->inchoate_id(),
                 e->weight);
}

static vector<This::XYTNode*>
topological_sort(const vector<This::XYTNode*>& flat)
{
   std::unordered_map<This::XYTNode*, size_t> lookup;
   lookup.reserve(flat.size());
   for(size_t i = 0; i < flat.size(); ++i) lookup[flat[i]] = i;
   vector<uint8_t> marks(flat.size(), 0);
   vector<This::XYTNode*> sorted;
   sorted.reserve(flat.size());

   auto get_index = [&](This::XYTNode* ptr) -> size_t {
      auto ii = lookup.find(ptr);
      Expects(ii != cend(lookup));
      return ii->second;
   };

   auto is_not_marked = [&](size_t index) { return marks.at(index) == 0; };
   auto is_temp_mark  = [&](size_t index) { return marks.at(index) == 1; };
   auto is_perm_mark  = [&](size_t index) { return marks.at(index) == 2; };
   auto set_temp_mark = [&](size_t index) { marks.at(index) = 1; };
   auto set_perm_mark = [&](size_t index) { marks.at(index) = 2; };

   std::function<void(This::XYTNode * ptr)> visit = [&](This::XYTNode* ptr) {
      const size_t index = get_index(ptr);
      if(is_perm_mark(index)) return; // we're done here
      Expects(!is_temp_mark(index));
      set_temp_mark(index);
      for(auto& e : ptr->edges()) { visit(e.dst); }
      set_perm_mark(index);
      sorted.push_back(ptr);
   };

   for(size_t i = 0; i < flat.size(); ++i)
      if(is_not_marked(i)) visit(flat[i]);
   Expects(flat.size() == sorted.size());

   std::reverse(begin(sorted), end(sorted));

   if(multiview_trace_mode()) {
      lookup.clear();
      for(size_t i = 0; i < sorted.size(); ++i) lookup[sorted[i]] = i;
      for(auto src : sorted) {
         const auto src_index = get_index(src);
         for(auto e : src->edges()) { Expects(src_index < get_index(e.dst)); }
      }
   }

   return sorted;
}

// ----------------------------------------------------- single-source-path-info
//
struct PathNode
{
   This::XYTEdge* edge = nullptr; // dstination node holds the edge
   int predecessor     = -1;      // for traversal
   float weight        = fNAN;
};

struct SingleSourcePathInfo
{
 private:
   size_t source_index    = 0;
   vector<PathNode> paths = {};

 public:
   static SingleSourcePathInfo
   make(This::XYTNode* src_node,
        const vector<This::XYTNode*>& sorted,
        std::function<bool(const This::XYTEdge*)> edge_is_covered = {})
   {
      SingleSourcePathInfo info;
      info.source_index = size_t(src_node->topological_pos());
      info.paths.resize(sorted.size());

      // Cost to get to starting node
      info.paths.at(info.source_index).weight = 0.0f;

      for(size_t i = info.source_index; i < sorted.size(); ++i) {
         const auto& u = info.paths.at(i);
         for(auto& e : sorted.at(i)->edges()) {
            Expects(std::isfinite(e.weight));
            if(edge_is_covered && edge_is_covered(&e)) continue;
            auto& v = info.paths.at(size_t(e.dst->topological_pos()));
            if(!std::isfinite(v.weight)
               || v.weight > u.weight + e.weight) { // Relax edge
               v.weight      = u.weight + e.weight;
               v.predecessor = int(i);
               v.edge        = &e;
            }
         }
      }

      return info;
   }

   float path_cost(This::XYTNode* dst) const noexcept
   {
      return paths.at(size_t(dst->topological_pos())).weight;
   }

   vector<This::XYTEdge*> path(This::XYTNode* dst) const noexcept
   {
      const bool invariant_check = multiview_trace_mode();

      const int src_idx = int(source_index);
      const int dst_idx = dst->topological_pos();

      if(!std::isfinite(paths.at(size_t(dst_idx)).weight)) return {};

      int counter = 0;
      {
         int idx = dst_idx;
         while(idx >= 0 && idx != src_idx) {
            auto& node = paths.at(size_t(idx));
            if(invariant_check) {
               Expects(std::isfinite(node.weight));
               Expects(node.predecessor >= 0);
               Expects(node.edge != nullptr);
               Expects(node.edge->dst->topological_pos() == idx);
               Expects(node.edge->src->topological_pos() == node.predecessor);
            }
            ++counter;
            idx = node.predecessor;
         }
      }

      vector<This::XYTEdge*> out;
      {
         out.reserve(size_t(counter));
         int idx = dst_idx;
         while(idx >= 0 && idx != src_idx) {
            auto& node = paths.at(size_t(idx));
            out.push_back(node.edge);
            idx = node.predecessor;
         }
         Expects(out.size() == size_t(counter));
         std::reverse(begin(out), end(out));
      }

      return out;
   }
};

// -------------------------------------------------------------- all-paths-info
//
struct AllPathsInfo
{
   vector<This::XYTNode*> sorted;
   std::unordered_map<This::XYTNode*, size_t> lookup;
   vector<SingleSourcePathInfo> info;
   vector<bool> marks;

   size_t get_node_index(This::XYTNode* ptr) const noexcept
   {
      auto ii = lookup.find(ptr);
      Expects(ii != cend(lookup));
      return ii->second;
   }

   float path_cost(This::XYTNode* src, This::XYTNode* dst) const noexcept
   {
      return info.at(get_node_index(src)).path_cost(dst);
   }

   vector<This::XYTEdge*> path(This::XYTNode* src,
                               This::XYTNode* dst) const noexcept
   {
      return info.at(get_node_index(src)).path(dst);
   }
};

static AllPathsInfo
dag_all_pairs_shortest_path(const vector<This::XYTNode*>& flat)
{
   AllPathsInfo out;

   auto& sorted = out.sorted;
   auto& lookup = out.lookup;
   auto& info   = out.info;
   auto& marks  = out.marks;

   sorted = topological_sort(flat);
   lookup.reserve(sorted.size());
   for(size_t i = 0; i < sorted.size(); ++i) lookup[sorted[i]] = i;
   marks.resize(sorted.size(), false);
   out.info.resize(sorted.size());

   // -- (*) -- Execute each source node in parallel
   ParallelJobSet pjobs;
   for(auto i = 0u; i < sorted.size(); ++i)
      pjobs.schedule([&info, &out, i]() {
         info[i] = SingleSourcePathInfo::make(out.sorted[i], out.sorted);
      });
   pjobs.execute();

   return out;
}

// --------------------------------------------------------- calc-id-of-interest
//
static constexpr bool calc_id_of_interest(const This::OpSkeleton2D& A,
                                          const This::OpSkeleton2D& B)
{
   return false;
   return (A.t == 1 && B.t == 4);
   return (A.p2d_address() == P2dAddress(1, 2, 3)
           && B.p2d_address() == P2dAddress(4, 2, 5));
}

// ------------------------------------------------------------ inchoate-summary
//
static string inchoate_summary(const This::InchoateTrack& it)
{
   std::stringstream ss{""};
   bool first = true;
   for(const auto& e : it.edges) {
      if(first) {
         first = false;
         ss << format("[{}, {}]", e->src->t(), e->src->frame_id());
      }
      ss << format(
          "-> {:4.2f} -> [{}, {}]", e->weight, e->dst->t(), e->dst->frame_id());
   }
   return format("it={:03d}, [{}]", it.idx, ss.str());
}

// --------------------------------------------------------------------- run-dot
//
static void run_dot(const string_view dot_source, const string_view outfile)
{
   const auto fname = format("{}.dot", extensionless(outfile));
   file_put_contents(fname, dot_source.data());

   const auto command = format("dot -Tpng {} > {}", fname, outfile);
   if(system(command.c_str()) != 0)
      FATAL(format("error executing 'dot', command: {}", command));

   delete_file(fname.c_str());
}

// --------------------------------------------------------------------- sum-lab
//
static void sum_lab(Vec3fImage& o, const LABImage& v)
{
   Expects(o.width == v.width);
   Expects(o.height == v.height);

   for(auto y = 0u; y < o.height; ++y) {
      auto o_ptr = o.ptr(y);
      auto e_ptr = o_ptr + o.width;
      auto v_ptr = v.ptr(y);
      while(o_ptr != e_ptr) *o_ptr++ += LAB_to_LAB_vec3f(*v_ptr++);
   }
}

// ---------------------------------------------------------------- init-patches
//
static void init_patches(This::InchoateTrack& it,
                         const vector<LABImage>& patches)
{
   it.model.resize(patches.size());
   for(size_t j = 0; j < patches.size(); ++j)
      it.model[j] = LAB_im_to_LAB_vec3f(patches[j]);
   it.model_sz = 1;
}

static void set_still_score(This& op, This::XYT& xyt)
{
   float sum   = 0.0f;
   int counter = 0;

   for(const auto skel : xyt.skels) {
      const auto info_ptr = skel->p2d_info_ptr;
      if(info_ptr->patches.size() == 0) continue;
      const auto p2d      = info_ptr->p2d_ptr.get();
      const int sensor_no = p2d->sensor_no();
      Expects(size_t(sensor_no) < op.blur_labs.size());
      const auto& still = op.blur_labs.at(size_t(sensor_no));
      Expects(still.width > 0 && still.height > 0);

      const int patch_w  = int(info_ptr->patches[0].width);
      const int patch_h  = int(info_ptr->patches[0].height);
      const auto patches = p2d->make_image_patches(patch_w, patch_h, still);
      Expects(patches.size() == info_ptr->patches.size());
      sum += calc_lab_patch_score(patches, info_ptr->patches);
      counter++;
   }

   xyt.still_score = (counter == 0) ? fNAN : sum / float(counter);
}

// ---------------------------------------------------------------- calc-prob-fp
//
static float calc_prob_fp(const This::XYT& xyt)
{
   if(xyt.skels.size() == 0) return 1.0f;

   float sum = 0.0f;
   for(auto skel : xyt.skels) {
      const auto val = skel->prob_false_positive;
      Expects(std::isfinite(val));
      sum += val;
   }

   return sum / float(xyt.skels.size());
}

// ---------------------------------------------------- clip-node-sequences-to-t
//
static vector<NodeSequence>
clip_node_sequences_to_t(const vector<NodeSequence>& seqs,
                         const int max_t_inclusive) noexcept(false)
{
   vector<NodeSequence> out = seqs;
   for(auto& seq : out) { seq.clip_nodes_to_t(max_t_inclusive); }
   return out;
}

// ------------------------------------------------------------- is-dt-candidate
// Is p2d A & B a candidate for a graph edge?
static bool is_dt_candidate(const Tracklet::Params& tracklet_params,
                            const This::OpSkeleton2D* skel_A,
                            const This::OpSkeleton2D* skel_B,
                            const int delta_t, // number of frames different
                            const float frame_duration,
                            const bool verbose)
{
   Expects(delta_t > 0);

   const auto of_interest = calc_id_of_interest(*skel_A, *skel_B);

   const Skeleton2DInfo& p2d_info_a = *skel_A->p2d_info_ptr;
   const Skeleton2DInfo& p2d_info_b = *skel_B->p2d_info_ptr;
   const auto& p2d_a                = *p2d_info_a.p2d_ptr;
   const auto& p2d_b                = *p2d_info_b.p2d_ptr;

   if(p2d_a.is_interpolation() || p2d_b.is_interpolation()) {
      if(verbose)
         TRACE(format("dt-candidate {} -> {}, interp={}/{} -> false",
                      skel_A->p2d_address().to_string(),
                      skel_B->p2d_address().to_string(),
                      str(p2d_a.is_interpolation()),
                      str(p2d_b.is_interpolation())));
      return false;
   }

   auto get_Xs = [&](const auto& p2d) {
      return clip_to_xy(p2d.best_3d_result().Xs_centre());
   };
   const auto Xa           = get_Xs(p2d_a);
   const auto Xb           = get_Xs(p2d_b);
   const auto dist         = (Xa - Xb).norm();                // metres
   const auto duration     = frame_duration * float(delta_t); // seconds
   const auto speed        = dist / duration; // metres per second
   const bool is_candidate = (speed < tracklet_params.speed_cap) || of_interest;

   if(verbose || of_interest)
      TRACE(format("dt-candidate {} -> {}, |{} - {}| = {} -> {} m/s -> {}",
                   skel_A->p2d_address().to_string(),
                   skel_B->p2d_address().to_string(),
                   str(Xa),
                   str(Xb),
                   dist,
                   speed,
                   str(is_candidate)));

   return is_candidate;
}

// -------------------------------------------------------------------- XYT::XYT
//
This::XYT::XYT(const This& ops,
               vector<OpSkeleton2D*>&& skels,
               const int t) noexcept
{
   auto get_X_pos = [&ops, t](const vector<OpSkeleton2D*>& skels) {
      Vector3f X;
      if(true || skels.size() > 1) {
         vector<const Skeleton2DInfo*> infos;
         infos.reserve(skels.size());
         std::transform(cbegin(skels),
                        cend(skels),
                        std::back_inserter(infos),
                        [](const auto ptr) { return ptr->p2d_info_ptr; });
         const auto pos_info = position_p2d_infos(
             &ops.scene_desc, &infos[0], infos.size(), ops.hist_sz, ops.aabb);
         X = ops.hist_to_X3f(pos_info.xy);

         if(false && t == 40)
            TRACE(format(
                "pos-info: {}, X: {}, infos: [{}]",
                pos_info.to_string(),
                X.to_string(),
                implode(cbegin(skels), cend(skels), ", ", [&](const auto ptr) {
                   return ptr->p2d_address().to_string();
                })));
      }

      Expects(X.is_finite());
      return X;
   };

   auto get_theta =
       [](const vector<OpSkeleton2D*>& skels) { // Update the viewing direction
          vector<float> thetas;
          thetas.resize(skels.size());
          std::transform(
              begin(skels), end(skels), begin(thetas), [](auto ptr) -> float {
                 Expects(ptr != nullptr);
                 return float(ptr->p2d_info_ptr->p2d_ptr->theta());
              });
          auto ii = std::partition(begin(thetas), end(thetas), [](auto x) {
             return std::isfinite(x);
          });
          thetas.erase(ii, end(thetas));

          return average_angles(begin(thetas), end(thetas));
       };

   auto& xyt       = *this;
   xyt.skels       = std::move(skels);
   const auto X    = get_X_pos(xyt.skels);
   xyt.x           = X.x;
   xyt.y           = X.y;
   xyt.t           = t;
   xyt.gaze_theta  = get_theta(xyt.skels);
   xyt.still_score = fNAN;
}

// ------------------------------------------------------------------- to-string
//
string This::XYT::to_string() const noexcept
{
   return format("[xyt={{{}, {}, {}}}, theta={}, skels=[{}]",
                 x,
                 y,
                 t,
                 to_degrees(gaze_theta),
                 implode(cbegin(skels), cend(skels), ", ", [](const auto ptr) {
                    return ptr->p2d_address().to_string();
                 }));
}

void This::classify_pose(XYTNode* node, const real speed) const noexcept
{
   if(!pose_classifier || node->xyt().skels.size() == 0) return;
   Expects(node->xyt().pose_classification.size() == 0);

   int counter = 0;
   vector<real> out_probs;
   vector<float> sums;
   auto process_one = [&](const Skeleton2DInfo& skel_info) {
      const auto features
          = calibration::make_feature_vector(scene_desc, skel_info, speed);
      const auto ret = pose_classifier->predict(features, true, out_probs);
      if(sums.empty()) sums.resize(out_probs.size(), 0.0f);
      out_probs.at(0) = 0.0; // Ignore "PoseAnnotation::NONE"
      for(auto&& [x, sum] : views::zip(out_probs, sums)) sum += float(x);
      ++counter;
   };

   // We're going to average the responses for all relevant skeleton
   for(auto op_skel_ptr : node->xyt().skels)
      process_one(*op_skel_ptr->p2d_info_ptr);

   // Bail if there's nothing
   Expects(counter > 0);

   // Average
   for(auto& x : sums) x /= float(counter);

   // And save in the XYTNode
   node->xyt().pose_classification = std::move(sums);
}

// ------------------------------------------------------------ computation-data
//
This::~This() {}

This::ComputationData(const SceneDescription& in_scene_desc,
                      const LocalizationData::Params& in_loc_params,
                      const Tracklet::Params& in_tracklet_params,
                      const Tracks::Params& in_params,
                      const real in_frame_duration,
                      const int in_max_frames_per_tracklet,
                      const Tracklet* in_tracklet,
                      const Tracklet* in_prev_tracklet,
                      Tracks* in_prev_track,
                      const size_t random_seed,
                      std::function<bool()> in_is_cancelled,
                      const string_view in_outdir,
                      const bool in_feedback)
    : is_trace_mode(multiview_trace_mode())
    , scene_desc(in_scene_desc)
    , random_generator(random_seed)
    , loc_params(in_loc_params)
    , tracklet_params(in_tracklet_params)
    , params(in_params)
    , tracklet(in_tracklet)
    , prev_tracklet(in_prev_tracklet)
    , frame_duration(in_frame_duration)
    , max_frames_per_tracklet(in_max_frames_per_tracklet)
    , is_cancelled(in_is_cancelled ? in_is_cancelled : []() { return false; })
    , outdir(in_outdir)
    , feedback(in_feedback)
    , n_frames((in_prev_tracklet == nullptr ? 0 : in_prev_tracklet->n_frames)
               + in_tracklet->n_frames)
    , start_frame(in_prev_tracklet != nullptr ? in_prev_tracklet->start_frame
                                              : in_tracklet->start_frame)
    , end_frame(in_tracklet->start_frame + in_tracklet->n_frames - 1)
    , hist_sz(tracklet->frames.front().loc_ptr->hist_sz)
    , aabb(tracklet->frames.front().loc_ptr->bounds)
    , w(int(tracklet->frames.front().loc_ptr->loc_hist.width))
    , h(int(tracklet->frames.front().loc_ptr->loc_hist.height))
    , prev_track(in_prev_track)
{
   const bool is_trace = multiview_trace_mode();
   const bool has_prev = (prev_track != nullptr);

   if(tracklet_params.frame_t_interp_window + 1
      >= params.fixed_window_prev_tracklet) {
      LOG_ERR(
          format("frame-t-interp-window = {}, but fixed_window_prev_tracklet = "
                 "{}. This is not enough. `fixed_window_prev_tracklet` should "
                 "be at least 2 frames longer.",
                 tracklet_params.frame_t_interp_window,
                 params.fixed_window_prev_tracklet));
   }

   Expects(end_frame < scene_desc.n_frames());

   // Timing data
   const auto now0  = tick();
   int pose_dat_sz0 = 0;

   { // Sanity
      Expects((prev_track == nullptr) == (prev_tracklet == nullptr));

      const auto t0 = tracklet->start_frame;
      const auto t1 = tracklet->n_frames;
      const auto p0 = has_prev ? prev_tracklet->start_frame : -1;
      const auto p1 = has_prev ? prev_tracklet->n_frames : 0;
      const auto q0 = has_prev ? prev_track->start_frame : -1;
      const auto q1 = has_prev ? prev_track->n_frames : 0;

      if(false) {
         TRACE(
             format("ComputationData: this-traklet = [{}..{}), prev-tracklet = "
                    "[{}..{}), prev-track [{}..{})",
                    t0,
                    t0 + t1,
                    p0,
                    p0 + p1,
                    q0,
                    q0 + q1));
      }

      Expects(unsigned(tracklet->n_frames) == tracklet->frames.size());
      Expects(tracklet->n_frames > 0);
      if(has_prev) {
         Expects(prev_tracklet != nullptr);
         Expects(tracklet->start_frame
                 == prev_tracklet->start_frame + prev_tracklet->n_frames);
         Expects(prev_tracklet->start_frame == prev_track->start_frame);
         Expects(prev_tracklet->n_frames == prev_track->n_frames);
      }
      Expects(start_frame + n_frames
              == tracklet->start_frame + tracklet->n_frames);
   }

   if(is_cancelled()) return;

   { // Read the classifiers
      if(!params.pose_classifier.empty()) {
         pose_classifier = get_classifier(params.pose_classifier);
         if(pose_classifier == nullptr)
            FATAL(format("failed to initialize pose-classifier"));
      }
   }

   { // Set up `interp-params`
      interp_params.patch_w = int(loc_params.pose2d_patch_w);
      interp_params.patch_h = int(loc_params.pose2d_patch_h);
      interp_params.projective_distance_threshold
          = tracklet_params.projective_distance_threshold;
      interp_params.frame_t_window = tracklet_params.frame_t_interp_window;
   }

   { // Setup `pose_dat`, ie., skeleton detections
      // Push p2ds from the `previous frame` and `current frame`
      const auto now  = tick();
      auto count_p2ds = [](const Tracklet* ptr) -> size_t {
         if(ptr == nullptr) return 0;
         size_t count = 0;
         for(const auto& frame : ptr->frames) {
            count += frame.p2d_info_ptrs.size();
            for(auto p : frame.p2d_info_ptrs) {
               Expects(p != nullptr);
               Expects(p->p2d_ptr != nullptr);
               Expects(p->p2d_ptr->is_interpolation() == false);
            }
         }
         return count;
      };
      const auto n_p2ds = count_p2ds(prev_tracklet) + count_p2ds(tracklet);

      { // Fill the pose-dat array
         auto info_to_op_skel = [&](int t, const Skeleton2DInfo* info_ptr) {
            OpSkeleton2D skel;
            skel.p2d_info_ptr = info_ptr;
            skel.t            = t;
            Expects(info_ptr != nullptr);
            Expects(info_ptr->p2d_ptr != nullptr);
            Expects(skel.is_interpolation() == false);
            return skel;
         };

         auto fill_pose_dat = [&](const Tracklet* ptr) {
            for(const auto& frame : ptr->frames)
               for(const Skeleton2DInfo* info_ptr : frame.p2d_info_ptrs)
                  pose_dat.emplace_back(
                      info_to_op_skel(frame.loc_ptr->frame_no, info_ptr));
         };
         pose_dat.reserve(n_p2ds);
         if(prev_tracklet) fill_pose_dat(prev_tracklet);
         fill_pose_dat(tracklet);
         Expects(pose_dat.size() == n_p2ds);
      }

      { // Fill the frame-dat array
         auto fill_frame_dat = [&](const Tracklet* ptr) {
            for(const auto& frame : ptr->frames) {
               frame_dat.emplace_back();
               auto& fd   = frame_dat.back();
               fd.loc_ptr = frame.loc_ptr.get();
               fd.t       = fd.loc_ptr->frame_no;
               fd.sensor_labs.resize(frame.sensor_labs.size());
               std::transform(cbegin(frame.sensor_labs),
                              cend(frame.sensor_labs),
                              begin(fd.sensor_labs),
                              [](const auto& o) {
                                 Expects(o.get() != nullptr);
                                 return o.get();
                              });
               fd.point_clouds.resize(frame.cam_pt_clouds.size());
               std::transform(cbegin(frame.cam_pt_clouds),
                              cend(frame.cam_pt_clouds),
                              begin(fd.point_clouds),
                              [](const auto& o) {
                                 Expects(o.get());
                                 return o.get();
                              });
               fd.p2ds.clear();
               fd.p2ds.reserve(frame.p2d_info_ptrs.size());
               fd.nodes.clear();
            }
         };

         frame_dat.reserve(size_t(n_frames));
         if(prev_tracklet) fill_frame_dat(prev_tracklet);
         fill_frame_dat(tracklet);

         { // sanity
            Expects(frame_dat.size() == size_t(n_frames));
            int t = start_frame;
            for(const auto& fd : frame_dat) Expects(fd.t == t++);
         }
      }

      { // Now place op_skel_2d pointers into frames
         for(auto& skel : pose_dat) {
            Expects(size_t(skel.t - start_frame) < frame_dat.size());
            auto& fd = frame_dat.at(size_t(skel.t - start_frame));
            Expects(fd.t == skel.t);
            fd.p2ds.push_back(&skel);
         }
      }

      pose_dat_sz0          = int(pose_dat.size());
      timing_setup_pose_dat = float(ms_tock_f(now));
   }

   { // Get Blurred LABs
      const auto now = tick();

      const int n_sensors = scene_desc.n_sensors();
      blur_labs.resize(size_t(n_sensors));
      for(int j = 0; j < n_sensors; ++j)
         pjobs.schedule([j, this]() {
            blur_labs[size_t(j)]
                = LAB_vec3f_im_to_LAB(scene_desc.accum_blurred_LAB(j));

            if(false) {
               const Point2 xy = scene_desc.bcam_lookup(j);
               TRACE(format("blur-lab[{}]: cam[{},{}] [{}x{}]",
                            j,
                            xy.x,
                            xy.y,
                            blur_labs[size_t(j)].width,
                            blur_labs[size_t(j)].height));
            }

            if(false && blur_labs[size_t(j)].size() > 0) {
               const string fname
                   = format("{}/still_{}_{}.png", outdir, start_frame, j);
               LAB_im_to_argb(blur_labs[size_t(j)]).save(fname);
            }
         });

      timing_calc_blurred_labs = float(ms_tock_f(now));
   }

   { // Init labels
      const auto now = tick();

      { // The maximum number of tracks for a given frame
         max_n_labels = 0;
         for(const auto& frame : frame_dat) {
            size_t non_interp_count = 0;
            for(const auto& p2d_ptr : frame.p2ds) {
               if(!p2d_ptr->is_interpolation()) non_interp_count++;
            }
            if(max_n_labels < non_interp_count) max_n_labels = non_interp_count;
         }

         if(prev_track != nullptr && max_n_labels < prev_track->seqs.size())
            max_n_labels = prev_track->seqs.size();

         max_n_labels = size_t(1.5 * real(max_n_labels));
         max_n_p2ds   = pose_dat.size();
      }

      { // Initialize the pose labels
         label_p2d_scores.resize(long(max_n_labels), long(max_n_p2ds));
         for(auto row = 0u; row < max_n_labels; ++row) {
            for(auto col = 0u; col < max_n_p2ds; ++col) {
               // TRACE(format("clear-track-p2d({}, {})", row, col));
               clear_track_p2d(row, col);
            }
         }
      }

      timing_init_labels = float(ms_tock_f(now));
   }

   { // p2d lookup
      const auto now = tick();

      p2d_lookup.clear();
      p2d_lookup.reserve(pose_dat.size());
      for(auto& op_skel : pose_dat) {
         const auto address = op_skel.p2d_address();
         Expects(p2d_lookup.find(address) == cend(p2d_lookup));
         p2d_lookup.insert({address, &op_skel});
      }

      timing_create_opskeleton_2d_lookup = float(ms_tock_f(now));
   }

   { // Create false positives
      const auto now = tick();

      auto process_i = [&](size_t i) {
         OpSkeleton2D& o       = pose_dat[i];
         o.prob_false_positive = calc_prob_false_positive(o);
      };

      for(size_t i = 0; i < pose_dat.size(); ++i)
         pjobs.schedule([i, &process_i]() { process_i(i); });
      pjobs.execute();

      timing_calc_prob_false_positive = float(ms_tock_f(now));
   }

   { // Create graph edges
      const auto now = tick();

      auto pose_dat_idx = [&](const OpSkeleton2D* ptr) -> int {
         int ret = int(ptr - &pose_dat[0]);
         Expects(size_t(ret) < pose_dat.size());
         Expects(&pose_dat[size_t(ret)] == ptr);
         return ret;
      };

      const int N = int(pose_dat.size()); // number of vertices
      vector<unsigned> counters((size_t(N)), 0);
      vector<std::pair<int, int>> p2ds_to_process;
      vector<SpinLock> padlocks((size_t(N)));
      p2ds_to_process.reserve(size_t(N * N));

      { // For each p2d-pair in the same frame (same or different sensor)
         for(const auto& f_dat : frame_dat) {
            for(size_t i = 0; i < f_dat.p2ds.size(); ++i) {
               if(f_dat.p2ds[i]->is_interpolation()) continue;
               const int sensor_no_i = f_dat.p2ds[i]->p2d_ptr()->sensor_no();
               for(size_t j = i + 1; j < f_dat.p2ds.size(); ++j) {
                  if(f_dat.p2ds[j]->is_interpolation()) continue;
                  const int sensor_no_j = f_dat.p2ds[j]->p2d_ptr()->sensor_no();
                  // if(sensor_no_i == sensor_no_j) // allow same-sensor
                  //    continue;
                  const auto A_idx = pose_dat_idx(f_dat.p2ds[i]);
                  const auto B_idx = pose_dat_idx(f_dat.p2ds[j]);
                  p2ds_to_process.emplace_back(A_idx, B_idx);
                  counters[size_t(A_idx)]++;
                  counters[size_t(B_idx)]++;
               }
            }
         }
      }

      // Now reserve space for each p2d
      for(size_t i = 0; i < size_t(N); ++i) {
         pose_dat[i].edges.reserve(counters[i]);
      }

      // Process a p2d pair
      auto process_p2d_pair = [&](size_t idx) {
         const auto [A_idx, B_idx] = p2ds_to_process[idx];
         OpSkeleton2D& A           = pose_dat[size_t(A_idx)];
         OpSkeleton2D& B           = pose_dat[size_t(B_idx)];

         const bool is_of_interest = calc_id_of_interest(A, B);

         Edge e            = make_edge(*A.p2d_info_ptr,
                            *B.p2d_info_ptr,
                            A.t,
                            B.t,
                            unsigned(A_idx),
                            unsigned(B_idx));
         const bool is_dt0 = (e.type == Edge::T0);

         if(is_of_interest) { TRACE(format("edge-weight was: {}", e.weight)); }

         if(std::isfinite(e.weight)) {
            Expects(std::isfinite(e.lab_patch_score));
            { // Directed graph
               lock_guard lock(padlocks[size_t(A_idx)]);
               A.edges.emplace_back(std::move(e));
            }

            if(is_dt0 && e.interp_ptr == nullptr) {
               Edge o = e;
               std::swap(o.this_idx, o.other_idx);
               {
                  lock_guard lock(padlocks[size_t(B_idx)]);
                  B.edges.emplace_back(std::move(o));
               }
            }
         }
      };

      // Enqueue the jobs
      for(size_t i = 0; i < p2ds_to_process.size(); ++i)
         pjobs.schedule([i, &process_p2d_pair]() { process_p2d_pair(i); });
      pjobs.execute();

      { // Create the edge lookup
         size_t sz = 0;
         for(const auto& o : pose_dat) { sz += o.edges.size(); }
         edge_lookup.reserve(sz);
         for(auto& o : pose_dat) {
            for(Edge& e : o.edges) {
               const uint64_t key = (uint64_t(e.this_idx) << 32)
                                    | (uint64_t(e.other_idx) << 0);
               edge_lookup.insert({key, &e});
            }
         }
      }

      timing_create_graph_edges = float(ms_tock_f(now));
   }

   { // Create frame XYT candidates
      const auto now = tick();

      for(auto i = 0u; i < frame_dat.size(); ++i)
         pjobs.schedule([i, this]() { init_frame_xyt_nodes_(frame_dat[i]); });
      pjobs.execute();

      { // Give each node a unique id, and initialize xnode-lookup
         int counter = 0;
         for(auto& frame : frame_dat) {
            if(false && is_trace_mode)
               TRACE(format("CREATE frame.t = {}", frame.t));
            for(auto& node : frame.nodes) {
               node.set_id(counter++);
               if(false && is_trace_mode)
                  cout << "   " << node.to_string() << endl;
            }
         }

         // Setup xnode_lookup
         auto insert_into_xnode_lookup = [&](auto op, XYTNode* o) {
            Expects(xnode_lookup.count(op) == 0);
            xnode_lookup[op] = o;
         };
         xnode_lookup.reserve(size_t(counter));
         for(auto& frame : frame_dat)
            for(auto& node : frame.nodes)
               for(auto& skel : node.xyt().skels)
                  insert_into_xnode_lookup(skel->p2d_address(), &node);

         if(false && is_trace_mode) {
            for(auto& frame : frame_dat) {
               TRACE(format("nodes for frame.t = {}", frame.t));
               for(auto& node : frame.nodes)
                  for(auto& skel : node.xyt().skels)
                     cout << "   " << skel->p2d_address().to_string() << endl;
            }
         }
      }

      timing_create_frame_xyts = float(ms_tock_f(now));
   }

   { // Create XYT still-scores
      const auto now = tick();

      for(auto& frame : frame_dat)
         for(auto& node : frame.nodes)
            pjobs.schedule(
                [this, &node]() { set_still_score(*this, node.xyt()); });
      pjobs.execute();

      timing_create_frame_xyt_still_scores = float(ms_tock_f(now));
   }

   { // Create XYT edges
      const auto now = tick();
      init_xyt_edges_();
      timing_create_xyt_edges = float(ms_tock_f(now));
   }

   { // Fixed inchoate tracks
      const auto now = tick();
      init_fixed_inchoate_tracks_();
      timing_create_fixed_inchoate_tracks = float(ms_tock_f(now));
   }

   { // Topologically sorted nodes
      const auto now      = tick();
      auto flat_xyt_nodes = [&]() -> vector<XYTNode*> {
         int count = 0;
         for(const auto& frame : frame_dat) count += frame.nodes.size();
         vector<XYTNode*> out;
         out.reserve(size_t(count));
         for(auto& frame : frame_dat)
            for(auto& node : frame.nodes) out.push_back(&node);
         return out;
      };

      sorted_xyt_nodes = topological_sort(flat_xyt_nodes());
      for(size_t i = 0; i < sorted_xyt_nodes.size(); ++i)
         sorted_xyt_nodes[i]->set_topological_pos(int(i));
      timing_topological_sort = float(ms_tock_f(now));
   }

   { // filter ground-truth for
      const auto now             = tick();
      const int this_start_frame = in_tracklet->start_frame;
      ground_truth_tps.reserve(100);
      for(const auto& tt : params.ground_truth.tracks)
         for(const auto& tp : tt.path)
            if(inclusive_between(this_start_frame, tp.t, end_frame))
               ground_truth_tps.push_back(tp);

      for(auto& fdat : frame_dat) {
         vector<Vector2> pts;
         for(const auto& tp :
             views::filter(ground_truth_tps,
                           [&](const auto& tp) { return fdat.t == tp.t; })) {
            pts.push_back(hist_to_X(tp.xy()));
            fdat.gt_tps.push_back(tp);
         }
         fdat.gt_spatial_index.init(pts);
      }
      timing_filter_ground_truth = float(ms_tock_f(now));
   }

   { // Sanity
      const auto now = tick();
      check_invariants_();
      timing_check_invariants = float(ms_tock_f(now));
   }

   tot_init_time = float(ms_tock_f(now0));
}

// ---------------------------------------------------------------- find-op-skel
//
This::OpSkeleton2D* This::find_op_skel(const P2dAddress& address) noexcept
{
   auto ii = p2d_lookup.find(address);
   Expects(ii != cend(p2d_lookup));
   OpSkeleton2D* op = ii->second;
   Expects(op->is_interpolation() == false);
   return op;
}

// ------------------------------------------------------ convert-pos-info-score
// Convert a pos-info score, such that lower is better, and
// less than zero indicates a match.
float This::convert_pos_info_score(const float score) const noexcept
{
   const auto w1     = std::clamp<float>(1.0f - score, 0.0f, 1.0f);
   const auto weight = (w1 + params.edge_d0_delta) * params.edge_d0_scale;
   return weight;
}

// -------------------------------------------------- init-frame-xyt-fixed-nodes
// * One thread operates on a `frame` at a time
void This::init_frame_fixed_xyt_nodes_(OpFrameData& frame,
                                       vector<bool>& marks) noexcept
{
   const vector<OpSkeleton2D*>& p2ds = frame.p2ds;
   Expects(p2ds.size() == marks.size());

   auto make_xyt_node_for_fixed_track = [&](const Node& node) -> XYTNode {
      Expects(node.p2ds().size() > 0);

      vector<OpSkeleton2D*> skels;
      skels.reserve(node.p2ds().size());

      // We find every skel for node.addresses
      for(const auto& address : node.p2ds()) {
         auto ii = std::find_if(
             cbegin(p2ds), cend(p2ds), [&address](const auto ptr) -> bool {
                return ptr->p2d_address() == address;
             });
         if(ii == cend(p2ds)) {
            LOG_ERR(format("failed to find address: {}", address.to_string()));
            cout << format("p2ds.size() == {}, t == {}", p2ds.size(), node.t())
                 << endl;
            for(const auto& p2d : p2ds) {
               cout << "   " << p2d->p2d_address().to_string() << endl;
            }
            Expects(false);
         }
         int idx = int(std::distance(cbegin(p2ds), ii));
         if(marks[size_t(idx)])
            FATAL(format(
                "address {} used twice?? idx = {}", address.to_string(), idx));
         Expects(marks[size_t(idx)] == false);
         marks[size_t(idx)] = true;
         skels.push_back(*ii);
      }

      Expects(skels.size() > 0);

      XYTNode out;
      out.set_xyt(XYT{*this, std::move(skels), node.t()});
      return out;
   };

   // Create nodes for fixed tracks
   const bool is_fixed = is_fixed_track_frame(frame.t);
   if(is_fixed) {
      Expects(prev_track);
      for(const auto& seq : prev_track->seqs)
         for(const auto& node : seq.nodes())
            if(node.t() == frame.t && !node.is_interp())
               frame.nodes.push_back(make_xyt_node_for_fixed_track(node));
   }
}

// -------------------------------------------------------- init-frame-xyt-nodes
// * One thread operates on a `frame` at a time
void This::init_frame_xyt_nodes_(OpFrameData& frame) noexcept
{
   const vector<OpSkeleton2D*>& p2ds = frame.p2ds;
   vector<bool> marks(p2ds.size(), false);

   // Fixed XYTs from previous track
   if(is_fixed_track_frame(frame.t)) init_frame_fixed_xyt_nodes_(frame, marks);

   const int total_p2ds = int(p2ds.size());
   const int n_fixed_xyt_nodes
       = int(frame.nodes.size()); // never touch these xyts
   int n_assigned = std::accumulate(cbegin(marks), cend(marks), 0);
   std::unordered_map<OrderedPair, float> score_lookup;

   struct NodeProposal
   {
      NodeProposal(const SceneDescription* scene_desc,
                   int i,
                   const OpSkeleton2D* op_skel,
                   real hist_sz,
                   const AABB& aabb)
      {
         const Skeleton2DInfo* info = op_skel->p2d_info_ptr;
         calc                       = position_p2d_infos_calculator(
             scene_desc, &info, 1, hist_sz, aabb);
         inds.push_back(i);
      }

      NodeProposal(NodeProposal&&) noexcept = default;
      NodeProposal& operator=(NodeProposal&&) noexcept = default;

      Skeleton2DInfoPositionCalculator calc;
      vector<int> inds; // p2d_inds
      size_t size() const noexcept { return inds.size(); }

      bool has_mark(const vector<bool>& marks) const noexcept
      {
         return std::any_of(cbegin(inds), cend(inds), [&](int i) {
            return marks.at(size_t(i));
         });
      }

      float score(const OpSkeleton2D* op_skel) const noexcept
      {
         return calc.score(op_skel->p2d_info_ptr).score;
      }

      void add_p2d_ind(const int p2d_ind,
                       const Skeleton2DInfo* info_ptr) noexcept
      {
         inds.push_back(p2d_ind);
         calc.add_skel_info(info_ptr);
      }
   };

   vector<NodeProposal> proposals;
   { // Create a proposal for every unassigned Skeleton
      proposals.reserve(size_t(total_p2ds - n_assigned));
      std::array<const Skeleton2DInfo*, 1> skels;
      for(int i = 0; i < total_p2ds; ++i) {
         if(marks[size_t(i)]) continue;
         proposals.emplace_back(&scene_desc, i, p2ds[size_t(i)], hist_sz, aabb);
      }
   }

   // Merge proposals, AND remove `ind1` from `proposals`
   auto merge_proposals = [&](int ind0, int ind1) {
      Expects(size_t(ind0) < proposals.size());
      Expects(size_t(ind1) < proposals.size());
      auto& A = proposals[size_t(ind0)];
      auto& B = proposals[size_t(ind1)];
      Expects(B.inds.size() == 1);
      for(auto p2d_ind : B.inds)
         A.add_p2d_ind(p2d_ind, p2ds[size_t(p2d_ind)]->p2d_info_ptr);

      { // Remove `ind1` from consideration
         using std::swap;
         if(size_t(ind1 + 1) != proposals.size())
            swap(proposals[size_t(ind1)], proposals.back());
         proposals.pop_back();
      }
   };

   while(true) {
      // Find the best merge possible
      Point2 best      = {-1, -1};
      float best_score = std::numeric_limits<float>::lowest();
      for(size_t i = 0; i < proposals.size(); ++i) {
         for(size_t j = i + 1; j < proposals.size(); ++j) {
            const bool i_is_1 = (proposals[i].inds.size() == 1);
            auto& A           = i_is_1 ? proposals[i] : proposals[j];
            auto& B           = i_is_1 ? proposals[j] : proposals[i];
            if(B.size() != 1) continue;
            const float score = A.score(p2ds.at(size_t(B.inds.at(0))));
            if(score > best_score) {
               best = Point2(int((i_is_1) ? i : j), int((i_is_1) ? j : i));
               best_score = score;
            }
         }
      }

      // Do that one merge
      bool did_merge = false;
      if(best_score > std::numeric_limits<float>::lowest()
         && convert_pos_info_score(best_score) < 0.0f) {
         merge_proposals(best[0], best[1]);
         did_merge = true;
      }

      // If we failed to merge, then we're done.
      if(!did_merge) break;
   }

   // Now convert proposals to nodes...
   frame.nodes.reserve(frame.nodes.size() + proposals.size());
   std::transform(cbegin(proposals),
                  cend(proposals),
                  std::back_inserter(frame.nodes),
                  [&](const auto& proposal) -> XYTNode {
                     Expects(proposal.size() > 0);
                     std::stringstream ss{""};
                     vector<OpSkeleton2D*> skels(proposal.size());
                     int counter = 0;
                     std::transform(cbegin(proposal.inds),
                                    cend(proposal.inds),
                                    begin(skels),
                                    [&](const int ind) {
                                       Expects(marks.at(size_t(ind)) == false);
                                       marks[size_t(ind)] = true;
                                       auto ret = p2ds.at(size_t(ind));
                                       Expects(ret != nullptr);
                                       return ret;
                                    });

                     Expects(skels.size() > 0);

                     XYTNode out;
                     out.set_xyt(XYT{*this, std::move(skels), frame.t});
                     return out;
                  });

   // Now remove suspect tracks
   auto ii = std::stable_partition(
       begin(frame.nodes), end(frame.nodes), [&](const auto& node) {
          return aabb.contains(real(node.x()), real(node.y()));
       });
   frame.nodes.erase(ii, end(frame.nodes));

   // Fix the frame-ids
   int counter = 0;
   for(auto& node : frame.nodes) {
      node.set_frame_id(counter++);
      Expects(&node - &(frame_dat.at(size_t(node.t() - start_frame)).nodes[0])
              == node.frame_id());
   }
}

// -------------------------------------------------------------- init-xyt-edges
//
void This::init_xyt_edges_() noexcept
{
   if(multiview_trace_mode()) { // For graphing speed-p values
      std::stringstream ss{""};
      const auto dt0 = 1.0 / 15.0;
      const auto dt1 = 1.0;
      const auto ddt = dt0;
      const auto dx0 = 0.0;
      const auto dx1 = 4.0;
      const auto ddx = 0.1;

      { // Header
         for(auto dx = dx0; dx < dx1; dx += ddx) ss << format("\t{}", dx);
         ss << "\n";
      }

      for(auto dt = dt0; dt <= dt1; dt += ddt) {
         ss << format("{}", dt);
         for(auto dx = dx0; dx < dx1; dx += ddx)
            ss << format("\t{}", calc_speed_p(float(dx), float(dt)));
         ss << "\n";
      }
      file_put_contents(format("{}/speed-p.csv", outdir), ss.str());
   }

   // -- (*) -- Test if XYTNode A and B are candidates
   auto is_xyt_edge_candidate = [&](XYTNode& A, XYTNode& B) -> bool {
      Expects(B.t() - A.t() > 0);
      const int delta_t       = B.t() - A.t();
      const auto dist         = (A.xy() - B.xy()).norm();
      const auto duration     = float(frame_duration) * float(delta_t);
      const auto speed        = dist / duration;
      const bool is_candidate = (speed <= tracklet_params.speed_cap);

      if(delta_t > 1 && is_candidate) {
         // Okay, we're looking at an interpolation, which can only
         // be "same-sensor" for at least one pair of skeletons....
         bool at_least_one = false;
         for(const auto sa : A.xyt().skels) {
            if(at_least_one) break;
            for(const auto sb : B.xyt().skels) {
               const auto sensor0 = sa->p2d_info_ptr->p2d_ptr->sensor_no();
               const auto sensor1 = sb->p2d_info_ptr->p2d_ptr->sensor_no();
               if(sensor0 == sensor1) {
                  at_least_one = true;
                  break;
               }
            }
         }

         return at_least_one;
      }

      return is_candidate;
   };

   auto calc_labs = [&](const OpSkeleton2D* sa,
                        const OpSkeleton2D* sb) -> std::pair<float, float> {
      const auto dt = (sb->t - sa->t);
      Expects(dt > 0);

      if(dt == 1) return calc_lab_score(*sa->p2d_info_ptr, *sb->p2d_info_ptr);

      const auto p2d_info0   = sa->p2d_info_ptr;
      const auto p2d_info1   = sb->p2d_info_ptr;
      const Skeleton2D& p2d0 = *p2d_info0->p2d_ptr;
      const Skeleton2D& p2d1 = *p2d_info1->p2d_ptr;
      const int sensor0      = p2d0.sensor_no();
      const int sensor1      = p2d1.sensor_no();
      const auto& dcam0      = scene_desc.dcam(p2d0.sensor_no());
      const auto& dcam1      = scene_desc.dcam(p2d1.sensor_no());

      if(sensor0 != sensor1) return {fNAN, fNAN};

      const auto p2d_params = Skeleton2D::Params{};
      auto interp_ptr
          = make_unique<Skeleton2DInterpolation>(projective_interpolation(
              interp_params,
              p2d_params,
              p2d0,
              p2d1,
              p2d_info0->patches,
              p2d_info1->patches,
              &dcam0,
              [this](int frame_no, int sensor_no) -> const LABImage* {
                 return get_lab_ptr(frame_no, sensor_no);
              }));

      const float lab_patch_score = interp_ptr->lab_score; // an average
      const float lab_p           = lab_score_to_lab_p(lab_patch_score);

      return {lab_patch_score, lab_p};
   };

   // -- (*) -- Make an XYT Edge
   auto finish_xyt_edge = [&](XYTEdge& e) {
      Expects(e.src != nullptr);
      Expects(e.dst != nullptr);
      Expects(!std::isfinite(e.weight));
      XYTNode& A = *e.src;
      XYTNode& B = *e.dst;
      Expects(B.t() - A.t() > 0);

      const int dt             = B.t() - A.t();
      const bool is_dt1        = (dt == 1);
      const float dist         = (A.xy() - B.xy()).norm();
      const float noise_factor = tracklet_params.noise_factor;
      const float duration     = float(frame_duration) * float(dt); // seconds
      const float speed        = (dist / duration); // metres / second
      const float speed_p      = calc_speed_p(dist, duration);

      float sum_lab_s = 0.0f;
      float sum_lab_p = 0.0f;
      int counter     = 0;
      for(const OpSkeleton2D* sa : A.xyt().skels) {
         for(const OpSkeleton2D* sb : B.xyt().skels) {
            const auto [lab_score, lab_p] = calc_labs(sa, sb);
            // Scores could be infinite if we're interpolating acorss
            // separate frames
            if(std::isfinite(lab_score) && std::isfinite(lab_p)) {
               sum_lab_s += lab_score;
               sum_lab_p += lab_p;
               ++counter;
            }
         }
      }
      Expects(counter > 0);
      const float lab_s = sum_lab_s / float(counter);
      const float lab_p = sum_lab_p / float(counter);

      if(dt < 1) {
         Expects(false);
      } else if(dt == 1) {
         auto w1  = std::clamp<float>(0.5f * (speed_p + lab_p), 0.0f, 1.0f);
         e.weight = (w1 + params.edge_d1_delta) * params.edge_d1_scale;
      } else {
         auto w1 = std::clamp<float>(
             0.5f * (speed_p + lab_p) + float(dt) * 0.05f, 0.0f, 1.0f);
         e.weight = (w1 + params.edge_dt_delta) * params.edge_dt_scale;
      }
   };

   // -- (*) -- Find and created edges
   {
      // We create all the necessary edges (weights not set), with
      // one job per frame.
      vector<vector<std::tuple<int, int, int>>> candidates(frame_dat.size());
      auto find_and_create = [&](const int i) {
         const int t_max   = int(tracklet_params.frame_t_interp_window);
         const int fdat_sz = int(frame_dat.size());
         auto& frame0      = frame_dat[size_t(i)];
         for(auto j = i + 1; j < fdat_sz && j <= i + t_max; ++j) {
            auto& frame1      = frame_dat[size_t(j)];
            const int delta_t = frame1.t - frame0.t;
            for(size_t u_idx = 0; u_idx < frame0.nodes.size(); ++u_idx) {
               XYTNode& u = frame0.nodes[u_idx];
               for(XYTNode& v : frame1.nodes) {
                  if(is_xyt_edge_candidate(u, v)) {
                     u.edges().emplace_back(&u, &v);
                     candidates[size_t(i)].emplace_back(
                         i, int(u_idx), u.edges().size() - 1);
                  }
               }
            }
         }
      };
      for(size_t i = 0; i < frame_dat.size(); ++i)
         pjobs.schedule([&find_and_create, i]() { find_and_create(int(i)); });
      pjobs.execute();

      // Create 2 jobs per hardware thread, and finish all those edges
      auto pairs = flatten(candidates);

      std::atomic<size_t> next_pair{0};
      auto make_edges = [&]() {
         while(true) {
            const size_t ind = next_pair++;
            if(ind >= pairs.size()) return; // we're done
            const auto [frame_i, node_ind, e_ind] = pairs[ind];
            Expects(size_t(frame_i) < frame_dat.size());
            Expects(size_t(node_ind) < frame_dat[size_t(frame_i)].nodes.size());
            Expects(size_t(e_ind) < frame_dat[size_t(frame_i)]
                                        .nodes[size_t(node_ind)]
                                        .edges()
                                        .size());
            XYTEdge& e = frame_dat[size_t(frame_i)]
                             .nodes[size_t(node_ind)]
                             .edges()[size_t(e_ind)];
            finish_xyt_edge(e);
         }
      };
      for(auto i = 0u; i < 2 * hardware_concurrency(); ++i)
         pjobs.schedule(make_edges);
      pjobs.execute();
   }

   for(auto& frame : frame_dat) {
      for(auto& node : frame.nodes) {
         for(auto& e : node.edges()) {
            Expects(e.src != nullptr);
            Expects(e.dst != nullptr);
         }
      }
   }
}

// -------------------------------------------------- init-fixed-inchoate-tracks
//
void This::init_fixed_inchoate_tracks_() noexcept
{
   auto set_inchoate_model = [&](InchoateTrack& it) {
      auto count_skels = [](const InchoateTrack& it) {
         size_t ret = 0;
         for(const auto& xyt : it.xyts) ret += xyt.skels.size();
         return ret;
      };
      const size_t n_skels = count_skels(it);

      auto process_xyt = [&](const XYT& xyt) {
         for(const auto& skel_ptr : xyt.skels) {
            Expects(skel_ptr->patches() != nullptr);
            auto& patches = *skel_ptr->patches();
            if(it.model.size() == 0) {
               init_patches(it, patches);
            } else if(it.model.size() == patches.size()) {
               for(size_t j = 0; j < patches.size(); ++j)
                  sum_lab(it.model[j], patches[j]);
               it.model_sz += 1;
            } else {
               Expects(false);
            }
         }
      };

      for(const auto& xyt : it.xyts) { process_xyt(xyt); }

      if(it.edges.size() > 0) {
         process_xyt(it.edges.front()->src->xyt());
         for(const auto e_ptr : it.edges) process_xyt(e_ptr->dst->xyt());
      }
   };

   auto find_node = [this](const Node& o) -> XYTNode* {
      // Find an XYTNode with addresses in o.p2ds()
      XYTNode* out = nullptr;

      bool has_error = false;
      for(const auto& address : o.p2ds()) {
         auto ii = xnode_lookup.find(address);
         Expects(ii != cend(xnode_lookup));
         XYTNode* xnode = ii->second;
         Expects(xnode != nullptr);
         if(out == nullptr) {
            out = xnode;
         } else if(xnode != out) {
            if(!has_error) {
               LOG_ERR(format("XYTNode mismatch!"));
               has_error = true;
               cout << out->to_string() << endl;
            }
            cout << xnode->to_string() << endl;
         }
      }

      if(has_error) {
         cout << xnode_lookup_string(o.t()) << endl;
         cout << endl;
         cout << frame_dat_xytnode_string(o.t()) << endl;
         FATAL(format("kBAM!"));
      }

      return out;
   };

   auto convert_node_vec = [&](InchoateTrack& it, const vector<Node>& nodes) {
      vector<XYTNode*> xnodes;
      xnodes.reserve(nodes.size());

      for(const auto& o : nodes) { // Convert SEQ to XYTNode*
         const bool is_fixed = is_fixed_track_frame(o.t());
         if(!is_fixed) continue;
         if(o.is_interp()) continue;
         XYTNode* x = find_node(o);
         if(x == nullptr) {
            WARN(format("failed to find XYTNode for o = {}", str(o)));
            continue;
         }
         xnodes.push_back(x);
      }

      // Convert `xnodes` into a sequence of edges
      // We cannot fix a track of length 1, there's no edges!
      it.edges.clear();
      for(size_t i = 1; i < xnodes.size(); ++i) {
         XYTEdge* e_ptr = xnodes[i - 1]->find_edge(xnodes[i - 0]);
         if(e_ptr == nullptr) {
            WARN(format("failed to find edge between nodes, skipping "
                        "node sequence!"));
            break;
         }
         it.edges.push_back(e_ptr); // e_ptr->...->inchoate_id() set below
      }
   };

   // Create the inchoate-tracks
   inchoates.clear();
   inchoates.reserve(max_n_labels);

   // Add in existing tracks
   if(prev_track != nullptr) {
      auto process_i = [&](size_t idx) {
         const NodeSequence& seq = prev_track->seqs[idx];
         InchoateTrack& it       = inchoates[idx];
         it.idx                  = int(idx);
         it.node_seq_id          = seq.id();
         convert_node_vec(it, seq.nodes());
         set_inchoate_model(it);
      };

      inchoates.resize(prev_track->seqs.size());
      for(size_t i = 0; i < prev_track->seqs.size(); ++i)
         pjobs.schedule([i, &process_i] { process_i(i); });
      pjobs.execute();
   }

   { // Remove empty tracks
      auto ii
          = std::partition(begin(inchoates),
                           end(inchoates),
                           [](auto& it) -> bool { return !it.is_available(); });
      inchoates.erase(ii, end(inchoates));
   }

   // Fix `idx`
   for(size_t i = 0; i < inchoates.size(); ++i) {
      inchoates[i].idx = int(i);
      for(auto edge_ptr : inchoates[i].edges) edge_ptr->set_inchoate_id(int(i));
   }

   check_invariants_();
}

// ---------------------------------------------------------------------- random
//
int This::random(int min_val, int max_val) noexcept // inclusive
{
   std::uniform_int_distribution<int> distribution{min_val, max_val};
   return distribution(random_generator);
}

float This::calc_speed_p(const float dist, const float duration) const noexcept
{
   Expects(duration > 0.0f);
   const float speed = dist / duration;
   const float speed_z
       = (speed - tracklet_params.speed_median) / tracklet_params.speed_stddev;
   return (dist < tracklet_params.noise_factor)
              ? (0.01f * dist / tracklet_params.noise_factor)
              : (1.0f - std::exp(-0.5f * square(std::max(0.0f, speed_z))));
}

// ------------------------------------------------------------- handling-scores
//
float This::track_p2d_score(unsigned label, unsigned p2d_idx) const noexcept
{
   Expects(label < label_p2d_scores.rows());
   Expects(p2d_idx < label_p2d_scores.cols());
   return label_p2d_scores(label, p2d_idx);
}

void This::set_track_p2d_score(unsigned label,
                               unsigned p2d_idx,
                               float val) noexcept
{
   Expects(label < label_p2d_scores.rows());
   Expects(p2d_idx < label_p2d_scores.cols());
   label_p2d_scores(label, p2d_idx) = val;
}

void This::clear_track_p2d(unsigned label, unsigned p2d_idx) noexcept
{
   set_track_p2d_score(label, p2d_idx, fNAN);
}

bool This::has_track_p2d(unsigned label, unsigned p2d_idx) const noexcept
{
   return std::isfinite(track_p2d_score(label, p2d_idx));
}

// -------------------------------------------------------- is-fixed-track-frame
//
bool This::is_fixed_track_frame(int frame_no) const noexcept
{
   return (prev_tracklet != nullptr)
          && unsigned(frame_no - start_frame)
                 < params.fixed_window_prev_tracklet;
}

// ------------------------------------------------------------------ total-cost
//
real This::total_cost(const vector<NodeSequence>& seqs) noexcept
{
   //
   return 0.0;
}

// ---------------------------------------------------- calc prob false positive
//
float This::calc_prob_false_positive(const OpSkeleton2D& op_p2d) noexcept
{
   const auto& p2d = *op_p2d.p2d_ptr();
   Expects(unsigned(p2d.sensor_no()) < unsigned(scene_desc.n_sensors()));
   const int sensor_no = p2d.sensor_no();
   const auto& dcam    = scene_desc.dcam(sensor_no);

   const Vector4f weights = to_vec4f(params.false_positive_weight_vector);

   const auto ret
       = calc_prob_p2d_false_positive(loc_params,
                                      p2d,
                                      op_p2d.hist_ptr(),
                                      scene_desc.scene_info.scene_bounds(),
                                      hist_sz,
                                      dcam,
                                      weights);

   return ret;
}

const LABImage* This::get_lab_ptr(int frame_no, int sensor_no) const
{
   int idx = frame_no - start_frame;
   Expects(idx >= 0 && idx < int(frame_dat.size()));
   const auto& labs = frame_dat[size_t(idx)].sensor_labs;
   if(size_t(sensor_no) >= labs.size())
      LOG_ERR(format("sensor = {} >= |labs| = {}. Frame-no = {}",
                     sensor_no,
                     labs.size(),
                     frame_no));
   Expects(size_t(sensor_no) < labs.size());
   auto lab_ptr = labs[size_t(sensor_no)];
   Expects(lab_ptr != nullptr);
   return lab_ptr;
}

This::Edge This::make_edge(const Skeleton2DInfo& p2d_info0,
                           const Skeleton2DInfo& p2d_info1,
                           const int frame0,
                           const int frame1,
                           const uint32_t this_idx,
                           const uint32_t other_idx) noexcept
{
   Expects(frame1 >= frame0);

   const auto now = tick();

   const Skeleton2D& p2d0   = *p2d_info0.p2d_ptr;
   const Skeleton2D& p2d1   = *p2d_info1.p2d_ptr;
   const int sensor0        = p2d0.sensor_no();
   const int sensor1        = p2d1.sensor_no();
   const auto& dcam0        = scene_desc.dcam(p2d0.sensor_no());
   const auto& dcam1        = scene_desc.dcam(p2d1.sensor_no());
   const auto X0            = clip_to_xy(p2d0.best_3d_result().Xs_centre());
   const auto X1            = clip_to_xy(p2d1.best_3d_result().Xs_centre());
   const auto dist          = (X1 - X0).norm();
   const int dt             = frame1 - frame0;
   const float duration     = float(frame_duration) * float(dt); // seconds
   const float noise_factor = tracklet_params.noise_factor;
   const float speed = (dt == 0) ? 0.0f : (dist / duration); // metres / second
   const float speed_z
       = (speed - tracklet_params.speed_median) / tracklet_params.speed_stddev;

   if(dt == 0 && dist > 2.0f) return {}; // distance too far for node
   if(dt >= 1 && speed > tracklet_params.speed_cap) return {};
   if(dt > 1 && sensor0 != sensor1) return {}; // interp requires same sensor

   float lab_patch_score                          = fNAN;
   float lab_p                                    = fNAN;
   float score                                    = fNAN;
   Vector3f X                                     = {};
   float weight                                   = fNAN;
   unique_ptr<Skeleton2DInterpolation> interp_ptr = nullptr;
   float loc_p                                    = fNAN;

   auto speed_p = [&]() {
      return (dist < noise_factor)
                 ? (0.01f * dist / noise_factor)
                 : (1.0f - std::exp(-0.5f * square(std::max(0.0f, speed_z))));
   };

   const auto& ldat0 = *(frame_dat.at(size_t(frame0 - start_frame)).loc_ptr);

   auto hist_cost_X = [&](const int frame_no, const auto& X) -> float {
      const auto idx = frame_no - start_frame;
      Expects(size_t(idx) < frame_dat.size());
      Expects(frame_dat[size_t(idx)].loc_ptr);
      return frame_dat[size_t(idx)].loc_ptr->hist_cost_X_01(X);
   };

   if(dt == 0) {
      auto is_of_interest = [&]() -> bool { return frame0 == 40; };

      std::array<const Skeleton2DInfo*, 2> infos = {&p2d_info0, &p2d_info1};
      const auto pos_info                        = position_p2d_infos(
          &scene_desc, &infos[0], infos.size(), hist_sz, aabb);
      X = hist_to_X3f(pos_info.xy);

      std::tie(lab_patch_score, lab_p) = calc_lab_score(p2d_info0, p2d_info1);
      score                            = pos_info.score; // Higher is better
      // But `w1` must be "lower is better".
      auto w1 = std::clamp<float>(1.0f - score, 0.0f, 1.0f);
      weight  = convert_pos_info_score(score);
      loc_p   = hist_cost_X(frame0, X);

      if(false && is_of_interest()) {
         TRACE(format(R"V0G0N(
edge {} <-> {}
   pos-info: count={}, xy={}, score={}
   X:        {} 
   patch-s:  {}  
   lab_p:    {}
   w1:       {}
   weight:   {}
)V0G0N",
                      pose_dat[this_idx].p2d_address().to_string(),
                      pose_dat[other_idx].p2d_address().to_string(),
                      pos_info.count,
                      str(pos_info.xy),
                      pos_info.score,
                      str(hist_to_X(pos_info.xy)),
                      lab_patch_score,
                      lab_p,
                      w1,
                      weight));
      }

   } else if(dt == 1) { // 1 frame

      // X is irrelevant
      std::tie(lab_patch_score, lab_p) = calc_lab_score(p2d_info0, p2d_info1);
      score                            = speed_p();
      loc_p   = 0.5f * (hist_cost_X(frame0, X0) + hist_cost_X(frame1, X1));
      auto w1 = std::clamp<float>(0.5f * (score + lab_p), 0.0f, 1.0f);
      weight  = (w1 + params.edge_d1_delta) * params.edge_d1_scale;

      const bool is_of_interest
          = calc_id_of_interest(pose_dat.at(this_idx), pose_dat.at(other_idx));
      if(is_of_interest) {
         TRACE(format(R"V0G0N(
edge {} <-> {}
   X:        |{} - {}| = {}
   speed-p:  {}
   patch-s:  {}  
   lab_p:    {}
   w1:       {}
   weight:   {}
)V0G0N",
                      pose_dat[this_idx].p2d_address().to_string(),
                      pose_dat[other_idx].p2d_address().to_string(),
                      X0.to_string(),
                      X1.to_string(),
                      (X0 - X1).norm(),
                      score,
                      lab_patch_score,
                      lab_p,
                      w1,
                      weight));
      }

   } else { // Interpolation
      Expects(dt > 1);
      Expects(&dcam0 == &dcam1);

      const auto p2d_params = Skeleton2D::Params{};

      interp_ptr
          = make_unique<Skeleton2DInterpolation>(projective_interpolation(
              interp_params,
              p2d_params,
              p2d0,
              p2d1,
              p2d_info0.patches,
              p2d_info1.patches,
              &dcam0,
              [this](int frame_no, int sensor_no) -> const LABImage* {
                 return get_lab_ptr(frame_no, sensor_no);
              }));

      const auto& infos = interp_ptr->p2d_infos;
      Expects(infos.size() > 0);

      lab_patch_score = interp_ptr->lab_score; // an average
      lab_p           = lab_score_to_lab_p(lab_patch_score);

      // X is irrelevant
      score  = speed_p();
      loc_p  = 0.5f * (hist_cost_X(frame0, X0) + hist_cost_X(frame1, X1));
      weight = std::clamp<float>(0.5f * (score + lab_p), 0.0f, 1.0f);

      const bool is_of_interest = false;
      if(is_of_interest) {
         const auto msg = format(
             R"V0G0N(
dt           =  {:02d} - {:02d} = {}
sensor       = [{}, {}]
id           = [{}, {}]
X0, X1       =  {}, {}
X            =  {}
dist/duratn  =  {}, {}
speed/z      =  {}, {}
patch_score  =  {}
s lab/loc-p  = [{}, {}, {}]
weight       =  {}
time         =  {}ms
)V0G0N",
             p2d1.frame_no(),
             p2d0.frame_no(),
             dt,
             p2d0.sensor_no(),
             p2d1.sensor_no(),
             p2d_info0.id,
             p2d_info1.id,
             str(p2d0.best_3d_result().Xs_centre()),
             str(p2d1.best_3d_result().Xs_centre()),
             str(X),
             dist,
             duration,
             speed,
             speed_z,
             lab_patch_score,
             score,
             lab_p,
             loc_p,
             weight,
             ms_tock_s(now));

         TRACE(format("{}", msg));

         if(false) {
            {
               const auto fname
                   = format("/tmp/debug-log_d{}_frame={}-{}_id={}-{}.text",
                            dt,
                            p2d0.frame_no(),
                            p2d1.frame_no(),
                            p2d_info0.id,
                            p2d_info1.id);

               file_put_contents(fname, msg);
            }

            {
               Expects(interp_ptr != nullptr);
               const auto fname
                   = format("/tmp/interp_dt{}_frame={}-{}_id={}-{}.mp4",
                            dt,
                            p2d0.frame_no(),
                            p2d1.frame_no(),
                            p2d_info0.id,
                            p2d_info1.id);
               create_interp_movie(
                   fname,
                   p2d_info0,
                   p2d_info1,
                   *interp_ptr,
                   [this](int frame_no, int sensor_no) -> const LABImage* {
                      return get_lab_ptr(frame_no, sensor_no);
                   });
            }
         }
      }
   }

   Edge edge;
   { // Initialize the output
      edge.X               = Vector2f(X.x, X.y);
      edge.dist            = dist;
      edge.weight          = weight;
      edge.lab_patch_score = lab_patch_score;
      edge.this_idx        = this_idx;
      edge.other_idx       = other_idx;
      edge.type            = (dt == 0)   ? Edge::T0
                             : (dt == 1) ? Edge::T1
                                         : Edge::INTERPOLATION;
      edge.interp_ptr      = std::move(interp_ptr);
   }
   return edge;
}

// ----------------------------------------------------------------- lookup-edge
//
const This::Edge* This::lookup_edge(const uint32_t a_idx,
                                    const uint32_t b_idx) const noexcept
{
   auto ii
       = edge_lookup.find(((uint64_t(a_idx) << 32) | (uint64_t(b_idx) << 0)));
   if(ii == cend(edge_lookup)) return nullptr;
   return ii->second;
}

const This::Edge* This::lookup_edge(const OpSkeleton2D& a,
                                    const OpSkeleton2D& b) const noexcept
{
   return lookup_edge(op_skel_2d_index_(&a), op_skel_2d_index_(&b));
}

// ---------------------------------------------------------- make-min-cut-graph
//
MinCutGraph This::make_min_cut_graph_() noexcept(false)
{
   MinCutGraph g;
   return g;
}

// ------------------------------------------------------------ check-invariants
//
void This::check_inchoate_(const InchoateTrack& inchoate) const noexcept
{
   const auto& edges = inchoate.edges;
   for(size_t i = 0; i < edges.size(); ++i) {
      Expects(edges[i] != nullptr);
      if(i > 0) Expects(edges[i - 1]->dst == edges[i]->src);
      Expects(std::isfinite(edges[i]->weight));
      Expects(edges[i]->src != nullptr);
      Expects(edges[i]->dst != nullptr);
      Expects(edges[i]->src->inchoate_id() == inchoate.idx);
      Expects(edges[i]->dst->inchoate_id() == inchoate.idx);
      Expects(edges[i]->src->t() < edges[i]->dst->t());
   }
}

void This::check_invariants_() const noexcept
{
   for(auto i = 0u; i < inchoates.size(); ++i) {
      const auto& inchoate = inchoates[i];
      Expects(inchoate.idx == int(i));
      //  Expects(inchoate.xyts.size() > 0 || inchoate.edges.size() > 0);
      for(auto j = 0u; j < inchoate.xyts.size(); ++j) {
         Expects(inchoate.xyts[j].skels.size() > 0);
         for(auto k = 0u; k < inchoate.xyts[j].skels.size(); ++k) {
            const auto ptr = inchoate.xyts[j].skels[k];
            Expects(ptr->label == int(i));
            Expects(ptr->offset == Point2(int(j), int(k)));
         }
      }
   }

   for(auto& inchoate : inchoates) { check_inchoate_(inchoate); }
}

// ----------------------------------------------------- update-label-p2d-scores
//
void This::update_label_p2d_scores_() noexcept
{
   //
}

// ------------------------------------------------------------- creating tracks
// DOES NO CHECKING, but we CAN do an interpolation here
//  * Add `op_skel_2d` to `inchoate`
//  * If `o_op_skel_2d` is non-null, then we pop XYTs from `inchoate`
//    such that the XYT we add to has the same `t` as `o_op_skel_2d->t`
static void push_node(This::InchoateTrack& inchoate,
                      This::OpSkeleton2D* op_skel_2d,
                      This::OpSkeleton2D* o_op_skel_2d = nullptr)
{
   Expects(!op_skel_2d->is_assigned());
   op_skel_2d->label = inchoate.idx;
   Expects(o_op_skel_2d == nullptr || o_op_skel_2d->label == inchoate.idx);

   const bool xyt_empty = inchoate.xyts.empty();
   const int op_t       = op_skel_2d->t;

   { // Do we need to pop xyts?
      if(o_op_skel_2d != nullptr) {
         const int o_op_t = o_op_skel_2d->t;
         while(!inchoate.xyts.empty() && inchoate.xyts.back().t > o_op_t) {
            const auto& xyt = inchoate.xyts.back();
            for(auto ptr : xyt.skels) { // unbind the skel
               ptr->label  = -1;
               ptr->offset = Point2{-1, -1};
            }
            inchoate.xyts.pop_back();
         }
         Expects(!inchoate.xyts.empty());
         const auto& xyt = inchoate.xyts.back();
         Expects(xyt.t == o_op_t);
         const auto ii
             = std::find(cbegin(xyt.skels), cend(xyt.skels), o_op_skel_2d);
         Expects(ii != cend(xyt.skels));
      }
   }

   { // Are we pushing a new XYT?
      if(xyt_empty || inchoate.xyts.back().t < op_t) {
         const auto X
             = op_skel_2d->p2d_info_ptr->p2d_ptr->best_3d_result().Xs_centre();
         inchoate.xyts.emplace_back(X.x, X.y, op_t);
      }
   }

   Expects(!inchoate.xyts.empty());
   Expects(inchoate.xyts.back().t == op_t);
   inchoate.xyts.back().skels.push_back(op_skel_2d);
   op_skel_2d->label  = inchoate.idx;
   op_skel_2d->offset = Point2(int(inchoate.xyts.size()) - 1,
                               int(inchoate.xyts.back().skels.size()) - 1);

   { // Update the 'X' position
      auto& xyt = inchoate.xyts.back();
      Expects(!xyt.skels.empty());
      Vector3f X = {0.0f, 0.0f, 0.0f};
      for(const auto ptr : xyt.skels)
         X += ptr->p2d_info_ptr->p2d_ptr->best_3d_result().Xs_centre();
      X /= float(xyt.skels.size());
      xyt.x = X.x;
      xyt.y = X.y;
   }

   { // Update the model -- NOTE, don't worry about popping
      Expects(op_skel_2d->patches() != nullptr);
      const auto& patches = *op_skel_2d->patches();

      if(inchoate.model.size() == 0) {
         init_patches(inchoate, patches);
      } else {
         for(size_t j = 0; j < patches.size(); ++j)
            sum_lab(inchoate.model[j], patches[j]);
         inchoate.model_sz += 1;
      }
   }
}

This::InchoateTrack& This::seed_node_sequence_(OpSkeleton2D* op_skel_2d)
{
   Expects(!op_skel_2d->is_assigned());
   inchoates.emplace_back();
   auto& inchoate = inchoates.back();
   inchoate.idx   = int(inchoates.size()) - 1;
   push_node(inchoate, op_skel_2d);
   return inchoate;
}

void This::extend_to_frame_(int frame_no)
{
   const auto idx = (frame_no - start_frame);
   Expects(size_t(idx) < frame_dat.size());
   auto& frame = frame_dat[size_t(idx)];

   { // sanity
      Expects(frame.t == frame_no);
      for(const auto& ptr : frame.p2ds)
         Expects(!ptr->is_assigned()); // should never start on a path
   }

   const int interp_window = int(tracklet_params.frame_t_interp_window);
   const unsigned n_p2ds   = unsigned(frame.p2ds.size());

   Expects(interp_window > 0);

   vector<OpSkeleton2D*> p2ds;
   vector<InchoateTrack*> itracks;

   // ---- score info 2D array, with getters/setters
   struct ScoreInfo
   {
      float score = fNAN; // score
      unsigned o_p2d_idx; // index into `p2ds` above
   };
   vector<ScoreInfo> scores;
   auto get_score
       = [&](unsigned itrack_idx, unsigned p2d_idx) -> const ScoreInfo& {
      const auto idx = itrack_idx * p2ds.size() + p2d_idx;
      return scores.at(idx);
   };
   auto set_score = [&](unsigned itrack_idx,
                        unsigned p2d_idx,
                        unsigned o_p2d_idx,
                        float val) {
      const auto idx = itrack_idx * p2ds.size() + p2d_idx;
      auto& s        = scores.at(idx);
      s.score        = val;
      s.o_p2d_idx    = o_p2d_idx;
   };

   constexpr real null_score = 1e9; // something really bad

   auto calc_and_save_score = [&](unsigned itrack_idx, unsigned p2d_idx) {
      Expects(itrack_idx < itracks.size() && itracks[itrack_idx] != nullptr);
      Expects(p2d_idx < p2ds.size() && p2ds[p2d_idx] != nullptr);
      const auto& itrack = *itracks[itrack_idx];
      const auto& op_s2d = *p2ds[p2d_idx];

      // How far back are we willing to look
      Expects(op_s2d.t == frame_no);

      auto score_xyt = [&](const XYT& xyt) {
         float worst_score = fNAN; // Something really *really* bad
         int o_p2d_idx     = -1;
         for(const auto& ptr : xyt.skels) {
            const auto e_ptr = lookup_edge(*ptr, op_s2d);
            if(e_ptr == nullptr) continue;
            const auto& e    = *e_ptr;
            const auto score = e.score();
            Expects(std::isfinite(score));
            if(!std::isfinite(worst_score) || score > worst_score) {
               worst_score = score;
               o_p2d_idx   = int(ptr - &pose_dat[0]);
               Expects(size_t(o_p2d_idx) < pose_dat.size());
               Expects(&pose_dat[size_t(o_p2d_idx)] == ptr);
            }
         }
         return std::pair<real, int>{real(worst_score), o_p2d_idx};
      };

      real best_score = dNAN; // Something really *really* bad
      for(auto ii = crbegin(itrack.xyts); ii != crend(itrack.xyts); ++ii) {
         if(ii->t + interp_window < frame_no) break; // nothing to do
         const auto [score, o_p2d_idx] = score_xyt(*ii);
         if(std::isfinite(score)
            && (score < best_score || !std::isfinite(best_score))) {
            best_score = score;
            set_score(itrack_idx,
                      unsigned(p2d_idx),
                      unsigned(o_p2d_idx),
                      float(score));
         }
      }

      if(!std::isfinite(best_score))
         set_score(itrack_idx,
                   p2d_idx,
                   unsigned(-1),
                   std::numeric_limits<float>::max());
   };

   auto populate_p2ds_and_itracks = [&]() -> bool { // TRUE if something to do
      p2ds.clear();
      itracks.clear();

      for(auto& ptr : frame.p2ds) // All unassigned p2ds
         if(!ptr->is_assigned()) p2ds.push_back(ptr);

      for(auto& inchoate : inchoates)
         if(inchoate.xyts.size() > 0)
            if(inchoate.xyts.back().t + interp_window >= frame_no)
               itracks.push_back(&inchoate);

      scores.resize(itracks.size() * p2ds.size());
      for(auto i = 0u; i < itracks.size(); ++i)
         for(auto j = 0u; j < p2ds.size(); ++j) calc_and_save_score(i, j);

      return (itracks.size() > 0) && (p2ds.size() > 0);
   };

   auto cost_fn = [&](unsigned itrack_idx, unsigned p2d_idx) -> double {
      return double(get_score(itrack_idx, p2d_idx).score);
   };

   // As few mallocs as possible
   p2ds.reserve(frame.p2ds.size());
   itracks.reserve(inchoates.size());

   while(true) {
      bool found_one = false;

      // Populate `p2ds` and `itracks`
      if(populate_p2ds_and_itracks()) {
         // Run Hungarian
         const auto ii
             = hungarian_algorithm(itracks.size(), p2ds.size(), cost_fn);

         // Augment tracks
         for(const auto& [tidx, pidx] : ii) {
            const auto [score, o_p2d_idx] = get_score(tidx, pidx);
            if(score > params.hungarian_score_threshold) continue; // 0.25 ?
            Expects(size_t(o_p2d_idx) < pose_dat.size());
            push_node(*itracks[tidx], p2ds[pidx], &pose_dat[o_p2d_idx]);
            found_one = true;
         }
      }

      // If no track has been augmented, then we break
      if(!found_one) {
         // Try to seed one sequence...
         for(auto& ptr : frame.p2ds) {
            if(!ptr->is_assigned()) {
               seed_node_sequence_(ptr);
               found_one = true;
            }
         }
      }

      if(!found_one) break;
   }
}

// --------------------------------------------------- extend-xyt-nodes-to-frame
//
void This::extend_xyt_nodes_to_frame_(int frame_no)
{
   check_invariants_(); // paranoid

   const bool verbose = false;
   //= is_trace_mode && (frame_no == 40 || frame_no == 41 || frame_no == 42);

   // -------------------------------------------------- //
   // (A) Extend all existing tracks to frame `frame-no` //
   // (B) Form new tracks                                //
   // -------------------------------------------------- //

   // (A) Extend existing tracks...
   //     We match history XYTNodes to the current set of XYTNodes

   // All edges finish at `dst_frame`, or more properly, one of `dst_nodes`
   const auto& dst_frame = frame_dat.at(size_t(frame_no - start_frame));
   Expects(dst_frame.t == frame_no);
   const auto& dst_nodes = dst_frame.nodes;
   const int n_dst_nodes = int(dst_nodes.size());

   const int t_max     = int(tracklet_params.frame_t_interp_window);
   const int frame0    = std::max(start_frame, frame_no - t_max - 1);
   const bool is_first = (frame_no == start_frame);
   Expects(!is_first || (frame0 == start_frame));
   const int n_tracks = int(inchoates.size());

   // the edges of interest to `frame-no`
   auto make_edges = [&]() -> vector<XYTEdge*> {
      vector<XYTEdge*> es;
      es.reserve(size_t(n_dst_nodes) * 4);
      for(int t = frame0; t < frame_no; ++t) {
         auto& src_frame = frame_dat.at(size_t(t - start_frame));
         for(auto& node : src_frame.nodes)
            for(auto& e : node.edges())
               if(e.dst->t() == frame_no)
                  es.push_back(&e); // This is an edge of interest
      }
      return es;
   };

   // The raw edges
   vector<XYTEdge*> raw_edges = make_edges();

   // Figure out the unique source nodes
   vector<const XYTNode*> src_nodes;
   src_nodes.reserve(raw_edges.size());
   for(const auto e_ptr : raw_edges) src_nodes.push_back(e_ptr->src);
   remove_duplicates(src_nodes);
   const int n_src_nodes = int(src_nodes.size());

   { // Sanity
      for(const auto e_ptr : raw_edges)
         Expects(e_ptr->src->t() < e_ptr->dst->t());
   }

   // Create an index that links a source node to its position in `src-nodes`
   std::unordered_map<const XYTNode*, int> src_lookup;
   src_lookup.reserve(src_nodes.size());
   for(size_t i = 0; i < src_nodes.size(); ++i)
      src_lookup[src_nodes[i]] = int(i);

   // Create an index that links a dst node to its position in frame.nodes
   std::unordered_map<const XYTNode*, int> dst_lookup;
   dst_lookup.reserve(dst_nodes.size());
   for(size_t i = 0; i < dst_nodes.size(); ++i)
      dst_lookup[&dst_nodes[i]] = int(i);

   auto lookup = [&](const auto& map, const auto& o) -> auto
   {
      auto ii = map.find(o); // Paranoid getting src-idx
      Expects(ii != end(map));
      return ii->second;
   };

   // Create an index that links a [src-idx, dst-idx] to an XYTEdge*
   std::unordered_map<Point2, XYTEdge*> edge_lookup;
   edge_lookup.reserve(raw_edges.size());
   for(auto e_ptr : raw_edges) {
      const int src_idx = lookup(src_lookup, e_ptr->src);
      const int dst_idx = lookup(dst_lookup, e_ptr->dst);
      Expects(src_nodes.at(size_t(src_idx)) == e_ptr->src);
      Expects(&dst_nodes.at(size_t(dst_idx)) == e_ptr->dst);
      edge_lookup[Point2{src_idx, dst_idx}] = e_ptr;
   }

   auto cost_fn = [&](int src_idx, int dst_idx) -> double {
      auto ii = edge_lookup.find(Point2(src_idx, dst_idx));
      if(ii == end(edge_lookup)) return 1e9; // Something pretty bad
      return double(ii->second->weight);
   };

   // Find the matching nodes
   const auto pairs = hungarian_algorithm(
       unsigned(n_src_nodes), unsigned(n_dst_nodes), cost_fn);

   // Convert pairs to edges
   auto convert_pair_to_select = [&](const auto& pairs) {
      vector<XYTEdge*> selected_edges, out_edges;
      selected_edges.reserve(pairs.size());
      for(const auto& ij : pairs) {
         auto ii = edge_lookup.find(Point2(int(ij.first), int(ij.second)));
         if(ii == cend(edge_lookup)) continue;
         Expects(ii->second->dst != nullptr);
         Expects(ii->second->src != nullptr);
         selected_edges.push_back(ii->second);
      }

      // Sort edges by `inchoate-id`, and then by `weight`
      std::sort(begin(selected_edges),
                end(selected_edges),
                [&](auto A, auto B) -> bool {
                   if(A->src->inchoate_id() == B->src->inchoate_id())
                      return (A->weight) < (B->weight);
                   return (A->src->inchoate_id()) < (B->src->inchoate_id());
                });

      // We want at most ONE edge for each inchoate... so remove undesirables
      out_edges.reserve(selected_edges.size());
      for(auto ptr : selected_edges) {
         const auto ptr_it_id = ptr->src->inchoate_id();
         const bool do_push
             = (ptr_it_id == -1 || out_edges.empty()
                || out_edges.back()->src->inchoate_id() != ptr_it_id);
         if(is_trace_mode) {
            const bool slow_do_push
                = (ptr_it_id == -1
                   || std::find_if(begin(out_edges),
                                   end(out_edges),
                                   [&](auto p) {
                                      return p->src->inchoate_id() == ptr_it_id;
                                   })
                          == end(out_edges));
            Expects(do_push == slow_do_push);
         }
         if(do_push) out_edges.push_back(ptr);
      }

      return out_edges;
   };
   vector<XYTEdge*> selected_edges = convert_pair_to_select(pairs);

   const auto delim = ",\n              ";
   string inchoate_summary0;
   if(verbose)
      inchoate_summary0 = implode(
          cbegin(inchoates), cend(inchoates), delim, inchoate_summary);

   { // Add edges to existing inchoate tracks
      auto print_edge = [](const XYTEdge* e) -> string {
         return format("[{}/{}/{}, {}/{}/{}, w={}]",
                       e->src->t(),
                       e->src->frame_id(),
                       e->src->inchoate_id(),
                       e->dst->t(),
                       e->dst->frame_id(),
                       e->dst->inchoate_id(),
                       e->weight);
      };

      auto print_inchoate = [&](const InchoateTrack& it) -> string {
         return format(
             "id: {}, edges: {{{}}}",
             it.idx,
             implode(cbegin(it.edges), cend(it.edges), ", ", print_edge));
      };

      auto create_inchoate = [&](XYTEdge* e_ptr) {
         if(verbose)
            TRACE(format("CREATE INCHOATE, e-ptr = {}", print_edge(e_ptr)));
         inchoates.push_back({});
         auto& inchoate = inchoates.back();
         inchoate.idx   = int(inchoates.size()) - 1;
         inchoate.edges.push_back(e_ptr);
         e_ptr->set_inchoate_id(inchoate.idx);
         if(verbose)
            TRACE(format("DONE CREATE, it = {}", print_inchoate(inchoate)));
      };

      auto add_edge_to_inchoate = [&](XYTEdge* e_ptr) {
         if(verbose) TRACE(format("ADD EDGE: {}", print_edge(e_ptr)));
         Expects(e_ptr->src != nullptr);
         Expects(e_ptr->dst != nullptr);
         Expects(e_ptr->src->t() < e_ptr->dst->t());
         const int inchoate_id = e_ptr->src->inchoate_id();

         Expects(size_t(inchoate_id) < inchoates.size());
         auto& inchoate = inchoates[size_t(inchoate_id)];
         auto& edges    = inchoate.edges;
         Expects(edges.size() > 0);
         if(verbose) TRACE(format("ADD TO: {}", print_inchoate(inchoate)));

         // Search backwards to find the edge-node in `inchoate.edges` that
         // leads to the src node in `e_ptr`
         auto ee = std::find_if(
             rbegin(edges), rend(edges), [e_ptr](const auto o_ptr) -> bool {
                return o_ptr->dst == e_ptr->src;
             });

         const size_t ee_idx
             = size_t(std::distance(begin(edges), ee.base()) - 1);

         const bool has_ee_idx = (ee != rend(edges));
         if(has_ee_idx) {
            Expects(ee_idx < edges.size());
            Expects(edges[ee_idx]->dst == e_ptr->src);

            // Here we actually remove the edges
            for(size_t i = edges.size() - 1; i > ee_idx; --i) {
               auto& e = edges.back();
               e->dst->set_inchoate_id(-1); // unset
               e->marked = false;
               edges.pop_back();
            }
         } else {
            if(e_ptr->src != edges.front()->src) {
               auto print_edge = [&](const XYTEdge* e) {
                  return format("[w={}, {} -> {}]",
                                e->weight,
                                e->src->to_string(),
                                e->dst->to_string());
               };
               LOG_ERR(format(
                   R"V0G0N(
To-add:  {}
Edges:  [{}]         
)V0G0N",
                   print_edge(e_ptr),
                   implode(cbegin(edges),
                           cend(edges),
                           ",\n         ",
                           print_edge)));
            }
            Expects(e_ptr->src == edges.front()->src);
            // Expects(edges.size() == 1);
            for(auto e : edges) { e->set_inchoate_id(-1); }
            edges.clear();
         }

         Expects(!has_ee_idx || edges.size() > 0);
         Expects(edges.size() == 0 || edges.back()->dst == e_ptr->src);

         e_ptr->marked = true;
         e_ptr->dst->set_inchoate_id(inchoate_id);
         if(edges.size() == 0) {
            Expects(e_ptr->src->inchoate_id() == -1);
            e_ptr->src->set_inchoate_id(inchoate.idx);
         } else {
            Expects(e_ptr->src->inchoate_id() == inchoate_id);
         }
         Expects(e_ptr->src->inchoate_id() == inchoate.idx);
         edges.push_back(e_ptr);
      };

      auto process_edge = [&](XYTEdge* e_ptr) {
         const int inchoate_id = e_ptr->src->inchoate_id();
         if(inchoate_id >= 0)
            add_edge_to_inchoate(e_ptr);
         else
            create_inchoate(e_ptr);
      };

      for(auto& e_ptr : selected_edges) process_edge(e_ptr);
   }

   if(multiview_trace_mode()) {
      auto print_node_p = [&](const XYTNode* o) { return o->to_string(); };
      auto print_node   = [&](const XYTNode& o) { return o.to_string(); };
      auto print_edge   = [&](const auto ptr) -> string {
         return format("w={:+5.3f}, [{}, {}] -> [{}, {}]",
                       ptr->weight,
                       ptr->src->t(),
                       ptr->src->frame_id(),
                       ptr->dst->t(),
                       ptr->dst->frame_id());
      };
      auto print_pair = [&](const auto& ij) {
         auto ii = edge_lookup.find(Point2(int(ij.first), int(ij.second)));
         const XYTEdge* e = (ii == cend(edge_lookup)) ? nullptr : ii->second;
         return format(
             "[{{{}, {}}}, w={:+5.3f}, it-id={}, [{}, {}] -> [{}, {}]]",
             ij.first,
             ij.second,
             cost_fn(int(ij.first), int(ij.second)),
             (!e ? "?"s : format("{:03d}", e->src->inchoate_id())),
             (!e ? "?"s : str(e->src->t())),
             (!e ? "?"s : str(e->src->frame_id())),
             (!e ? "?"s : str(e->dst->t())),
             (!e ? "?"s : str(e->dst->frame_id())));
      };

      const auto msg = format(
          R"V0G0N(
--------------------------------------------------------- extend xyt to frame {}
n-dst-nodes:  {}
t-max:        {}
frame0:       {}
is-first:     {}
n-tracks:     {}
inchoates:   [{}]
n-raw-edges: [{}]              
src-nodes:   [{}]
dst-nodes:   [{}]
pairs:       [{}]
selected-es: [{}]
inchoates:   [{}]


)V0G0N",
          frame_no,
          n_dst_nodes,
          t_max,
          frame0,
          str(is_first),
          n_tracks,
          inchoate_summary0,
          implode(cbegin(raw_edges), cend(raw_edges), delim, print_edge),
          implode(cbegin(src_nodes), cend(src_nodes), delim, print_node_p),
          implode(cbegin(dst_nodes), cend(dst_nodes), delim, print_node),
          implode(cbegin(pairs), cend(pairs), delim, print_pair),
          implode(
              cbegin(selected_edges), cend(selected_edges), delim, print_edge),
          implode(cbegin(inchoates), cend(inchoates), delim, inchoate_summary));
      // TRACE(msg);
      const auto fname = format("{}/extend-xyt-to-frame.text", outdir);
      file_append_contents(fname, msg);
      TRACE(format("appended extend-xyt-to-frame {} to {}", frame_no, outdir));
   }

   check_invariants_(); // ensure we didn't screw anything up
}

// ---------------------------------------------------------------- extend-track
//
void This::extend_track_(InchoateTrack& inchoate) noexcept
{
   constexpr unsigned sink_id = std::numeric_limits<unsigned>::max();
   constexpr real sink_weight = 1e3;

   if(inchoate.xyts.size() == 0) return;

   XYT& xyt = inchoate.xyts.back();
   Expects(xyt.skels.size() > 0);
   const unsigned source_id = op_skel_2d_index_(xyt.skels.back());

   auto edge_weight = [&](const unsigned& u, const unsigned& v) -> real {
      if(v == sink_id || u == sink_id) return sink_weight;
      const auto e_ptr = lookup_edge(u, v);
      Expects(e_ptr != nullptr);
      return real(e_ptr->weight);
   };

   auto for_each_neighbour
       = [&](const unsigned u, std::function<void(const unsigned&)> f) {
            f(sink_id); // every edge joins the sink
            Expects(u < pose_dat.size());
            for(const auto& e : pose_dat[u].edges) {
               Expects(e.this_idx == u);
               Expects(e.other_idx < pose_dat.size());
               if(!pose_dat[e.other_idx].is_assigned()) f(e.other_idx);
            }
         };

   // Calculate the path
   auto path = shortest_path_sparse<unsigned>(
       source_id, sink_id, edge_weight, for_each_neighbour);

   Expects(path.size() > 0 && path.back() == sink_id);
   path.pop_back();
   Expects(path.size() > 0 && path.front() == source_id);

   // Add to the output path
   for(auto ii = std::next(cbegin(path)); ii != cend(path); ++ii) {
      OpSkeleton2D* o = &pose_dat[*ii];
      Expects(!o->is_assigned());
      push_node(inchoate, o);
   }
}

bool This::seed_one_track_() noexcept
{
   for(auto& frame : frame_dat) {
      for(auto& ptr : frame.p2ds) {
         if(!ptr->is_assigned()) {
            extend_track_(seed_node_sequence_(ptr));
            return true;
         }
      }
   }

   return false;
}

// ----------------------------------------------------------------- calc-speeds
// Just one speed for an entire inchoate track
static real calc_av_speed(const vector<Vector2>& Xs,
                          const real frame_duration) noexcept
{
   if(Xs.size() < 2) return 0.0;
   real total = 0.0;
   for(size_t i = 1; i < Xs.size(); ++i)
      total += (Xs.at(i - 1) - Xs.at(i - 0)).norm();
   return total / (frame_duration * real(Xs.size() - 1));
}

real This::calc_speed(const InchoateTrack& it) const noexcept
{
   if(it.edges.size() == 0) return 0.0;
   vector<Vector2> Xs;
   Xs.reserve(it.edges.size() + 1);
   Xs.push_back(hist_to_X(it.edges.front()->src->xy()));
   for(const auto e_ptr : it.edges) Xs.push_back(hist_to_X(e_ptr->dst->xy()));
   return calc_av_speed(Xs, frame_duration);
}

real This::calc_speed(const tracks::NodeSequence& seq) const noexcept
{
   vector<Vector2> Xs;
   Xs.reserve(seq.nodes().size());
   std::transform(cbegin(seq.nodes()),
                  cend(seq.nodes()),
                  std::back_inserter(Xs),
                  [&](const auto& o) -> Vector2 { return hist_to_X(o.xy()); });
   return calc_av_speed(Xs, frame_duration);
}

// ---------------------------------------------------------- fill-training-data
//
void This::fill_training_data_()
{
   // Only concern ourselves with tracklets after this frame
   const int frame0 = tracklet->start_frame;
   Expects(end_frame >= frame0);

   // The speeds
   vector<real> seq_speeds;
   {
      seq_speeds.reserve(complete_seqs.size());
      std::transform(cbegin(complete_seqs),
                     cend(complete_seqs),
                     std::back_inserter(seq_speeds),
                     [this](const auto& seq) { return calc_speed(seq); });
   }

   // Build an index of current node sequences...
   vector<vector<int>> seq_indicies;       // -> complete_seq.at(...)
   vector<Spatial2Index> spatial_indicies; // -> seq_indices.at(...)
   {
      struct NodeData
      {
         size_t complete_seq_ind = 0;
         Vector2f xy             = {0.0f, 0.0f};
         int t                   = 0;
      };

      vector<NodeData> node_data;
      for(size_t ind = 0; ind < complete_seqs.size(); ++ind)
         for(const auto& node : complete_seqs.at(ind).nodes())
            if(node.t() >= frame0)
               node_data.push_back({ind, node.xy(), node.t()});

      vector<vector<Vector2>> positions;
      seq_indicies.resize(size_t(end_frame - frame0 + 1));
      positions.resize(size_t(end_frame - frame0 + 1));
      for(const auto& nd : node_data) {
         Expects(nd.t - frame0 >= 0);
         seq_indicies.at(size_t(nd.t - frame0))
             .push_back(int(nd.complete_seq_ind));
         positions.at(size_t(nd.t - frame0)).push_back(hist_to_X(nd.xy));
      }

      spatial_indicies.resize(size_t(end_frame - frame0 + 1));
      for(size_t ind = 0; ind < spatial_indicies.size(); ++ind)
         spatial_indicies.at(ind).init(positions.at(ind));
   }

   auto get_track_speed = [&](const Vector2& X, const int t) -> real {
      const auto& spatial_index = spatial_indicies.at(size_t(t - frame0));
      const auto& seq_index     = seq_indicies.at(size_t(t - frame0));
      const auto [Y, q_ind]     = spatial_index.query_nearest(X);
      const auto dist           = (X - Y).norm();
      const bool is_match       = Y.is_finite() && (q_ind < seq_index.size())
                            && (dist < 1.5 * human::k_adult_male_radius);
      if(!is_match) return real(NAN);
      const auto& seq = complete_seqs.at(size_t(seq_index.at(q_ind)));
      return seq_speeds.at(size_t(seq_index.at(q_ind)));
   };

   vector<vector<unique_ptr<ARGBImage>>> argb_memo_;
   auto get_argb
       = [&](const int frame_no, const int sensor_no) -> const ARGBImage& {
      if(argb_memo_.size() != frame_dat.size()) { // Lazy init
         argb_memo_.resize(frame_dat.size());
         for(size_t i = 0; i < frame_dat.size(); ++i)
            argb_memo_[i].resize(frame_dat[i].sensor_labs.size());
      }
      const size_t ind = size_t(frame_no - start_frame);
      Expects(ind < argb_memo_.size());
      Expects(size_t(sensor_no) < argb_memo_.at(ind).size());
      auto& memo = argb_memo_[ind][size_t(sensor_no)];

      if(memo == nullptr) {
         const auto lab_img_ptr = get_lab_ptr(frame_no, sensor_no);
         Expects(lab_img_ptr != nullptr);
         auto argb = LAB_im_to_argb(*lab_img_ptr); // make this better
         memo      = make_unique<ARGBImage>(std::move(argb));
      }

      return *memo;
   };

   auto make_training_data = [&](const auto& p2d, const auto& fdat) {
      Expects(!params.export_training_data_prefix.empty());
      const auto& spatial_index = fdat.gt_spatial_index;
      const auto& pinfo         = *p2d.p2d_info_ptr;
      const auto X3d            = pinfo.p2d_ptr->best_3d_result().Xs_centre();
      const auto X              = to_vec2(Vector2f(X3d.x, X3d.y));
      const auto speed          = get_track_speed(X, fdat.t);
      const auto [Y, gt_ind]    = spatial_index.query_nearest(X);
      const auto dist           = (X - Y).norm();
      const bool is_match = Y.is_finite() && (gt_ind < spatial_index.size())
                            && (dist < 1.5 * human::k_adult_male_radius);

      const auto pose
          = (is_match) ? fdat.gt_tps.at(gt_ind).pose : PoseAnnotation::NONE;

      const auto tdp = calibration::make_training_datapoint(
          scene_desc, pinfo, pose, speed, !is_match);

      {
         const int frame_no  = p2d.t;           //
         const int sensor_no = p2d.sensor_no(); //
         const int loc_id    = p2d.p2d_id();    //
         const auto features = tdp.flatten();   // vector<real>...
         const auto labels   = tdp.labels();    // vector<real>...

         Expects(frame_no == fdat.t);
         const auto file_dir = format("{}/training-data", outdir);
         mkdir_p(file_dir);

         const auto file_prefix = format("{}/{}.{:02d}.{:04d}.{:02d}",
                                         file_dir,
                                         params.export_training_data_prefix,
                                         sensor_no,
                                         frame_no,
                                         loc_id);

         // Output the text data
         const auto text_fname = format("{}.text", file_prefix);
         const auto text_dat
             = format("{}.{:02d}.{:04d}.{:02d}, {}, {}",
                      params.export_training_data_prefix,
                      sensor_no,
                      frame_no,
                      loc_id,
                      implode(cbegin(labels), cend(labels), ", "),
                      implode(cbegin(features), cend(features), ", "));
         file_put_contents(text_fname, text_dat);

         // Output the image data
         ARGBImage argb = get_argb(frame_no, sensor_no);
         Expects(p2d.p2d_ptr() != nullptr);
         render_pose(argb, *p2d.p2d_ptr());
         render_string(argb,
                       str_replace(",", "\n", text_dat),
                       Point2(10, 10),
                       k_yellow,
                       k_black);
         argb.save(format("{}.png", file_prefix));
      }

      out.training_data.push_back(tdp);
   };

   for(const auto& fdat : frame_dat)
      if(fdat.t >= frame0)
         for(const auto& p2d_ptr : fdat.p2ds)
            if(!p2d_ptr->is_interpolation()) make_training_data(*p2d_ptr, fdat);
}

// ------------------------------------------------------------- finalize-result
//
void This::finalize_result_()
{
   { // ---- (*) Paranoid
      check_invariants_();
   }

   auto push_xyt = [this](vector<Node>& nodes, const XYT& xyt) {
      auto make_node = [&](const XYT& xyt) -> Node {
         Node node;
         vector<P2dAddress> addresses;
         Expects(xyt.skels.size() < 1000000);
         addresses.reserve(xyt.skels.size());
         for(auto& ptr : xyt.skels) addresses.push_back(ptr->p2d_address());
         Expects(addresses.size() < 1000000);
         node.set_p2ds(std::move(addresses));
         node.set_xy(X_to_hist(Vector2f(xyt.x, xyt.y)));
         node.set_t(xyt.t);
         node.set_gaze(xyt.gaze_theta);
         node.set_prob_fp(calc_prob_fp(xyt));
         node.set_still_score(xyt.still_score);
         node.set_pose_scores(xyt.pose_classification);
         Expects(std::isfinite(node.prob_fp()));
         return node;
      };

      Node node = make_node(xyt);
      const bool is_interp
          = !nodes.empty() && (nodes.back().t() + 1 < node.t());

      if(false) {
         INFO(format(
             "PUSH |nodes| = {}, from [{}, {}]. xyt.t = {}/{}, interp = {}",
             nodes.size(),
             (nodes.empty() ? -1 : nodes.front().t()),
             (nodes.empty() ? -1 : nodes.back().t()),
             xyt.t,
             node.t(),
             str(is_interp)));
      }

      if(is_interp) {             // Create in-between nodes
         Expects(!nodes.empty()); // how could we interpolate?
         const int t0      = nodes.back().t();
         const int t1      = node.t();
         const Vector2f X0 = to_vec2f(nodes.back().xy());
         const Vector2f X1 = to_vec2f(node.xy());
         if(false) INFO(format(" ... [t0..t1] = [{}..{}]", t0, t1));
         for(int t = t0 + 1; t < t1; ++t) {
            const float perc = float(t - t0) / float(t1 - t0 - 1);
            const Vector2f X = X0 + perc * (X1 - X0);
            Node n;
            n.set_t(t);
            n.set_xy(X);
            n.set_interp(true);
            nodes.emplace_back(std::move(n));
            if(false) INFO(format(" ... pushed node.t = {}", t));
         }
      }

      Expects(nodes.empty() || nodes.back().t() + 1 == node.t());
      nodes.emplace_back(std::move(node));
   };

   auto unpack_nodes = [&](const InchoateTrack& it, int& next_seq_id) {
      const bool verbose = false;
      if(verbose) TRACE(format("unpack nodes, |edges| = {}", it.edges.size()));
      const vector<XYTEdge*>& edges = it.edges;
      Expects(edges.size() > 0);
      const auto t0 = edges.front()->src->t();
      const auto t1 = edges.back()->dst->t();

      vector<Node> nodes;
      nodes.reserve(size_t(t1 - t0 + 1));

      if(verbose && is_trace_mode)
         TRACE(format("building nodes for track: {}", it.idx));
      push_xyt(nodes, edges.front()->src->xyt());

      if(verbose && is_trace_mode)
         cout << "   " << edges.front()->src->to_string() << endl;
      for(auto e : edges) {
         push_xyt(nodes, e->dst->xyt());

         if(verbose && is_trace_mode)
            cout << "   " << e->dst->to_string() << endl;
      }

      const int id = (it.node_seq_id >= 0) ? it.node_seq_id : next_seq_id++;
      return tracks::NodeSequence{id, std::move(nodes)};
   };

   { // ---- (*) Write inchoate tracks
      // Unpack: [t0, t1)
      auto unpack = [&](const InchoateTrack& it, int& next_seq_id) {
         Expects(!it.is_available());
         auto seq = unpack_nodes(it, next_seq_id);

         for(auto i = 1u; i < seq.nodes().size(); ++i)
            Expects(seq.nodes()[i - 0].t() == seq.nodes()[i - 1].t() + 1);

         { // sanity
            for(const auto& node : seq.nodes())
               Expects(node.is_interp() || node.p2ds().size() > 0);
         }

         seq.check_invariants();

         return seq;
      };

      auto clip = [&](const NodeSequence& seq, const int t0, const int t1) {
         seq.check_invariants();
         vector<Node> nodes;
         nodes.reserve(size_t(t1 - t0));
         for(const auto& node : seq.nodes())
            if(node.t() >= t0 && node.t() < t1) nodes.push_back(node);
         auto ret = tracks::NodeSequence{seq.id(), std::move(nodes)};
         ret.check_invariants();
         return ret;
      };

      auto get_next_new_node_seq_id = [&]() -> int {
         int max_id = (prev_track == nullptr) ? -1 : prev_track->max_id;
         for(const InchoateTrack& it : inchoates)
            if(max_id < it.node_seq_id) max_id = it.node_seq_id;
         return max_id + 1;
      };
      int next_seq_id = get_next_new_node_seq_id();

      vector<tracks::NodeSequence> prev_seqs;
      if(prev_track) std::swap(prev_seqs, prev_track->seqs);
      Expects(out.seqs.size() == 0);

      for(const InchoateTrack& it : inchoates) {
         if(it.is_available()) continue;

         NodeSequence seq = unpack(it, next_seq_id);

         {
            auto ii = std::find_if(
                cbegin(prev_seqs),
                cend(prev_seqs),
                [id = seq.id()](const auto& o) { return id == o.id(); });
            if(ii != cend(prev_seqs)) seq.prepend_missing(*ii);
         }

         if(seq.size() == 0) continue;

         const auto t0 = seq.nodes().front().t();
         const auto t1 = seq.nodes().back().t();

         if(prev_track && t0 < tracklet->start_frame) {
            auto end = std::min(t1 + 1, tracklet->start_frame);
            prev_track->seqs.push_back(clip(seq, t0, end));
         }
         if(t1 >= tracklet->start_frame) {
            auto start = std::max(tracklet->start_frame, t0);
            out.seqs.push_back(clip(seq, start, t1 + 1));
         }

         complete_seqs.push_back(seq);
      }
   }

   { // Remove empty tracks
      auto do_remove_empties = [&](auto& seqs) {
         auto ii = std::stable_partition(
             begin(seqs), end(seqs), [&](auto& seq) { return seq.size() > 0; });
         seqs.erase(ii, end(seqs));
      };
      if(prev_track) do_remove_empties(prev_track->seqs);
      do_remove_empties(out.seqs);
   }

   auto get_max_id = [](const Tracks* sentinal, const auto& tts) {
      int ret = (sentinal == nullptr) ? -1 : sentinal->max_id;
      for(const auto& tt : tts)
         if(ret < tt.id()) ret = tt.id();
      return ret;
   };

   if(prev_track) prev_track->max_id = get_max_id(prev_track, prev_track->seqs);

   out.max_id               = get_max_id(prev_track, out.seqs);
   out.w                    = w;
   out.h                    = h;
   out.aabb                 = aabb;
   out.hist_sz              = hist_sz;
   out.start_frame          = tracklet->start_frame;
   out.n_frames             = tracklet->n_frames;
   out.max_frames_per_track = max_frames_per_tracklet;

   if(false && is_trace_mode) {
      auto print_track = [&](const string_view s, const Tracks& tts) -> string {
         return format(
             R"V0G0N(
{} Track
  max-id  :  {}
  dims    :  {}x{}
  len     : [{}..{})
  fpt     :  {}
  hist    :  bounds={}, sz={}
  seq-sum : [{}]
)V0G0N",
             s,
             tts.max_id,
             tts.w,
             tts.h,
             tts.start_frame,
             tts.start_frame + tts.n_frames,
             tts.max_frames_per_track,
             str(tts.aabb),
             tts.hist_sz,
             implode(cbegin(tts.seqs),
                     cend(tts.seqs),
                     ",\n             ",
                     [&](const auto& seq) {
                        return format("[#{}, [{}]]",
                                      seq.id(),
                                      implode(cbegin(seq.nodes()),
                                              cend(seq.nodes()),
                                              ", ",
                                              [&](const auto& o) {
                                                 return format(
                                                     "{}{}{}{}",
                                                     (std::isfinite(o.gaze())
                                                          ? ""
                                                          : ANSI_DIM),
                                                     (o.is_interp() ? "i" : ""),
                                                     o.t(),
                                                     ANSI_COLOUR_RESET);
                                              }));
                     }));
      };
      if(prev_track) TRACE(format("{}", print_track("Prev", *prev_track)));
      TRACE(format("{}", print_track("This", out)));
   }

   // TRACE("validating previous");
   if(prev_track) track_ops::validate_tracks_object(*prev_track);

   // TRACE("validating current");
   track_ops::validate_tracks_object(out);

   if(prev_track != nullptr) {
      const auto p0 = prev_track->start_frame;
      const auto p1 = p0 + prev_track->n_frames;
      Expects(p1 == out.start_frame);
   }
}

// ----------------------------------------- extend-xyt-nodes-to-frame-all-paths
//
void This::extend_xyt_nodes_to_frame_all_paths_(int frame_no)
{
   check_invariants_(); // paranoid

   // -- (*) -- Setup
   const bool verbose   = false;
   const bool write_log = verbose || is_trace_mode;
   const int t_max      = int(tracklet_params.frame_t_interp_window);
   const int frame0     = std::max(start_frame, frame_no - t_max - 1);
   const bool is_first  = (frame_no == start_frame);
   Expects(!is_first || (frame0 == start_frame));
   const int n_tracks = int(inchoates.size());
   if(is_first) return; // nothing to do on the first frame
   vector<XYTNode*> src_nodes;

   // -- (*) -- Possible destination nodes for `frame-no`
   auto& dst_frame = frame_dat.at(size_t(frame_no - start_frame));
   Expects(dst_frame.t == frame_no);
   vector<XYTNode*> dst_nodes(dst_frame.nodes.size());
   for(auto i = 0u; i < dst_frame.nodes.size(); ++i)
      dst_nodes[i] = &dst_frame.nodes[i];

   // -- (*) -- Setup feedback
   std::stringstream feedback_ss{""};
   string inchoate_summary0  = ""s;
   string dst_nodes_summary0 = ""s;
   const auto delim          = ",\n              ";
   if(write_log) {
      inchoate_summary0 = implode(
          cbegin(inchoates), cend(inchoates), delim, inchoate_summary);
      dst_nodes_summary0 = implode(
          cbegin(dst_nodes), cend(dst_nodes), delim, [&](const XYTNode* o) {
             return o->to_string();
          });
   }

   // -- (*) -- Extend all existing inchoates to `frame-no`
   auto extend_inchoate = [&](InchoateTrack& it) {
      if(dst_nodes.size() == 0) return;

      if(write_log) {
         feedback_ss << format("Extend inchoate.id={}\n", it.idx);
      }

      // Find oldest edge in `it`, with src-node.t >= frame0
      const auto ii = std::lower_bound(
          begin(it.edges),
          end(it.edges),
          frame0,
          [&](const auto& elem, int val) { return elem->src->t() < val; });
      const bool has_ii = (ii != end(it.edges));
      if(!has_ii) {
         feedback_ss << format(
             " * failed to find relevant edge in: {}\n",
             implode(begin(it.edges), end(it.edges), ", ", print_edge));
         return; // bail
      } else if(write_log) {
         feedback_ss << format(
             " * from edge={} to dst-nodes\n   nodes = [{}]\n",
             print_edge(*ii),
             implode(begin(dst_nodes),
                     end(dst_nodes),
                     ",\n            ",
                     print_node));
      }

      // Now find the best continuation...
      XYTNode* src = (*ii)->src;
      XYTNode* dst = nullptr;
      Expects(src->t() >= frame0);

      // Get the shortest paths from `src`
      auto edge_is_covered = [&](const XYTEdge* e_ptr) -> bool {
         auto src_it_id = e_ptr->src->inchoate_id();
         auto dst_it_id = e_ptr->dst->inchoate_id();
         return (e_ptr->dst->t() > frame_no)
                || (src_it_id >= 0 && src_it_id != it.idx)
                || (dst_it_id >= 0 && dst_it_id != it.idx);
      };
      const auto path_info
          = SingleSourcePathInfo::make(src, sorted_xyt_nodes, edge_is_covered);

      { // Get the best destination node
         float best_score = std::numeric_limits<float>::max();
         for(auto node : dst_nodes) {
            const auto dist = path_info.path_cost(node);
            if(!std::isfinite(dist)) continue;
            if(dist < best_score) {
               best_score = dist;
               dst        = node;
            }
         }
      }

      if(dst == nullptr) {
         if(write_log)
            feedback_ss << format(
                " * failed to find destination, "
                "edge={}, weights={}\n",
                print_edge(*ii),
                implode(
                    begin(dst_nodes), end(dst_nodes), ", ", [&](const auto o) {
                       return path_info.path_cost(o);
                    }));
         return; // nothing to be done
      } else if(write_log) {
         feedback_ss << format(" * extend edge from\n   {}\n   ->\n   {}\n",
                               print_node(src),
                               print_node(dst));
      }

      // Extract the path
      auto path = path_info.path(dst);
      Expects(path.size() > 0);
      Expects(path.front()->src == src);
      Expects(path.back()->dst == dst);
      if(write_log) {
         feedback_ss << format(
             " * add: [{}]\n",
             implode(cbegin(path), cend(path), ", ", print_edge));
      }

      { // Check we didn't overwrite another path
         bool has_error = false;
         for(auto e_ptr : path)
            if(edge_is_covered(e_ptr)) { has_error = true; }
         if(has_error) {
            FATAL(format("discovered a covered-edge in path [{}]",
                         implode(cbegin(path), cend(path), ", ", print_edge)));
         }
      }

      // Remove edges
      auto remove_edges = [&](auto& inchoate, auto dd) {
         if(write_log && dd != end(inchoate.edges)) {
            feedback_ss << format(
                " * inchoate.id={} remove edges [{}]\n",
                inchoate.idx,
                implode(dd, end(inchoate.edges), ", ", print_edge));
         }

         for(auto kk = dd; kk != end(inchoate.edges); ++kk) {
            XYTEdge* e_ptr = *kk;
            Expects(e_ptr != nullptr);
            Expects(e_ptr->dst != nullptr);
            Expects(e_ptr->src != nullptr);
            e_ptr->set_inchoate_id(-1);
         }
         inchoate.edges.erase(dd, end(inchoate.edges));
      };
      remove_edges(it, ii);
      Expects(it.edges.size() == 0 || it.edges.back()->dst == src);
      Expects(src->inchoate_id() == -1);

      // It is possible (unlikely) that we're adding a path that writes over
      // another inchoate... here we account for this, by just removing
      // nodes and edges from that path
      // auto tidy_path = [&](XYTNode* o) {
      //    Expects(size_t(o->inchoate_id()) < inchoates.size());
      //    Expects(o->inchoate_id() != it.idx);
      //    auto& o_it = inchoates.at(o->inchoate_id());
      //    auto dd
      //        = std::find_if(begin(o_it.edges), end(o_it.edges), [&](auto
      //        e) {
      //             return e->src == o || e->dst == o;
      //          });
      //    Expects(dd != end(o_it.edges));
      //    remove_edges(o_it, dd);
      //    if(o_it.edges.size() > 0)
      //       o_it.edges.back()->dst->set_inchoate_id(o_it.idx);
      //    check_inchoate_(o_it);
      // };
      for(auto e_ptr : path) { Expects(e_ptr->dst->inchoate_id() == -1); }

      // Now write this continuation
      src->set_inchoate_id(it.idx);
      for(auto e_ptr : path) {
         Expects(e_ptr->src->inchoate_id() == it.idx);
         Expects(e_ptr->dst->inchoate_id() == -1);
         e_ptr->set_inchoate_id(it.idx);
         it.edges.push_back(e_ptr);
      }
      check_inchoate_(it);

      if(write_log) feedback_ss << "\n";
   };
   for(auto& it : inchoates) {
      // Remove dst-nodes which are spoken for
      dst_nodes.erase(std::partition(begin(dst_nodes),
                                     end(dst_nodes),
                                     [](const auto& ptr) {
                                        return ptr->inchoate_id() == -1;
                                     }),
                      end(dst_nodes));

      // Extend `it`
      extend_inchoate(it);
   }

   // Now we just create the best pairs of edges that we can...
   vector<XYTEdge*> raw_edges;
   for(auto& node : frame_dat.at(size_t(frame_no - 1 - start_frame)).nodes) {
      if(node.inchoate_id() != -1) continue; // already spoken for
      for(auto& e : node.edges()) {
         if(e.dst->t() != frame_no) continue;     // going to the wrong frame
         if(e.dst->inchoate_id() != -1) continue; // already spoken for
         raw_edges.push_back(&e);
      }
   }

   // Source nodes, and their `src_lookup`
   src_nodes.reserve(raw_edges.size());
   for(const auto e_ptr : raw_edges) src_nodes.push_back(e_ptr->src);
   remove_duplicates(src_nodes);
   const int n_src_nodes = int(src_nodes.size());
   std::unordered_map<const XYTNode*, int> src_lookup;
   src_lookup.reserve(src_nodes.size());
   for(size_t i = 0; i < src_nodes.size(); ++i)
      src_lookup[src_nodes[i]] = int(i);

   // Dst nodes, and their `src_lookup`
   dst_nodes.clear();
   for(const auto e_ptr : raw_edges) dst_nodes.push_back(e_ptr->dst);
   remove_duplicates(dst_nodes);
   const int n_dst_nodes = int(dst_nodes.size());
   std::unordered_map<const XYTNode*, int> dst_lookup;
   dst_lookup.reserve(dst_nodes.size());
   for(size_t i = 0; i < dst_nodes.size(); ++i)
      dst_lookup[dst_nodes[i]] = int(i);

   // Paranoid utility for lookup up maps
   auto lookup = [&](const auto& map, const auto& o) -> auto
   {
      auto ii = map.find(o); // Paranoid getting src-idx
      Expects(ii != end(map));
      return ii->second;
   };

   // We need an edge lookup...
   std::unordered_map<Point2, XYTEdge*> edge_lookup;
   edge_lookup.reserve(raw_edges.size());
   for(auto e_ptr : raw_edges) {
      const int src_idx = lookup(src_lookup, e_ptr->src);
      const int dst_idx = lookup(dst_lookup, e_ptr->dst);
      Expects(src_nodes.at(size_t(src_idx)) == e_ptr->src);
      Expects(dst_nodes.at(size_t(dst_idx)) == e_ptr->dst);
      edge_lookup[Point2{src_idx, dst_idx}] = e_ptr;
   }

   auto cost_fn = [&](int src_idx, int dst_idx) -> double {
      auto ii = edge_lookup.find(Point2(src_idx, dst_idx));
      if(ii == end(edge_lookup)) return 1e9; // Something pretty bad
      return real(ii->second->weight);
   };

   const auto pairs = hungarian_algorithm(n_src_nodes, n_dst_nodes, cost_fn);

   // Convert pairs to edges
   auto convert_pairs_to_edges = [&](const auto& pairs) {
      vector<XYTEdge*> selected_edges;
      selected_edges.reserve(pairs.size());
      for(const auto& ij : pairs) {
         auto ii = edge_lookup.find(Point2(int(ij.first), int(ij.second)));
         if(ii == cend(edge_lookup)) continue;
         Expects(ii->second->dst != nullptr);
         Expects(ii->second->src != nullptr);
         Expects(ii->second->src->inchoate_id() == -1);
         Expects(ii->second->dst->inchoate_id() == -1);
         selected_edges.push_back(ii->second);
      }
      return selected_edges;
   };

   auto print_inchoate = [&](const InchoateTrack& it) -> string {
      return format(
          "id: {}, edges: {{{}}}",
          it.idx,
          implode(cbegin(it.edges), cend(it.edges), ", ", print_edge));
   };

   auto create_inchoate = [&](XYTEdge* e_ptr) {
      if(verbose)
         TRACE(format("CREATE INCHOATE, e-ptr = {}", print_edge(e_ptr)));
      inchoates.push_back({});
      auto& inchoate = inchoates.back();
      inchoate.idx   = int(inchoates.size()) - 1;
      inchoate.edges.push_back(e_ptr);
      Expects(e_ptr->src->inchoate_id() == -1);
      Expects(e_ptr->dst->inchoate_id() == -1);
      e_ptr->set_inchoate_id(inchoate.idx);
      Expects(inchoate.edges.size() > 0);
      check_inchoate_(inchoate);
      if(verbose)
         TRACE(format("DONE CREATE, it = {}", print_inchoate(inchoate)));
   };

   const auto selected_edges = convert_pairs_to_edges(pairs);
   for(auto e_ptr : selected_edges) create_inchoate(e_ptr);

   check_invariants_(); // paranoid

   if(write_log) {
      auto print_pair = [&](const auto& ij) {
         auto ii = edge_lookup.find(Point2(int(ij.first), int(ij.second)));
         const XYTEdge* e = (ii == cend(edge_lookup)) ? nullptr : ii->second;
         return format(
             "[{{{}, {}}}, w={:+5.3f}, it-id={}, [{}, {}] -> [{}, {}]]",
             ij.first,
             ij.second,
             cost_fn(int(ij.first), int(ij.second)),
             (!e ? "?"s : format("{:03d}", e->src->inchoate_id())),
             (!e ? "?"s : str(e->src->t())),
             (!e ? "?"s : str(e->src->frame_id())),
             (!e ? "?"s : str(e->dst->t())),
             (!e ? "?"s : str(e->dst->frame_id())));
      };

      const bool is_first = (frame_no == start_frame);

      const string msg = format(
          R"V0G0N(
---------------------------------------------- extend xyt to frame, all-paths {}
n-dst-nodes:  {}
t-max:        {}
frame0:       {}
is-first:     {}
n-tracks:     {}
inchoates:   [{}]
n-raw-edges: [{}]              
src-nodes:   [{}]
dst-nodes:   [{}]
pairs:       [{}]
selected-es: [{}]
inchoates:   [{}]

{}

)V0G0N",
          frame_no,
          n_dst_nodes,
          t_max,
          frame0,
          str(is_first),
          n_tracks,
          inchoate_summary0,
          implode(cbegin(raw_edges), cend(raw_edges), delim, print_edge),
          implode(cbegin(src_nodes), cend(src_nodes), delim, print_node),
          dst_nodes_summary0,
          implode(cbegin(pairs), cend(pairs), delim, print_pair),
          implode(
              cbegin(selected_edges), cend(selected_edges), delim, print_edge),
          implode(cbegin(inchoates), cend(inchoates), delim, inchoate_summary),
          feedback_ss.str());

      if(is_trace_mode) {
         TRACE(format("{}", msg));
      } else if(verbose) {
         INFO(format("{}", msg));
      }

      if(is_trace_mode) {
         const auto fname = format("{}/extend-xyt-to-frame.text", outdir);
         file_append_contents(fname, msg);
         TRACE(
             format("appended extend-xyt-to-frame {} to {}", frame_no, outdir));
      }
   }
}

// --------------------------------------------------------------------- execute
//
Tracks This::execute()
{
   const auto now = tick();

   // ---- (*) Create the tracks
   {
      const int start_dt = (prev_tracklet == nullptr)
                               ? 0
                               : int(params.fixed_window_prev_tracklet);
      for(auto t = start_frame + start_dt; t <= end_frame; ++t)
         extend_xyt_nodes_to_frame_all_paths_(t);
   }

   // ---- (*) Pose classification
   {
      const auto pose_now = tick();
      if(pose_classifier != nullptr) {
         auto push_job = [&](XYTNode* node, const real speed) {
            pjobs.schedule(
                [this, node, speed]() { classify_pose(node, speed); });
         };
         for(auto& it : inchoates) {
            const real speed = calc_speed(it);
            if(it.edges.size() > 0) push_job(it.edges.front()->src, speed);
            for(auto e_ptr : it.edges) push_job(e_ptr->dst, speed);
         }
         pjobs.execute();
      }
      timing_pose_classification = float(ms_tock_f(pose_now));
   }

   // ---- (*) Finalize the result
   finalize_result_();

   if(!params.export_training_data_prefix.empty()) fill_training_data_();

   tot_track_time = std::max<float>(
       0.0f, float(ms_tock_f(now)) - timing_pose_classification);

   if(is_trace_mode) { // Render the timing chart
      TRACE("OPS TIMING");
      cout << indent(timings_string(), 3) << endl;

      if(false) {
         const string dot_source = make_dot();
         const string dot_fname  = format(
             "{}/comp-data_frames[{}-{}].png", outdir, start_frame, end_frame);
         run_dot(dot_source, dot_fname);
         TRACE(format("Saved dot to: {}", dot_fname));
      }

      if(false) {
         const string dot_source = make_xyt_dot();
         const string dot_fname  = format(
             "{}/xyt_frames[{}-{}].png", outdir, start_frame, end_frame);
         run_dot(dot_source, dot_fname);
         TRACE(format("Saved dot to: {}", dot_fname));
      }
   } else {
      // INFO("OPS TIMING");
      // cout << indent(timings_string(), 3) << endl;
   }

   return out;
}

// ------------------------------------------------------- calc tracks comp data
//
std::unique_ptr<ComputationData>
calc_tracks_comp_data(const SceneDescription& scene_desc,
                      const LocalizationData::Params& loc_params,
                      const Tracklet::Params& tracklet_params,
                      const Tracks::Params& params,
                      const real frame_duration,
                      const int max_frames_per_tracklet,
                      const Tracklet* tracklet,
                      const Tracklet* prev_tracklet,
                      Tracks* prev_track,
                      std::function<bool()> is_cancelled,
                      const string_view outdir,
                      const bool feedback) noexcept(false)
{
   auto get_random_seed = [&]() -> std::size_t {
      if(params.random_seed == 0) {
         std::uniform_int_distribution<std::size_t> dist;
         std::random_device rd;
         std::mt19937_64 gen(rd());
         return dist(gen);
      }
      return params.random_seed;
   };

   return std::make_unique<ComputationData>(scene_desc,
                                            loc_params,
                                            tracklet_params,
                                            params,
                                            frame_duration,
                                            max_frames_per_tracklet,
                                            tracklet,
                                            prev_tracklet,
                                            prev_track,
                                            get_random_seed(),
                                            is_cancelled,
                                            outdir,
                                            feedback);
}

// -------------------------------------------------------------- timings-string
//
string This::timings_string() const noexcept
{
   const auto tot_time
       = tot_init_time + tot_track_time + timing_pose_classification;
   const auto amortized_time = tot_time / float(end_frame - start_frame + 1);

   return trim_copy(format(R"V0G0N(
Timings frames [{}-{}) with |p2d| = {}, |inchoates| = {}, out [{}-{})

Setup `pose_dat`:               {:7.3f}ms
Calc blurred LABs               {:7.3f}ms
Init labels:                    {:7.3f}ms
Create OpSkeleton2D lookup:     {:7.3f}ms
Create 'fixed' inchoate tracks: {:7.3f}ms
Calc prob_false-positives:      {:7.3f}ms
Create graph edges:             {:7.3f}ms
Create frame XYTs:              {:7.3f}ms
Calc frame XYT still scores:    {:7.3f}ms
Create xyt edges:               {:7.3f}ms
Topological sort:               {:7.3f}ms
Check invariants:               {:7.3f}ms
Filter ground truth:            {:7.3f}ms
Pose classification:            {:7.3f}ms
Tracks time:                    {:7.3f}ms
-----------------------------------------
Total:                          {:7.3f}ms
Amortized:                      {:7.3f}ms
)V0G0N",
                           start_frame,
                           end_frame + 1,
                           pose_dat.size(),
                           inchoates.size(),
                           out.start_frame,
                           out.start_frame + out.n_frames,
                           timing_setup_pose_dat,
                           timing_calc_blurred_labs,
                           timing_init_labels,
                           timing_create_opskeleton_2d_lookup,
                           timing_create_fixed_inchoate_tracks,
                           timing_calc_prob_false_positive,
                           timing_create_graph_edges,
                           timing_create_frame_xyts,
                           timing_create_frame_xyt_still_scores,
                           timing_create_xyt_edges,
                           timing_topological_sort,
                           timing_check_invariants,
                           timing_filter_ground_truth,
                           timing_pose_classification,
                           tot_track_time,
                           tot_time,
                           amortized_time));
}

// -------------------------------------------------------------------- make-dot
//
static string kolor_attr(const int id)
{
   const auto k = (id < 0) ? k_black : colour_set_4(id);
   return format("color=\"#{:06x}\"", k);
}

static size_t skel_id(const This& o, const This::OpSkeleton2D* skel)
{
   const ptrdiff_t dist = (skel - &o.pose_dat[0]);
   Expects(dist >= 0 && size_t(dist) < o.pose_dat.size());
   return size_t(dist);
}

string This::make_dot() const noexcept
{
   const int t0 = start_frame;

   auto encode_node
       = [&](auto& ss, const int id, const int y_pos, const auto& op_skel) {
            const auto& info = *op_skel->p2d_info_ptr;
            const auto t     = info.p2d_ptr->frame_no();
            const auto pos_x = 2 * (t - t0);
            const auto pos_y = y_pos;
            const auto label = format("#{:02d}, addr = {}\np(fp) = {:5.3f}",
                                      t,
                                      op_skel->p2d_address().to_string(),
                                      op_skel->prob_false_positive);
            ss << format("   node_{} [label=\"{}\", {}];\n",
                         id,
                         encode_dot_label(label),
                         kolor_attr(info.id));
         };

   auto encode_nodes = [&](std::stringstream& ss) {
      for(const auto& op_frame : frame_dat) {
         for(size_t i = 0; i < op_frame.p2ds.size(); ++i) {
            const OpSkeleton2D* op_skel = op_frame.p2ds[i];
            encode_node(ss, int(skel_id(*this, op_skel)), int(i), op_skel);
         }
      }
   };

   auto edge_colour_attr = [&](const auto& e) {
      const OpSkeleton2D* sk_t = &pose_dat[e.this_idx];
      const OpSkeleton2D* sk_o = &pose_dat[e.other_idx];

      auto is_edge_of_interest
          = [&](auto a, auto b) -> bool { return (sk_t == a) && (sk_o == b); };

      if(sk_t->label == sk_o->label && sk_o->label != -1) {
         const auto& inchoate        = inchoates[size_t(sk_o->label)];
         const OpSkeleton2D* last_sk = nullptr;
         for(const XYT& xyt : inchoate.xyts) {
            for(auto ptr : xyt.skels) {
               if(is_edge_of_interest(last_sk, ptr))
                  return kolor_attr(inchoate.idx);
               last_sk = ptr;
            }
         }
      }
      return kolor_attr(-1);
   };

   auto encode_edges = [&](std::stringstream& ss) {
      for(const auto& op_skel : pose_dat) {
         for(size_t e_idx = 0; e_idx < op_skel.edges.size(); ++e_idx) {
            const auto& e     = op_skel.edges[e_idx];
            const auto k_attr = edge_colour_attr(e);
            ss << format("   node_{} -> node_{} [label = \"{}\", {}];\n",
                         e.this_idx,
                         e.other_idx,
                         encode_dot_label(format("{:5.3f}", e.weight)),
                         k_attr);
         }
      }
   };

   std::stringstream ss("");
   ss << "digraph DAG {\n";
   encode_nodes(ss);
   encode_edges(ss);
   ss << "}\n";

   return ss.str();
}

string This::make_xyt_dot() const noexcept
{
   const int t0 = start_frame;

   auto encode_node = [&](auto& ss, const XYTNode& node) {
      Expects(&node - &(frame_dat.at(size_t(node.t() - start_frame)).nodes[0])
              == node.frame_id());
      const auto label = format(
          "#{:02d}, id={},{} sz={}\nfp={:4.2f}, X={{{:3.1f}, {:3.1f}}}",
          node.t(),
          node.frame_id(),
          node.inchoate_id(),
          node.size(),
          node.prob_false_positive(),
          node.x(),
          node.y());
      ss << format("   node_{} [label=\"{}\", {}];\n",
                   node.id(),
                   encode_dot_label(label),
                   kolor_attr(node.inchoate_id()));
   };

   auto encode_nodes = [&](std::stringstream& ss) {
      for(const auto& op_frame : frame_dat)
         for(const auto& node : op_frame.nodes) encode_node(ss, node);
   };

   auto encode_edge = [&](std::stringstream& ss, const XYTEdge& e) {
      Expects(e.src != nullptr);
      Expects(e.dst != nullptr);
      Expects(std::isfinite(e.weight));
      const auto k_attr = kolor_attr(e.marked ? e.src->inchoate_id() : -1);
      ss << format("   node_{} -> node_{} [label = \"{}\", {}];\n",
                   e.src->id(),
                   e.dst->id(),
                   encode_dot_label(format("{:5.3f}", e.weight)),
                   k_attr);
   };

   auto encode_edges = [&](std::stringstream& ss) {
      for(const auto& op_frame : frame_dat)
         for(const auto& node : op_frame.nodes)
            for(const auto& e : node.edges()) encode_edge(ss, e);
   };

   std::stringstream ss("");
   ss << "digraph DAG {\n";
   encode_nodes(ss);
   encode_edges(ss);
   ss << "}\n";

   return ss.str();
}

// --------------------------------------------------------- xnode-lookup-string
//
string This::xnode_lookup_string(int frame_no) const noexcept
{
   std::stringstream ss{""};

   vector<std::pair<P2dAddress, XYTNode*>> stuff;
   stuff.reserve(xnode_lookup.size());
   for(const auto& ii : xnode_lookup)
      if((frame_no == -1) || (ii.first.frame_no == frame_no))
         stuff.emplace_back(ii.first, ii.second);

   std::sort(begin(stuff), end(stuff), [](const auto& ii, const auto& jj) {
      return ii.second < jj.second;
   });

   int last_frame_no = -1;
   for(const auto& ii : stuff) {
      const P2dAddress& addr = ii.first;
      const XYTNode* node    = ii.second;

      if(addr.frame_no != last_frame_no) {
         last_frame_no = addr.frame_no;
         ss << endl;
      }

      ss << format(" #{:03d} addr = {} -> {}, {}",
                   node->t(),
                   addr.to_string(),
                   node->to_string(),
                   fmt::ptr(node))
         << endl;
   }

   return ss.str();
}

string This::frame_dat_xytnode_string(int frame_no) const noexcept
{
   std::stringstream ss{""};

   vector<const XYTNode*> ostuff;
   bool first = true;

   for(const auto& frame : frame_dat) {
      if(frame_no >= 0 && frame_no != frame.t) continue;

      if(first) {
         first = false;
         ss << endl;
      }

      ostuff.resize(frame.nodes.size());
      std::transform(begin(frame.nodes),
                     end(frame.nodes),
                     begin(ostuff),
                     [](auto& node) { return &node; });

      for(const auto ptr : ostuff) {
         ss << format("   {}, {}", ptr->to_string(), fmt::ptr(ptr)) << endl;
         for(const auto o_skel_ptr : ptr->xyt().skels) {
            ss << format("      {}", o_skel_ptr->p2d_address().to_string())
               << endl;
         }
      }
   }

   return ss.str();
}

} // namespace perceive::tracks
