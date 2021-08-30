
#pragma once

#include "tracks-exec.hpp"

#include "perceive/cost-functions/classifier/classifier.hpp"
#include "perceive/cost-functions/tracklets/tracklet-exec.hpp"
#include "perceive/geometry/spatial-index.hpp"
#include "perceive/optimization/min-cut-graph.hpp"
#include "perceive/utils/threads.hpp"

#include <random>
#include <tuple>

namespace perceive::tracks
{
struct AllPathsInfo;

struct ComputationData
{
 public:
   using NodeSequence = tracks::NodeSequence;

   const bool is_trace_mode = false;
   const SceneDescription& scene_desc;

   std::minstd_rand random_generator; // linear_congruential_engine

   const LocalizationData::Params loc_params;
   const Tracklet::Params tracklet_params;
   const Tracks::Params params;
   Skeleton2DInterpolation::Params interp_params;

   const Tracklet* tracklet           = nullptr;
   const Tracklet* prev_tracklet      = nullptr;
   const real frame_duration          = dNAN;
   const int max_frames_per_tracklet  = -1;
   std::function<bool()> is_cancelled = nullptr;
   const string outdir                = ""s;
   const bool feedback                = false;

   const int n_frames    = 0;
   const int start_frame = 0;
   const int end_frame   = 0; // inclusive
   const real hist_sz    = 0.0;
   const AABB aabb;
   const int w = 0;
   const int h = 0;

   const Classifier* pose_classifier = nullptr;

   // Timings
   float timing_setup_pose_dat                = 0.0f;
   float timing_calc_blurred_labs             = 0.0f;
   float timing_init_labels                   = 0.0f;
   float timing_create_frame_xyts             = 0.0f;
   float timing_create_frame_xyt_still_scores = 0.0f;
   float timing_create_opskeleton_2d_lookup   = 0.0f;
   float timing_create_fixed_inchoate_tracks  = 0.0f;
   float timing_calc_prob_false_positive      = 0.0f;
   float timing_create_graph_edges            = 0.0f;
   float timing_create_xyt_edges              = 0.0f;
   float timing_check_invariants              = 0.0f;
   float timing_making_tracks                 = 0.0f;
   float timing_topological_sort              = 0.0f;
   float timing_filter_ground_truth           = 0.0f;
   float timing_pose_classification           = 0.0f;
   float tot_init_time                        = 0.0f;
   float tot_track_time                       = 0.0f;

   struct Edge
   {
      enum EdgeType : int8_t {
         UNSET = 0,
         T0,           // The same frame
         T1,           // Subsequent frame
         INTERPOLATION // And interpolation to a future frame
      };

      Vector2f X            = {};   // on the floor
      float dist            = fNAN; // distance travelled across the floor
      float weight          = fNAN;
      float lab_patch_score = fNAN; // How well do color patches match?
      uint32_t this_idx     = 0;
      uint32_t other_idx    = 0;
      EdgeType type         = UNSET;
      shared_ptr<const Skeleton2DInterpolation> interp_ptr = nullptr;

      float score() const noexcept
      {
         return weight + lab_patch_score / 1000.0f;
      }
   };

   // Per pose data for the computation
   struct OpSkeleton2D
   {
      const Skeleton2DInfo* p2d_info_ptr = nullptr;

      vector<Edge> edges;
      int t     = -1; // frame number
      int label = -1; // The track label
      Point2 offset
          = {-1, -1}; // this == inchoates[label].xyts[offset.x].nodes[offset.y]
      bool is_fixed
          = false; // OpSkeleton2D cannot be reassigned to another track
      float prob_false_positive = fNAN;

      int p2d_id() const noexcept { return p2d_info_ptr->id; }

      int sensor_no() const noexcept
      {
         return p2d_info_ptr->p2d_ptr->sensor_no();
      }

      P2dAddress p2d_address() const noexcept
      {
         return {t, sensor_no(), p2d_id()};
      }

      const Skeleton2D* p2d_ptr() const noexcept
      {
         return (p2d_info_ptr == nullptr) ? nullptr
                                          : p2d_info_ptr->p2d_ptr.get();
      }

      bool is_interpolation() const noexcept
      {
         const auto ptr = p2d_ptr();
         if(ptr == nullptr) return false;
         const auto ret = ptr->is_interpolation();
         return ret;
      }

      const vector<SparseHistCell>* hist_ptr() const noexcept
      {
         return (p2d_info_ptr == nullptr) ? nullptr : &p2d_info_ptr->hist;
      }

      const vector<LABImage>* patches() const noexcept
      {
         return (p2d_info_ptr == nullptr) ? nullptr : &p2d_info_ptr->patches;
      }

      Vector3f best_X() const noexcept
      {
         return (p2d_info_ptr == nullptr)
                    ? Vector3f::nan()
                    : p2d_info_ptr->p2d_ptr->best_3d_result().Xs_centre();
      }

      bool is_assigned() const noexcept { return label >= 0; }
   };

   struct XYT
   {
      vector<OpSkeleton2D*> skels       = {}; // The nodes of this track
      float x                           = fNAN;
      float y                           = fNAN;
      int t                             = -1;
      float gaze_theta                  = fNAN;
      float still_score                 = fNAN;
      bool is_interp                    = false;
      vector<float> pose_classification = {};

      XYT(float x_ = 0.0f, float y_ = 0.0f, int t_ = 0)
          : x(x_)
          , y(y_)
          , t(t_)
      {}

      XYT(const ComputationData& ops,
          vector<OpSkeleton2D*>&& skels,
          const int t)
      noexcept;

      string to_string() const noexcept;
   };

   struct XYTNode;
   struct InchoateTrack;

   struct XYTEdge
   {
      XYTNode* src = nullptr;
      XYTNode* dst = nullptr;
      float weight = fNAN;
      bool marked  = false; // when graph travesal goes through this node

      XYTEdge() = default;
      XYTEdge(XYTNode* a, XYTNode* b, float w = fNAN)
          : src(a)
          , dst(b)
          , weight(w)
      {}

      int dst_frame() const noexcept { return dst->t(); }

      void set_inchoate_id(int id)
      {
         if(src) src->set_inchoate_id(id);
         if(dst) dst->set_inchoate_id(id);
         marked = (id >= 0);
      }

      bool dt() const noexcept { return dst->t() - src->t(); }

      bool is_assigned() const noexcept
      {
         return (src->inchoate_id() >= 0) && (dst->inchoate_id() >= 0);
      }
   };

   struct XYTNode
   {
    private:
      int id_                = -1; // unique across all frame
      int frame_id_          = -1; // unique within a single frame
      int inchoate_id_       = -1; // if set, then Node is assigned to a track
      XYT xyt_               = {};
      vector<XYTEdge> edges_ = {};

      // The nodes keep a topological sort position, that points to
      // it's location (in topological order) in `sorted_xyt_nodes` below
      int topological_pos_ = -1;

    public:
      void set_id(int id) noexcept { id_ = id; }
      void set_frame_id(int frame_id) noexcept { frame_id_ = frame_id; }
      void set_inchoate_id(int val) noexcept { inchoate_id_ = val; }
      void set_xyt(XYT&& xyt) noexcept { xyt_ = std::move(xyt); }
      void set_topological_pos(int val) noexcept { topological_pos_ = val; }

      int id() const noexcept { return id_; }
      int frame_id() const noexcept { return frame_id_; }
      int inchoate_id() const noexcept { return inchoate_id_; }
      float x() const noexcept { return xyt_.x; }
      float y() const noexcept { return xyt_.y; }
      Vector2f xy() const noexcept { return Vector2f(x(), y()); }
      float gaze_theta() const noexcept { return xyt_.gaze_theta; }
      int t() const noexcept { return xyt_.t; }
      bool is_interp() const noexcept { return xyt_.is_interp; }
      int topological_pos() const noexcept { return topological_pos_; }
      XYT& xyt() noexcept { return xyt_; }

      const XYT& xyt() const noexcept { return xyt_; }
      const auto& skels() const noexcept { return xyt_.skels; }
      auto& edges() noexcept { return edges_; }
      const auto& edges() const noexcept { return edges_; }

      XYTEdge* find_edge(const XYTNode* o) noexcept
      {
         auto ii = std::find_if(
             begin(edges()), end(edges()), [this, o](const auto e) {
                Expects(e.src == this);
                return e.dst == o;
             });
         return (ii == end(edges())) ? nullptr : &(*ii);
      }

      size_t size() const noexcept { return skels().size(); }

      float prob_false_positive() const noexcept
      {
         if(size() == 0) return 1.0f;
         auto average = 0.0f;
         for(auto ptr : skels()) average += ptr->prob_false_positive;
         return average / float(size());
      }

      string to_string() const noexcept
      {
         return format("[{:03d}, id={:03d}, frame-id={:03d}, inchoate={:03d}, "
                       "xy={{{:4.3f}, "
                       "{:4.3f}}}, skels={{{}}}]",
                       t(),
                       id(),
                       frame_id(),
                       inchoate_id(),
                       x(),
                       y(),
                       implode(begin(xyt().skels),
                               end(xyt().skels),
                               ", ",
                               [](const OpSkeleton2D* ptr) {
                                  return ptr->p2d_address().to_string();
                               }));
      }
   };

   // Per frame data for the computation
   struct OpFrameData
   {
      const LocalizationData* loc_ptr;        // 1 for each frame
      vector<const LABImage*> sensor_labs;    // 1 for each sensor
      vector<const PointCloud*> point_clouds; // 1 for each camera
      vector<OpSkeleton2D*> p2ds;             // for this frame
      vector<XYTNode> nodes;                  // XYT possibilities for frame
      Spatial2Index gt_spatial_index;         // for finding ground-truh
      vector<TrackPoint> gt_tps;              // ground-truth track-points
      int t = -1;                             // frame number
   };

   // Track in the process of being created
   struct InchoateTrack
   {
      int idx                  = -1; // Index within `inchoates`
      int node_seq_id          = -1; // Track 'id' to preserve is possible
      vector<XYT> xyts         = {}; // 1 for each frame in the sequence
      vector<XYTEdge*> edges   = {}; // sets of xyts for a given frame
      vector<Vec3fImage> model = {}; // The reference color model
      int model_sz             = 0;  // model is a sum. Divide to get average
      bool is_available() const noexcept
      {
         return xyts.empty() && edges.empty();
      }
   };

   //@{ Dynamic Computation Data
   ParallelJobSet pjobs = {};
   Tracks* prev_track   = nullptr;
   Tracks out           = {};
   vector<OpFrameData> frame_dat;
   vector<OpSkeleton2D> pose_dat; // All `is-interpolation == false`
   vector<XYTNode*> sorted_xyt_nodes;
   vector<LABImage> blur_labs;
   std::unordered_map<P2dAddress, OpSkeleton2D*> p2d_lookup; // only non-interp
   std::unordered_map<P2dAddress, XYTNode*> xnode_lookup;    // only non-interp
   std::unordered_map<uint64_t, Edge*> edge_lookup;
   size_t max_n_labels = 0; // maximum number of tracks, zero => zero detections
   size_t max_n_p2ds   = 0;
   MatrixXf label_p2d_scores; // rows=labels, cols=p2ds, cells=scores
   vector<tracks::NodeSequence> complete_seqs; // for filling training data
   vector<InchoateTrack> inchoates;
   vector<TrackPoint> ground_truth_tps;
   const LABImage* get_lab_ptr(int frame_no, int sensor_no) const;
   real calc_speed(
       const InchoateTrack& it) const noexcept; // one speed for entire track
   real calc_speed(const tracks::NodeSequence& seq) const noexcept;
   void classify_pose(XYTNode* node, const real speed) const noexcept;
   //@}

   //@{
   int random(int min_val, int max_val) noexcept; // inclusive
   float calc_speed_p(const float dist, const float duration) const noexcept;
   //@}

   //@{ For handling scores
   float track_p2d_score(unsigned label, unsigned p2d_idx) const noexcept;
   void
   set_track_p2d_score(unsigned label, unsigned p2d_idx, float val) noexcept;
   void clear_track_p2d(unsigned label, unsigned p2d_idx) noexcept;
   bool has_track_p2d(unsigned label, unsigned p2d_idx) const noexcept;
   //@}

   bool is_fixed_track_frame(int frame_no) const noexcept;
   real total_cost(const vector<NodeSequence>&) noexcept;

   //@{ Helpers
   void init_xyt_edges_() noexcept;

   void init_frame_fixed_xyt_nodes_(OpFrameData& frame,
                                    vector<bool>& marks) noexcept;
   void init_frame_xyt_nodes_(OpFrameData& frame) noexcept;

   float convert_pos_info_score(const float score) const noexcept;
   void init_fixed_inchoate_tracks_() noexcept;
   float calc_prob_false_positive(const OpSkeleton2D& op_p2d) noexcept;

   Edge make_edge(const Skeleton2DInfo& p2d0,
                  const Skeleton2DInfo& p2d1,
                  const int frame0,
                  const int frame1,
                  const uint32_t this_idx,
                  const uint32_t other_idx) noexcept;

   const Edge* lookup_edge(const uint32_t a_idx,
                           const uint32_t b_idx) const noexcept;
   const Edge* lookup_edge(const OpSkeleton2D& a,
                           const OpSkeleton2D& b) const noexcept;

   OpSkeleton2D* find_op_skel(const P2dAddress& address) noexcept;

   MinCutGraph make_min_cut_graph_() noexcept(false); // alloc_error

   template<typename T> Vector2 hist_to_X(const T& h) const noexcept
   {
      return Vector2(real(h.x) * hist_sz + aabb.left,
                     real(h.y) * hist_sz + aabb.top);
   }

   template<typename T> Vector3f hist_to_X3f(const T& h) const noexcept
   {
      const auto x = hist_to_X(h);
      return Vector3f(float(x.x), float(x.y), 0.0f);
   }

   template<typename T> Vector2f X_to_hist(const T& xy) const noexcept
   {
      return Vector2f(float((real(xy.x) - aabb.left) / hist_sz),
                      float((real(xy.y) - aabb.top) / hist_sz));
   }
   //@}

   // @{
   void extend_xyt_nodes_to_frame_all_paths_(int frame_no);
   //@}

   //@{ For making tracks
   InchoateTrack& seed_node_sequence_(OpSkeleton2D* op_skel_2d);
   void extend_to_frame_(int frame_no);
   void extend_xyt_nodes_to_frame_(int frame_no);

   void extend_track_(InchoateTrack& inchoate) noexcept;
   bool seed_one_track_() noexcept;
   //@}

   //@{ Operations
   void check_inchoate_(const InchoateTrack&) const noexcept;
   void check_invariants_() const noexcept;
   void update_label_p2d_scores_() noexcept;
   void fill_training_data_();
   void finalize_result_();
   //@}

   unsigned op_skel_2d_index_(const OpSkeleton2D* op_skel) const noexcept
   {
      Expects(pose_dat.size() > 0);
      const ptrdiff_t idx = (op_skel - &pose_dat[0]);
      Expects(idx >= 0);
      Expects(size_t(idx) < pose_dat.size());
      return unsigned(idx);
   }

 public:
   ComputationData(const SceneDescription& scene_desc,
                   const LocalizationData::Params& loc_params,
                   const Tracklet::Params& tracklet_params,
                   const Tracks::Params& params,
                   const real frame_duration,
                   const int max_frames_per_tracklet,
                   const Tracklet* tracklet,
                   const Tracklet* prev_tracklet,
                   Tracks* prev_track,
                   const size_t random_seed,
                   std::function<bool()> is_cancelled,
                   const string_view outdir,
                   const bool feedback);

   ComputationData(ComputationData&&) = default;
   ComputationData& operator=(ComputationData&&) = delete;

   ~ComputationData();

   Tracks execute(); // Also updates `prev-track` if non-null

   string timings_string() const noexcept;
   string make_dot() const noexcept;
   string make_xyt_dot() const noexcept;
   string xnode_lookup_string(int frame_no = -1) const noexcept;
   string frame_dat_xytnode_string(int frame_no = -1) const noexcept;
};

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
                      std::function<bool()> in_is_cancelled,
                      const string_view outdir,
                      const bool feedback) noexcept(false);

} // namespace perceive::tracks
