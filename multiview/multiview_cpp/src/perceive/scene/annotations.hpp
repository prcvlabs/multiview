
#pragma once

#include "perceive/cost-functions/fowlkes/fowlkes-result.hpp"
#include "perceive/cost-functions/tracks/labeled-track-point.hpp"

namespace perceive
{
class AnnotationData
{
 public:
   // The annotation data is, at base, a big selection of
   // labeled-track-point objects, belonging to zero or more
   // tracks.
   struct State
   {
      vector<LabeledTrackPoint> ltps = {};
      int selected_index             = -1;
      bool dirty                     = false;

      size_t size() const noexcept { return ltps.size(); }

      bool operator==(const State&) const noexcept;
      bool operator!=(const State&) const noexcept;

      Json::Value to_json() const noexcept;
      bool load_json(const Json::Value&) noexcept(false);
   };

   struct Command
   {
      enum CommandType : int {
         NOP = 0,
         SELECT_TRACK_POINT,
         CREATE_AND_SELECT_TRACKPOINT,
         DELETE_SELECTED_TRACK,
         UPDATE_SELECTED_TRACK_ID,
         UPDATE_SELECTED_POSE,
         UPDATE_SELECTED_TRACKPOINT_XY,
         UPDATE_SELECTED_TRACKPOINT_THETA,
         UPDATE_POS_ALL_TPS,
         APPLY_COMMANDS
      };

      enum State : int {
         BLACK, // The command has not been executed
         WHITE  // The command has been executed
      };

      Command() = default;
      Command(const Command&) noexcept;
      Command(Command&&) noexcept = default;
      ~Command();
      Command& operator=(const Command&) noexcept;
      Command& operator=(Command&&) noexcept = default;

      CommandType op = NOP;
      State state    = BLACK;
      int old_idx    = -1;
      int new_idx    = -1;
      LabeledTrackPoint ltp;
      bool old_dirty = false;
      vector<unique_ptr<Command>> cmds;

      bool operator==(const Command&) const noexcept;
      bool operator!=(const Command&) const noexcept;
      Json::Value to_json() const noexcept;
      bool load_json(const Json::Value&) noexcept(false);

      static CommandType str_to_op(const string_view) noexcept(false);
      static const char* op_to_str(const CommandType) noexcept;
   };

 private:
   State state_                   = {};
   vector<vector<Command>> undos_ = {};
   vector<vector<Command>> redos_ = {};

   // Never edited. If we load a file where the `state` doesn't
   // match the `undoes`, then we load `state` into state0, and
   // clear out the `undos`.
   // INVARIANT: at all times, if we undo all the undoes, we get
   // back to state0_
   State state0_ = {};

   int hist_zoom_level_ = 0;
   int hist_rotation_   = 0;

   LabeledTrackPoint* selected_trackpoint_() noexcept; // under edit

   // Apply and undo
   Command::CommandType last_command_type_() const noexcept;
   void push_command_(const Command& command) noexcept;
   void set_undo_checkpoint_() noexcept;
   bool apply_(Command command) noexcept;
   bool execute_(Command& command) noexcept;

   //@{ State mutations are managed with these functions
   bool select_trackpoint_(Command& command);
   bool create_and_select_trackpoint_(Command& command);
   bool delete_selected_track_(Command& command);
   bool update_selected_track_id_(Command& command);
   bool update_selected_pose_(Command& command);
   bool update_selected_trackpoint_xy_(Command& command);
   bool update_selected_trackpoint_theta_(Command& command);
   bool update_pos_all_tps_(Command& command);
   bool apply_commands_(Command& command);
   //@}

 public:
   bool operator==(const AnnotationData&) const noexcept;
   bool operator!=(const AnnotationData&) const noexcept;

   const State& state() const noexcept; // data being edited
   const LabeledTrackPoint* selected_trackpoint() const noexcept;
   bool is_dirty() const noexcept { return state().dirty; }

   // If we apply all `undos`, we should get to `state0`.
   // This isn't the cheapest operation. (Don't do it in a tight loop.)
   bool test_invariant() const noexcept;

   //@{ Undo/redo system
   bool undo(); // TRUE iff it actually mutated the state
   bool redo(); // TRUE iff it actually mutated the state
   //@}

   //@{ Make commands
   Command make_select_trackpoint(int new_idx);
   Command make_create_and_select_trackpoint(Point2 hist_pos, int frame_no);
   Command make_create_and_select_ltp(const LabeledTrackPoint& ltp);
   Command make_delete_selected_track();
   Command make_update_selected_track_id(int track_id);
   Command make_update_selected_pose(const PoseAnnotation pose);
   Command make_update_selected_trackpoint_xy(Point2 new_xy);
   Command make_update_selected_trackpoint_theta(const real theta);
   Command make_update_pos_all_tps(const Point2 dxy);
   Command make_apply_commands(const vector<Command>&);
   //@}

   //@{ State mutations are managed here. Returns TRUE if state changed.
   bool select_trackpoint(int new_idx); // mutations apply to selected track
   bool create_and_select_trackpoint(Point2 hist_pos, int frame_no);
   bool create_and_select_ltp(const LabeledTrackPoint& ltp);
   bool delete_selected_track();
   bool update_selected_track_id(int track_id);
   bool update_selected_pose(const PoseAnnotation pose);
   bool update_selected_trackpoint_xy(Point2 new_xy); // no bounds checking
   bool update_selected_trackpoint_theta(const real theta); // NAN to remove
   bool update_pos_all_tps(const Point2 dxy);
   bool apply_commands(const vector<Command>&);
   //@}

   //@{ Some options that require no undo
   auto hist_zoom_level() const noexcept { return hist_zoom_level_; }
   auto hist_rotation() const noexcept { return hist_rotation_; }
   void set_hist_zoom_level(int val) noexcept { hist_zoom_level_ = val; }
   void set_hist_rotation(int val) noexcept { hist_rotation_ = val; }
   //@}

   //@{ File I/O
   Json::Value to_json() const noexcept;
   Json::Value to_save_json() const noexcept; // empties `redoes`
   bool load_json(const Json::Value&) noexcept(false);
   // Used when loading a ground-truth object
   //@}

   //@{ Load/Export
   bool load_ground_truth(const string_view raw_pipeline_output) noexcept;
   bool load_annotation_data(const string_view json_data) noexcept;
   string export_annotation_data() const noexcept;
   bool init_from_pipeline_output(const FowlkesResult&) noexcept;
   //@}

   //@{
   string to_string() const noexcept;
   friend string str(const AnnotationData& o) noexcept;
   //@}
};

// --- //
/**
 * Loading Protocol
 * @see TracksEditor::init_tracker_file()
 */
string get_ground_truth_fname(const string_view annotation_dir) noexcept;
string get_ground_truth_backup_fname(const string_view annotation_dir) noexcept;
string get_annotation_fname(const string_view annotation_dir) noexcept;
string get_annotation_backup_fname(const string_view annotation_dir) noexcept;

} // namespace perceive
