
#include "annotations.hpp"

#include "perceive/pipeline/pipeline-output.hpp"

#define This AnnotationData

namespace perceive
{
// ----------------------------------------------------------------------- State
//
bool This::State::operator==(const State& o) const noexcept
{
   return dirty == o.dirty && selected_index == o.selected_index
          && ltps == o.ltps;
}

bool This::State::operator!=(const State& o) const noexcept
{
   return !(*this == o);
}

Json::Value This::State::to_json() const noexcept
{
   auto x    = Json::Value{Json::objectValue};
   x["ltps"] = json_save_t(
       cbegin(ltps), cend(ltps), [](const auto& o) { return o.to_json(); });
   x["selected_index"] = json_save(selected_index);
   x["dirty"]          = json_save(dirty);
   return x;
}

bool This::State::load_json(const Json::Value& x) noexcept(false)
{
   State o;

   const auto op = "reading AnnotationData::State";

   if(!json_try_load_key(o.dirty, x, "dirty", op)) return false;

   if(!json_try_load_key(o.selected_index, x, "selected_index", op))
      return false;

   if(!x.isMember("ltps")) {
      WARN(format("failed to find key 'ltps' when {}", op));
      return false;
   }

   const auto& arr = x["ltps"];
   if(arr.type() != Json::arrayValue) {
      WARN(format("expected array key for 'ltps', while {}", op));
      return false;
   }

   o.ltps.resize(arr.size());
   for(size_t i = 0; i < o.ltps.size(); ++i) {
      if(!o.ltps[i].load_json(arr[int(i)])) {
         WARN(format("failed to read 'ltps', while {}", op));
         return false;
      }
   }

   *this = o;
   return true;
}

// --------------------------------------------------------------------- Command
//
This::Command::Command(const Command& o) noexcept { *this = o; }

This::Command::~Command() {}

This::Command& This::Command::operator=(const Command& o) noexcept
{
   this->op        = o.op;
   this->state     = o.state;
   this->old_idx   = o.old_idx;
   this->new_idx   = o.new_idx;
   this->ltp       = o.ltp;
   this->old_dirty = o.old_dirty;

   cmds.clear();
   cmds.resize(o.cmds.size());
   std::transform(
       cbegin(o.cmds), cend(o.cmds), begin(cmds), [&](const auto& x) {
          Expects(x != nullptr);
          return make_unique<Command>(*x);
       });

   return *this;
}

bool This::Command::operator==(const Command& o) const noexcept
{
#define TEST(x) (x == o.x)
   return TEST(op) && TEST(state) && TEST(old_idx) && TEST(new_idx) && TEST(ltp)
          && TEST(old_dirty);
#undef TEST
}

bool This::Command::operator!=(const Command& o) const noexcept
{
   return !(*this == o);
}

Json::Value This::Command::to_json() const noexcept
{
   auto x         = Json::Value{Json::objectValue};
   x["op"]        = op_to_str(op);
   x["applied"]   = json_save(state == WHITE);
   x["old_idx"]   = json_save(old_idx);
   x["new_idx"]   = json_save(new_idx);
   x["old_dirty"] = json_save(old_dirty);
   x["ltp"]       = ltp.to_json();
   x["cmds"]      = json_save_t(
       cbegin(cmds), cend(cmds), [](const auto& o) { return o->to_json(); });

   return x;
}

bool This::Command::load_json(const Json::Value& x) noexcept(false)
{
   Command o;

   const auto op  = "loading Command";
   bool has_error = false;

   string op_s  = ""s;
   bool applied = false;
   if(!json_try_load_key(op_s, x, "op", op)) has_error = true;
   if(!json_try_load_key(applied, x, "applied", op)) has_error = true;
   if(!json_try_load_key(o.old_idx, x, "old_idx", op)) has_error = true;
   if(!json_try_load_key(o.new_idx, x, "new_idx", op)) has_error = true;
   if(!json_try_load_key(o.old_dirty, x, "old_dirty", op)) has_error = true;
   if(!o.ltp.load_json(x["ltp"])) has_error = true;
   if(has_key(x, "cmds")) {
      const auto& arr = x["cmds"];
      if(arr.type() != Json::arrayValue) {
         WARN(format("expected json-array for 'cmds' while {}", op));
         has_error = true;
      } else {
         o.cmds.resize(arr.size());
         std::transform(cbegin(arr),
                        cend(arr),
                        begin(o.cmds),
                        [&](const auto& json_val) -> unique_ptr<Command> {
                           auto ret = make_unique<Command>();
                           ret->load_json(json_val);
                           return ret;
                        });
      }
   }

   try {
      o.op = str_to_op(op_s);
   } catch(std::exception& e) {
      WARN(format("failed to decode op='{}', while {}", op_s, op));
      has_error = true;
   }

   if(!has_error) {
      o.state = (applied == true) ? WHITE : BLACK;
      *this   = o;
      return true;
   }
   return false;
}

This::Command::CommandType
This::Command::str_to_op(const string_view s) noexcept(false)
{
#define CASE(x) \
   if(s == #x) return x;
   CASE(NOP);
   CASE(SELECT_TRACK_POINT);
   CASE(CREATE_AND_SELECT_TRACKPOINT);
   CASE(DELETE_SELECTED_TRACK);
   CASE(UPDATE_SELECTED_TRACK_ID);
   CASE(UPDATE_SELECTED_POSE);
   CASE(UPDATE_SELECTED_TRACKPOINT_XY);
   CASE(UPDATE_SELECTED_TRACKPOINT_THETA);
   CASE(UPDATE_POS_ALL_TPS);
   CASE(APPLY_COMMANDS);

   throw std::runtime_error(
       format("unknown CommandType: {}, returning NOP", s));
#undef CASE
}

const char* This::Command::op_to_str(const CommandType cmd) noexcept
{
#define CASE(x) \
   case x: return #x
   switch(cmd) {
      CASE(NOP);
      CASE(SELECT_TRACK_POINT);
      CASE(CREATE_AND_SELECT_TRACKPOINT);
      CASE(DELETE_SELECTED_TRACK);
      CASE(UPDATE_SELECTED_TRACK_ID);
      CASE(UPDATE_SELECTED_POSE);
      CASE(UPDATE_SELECTED_TRACKPOINT_XY);
      CASE(UPDATE_SELECTED_TRACKPOINT_THETA);
      CASE(UPDATE_POS_ALL_TPS);
      CASE(APPLY_COMMANDS);

   default: FATAL(format("logic error, cmd = {}", int(cmd)));
   }
   return "<unknoown>";
#undef CASE
}

// ----------------------------------------------------------------- operator==
//
bool This::operator==(const AnnotationData& o) const noexcept
{
#define TEST(x) (x == o.x)
   return TEST(state_) && TEST(state0_) && TEST(undos_) && TEST(redos_);
#undef TEST
}

bool This::operator!=(const AnnotationData& o) const noexcept
{
   return !(*this == o);
}

// -----------------------------------------------------------------------------
//
const This::State& This::state() const noexcept { return state_; }

const LabeledTrackPoint* This::selected_trackpoint() const noexcept
{
   return const_cast<This*>(this)->selected_trackpoint_();
}

LabeledTrackPoint* This::selected_trackpoint_() noexcept
{
   auto& e = state_;
   return (size_t(e.selected_index) < e.ltps.size())
              ? &e.ltps[size_t(e.selected_index)]
              : nullptr;
}

// ------------------------------------------------------------ undo/redo system
//
bool This::undo()
{
   if(undos_.size() == 0) return false; // nothing to undo

   // Grab the commands to revert
   vector<Command> cmds = std::move(undos_.back());
   undos_.pop_back(); //

   // Revert by applying commands in REVERSE order
   for(auto ii = rbegin(cmds); ii != rend(cmds); ++ii) execute_(*ii);

   // Add the commands as a `redo`
   redos_.push_back(std::move(cmds));

   return true;
}

bool This::redo()
{
   if(redos_.size() == 0) return false; // nothing to redo

   // Grab the commands to redo,
   vector<Command> cmds = std::move(redos_.back());
   redos_.pop_back();

   // Apply the commands in order
   for(auto ii = begin(cmds); ii != end(cmds); ++ii) execute_(*ii);

   // Add the commands as an `undo`
   undos_.push_back(std::move(cmds));

   return true;
}

// -------------------------------------------------------------------- Commands
//
This::Command::CommandType This::last_command_type_() const noexcept
{
   if(undos_.size() == 0) return Command::NOP;
   if(undos_.back().size() == 0) return Command::NOP;
   return undos_.back().back().op;
}

void This::push_command_(const Command& command) noexcept
{
   // Redos lose logical sense if they hang around
   const bool we_keep_redos_as_undo = false;
   if(we_keep_redos_as_undo) {
      std::reverse(begin(redos_), end(redos_));
      vector<Command> all_redos = flatten(redos_);
      if(all_redos.size() > 0) undos_.emplace_back(std::move(all_redos));
   } else {
      redos_.clear();
   }

   // If we've got no undo-set at the moment, then create it
   if(undos_.size() == 0) set_undo_checkpoint_();

   // We're ready to push our lowly command
   undos_.back().push_back(command);
}

void This::set_undo_checkpoint_() noexcept
{
   // Create a new `undo`, but not if there's an empty undo already in place
   if(undos_.size() == 0 || undos_.back().size() != 0) undos_.emplace_back();
}

bool This::apply_(Command command) noexcept
{
   const auto ret = execute_(command);
   if(ret) push_command_(command); // only push if state has been modified
   return ret;
}

bool This::execute_(Command& command) noexcept
{
   switch(command.op) {
   case Command::NOP: return false;
   case Command::SELECT_TRACK_POINT: return select_trackpoint_(command);
   case Command::CREATE_AND_SELECT_TRACKPOINT:
      return create_and_select_trackpoint_(command);
   case Command::DELETE_SELECTED_TRACK: return delete_selected_track_(command);
   case Command::UPDATE_SELECTED_TRACK_ID:
      return update_selected_track_id_(command);
   case Command::UPDATE_SELECTED_POSE: return update_selected_pose_(command);
   case Command::UPDATE_SELECTED_TRACKPOINT_XY:
      return update_selected_trackpoint_xy_(command);
   case Command::UPDATE_SELECTED_TRACKPOINT_THETA:
      return update_selected_trackpoint_theta_(command);
   case Command::UPDATE_POS_ALL_TPS: return update_pos_all_tps_(command);
   case Command::APPLY_COMMANDS: return apply_commands_(command);
   }

   Expects(false);
   return false;
}

// ------------------------------------------------------------------ Operations

bool This::select_trackpoint_(Command& cmd)
{
   Expects(cmd.op == Command::SELECT_TRACK_POINT);

   if(cmd.state == Command::BLACK) { // Apply
      const auto& track_points = state_.ltps;
      if(cmd.new_idx >= int(track_points.size())
         || cmd.new_idx == state_.selected_index) {
         cmd.op = Command::NOP;
         return false; // nothing changed
      }

      set_undo_checkpoint_();
      cmd.old_idx           = state_.selected_index;
      state_.selected_index = cmd.new_idx;
      cmd.state             = Command::WHITE; // Applied
      return true;
   }

   // Revert
   Expects(state_.selected_index == cmd.new_idx);
   state_.selected_index = cmd.old_idx;
   cmd.state             = Command::BLACK; // Reverted
   return true;
}

bool This::create_and_select_trackpoint_(Command& cmd)
{
   Expects(cmd.op == Command::CREATE_AND_SELECT_TRACKPOINT);

   if(cmd.state == Command::BLACK) { // Apply
      cmd.old_idx   = state_.selected_index;
      cmd.old_dirty = state_.dirty;
      set_undo_checkpoint_();
      state_.ltps.push_back(cmd.ltp);
      state_.selected_index = int(state_.ltps.size()) - 1;
      state_.dirty          = true;
      cmd.state             = Command::WHITE; // Applied
      return true;
   }

   // Revert
   auto& track_points = state_.ltps;
   Expects(track_points.size() > 0);
   Expects(state_.dirty == true);
   state_.ltps.pop_back();
   state_.selected_index = cmd.old_idx;
   state_.dirty          = cmd.old_dirty;
   cmd.state             = Command::BLACK; // Reverted
   return true;
}

bool This::delete_selected_track_(Command& cmd) // kBAM!
{
   Expects(cmd.op == Command::DELETE_SELECTED_TRACK);

   if(cmd.state == Command::BLACK) { // Apply
      if(!selected_trackpoint()) {
         cmd.op = Command::NOP;
         return false;
      }

      cmd.old_idx   = state_.selected_index;
      cmd.old_dirty = state_.dirty;
      set_undo_checkpoint_();
      auto& track_points = state_.ltps;
      const size_t i     = size_t(state_.selected_index);
      Expects(i < track_points.size());
      cmd.ltp = track_points[i];
      if(i + 1 < track_points.size())
         std::swap(track_points[i], track_points.back());
      track_points.pop_back();
      state_.selected_index = -1;
      state_.dirty          = true;
      cmd.state             = Command::WHITE; // Applied
      return true;
   }

   auto& track_points = state_.ltps;
   track_points.push_back(cmd.ltp);
   if(cmd.old_idx + 1 < int(track_points.size()))
      std::swap(track_points.back(), track_points[size_t(cmd.old_idx)]);
   state_.selected_index = cmd.old_idx;
   state_.dirty          = cmd.old_dirty;
   cmd.state             = Command::BLACK; // Reverted
   return true;
}

bool This::update_selected_track_id_(Command& cmd)
{
   Expects(cmd.op == Command::UPDATE_SELECTED_TRACK_ID);

   if(cmd.state == Command::BLACK) { // Apply
      if(!selected_trackpoint()
         || selected_trackpoint()->track_id == cmd.ltp.track_id) {
         cmd.op = Command::NOP;
         return false;
      }

      cmd.old_dirty = state_.dirty;
      set_undo_checkpoint_();
      std::swap(selected_trackpoint_()->track_id, cmd.ltp.track_id);
      state_.dirty = true;
      cmd.state    = Command::WHITE; // Applied
      return true;
   }

   Expects(selected_trackpoint());
   std::swap(selected_trackpoint_()->track_id, cmd.ltp.track_id);
   state_.dirty = cmd.old_dirty;
   cmd.state    = Command::BLACK; // Reverted
   return true;
}

bool This::update_selected_pose_(Command& cmd)
{
   Expects(cmd.op == Command::UPDATE_SELECTED_POSE);

   if(cmd.state == Command::BLACK) { // Apply
      if(!selected_trackpoint()
         || selected_trackpoint()->tp.pose == cmd.ltp.tp.pose) {
         cmd.op = Command::NOP;
         return false;
      }
      cmd.old_dirty = state_.dirty;
      set_undo_checkpoint_();
      std::swap(selected_trackpoint_()->tp.pose, cmd.ltp.tp.pose);
      state_.dirty = true;
      cmd.state    = Command::WHITE; // Applied
      return true;
   }

   Expects(selected_trackpoint());
   std::swap(selected_trackpoint_()->tp.pose, cmd.ltp.tp.pose);
   state_.dirty = cmd.old_dirty;
   cmd.state    = Command::BLACK; // Reverted
   return true;
}

bool This::update_selected_trackpoint_xy_(Command& cmd)
{
   Expects(cmd.op == Command::UPDATE_SELECTED_TRACKPOINT_XY);

   if(cmd.state == Command::BLACK) { // Apply
      if(!selected_trackpoint()
         || selected_trackpoint_()->tp.xy() == cmd.ltp.tp.xy()) {
         cmd.op = Command::NOP;
         return false;
      }

      cmd.old_dirty = state_.dirty;
      if(last_command_type_() != cmd.op) set_undo_checkpoint_();
      std::swap(selected_trackpoint_()->tp.x, cmd.ltp.tp.x);
      std::swap(selected_trackpoint_()->tp.y, cmd.ltp.tp.y);
      state_.dirty = true;
      cmd.state    = Command::WHITE; // Applied
      return true;
   }

   Expects(selected_trackpoint());
   std::swap(selected_trackpoint_()->tp.x, cmd.ltp.tp.x);
   std::swap(selected_trackpoint_()->tp.y, cmd.ltp.tp.y);
   state_.dirty = cmd.old_dirty;
   cmd.state    = Command::BLACK; // Reverted
   return true;
}

bool This::update_selected_trackpoint_theta_(Command& cmd)
{
   Expects(cmd.op == Command::UPDATE_SELECTED_TRACKPOINT_THETA);

   if(cmd.state == Command::BLACK) { // Apply
      if(!selected_trackpoint()
         || selected_trackpoint()->tp.gaze_direction
                == cmd.ltp.tp.gaze_direction) {
         cmd.op = Command::NOP;
         return false;
      }

      cmd.old_dirty = state_.dirty;
      if(last_command_type_() != cmd.op) set_undo_checkpoint_();
      std::swap(selected_trackpoint_()->tp.gaze_direction,
                cmd.ltp.tp.gaze_direction);
      state_.dirty = true;
      cmd.state    = Command::WHITE; // Applied
      return true;
   }

   Expects(selected_trackpoint());
   std::swap(selected_trackpoint_()->tp.gaze_direction,
             cmd.ltp.tp.gaze_direction);
   state_.dirty = cmd.old_dirty;
   cmd.state    = Command::BLACK; // Reverted
   return true;
}

bool This::update_pos_all_tps_(Command& cmd)
{
   Expects(cmd.op == Command::UPDATE_POS_ALL_TPS);

   if(cmd.state == Command::BLACK) { // Apply
      if(state_.size() == 0) {
         cmd.op = Command::NOP;
         return false;
      }

      cmd.old_dirty = state_.dirty;
      if(last_command_type_() != cmd.op) set_undo_checkpoint_();

      for(auto& ltp : state_.ltps) {
         ltp.tp.x += cmd.ltp.tp.x;
         ltp.tp.y += cmd.ltp.tp.y;
      }

      state_.dirty = true;
      cmd.state    = Command::WHITE; // Applied
      return true;
   }

   //
   for(auto& ltp : state_.ltps) {
      ltp.tp.x -= cmd.ltp.tp.x;
      ltp.tp.y -= cmd.ltp.tp.y;
   }

   state_.dirty = cmd.old_dirty;
   cmd.state    = Command::BLACK; // Reverted
   return true;
}

bool This::apply_commands_(Command& cmd)
{
   Expects(cmd.op == Command::APPLY_COMMANDS);

   auto apply_all = [&]() {
      auto& seq = cmd.cmds;

      const auto undos_index = undos_.size();
      set_undo_checkpoint_();

      bool success            = seq.size() > 0;
      size_t last_working_seq = seq.size();

      Json::StyledWriter writer;
      for(size_t i = 0; i < seq.size() && success; ++i) {
         if(execute_(*seq.at(i)))
            last_working_seq = i;
         else
            success = false;
      }

      if(!success) { // Undo!!!
         if(last_working_seq < seq.size())
            for(auto i = int64_t(last_working_seq); i >= 0; --i)
               execute_(*seq.at(size_t(i)));
      }

      // We want `undos_` to be clear
      if(undos_index < undos_.size())
         undos_.erase(begin(undos_) + long(undos_index), end(undos_));

      return success;
   };

   if(cmd.state == Command::BLACK) { // Apply
      if(cmd.cmds.size() == 0) {
         cmd.op = Command::NOP;
         return false;
      }

      cmd.old_dirty = state_.dirty;

      INFO("ABOUT TO APPLY");
      // ---- Apply in forward order
      if(!apply_all()) {
         cmd.op = Command::NOP;
         cmd.cmds.clear();
         return false;
      }
      // ----

      state_.dirty = true;
      cmd.state    = Command::WHITE; // Applied
      return true;
   }

   // ---- Apply in reverse order
   for(auto ii = rbegin(cmd.cmds); ii != rend(cmd.cmds); ++ii) execute_(**ii);
   // ----

   state_.dirty = cmd.old_dirty;
   cmd.state    = Command::BLACK; // Reverted
   return true;
}

// -------------------------------------------------------------- test invariant
//
bool This::test_invariant() const noexcept
{
   AnnotationData adat = *this;

   // Now, check the invariant: does applying all of the undos lead us
   // to `state0`?
   while(adat.undo())
      ;
   const bool invariant_holds = (adat.state_ == adat.state0_);
   return invariant_holds;
}

// ------------------------------------------------------------- Making Commands
//
This::Command This::make_select_trackpoint(int new_idx)
{
   Command cmd;
   cmd.op      = Command::SELECT_TRACK_POINT;
   cmd.new_idx = new_idx;
   return cmd;
}

This::Command This::make_create_and_select_trackpoint(Point2 hist_pos,
                                                      int frame_no)
{
   Command cmd;
   cmd.op       = Command::CREATE_AND_SELECT_TRACKPOINT;
   cmd.ltp      = LabeledTrackPoint{};
   cmd.ltp.tp.x = float(hist_pos.x); // SAFE ROUNDING, remember that
   cmd.ltp.tp.y = float(hist_pos.y); // these are annotations
   cmd.ltp.tp.t = frame_no;
   return cmd;
}

This::Command This::make_create_and_select_ltp(const LabeledTrackPoint& ltp)
{
   Command cmd;
   cmd.op  = Command::CREATE_AND_SELECT_TRACKPOINT;
   cmd.ltp = ltp;
   return cmd;
}

This::Command This::make_delete_selected_track()
{
   Command cmd;
   cmd.op = Command::DELETE_SELECTED_TRACK;
   return cmd;
}

This::Command This::make_update_selected_track_id(int track_id)
{
   Command cmd;
   cmd.op           = Command::UPDATE_SELECTED_TRACK_ID;
   cmd.ltp.track_id = track_id;
   return cmd;
}

This::Command This::make_update_selected_pose(const PoseAnnotation pose)
{
   Command cmd;
   cmd.op          = Command::UPDATE_SELECTED_POSE;
   cmd.ltp.tp.pose = pose;
   return cmd;
}

This::Command This::make_update_selected_trackpoint_xy(Point2 hist_pos)
{
   Command cmd;
   cmd.op       = Command::UPDATE_SELECTED_TRACKPOINT_XY;
   cmd.ltp.tp.x = float(hist_pos.x);
   cmd.ltp.tp.y = float(hist_pos.y);
   return cmd;
}

This::Command This::make_update_selected_trackpoint_theta(const real theta)
{
   Command cmd;
   cmd.op                    = Command::UPDATE_SELECTED_TRACKPOINT_THETA;
   cmd.ltp.tp.gaze_direction = float(theta);
   return cmd;
}

This::Command This::make_update_pos_all_tps(const Point2 dxy)
{
   Command cmd;
   cmd.op       = Command::UPDATE_POS_ALL_TPS;
   cmd.ltp.tp.x = float(dxy.x);
   cmd.ltp.tp.y = float(dxy.y);
   return cmd;
}

This::Command This::make_apply_commands(const vector<Command>& seq)
{
   Command cmd;
   cmd.op = Command::APPLY_COMMANDS;
   cmd.cmds.reserve(seq.size());
   std::transform(cbegin(seq),
                  cend(seq),
                  std::back_inserter(cmd.cmds),
                  [](const auto& cmd) -> unique_ptr<Command> {
                     return make_unique<Command>(cmd);
                  });
   return cmd;
}

// -------------------------------------------------------------- Making and run
//

bool This::select_trackpoint(int new_idx)
{
   return apply_(make_select_trackpoint(new_idx));
}

bool This::create_and_select_trackpoint(Point2 hist_pos, int frame_no)
{
   return apply_(make_create_and_select_trackpoint(hist_pos, frame_no));
}

bool This::create_and_select_ltp(const LabeledTrackPoint& ltp)
{
   return apply_(make_create_and_select_ltp(ltp));
}

bool This::delete_selected_track()
{
   return apply_(make_delete_selected_track());
}

bool This::update_selected_track_id(int track_id)
{
   return apply_(make_update_selected_track_id(track_id));
}

bool This::update_selected_pose(const PoseAnnotation pose)
{
   return apply_(make_update_selected_pose(pose));
}

bool This::update_selected_trackpoint_xy(Point2 new_xy)
{
   return apply_(make_update_selected_trackpoint_xy(new_xy));
}

bool This::update_selected_trackpoint_theta(const real theta)
{
   return apply_(make_update_selected_trackpoint_theta(theta));
}

bool This::update_pos_all_tps(const Point2 dxy)
{
   return apply_(make_update_pos_all_tps(dxy));
}

bool This::apply_commands(const vector<Command>& seq)
{
   return apply_(make_apply_commands(seq));
}

// ------------------------------------------------------------------- to-string
//

Json::Value This::to_json() const noexcept
{
   auto do_vec_vec_cmd = [&](const auto& x) {
      return json_save_t(cbegin(x), cend(x), [&](const auto& vec) {
         return json_save_t(
             cbegin(vec), cend(vec), [](const auto& o) { return o.to_json(); });
      });
   };

   auto x     = Json::Value{Json::objectValue};
   x["state"] = state().to_json();
   x["undos"] = do_vec_vec_cmd(undos_);
   if(state0_.size() > 0) x["state0"] = state0_.to_json();
   if(redos_.size() > 0) x["redos"] = do_vec_vec_cmd(redos_);
   x["hist_rotation"]   = hist_rotation();
   x["hist_zoom_level"] = hist_zoom_level();
   return x;
}

Json::Value This::to_save_json() const noexcept
{
   auto x          = to_json();
   const char* key = "redos";
   if(x.isMember(key)) x.removeMember(key); // Don't save redos
   return x;
}

bool This::load_json(const Json::Value& envelope) noexcept(false)
{
   State state, state0;
   vector<vector<Command>> undos, redos;

   auto load_vec_vec_cmd
       = [](const Json::Value& o, vector<vector<Command>>& undos) {
            if(o.type() != Json::arrayValue)
               throw std::runtime_error(
                   "expected Json array when reading undos/redos");
            undos.resize(o.size());
            for(size_t i = 0; i < undos.size(); ++i) {
               const auto& x = o[int(i)];
               if(x.type() != Json::arrayValue)
                  throw std::runtime_error(
                      "expected Json array of arrays when reading undos/redos");
               undos[i].resize(x.size());
               for(size_t j = 0; j < undos[i].size(); ++j)
                  if(!undos[i][j].load_json(x[int(j)]))
                     throw std::runtime_error("failed to load Command");
            }
         };

   try {
      // Read state
      if(!envelope.isMember("state"))
         throw std::runtime_error("expects key 'state'!");
      state.load_json(envelope["state"]);

      // Read undos
      if(!envelope.isMember("undos"))
         throw std::runtime_error("expects key 'undos'!");
      else
         load_vec_vec_cmd(envelope["undos"], undos);

      if(envelope.isMember("redos")) load_vec_vec_cmd(envelope["redos"], redos);

      // Read state0
      if(envelope.isMember("state0")) state0.load_json(envelope["state0"]);

   } catch(std::exception& e) {
      WARN(format("failed to load 'Json::Value' envelope: {}", e.what()));
      return false;
   }

   // Let's rig up the result, and test the invariant...
   AnnotationData adat;
   adat.state_  = state;
   adat.state0_ = state0;
   adat.undos_  = undos;
   adat.redos_  = redos;

   // This is our provisional result
   const bool invariant_holds = adat.test_invariant();

   if(!invariant_holds) {
      INFO(format("anotation-data invariant does not hold. Undo information "
                  "not laoded"));
      adat.state_  = State{};
      adat.state0_ = state;
      adat.undos_.clear();
      adat.redos_.clear();
   }

   try {
      if(envelope.isMember("hist_rotation"))
         json_load(envelope["hist_rotation"], adat.hist_rotation_);
      if(envelope.isMember("hist_zoom_level"))
         json_load(envelope["hist_zoom_level"], adat.hist_zoom_level_);
   } catch(std::exception& e) {
      LOG_ERR(format("error reading annotations envelope: {}", e.what()));
   }

   *this = adat;
   return true;
}

// ---------------------------------------------------------------------- export
//
string This::export_annotation_data() const noexcept
{
   Json::StyledWriter writer;
   return writer.write(to_save_json());
}

bool This::load_ground_truth(const string_view raw_pipeline_output) noexcept
{
   try {
      PipelineOutput pipeline_output;
      read(pipeline_output, string(raw_pipeline_output));
      const bool ret
          = init_from_pipeline_output(pipeline_output.tracks_to_fowlkes());
      return ret;
   } catch(std::exception& e) {
      WARN(format("failed to read ground-truth data: {}", e.what()));
   }

   return false;
}

bool This::load_annotation_data(const string_view json_data) noexcept
{
   Json::Value dat;

   try {
      dat = parse_json(json_data.data());
   } catch(std::exception& e) {
      WARN(format("parse error reading `annotation-data`: {}", e.what()));
      return false;
   }

   return load_json(dat);
}

bool This::init_from_pipeline_output(const FowlkesResult& fowlkes) noexcept
{
   State state;
   state.dirty          = false;
   state.selected_index = -1;

   int counter = 0;
   for(const auto& tts : fowlkes.tracks) counter += tts.path.size();
   state.ltps.reserve(size_t(counter));

   for(const auto& tts : fowlkes.tracks) {
      for(const auto& tp : tts.path) {
         LabeledTrackPoint ltps;
         ltps.track_id = tts.id;
         ltps.tp       = tp;
         state.ltps.push_back(ltps);
      }
   }

   AnnotationData adat;
   adat.state_  = state;
   adat.state0_ = std::move(state);
   Expects(adat.test_invariant());

   *this = std::move(adat);
   return true;
}

// ------------------------------------------------------------------- filenames
//
string get_ground_truth_fname(const string_view annotation_directory) noexcept
{
   return format("{}/ground-truth.json", annotation_directory);
}

string get_ground_truth_backup_fname(const string_view annotation_dir) noexcept
{
   const auto now = Timestamp::now();
   string ts      = now.to_string();
   for(auto& c : ts)
      if(c == ':') c = '-';
   return format("{}/zz-ground-truth_data_{}.json", annotation_dir, ts);
}

string get_annotation_fname(const string_view annotation_directory) noexcept
{
   return format("{}/annotation-data.json", annotation_directory);
}

string get_annotation_backup_fname(const string_view annotation_dir) noexcept
{
   const auto now = Timestamp::now();
   string ts      = now.to_string();
   for(auto& c : ts)
      if(c == ':') c = '-';
   return format("{}/zz-backup_annotation-data_{}.json", annotation_dir, ts);
}

// ------------------------------------------------------------------- to-string
//
string This::to_string() const noexcept
{
   Json::StyledWriter writer;
   return writer.write(to_json());
}

string str(const AnnotationData& o) noexcept { return o.to_string(); }

} // namespace perceive
