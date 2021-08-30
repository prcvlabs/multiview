
#include "fowlkes-result.hpp"

#include "perceive/io/json-io.hpp"

#include "perceive/cost-functions/tracks/node-sequence.hpp"
#include "perceive/cost-functions/tracks/tracks-exec.hpp"

#define This FowlkesResult

namespace perceive
{
bool This::operator==(const FowlkesResult& o) const noexcept
{
#define TEST(x) (x == o.x)
   return TEST(w) and TEST(h) and TEST(frames) and TEST(tracks);
#undef TEST
}

bool This::operator!=(const FowlkesResult& o) const noexcept
{
   return !(*this == o);
}

// ---------------------------------------------------------------------- bounds

AABB This::bounds() const noexcept
{
   return AABB(top_left.x,
               top_left.y,
               top_left.x + real(w) * hist_sz,
               top_left.y + real(h) * hist_sz);
}

// ---------------------------------------------------------------- memory-usage

size_t This::memory_usage() const noexcept
{
   Expects(tracks.capacity() >= tracks.size());
   return sizeof(This) + (tracks.capacity() - tracks.size()) * sizeof(Track)
          + std::accumulate(cbegin(tracks),
                            cend(tracks),
                            size_t(0),
                            [](size_t sz, const Track& tt) {
                               return sz + tt.memory_usage();
                            });
}

// ------------------------------------------------------------------- to-string

std::string This::to_string() const noexcept { return to_json_string(); }

std::string This::to_json_string() const noexcept
{
   const auto track_s = implode(
       cbegin(tracks), cend(tracks), ",\n", [](const Track& track) -> string {
          return indent(track.to_json_string(), 4);
       });

   return format(R"V0G0N(
{{
   "width":           {},
   "height":          {},
   "frames":          {},
   "start-frame":     {},
   "top-left":       [{}, {}],
   "hist-sz":         {},
   "frame-duration":  {},
   "tracks":         [{}]
}})V0G0N",
                 w,
                 h,
                 frames,
                 start_frame,
                 top_left.x,
                 top_left.y,
                 hist_sz,
                 frame_duration,
                 track_s);
}

Json::Value This::to_json() const noexcept
{
   return parse_json(to_json_string());
}

// ------------------------------------------------------------------ brief info

std::string This::brief_info() const noexcept
{
   const auto tracks_s = implode(
       cbegin(tracks),
       cend(tracks),
       "\n      ",
       [&](const Track& track) -> string { return track.brief_info(); });

   return format(R"V0G0N(
FowlkesResult, summary:
   wh          = [{}x{}], 
   frames      =  {},
   start-frame = {},
   n-tracks    =  {},
      {}
)V0G0N",
                 w,
                 h,
                 frames,
                 start_frame,
                 tracks.size(),
                 tracks_s);
}

// ------------------------------------------------------------------- to-tracks

Tracks This::to_tracks() const noexcept
{
   Tracks tt;
   tt.seqs   = tracks::tracks_to_node_sequence(cbegin(tracks), cend(tracks));
   tt.max_id = -1;
   tt.max_frames_per_track = 0;
   tt.w                    = w;
   tt.h                    = h;
   tt.start_frame          = 0;
   tt.n_frames             = 0;

   for(const auto& o : tracks) {
      if(o.id > tt.max_id) tt.max_id = o.id;
      if(int(o.path.size()) > tt.max_frames_per_track)
         tt.max_frames_per_track = int(o.path.size());
      if(o.path.size() > 0 && o.path.back().t >= int(tt.n_frames))
         tt.n_frames = o.path.back().t + 1;
   }

   return tt;
}

// ------------------------------------------------------------------ read/write

static void json_load(const Json::Value& o, TrackPoint& x) { read(x, o); }

void read(FowlkesResult& data, const Json::Value& node) noexcept(false)
{
   json_load(get_key(node, "width"), data.w);
   json_load(get_key(node, "height"), data.h);
   json_load(get_key(node, "frames"), data.frames);

   if(has_key(node, "start-frame"))
      json_load(get_key(node, "start-frame"), data.start_frame);

   // The tracks
   json_load_t<Track>(
       get_key(node, "tracks"),
       data.tracks,
       [&](const Json::Value& node, Track& track) { read(track, node); });

   const auto op = "loading fowlkes result"s;
   json_try_load_key(data.top_left, node, "top-left", op, true);
   json_try_load_key(data.hist_sz, node, "hist-sz", op, true);
   json_try_load_key(data.frame_duration, node, "frame-duration", op, true);
}

void read(FowlkesResult& data, string_view s) noexcept(false)
{
   Json::Value root = parse_json(string(begin(s), end(s)));
   read(data, root);
}

void write(const FowlkesResult& data, Json::Value& node) noexcept(false)
{
   node = data.to_json();
}

void write(const FowlkesResult& data, string& s) noexcept(false)
{
   Json::StyledWriter writer;
   Json::Value root;
   write(data, root);
   s = writer.write(root);
}

void save(const FowlkesResult& data, string_view fname) noexcept(false)
{
   Json::StyledWriter writer;
   Json::Value root;
   write(data, root);
   file_put_contents(fname, writer.write(root));
}

void load(FowlkesResult& data, string_view fname) noexcept(false)
{
   string raw       = file_get_contents(fname);
   Json::Value root = parse_json(raw);
   read(data, root);
}

// --------------------------------------------------------- make-fowlkes-result
//
FowlkesResult make_fowlkes_result(
    const int start_frame,
    const int n_frames,
    std::function<const Tracks*(const int frame_no)> get_track_ptr,
    const Vector2& top_left,
    const real hist_sz,
    const real frame_duration) noexcept(false)
{
   Expects(start_frame >= 0);
   Expects(n_frames > 0);

   FowlkesResult ret;

   ret.start_frame    = start_frame;
   ret.top_left       = top_left;
   ret.hist_sz        = hist_sz;
   ret.frame_duration = frame_duration;

   vector<const Tracks*> tracks;
   tracks.reserve(size_t(n_frames));
   for(auto t = start_frame; t < start_frame + n_frames; ++t)
      tracks.push_back(get_track_ptr(t));

   Expects(std::all_of(
       cbegin(tracks), cend(tracks), [](auto p) { return p != nullptr; }));
   Expects(tracks.size() > 0);

   if(false) {
      int counter = 0;
      for(const auto track : tracks)
         TRACE(
             format(" * track[{}].size() = {}", counter++, track->seqs.size()));
   }

   // Set width/height/frames
   const Tracks* tracks0 = tracks[0];
   ret.w                 = tracks0->w;
   ret.h                 = tracks0->h;
   ret.frames            = n_frames;

   // What is the maximum id of any particular track?
   int max_id = -1; // i.e., there's no track
   for(auto& ptr : tracks)
      for(const auto& tt : ptr->seqs)
         if(tt.id() > max_id) max_id = tt.id();

   // Reserve space in 'ret'
   ret.tracks.resize(size_t(max_id + 1));
   auto push_track = [&](const Track& track) {
      // INFO(format(" merge track {}, {} elements", track.id,
      // track.path.size()));
      Expects(track.id >= 0);
      Expects(track.id <= max_id);
      auto& dst = ret.tracks[size_t(track.id)];
      if(dst.id < 0) {
         dst = track;
      } else {
         Expects(dst.id == track.id);
         dst.path.insert(end(dst.path), cbegin(track.path), cend(track.path));
         dst.cost   = 0.5f * (dst.cost + track.cost);
         dst.height = 0.5f * (dst.height + track.height);
      }
   };

   // Copy in the actual tracks
   const Tracks* last_ptr = nullptr;
   for(const auto ptr : tracks) {
      if(ptr == last_ptr) continue; // Prevents duplication
      const auto tts
          = tracks::node_sequence_to_tracks(cbegin(ptr->seqs), cend(ptr->seqs));
      for(const auto& tt : tts) push_track(tt);
      last_ptr = ptr;
   }

   // Remove completely empty tracks
   auto ii = std::stable_partition(
       begin(ret.tracks), end(ret.tracks), [&](const Track& x) -> bool {
          return x.size() > 0;
       });
   ret.tracks.erase(ii, end(ret.tracks));

   return ret;
}

} // namespace perceive
