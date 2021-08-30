
#define CATCH_CONFIG_PREFIX_ALL

#include <algorithm>
#include <deque>
#include <iterator>

#include "perceive/contrib/catch.hpp"

#include "perceive/scene/annotations.hpp"

constexpr bool feedback = false;

namespace perceive
{
CATCH_TEST_CASE("AnnotationsData", "[annotations-data]")
{
   // This code should just finish without tripping the memory sanitizer
   CATCH_SECTION("annotations-data")
   { //
      AnnotationData old;
      AnnotationData adat;

      return;

      CATCH_REQUIRE(adat.selected_trackpoint() == nullptr);
      const auto state = adat.state();
      CATCH_REQUIRE(state == adat.state()); // hopefully simple

      vector<AnnotationData> bak;

      auto check_push = [&](const bool ret) {
         {
            Json::Value a1 = adat.to_json();
            AnnotationData b;
            const bool success = b.load_json(a1);
            CATCH_REQUIRE(success);
            CATCH_REQUIRE(adat.test_invariant());
            CATCH_REQUIRE(b.test_invariant());

            if(adat != b) {
               LOG_ERR("serialization violation:");
               cout << "Adat\n" << str(adat) << "\n\n";
               cout << "b\n" << str(b) << "\n\n";
            }
            CATCH_REQUIRE(adat == b);
         }

         if(ret) {
            bak.push_back(adat);
            CATCH_REQUIRE(adat.state() != old.state());
            adat.undo();
            CATCH_REQUIRE(old.state() == adat.state());
            adat.redo();
            CATCH_REQUIRE(bak.back().state() == adat.state());
         }
      };

#define APPLY0(x)                                                           \
   {                                                                        \
      old      = adat;                                                      \
      bool ret = adat.x();                                                  \
      check_push(ret);                                                      \
      if(feedback)                                                          \
         TRACE(format("APPLY {}() = {}\n{}\n\n", #x, str(ret), str(adat))); \
   }
#define APPLY1(x, y)                                                     \
   {                                                                     \
      old      = adat;                                                   \
      bool ret = adat.x((y));                                            \
      check_push(ret);                                                   \
      if(feedback)                                                       \
         TRACE(format(                                                   \
             "APPLY {}({}) = {}\n{}\n\n", #x, #y, str(ret), str(adat))); \
   }
#define APPLY2(x, y0, y1)                                  \
   {                                                       \
      old      = adat;                                     \
      bool ret = adat.x(Point2((y0), (y1)));               \
      check_push(ret);                                     \
      if(feedback)                                         \
         TRACE(format("APPLY {}({{{}, {}}}) = {}\n{}\n\n", \
                      #x,                                  \
                      #y0,                                 \
                      #y1,                                 \
                      str(ret),                            \
                      str(adat)));                         \
   }
#define APPLY3(x, y0, y1, z)                                   \
   {                                                           \
      old      = adat;                                         \
      bool ret = adat.x(Point2((y0), (y1)), (z));              \
      check_push(ret);                                         \
      if(feedback)                                             \
         TRACE(format("APPLY {}({{{}, {}}}, {}) = {}\n{}\n\n", \
                      #x,                                      \
                      #y0,                                     \
                      #y1,                                     \
                      #z,                                      \
                      str(ret),                                \
                      str(adat)));                             \
   }

      if(feedback) TRACE(format("INIT\n{}\n\n", str(adat)));
      APPLY0(delete_selected_track);
      APPLY1(select_trackpoint, 1);
      APPLY3(create_and_select_trackpoint, 1, 0, 2);
      APPLY1(update_selected_track_id, 4);
      APPLY2(update_selected_trackpoint_xy, 10, 9);
      APPLY1(select_trackpoint, 1);
      APPLY1(select_trackpoint, 0);
      APPLY1(select_trackpoint, -1);
      APPLY2(update_selected_trackpoint_xy, 10, 9);
      APPLY1(select_trackpoint, 0);
      APPLY1(update_selected_trackpoint_theta, to_radians(100));
      APPLY1(update_selected_trackpoint_theta, to_radians(90));
      // APPLY2(update_selected_trackpoint_xy, 1, 9);
      APPLY3(create_and_select_trackpoint, 4, 5, 3);
      APPLY1(update_selected_track_id, 1);
      APPLY1(update_selected_trackpoint_theta, to_radians(45));
      APPLY0(delete_selected_track);
      APPLY0(delete_selected_track);
      APPLY1(select_trackpoint, 0);
      APPLY0(delete_selected_track);

#undef APPLY0
#undef APPLY1
#undef APPLY2
#undef APPLY3
   }
}

} // namespace perceive
