
#include <algorithm>
#include <iterator>

#define CATCH_CONFIG_PREFIX_ALL
#include "perceive/contrib/catch.hpp"

#include "perceive/movie/debug-movie.hpp"
#include "perceive/movie/movie-layout.hpp"
#include "perceive/utils/file-system.hpp"

static const bool feedback = false;

namespace perceive
{
using namespace movie;

CATCH_TEST_CASE("DebugMovieDimensions", "[debug-movie-dimensions]")
{
   CATCH_SECTION("debug-movie-dimensions")
   {
      {
         auto ml = make_movie_layout(800,
                                     600, // target width/height
                                     10,
                                     10,
                                     8,
                                     1024,
                                     768, // movie layout
                                     3,
                                     240,
                                     160);
         if(feedback) {
            cout << str(ml) << endl << endl;
            ml.debug_image().save("/tmp/zzz-ml0.png");
         }
         CATCH_REQUIRE(ml.is_init);
      }

      {
         auto ml = make_movie_layout(800,
                                     600, // target width/height
                                     10,
                                     10,
                                     4,
                                     1024,
                                     768, // movie layout
                                     3,
                                     160,
                                     240);
         if(feedback) {
            cout << str(ml) << endl << endl;
            ml.debug_image().save("/tmp/zzz-ml1.png");
         }
         CATCH_REQUIRE(ml.is_init);
      }

      {
         auto ml = make_movie_layout(800,
                                     600, // target width/height
                                     10,
                                     10,
                                     3,
                                     1024,
                                     768, // movie layout
                                     3,
                                     160,
                                     240);
         if(feedback) {
            cout << str(ml) << endl << endl;
            ml.debug_image().save("/tmp/zzz-ml2.png");
         }
         CATCH_REQUIRE(ml.is_init);
      }

      {
         auto ml = make_movie_layout(800,
                                     600, // target width/height
                                     10,
                                     10,
                                     2,
                                     1024,
                                     768, // movie layout
                                     3,
                                     160,
                                     240);
         if(feedback) {
            cout << str(ml) << endl << endl;
            ml.debug_image().save("/tmp/zzz-ml3.png");
         }
         CATCH_REQUIRE(ml.is_init);
      }

      {
         auto ml = make_movie_layout(800,
                                     600, // target width/height
                                     10,
                                     10,
                                     1,
                                     1024,
                                     768, // movie layout
                                     3,
                                     160,
                                     240);
         if(feedback) {
            cout << str(ml) << endl << endl;
            ml.debug_image().save("/tmp/zzz-ml4.png");
         }
         CATCH_REQUIRE(ml.is_init);
      }

      {
         auto ml = make_movie_layout(800,
                                     600, // target width/height
                                     0,
                                     10,
                                     2,
                                     896,
                                     672, // movie layout
                                     3,
                                     70,
                                     40);
         if(feedback) {
            cout << str(ml) << endl << endl;
            ml.debug_image().save("/tmp/zzz-ml5.png");
         }
         CATCH_REQUIRE(ml.is_init);
      }
   }
}
} // namespace perceive
