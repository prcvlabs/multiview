
#include "gl-fragment-buffer.hpp"

#ifdef USING_OPENGL
#include <GL/gl.h>
#endif

#define This GlFragmentBuffer

namespace perceive
{
GlFragmentBuffer& This::instance()
{
   static GlFragmentBuffer instance_;
   return instance_;
}

vector<GlFragment> This::get_and_flush()
{
   vector<GlFragment> ret;
   {
      using std::swap;
      lock_guard lock(padlock_);
      swap(ret, frags_);
   }
   return ret;
}

#ifndef USING_OPENGL
void This::sort_render_and_flush() { Expects(false); }
#else
void This::sort_render_and_flush()
{
   auto frags = get_and_flush();
   std::sort(begin(frags), end(frags));

   glEnable(GL_BLEND);
   glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
   // glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_TRUE);
   // glCullFace(GL_FRONT_AND_BACK);
   glDisable(GL_CULL_FACE);
   glBegin(GL_TRIANGLES);

   bool last_was_line = false; // ie. start with GL_TRIANGLES
   Vector4f last_k    = Vector4f::nan();

   for(const auto& f : frags) {
      if(f.kolour != last_k) {
         last_k = f.kolour;
         glColor4fv(f.kolour.ptr());
      }

      if(f.is_line) {
         if(last_was_line == false) {
            glEnd();
            glBegin(GL_LINES);
            last_was_line = true;
         }
         glVertex3fv(f.triangle[0].ptr());
         glVertex3fv(f.triangle[1].ptr());
      } else {
         if(last_was_line == true) {
            glEnd();
            glBegin(GL_TRIANGLES);
            last_was_line = false;
         }
         glVertex3fv(f.triangle[0].ptr());
         glVertex3fv(f.triangle[1].ptr());
         glVertex3fv(f.triangle[2].ptr());
      }
   }
   glEnd();
   glEnable(GL_CULL_FACE);
}
#endif

} // namespace perceive
