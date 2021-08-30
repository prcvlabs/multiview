
#include "stdinc.hpp"
#include "sprite-main-inc.hpp"
#include "perceive/utils/file-system.hpp"
#include "perceive/graphics/sprite.hpp"

#ifndef USING_OPENGL

namespace perceive::sprite_main
{
int run_main(int argc, char** argv)
{
   WARN(format("opengl hasn't been compiled into this build"));
   return EXIT_SUCCESS;
}

} // namespace perceive::sprite_main

#else

#ifdef USE_EGL

namespace perceive::sprite_main
{
int run_main(int argc, char** argv)
{
   WARN(format("glut hasn't been compiled into this build"));
   return EXIT_SUCCESS;
}

} // namespace perceive::sprite_main

#else

#include <GL/glut.h>

static const perceive::Sprite* sprite_ptr = nullptr;

// --------------------------------------------------------------------- Reshape

static void reshape(int width, int height)
{
   const auto aspect_ratio = double(width) / height;
   const auto znear = 1.0, zfar = 1000.0, fov = 45.0;

   glMatrixMode(GL_PROJECTION);
   glLoadIdentity();
   gluPerspective(fov, aspect_ratio, znear, zfar);
   glViewport(0, 0, width, height);
}

// ---------------------------------------------------------------- Apply Motion

static void update_rotation(float& angle)
{
   static GLint prev_time     = 0;
   static GLint prev_fps_time = 0;
   static int frames          = 0;

   int time = glutGet(GLUT_ELAPSED_TIME);
   angle += (time - prev_time) * 0.01;
   prev_time = time;

   frames += 1;
   if((time - prev_fps_time) > 1000) {
      int current_fps = frames * 1000 / (time - prev_fps_time);
      // printf("%d fps\n", current_fps);
      frames        = 0;
      prev_fps_time = time;
   }

   glutPostRedisplay();
}

// --------------------------------------------------------------------- Display

static void display()
{
   static float angle = 0.f;

   glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

   glMatrixMode(GL_MODELVIEW);
   glLoadIdentity();
   gluLookAt(0.f, 0.f,
             3.f, // eye
             0.f, 0.f,
             -5.f, // center
             0.f, 1.f,
             0.f); // up

   // rotate it around the y axis
   glRotatef(angle, 0.f, 1.f, 0.f);

   if(sprite_ptr != nullptr) {
      const auto& sprite = *sprite_ptr;
      // scale the whole asset to fit into our view frustum
      auto range     = sprite.max_xyz() - sprite.min_xyz();
      auto inv_scale = std::max(range.x, std::max(range.y, range.z));
      auto scale     = 1.0 / inv_scale;
      glScaled(scale, scale, scale);

      // center the model
      glTranslated(-sprite.center().x, -sprite.center().y, -sprite.center().z);

      { // Draw the XYZ axis
         glDisable(GL_LIGHTING);
         glBegin(GL_LINES);
         glColor3d(1.0, 0.0, 0.0);
         glVertex3d(0.0, 0.0, 0.0);
         glVertex3d(2.0, 0.0, 0.0);
         glColor3d(0.0, 1.0, 0.0);
         glVertex3d(0.0, 0.0, 0.0);
         glVertex3d(0.0, 2.0, 0.0);
         glColor3d(0.0, 1.0, 1.0);
         glVertex3d(0.0, 0.0, 0.0);
         glVertex3d(0.0, 0.0, 2.0);
         glEnd();
      }

      // Call render on the sprite
      render_gl(sprite);
   }

   glutSwapBuffers();

   update_rotation(angle);
}

namespace perceive::sprite_main
{
// ------------------------------------------------------------------- Show Help

static void show_help(string arg0)
{
   cout << format(R"V0G0N(

   Usage: {:s} <filename>

      Uses assimp to load <filename>, and displays it.

)V0G0N",
                  basename(arg0));
}

// -------------------------------------------------------------------- Run Main

int run_main(int argc, char** argv)
{
   cout << format("Welcome to 'sprite-main'") << endl;

   // ---- Parse the command line
   if(argc != 2) {
      show_help(argv[0]);
      return EXIT_FAILURE;
   }

   auto filename = string(argv[1]);
   if(filename == "-h" || filename == "--help") {
      show_help(argv[0]);
      return EXIT_SUCCESS;
   }

   if(!is_regular_file(filename)) {
      cout << format("Failed to locate file: '{:s}'", filename) << endl;
      return EXIT_FAILURE;
   }

   // ---- Load the sprite
   // get a handle to the predefined STDOUT log stream and attach
   // it to the logging system. It remains active for all further
   // calls to aiImportFile(Ex) and aiApplyPostProcessing.
   Sprite sprite;
   load(sprite, filename);
   sprite_ptr = &sprite;

   // ---- Set up GLUT

   glutInitWindowSize(900, 600);
   glutInitWindowPosition(100, 100);
   glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH);
   glutInit(&argc, argv);

   glutCreateWindow("Assimp - Very simple OpenGL sample");
   glutReshapeFunc(reshape);
   glutDisplayFunc(display);

   glClearColor(0.1f, 0.1f, 0.1f, 1.f);

   glEnable(GL_LIGHTING);
   glEnable(GL_LIGHT0); /* Uses default lighting parameters */

   glEnable(GL_DEPTH_TEST);

   glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_TRUE);
   glEnable(GL_NORMALIZE);

   glColorMaterial(GL_FRONT_AND_BACK, GL_DIFFUSE);

   glutGet(GLUT_ELAPSED_TIME);
   glutMainLoop();

   bool success = true;
   return success ? EXIT_SUCCESS : EXIT_FAILURE;
}

} // namespace perceive::sprite_main

#endif // EGL
#endif // USING_OPENGL
