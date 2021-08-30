
#include "annotation-app.hpp"

#include "gl/frame-texture.hpp"
#include "imgui-utils/event.hpp"
#include "imgui-utils/imgui-helpers.hpp"
#include "imgui-utils/single-window-sdl-app.hpp"
#include "perceive/graphics/image-container.hpp"
#include "perceive/graphics/movie-reader.hpp"
#include "perceive/io/lazy-s3.hpp"
#include "perceive/scene/pose-annotation.hpp"
#include "perceive/utils/file-system.hpp"

#include <SDL.h>
#include <imgui.h>
#include <imgui_impl_opengl3.h>
#include <imgui_impl_sdl.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <GL/glew.h>

#define This AnnotationApp

namespace perceive::gui
{
// --------------------------------------------------------------------- run app

int This::run_app(int argc, char** argv) noexcept
{
   const auto config = parse_command_line(argc, argv);
   if(config.has_error) {
      LOG_ERR(format("Aborting due to previous errors."));
      return EXIT_FAILURE;
   } else if(config.show_help) {
      show_help(argv[0]);
      return EXIT_SUCCESS;
   }

   if(config.print_files) {
      cout << str(config.manifest) << endl;
      return EXIT_SUCCESS;
   }

   AnnotationApp app(config);
   return app.exec();
}

// ---------------------------------------------------------------- load-texture

static FrameTexture load_texture(string_view fname) noexcept
{
   if(fname.empty()) return {};

   const auto icon_dir = format("{}/assets/icons", multiview_asset_s3_bucket());
   const auto full_fname = format("{}/{}", icon_dir, fname);

   try {
      vector<char> data;
      lazy_load(full_fname, data);
      cv::Mat im = decode_image_to_cv_mat(data);
      FrameTexture tex;
      tex.update(im);
      return tex;
   } catch(std::exception& e) {
      LOG_ERR(format("failed to load icon '{}'", full_fname));
   }
   return {};
}

// ------------------------------------------------------ pack/unpack annotation

static vector<string> unpack_annotation(string_view annotation) noexcept
{
   auto parts = explode(annotation, ",");
   for(auto& s : parts) trim(s);
   return parts;
}

static string pack_annotation(const vector<string>& parts)
{
   return rng::implode(parts, ", ");
}

// ----------------------------------------------------------------------- Pimpl
// None, Stand, Walk, sIt, Lay, Phone, Other
//
struct This::Pimpl
{
   AnnotationApp& parent;
   Config config;
   unique_ptr<SingleWindowApp> window;

   // Our state
   ImVec4 clear_color = ImVec4{0.0f, 0.0f, 0.0f, 1.00f};

   // Main loop
   bool is_done = false;

   FrameTexture current_tex   = {};
   MovieReader movie          = {};
   vector<string> annotations = {};

   array<FrameTexture, pose_icon_fnames.size()> icons = {};

   // ------------------------------------------------------------- Construction

   Pimpl(AnnotationApp& parent_, Config&& config_)
       : parent(parent_)
       , config(config_)
       , window(nullptr)
   {
      SingleWindowParams window_params;
      window_params.window_name = "multiview annotation-gui";

      INFO("Making Single Window App");
      window = make_unique<SingleWindowApp>(window_params);

      // Load the icons
      INFO("Loading icons");
      std::transform(cbegin(pose_icon_fnames),
                     cend(pose_icon_fnames),
                     begin(icons),
                     load_texture);

      // Open the movie file
      INFO(format("Loading movie file '{}'", config.manifest.mp4_fname));
      if(!movie.open(config.manifest.mp4_fname))
         FATAL(format("failed to open movie file: '{}'",
                      config.manifest.mp4_fname));

      // Open the annotations file
      INFO(
          format("Loading annotations '{}'", config.manifest.annotation_fname));
      annotations = explode(
          file_get_contents(config.manifest.annotation_fname), "\n", true);

      // Now make sure that our text files line up with the movie file
      if(annotations.size() != size_t(movie.n_frames())) {
         FATAL(format("movie file has {} frames, but there are {} annotations!",
                      movie.n_frames(),
                      annotations.size()));
      }

      go_to_frame(0);
   }

   // --------------------------------------------------------------- Frame Text

   string frame_no_text() const noexcept
   {
      return format("Frame {:04d}/{:04d}", movie.frame_no(), movie.n_frames());
   }

   string current_annotation() const noexcept
   {
      const size_t ind = size_t(movie.frame_no());
      if(ind < annotations.size()) return annotations[ind];
      return ""s;
   }

   bool update_current_annotation(string value) noexcept
   {
      const size_t ind = size_t(movie.frame_no());
      if(ind < annotations.size()) {
         annotations[ind] = value;
         return true;
      }
      return false;
   }

   // -------------------------------------------------------------- Go to Frame

   void go_to_frame(int frame_no) noexcept
   {
      if(frame_no < 0) frame_no = 0;
      if(frame_no >= movie.n_frames()) frame_no = movie.n_frames() - 1;
      if(frame_no < 0) return; // there was no frame!

      movie.seek(frame_no);
      Expects(movie.frame_no() == frame_no);

      current_tex.update(movie.current_frame());
   }

   void prev_frame() noexcept
   {
      if(movie.frame_no() <= 0)
         go_to_frame(movie.n_frames() - 1);
      else
         go_to_frame(movie.frame_no() - 1);
   }

   void next_frame() noexcept
   {
      if(movie.frame_no() + 1 >= movie.n_frames())
         go_to_frame(0);
      else
         go_to_frame(movie.frame_no() + 1);
   }

   // --------------------------------------------------------- Save Annotations

   bool save_annotations() noexcept
   {
      const auto fname        = config.manifest.annotation_fname;
      const auto backup_fname = format("{}.bak", fname);
      if(!is_regular_file(backup_fname)) {
         INFO(format("backing up '{}'", fname));
         const auto raw = file_get_contents(fname);
         file_put_contents(backup_fname, raw);
      }

      const auto data = rng::implode(annotations, "\n");
      file_put_contents(fname, data);
      return true;
   }

   // -------------------------------------------------------------- Update Pose

   PoseAnnotation current_pose() noexcept
   {
      const auto fields = unpack_annotation(current_annotation());
      if(fields.size() > 1) {
         const int pose_int  = int(atof(fields.at(1).c_str()));
         PoseAnnotation pose = to_pose_annotation(pose_int);
         if(pose_int != int(pose))
            WARN(format("invalid pose data: '{}'", fields.at(1)));
         return pose;
      } else {
         WARN(format("failed to unpack: '{}'", current_annotation()));
      }
      return PoseAnnotation::NONE;
   }

   void update_current_pose(PoseAnnotation pose) noexcept
   {
      auto fields = unpack_annotation(current_annotation());
      if(fields.size() > 1) {
         fields[1]       = format("{}", float(int(pose)));
         string new_line = rng::implode(fields, ", ");
         update_current_annotation(new_line);
         save_annotations();
      } else {
         WARN(format("failed to unpack: '{}'", current_annotation()));
      }
   }

   // ------------------------------------------------------------ Process Event

   void process_event(const SDL_Event& event) noexcept
   {
      switch(event.type) {
      case SDL_QUIT: is_done = true; break;
      case SDL_WINDOWEVENT:
         if(event.window.event == SDL_WINDOWEVENT_CLOSE
            && event.window.windowID == window->id())
            is_done = true;
         break;
      case SDL_KEYDOWN:
      case SDL_KEYUP: on_key_event(KeyEvent(event.key)); break;
      default:
         // Unhandled event
         break;
      }
   }

   // ------------------------------------------------------------- on-key-event

   void on_key_event(const KeyEvent& ev) noexcept
   {
      if(ev.type == SDL_KEYUP || ev.repeat == true) {
         switch(ev.keycode) {
         case SDLK_RIGHT: next_frame(); break;
         case SDLK_LEFT: prev_frame(); break;
         case SDLK_n: return update_current_pose(PoseAnnotation::NONE);
         case SDLK_s: return update_current_pose(PoseAnnotation::STAND);
         case SDLK_w: return update_current_pose(PoseAnnotation::WALK);
         case SDLK_i: return update_current_pose(PoseAnnotation::SIT);
         case SDLK_l: return update_current_pose(PoseAnnotation::LAY);
         case SDLK_p: return update_current_pose(PoseAnnotation::PHONE);
         case SDLK_o: return update_current_pose(PoseAnnotation::OTHER);
         default:
            break;
            // None, Stand, Walk, sIt, Lay, Phone, Other
         }
      }
   }

   // -------------------------------------------------------------- Update Loop

   void update_loop() noexcept
   {
      SDL_Event event;
      while(SDL_PollEvent(&event)) {
         ImGui_ImplSDL2_ProcessEvent(&event);
         process_event(event);
      }

      ImGuiIO& io = ImGui::GetIO();

      // Get the window w/h
      const auto [w, h] = window->size();

      // Start the Dear ImGui frame
      ImGui_ImplOpenGL3_NewFrame();
      ImGui_ImplSDL2_NewFrame(window->sdl_window);
      ImGui::NewFrame();

      { // Create the user interface
         ImGui::SetNextWindowPos(ImVec2(.0f, .0f), ImGuiCond_Always);
         ImGui::SetNextWindowSize(ImVec2(float(w + 2), float(h + 2)),
                                  ImGuiCond_Always);
         ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0f);
         ImGui::PushStyleVar(ImGuiStyleVar_FrameRounding, 0.0f);
         ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0.0f);

         make_ui(w, h);

         ImGui::PopStyleVar();
         ImGui::PopStyleVar();
         ImGui::PopStyleVar();
      }

      // Rendering
      ImGui::Render();
      glViewport(0, 0, int(io.DisplaySize.x), int(io.DisplaySize.y));
      glClearColor(clear_color.x, clear_color.y, clear_color.z, clear_color.w);
      glClear(GL_COLOR_BUFFER_BIT);
      ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
      window->gl_swap_window();
   }

   // ------------------------------------------------------------------ make-ui

   void make_ui(const int w, const int h)
   {
      static float f     = 0.0f;
      static int counter = 0;
      static auto im_win_flags
          = ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove
            | ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoScrollbar
            | ImGuiWindowFlags_NoScrollWithMouse | ImGuiWindowFlags_NoTitleBar
            | ImGuiWindowFlags_NoSavedSettings;

      ImGui::Begin("Hello, world!", nullptr, im_win_flags);

      const auto pose = current_pose();
      auto get_field0 = [this]() {
         const auto fields = unpack_annotation(current_annotation());
         if(fields.size() > 0) return fields.front();
         return "<no annotation>"s;
      };
      imgui::Text("{} :: {} :: {}", frame_no_text(), get_field0(), str(pose));

      ImGui::Image(imgui::convert_texture(current_tex.tex_id()),
                   imgui::to_im_vec2(current_tex.dims()));

      ImGui::Text("Application average %.3f ms/frame (%.1f FPS)",
                  double(1000.0f / ImGui::GetIO().Framerate),
                  double(ImGui::GetIO().Framerate));

      {
         const float x_pos = 920.0;
         ImGui::SetCursorPos(ImVec2(x_pos, 35.0));
         // imgui::Text("{}", str(pose));
         if(pose != PoseAnnotation::NONE) {
            Expects(size_t(pose) < icons.size());
            const auto& tex = icons.at(size_t(pose));
            ImGui::SetCursorPos(ImVec2(x_pos, ImGui::GetCursorPosY()));
            ImGui::Image(imgui::convert_texture(tex.tex_id()),
                         imgui::to_im_vec2(tex.dims()));
         }
      }
      ImGui::End();
   }
};

// ---------------------------------------------------------------- Construction

This::This(Config config) noexcept(false)
    : pimpl_(new Pimpl(*this, std::move(config)))
{}

This::~This() = default;

// -----------------------------------------------------------------------------

int This::exec() noexcept
{
   while(!pimpl_->is_done) pimpl_->update_loop();
   return 0;
}

const Config& This::config() const noexcept { return pimpl_->config; }

} // namespace perceive::gui

#undef This
