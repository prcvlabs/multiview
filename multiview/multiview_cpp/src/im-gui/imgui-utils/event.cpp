
#include "event.hpp"

#include "perceive/io/json-io.hpp"

namespace perceive ::gui
{
// -------------------------------------------------------------------- KeyEvent

KeyEvent::KeyEvent(const SDL_KeyboardEvent& x) noexcept
    : type(x.type)
    , timestamp(x.timestamp)
    , window_id(x.windowID)
    , scancode(x.keysym.scancode)
    , keycode(x.keysym.sym)
    , modifiers(x.keysym.mod)
    , state(x.state)
    , repeat(x.repeat != 0)
{}

Json::Value KeyEvent::to_json() const noexcept
{
   auto o = Json::Value{Json::objectValue};

#define WRITE(x) o[#x] = json_save(x);
   WRITE(type);
   WRITE(timestamp);
   WRITE(window_id);
   WRITE(scancode);
   WRITE(keycode);
   WRITE(modifiers);
   WRITE(state);
   WRITE(repeat);
#undef WRITE

   return o;
}

KeyEvent KeyEvent::from_json(const Json::Value&) noexcept(false)
{
   FATAL("TODO");
   return {};
}

string KeyEvent::to_string() const noexcept
{
   static thread_local Json::StyledWriter writer;
   return writer.write(to_json());
}

string str(const KeyEvent& o) noexcept { return o.to_string(); }

SDL_Event to_sdl_event(const KeyEvent& o) noexcept
{
   SDL_Event ev;
   ev.type                = o.type;
   ev.key.type            = o.type;
   ev.key.timestamp       = o.timestamp;
   ev.key.windowID        = o.window_id;
   ev.key.keysym.scancode = SDL_Scancode(o.scancode);
   ev.key.keysym.sym      = o.keycode;
   ev.key.keysym.mod      = o.modifiers;
   ev.key.state           = o.state;
   ev.key.repeat          = uint8_t(o.repeat);
   return ev;
}

// ---------------------------------------------- update-window-state-from-event
//
void update_window_state_from_event(SDL_Window* window,
                                    const Event& event) noexcept
{}
// {
//    std::visit(
//        overloaded{[](const TimeElapsed& o) {
//                      try {
//                         std::this_thread::sleep_for(o.elapsed);
//                      } catch(std::exception& e) {
//                         WARN(
//                             format("exception processing sleep: {}",
//                             e.what()));
//                      }
//                   },
//                   [window](const Moved<Mouse>& o) {
//                      SDL_WarpMouseInWindow(window, o.source.x, o.source.y);
//                   },
//                   [](const auto&) {}},
//        event);
// }

// ------------------------------------------------------------------ make-event
//
Event make_event(const SDL_Event& ev) noexcept
{
   switch(ev.type) {
   case SDL_AUDIODEVICEADDED:
   case SDL_AUDIODEVICEREMOVED:
      // SDL_AudioDeviceEvent
      return std::monostate{};

   case SDL_CONTROLLERAXISMOTION:
      // SDL_ControllerAxisEvent
      return std::monostate{};

   case SDL_CONTROLLERBUTTONDOWN:
   case SDL_CONTROLLERBUTTONUP:
      // SDL_ControllerButtonEvent
      return std::monostate{};

   case SDL_CONTROLLERDEVICEADDED:
   case SDL_CONTROLLERDEVICEREMOVED:
   case SDL_CONTROLLERDEVICEREMAPPED:
      // SDL_ControllerDeviceEvent
      return std::monostate{};

   case SDL_DOLLARGESTURE:
   case SDL_DOLLARRECORD:
      // SDL_DollarGestureEvent
      return std::monostate{};

   case SDL_DROPFILE:
   case SDL_DROPTEXT:
   case SDL_DROPBEGIN:
   case SDL_DROPCOMPLETE:
      // SDL_DropEvent
      return std::monostate{};

   case SDL_FINGERMOTION:
   case SDL_FINGERDOWN:
   case SDL_FINGERUP:
      // SDL_TouchFingerEvent
      return std::monostate{};

   case SDL_KEYDOWN:
   case SDL_KEYUP:
      // SDL_KeyboardEvent
      return KeyEvent(ev.key);

   case SDL_JOYAXISMOTION:
      // SDL_JoyAxisEvent
      return std::monostate{};

   case SDL_JOYBALLMOTION:
      // SDL_JoyBallEvent
      return std::monostate{};

   case SDL_JOYHATMOTION:
      // SDL_JoyHatEvent
      return std::monostate{};

   case SDL_JOYBUTTONDOWN:
   case SDL_JOYBUTTONUP:
      // SDL_JoyButtonEvent
      return std::monostate{};

   case SDL_JOYDEVICEADDED:
   case SDL_JOYDEVICEREMOVED:
      // SDL_JoyDeviceEvent
      return std::monostate{};

   case SDL_MOUSEMOTION:
      // SDL_MouseMotionEvent
      return std::monostate{};

   case SDL_MOUSEBUTTONDOWN:
   case SDL_MOUSEBUTTONUP:
      // SDL_MouseButtonEvent
      return std::monostate{};

   case SDL_MOUSEWHEEL:
      // SDL_MouseWheelEvent
      return std::monostate{};

   case SDL_MULTIGESTURE:
      // SDL_MultiGestureEvent
      return std::monostate{};

   case SDL_QUIT:
      // SDL_QuitEvent
      return std::monostate{};

   case SDL_SYSWMEVENT:
      // SDL_SysWMEvent
      return std::monostate{};

   case SDL_TEXTEDITING:
      // SDL_TextEditingEvent
      return std::monostate{};

   case SDL_TEXTINPUT:
      // SDL_TextInputEvent
      return std::monostate{};

   case SDL_USEREVENT:
      // SDL_UserEvent
      return std::monostate{};

   case SDL_WINDOWEVENT:
      // SDL_WindowEvent
      return std::monostate{};

   default: return std::monostate{};
   }
}

} // namespace perceive::gui
