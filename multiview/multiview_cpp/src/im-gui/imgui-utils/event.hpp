
#pragma once

#include <array>
#include <chrono>
#include <string>
#include <thread>
#include <variant>

#include "SDL.h"

namespace perceive::gui
{
using clock_type = std::chrono::steady_clock;

// template<typename Source> struct Pressed
// {
//    constexpr static std::string_view name{"Pressed"};
//    constexpr static std::array elements{std::string_view{"source"}};
//    Source source;
// };

// template<typename Source> struct Released
// {
//    constexpr static std::string_view name{"Released"};
//    constexpr static std::array elements{std::string_view{"source"}};
//    Source source;
// };

// template<typename Source> struct Moved
// {
//    constexpr static std::string_view name{"Moved"};
//    constexpr static std::array elements{std::string_view{"source"}};
//    Source source;
// };

// struct Mouse
// {
//    constexpr static std::string_view name{"Mouse"};
//    constexpr static auto elements
//        = to_array<std::string_view>({"window_id", "x", "y"});

//    uint32_t window_id = 0; // window associated with key event
//    int x              = 0;
//    int y              = 0;
// };

// struct MouseButton
// {
//    constexpr static std::string_view name{"MouseButton"};
//    constexpr static auto elements
//        = to_array<std::string_view>({"button", "clicks", "mouse"});

//    int button  = 0; // which button was clicked
//    int clicks  = 0; // 1 for single-click, 2 for double-click
//    Mouse mouse = {};
// };

struct KeyEvent
{
   constexpr static std::string_view name{"KeyEvent"};

   uint32_t type      = 0;     // SDL_KEYDOWN or SDL_KEYUP
   uint32_t timestamp = 0;     // SDL event timestamp
   uint32_t window_id = 0;     // window associated with key event
   int32_t scancode   = 0;     // SDL_Scancode
   int32_t keycode    = 0;     // SDL_Keycode
   uint16_t modifiers = 0;     // SDL_Keymod
   uint8_t state      = 0;     // SDL_PRESSED or SDL_RELEASED
   bool repeat        = false; // results of a key repeat

   KeyEvent() = default;
   KeyEvent(const SDL_KeyboardEvent&) noexcept;
   KeyEvent(const KeyEvent&) = default;
   KeyEvent(KeyEvent&&)      = default;
   ~KeyEvent()               = default;
   KeyEvent& operator=(const KeyEvent&) = default;
   KeyEvent& operator=(KeyEvent&&) = default;

   Json::Value to_json() const noexcept;
   static KeyEvent from_json(const Json::Value&) noexcept(false);
   string to_string() const noexcept;
   friend string str(const KeyEvent&) noexcept;
   friend SDL_Event to_sdl_event(const KeyEvent&) noexcept;
};

// struct CloseWindow
// {
//    constexpr static std::string_view name{"CloseWindow"};
//    constexpr static std::array<std::string_view, 0> elements{};
// };

// struct TimeElapsed
// {
//    constexpr static std::string_view name{"TimeElapsed"};
//    constexpr static auto elements = to_array<std::string_view>({"elapsed"});
//    clock_type::duration elapsed;

//    [[nodiscard]] int64_t micros() const
//    {
//       return duration_cast<std::chrono::microseconds>(elapsed).count();
//    }
// };

using Event = std::variant<std::monostate, KeyEvent>;

/// When we pull an Event object (from some object store), we may need
/// to update the underlying SDL gui state. For example, if we're pulling
/// mouse event, we may need to update the position of the mouse.
void update_window_state_from_event(SDL_Window* window,
                                    const Event& event) noexcept;

/// The `Event` object will be serializable, which will enable
/// GUI testing. (yay!)
Event make_event(const SDL_Event& ev) noexcept;

} // namespace perceive::gui
