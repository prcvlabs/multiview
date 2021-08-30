
#pragma once

#ifdef USING_OPENGL

namespace perceive
{
const char* gl_error_to_cstr(int err) noexcept;
void check_and_throw_on_gl_error(const char* preamble
                                 = nullptr) noexcept(false);
bool check_and_warn_on_gl_error(const char* preamble = nullptr) noexcept;

} // namespace perceive

#endif
