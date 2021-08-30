
#pragma once

#include "perceive/foundation.hpp"
#include "perceive/geometry/vector.hpp"

namespace perceive
{
void gl_setup_windowless_context();
void glew_init();

void ensure_glew_and_gl_setup();

} // namespace perceive
