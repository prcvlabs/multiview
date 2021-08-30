
#pragma once

#include "gui-render-options.hpp"

#include "perceive/foundation.hpp"
#include "perceive/utils/threads.hpp"

namespace perceive
{
namespace pipeline::localization
{
   struct Result;
}

// ------------------------------------------------------------------- Gl Render

void gui_render_point_cloud(const GuiRenderOptions& opts,
                            const pipeline::localization::Result* loc_ptr,
                            const AABB& bounds);

} // namespace perceive
