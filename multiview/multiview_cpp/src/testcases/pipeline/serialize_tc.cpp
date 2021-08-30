
#include <algorithm>
#include <iterator>

#define CATCH_CONFIG_PREFIX_ALL
#include "perceive/contrib/catch.hpp"

#include "perceive/scene/scene-description.hpp"
#include "perceive/pipeline/frame-results.hpp"
#include "perceive/pipeline/nodes/nodes.hpp"

namespace perceive::pipeline
{
static std::string json_to_string(Json::Value o)
{
   Json::StreamWriterBuilder builder;
   return Json::writeString(builder, o);
}

CATCH_TEST_CASE("PipelineSerialize", "[pipeline_serialize]") {}

} // namespace perceive::pipeline
