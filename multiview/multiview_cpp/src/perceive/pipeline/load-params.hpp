
#pragma once

#include "perceive/cost-functions/localization/localization-data.hpp"
#include "perceive/io/struct-meta.hpp"

namespace perceive
{
LocalizationData::Params
read_localization_params(const Json::Value& pipeline_params) noexcept(false);
}
