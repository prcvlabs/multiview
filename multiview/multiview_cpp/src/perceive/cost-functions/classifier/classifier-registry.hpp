
#pragma once

#include "classifier.hpp"

namespace perceive
{
// Threadsafe
const Classifier* get_classifier(string_view key) noexcept;
} // namespace perceive
