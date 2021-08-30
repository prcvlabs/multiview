
#pragma once

#include "gui/app-state.hh"
#include "gui/qt-helpers.hpp"
#include "gui/widgets/sensor-selector.hh"

#include <QFormLayout>

namespace perceive
{
SensorSelector* make_sensor_selector() noexcept;

void finish_form_layout(QFormLayout* layout, const int width) noexcept;

} // namespace perceive
