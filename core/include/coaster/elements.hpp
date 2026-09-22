#pragma once
#include "coaster/spline.hpp"
namespace coaster {
Program ascent(const State &, double rise, double horizontalRun, double exitSpeed, const Cancel &cancel = {});
Program wave(const State &, double height, const Cancel &cancel = {});
Program edgeAct(const State &, double headingChange, double negative = -1.2, const Cancel &cancel = {});
Program brake(const State &, double targetSpeed, double seconds);
Program inclinedBoost(const State &, double targetSpeed, const Cancel &cancel = {});
} // namespace coaster
