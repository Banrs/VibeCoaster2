#pragma once
#include "coaster/elements.hpp"
namespace coaster {
struct JourneyParts {
    std::vector<Program> active;
};
struct JourneyHint {
    std::vector<double> cuts;
    std::array<double, 2> headings{};
    int winding{};
    double middleOffset{};
};
struct TerminalShape {
    double shoulderHeight{7.2}, deceleration{2}, flattenDistance{30};
};
struct BrakingParts {
    Program entry, stop;
};
BrakingParts finishBrakes(const State &, double seconds, const Cancel &cancel = {},
                          double finalEntrySpeed = -1, const TerminalShape &shape = {});
struct BrakePlan {
    double horizontalLength{}, entryHeight{}, entryPitch{}, entrySeconds{}, finalSeconds{};
};
JourneyParts closeJourney(const State &, Vec3 target, Vec3 exitForward, double seconds,
                          const Cancel &cancel = {},
                          const std::function<bool(const Program &)> &routeFilter = {},
                          JourneyHint *hint = nullptr);
Program terminalEntry(const State &, const Cancel &cancel = {}, const TerminalShape &shape = {});
BrakePlan planBrakes(double entrySpeed, double seconds, const Cancel &cancel = {},
                     double finalEntrySpeed = -1, const TerminalShape &shape = {});
double coastingExitSpeed(double speed, double seconds, double rolling, double drag);
} // namespace coaster
