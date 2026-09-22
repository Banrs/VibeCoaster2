#pragma once
#include "coaster/motion.hpp"
#include "coaster/terrain.hpp"

namespace coaster {
struct Frame {
    Vec3 p, t, u, r, k, upS, upSS;
    double drive{}, sourceSpeed{}, sourceTime{};
    std::size_t element{};
};
struct Span {
    Vec3 origin;
    std::array<Vec3, 10> p;
    std::array<Vec3, 8> u;
    double begin{}, length{}, time{}, duration{}, speedA{}, speedB{}, driveA{}, driveB{};
    std::size_t element{};
};
struct Track {
    std::vector<Program> source;
    std::vector<Span> spans;
    std::vector<double> elementEnds, elementTimes;
    std::vector<std::array<double, 2>> operationSpeed;
    double length{}, sourceDuration{}, sourceStep{.02};
    bool closed{};
    Frame at(double distance) const;
    Frame at(double distance, std::size_t &hint) const;
};
Track compile(std::vector<Program>, double step = .02, const Cancel &cancel = {});
struct ReplayResult {
    double position{}, forward{}, up{}, speed{}, energy{}, portPosition{}, portTangent{}, portUp{},
        portCurvature{}, portThird{}, portUpThird{};
};
ReplayResult assessReplay(const Track &, double independentStep = .01, const Cancel &cancel = {});
} // namespace coaster
