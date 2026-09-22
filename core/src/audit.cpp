#include "coaster/persistence.hpp"
#include <chrono>
#include <iostream>
#include <iomanip>
using namespace coaster;
int main(int argc, char **argv) {
    try {
        if (argc < 2)
            throw std::runtime_error("audit requires a saved candidate");
        bool passed = true;
        auto d = loadDesign(argv[1]);
        std::cout << std::setprecision(10);
        auto report = [&](const char *name, const Simulation &s, double seconds) {
            passed = passed && s.failures.empty();
            std::cout << name << " time=" << seconds << " active=" << s.active << " terminal=" << s.terminal
                      << " speed=" << s.maxSpeed * 3.6 << " energy=" << s.energyResidual
                      << " failures=" << s.failures.size() << '\n';
            for (std::size_t i = 0; i < 3; ++i) {
                const auto lo = s.minimum[i], hi = s.maximum[i], rate = s.rate[i];
                std::cout << "seat " << i << " min=" << lo.x << ',' << lo.y << ',' << lo.z << " max=" << hi.x
                          << ',' << hi.y << ',' << hi.z << " rate=" << rate.x << ',' << rate.y << ','
                          << rate.z << " astm=" << s.acceleration[i].passed << '\n';
            }
            for (std::size_t i = 0; i < 3; ++i) {
                auto lo = std::min_element(
                    s.playback.begin(), s.playback.end(),
                    [&](const Sample &a, const Sample &b) { return a.force[i].z < b.force[i].z; });
                auto hi = std::max_element(
                    s.playback.begin(), s.playback.end(),
                    [&](const Sample &a, const Sample &b) { return a.force[i].z < b.force[i].z; });
                std::cout << "peaks " << i << " min=" << lo->time << " "
                          << d.track.source[d.track.at(lo->s).element].id << " max=" << hi->time << " "
                          << d.track.source[d.track.at(hi->s).element].id << "\n";
            }
            for (const auto &f : s.failures)
                std::cout << "failure " << f << '\n';
        };
        report("nominal", *d.baseline, 0);
        for (const auto &[name, scenario] :
             std::array<std::pair<const char *, Scenario>, 3>{{{"full", {1, 1, false}},
                                                               {"lower-drag-trims", {.8, 1, true}},
                                                               {"lower-drag-full", {.8, 1, false}}}}) {
            const auto start = std::chrono::steady_clock::now();
            const auto s = simulate(d.track, scenario);
            report(name, s, std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count());
        }
        const auto start = std::chrono::steady_clock::now();
        auto fine = compile(d.track.source, .01);
        fine.operationSpeed = d.track.operationSpeed;
        const auto proof = assessReplay(fine, .005);
        const auto s = simulate(fine, {}, 1. / 1920);
        report("half-space-half-time", s,
               std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count());
        std::cout << "refinement position=" << proof.position << " forward=" << proof.forward
                  << " up=" << proof.up << " energy=" << proof.energy << '\n';
        return passed ? 0 : 2;
    } catch (const std::exception &e) {
        std::cerr << e.what() << '\n';
        return 1;
    }
}
