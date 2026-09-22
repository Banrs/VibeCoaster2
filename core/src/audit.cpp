#include "coaster/persistence.hpp"
#include <iomanip>
#include <iostream>
using namespace coaster;
int main(int argc, char **argv) {
    try {
        if (argc < 2)
            throw std::runtime_error("audit requires a saved candidate");
        const auto d = loadDesign(argv[1]);
        const auto &v = *d.validation;
        std::cout << std::setprecision(10);
        auto report = [&](const char *name, const Simulation &s, double seconds) {
            std::cout << name << " time=" << seconds << " step=" << s.dt << " active=" << s.active
                      << " terminal=" << s.terminal << " speed=" << s.maxSpeed * 3.6
                      << " energy=" << s.energyResidual << " failures=" << s.failures.size() << '\n';
            for (std::size_t i = 0; i < s.demand.size(); ++i) {
                const auto &q = s.demand[i];
                const auto &zone = s.hardware.zones[i];
                std::cout << "hardware " << zone.id << " interval=" << zone.begin << ',' << zone.end
                          << " force_N=" << q.peakForce << " reaction_N=" << q.peakReactionForce
                          << " power_W=" << q.peakPower << " input_J=" << q.positiveWork
                          << " absorbed_J=" << q.absorbedWork << " engaged=" << q.minimumEngaged
                          << " powerResidual_W=" << q.maximumWorkResidual << '\n';
            }
            for (std::size_t i = 0; i < 3; ++i) {
                const auto lo = s.minimum[i], hi = s.maximum[i], rate = s.rate[i];
                std::cout << "seat " << i << " min=" << lo.x << ',' << lo.y << ',' << lo.z << " max=" << hi.x
                          << ',' << hi.y << ',' << hi.z << " rate=" << rate.x << ',' << rate.y << ','
                          << rate.z << " astm=" << s.acceleration[i].passed
                          << " nominal=" << s.envelope[i].nominalPassed
                          << " peakAllowance=" << s.envelope[i].peakAllowancePassed
                          << " excessPercent=" << s.envelope[i].maximumExcessPercent << '\n';
            }
        };
        std::cout << "fresh-validation seconds=" << v.seconds << " setup=" << v.setupSeconds
                  << " workers=" << v.workers << " siteRevision=" << terrainRevision << '\n';
        report("nominal", *d.baseline, v.checkSeconds[0]);
        constexpr std::array<const char *, 3> names{"full", "lower-drag-trims", "lower-drag-full"};
        constexpr std::array<const char *, 3> refinements{"half-time", "half-space", "half-space-half-time"};
        for (std::size_t i = 0; i < 3; ++i)
            report(names[i], v.operating[i], v.checkSeconds[1 + i]);
        for (std::size_t i = 0; i < 3; ++i)
            report(refinements[i], v.refined[i], v.checkSeconds[4 + i]);
        std::cout << "replay position=" << v.replay.position << " forward=" << v.replay.forward
                  << " up=" << v.replay.up << " energy=" << v.replay.energy << '\n';
        std::cout << "refined-replay position=" << v.refinedReplay.position
                  << " forward=" << v.refinedReplay.forward << " up=" << v.refinedReplay.up
                  << " energy=" << v.refinedReplay.energy << '\n';
        std::cout << "terrain-track-clearance continuous=" << v.clearance.continuous
                  << " terrainHits=" << v.clearance.terrainHits << " trackHits=" << v.clearance.trackHits
                  << " groundGap=" << v.clearance.minimumGround << " trackGap=" << v.clearance.minimumNonlocal
                  << '\n';
        std::cout << "source-ports position=" << v.replay.portPosition << " tangent=" << v.replay.portTangent
                  << " curvature=" << v.replay.portCurvature << " third=" << v.replay.portThird
                  << " up=" << v.replay.portUp << " upFirst=" << v.replay.portUpFirst
                  << " upSecond=" << v.replay.portUpSecond << " upThird=" << v.replay.portUpThird << '\n';
        std::cout << "scene-clearance seconds=" << v.checkSeconds[8] << " parts=" << v.scene.parts.size()
                  << " supports=" << v.scene.supportAnchors.size()
                  << " minimumGap=" << v.scene.minimumCertifiedGap << '\n';
        return 0;
    } catch (const std::exception &e) {
        std::cerr << e.what() << '\n';
        return 1;
    }
}