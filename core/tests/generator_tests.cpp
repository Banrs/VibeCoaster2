#include "coaster/persistence.hpp"
#include <iostream>
#include <fstream>
#include <chrono>
using namespace coaster;
namespace {
void require(bool v, const char *m) {
    if (!v)
        throw std::runtime_error(m);
}
} // namespace
int main(int argc, char **argv) {
    try {
        const bool extended = argc > 1 && std::string(argv[1]) == "--extended";
        std::vector<Recipe> cases;
        Recipe r;
        cases.push_back(r);
        r.seed = 77;
        cases.push_back(r);
        r.openingHeight = 80;
        r.loopHeight = 140;
        r.immelmannHeight = 100;
        cases.push_back(r);
        if (extended) {
            r = Recipe{};
            r.style = "flow";
            cases.push_back(r);
            r.seed = 77;
            r.style = "intense";
            cases.push_back(r);
            r = Recipe{};
            r.topSpeedKph = 290;
            cases.push_back(r);
            r.topSpeedKph = 305;
            cases.push_back(r);
            r = Recipe{};
            r.openingHeight = 65;
            cases.push_back(r);
            r.openingHeight = 85;
            cases.push_back(r);
        }
        Vec3 firstStation;
        int failures = 0;
        for (std::size_t i = 0; i < cases.size(); ++i) {
            const auto start = std::chrono::steady_clock::now();
            try {
                auto d = generate(cases[i]);
                require(d.baseline && d.baseline->assessed, "Missing assessed train trace");
                require(d.baseline->failures.empty(),
                        d.baseline->failures.empty() ? "" : "Nominal force assessment failed");
                const auto c = assessClearance(d.track, d.recipe.plateau);
                require(!c.trackHits && !c.terrainHits && c.continuous, "Swept clearance failed");
                require(d.track.closed, "Station did not close");
                require(std::abs(d.baseline->active - 180) < .05, "Active duration");
                if (i == 0)
                    firstStation = d.track.at(0).p;
                if (i == 1)
                    require(norm(firstStation - d.track.at(0).p) > .1, "Seed was silently ignored");
                for (const auto &p : d.track.source) {
                    double height = 0;
                    if (p.id == "opening")
                        height = d.recipe.openingHeight;
                    else if (p.id == "camelback")
                        height = d.recipe.camelbackHeight;
                    else if (p.id == "loop")
                        height = d.recipe.loopHeight;
                    else if (p.id == "immelmann")
                        height = d.recipe.immelmannHeight;
                    if (height)
                        require(std::abs(shoot(p, .005).maximumHeight - p.initial.p.z - height) < .03,
                                "Edited height does not match geometry");
                }
                const auto ravine = std::find_if(d.track.source.begin(), d.track.source.end(),
                                                 [](const auto &p) { return p.id == "ravine-roll"; });
                require(ravine != d.track.source.end(), "Missing descending ravine roll");
                const auto ravineEnd = shoot(*ravine, .005).end.p;
                require(norm(ravineEnd - Vec3{790, -660, -8}) < .005,
                        "Ravine roll misses the fixed site destination");
                require(ravineEnd.z - ground(ravineEnd.x, ravineEnd.y, d.recipe.plateau) < 8,
                        "Ravine exit is not close to its fixed floor");
                const auto proof = assessReplay(d.track, .01);
                require(proof.position < .005 && proof.forward < 1e-4 && proof.up < 1e-4 &&
                            proof.energy < .001,
                        "Independent source replay diverged");
                require(proof.portPosition < .001 && proof.portTangent < 1e-5 && proof.portUp < 1e-5 &&
                            proof.portCurvature < 1e-5 && proof.portThird < 1e-5 && proof.portUpThird < 1e-4,
                        "Source boundary derivative continuity failed");
                require(d.baseline->terminal >= 5 && d.baseline->terminal <= 10, "Terminal braking duration");
                validateRide(d);
                require(d.validation != nullptr, "Complete fresh dynamics evidence missing");
                std::cout << "PASS case=" << i << " seed=" << cases[i].seed << " style=" << cases[i].style
                          << " speed=" << cases[i].topSpeedKph << " seconds="
                          << std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count()
                          << std::endl;
            } catch (const std::exception &e) {
                ++failures;
                std::cout << "FAIL case=" << i << " seed=" << cases[i].seed << " style=" << cases[i].style
                          << " speed=" << cases[i].topSpeedKph << " reason=" << e.what() << std::endl;
            }
        }
        return failures ? 1 : 0;
    } catch (const std::exception &e) {
        std::cerr << e.what() << '\n';
        return 1;
    }
}
