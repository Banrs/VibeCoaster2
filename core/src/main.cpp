#include "coaster/persistence.hpp"
#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>

using namespace coaster;
int main(int argc, char **argv) {
    try {
        const auto start = std::chrono::steady_clock::now();
        const std::string mode = argc > 1 ? argv[1] : "probe";
        Recipe r;
        if (mode != "load" && argc > 2 && std::string(argv[2]) != "-")
            r = readRecipe(argv[2]);
        if (argc > 3)
            r.seed = static_cast<unsigned>(std::stoul(argv[3]));
        if (argc > 4)
            r.openingHeight = std::stod(argv[4]);
        auto design = mode == "load" ? loadDesign(argv[2]) : generate(r);
        r = design.recipe;
        const auto authored = std::chrono::steady_clock::now();
        const auto replay = assessReplay(design.track);
        auto sim = design.baseline ? *design.baseline : simulate(design.track);
        const auto clearance = assessClearance(design.track, r.plateau);
        if (clearance.trackHits)
            sim.failures.push_back("Nonlocal track clearance failed");
        if (clearance.terrainHits)
            sim.failures.push_back("Terrain clearance failed");
        std::cerr << "PORT " << replay.portPosition << " " << replay.portTangent << " " << replay.portUp
                  << " " << replay.portCurvature << " " << replay.portThird << " " << replay.portUpThird
                  << "\n";
        const bool accepted = sim.failures.empty() && std::abs(sim.active - r.activeSeconds) < .25 &&
                              sim.terminal >= 5 && sim.terminal <= 10 && clearance.terrainHits == 0 &&
                              clearance.trackHits == 0;
        std::cout << std::setprecision(10)
                  << "{\n  \"nativeCandidatePass\": " << (accepted ? "true" : "false")
                  << ",\n  \"seed\": " << r.seed
                  << ",\n  \"authorSeconds\": " << std::chrono::duration<double>(authored - start).count()
                  << ",\n  \"totalSeconds\": "
                  << std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count()
                  << ",\n  \"length\": " << design.track.length << ",\n  \"duration\": " << sim.duration
                  << ",\n  \"active\": " << sim.active << ",\n  \"terminal\": " << sim.terminal
                  << ",\n  \"speedKph\": " << sim.maxSpeed * 3.6 << ",\n  \"launchTime\": " << sim.launchTime
                  << ",\n  \"replayPosition\": " << replay.position
                  << ",\n  \"replaySpeed\": " << replay.speed
                  << ",\n  \"sourceEnergyResidual\": " << replay.energy
                  << ",\n  \"minimumGround\": " << clearance.minimumGround
                  << ",\n  \"terrainHits\": " << clearance.terrainHits
                  << ",\n  \"trackHits\": " << clearance.trackHits
                  << ",\n  \"minimumTrackGap\": " << clearance.minimumNonlocal
                  << ",\n  \"ordinaryMeanRailHeight\": " << clearance.ordinaryMeanHeight
                  << ",\n  \"ordinaryMaxRailHeight\": " << clearance.ordinaryMaxHeight
                  << ",\n  \"clifftopActive\": " << sim.clifftopActive << ",\n  \"lipBraking\": " << sim.lip
                  << ",\n  \"trainEnergyResidual\": " << sim.energyResidual
                  << ",\n  \"closed\": " << (design.track.closed ? "true" : "false") << ",\n  \"seats\": [\n";
        for (std::size_t i = 0; i < 3; ++i) {
            const auto a = sim.minimum[i], b = sim.maximum[i], q = sim.rate[i];
            std::cout << "    {\"min\": [" << a.x << ',' << a.y << ',' << a.z << "], \"max\": [" << b.x << ','
                      << b.y << ',' << b.z << "], \"rate\": [" << q.x << ',' << q.y << ',' << q.z
                      << "], \"astmPass\": " << (sim.acceleration[i].passed ? "true" : "false")
                      << ", \"nominalEnvelopePass\": " << (sim.envelope[i].nominalPassed ? "true" : "false")
                      << ", \"peakAllowancePass\": "
                      << (sim.envelope[i].peakAllowancePassed ? "true" : "false")
                      << ", \"maximumPeakExcessPercent\": " << sim.envelope[i].maximumExcessPercent
                      << ", \"diagnostics\": " << sim.acceleration[i].diagnostics.size() << '}'
                      << (i < 2 ? "," : "") << '\n';
        }
        for (const auto &seat : sim.acceleration)
            for (const auto &d : seat.diagnostics)
                std::cerr << "ASTM " << d.seatIndex << " " << d.rule << " " << d.axis << " "
                          << d.startTimeSeconds << ".." << d.endTimeSeconds << " actual=" << d.actual
                          << " limit=" << d.limit << "\n";
        std::cout << "  ],\n  \"elements\": [\n";
        for (std::size_t i = 0; i < design.track.source.size(); ++i) {
            const auto &p = design.track.source[i];
            const auto q = shoot(p);
            std::cout << "    {\"id\": \"" << p.id << "\", \"seconds\": " << p.duration()
                      << ", \"maxZ\": " << q.maximumHeight << ", \"end\": [" << q.end.p.x << ',' << q.end.p.y
                      << ',' << q.end.p.z << "], \"speed\": " << q.end.v << '}'
                      << (i + 1 < design.track.source.size() ? "," : "") << '\n';
        }
        std::cout << "  ],\n  \"failures\": [";
        for (std::size_t i = 0; i < sim.failures.size(); ++i)
            std::cout << (i ? ", " : "") << '"' << sim.failures[i] << '"';
        std::cout << "]\n}\n";
        std::filesystem::create_directories("out");
        std::ofstream csv("out/track.csv");
        csv << "s,x,y,z,ux,uy,uz,speed,role\n";
        for (double s = 0; s <= design.track.length; s += 2) {
            const auto f = design.track.at(s);
            csv << s << ',' << f.p.x << ',' << f.p.y << ',' << f.p.z << ',' << f.u.x << ',' << f.u.y << ','
                << f.u.z << ',' << f.sourceSpeed << ',' << roleName(design.track.source[f.element].role)
                << '\n';
        }
        std::ofstream forces("out/forces.csv");
        forces << "t,s,v,frontX,frontY,frontZ,middleZ,rearZ\n";
        for (const auto &q : sim.playback)
            forces << q.time << ',' << q.s << ',' << q.speed << ',' << q.force[0].x << ',' << q.force[0].y
                   << ',' << q.force[0].z << ',' << q.force[1].z << ',' << q.force[2].z << '\n';
        std::ofstream controls("out/controls.csv");
        controls << "element,t,normal,drive\n";
        double time = 0;
        for (const auto &p : design.track.source) {
            for (const auto &c : p.controls)
                controls << p.id << ',' << time + c.time << ',' << c.normal << ',' << c.drive << '\n';
            time += p.duration();
        }
        if (mode == "save") {
            if (!accepted)
                throw std::runtime_error("Candidate failed validation; existing save retained");
            saveDesign(design, "out/candidate.vcd");
        }
        return mode == "probe" ? 0 : accepted ? 0 : 2;
    } catch (const std::exception &e) {
        std::cerr << "ERROR: " << e.what() << '\n';
        return 1;
    }
}
