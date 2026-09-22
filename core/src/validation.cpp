#include "coaster/validation.hpp"
#include <atomic>
#include <chrono>
#include <future>
#include <mutex>
#include <thread>

namespace coaster {
namespace {
void requireReplay(const ReplayResult &p) {
    for (double value :
         {p.position, p.forward, p.up, p.speed, p.energy, p.portPosition, p.portTangent, p.portUp,
          p.portCurvature, p.portThird, p.portUpFirst, p.portUpSecond, p.portUpThird})
        if (!std::isfinite(value))
            throw std::runtime_error("Ride replay evidence is not finite");
    if (!std::isfinite(p.position) || !std::isfinite(p.energy) || p.position > .005 || p.forward > 1e-4 ||
        p.up > 1e-4 || p.energy > .001 || p.portPosition > .001 || p.portTangent > 1e-5 || p.portUp > 1e-5 ||
        p.portCurvature > 1e-5 || p.portThird > 1e-5 || p.portUpFirst > 1e-5 || p.portUpSecond > 1e-5 ||
        p.portUpThird > 1e-4)
        throw std::runtime_error("Ride source replay/continuity validation failed");
}
void requireSimulation(const Simulation &s, const char *name) {
    if (!s.completed || !s.assessed || !s.failures.empty() || s.terminal < 5 || s.terminal > 10 ||
        !std::isfinite(s.energyResidual) || s.energyResidual > .002)
        throw std::runtime_error(std::string("Ride physical/clearance validation failed: ") + name +
                                 (s.failures.empty() ? "" : ": " + s.failures.front()));
}
void requireConvergence(const Simulation &s, const Simulation &nominal, bool temporal) {
    if (std::abs(s.active - nominal.active) > .025 ||
        s.energyResidual > std::max(1e-6, nominal.energyResidual * (temporal ? .4 : 1.05)))
        throw std::runtime_error("Ride timing/work-energy refinement did not converge");
    for (std::size_t seat = 0; seat < 3; ++seat)
        if (norm(s.minimum[seat] - nominal.minimum[seat]) > .01 ||
            norm(s.maximum[seat] - nominal.maximum[seat]) > .01 ||
            norm(s.rate[seat] - nominal.rate[seat]) > .4)
            throw std::runtime_error("Ride seat-force refinement did not converge");
}
void trimPlayback(Simulation &s) {
    std::vector<Sample>().swap(s.playback);
}
} // namespace
void validateRide(Design &design, const Cancel &cancel) {
    const auto started = std::chrono::steady_clock::now();
    design.validation.reset();
    poll(cancel);
    validateAuthoring(design, cancel);
    auto &track = design.track;
    if (!track.closed || track.length < 1000 || track.length > 20000)
        throw std::runtime_error("Ride physical/clearance validation failed: open or unsupported circuit");
    track.operationSpeed.clear();
    const auto reference = simulate(track, {}, 1. / 240, cancel, false);
    if (!reference.completed)
        throw std::runtime_error("Ride operating reference cannot finish");
    for (const auto &q : reference.playback)
        track.operationSpeed.push_back({q.s, q.speed});
    auto fine = compile(track.source, .01, cancel);
    fine.operationSpeed = track.operationSpeed;
    auto evidence = std::make_shared<RideValidation>();
    evidence->setupSeconds =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count();
    Simulation nominal;
    using Check = std::function<void(const Cancel &)>;
    std::vector<Check> checks;
    checks.push_back([&](const Cancel &c) { nominal = simulate(track, {}, 1. / 960, c); });
    constexpr std::array<Scenario, 3> scenarios{{{1, 1, false}, {.8, 1, true}, {.8, 1, false}}};
    for (std::size_t i = 0; i < scenarios.size(); ++i)
        checks.push_back(
            [&, i](const Cancel &c) { evidence->operating[i] = simulate(track, scenarios[i], 1. / 960, c); });
    checks.push_back([&](const Cancel &c) { evidence->refined[0] = simulate(track, {}, 1. / 1920, c); });
    checks.push_back([&](const Cancel &c) { evidence->refined[1] = simulate(fine, {}, 1. / 960, c); });
    checks.push_back([&](const Cancel &c) { evidence->refined[2] = simulate(fine, {}, 1. / 1920, c); });
    checks.push_back([&](const Cancel &c) {
        evidence->replay = assessReplay(track, .01, c);
        evidence->refinedReplay = assessReplay(fine, .005, c);
        evidence->clearance = assessClearance(track, design.recipe.plateau, .5, c);
    });
    checks.push_back([&](const Cancel &c) {
        try {
            evidence->scene = authorStructures(track, design.recipe.plateau, c);
        } catch (const std::runtime_error &error) {
            throw std::runtime_error(std::string("Ride physical/clearance validation failed: ") +
                                     error.what());
        }
    });
    // Bound independent CPU checks, leaving a core for the interactive runtime.
    // Each check owns its output and reads immutable tracks. Even callbacks
    // with mutable caller state are never invoked concurrently.
    std::atomic<std::size_t> next{};
    std::atomic<bool> stopped{};
    std::mutex callbackMutex;
    std::vector<std::exception_ptr> errors(checks.size());
    const unsigned available = std::thread::hardware_concurrency();
    const unsigned workers = std::clamp(available > 1 ? available - 1 : 1U, 1U, 8U);
    evidence->workers = workers;
    std::vector<std::future<void>> tasks;
    for (unsigned worker = 0; worker < workers; ++worker)
        tasks.push_back(std::async(std::launch::async, [&] {
            unsigned polls = 0;
            Cancel checkCancel = [&] {
                if (stopped.load(std::memory_order_relaxed))
                    return true;
                if ((polls++ & 255U) != 0)
                    return false;
                std::lock_guard lock(callbackMutex);
                if (cancel && cancel()) {
                    stopped.store(true, std::memory_order_relaxed);
                    return true;
                }
                return false;
            };
            for (;;) {
                const auto i = next.fetch_add(1);
                if (i >= checks.size())
                    return;
                try {
                    poll(checkCancel);
                    const auto checkStarted = std::chrono::steady_clock::now();
                    checks[i](checkCancel);
                    evidence->checkSeconds[i] =
                        std::chrono::duration<double>(std::chrono::steady_clock::now() - checkStarted)
                            .count();
                } catch (...) {
                    errors[i] = std::current_exception();
                }
            }
        }));
    for (auto &task : tasks)
        task.get();
    poll(cancel);
    for (const auto &error : errors)
        if (error)
            std::rethrow_exception(error);
    requireReplay(evidence->replay);
    requireReplay(evidence->refinedReplay);
    const auto &clearance = evidence->clearance;
    if (!clearance.continuous || clearance.terrainHits || clearance.trackHits)
        throw std::runtime_error("Ride physical/clearance validation failed: occupied volume");
    requireSimulation(nominal, "nominal");
    if (std::abs(nominal.active - design.recipe.activeSeconds) > .05 || nominal.launchTime <= 0 ||
        nominal.launchTime > 1.4 || std::abs(nominal.maxSpeed * 3.6 - design.recipe.topSpeedKph) > .05)
        throw std::runtime_error("Ride authoring timing or speed differs from its physical result");
    constexpr std::array<const char *, 3> names{"full", "lower-drag-trims", "lower-drag-full"};
    for (std::size_t i = 0; i < evidence->operating.size(); ++i) {
        requireSimulation(evidence->operating[i], names[i]);
        trimPlayback(evidence->operating[i]);
    }
    for (std::size_t i = 0; i < evidence->refined.size(); ++i) {
        requireSimulation(evidence->refined[i], "refinement");
        requireConvergence(evidence->refined[i], nominal, i != 1);
        trimPlayback(evidence->refined[i]);
    }
    poll(cancel);
    evidence->seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count();
    design.baseline = std::make_shared<Simulation>(std::move(nominal));
    design.validation = std::move(evidence);
}
} // namespace coaster