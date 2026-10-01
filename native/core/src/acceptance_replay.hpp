#pragma once
#include "acceptance_internal.hpp"
#include "progress_internal.hpp"
#include "simulation_internal.hpp"
#include <atomic>
#include <future>

namespace coaster {

// Workers borrow the design. Cancellation and joins must precede its destruction.
class AcceptanceReplay {
    Design& design;
    Cancel cancel;
    std::atomic<bool> stopped{false};
    std::future<SimulationResult> coarse, fine;
    std::future<SpatialReplay> spatial;

    bool stopping() const { return stopped.load() || (cancel && cancel()); }
    void enter(WorkRecorder* work, WorkPhase phase, const char* detail) const {
        if (work) work->enter(phase, design.candidate, detail);
    }
public:
    AcceptanceReplay(Design& value, Cancel cancellation) : design(value), cancel(std::move(cancellation)) {
        auto replay = [this](double step) {
            return simulate(design.track, design.operations, design.request.train, step,
                [this] { return stopping(); });
        };
        coarse = std::async(std::launch::async, replay, design.request.simulationStep);
        fine = std::async(std::launch::async, replay, design.request.simulationStep * .5);
    }
    ~AcceptanceReplay() { join(); }
    void join() {
        stopped.store(true);
        if (coarse.valid()) coarse.wait();
        if (fine.valid()) fine.wait();
        if (spatial.valid()) spatial.wait();
    }
    AcceptanceReplay(const AcceptanceReplay&) = delete;
    AcceptanceReplay& operator=(const AcceptanceReplay&) = delete;

    void startSpatial() {
        spatial = std::async(std::launch::async, [this] {
            return replaySpatialRefinement(design, [this] { return stopping(); });
        });
    }
    void finish(const ClearanceSweep& sweep, WorkRecorder* work) {
        enter(work, WorkPhase::Forces, "Checking measured seat loads and ride targets");
        design.simulation = coarse.get();
        evaluateTargets(design, &sweep);
        enter(work, WorkPhase::Authorship, "Checking editable sources and continuous motion");
        assessAuthorship(design, cancel);
        assessMotion(design, cancel);
        enter(work, WorkPhase::Refinement, "Checking independent time and spatial refinement");
        verifyConvergenceWith(design, [this] { return fine.get(); }, cancel);
        if (design.report.valid() && design.convergence.passed)
            verifySpatialRefinementWith(design, [this] { return spatial.get(); }, cancel);
        join();
        if (design.checksPassed()) freezeAcceptedRevision(design);
    }
};
}
