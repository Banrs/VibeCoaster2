#pragma once
#include <array>
#include <cstddef>
#include <functional>
#include <string>

namespace coaster {
enum class WorkPhase {
    Authoring, Motion, Geometry, Structures, Forces, Refinement, Authorship,
    Parsing, Serialization, MeshPreparation, SceneCommit, Complete, Count
};
inline const char* phaseKey(WorkPhase phase){
    static constexpr std::array names{"authoring","motion","geometry","structures","forces","refinement","authorship","parsing","serialization","meshPreparation","sceneCommit","complete"};
    const auto i=static_cast<std::size_t>(phase);return i<names.size()?names[i]:"unknown";
}
inline const char* phaseName(WorkPhase phase){
    switch(phase){
        case WorkPhase::Authoring:return "Designing ride";
        case WorkPhase::Motion:return "Solving speed and energy";
        case WorkPhase::Geometry:return "Checking track and clearance";
        case WorkPhase::Structures:return "Building supports and station";
        case WorkPhase::Forces:return "Checking rider forces";
        case WorkPhase::Refinement:return "Verifying numerical accuracy";
        case WorkPhase::Authorship:return "Checking editable geometry";
        case WorkPhase::Parsing:return "Reading saved ride";
        case WorkPhase::Serialization:return "Saving verified ride";
        case WorkPhase::MeshPreparation:return "Preparing scene";
        case WorkPhase::SceneCommit:return "Building scene";
        case WorkPhase::Complete:return "Ready to ride";
        default:return "Working";
    }
}
struct WorkProgress {
    WorkPhase phase{WorkPhase::Authoring};
    int candidate{};
    double completedWork{},totalWork{}; // Zero total means no honest percentage is available.
    std::string detail;
};
using Progress=std::function<void(const WorkProgress&)>;
struct WorkTimings {
    std::array<double,static_cast<std::size_t>(WorkPhase::Count)> seconds{};
};
}
