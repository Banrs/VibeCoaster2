#pragma once
#include "coaster/fvd.hpp"

namespace coaster {
// Point-source authoring only. These are open sections, not accepted rides.
// Pose uses SI and an orthonormal forward/up frame; right=cross(forward,up).
struct LayoutModulePose {
    Vec3 position{},forward{1,0,0},up{0,0,1};
};
struct EnergyLoopModuleRequest {
    LayoutModulePose entry;
    double height{60},apexSpeed{20},normalG{3.5},apexNormalG{.5},pushRampSeconds{1.2};
    double crossingOffset{18},portLength{8},sampleSpacing{1.5};
    double rollingAcceleration{},dragAccelerationCoefficient{};
    size_t maxSamples{20000};
};
struct EnergyLoopModule {
    LayoutModulePose entry,exit;
    std::vector<AuthoredPoint> points;
    Track track;
    FvdRequest authoring;
    std::vector<FvdSample> samples;
    FvdAssessment assessment;
    FvdSample loopEntry,apex,loopExit;
    FvdSample crossingEntry,crossingExit;
    double height{};
    bool geometryBuilt{},canonicalBuilt{},cancelled{};
    ValidationReport report;
};
// Rigidly places one continuous loss-aware FVD loop. Normal force and physical
// twist separate the actual low crossing arms during integration. The real exit
// position AND heading must be used for continuation; neither is post-warped.
// Actual samples, apex/guard boundaries and replay assessment remain available
// for section review. Routing points do not replace the canonical source jets.
EnergyLoopModule buildEnergyLoopModule(const EnergyLoopModuleRequest&,Cancel cancel={});


}
