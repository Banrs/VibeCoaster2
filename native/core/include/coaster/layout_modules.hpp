#pragma once
#include "coaster/coaster.hpp"

namespace coaster {
// Geometric authoring only. These are open sections, not accepted rides.
// Pose uses SI and an orthonormal forward/up frame; right=cross(forward,up).
struct LayoutModulePose {
    Vec3 position{},forward{1,0,0},up{0,0,1};
};
struct EnergyLoopModuleRequest {
    LayoutModulePose entry;
    double height{60},apexSpeed{20},normalG{3.5},apexNormalG{.5},pushRampSeconds{1.2};
    double lateralOffset{36},portLength{8},sampleSpacing{1.5};
    size_t maxSamples{20000};
};
struct EnergyLoopModule {
    LayoutModulePose entry,exit;
    std::vector<AuthoredPoint> points;
    Track track;
    double height{},pitchForwardDisplacement{},idealEntrySpeed{},idealApexSpeed{};
    bool geometryBuilt{},canonicalBuilt{},cancelled{};
    ValidationReport report;
};
// Two energy-authored pitches meet at a positive-load inverted crest. The
// reflected descent restores heading and height, with actual netX=2*pitchX+2*portLength.
// A smooth caller-selected lateral offset separates the loop's crossing arms.
// No frame roll or endpoint displacement correction is used to fake closure.
EnergyLoopModule buildEnergyLoopModule(const EnergyLoopModuleRequest&,Cancel cancel={});


}
