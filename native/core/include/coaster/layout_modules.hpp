#pragma once
#include "coaster/coaster.hpp"

namespace coaster {
// Geometric authoring only. These are open sections, not accepted rides.
// Pose uses SI and an orthonormal forward/up frame; right=cross(forward,up).
struct LayoutModulePose {
    Vec3 position{},forward{1,0,0},up{0,0,1};
};
enum class ReversingModuleKind { Immelmann, DiveLoop };
struct ReversingModuleRequest {
    ReversingModuleKind kind{ReversingModuleKind::Immelmann};
    LayoutModulePose entry;
    double height{100},rollLength{180},portLength{8},sampleSpacing{1.5};
    double pitchShape{0},rollShape{0}; // [-.65,.65], independently redistribute pitch/roll curvature.
    double rollOverlap{0}; // Fraction [0,.4] of pitch arc shared with the half-roll.
    int rollDirection{1}; // +1 or -1 physical rotation about the moving tangent.
    size_t maxSamples{20000};
};
struct ReversingModule {
    ReversingModuleKind kind{ReversingModuleKind::Immelmann};
    LayoutModulePose entry,exit;
    std::vector<AuthoredPoint> points;
    Track track;
    size_t pitchBeginIndex{},pitchEndIndex{},rollBeginIndex{},rollEndIndex{};
    double pitchLength{},height{},rollLength{};
    double pitchForwardDisplacement{},idealEntrySpeed{},idealApexSpeed{}; // Energy-authored source diagnostics only.
    double sampledMaxCurvature{},sampledMaxFrameTwistPerMeter{}; // Diagnostics, not continuous bounds.
    bool geometryBuilt{},canonicalBuilt{},cancelled{};
    ValidationReport report;
};
// Immelmann: upward pitch through pi with a following half-roll to upright.
// Dive loop: half-roll to inverted with a following downward pitch through pi.
// rollOverlap shares the last/first part of the pitch arc with the roll,
// retaining the centerline dimensions while avoiding a separate motion reset.
// Both reverse forward and restore entry up. In the entry frame, endpoints
// are (-rollLength,0,+height) and (+rollLength,0,-height), respectively.
// Equal straight guards at both ports cancel in these endpoint displacements.
// Pitch/roll progression uses S(u + shape*sin(2*pi*u)/(2*pi)); the
// bounded warp remains monotone and symmetric, preserving a true half-loop
// and exact height while varying its proportions. Pitch normalization solves
// the arc length again for each shape instead of scaling a fixed polyline.
// Pitch angle and roll angle use quintic smoothstep; angle first/second arc
// derivatives vanish at the ends of each motion. Position integrates the unit
// tangent by positive 8-point Gauss quadrature. Canonical knots retain the
// analytic tangent/curvature/up, and use the existing G3/C2 cache builder.
// Every point supplies upHint for insertion into an independently compiled
// circuit. Recompiling those points still requires all full-ride acceptance
// gates; this function does not size drives or certify finite-train forces.
ReversingModule buildReversingModule(const ReversingModuleRequest&,Cancel cancel={});
// Force-designed alternative; legacy geometric authoring above is unchanged.
// geometry.pitchShape must be zero: normalG and pushRampSeconds redistribute
// pitch curvature through gravity-coupled FVD instead. Entry is level/upright.
// Height is a source climb, not height above terrain. Consume the actual exit
// position: Immelmann netX = pitchForwardDisplacement - rollLength; dive is its
// reverse. Full finite-train replay must determine final entry/apex forces.
struct EnergyReversingModuleRequest {
    ReversingModuleRequest geometry;
    double apexSpeed{24},normalG{3.5},pushRampSeconds{1.2};
};
ReversingModule buildEnergyReversingModule(const EnergyReversingModuleRequest&,Cancel cancel={});
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
