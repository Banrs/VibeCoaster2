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
    double sampledMaxCurvature{},sampledMaxFrameTwistPerMeter{}; // Diagnostics, not continuous bounds.
    bool geometryBuilt{},canonicalBuilt{},cancelled{};
    ValidationReport report;
};
// Immelmann: upward pitch through pi, THEN half-roll to upright.
// Dive loop: half-roll to inverted, THEN downward pitch through pi.
// Both reverse forward and restore entry up. In the entry frame, endpoints
// are (-rollLength,0,+height) and (+rollLength,0,-height), respectively.
// Equal straight guards at both ports cancel in these endpoint displacements.
// Pitch/roll progression uses S(u + shape*sin(2*pi*u)/(2*pi)); the
// bounded warp remains monotone and symmetric, preserving a true half-loop
// and exact height while varying its proportions. Pitch normalization solves
// the arc length again for each shape instead of scaling a fixed polyline.
// Pitch angle and roll angle use quintic smoothstep; angle first/second arc
// derivatives vanish at each section boundary. Position integrates the unit
// tangent by positive 8-point Gauss quadrature. Canonical knots retain the
// analytic tangent/curvature/up, and use the existing G3/C2 cache builder.
// Every point supplies upHint for insertion into an independently compiled
// circuit. Recompiling those points still requires all full-ride acceptance
// gates; this function does not size drives or certify finite-train forces.
ReversingModule buildReversingModule(const ReversingModuleRequest&,Cancel cancel={});
}
