#pragma once
#include "coaster/coaster.hpp"
#include <functional>

namespace coaster {
// Canonical polynomial construction shared by incremental authoring and reload.
Span compileCanonicalSpan(const Knot &first, const Knot &last, bool authoredGeometry = true);
TrackSample sampleSpanGeometry(const Span &, double parameter, Element = Element::Return);

struct SourceCurveSample {
    Knot knot;
    bool ownsFrame{};
};
using SourceCurve = std::function<SourceCurveSample(double)>;

struct CurveResolution {
    double maximumSpan{12};
    double positionTolerance{1e-4};
    double directionTolerance{2e-6};
    double curvatureTolerance{2e-6};
    double curvatureRateTolerance{2e-7};
};

// Retains exact source endpoints and phase boundaries. Only the representation
// is approximated; positions, directions and physical derivative jets at every
// emitted knot are sampled from the source, never reconstructed from neighbours.
std::vector<double> compileCurveDistances(const SourceCurve &, double begin, double end,
                                          CurveResolution = {}, Cancel = {});
// Uniform dyadic source samples preserve the existing compact spline provenance.
size_t compileUniformCurveIntervals(const SourceCurve &, double length, CurveResolution = {},
                                    Cancel = {});
} // namespace coaster
