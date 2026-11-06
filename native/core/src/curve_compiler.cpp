#include "curve_compiler.hpp"
#include "motion_spline.hpp"
#include <stdexcept>

namespace coaster {
namespace {
struct PolynomialJet {
    Vec3 position, first, second, third, fourth;
};
PolynomialJet evaluate(const Span &span, double u) {
    PolynomialJet jet{span.c.back()};
    for (int i = int(span.c.size()) - 2; i >= 0; --i) {
        jet.fourth = jet.fourth * u + jet.third * 4;
        jet.third = jet.third * u + jet.second * 3;
        jet.second = jet.second * u + jet.first * 2;
        jet.first = jet.first * u + jet.position;
        jet.position = jet.position * u + span.c[size_t(i)];
    }
    return jet;
}

std::array<Vec3, 8> framePolynomial(const Knot &a, const Knot &b, const Span &span) {
    const auto left = evaluate(span, 0), right = evaluate(span, 1);
    auto parameterJets = [](const Knot &knot, const PolynomialJet &jet) {
        const double q = norm(jet.first), qu = dot(jet.first, jet.second) / q;
        const double quu = (dot(jet.second, jet.second) + dot(jet.first, jet.third) - qu * qu) / q;
        return std::array<Vec3, 4>{
            knot.up, knot.upFirst * q, knot.upSecond * (q * q) + knot.upFirst * qu,
            knot.upThird * (q * q * q) + knot.upSecond * (3 * q * qu) + knot.upFirst * quu};
    };
    const auto x = parameterJets(a, left), y = parameterJets(b, right);
    const Vec3 c2 = x[2] * .5, c3 = x[3] / 6, p = y[0] - x[0] - x[1] - c2 - c3;
    const Vec3 v = y[1] - x[1] - c2 * 2 - c3 * 3, acc = y[2] - c2 * 2 - c3 * 6, j = y[3] - c3 * 6;
    return {x[0],
            x[1],
            c2,
            c3,
            p * 35 - v * 15 + acc * 2.5 - j / 6,
            p * (-84) + v * 39 - acc * 7 + j * .5,
            p * 70 - v * 34 + acc * 6.5 - j * .5,
            p * (-20) + v * 10 - acc * 2 + j / 6};
}

bool represents(const SourceCurve &source, double begin, double end, const SourceCurveSample &a,
                const SourceCurveSample &b, const CurveResolution &resolution) {
    if (end - begin > resolution.maximumSpan)
        return false;
    Span span;
    try {
        span = compileCanonicalSpan(a.knot, b.knot);
    } catch (const std::runtime_error &) {
        return false;
    }
    Track frame;
    if (a.ownsFrame && b.ownsFrame) {
        span.referenceUp = framePolynomial(a.knot, b.knot, span);
        frame.knots = {a.knot, b.knot};
        frame.spans = {span};
        frame.length = span.length;
        frame.closed = false;
        frame.authoredGeometry = frame.authoredFrame = true;
    }
    // Interior checks are distinct from endpoint interpolation conditions.
    // Non-dyadic probes also detect a lobe hidden between recursive midpoints.
    for (double fraction : {.173, .381, .619, .827}) {
        const auto expected = source(begin + (end - begin) * fraction);
        const auto raw = evaluate(span, fraction);
        const auto actual =
            detail::arcJet(raw.position, raw.first, raw.second, raw.third, raw.fourth);
        if (norm(actual.position - expected.knot.position) > resolution.positionTolerance ||
            norm(actual.tangent - expected.knot.tangent) > resolution.directionTolerance ||
            norm(actual.curvature - expected.knot.curvature) > resolution.curvatureTolerance ||
            norm(actual.third - expected.knot.third) > resolution.curvatureRateTolerance)
            return false;
        if (expected.ownsFrame && !frame.spans.empty()) {
            const auto physical = sampleSpanKinematics(frame, 0, fraction);
            if (norm(physical.sample.up - expected.knot.up) > 2e-5 ||
                norm(physical.upS - expected.knot.upFirst) > 2e-6 ||
                norm(physical.upSS - expected.knot.upSecond) > 2e-7 ||
                norm(physical.upSSS - expected.knot.upThird) > 2e-8)
                return false;
        }
    }
    return true;
}
} // namespace

TrackSample sampleSpanGeometry(const Span &span, double parameter, Element element) {
    if (!std::isfinite(parameter) || parameter < 0 || parameter > 1)
        throw std::invalid_argument("Geometry parameter is outside its span");
    const auto jet = evaluate(span, parameter);
    const double q = norm(jet.first);
    if (!(q > 0))
        throw std::runtime_error("Degenerate source geometry");
    const Vec3 tangent = jet.first / q;
    const Vec3 up = unit(Vec3{0, 0, 1} - tangent * tangent.z);
    return {jet.position,
            tangent,
            (jet.second - tangent * dot(tangent, jet.second)) / (q * q),
            up,
            cross(tangent, up),
            element};
}

std::vector<double> compileCurveDistances(const SourceCurve &source, double begin, double end,
                                          CurveResolution resolution, Cancel cancel) {
    if (!std::isfinite(begin) || !std::isfinite(end) || end <= begin)
        throw std::invalid_argument("Invalid source curve interval");
    std::vector<double> result{begin};
    auto split = [&](auto &&self, double a, double b, const SourceCurveSample &first,
                     const SourceCurveSample &last, int depth) -> void {
        if (cancel && cancel())
            throw std::runtime_error("CANCELLED");
        if (represents(source, a, b, first, last, resolution)) {
            result.push_back(b);
            return;
        }
        if (depth >= 20 || b - a < .002)
            throw std::runtime_error(
                "Source curve does not converge to its canonical representation");
        const double middle = (a + b) * .5;
        const auto sample = source(middle);
        self(self, a, middle, first, sample, depth + 1);
        self(self, middle, b, sample, last, depth + 1);
    };
    split(split, begin, end, source(begin), source(end), 0);
    return result;
}

size_t compileUniformCurveIntervals(const SourceCurve &source, double length,
                                    CurveResolution resolution, Cancel cancel) {
    size_t count = 4;
    while (length / count > resolution.maximumSpan)
        count *= 2;
    for (; count <= 65536; count *= 2) {
        bool accepted = true;
        auto first = source(0);
        for (size_t i = 1; i <= count; ++i) {
            if (cancel && cancel())
                throw std::runtime_error("CANCELLED");
            const double a = length * (i - 1) / count, b = length * i / count;
            const auto last = source(b);
            if (!represents(source, a, b, first, last, resolution)) {
                accepted = false;
                break;
            }
            first = last;
        }
        if (accepted)
            return count;
    }
    throw std::runtime_error("Spline source does not converge to its canonical representation");
}
} // namespace coaster
