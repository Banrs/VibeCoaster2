#include "coaster/track.hpp"

namespace coaster {
namespace {
constexpr std::array<double, 10> fact{1, 1, 2, 6, 24, 120, 720, 5040, 40320, 362880};
template <std::size_t N>
std::array<Vec3, 2 * N> hermite(std::array<Vec3, N> a, std::array<Vec3, N> b, double h) {
    std::array<Vec3, 2 * N> c{};
    double power = 1;
    for (std::size_t i = 0; i < N; ++i) {
        c[i] = a[i] * (power / fact[i]);
        power *= h;
    }
    std::array<std::array<double, N + 1>, N> m{};
    for (int axis = 0; axis < 3; ++axis) {
        power = 1;
        for (std::size_t i = 0; i < N; ++i) {
            Vec3 delta = b[i] * power;
            for (std::size_t j = i; j < N; ++j)
                delta = delta - c[j] * (fact[j] / fact[j - i]);
            for (std::size_t j = 0; j < N; ++j)
                m[i][j] = fact[N + j] / fact[N + j - i];
            m[i][N] = axis == 0 ? delta.x : axis == 1 ? delta.y : delta.z;
            power *= h;
        }
        if (!eliminate<N>(m))
            throw std::runtime_error("Singular Hermite basis");
        for (std::size_t i = 0; i < N; ++i) {
            if (axis == 0)
                c[N + i].x = m[i][N];
            else if (axis == 1)
                c[N + i].y = m[i][N];
            else
                c[N + i].z = m[i][N];
        }
    }
    return c;
}
template <std::size_t N> Vec3 evaluate(const std::array<Vec3, N> &c, double x, int derivative) {
    Vec3 result;
    for (int i = int(N) - 1; i >= derivative; --i)
        result = result * x + c[std::size_t(i)] * (fact[std::size_t(i)] / fact[std::size_t(i - derivative)]);
    return result;
}
template <std::size_t N, std::size_t D>
std::array<Vec3, D> evaluateDerivatives(const std::array<Vec3, N> &c, double x) {
    // Generalized Horner evaluation shares work between the exact polynomial
    // derivatives. No lookup table, finite difference or resampling is used.
    std::array<Vec3, D> result{};
    for (std::size_t i = N; i-- > 0;) {
        for (std::size_t j = D; --j > 0;)
            result[j] = result[j] * x + result[j - 1] * double(j);
        result[0] = result[0] * x + c[i];
    }
    return result;
}
struct VectorJet {
    Vec3 a, b, c;
};
VectorJet normalize(VectorJet v) {
    const double n = norm(v.a), nd = dot(v.a, v.b) / n, ndd = (dot(v.b, v.b) + dot(v.a, v.c) - nd * nd) / n;
    const Vec3 a = v.a / n, b = (v.b - a * nd) / n, c = (v.c - b * (2 * nd) - a * ndd) / n;
    return {a, b, c};
}
Frame sample(const Span &s, double x) {
    const double h = s.length;
    const auto position = evaluateDerivatives<10, 4>(s.p, x);
    const auto orientation = evaluateDerivatives<8, 3>(s.u, x);
    const auto p = position[0] + s.origin, p1 = position[1] / h, p2 = position[2] / (h * h),
               p3 = position[3] / (h * h * h);
    const auto tangent = normalize({p1, p2, p3});
    const VectorJet up{orientation[0], orientation[1] / h, orientation[2] / (h * h)};
    const double d = dot(up.a, tangent.a), dd = dot(up.b, tangent.a) + dot(up.a, tangent.b),
                 ddd = dot(up.c, tangent.a) + 2 * dot(up.b, tangent.b) + dot(up.a, tangent.c);
    const auto oriented = normalize({up.a - tangent.a * d, up.b - tangent.b * d - tangent.a * dd,
                                     up.c - tangent.c * d - tangent.b * (2 * dd) - tangent.a * ddd});
    return {p,
            tangent.a,
            oriented.a,
            cross(tangent.a, oriented.a),
            tangent.b,
            oriented.b,
            oriented.c,
            std::lerp(s.driveA, s.driveB, x),
            std::lerp(s.speedA, s.speedB, x),
            s.time + s.duration * x,
            s.element};
}
} // namespace
Frame Track::at(double distance) const {
    std::size_t hint = spans.size();
    return at(distance, hint);
}
Frame Track::at(double distance, std::size_t &hint) const {
    if (spans.empty() || !std::isfinite(distance))
        throw std::runtime_error("Invalid track query");
    if (distance < 0) {
        hint = 0;
        auto f = sample(spans.front(), 0);
        f.p += f.t * distance;
        f.k = f.upS = f.upSS = {};
        f.drive = 0;
        return f;
    }
    if (distance > length) {
        hint = spans.size() - 1;
        auto f = sample(spans.back(), 1);
        f.p += f.t * (distance - length);
        f.k = f.upS = f.upSS = {};
        f.drive = 0;
        return f;
    }
    if (hint >= spans.size() || distance < spans[hint].begin ||
        distance > spans[hint].begin + spans[hint].length) {
        if (hint + 1 < spans.size() && distance >= spans[hint + 1].begin &&
            distance <= spans[hint + 1].begin + spans[hint + 1].length)
            ++hint;
        else {
            auto hi = std::upper_bound(spans.begin(), spans.end(), distance,
                                       [](double d, const Span &s) { return d < s.begin; });
            hint = hi == spans.begin() ? 0 : std::size_t(hi - spans.begin() - 1);
        }
    }
    const auto &s = spans[hint];
    return sample(s, std::clamp((distance - s.begin) / s.length, 0., 1.));
}
Track compile(std::vector<Program> source, double step, const Cancel &cancel) {
    if (source.empty() || source.size() > 100 || !(step >= .001 && step <= .1))
        throw std::runtime_error("Invalid geometry source");
    Track track;
    track.source = std::move(source);
    track.sourceStep = step;
    double time = 0;
    std::vector<std::vector<Jet>> programs;
    for (const auto &p : track.source) {
        poll(cancel);
        auto jets = replay(p, step, cancel);
        if (p.role == Role::Terminal)
            while (jets.size() > 2 && jets.back().s - jets[jets.size() - 2].s < .02)
                jets.erase(jets.end() - 2);
        programs.push_back(std::move(jets));
    }
    // Both interpolants use the same jets at every source boundary. Keep
    // the originals in source so independent replay still measures changes.
    // Distribute tiny integration position drift as a rigid translation;
    // never compress it into the final short interval of an element.
    for (std::size_t i = 1; i < programs.size(); ++i) {
        auto &a = programs[i - 1].back();
        auto &b = programs[i].front();
        if (norm(a.position[0] - b.position[0]) > .001 || norm(a.position[1] - b.position[1]) > 1e-5 ||
            norm(a.up[0] - b.up[0]) > 1e-5)
            throw std::runtime_error("Source ports are not coincident");
        const Vec3 translation = a.position[0] - b.position[0];
        for (auto &q : programs[i])
            q.position[0] += translation;
        for (std::size_t k = 0; k < 5; ++k) {
            const Vec3 shared = (a.position[k] + b.position[k]) * .5;
            a.position[k] = b.position[k] = shared;
        }
        for (std::size_t k = 0; k < 4; ++k) {
            const Vec3 shared = (a.up[k] + b.up[k]) * .5;
            a.up[k] = b.up[k] = shared;
        }
    }
    for (std::size_t index = 0; index < track.source.size(); ++index) {
        poll(cancel);
        const auto &p = track.source[index];
        const auto &jets = programs[index];
        for (std::size_t i = 1; i < jets.size(); ++i) {
            const auto &a = jets[i - 1];
            const auto &b = jets[i];
            const double h = b.s - a.s;
            if (!(h > 1e-8))
                throw std::runtime_error("Nonmonotonic source distance");
            auto pa = a.position, pb = b.position;
            pa[0] = {};
            pb[0] = b.position[0] - a.position[0];
            track.spans.push_back({a.position[0], hermite(pa, pb, h), hermite(a.up, b.up, h), track.length, h,
                                   time + a.time, b.time - a.time, a.speed, b.speed, a.drive, b.drive,
                                   index});
            track.length += h;
        }
        time += p.duration();
        track.elementEnds.push_back(track.length);
        track.elementTimes.push_back(time);
    }
    track.sourceDuration = time;
    track.closed =
        track.source.back().role == Role::Terminal && norm(track.at(0).p - track.at(track.length).p) < .01;
    return track;
}
ReplayResult assessReplay(const Track &track, double step, const Cancel &cancel) {
    ReplayResult result;
    double distance = 0;
    std::size_t hint = 0;
    Jet prior{};
    bool hasPrior = false;
    for (const auto &p : track.source) {
        auto jets = replay(p, step, cancel);
        if (p.role == Role::Terminal)
            while (jets.size() > 2 && jets.back().s - jets[jets.size() - 2].s < 1e-6)
                jets.erase(jets.end() - 2);
        const auto &first = jets.front();
        if (hasPrior) {
            result.portPosition = std::max(result.portPosition, norm(first.position[0] - prior.position[0]));
            result.portTangent = std::max(result.portTangent, norm(first.position[1] - prior.position[1]));
            result.portUp = std::max(result.portUp, norm(first.up[0] - prior.up[0]));
            result.portCurvature =
                std::max(result.portCurvature, norm(first.position[2] - prior.position[2]));
            result.portThird = std::max(result.portThird, norm(first.position[3] - prior.position[3]));
            result.portUpThird = std::max(result.portUpThird, norm(first.up[3] - prior.up[3]));
        }
        for (const auto &q : jets) {
            const auto f = track.at(distance + q.s - p.initial.s, hint);
            result.position = std::max(result.position, norm(f.p - q.position[0]));
            result.forward = std::max(result.forward, norm(f.t - q.position[1]));
            result.up = std::max(result.up, norm(f.u - q.up[0]));
            result.speed = std::max(result.speed, std::abs(f.sourceSpeed - q.speed));
        }
        const auto shot = shoot(p, step, cancel);
        const auto &end = shot.end;
        const double initial = .5 * p.initial.v * p.initial.v + gravity * p.initial.p.z,
                     final = .5 * end.v * end.v + gravity * end.p.z + (end.lossWork - p.initial.lossWork) -
                             (end.driveWork - p.initial.driveWork);
        result.energy = std::max(result.energy, std::abs(final - initial));
        prior = jets.back();
        hasPrior = true;
        distance += prior.s - p.initial.s;
    }
    return result;
}
} // namespace coaster
