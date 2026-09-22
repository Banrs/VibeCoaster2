#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>

namespace coaster {
constexpr double pi = 3.14159265358979323846, gravity = 9.80665;
struct Vec3 {
    double x{}, y{}, z{};
    Vec3 operator+(Vec3 b) const { return {x + b.x, y + b.y, z + b.z}; }
    Vec3 operator-(Vec3 b) const { return {x - b.x, y - b.y, z - b.z}; }
    Vec3 operator-() const { return {-x, -y, -z}; }
    Vec3 operator*(double s) const { return {x * s, y * s, z * s}; }
    Vec3 operator/(double s) const { return *this * (1 / s); }
    Vec3 &operator+=(Vec3 b) {
        *this = *this + b;
        return *this;
    }
};
inline double dot(Vec3 a, Vec3 b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}
inline Vec3 cross(Vec3 a, Vec3 b) {
    return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}
inline double norm(Vec3 a) {
    return std::sqrt(dot(a, a));
}
inline bool finite(Vec3 a) {
    return std::isfinite(a.x) && std::isfinite(a.y) && std::isfinite(a.z);
}
inline Vec3 unit(Vec3 a) {
    const double n = norm(a);
    if (n < 1e-14 || !std::isfinite(n))
        throw std::runtime_error("Degenerate frame");
    return a / n;
}
inline Vec3 rotate(Vec3 a, Vec3 axis, double r) {
    return a * std::cos(r) + cross(axis, a) * std::sin(r) + axis * (dot(axis, a) * (1 - std::cos(r)));
}
inline double smooth(double x) {
    x = std::clamp(x, 0., 1.);
    return x * x * x * (10 + x * (-15 + 6 * x));
}
inline double rad(double degrees) {
    return degrees * pi / 180;
}
template <std::size_t N> bool eliminate(std::array<std::array<double, N + 1>, N> &m) {
    for (std::size_t j = 0; j < N; ++j) {
        std::size_t pivot = j;
        for (std::size_t i = j + 1; i < N; ++i)
            if (std::abs(m[i][j]) > std::abs(m[pivot][j]))
                pivot = i;
        if (std::abs(m[pivot][j]) < 1e-12)
            return false;
        std::swap(m[pivot], m[j]);
        const double d = m[j][j];
        for (std::size_t k = j; k <= N; ++k)
            m[j][k] /= d;
        for (std::size_t i = 0; i < N; ++i)
            if (i != j) {
                const double f = m[i][j];
                for (std::size_t k = j; k <= N; ++k)
                    m[i][k] -= f * m[j][k];
            }
    }
    return true;
}
template <std::size_t N, class F>
bool solve(std::array<double, N> &p, const std::array<std::pair<double, double>, N> &bounds, F residual) {
    auto score = [](const auto &r) {
        double s = 0;
        for (double x : r)
            s += x * x;
        return s;
    };
    for (int iteration = 0; iteration < 40; ++iteration) {
        const auto r = residual(p);
        const double s = score(r);
        if (!std::isfinite(s))
            return false;
        if (s < 1e-16)
            return true;
        std::array<std::array<double, N + 1>, N> m{};
        for (std::size_t j = 0; j < N; ++j) {
            auto q = p;
            const double e = 1e-4 * std::max(1., std::abs(p[j]));
            q[j] += e;
            const auto shifted = residual(q);
            for (std::size_t i = 0; i < N; ++i)
                m[i][j] = (shifted[i] - r[i]) / e;
        }
        for (std::size_t i = 0; i < N; ++i)
            m[i][N] = -r[i];
        if (!eliminate<N>(m))
            break;
        bool improved = false;
        for (int k = 0; k < 14; ++k) {
            auto q = p;
            bool valid = true;
            for (std::size_t j = 0; j < N; ++j) {
                q[j] += m[j][N] * std::ldexp(1., -k);
                valid &= q[j] >= bounds[j].first && q[j] <= bounds[j].second;
            }
            if (!valid)
                continue;
            try {
                if (score(residual(q)) < s) {
                    p = q;
                    improved = true;
                    break;
                }
            } catch (const std::runtime_error &) {
            }
        }
        if (!improved)
            break;
    }
    // Damped least squares recovers constrained sources when a pure Newton
    // step runs into a duration bound. Geometry/ride acceptance is unchanged.
    double damping = .01;
    for (int iteration = 0; iteration < 100; ++iteration) {
        const auto r = residual(p);
        const double current = score(r);
        if (current < 1e-14)
            return true;
        std::array<std::array<double, N>, N> jac{};
        for (std::size_t j = 0; j < N; ++j) {
            auto q = p;
            const double e = 1e-4 * std::max(1., std::abs(p[j]));
            q[j] += e;
            const auto shifted = residual(q);
            for (std::size_t i = 0; i < N; ++i)
                jac[i][j] = (shifted[i] - r[i]) / e;
        }
        std::array<std::array<double, N + 1>, N> m{};
        for (std::size_t i = 0; i < N; ++i) {
            for (std::size_t j = 0; j < N; ++j)
                for (std::size_t k = 0; k < N; ++k)
                    m[i][j] += jac[k][i] * jac[k][j];
            for (std::size_t k = 0; k < N; ++k)
                m[i][N] -= jac[k][i] * r[k];
            m[i][i] += damping * std::max(.001, m[i][i]);
        }
        if (!eliminate<N>(m))
            return false;
        auto q = p;
        for (std::size_t i = 0; i < N; ++i)
            q[i] = std::clamp(p[i] + m[i][N], bounds[i].first, bounds[i].second);
        bool improved = false;
        try {
            improved = score(residual(q)) < current;
        } catch (const std::runtime_error &) {
        }
        if (improved) {
            p = q;
            damping = std::max(1e-8, damping * .3);
        } else
            damping *= 10;
        if (damping > 1e8)
            return false;
    }
    return false;
}
} // namespace coaster
