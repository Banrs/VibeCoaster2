#include "coaster/persistence.hpp"
#include <iostream>

using namespace coaster;
namespace {
void check(bool yes, const char *why) {
    if (!yes)
        throw std::runtime_error(why);
}
void near(double a, double b, double e, const char *why) {
    check(std::abs(a - b) <= e, why);
}
} // namespace
int main() {
    try {
        const auto nominal = assessForceEnvelope({-4.5, -1.5, -1.5}, {4.5, 1.5, 5});
        check(nominal.nominalPassed && nominal.peakAllowancePassed && nominal.maximumExcessPercent == 0,
              "Nominal envelope boundary");
        const auto allowed = assessForceEnvelope({0, 0, 1}, {0, 0, 5.049});
        check(!allowed.nominalPassed && allowed.peakAllowancePassed, "Below-one-percent peak allowance");
        check(!assessForceEnvelope({0, 0, 1}, {0, 0, 5 * 1.01}).peakAllowancePassed,
              "Exact one-percent peak was accepted");
        check(!assessForceEnvelope({-4.5 * 1.01, 0, 1}, {0, 0, 1}).peakAllowancePassed,
              "Negative X exact allowance boundary");
        check(!assessForceEnvelope({0, -1.5 * 1.01, -1.5 * 1.01}, {0, 0, 1}).peakAllowancePassed,
              "Negative Y/Z exact allowance boundary");
        check(assessForceEnvelope({0, 0, 1}, {std::nextafter(4.5 * 1.01, 0.), 0, 1}).peakAllowancePassed,
              "Strict interior peak boundary");
        check(!assessForceEnvelope({0, -1.515, 1}, {0, 0, 1}).peakAllowancePassed,
              "Literal one-percent Y boundary was accepted through rounding");
        check(!assessForceEnvelope({0, 0, -1.515}, {0, 0, 1}).peakAllowancePassed,
              "Literal one-percent Z boundary was accepted through rounding");
        State s;
        s.v = 0;
        auto p = launch(s, 50, 1.4);
        const auto q = shoot(p, .001);
        near(q.end.v, 50, 1e-4, "Launch reaches 180 km/h at 1.4 seconds");
        double maxDrive = 0, maxRate = 0;
        for (double t = 0; t <= 1.4; t += .0001) {
            const auto c = controlAt(p, t);
            maxDrive = std::max(maxDrive, c.drive / gravity);
            maxRate = std::max(maxRate, std::abs(c.first[3]) / gravity);
        }
        check(maxDrive <= 4.5, "Launch specific acceleration");
        check(maxRate <= 20, "Launch onset rate");
        auto track = compile({p});
        const auto replayed = assessReplay(track, .005);
        check(replayed.position < 1e-4, "Independent launch replay");
        check(replayed.energy < 1e-4, "Launch work-energy");
        s = q.end;
        HillShape h;
        h.height = 65;
        const auto low = hill(s, h);
        const auto a = shoot(low, .005);
        h.height = 85;
        const auto high = hill(s, h);
        const auto b = shoot(high, .005);
        near(a.maximumHeight - s.p.z, 65, .01, "65 m edit authors crest");
        near(b.maximumHeight - s.p.z, 85, .01, "85 m edit authors crest");
        check(std::abs(a.end.p.x - b.end.p.x) > 1, "Height edit changes geometry");
        bool cancelled = false;
        try {
            replay(high, .01, [] { return true; });
        } catch (const Cancelled &) {
            cancelled = true;
        }
        check(cancelled, "Replay cancellation");
        std::cout
            << "PASS launch, independent replay, work-energy, authored height edits and cancellation; peak="
            << maxDrive << "g rate=" << maxRate << "g/s\n";
        return 0;
    } catch (const std::exception &e) {
        std::cerr << "FAIL: " << e.what() << '\n';
        return 1;
    }
}
