#include "coaster/persistence.hpp"
#include "coaster/spline.hpp"
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
        HardwarePlan hardware{
            {{"motor", ActuatorKind::LinearMotor, 0, 100}, {"brake", ActuatorKind::Brake, 100, 200}},
            {0, 1, -1}};
        std::array<Frame, 6> reactionFrames;
        for (auto &f : reactionFrames) {
            f.t = {1, 0, 0};
            f.u = {0, 0, 1};
            f.upS = {-.2, 0, 0};
        }
        const auto delivery = deliverDrive(hardware, 0, 0, 20, reactionFrames, 60000);
        check(delivery.engaged == 3, "Hardware ignored the train's finite entry footprint");
        near(delivery.forcePerReaction, 60000 / (3 * 1.06), 1e-8, "Reaction Jacobian force allocation");
        near(delivery.power, 1200000, 1e-8, "Hardware virtual work changed delivered power");
        near(delivery.residual, 0, 1e-8, "Hardware generalized-force residual");
        near(deliverDrive(hardware, 1, 150, 20, reactionFrames, -60000).power, -1200000, 1e-8,
             "Brake energy absorption sign");
        auto rejectedDrive = [&](std::size_t element, double s, double force) {
            try {
                deliverDrive(hardware, element, s, 20, reactionFrames, force);
            } catch (const std::runtime_error &) {
                return true;
            }
            return false;
        };
        check(rejectedDrive(0, 120, 60000), "Uncovered drive was accepted");
        check(rejectedDrive(1, 150, 60000), "Brake supplied positive power");
        check(rejectedDrive(2, 50, 60000), "Coast element acquired motor authority");
        check(deliverDrive(hardware, 2, 50, 20, reactionFrames, 0).zone == -1,
              "Coasting required an actuator");
        HardwarePlan catchPlan{{{"launch", ActuatorKind::CableLaunch, -10.4, 40}}, {0}};
        const auto caught = deliverDrive(catchPlan, 0, 0, 0, reactionFrames, 350000);
        check(caught.engaged == 1, "Launch catch did not attach to the rear reaction point");
        near(caught.power, 0, 0, "Stationary launch has nonzero mechanical power");
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
        Track parabola;
        Span curved;
        curved.p[1] = {100, 0, 0};
        curved.p[2] = {0, 0, 20};
        curved.u[0] = {0, 0, 1};
        curved.length = 100;
        parabola.spans.push_back(curved);
        parabola.length = 100;
        for (double distance : {0., 9., 50., 99., 100.}) {
            const auto f = parabola.at(distance);
            const double slope = .004 * distance, n = std::sqrt(1 + slope * slope);
            const Vec3 t{1 / n, 0, slope / n};
            const Vec3 td = Vec3{-slope, 0, 1} * (.004 / std::pow(n, 3));
            const Vec3 tdd = Vec3{2 * slope * slope - 1, 0, -3 * slope} * (.004 * .004 / std::pow(n, 5));
            check(norm(f.p - Vec3{distance, 0, .002 * distance * distance}) < 1e-10 &&
                      norm(f.t - t) < 1e-12 && norm(f.k - td) < 1e-12 &&
                      norm(f.u - Vec3{-t.z, 0, t.x}) < 1e-12 && norm(f.upS - Vec3{-td.z, 0, td.x}) < 1e-12 &&
                      norm(f.upSS - Vec3{-tdd.z, 0, tdd.x}) < 1e-12,
                  "Analytic curved frame or its derivatives disagree");
        }
        State circleStart;
        circleStart.p = {0, 0, 4};
        circleStart.v = 50;
        auto firstHalf = terrainSpline(circleStart, {200, pi});
        auto secondHalf = terrainSpline(shoot(firstHalf).end, {200, pi});
        secondHalf.role = Role::Terminal;
        secondHalf.geometry.back().value[2] += .1;
        const auto mismatchedClosure = compile({firstHalf, secondHalf});
        check(mismatchedClosure.closed, "Closed-position fixture did not close");
        check(assessReplay(mismatchedClosure).portUp > .09,
              "A position-closed circuit hid its station orientation discontinuity");
        bool fitCancelled = false;
        try {
            std::array<double, 1> parameter{0};
            solve<1>(parameter, {{{-2, 2}}}, [](const auto &q) {
                if (q[0] < -.5)
                    throw Cancelled();
                return std::array<double, 1>{q[0] + 1};
            });
        } catch (const Cancelled &) {
            fitCancelled = true;
        }
        check(fitCancelled, "Numerical fitting swallowed cancellation during a trial step");
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
