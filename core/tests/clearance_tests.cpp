#include "coaster/simulation.hpp"
#include "coaster/scene.hpp"
#include "coaster/vehicle.hpp"
#include <iostream>
using namespace coaster;
namespace {
void require(bool v, const char *m) {
    if (!v)
        throw std::runtime_error(m);
}
Track crossing(double height) {
    Track t;
    Program p;
    p.role = Role::Journey;
    t.source.push_back(p);
    Span a;
    a.origin = {-20, -500, 4};
    a.p[1] = {40, 0, 0};
    a.u[0] = {0, 0, 1};
    a.length = 40;
    Span b;
    b.origin = {0, -520, height};
    b.p[1] = {0, 40, 0};
    b.u[0] = {0, 0, 1};
    b.begin = 140;
    b.length = 40;
    t.spans = {a, b};
    t.length = 180;
    return t;
}
} // namespace
int main() {
    try {
        const auto hit = assessClearance(crossing(4), 210, 1);
        require(hit.trackHits > 0, "Crossing occupied volumes were accepted");
        // The swept body only overlaps adjacent cars locally. A crossing merely
        // 20 m away along the route still belongs in the nonlocal check.
        auto nearby = crossing(4);
        nearby.spans[0].origin = {-5, -500, 4};
        nearby.spans[0].p[1] = {10, 0, 0};
        nearby.spans[0].length = 10;
        nearby.spans[1].origin = {0, -505, 4};
        nearby.spans[1].p[1] = {0, 10, 0};
        nearby.spans[1].begin = 20;
        nearby.spans[1].length = 10;
        nearby.length = 30;
        require(assessClearance(nearby, 210, 1).trackHits > 0, "Nearby nonadjacent crossing was skipped");
        const auto clear = assessClearance(crossing(13), 210, 1);
        require(!clear.trackHits && !clear.terrainHits && clear.continuous,
                "Separated crossing should certify");
        auto buried = crossing(13);
        buried.spans[0].origin.z = -2;
        require(assessClearance(buried, 210, 1).terrainHits > 0, "Buried occupied volume was accepted");
        Frame upright;
        upright.t = {1, 0, 0};
        upright.u = {0, 0, 1};
        upright.r = {0, -1, 0};
        require(norm(vehicleVector(upright, {1, 2, 3}) - Vec3{1, 2, 3}) < 1e-12,
                "Vehicle clearance basis differs from the renderer's forward/up transform");
        const auto &vehicle = vehicleGeometry();
        require(vehicle.parts.size() == 35 && vehicle.occupied.size() == 39,
                "Vehicle render/occupied components are incomplete");
        for (const auto &part : vehicle.occupied)
            for (int corner = 0; corner < 8; ++corner) {
                Vec3 point = part.center;
                for (std::size_t axis = 0; axis < 3; ++axis)
                    point += part.axes[axis] * (part.half[axis] * ((corner & (1 << axis)) ? 1 : -1));
                require(std::abs(point.x) <= 1.90000001 && std::abs(point.y) <= 1.35000001 &&
                            point.z >= -.90000001 && point.z <= 3.00000001,
                        "A rendered/occupied vehicle part escapes the broad continuous bounds");
            }
        const auto reference = crossing(13);
        const ObstacleIndex obstacles(reference);
        require(!obstacles.assess(beamObstacle("intruding-post", {0, -500, 0}, {0, -500, 10}, .23)).clear,
                "A support through the occupied car was accepted");
        require(obstacles.assess(beamObstacle("clear-post", {-10, -507, 0}, {-10, -507, 10}, .23)).clear,
                "A separated support was rejected");
        require(!obstacles
                     .assess(beamObstacle("thin-between-samples", {-7.137, -502, 5}, {-7.137, -498, 5}, .002))
                     .clear,
                "A thin obstruction between spatial samples was missed");
        require(!obstacles
                     .assess({"platform-too-close",
                              {-10, -498.152, 5},
                              {{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}},
                              {1, .25, .1}})
                     .clear,
                "Object clearance margin was weakened");
        require(
            obstacles
                .assess(
                    {"platform-clear", {-10, -498.14, 5}, {{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}}, {1, .25, .1}})
                .clear,
            "A certified platform gap was rejected");
        bool objectCancelled = false;
        try {
            obstacles.assess(beamObstacle("cancel", {0, -500, 0}, {0, -500, 10}, .23), [] { return true; });
        } catch (const Cancelled &) {
            objectCancelled = true;
        }
        require(objectCancelled, "Static scene clearance ignored cancellation");
        bool cancelled = false;
        try {
            assessClearance(crossing(13), 210, 1, [] { return true; });
        } catch (const Cancelled &) {
            cancelled = true;
        }
        require(cancelled, "Swept check ignored cancellation");
        std::cout << "PASS continuous occupied-volume crossings, terrain and cancellation\n";
        return 0;
    } catch (const std::exception &e) {
        std::cerr << e.what() << '\n';
        return 1;
    }
}
