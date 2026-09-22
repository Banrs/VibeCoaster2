#include "coaster/simulation.hpp"
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
