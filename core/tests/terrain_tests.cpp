#include "coaster/terrain.hpp"
#include <iostream>
using namespace coaster;
namespace {
void require(bool value, const char *message) {
    if (!value)
        throw std::runtime_error(message);
}
} // namespace
int main() {
    try {
        for (const auto *axis : {&terrainXCoordinates(), &terrainYCoordinates()}) {
            require(axis->front() == -20000 && axis->back() == 20000, "Terrain does not reach the horizon");
            for (std::size_t i = 1; i < axis->size(); ++i)
                require((*axis)[i] > (*axis)[i - 1], "Terrain coordinates not strictly ordered");
        }
        const std::array<std::array<double, 4>, 7> regions{{{-21.9, -501.85, -17.1, -498.15},
                                                            {-5, -505, 5, -495},
                                                            {71.3, -4.1, 75.8, 3.9},
                                                            {496.25, -103.75, 502.5, -97.5},
                                                            {9000, -12000, 9600, -11600},
                                                            {0, 0, 0, 0},
                                                            {-800, -1650, -795, -1645}}};
        for (const auto &box : regions) {
            const double upper = terrainUpperBound({box[0], box[1], 0}, {box[2], box[3], 0}, 210);
            for (int i = 0; i <= 19; ++i)
                for (int j = 0; j <= 23; ++j) {
                    const double x = std::lerp(box[0], box[2], i / 19.),
                                 y = std::lerp(box[1], box[3], j / 23.);
                    require(ground(x, y) <= upper + 1e-9, "Terrain bound misses the rendered surface");
                }
        }
        for (double x : {-800., 0., 75., 500., 1500.})
            for (double y : {-1650., -500., 0., 15., 800.})
                require(std::abs(ground(x, y) - siteElevation(x, y)) < 1e-9,
                        "Mesh vertex height differs from site model");
        bool rejected = false;
        try {
            ground(20001, 0);
        } catch (const std::runtime_error &) {
            rejected = true;
        }
        require(rejected, "Out-of-site terrain query was accepted");
        std::cout << "PASS shared terrain vertices, triangle bounds, grid boundaries and domain\n";
        return 0;
    } catch (const std::exception &e) {
        std::cerr << e.what() << '\n';
        return 1;
    }
}
