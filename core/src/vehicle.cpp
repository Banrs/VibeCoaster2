#include "coaster/vehicle.hpp"

namespace coaster {
const VehicleGeometry &vehicleGeometry() {
    static const VehicleGeometry model = [] {
        VehicleGeometry result;
        const std::array<Vec3, 3> axes{{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}};
        auto box = [&](std::string id, Vec3 center, std::array<double, 3> half, VehicleMaterial material) {
            ClearanceObstacle shape{std::move(id), center, axes, half};
            result.parts.push_back({shape, material, false});
            result.occupied.push_back(std::move(shape));
        };
        auto beam = [&](std::string id, Vec3 from, Vec3 to, double radius, VehicleMaterial material) {
            auto shape = beamObstacle(std::move(id), from, to, radius);
            result.parts.push_back({shape, material, true});
            result.occupied.push_back(std::move(shape));
        };
        box("body", {0, 0, .17}, {1.5, 1.05, .38}, VehicleMaterial::Shell);
        int seat = 0;
        for (double x : {-.7, .7})
            for (double y : {-.57, .57}) {
                const std::string id = "seat-" + std::to_string(seat++);
                box(id + "-base", {x, y, .62}, {.44, .38, .14}, VehicleMaterial::Seat);
                box(id + "-lower-back", {x - .55, y, 1.04}, {.12, .39, .30}, VehicleMaterial::Seat);
                box(id + "-upper-back", {x - .55, y, 1.37}, {.12, .29, .18}, VehicleMaterial::Seat);
                box(id + "-headrest", {x - .55, y, 1.64}, {.12, .18, .17}, VehicleMaterial::Seat);
                beam(id + "-bar", {x + .3, y - .28, .86}, {x + .3, y + .28, .86}, .055,
                     VehicleMaterial::Metal);
                result.occupied.push_back({id + "-occupant", {x, y, 1.6}, axes, {.65, .78, 1.4}});
            }
        int wheel = 0;
        for (double x : {-1.1, 1.1})
            for (double side : {-1., 1.}) {
                const std::string id = "wheel-" + std::to_string(wheel++);
                const double y = .65 * side;
                // Running, upstop and side rollers share the model's 105 mm
                // rail radius. Their geometry is included in the swept proof.
                beam(id + "-running", {x, y - .08, .275}, {x, y + .08, .275}, .17, VehicleMaterial::Metal);
                beam(id + "-upstop", {x, y - .07, -.245}, {x, y + .07, -.245}, .14, VehicleMaterial::Metal);
                beam(id + "-side", {x, side * .895, -.065}, {x, side * .895, .065}, .14,
                     VehicleMaterial::Metal);
            }
        for (double side : {-1., 1.})
            beam(side < 0 ? "rear-coupler" : "front-coupler", {side * 1.5, 0, .05}, {side * 1.9, 0, .05}, .08,
                 VehicleMaterial::Metal);
        return result;
    }();
    return model;
}
} // namespace coaster