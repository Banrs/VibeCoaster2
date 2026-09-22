#pragma once
#include "coaster/track.hpp"
#include <memory>

namespace coaster {
// An oriented solid enclosing a rendered static object, in metres. A finite
// cylindrical member is conservatively enclosed by beamObstacle().
struct ClearanceObstacle {
    std::string id;
    Vec3 center;
    std::array<Vec3, 3> axes{{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}};
    std::array<double, 3> half{};
};
ClearanceObstacle beamObstacle(std::string id, Vec3 from, Vec3 to, double radius);
struct ObstacleResult {
    bool clear{true};
    double minimumCertifiedGap{1e9}, firstTrackS{-1};
};
// The track must remain alive and immutable while this index is used. Queries
// use the same continuous bounds as terrain/track clearance, then refine
// against the shared rendered vehicle and occupant solids. Static objects
// have no local-distance exclusion and retain the full 250 mm margin.
class ObstacleIndex {
    struct Impl;
    std::shared_ptr<const Impl> data;

  public:
    explicit ObstacleIndex(const Track &, const Cancel &cancel = {});
    ObstacleResult assess(const ClearanceObstacle &, const Cancel &cancel = {}) const;
};
enum class SceneMaterial { Support, Platform, StationFrame, Roof };
struct ScenePart {
    ClearanceObstacle shape;
    SceneMaterial material;
    bool cylinder{};
};
struct SceneGeometry {
    std::vector<ScenePart> parts;
    std::vector<double> supportAnchors;
    double minimumCertifiedGap{1e9};
};
// Every support interval must produce a connected assembly. Failed candidates
// may move or change form; omitting a support is never an acceptance shortcut.
SceneGeometry authorStructures(const Track &, double plateau, const Cancel &cancel = {});
} // namespace coaster