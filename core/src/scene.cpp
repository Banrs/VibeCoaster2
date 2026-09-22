#include "coaster/scene.hpp"

namespace coaster {
SceneGeometry authorStructures(const Track &track, double plateau, const Cancel &cancel) {
    const ObstacleIndex index(track, cancel);
    SceneGeometry result;
    auto accept = [&](const std::vector<ScenePart> &parts) {
        double gap = 1e9;
        for (const auto &part : parts) {
            const auto proof = index.assess(part.shape, cancel);
            if (!proof.clear)
                return false;
            gap = std::min(gap, proof.minimumCertifiedGap);
        }
        result.minimumCertifiedGap = std::min(result.minimumCertifiedGap, gap);
        result.parts.insert(result.parts.end(), parts.begin(), parts.end());
        return true;
    };
    auto member = [](const std::string &id, Vec3 from, Vec3 to, double radius,
                     SceneMaterial material = SceneMaterial::Support) {
        return ScenePart{beamObstacle(id, from, to, radius), material, true};
    };
    auto column = [&](const std::string &id, Vec3 top, ScenePart &part) {
        Vec3 base = top;
        base.z = ground(top.x, top.y, plateau);
        const double height = top.z - base.z;
        if (height < .3)
            return false;
        base.z -= .25; // Embed the footing in the unchanged terrain.
        part = member(id, base, top, std::clamp(.2 + .004 * height, .2, 1.1));
        return true;
    };
    std::size_t number = 0;
    for (double target = 8; target < track.length; target += 18, ++number) {
        poll(cancel);
        bool placed = false;
        const std::string id = "support-" + std::to_string(number);
        for (double offset : {0., -3., 3., -6., 6.}) {
            const double s = std::clamp(target + offset, 1., track.length - 1);
            if (!result.supportAnchors.empty() && s - result.supportAnchors.back() < 6)
                continue;
            if (target + 18 >= track.length && !result.supportAnchors.empty() &&
                track.length - s + result.supportAnchors.front() > 30)
                continue;
            const auto f = track.at(s);
            // The connector ends inside the rendered backbone's hexagonal
            // section. Its complete volume is checked against the vehicle.
            const Vec3 attachment = f.p - f.u * .78;
            ScenePart direct;
            if (column(id + "-column", attachment, direct) && accept({direct}))
                placed = true;
            for (double width : {3.4, 5., 7., 10., 14.}) {
                if (placed)
                    break;
                for (double shift : {0., -6., 6., -12., 12.}) {
                    const Vec3 center = f.p - f.u * 1.6 + f.t * shift, left = center + f.r * width,
                               right = center - f.r * width;
                    const auto connector = member(id + "-connector", attachment, center, .18);
                    ScenePart a, b;
                    const bool hasLeft = column(id + "-left", left, a),
                               hasRight = column(id + "-right", right, b);
                    if (hasLeft && hasRight &&
                        accept({connector, a, b, member(id + "-cross", left, right, .24)}))
                        placed = true;
                    else if (hasLeft && accept({connector, a, member(id + "-arm", center, left, .24)}))
                        placed = true;
                    else if (hasRight && accept({connector, b, member(id + "-arm", center, right, .24)}))
                        placed = true;
                    if (placed)
                        break;
                }
            }
            if (placed) {
                result.supportAnchors.push_back(s);
                break;
            }
        }
        if (!placed)
            throw std::runtime_error("No clear connected support assembly at " + std::to_string(target) +
                                     " m in " + track.source[track.at(target).element].id);
    }
    for (std::size_t i = 1; i < result.supportAnchors.size(); ++i)
        if (result.supportAnchors[i] - result.supportAnchors[i - 1] > 30.000001)
            throw std::runtime_error("Support interval exceeded the authored maximum");
    const auto f = track.at(0);
    std::vector<ScenePart> station;
    station.push_back(
        {{"station-platform", f.p - f.t * 10 - f.r * 3.5 - f.u * .5, {f.t, f.r, f.u}, {17, 1.8, .3}},
         SceneMaterial::Platform,
         false});
    int post = 0;
    for (double s : {-25., 3.})
        for (double side : {-5., -2.}) {
            const Vec3 base = f.p + f.t * s + f.r * side - f.u * .2;
            station.push_back(member("station-post-" + std::to_string(post++), base, base + Vec3{0, 0, 4.5},
                                     .13, SceneMaterial::StationFrame));
        }
    for (double s : {-24., 4.})
        for (double side : {-4.8, -2.2}) {
            ScenePart pier;
            if (!column("station-pier-" + std::to_string(post++), f.p + f.t * s + f.r * side - f.u * .65,
                        pier))
                throw std::runtime_error("Station platform cannot reach its fixed ground footing");
            pier.material = SceneMaterial::StationFrame;
            station.push_back(std::move(pier));
        }
    station.push_back(
        {{"station-roof", f.p - f.t * 11 - f.r * 3.5 + f.u * 4.4, {f.t, f.r, f.u}, {16, 2.1, .15}},
         SceneMaterial::Roof,
         false});
    if (!accept(station))
        throw std::runtime_error("Station structures intrude into the occupied vehicle sweep");
    return result;
}
} // namespace coaster