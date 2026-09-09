#pragma once
#include "coaster/coaster.hpp"

namespace coaster::detail {
// Samples are authored in the plan-local frame, before terrain composition.
// Keep a reversing pair together: its dive entry is intentionally elevated.
struct TerrainModuleFootprint {
    std::vector<Vec3> samples;
    Vec3 entrance;
};
struct TerrainModulePlacement {
    double minimumDatum{},entranceGroundHeight{},meanExcessPedestalHeight{},score{};
};
// A cheap placement ranking estimate, not a clearance certificate. The complete
// composed train envelope and support layout still decide physical acceptance.
inline TerrainModulePlacement assessTerrainModulePlacement(const Terrain& terrain,Vec3 origin,double heading,
    const TerrainModuleFootprint& footprint,double clearance){
    const double c=std::cos(heading),s=std::sin(heading);
    auto ground=[&](Vec3 p){return terrain.height(origin.x+c*p.x-s*p.y,origin.y+s*p.x+c*p.y);};
    const double entranceGround=ground(footprint.entrance);
    TerrainModulePlacement result;
    result.minimumDatum=entranceGround+clearance-footprint.entrance.z;
    for(const auto& p:footprint.samples)result.minimumDatum=std::max(result.minimumDatum,ground(p)+clearance-p.z);
    result.entranceGroundHeight=result.minimumDatum+footprint.entrance.z-entranceGround;
    // Charge the unnecessary pedestal below the authored entry datum, not the
    // intrinsic height of a record element or the high connector in a reversal.
    for(const auto& p:footprint.samples)
        result.meanExcessPedestalHeight+=std::max(0.,result.minimumDatum+footprint.entrance.z-ground(p)-24.);
    if(!footprint.samples.empty())result.meanExcessPedestalHeight/=footprint.samples.size();
    result.score=2*std::max(0.,result.entranceGroundHeight-24.)+result.meanExcessPedestalHeight;
    return result;
}
}
