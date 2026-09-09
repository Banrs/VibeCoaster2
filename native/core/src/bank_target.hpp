#pragma once
#include "coaster/coaster.hpp"

namespace coaster::detail {
// Banking aligns the force axis, not necessarily its positive direction:
// negative normal load remains airtime. Choose the axis nearest the authored
// orientation so a tiny lateral component cannot request a half-turn.
inline double forceAxisBank(double normal,double lateral,double authored){
    if(std::hypot(normal,lateral)<=1e-5)return std::clamp(authored,-1.5,1.5);
    const double angle=std::atan2(lateral,normal);
    return std::clamp(authored+std::remainder(angle-authored,pi),-1.5,1.5);
}
}
