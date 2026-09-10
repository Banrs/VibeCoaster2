#pragma once
#include "coaster/coaster.hpp"

namespace coaster::detail {
// Banking aligns the force axis, not necessarily its positive direction:
// negative normal load remains airtime. Choose the axis nearest the authored
// orientation so a tiny lateral component cannot request a half-turn.
// A curved crest can require overbanking; clipping its axis leaves a real
// lateral force. Canonical rider forces and frame derivatives still gate it.
inline double forceAxisBank(double normal,double lateral,double authored){
    if(std::hypot(normal,lateral)<=1e-5)return authored;
    const double angle=std::atan2(lateral,normal);
    return authored+std::remainder(angle-authored,pi);
}
}
