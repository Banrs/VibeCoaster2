#pragma once
#include <stdexcept>

namespace coaster {
// An explicit saved choice; changing the art must never reinterpret an old ride.
enum class TrackProfile { Legacy, Exa };
struct TrackSection {
    double gauge,railRadius,spineDepth,spineRadius;
    double tieHalfWidth,tieHeight,tieRadius,tieSpacing;
    double hardwareRadius,trainWidth;
    constexpr double bottom() const {return -spineDepth-spineRadius;}
    constexpr double hardwarePadding() const {return hardwareRadius==.9?.06:.02+.04*hardwareRadius+.004;}
};
inline TrackSection trackSection(TrackProfile profile){
    switch(profile){
    case TrackProfile::Legacy:return {1.3,.085,.55,.16,.825,-.19,.08,3.,.9,1.9};
    case TrackProfile::Exa:return {1.4,.105,.8,.34,.62,0,.065,1.4,1.25,2.35};
    }
    throw std::invalid_argument("Unknown track section profile");
}
}
