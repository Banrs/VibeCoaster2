#pragma once
#include "coaster/coaster.hpp"

namespace coaster {
// Reduced eddy-current model: linear near rest, a finite-speed peak, then 1/v.
// peakForce is the measured/rated peak per car, not a commanded deceleration.
// Prototype coefficients require hardware calibration; this is not a stop/hold brake.
inline double eddyBrakeForce(double speed,double peakForce,double peakSpeed) {
    const double ratio=speed/peakSpeed;
    return -peakForce*2*ratio/(1+ratio*ratio);
}
inline double trimFieldCoverage(const Operation& op,double carDistance) {
    return smooth((carDistance-op.start)/op.exitFadeMeters)*smooth((op.end-carDistance)/op.exitFadeMeters);
}
// Upstream excess kinetic energy selects the effective exposed magnet area.
// The command is latched before arrival; no in-zone speed clamping or chatter.
inline double trimDeployment(const Operation& op,double sensedSpeed,double carMass) {
    const double excess=.5*std::max(0.,sensedSpeed*sensedSpeed-op.targetSpeed*op.targetSpeed);
    const double capacity=-eddyBrakeForce(sensedSpeed,op.maxForce,op.trimPeakSpeed)/carMass*(op.end-op.start-op.exitFadeMeters);
    // Reserve magnet area for changing speed through a descending field and
    // continued excess energy after it. The latched command and rated force
    // remain bounded; nominal zero excess still leaves the fins retracted.
    return capacity>0?std::clamp(1.25*excess/capacity,0.,1.):0;
}
}
