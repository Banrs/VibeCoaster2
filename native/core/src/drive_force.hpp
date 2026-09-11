#pragma once
#include "coaster/coaster.hpp"

namespace coaster::detail {
// Inputs are the validated operation and the current car's committed entry
// time. Membership is checked by the caller, including wrapped intervals.
inline double appliedDriveForce(const Operation& op,double carMass,double speed,double time,
    double entry,double remaining,double stoppingDistance){
    const double ramp=smooth((time-(entry<0?time:entry))/std::max(.001,op.rampSeconds))*smooth(remaining/op.exitFadeMeters);
    double target=op.targetSpeed;
    if(op.kind==DriveKind::Station)target=std::sqrt(2*op.stopDeceleration*std::max(0.,stoppingDistance-op.stopOffset));
    // Feed the stopping profile's changing velocity into its tracking control.
    // A proportional controller alone must lag a descending target and can
    // overrun the station when the same physical profile uses stronger brakes.
    // dv_target/dt = -deceleration*speed/target; below 1 m/s the denominator
    // stays bounded as the distance profile reaches zero. Force caps still apply.
    const double feedForward=op.kind==DriveKind::Station?
        -op.stopDeceleration*speed/std::max(target,1.):0;
    const double demand=((target-speed)*4+feedForward)*carMass;
    const double cap=std::min(op.maxForce,op.maxPower/std::max(1.,speed));
    return (op.kind==DriveKind::Launch||op.kind==DriveKind::Boost?
        std::clamp(demand,0.,cap):std::clamp(demand,-cap,0.))*ramp;
}
}
