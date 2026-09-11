#pragma once
#include "drive_force.hpp"
#include <stdexcept>

namespace coaster::detail {
// Flat work estimate using the actual force/power/governor law. The complete
// train, entry ramp and persisted exit fade each retain physical rail space.
// Terrain composition and the finite-train 960/1920 Hz replay remain decisive.
inline double plannedBoostLength(double entrySpeed,const Operation& motor,const TrainConfig& train){
    const double finish=std::max(0.,motor.targetSpeed-.5);
    const double drag=.5*train.airDensity*train.dragCdA/(train.cars*train.carMass);
    double length=(train.cars-1)*train.spacing+motor.targetSpeed*motor.rampSeconds+motor.exitFadeMeters;
    constexpr int intervals=128;
    const double dv=std::max(0.,finish-entrySpeed)/intervals;
    for(int i=0;i<intervals&&dv>0;++i){
        const double v=entrySpeed+(i+.5)*dv;
        const double acceleration=appliedDriveForce(motor,train.carMass,v,motor.rampSeconds,0,motor.exitFadeMeters,0)/train.carMass
            -gravity*train.rollingResistance-drag*v*v;
        if(acceleration<=0)throw std::runtime_error("Booster cannot reach its requested speed with the declared force and power");
        length+=v*dv/acceleration;
    }
    return length;
}
}
