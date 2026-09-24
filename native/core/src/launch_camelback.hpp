#pragma once
#include "coaster/fvd.hpp"
#include "coaster/recipe.hpp"

namespace coaster {
struct LaunchCamelback {
    FvdHillResult pullout;
    FvdCamelbackResult camelback;
    double retainedBeginDistance{}, referenceEntrySpeed{};
};
// One monotone force ramp leaves the straight downhill LSM and meets the
// reference hill at its first positive-load shoulder. From that shoulder
// through the crest/descent/recovery, the reference source is unchanged.
LaunchCamelback designLaunchCamelback(double speed,double grade,
    const CamelbackParameters&,double rollingAcceleration,double dragCoefficient,Cancel={});
}
