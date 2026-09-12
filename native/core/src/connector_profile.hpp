#pragma once
#include "coaster/coaster.hpp"
#include <array>
#include <stdexcept>
namespace coaster::detail {
// Reference planar crest, z=H sin^4(pi*s/L), has apex curvature
// -4*pi^2*H/L^2. Combining v_apex^2=v_entry^2-2*g*H with a desired
// normal-load reduction gives the smaller physical quadratic root below.
// This is a source sizing estimate, not a promised finite-train rider load.
inline double connectorCrestHeight(double length,double entrySpeed){
    if(!std::isfinite(length)||length<=0||length>100000||!std::isfinite(entrySpeed)||entrySpeed<=0||entrySpeed>250)throw std::invalid_argument("Connector requires finite physical length and speed");
    const double v2=entrySpeed*entrySpeed,v4=v2*v2;
    const double maximumDelta=pi*pi*v4/(2*gravity*gravity*length*length);
    const double delta=std::min(.85,maximumDelta*.8);
    const double discriminant=v4-2*delta*gravity*gravity*length*length/(pi*pi);
    return std::min(48.,(v2-std::sqrt(discriminant))/(4*gravity));
}
// Value and first three normalized-coordinate derivatives. Zero port jets
// permit adjacent crests without a flat holding segment. Small monotone warp
// changes proportions; actual geometry/forces must pass canonical full replay.
inline std::array<double,4> connectorProfileJet(double u,double shape){
    if(!std::isfinite(u)||u<0||u>1||!std::isfinite(shape)||std::abs(shape)>.1)throw std::invalid_argument("Connector profile lies outside its authored domain");
    if(u==0||u==1)return {};
    const double phase=2*pi*u,q=u+shape*std::sin(phase)/(2*pi),q1=1+shape*std::cos(phase),q2=-2*pi*shape*std::sin(phase),q3=-4*pi*pi*shape*std::cos(phase);
    const double s=std::sin(pi*q),c=std::cos(pi*q),f=std::pow(s,4),f1=4*pi*s*s*s*c,f2=4*pi*pi*(3*s*s*c*c-f),f3=4*pi*pi*pi*(6*s*c*c*c-10*s*s*s*c);
    return {f,f1*q1,f2*q1*q1+f1*q2,f3*q1*q1*q1+3*f2*q1*q2+f1*q3};
}
}
