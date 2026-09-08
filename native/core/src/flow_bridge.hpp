#pragma once
#include "coaster/coaster.hpp"
#include <array>
#include <stdexcept>
namespace coaster::detail {
// Unique septic matching position and the first three arc-length derivatives
// at both ports. Its parameter is normalized; the supplied length carries all
// derivative units. Subsequent canonical compilation and replay remain required.
inline std::array<Vec3,8> flowBridgePolynomial(const TrackKinematics& a,const TrackKinematics& b,double span){
    if(!std::isfinite(span)||span<=0)throw std::invalid_argument("Flow bridge requires a finite positive span");
    for(const auto* port:{&a,&b})if(!finite(port->sample.position)||!finite(port->sample.tangent)||!finite(port->sample.curvature)||!finite(port->curvatureS))throw std::invalid_argument("Flow bridge requires finite endpoint jets");
    std::array<Vec3,8> c{};c[0]=a.sample.position;c[1]=a.sample.tangent*span;c[2]=a.sample.curvature*(span*span/2);c[3]=a.curvatureS*(span*span*span/6);
    const Vec3 p=b.sample.position-c[0]-c[1]-c[2]-c[3],v=b.sample.tangent*span-c[1]-c[2]*2-c[3]*3,acc=b.sample.curvature*(span*span)-c[2]*2-c[3]*6,j=b.curvatureS*(span*span*span)-c[3]*6;
    c[4]=p*35-v*15+acc*2.5-j/6;c[5]=p*(-84)+v*39-acc*7+j*.5;c[6]=p*70-v*34+acc*6.5-j*.5;c[7]=p*(-20)+v*10-acc*2+j/6;
    for(const auto& coefficient:c)if(!finite(coefficient))throw std::invalid_argument("Flow bridge endpoint scaling overflowed");
    return c;
}
}
