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
template<class T>inline std::array<T,6> flowFramePolynomial(T a,T aS,T aSS,T b,T bS,T bSS,double span){
    T c1=aS*span,c2=aSS*(span*span*.5),p=b-a-c1-c2,v=bS*span-c1-c2*2,acc=bSS*(span*span)-c2*2;
    return {a,c1,c2,p*10-v*4+acc*.5,p*(-15)+v*7-acc,p*6-v*3+acc*.5};
}
template<class T,size_t N>inline T flowValue(const std::array<T,N>& c,double u){T value=c.back();for(size_t i=N-1;i-->0;)value=value*u+c[i];return value;}
// Replace a permitted transition window while retaining the surrounding
// derivative stencil. Reference up and bank remain separate, so later banking
// correction still operates on the same physical representation.
inline void blendAuthoredJoin(std::vector<AuthoredPoint>& points,size_t first,size_t last,Cancel cancel={}){
    if(first<4||last+5>=points.size()||last<=first)throw std::invalid_argument("Flow join requires four authored guards at each end");
    std::vector<AuthoredPoint> local(points.begin()+first-4,points.begin()+last+5);
    Track source=compile(local,false);const size_t right=last-first+4;
    const double begin=source.spans[4].start,end=source.spans[right].start,span=end-begin;
    auto bankJet=[&](size_t index){const auto& p=source.spans[index];double q=norm(p.c[1]),qu=dot(p.c[1],p.c[2]*2)/q;
        return std::array<double,3>{p.bank[0],p.bank[1]/q,2*p.bank[2]/(q*q)-p.bank[1]*qu/(q*q*q)};};
    const auto ba=bankJet(4),bb=bankJet(right);
    const auto bank=flowFramePolynomial(ba[0],ba[1],ba[2],bb[0],bb[1],bb[2],span);
    for(auto& knot:source.knots)knot.bank=0;
    rebuildFramePolynomials(source);
    const auto a=sampleKinematics(source,begin),b=sampleKinematics(source,end);
    const auto position=flowBridgePolynomial(a,b,span);
    const auto up=flowFramePolynomial(a.sample.up,a.upS,a.upSS,b.sample.up,b.upS,b.upSS,span);
    for(size_t i=first;i<=last;++i){
        if(cancel&&cancel())throw std::runtime_error("CANCELLED");
        double u=(source.spans[i-first+4].start-begin)/span;
        points[i].position=flowValue(position,u);points[i].upHint=flowValue(up,u);points[i].bank=flowValue(bank,u);
        if(!finite(points[i].position)||!finite(points[i].upHint)||!std::isfinite(points[i].bank)||norm(points[i].upHint)<.5)
            throw std::runtime_error("Flow join left its finite reference-frame domain");
    }
}
}
