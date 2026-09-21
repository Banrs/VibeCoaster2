#pragma once
#include "coaster/coaster.hpp"
#include <array>
#include <complex>
#include <stdexcept>

namespace coaster::detail {
// One physical arc-length jet is shared by adjoining spline/FVD motion. The
// fourth position derivative makes the tangent (and physical frame) C3.
struct MotionJet {Vec3 position,tangent,curvature,third,fourth;};
struct AngleJet {double value{},first{},second{},third{};};
inline std::array<double,8> anglePolynomial(AngleJet a,AngleJet b,double length){
    const double c1=a.first*length,c2=a.second*length*length*.5,c3=a.third*std::pow(length,3)/6;
    const double p=b.value-a.value-c1-c2-c3,v=b.first*length-c1-2*c2-3*c3,k=b.second*length*length-2*c2-6*c3,j=b.third*std::pow(length,3)-6*c3;
    return {a.value,c1,c2,c3,35*p-15*v+2.5*k-j/6,-84*p+39*v-7*k+j*.5,70*p-34*v+6.5*k-j*.5,-20*p+10*v-2*k+j/6};
}
inline AngleJet angleAt(const std::array<double,8>& c,double s,double length){
    const double u=std::clamp(s/length,0.,1.);AngleJet q{c.back()};
    for(int i=6;i>=0;--i){q.third=q.third*u+3*q.second;q.second=q.second*u+2*q.first;q.first=q.first*u+q.value;q.value=q.value*u+c[i];}
    q.first/=length;q.second/=length*length;q.third/=std::pow(length,3);return q;
}
inline std::array<AngleJet,2> directionAngles(const MotionJet& q){
    const double pitch=std::asin(q.tangent.z),c=std::cos(pitch),p1=q.curvature.z/c,p2=(q.third.z+std::sin(pitch)*p1*p1)/c;
    const double p3=(q.fourth.z+c*p1*p1*p1+3*std::sin(pitch)*p1*p2)/c;
    using C=std::complex<double>;const C t(q.tangent.x,q.tangent.y),k(q.curvature.x,q.curvature.y),j(q.third.x,q.third.y),snap(q.fourth.x,q.fourth.y),a=k/t;
    return {{{pitch,p1,p2,p3},{std::atan2(t.imag(),t.real()),a.imag(),(j/t-a*a).imag(),(snap/t-3.*j/t*a+2.*a*a*a).imag()}}};
}
inline AngleJet sinJet(AngleJet x){const double s=std::sin(x.value),c=std::cos(x.value);return {s,c*x.first,c*x.second-s*x.first*x.first,c*x.third-3*s*x.first*x.second-c*x.first*x.first*x.first};}
inline AngleJet cosJet(AngleJet x){x.value+=pi*.5;return sinJet(x);}
inline AngleJet multiply(AngleJet a,AngleJet b){return {a.value*b.value,a.first*b.value+a.value*b.first,a.second*b.value+2*a.first*b.first+a.value*b.second,a.third*b.value+3*a.second*b.first+3*a.first*b.second+a.value*b.third};}
inline MotionJet directionJet(AngleJet pitch,AngleJet yaw){
    const auto x=multiply(cosJet(pitch),cosJet(yaw)),y=multiply(cosJet(pitch),sinJet(yaw)),z=sinJet(pitch);
    return {{},{x.value,y.value,z.value},{x.first,y.first,z.first},{x.second,y.second,z.second},{x.third,y.third,z.third}};
}
template<class F> Vec3 integrateDirection(const F& direction,double begin,double end){
    constexpr double nodes[]{.1834346424956498,.525532409916329,.796666477413627,.960289856497536},weights[]{.362683783378362,.313706645877887,.222381034453374,.101228536290376};
    Vec3 sum{};const double mid=(begin+end)*.5,half=(end-begin)*.5;
    for(int i=0;i<4;++i)sum=sum+(direction(mid-half*nodes[i])+direction(mid+half*nodes[i]))*weights[i];return sum*half;
}
template<class T,size_t N> T polynomial(const std::array<T,N>& c,double u){T v=c.back();for(size_t i=N-1;i-->0;)v=v*u+c[i];return v;}
inline MotionJet reparameterize(Vec3 p,Vec3 d1,Vec3 d2,Vec3 d3,Vec3 d4,double q,double q1,double q2,double q3){
    return {p,d1/q,d2/std::pow(q,2)-d1*(q1/std::pow(q,3)),
        d3/std::pow(q,3)-d2*(3*q1/std::pow(q,4))+d1*(3*q1*q1/std::pow(q,5)-q2/std::pow(q,4)),
        d4/std::pow(q,4)-d3*(6*q1/std::pow(q,5))+d2*(15*q1*q1/std::pow(q,6)-4*q2/std::pow(q,5))+
            d1*(10*q1*q2/std::pow(q,6)-15*q1*q1*q1/std::pow(q,7)-q3/std::pow(q,5))};
}
inline MotionJet arcJet(Vec3 p,Vec3 d1,Vec3 d2,Vec3 d3,Vec3 d4){
    const double q=norm(d1),q1=dot(d1,d2)/q;
    const double q2=(dot(d2,d2)+dot(d1,d3)-q1*q1)/q;
    const double q3=(3*dot(d2,d3)+dot(d1,d4)-3*q1*q2)/q;
    return reparameterize(p,d1,d2,d3,d4,q,q1,q2,q3);
}
inline MotionJet jet(const TrackKinematics& q){return {q.sample.position,q.sample.tangent,q.sample.curvature,q.curvatureS,q.curvatureSS};}
inline MotionJet planarJet(Vec3 p,double heading,double pitch,double curvature=0,double curvatureRate=0){
    const Vec3 t{std::cos(heading)*std::cos(pitch),std::sin(heading)*std::cos(pitch),std::sin(pitch)};
    const Vec3 n{-std::cos(heading)*std::sin(pitch),-std::sin(heading)*std::sin(pitch),std::cos(pitch)};
    return {p,t,n*curvature,n*curvatureRate-t*(curvature*curvature),t*(-3*curvature*curvatureRate)-n*(curvature*curvature*curvature)};
}
// Exact restriction of a normalized polynomial to a smaller arc interval.
// All value/rate/acceleration/jerk data remain the same physical function.
inline std::array<double,8> restrictAngle(const std::array<double,8>& c,double length,double begin,double end){
    std::array<double,8> out{};const double offset=begin/length,scale=(end-begin)/length;
    for(int power=0;power<8;++power){double choose=1;
        for(int j=0;j<=power;++j){out[j]+=c[power]*choose*std::pow(offset,power-j)*std::pow(scale,j);choose*=double(power-j)/(j+1);}}
    return out;
}

}
