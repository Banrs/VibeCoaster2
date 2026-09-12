#pragma once
#include "coaster/coaster.hpp"
#include <stdexcept>
namespace coaster::detail {
struct TerrainTransferInfeasible : std::runtime_error {using std::runtime_error::runtime_error;};
struct TerrainTransfer {
    double start{},finish{},length{};
    double activeStart{},activeLength{};
    double progress(double u) const {
        if(u<=0)return 0;if(u>=1)return 1;
        const double t=std::min(u,1-u),value=t*t*t*t*(35+t*(-84+t*(70-20*t)));
        return u>.5?1-value:value;
    }
    double height(double s) const {
        if(s<=0)return start;if(s>=length)return finish;
        if(activeLength>0)return start+(finish-start)*progress((s-activeStart)/activeLength);
        return start+(finish-start)*progress(s/length);
    }
};
inline std::array<double,3> terrainVerticalJet(const TerrainTransfer& profile,double distance){
    const double length=profile.activeLength>0?profile.activeLength:profile.length;
    const double begin=profile.activeLength>0?profile.activeStart:0;
    const double u=(distance-begin)/length,rise=profile.finish-profile.start;
    if(u<=0)return {profile.start,0,0};if(u>=1)return {profile.finish,0,0};
    const double slope=rise/length*140*std::pow(u*(1-u),3);
    const double second=rise/(length*length)*420*std::pow(u*(1-u),2)*(1-2*u);
    return {profile.height(distance),slope,second};
}
// Exact minimum S7 span for the requested geometric grade and curvature.
// Eliminating A=19600*(rise/length)^2 from the curvature-peak equation leaves
// one strictly decreasing scalar function on 0<t=u(1-u)<1/5.
inline double minimumTerrainWindowLength(double rise,double maxGrade,double maxVerticalCurvature){
    if(!std::isfinite(rise)||rise<0||!std::isfinite(maxGrade)||maxGrade<=0||
       !std::isfinite(maxVerticalCurvature)||maxVerticalCurvature<=0)
        throw std::invalid_argument("Invalid terrain window requirement");
    if(rise==0)return 0;
    double lo=0,hi=.2;
    for(int iteration=0;iteration<56;++iteration){
        const double t=(lo+hi)*.5;
        const double scaledCurvature=(2-10*t)*std::sqrt(7-26*t)/(1260*t*t*t*t*(1-4*t));
        if(scaledCurvature>maxVerticalCurvature*rise)lo=t;else hi=t;
    }
    const double curvatureLength=std::sqrt((140./9)*rise/maxVerticalCurvature*hi*hi*std::pow(7-26*hi,1.5)/(1-4*hi));
    return std::max(rise*(35./16)/maxGrade,curvatureLength);
}
}
