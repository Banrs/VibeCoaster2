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
// A climb or descent owns a bounded active window, independently of the
// distance between its fixed ports. Its level extensions and septic ports share zero first three
// height derivatives. Fixed motor domains use the same unwarped S7 profile.
inline TerrainTransfer placeTerrainWindow(const std::vector<double>& distance,const std::vector<double>& floor,
    double start,double finish,double minimumActiveLength,Cancel cancel={}){
    if(distance.size()<2||distance.size()!=floor.size()||distance.front()!=0||!std::isfinite(start)||!std::isfinite(finish)||
       !std::isfinite(minimumActiveLength)||minimumActiveLength<0)
        throw std::invalid_argument("Invalid terrain window inputs");
    for(size_t i=0;i<distance.size();++i)if(!std::isfinite(distance[i])||!std::isfinite(floor[i])||(i&&distance[i]<=distance[i-1]))
        throw std::invalid_argument("Terrain window needs increasing finite horizontal stations");
    if(cancel&&cancel())throw std::runtime_error("CANCELLED");
    if(finish>start){
        std::vector<double> reversedDistance(distance.size()),reversedFloor(floor.rbegin(),floor.rend());
        for(size_t i=0;i<distance.size();++i)reversedDistance[i]=distance.back()-distance[distance.size()-1-i];
        auto result=placeTerrainWindow(reversedDistance,reversedFloor,finish,start,minimumActiveLength,cancel);
        std::swap(result.start,result.finish);result.activeStart=result.length-result.activeStart-result.activeLength;
        return result;
    }
    TerrainTransfer result{start,finish,distance.back()};
    if(floor.front()>start+1e-8||floor.back()>finish+1e-8)
        throw TerrainTransferInfeasible("Terrain window cannot clear its fixed port");
    const double rise=start-finish;
    std::vector<double> progressLimit(distance.size(),1);
    double longest=result.length;
    for(size_t i=0;i<distance.size();++i){
        if((i&63)==0&&cancel&&cancel())throw std::runtime_error("CANCELLED");
        if(floor[i]>start+1e-8)throw TerrainTransferInfeasible("Terrain window needs a higher port or another route");
        if(floor[i]<=finish||rise==0)continue;
        const double target=std::max(0.,(start-floor[i])/rise);
        double lo=0,hi=1;
        for(int j=0;j<52;++j){double u=(lo+hi)*.5,value=u*u*u*u*(35+u*(-84+u*(70-20*u)));if(value<=target)lo=u;else hi=u;}
        progressLimit[i]=lo;
        // The earliest clear window must also finish inside the fixed domain.
        longest=std::min(longest,(result.length-distance[i])/(1-lo));
    }
    if(rise==0)return result;
    const double shortest=minimumActiveLength;
    if(shortest>longest||!std::isfinite(shortest))
        throw TerrainTransferInfeasible("Terrain window exceeds its grade or vertical curvature budget");
    const auto earliest=[&](double length){
        double offset=0;for(size_t i=0;i<distance.size();++i)if(floor[i]>finish)offset=std::max(offset,distance[i]-length*progressLimit[i]);return offset;
    };
    // Integral height above the fixed floor differs only by rise*(a+L/2).
    // The maximum of affine floor constraints makes this objective convex.
    // Search only the feasible interval; never trade clearance for proximity.
    const auto score=[&](double length){return earliest(length)+length*.5;};
    double lo=shortest,hi=longest;
    for(int j=0;j<64;++j){
        if((j&7)==0&&cancel&&cancel())throw std::runtime_error("CANCELLED");
        double left=(2*lo+hi)/3,right=(lo+2*hi)/3;if(score(left)<score(right))hi=right;else lo=left;
    }
    result.activeLength=(lo+hi)*.5;
    for(double length:{shortest,longest})if(score(length)<score(result.activeLength))result.activeLength=length;
    result.activeStart=earliest(result.activeLength);
    for(size_t i=0;i<distance.size();++i)if(result.height(distance[i])+1e-7<floor[i])
        throw TerrainTransferInfeasible("Terrain window did not clear its sampled envelope");
    return result;
}
inline TerrainTransfer fitTerrainWindow(const std::vector<double>& distance,const std::vector<double>& floor,
    double start,double finish,double maxGrade,double maxVerticalCurvature,Cancel cancel={}){
    return placeTerrainWindow(distance,floor,start,finish,minimumTerrainWindowLength(std::abs(finish-start),maxGrade,maxVerticalCurvature),cancel);
}
}
