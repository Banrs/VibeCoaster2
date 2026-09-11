#pragma once
#include "coaster/coaster.hpp"
#include <stdexcept>
namespace coaster::detail {
struct TerrainTransferInfeasible : std::runtime_error {using std::runtime_error::runtime_error;};
struct TerrainTransfer {
    double start{},finish{},length{},timing{1};
    double activeStart{},activeLength{};
    double progress(double u) const {
        if(u<=0)return 0;if(u>=1)return 1;
        const double w=finish>=start?-std::expm1(timing*std::log1p(-u)):std::exp(timing*std::log(u));
        const double t=std::min(w,1-w),value=t*t*t*t*(35+t*(-84+t*(70-20*t)));
        return w>.5?1-value:value;
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
    const bool climb=rise>=0;const double p=profile.timing,x=climb?1-u:u,w=p==1?x:std::pow(x,p);
    const double slope=rise/length*140*p*std::pow(x,4*p-1)*std::pow(1-w,3);
    const double second=(climb?-1:1)*rise/(length*length)*140*p*std::pow(x,4*p-2)*std::pow(1-w,2)*((4*p-1)-(7*p-1)*w);
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
// height derivatives. Other transfers retain their existing authoring families.
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
    TerrainTransfer result{start,finish,distance.back(),1};
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
// A terrain transfer is a monotone climb/descent with level C3 ports. Moving
// its timing clears an escarpment without raising an artificial summit above
// either port. This authoring family cannot cross a ridge higher than both
// ports; a different placement is required in that case. XY remains authored.
inline TerrainTransfer placeTerrainTransfer(const std::vector<double>& distance,const std::vector<double>& floor,
    double start,double finish,double maxGrade,Cancel cancel={}){
    if(distance.size()<2||distance.size()!=floor.size()||distance.front()!=0||!std::isfinite(start)||!std::isfinite(finish)||
       !std::isfinite(maxGrade)||maxGrade<=0)
        throw std::invalid_argument("Invalid terrain transfer inputs");
    for(size_t i=0;i<distance.size();++i)if(!std::isfinite(distance[i])||!std::isfinite(floor[i])||(i&&distance[i]<=distance[i-1]))
        throw std::invalid_argument("Terrain transfer needs increasing finite horizontal stations");
    TerrainTransfer result{start,finish,distance.back(),1};
    if(floor.front()>start+1e-8||floor.back()>finish+1e-8)throw TerrainTransferInfeasible("Terrain transfer cannot clear its fixed port");
    for(size_t i=1;i+1<distance.size();++i){
        if((i&63)==0&&cancel&&cancel())throw std::runtime_error("CANCELLED");
        if(floor[i]<=result.height(distance[i]))continue;
        if(floor[i]>=std::max(start,finish))throw TerrainTransferInfeasible("Terrain transfer needs a higher port or another route");
        const double target=(floor[i]-start)/(finish-start),u=distance[i]/result.length;
        double lo=0,hi=1;
        for(int j=0;j<52;++j){double w=(lo+hi)*.5,value=w*w*w*w*(35+w*(-84+w*(70-20*w)));if(value<target)lo=w;else hi=w;}
        // Every interior height increases monotonically with timing, for both
        // climbing and descending transfers. The largest required timing is
        // therefore the least departure from the original S7 transfer.
        const double w=(lo+hi)*.5;
        const double timing=finish>start?std::log1p(-w)/std::log1p(-u):std::log(w)/std::log(u);
        result.timing=std::max(result.timing,timing*(1+1e-12));
    }
    const double p=result.timing,rise=std::abs(finish-start);
    // Exact maximum slope of S7(u^p); ascent is its reflected complement.
    const double peakW=(4*p-1)/(7*p-1),peakU=std::pow(peakW,1/p);
    const double grade=rise/result.length*140*p*std::pow(peakU,4*p-1)*std::pow(1-peakW,3);
    if(!std::isfinite(p)||!std::isfinite(grade)||grade>maxGrade)
        throw TerrainTransferInfeasible("Terrain transfer timing exceeds its grade budget");
    for(size_t i=0;i<distance.size();++i)if(result.height(distance[i])+1e-7<floor[i])
        throw std::runtime_error("Terrain transfer did not clear its sampled envelope");
    return result;
}
inline TerrainTransfer fitTerrainTransfer(const std::vector<double>& distance,const std::vector<double>& floor,
    double start,double finish,double maxGrade,double maxVerticalCurvature,Cancel cancel={}){
    if(!std::isfinite(maxVerticalCurvature)||maxVerticalCurvature<=0)throw std::invalid_argument("Invalid terrain curvature budget");
    auto result=placeTerrainTransfer(distance,floor,start,finish,maxGrade,cancel);
    for(int i=1;i<1024;++i){
        if((i&63)==0&&cancel&&cancel())throw std::runtime_error("CANCELLED");
        const double w=double(i)/1024;
        const double u=finish>=start?1-std::pow(1-w,1/result.timing):std::pow(w,1/result.timing);
        const auto jet=terrainVerticalJet(result,result.length*u);
        const double curvature=std::abs(jet[2])/std::pow(1+jet[1]*jet[1],1.5);
        if(!std::isfinite(curvature)||curvature>maxVerticalCurvature)throw TerrainTransferInfeasible("Terrain transfer timing exceeds its vertical curvature budget");
    }
    return result;
}
}
