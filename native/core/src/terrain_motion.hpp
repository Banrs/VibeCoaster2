#pragma once
#include "terrain_transfer.hpp"
#include "passive_transfer.hpp"

namespace coaster::detail {
// Signed authoring intent is independent of the final rider-force assessment.
// Powered motion is an upper energy bound from declared force and target speed;
// neither the target nor speculative brake work replaces the incoming state.
struct TerrainMotion {
    double entrySpeed{},rollingAcceleration{},dragAccelerationCoefficient{};
    double minimumNormalG{},maximumNormalG{};
    double driveTargetSpeed{},driveAcceleration{};
};
struct TerrainMotionAssessment {
    bool positiveEnergyBound{true};
    double minimumNormalG{1},maximumNormalG{1},exitSpeedBound{};
};
inline TerrainMotionAssessment assessTerrainMotion(const TerrainTransfer& profile,const TerrainMotion& motion,Cancel cancel={},int intervals=1024){
    for(double v:{motion.entrySpeed,motion.rollingAcceleration,motion.dragAccelerationCoefficient,motion.minimumNormalG,motion.maximumNormalG,motion.driveTargetSpeed,motion.driveAcceleration})
        if(!std::isfinite(v))throw std::invalid_argument("Nonfinite terrain motion intent");
    if(motion.entrySpeed<=0||motion.rollingAcceleration<0||motion.dragAccelerationCoefficient<0||motion.driveTargetSpeed<0||motion.driveAcceleration<0||motion.minimumNormalG>=1||motion.maximumNormalG<=1||intervals<128||intervals>8192)
        throw std::invalid_argument("Terrain motion intent left its physical domain");
    if(cancel&&cancel())throw std::runtime_error("CANCELLED");
    const double activeLength=profile.activeLength>0?profile.activeLength:profile.length;
    const double activeStart=profile.activeLength>0?profile.activeStart:0;
    const bool powered=motion.driveTargetSpeed>0,climb=profile.finish>=profile.start;
    const double initial=motion.entrySpeed*motion.entrySpeed,target=motion.driveTargetSpeed*motion.driveTargetSpeed;
    double passive=initial,driven=initial,previousDistance=0;auto previous=terrainVerticalJet(profile,0);
    TerrainMotionAssessment result;
    auto advance=[&](double distance){
        if(distance==previousDistance)return;
        const auto current=terrainVerticalJet(profile,distance),middle=terrainVerticalJet(profile,(previousDistance+distance)*.5);
        const double ds=(distance-previousDistance)*(std::hypot(1.,previous[1])+4*std::hypot(1.,middle[1])+std::hypot(1.,current[1]))/6;
        const auto passiveStep=pathEnergyStep(ds,current[0]-previous[0],-motion.rollingAcceleration,motion.dragAccelerationCoefficient);
        passive=passive*passiveStep.attenuation+passiveStep.offsetSpeedSquared;
        const auto drivenStep=pathEnergyStep(ds,current[0]-previous[0],(powered?motion.driveAcceleration:0)-motion.rollingAcceleration,motion.dragAccelerationCoefficient);
        driven=driven*drivenStep.attenuation+drivenStep.offsetSpeedSquared;
        const double targetBound=climb?std::max(passive,target):std::max(initial,target)+2*gravity*(profile.start-current[0]);
        const double bound=powered?std::min(driven,targetBound):passive;
        if(!std::isfinite(bound)||bound<=0){result.positiveEnergyBound=false;return;}
        const double gravityNormal=1/std::hypot(1.,current[1]);
        const double normal=gravityNormal+bound*current[2]*std::pow(gravityNormal,3)/gravity;
        result.minimumNormalG=std::min({result.minimumNormalG,gravityNormal,normal});
        result.maximumNormalG=std::max({result.maximumNormalG,gravityNormal,normal});
        result.exitSpeedBound=std::sqrt(bound);previous=current;previousDistance=distance;
    };
    advance(activeStart);
    for(int i=1;i<=intervals&&result.positiveEnergyBound;++i){
        if((i&63)==0&&cancel&&cancel())throw std::runtime_error("CANCELLED");
        const double w=double(i)/intervals;
        const double u=profile.timing==1?w:(climb?1-std::pow(1-w,1/profile.timing):std::pow(w,1/profile.timing));
        advance(activeStart+activeLength*u);
    }
    if(result.positiveEnergyBound)advance(profile.length);
    return result;
}
inline bool terrainMotionFits(const TerrainMotionAssessment& assessment,const TerrainMotion& motion){
    return assessment.positiveEnergyBound&&assessment.minimumNormalG>=motion.minimumNormalG&&assessment.maximumNormalG<=motion.maximumNormalG;
}
inline double minimumTerrainMotionLength(double rise,double maxGrade,TerrainMotion motion,Cancel cancel={}){
    if(!std::isfinite(rise)||!std::isfinite(maxGrade)||maxGrade<=0)throw std::invalid_argument("Invalid terrain motion span");
    // Validate the same physical intent even when no height change is needed.
    assessTerrainMotion({0,0,1,1},motion,cancel);
    if(rise==0)return 0;
    // A preceding level powered interval can reach at most its declared target.
    // This is a conservative active-window inlet bound, never a speed reset.
    motion.entrySpeed=std::max(motion.entrySpeed,motion.driveTargetSpeed);
    double bracketGrade=maxGrade;
    if(motion.minimumNormalG>0){const double floor=(1+motion.minimumNormalG)*.5;bracketGrade=std::min(bracketGrade,std::sqrt(1/(floor*floor)-1));}
    const double gravityFloor=1/std::hypot(1.,bracketGrade);
    const double forceBudget=std::min(motion.maximumNormalG-1,gravityFloor-motion.minimumNormalG);
    const double speedBound=motion.entrySpeed*motion.entrySpeed+2*gravity*std::max(0.,-rise);
    double lo=std::abs(rise)*(35./16)/maxGrade;
    double hi=minimumTerrainWindowLength(std::abs(rise),bracketGrade,forceBudget*gravity/speedBound);
    auto excess=[&](double length){const auto result=assessTerrainMotion({0,rise,length,1},motion,cancel);
        return result.positiveEnergyBound?std::max(motion.minimumNormalG-result.minimumNormalG,result.maximumNormalG-motion.maximumNormalG):INFINITY;};
    double lowExcess=excess(lo),highExcess=excess(hi);int previousSide=0;
    if(highExcess>0)throw TerrainTransferInfeasible("Terrain transfer lacks source or declared motor energy");
    if(lowExcess<=0)return lo;
    // Bracketed Illinois interpolation retains a genuinely feasible high end.
    // Only search weights change; every candidate uses the same physical screen.
    for(int iteration=0;iteration<40;++iteration){
        double middle=(lo*highExcess-hi*lowExcess)/(highExcess-lowExcess);
        if(!(middle>lo&&middle<hi))middle=(lo+hi)*.5;
        const double error=excess(middle);
        if(error>0){lo=middle;lowExcess=error;if(previousSide==1)highExcess*=.5;previousSide=1;}
        else{hi=middle;highExcess=error;if(previousSide==-1)lowExcess*=.5;previousSide=-1;if(-error<1e-8)return hi;}
        if(hi-lo<1e-7)return hi;
    }
    return hi;
}
inline TerrainTransfer fitTerrainMotionWindow(const std::vector<double>& distance,const std::vector<double>& floor,
    double start,double finish,double maxGrade,const TerrainMotion& motion,Cancel cancel={}){
    auto result=placeTerrainWindow(distance,floor,start,finish,minimumTerrainMotionLength(finish-start,maxGrade,motion,cancel),cancel);
    if(!terrainMotionFits(assessTerrainMotion(result,motion,cancel),motion))throw TerrainTransferInfeasible("Terrain window exceeds its signed load intent");
    return result;
}
inline TerrainTransfer fitTerrainMotionTransfer(const std::vector<double>& distance,const std::vector<double>& floor,
    double start,double finish,double maxGrade,const TerrainMotion& motion,Cancel cancel={}){
    auto result=placeTerrainTransfer(distance,floor,start,finish,maxGrade,cancel);
    if(!terrainMotionFits(assessTerrainMotion(result,motion,cancel),motion))throw TerrainTransferInfeasible("Terrain transfer exceeds its signed load intent");
    return result;
}
} // namespace coaster::detail
