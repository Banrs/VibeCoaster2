#pragma once
#include "coaster/coaster.hpp"
#include <stdexcept>

namespace coaster {
// Signed physical rotation about tangent, right = tangent x up, and up.
// These are body-axis components, not Euler-angle derivatives. Consequently
// they remain defined at vertical tangents and across atan2 branch cuts.
enum class AngularAxis { Roll, Pitch, Yaw };
enum class AngularDerivative { Velocity, Acceleration, Jerk };
enum class AngularDirection { Nonnegative, Nonpositive, Stationary };
struct SignedAngularMotion {
    std::array<double,3> velocity{},acceleration{},jerk{};
};
inline SignedAngularMotion signedAngularMotion(const TrackKinematics& k,double speed,double acceleration,double accelerationRate) {
    if(!std::isfinite(speed)||speed<0||!std::isfinite(acceleration)||!std::isfinite(accelerationRate))
        throw std::invalid_argument("Angular motion requires finite forward replay dynamics");
    const auto& q=k.sample;
    const Vec3 right=cross(q.tangent,q.up);
    const Vec3 rightS=cross(q.curvature,q.up)+cross(q.tangent,k.upS);
    const Vec3 rightSS=cross(k.curvatureS,q.up)+cross(q.curvature,k.upS)*2+cross(q.tangent,k.upSS);
    const double twist=dot(k.upS,right),twistS=dot(k.upSS,right)+dot(k.upS,rightS);
    const double twistSS=dot(k.upSSS,right)+2*dot(k.upSS,rightS)+dot(k.upS,rightSS);
    // Darboux-vector derivatives use the complete frame, including authored
    // twist; curvature/torsion alone cannot describe a coaster's physical roll.
    const Vec3 w=cross(q.tangent,q.curvature)+q.tangent*twist;
    const Vec3 ws=cross(q.tangent,k.curvatureS)+q.curvature*twist+q.tangent*twistS;
    const Vec3 wss=cross(q.curvature,k.curvatureS)+cross(q.tangent,k.curvatureSS)+
        k.curvatureS*twist+q.curvature*(2*twistS)+q.tangent*twistSS;
    const Vec3 velocity=w*speed,alpha=ws*(speed*speed)+w*acceleration;
    const Vec3 worldJerk=wss*(speed*speed*speed)+ws*(3*speed*acceleration)+w*accelerationRate;
    // d(Q^T omega)/dt = Q^T alpha; differentiating again requires the
    // rotating-basis term. Projecting world angular jerk alone is incorrect.
    const Vec3 bodyJerkWorld=worldJerk-cross(velocity,alpha);
    if(!finite(velocity)||!finite(alpha)||!finite(bodyJerkWorld))throw std::invalid_argument("Nonfinite physical angular motion");
    SignedAngularMotion result;const std::array<Vec3,3> axes{q.tangent,right,q.up};
    for(size_t i=0;i<axes.size();++i){result.velocity[i]=dot(velocity,axes[i]);result.acceleration[i]=dot(alpha,axes[i]);result.jerk[i]=dot(bodyJerkWorld,axes[i]);}
    return result;
}
struct AngularPhaseIntent {
    std::string id;
    double beginDistance{},endDistance{};
    AngularAxis axis{AngularAxis::Roll};
    AngularDerivative derivative{AngularDerivative::Velocity};
    AngularDirection direction{AngularDirection::Stationary};
    double numericalTolerance{1e-8}; // rad/s, rad/s^2 or rad/s^3, respectively.
};
struct AngularPhaseAssessment {
    double minimum{INFINITY},maximum{-INFINITY},minimumDistance{},maximumDistance{};
    size_t samples{};
};
struct AngularAuditSettings {
    double maximumDistanceStep{.25},maximumTimeStep{1./240};
    size_t maximumSamples{2000000};
};
struct AngularPhaseAudit {
    ValidationReport report;
    std::vector<AngularPhaseAssessment> phases;
    bool performed{},cancelled{};
};
// This is an independently sampled intent check, not a continuous interval
// proof or a manufacturer comfort limit. Intent must come from authoring; do
// not manufacture phases from the generated extrema. Unlisted channels and
// intervals are deliberately unassessed. Refine both sampling bounds when
// comparing acceptance results, just as for the physical replay itself.
inline AngularPhaseAudit assessAngularPhases(const Track& track,const std::vector<Frame>& replay,
        const std::vector<AngularPhaseIntent>& intent,const AngularAuditSettings& settings={},Cancel cancel={}) {
    AngularPhaseAudit result;
    auto invalid=[&](const char* message){result.report.fail("ANGULAR_PHASE_INPUT",message);return result;};
    if(track.spans.empty()||replay.size()<2||intent.empty()||intent.size()>1024||
       !std::isfinite(settings.maximumDistanceStep)||settings.maximumDistanceStep<=0||
       !std::isfinite(settings.maximumTimeStep)||settings.maximumTimeStep<=0||settings.maximumSamples<2)
        return invalid("Angular phase audit needs a track, forward replay, explicit phases and finite positive sampling bounds");
    for(size_t i=0;i<replay.size();++i){const auto& f=replay[i];
        if(!std::isfinite(f.time)||!std::isfinite(f.distance)||!std::isfinite(f.speed)||f.speed<0||
           !std::isfinite(f.acceleration)||!std::isfinite(f.accelerationRate)||
           (i&&(f.time<=replay[i-1].time||f.distance<replay[i-1].distance)))
            return invalid("Angular phase replay must have finite, increasing time and nondecreasing forward distance");
    }
    for(const auto& phase:intent)
        if(phase.id.empty()||!std::isfinite(phase.beginDistance)||!std::isfinite(phase.endDistance)||
           phase.beginDistance<0||phase.endDistance<=phase.beginDistance||phase.endDistance>track.length||
           phase.beginDistance<replay.front().distance||phase.endDistance>replay.back().distance||
           int(phase.axis)<0||int(phase.axis)>2||int(phase.derivative)<0||int(phase.derivative)>2||
           int(phase.direction)<0||int(phase.direction)>2||!std::isfinite(phase.numericalTolerance)||phase.numericalTolerance<0)
            return invalid("Angular phases must explicitly identify a valid covered interval, axis, derivative, direction and numerical tolerance");
    result.performed=true;result.phases.resize(intent.size());size_t totalSamples=0;
    try {
        for(size_t phaseIndex=0;phaseIndex<intent.size();++phaseIndex){
            const auto& phase=intent[phaseIndex];auto& assessment=result.phases[phaseIndex];size_t hint=track.spans.size();
            auto evaluate=[&](const ReplayMotion& motion){
                if(totalSamples>=settings.maximumSamples)throw std::runtime_error("Angular phase audit exceeded its explicit sample budget");
                if((totalSamples++&255)==0&&cancel&&cancel()){result.cancelled=true;throw std::runtime_error("Angular phase audit cancelled");}
                if(!std::isfinite(motion.distance)||motion.distance<phase.beginDistance-1e-7||motion.distance>phase.endDistance+1e-7)
                    throw std::runtime_error("Angular replay left its authored phase interval");
                const auto at=track.locate(motion.distance,hint);
                const auto measured=signedAngularMotion(sampleSpanKinematics(track,at.span,at.parameter),motion.speed,motion.acceleration,motion.jerk);
                const auto& channel=phase.derivative==AngularDerivative::Velocity?measured.velocity:
                    phase.derivative==AngularDerivative::Acceleration?measured.acceleration:measured.jerk;
                const double value=channel[size_t(phase.axis)];++assessment.samples;
                if(value<assessment.minimum){assessment.minimum=value;assessment.minimumDistance=motion.distance;}
                if(value>assessment.maximum){assessment.maximum=value;assessment.maximumDistance=motion.distance;}
            };
            auto high=std::upper_bound(replay.begin(),replay.end(),phase.beginDistance,[](double s,const Frame& f){return s<f.distance;});
            size_t frame=high==replay.begin()?0:size_t(high-replay.begin()-1);
            for(;frame+1<replay.size()&&replay[frame].distance<=phase.endDistance;++frame){
                const auto& a=replay[frame];const auto& b=replay[frame+1];
                if(b.distance<phase.beginDistance)continue;
                auto timeAt=[&](double distance){
                    if(distance<=a.distance)return a.time;if(distance>=b.distance)return b.time;
                    double low=a.time,upper=b.time;
                    for(int i=0;i<48;++i){const double mid=(low+upper)*.5;
                        if(interpolateMotion(a,b,mid).distance<distance)low=mid;else upper=mid;}
                    return (low+upper)*.5;
                };
                const double begin=std::max(phase.beginDistance,a.distance),end=std::min(phase.endDistance,b.distance);
                if(end<begin)continue;
                const double first=begin==a.distance?a.time:timeAt(begin),last=end==b.distance?b.time:timeAt(end);
                const double timeCount=std::max(1.,std::ceil((last-first)/settings.maximumTimeStep));
                const double distanceCount=std::max(1.,std::ceil((end-begin)/settings.maximumDistanceStep));
                if(!std::isfinite(timeCount)||!std::isfinite(distanceCount)||timeCount+distanceCount+2>double(settings.maximumSamples-totalSamples))
                    throw std::runtime_error("Angular phase audit exceeded its explicit sample budget");
                // Merge independent uniform time and distance grids. Uniform
                // time alone can skip a short feature when speed is changing.
                std::vector<double> times;times.reserve(size_t(timeCount+distanceCount+2));
                for(size_t i=0;i<=size_t(timeCount);++i)times.push_back(std::lerp(first,last,double(i)/timeCount));
                for(size_t i=1;i<size_t(distanceCount);++i)times.push_back(timeAt(std::lerp(begin,end,double(i)/distanceCount)));
                std::sort(times.begin(),times.end());times.erase(std::unique(times.begin(),times.end()),times.end());
                double previous=begin;
                for(double time:times){const auto motion=interpolateMotion(a,b,time);
                    if(motion.distance<previous-1e-9)throw std::runtime_error("Angular replay reverses inside a presentation interval");
                    previous=motion.distance;evaluate(motion);
                }
                if(b.distance>=phase.endDistance)break;
            }
            if(!assessment.samples)throw std::runtime_error("Angular phase has no replay samples");
            const bool low=phase.direction!=AngularDirection::Nonpositive&&assessment.minimum< -phase.numericalTolerance;
            const bool highViolation=phase.direction!=AngularDirection::Nonnegative&&assessment.maximum>phase.numericalTolerance;
            if(low||highViolation){const bool useLow=low&&(!highViolation||-assessment.minimum>=assessment.maximum);
                result.report.fail("ANGULAR_PHASE_DIRECTION",phase.id+": physical angular motion contradicts its authored phase",
                    useLow?assessment.minimumDistance:assessment.maximumDistance,useLow?assessment.minimum:assessment.maximum,phase.numericalTolerance);}
        }
    }catch(const std::exception& e){result.report.fail(result.cancelled?"CANCELLED":"ANGULAR_PHASE_REPLAY",e.what());}
    return result;
}
}
