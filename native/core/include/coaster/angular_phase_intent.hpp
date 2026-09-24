#pragma once
#include "coaster/angular_motion.hpp"
#include "coaster/fvd.hpp"

namespace coaster {
// Independently authored source-clock intent. Resolution only places these
// intervals on the retained canonical source; it never searches its extrema.
struct FvdAngularPhaseIntent {
    std::string id;
    double beginTime{},endTime{};
    AngularAxis axis{AngularAxis::Roll};
    AngularDerivative derivative{AngularDerivative::Velocity};
    AngularDirection direction{AngularDirection::Stationary};
    double numericalTolerance{1e-8};
};
struct DerivedFvdRollPhases {
    std::vector<FvdAngularPhaseIntent> phases;
    std::vector<std::pair<double,double>> unresolved;
};
namespace angular_detail {
inline double choose(int n,int k){double x=1;for(int i=1;i<=k;++i)x*=double(n+1-i)/i;return x;}
inline std::array<double,7> restrictPolynomial(const std::array<double,7>& c,double offset,double scale){
    std::array<double,7> out{};
    for(int power=0;power<7;++power)for(int j=0;j<=power;++j)
        out[j]+=c[power]*choose(power,j)*std::pow(offset,power-j)*std::pow(scale,j);
    return out;
}
inline void sourceMapping(const Track& final,const ForceAuthoring& source,const FvdResult& rebuilt){
    if(!rebuilt.assessment.passed||rebuilt.samples.size()<2||source.sourceDistances.size()<2||source.firstKnot>=final.knots.size()||
       source.sourceDistances.size()>final.knots.size()-source.firstKnot||std::abs(source.hand)!=1||source.laneShift!=0)
        throw std::invalid_argument("Angular source audit requires a verified rigidly placed FVD source and valid retained mapping");
    double previous=-1;
    for(double s:source.sourceDistances){if(!std::isfinite(s)||s<=previous||s<0||s>rebuilt.track.length+1e-6)
        throw std::invalid_argument("Invalid retained angular source distances");previous=s;}
}
inline double finalDistance(const Track& track,const ForceAuthoring& source,double at){
    auto hi=std::upper_bound(source.sourceDistances.begin(),source.sourceDistances.end(),at);
    const size_t count=source.sourceDistances.size();
    const size_t i=hi==source.sourceDistances.begin()?0:std::min(count-2,size_t(hi-source.sourceDistances.begin()-1));
    auto distance=[&](size_t knot){return knot<track.spans.size()?track.spans[knot].start:track.length;};
    const double fraction=std::clamp((at-source.sourceDistances[i])/(source.sourceDistances[i+1]-source.sourceDistances[i]),0.,1.);
    return std::lerp(distance(source.firstKnot+i),distance(source.firstKnot+i+1),fraction);
}
inline double sourceDistance(const FvdResult& source,double time){
    if(time<source.samples.front().time||time>source.samples.back().time||!std::isfinite(time))throw std::invalid_argument("Angular phase is outside its source clock");
    const auto hi=std::upper_bound(source.samples.begin(),source.samples.end(),time,[](double t,const FvdSample& q){return t<q.time;});
    if(hi==source.samples.end())return source.track.length;
    const size_t i=hi==source.samples.begin()?0:size_t(hi-source.samples.begin()-1);
    const auto& a=source.samples[i];const auto& b=source.samples[i+1];const double duration=b.time-a.time,u=(time-a.time)/duration;
    const double sa=source.track.spans[i].start,sb=i+1<source.track.spans.size()?source.track.spans[i+1].start:source.track.length;
    // Independent integrated source speed supplies a cubic clock mapping.
    // Exact sampled/control boundaries map to exact canonical knot distances.
    return (2*u*u*u-3*u*u+1)*sa+(u*u*u-2*u*u+u)*duration*a.speed+(-2*u*u*u+3*u*u)*sb+(u*u*u-u*u)*duration*b.speed;
}
inline ReplayMotion replayAt(const std::vector<Frame>& replay,double distance){
    if(replay.size()<2||distance<replay.front().distance||distance>replay.back().distance)throw std::invalid_argument("Angular source is not covered by the final physical replay");
    auto hi=std::upper_bound(replay.begin(),replay.end(),distance,[](double s,const Frame& f){return s<f.distance;});
    if(hi==replay.end()){const auto& f=replay.back();return {f.distance,f.speed,f.acceleration,f.accelerationRate};}
    const auto& a=*(hi-1);const auto& b=*hi;double low=a.time,high=b.time;
    for(int i=0;i<40;++i){const double t=(low+high)*.5;if(interpolateMotion(a,b,t).distance<distance)low=t;else high=t;}
    return interpolateMotion(a,b,(low+high)*.5);
}
}
// The existing physical twist/control metadata can prove roll-velocity signs.
// Bernstein bounds include any additive baseline. Opposing/mixed controls are
// explicitly unresolved; gravity-referenced bank is not physical tangent twist.
// No claim about unlisted pitch/yaw or acceleration/jerk phases is implied.
inline DerivedFvdRollPhases deriveFvdRollVelocityPhases(const FvdRequest& request){
    if(request.controls.size()<2)throw std::invalid_argument("Angular intent needs a finite FVD control timeline");
    std::vector<double> times;double previous=-1;
    for(const auto& c:request.controls){if(!std::isfinite(c.time)||c.time<=previous||!std::isfinite(c.rollRate)||!std::isfinite(c.first[2])||!std::isfinite(c.second[2]))throw std::invalid_argument("Invalid angular control timeline");times.push_back(c.time);previous=c.time;}
    const double domainEnd=times.back();previous=times.front();
    for(const auto& p:request.twists){if(!std::isfinite(p.begin)||!std::isfinite(p.end)||!std::isfinite(p.angle)||p.begin<previous||p.end<=p.begin||p.end>domainEnd)throw std::invalid_argument("Invalid angular twist phase");times.push_back(p.begin);times.push_back(p.end);previous=p.end;}
    std::sort(times.begin(),times.end());times.erase(std::unique(times.begin(),times.end()),times.end());DerivedFvdRollPhases result;
    for(size_t i=1;i<times.size();++i){const double begin=times[i-1],end=times[i];
        if(request.gravityReferencedRoll){result.unresolved.push_back({begin,end});continue;}
        std::array<double,7> polynomial{};
        if(request.twists.empty()||request.additiveTwists){
            const auto a=sampleFvdControl(request.controls,begin),b=sampleFvdControl(request.controls,end);const double length=end-begin;
            polynomial[0]=a.rollRate;polynomial[1]=a.first[2]*length;polynomial[2]=a.second[2]*length*length*.5;
            const double p=b.rollRate-polynomial[0]-polynomial[1]-polynomial[2],v=b.first[2]*length-polynomial[1]-2*polynomial[2],q=b.second[2]*length*length-2*polynomial[2];
            polynomial[3]=10*p-4*v+.5*q;polynomial[4]=-15*p+7*v-q;polynomial[5]=6*p-3*v+.5*q;
        }
        for(const auto& phase:request.twists)if(begin>=phase.begin&&end<=phase.end){
            const double duration=phase.end-phase.begin,factor=140*phase.angle/duration;
            const auto twist=angular_detail::restrictPolynomial({0,0,0,factor,-3*factor,3*factor,-factor},(begin-phase.begin)/duration,(end-begin)/duration);
            for(size_t j=0;j<polynomial.size();++j)polynomial[j]+=twist[j];break;
        }
        double low=INFINITY,high=-INFINITY,scale=1;
        for(double c:polynomial)scale=std::max(scale,std::abs(c));
        for(int j=0;j<7;++j){double coefficient=0;for(int k=0;k<=j;++k)coefficient+=polynomial[k]*angular_detail::choose(j,k)/angular_detail::choose(6,k);low=std::min(low,coefficient);high=std::max(high,coefficient);}
        const double roundoff=1e-12*scale;AngularDirection direction;
        if(std::max(std::abs(low),std::abs(high))<=roundoff)direction=AngularDirection::Stationary;
        else if(low>=-roundoff)direction=AngularDirection::Nonnegative;
        else if(high<=roundoff)direction=AngularDirection::Nonpositive;
        else{result.unresolved.push_back({begin,end});continue;}
        result.phases.push_back({"authored-roll-"+std::to_string(i-1),begin,end,AngularAxis::Roll,AngularDerivative::Velocity,direction,1e-7});
    }
    return result;
}
inline std::vector<AngularPhaseIntent> resolveFvdAngularPhases(const Track& track,const ForceAuthoring& source,const FvdResult& rebuilt,const std::vector<FvdAngularPhaseIntent>& phases){
    angular_detail::sourceMapping(track,source,rebuilt);std::vector<AngularPhaseIntent> result;
    for(const auto& p:phases){
        if(p.id.empty()||!std::isfinite(p.beginTime)||!std::isfinite(p.endTime)||p.endTime<=p.beginTime||int(p.axis)<0||int(p.axis)>2||
           int(p.derivative)<0||int(p.derivative)>2||int(p.direction)<0||int(p.direction)>2||!std::isfinite(p.numericalTolerance)||p.numericalTolerance<0)
            throw std::invalid_argument("Invalid independently authored angular phase");
        const double begin=std::max(source.sourceDistances.front(),angular_detail::sourceDistance(rebuilt,p.beginTime));
        const double end=std::min(source.sourceDistances.back(),angular_detail::sourceDistance(rebuilt,p.endTime));
        if(end<=begin)continue;
        auto direction=p.direction;
        if(source.hand<0&&p.axis!=AngularAxis::Pitch&&direction!=AngularDirection::Stationary)
            direction=direction==AngularDirection::Nonnegative?AngularDirection::Nonpositive:AngularDirection::Nonnegative;
        result.push_back({source.name+"/"+p.id,angular_detail::finalDistance(track,source,begin),angular_detail::finalDistance(track,source,end),p.axis,p.derivative,direction,p.numericalTolerance});
    }
    return result;
}
struct FvdAngularAgreement {
    ValidationReport report;
    std::array<double,3> maximumError{},distance{}; // velocity, acceleration, jerk; maximum across signed axes.
    size_t samples{};
    bool cancelled{};
};
// Compare final canonical motion to freshly reintegrated authoring at the SAME
// actual train speed/acceleration/jerk. This tests preservation of all physical
// derivatives without mistaking changed train speed for changed authoring.
inline FvdAngularAgreement assessFvdAngularAgreement(const Track& final,const ForceAuthoring& source,const FvdResult& rebuilt,
        const std::vector<Frame>& replay,const std::array<double,3>& tolerances,unsigned samplesPerSpan=2,Cancel cancel={}){
    FvdAngularAgreement result;
    try{
        angular_detail::sourceMapping(final,source,rebuilt);
        if(samplesPerSpan<1||samplesPerSpan>16)throw std::invalid_argument("Angular agreement sampling must be 1..16 per retained span");
        for(double tolerance:tolerances)if(!std::isfinite(tolerance)||tolerance<0)throw std::invalid_argument("Angular agreement needs explicit finite numerical tolerances");
        const auto& distances=source.sourceDistances;size_t finalHint=final.spans.size(),sourceHint=rebuilt.track.spans.size();
        const size_t intervals=distances.size()-1;
        for(size_t i=0;i<=intervals*samplesPerSpan;++i){
            if((i&255)==0&&cancel&&cancel()){result.cancelled=true;throw std::runtime_error("Angular source agreement cancelled");}
            const size_t span=std::min(intervals-1,i/samplesPerSpan);const double fraction=double(i-span*samplesPerSpan)/samplesPerSpan;
            const double original=std::min(rebuilt.track.length,std::lerp(distances[span],distances[span+1],fraction));
            const double distance=angular_detail::finalDistance(final,source,original);const auto motion=angular_detail::replayAt(replay,distance);
            const auto actualAt=final.locate(distance,finalHint),expectedAt=rebuilt.track.locate(original,sourceHint);
            const auto actual=signedAngularMotion(sampleSpanKinematics(final,actualAt.span,actualAt.parameter),motion.speed,motion.acceleration,motion.jerk);
            const auto expected=signedAngularMotion(sampleSpanKinematics(rebuilt.track,expectedAt.span,expectedAt.parameter),motion.speed,motion.acceleration,motion.jerk);
            const std::array<std::array<double,3>,3> values{actual.velocity,actual.acceleration,actual.jerk},reference{expected.velocity,expected.acceleration,expected.jerk};
            for(size_t order=0;order<3;++order)for(size_t axis=0;axis<3;++axis){const double error=std::abs(values[order][axis]-reference[order][axis]*(axis==1?1:source.hand));
                if(error>result.maximumError[order]){result.maximumError[order]=error;result.distance[order]=distance;}}
            ++result.samples;
        }
        const char* names[]{"velocity","acceleration","jerk"};
        for(size_t order=0;order<3;++order)if(result.maximumError[order]>tolerances[order])
            result.report.fail("AUTHORING_ANGULAR",source.name+": signed angular "+names[order]+" changed from its independent FVD source",result.distance[order],result.maximumError[order],tolerances[order]);
    }catch(const std::exception& e){result.report.fail(result.cancelled?"CANCELLED":"AUTHORING_ANGULAR_INPUT",e.what());}
    return result;
}
}
