#include "launch_camelback.hpp"
#include <stdexcept>

namespace coaster {
namespace {
void valid(const FvdResult& result){
    if(!result.report.valid()||!result.assessment.passed)
        throw std::runtime_error(result.report.errors.empty()?"Launch pullout source replay failed":result.report.errors.front().message);
}
FvdCamelbackResult reference(double speed,const CamelbackParameters& p,double rolling,double drag,Cancel cancel){
    if(!std::isfinite(p.profileScale)||p.profileScale<.8||p.profileScale>1.2||
        !std::isfinite(p.tailCutSeconds)||p.tailCutSeconds<0||p.tailCutSeconds>1||
        !std::isfinite(p.releaseSeconds)||p.releaseSeconds<.2||p.releaseSeconds>2||
        !std::isfinite(p.exitNormalG)||p.exitNormalG<1||p.exitNormalG>2)
        throw std::invalid_argument("Protected camelback recovery left its bounded force domain");
    FvdCamelbackResult result;auto& r=result.authoring;
    constexpr double grade=-10*pi/180;
    r.position={};r.forward={std::cos(grade),0,std::sin(grade)};r.up={-std::sin(grade),0,std::cos(grade)};
    r.speed=speed;r.step=.0025;r.rollingAcceleration=rolling;r.dragAccelerationCoefficient=drag;
    const double timeScale=speed/83.5*std::sqrt(p.profileScale);
    // Independently fitted against all frozen rail picks. The ascent is loaded
    // to target fifteen percent above the observed FF shoulder at the finite-train
    // seats. The crest has separate entry, middle and exit
    // loads. Every transition retains value/rate/second derivative, with no
    // geometric repair. The FF video phase comparison is separate from this fit.
    constexpr std::array<double,6> durations{1.456381166011514,0.068271557999579319,5.1605579807169217,0.37831993686002796,0.40299920800484751,6.1198248124508581};
    constexpr std::array<double,6> loads{4.2501176531780898,4.2501176531780898,-1.2989637242239358,-0.87941712069660594,-1.2528542934767881,4.5639355198747698};
    constexpr double recoveryPower=2.6944467829560046;
    r.controls={{0,std::cos(grade),0,0}};double time=0;
    for(size_t i=0;i<durations.size();++i){
        const double duration=durations[i]*timeScale,begin=time;time+=duration;
        if(i==2){
            // A short loaded shoulder and negative hold bound the ascent
            // unload. Both quintic endpoints meet the adjacent zero jets.
            constexpr double startFraction=0.004934846372012555,endFraction=0.99905265009748567;
            r.controls.push_back({begin+duration*startFraction,loads[i-1],0,0});
            r.controls.push_back({begin+duration*endFraction,loads[i],0,0});
        }
        if(i!=5){r.controls.push_back({time,loads[i],0,0});continue;}
        // A monotone asymmetric easing delays the positive recovery after
        // airtime. Its analytic value/rate/second derivative become ordinary
        // quintic Hermite controls; all subdivision joins retain those jets.
        const double from=loads[i-1],change=loads[i]-from;
        for(int j=1;j<=8;++j){
            const double u=j/8.,w=1-u,q=u*u*u*(10+u*(-15+6*u));
            const double qd=30*u*u*w*w,qdd=60*u*w*(1-2*u);
            FvdControl c{begin+duration*u,from+change*std::pow(q,recoveryPower),0,0};
            c.first[0]=change*recoveryPower*std::pow(q,recoveryPower-1)*qd/duration;
            c.second[0]=change*recoveryPower*(std::pow(q,recoveryPower-1)*qdd+
                (recoveryPower-1)*std::pow(q,recoveryPower-2)*qd*qd)/(duration*duration);
            r.controls.push_back(c);
        }
    }
    result.originalEndTime=time+(0.25478378531183493+.75)*timeScale;
    result.protectedEndTime=result.originalEndTime-p.tailCutSeconds*timeScale;
    if(result.protectedEndTime<=time+.001)throw std::runtime_error("Tail cut must remain inside the protected final constant-load hold");
    r.controls.push_back({result.protectedEndTime,loads.back(),0,0});
    const double releaseEnd=result.protectedEndTime+p.releaseSeconds*timeScale;
    r.controls.push_back({releaseEnd,p.exitNormalG,0,0});
    const double minimumPitch=p.minimumExitPitchDegrees*pi/180;
    if(!std::isfinite(minimumPitch)||minimumPitch<0||minimumPitch>.45)
        throw std::invalid_argument("Protected camelback exit pitch left its bounded domain");
    auto replay=[&](){result.section=designFvdSection(r,cancel);valid(result.section);return std::atan2(result.section.samples.back().forward.z,result.section.samples.back().forward.x);};
    if(replay()<minimumPitch){
        double lo=0,hi=.1;r.controls.push_back({releaseEnd+hi,p.exitNormalG,0,0});
        while(replay()<minimumPitch){lo=hi;hi*=2;if(hi>8)throw std::runtime_error("Low-load recovery cannot reach its rising port within eight seconds");r.controls.back().time=releaseEnd+hi;}
        for(int iteration=0;iteration<28;++iteration){const double mid=(lo+hi)*.5;r.controls.back().time=releaseEnd+mid;if(replay()<minimumPitch)lo=mid;else hi=mid;}
        r.controls.back().time=releaseEnd+hi;replay();
    }
    const auto boundary=std::lower_bound(result.section.samples.begin(),result.section.samples.end(),result.protectedEndTime,
        [](const FvdSample& q,double t){return q.time<t;});
    result.protectedEndDistance=result.section.track.spans.at(size_t(boundary-result.section.samples.begin())).start;
    for(const auto& q:result.section.samples)if(q.forward.x<=0||std::abs(q.position.y)>1e-10)
        throw std::runtime_error("Protected camelback must retain its forward planar silhouette");
    return result;
}
FvdHillResult pullout(double speed,double grade,double pitch,double normal,double rolling,double drag,Cancel cancel){
    FvdHillResult result;auto& r=result.authoring;
    r.position={};r.forward={std::cos(grade),0,std::sin(grade)};r.up={-std::sin(grade),0,std::cos(grade)};
    r.speed=speed;r.step=.005;r.rollingAcceleration=rolling;r.dragAccelerationCoefficient=drag;
    auto shoot=[&](double duration){
        if(cancel&&cancel())throw std::runtime_error("CANCELLED");
        r.controls={{0,std::cos(grade),0,0},{duration,normal,0,0}};
        auto section=designFvdSection(r,cancel);valid(section);
        const auto& end=section.samples.back();const double residual=std::atan2(end.forward.z,end.forward.x)-pitch;
        result.section=std::move(section);return residual;
    };
    double lo=.05,hi=.5,low=shoot(lo),high=shoot(hi);
    while(high<=0&&hi<8){lo=hi;low=high;hi=std::min(8.,hi*1.5);high=shoot(hi);}
    if(low>=0||high<=0)throw std::runtime_error("Downhill launch pullout has no bounded monotone force-ramp solution");
    for(int iteration=0;iteration<42;++iteration){
        // Safeguarded interpolation avoids repeated full bisection while
        // retaining a true bracket for the physical pitch residual.
        const double t=std::clamp(lo-low*(hi-lo)/(high-low),lo+.1*(hi-lo),hi-.1*(hi-lo));
        const double error=shoot(t);
        if(std::abs(error)<1e-12)break;
        if(error<0){lo=t;low=error;}else{hi=t;high=error;}
        if(iteration==41)throw std::runtime_error("Downhill launch pullout pitch solve did not converge");
    }
    double prior=grade;
    for(const auto& q:result.section.samples){
        const double angle=std::asin(q.forward.z);
        if(angle<prior-1e-9||angle>pitch+1e-8||std::abs(q.position.y)>1e-10)
            throw std::runtime_error("Downhill launch pullout added an unintended pitch reversal or yaw");
        prior=angle;
    }
    return result;
}
}
LaunchCamelback designLaunchCamelback(double speed,double grade,const CamelbackParameters& p,
    double rolling,double drag,Cancel cancel){
    if(!std::isfinite(speed)||speed<65||speed>88||!std::isfinite(grade)||grade>=0||grade< -20*pi/180)
        throw std::invalid_argument("Launch-to-camelback entry needs 65..88 m/s on a downhill grade up to 20 degrees");
    LaunchCamelback result;double referenceSpeed=speed,previousEnergy=NAN,previousResidual=NAN,lastError=NAN;
    for(int iteration=0;iteration<12;++iteration){
        if(cancel&&cancel())throw std::runtime_error("CANCELLED");
        result.camelback=reference(referenceSpeed,p,rolling,drag,cancel);
        const double time=result.camelback.authoring.controls[1].time;
        const auto& samples=result.camelback.section.samples;
        const auto at=std::lower_bound(samples.begin(),samples.end(),time,[](const FvdSample& q,double t){return q.time<t;});
        if(at==samples.end()||std::abs(at->time-time)>1e-9)throw std::runtime_error("Protected camelback shoulder is missing");
        const size_t index=size_t(at-samples.begin());
        result.retainedBeginDistance=result.camelback.section.track.spans.at(index).start;
        result.pullout=pullout(speed,grade,std::asin(at->forward.z),result.camelback.authoring.controls[1].normalG,rolling,drag,cancel);
        const double actual=result.pullout.section.samples.back().speed;
        const double error=actual-at->speed;lastError=error;
        if(std::abs(error)<1e-9){
            result.referenceEntrySpeed=referenceSpeed;
            const auto incoming=sampleKinematics(result.pullout.section.track,result.pullout.section.track.length);
            const auto protectedPort=sampleKinematics(result.camelback.section.track,result.retainedBeginDistance);
            if(norm(incoming.sample.tangent-protectedPort.sample.tangent)>1e-7||
                norm(incoming.sample.curvature-protectedPort.sample.curvature)>1e-7||
                norm(incoming.curvatureS-protectedPort.curvatureS)>1e-7||
                norm(incoming.curvatureSS-protectedPort.curvatureSS)>1e-7||
                norm(incoming.sample.up-protectedPort.sample.up)>1e-7||
                norm(incoming.upS-protectedPort.upS)>1e-7||
                norm(incoming.upSS-protectedPort.upSS)>1e-7||
                norm(incoming.upSSS-protectedPort.upSSS)>1e-7)
                throw std::runtime_error("Launch pullout missed the protected live geometry jet");
            return result;
        }
        const double energy=referenceSpeed*referenceSpeed,residual=actual*actual-at->speed*at->speed;
        double next=energy+residual;
        if(std::isfinite(previousEnergy)&&std::abs(residual-previousResidual)>1e-10){
            const double secant=energy-residual*(energy-previousEnergy)/(residual-previousResidual);
            if(std::isfinite(secant)&&secant>65*65&&secant<90*90)next=secant;
        }
        previousEnergy=energy;previousResidual=residual;referenceSpeed=std::sqrt(next);
        if(referenceSpeed<65||referenceSpeed>90)throw std::runtime_error("Protected camelback energy match left its supported family");
    }
    throw std::runtime_error("Launch-to-camelback inherited energy did not converge; speed residual="+std::to_string(lastError));
}
}
