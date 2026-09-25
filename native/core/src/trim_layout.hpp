#pragma once
#include "authoring.hpp"

namespace coaster {
// Hardware regulates off-design excess energy. It does not make a nominally
// rejected layout pass: nominal upstream thresholds leave every fin retracted.
inline void planTrimBrakes(Design& d,const std::vector<Frame>& frames) {
    if(!d.request.style.automaticTrims||frames.empty())return;
    const double half=(d.request.train.cars-1)*d.request.train.spacing*.5;
    double last=-1000;
    for(const auto& section:d.sections) {
        if(section.end-section.start<90||section.start-last<250)continue;
        // Regulate the energy of a turning approach before an inversion.
        // Segmented fins can follow a stable banked descent; keeping only
        // uphill upright sites left the wave-to-loop run unprotected. Longer
        // banks provide capacity without increasing the rated brake force.
        const auto middle=d.track.sample((section.start+section.end)*.5);
        const bool loopExit=section.role==RideRole::Loop&&middle.tangent.z<0;
        const bool inversionApproach=(middle.element==Element::Turn||loopExit)&&d.track.sample(std::min(d.track.length,section.end+1)).element==Element::Inversion;
        const double centre=section.start+(section.end-section.start)*(loopExit?.70:inversionApproach?.48:.62);
        const double speed=replayValueAt(frames,centre),length=std::clamp(speed*(inversionApproach?1.2:.8),18.,inversionApproach?80.:55.);
        const double begin=centre-length*.5,end=centre+length*.5;
        bool fits=speed>20;
        for(double s=begin-half;s<=end+half;s+=.5){const auto q=sampleKinematics(d.track,s);
            const auto upright=unit(Vec3{0,0,1}-q.sample.tangent*q.sample.tangent.z);
            fits=fits&&((q.sample.tangent.z>.04&&dot(q.sample.up,upright)>.7)||(inversionApproach&&std::abs(q.sample.tangent.z)>.04&&q.sample.up.z>0))&&norm(q.sample.curvature)<.02&&norm(q.upS)<.025;}
        for(const auto& op:d.operations){const double stop=op.end<op.start?d.track.length:op.end;
            if(begin<stop+2*half+5&&end>op.start-2*half-5)fits=false;}
        // Keep the default section plan from stacking trim hardware immediately before a nearby motor.
        const auto current=std::find_if(d.sections.begin(),d.sections.end(),[&](const RideSection& item){return &item==&section;});
        if(current!=d.sections.end()&&current+1!=d.sections.end()) {
            const auto& adjacent=*(current+1);
            if(adjacent.role==section.role&&adjacent.recipeId==section.recipeId&&adjacent.start>=section.end-1e-6&&adjacent.start<=section.end+1e-6) {
                for(const auto& motor:d.operations)if((motor.kind==DriveKind::Boost||motor.kind==DriveKind::Launch)&&motor.start>=section.end&&motor.start<=adjacent.end) {
                    const double elapsed=replayValueAt(frames,motor.start,true)-replayValueAt(frames,end,true);
                    const double approachSpeed=replayValueAt(frames,end);
                    bool interveningForce=false;
                    for(const auto& other:d.operations)if(&other!=&motor&&other.kind!=DriveKind::Trim&&other.start<motor.start&&other.end>end)interveningForce=true;
                    if(!interveningForce&&elapsed>=0&&approachSpeed>20&&elapsed<=6)fits=false;
                }
            }
        }
        if(!fits)continue;
        const double ramp=.35,lead=speed*(ramp+.35)+half;
        if(begin-lead-half<frames.front().distance)continue;
        Operation op{begin,end,DriveKind::Trim};
        op.targetSpeed=replayValueAt(frames,begin-lead-half)+1./3.6;
        op.maxForce=d.request.train.carMass*gravity*.45;op.trimPeakSpeed=25;
        op.maxPower=2*op.maxForce*op.trimPeakSpeed;op.rampSeconds=ramp;
        op.exitFadeMeters=std::min(length*.25,speed*.16);op.trimSensorLead=lead;
        d.operations.push_back(op);last=section.start;
    }
}
}


