#include "coaster/layout_modules.hpp"
#include "coaster/fvd.hpp"
#include <stdexcept>

namespace coaster {
namespace {
bool validPose(const LayoutModulePose& p){
    return finite(p.position)&&finite(p.forward)&&finite(p.up)&&norm(p.position)<=100000&&
        std::abs(norm(p.forward)-1)<1e-9&&std::abs(norm(p.up)-1)<1e-9&&std::abs(dot(p.forward,p.up))<1e-9;
}
}
EnergyLoopModule buildEnergyLoopModule(const EnergyLoopModuleRequest& r,Cancel cancel){
    EnergyLoopModule result;result.entry=r.entry;result.track.closed=false;
    auto fail=[&](const char* code,const char* message){result.report.fail(code,message);};
    if(!validPose(r.entry)||std::abs(r.entry.forward.z)>1e-9||norm(r.entry.up-Vec3{0,0,1})>1e-9||
        !std::isfinite(r.lateralOffset)||std::abs(r.lateralOffset)>100||!std::isfinite(r.apexNormalG)||r.apexNormalG<0||r.apexNormalG>1.5||
        !std::isfinite(r.sampleSpacing)||r.sampleSpacing<.25||r.sampleSpacing>2||
        !std::isfinite(r.portLength)||r.portLength<4*r.sampleSpacing||r.portLength>100){
        fail("LAYOUT_MODULE_INPUT","Unsupported energy-authored full-loop pose, offset or sampling");return result;}
    if(r.maxSamples<12||r.maxSamples>50000){fail("LAYOUT_MODULE_BUDGET","Unsupported loop sample budget");return result;}
    auto cancelled=[&]{if(cancel&&cancel()){result.cancelled=true;fail("CANCELLED","Energy-authored full loop cancelled");return true;}return false;};
    if(cancelled())return result;
    auto pitch=designFvdPitch({r.height,r.apexSpeed,r.normalG,r.pushRampSeconds,r.apexNormalG},cancel);
    result.report=pitch.section.report;result.cancelled=pitch.section.cancelled;
    if(!result.report.valid()||!pitch.section.assessment.passed)return result;
    result.height=pitch.height;result.pitchForwardDisplacement=pitch.forwardDisplacement;
    result.idealEntrySpeed=pitch.authoring.speed;result.idealApexSpeed=r.apexSpeed;
    const double L=pitch.section.track.length,D=pitch.forwardDisplacement;
    const int steps=int(std::ceil(L/r.sampleSpacing)),guards=int(std::ceil(r.portLength/r.sampleSpacing));
    if(2*size_t(steps)+2*guards+1>r.maxSamples){fail("LAYOUT_MODULE_BUDGET","Energy-authored full loop exceeds sample budget");return result;}
    const Vec3 right=cross(r.entry.forward,r.entry.up);
    auto transform=[&](Vec3 p){return r.entry.forward*p.x-right*p.y+r.entry.up*p.z;};
    auto append=[&](Vec3 p,Vec3 up,Element e){result.points.push_back({r.entry.position+transform(p),0,e,transform(up)});};
    for(int i=0;i<=guards;++i)append({r.portLength*i/guards,0,0},{0,0,1},Element::Return);
    for(int i=1;i<=2*steps;++i){if((i&63)==0&&cancelled())return result;
        double s=L*i/steps,u=double(i)/(2*steps);bool descending=i>steps;
        auto q=pitch.section.track.sample(descending?2*L-s:s);
        Vec3 p=descending?Vec3{2*D-q.position.x,q.position.y,q.position.z}:q.position;
        Vec3 up=descending?Vec3{-q.up.x,q.up.y,q.up.z}:q.up;
        p.x+=r.portLength;p.y+=r.lateralOffset*u*u*u*u*(35+u*(-84+u*(70-20*u)));
        append(p,up,Element::Inversion);
    }
    for(int i=1;i<=guards;++i)append({r.portLength+2*D+r.portLength*i/guards,r.lateralOffset,0},{0,0,1},Element::Return);
    result.geometryBuilt=true;if(cancelled())return result;
    try{result.track=compile(result.points,false);result.canonicalBuilt=true;auto end=result.track.sample(result.track.length);result.exit={end.position,end.tangent,end.up};}
    catch(const std::exception& e){fail("LAYOUT_MODULE_CANONICAL",e.what());}
    return result;
}

}
