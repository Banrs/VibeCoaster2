#include "coaster/layout_modules.hpp"
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
        !std::isfinite(r.sampleSpacing)||r.sampleSpacing<.25||r.sampleSpacing>2||
        !std::isfinite(r.portLength)||r.portLength<4*r.sampleSpacing||r.portLength>100){
        fail("LAYOUT_MODULE_INPUT","Unsupported energy-authored full-loop pose or sampling");return result;}
    if(r.maxSamples<12||r.maxSamples>50000){fail("LAYOUT_MODULE_BUDGET","Unsupported loop sample budget");return result;}
    auto cancelled=[&]{if(cancel&&cancel()){result.cancelled=true;fail("CANCELLED","Energy-authored full loop cancelled");return true;}return false;};
    if(cancelled())return result;
    FvdLoopRequest request;request.height=r.height;request.apexSpeed=r.apexSpeed;request.normalG=r.normalG;
    request.apexNormalG=r.apexNormalG;request.pushRampSeconds=r.pushRampSeconds;request.crossingOffset=r.crossingOffset;
    request.portLength=r.portLength;request.maxSamples=r.maxSamples;
    request.rollingAcceleration=r.rollingAcceleration;request.dragAccelerationCoefficient=r.dragAccelerationCoefficient;
    auto source=designFvdLoop(request,cancel);result.report=source.section.report;result.cancelled=source.section.cancelled;
    if(!result.report.valid()||!source.section.assessment.passed)return result;
    result.height=source.apex.position.z;result.authoring=std::move(source.authoring);
    result.samples=std::move(source.section.samples);result.assessment=source.section.assessment;
    result.loopEntry=source.loopEntry;result.apex=source.apex;result.loopExit=source.loopExit;result.track=std::move(source.section.track);
    result.crossingEntry=source.crossingEntry;result.crossingExit=source.crossingExit;
    const Vec3 right=cross(r.entry.forward,r.entry.up);
    const auto transform=[&](Vec3 p){return r.entry.forward*p.x-right*p.y+r.entry.up*p.z;};
    const auto place=[&](FvdSample& q){q.position=r.entry.position+transform(q.position);q.forward=transform(q.forward);q.up=transform(q.up);q.curvature=transform(q.curvature);};
    result.authoring.position=r.entry.position;result.authoring.forward=r.entry.forward;result.authoring.up=r.entry.up;
    place(result.loopEntry);place(result.apex);place(result.loopExit);place(result.crossingEntry);place(result.crossingExit);
    for(auto& q:result.samples){if(cancelled())return result;place(q);}
    for(size_t i=0;i<result.track.knots.size();++i){
        auto& k=result.track.knots[i];const auto& q=result.samples[i];
        k.position=q.position;k.tangent=q.forward;k.up=q.up;k.curvature=q.curvature;
        k.element=q.time>=result.loopEntry.time&&q.time<=result.loopExit.time?Element::Inversion:Element::Return;
    }
    try{
        if(cancelled())return result;result.track.rebuild();if(cancelled())return result;
        // Sample each real phase boundary exactly; only translation/yaw has
        // changed the source, and its canonical tangent/curvature/up survive.
        const std::array<double,5> boundaries{0,result.loopEntry.distance,result.apex.distance,result.loopExit.distance,result.track.length};
        result.points.push_back({result.samples.front().position,0,Element::Return,result.samples.front().up});
        for(size_t phase=1;phase<boundaries.size();++phase){
            const double start=boundaries[phase-1],length=boundaries[phase]-start;
            const size_t count=size_t(std::ceil(length/r.sampleSpacing));
            if(count>r.maxSamples-result.points.size()){fail("LAYOUT_MODULE_BUDGET","Energy-authored full loop exceeds routing sample budget");return result;}
            for(size_t i=1;i<=count;++i){if(cancelled())return result;
                const auto q=result.track.sample(start+length*i/count);
                result.points.push_back({q.position,0,phase==2||phase==3?Element::Inversion:Element::Return,q.up});
            }
        }
        const auto& end=result.samples.back();result.exit={end.position,end.forward,end.up};result.geometryBuilt=true;result.canonicalBuilt=true;
    }
    catch(const std::exception& e){fail("LAYOUT_MODULE_CANONICAL",e.what());}
    return result;
}

}
