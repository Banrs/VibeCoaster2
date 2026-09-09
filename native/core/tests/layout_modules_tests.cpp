#include "coaster/layout_modules.hpp"
#include "coaster/fvd.hpp"
#include <iostream>
#include <stdexcept>
using namespace coaster;
namespace {
int checks=0;
void check(bool value,const char* message){++checks;if(!value)throw std::runtime_error(message);}
void near(double value,double expected,double tolerance,const char* message){check(std::isfinite(value)&&std::abs(value-expected)<=tolerance,message);}
void nearVector(Vec3 value,Vec3 expected,double tolerance,const char* message){near(norm(value-expected),0,tolerance,message);}
void good(const ReversingModule& m){
    for(const auto& e:m.report.errors)std::cerr<<e.code<<": "<<e.message<<'\n';
    check(m.geometryBuilt&&m.canonicalBuilt&&!m.cancelled&&m.report.valid(),"Open module builds canonical geometry");
    check(!m.track.closed,"Module never claims circuit closure");
}
void checkJoins(const Track& track){
    for(size_t i=1;i<track.spans.size();++i){
        auto a=sampleSpanKinematics(track,i-1,1),b=sampleSpanKinematics(track,i,0);
        nearVector(a.sample.position,b.sample.position,2e-9,"G0 canonical join");
        nearVector(a.sample.tangent,b.sample.tangent,2e-10,"G1 canonical join");
        nearVector(a.sample.curvature,b.sample.curvature,2e-9,"G2 canonical join");
        nearVector(a.curvatureS,b.curvatureS,2e-8,"G3 canonical join");
        nearVector(a.sample.up,b.sample.up,2e-10,"Frame orientation join");
        nearVector(a.upS,b.upS,2e-9,"C1 frame join");
        nearVector(a.upSS,b.upSS,2e-8,"C2 frame join");
    }
}
void checkModule(const ReversingModuleRequest& request,const ReversingModule& m){
    good(m);const bool imm=request.kind==ReversingModuleKind::Immelmann;double sign=imm?1:-1;
    nearVector(m.exit.forward,request.entry.forward*(-1),1e-13,"True direction-reversing module exit");
    nearVector(m.exit.up,request.entry.up,1e-13,"Half-roll returns exit to upright");
    nearVector(m.exit.position,request.entry.position+request.entry.forward*(imm?-request.rollLength:request.rollLength)+request.entry.up*(sign*request.height),2e-10,"Exact endpoint pose includes height and longitudinal displacement");
    check(imm?m.pitchEndIndex==m.rollBeginIndex:m.rollEndIndex==m.pitchBeginIndex,"Pitch and roll order matches the selected element");
    auto pitchMid=m.track.knots[(m.pitchBeginIndex+m.pitchEndIndex)/2];
    nearVector(pitchMid.tangent,request.entry.up*sign,2e-13,"Half-loop passes vertical with correct ascent/descent direction");
    const auto& inverted=m.track.knots[imm?m.pitchEndIndex:m.pitchBeginIndex];
    nearVector(inverted.up,request.entry.up*(-1),1e-13,"Pitch/roll junction is physically inverted");
    if(imm)near(dot(inverted.position-request.entry.position,request.entry.up),request.height,2e-10,"Inverted apex reaches the requested local module height");
    double rollAngleSign=dot(m.track.knots[(m.rollBeginIndex+m.rollEndIndex)/2].up,cross(m.track.knots[m.rollBeginIndex].tangent,m.track.knots[m.rollBeginIndex].up));
    check(rollAngleSign*request.rollDirection>.99,"Roll handedness is explicit");
    for(size_t index:{size_t(0),m.track.knots.size()-1}){
        near(norm(m.track.knots[index].curvature),0,1e-14,"Zero curvature at open module ports");
    }
    for(double distance:{0.,m.track.length}){
        auto q=sampleKinematics(m.track,distance);
        near(norm(q.curvatureS),0,1e-10,"Straight guards give zero curvature derivative at ports");
        near(norm(q.upS),0,1e-10,"Zero first frame derivative at ports");
        near(norm(q.upSS),0,1e-10,"Zero second frame derivative at ports");
    }
    for(double s=0;s<=m.track.length;s+=.37){
        auto q=m.track.sample(s);check(finite(q.position)&&finite(q.tangent)&&finite(q.up)&&finite(q.curvature),"Canonical sampled frame is finite");
        near(norm(q.tangent),1,2e-12,"Unit canonical tangent");near(norm(q.up),1,2e-12,"Unit canonical up");near(dot(q.tangent,q.up),0,2e-12,"Orthonormal canonical frame");
    }
    checkJoins(m.track);
    // The generator consumes AuthoredPoint up hints, so also check that path.
    Track recompiled=compile(m.points,false);checkJoins(recompiled);
    nearVector(recompiled.sample(0).up,request.entry.up,2e-9,"Up hints survive the generator-facing compile path");
    nearVector(recompiled.sample(recompiled.length).up,request.entry.up,2e-9,"Exit up survives generator-facing compile");
}
}
int main(){try{
    ReversingModuleRequest request;
    auto immelmann=buildReversingModule(request);checkModule(request,immelmann);
    auto repeated=buildReversingModule(request);good(repeated);
    check(repeated.points.size()==immelmann.points.size(),"Deterministic sample count");
    for(size_t i=0;i<repeated.points.size();++i){nearVector(repeated.points[i].position,immelmann.points[i].position,0,"Deterministic geometry");nearVector(repeated.points[i].upHint,immelmann.points[i].upHint,0,"Deterministic orientation");}
    request.kind=ReversingModuleKind::DiveLoop;request.rollDirection=-1;
    auto dive=buildReversingModule(request);checkModule(request,dive);
    // A connector is the caller's responsibility: laterally separating the
    // reversed dive entry prevents coincident modules without hidden closure.
    request.entry=immelmann.exit;request.entry.position=request.entry.position+cross(request.entry.forward,request.entry.up)*60;
    auto paired=buildReversingModule(request);good(paired);
    nearVector(paired.exit.forward,immelmann.entry.forward,1e-13,"Paired reversing modules restore the initial heading");
    near(dot(paired.exit.position-immelmann.entry.position,immelmann.entry.up),0,1e-10,"Paired reversing modules restore height");
    near(std::abs(dot(paired.entry.position-immelmann.exit.position,cross(request.entry.forward,request.entry.up))),60,1e-12,"Caller-provided lateral separation is retained");
    request=ReversingModuleRequest{};request.entry.position={123,-456,37};double heading=.71;
    request.entry.forward={std::cos(heading),std::sin(heading),0};request.height=128;request.rollLength=221;request.sampleSpacing=1.2;request.rollDirection=-1;
    auto rotated=buildReversingModule(request);checkModule(request,rotated);
    request.sampleSpacing=.6;auto refined=buildReversingModule(request);good(refined);
    near(refined.pitchLength,rotated.pitchLength,0,"Requested geometry dimensions do not depend on tessellation");
    nearVector(refined.exit.position,rotated.exit.position,2e-10,"Refining samples does not move endpoint ports");
    near(refined.track.length,rotated.track.length,2e-4,"Canonical length converges under sample refinement");
    // Pitch and roll progression are independent shape controls. Dimensions
    // change the integrated geometry, with 1/scale curvature under uniform
    // scaling; a named topology alone is not a force or record certificate.
    for(double shape:{-.6,-.25,.3,.6}){
        ReversingModuleRequest shaped;shaped.height=80+35*(shape+.6);shaped.rollLength=140+80*(shape+.6);
        shaped.pitchShape=shape;shaped.rollShape=-shape;shaped.rollDirection=shape<0?-1:1;
        shaped.kind=shape<0?ReversingModuleKind::DiveLoop:ReversingModuleKind::Immelmann;
        auto m=buildReversingModule(shaped);checkModule(shaped,m);
        check(m.sampledMaxCurvature>0&&std::isfinite(m.sampledMaxFrameTwistPerMeter),"Sampled curvature/twist diagnostics are finite");
        std::cout<<"shape="<<shape<<" height="<<shaped.height<<" pitch_length="<<m.pitchLength<<" sampled_max_curvature="<<m.sampledMaxCurvature<<" equivalent_ascent_energy_speed_mps="<<std::sqrt(2*gravity*shaped.height)<<'\n';
    }
    ReversingModuleRequest small;small.height=70;small.rollLength=100;small.portLength=8;small.sampleSpacing=.75;small.pitchShape=.3;small.rollShape=-.4;
    auto smallModule=buildReversingModule(small);good(smallModule);
    auto large=small;large.height*=2;large.rollLength*=2;large.portLength*=2;large.sampleSpacing*=2;
    auto largeModule=buildReversingModule(large);good(largeModule);
    near(largeModule.track.length,2*smallModule.track.length,1e-8,"Uniform scaling doubles canonical path length");
    near(largeModule.sampledMaxCurvature,smallModule.sampledMaxCurvature*.5,2e-10,"Uniform scaling halves curvature");
    near(largeModule.sampledMaxFrameTwistPerMeter,smallModule.sampledMaxFrameTwistPerMeter*.5,2e-10,"Uniform scaling halves spatial twist");
    auto rounder=small;rounder.pitchShape=-.3;auto rounderModule=buildReversingModule(rounder);good(rounderModule);
    check(std::abs(rounderModule.pitchLength-smallModule.pitchLength)>1,"Shape control changes integrated proportions at fixed height");
    // A compound inversion keeps the centerline and endpoints while sharing
    // pitch and physical roll over a finite arc, rather than stopping one
    // motion completely before the other begins.
    for(auto kind:{ReversingModuleKind::Immelmann,ReversingModuleKind::DiveLoop})for(int hand:{-1,1}){
        auto compound=small;compound.kind=kind;compound.rollDirection=hand;compound.rollOverlap=.3;
        auto m=buildReversingModule(compound);good(m);checkJoins(m.track);
        auto separate=compound;separate.rollOverlap=0;auto original=buildReversingModule(separate);good(original);
        nearVector(m.exit.position,original.exit.position,1e-12,"Compound roll preserves endpoint position");
        nearVector(m.exit.up,original.exit.up,1e-12,"Compound roll restores exit orientation");
        near(m.track.length,original.track.length,1e-9,"Compound roll does not compress the centerline");
        bool simultaneous=false;
        for(size_t i=m.pitchBeginIndex;i<m.pitchEndIndex;++i){
            auto q=sampleSpanKinematics(m.track,i,.5);
            simultaneous|=norm(q.sample.curvature)>1e-4&&std::abs(dot(q.upS,q.sample.right))>1e-3;
        }
        check(simultaneous,"Pitch and roll must actually overlap");
        checkJoins(compile(m.points,false));
        check(m.sampledMaxFrameTwistPerMeter<original.sampledMaxFrameTwistPerMeter,"Sharing the roll arc reduces its peak spatial twist");
    }
    auto reject=[](ReversingModuleRequest r,const char* expected){auto value=buildReversingModule(r);check(!value.report.valid()&&!value.geometryBuilt&&!value.canonicalBuilt&&value.report.errors.front().code==expected,"Unsupported geometry is rejected explicitly");};
    request=ReversingModuleRequest{};request.height=NAN;reject(request,"LAYOUT_MODULE_INPUT");
    request=ReversingModuleRequest{};request.pitchShape=.66;reject(request,"LAYOUT_MODULE_INPUT");
    request=ReversingModuleRequest{};request.rollShape=INFINITY;reject(request,"LAYOUT_MODULE_INPUT");
    request=ReversingModuleRequest{};request.rollOverlap=.41;reject(request,"LAYOUT_MODULE_INPUT");
    request=ReversingModuleRequest{};request.entry.up=request.entry.forward;reject(request,"LAYOUT_MODULE_INPUT");
    request=ReversingModuleRequest{};request.portLength=1;reject(request,"LAYOUT_MODULE_INPUT");
    request=ReversingModuleRequest{};request.rollDirection=0;reject(request,"LAYOUT_MODULE_INPUT");
    request=ReversingModuleRequest{};request.maxSamples=15;reject(request,"LAYOUT_MODULE_BUDGET");
    auto cancelled=buildReversingModule(ReversingModuleRequest{},[]{return true;});check(cancelled.cancelled&&!cancelled.geometryBuilt&&!cancelled.canonicalBuilt&&!cancelled.report.valid(),"Cancelled authoring cannot be mistaken for completed geometry");
    int polls=0;cancelled=buildReversingModule(ReversingModuleRequest{},[&]{return ++polls>=3;});check(cancelled.cancelled&&!cancelled.geometryBuilt&&!cancelled.report.valid(),"Cancellation during geometry authoring");
    // Force-designed geometry must solve actual energy and asymmetric ports,
    // not recover a desired load by enlarging the legacy symmetric half-loop.
    for(double height:{80.,95.,120.})for(double apex:{23.,26.}){
        EnergyReversingModuleRequest energy;energy.geometry.height=height;energy.geometry.rollLength=60;
        energy.geometry.rollOverlap=.23;energy.apexSpeed=apex;
        auto up=buildEnergyReversingModule(energy);good(up);checkJoins(up.track);
        check(up.pitchForwardDisplacement>10,"Energy pitch retains its nonzero forward displacement");
        near(up.exit.position.x,up.pitchForwardDisplacement-energy.geometry.rollLength,1e-5,"Immelmann uses actual force-authored netX");
        near(up.exit.position.z,height,1e-5,"Force shoot closes requested height without scaling");
        nearVector(up.exit.forward,Vec3{-1,0,0},1e-7,"Energy pitch reverses forward");
        nearVector(up.exit.up,Vec3{0,0,1},1e-7,"Energy roll restores upright frame");
        near(up.idealEntrySpeed*up.idealEntrySpeed-apex*apex,2*gravity*height,1e-8,"Energy metadata retains the actual source climb");
        double normalMax=-100,lateralMax=0,minimumSpeed=1000;
        for(double at=0;at<up.track.length;at+=.4){auto q=up.track.sample(at);double v=std::sqrt(up.idealEntrySpeed*up.idealEntrySpeed-2*gravity*q.position.z);
            auto force=measureSeatForces(up.track,at,v,-gravity*q.tangent.z,0);
            normalMax=std::max(normalMax,force.vertical);lateralMax=std::max(lateralMax,std::abs(force.lateral));minimumSpeed=std::min(minimumSpeed,v);}
        check(normalMax<3.51&&normalMax>3.45,"Canonical source retains ordinary 3.5g pitch intent");
        check(lateralMax<1.5&&minimumSpeed>apex-.01,"Compound roll preserves bounded point forces and nonstalled apex");
        for(int hand:{-1,1}){energy.geometry.kind=ReversingModuleKind::DiveLoop;energy.geometry.rollDirection=hand;
            auto down=buildEnergyReversingModule(energy);good(down);checkJoins(down.track);
            near(down.exit.position.x,-up.exit.position.x,1e-5,"Dive consumes reversed asymmetric displacement");
            near(down.exit.position.z,-height,1e-5,"Dive restores the real source height");
            nearVector(down.exit.forward,Vec3{-1,0,0},1e-7,"Dive reverses heading");}
    }
    for(double height:{55.,60.,68.}){
        EnergyLoopModuleRequest loop;loop.height=height;loop.lateralOffset=0;
        auto full=buildEnergyLoopModule(loop);
        check(full.geometryBuilt&&full.canonicalBuilt&&full.report.valid(),"Energy full loop builds from shared force source");checkJoins(full.track);
        near(full.exit.position.x,2*full.pitchForwardDisplacement+2*loop.portLength,1e-5,"Full-loop displacement includes both forward guards");
        nearVector(full.exit.forward,Vec3{1,0,0},1e-7,"Full pitch loop restores forward heading");
        nearVector(full.exit.up,Vec3{0,0,1},1e-7,"Full pitch loop restores upright without frame roll");
        double apexDistance=full.track.length*.5;auto q=full.track.sample(apexDistance);
        near(q.position.z,height,1e-4,"Loop apex retains requested source height");
        nearVector(q.up,Vec3{0,0,-1},1e-6,"Full loop crest is inverted");
        auto force=measureSeatForces(full.track,apexDistance,loop.apexSpeed,0,0);
        near(force.vertical,loop.apexNormalG,.003,"Full-loop apex preserves positive support instead of a flat hanging reset");
        loop.lateralOffset=36;auto offset=buildEnergyLoopModule(loop);
        check(offset.canonicalBuilt&&offset.report.valid(),"Laterally separated full-loop source builds");checkJoins(offset.track);
        near(offset.exit.position.y,36,1e-9,"Actual loop endpoint retains caller lateral separation");
    }
    EnergyReversingModuleRequest invalidEnergy;invalidEnergy.geometry.pitchShape=.1;
    check(!buildEnergyReversingModule(invalidEnergy).report.valid(),"Legacy pitch shape is not silently ignored by energy authoring");
    invalidEnergy={};invalidEnergy.apexSpeed=NAN;check(!buildEnergyReversingModule(invalidEnergy).report.valid(),"Nonfinite force source speed rejected");
    auto unavailable=designFvdPitch({55,24,3.5,1.2,.5});check(!unavailable.section.report.valid(),"Unsolved force and energy intent remains a failure");
    auto cancelledEnergy=buildEnergyReversingModule(EnergyReversingModuleRequest{},[]{return true;});
    check(cancelledEnergy.cancelled&&!cancelledEnergy.geometryBuilt,"Energy module propagates early cancellation");
    polls=0;cancelledEnergy=buildEnergyReversingModule(EnergyReversingModuleRequest{},[&]{return ++polls>20;});
    check(cancelledEnergy.cancelled&&!cancelledEnergy.geometryBuilt,"Energy shooting propagates cancellation");
    auto cancelledLoop=buildEnergyLoopModule(EnergyLoopModuleRequest{},[]{return true;});
    check(cancelledLoop.cancelled&&!cancelledLoop.geometryBuilt,"Full-loop authoring propagates cancellation");
    std::cout<<"PASS "<<checks<<" reversing-module checks: genuine pitch/roll order, endpoint poses, inverted apex, canonical G3/C2 joins, deterministic variation, invalid input and cancellation\n";
    return 0;
}catch(const std::exception& e){std::cerr<<"FAIL after "<<checks<<": "<<e.what()<<'\n';return 1;}}
