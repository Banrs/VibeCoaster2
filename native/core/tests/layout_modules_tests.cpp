#include "coaster/layout_modules.hpp"
#include <iostream>
#include <stdexcept>
using namespace coaster;
namespace {
int checks=0;
void check(bool value,const char* message){++checks;if(!value)throw std::runtime_error(message);}
void near(double value,double expected,double tolerance,const char* message){check(std::isfinite(value)&&std::abs(value-expected)<=tolerance,message);}
void nearVector(Vec3 value,Vec3 expected,double tolerance,const char* message){near(norm(value-expected),0,tolerance,message);}
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
}
int main(){try{
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
    EnergyLoopModuleRequest request;auto base=buildEnergyLoopModule(request),repeated=buildEnergyLoopModule(request);
    check(base.canonicalBuilt&&repeated.canonicalBuilt,"Repeated force-authored loop builds");
    check(base.points.size()==repeated.points.size(),"Deterministic sample count");
    for(size_t i=0;i<base.points.size();++i){nearVector(repeated.points[i].position,base.points[i].position,0,"Deterministic geometry");nearVector(repeated.points[i].upHint,base.points[i].upHint,0,"Deterministic orientation");}
    request.entry.position={123,-456,37};const double heading=.71;request.entry.forward={std::cos(heading),std::sin(heading),0};
    auto rotated=buildEnergyLoopModule(request);check(rotated.canonicalBuilt&&rotated.report.valid(),"Translated and rotated force-authored loop builds");checkJoins(rotated.track);
    near(rotated.track.length,base.track.length,1e-7,"Rigid world placement preserves length");
    check(rotated.points.size()==base.points.size(),"Rigid placement preserves source sampling");
    for(size_t i=0;i<base.points.size();++i){const auto p=base.points[i].position;
        nearVector(rotated.points[i].position,request.entry.position+Vec3{p.x*std::cos(heading)-p.y*std::sin(heading),p.x*std::sin(heading)+p.y*std::cos(heading),p.z},1e-10,"Placement rotates actual force-authored geometry");}
    auto reject=[](EnergyLoopModuleRequest r){auto value=buildEnergyLoopModule(r);check(!value.report.valid()&&!value.geometryBuilt&&!value.canonicalBuilt,"Unsupported loop authoring is rejected explicitly");};
    auto invalid=request;invalid.height=NAN;reject(invalid);
    invalid=request;invalid.apexSpeed=NAN;reject(invalid);
    invalid=request;invalid.lateralOffset=INFINITY;reject(invalid);
    invalid=request;invalid.entry.up=invalid.entry.forward;reject(invalid);
    invalid=request;invalid.portLength=1;reject(invalid);
    invalid=request;invalid.maxSamples=15;reject(invalid);
    auto cancelledLoop=buildEnergyLoopModule(EnergyLoopModuleRequest{},[]{return true;});
    check(cancelledLoop.cancelled&&!cancelledLoop.geometryBuilt,"Full-loop authoring propagates cancellation");
    int polls=0;cancelledLoop=buildEnergyLoopModule(request,[&]{return ++polls>20;});
    check(cancelledLoop.cancelled&&!cancelledLoop.geometryBuilt,"Full-loop source shooting propagates cancellation");
    std::cout<<"PASS "<<checks<<" full-loop checks: force-authored dimensions, crest load, canonical G3/C2 joins, deterministic geometry, rigid placement, invalid input and cancellation\n";
    return 0;
}catch(const std::exception& e){std::cerr<<"FAIL after "<<checks<<": "<<e.what()<<'\n';return 1;}}
