#include "coaster/layout_modules.hpp"
#include "../src/source_geometry.hpp"
#include <iostream>
#include <stdexcept>
using namespace coaster;
namespace {
int checks=0;
void check(bool value,const char* message){++checks;if(!value)throw std::runtime_error(message);}
void near(double value,double expected,double tolerance,const char* message){check(std::isfinite(value)&&std::abs(value-expected)<=tolerance,message);}
void nearVector(Vec3 value,Vec3 expected,double tolerance,const char* message){near(norm(value-expected),0,tolerance,message);}
void good(const EnergyLoopModule& module){
    if(!module.report.valid())for(const auto& error:module.report.errors)std::cerr<<error.code<<": "<<error.message<<'\n';
    check(module.geometryBuilt&&module.canonicalBuilt&&module.report.valid()&&module.assessment.passed,"Continuous energy-authored full loop builds and replays");
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
void checkSourcePlacement(const Track& source,double landmark){
    const detail::SourceGeometry geometry(source,Element::Inversion,{landmark});
    const Vec3 origin{123,-456,37};
    const auto yaw=[](Vec3 p){return Vec3{-p.y,p.x,p.z};};
    bool found=false;Track placed;placed.closed=false;
    for(size_t i=0;i<geometry.points.size();++i){
        const auto& point=geometry.points[i];const auto frame=detail::placeSourceFrame(point.frame,origin,pi/2);
        const auto actual=source.sample(point.distance);found|=point.distance==landmark;
        nearVector(frame.position,origin+yaw(actual.position),1e-10,"Placed source uses actual canonical positions");
        nearVector(frame.tangent,yaw(actual.tangent),1e-12,"Source placement preserves tangents");
        nearVector(frame.curvature,yaw(actual.curvature),1e-12,"Source placement preserves curvature");
        nearVector(frame.up,yaw(actual.up),1e-12,"Source placement preserves the physical force frame");
        placed.knots.push_back({frame.position,frame.tangent,frame.curvature,frame.up,0,frame.element});
    }
    check(found,"Nonuniform physical landmark is retained exactly");placed.rebuild();checkJoins(placed);
    near(placed.length,source.length,2e-5,"Placed source retains canonical arc length");
    for(double s=1;s<source.length-1;s+=2.3){
        const auto expected=source.sample(s),actual=placed.sample(s);
        nearVector(actual.position,origin+yaw(expected.position),3e-5,"Resampled placement retains source interior");
        nearVector(actual.curvature,yaw(expected.curvature),2e-5,"Resampled placement retains physical curvature between controls");
        const auto originalForce=measureSeatForces(source,s,45,0,0),placedForce=measureSeatForces(placed,s,45,0,0);
        near(placedForce.vertical,originalForce.vertical,.005,"Placed source preserves independent rider normal force");
        near(placedForce.lateral,originalForce.lateral,.005,"Placed source preserves independent rider lateral force");
    }
}
}
int main(){try{
    const TrainConfig train;
    for(double height:{55.,60.,68.}){
        EnergyLoopModuleRequest loop;loop.height=height;loop.crossingOffset=0;
        loop.rollingAcceleration=gravity*train.rollingResistance;
        loop.dragAccelerationCoefficient=.5*train.airDensity*train.dragCdA/(train.cars*train.carMass);
        auto full=buildEnergyLoopModule(loop);
        good(full);checkJoins(full.track);
        nearVector(full.exit.position,full.samples.back().position,0,"The module exposes its actual integrated endpoint");
        nearVector(full.exit.forward,Vec3{1,0,0},1e-7,"Full pitch loop restores forward heading");
        nearVector(full.exit.up,Vec3{0,0,1},1e-7,"Full pitch loop restores upright without frame roll");
        const double apexDistance=full.apex.distance;auto q=full.track.sample(apexDistance);
        near(q.position.z,height,1e-4,"Loop apex retains requested source height");
        nearVector(q.up,Vec3{0,0,-1},1e-6,"Full loop crest is inverted");
        auto force=measureSeatForces(full.track,apexDistance,full.apex.speed,0,0);
        near(force.vertical,loop.apexNormalG,.003,"Full-loop apex preserves positive support instead of a flat hanging reset");
        check(full.samples.back().speed<full.samples.front().speed-.1,"The module retains real depletion across the whole loop");
        near(full.loopEntry.distance,loop.portLength,1e-5,"Entry guard length includes its real losses");
        near(full.samples.back().distance-full.loopExit.distance,loop.portLength,1e-5,"Exit guard length includes its real losses");
        loop.crossingOffset=18;auto offset=buildEnergyLoopModule(loop);
        good(offset);checkJoins(offset.track);
        nearVector(offset.crossingExit.position-offset.crossingEntry.position,{0,18,0},1e-5,"Actual low crossing arms retain the requested separation");
        check(offset.exit.forward.y<-.3,"The laterally separated module retains its genuine changed exit heading");
        for(size_t i=0;i<offset.samples.size();i+=127){const auto& s=offset.samples[i];const auto actual=offset.track.sample(s.distance);
            nearVector(actual.position,s.position,1e-6,"The module preserves integrated canonical positions without post-warp");
            nearVector(actual.tangent,s.forward,1e-7,"The module preserves integrated canonical tangents");
            nearVector(actual.up,s.up,1e-7,"The module preserves the actual force frame");
            nearVector(actual.curvature,s.curvature,1e-6,"The module preserves loss-aware source curvature");
        }
    }
    EnergyLoopModuleRequest request;auto base=buildEnergyLoopModule(request),repeated=buildEnergyLoopModule(request);
    good(base);good(repeated);
    checkSourcePlacement(base.track,base.apex.distance);
    const auto reversal=designFvdImmelmann(FvdImmelmannRequest{});
    check(reversal.section.assessment.passed&&reversal.section.report.valid(),"Immelmann source for placement test is valid");
    checkSourcePlacement(reversal.section.track,reversal.rollExit.distance);
    check(base.points.size()==repeated.points.size(),"Deterministic sample count");
    for(size_t i=0;i<base.points.size();++i){nearVector(repeated.points[i].position,base.points[i].position,0,"Deterministic geometry");nearVector(repeated.points[i].upHint,base.points[i].upHint,0,"Deterministic orientation");}
    request.entry.position={123,-456,37};const double heading=.71;request.entry.forward={std::cos(heading),std::sin(heading),0};
    auto rotated=buildEnergyLoopModule(request);good(rotated);checkJoins(rotated.track);
    near(rotated.track.length,base.track.length,1e-7,"Rigid world placement preserves length");
    check(rotated.points.size()==base.points.size(),"Rigid placement preserves source sampling");
    const auto transform=[&](Vec3 p){return Vec3{p.x*std::cos(heading)-p.y*std::sin(heading),p.x*std::sin(heading)+p.y*std::cos(heading),p.z};};
    for(size_t i=0;i<base.points.size();++i){const auto p=base.points[i].position;
        nearVector(rotated.points[i].position,request.entry.position+transform(p),1e-7,"Placement rotates actual force-authored geometry");}
    nearVector(rotated.exit.forward,transform(base.exit.forward),1e-10,"World placement retains the real rotated exit heading");
    nearVector(rotated.apex.position,request.entry.position+transform(base.apex.position),1e-10,"Actual source apex checkpoint follows world placement");
    for(size_t i=0;i<base.samples.size();i+=127){const auto& a=base.samples[i];const auto& b=rotated.samples[i];
        nearVector(b.position,request.entry.position+transform(a.position),1e-10,"Integrated source history follows rigid placement");
        near(b.speed,a.speed,0,"Rigid placement cannot alter source speed");near(b.dissipatedWorkPerMass,a.dissipatedWorkPerMass,0,"Rigid placement cannot alter dissipated work");
        const auto force=measureSeatForces(rotated.track,b.distance,b.speed,0,0);
        near(force.vertical,dot(a.curvature*(a.speed*a.speed)+Vec3{0,0,gravity},a.up)/gravity,1e-5,"World canonical geometry retains the actual source normal-force history");
        near(force.lateral,0,1e-5,"The rotated physical bank produces the intended centerline body-lateral force");
    }
    auto placedReplay=designFvdSection(rotated.authoring);
    check(placedReplay.assessment.passed&&placedReplay.report.valid(),"World-placed authoring reproduces an independently integrated canonical section");
    nearVector(placedReplay.samples.back().position,rotated.exit.position,1e-6,"Actual world-space exit is derived by integration, not only transformed presentation points");
    near(placedReplay.samples.back().speed,rotated.samples.back().speed,1e-6,"World-space source replay preserves the actual speed");
    auto reject=[](EnergyLoopModuleRequest r){auto value=buildEnergyLoopModule(r);check(!value.report.valid()&&!value.geometryBuilt&&!value.canonicalBuilt,"Unsupported loop authoring is rejected explicitly");};
    auto invalid=request;invalid.height=NAN;reject(invalid);
    invalid=request;invalid.apexSpeed=NAN;reject(invalid);
    invalid=request;invalid.crossingOffset=INFINITY;reject(invalid);
    invalid=request;invalid.entry.up=invalid.entry.forward;reject(invalid);
    invalid=request;invalid.portLength=1;reject(invalid);
    invalid=request;invalid.maxSamples=15;reject(invalid);
    invalid=request;invalid.rollingAcceleration=-.01;reject(invalid);
    invalid=request;invalid.dragAccelerationCoefficient=NAN;reject(invalid);
    auto cancelledLoop=buildEnergyLoopModule(EnergyLoopModuleRequest{},[]{return true;});
    check(cancelledLoop.cancelled&&!cancelledLoop.geometryBuilt,"Full-loop authoring propagates cancellation");
    int polls=0;cancelledLoop=buildEnergyLoopModule(request,[&]{return ++polls>20;});
    check(cancelledLoop.cancelled&&!cancelledLoop.geometryBuilt,"Full-loop source shooting propagates cancellation");
    std::cout<<"PASS "<<checks<<" full-loop checks: force-authored dimensions, crest load, canonical G3/C2 joins, deterministic geometry, rigid placement, invalid input and cancellation\n";
    return 0;
}catch(const std::exception& e){std::cerr<<"FAIL after "<<checks<<": "<<e.what()<<'\n';return 1;}}
