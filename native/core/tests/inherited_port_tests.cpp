#include "coaster/fvd.hpp"
#include "coaster/angular_phase_intent.hpp"
#include "../src/authoring.hpp"
#include "coaster/clearance.hpp"
#include <iostream>
#include <iomanip>
#include <stdexcept>
using namespace coaster;
namespace {
int checks=0;
void check(bool yes,const char* message){++checks;if(!yes)throw std::runtime_error(message);}
void near(double value,double expected,double tolerance,const char* message){
    if(!std::isfinite(value)||std::abs(value-expected)>tolerance){std::cerr<<message<<": "<<value<<" vs "<<expected<<" tolerance "<<tolerance<<'\n';check(false,message);}++checks;
}
void good(const FvdResult& result){
    if(!result.report.valid())for(const auto& e:result.report.errors)std::cerr<<e.code<<": "<<e.message<<'\n';
    check(result.report.valid()&&result.assessment.passed,"Inherited source passes independent canonical replay");
}
void samePort(const Knot& a,const Knot& b){
    near(norm(a.position-b.position),0,1e-7,"Inherited position");near(norm(a.tangent-b.tangent),0,1e-7,"Inherited tangent");
    near(norm(a.curvature-b.curvature),0,1e-7,"Inherited curvature");near(norm(a.third-b.third),0,1e-7,"Inherited third position derivative");
    near(norm(a.fourth-b.fourth),0,1e-7,"Inherited fourth position derivative");near(norm(a.up-b.up),0,1e-7,"Inherited physical up");
    near(norm(a.upFirst-b.upFirst),0,1e-7,"Inherited first frame derivative");near(norm(a.upSecond-b.upSecond),0,1e-7,"Inherited second frame derivative");
    near(norm(a.upThird-b.upThird),0,1e-7,"Inherited third frame derivative");
}
Vec3 yawed(Vec3 p,double a){return {p.x*std::cos(a)-p.y*std::sin(a),p.x*std::sin(a)+p.y*std::cos(a),p.z};}
Knot located(Knot k,Vec3 origin,double yaw){
    k.position=origin+yawed(k.position,yaw);k.tangent=yawed(k.tangent,yaw);k.curvature=yawed(k.curvature,yaw);
    k.third=yawed(k.third,yaw);k.fourth=yawed(k.fourth,yaw);k.up=yawed(k.up,yaw);k.upFirst=yawed(k.upFirst,yaw);
    k.upSecond=yawed(k.upSecond,yaw);k.upThird=yawed(k.upThird,yaw);return k;
}
void loopPlaneYaw(const FvdHillResult& loop,double entryYaw,double requestedYaw){
    good(loop.section);const auto& samples=loop.section.samples;double planeYaw=entryYaw,integral=0,previousOmega=0;
    double minimumSignedOmega=INFINITY,maximumOmega=0;bool ascendingVertical=false,descendingVertical=false,inverted=false;
    for(size_t i=0;i<samples.size();++i){const auto& sample=samples[i];const auto& k=loop.section.track.knots[i];
        const Vec3 right=cross(k.tangent,k.up);
        const Vec3 omega=cross(k.tangent,k.curvature)*sample.speed+k.tangent*(dot(k.upFirst,right)*sample.speed);
        const double angle=std::atan2(right.x,-right.y),change=std::remainder(angle-planeYaw,2*pi);planeYaw+=change;
        const double hand=requestedYaw<0?-1:1;
        check(change*hand>=-1e-8,"Loop-plane heading cannot reverse around either vertical tangent");
        near(right.z,0,1e-5,"The authored loop plane remains vertical through inversion");
        minimumSignedOmega=std::min(minimumSignedOmega,omega.z*hand);maximumOmega=std::max(maximumOmega,std::abs(omega.z));
        check(omega.z*hand>=-1e-6,"Physical world-vertical angular velocity keeps the requested yaw hand");
        if(i)integral+=(previousOmega+omega.z)*.5*(sample.time-samples[i-1].time);previousOmega=omega.z;
        ascendingVertical|=k.tangent.z>.999;descendingVertical|=k.tangent.z<-.999;inverted|=k.up.z<-.99;
    }
    check(ascendingVertical&&descendingVertical&&inverted,"Signed yaw is checked through both vertical tangents and the inverted apex");
    near(planeYaw-entryYaw,requestedYaw,1e-5,"Horizontal binormal recovers the requested signed loop-plane change without Euler ambiguity");
    near(integral,requestedYaw,1e-5,"Independent angular-velocity integration recovers the same physical plane rotation");
    if(std::abs(requestedYaw)>.01)check(maximumOmega>.001,"Requested yaw is actual world-vertical rotation, not only final heading metadata");
    std::cout<<"LOOP_PLANE_YAW requested="<<requestedYaw<<" planeChange="<<planeYaw-entryYaw<<" integratedOmegaZ="<<integral
        <<" minimumSignedOmegaZ="<<minimumSignedOmega<<" maximumAbsOmegaZ="<<maximumOmega<<" controls="<<loop.authoring.controls.size()<<'\n';
}
void probeWave(const char* label,FvdWaveRequest request){
    FvdRequest initial;initial.position={};initial.speed=request.entrySpeed;
    initial.forward={std::cos(request.entryPitch),0,std::sin(request.entryPitch)};
    initial.up={-initial.forward.z,0,initial.forward.x};
    initial.rollingAcceleration=request.rollingAcceleration;initial.dragAccelerationCoefficient=request.dragAccelerationCoefficient;
    const double normal=request.entryNormalG==0?std::cos(request.entryPitch):request.entryNormalG;
    initial.controls={{0,normal,0,0},{.02,normal,0,0}};
    const auto portSource=designFvdSection(initial);good(portSource);
    const auto isolated=designFvdWave(request);
    request.entry=makeFvdEntry(portSource.track.knots.front(),request.entrySpeed,request.rollingAcceleration,request.dragAccelerationCoefficient);
    const auto inherited=designFvdWave(request);
    for(const auto& candidate:std::array<std::pair<const char*,const FvdHillResult*>,2>{{{"legacy",&isolated},{"inherited",&inherited}}}){
        const auto& result=candidate.second->section;
        std::cout<<"WAVE_PROBE "<<label<<" path="<<candidate.first<<" speed="<<request.entrySpeed<<" pitch="<<request.entryPitch
            <<" normal="<<normal<<" rise="<<request.height<<" exitHeight="<<request.exitHeight<<" passed="<<(result.report.valid()&&result.assessment.passed)<<'\n';
        if(!result.samples.empty()){const auto& end=result.samples.back();std::cout<<" endpoint="<<end.position.x<<','<<end.position.y<<','<<end.position.z<<" duration="<<end.time<<'\n';}
        for(const auto& error:result.report.errors)std::cout<<error.code<<": "<<error.message<<'\n';
    }
}
}
int main(int argc,char** argv){try{
    if(argc==2&&std::string(argv[1])=="--wave-probe"){
        std::cout<<std::setprecision(12)<<"Diagnostic solver comparisons; failure is a bounded-family solve result, not proof of physical impossibility.\n";
        FvdWaveRequest request;request.rollingAcceleration=gravity*.004;request.dragAccelerationCoefficient=.0002041666666667;
        probeWave("original-pose",request);request.entryPitch=0;probeWave("level-entry-same-intent",request);
        request.entrySpeed=72.9563;request.entryPitch=.02;request.entryNormalG=1.9;request.entryHoldSeconds=1.8;
        request.height=90;request.exitHeight=10;request.bankAngle=73*pi/180;request.exitNormalG=3.5;
        probeWave("rounded-measured-chain-port",request);return 0;
    }

    // Independent banked-circle jets: constant speed, constant load, zero
    // transported tangent twist. No authoring replay generates this oracle.
    const double radius=100,speed=20,bank=std::atan(speed*speed/(radius*gravity));
    Knot circle{{2,3,40},{1,0,0},{0,-1/radius,0},{0,-std::sin(bank),std::cos(bank)},0,Element::Turn,
        {-1/(radius*radius),0,0},{0,1/(radius*radius*radius),0},
        {-std::sin(bank)/radius,0,0},{0,std::sin(bank)/(radius*radius),0},{std::sin(bank)/(radius*radius*radius),0,0}};
    const auto analytic=makeFvdEntry(circle,speed,0,0);
    near(analytic.initial.normalG,1/std::cos(bank),1e-12,"Analytic circle entry normal load");
    near(analytic.initial.lateralG,0,1e-12,"Analytic circle entry lateral load");
    for(double value:analytic.initial.first)near(value,0,1e-12,"Analytic circle has constant controls");
    for(double value:analytic.initial.second)near(value,0,1e-12,"Analytic circle has no hidden control curvature");
    near(analytic.initial.rollRate,0,1e-12,"Analytic circle has zero tangent twist");
    auto inconsistent=circle;inconsistent.upFirst={};bool refused=false;
    try{(void)makeFvdEntry(inconsistent,speed,0,0);}catch(const std::exception&){refused=true;}
    check(refused,"A changing spline frame cannot masquerade as a zero-derivative placeholder");

    FvdRequest prefix;prefix.position={};prefix.speed=65;prefix.rollingAcceleration=gravity*.004;prefix.dragAccelerationCoefficient=.0002041666666667;
    FvdControl last{.6,1.35,.04,.07,.3};last.first={.12,-.05,.03,.2};last.second={-.04,.02,-.01,.05};
    prefix.controls={{0,1,0,0,0},last};const auto before=designFvdSection(prefix);good(before);
    const auto port=makeFvdEntry(before.track.knots.back(),before.samples.back().speed,prefix.rollingAcceleration,prefix.dragAccelerationCoefficient,
        {last.drive,last.first[3],last.second[3]});
    near(port.initial.normalG,last.normalG,1e-9,"Recover incoming normal force without resetting it");
    near(port.initial.lateralG,last.lateralG,1e-9,"Recover incoming lateral force without resetting it");
    near(port.initial.rollRate,last.rollRate,1e-9,"Recover incoming transported twist");
    for(size_t i=0;i<4;++i){near(port.initial.first[i],last.first[i],1e-8,"Recover all incoming control first derivatives");near(port.initial.second[i],last.second[i],1e-8,"Recover all incoming control second derivatives");}
    FvdRequest suffix;suffix.position=port.jet.position;suffix.forward=port.jet.tangent;suffix.up=port.jet.up;suffix.speed=port.speed;
    suffix.rollingAcceleration=prefix.rollingAcceleration;suffix.dragAccelerationCoefficient=prefix.dragAccelerationCoefficient;
    suffix.controls={port.initial,{1,1.4,0,0,0}};suffix.twists={{.2,.8,.1}};suffix.additiveTwists=true;
    const auto after=designFvdSection(suffix);good(after);samePort(after.track.knots.front(),port.jet);
    auto legacy=suffix;legacy.additiveTwists=false;
    const auto ambiguous=designFvdSection(legacy);
    check(!ambiguous.report.valid()&&ambiguous.report.errors.front().code=="FVD_INPUT","Mixed twist ownership requires the explicit additive flag");
    for(auto& c:legacy.controls){c.rollRate=0;c.first[2]=c.second[2]=0;}
    const auto replaced=designFvdSection(legacy);good(replaced);
    check(norm(replaced.track.knots.front().upFirst-port.jet.upFirst)>.0001,"Legacy replacement and additive twist are explicitly different semantics");
    FvdRequest excessive;excessive.speed=65;excessive.additiveTwists=true;
    excessive.controls={{0,1,0,7},{1,1,0,7}};excessive.twists={{0,1,4}};
    const auto overlaid=designFvdSection(excessive);
    check(!overlaid.report.valid()&&overlaid.report.errors.front().code=="FVD_INPUT","Individually bounded channels cannot exceed the combined twist domain");
    Design stored;ForceAuthoring saved;saved.name="inherited-twist";saved.program=suffix;saved.sourceDistances={0,after.track.length};stored.forcePrograms.push_back(saved);
    Design restored;std::string error;const auto payload=authorshipPayload(stored);
    check(parseAuthorshipPayload(payload,restored,error)&&restored.forcePrograms.front().program.additiveTwists,"Persist inherited additive twist semantics");
    const auto replay=designFvdSection(restored.forcePrograms.front().program);good(replay);
    samePort(replay.track.knots.front(),port.jet);near(norm(replay.samples.back().position-after.samples.back().position),0,1e-10,"Saved inherited authoring replays the same geometry");

    // Each existing template also accepts a rotated/translated entry. These
    // neutral-port checks preserve the isolated fixtures without making their
    // old shape the required artistic design for future recipes.
    Knot neutral{{},{1,0,0},{},{0,0,1}};const auto placed=located(neutral,{120,-80,35},.37);
    FvdLoopRequest loop;loop.rollingAcceleration=prefix.rollingAcceleration;loop.dragAccelerationCoefficient=prefix.dragAccelerationCoefficient;
    loop.entry=makeFvdEntry(placed,loop.entrySpeed,loop.rollingAcceleration,loop.dragAccelerationCoefficient);
    const auto looped=designFvdLoop(loop);good(looped.section);samePort(looped.section.track.knots.front(),placed);
    near(std::remainder(std::atan2(looped.section.samples.back().forward.y,looped.section.samples.back().forward.x)-.37-loop.yawAngle,2*pi),0,1e-6,"Inherited loop yaw is relative to its actual entry heading");
    loopPlaneYaw(looped,.37,loop.yawAngle);
    check(validateSelfClearance(looped.section.track,TrainConfig{}).valid(),
        "Completing the authored plane yaw on ascent separates the gentle loop's rising and descending branches");

    auto mirrored=loop;mirrored.yawAngle=-loop.yawAngle;
    const auto reverseLoop=designFvdLoop(mirrored);loopPlaneYaw(reverseLoop,.37,mirrored.yawAngle);samePort(reverseLoop.section.track.knots.front(),placed);
    const Vec3 forwardEnd=yawed(looped.section.samples.back().position-placed.position,-.37);
    const Vec3 mirroredEnd=yawed(reverseLoop.section.samples.back().position-placed.position,-.37);
    near(mirroredEnd.x,forwardEnd.x,1e-5,"Mirrored plane-yaw retains the forward footprint");
    near(mirroredEnd.y,-forwardEnd.y,1e-5,"Mirrored plane-yaw mirrors lateral geometry");
    near(mirroredEnd.z,forwardEnd.z,1e-5,"Mirrored plane-yaw retains the same elevation history");
    Design loopSaved;ForceAuthoring loopSource;loopSource.name="coordinated-loop";loopSource.program=looped.authoring;
    loopSource.sourceDistances={0,looped.section.track.length};loopSaved.forcePrograms.push_back(loopSource);Design loopLoaded;
    check(parseAuthorshipPayload(authorshipPayload(loopSaved),loopLoaded,error),"Coordinated controls fit the existing persisted authoring format");
    const auto loopReplay=designFvdSection(loopLoaded.forcePrograms.front().program);good(loopReplay);samePort(loopReplay.track.knots.front(),placed);
    near(norm(loopReplay.samples.back().position-looped.section.samples.back().position),0,1e-9,"Saved coordinated force controls replay the same spatial loop");
    auto splitLoop=loop;splitLoop.normalG=4.4;splitLoop.exitPositiveG=3.9;
    const auto strongLoop=designFvdLoop(splitLoop);good(strongLoop.section);samePort(strongLoop.section.track.knots.front(),placed);
    check(validateSelfClearance(strongLoop.section.track,TrainConfig{}).valid(),
        "A stronger entry with a separate recovery load clears the original colliding loop fixture");
    const auto apex=std::max_element(strongLoop.section.samples.begin(),strongLoop.section.samples.end(),[](const auto& a,const auto& b){return a.position.z<b.position.z;});
    double entryPeak=0,recoveryPeak=0;
    for(const auto& q:strongLoop.section.samples){const double load=sampleFvdControl(strongLoop.authoring.controls,q.time).normalG;
        (q.time<apex->time?entryPeak:recoveryPeak)=std::max(q.time<apex->time?entryPeak:recoveryPeak,load);}
    near(entryPeak,4.4,1e-9,"Distinct loop entry force is retained through coordinated yaw");
    near(recoveryPeak,3.9,1e-9,"Descending loop recovery does not inherit the higher entry force");
    loop.entry=port;const auto liveLoop=designFvdLoop(loop);
    check(!liveLoop.section.report.valid()&&liveLoop.section.report.errors.front().code=="FVD_LOOP_ENTRY_FAMILY",
        "A nonplanar inherited loop entry rejects explicitly instead of resetting live lateral/twist jets");
    FvdWaveRequest wave;wave.rollingAcceleration=prefix.rollingAcceleration;wave.dragAccelerationCoefficient=prefix.dragAccelerationCoefficient;
    // Match the actual isolated fixture's -15-degree entry, rather than
    // silently changing it to a level port while retaining its 75 m rise and
    // -11 m exit constraints. --wave-probe records that separate level case.
    auto wavePort=neutral;wavePort.tangent={std::cos(wave.entryPitch),0,std::sin(wave.entryPitch)};
    wavePort.up={-wavePort.tangent.z,0,wavePort.tangent.x};wavePort=located(wavePort,{120,-80,35},.37);
    wave.entry=makeFvdEntry(wavePort,wave.entrySpeed,wave.rollingAcceleration,wave.dragAccelerationCoefficient);
    const auto waved=designFvdWave(wave);good(waved.section);samePort(waved.section.track.knots.front(),wavePort);
    FvdLoopRequest fromWave;fromWave.rollingAcceleration=wave.rollingAcceleration;fromWave.dragAccelerationCoefficient=wave.dragAccelerationCoefficient;
    fromWave.entry=makeFvdEntry(waved.section.track.knots.back(),waved.section.samples.back().speed,fromWave.rollingAcceleration,fromWave.dragAccelerationCoefficient);
    fromWave.height=145*std::pow(fromWave.entry->speed/65,2);
    const auto connectedLoop=designFvdLoop(fromWave);good(connectedLoop.section);samePort(connectedLoop.section.track.knots.front(),fromWave.entry->jet);
    const double waveYaw=std::atan2(fromWave.entry->jet.tangent.y,fromWave.entry->jet.tangent.x);
    loopPlaneYaw(connectedLoop,waveYaw,fromWave.yawAngle);
    FvdImmelmannRequest immelmann;immelmann.rollingAcceleration=prefix.rollingAcceleration;immelmann.dragAccelerationCoefficient=prefix.dragAccelerationCoefficient;
    immelmann.entry=makeFvdEntry(placed,immelmann.entrySpeed,immelmann.rollingAcceleration,immelmann.dragAccelerationCoefficient);
    const auto inverted=designFvdImmelmann(immelmann);good(inverted.section);samePort(inverted.section.track.knots.front(),placed);
    check(designFvdLoop(loop,[]{return true;}).section.cancelled,"Inherited loop cancellation remains explicit");

    auto splitImmelmann=immelmann;splitImmelmann.height=90;splitImmelmann.normalG=4.64;splitImmelmann.exitPositiveG=3.8;splitImmelmann.crestG=1.1;
    const auto strongerInversion=designFvdImmelmann(splitImmelmann);good(strongerInversion.section);
    double ascentLoad=0,valleyLoad=0;
    for(const auto& q:strongerInversion.section.samples){const double load=sampleFvdControl(strongerInversion.authoring.controls,q.time).normalG;
        if(q.time<strongerInversion.apex.time)ascentLoad=std::max(ascentLoad,load);else valleyLoad=std::max(valleyLoad,load);}
    near(ascentLoad,4.64,1e-9,"Immelmann retains its stronger ascending shoulder");near(valleyLoad,3.8,1e-9,"Immelmann recovery has a separately authored load");

    FvdApproachRequest approach;approach.rollingAcceleration=prefix.rollingAcceleration;approach.dragAccelerationCoefficient=prefix.dragAccelerationCoefficient;
    const auto approachPort=located(neutral,{200,-100,4.5},pi/3);
    approach.entry=makeFvdEntry(approachPort,50,approach.rollingAcceleration,approach.dragAccelerationCoefficient);
    approach.endPosition={550,150,4.5};approach.endHeading=0;
    const auto arrived=designFvdApproach(approach);good(arrived.section);samePort(arrived.section.track.knots.front(),approachPort);
    near(norm(arrived.section.samples.back().position-approach.endPosition),0,1e-7,"Force-authored approach reaches its actual final position without snapping");
    double previousHeading=pi/3;
    for(const auto& q:arrived.section.samples){
        near(q.position.z,4.5,1e-7,"Coordinated normal force preserves the level approach");
        near(sampleFvdControl(arrived.authoring.controls,q.time).normalG,1/q.up.z,1e-7,"Independent gravity balance verifies the prescribed bank/load relationship");
        const double heading=std::atan2(q.forward.y,q.forward.x);
        check(heading<=previousHeading+1e-8,"The station turn never reverses its intended heading change");previousHeading=heading;
    }
    near(previousHeading,0,1e-8,"Force-authored approach reaches the brake heading");
    auto displaced=approach;displaced.endPosition.z+=1;
    check(!designFvdApproach(displaced).section.report.valid(),"A level family cannot conceal a mismatched station height");
    auto curved=approach;curved.entry=analytic;
    check(!designFvdApproach(curved).section.report.valid(),"Approach cannot erase a live curved incoming port");
    check(designFvdApproach(approach,[]{return true;}).section.cancelled,"Approach shooting cancellation remains explicit");

    FvdHillRequest terminalHill;terminalHill.entry=makeFvdEntry(placed,55,prefix.rollingAcceleration,prefix.dragAccelerationCoefficient);
    terminalHill.height=35;terminalHill.exitHeight=-2;terminalHill.exitPitch=0;terminalHill.positiveG=3.2;terminalHill.airtimeG=-1.25;
    terminalHill.rampSeconds=1;terminalHill.exitRampSeconds=1.5;terminalHill.twistAngle=25*pi/180;terminalHill.exitNormalG=1;
    terminalHill.rollingAcceleration=prefix.rollingAcceleration;terminalHill.dragAccelerationCoefficient=prefix.dragAccelerationCoefficient;
    const auto finishedHill=designFvdHill(terminalHill);good(finishedHill.section);samePort(finishedHill.section.track.knots.front(),placed);
    const auto& terminal=finishedHill.section.track.knots.back();
    near(terminal.position.z+500*terminal.tangent.z,placed.position.z-2,1e-7,
        "Terminal hill resolves its physical pitch/height accurately enough for a long station straight");
    near(norm(terminal.up-Vec3{0,0,1}),0,1e-8,"Terminal hill unbanks within its force program");
    near(norm(terminal.curvature)+norm(terminal.third)+norm(terminal.fourth),0,1e-7,"Terminal force release has straight complete geometry jets");
    const auto terminalPort=makeFvdEntry(terminal,finishedHill.section.samples.back().speed,terminalHill.rollingAcceleration,terminalHill.dragAccelerationCoefficient);
    near(terminalPort.initial.normalG,1,1e-9,"The following approach inherits a physical1g port without resetting it");

    // Two true source owners cannot overwrite a shared physical frame. A
    // constant rolled frame with compensating force stays exactly straight,
    // so position-jet checks alone cannot detect this mismatch.
    Design design;MotionBuilder builder(design);FvdRequest straight;straight.position={};straight.controls={{0,1,0,0},{1,1,0,0}};
    const auto first=designFvdSection(straight);good(first);builder.force(first,straight,Element::Turn,"first",builder.cursor.position);
    const auto old=design.track.knots.back();const auto count=design.track.knots.size();const auto owned=builder.fvdEntry(straight.speed);samePort(owned.jet,old);
    auto wrong=straight;wrong.up={0,-std::sin(.2),std::cos(.2)};wrong.controls={{0,std::cos(.2),-std::sin(.2),0},{1,std::cos(.2),-std::sin(.2),0}};
    const auto second=designFvdSection(wrong);good(second);refused=false;
    try{builder.force(second,wrong,Element::Turn,"wrong-frame",builder.cursor.position);}catch(const std::exception& e){refused=std::string(e.what()).find("physical frame")!=std::string::npos;}
    check(refused,"Adjacent force sources reject a genuine physical frame mismatch");
    check(design.track.knots.size()==count&&design.forcePrograms.size()==1,"Frame mismatch is rejected before adding source or geometry");samePort(design.track.knots.back(),old);
    // The live ravine exit can be almost level without being exactly level.
    // Resetting even this tiny pitch produced a large first-span jerk artifact.
    FvdHillRequest returnHill;returnHill.height=35;returnHill.exitHeight=0;returnHill.exitPitch=-.12;
    returnHill.positiveG=3.2;returnHill.rampSeconds=1;returnHill.exitRampSeconds=1.5;
    returnHill.rollingAcceleration=gravity*.004;returnHill.dragAccelerationCoefficient=.0002041666666667;
    const auto live=detail::planarJet({3000,-2000,60},.37,3.35e-11);
    const Knot liveKnot{live.position,live.tangent,live.curvature,
        unit(Vec3{0,0,1}-live.tangent*live.tangent.z),0,Element::Return,live.third,live.fourth};
    returnHill.entry=makeFvdEntry(liveKnot,55,returnHill.rollingAcceleration,returnHill.dragAccelerationCoefficient);
    const auto inheritedHill=designFvdHill(returnHill);good(inheritedHill.section);
    samePort(inheritedHill.section.track.knots.front(),liveKnot);
    near(norm(inheritedHill.section.track.knots.front().tangent-liveKnot.tangent),0,2e-15,
        "Return hill inherits a tiny physical pitch instead of resetting it inside the join tolerance");

    // Force and twist boundaries must survive source remeshing; a uniform
    // grid can put the onset of angular jerk inside a canonical span.
    Design phases;MotionBuilder phaseBuilder(phases);FvdRequest phaseRequest;
    phaseRequest.position={};phaseRequest.speed=40;
    phaseRequest.controls={{0,1,0,0},{.41,2,0,0},{1.07,1.3,0,0},{1.6,1,0,0}};
    phaseRequest.twists={{.137,.853,.3}};
    const auto phaseSource=designFvdSection(phaseRequest);good(phaseSource);
    phaseBuilder.force(phaseSource,phaseRequest,Element::Turn,"phases",phaseBuilder.cursor.position);
    phases.track.closed=false;phases.track.authoredFrame=true;phases.track.rebuild();
    for(double time:{.137,.41,.853,1.07}){
        const auto sample=std::find_if(phaseSource.samples.begin(),phaseSource.samples.end(),[&](const auto& q){return q.time==time;});
        check(sample!=phaseSource.samples.end(),"Canonical source integration retains every force and twist boundary");
        const double distance=phaseSource.track.spans[size_t(sample-phaseSource.samples.begin())].start;
        const auto& retained=phases.forcePrograms.front().sourceDistances;
        check(std::find(retained.begin(),retained.end(),distance)!=retained.end(),"Final geometry retains every source phase boundary");
    }
    Frame beginFrame,endFrame;beginFrame.speed=endFrame.speed=40;endFrame.time=phases.track.length/40;endFrame.distance=phases.track.length;
    const auto agreement=assessFvdAngularAgreement(phases.track,phases.forcePrograms.front(),phaseSource,{beginFrame,endFrame},{1e-5,1e-4,.01},8);
    check(agreement.report.valid(),"Eight interior subdivisions preserve physical source angular velocity, acceleration and jerk");
    std::cout<<"PASS "<<checks<<" inherited FVD port checks\n";return 0;
}catch(const std::exception& e){std::cerr<<"FAIL after "<<checks<<": "<<e.what()<<'\n';return 1;}}
