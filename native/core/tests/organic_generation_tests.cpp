#include "coaster/coaster.hpp"
#include <iostream>
#include <stdexcept>
#include <tuple>
using namespace coaster;
namespace {
int checks=0;
void check(bool value,const char* message){++checks;if(!value)throw std::runtime_error(message);}
struct ObservedInversions {int fullLoop{},immelmann{},diveLoop{};double immelmannRise{},immelmannForwardExtent{};};
ObservedInversions classifyGeometry(const Design& design){
    ObservedInversions observations;
    // Type is inferred from canonical tangent, up and elevation, independently
    // of generator module names or planning diagnostics. The label merely
    // supplies each measured span group's interval.
    for(const auto& region:design.inversionDimensions){
        auto start=design.track.sample(region.startDistance),end=design.track.sample(region.endDistance);
        Vec3 forward=unit(Vec3{start.tangent.x,start.tangent.y,0});
        Vec3 exitForward=unit(Vec3{end.tangent.x,end.tangent.y,0});
        double headingAgreement=dot(forward,exitForward),rise=end.position.z-start.position.z;
        constexpr double portAngle=15*pi/180,phaseTolerance=2*portAngle,headingCosine=.984807753012208;
        double pitch=std::atan2(start.tangent.z,dot(start.tangent,forward)),initialPitch=pitch,roll=0,initialRoll=0;
        double firstAscendingVertical=INFINITY,firstDescendingVertical=INFINITY,ascendingRoll=0,descendingRoll=0;
        double invertedApex=-INFINITY;const int samples=int(std::ceil(region.pathLength/.25));
        for(int i=0;i<=samples;++i){
            double distance=region.startDistance+region.pathLength*i/samples;auto sample=design.track.sample(distance);
            check(finite(sample.position)&&finite(sample.tangent)&&finite(sample.up),"Generated canonical inversion has finite frames");
            double projected=dot(sample.tangent,forward);
            check(std::hypot(projected,sample.tangent.z)>.5,"Pitch topology retains a resolved vertical-plane projection");
            pitch+=std::remainder(std::atan2(sample.tangent.z,projected)-pitch,2*pi);
            Vec3 pitchUp=Vec3{0,0,1}*std::cos(pitch)-forward*std::sin(pitch);
            pitchUp=unit(pitchUp-sample.tangent*dot(pitchUp,sample.tangent));
            double angle=std::atan2(dot(sample.up,cross(sample.tangent,pitchUp)),dot(sample.up,pitchUp));
            roll+=std::remainder(angle-roll,2*pi);if(i==0)initialRoll=roll;
            if(sample.tangent.z>std::cos(portAngle)&&!std::isfinite(firstAscendingVertical)){firstAscendingVertical=distance;ascendingRoll=roll;}
            if(sample.tangent.z< -std::cos(portAngle)&&!std::isfinite(firstDescendingVertical)){firstDescendingVertical=distance;descendingRoll=roll;}
            if(sample.up.z<-.5)invertedApex=std::max(invertedApex,sample.position.z-design.request.terrain.height(sample.position.x,sample.position.y));
        }
        double pitchSweep=pitch-initialPitch,rollSweep=roll-initialRoll;
        // A full pitch revolution survives borrowed rising ports. A barrel
        // roll has no pitch revolution; a horizontal hairpin lacks a vertical
        // landmark. Neither can substitute for these actual inversion shapes.
        // The exit may already turn into the adjoining S while completing
        // the same full pitch revolution; it need not recover a straight port.
        if(headingAgreement>std::cos(phaseTolerance)&&std::abs(pitchSweep-2*pi)<phaseTolerance){
            check(std::isfinite(firstAscendingVertical)&&std::isfinite(firstDescendingVertical)&&firstAscendingVertical<firstDescendingVertical,"Retained full loop contains ascent and descent pitch in its original heading");
            check(std::abs(rollSweep)<phaseTolerance,"Full loop is a pitch revolution without an added barrel roll");
            ++observations.fullLoop;
        }else if(headingAgreement< -.9&&region.verticalExtent>70&&rise>=-1&&rise<40&&std::abs(pitchSweep-pi)<phaseTolerance){
            check(std::isfinite(firstAscendingVertical)&&!std::isfinite(firstDescendingVertical),"Immelmann contains an actual ascending half-loop");
            check(std::abs(std::abs(rollSweep)-pi)<phaseTolerance&&std::abs(ascendingRoll-initialRoll)<pi/3,"Compound Immelmann reaches ascending vertical before completing most of its half-roll");
            observations.immelmannRise=region.verticalExtent;observations.immelmannForwardExtent=region.forwardExtent;++observations.immelmann;
        }else if(headingAgreement< -headingCosine&&rise< -70&&std::abs(pitchSweep+pi)<phaseTolerance){
            check(std::isfinite(firstDescendingVertical)&&!std::isfinite(firstAscendingVertical),"Dive loop contains an actual descending half-loop");
            check(std::abs(std::abs(rollSweep)-pi)<phaseTolerance&&std::abs(descendingRoll-initialRoll)>2*pi/3,"Compound dive loop completes most of its half-roll before descending vertical");
            ++observations.diveLoop;
        }else check(false,"Every inversion group in the paired fixture has a geometrically recognized complete topology");
        check(invertedApex>=design.request.targets.inversionHeight,"Each genuine inverted apex meets the terrain-relative target");
    }
    check(observations.fullLoop==1&&observations.immelmann==1&&observations.diveLoop==0,"Generated ride retains its full loop and complete low-exit Immelmann");
    return observations;
}
void checkPropulsionCorridors(const Design& design){
    constexpr double carAlignmentMargin=1.5;
    const double trainSpan=(design.request.train.cars-1)*design.request.train.spacing;
    for(const auto& operation:design.operations){
        if(operation.kind!=DriveKind::Launch&&operation.kind!=DriveKind::Boost)continue;
        const auto first=design.track.sample(operation.start-carAlignmentMargin);
        const double heading=std::atan2(first.tangent.y,first.tangent.x),pitch=std::asin(first.tangent.z);
        double minimumPitch=pitch,maximumPitch=pitch;
        auto observe=[&](double distance){
            const auto k=sampleKinematics(design.track,distance);const auto& p=k.sample;
            const Vec3 upright=unit(Vec3{0,0,1}-p.tangent*p.tangent.z);
            check(std::abs(std::remainder(std::atan2(p.tangent.y,p.tangent.x)-heading,2*pi))<.001,"Every motor retains one fixed plan heading throughout the powered car footprint");
            check(std::abs(cross(p.tangent,p.curvature).z)<1e-5&&std::abs(p.curvature.z)<1e-4,"Every powered car remains on essentially linear geometry");
            check(dot(p.up,upright)>.9998&&norm(k.upS)<.001,"Each powered car remains upright and aligned with its motor");
            minimumPitch=std::min(minimumPitch,std::asin(p.tangent.z));maximumPitch=std::max(maximumPitch,std::asin(p.tangent.z));
        };
        for(double distance=operation.start-carAlignmentMargin;distance<operation.end+carAlignmentMargin;distance+=.125)observe(distance);
        observe(operation.end+carAlignmentMargin);
        for(const auto& span:design.track.spans)if(span.start>=operation.start-carAlignmentMargin&&span.start<=operation.end+carAlignmentMargin)observe(span.start);
        check(maximumPitch-minimumPitch<.005,"A locally gentle motor path also retains nearly constant grade over its full length");
        // Datum fitting has a 0.1-degree allowance around the visual five-degree slope.
        check(std::max(std::abs(minimumPitch),std::abs(maximumPitch))<=.05*pi/180||minimumPitch>=4.9*pi/180||maximumPitch<=-4.9*pi/180,"Every active-car motor footprint is genuinely level or visibly inclined");
        check(operation.end-operation.start<300,"Motor straights reserve working length without long empty corridors");
        double actualWorkSeconds=0;
        for(size_t i=1;i<design.simulation.frames.size();++i){const auto& f=design.simulation.frames[i];const auto& before=design.simulation.frames[i-1];
            if(f.distance-trainSpan*.5<=operation.start||f.distance+trainSpan*.5>=operation.end||f.speed>=operation.targetSpeed-.1)continue;
            const double acceleration=(f.speed-before.speed)/(f.time-before.time);
            double grade=0;for(int car=0;car<design.request.train.cars;++car)grade+=design.track.sample(f.distance+trainSpan*.5-car*design.request.train.spacing).tangent.z/design.request.train.cars;
            const double resistance=gravity*design.request.train.rollingResistance+.5*design.request.train.airDensity*design.request.train.dragCdA*f.speed*f.speed/(design.request.train.cars*design.request.train.carMass);
            if(acceleration+gravity*grade+resistance>.1)actualWorkSeconds+=f.time-before.time;
        }
        check(actualWorkSeconds>.15,"Each section booster performs measured work below target speed");
    }

}
void checkTerminalBrake(const Design& design){
    const auto station=std::find_if(design.operations.begin(),design.operations.end(),[](const Operation& op){return op.kind==DriveKind::Station;});
    check(station!=design.operations.end(),"The terminal brake remains an explicit physical operation");
    const double trainSpan=(design.request.train.cars-1)*design.request.train.spacing;
    const double turnExit=station->start-trainSpan,finish=design.track.length+trainSpan*.5+30;
    const auto brakeEntry=design.track.sample(turnExit);
    const double heading=std::atan2(brakeEntry.tangent.y,brakeEntry.tangent.x);
    for(double distance=turnExit;distance<finish+trainSpan*.5;distance+=.25){const auto p=design.track.sample(distance);
        const Vec3 upright=unit(Vec3{0,0,1}-p.tangent*p.tangent.z);
        check(std::abs(std::remainder(std::atan2(p.tangent.y,p.tangent.x)-heading,2*pi))<.001&&std::abs(cross(p.tangent,p.curvature).z)<1e-5&&dot(p.up,upright)>.9998,"The full braking train is upright and plan-straight after clearing the final bank; a vertical grade is allowed");
    }
    for(double distance=design.track.length+design.station.boardingBegin;distance<=finish+trainSpan*.5;distance+=.25){const auto p=design.track.sample(distance);
        check(std::abs(p.tangent.z)<.001&&p.up.z>.9999,"The actual station bay and whole-train stopping continuation remain level across the seam");
    }
    // Lowering the ending may make this approach level. It must still hand
    // over to the fixed station without adding a new uphill element.
    const double brakeDrop=brakeEntry.position.z-design.track.sample(design.track.length).position.z;
    check(brakeDrop>=-.1,"The final brake reaches the level station without an artificial uphill approach");
    check(station->stopDeceleration==6&&station->maxForce==design.request.train.carMass*7&&station->rampSeconds==.5,"Graded braking retains the original net target, bounded actuator capacity and entry ramp");
    auto timeAt=[&](double distance){auto right=std::lower_bound(design.simulation.frames.begin(),design.simulation.frames.end(),distance,[](const Frame& f,double s){return f.distance<s;});const auto& left=*(right-1);double u=(distance-left.distance)/(right->distance-left.distance);return left.time+u*(right->time-left.time);};
    const double entryTime=timeAt(turnExit);
    auto entry=std::lower_bound(design.simulation.frames.begin(),design.simulation.frames.end(),turnExit,[](const Frame& f,double s){return f.distance<s;});
    check(entry->speed>40,"Terminal banked turn coasts at healthy ride speed until the straight brake approach");
    check(design.simulation.frames.back().speed==0&&std::abs(design.simulation.frames.back().distance-finish)<.25,"Physical braking reaches actual zero speed at the unchanged station reference without a snap");
    check(design.simulation.frames.back().time-entryTime<14,"Straight terminal stopping no longer spends the banked turn creeping toward a distant target");
}
void checkDeadTrack(const Design& design){
    // Constant heading, pitch and roll count as dead track at any grade.
    // Exclude powered spans, which require straight corridors.
    std::vector<char> powered(size_t(design.track.length)+2,0);
    for(const auto& operation:design.operations){
        const double end=operation.kind==DriveKind::Station?design.track.length:operation.end;
        for(double s=std::max(0.,operation.start-40);s<std::min(design.track.length,end+40);s+=1)powered[size_t(s)]=1;}
    constexpr double h=2,still=2e-4;double dead=0,longest=0,run=0;
    const auto& frames=design.simulation.frames;
    for(size_t i=1;i<frames.size();++i){
        const double s=frames[i].distance;
        if(s<0||s>=design.track.length||powered[size_t(s)]){run=0;continue;}
        const auto a=design.track.sample(std::max(0.,s-h)),b=design.track.sample(std::min(design.track.length,s+h));
        const double pitchRate=std::abs(std::asin(std::clamp(b.tangent.z,-1.,1.))-std::asin(std::clamp(a.tangent.z,-1.,1.)))/(2*h);
        const double headingRate=std::abs(std::remainder(std::atan2(b.tangent.y,b.tangent.x)-std::atan2(a.tangent.y,a.tangent.x),2*pi))/(2*h);
        const double rollRate=std::abs(dot(cross(a.up,b.up),design.track.sample(s).tangent))/(2*h);
        if(pitchRate<still&&headingRate<still&&rollRate<still){const double step=frames[i].time-frames[i-1].time;dead+=step;run+=step;longest=std::max(longest,run);}
        else run=0;
    }
    std::cout<<"deadSeconds="<<dead<<" deadShare="<<dead/design.simulation.metrics.duration<<" longestDead="<<longest<<"\n";
    check(dead<design.simulation.metrics.duration*.09,"Unpowered geometry that holds heading, pitch and roll constant stays a small share of the ride");
    check(longest<5,"No single stretch leaves the rider on unchanging geometry for five seconds");
}
void checkC3Transitions(const Design& design){
    double largestUpJump=0,largestTangentJump=0;
    for(size_t i=0;i<design.track.spans.size();++i){
        const auto a=sampleSpanKinematics(design.track,i,1),b=sampleSpanKinematics(design.track,(i+1)%design.track.spans.size(),0);
        check(norm(a.sample.position-b.sample.position)<1e-8&&norm(a.sample.tangent-b.sample.tangent)<1e-9,"Every generated position/tangent join closes, including the station seam");
        check(norm(a.sample.curvature-b.sample.curvature)<1e-9&&norm(a.curvatureS-b.curvatureS)<1e-9,"Every generated centreline join agrees through the third arc derivative");
        check(norm(a.sample.up-b.sample.up)<1e-9&&norm(a.upS-b.upS)<1e-9&&norm(a.upSS-b.upSS)<1e-9,"Every generated roll/up join agrees through second order");
        largestUpJump=std::max(largestUpJump,norm(a.upSSS-b.upSSS));largestTangentJump=std::max(largestTangentJump,norm(a.curvatureSS-b.curvatureSS));
    }
    check(largestUpJump<1e-9&&largestTangentJump<1e-9,"The complete physical rider frame is C3 at every knot, module transition and closed seam");
    std::cout<<"maximumUpThirdJump="<<largestUpJump<<" maximumTangentThirdJump="<<largestTangentJump<<'\n';
}
Design generateChecked(uint64_t seed){
    GenerationRequest request;request.seed=seed;request.targets.requireIntensity=false;
    request.maxCandidates=1;
    auto design=generate(request);
    if(!design.accepted())for(const auto* report:{&design.report,&design.simulation.report})for(const auto& error:report->errors)std::cerr<<"seed "<<seed<<' '<<error.code<<": "<<error.message<<'\n';
    check(design.accepted(),"Entire generated circuit passes unmodified geometry, train forces, target and convergence gates");
    check(design.simulation.metrics.duration<=200&&design.simulation.frames.back().speed==0,"Ordinary maintenance fixtures complete the physical stop within200seconds");
    check(design.convergence.coarseStep==1./960&&design.convergence.fineStep==1./1920,"Full ride uses required 960/1920 Hz simulation and verification");
    check(design.simulation.metrics.maxGroundHeight>=request.targets.height&&std::abs(design.simulation.metrics.maxSpeed-request.targets.speed)<=request.targets.speed*.01,"Record hill clears its height and the ride lands on the dialled speed setpoint");
    std::cout<<"seed="<<seed<<" accepted=true candidate="<<design.candidate<<" length="<<design.track.length<<" topology="<<design.topology<<'\n';
    return design;
}
bool same(Vec3 a,Vec3 b){return a.x==b.x&&a.y==b.y&&a.z==b.z;}
}
int main(){try{
    auto first=generateChecked(42);classifyGeometry(first);checkPropulsionCorridors(first);checkTerminalBrake(first);checkDeadTrack(first);checkC3Transitions(first);
    check(first.track.length<6000,"The complete record ride avoids the old multi-kilometre transit corridors");
    check(first.simulation.metrics.minVerticalG<0,"The complete ride includes actual measured airtime");
    // A rising hero element changes heading before its crest; pitch and bank
    // carry into the descending sweep without an upright straight reset.
    double hillHeading=0;bool hill=false;TrackSample previous{};
    for(double s=200;s<first.track.length;s+=2){auto q=first.track.sample(s);if(q.element==Element::Hill){if(hill)hillHeading+=std::abs(std::remainder(std::atan2(q.tangent.y,q.tangent.x)-std::atan2(previous.tangent.y,previous.tangent.x),2*pi));hill=true;previous=q;}else if(hill)break;}
    check(hillHeading>25*pi/180,"The main hill participates in the route's turn instead of terminating a straight corridor");
    auto second=generateChecked(5);checkPropulsionCorridors(second);checkTerminalBrake(second);
    check(std::abs(first.track.length-second.track.length)>1,"Seeds change the physical route");
    for(const auto& menu:std::array<std::pair<double,double>,2>{{{180,65},{240,80}}}){
        GenerationRequest request;request.targets.requireIntensity=false;request.targets.height=menu.first;request.targets.speed=menu.second;
        const auto design=generate(request);check(design.accepted(),"Both menu extremes pass the unchanged physical gates");checkPropulsionCorridors(design);checkTerminalBrake(design);
    }
    GenerationRequest intense;intense.seed=1;intense.targets.referenceId="TEST_ONLY_SYNTHETIC_NOT_I305";intense.targets.referenceExposure=30;
    const auto held=generate(intense);check(held.accepted()&&held.simulation.metrics.exposure10Seconds>=33,"A configured sustained-intensity target changes the route and meets its measured exposure without relaxing force or clearance gates");
    const auto repeated=generateChecked(42);bool identical=first.track.knots.size()==repeated.track.knots.size();
    if(identical)for(size_t i=0;i<first.track.knots.size();++i){const auto& a=first.track.knots[i];const auto& b=repeated.track.knots[i];identical&=same(a.position,b.position)&&same(a.tangent,b.tangent)&&same(a.curvature,b.curvature)&&same(a.up,b.up)&&a.bank==b.bank&&a.element==b.element;}
    check(identical&&reportJson(first)==reportJson(repeated),"Seed reproduces every canonical knot and independently measured telemetry");
    GenerationRequest request;auto cancelled=generate(request,[]{return true;});check(cancelled.simulation.cancelled&&!cancelled.accepted(),"Cancelled generation cannot be accepted");
    auto strict=first;strict.report={};strict.request.targets.requireIntensity=true;evaluateTargets(strict);check(!strict.accepted(),"Missing benchmark cannot establish an all-records claim");
    std::cout<<"PASS "<<checks<<" complete-route physics, C3 transitions, inversion topology, propulsion, pacing and determinism checks\n";return 0;
}catch(const std::exception& e){std::cerr<<"FAIL after "<<checks<<": "<<e.what()<<'\n';return 1;}}
