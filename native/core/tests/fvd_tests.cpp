#include "coaster/fvd.hpp"
#include <iostream>
#include <stdexcept>
using namespace coaster;
namespace {
int checks=0;
void check(bool condition,const char* message){++checks;if(!condition)throw std::runtime_error(message);}
void near(double value,double expected,double tolerance,const char* message){check(std::isfinite(value)&&std::abs(value-expected)<=tolerance,message);}
void good(const FvdResult& r){
    if(!r.report.valid()){
        for(const auto& e:r.report.errors)std::cerr<<e.code<<": "<<e.message<<'\n';
        std::cerr<<"residual normal="<<r.assessment.maxNormalResidualG<<" lateral="<<r.assessment.maxLateralResidualG<<" roll="<<r.assessment.maxRollResidualRadPerSecond<<" distance="<<r.assessment.endDistanceError<<" speed="<<r.assessment.endSpeedError<<'\n';
    }
    check(r.integrated&&r.canonicalBuilt&&!r.cancelled&&r.report.valid()&&r.assessment.performed&&r.assessment.passed,"Section integrates, fits and passes its sampled point replay");
}
FvdRequest constant(double normal,double lateral,double duration=1){FvdRequest r;r.controls={{0,normal,lateral,0},{duration,normal,lateral,0}};return r;}
}
int main(){try{
    FvdLoopRequest loop;loop.rollingAcceleration=gravity*.004;loop.dragAccelerationCoefficient=.0002041666666667;
    const auto looped=designFvdLoop(loop);good(looped.section);
    const auto& loopEnd=looped.section.samples.back();
    near(std::atan2(loopEnd.forward.y,loopEnd.forward.x),loop.yawAngle,1e-6,"Loop has actual requested endpoint heading change");
    near(loopEnd.forward.z,0,1e-6,"The full loop finishes level without forcing endpoint XYZ");
    near(norm(loopEnd.up-Vec3{0,0,1}),0,1e-6,"Loop returns its physical rider frame upright");
    double loopHeight=0,minimumUp=1,lateralMotion=0;
    for(const auto& point:looped.section.samples){loopHeight=std::max(loopHeight,point.position.z);minimumUp=std::min(minimumUp,point.up.z);lateralMotion=std::max(lateralMotion,std::abs(point.position.y));}
    near(loopHeight,loop.height,1e-4,"Loop reaches its independently measured requested apex");
    check(minimumUp<-.9&&lateralMotion>5,"Loop contains a real inversion and spatial yaw motion");
    check(looped.authoring.controls[3].normalG==loop.crestG&&looped.authoring.controls[4].normalG==loop.crestG,"An explicit low-load crest separates the high-load flanks");
    check(std::abs(looped.authoring.controls[1].lateralG)>.01,"Requested loop yaw is authored by actual lateral acceleration");
    near(looped.authoring.controls.back().normalG,loop.exitNormalG,0,"Loop exposes the requested live exit load");
    std::cout<<"Loop fixture: speed="<<loop.entrySpeed<<" height="<<loopHeight<<" end="<<loopEnd.position.x<<","<<loopEnd.position.y<<","<<loopEnd.position.z<<" length="<<looped.section.track.length<<" duration="<<loopEnd.time<<'\n';
    auto mirroredLoop=loop;mirroredLoop.yawAngle=-loop.yawAngle;
    const auto mirror=designFvdLoop(mirroredLoop);good(mirror.section);
    near(mirror.section.samples.back().position.x,loopEnd.position.x,1e-5,"Loop handedness preserves forward footprint");
    near(mirror.section.samples.back().position.y,-loopEnd.position.y,1e-5,"Loop yaw mirrors the integrated lateral geometry");
    check(designFvdLoop(loop,[]{return true;}).section.cancelled,"Loop solve propagates cancellation");
    FvdImmelmannRequest inversion;
    inversion.rollingAcceleration=gravity*.002;
    inversion.dragAccelerationCoefficient=.5*1.225*2.4/9000;
    const auto immelmann=designFvdImmelmann(inversion);good(immelmann.section);
    near(immelmann.apex.position.z,inversion.height,1e-5,"Immelmann retains its full apex");
    check(immelmann.apex.up.z<-.5&&immelmann.apex.forward.x<-.5,"Immelmann has a true inverted reversal");
    check(immelmann.rollExit.forward.z<-.1,"Immelmann roll finishes on the descent");
    near(immelmann.exit.position.z,inversion.exitHeight,1e-5,"Immelmann completes its valley");
    near(norm(immelmann.exit.up-Vec3{0,0,1}),0,1e-6,"Immelmann exits upright");
    for(const auto& q:immelmann.section.samples){
        check(q.time<=immelmann.apex.time+1e-8?q.forward.z>=-1e-6:q.forward.z<=1e-6,"Immelmann has one ascent and one complete descent");
        near(.5*q.speed*q.speed+gravity*q.position.z+q.dissipatedWorkPerMass,
            .5*inversion.entrySpeed*inversion.entrySpeed,1e-5,"Immelmann conserves energy including real losses");
    }
    auto straightRequest=constant(1,0,2);auto straight=designFvdSection(straightRequest);good(straight);
    for(const auto& q:straight.samples){
        near(norm(q.position-Vec3{20*q.time,0,50}),0,2e-11,"Analytic level 1g straight position");
        near(q.speed,20,1e-12,"Level track conserves speed without propulsion");
        near(norm(q.curvature),0,1e-12,"1g cancels gravity on level track");
    }
    check(!straight.track.closed,"A section is not a closed circuit");
    // Independent horizontal-circle oracle: rightward bend, radius R, bank b.
    // Up=(sin(b)*inward+cos(b)*Z); normal=sec(b), lateral=0,
    // v^2/R=g*tan(b), tangent twist is zero (angular velocity is vertical).
    const double radius=100,velocity=20,b=std::atan(velocity*velocity/(radius*gravity));
    auto circleRequest=constant(1/std::cos(b),0,3);circleRequest.up={0,-std::sin(b),std::cos(b)};

    auto circle=designFvdSection(circleRequest);good(circle);
    for(const auto& q:circle.samples){
        double angle=velocity*q.time/radius;
        Vec3 p{radius*std::sin(angle),-radius*(1-std::cos(angle)),50};
        near(norm(q.position-p),0,8e-5,"Analytic banked circular turn position");
        near(q.speed,velocity,2e-5,"Gravity tangent vanishes on horizontal circle");
        near(norm(q.curvature),1/radius,2e-7,"Analytic circular curvature");
        near(norm(q.forward),1,2e-14,"SO(3) forward remains unit");
        near(dot(q.forward,q.up),0,2e-14,"SO(3) axes remain perpendicular");
    }
    check(circle.track.authoredGeometry&&circle.track.authoredFrame,"FVD retains analytic geometry and physical-frame jets");
    for(size_t i=0;i<circle.track.knots.size();i+=23){const auto& q=circle.track.knots[i];
        near(norm(q.third+q.tangent/(radius*radius)),0,3e-8,"Analytic circle third position derivative");
        near(norm(q.fourth+q.curvature/(radius*radius)),0,3e-9,"Analytic circle fourth position derivative");}
    // Independent constant-pitch, constant-bank helix under gravity. Its
    // heading has a logarithmic closed form as the speed falls on the climb.
    const double climb=.2,bank=.6,v0=40;
    FvdRequest helix=constant(std::cos(climb)/std::cos(bank),0,2);
    helix.gravityReferencedRoll=true;helix.speed=v0;helix.forward={std::cos(climb),0,std::sin(climb)};
    helix.up={-std::sin(climb)*std::cos(bank),-std::sin(bank),std::cos(climb)*std::cos(bank)};
    const auto banked=designFvdSection(helix);good(banked);
    for(size_t i=0;i<banked.samples.size();i+=13){const auto& q=banked.samples[i];const auto& k=banked.track.knots[i];
        const double v=v0-gravity*std::sin(climb)*q.time;
        const double yaw=std::tan(bank)/std::sin(climb)*std::log(v/v0);
        const Vec3 t{std::cos(climb)*std::cos(yaw),std::cos(climb)*std::sin(yaw),std::sin(climb)};
        const Vec3 upright{-std::sin(climb)*std::cos(yaw),-std::sin(climb)*std::sin(yaw),std::cos(climb)};
        const Vec3 right{std::sin(yaw),-std::cos(yaw),0};
        near(norm(q.forward-t),0,1e-9,"Constant-bank helix follows analytic logarithmic yaw");
        near(norm(q.up-upright*std::cos(bank)-right*std::sin(bank)),0,1e-9,"Physical bank stays constant through changing pitch reference");
        near(q.speed,v,1e-9,"Helix speed follows gravitational work");
        near(k.curvature.z,0,1e-11,"Analytic helix keeps constant pitch");
        near(k.third.z,0,1e-11,"Bank compensation carries geometry third derivative");
        near(k.fourth.z,0,1e-11,"Bank compensation carries geometry fourth derivative");
        near(k.upFirst.z,0,1e-11,"Bank compensation carries frame first derivative");
        near(k.upSecond.z,0,1e-11,"Bank compensation carries frame second derivative");
        near(k.upThird.z,0,1e-11,"Bank compensation carries frame third derivative");
    }
    helix.gravityReferencedRoll=false;const auto transported=designFvdSection(helix);good(transported);
    check(norm(transported.samples.back().up-banked.samples.back().up)>.02,"Transported twist and physical bank remain distinct intentional controls");
    auto vertical=helix;vertical.gravityReferencedRoll=true;vertical.forward={0,0,1};vertical.up={1,0,0};
    const auto invalidBank=designFvdSection(vertical);
    check(!invalidBank.report.valid()&&invalidBank.report.errors.front().code=="FVD_BANK_DOMAIN","Gravity bank fails closed at a vertical tangent");
    FvdWaveRequest wave;wave.rollingAcceleration=gravity*.004;wave.dragAccelerationCoefficient=.0002041666666667;
    const auto reversal=designFvdWave(wave);good(reversal.section);
    const auto& waveEnd=reversal.section.samples.back();
    near(waveEnd.position.z,wave.exitHeight,1e-4,"FVD wave meets its integrated exit height");
    near(waveEnd.forward.z,0,1e-6,"FVD wave meets a level loaded exit");
    check(waveEnd.curvature.z>.005,"FVD wave retains positive curvature into the loop");
    near(norm(waveEnd.up-Vec3{0,0,1}),0,1e-6,"Physical wave bank closes without a roll connector");
    near(std::abs(std::atan2(waveEnd.forward.y,waveEnd.forward.x)),pi,1e-6,"FVD wave completes a full reversal");
    Design bankSource;ForceAuthoring bankProgram;bankProgram.name="wave";bankProgram.program=reversal.authoring;bankProgram.sourceDistances={0,reversal.section.track.length};bankSource.forcePrograms.push_back(bankProgram);
    const auto bankPayload=authorshipPayload(bankSource);Design bankReload;std::string bankError;
    check(parseAuthorshipPayload(bankPayload,bankReload,bankError)&&bankReload.forcePrograms[0].program.gravityReferencedRoll&&authorshipPayload(bankReload)==bankPayload,"Bank reference survives exact editable-source serialization");
    check(!parseAuthorshipPayload(bankPayload+" 0",bankReload,bankError),"Trailing bank reference data rejects");
    auto risingWave=wave;risingWave.entrySpeed=71.456355;risingWave.entryPitch=.02;risingWave.entryNormalG=1.9;risingWave.entryHoldSeconds=1.8;risingWave.exitNormalG=1.5;
    risingWave.height=90;risingWave.exitHeight=10;risingWave.bankAngle=73*pi/180;
    const auto rising=designFvdWave(risingWave);good(rising.section);
    near(rising.authoring.controls.front().normalG,1.9,0,"A rising wave inherits its explicit entry load");
    near(sampleFvdControl(rising.authoring.controls,1.5).normalG,1.9,0,"The history hold does not begin the higher wave load early");
    check(rising.authoring.twists.front().begin<risingWave.entryHoldSeconds,"Coordinated banking can turn the climb while its history load remains held");
    near(rising.authoring.controls.back().normalG,1.5,0,"Wave exit can release its old high-load loop dependency");
    int risingChanges=0,previousRise=1;for(const auto& q:rising.section.samples){const int sign=q.forward.z>1e-5?1:q.forward.z< -1e-5?-1:0;if(sign&&sign!=previousRise){++risingChanges;previousRise=sign;}}
    check(risingChanges==1,"Rising-entry wave has exactly one crest without an invented initial valley");
    auto ballisticRequest=constant(0,0,2);ballisticRequest.speed=30;ballisticRequest.step=.01;
    auto coarse=designFvdSection(ballisticRequest);good(coarse);
    ballisticRequest.step=.005;auto fine=designFvdSection(ballisticRequest);good(fine);
    const Vec3 exactPosition{60,0,50-.5*gravity*4};const double exactSpeed=std::hypot(30,gravity*2);
    double coarseError=norm(coarse.samples.back().position-exactPosition),fineError=norm(fine.samples.back().position-exactPosition);
    check(fineError<coarseError*.3&&fineError<.001,"Ballistic trajectory has second-order step convergence");
    near(fine.samples.back().speed,exactSpeed,.0002,"Independent analytic ballistic speed");
    check(fine.assessment.maxNormalResidualG<.005&&fine.assessment.maxLateralResidualG<1e-9,"Canonical zero-g replay measures ballistic forces independently");
    // Vary every channel through smooth controls; compare actual step halves.
    FvdRequest changing;changing.speed=40;changing.step=.01;
    changing.controls={{0,1,0,0},{.8,1.8,.25,.3},{1.6,.4,-.2,-.2},{2.4,1,0,0}};
    auto variedCoarse=designFvdSection(changing);good(variedCoarse);changing.step=.005;
    auto variedFine=designFvdSection(changing);good(variedFine);changing.step=.0025;
    auto variedFinest=designFvdSection(changing);good(variedFinest);
    // A split inside a live force/roll/drive ramp inherits nonzero jets. This
    // must be the same programme, rather than a fresh easing to a neutral port.
    FvdRequest continuous;continuous.speed=35;continuous.step=.005;
    FvdChannels channels{{{{0,1},{.7,1.6,.3,-.2},{2,1.1}},
                         {{0,0},{1.2,.15},{2,-.1}},
                         {{0,0},{.9,.18},{2,0}},
                         {{0,4},{1.4,2},{2,0}}}};
    continuous.controls=combineFvdChannels(channels);
    auto unsplit=designFvdSection(continuous);good(unsplit);
    const auto inherited=sampleFvdControl(continuous.controls,.43);
    check(std::abs(inherited.first[0])>.1&&std::abs(inherited.first[3])>.1,"Continuous boundaries carry nonzero force and drive derivatives");
    continuous.controls.insert(std::upper_bound(continuous.controls.begin(),continuous.controls.end(),inherited.time,[](double t,const FvdControl& c){return t<c.time;}),inherited);
    continuous.chapters={{0,"entry"},{.43,"same continuous motion"},{1.17,"exit"}};
    auto split=designFvdSection(continuous);good(split);
    near(norm(split.samples.back().position-unsplit.samples.back().position),0,1e-7,"Splitting controls and naming chapters preserves geometry");
    near(split.samples.back().speed,unsplit.samples.back().speed,1e-8,"Segmentation preserves physical energy and exit speed");
    check(split.samples.back().drivenWorkPerMass>100&&split.assessment.maxEnergyDrift<1e-5,"Real actuator work closes the independent FVD energy balance");
    // A graded motor needs cos(pitch) normal load; 1g would bend the rail.
    FvdRequest graded;const double grade=.3;graded.forward={std::cos(grade),0,std::sin(grade)};graded.up={-std::sin(grade),0,std::cos(grade)};
    graded.controls={{0,std::cos(grade),0,0,8},{2,std::cos(grade),0,0,8}};
    auto motor=designFvdSection(graded);good(motor);
    near(norm(motor.samples.back().forward-graded.forward),0,1e-10,"Powered grade retains its actual inherited pitch without curvature");
    near(motor.samples.back().speed,graded.speed+2*(8-gravity*std::sin(grade)),1e-9,"Graded actuator includes gravity with correct sign and units");
    const double firstDifference=norm(variedCoarse.samples.back().position-variedFine.samples.back().position);
    const double secondDifference=norm(variedFine.samples.back().position-variedFinest.samples.back().position);
    check(secondDifference<firstDifference*.35,"Smooth force and roll controls converge under genuine step halving");
    check(variedFine.assessment.evaluations>variedFine.samples.size(),"Canonical replay samples between authoring knots");
    for(bool twisted:{false,true}) {
        FvdHillRequest hill;hill.entrySpeed=64;hill.height=105;hill.twistAngle=twisted?55*pi/180:0;
        hill.rollingAcceleration=gravity*.004;hill.dragAccelerationCoefficient=.0002;
        const auto solved=designFvdHill(hill);good(solved.section);
        double height=0;for(const auto& point:solved.section.samples)height=std::max(height,point.position.z);
        near(height,hill.height,.01,"Force hill reaches its authored crest height");
        const auto& exit=solved.section.samples.back();near(exit.position.z,hill.exitHeight,1e-4,"Force hill reaches its exit height");
        near(std::atan2(exit.forward.z,std::hypot(exit.forward.x,exit.forward.y)),hill.exitPitch,1e-6,"Force hill keeps the nonneutral descending exit pitch");
        check(std::abs(exit.curvature.z)>.001,"Live exit retains meaningful curvature");
        if(twisted)check(std::abs(exit.position.y)>5&&std::abs(exit.up.y)>.3,"Twisted drop changes geometry and physical orientation");
        Design authored;ForceAuthoring source;source.name="test-hill";source.program=solved.authoring;source.sourceDistances={0,solved.section.track.length};authored.forcePrograms.push_back(source);
        const auto payload=authorshipPayload(authored);Design reloaded;std::string error;
        check(parseAuthorshipPayload(payload,reloaded,error)&&authorshipPayload(reloaded)==payload,"FVD timelines, twist phases and derivatives survive exact authoring serialization");
        check(!parseAuthorshipPayload(payload.substr(0,payload.size()/2),reloaded,error),"Truncated source data is rejected");
    }
    // The photo-derived hill has a gentler ascent and steeper recovery side.
    FvdHillRequest giant;giant.entrySpeed=83.5;giant.height=224.654746724;giant.exitHeight=-3;giant.exitPitch=.12;
    giant.positiveG=2.934128338;giant.exitPositiveG=4.005031584;giant.airtimeG=-0.895986970;giant.crestLoadChangeG=-0.060263335;
    giant.rampSeconds=2.425134587;giant.exitRampSeconds=3.286095368;giant.ascentReleaseSeconds=2.907610573;
    giant.rollingAcceleration=gravity*.004;giant.dragAccelerationCoefficient=.0002041666666667;
    const auto shaped=designFvdHill(giant);good(shaped.section);
    double ascentPitch=0,descentPitch=0;
    for(const auto& q:shaped.section.samples){ascentPitch=std::max(ascentPitch,std::asin(q.forward.z));descentPitch=std::max(descentPitch,-std::asin(q.forward.z));near(q.position.y,0,1e-12,"Reference-shaped camelback remains planar");}
    check(descentPitch>ascentPitch,"The traced silhouette has a steeper descent without a prescribed pitch target");
    // Independent points from the unchanged reference image cover the crown,
    // both flanks and lower transitions. Use one scale, with zero camera yaw.
    const auto apex=std::max_element(shaped.section.samples.begin(),shaped.section.samples.end(),[](const auto& a,const auto& b){return a.position.z<b.position.z;});
    const std::array<Vec3,14> trace{{{140,915,0},{350,782,0},{500,567,0},{650,246,0},{800,79,0},{900,29,0},{1000,13,0},{1150,51,0},{1300,171,0},{1450,395,0},{1575,578,0},{1700,692,0},{1900,817,0},{2040,880,0}}};
    double squaredError=0,maximumError=0;
    for(const auto& point:trace){double error=1e9;
        for(const auto& q:shaped.section.samples){const Vec3 projected{996+(apex->position.x-q.position.x)*4.204043308,13+(apex->position.z-q.position.z)*4.204043308,0};error=std::min(error,norm(projected-point));}
        squaredError+=error*error;maximumError=std::max(maximumError,error);
    }
    check(std::sqrt(squaredError/trace.size())<3.5&&maximumError<7,"The whole FVD silhouette follows the independent photo trace without squeezing or rotating it");
    check(shaped.authoring.controls[1].normalG==giant.positiveG&&shaped.authoring.controls.back().normalG==giant.exitPositiveG,"Ascent and recovery own separate force timelines");
    const auto crest=sampleFvdControl(shaped.authoring.controls,(shaped.authoring.controls[3].time+shaped.authoring.controls[4].time)*.5);
    check(crest.normalG<giant.airtimeG&&crest.normalG>giant.airtimeG+giant.crestLoadChangeG&&crest.first[0]<0,"The asymmetric crown carries a continuously changing load through its interior");
    auto excessiveCrest=giant;excessiveCrest.crestLoadChangeG=-1;
    check(!designFvdHill(excessiveCrest).section.report.valid(),"Independent crest shaping cannot bypass the source force envelope");
    FvdCamelbackRequest protectedHill;protectedHill.hill=giant;protectedHill.hill.exitHeight=0;
    const auto protectedReference=designFvdHill(protectedHill.hill);good(protectedReference.section);
    const auto recovered=designFvdCamelback(protectedHill);good(recovered.section);
    near(recovered.originalEndTime-recovered.protectedEndTime,protectedHill.tailCutSeconds,1e-12,"Protected camelback only shortens its final constant-load hold");
    for(size_t i=0;i+1<protectedReference.authoring.controls.size();++i){const auto& a=protectedReference.authoring.controls[i];const auto& retained=recovered.authoring.controls[i];
        check(a.time==retained.time&&a.normalG==retained.normalG&&a.lateralG==retained.lateralG&&a.rollRate==retained.rollRate&&a.first==retained.first&&a.second==retained.second,"All protected ascent, crest and descent controls remain exact");}
    for(double s=0;s<=recovered.protectedEndDistance;s+=7.3)
        near(norm(recovered.section.track.sample(s).position-protectedReference.section.track.sample(s).position),0,1e-7,"The protected native geometry prefix remains unchanged");
    const auto& recoveredEnd=recovered.section.samples.back();
    near(std::asin(recoveredEnd.forward.z),protectedHill.minimumExitPitch,1e-6,"Low-load camelback recovery ends at its first requested rising port");
    near(recovered.authoring.controls.back().normalG,protectedHill.exitNormalG,0,"Protected recovery carries its explicit low exit load");
    check(recoveredEnd.position.x>protectedReference.section.samples.back().position.x&&recoveredEnd.position.z<0,"Recovery reports its real extra footprint and lower exit instead of forcing the old endpoint");
    auto invalidCut=protectedHill;invalidCut.tailCutSeconds=2;
    check(!designFvdCamelback(invalidCut).section.report.valid(),"An excessive cut cannot remove protected force phases");
    check(designFvdCamelback(protectedHill,[]{return true;}).section.cancelled,"Protected camelback propagates cancellation");
    FvdTerrainActRequest terrainAct;terrainAct.entrySpeed=40;terrainAct.pitchRateS=-.009;terrainAct.pitchSecondS=.00001;terrainAct.pitchThirdS=-.0000003;terrainAct.airtimeG=-.99;
    const auto act=designFvdTerrainAct(terrainAct);good(act.section);
    const auto inheritedAct=sampleKinematics(act.section.track,0);
    near(inheritedAct.sample.curvature.z,terrainAct.pitchRateS,1e-10,"Terrain act inherits the live pitch curvature");
    near(inheritedAct.curvatureS.z,terrainAct.pitchSecondS,1e-10,"Terrain act inherits the live pitch second derivative");
    near(inheritedAct.curvatureSS.z,terrainAct.pitchThirdS-std::pow(terrainAct.pitchRateS,3),1e-9,"Terrain act inherits the live pitch third derivative");
    near(act.section.samples.back().position.z,terrainAct.heightChange,1e-5,"Terrain act meets its prescribed elevation change");
    near(std::atan2(act.section.samples.back().forward.y,act.section.samples.back().forward.x),terrainAct.headingChange,1e-6,"Terrain act retains its physical heading change");
    terrainAct.headingChange=-terrainAct.headingChange;const auto mirrorAct=designFvdTerrainAct(terrainAct);good(mirrorAct.section);
    check(act.section.samples.size()==mirrorAct.section.samples.size(),"Both terrain-act hands use the same force timeline");
    for(size_t i=0;i<act.section.samples.size();i+=100){auto p=act.section.samples[i].position;p.y=-p.y;near(norm(p-mirrorAct.section.samples[i].position),0,1e-8,"A mirrored terrain act has the independently reflected geometry");}
    check(designFvdTerrainAct(terrainAct,[]{return true;}).section.cancelled,"Terrain act does not swallow cancellation inside its shooting search");
    check(Limits{}.maxVerticalG==5&&Limits{}.maxLateralG==1.5,"User vertical and lateral target caps remain explicit");
    auto reject=[](FvdRequest r,const char* code){auto result=designFvdSection(r);check(!result.report.valid()&&!result.report.errors.empty()&&result.report.errors.front().code==code,"Unsupported request rejects explicitly");};
    FvdRequest bad;bad.speed=std::numeric_limits<double>::quiet_NaN();reject(bad,"FVD_INPUT");
    bad=FvdRequest{};bad.controls[1].rollRate=INFINITY;reject(bad,"FVD_INPUT");
    bad=FvdRequest{};bad.controls[1].time=0;reject(bad,"FVD_INPUT");
    bad=FvdRequest{};bad.up=bad.forward;reject(bad,"FVD_INPUT");
    bad=FvdRequest{};bad.maxSamples=4;reject(bad,"FVD_BUDGET");
    bad=FvdRequest{};bad.step=0;reject(bad,"FVD_INPUT");
    bad=FvdRequest{};bad.controls[1].normalG=21;reject(bad,"FVD_INPUT");
    bad=constant(0,0,1);bad.forward={0,0,1};bad.up={1,0,0};bad.speed=2;
    reject(bad,"FVD_SPEED");
    auto cancelled=designFvdSection(FvdRequest{},[]{return true;});check(cancelled.cancelled&&!cancelled.integrated&&!cancelled.report.valid(),"Cancellation before integration");
    int polls=0;cancelled=designFvdSection(FvdRequest{},[&]{return ++polls>30;});
    check(cancelled.cancelled&&!cancelled.integrated&&!cancelled.assessment.passed,"Cancellation during integration cannot pass");
    // Force a residual rejection instead of confusing raw integration with fit.
    changing.step=.02;changing.forceToleranceG=1e-8;
    auto residual=designFvdSection(changing);
    check(residual.integrated&&residual.canonicalBuilt&&residual.assessment.performed&&!residual.assessment.passed&&!residual.report.valid(),"A canonical residual failure is distinct from integration completion");
    std::cout<<"PASS "<<checks<<" FVD section checks: analytic straight/circle/ballistic, SO(3), smooth profile convergence, canonical point replay, invalid input and cancellation\n";
    return 0;
}catch(const std::exception& e){std::cerr<<"FAIL after "<<checks<<": "<<e.what()<<'\n';return 1;}}
