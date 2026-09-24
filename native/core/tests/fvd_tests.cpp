#include "coaster/fvd.hpp"
#include "coaster/angular_phase_intent.hpp"
#include "signature_reference.hpp"
#include <iostream>
#include <stdexcept>
using namespace coaster;
namespace {
int checks=0;
void check(bool condition,const char* message){++checks;if(!condition)throw std::runtime_error(message);}
void near(double value,double expected,double tolerance,const char* message){if(!std::isfinite(value)||std::abs(value-expected)>tolerance)std::cerr<<message<<": value="<<value<<" expected="<<expected<<" tolerance="<<tolerance<<'\n';check(std::isfinite(value)&&std::abs(value-expected)<=tolerance,message);}
void good(const FvdResult& r){
    if(!r.report.valid()){
        for(const auto& e:r.report.errors)std::cerr<<e.code<<": "<<e.message<<'\n';
        std::cerr<<"residual normal="<<r.assessment.maxNormalResidualG<<" lateral="<<r.assessment.maxLateralResidualG<<" roll="<<r.assessment.maxRollResidualRadPerSecond<<" distance="<<r.assessment.endDistanceError<<" speed="<<r.assessment.endSpeedError<<'\n';
    }
    check(r.integrated&&r.canonicalBuilt&&!r.cancelled&&r.report.valid()&&r.assessment.performed&&r.assessment.passed,"Section integrates, fits and passes its sampled point replay");
}
FvdRequest constant(double normal,double lateral,double duration=1){FvdRequest r;r.controls={{0,normal,lateral,0},{duration,normal,lateral,0}};return r;}
void planarAngularFidelity(){
    // Independently differentiate v*thetaDot=g*(G-cos(theta)). Exercise the
    // interiors of source spans, including near their ends, at two world origins.
    for(double offset:{0.,4000.})for(double step:{.01,.005,.0025}){
        FvdRequest request;request.position={offset,0,offset+50};request.speed=80;request.step=step;
        request.rollingAcceleration=.04;request.dragAccelerationCoefficient=.0002;
        request.controls={{0,1,0,0},{.3,4,0,0},{.7,-1,0,0},{1.1,2,0,0}};
        const auto source=designFvdSection(request);good(source);
        for(size_t i=0;i<source.track.spans.size();++i)for(double u:{.0625,.25,.5,.875,.9375}){
            const auto& a=source.samples[i];const auto& b=source.samples[i+1];
            const auto c=sampleFvdControl(request.controls,std::lerp(a.time,b.time,u));
            const auto q=sampleSpanKinematics(source.track,i,u);
            const double v=std::lerp(a.speed,b.speed,u),sine=q.sample.tangent.z,cosine=q.sample.tangent.x;
            const double acceleration=-gravity*sine-request.rollingAcceleration-request.dragAccelerationCoefficient*v*v;
            const double omega=gravity*(c.normalG-cosine)/v;
            const double alpha=(gravity*(c.first[0]+sine*omega)-omega*acceleration)/v;
            const double jerk=-gravity*cosine*omega-2*request.dragAccelerationCoefficient*v*acceleration;
            const double expected=(gravity*(c.second[0]+cosine*omega*omega+sine*alpha)-2*alpha*acceleration-omega*jerk)/v;
            near(signedAngularMotion(q,v,acceleration,jerk).jerk[1],expected,.001,
                "Interior canonical pitch jerk agrees with the independent planar ODE under refinement and translation");
        }
    }
}
struct PhotoError {double rms{},maximum{};bool matches(double maximumLimit=7)const{return rms<3.5&&maximum<maximumLimit;}};
PhotoError photoError(const FvdResult& section,bool canonical,double widthScale=1,bool reverse=false){
    // The photo's translation is the recorded crown. Only the model crown is
    // located; neither scale, camera yaw nor separate flank scales are fitted.
    const auto apex=std::max_element(section.samples.begin(),section.samples.end(),[](const auto& a,const auto& b){return a.position.z<b.position.z;});
    auto project=[&](Vec3 p){return Vec3{
        signature_reference::apexDisplay[0]+(apex->position.x-p.x)*signature_reference::pixelsPerMeter*widthScale*(reverse?-1:1),
        signature_reference::apexDisplay[1]+(apex->position.z-p.z)*signature_reference::pixelsPerMeter,0};};
    std::vector<Vec3> contour;
    if(canonical){
        for(double distance=0;distance<section.track.length;distance+=.5)contour.push_back(project(section.track.sample(distance).position));
        contour.push_back(project(section.track.sample(section.track.length).position));
    }else for(const auto& sample:section.samples)contour.push_back(project(sample.position));
    PhotoError result;
    for(const auto& pixel:signature_reference::railPixels){
        const Vec3 target{pixel[0]*signature_reference::sourceToDisplay,pixel[1]*signature_reference::sourceToDisplay,0};
        double distance=std::numeric_limits<double>::infinity();
        for(size_t i=1;i<contour.size();++i){const auto a=contour[i-1],delta=contour[i]-a;
            const double square=dot(delta,delta),u=square>0?std::clamp(dot(target-a,delta)/square,0.,1.):0;
            distance=std::min(distance,norm(a+delta*u-target));}
        result.rms+=distance*distance;result.maximum=std::max(result.maximum,distance);
    }
    result.rms=std::sqrt(result.rms/signature_reference::railPixels.size());return result;
}
void photoMatches(const FvdResult& section,bool canonical,const char* message,double maximumLimit=7){
    if(canonical)for(double distance=0;distance<section.track.length;distance+=2.3){
        const auto q=section.track.sample(distance);
        near(q.position.y,0,1e-10,"Photo comparison cannot hide an out-of-plane compiled position");
        near(q.tangent.y,0,1e-10,"The protected compiled silhouette has zero physical yaw");
        near(q.up.y,0,1e-10,"The protected compiled silhouette has no hidden physical roll");
    }
    const auto error=photoError(section,canonical);
    if(!error.matches(maximumLimit))std::cerr<<"Photo trace RMS="<<error.rms<<" max="<<error.maximum<<" canonical="<<canonical<<'\n';
    check(error.matches(maximumLimit),message);
}
Vec3 reflected(Vec3 p){p.y=-p.y;return p;}
}
int main(){try{
    planarAngularFidelity();
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
    // Endpoint yaw alone also permits a sideways translation or wrong
    // interior hand. Reflection must hold throughout the physical inversion;
    // no historical loop envelope is a design requirement for future recipes.
    near(mirror.section.track.length,looped.section.track.length,1e-6,"Mirrored loops retain the same real arc length");
    check(mirror.section.samples.size()==looped.section.samples.size(),"Mirrored loops retain the same integration timeline");
    for(size_t i=0;i<looped.section.samples.size();i+=37){const auto& a=looped.section.samples[i];const auto& b=mirror.section.samples[i];
        near(a.time,b.time,1e-8,"Loop reflection preserves physical time through the inversion");
        near(norm(reflected(a.position)-b.position),0,1e-5,"Loop handedness reflects every sampled interior position");
        near(norm(reflected(a.forward)-b.forward),0,1e-7,"Loop handedness reflects interior pitch and yaw together");
        near(norm(reflected(a.up)-b.up),0,1e-7,"Loop handedness reflects the physical rider frame through inversion");
        near(norm(reflected(a.curvature)-b.curvature),0,1e-8,"Loop handedness reflects interior curvature rather than merely moving the exit");
    }
    for(double distance=0;distance<looped.section.track.length;distance+=2.3){
        const auto a=looped.section.track.sample(distance),b=mirror.section.track.sample(distance);
        near(norm(reflected(a.position)-b.position),0,1e-5,"Compiled loop preserves the mirrored interior trajectory");
        near(norm(reflected(a.tangent)-b.tangent),0,1e-7,"Compiled loop preserves the mirrored interior heading");
        near(norm(reflected(a.up)-b.up),0,1e-7,"Compiled loop preserves the mirrored interior physical roll");
    }
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
    auto separated=inversion;separated.rollOverlapFraction=0;
    const auto separatePhases=designFvdImmelmann(separated);good(separatePhases.section);
    near(separatePhases.apex.position.z,separated.height,1e-5,"Separated Immelmann solve retains its prescribed apex");
    near(separatePhases.exit.position.z,separated.exitHeight,1e-5,"Separated Immelmann solve reaches its actual valley port");
    auto yawing=separated;yawing.yawAngle=-20*pi/180;
    const auto yawed=designFvdImmelmann(yawing);good(yawed.section);
    const double yawBegin=separatePhases.authoring.controls[1].time,yawDuration=separatePhases.apex.time-yawBegin;
    for(size_t i=0;i<yawed.section.samples.size();i+=37){
        const auto& q=yawed.section.samples[i];
        const auto base=separatePhases.section.track.sample(angular_detail::sourceDistance(separatePhases.section,q.time));
        const double u=std::clamp((q.time-yawBegin)/yawDuration,0.,1.);
        const double angle=yawing.yawAngle*u*u*u*u*(35+u*(-84+u*(70-20*u)));
        const auto rotate=[&](Vec3 v){return Vec3{v.x*std::cos(angle)-v.y*std::sin(angle),v.x*std::sin(angle)+v.y*std::cos(angle),v.z};};
        near(norm(q.forward-rotate(base.tangent)),0,1e-6,"Coordinated Immelmann replay follows its independently prescribed yawing tangent");
        near(norm(q.up-rotate(base.up)),0,1e-6,"Coordinated Immelmann keeps its physical rider frame under yaw");
        near(q.position.z,base.position.z,1e-6,"World-vertical frame yaw retains the solved height trajectory");
    }
    near(yawed.exit.speed,separatePhases.exit.speed,1e-7,"Coordinated yaw preserves coasting energy and exit speed");
    auto noReversal=yawing;noReversal.yawAngle=-1.4;
    const auto wrongHeading=designFvdImmelmann(noReversal);
    check(!wrongHeading.section.report.valid()&&std::any_of(wrongHeading.section.report.errors.begin(),wrongHeading.section.report.errors.end(),[](const Finding& f){return f.code=="FVD_IMMELMANN_HEADING";}),
        "A loaded roll cannot be accepted as an Immelmann without the existing physical heading reversal");
    auto loadedEntry=constant(1.5,0,.02);loadedEntry.speed=58.5645947352;loadedEntry.position={};
    loadedEntry.rollingAcceleration=gravity*.004;loadedEntry.dragAccelerationCoefficient=.0002041666666667;
    const auto loadedPort=designFvdSection(loadedEntry);good(loadedPort);
    FvdImmelmannRequest loaded;loaded.rollingAcceleration=loadedEntry.rollingAcceleration;loaded.dragAccelerationCoefficient=loadedEntry.dragAccelerationCoefficient;
    loaded.entry=makeFvdEntry(loadedPort.track.knots.front(),loadedEntry.speed,loaded.rollingAcceleration,loaded.dragAccelerationCoefficient);
    loaded.height=87.3*std::pow(loadedEntry.speed/53,2);loaded.normalG=4.4;loaded.crestG=3.5;loaded.rollExitG=2;loaded.exitPositiveG=3;
    loaded.rampSeconds=1.0;loaded.exitRampSeconds=1.2;loaded.rollOverlapFraction=0;loaded.rollReleaseFraction=.45;loaded.yawAngle=45*pi/180;loaded.hand=-1;
    loaded.exitPitch=-5*pi/180;loaded.exitNormalG=std::cos(loaded.exitPitch);
    const auto heldRoll=designFvdImmelmann(loaded);good(heldRoll.section);
    const auto halfRoll=std::min_element(heldRoll.section.samples.begin(),heldRoll.section.samples.end(),[&](const FvdSample& a,const FvdSample& b){
        const auto score=[&](const FvdSample& q){return q.time<heldRoll.apex.time||q.time>heldRoll.rollExit.time?2.:std::abs(q.up.z);};return score(a)<score(b);});
    const double halfAcceleration=-gravity*halfRoll->forward.z-loaded.rollingAcceleration-loaded.dragAccelerationCoefficient*halfRoll->speed*halfRoll->speed;
    check(measureSeatForces(heldRoll.section.track,halfRoll->distance,halfRoll->speed,halfAcceleration,1.2).vertical>3,
        "The intentionally loaded Immelmann retains over 3g through its half-roll with broad acceleration ramps");
    auto phasedCrown=loaded;phasedCrown.normalG=4.85;phasedCrown.crestG=3.8;
    phasedCrown.height=85*std::pow(loadedEntry.speed/53,2);phasedCrown.ascentReleaseSeconds=1.2;phasedCrown.rollReleaseFraction=.45;
    const auto phased=designFvdImmelmann(phasedCrown);good(phased.section);
    const auto beforeCrown=sampleFvdControl(phased.authoring.controls,phased.apex.time-.25);
    near(beforeCrown.normalG,phasedCrown.crestG,1e-7,"Prescribed ascent release reaches a genuine crown hold before the apex");
    near(beforeCrown.first[0],0,1e-7,"Crown hold retains zero normal-force rate");
    near(beforeCrown.second[0],0,1e-7,"Crown hold retains zero normal-force acceleration");
    // Both speeds previously exhausted the legacy unload-duration seeds,
    // despite lying between constructible members of this held-crown family.
    for(double speed:{55.963245,57.5895085102}){
        auto incoming=loadedEntry;incoming.speed=speed;
        const auto port=designFvdSection(incoming);good(port);
        auto nearby=phasedCrown;
        nearby.entry=makeFvdEntry(port.track.knots.front(),speed,nearby.rollingAcceleration,nearby.dragAccelerationCoefficient);
        nearby.height=85*std::pow(speed/53,2);
        const auto solved=designFvdImmelmann(nearby);good(solved.section);
        near(solved.apex.position.z,nearby.height,1e-5,"Nearby reached speeds retain the requested held-crown apex height");
        near(solved.apex.forward.z,0,1e-6,"Nearby reached speeds retain the horizontal inversion apex");
    }
    auto coincidentRelease=loaded;coincidentRelease.rollReleaseFraction=.2;
    good(designFvdImmelmann(coincidentRelease).section);
    auto unresolvedRelease=loaded;unresolvedRelease.rollReleaseFraction=1e-8;
    const auto tooShortRelease=designFvdImmelmann(unresolvedRelease);
    check(!tooShortRelease.section.report.valid()&&std::any_of(tooShortRelease.section.report.errors.begin(),tooShortRelease.section.report.errors.end(),[](const Finding& f){return f.code=="FVD_IMMELMANN_KNOT_SPACING";}),
        "Sub-millisecond crown holds retain their explicit spacing rejection");
    auto unresolvedOverlap=inversion;unresolvedOverlap.rollOverlapFraction=1e-8;
    const auto tooClose=designFvdImmelmann(unresolvedOverlap);
    check(!tooClose.section.report.valid()&&std::any_of(tooClose.section.report.errors.begin(),tooClose.section.report.errors.end(),[](const Finding& f){return f.code=="FVD_IMMELMANN_KNOT_SPACING";}),
        "Near-coincident roll/force boundaries reject explicitly before source integration");
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
    check(coarseError<1e-10&&fineError<1e-10,"Both integration steps recover the independent ballistic position to roundoff");
    near(fine.samples.back().speed,exactSpeed,1e-10,"Independent analytic ballistic speed");
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
    const double rounding=64*std::numeric_limits<double>::epsilon()*norm(variedFinest.samples.back().position);
    check(secondDifference<firstDifference*.35+rounding,"Smooth force and roll controls converge under genuine step halving until the coordinate roundoff floor");
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
    photoMatches(shaped.section,false,"The complete source camelback follows all frozen photo picks with one isotropic scale and no yaw");
    photoMatches(shaped.section,true,"The compiled camelback follows the independent frozen photo silhouette");
    check(!photoError(shaped.section,true,1.1).matches(),"The photo oracle rejects horizontal squeezing or stretching");
    check(!photoError(shaped.section,true,1,true).matches(),"The photo oracle rejects swapping the asymmetric ascent and descent");
    check(shaped.authoring.controls[1].normalG==giant.positiveG&&shaped.authoring.controls.back().normalG==giant.exitPositiveG,"Ascent and recovery own separate force timelines");
    const auto crest=sampleFvdControl(shaped.authoring.controls,(shaped.authoring.controls[3].time+shaped.authoring.controls[4].time)*.5);
    check(crest.normalG<giant.airtimeG&&crest.normalG>giant.airtimeG+giant.crestLoadChangeG&&crest.first[0]<0,"The asymmetric crown carries a continuously changing load through its interior");
    auto excessiveCrest=giant;excessiveCrest.crestLoadChangeG=-1;
    check(!designFvdHill(excessiveCrest).section.report.valid(),"Independent crest shaping cannot bypass the source force envelope");
    FvdCamelbackRequest protectedHill;protectedHill.hill=giant;protectedHill.hill.exitHeight=0;
    const auto protectedReference=designFvdHill(protectedHill.hill);good(protectedReference.section);
    const auto recovered=designFvdCamelback(protectedHill);good(recovered.section);
    // Default1 changed the fitted hill's exitHeight from -3 m to 0 m before
    // protecting it. The independently archived completed-shape.csv therefore
    // measures 3.183 px RMS / 11.250 px maximum against this same full trace.
    // Keep that historical discrepancy explicit; only the original fit uses
    // the stricter 7 px maximum. The recovery is not an exact photo replica.
    photoMatches(recovered.section,false,"The final protected source stays within its documented full-photo silhouette discrepancy",12);
    photoMatches(recovered.section,true,"The final compiled protected camelback meets the independent photo bound rather than only matching its regenerated source",12);
    near(recovered.originalEndTime-recovered.protectedEndTime,protectedHill.tailCutSeconds,1e-12,"Protected camelback only shortens its final constant-load hold");
    for(size_t i=0;i+1<protectedReference.authoring.controls.size();++i){const auto& a=protectedReference.authoring.controls[i];const auto& retained=recovered.authoring.controls[i];
        check(a.time==retained.time&&a.normalG==retained.normalG&&a.lateralG==retained.lateralG&&a.rollRate==retained.rollRate&&a.drive==retained.drive&&a.first==retained.first&&a.second==retained.second,"All protected ascent, crest and descent controls remain exact");}
    for(double distance=0;distance<=recovered.protectedEndDistance;distance+=7.3){
        const auto a=sampleKinematics(recovered.section.track,distance),b=sampleKinematics(protectedReference.section.track,distance);
        near(norm(a.sample.position-b.sample.position),0,1e-7,"The protected native geometry prefix remains unchanged");
        near(norm(a.sample.tangent-b.sample.tangent),0,1e-8,"The protected prefix preserves tangent and pitch");
        near(norm(a.sample.curvature-b.sample.curvature),0,1e-8,"The protected prefix preserves curvature");
        near(norm(a.curvatureS-b.curvatureS),0,1e-8,"The protected prefix preserves its third position derivative");
        near(norm(a.sample.up-b.sample.up),0,1e-8,"The protected prefix preserves the physical rider orientation");
        near(norm(a.upS-b.upS),0,1e-8,"The protected prefix preserves its first orientation derivative");
        near(norm(a.upSS-b.upSS),0,1e-8,"The protected prefix preserves its second orientation derivative");
    }
    // The shortened constant-load hold uses a different integration partition.
    // Third derivatives of two separately normalized tiny canonical polynomials
    // amplify their roundoff (observed 1.00173e-8), so compare the source-owned
    // analytic jets at shared knots, not a second resampling of those jets.
    // This covers every ascent/crest/descent/pullout phase. Across the remaining
    // constant-load prefix, compare the actual time-dependent controls and jets.
    const double finalHoldBegin=protectedReference.authoring.controls[protectedReference.authoring.controls.size()-2].time;
    size_t sharedKnots=0;
    for(size_t i=0;i<protectedReference.section.samples.size()&&protectedReference.section.samples[i].time<=finalHoldBegin+1e-10;++i){
        check(i<recovered.section.samples.size(),"Protected analytic knot exists");
        near(protectedReference.section.samples[i].time,recovered.section.samples[i].time,0,"Protected phases retain the same source timeline");
        const auto& a=protectedReference.section.track.knots[i];const auto& b=recovered.section.track.knots[i];
        near(norm(a.up-b.up),0,1e-12,"Protected analytic source orientation remains exact");
        near(norm(a.upFirst-b.upFirst),0,1e-12,"Protected analytic first orientation derivative remains exact");
        near(norm(a.upSecond-b.upSecond),0,1e-12,"Protected analytic second orientation derivative remains exact");
        near(norm(a.upThird-b.upThird),0,1e-12,"Protected analytic third orientation derivative remains exact");
        ++sharedKnots;
    }
    check(sharedKnots>100,"Protected analytic frame comparison covers the full authored hill body");
    for(double time=0;time<=recovered.protectedEndTime;time+=.137){
        const auto a=sampleFvdControl(protectedReference.authoring.controls,time),b=sampleFvdControl(recovered.authoring.controls,time);
        near(a.normalG,b.normalG,1e-12,"Complete protected normal-force timeline remains unchanged");
        near(a.lateralG,b.lateralG,1e-12,"Complete protected lateral-force timeline remains unchanged");
        near(a.rollRate,b.rollRate,1e-12,"Complete protected twist timeline remains unchanged");
        near(a.drive,b.drive,1e-12,"Complete protected drive timeline remains unchanged");
        for(size_t channel=0;channel<4;++channel){near(a.first[channel],b.first[channel],1e-12,"Protected first control derivative remains unchanged");near(a.second[channel],b.second[channel],1e-12,"Protected second control derivative remains unchanged");}
    }
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
