#include "coaster/fvd.hpp"
#include <iostream>
#include <stdexcept>
#include <future>
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
void checkPointLosses(){
    for(int mode:{0,1,2}){
        FvdRequest r;r.position={0,0,0};r.speed=75;r.step=.005;
        r.controls={{0,1,0,0},{8,1,0,0}};
        r.rollingAcceleration=mode==1?0:.1;
        r.dragAccelerationCoefficient=mode==0?0:.00016;
        double speed,displacement;
        if(mode==0){
            speed=75-.1*8;displacement=75*8-.5*.1*8*8;
        }else if(mode==1){
            speed=75/(1+.00016*75*8);displacement=std::log(1+.00016*75*8)/.00016;
        }else{
            const double a=.1,b=.00016;
            const double initial=std::atan(75*std::sqrt(b/a));
            const double angle=initial-std::sqrt(a*b)*8;
            speed=std::sqrt(a/b)*std::tan(angle);
            displacement=std::log(std::cos(angle)/std::cos(initial))/b;
        }
        auto q=designFvdSection(r);good(q);const auto& end=q.samples.back();
        near(end.speed,speed,1e-8,"Analytic rolling/drag speed");
        near(end.position.x,displacement,1e-7,"Analytic rolling/drag displacement");
        near(end.dissipatedWorkPerMass,.5*(75*75-speed*speed),1e-7,"Analytic dissipated work");
        near(q.assessment.maxEnergyDrift,0,1e-7,"Mechanical energy plus work stays conserved");
        near(end.position.z,0,1e-10,"Tangential losses do not alter straight-track elevation");
        near(norm(end.curvature),0,1e-10,"Tangential losses do not introduce curvature");
    }
    FvdRequest invalid;invalid.dragAccelerationCoefficient=-.001;
    auto q=designFvdSection(invalid);
    check(!q.report.valid()&&q.report.errors.front().code=="FVD_INPUT","Negative loss coefficient rejected");
}
void checkTallHill(){
    const auto source=[&](double height,double speed,bool lossy){
        FvdTallHillRequest r;r.height=height;r.speed=speed;
        if(lossy){
            TrainConfig t;r.rollingAcceleration=gravity*t.rollingResistance;
            r.dragAccelerationCoefficient=.5*t.airDensity*t.dragCdA/(t.cars*t.carMass);
        }
        auto q=designFvdTallHill(r);good(q.section);
        near(q.height,height,1e-4,"Tall hill closes requested height without spatial scaling");
        near(q.exit.position.z,0,1e-5,"Independent descent closes inlet elevation");
        near(norm(q.exit.forward-Vec3{1,0,0}),0,1e-6,"Independent descent closes level heading");
        near(norm(q.section.samples.front().curvature),0,1e-8,"Tall hill has a zero-curvature inlet");
        near(norm(q.exit.curvature),0,1e-8,"Tall hill has a zero-curvature outlet");
        near(.5*q.exit.speed*q.exit.speed+q.exit.dissipatedWorkPerMass,.5*speed*speed,1e-5,
            "Exit energy accounts for dissipated work");
        if(lossy){
            check(q.exit.speed<speed-.1&&q.exit.dissipatedWorkPerMass>1,"Lossy descent retains lower exit energy");
            const auto& c=q.authoring.controls;
            const double ascentHold=c[3].time-c[2].time;
            const double descentHold=c[c.size()-3].time-c[c.size()-4].time;
            check(std::abs(ascentHold-descentHold)>.01,"Lossy descent is solved independently of ascent");
        }else{
            near(q.exit.speed,speed,1e-5,"Default-zero hill retains gravity-only exit energy");
            check(q.exit.dissipatedWorkPerMass==0,"Default loss coefficients are zero");
        }
    };
    source(230,75,false);
    source(230,75,true);
    source(280,90,true); // Upper height/speed boundary remains a real solved source.
    FvdTallHillRequest impossible;impossible.height=280;impossible.speed=75;
    impossible.rollingAcceleration=gravity*.002;
    impossible.dragAccelerationCoefficient=.5*1.225*2.4/9000;
    auto failure=designFvdTallHill(impossible);
    check(!failure.section.report.valid()&&failure.section.report.errors.front().code=="FVD_TALL_ENERGY",
        "Even ideal vertical-ascent energy cannot reach the requested lossy height");
    FvdTallHillRequest invalid;invalid.height=281;
    failure=designFvdTallHill(invalid);
    check(!failure.section.canonicalBuilt&&!failure.section.report.valid()&&
        failure.section.report.errors.front().code=="FVD_TALL_INPUT","Out-of-family tall height rejected");
    for(int stop:{1,1000}){
        int polls=0;failure=designFvdTallHill({},[&]{return ++polls>=stop;});
        check(failure.section.cancelled&&!failure.section.report.valid()&&
            failure.section.report.errors.front().code=="CANCELLED","Cancellation preserved before and during source shooting");
    }
}
}
int main(){try{
    checkPointLosses();checkTallHill();
    FvdReversalRequest reversalRequest;
    auto reversal=designFvdReversal(reversalRequest);good(reversal.section);
    check(reversal.geometricApexHeight>80&&reversal.geometricApexHeight<95&&
        reversal.highestInvertedHeight>80,"Descending reversal measures a high inverted apex distinct from its exit");
    near(reversal.exit.sample.position.z,50,1e-5,"Reversal closes requested lower exit height without a warp");
    near(reversal.exit.sample.tangent.z,std::sin(reversalRequest.exitPitch),1e-7,"Reversal retains actual descending exit pitch");
    check(reversal.exit.sample.tangent.x<-.7&&std::abs(reversal.exit.sample.position.y)>.1,"Spatial reversal returns actual free heading and displacement");
    near(norm(reversal.exit.sample.curvature),0,1e-7,"Descending exit normal intent supplies a straight-compatible port");
    check(norm(reversal.exit.curvatureS)<2e-6&&norm(reversal.exit.upS)<1e-7&&norm(reversal.exit.upSS)<1e-6,
        "Reversal returns smooth position and frame port derivatives");
    check(reversal.intent.size()==reversal.section.samples.size(),"Continuous intent accompanies actual source samples");
    bool rollsDescending=false;
    for(size_t i=0;i<reversal.section.samples.size();i+=17){
        const auto& q=reversal.section.samples[i];const auto& intended=reversal.intent[i];
        near(q.speed*q.speed+2*gravity*q.position.z,reversalRequest.entrySpeed*reversalRequest.entrySpeed,1e-5,
            "Reversal independently conserves mechanical energy");
        const Vec3 specific=q.curvature*(q.speed*q.speed)+q.forward*(-gravity*q.forward.z)+Vec3{0,0,gravity};
        near(dot(specific,q.up)/gravity,intended.normalG,1e-9,"Integrated reversal geometry supplies intended body normal force");
        near(dot(specific,cross(q.forward,q.up))/gravity,intended.lateralG,1e-9,"Integrated reversal geometry supplies intended body lateral force");
        rollsDescending|=q.forward.z<-.1&&std::abs(intended.rollRate)>.2;
    }
    check(rollsDescending,"Roll overlaps the curved descent rather than using a level rolling tail");
    near(reversal.intent.back().normalG,std::cos(reversalRequest.exitPitch),1e-12,"Terminal force follows sloping gravity projection");
    near(reversal.intent.back().lateralG,0,1e-12,"Terminal lateral force closes independently of solved total twist");
    near(reversal.intent.back().rollRate,0,1e-12,"Terminal twist vanishes smoothly");
    auto mirrorJob=std::async(std::launch::async,[&]{auto r=reversalRequest;r.hand=-1;return designFvdReversal(r);});
    auto finerRequest=reversalRequest;finerRequest.step*=.5;
    auto refinedReversal=designFvdReversal(finerRequest);good(refinedReversal.section);
    auto mirrored=mirrorJob.get();good(mirrored.section);
    near(norm(reversal.exit.sample.position-refinedReversal.exit.sample.position),0,1e-6,"Continuous reversal endpoint survives real step halving");
    near(norm(reversal.exit.sample.tangent-refinedReversal.exit.sample.tangent),0,1e-7,"Continuous reversal frame survives real step halving");
    check(mirrored.section.samples.size()==reversal.section.samples.size(),"Mirrored source uses the same temporal sampling");
    for(size_t i=0;i<reversal.section.samples.size();i+=29){
        auto q=reversal.section.samples[i];q.position.y=-q.position.y;q.forward.y=-q.forward.y;q.up.y=-q.up.y;
        const auto& m=mirrored.section.samples[i];
        near(norm(q.position-m.position)+norm(q.forward-m.forward)+norm(q.up-m.up),0,1e-7,"Parallel mirrored source has no shared evaluator state");
    }
    for(int variant=0;variant<4;++variant){
        auto r=reversalRequest;
        if(variant==0)r.hand=0;if(variant==1)r.exitPitch=0;if(variant==2)r.normalG=20;if(variant==3)r.entrySpeed=std::nan("");
        auto bad=designFvdReversal(r);check(!bad.section.report.valid()&&!bad.section.integrated,"Invalid reversal intent rejects before integration");
    }
    auto starvedReversal=reversalRequest;starvedReversal.entrySpeed=35;starvedReversal.exitHeight=100;
    auto starved=designFvdReversal(starvedReversal);
    check(!starved.section.report.valid()&&starved.section.report.errors.front().code=="FVD_REVERSAL_ENERGY","Infeasible exit energy is not repaired by a speed clamp");
    size_t reversalPolls=0;auto interrupted=designFvdReversal(reversalRequest,[&]{return ++reversalPolls>8;});
    check(interrupted.section.cancelled&&!interrupted.section.integrated,"Cancellation stops reversal shooting before canonical construction");
    FvdPulloutRequest pulloutRequest;pulloutRequest.entry=reversal.section.samples.back();
    auto pullout=designFvdPullout(pulloutRequest);good(pullout.section);
    near(norm(pullout.entry.sample.position-reversal.exit.sample.position),0,1e-9,"Pullout consumes actual reversal position");
    near(norm(pullout.entry.sample.tangent-reversal.exit.sample.tangent)+norm(pullout.entry.sample.up-reversal.exit.sample.up),0,1e-7,
        "Pullout consumes the free descending heading and upright frame");
    near(pullout.exit.sample.position.z,pulloutRequest.targetHeight,1e-5,"Pullout closes the lower valley height");
    near(pullout.exit.sample.tangent.z,0,1e-7,"Pullout ends level after a genuine descending passage");
    const Vec3 pulloutHeading=unit(Vec3{pulloutRequest.entry.forward.x,pulloutRequest.entry.forward.y,0});
    near(norm(pullout.exit.sample.tangent-pulloutHeading),0,1e-7,"Pullout retains the actual horizontal heading");
    near(pullout.section.samples.back().speed,reversalRequest.entrySpeed,1e-5,"Returning to entry height restores point-source energy without propulsion");
    check(pullout.slopeHoldSeconds<1&&pullout.duration<5,"Fixture reaches its valley through a short sloped approach and pullout");
    check(norm(pullout.entry.sample.curvature)<1e-7&&norm(pullout.exit.sample.curvature)<1e-7&&norm(pullout.exit.upS)<1e-7,
        "Pullout source ports retain vanishing curvature and twist");
    double pulloutPeak=0;
    for(size_t i=0;i<pullout.section.samples.size();i+=13){
        const auto& q=pullout.section.samples[i];
        check(q.forward.z<=1e-7&&q.position.z>=-1e-5,"Pullout descends monotonically instead of inserting a level reset");
        const Vec3 specific=q.curvature*(q.speed*q.speed)+q.forward*(-gravity*q.forward.z)+Vec3{0,0,gravity};
        pulloutPeak=std::max(pulloutPeak,dot(specific,q.up)/gravity);
        near(dot(specific,cross(q.forward,q.up))/gravity,0,1e-7,"Unbanked pullout does not create arbitrary lateral loading");
    }
    near(pulloutPeak,pulloutRequest.normalG,.002,"Pullout geometry supplies authored normal load");
    auto finerPulloutRequest=pulloutRequest;finerPulloutRequest.step*=.5;
    auto finerPullout=designFvdPullout(finerPulloutRequest);good(finerPullout.section);
    near(norm(pullout.exit.sample.position-finerPullout.exit.sample.position),0,1e-6,"Pullout endpoint survives actual step halving");
    auto shallowPullout=pulloutRequest;shallowPullout.targetHeight=shallowPullout.entry.position.z-.1;
    auto impossiblePullout=designFvdPullout(shallowPullout);
    check(!impossiblePullout.section.report.valid()&&!impossiblePullout.section.integrated,
        "A valley too close for the authored force cannot be forced by a geometry warp");
    auto invalidPullout=pulloutRequest;invalidPullout.entry.curvature={.01,0,0};
    check(!designFvdPullout(invalidPullout).section.report.valid(),"Pullout rejects unsupported entry curvature instead of zeroing it");
    size_t pulloutPolls=0;
    check(designFvdPullout(pulloutRequest,[&]{return ++pulloutPolls>8;}).section.cancelled,"Cancellation interrupts pullout shooting");
    for(double push:{2.05,2.2,2.5}){
        FvdAirtimeRequest request;request.pushG=push;
        auto hill=designFvdAirtime(request);good(hill.section);
        check(hill.span>350&&hill.span<550&&hill.height>15&&hill.height<40,"Force-authored airtime has useful bounded scale");
        const auto& end=hill.section.samples.back();
        near(end.position.z,0,1e-4,"Airtime closes height without spatial scaling");
        near(norm(end.forward-Vec3{1,0,0}),0,1e-5,"Airtime ends horizontal");
        near(end.speed,request.speed,1e-4,"Symmetric gravity-coupled hill closes energy");
        size_t apex=0;for(size_t i=1;i<hill.section.samples.size();++i)if(hill.section.samples[i].position.z>hill.section.samples[apex].position.z)apex=i;
        const auto& top=hill.section.samples[apex];
        double measured=dot(top.curvature*(top.speed*top.speed)+Vec3{0,0,gravity},top.up)/gravity;
        near(measured,request.crestG,.001,"Integrated geometric crest produces requested normal force");
        for(size_t i=0;i<hill.section.samples.size();i+=19){
            const auto& q=hill.section.samples[i];
            near(q.speed*q.speed+2*gravity*q.position.z,request.speed*request.speed,1e-5,"Independent airtime mechanical-energy oracle");
        }
    }
    check(designFvdAirtime(FvdAirtimeRequest{},[]{return true;}).section.cancelled,"Airtime shooting respects cancellation");
    FvdAirtimeRequest invalidHill;invalidHill.pushG=8;
    check(!designFvdAirtime(invalidHill).section.report.valid(),"Invalid airtime control rejects");
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
    const double firstDifference=norm(variedCoarse.samples.back().position-variedFine.samples.back().position);
    const double secondDifference=norm(variedFine.samples.back().position-variedFinest.samples.back().position);
    check(secondDifference<firstDifference*.35,"Smooth force and roll controls converge under genuine step halving");
    check(variedFine.assessment.evaluations>variedFine.samples.size(),"Canonical replay samples between authoring knots");
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
