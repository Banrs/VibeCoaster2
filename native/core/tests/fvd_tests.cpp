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
