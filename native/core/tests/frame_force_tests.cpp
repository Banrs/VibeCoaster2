#include "coaster/coaster.hpp"
#include "../src/bank_target.hpp"
#include "../src/turn_bank_profile.hpp"
#include "../src/turn_shape.hpp"
#include <iostream>
#include <stdexcept>
using namespace coaster;
static int checks=0;
static void check(bool value,const char* message){++checks;if(!value)throw std::runtime_error(message);}
static void near(double a,double b,double tolerance,const char* message){check(std::isfinite(a)&&std::isfinite(b)&&std::abs(a-b)<=tolerance,message);}
static Track straight(double linear,double quadratic){
    Track t;t.closed=false;
    for(double x:{0.,1.3,3.9,7.,11.2,17.,26.,39.,54.,73.,98.,126.,160.})
        t.knots.push_back({{x,0,50},{1,0,0},{},{0,0,1},.13+linear*x+quadratic*x*x,Element::Return});
    t.rebuild();return t;
}
int main(){try{
    // An independent fine-step heading/displacement integral checks the bank-
    // first turn. Bank intent balances gravity and lateral acceleration at the
    // authored speed; it is not a smoothed replacement for measured forces.
    for(double heading:{1.5,pi,4.2}){
        const double speed=47.64,radius=74,seconds=1.5,peak=std::atan(speed*speed/(gravity*radius));
        auto turn=detail::makeTurn(heading,radius,speed*seconds,peak);
        auto mirror=detail::makeTurn(-heading,radius,speed*seconds,peak);
        double angle=0;Vec3 position{};const int count=200000;const double ds=turn.length/count;
        for(int i=0;i<count;++i){
            double s=(i+.5)*ds,u=std::clamp(std::min(s,turn.length-s)/turn.ramp,0.,1.);
            double beta=peak*u*u*u*(10+u*(-15+6*u));
            double curvature=gravity*std::tan(beta)/(speed*speed);
            position=position+Vec3{std::cos(angle+curvature*ds/2),std::sin(angle+curvature*ds/2),0}*ds;angle+=curvature*ds;
        }
        near(angle,heading,2e-9,"Bank-first turn retains the requested total heading");
        near(norm(position-turn.points.back()),0,.006,"Bank-first turn displacement agrees with independent fine integration");
        near(turn.points.back().x,mirror.points.back().x,1e-12,"Bank-first turn mirrors its longitudinal displacement");
        near(turn.points.back().y,-mirror.points.back().y,1e-12,"Bank-first turn mirrors its lateral displacement");
        near(turn.bankAt(turn.ramp),-peak,1e-12,"Turn bank opposes signed curvature in the canonical frame");
        near(mirror.bankAt(mirror.ramp),peak,1e-12,"Mirrored turn reverses bank intent");
        for(double s:{0.,turn.length})near(turn.bankAt(s),0,0,"Turn bank returns upright at both ports");
        const double h=1e-3;
        near(turn.bankAt(h)/h,0,1e-9,"Entry bank has zero first arc jet");
        near((turn.bankAt(2*h)-2*turn.bankAt(h))/(h*h),0,1e-6,"Entry bank has zero second arc jet");
        near(turn.bankAt(turn.length-h)/h,0,1e-9,"Exit bank has zero first arc jet");
        near((turn.bankAt(turn.length-2*h)-2*turn.bankAt(turn.length-h))/(h*h),0,1e-6,"Exit bank has zero second arc jet");
    }
    {
        const double speed=47.64,radius=74,seconds=1.5,peak=std::atan(speed*speed/(gravity*radius)),dt=1e-4;
        auto turn=detail::makeTurn(pi,radius,speed*seconds,peak);
        double rate=0,acceleration=0,oldAcceleration=0;
        auto oldBank=[&](double time){return std::atan(std::tan(peak)*smooth(time/seconds));};
        for(double time=dt;time<seconds-dt;time+=dt){
            double a=turn.bankAt((time-dt)*speed),b=turn.bankAt(time*speed),c=turn.bankAt((time+dt)*speed);
            rate=std::max(rate,std::abs(c-a)/(2*dt));acceleration=std::max(acceleration,std::abs(c-2*b+a)/(dt*dt));
            oldAcceleration=std::max(oldAcceleration,std::abs(oldBank(time+dt)-2*oldBank(time)+oldBank(time-dt))/(dt*dt));
        }
        near(rate,peak*1.875/seconds,1e-6,"Turn roll rate follows the authored quintic bank trajectory");
        near(acceleration,peak*(10*std::sqrt(3.)/3)/(seconds*seconds),1e-5,"Turn roll acceleration follows the authored quintic bank trajectory");
        check(acceleration<oldAcceleration*.6,"Bank-first geometry removes nonlinear roll acceleration concentration");
    }
    // Canyon0's terminal crest has negative normal load before lateral
    // curvature grows. Its almost vertical force must not request an85deg roll.
    for(double lateral:{2.24111807115e-6,-1.95299849792e-5}){
        const double normal=-.109125664584;
        double bank=detail::forceAxisBank(normal*gravity,lateral*gravity,0);
        check(std::abs(bank)<.001,"Negative-load crest retains nearly upright bank");
        near(lateral*std::cos(bank)-normal*std::sin(bank),0,1e-12,"Force-axis banking cancels the actual lateral component");
        check(normal*std::cos(bank)+lateral*std::sin(bank)<0,"Force-axis banking preserves negative normal force");
        near(bank,detail::forceAxisBank(-normal*gravity,-lateral*gravity,0),1e-12,"Reversing force direction leaves its bank axis unchanged");
    }
    near(detail::forceAxisBank(gravity,.7*gravity,0),std::atan2(.7,1.),1e-12,"Positive-load bank target remains unchanged");
    near(detail::forceAxisBank(0,0,.4),.4,0,"Zero resultant retains authored bank");
    near(detail::forceAxisBank(0,0,2),2,0,"Zero resultant retains an authored overbank");
    // The grounded canyon climb combines pitch unwind with horizontal turn
    // force. Its resultant passes below horizontal while remaining positive
    // in the rider frame; an85-degree cap leaves over1.6g lateral force.
    for(double hand:{-1.,1.})for(double degrees:{88.,90.,92.,118.}){
        const double angle=hand*degrees*pi/180,authored=hand*70*pi/180;
        const double normal=3.1*gravity*std::cos(angle),lateral=3.1*gravity*std::sin(angle);
        const double bank=detail::forceAxisBank(normal,lateral,authored);
        near(bank,angle,1e-12,"Curved crest bank follows its force axis through ninety degrees");
        near(lateral*std::cos(bank)-normal*std::sin(bank),0,1e-12,"Overbanked crest cancels its lateral force");
        near(normal*std::cos(bank)+lateral*std::sin(bank),3.1*gravity,1e-12,"Overbanked crest retains its positive rider load");
        near(detail::forceAxisBank(-normal,-lateral,authored),bank,1e-12,"Overbank axis is invariant when resultant force reverses");
    }
    {
        Track source;source.closed=false;
        for(int i=0;i<=50;++i){double angle=i*6./150;
            source.knots.push_back({{150*std::sin(angle),150*(1-std::cos(angle)),80},{std::cos(angle),std::sin(angle),0},{-std::sin(angle)/150,std::cos(angle)/150,0},{0,0,1},0,Element::Turn});}
        source.rebuild();double previousPeak=0;
        for(double velocity:{20.,40.}){
            auto fitted=source;std::vector<double> speeds(source.spans.size(),velocity);
            const size_t first=3,last=46;auto beforeA=sampleSpanKinematics(source,first,0),beforeB=sampleSpanKinematics(source,last,0);
            check(detail::fitTurnBankProfile(fitted,first,last,speeds),"Continuous turn bank profile fits the actual force demand");
            double baselineCost=0,fittedCost=0,peak=0;
            for(size_t i=first;i<last;++i){
                double s=source.spans[i].start;auto a=measureSeatForces(source,s,velocity,0,0),b=measureSeatForces(fitted,s,velocity,0,0);
                baselineCost+=a.lateral*a.lateral*source.spans[i].length;fittedCost+=b.lateral*b.lateral*source.spans[i].length;
                peak=std::max(peak,std::abs(fitted.knots[i].bank));check(std::abs(fitted.knots[i].bank)<=1.5+1e-12,"Continuous bank fit retains the existing bank bound");
            }
            check(fittedCost<baselineCost,"Continuous bank fit reduces actual lateral force residual");
            check(peak>previousPeak,"Higher speed changes the fitted bank rather than using a fixed angle");previousPeak=peak;
            for(auto pair:{std::pair{first,beforeA},std::pair{last,beforeB}}){auto after=sampleSpanKinematics(fitted,pair.first,0);
                near(norm(after.sample.position-pair.second.sample.position),0,0,"Bank fitting leaves boundary position exact");
                near(norm(after.sample.up-pair.second.sample.up),0,0,"Bank fitting retains boundary orientation exact");
                near(norm(after.upS-pair.second.upS),0,0,"Bank fitting retains exact first frame jet");
                near(norm(after.upSS-pair.second.upSS),0,0,"Bank fitting retains exact second frame jet");
            }
            for(size_t i=0;i<source.knots.size();++i)if(i<=first+3||i>=last-3)
                near(fitted.knots[i].bank,source.knots[i].bank,0,"Turn bank fitting preserves all exterior and guard knots");
        }
    }
    {
        Track ring;ring.closed=true;
        for(int i=0;i<64;++i){double angle=2*pi*i/64;
            ring.knots.push_back({{150*std::sin(angle),150*(1-std::cos(angle)),80},{std::cos(angle),std::sin(angle),0},{-std::sin(angle)/150,std::cos(angle)/150,0},{0,0,1},0,i<3?Element::Station:Element::Turn});}
        ring.knots.push_back(ring.knots.front());ring.rebuild();auto before=sampleSpanKinematics(ring,0,0);
        std::vector<double> speeds(ring.spans.size(),20);
        check(detail::fitTurnBankProfile(ring,3,ring.spans.size(),speeds),"Terminal turn ending at the closed seam receives its bank fit");
        auto after=sampleSpanKinematics(ring,0,0);
        near(norm(after.sample.up-before.sample.up),0,0,"Terminal fit preserves exact seam orientation");
        near(norm(after.upS-before.upS),0,0,"Terminal fit preserves exact seam first frame jet");
        near(norm(after.upSS-before.upSS),0,0,"Terminal fit preserves exact seam second frame jet");
    }
    // Independent closed-form rigid offset on a straight track. Quadratic bank
    // tests angular acceleration as well as the centripetal offset term.
    for(double quadratic:{0.,.0003}){
        auto t=straight(.015,quadratic);const double v=37,a=2.3,h=1.2;
        for(double s:{.4,1.3,2.8,3.9,8.,17.,23.,39.,62.,98.,131.}){
            double bank=.13+.015*s+quadratic*s*s,rate=.015+2*quadratic*s,second=2*quadratic;
            auto k=sampleKinematics(t,s);auto f=measureSeatForces(t,s,v,a,h);
            near(f.vertical,std::cos(bank)-h*rate*rate*v*v/gravity,2e-8,"Analytic rotating offset vertical force");
            near(f.lateral,-std::sin(bank)+h*(a*rate+v*v*second)/gravity,2e-8,"Analytic rotating offset lateral force");
            near(f.longitudinal,a/gravity,2e-8,"Offset rotation does not invent longitudinal force");
            Vec3 up{0,-std::sin(bank),std::cos(bank)},right=cross({1,0,0},up);
            near(norm(k.upS-right*rate),0,2e-10,"Exact bank first arc derivative on unequal spans");
            near(norm(k.upSS-(right*second-up*(rate*rate))),0,2e-10,"Exact bank second arc derivative on unequal spans");
        }
    }
    // A quartic bank on unequal straight spans is an independent analytic
    // oracle for the five-point jets. Three-point jets miss this by .001 g.
    auto quartic=straight(0,0);auto bank=[](double s){return .13+.003*s+.00003*s*s+1e-7*s*s*s-2e-10*s*s*s*s;};
    for(auto& knot:quartic.knots)knot.bank=bank(knot.position.x);quartic.rebuild();
    for(double s=.25;s<159.75;s+=.37){double b=bank(s),first=.003+.00006*s+3e-7*s*s-8e-10*s*s*s,second=.00006+6e-7*s-24e-10*s*s;Vec3 u{0,-std::sin(b),std::cos(b)},r=cross({1,0,0},u);auto k=sampleKinematics(quartic,s);auto f=measureSeatForces(quartic,s,37,2.3,1.2);
        near(f.vertical,std::cos(b)-1.2*first*first*37*37/gravity,1e-10,"Quartic-bank analytic vertical force");near(f.lateral,-std::sin(b)+1.2*(2.3*first+37*37*second)/gravity,1e-10,"Quartic-bank analytic lateral force");near(norm(k.upS-r*first),0,1e-11,"Quartic-bank first derivative on unequal spans");near(norm(k.upSS-(r*second-u*(first*first))),0,1e-11,"Quartic-bank second derivative on unequal spans");}
    std::vector<AuthoredPoint> points;
    for(int i=0;i<=90;++i){double s=180*std::pow(i/90.,1.12),theta=s/500;Vec3 tangent=unit({std::cos(theta),std::sin(theta),(2./30)*std::cos(s/30)});
        Vec3 up=unit(Vec3{0,0,1}-tangent*tangent.z);up=rotate(up,tangent,.02*std::sin(s/19));
        points.push_back({{500*std::sin(theta),500*(1-std::cos(theta)),50+2*std::sin(s/30)},.25*std::sin(s/24),Element::Return,up});}
    auto curved=compile(points,false);
    for(size_t i=1;i<curved.spans.size();++i){auto a=sampleSpanKinematics(curved,i-1,1),b=sampleSpanKinematics(curved,i,0);
        near(norm(a.sample.up-b.sample.up),0,1e-9,"Reference frame value is continuous");
        near(norm(a.upS-b.upS),0,2e-8,"Reference frame first derivative has no impulse at a knot");
        near(norm(a.upSS-b.upSS),0,2e-7,"Reference frame second derivative is continuous in true arc");
        near(norm(a.curvatureS-b.curvatureS),0,2e-7,"Position supplies continuous curvature derivative");
    }
    for(size_t i=0;i<curved.spans.size();i+=3)for(double u:{0.,.23,.61,1.}){
        auto k=sampleSpanKinematics(curved,i,u);auto p=curved.sampleSpan(i,u);
        near(norm(p.position-k.sample.position)+norm(p.up-k.sample.up)+norm(p.curvature-k.sample.curvature),0,2e-11,"Renderer and force evaluator share the same canonical frame");
        near(dot(k.sample.up,k.upS),0,2e-12,"Unit-up first derivative identity");
        near(dot(k.sample.up,k.upSS)+dot(k.upS,k.upS),0,2e-11,"Unit-up second derivative identity");
        near(dot(k.sample.tangent,k.upS)+dot(k.sample.curvature,k.sample.up),0,2e-11,"Frame remains orthogonal while differentiating");
        near(dot(k.curvatureS,k.sample.up)+2*dot(k.sample.curvature,k.upS)+dot(k.sample.tangent,k.upSS),0,2e-10,"Second orthogonality derivative identity");
    }
    auto track=straight(.003,0);TrainConfig train;train.cars=1;
    std::vector<Operation> ops{{0,track.length,DriveKind::Launch,20,4000,1000000,.2}};
    auto coarse=simulate(track,ops,train,1./240),fine=simulate(track,ops,train,1./480);
    check(coarse.completed&&fine.completed&&coarse.report.valid()&&fine.report.valid(),"Physical test train completes at both actual steps");
    near(coarse.metrics.maxSpeed,fine.metrics.maxSpeed,.01,"Unchanged finite-train dynamics converge");
    for(const auto* result:{&coarse,&fine}){
        double rate=0;for(const auto& seat:result->metrics.seats)rate=std::max(rate,seat.axes[0].maxRateGps);
        near(rate,result->metrics.maxJerkGps,0,"Global and per-seat analytic vertical rates use one convention");
    }
    check(std::abs(coarse.metrics.maxJerkGps-fine.metrics.maxJerkGps)/std::max(1.,std::abs(fine.metrics.maxJerkGps))<.02,"Analytic vertical rate converges without differencing or smoothing");
    auto finest=simulate(track,ops,train,1./4000);check(finest.completed&&finest.report.valid(),"Minimum public requested step has a valid genuine half-step");
    check(!simulate(track,ops,train,1./8000).report.valid(),"Unsupported finer step rejects instead of clamping");
    // Isolated finite drive zones must fade their actual capped force, not just
    // coalesce duplicate sections. Without the fade, the 6m/s^2 exit cut gives
    // an apparent longitudinal rate proportional to the sample frequency.
    auto controlled=straight(0,0);TrainConfig controlTrain;controlTrain.cars=1;controlTrain.carMass=100;controlTrain.dragCdA=0;controlTrain.airDensity=0;controlTrain.rollingResistance=0;
    std::vector<Operation> controlOps{{0,20,DriveKind::Launch,20,2000,1000000,.2},{20,100,DriveKind::Boost,100,600,1000000,.2}};
    for(auto& op:controlOps)op.exitFadeMeters=10;
    auto c=simulate(controlled,controlOps,controlTrain,1./960),f=simulate(controlled,controlOps,controlTrain,1./1920);
    check(c.completed&&f.completed&&c.report.valid()&&f.report.valid(),"Fading isolated drive zones retain physical completion");
    double maxRate=f.metrics.seats[0].axes[2].maxRateGps;
    // Product rule bound: demand feedback gain4, entry smoothstep slope1.5,
    // and spatial fade slope1.5/distance; all multiplied by physical caps.
    double rateBound=(4*20+20*1.5/.2+20*1.5*f.metrics.maxSpeed/10)/gravity;
    check(maxRate>0&&maxRate<rateBound,"Finite drive force respects its analytic derivative bound at isolated exits");
    check(std::abs(c.metrics.seats[0].axes[2].maxRateGps-maxRate)/std::max(1.,maxRate)<.02,"Actual longitudinal-rate measurement converges after a finite exit fade");
    check(f.metrics.maxLongitudinalG<=20/gravity+1e-10,"Entry and exit multipliers never raise the force cap");
    for(double fade:{0.,.001,1001.,double(NAN),double(INFINITY)}){auto badOps=controlOps;badOps[0].exitFadeMeters=fade;auto invalid=simulate(controlled,badOps,controlTrain,1./960);check(!invalid.report.valid()&&invalid.report.errors.front().code=="DRIVE_CONFIG","Unsupported fade rejects before simulation");}
    // Wrapping profiles have the same exit distance while occupying the same
    // first interval. The second interval must still produce bounded drive.
    auto nonwrap=controlOps;nonwrap.resize(1);nonwrap[0].start=0;nonwrap[0].end=80;nonwrap[0].targetSpeed=100;nonwrap[0].maxForce=600;
    auto wrap=nonwrap;wrap[0].start=140;
    auto normal=simulate(controlled,nonwrap,controlTrain,1./960),wrapped=simulate(controlled,wrap,controlTrain,1./960);
    check(normal.completed&&wrapped.completed,"Wrapping and nonwrapping drive fixtures complete");
    size_t common=0;for(size_t i=0;i<std::min(normal.frames.size(),wrapped.frames.size())&&normal.frames[i].distance<80;++i){near(normal.frames[i].speed,wrapped.frames[i].speed,1e-12,"Wrapped forward-exit distance matches the equivalent active interval");near(normal.frames[i].seats[0].longitudinal,wrapped.frames[i].seats[0].longitudinal,1e-12,"Wrapped interval preserves the actual force multiplier");++common;}
    check(common>30&&wrapped.frames.back().speed>normal.frames.back().speed,"Wrapped second active interval is not incorrectly faded to zero");
    std::cout<<"PASS "<<checks<<" frame/force checks: true-arc C2 joins, analytic offset force, renderer parity, finite train and actual half-step rates\n";return 0;
}catch(const std::exception& e){std::cerr<<"FAIL after "<<checks<<": "<<e.what()<<'\n';return 1;}}
