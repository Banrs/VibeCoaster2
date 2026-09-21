#include "coaster/coaster.hpp"
#include "../src/simulation_internal.hpp"
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
    size_t positionHint=curved.spans.size();
    for(double s=0.;s<=curved.length;s+=.73){
        const auto position=curved.position(s,positionHint);
        near(norm(position-curved.sample(s).position),0,2e-12,"Position-only canonical sampling matches the full frame position");
    }
    for(size_t i=1;i<curved.spans.size();++i){auto a=sampleSpanKinematics(curved,i-1,1),b=sampleSpanKinematics(curved,i,0);
        near(norm(a.sample.up-b.sample.up),0,1e-9,"Reference frame value is continuous");
        near(norm(a.upS-b.upS),0,2e-8,"Reference frame first derivative has no impulse at a knot");
        near(norm(a.upSS-b.upSS),0,2e-7,"Reference frame second derivative is continuous in true arc");
        near(norm(a.curvatureS-b.curvatureS),0,2e-7,"Position supplies continuous curvature derivative");
        near(norm(a.upSSS-b.upSSS),0,2e-7,"Rider frame third derivative is continuous at every knot");
        near(norm(a.curvatureSS-b.curvatureSS),0,2e-7,"Tangent third derivative is continuous at every knot");
    }
    for(size_t i=0;i<curved.spans.size();i+=3)for(double u:{0.,.23,.61,1.}){
        auto k=sampleSpanKinematics(curved,i,u);auto p=curved.sampleSpan(i,u);
        near(norm(p.position-k.sample.position)+norm(p.up-k.sample.up)+norm(p.curvature-k.sample.curvature),0,2e-11,"Renderer and force evaluator share the same canonical frame");
        near(dot(k.sample.up,k.upS),0,2e-12,"Unit-up first derivative identity");
        near(dot(k.sample.up,k.upSS)+dot(k.upS,k.upS),0,2e-11,"Unit-up second derivative identity");
        near(dot(k.sample.tangent,k.upS)+dot(k.sample.curvature,k.sample.up),0,2e-11,"Frame remains orthogonal while differentiating");
        near(dot(k.curvatureS,k.sample.up)+2*dot(k.sample.curvature,k.upS)+dot(k.sample.tangent,k.upSS),0,2e-10,"Second orthogonality derivative identity");
        near(dot(k.sample.up,k.upSSS)+3*dot(k.upS,k.upSS),0,2e-10,"Unit-up third derivative identity");
        near(dot(k.curvatureSS,k.sample.up)+3*dot(k.curvatureS,k.upS)+3*dot(k.sample.curvature,k.upSS)+dot(k.sample.tangent,k.upSSS),0,2e-10,"Third orthogonality derivative identity");
    }
    for(double s=.25;s<159.75;s+=.37){
        double b=bank(s),first=.003+.00006*s+3e-7*s*s-8e-10*s*s*s,second=.00006+6e-7*s-24e-10*s*s,third=6e-7-48e-10*s;
        Vec3 u{0,-std::sin(b),std::cos(b)},r=cross({1,0,0},u);
        near(norm(sampleKinematics(quartic,s).upSSS-(r*(third-first*first*first)-u*(3*first*second))),0,1e-10,"Quartic-bank closed-form third derivative on unequal spans");
    }
    auto track=straight(.003,0);TrainConfig train;train.cars=1;
    // Differentiate a physical seat trajectory in time, independently of the
    // analytic frame jets. Component force rate includes rotating rider axes.
    const double at=80,velocity=31,accel=3,accelRate=-1.2,height=1.2,h=.0005;
    const auto dynamic=measureSeatDynamics(track,at,velocity,accel,accelRate,height);
    auto stateAt=[&](double t){return std::array<double,3>{at+velocity*t+.5*accel*t*t+accelRate*t*t*t/6,velocity+accel*t+.5*accelRate*t*t,accel+accelRate*t};};
    auto specificAt=[&](double t){const auto x=stateAt(t);const auto k=sampleKinematics(track,x[0]);return (k.sample.tangent+k.upS*height)*x[2]+(k.sample.curvature+k.upSS*height)*(x[1]*x[1])+Vec3{0,0,gravity};};
    near(norm(dynamic.inertialJerk-(specificAt(h)-specificAt(-h))/(2*h)),0,1e-5,"Seat inertial jerk matches an independent time derivative including offset rotation");
    const auto xm=stateAt(-h),xp=stateAt(h);const auto fm=measureSeatForces(track,xm[0],xm[1],xm[2],height),fp=measureSeatForces(track,xp[0],xp[1],xp[2],height);
    near(dynamic.rate.vertical,(fp.vertical-fm.vertical)/(2*h),1e-6,"Vertical body-axis rate includes rotation");
    near(dynamic.rate.lateral,(fp.lateral-fm.lateral)/(2*h),1e-6,"Lateral body-axis rate includes rotation");
    near(dynamic.rate.longitudinal,(fp.longitudinal-fm.longitudinal)/(2*h),1e-6,"Longitudinal body-axis rate includes actuator jerk");
    near(norm(dynamic.angularVelocity),.003*velocity,1e-9,"Angular velocity uses the physical frame through twist");
    near(norm(dynamic.angularJerk),.003*std::abs(accelRate),1e-8,"Constant spatial twist has the expected time angular jerk");
    std::vector<Operation> ops{{0,track.length,DriveKind::Launch,20,4000,1000000,.2}};
    auto coarse=simulate(track,ops,train,1./240),fine=simulate(track,ops,train,1./480);
    const auto motion=simulateMotion(track,ops,train,1./240,{});
    check(motion.completed==coarse.completed&&motion.cancelled==coarse.cancelled&&motion.frames.size()==coarse.frames.size(),"Motion-only authoring preserves completion and trace cadence");
    for(size_t i=0;i<motion.frames.size();++i){const auto& a=motion.frames[i];const auto& b=coarse.frames[i];
        check(a.time==b.time&&a.distance==b.distance&&a.speed==b.speed,"Preliminary authoring motion is bit-exact with full rider-force replay");}
    check(coarse.completed&&fine.completed&&coarse.report.valid()&&fine.report.valid(),"Physical test train completes at both actual steps");
    near(coarse.metrics.maxSpeed,fine.metrics.maxSpeed,.01,"Unchanged finite-train dynamics converge");
    for(const auto* result:{&coarse,&fine}){
        double rate=0;for(const auto& seat:result->metrics.seats)rate=std::max(rate,seat.axes[0].maxRateGps);
        near(rate,result->metrics.maxJerkGps,0,"Global and per-seat analytic vertical rates use one convention");
        double exposure=0;for(const auto& seat:result->metrics.seats)exposure=std::max(exposure,seat.exposure10Seconds);
        near(exposure,result->metrics.exposure10Seconds,0,"Global exposure is the maximum of the same per-seat measurements");
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
    std::cout<<"PASS "<<checks<<" frame/force checks: true-arc C3 joins, analytic offset force, renderer parity, finite train and actual half-step rates\n";return 0;
}catch(const std::exception& e){std::cerr<<"FAIL after "<<checks<<": "<<e.what()<<'\n';return 1;}}
