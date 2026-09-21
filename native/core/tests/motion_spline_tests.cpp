#include "../src/banking.hpp"
#include "../src/motion_program.hpp"
#include "../src/authoring.hpp"
#include "coaster/fvd.hpp"
#include <iostream>
#include <stdexcept>
#include <source_location>
using namespace coaster;
using namespace coaster::detail;
namespace {
int checks=0;
void check(bool value,const char* message){++checks;if(!value)throw std::runtime_error(message);}
void close(Vec3 a,Vec3 b,double tolerance=1e-9,std::source_location location=std::source_location::current()){if(norm(a-b)>=tolerance*std::max(1.,norm(b)))std::cerr<<"line="<<location.line()<<" error="<<norm(a-b)<<" tolerance="<<tolerance<<" a="<<a.x<<","<<a.y<<","<<a.z<<" b="<<b.x<<","<<b.y<<","<<b.z<<"\n";check(norm(a-b)<tolerance*std::max(1.,norm(b)),"Motion derivative differs from independent expectation");}
void derivatives(const MotionProgram& p) {
    // Probe limits rather than the exact endpoint accessor, which returns the
    // supplied boundary. A broken endpoint coefficient must fail this test.
    for(bool end:{false,true}) {
        const double h=1e-6;const auto far=p.direction(end?p.length-h:h),near=p.direction(end?p.length-h*.5:h*.5),expected=end?p.end:p.begin;
        // Extrapolate the one-sided interior limit. A finite offset also has
        // its real next derivative; it need not equal the endpoint value.
        // The exact endpoint accessor is deliberately never used as actual.
        close(near.tangent*2-far.tangent,expected.tangent,1e-8);close(near.curvature*2-far.curvature,expected.curvature,1e-9);
        close(near.third*2-far.third,expected.third,1e-10);close(near.fourth*2-far.fourth,expected.fourth,1e-10);
    }
    for(int i=1;i<100;++i) {
        const double s=p.length*i/100,h=.001;
        const auto lo=p.direction(s-h),hi=p.direction(s+h),q=p.direction(s);
        close((hi.tangent-lo.tangent)/(2*h),q.curvature,2e-8);
        close((hi.curvature-lo.curvature)/(2*h),q.third,1e-9);
        close((hi.third-lo.third)/(2*h),q.fourth,1e-9);
        close((p.displacement(s+h)-p.displacement(s-h))/(2*h),q.tangent,2e-8);
    }
}
}
int main(){try{
    const std::array<double,8> coefficients{5,7,2,-.2,.03,-.004,.0005,-.00006};
    auto scalar=[&](double t,int order){double value=0;for(int k=7;k>=order;--k){double factor=1;for(int j=0;j<order;++j)factor*=k-j;value=value*t+coefficients[k]*factor;}return value;};
    auto frame=[&](double t){Frame f;f.time=t;f.distance=scalar(t,0);f.speed=scalar(t,1);f.acceleration=scalar(t,2);f.accelerationRate=scalar(t,3);return f;};
    for(double t=0;t<=2;t+=.013){const auto q=interpolateMotion(frame(0),frame(2),t);
        check(std::abs(q.distance-scalar(t,0))<1e-10&&std::abs(q.speed-scalar(t,1))<1e-10&&std::abs(q.acceleration-scalar(t,2))<1e-10&&std::abs(q.jerk-scalar(t,3))<1e-9,"Replay must reproduce an independent seventh-degree time trajectory");}
    const auto stopped=interpolateMotion(frame(1),frame(1),1);check(stopped.distance==scalar(1,0)&&stopped.speed==scalar(1,1),"Paused replay has no division by zero");

    // Linear angle splines have an exact circular-arc oracle. Greville points
    // reproduce a linear function for any nonuniform B-spline knot vector.
    MotionProgram circle;circle.length=320;constexpr double radius=240;
    std::array<double,MotionProgram::controlCount+MotionProgram::degree+1> knots{};
    constexpr int intervals=MotionProgram::controlCount-MotionProgram::degree;
    for(size_t i=0;i<knots.size();++i)knots[i]=i<=7?0:i>=32?1:.5*(1-std::cos(pi*(int(i)-7)/intervals));
    for(int i=0;i<MotionProgram::controlCount;++i){double mean=0;for(int j=1;j<=7;++j)mean+=knots[i+j]/7;circle.heading[i]=mean*circle.length/radius;}
    auto exact=[&](double s){auto q=directionJet({},{s/radius,1/radius,0,0});q.position={radius*std::sin(s/radius),radius*(1-std::cos(s/radius)),0};return q;};
    circle.begin=exact(0);circle.end=exact(circle.length);derivatives(circle);
    for(int i=0;i<=100;++i){double s=circle.length*i/100;auto q=circle.direction(s),e=exact(s);close(q.tangent,e.tangent,1e-12);close(q.curvature,e.curvature,1e-12);close(q.third,e.third,1e-12);close(q.fourth,e.fourth,1e-12);close(circle.displacement(s),e.position,1e-10);}

    for(double degrees:{178.5,180.}){
        const double length=radius*degrees*pi/180;const auto start=exact(0),finish=exact(length);
        const auto reversal=solveMotion(start,finish,length,{30,0,0,0});
        derivatives(reversal);close(reversal.displacement(reversal.length),finish.position,1e-8);
        const auto bearing=unit(finish.position);
        check(dot(finish.tangent,bearing)<.02,"Near-semicircle fixture exercises the former false rejection");
        for(int i=0;i<=100;++i){const auto q=reversal.direction(reversal.length*i/100);
            check(dot(q.tangent,bearing)>=-1e-9&&q.curvature.y*q.tangent.x-q.curvature.x*q.tangent.y>0,"A broad half-turn advances without a hidden heading reversal");}
    }
    auto a=planarJet({0,0,8},0,0),b=planarJet({600,0,225},0,0,-.006);
    const MotionIntent force{83.5,gravity*.004,.0002,0};
    int cancelCalls=0;bool cancelledSolve=false;
    try{solveMotion(a,b,680,force,[&]{return ++cancelCalls>=3;});}catch(const std::runtime_error& error){cancelledSolve=std::string(error.what())=="CANCELLED";}
    check(cancelledSolve&&cancelCalls<=4,"Cancellation interrupts the inner placement solve without retrying another initialization");
    const auto hill=solveMotion(a,b,680,force);derivatives(hill);close(hill.displacement(hill.length),b.position-a.position,2e-7);
    double previous=0;int pitchExtrema=0,previousSign=1;
    for(int i=1;i<1000;++i){auto q=hill.direction(hill.length*i/1000);const double pitch=std::asin(q.tangent.z);int sign=pitch>previous?1:-1;if(sign!=previousSign)++pitchExtrema;previousSign=sign;previous=pitch;check(q.tangent.z>=-1e-9,"Single ascent cannot hide an interior downhill dip");}
    check(pitchExtrema==1,"Single ascent has exactly one pitch maximum");
    const Vec3 offset{4200,-3500,180};a.position=a.position+offset;b.position=b.position+offset;
    const auto shifted=solveMotion(a,b,680,force);
    check(std::abs(shifted.length-hill.length)<1e-10,"Translation cannot change the solved length");
    for(int i=1;i<100;++i){const auto q=hill.direction(hill.length*i/100),r=shifted.direction(shifted.length*i/100);close(q.tangent,r.tangent,1e-11);close(q.fourth,r.fourth,1e-11);}

    auto entry=directionJet({.2,.001,-.00001,.0000001},{.2,.002,.000003,-.00000002});entry.position={0,0,20};
    auto exit=directionJet({0,-.006,0,0},{.7,.001,0,0});exit.position={300,155,105};
    const auto live=solveMotion(entry,exit,370,{58,gravity*.004,.0002,0});derivatives(live);
    close(live.displacement(live.length),exit.position-entry.position,2e-7);
    // A turning pull-up places the next FVD element from its integrated motion.
    // Forcing both its position and full direction jet formerly concentrated
    // the turn into a few metres; a lone septic also introduced pitch shoulders.
    auto turnEntry=directionJet({.12,.00575,0,0},{0,0,0,0});turnEntry.position={12,-7,4};
    const AngleJet turnPitch{55*pi/180,.0121,0,0},turnYaw{45*pi/180,0,0,0};
    const double turnLength=215;
    const auto hint=polynomialMotion(turnEntry,anglePolynomial({.12,.00575,0,0},turnPitch,turnLength),anglePolynomial({},turnYaw,turnLength),turnLength);
    const auto turning=solveDirectionMotion(turnEntry,hint.end,turnLength,{71,gravity*.004,.0002,0,4.4,-.95});
    derivatives(turning);
    check(std::abs(turning.length-turnLength)<1e-8,"Free-displacement authoring preserves its selected pacing length");
    close(turning.end.position,turnEntry.position+turning.displacement(turning.length),1e-9);
    check(norm(turning.end.position-hint.end.position)>.1,"The free motion solver must place the element instead of forcing the rough endpoint");
    for(int i=0;i<=2000;++i){const auto q=turning.direction(turning.length*i/2000);
        check(q.curvature.z>=-1e-9,"A loaded turning pull-up has no interior pitch shoulder");
        check(cross(q.tangent,q.curvature).z>=-1e-9,"The directional handoff has one purposeful turn without a yaw reversal");}
    const auto gradedStart=planarJet({0,0,51.78},0,6*pi/180),gradedEnd=planarJet({141.36,0,65.7986},0,0,-.003793);
    const auto graded=solveMotion(gradedStart,gradedEnd,159,{66,gravity*.004,.0002,0});
    const auto levelStart=planarJet({0,0,5.00000003},pi/2,0),levelEnd=planarJet({210,175,5},-.2,0);
    const auto levelTurn=solveMotion(levelStart,levelEnd,350,{49,gravity*.004,.0002,0,4.4,-.95});
    derivatives(levelTurn);close(levelTurn.displacement(levelTurn.length),levelEnd.position-levelStart.position,1e-9);
    for(int i=0;i<=250;++i){const auto q=levelTurn.direction(levelTurn.length*i/250);
        check(std::abs(q.tangent.z)<1e-6&&norm(q.curvature)*49*49/gravity<4,"A nanometre height residual cannot collapse a level corridor into a sharp endpoint bend");}
    derivatives(graded);close(graded.displacement(graded.length),gradedEnd.position-gradedStart.position,2e-7);
    for(int i=0;i<=100;++i){const auto q=graded.direction(graded.length*i/100);check(q.tangent.z>=-1e-9&&q.tangent.z<std::sin(15*pi/180),"Graded launch exit releases without a dip or exaggerated shoulder");}
    for(auto interval:std::array<std::pair<double,double>,3>{{{0,70},{70,190},{190,300}}}){
        const auto clipped=restrictAngle(coefficients,300,interval.first,interval.second);
        for(int i=0;i<=100;++i){const double at=(interval.second-interval.first)*i/100;
            const auto full=angleAt(coefficients,interval.first+at,300),part=angleAt(clipped,at,interval.second-interval.first);
            check(std::abs(full.value-part.value)<1e-11&&std::abs(full.first-part.first)<1e-12&&std::abs(full.second-part.second)<1e-13&&std::abs(full.third-part.third)<1e-14,"Clipped authoring chapters retain the exact independent polynomial and its time-relevant derivatives");}
    }
    // Constant-speed analytic wavy circle: the crest's required normal load
    // is negative while lateral acceleration stays finite. Banking must retain
    // airtime, rather than picking the opposite physical force branch.
    Design banking;banking.track.authoredGeometry=true;constexpr int ringSamples=1600;
    for(int i=0;i<=ringSamples;++i){const double u=2*pi*i/ringSamples,R=80,H=50;
        const auto q=arcJet({R*std::sin(u),R*(1-std::cos(u)),H*(1-std::cos(2*u))},
            {R*std::cos(u),R*std::sin(u),2*H*std::sin(2*u)},
            {-R*std::sin(u),R*std::cos(u),4*H*std::cos(2*u)},
            {-R*std::cos(u),-R*std::sin(u),-8*H*std::sin(2*u)},
            {R*std::sin(u),-R*std::cos(u),-16*H*std::cos(2*u)});
        const auto up=unit(Vec3{0,0,1}-q.tangent*q.tangent.z);
        banking.track.knots.push_back({q.position,q.tangent,q.curvature,up,0,Element::Turn,q.third,q.fourth});}
    banking.track.knots.back()=banking.track.knots.front();banking.track.rebuild();
    std::vector<Frame> ringFrames;
    for(size_t i=0;i<banking.track.knots.size();++i){Frame f;f.distance=i<banking.track.spans.size()?banking.track.spans[i].start:banking.track.length;f.time=f.distance/25;f.speed=25;ringFrames.push_back(f);}
    authorBanking(banking,ringFrames);
    for(int crest:{ringSamples/4,3*ringSamples/4}){const double at=banking.track.spans[crest].start;const auto q=banking.track.sample(at);
        const auto dynamics=measureSeatDynamics(banking.track,at,25,0,0,0);
        check(q.up.z>0&&dynamics.force.vertical<-.6&&std::abs(dynamics.force.lateral)<.6,"A turning airtime crest stays upright and genuinely negative-G without an antipodal bank flip");}

    Design hybrid;hybrid.track.closed=false;MotionBuilder builder(hybrid);builder.line(100,Element::Return,"prefix");
    FvdRequest authored;authored.position={};authored.speed=35;authored.controls={{0,1,0,0},{1,2,0,0},{2,2,0,0},{3,1,0,0}};authored.twists={{.2,2.8,.6}};
    const auto source=designFvdSection(authored);check(source.assessment.passed,"Independent physical-roll source builds");
    const auto owned=builder.force(source,authored,Element::Turn,"authored-roll",builder.cursor.position);
    const auto atEnd=directionAngles(builder.cursor);
    builder.programme(polynomialMotion(builder.cursor,anglePolynomial(atEnd[0],{},100),anglePolynomial(atEnd[1],atEnd[1],100),100),Element::Return,"release");hybrid.track.rebuild();
    const auto sourceKnots=hybrid.track.knots;std::vector<Frame> hybridFrames;
    for(size_t i=0;i<hybrid.track.knots.size();++i){Frame f;f.distance=i<hybrid.track.spans.size()?hybrid.track.spans[i].start:hybrid.track.length;f.time=f.distance/35;f.speed=35;hybridFrames.push_back(f);}
    authorBanking(hybrid,hybridFrames);
    for(size_t i=owned.first;i<=owned.second;++i){const auto q=sampleSpanKinematics(hybrid.track,i,0);const auto& expected=sourceKnots[i];
        close(q.sample.up,expected.up,1e-10);close(q.upS,expected.upFirst,1e-10);close(q.upSS,expected.upSecond,1e-10);close(q.upSSS,expected.upThird,1e-9);}
    for(size_t i=1;i<hybrid.track.spans.size();++i){const auto a=sampleSpanKinematics(hybrid.track,i-1,1),b=sampleSpanKinematics(hybrid.track,i,0);
        close(a.sample.up,b.sample.up,1e-9);close(a.upS,b.upS,1e-9);close(a.upSS,b.upSS,1e-8);close(a.upSSS,b.upSSS,1e-7);}
    check(norm(hybrid.track.knots.back().position-sourceKnots.back().position)<1e-12,"Banking never closes an open authoring fixture");

    // Transported reference crosses the nearest-angle branch after an inversion.
    // A single deliberate release must not acquire an extra full revolution.
    std::vector<double> upright(101),reference(101);
    for(size_t i=0;i<upright.size();++i){upright[i]=std::remainder(.03*i,2*pi);reference[i]=.03*i+2*pi*i/100;}
    const auto roll=continuousRollTarget(upright,reference,90,.65);
    const auto equivalent=continuousRollTarget(upright,reference,90,.65+2*pi);
    for(size_t i=1;i<roll.size();++i){
        check(std::abs((roll[i]-roll[i-1])-.03)<1e-12,"Commanded roll stays continuous across the branch cut");
        check(std::abs(roll[i]-equivalent[i])<1e-12,"Equivalent physical bank requests cannot add an extra spin");
    }
    std::cout<<"PASS "<<checks<<" analytic arc, live endpoint jets, independent derivatives, interior shape and time replay checks\n";
}catch(const std::exception& e){std::cerr<<"FAIL: "<<e.what()<<'\n';return 1;}}
