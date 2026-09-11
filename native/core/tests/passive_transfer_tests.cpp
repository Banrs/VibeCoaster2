#include "../src/passive_transfer.hpp"
#include <iostream>
#include <stdexcept>
using namespace coaster;
namespace {
int checks=0;
void check(bool okay,const char* message){++checks;if(!okay)throw std::runtime_error(message);}
void near(double a,double b,double tolerance,const char* message){check(std::isfinite(a)&&std::abs(a-b)<=tolerance,message);}
double speedAt(const SimulationResult& sim,double s){auto right=std::lower_bound(sim.frames.begin(),sim.frames.end(),s,[](const Frame& f,double x){return f.distance<x;});check(right!=sim.frames.begin()&&right!=sim.frames.end(),"Replay covers transfer boundary");auto left=right-1;double u=(s-left->distance)/(right->distance-left->distance);return left->speed+u*(right->speed-left->speed);}
Track path(bool shaped){std::vector<AuthoredPoint> points;
    for(int i=0;i<=1500;++i){double x=i,u=std::clamp((x-350)/600.,0.,1.);double z=50+(shaped?70*std::pow(std::sin(pi*u),4):0);
        double lateral=shaped?80*smooth(std::clamp((x-400)/500.,0.,1.)):0;
        points.push_back({{x,lateral,z},0,Element::Return,{0,0,1}});}
    return compile(points,false);
}
// Independent spatial RK4 integrates the actual mean train grade, rather than
// the affine helper's piecewise potential changes.
detail::PassiveTransferEstimate reference(const Track& track,const TrainConfig& train,double begin,double end,double speed){
    const double drag=train.airDensity*train.dragCdA/(train.cars*train.carMass),rolling=gravity*train.rollingResistance;
    auto derivative=[&](double s,double w){double grade=0;for(int car=0;car<train.cars;++car)
        grade+=track.sample(s+(train.cars-1)*train.spacing*.5-car*train.spacing).tangent.z;
        return -2*(gravity*grade/train.cars+rolling)-drag*w;};
    const int steps=int(std::ceil((end-begin)/.25));const double ds=(end-begin)/steps;double w=speed*speed;
    for(int i=0;i<steps;++i){const double s=begin+i*ds,k1=derivative(s,w),k2=derivative(s+ds*.5,w+k1*ds*.5),k3=derivative(s+ds*.5,w+k2*ds*.5),k4=derivative(s+ds,w+k3*ds);
        w+=ds*(k1+2*k2+2*k3+k4)/6;if(w<=0)return {};}
    return {true,std::sqrt(w)};
}
}
int main(){try{
    TrainConfig train;auto flat=path(false);double begin=350,end=1000,v=55;
    double drag=train.airDensity*train.dragCdA/(train.cars*train.carMass),rolling=gravity*train.rollingResistance;
    double expected=std::sqrt((v*v+2*rolling/drag)*std::exp(-drag*(end-begin))-2*rolling/drag);
    const auto flatTransfer=detail::makePassiveTransfer(flat,train,begin,end);
    auto predicted=flatTransfer.forward(v);check(predicted.reached,"Flat transfer has sufficient energy");near(predicted.speed,expected,1e-9,"Flat drag and rolling agree with independent closed-form solution");
    auto inlet=flatTransfer.inverse(expected);check(inlet.reached,"Flat inverse is physically reachable");near(inlet.speed,v,1e-10,"Direct inverse recovers the closed-form inlet speed");
    near(flatTransfer.retention,std::exp(-drag*(end-begin)),1e-13,"Drag attenuation is independent of inlet energy");
    near(flatTransfer.offsetSpeedSquared,-2*rolling/drag*(1-std::exp(-drag*(end-begin))),1e-10,"Affine source agrees with closed-form rolling work");
    near(flatTransfer.minimumEntrySpeedSquared,2*rolling/drag*(std::exp(drag*(end-begin))-1),1e-10,"Flat endpoint stall boundary agrees with independent dissipative work");
    check(!flatTransfer.inverse(0).reached,"Positive inlet energy that reaches exactly zero at the endpoint is still a stall");
    check(!flatTransfer.forward(std::sqrt(flatTransfer.minimumEntrySpeedSquared-.001)).reached&&flatTransfer.forward(std::sqrt(flatTransfer.minimumEntrySpeedSquared+.001)).reached,"Both sides of a loss-only endpoint barrier are distinguished");
    auto hill=path(true);auto lossless=train;lossless.dragCdA=0;lossless.rollingResistance=0;
    auto meanHeight=[&](double s){double z=0;for(int c=0;c<train.cars;++c)z+=hill.sample(s+(train.cars-1)*train.spacing*.5-c*train.spacing).position.z;return z/train.cars;};
    const auto climb=detail::makePassiveTransfer(hill,lossless,350,650);
    predicted=climb.forward(55);check(predicted.reached,"Gravity transfer reaches the hill");near(predicted.speed,std::sqrt(55*55-2*gravity*(meanHeight(650)-meanHeight(350))),1e-9,"Zero-loss transfer conserves whole-train mean potential energy");
    inlet=climb.inverse(predicted.speed);check(inlet.reached,"Zero-loss inverse reaches the crest");near(inlet.speed,55,1e-10,"Inverse conserves finite-train potential energy");
    const auto crestAndReturn=detail::makePassiveTransfer(hill,lossless,350,1100);
    check(!crestAndReturn.forward(10).reached,"Positive endpoint energy cannot conceal an interior hill stall");
    inlet=crestAndReturn.inverse(10);check(!inlet.reached,"An exact low exit target is infeasible across the earlier crest");
    double barrier=0;for(double s=350;s<=1100;s+=.25)barrier=std::max(barrier,2*gravity*(meanHeight(s)-meanHeight(350)));
    near(crestAndReturn.minimumEntrySpeedSquared,barrier,.01,"Interior traversal barrier agrees with independent dense mean-potential sampling");
    check(!crestAndReturn.forward(std::sqrt(barrier-.02)).reached&&crestAndReturn.forward(std::sqrt(barrier+.02)).reached,"Speeds on either side of the interior energy barrier produce distinct outcomes");
    const auto descent=detail::makePassiveTransfer(hill,lossless,650,1100);
    check(!descent.inverse(5).reached,"Gravity-only descent cannot achieve a target below its unavoidable gained energy");
    auto flatLossless=detail::makePassiveTransfer(flat,lossless,350,1100);
    check(!flatLossless.inverse(0).reached,"Zero endpoint speed is an endpoint stall over a nonempty passive interval");
    near(flatLossless.inverse(20).speed,20,1e-12,"Flat zero-loss transport has an identity speed map");
    auto empty=detail::makePassiveTransfer(flat,train,400,400);check(empty.forward(0).reached&&empty.inverse(0).reached,"An empty interval requires no traversal energy");
    near(empty.inverse(20).speed,20,1e-12,"Empty interval inverse preserves speed");
    for(double finish:{600.,750.,1050.}){
        const auto transfer=detail::makePassiveTransfer(hill,train,begin,finish),fine=detail::makePassiveTransfer(hill,train,begin,finish,{},1);
        for(double speed:{30.,45.,55.,70.}){const auto actual=reference(hill,train,begin,finish,speed),estimate=transfer.forward(speed);
            check(actual.reached==estimate.reached,"Affine forward map agrees with independent RK4 reachability");
            if(!actual.reached)continue;
            near(estimate.speed,actual.speed,.00025,"Affine forward speed agrees with independent RK4 across hill phases");
            const auto needed=transfer.inverse(actual.speed),refined=fine.inverse(actual.speed);
            check(needed.reached&&refined.reached,"Exact exit target remains reachable under inverse and half step");
            near(needed.speed,speed,.0002,"Direct inverse recovers inlet from independent hill transport");
            near(needed.speed,refined.speed,.0002,"Inverse required inlet converges under spatial step halving");
        }
    }
    Operation launch{0,325,DriveKind::Launch,55,train.carMass*8,train.carMass*800,.5};launch.exitFadeMeters=20;
    auto sim=simulate(hill,{launch},train,1./960);check(sim.completed&&sim.report.valid(),"Independent launched then passive finite train completes");
    for(double finish:{600.,750.,1050.}){double initial=speedAt(sim,begin),observed=speedAt(sim,finish);
        auto transfer=detail::makePassiveTransfer(hill,train,begin,finish);
        auto coarse=detail::estimatePassiveTransfer(hill,train,begin,finish,initial),fine=detail::estimatePassiveTransfer(hill,train,begin,finish,initial,{},1);
        check(coarse.reached&&fine.reached,"Passive observer reaches ascending, descending and post-hill boundaries");
        near(coarse.speed,observed,.003,"Passive energy planning agrees with actual finite-train simulation");
        near(coarse.speed,fine.speed,.0002,"Spatial step refinement keeps transfer error small relative to authoring tolerance");
        auto required=transfer.inverse(observed);check(required.reached,"Measured finite-train exit can be inverted without a search");near(required.speed,initial,.003,"Direct inverse agrees with independently simulated inlet");
        std::cout<<"finish="<<finish<<" actual="<<observed<<" predicted="<<coarse.speed<<" difference="<<coarse.speed-observed<<'\n';}
    auto stopped=detail::estimatePassiveTransfer(hill,train,350,650,8);check(!stopped.reached,"Insufficient uphill energy is reported instead of clamped into a successful transfer");
    int polls=0;bool cancelled=false;try{detail::estimatePassiveTransfer(hill,train,350,1000,55,[&]{return ++polls>=3;});}catch(const std::runtime_error& e){cancelled=std::string(e.what())=="CANCELLED";}check(cancelled,"Transfer planning remains cancellable");
    polls=0;detail::makePassiveTransfer(flat,train,350,1000,[&]{++polls;return false;},.25);check(polls==2601,"Transfer integration is bounded by span divided by its explicit step");
    cancelled=false;try{detail::makePassiveTransfer(flat,train,350,350,[]{return true;});}catch(const std::runtime_error& e){cancelled=std::string(e.what())=="CANCELLED";}check(cancelled,"Already-cancelled work does not return a successful empty map");
    for(double step:{0.,.249,2.001,double(INFINITY)}){bool rejected=false;try{detail::makePassiveTransfer(flat,train,350,1000,{},step);}catch(const std::invalid_argument&){rejected=true;}check(rejected,"Invalid integration steps cannot evade the bounded work contract");}
    for(double speed:{-1.,double(INFINITY),1e308}){bool forwardRejected=false,inverseRejected=false;try{flatTransfer.forward(speed);}catch(const std::invalid_argument&){forwardRejected=true;}try{flatTransfer.inverse(speed);}catch(const std::invalid_argument&){inverseRejected=true;}check(forwardRejected&&inverseRejected,"Both map directions reject invalid or overflowing speed inputs");}
    std::cout<<"PASS "<<checks<<" passive energy transfer checks\n";return 0;
}catch(const std::exception& e){std::cerr<<"FAIL "<<checks<<": "<<e.what()<<'\n';return 1;}}
