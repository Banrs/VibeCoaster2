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
}
int main(){try{
    TrainConfig train;auto flat=path(false);double begin=350,end=1000,v=55;
    double drag=train.airDensity*train.dragCdA/(train.cars*train.carMass),rolling=gravity*train.rollingResistance;
    double expected=std::sqrt((v*v+2*rolling/drag)*std::exp(-drag*(end-begin))-2*rolling/drag);
    auto predicted=detail::estimatePassiveTransfer(flat,train,begin,end,v);check(predicted.reached,"Flat transfer has sufficient energy");near(predicted.speed,expected,1e-9,"Flat drag and rolling agree with independent closed-form solution");
    auto hill=path(true);auto lossless=train;lossless.dragCdA=0;lossless.rollingResistance=0;
    auto meanHeight=[&](double s){double z=0;for(int c=0;c<train.cars;++c)z+=hill.sample(s+(train.cars-1)*train.spacing*.5-c*train.spacing).position.z;return z/train.cars;};
    predicted=detail::estimatePassiveTransfer(hill,lossless,350,650,55);check(predicted.reached,"Gravity transfer reaches the hill");near(predicted.speed,std::sqrt(55*55-2*gravity*(meanHeight(650)-meanHeight(350))),1e-9,"Zero-loss transfer conserves whole-train mean potential energy");
    Operation launch{0,325,DriveKind::Launch,55,train.carMass*8,train.carMass*800,.5};launch.exitFadeMeters=20;
    auto sim=simulate(hill,{launch},train,1./960);check(sim.completed&&sim.report.valid(),"Independent launched then passive finite train completes");
    for(double finish:{600.,750.,1050.}){double initial=speedAt(sim,begin),observed=speedAt(sim,finish);
        auto coarse=detail::estimatePassiveTransfer(hill,train,begin,finish,initial),fine=detail::estimatePassiveTransfer(hill,train,begin,finish,initial,{},1);
        check(coarse.reached&&fine.reached,"Passive observer reaches ascending, descending and post-hill boundaries");
        near(coarse.speed,observed,.003,"Passive energy planning agrees with actual finite-train simulation");
        near(coarse.speed,fine.speed,.0002,"Spatial step refinement keeps transfer error small relative to authoring tolerance");
        std::cout<<"finish="<<finish<<" actual="<<observed<<" predicted="<<coarse.speed<<" difference="<<coarse.speed-observed<<'\n';}
    auto stopped=detail::estimatePassiveTransfer(hill,train,350,650,8);check(!stopped.reached,"Insufficient uphill energy is reported instead of clamped into a successful transfer");
    int polls=0;bool cancelled=false;try{detail::estimatePassiveTransfer(hill,train,350,1000,55,[&]{return ++polls>=3;});}catch(const std::runtime_error& e){cancelled=std::string(e.what())=="CANCELLED";}check(cancelled,"Transfer planning remains cancellable");
    std::cout<<"PASS "<<checks<<" passive energy transfer checks\n";return 0;
}catch(const std::exception& e){std::cerr<<"FAIL "<<checks<<": "<<e.what()<<'\n';return 1;}}
