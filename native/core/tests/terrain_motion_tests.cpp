#include "../src/terrain_motion.hpp"
#include <iomanip>
#include <iostream>
using namespace coaster;
namespace {
int checks=0;
void check(bool condition,const char* message){++checks;if(!condition)throw std::runtime_error(message);}
double slope(const detail::TerrainTransfer& shape,double x){const double h=shape.length*1e-5;return (shape.height(x+h)-shape.height(x-h))/(2*h);}
double second(const detail::TerrainTransfer& shape,double x){const double h=shape.length*1e-5;return (shape.height(x+h)-2*shape.height(x)+shape.height(x-h))/(h*h);}
// Independent RK4 in horizontal distance, differentiating only sampled height.
// Powered reference has a finite .5 second linear force onset and a real speed
// controller. The planning bound is allowed to overestimate this trajectory.
detail::TerrainMotionAssessment reference(const detail::TerrainTransfer& shape,const detail::TerrainMotion& intent,double maximumStep=.125){
    double w=intent.entrySpeed*intent.entrySpeed,time=0;detail::TerrainMotionAssessment result;
    const int count=int(std::ceil(shape.length/maximumStep));const double dx=shape.length/count;
    auto rate=[&](double x,double energy,double t){double grade=slope(shape,x),arc=std::hypot(1.,grade),v=std::sqrt(std::max(.001,energy));
        double drive=intent.driveTargetSpeed>v?intent.driveAcceleration*std::clamp(t/.5,0.,1.):0;
        return std::pair{-2*gravity*grade+2*(drive-intent.rollingAcceleration-intent.dragAccelerationCoefficient*energy)*arc,arc/v};};
    for(int i=0;i<count;++i){double x=i*dx;auto a=rate(x,w,time),b=rate(x+dx*.5,w+a.first*dx*.5,time+a.second*dx*.5),c=rate(x+dx*.5,w+b.first*dx*.5,time+b.second*dx*.5),d=rate(x+dx,w+c.first*dx,time+c.second*dx);
        w+=dx*(a.first+2*b.first+2*c.first+d.first)/6;time+=dx*(a.second+2*b.second+2*c.second+d.second)/6;
        if(w<=0){result.positiveEnergyBound=false;return result;}
        double grade=slope(shape,x+dx),normal=1/std::hypot(1.,grade)+w*second(shape,x+dx)/(gravity*std::pow(1+grade*grade,1.5));
        result.minimumNormalG=std::min(result.minimumNormalG,normal);result.maximumNormalG=std::max(result.maximumNormalG,normal);
    }
    result.exitSpeedBound=std::sqrt(w);return result;
}
}
int main(){try{
    detail::TerrainMotion intent;intent.rollingAcceleration=gravity*.002;intent.dragAccelerationCoefficient=.5*1.225*2.4/9000;intent.minimumNormalG=-2.8;intent.maximumNormalG=6;
    for(double drop:{20.,80.,200.,250.})for(double entry:{40.,59.714756,75.}){
        intent.entrySpeed=entry;double length=detail::minimumTerrainMotionLength(-drop,2,intent);detail::TerrainTransfer shape{0,-drop,length};
        auto result=detail::assessTerrainMotion(shape,intent),fine=detail::assessTerrainMotion(shape,intent,{},2048),actual=reference(shape,intent);
        check(detail::terrainMotionFits(result,intent),"Returned minimum obeys the signed source intent");
        check(actual.positiveEnergyBound,"Independent loss-aware RK4 reaches the descent exit");
        check(std::abs(actual.exitSpeedBound-result.exitSpeedBound)<.00005,"Terrain energy agrees with independent height-differentiated RK4");
        check(std::abs(actual.minimumNormalG-result.minimumNormalG)<.00005&&std::abs(actual.maximumNormalG-result.maximumNormalG)<.00008,"Local signed normal load agrees with independent geometry and RK4");
        check(std::abs(fine.exitSpeedBound-result.exitSpeedBound)<.00005,"Point-energy bound converges under step halving");
        check(!detail::terrainMotionFits(detail::assessTerrainMotion({0,-drop,length*.999},intent),intent),"A shortened force-limited descent exceeds its unchanged force intent");
    }
    intent.entrySpeed=59.714756;intent.minimumNormalG=-1;intent.maximumNormalG=3.5;
    const double returnLength=detail::minimumTerrainMotionLength(-200,2,intent);
    const auto returnIntent=detail::assessTerrainMotion({0,-200,returnLength},intent),returnActual=reference({0,-200,returnLength},intent);
    check(detail::terrainMotionFits(returnIntent,intent),"Ordinary descent follows its selected intent instead of the acceptance ceiling");
    check(returnActual.positiveEnergyBound&&std::abs(returnActual.maximumNormalG-returnIntent.maximumNormalG)<.0001,"Ordinary descent intent agrees with independent physical replay");
    const auto raised=detail::assessTerrainMotion({0,0,400},intent);
    check(raised.exitSpeedBound<intent.entrySpeed,"An ordinary level transfer retains real rolling and drag losses");
    // Incoming overspeed remains in the powered ascent; it is not set to41.67.
    intent.entrySpeed=75;intent.minimumNormalG=0;intent.maximumNormalG=2;intent.driveTargetSpeed=150/3.6;intent.driveAcceleration=12;
    double poweredLength=detail::minimumTerrainMotionLength(200,2,intent);
    auto powered=detail::assessTerrainMotion({0,200,poweredLength},intent),actual=reference({0,200,poweredLength},intent);
    check(detail::terrainMotionFits(powered,intent)&&actual.positiveEnergyBound,"Declared finite-force powered ascent has a valid load bound and independent traversal");
    check(actual.minimumNormalG>=powered.minimumNormalG-.002&&actual.maximumNormalG<=powered.maximumNormalG+.002,"Finite-ramp powered trajectory remains inside the local-force planning bound");
    check(actual.exitSpeedBound<=powered.exitSpeedBound+.002,"Powered exit bound does not understate the independently integrated trajectory");
    intent.entrySpeed=20;intent.minimumNormalG=-1;intent.maximumNormalG=3;intent.driveTargetSpeed=65;intent.driveAcceleration=3.5;
    auto finite=detail::assessTerrainMotion({0,0,10},intent);
    check(finite.exitSpeedBound<22&&finite.exitSpeedBound>20,"Ten metres of finite motor work cannot reset20 m/s to the65 m/s target");
    auto noPower=intent;noPower.driveTargetSpeed=noPower.driveAcceleration=0;
    check(!detail::assessTerrainMotion({0,200,800},noPower).positiveEnergyBound,"Insufficient source energy is not replaced by phantom propulsion");
    bool cancelled=false;try{detail::minimumTerrainMotionLength(-200,2,intent,[]{return true;});}catch(const std::runtime_error& e){cancelled=std::string(e.what())=="CANCELLED";}check(cancelled,"Terrain energy screening propagates cancellation");
    intent.entrySpeed=59.714756;intent.driveTargetSpeed=intent.driveAcceleration=0;intent.minimumNormalG=-2.8;intent.maximumNormalG=6;
    const double active=detail::minimumTerrainMotionLength(-200,2,intent);
    const detail::TerrainTransfer window{200,0,active+350,150,active};
    const auto windowBound=detail::assessTerrainMotion(window,intent),windowActual=reference(window,intent);
    check(detail::terrainMotionFits(windowBound,intent)&&windowActual.positiveEnergyBound,"A certified source window and its level extensions retain physical traversal");
    check(std::abs(windowBound.exitSpeedBound-windowActual.exitSpeedBound)<.0001,"Active-window energy includes the real passive approach and exit losses");
    int polls=0;cancelled=false;
    try{detail::assessTerrainMotion(window,intent,[&]{return ++polls==3;});}catch(const std::runtime_error& e){cancelled=std::string(e.what())=="CANCELLED";}
    check(cancelled,"Source motion assessment remains cancellable while traversing its active interval");
    std::cout<<std::setprecision(12)<<"PASS "<<checks<<" terrain motion checks; powered200m ascent at75m/s into41.6667 target needs "<<poweredLength<<"m, bounded G "<<powered.minimumNormalG<<".."<<powered.maximumNormalG<<", actual G "<<actual.minimumNormalG<<".."<<actual.maximumNormalG<<"; ordinary200m descent at59.714756m/s needs "<<returnLength<<"m, G "<<returnIntent.minimumNormalG<<".."<<returnIntent.maximumNormalG<<"\n";
    return 0;
}catch(const std::exception& e){std::cerr<<"FAIL after "<<checks<<": "<<e.what()<<'\n';return 1;}}
