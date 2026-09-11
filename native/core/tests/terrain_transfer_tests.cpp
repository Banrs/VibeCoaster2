#include "../src/terrain_transfer.hpp"
#include <iostream>
#include <stdexcept>
using namespace coaster;
namespace {
int checks=0;
void check(bool okay,const char* message){++checks;if(!okay)throw std::runtime_error(message);}
template<class F>void rejects(F action,const char* message){bool failed=false;try{action();}catch(const std::runtime_error&){failed=true;}check(failed,message);}
}
int main(){try{
    for(double rise:{1.,20.,80.,207.0828558894449,250.})for(double grade:{.25,1.,2.,10.})for(double curvature:{.0001,.001,.0056486304,.02}){
        const double length=detail::minimumTerrainWindowLength(rise,grade,curvature);
        const auto fitted=detail::fitTerrainWindow({0,length*.5,length},{-1,-1,-1},rise,0,grade,curvature);
        double measuredGrade=0,measuredCurvature=0;
        for(int i=1;i<4000;++i){
            const double at=length*i/4000,step=length/100000;
            const double before=fitted.height(at-step),middle=fitted.height(at),after=fitted.height(at+step);
            const double slope=(after-before)/(2*step),second=(after-2*middle+before)/(step*step);
            measuredGrade=std::max(measuredGrade,std::abs(slope));
            measuredCurvature=std::max(measuredCurvature,std::abs(second)/std::pow(1+slope*slope,1.5));
        }
        check(measuredGrade<=grade*1.00001&&measuredCurvature<=curvature*1.00002,"Shared minimum span respects independently measured grade and curvature");
        check(std::max(measuredGrade/grade,measuredCurvature/curvature)>.9999,"At least one physical span constraint is active");
        rejects([&]{detail::fitTerrainWindow({0,length*.4995,length*.999},{-1,-1,-1},rise,0,grade,curvature);},"Shortening a minimum-span transition violates a physical requirement");
    }
    check(detail::minimumTerrainWindowLength(0,2,.005)==0,"Zero rise has no invented minimum terrain length");
    double previous=0;bool cancelled=false,invalidInput=false;
    // A late cliff followed by a long station return must not stretch the
    // descent across the entire return or remain on a high artificial shelf.
    std::vector<double> terminalDistance(1001),terminalFloor(1001);
    for(size_t i=0;i<terminalDistance.size();++i){terminalDistance[i]=i*2.;terminalFloor[i]=i<400?180:10;}
    auto terminal=detail::fitTerrainWindow(terminalDistance,terminalFloor,220,10,1,.003);
    check(terminal.activeStart>0&&terminal.activeStart+terminal.activeLength<1500,"Late cliff descent finishes before the distant station return");
    previous=terminal.start;
    for(size_t i=0;i<terminalDistance.size();++i){double z=terminal.height(terminalDistance[i]);check(z+1e-7>=terminalFloor[i],"Active window clears the complete sampled cliff");check(z<=previous&&z>=terminal.finish,"Terminal descent remains monotone within fixed port heights");previous=z;}
    const double windowEnd=terminal.activeStart+terminal.activeLength;
    check(terminal.height(terminal.activeStart)==220&&terminal.height(windowEnd)==10,"Active window ports retain exact heights");
    check(terminal.height(terminal.activeStart-1)==220&&terminal.height(windowEnd+1)==10,"Outside the active window the level extensions are exact");
    const double h=terminal.activeLength*.001;
    const double entryOne=220-terminal.height(terminal.activeStart+h),entryTwo=220-terminal.height(terminal.activeStart+2*h);
    const double exitOne=terminal.height(windowEnd-h)-10,exitTwo=terminal.height(windowEnd-2*h)-10;
    check(entryTwo/entryOne>15.8&&entryTwo/entryOne<16.1&&exitTwo/exitOne>15.8&&exitTwo/exitOne<16.1,"Both interior window joins have fourth-order displacement and C3 level ports");
    double measuredGrade=0,measuredCurvature=0;
    for(int i=1;i<2000;++i){
        double at=terminal.activeStart+terminal.activeLength*i/2000,step=terminal.activeLength/100000;
        double before=terminal.height(at-step),middle=terminal.height(at),after=terminal.height(at+step);
        double slope=(after-before)/(2*step),second=(after-2*middle+before)/(step*step);
        measuredGrade=std::max(measuredGrade,std::abs(slope));measuredCurvature=std::max(measuredCurvature,std::abs(second)/std::pow(1+slope*slope,1.5));
    }
    check(measuredGrade<=1.000001&&measuredCurvature<=.00300001,"Independent height differences respect grade and geometric curvature budgets");
    check(measuredCurvature>.00299,"Curvature-limited descent does not silently use a weaker authoring budget");
    for(size_t i=1001;i<=1500;++i){terminalDistance.push_back(i*2.);terminalFloor.push_back(10);}
    auto extended=detail::fitTerrainWindow(terminalDistance,terminalFloor,220,10,1,.003);
    check(std::abs(extended.activeStart-terminal.activeStart)<1e-5&&std::abs(extended.activeLength-terminal.activeLength)<1e-5,"Extra level station distance does not delay the same terrain descent");
    std::vector<double> reflectedFloor(terminalFloor.rbegin(),terminalFloor.rend());
    auto reflected=detail::fitTerrainWindow(terminalDistance,reflectedFloor,10,220,1,.003);
    for(double at:terminalDistance)check(std::abs(reflected.height(at)-extended.height(extended.length-at))<1e-8,
        "A climb follows the reflected terrain without forcing its ascent to start at the remote inlet");
    auto terminalLevel=detail::fitTerrainWindow({0,100,200},{9,9,9},10,10,1,.003);
    check(terminalLevel.height(100)==10&&terminalLevel.activeLength==0,"Equal clear terminal ports remain level without a fictitious descent");
    rejects([&]{detail::fitTerrainWindow({0,100,200},{9,11,9},10,10,1,.003);},"A level terminal transfer cannot cross a buried interior");
    rejects([&]{detail::fitTerrainWindow({0,800,1000},{180,220,10},220,10,1,.003);},"A cliff leaving insufficient physical descent distance rejects the site");
    rejects([&]{detail::fitTerrainWindow({0,100,200},{221,0,0},220,10,1,.003);},"A buried terminal fixed port remains infeasible");
    rejects([&]{detail::fitTerrainWindow({0,100,200},{0,221,0},220,10,1,.003);},"A terminal ridge above the start cannot manufacture an extra summit");
    int cancellationPolls=0;cancelled=false;
    try{detail::fitTerrainWindow(terminalDistance,terminalFloor,220,10,1,.003,[&]{return ++cancellationPolls==30;});}catch(const detail::TerrainTransferInfeasible&){throw std::runtime_error("Terminal cancellation masqueraded as a site rejection");}catch(const std::runtime_error& e){cancelled=std::string(e.what())=="CANCELLED";}
    check(cancelled,"Terminal window search remains cancellable after reading the terrain envelope");
    for(int invalid=0;invalid<4;++invalid){
        invalidInput=false;
        try{detail::fitTerrainWindow(invalid==0?std::vector<double>{0,100,100}:std::vector<double>{0,100,200},
            invalid==1?std::vector<double>{0,std::numeric_limits<double>::infinity(),0}:std::vector<double>{0,0,0},220,invalid==2?std::numeric_limits<double>::quiet_NaN():10,invalid==3?0:1,.003);}
        catch(const std::invalid_argument&){invalidInput=true;}
        check(invalidInput,"Malformed terminal inputs are programming errors, not site rejections");
    }
    std::cout<<"PASS "<<checks<<" terrain transfer checks\n";
}catch(const std::exception& e){std::cerr<<"FAIL "<<checks<<": "<<e.what()<<'\n';return 1;}}
