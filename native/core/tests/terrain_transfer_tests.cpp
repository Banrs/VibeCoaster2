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
    std::vector<double> s(601),floor(601,0);for(size_t i=0;i<s.size();++i)s[i]=i*2.;
    auto ordinary=detail::fitTerrainTransfer(s,floor,10,220,2,.02);check(ordinary.timing==1,"Unobstructed transfer retains its S7 timing");
    // A cliff rises earlier than the original transfer. Its entire sampled
    // lower envelope must clear, without an added summit or reversed grade.
    for(size_t i=0;i<s.size();++i)floor[i]=i<160?0:180;
    auto climb=detail::fitTerrainTransfer(s,floor,10,220,2,.02);
    check(climb.timing>1,"Early escarpment advances the climb");
    double previous=10;
    for(size_t i=0;i<s.size();++i){double z=climb.height(s[i]);check(z+1e-7>=floor[i],"Whole sampled cliff envelope clears");check(z>=previous&&z<=220,"Climb remains monotone and inside its port heights");previous=z;}
    std::reverse(floor.begin(),floor.end());auto descent=detail::fitTerrainTransfer(s,floor,220,10,2,.02);
    for(size_t i=0;i<s.size();++i)check(std::abs(descent.height(s[i])-climb.height(1200-s[i]))<1e-8,"Late descent is the reflected early climb");
    check(climb.height(0)==10&&climb.height(1200)==220,"Both absolute port heights are exact");
    double a=climb.height(.1)-10,b=climb.height(.2)-10;
    check(a>0&&b/a>15.5&&b/a<16.5,"Endpoint displacement is fourth order, preserving zero first three jets");
    auto bad=floor;bad.front()=221;rejects([&]{detail::fitTerrainTransfer(s,bad,220,10,2,.02);},"A buried fixed port is infeasible");
    bad.assign(s.size(),0);bad[300]=221;rejects([&]{detail::fitTerrainTransfer(s,bad,10,220,2,.02);},"A ridge above both ports requires another route");
    bad.assign(s.size(),0);bad[1]=100;rejects([&]{detail::fitTerrainTransfer(s,bad,10,220,2,.02);},"Near-port deficit cannot create an unbounded remote hump");
    rejects([&]{detail::fitTerrainTransfer(s,floor,220,10,2,.00001);},"Curvature authoring budget rejects a compressed transfer");
    auto level=detail::fitTerrainTransfer(s,std::vector<double>(s.size(),9),10,10,2,.02);check(level.height(600)==10,"Level clear ports remain level");
    rejects([&]{detail::fitTerrainTransfer(s,std::vector<double>(s.size(),11),10,10,2,.02);},"Level buried ports cannot invent a hill");
    bool cancelled=false;try{detail::fitTerrainTransfer(s,floor,220,10,2,.02,[]{return true;});}catch(const std::runtime_error& e){cancelled=std::string(e.what())=="CANCELLED";}check(cancelled,"Transfer fit remains cancellable");
    bool siteFailure=false;
    try{detail::fitTerrainTransfer(s,std::vector<double>(s.size(),221),10,220,2,.02);}catch(const detail::TerrainTransferInfeasible&){siteFailure=true;}
    check(siteFailure,"Expected terrain infeasibility has a distinct site-selection type");
    bool invalidInput=false;
    try{detail::fitTerrainTransfer(s,floor,220,10,0,.02);}catch(const detail::TerrainTransferInfeasible&){throw std::runtime_error("Invalid input masqueraded as a site rejection");}catch(const std::invalid_argument&){invalidInput=true;}
    check(invalidInput,"Input errors cannot silently trigger another terrain site");
    cancelled=false;
    try{detail::fitTerrainTransfer(s,floor,220,10,2,.02,[]{return true;});}catch(const detail::TerrainTransferInfeasible&){throw std::runtime_error("Cancellation masqueraded as a site rejection");}catch(const std::runtime_error& e){cancelled=std::string(e.what())=="CANCELLED";}
    check(cancelled,"Site feasibility retries cannot consume cancellation");
    // A late cliff followed by a long station return must not stretch the
    // descent across the entire return or remain on a high artificial shelf.
    std::vector<double> terminalDistance(1001),terminalFloor(1001);
    for(size_t i=0;i<terminalDistance.size();++i){terminalDistance[i]=i*2.;terminalFloor[i]=i<400?180:10;}
    auto terminal=detail::fitTerrainWindow(terminalDistance,terminalFloor,220,10,1,.003);
    auto stretched=detail::fitTerrainTransfer(terminalDistance,terminalFloor,220,10,1,.003);
    check(terminal.activeStart>0&&terminal.activeStart+terminal.activeLength<1500,"Late cliff descent finishes before the distant station return");
    check(terminal.height(1200)+40<stretched.height(1200),"Active descent removes the full-domain high return shelf");
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
