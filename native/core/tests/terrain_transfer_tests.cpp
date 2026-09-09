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
    // Actual canyon hill-to-ascent boundary: the terrain baseline supplies a
    // 25-degree climb. Replacing it with a level port made the later join
    // introduce an artificial negative-G crest.
    std::vector<double> jetDistance(251),jetFloor(251,0);
    for(size_t i=0;i<jetDistance.size();++i)jetDistance[i]=497.220947182507*i/250;
    jetFloor.back()=219.;
    const std::array<double,3> initial{.47470389747531,.000727294335657282,-3.94367146250232e-6},final{};
    auto joined=detail::fitJetTerrainTransfer(jetDistance,jetFloor,81.888994266125,219.98647787138,initial,final,2,.0058);
    check(std::abs((joined.height(.01)-joined.height(0))/.01-initial[0])<1e-5,"Nonlevel transfer preserves actual incoming grade instead of resetting level");
    check(joined.height(0)==81.888994266125&&joined.height(joined.length)==219.98647787138,"Jet-compatible transfer preserves both absolute heights");
    previous=joined.start;for(double at:jetDistance){double z=joined.height(at);check(z+1e-8>=previous&&z<=joined.finish+1e-8,"Matched height jets retain the monotone climb without a new hump");previous=z;}
    rejects([&]{detail::fitJetTerrainTransfer(jetDistance,jetFloor,joined.start,joined.finish,initial,final,.2,.0058);},"Infeasible port grade is rejected, not clamped");
    rejects([&]{detail::fitJetTerrainTransfer(jetDistance,jetFloor,joined.start,joined.finish,initial,final,2,.0001);},"Jet-compatible profiles still obey the authoring curvature budget");
    jetFloor[125]=230;rejects([&]{detail::fitJetTerrainTransfer(jetDistance,jetFloor,joined.start,joined.finish,initial,final,2,.0058);},"A buried jet-compatible transfer requires a different route");
    std::vector<AuthoredPoint> ports(33);
    for(size_t i=0;i<ports.size();++i){double x=double(i);ports[i].position={x,0,i<=8?.25*x:i>=24?10:2+8*std::sin((x-8)*pi/16)};}
    const auto incoming=detail::terrainPortHeightJet(ports,8,true),outgoing=detail::terrainPortHeightJet(ports,24,false);
    check(std::abs(incoming[0]-.25)<1e-10&&std::abs(incoming[1])<1e-10&&std::abs(incoming[2])<1e-10,"Incoming jets come from preserved track, excluding the replaced interior");
    check(std::abs(outgoing[0])+std::abs(outgoing[1])+std::abs(outgoing[2])<1e-10,"Flattened station outgoing jet ignores the old descent height jump");
    std::cout<<"PASS "<<checks<<" terrain transfer checks\n";
}catch(const std::exception& e){std::cerr<<"FAIL "<<checks<<": "<<e.what()<<'\n';return 1;}}
