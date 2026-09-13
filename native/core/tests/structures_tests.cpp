#include "coaster/coaster.hpp"
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
using namespace coaster;
namespace fs=std::filesystem;
static int checks=0;
static void check(bool b,const std::string& s){++checks;if(!b)throw std::runtime_error(s);}
static bool code(const ValidationReport& r,const char* c){for(const auto& e:r.errors)if(e.code==c)return true;return false;}
static std::string bytes(const fs::path& p){std::ifstream f(p,std::ios::binary);return {std::istreambuf_iterator<char>(f),{}};}
static uint64_t hash(const std::string& s){uint64_t h=14695981039346656037ull;for(unsigned char c:s){h^=c;h*=1099511628211ull;}return h;}
static void write(const fs::path& p,const std::string& s){std::ofstream f(p,std::ios::binary);f<<"COASTER 5 "<<s.size()<<' '<<hash(s)<<'\n'<<s;}
int main(){try{
    const auto out=fs::current_path()/"structure-test-output";fs::create_directories(out);
    GenerationRequest req;req.targets.requireIntensity=false;Design d=generate(req);check(d.accepted(),"Combined candidate accepted");
    check(d.station.enabled&&!d.station.boxes.empty(),"New candidate owns station geometry");check(validateDesignStructures(d).valid(),"Every new canonical structure validated");
    for(const auto& s:d.supports){check(!s.members.empty(),"New support explicit");check(!supportStationCollision(s,d.station),"Every member clears the station");}
    auto bad=d;bad.station={};check(code(validateDesignStructures(bad),"REQUIRED_STRUCTURE"),"New provenance cannot omit station");
    bad=d;bad.supports[0].members.clear();check(code(validateDesignStructures(bad),"REQUIRED_STRUCTURE"),"New provenance cannot omit member geometry");
    bad=d;bad.generationVersion="unknown";check(code(validateDesignStructures(bad),"GENERATOR_VERSION"),"Unknown provenance never gains legacy exemption");
    for(const auto* identity:{"0.2.1","0.3.0-pacing.2","0.4.0-foundation.1","0.4.0-foundation.2-work"}){
        auto old=d;old.generationVersion=identity;check(code(validateDesignStructures(old),"GENERATOR_VERSION"),"Old geometry semantics are never reinterpreted under new runtime");
        old.station={};old.supports.front().members.clear();check(code(validateDesignStructures(old),"GENERATOR_VERSION"),"Old identity cannot gain absence exemptions");
    }
    const auto good=out/"accepted.coaster",malformed=out/"malformed.coaster";std::string error;
    check(saveDesign(d,good.string(),error),"Combined save: "+error);const auto saved=bytes(good),payload=saved.substr(saved.find('\n')+1);
    Design replay;check(loadDesign(good.string(),replay,error),"Combined replay: "+error);check(stationPayload(d.station)==stationPayload(replay.station),"Canonical station fields preserved exactly");check(reportJson(d)==reportJson(replay),"Replay recomputes identical physics/reference/dimensions");
    auto rejected=[&](const std::string& p,const std::string& expected="REQUIRED_STRUCTURE"){write(malformed,p);Design prior=d;check(!loadDesign(malformed.string(),prior,error),"Checksummed missing geometry rejected");check(error.find(expected)!=std::string::npos,"Missing geometry receives explicit completeness finding: "+error);check(reportJson(prior)==reportJson(d),"Rejected load preserves accepted design");};
    const auto ext=payload.find("EXTENSIONS ");check(ext!=std::string::npos,"Schema5 extension directory present");
    // Strip only the structure under test. A valid landscape remains mandatory
    // in the new exact version, so it must survive the missing-station fixture.
    std::istringstream directory(payload.substr(ext));std::string tag,terrainBlock;size_t extensionCount=0;
    directory>>tag>>extensionCount;check(tag=="EXTENSIONS", "Extension directory parsed");
    for(size_t i=0;i<extensionCount;++i){std::string name;int version=0;size_t length=0;directory>>name>>version>>length;check(directory.get()=='\n',"Extension terminator");std::string data(length,'\0');check(bool(directory.read(data.data(),std::streamsize(length))),"Complete extension payload");if(name=="TERRAIN_PROFILE")terrainBlock=name+" "+std::to_string(version)+" "+std::to_string(length)+"\n"+data;}
    check(!terrainBlock.empty(),"Exact saved terrain profile retained in structure fixtures");
    rejected(payload.substr(0,ext)+"EXTENSIONS 0\n","Missing required terrain profile");
    rejected(payload.substr(0,ext)+"EXTENSIONS 1\n"+terrainBlock);
    rejected(payload.substr(0,ext),"Missing COASTER5 extension directory");
    std::vector<std::string> rows;std::istringstream in(payload);std::string line;while(std::getline(in,line))rows.push_back(line);
    const size_t first=5+d.track.knots.size()+d.operations.size(),count=d.supports.front().members.size();
    rows[first]=rows[first].substr(0,rows[first].find_last_of(' '))+" 0";rows.erase(rows.begin()+first+1,rows.begin()+first+1+count);std::string missing;for(const auto& row:rows)missing+=row+'\n';rejected(missing);
    for(int problem=0;problem<2;++problem){bad=d;if(problem==0)bad.station={};else bad.supports[0].members.clear();check(!saveDesign(bad,good.string(),error),"Incomplete new geometry cannot save");check(bytes(good)==saved,"Rejected save preserves accepted bytes");}
    // Connected station crossbar clears the train and towers but cuts the spine.
    bad=d;const auto q=bad.track.sample(5);bad.station.boxes.push_back({q.position-q.up*.66,q.tangent,q.right,q.up,{1.,1.5,.03},StationRole::Post});
    check(validateStationDefinition(bad.station).valid(),"Adverse spine crossbar is connected canonical station geometry");
    check(code(validateDesignStructures(bad),"STATION_HARDWARE_CLEARANCE"),"Direct station/spine obstruction rejected");
    check(!saveDesign(bad,good.string(),error),"Station/spine overlap cannot overwrite accepted save");check(bytes(good)==saved,"Rejected crossbar save preserves exact bytes");
    const std::string adverse=stationPayload(bad.station);write(malformed,payload.substr(0,ext)+"EXTENSIONS 2\n"+terrainBlock+"STATION 1 "+std::to_string(adverse.size())+"\n"+adverse);
    Design retained=d;check(!loadDesign(malformed.string(),retained,error),"Checksummed station/spine overlap cannot load");check(error.find("STATION_HARDWARE_CLEARANCE")!=std::string::npos,"Load reports exact station hardware certification gap");check(reportJson(retained)==reportJson(d),"Crossbar load preserves last accepted ride");
    const auto platform=*std::find_if(d.station.boxes.begin(),d.station.boxes.end(),[](const StationBox& b){return b.role==StationRole::Platform;});
    Support member;member.members={{{}, {}, .18,.18,SupportMemberKind::Steel,true}};member.members[0].base=platform.center-platform.forward;member.members[0].top=platform.center+platform.forward;member.attachment=member.members[0].top;
    check(supportStationCollision(member,d.station),"Spine-contact flag never exempts station intersection");
    member.members[0].kind=SupportMemberKind::Footing;member.members[0].spineContact=false;member.members[0].radiusBase=member.members[0].radiusTop=2;member.members[0].base=platform.center+platform.up*(platform.half.z+1.9)-platform.forward;member.members[0].top=member.members[0].base+platform.forward*2;
    check(supportStationCollision(member,d.station),"Full footing radius participates in station clearance");
    member.members[0].base=member.members[0].base+platform.up*20;member.members[0].top=member.members[0].top+platform.up*20;check(!supportStationCollision(member,d.station),"Distant canonical member stays clear");
    member.members.clear();member.base=platform.center-platform.forward;member.top=platform.center+platform.forward;check(supportStationCollision(member,d.station),"Legacy physical member uses same mutual clearance check");
    bool cancelled=false;try{supportStationCollision(member,d.station,[]{return true;});}catch(const std::exception& e){cancelled=std::string(e.what())=="CANCELLED";}check(cancelled,"Mutual clearance cancellation");check(code(validateDesignStructures(d,[]{return true;}),"CANCELLED"),"Design structure cancellation");
    const auto fixtures=fs::path(__FILE__).parent_path()/"fixtures";
    for(int i=1;i<=3;++i){const auto f=fixtures/("SYNTHETIC-legacy-v3-reference-"+std::to_string(i)+".coaster");const auto source=bytes(f);check(!source.empty(),"Historical fixture exists");Design retained=d;check(!loadDesign(f.string(),retained,error),"Old schema is explicitly unsupported");check(reportJson(retained)==reportJson(d),"Unsupported old schema leaves current accepted ride intact");check(bytes(f)==source,"Original historical fixture unchanged");}
    std::cout<<"PASS "<<checks<<" combined station/member completeness, mutual clearance, schema5 stripping, exact replay and unsupported-old-schema checks\n";return 0;
}catch(const std::exception& e){std::cerr<<"FAIL after "<<checks<<": "<<e.what()<<'\n';return 1;}}
