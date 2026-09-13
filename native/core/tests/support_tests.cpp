#include "coaster/coaster.hpp"
#include "coaster/support_mesh.hpp"
#include "CoordinateContract.h"
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
using namespace coaster;
namespace fs=std::filesystem;
static int checks=0;
static void check(bool v,const std::string& m){++checks;if(!v)throw std::runtime_error(m);}
static std::string bytes(const fs::path& p){std::ifstream f(p,std::ios::binary);return {std::istreambuf_iterator<char>(f),{}};}
static void exact(Vec3 a,Vec3 b,const char* what){check(a.x==b.x&&a.y==b.y&&a.z==b.z,what);}
static uint64_t hash(const std::string& s){uint64_t h=14695981039346656037ull;for(unsigned char c:s){h^=c;h*=1099511628211ull;}return h;}
static std::vector<std::string> lines(const std::string& s){std::istringstream in(s.substr(s.find('\n')+1));std::vector<std::string> out;std::string l;while(std::getline(in,l))out.push_back(l);return out;}
static std::vector<std::string> tokens(const std::string& s){std::istringstream in(s);std::vector<std::string> v;std::string x;while(in>>x)v.push_back(x);return v;}
static std::string join(const std::vector<std::string>& v,const char* sep){std::string s;for(auto& x:v){if(!s.empty())s+=sep;s+=x;}return s;}
static void corrupt(const fs::path& p,std::vector<std::string> v,size_t row,size_t column,const std::string& value){auto fields=tokens(v[row]);fields.at(column)=value;v[row]=join(fields," ");std::string payload=join(v,"\n")+"\n";std::ofstream f(p,std::ios::binary);f<<"COASTER 5 "<<payload.size()<<' '<<hash(payload)<<'\n'<<payload;}
static bool code(const ValidationReport& r,const char* s){for(auto& f:r.errors)if(f.code==s)return true;return false;}
static void sameSupport(const Support& a,const Support& b){exact(a.base,b.base,"Support base exact");exact(a.top,b.top,"Support top exact");exact(a.attachment,b.attachment,"Support attachment exact");check(a.hasAttachment==b.hasAttachment&&a.trackDistance==b.trackDistance,"Support contact identity exact");check(a.members.size()==b.members.size(),"Member count exact");for(size_t i=0;i<a.members.size();++i){auto& x=a.members[i];auto& y=b.members[i];exact(x.base,y.base,"Member base exact");exact(x.top,y.top,"Member top exact");check(x.radiusBase==y.radiusBase&&x.radiusTop==y.radiusTop&&x.kind==y.kind&&x.spineContact==y.spineContact,"Member radii/kind/contact exact");}}
int main(int argc,char** argv){try{
    if(argc!=2)throw std::runtime_error("Pass historical fixture directory");fs::path fixtures=fs::absolute(argv[1]),root=fs::current_path(),out=root/"support-test-output";fs::create_directories(out);
    std::string error;int legacyCases=0,legacyRejected=0;
    for(const auto& relative:{"legacy-v021-hills.coaster","legacy-v021-canyon.coaster","legacy-v021.coaster","pacing-hills.coaster","pacing-canyon.coaster","pacing-flat.coaster"}){
        fs::path path=fixtures/relative;std::string original=bytes(path);check(!original.empty(),"Historical fixture exists");Design retained;retained.request.seed=98765;const auto before=reportJson(retained);
        check(!loadDesign(path.string(),retained,error),"Prior geometry schemas are explicitly unsupported");check(reportJson(retained)==before,"Unsupported load retains its destination");check(bytes(path)==original,"Historical fixture unchanged");++legacyRejected;
    }
    GenerationRequest baselineRequest;baselineRequest.seed=42;baselineRequest.terrain.kind=TerrainKind::Hills;baselineRequest.targets.requireIntensity=false;Design baseline=generate(baselineRequest);check(baseline.accepted(),"Current geometry baseline accepted");
    auto d=baseline;d.station=buildStation(d.track,d.request.terrain,d.request.train);buildSupportLayout(d);d.generationVersion=generatorVersion;d.report=validateGeometry(d.track,d.request.terrain,d.request.limits,d.request.train,d.supports);d.simulation=simulate(d.track,d.operations,d.request.train,d.request.simulationStep);evaluateTargets(d);verifyConvergence(d);check(d.accepted(),"Explicit tower replacement accepted");check(validateDesignStructures(d).valid(),"Explicit replacement station and supports mutually clear");check(d.track.knots.size()==baseline.track.knots.size(),"Support replacement preserves knot count");
    for(size_t i=0;i<d.track.knots.size();++i){auto& a=d.track.knots[i];auto& b=baseline.track.knots[i];exact(a.position,b.position,"Track position unchanged");exact(a.tangent,b.tangent,"Track tangent unchanged");exact(a.curvature,b.curvature,"Track curvature unchanged");exact(a.up,b.up,"Track up unchanged");check(a.bank==b.bank&&a.element==b.element,"Track bank/element unchanged");}
    check(d.simulation.frames.size()==baseline.simulation.frames.size(),"Physics sample count unchanged");for(size_t i=0;i<d.simulation.frames.size();++i){auto& a=d.simulation.frames[i];auto& b=baseline.simulation.frames[i];check(a.time==b.time&&a.distance==b.distance&&a.speed==b.speed,"Physics timing/distance/speed unchanged");for(int seat=0;seat<3;++seat)check(a.seats[seat].vertical==b.seats[seat].vertical&&a.seats[seat].lateral==b.seats[seat].lateral&&a.seats[seat].longitudinal==b.seats[seat].longitudinal,"Measured rider forces unchanged");}
    size_t members=0,footings=0;double tallest=0,maxRadius=0;
    for(auto& support:d.supports){check(!support.members.empty(),"Every new support explicit");check(validateSupportMembers(support,d.request.terrain).valid(),"Member shape/terrain/connectivity valid");tallest=std::max(tallest,support.top.z-support.base.z);
        for(auto& member:support.members){++members;if(member.kind==SupportMemberKind::Footing)++footings;maxRadius=std::max({maxRadius,member.radiusBase,member.radiusTop});auto mesh=supportMemberMesh(member);
            check(mesh.positions.size()==34&&mesh.indices.size()==96&&mesh.normals.size()==34,"Closed frustum buffer size");Vec3 axis=unit(member.top-member.base);double length=norm(member.top-member.base);
            for(size_t i=0;i<mesh.positions.size();++i){Vec3 delta=mesh.positions[i]-member.base;double z=dot(delta,axis),radial=norm(delta-axis*z),radius=member.radiusBase+(member.radiusTop-member.radiusBase)*std::clamp(z/length,0.,1.);
                check(z>=-1e-8&&z<=length+1e-8&&radial<=radius+1e-8,"Every mesh vertex inside canonical tapered solid");check(std::abs(norm(mesh.normals[i])-1)<1e-9,"Unit mesh normal");auto p=VibeCoordinates::Position(mesh.positions[i]);check(norm(VibeCoordinates::CorePosition(p)-mesh.positions[i])<1e-9,"Coordinate roundtrip within floating-point precision");}
            for(size_t i=0;i<mesh.indices.size();i+=3){uint32_t a=mesh.indices[i],b=mesh.indices[i+1],c=mesh.indices[i+2];check(a<34&&b<34&&c<34,"Valid triangle indices");Vec3 n=cross(mesh.positions[b]-mesh.positions[a],mesh.positions[c]-mesh.positions[a]);check(dot(n,mesh.normals[a]+mesh.normals[b]+mesh.normals[c])>0,"Every triangle faces outward");
                auto u=VibeCoordinates::Position(mesh.positions[a]),v=VibeCoordinates::Position(mesh.positions[c]),w=VibeCoordinates::Position(mesh.positions[b]);auto expected=VibeCoordinates::Direction(mesh.normals[a]+mesh.normals[b]+mesh.normals[c]);check(dot(cross(Vec3{v.X-u.X,v.Y-u.Y,v.Z-u.Z},Vec3{w.X-u.X,w.Y-u.Y,w.Z-u.Z}),Vec3{expected.X,expected.Y,expected.Z})>0,"Reflected UE winding remains outward");}
        }
    }
    // Regression: a horizontal cap offset crossed riders on the steeply banked
    // low canyon turn. Placement must use the canonical bank frame without
    // changing its track, request, force envelope or every-member validation.
    GenerationRequest bankedRequest;bankedRequest.seed=24;bankedRequest.terrain.kind=TerrainKind::Canyon;bankedRequest.targets.requireIntensity=false;
    auto banked=generate(bankedRequest);check(banked.accepted(),"Banked canyon outreach regression accepted under unchanged gates");
    check(validateDesignStructures(banked).valid(),"Banked outreach clears canonical station and train");
    for(const auto& tower:banked.supports){auto frame=banked.track.sample(tower.trackDistance);
        const double standoff=-dot(tower.top-tower.attachment,frame.up);check(std::abs(standoff-2)<1e-8||std::abs(standoff-6)<1e-8||std::abs(standoff-10)<1e-8,"Tower cap uses a bounded canonical under-spine stand-off across banking");}
    const auto bankedPath=out/"banked-outreach.coaster";check(saveDesign(banked,bankedPath.string(),error),"Banked outreach saves after independent validation: "+error);Design bankedReplay;
    check(loadDesign(bankedPath.string(),bankedReplay,error),"Banked outreach normally replays: "+error);
    const std::string pacingWarning="Moving ride exceeds the 180-second pacing goal; physical acceptance is unchanged.";
    for(const auto* ride:{&banked,&bankedReplay})check((std::find(ride->report.warnings.begin(),ride->report.warnings.end(),pacingWarning)!=ride->report.warnings.end())==(movingRideSeconds(*ride)>180),"Pacing warning derives from actual duration on both generation and saved replay");
    check(reportJson(banked)==reportJson(bankedReplay),"Banked outreach complete physical replay is exact");
    auto good=out/"new.coaster";check(saveDesign(d,good.string(),error),"New save: "+error);Design replay;check(loadDesign(good.string(),replay,error),"New load: "+error);check(reportJson(d)==reportJson(replay),"Schema5 exact independent replay");for(size_t i=0;i<d.supports.size();++i)sameSupport(d.supports[i],replay.supports[i]);
    auto support=d.supports.front();
    check(!support.members.empty()&&support.members.front().kind==SupportMemberKind::Footing,"Malformed-footing fixtures start from a real footing");
    check(support.members.back().spineContact,"Malformed-contact fixtures start from the verified spine endpoint");
    auto invalid=[&](Support s,const char* name){check(!validateSupportMembers(s,d.request.terrain).valid(),name);};
    for(double radius:{0.,-1.,6.,double(NAN),double(INFINITY)}){auto x=support;x.members[0].radiusBase=radius;invalid(x,"Bad member radius rejected");}
    {auto x=support;x.members[0].kind=SupportMemberKind(9);invalid(x,"Invalid material kind rejected");}
    {auto x=support;x.members[0].top.x=NAN;invalid(x,"Nonfinite endpoint rejected");}
    {auto x=support;x.members[0].base.z+=100; x.members[0].top.z+=100;invalid(x,"Floating footing rejected");}
    {auto x=support;x.members[0].top.x+=1;invalid(x,"Tilted footing rejected");}
    {auto x=support;x.members.back().top.x+=1;invalid(x,"Unverified spine contact rejected");}
    {auto x=support;x.members.back().spineContact=false;invalid(x,"Missing spine contact rejected");}
    // Appending an explicitly disconnected, otherwise valid steel solid works
    // for every support family. Index 4 was out of bounds for three-member posts.
    {auto x=support;Vec3 base{support.base.x+100,support.base.y+100,0};
        base.z=d.request.terrain.height(base.x,base.y)+10;const Vec3 top=base+Vec3{0,0,3};
        for(const auto& m:x.members)check(norm(base-m.base)>1e-5&&norm(base-m.top)>1e-5&&norm(top-m.base)>1e-5&&norm(top-m.top)>1e-5,"Detached fixture endpoints share no existing graph node");
        x.members.push_back({base,top,.2,.2,SupportMemberKind::Steel,false});
        const auto result=validateSupportMembers(x,d.request.terrain);
        check(result.errors.size()==1&&code(result,"SUPPORT_CONNECTIVITY"),"Otherwise valid detached steel fails specifically SUPPORT_CONNECTIVITY");}

    {auto x=support;x.members.resize(maxSupportMembers+1);invalid(x,"Member count budget rejected");}
    std::vector<TrackSample> frames;int n=int(std::ceil(d.track.length/2));for(int i=0;i<n;++i)frames.push_back(d.track.sample(d.track.length*i/n));
    auto sweep=buildClearanceSweep(d.track,d.request.train);
    int calls=0;check(supportCollision(support,sweep,[&]{return ++calls>3;})==-2,"Cancellation during frame/member collision checks");
    calls=0;check(code(validateSupportMembers(support,d.request.terrain,[&]{return ++calls>4;}),"CANCELLED"),"Cancellation during member terrain validation");
    // Deliberately route a flagged joint through a physical rider at an actual frame.
    {auto x=support;auto q=frames[0];x.members={{q.position+q.up*1.,x.attachment,.18,.18,SupportMemberKind::Steel,true}};check(supportCollision(x,sweep)>=0,"Own-joint flag never exempts the train");}
    // A footing centreline misses the train, but its full radius must still collide.
    {auto x=support;auto q=frames[0];Vec3 p=q.position+q.right*3.;x.members={{p-q.up,p+q.up*3,2.,2.,SupportMemberKind::Footing,false}};check(supportCollision(x,sweep)>=0,"Footing full radius envelope checked");}
    const auto goodBytes=bytes(good);auto rows=lines(goodBytes);size_t firstSupport=5+d.track.knots.size()+d.operations.size(),firstMember=firstSupport+1;int malformed=0;
    for(auto [col,value]:std::vector<std::pair<size_t,std::string>>{{6,"0"},{6,"-1"},{6,"nan"},{6,"1e309"},{6,"6"},{8,"9"},{9,"2"},{0,"nan"},{5,"9999999"}}){auto bad=out/"malformed.coaster";corrupt(bad,rows,firstMember,col,value);Design unchanged=d;check(!loadDesign(bad.string(),unchanged,error),"Checksummed malformed member rejected: "+value);check(reportJson(unchanged)==reportJson(d),"Failed load retains accepted ride");++malformed;}
    for(auto value:{"513","60001","-1","18446744073709551615"}){auto bad=out/"malformed.coaster";corrupt(bad,rows,firstSupport,11,value);Design unchanged=d;check(!loadDesign(bad.string(),unchanged,error),"Checksummed oversized count rejected");check(reportJson(unchanged)==reportJson(d),"Oversized load retains accepted ride");++malformed;}
    {auto bad=d;bad.supports[0].members[0].radiusBase=NAN;check(!saveDesign(bad,good.string(),error),"Bad canonical member cannot overwrite accepted save");check(bytes(good)==goodBytes,"Rejected save preserves exact prior file");}
    check(!saveDesign(d,good.string(),error,[]{return true;}),"Cancelled save rejected");check(bytes(good)==goodBytes,"Cancelled save preserves exact prior file");
    std::ofstream result(root/"support-test-results.json");result<<std::setprecision(12)<<"{\"passed\":true,\"checks\":"<<checks<<",\"legacyFiles\":"<<legacyCases<<",\"unsupportedLegacySchemas\":"<<legacyRejected<<",\"newMembersMeshed\":"<<members<<",\"footings\":"<<footings<<",\"maxTowerHeight\":"<<tallest<<",\"maxMemberRadius\":"<<maxRadius<<",\"checksummedMalformedCases\":"<<malformed<<",\"ueCompiled\":false}";
    std::cout<<"PASS "<<checks<<" support assertions; "<<legacyCases<<" legacy upgrades (unsupported), "<<legacyRejected<<" explicit unsupported old schemas, "<<members<<" closed member meshes, "<<malformed<<" checksummed malformed cases; canonical geometry and physics unchanged. UE compilation not performed.\n";
}catch(const std::exception& e){std::cerr<<"FAIL after "<<checks<<": "<<e.what()<<'\n';return 1;}}
