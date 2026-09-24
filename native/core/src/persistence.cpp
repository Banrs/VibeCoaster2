#include "coaster/coaster.hpp"
#include "coaster/clearance.hpp"
#include "simulation_internal.hpp"
#include "progress_internal.hpp"
#include "acceptance_internal.hpp"
#include <fstream>
#include <iomanip>
#include <sstream>
#include <filesystem>
#include <set>
#include <atomic>
#include <future>
#include <mutex>
#include <cerrno>
#include <chrono>
#include <cstdio>
#include <charconv>
#include <fcntl.h>
#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#include <io.h>
#include <share.h>
#include <sys/stat.h>
#else
#include <unistd.h>
#endif

namespace coaster {
namespace {
std::filesystem::path utf8path(const std::string& s){return std::filesystem::path(std::u8string(reinterpret_cast<const char8_t*>(s.data()),s.size()));}
// Compatible rides retain their authored operations and provenance; every
// version passes the same geometry, structure and independent replay checks.
uint64_t checksum(const std::string& s){uint64_t h=14695981039346656037ull;for(unsigned char c:s){h^=c;h*=1099511628211ull;}return h;}
std::string quote(const std::string& s){std::ostringstream o;o<<'"';for(unsigned char c:s){switch(c){case '"':o<<"\\\"";break;case '\\':o<<"\\\\";break;case '\n':o<<"\\n";break;case '\r':o<<"\\r";break;case '\t':o<<"\\t";break;default:if(c<32)o<<"\\u"<<std::hex<<std::setw(4)<<std::setfill('0')<<int(c)<<std::dec;else o<<c;}}o<<'"';return o.str();}
void number(std::ostream& o,double x){if(std::isfinite(x))o<<x;else o<<"null";}
void vec(std::ostream& o,Vec3 v){
    char bytes[96];char* next=bytes;
    for(double value:{v.x,v.y,v.z}){
        const auto result=std::to_chars(next,bytes+sizeof(bytes)-1,value,std::chars_format::general,17);
        if(result.ec!=std::errc{})throw std::runtime_error("Coordinate serialization failed");
        next=result.ptr;*next++=' ';
    }
    o.write(bytes,next-bytes);
}
void vec(std::istream& i,Vec3& v){i>>v.x>>v.y>>v.z;}
std::string extensionTail(const Design& d,Cancel cancel){
    const auto& r=d.request;
    std::vector<std::pair<std::string,std::string>> blocks;
    // Existing flat COASTER6 payloads remain byte-compatible. The new terrain
    // kind owns its resolved landform parameters rather than regenerating them.
    if(r.terrain.kind==TerrainKind::Flat)blocks.push_back({"TERRAIN_PROFILE","1 1 0 0 0 0 600\n"});
    else {std::ostringstream b;b.imbue(std::locale::classic());b<<std::setprecision(17);
        const auto& t=r.terrain;b<<t.centerX<<' '<<t.centerY<<' '<<t.heightMeters<<' '<<t.radiusX<<' '<<t.radiusY<<' '<<t.bend<<'\n';
        blocks.push_back({"TERRAIN_PROFILE",b.str()});}
    if(r.terrain.plateau>0){const auto& t=r.terrain;std::ostringstream b;b.imbue(std::locale::classic());b<<std::setprecision(17)<<t.plateau<<' '<<t.cliffX<<' '<<t.cliffY<<' '<<t.cliffHeading<<' '<<t.cliffWidth<<'\n';blocks.push_back({"ESCARPMENT",b.str()});}
    if(r.terrain.cliffCurvature!=0){std::ostringstream b;b.imbue(std::locale::classic());b<<std::setprecision(17)<<r.terrain.cliffCurvature<<'\n';blocks.push_back({"CLIFF_FRONT",b.str()});}
    if(r.terrain.backSlope){
        const auto& q=*r.terrain.backSlope;std::ostringstream b;b.imbue(std::locale::classic());
        b<<std::setprecision(17)<<q.x<<' '<<q.y<<' '<<q.height<<' '<<q.gradeX<<' '<<q.gradeY;
        if(q.width>0)b<<' '<<q.width;
        b<<'\n';blocks.push_back({q.width>0?"TERRAIN_BACK_SLOPE_BOUNDED":"TERRAIN_BACK_SLOPE",b.str()});
    }
    if(!r.terrain.ravines.empty()){std::ostringstream b;b.imbue(std::locale::classic());b<<std::setprecision(17)<<r.terrain.ravines.size()<<'\n';
        for(const auto& q:r.terrain.ravines)b<<q.x0<<' '<<q.y0<<' '<<q.x1<<' '<<q.y1<<' '<<q.depth0<<' '<<q.depth1<<' '<<q.width0<<' '<<q.width1<<'\n';blocks.push_back({"TERRAIN_RAVINES",b.str()});}
    if(!r.terrain.ramps.empty()){std::ostringstream b;b.imbue(std::locale::classic());b<<std::setprecision(17)<<r.terrain.ramps.size()<<'\n';for(const auto& q:r.terrain.ramps)b<<q.x0<<' '<<q.y0<<' '<<q.x1<<' '<<q.y1<<' '<<q.h0<<' '<<q.h1<<' '<<q.grade0<<' '<<q.grade1<<' '<<q.width<<'\n';blocks.push_back({"TERRAIN_RAMPS",b.str()});}
    if(!r.terrain.knolls.empty()){std::ostringstream b;b.imbue(std::locale::classic());b<<std::setprecision(17)<<r.terrain.knolls.size()<<'\n';for(const auto& q:r.terrain.knolls)b<<q.x<<' '<<q.y<<' '<<q.height<<' '<<q.radius<<'\n';blocks.push_back({"TERRAIN_KNOLLS",b.str()});}
    if(!r.terrain.foothills.empty()){std::ostringstream b;b.imbue(std::locale::classic());b<<std::setprecision(17)<<r.terrain.foothills.size()<<'\n';for(const auto& q:r.terrain.foothills)b<<q.x<<' '<<q.y<<' '<<q.height<<' '<<q.radius<<'\n';blocks.push_back({"TERRAIN_FOOTHILLS",b.str()});}
    if(!r.terrain.ridge.points.empty()){const auto& ridge=r.terrain.ridge;std::ostringstream b;b.imbue(std::locale::classic());b<<std::setprecision(17)<<ridge.points.size()<<' '<<ridge.spineCount<<' '<<ridge.width<<' '<<ridge.curvature<<'\n';
        for(const auto& p:ridge.points)b<<p.x<<' '<<p.y<<' '<<p.height<<' '<<p.gx<<' '<<p.gy<<'\n';blocks.push_back({"TERRAIN_RIDGE",b.str()});}
    if(d.station.enabled)blocks.push_back({"STATION",stationPayload(d.station,cancel)});
    if(!d.sections.empty()){std::ostringstream b;b<<std::setprecision(17)<<r.style.airtime<<' '<<r.style.signatureRollDegrees<<' '<<r.style.returnStyle<<' '<<r.style.automaticTrims<<'\n';blocks.push_back({"RIDE_STYLE",b.str()});}
    if(!d.sections.empty()){
        std::ostringstream b;b.imbue(std::locale::classic());b<<std::setprecision(17)<<d.sections.size()<<'\n';
        for(const auto& s:d.sections)b<<std::quoted(s.name)<<' '<<s.start<<' '<<s.end<<' '<<s.heightReversals<<' '<<s.planar<<'\n';
        blocks.push_back({"MOTION_SECTIONS",b.str()});
        if(!r.recipe.elements.empty()){
            std::ostringstream roles;roles<<d.sections.size()<<'\n';
            for(const auto& section:d.sections)roles<<int(section.role)<<' '<<std::quoted(section.recipeId)<<'\n';
            blocks.push_back({"SECTION_ROLES",roles.str()});
        }
    }
    blocks.push_back({"AUTHORING",authorshipPayload(d)});
    if(!d.landmarks.empty()){
        std::ostringstream b;b.imbue(std::locale::classic());b<<std::setprecision(17)<<d.landmarks.size()<<'\n';
        for(const auto& landmark:d.landmarks)b<<int(landmark.kind)<<' '<<landmark.distance<<'\n';
        blocks.push_back({"MOTION_LANDMARKS",b.str()});
    }
    if(!r.recipe.elements.empty())blocks.push_back({"RIDE_RECIPE",recipePayload(r.recipe)});
    const auto trimCount=std::count_if(d.operations.begin(),d.operations.end(),[](const Operation& op){return op.kind==DriveKind::Trim;});
    if(trimCount){std::ostringstream b;b.imbue(std::locale::classic());b<<std::setprecision(17)<<trimCount<<'\n';
        for(size_t i=0;i<d.operations.size();++i){const auto& op=d.operations[i];if(op.kind==DriveKind::Trim)b<<i<<' '<<op.trimPeakSpeed<<' '<<op.trimSensorLead<<'\n';}
        blocks.push_back({"TRIM_BRAKES",b.str()});}
    if(r.targets.reference.processed)blocks.push_back({"REFERENCE",serializeReference(r.targets.reference)});
    if(std::isfinite(r.limits.maxLateralRateGps)||std::isfinite(r.limits.maxLongitudinalRateGps)){
        std::ostringstream b;b.imbue(std::locale::classic());b<<std::setprecision(17);
        for(double x:{r.limits.maxLateralRateGps,r.limits.maxLongitudinalRateGps})b<<std::isfinite(x)<<' '<<(std::isfinite(x)?x:0)<<' ';b<<'\n';blocks.push_back({"AXIS_RATE_LIMITS",b.str()});
    }
    std::ostringstream out;out<<"EXTENSIONS "<<blocks.size()<<'\n';for(const auto& [name,bytes]:blocks)out<<name<<" 1 "<<bytes.size()<<'\n'<<bytes;return out.str();
}
std::string designPayload(const Design& d,Cancel cancel){
        std::ostringstream p;p.imbue(std::locale::classic());p<<std::setprecision(17);
        const auto& r=d.request;p<<std::quoted(d.generationVersion)<<' '<<r.seed<<' '<<int(r.terrain.kind)<<' '<<r.maxCandidates<<' '<<r.simulationStep<<'\n';
        p<<r.targets.height<<' '<<r.targets.speed<<' '<<r.targets.inversionHeight<<' '<<r.targets.launchSeconds<<' '<<r.targets.requireIntensity<<' '<<std::isfinite(r.targets.referenceExposure)<<' '<<(std::isfinite(r.targets.referenceExposure)?r.targets.referenceExposure:0)<<' '<<std::quoted(r.targets.referenceId)<<'\n';
        const auto& l=r.limits;p<<l.minVerticalG<<' '<<l.maxVerticalG<<' '<<l.maxLateralG<<' '<<l.maxLongitudinalG<<' '<<l.maxJerkGps<<' '<<l.minClearance<<'\n';
        const auto& t=r.train;p<<t.cars<<' '<<t.carMass<<' '<<t.spacing<<' '<<t.seatHeight<<' '<<t.dragCdA<<' '<<t.rollingResistance<<' '<<t.airDensity<<'\n';
        p<<std::quoted(d.topology)<<' '<<d.candidate<<' '<<d.track.closed<<' '<<d.track.knots.size()<<' '<<d.operations.size()<<' '<<d.supports.size()<<'\n';
        for(const auto& k:d.track.knots){vec(p,k.position);vec(p,k.tangent);vec(p,k.curvature);vec(p,k.third);vec(p,k.fourth);vec(p,k.up);vec(p,k.upFirst);vec(p,k.upSecond);vec(p,k.upThird);p<<k.bank<<' '<<int(k.element)<<'\n';}
        for(const auto& op:d.operations)p<<op.start<<' '<<op.end<<' '<<int(op.kind)<<' '<<op.targetSpeed<<' '<<op.maxForce<<' '<<op.maxPower<<' '<<op.rampSeconds<<' '<<op.stopDeceleration<<' '<<op.stopOffset<<' '<<op.exitFadeMeters<<'\n';
        for(const auto& s:d.supports){vec(p,s.base);vec(p,s.top);vec(p,s.attachment);p<<s.hasAttachment<<' '<<s.trackDistance<<' '<<s.members.size()<<'\n';
            for(const auto& m:s.members){if(cancel&&cancel())throw std::runtime_error("CANCELLED");vec(p,m.base);vec(p,m.top);p<<m.radiusBase<<' '<<m.radiusTop<<' '<<int(m.kind)<<' '<<m.spineContact<<'\n';}}
        p<<extensionTail(d,cancel);
        return p.str();
}
std::string cachePayload(const Track& track){
    std::string bytes;bytes.reserve(track.spans.size()*sizeof(Span)+32);
    auto scalar=[&](auto value){bytes.append(reinterpret_cast<const char*>(&value),sizeof(value));};
    auto vector=[&](Vec3 value){scalar(value.x);scalar(value.y);scalar(value.z);};
    scalar(track.length);scalar(track.closed);scalar(track.authoredGeometry);scalar(track.authoredFrame);scalar(track.spans.size());
    for(const auto& span:track.spans){for(auto p:span.c)vector(p);for(auto p:span.referenceUp)vector(p);for(double bank:span.bank)scalar(bank);scalar(span.start);scalar(span.length);for(double coefficient:span.arcPolynomial)scalar(coefficient);scalar(span.polynomialArc);}
    return bytes;
}
bool parseExtensions(std::istream& p,Design& out,std::string& error,Cancel cancel){
    p>>std::ws;if(p.eof()){error="Missing COASTER6 extension directory";return false;}
    Design result=out;auto& request=result.request;std::string tag;size_t count;p>>tag>>count;
    if(!p||tag!="EXTENSIONS"||count>24){error="Invalid extension directory";return false;}
    std::set<std::string> seen;
    std::vector<std::pair<RideRole,std::string>> sectionRoles;
    for(size_t i=0;i<count;++i){if(cancel&&cancel()){error="CANCELLED";return false;}std::string name;int version;size_t size;p>>name>>version>>size;
        if(!p||version!=1||size==0||size>1024*1024||!seen.insert(name).second||p.get()!='\n'){error="Malformed, duplicate or oversized extension";return false;}
        std::string bytes(size,'\0');if(!p.read(bytes.data(),std::streamsize(size))){error="Truncated extension";return false;}
        if(name=="REFERENCE"){
            Targets parsed=request.targets;if(!parseReference(bytes,parsed,error))return false;
            if(parsed.referenceExposure!=request.targets.referenceExposure||parsed.referenceId!=request.targets.referenceId){error="Reference extension disagrees with saved scalar target";return false;}request.targets=std::move(parsed);
        }else if(name=="AXIS_RATE_LIMITS"){
            std::istringstream b(bytes);b.imbue(std::locale::classic());bool lat,lon;double a,c;b>>lat>>a>>lon>>c;
            if(!b||!std::isfinite(a)||!std::isfinite(c)||(lat?a<=0:a!=0)||(lon?c<=0:c!=0)){error="Invalid axis rate extension";return false;}b>>std::ws;if(!b.eof()){error="Trailing axis rate extension";return false;}
            request.limits.maxLateralRateGps=lat?a:NAN;request.limits.maxLongitudinalRateGps=lon?c:NAN;
        }else if(name=="TERRAIN_PROFILE"){
            std::istringstream b(bytes);b.imbue(std::locale::classic());
            if(request.terrain.kind==TerrainKind::Flat){
                for(double expected:{1.,1.,0.,0.,0.,0.,600.}){double value;if(!(b>>value)||value!=expected){error="Invalid flat ground profile";return false;}}
            }else {auto& t=request.terrain;b>>t.centerX>>t.centerY>>t.heightMeters>>t.radiusX>>t.radiusY>>t.bend;
                if(!b||!t.valid()){error="Invalid highlands profile";return false;}}
            b>>std::ws;if(!b.eof()){error="Trailing terrain profile data";return false;}
        }else if(name=="ESCARPMENT"){
            auto& t=request.terrain;std::istringstream b(bytes);b.imbue(std::locale::classic());b>>t.plateau>>t.cliffX>>t.cliffY>>t.cliffHeading>>t.cliffWidth;
            if(!b||t.kind!=TerrainKind::Highlands||t.plateau<=0||!t.valid()){error="Invalid escarpment profile";return false;}b>>std::ws;if(!b.eof()){error="Trailing escarpment profile data";return false;}
        }else if(name=="CLIFF_FRONT"){
            std::istringstream b(bytes);b.imbue(std::locale::classic());double curvature=0;b>>curvature;
            if(!b||request.terrain.kind!=TerrainKind::Highlands||!std::isfinite(curvature)||curvature<0||curvature>.01){error="Invalid cliff curvature";return false;}
            b>>std::ws;if(!b.eof()){error="Trailing cliff-front data";return false;}request.terrain.cliffCurvature=curvature;
        }else if(name=="TERRAIN_BACK_SLOPE"||name=="TERRAIN_BACK_SLOPE_BOUNDED"){
            if(request.terrain.backSlope){error="Duplicate terrain rear slope";return false;}
            const bool bounded=name=="TERRAIN_BACK_SLOPE_BOUNDED";
            std::istringstream b(bytes);b.imbue(std::locale::classic());TerrainSlope q;b>>q.x>>q.y>>q.height>>q.gradeX>>q.gradeY;
            if(bounded)b>>q.width;
            if(!b||request.terrain.kind!=TerrainKind::Highlands||!q.valid()||(bounded&&q.width==0)){error="Invalid terrain rear slope";return false;}
            b>>std::ws;if(!b.eof()){error="Trailing terrain rear-slope data";return false;}request.terrain.backSlope=q;
        }else if(name=="MOTION_LANDMARKS"){
            std::istringstream b(bytes);b.imbue(std::locale::classic());size_t landmarkCount=0;b>>landmarkCount;std::set<int> kinds;
            if(!b||landmarkCount==0||landmarkCount>64){error="Invalid motion landmark count";return false;}
            for(size_t j=0;j<landmarkCount;++j){int kind=0;double distance=0;b>>kind>>distance;
                if(!b||kind<0||kind>int(LandmarkKind::BrakeEntry)||!std::isfinite(distance)||distance<0||!kinds.insert(kind).second){error="Invalid or duplicate motion landmark";return false;}
                result.landmarks.push_back({LandmarkKind(kind),distance});}
            b>>std::ws;if(!b.eof()){error="Trailing landmark data";return false;}
        }else if(name=="SECTION_ROLES"){
            std::istringstream b(bytes);b.imbue(std::locale::classic());size_t roleCount=0;b>>roleCount;
            if(!b||roleCount==0||roleCount>512){error="Invalid section role count";return false;}
            for(size_t j=0;j<roleCount;++j){int role=0;std::string id;b>>role>>std::quoted(id);
                if(!b||role<=int(RideRole::Unspecified)||role>int(RideRole::Brakes)||id.empty()||id.size()>64){error="Invalid section role or recipe ID";return false;}
                sectionRoles.push_back({RideRole(role),std::move(id)});}
            b>>std::ws;if(!b.eof()){error="Trailing section role data";return false;}
        }else if(name=="TERRAIN_RAVINES"){
            std::istringstream b(bytes);b.imbue(std::locale::classic());size_t ravineCount=0;b>>ravineCount;
            if(!b||ravineCount>16||request.terrain.kind!=TerrainKind::Highlands){error="Invalid ravine count";return false;}
            for(size_t j=0;j<ravineCount;++j){TerrainRavine q;b>>q.x0>>q.y0>>q.x1>>q.y1>>q.depth0>>q.depth1>>q.width0>>q.width1;
                if(!b||!q.valid()){error="Invalid ravine geometry";return false;}request.terrain.ravines.push_back(q);}
            b>>std::ws;if(!b.eof()){error="Trailing ravine data";return false;}
        }else if(name=="TERRAIN_RIDGE"){
            auto& ridge=request.terrain.ridge;std::istringstream b(bytes);b.imbue(std::locale::classic());size_t n=0;b>>n>>ridge.spineCount>>ridge.width>>ridge.curvature;
            if(!b||n<2||n>512||request.terrain.kind!=TerrainKind::Highlands){error="Invalid terrain ridge count";return false;}
            ridge.points.resize(n);for(auto& point:ridge.points)b>>point.x>>point.y>>point.height>>point.gx>>point.gy;
            if(!b||!ridge.valid()){error="Invalid terrain ridge";return false;}b>>std::ws;if(!b.eof()){error="Trailing terrain ridge data";return false;}
        }else if(name=="TERRAIN_KNOLLS"||name=="TERRAIN_FOOTHILLS"){
            std::istringstream b(bytes);b.imbue(std::locale::classic());size_t crowns=0;b>>crowns;if(!b||crowns==0||crowns>8){error="Invalid terrain crown count";return false;}
            auto& collection=name=="TERRAIN_FOOTHILLS"?request.terrain.foothills:request.terrain.knolls;
            for(size_t j=0;j<crowns;++j){TerrainKnoll q;b>>q.x>>q.y>>q.height>>q.radius;if(!b||!q.valid()){error="Invalid terrain crown";return false;}collection.push_back(q);}b>>std::ws;if(!b.eof()){error="Trailing terrain crown data";return false;}
        }else if(name=="TERRAIN_RAMPS"){
            std::istringstream b(bytes);b.imbue(std::locale::classic());size_t ramps;b>>ramps;if(!b||ramps>8||request.terrain.kind!=TerrainKind::Highlands){error="Invalid terrain ramp count";return false;}
            for(size_t j=0;j<ramps;++j){TerrainRamp q;b>>q.x0>>q.y0>>q.x1>>q.y1>>q.h0>>q.h1>>q.grade0>>q.grade1>>q.width;if(!b||!q.valid()){error="Invalid terrain ramp";return false;}request.terrain.ramps.push_back(q);}b>>std::ws;if(!b.eof()){error="Trailing terrain ramp data";return false;}
        }else if(name=="RIDE_STYLE"){
            std::istringstream b(bytes);b>>request.style.airtime>>request.style.signatureRollDegrees>>request.style.returnStyle;
            if(!b){error="Invalid ride style";return false;}b>>std::ws;
            request.style.automaticTrims=false;
            if(!b.eof()){b>>request.style.automaticTrims;if(!b){error="Invalid automatic trim setting";return false;}b>>std::ws;}
            if(!b.eof()){error="Trailing ride style data";return false;}
        }else if(name=="MOTION_SECTIONS"){
            std::istringstream b(bytes);b.imbue(std::locale::classic());size_t sections=0;b>>sections;
            if(!b||sections==0||sections>512){error="Invalid motion section count";return false;}
            result.sections.clear();
            for(size_t j=0;j<sections;++j){RideSection s;b>>std::quoted(s.name)>>s.start>>s.end>>s.heightReversals>>s.planar;
                if(!b||!validIdentifier(s.name,128)){error="Invalid motion section";return false;}result.sections.push_back(s);}
            b>>std::ws;if(!b.eof()){error="Trailing motion section data";return false;}
        }else if(name=="TRIM_BRAKES"){
            std::istringstream b(bytes);b.imbue(std::locale::classic());size_t n=0;b>>n;std::set<size_t> indices;
            if(!b||n==0||n>result.operations.size()){error="Invalid trim brake count";return false;}
            for(size_t trim=0;trim<n;++trim){size_t index=0;double peak=0,lead=0;b>>index>>peak>>lead;
                if(!b||index>=result.operations.size()||result.operations[index].kind!=DriveKind::Trim||!indices.insert(index).second){error="Invalid trim brake mapping";return false;}
                auto& op=result.operations[index];op.trimPeakSpeed=peak;op.trimSensorLead=lead;
                if(!validDriveParameters(op)){error="Invalid trim brake profile";return false;}}
            b>>std::ws;if(!b.eof()){error="Trailing trim brake data";return false;}
        }else if(name=="AUTHORING"){
            if(!parseAuthorshipPayload(bytes,result,error))return false;
        }else if(name=="RIDE_RECIPE"){
            if(!parseRecipe(bytes,request.recipe,error))return false;
        }else if(name=="STATION"){
            if(!parseStationPayload(bytes,result.station,error,cancel))return false;
        }else{error="Unknown extension: "+name;return false;}
    }
    if(!seen.count("TERRAIN_PROFILE")){error="Missing required terrain profile";return false;}
    if(!request.recipe.elements.empty()){
        if(sectionRoles.size()!=result.sections.size()){error="Recipe requires a complete typed section mapping";return false;}
        for(size_t i=0;i<sectionRoles.size();++i){
            const auto& [role,id]=sectionRoles[i];
            const auto element=std::find_if(request.recipe.elements.begin(),request.recipe.elements.end(),[&](const RecipeElement& e){return e.id==id&&e.role==role;});
            if(element==request.recipe.elements.end()){error="Section does not belong to its saved recipe";return false;}
            result.sections[i].role=role;result.sections[i].recipeId=id;
        }
    }else if(!sectionRoles.empty()){error="Typed sections require their saved recipe";return false;}
    for(const auto& op:result.operations)if(!validDriveParameters(op)){error="Missing or invalid operation profile";return false;}
    p>>std::ws;if(!p.eof()){error="Unexpected data after extensions";return false;}out=std::move(result);return true;
}
bool recheck(Design& d,Cancel cancel,WorkRecorder* work=nullptr){
    std::mutex cancellationMutex;
    const Cancel requestedCancel=std::move(cancel);
    if(requestedCancel)cancel=[&]{std::lock_guard lock(cancellationMutex);return requestedCancel();};
    if(!supportedGeneratorVersion(d.generationVersion)){d.report.fail("GENERATOR_VERSION","Unsupported generation provenance");return false;}
    d.report=validateRequest(d.request);if(!d.report.valid())return false;
    for(const auto& op:d.operations)if(!validDriveParameters(op)){d.report.fail("DRIVE_CONFIG","Invalid explicit drive operation");return false;}
    if(work)work->enter(WorkPhase::Geometry,d.candidate,"Checking saved track and clearance");
    d.track.rebuild();d.inversionDimensions=d.request.recipe.elements.empty()?measureInversionDimensions(d.track,cancel):measureInversionDimensions(d.track,d.sections,cancel);
    // Rebuild certifies the numerical interpolation domain. Independent
    // dynamics and spatial checks can now overlap the solid-clearance pass;
    // no result is accepted until all of them have joined and passed.
    std::atomic<bool> stopFine{false};
    const Cancel stop=[&]{return stopFine.load()||(cancel&&cancel());};
    auto coarse=std::async(std::launch::async,[&]{return simulate(d.track,d.operations,d.request.train,d.request.simulationStep,stop);});
    auto fine=std::async(std::launch::async,[&]{return simulate(d.track,d.operations,d.request.train,d.request.simulationStep*.5,stop);});
    auto spatial=std::async(std::launch::async,[&]{return replaySpatialRefinement(d,stop);});
    auto sweep=buildClearanceSweepVerified(d.track,d.request.train,cancel);sweep.prepareGround(d.request.terrain,cancel);
    // Both checks read the rebuilt design and the prepared sweep.
    auto structures=std::async(std::launch::async,[&]{return validateDesignStructures(d,sweep,cancel);});
    d.report=validateGeometry(d.track,d.request.terrain,d.request.limits,d.request.train,d.supports,sweep,cancel);
    if(work)work->enter(WorkPhase::Structures,d.candidate,"Checking saved supports and station");
    auto structureReport=structures.get();d.report.errors.insert(d.report.errors.end(),structureReport.errors.begin(),structureReport.errors.end());
    if(!d.report.valid()){
        stopFine.store(true);
        if(std::any_of(d.report.errors.begin(),d.report.errors.end(),[](const Finding& f){return f.code=="CANCELLED";}))d.simulation.cancelled=true;
        return false;
    }
    // Both resolutions read the same rebuilt geometry; callback invocations stay
    // serialized, and the worker joins before any design can escape this check.
    if(work)work->enter(WorkPhase::Forces,d.candidate,"Replaying all seats at both native resolutions");
    d.convergence={};d.simulation=coarse.get();
    evaluateTargets(d,&sweep);
    if(work)work->enter(WorkPhase::Authorship,d.candidate,"Checking editable sources and continuous motion");
    assessAuthorship(d,cancel);assessMotion(d,cancel);
    if(work)work->enter(WorkPhase::Refinement,d.candidate,"Checking independent time and spatial refinement");
    verifyConvergenceWith(d,[&]{return fine.get();},cancel);
    if(fine.valid()){stopFine.store(true);fine.wait();}
    if(d.report.valid()&&d.convergence.passed)verifySpatialRefinementWith(d,[&]{return spatial.get();},cancel);
    if(spatial.valid()){stopFine.store(true);spatial.wait();}
    if(d.checksPassed())freezeAcceptedRevision(d);
    return d.accepted();
}
}
void freezeAcceptedRevision(Design& d){
    if(!d.checksPassed())throw std::runtime_error("Incomplete native checks cannot create an accepted revision");
    d.acceptedRequest_=d.request;
    d.acceptedPayload_=std::make_shared<const std::string>(designPayload(d,{}));
    d.acceptedCache_=std::make_shared<const std::string>(cachePayload(d.track));
}
std::string reportJson(const Design& d){
    std::ostringstream o;o.imbue(std::locale::classic());o<<std::setprecision(17);
    o<<"{\"schemaVersion\":1,\"generatorVersion\":"<<quote(d.generationVersion)<<",\"runtimeVersion\":"<<quote(generatorVersion)<<",\"buildCommit\":"<<quote(COASTER_BUILD_COMMIT)<<",\"seed\":"<<d.request.seed<<",\"terrain\":"<<quote(d.request.terrain.name())<<",\"preset\":"<<quote(d.request.targets.requireIntensity?"reference":"default")<<",\"intensityRequired\":"<<(d.request.targets.requireIntensity?"true":"false")<<",\"accepted\":"<<(d.accepted()?"true":"false")<<",\"completed\":"<<(d.simulation.completed?"true":"false")<<",\"cancelled\":"<<(d.simulation.cancelled?"true":"false")<<",\"candidate\":"<<d.candidate<<",\"topology\":"<<quote(d.topology)<<",\"lengthMeters\":";number(o,d.track.length);
    o<<",\"timingsSeconds\":{";
    for(size_t i=0;i<d.timings.seconds.size();++i){if(i)o<<',';o<<quote(phaseKey(WorkPhase(i)))<<':';number(o,d.timings.seconds[i]);}o<<'}';
    o<<",\"accelerationAssessment\":{\"edition\":\"ASTM F2291-25\",\"scope\":\"upright Class 4/5 base case; numerical acceleration checks; not whole-standard certification\",\"sampleRateHz\":";number(o,1/d.request.simulationStep);o<<",\"seats\":[";
    for(size_t seat=0;seat<d.simulation.acceleration.size();++seat){
        if(seat)o<<',';const auto& a=d.simulation.acceleration[seat];
        o<<"{\"seat\":"<<seat<<",\"performed\":"<<(a.performed?"true":"false")<<",\"passed\":"<<(a.passed?"true":"false")<<",\"cancelled\":"<<(a.cancelled?"true":"false")<<",\"filteredExtremaXYZ\":[";
        for(size_t axis=0;axis<3;++axis){if(axis)o<<',';const auto& x=a.filteredExtrema[axis];o<<"{\"minG\":";number(o,x.minimumG);o<<",\"maxG\":";number(o,x.maximumG);o<<",\"minTime\":";number(o,x.minimumTimeSeconds);o<<",\"maxTime\":";number(o,x.maximumTimeSeconds);o<<'}';}
        o<<"],\"onset100msXYZ\":[";
        for(size_t axis=0;axis<3;++axis){if(axis)o<<',';const auto& x=a.onset100ms[axis];o<<"{\"available\":"<<(x.available?"true":"false")<<",\"minGps\":";number(o,x.minimumGps);o<<",\"maxGps\":";number(o,x.maximumGps);o<<'}';}
        o<<"],\"diagnostics\":[";
        for(size_t i=0;i<a.diagnostics.size();++i){if(i)o<<',';const auto& f=a.diagnostics[i];o<<"{\"clause\":"<<quote(f.clause)<<",\"rule\":"<<quote(f.rule)<<",\"axis\":"<<quote(f.axis)<<",\"sign\":"<<quote(f.sign)<<",\"beginSeconds\":";number(o,f.startTimeSeconds);o<<",\"endSeconds\":";number(o,f.endTimeSeconds);o<<",\"actual\":";number(o,f.actual);o<<",\"limit\":";number(o,f.limit);o<<",\"utilization\":";number(o,f.utilization);o<<'}';}
        o<<"]}";
    }o<<"]}";
    const auto& author=d.authorship;
    o<<",\"authorship\":{\"performed\":"<<(author.performed?"true":"false")<<",\"passed\":"<<(author.passed?"true":"false")
     <<",\"forceProgrammes\":"<<d.forcePrograms.size()<<",\"splineProgrammes\":"<<d.splinePrograms.size()
     <<",\"maximumSourcePositionErrorMeters\":"<<author.maximumSourcePositionError
     <<",\"maximumNormalResidualG\":"<<author.maximumNormalResidualG<<",\"maximumLateralResidualG\":"<<author.maximumLateralResidualG
     <<",\"maximumRollResidualRadians\":"<<author.maximumRollResidualRadians<<'}';
    o<<",\"stationEnabled\":"<<(d.station.enabled?"true":"false")<<",\"stationPartCount\":"<<d.station.boxes.size();
    o<<",\"maxCandidates\":"<<d.request.maxCandidates<<",\"simulationStep\":";number(o,d.request.simulationStep);
    const auto& c=d.convergence;
    o<<",\"convergence\":{\"performed\":"<<(c.performed?"true":"false")<<",\"passed\":"<<(c.passed?"true":"false")<<",\"coarseStep\":";number(o,c.coarseStep);o<<",\"fineStep\":";number(o,c.fineStep);
    o<<",\"maxSpeedRelativeError\":";number(o,c.maxSpeedRelativeError);o<<",\"maxForceRelativeError\":";number(o,c.maxForceRelativeError);o<<",\"metrics\":[";
    bool convergenceComma=false;for(const auto& metric:c.metrics){if(convergenceComma)o<<',';convergenceComma=true;o<<"{\"name\":"<<quote(metric.name)<<",\"coarse\":";number(o,metric.coarse);o<<",\"fine\":";number(o,metric.fine);o<<",\"absoluteDifference\":";number(o,metric.absoluteDifference);o<<",\"tolerance\":";number(o,metric.tolerance);o<<'}';}o<<"]}";
    o<<",\"intensityComparison\":\"Maximum over physical front/middle/rear seats of the strongest ten-second integral of max(vertical_g,0); configured reference identity is external\"";
    auto object=[&](const char* name,const std::vector<std::pair<const char*,double>>& fields){o<<','<<quote(name)<<":{";bool first=true;for(auto [key,value]:fields){if(!first)o<<',';first=false;o<<quote(key)<<':';number(o,value);}o<<'}';};
    const auto& terrain=d.request.terrain;object("terrainProfile",{{"centerXMeters",terrain.centerX},{"centerYMeters",terrain.centerY},{"ridgeHeightMeters",terrain.kind==TerrainKind::Flat?0:terrain.heightMeters},{"radiusXMeters",terrain.radiusX},{"radiusYMeters",terrain.radiusY},{"bend",terrain.bend},{"plateau",terrain.plateau},{"cliffX",terrain.cliffX},{"cliffY",terrain.cliffY},{"cliffHeading",terrain.cliffHeading},{"cliffWidth",terrain.cliffWidth},{"cliffCurvature",terrain.cliffCurvature},{"meshStepMeters",Terrain::gridStep},{"globalSlopeBound",terrain.slopeBound()}});
    if(terrain.backSlope){const auto& q=*terrain.backSlope;object("terrainBackSlope",{{"x",q.x},{"y",q.y},{"height",q.height},{"gradeX",q.gradeX},{"gradeY",q.gradeY},{"width",q.width}});}
    o<<",\"terrainFoothills\":[";for(size_t i=0;i<terrain.foothills.size();++i){if(i)o<<',';const auto& k=terrain.foothills[i];o<<'['<<k.x<<','<<k.y<<','<<k.height<<','<<k.radius<<']';}o<<']';
    o<<",\"terrainRavines\":[";for(size_t i=0;i<terrain.ravines.size();++i){if(i)o<<',';const auto& q=terrain.ravines[i];o<<'['<<q.x0<<','<<q.y0<<','<<q.x1<<','<<q.y1<<','<<q.depth0<<','<<q.depth1<<','<<q.width0<<','<<q.width1<<']';}o<<']';
    o<<",\"terrainRamps\":[";
    for(size_t i=0;i<terrain.ramps.size();++i){if(i)o<<',';const auto& r=terrain.ramps[i];o<<'[';bool comma=false;for(double v:{r.x0,r.y0,r.x1,r.y1,r.h0,r.h1,r.grade0,r.grade1,r.width}){if(comma)o<<',';comma=true;number(o,v);}o<<']';}o<<']';
    o<<",\"terrainRidge\":{\"spineCount\":"<<terrain.ridge.spineCount<<",\"width\":"<<terrain.ridge.width<<",\"curvature\":"<<terrain.ridge.curvature<<",\"points\":[";
    for(size_t i=0;i<terrain.ridge.points.size();++i){if(i)o<<',';const auto& p=terrain.ridge.points[i];o<<'['<<p.x<<','<<p.y<<','<<p.height<<','<<p.gx<<','<<p.gy<<']';}o<<"]}";
    o<<",\"terrainKnolls\":[";for(size_t i=0;i<terrain.knolls.size();++i){if(i)o<<',';const auto& k=terrain.knolls[i];o<<'['<<k.x<<','<<k.y<<','<<k.height<<','<<k.radius<<']';}o<<']';
    const auto& l=d.request.limits;object("limits",{{"minVerticalG",l.minVerticalG},{"maxVerticalG",l.maxVerticalG},{"maxLateralG",l.maxLateralG},{"maxLongitudinalG",l.maxLongitudinalG},{"maxJerkGps",l.maxJerkGps},{"minClearance",l.minClearance},{"maxLateralRateGps",l.maxLateralRateGps},{"maxLongitudinalRateGps",l.maxLongitudinalRateGps}});
    const auto& t=d.request.train;object("train",{{"cars",double(t.cars)},{"riderCapacity",double(riderCapacity(t))},{"carMass",t.carMass},{"spacing",t.spacing},{"seatHeight",t.seatHeight},{"dragCdA",t.dragCdA},{"rollingResistance",t.rollingResistance},{"airDensity",t.airDensity}});
    object("style",{{"airtime",d.request.style.airtime},{"signatureRollDegrees",d.request.style.signatureRollDegrees},{"returnStyle",double(d.request.style.returnStyle)},{"automaticTrims",double(d.request.style.automaticTrims)}});
    o<<",\"targets\":{\"heightMeters\":";number(o,d.request.targets.height);o<<",\"speedMps\":";number(o,d.request.targets.speed);o<<",\"inversionHeightMeters\":";number(o,d.request.targets.inversionHeight);o<<",\"launchSeconds\":";number(o,d.request.targets.launchSeconds);o<<",\"referenceExposure\":";number(o,d.request.targets.referenceExposure);o<<",\"referenceId\":"<<quote(d.request.targets.referenceId)<<"}";
    o<<','<<quote("trimBrakes")<<":[";bool trimComma=false;
    for(const auto& trim:d.simulation.trims){if(trimComma)o<<',';trimComma=true;const auto& op=d.operations[trim.operation];
        o<<'{'<<quote("operation")<<':'<<trim.operation<<','<<quote("start")<<':'<<op.start<<','<<quote("end")<<':'<<op.end<<','<<quote("sensorTime")<<':';number(o,trim.sensorTime);
        o<<','<<quote("sensedSpeed")<<':';number(o,trim.sensedSpeed);o<<','<<quote("thresholdSpeed")<<':'<<op.targetSpeed<<','<<quote("deployment")<<':'<<trim.deployment<<','<<quote("energyJoules")<<':'<<trim.energyJoules<<','<<quote("peakPowerWatts")<<':'<<trim.peakPowerWatts<<'}';}
    o<<"],"<<quote("metrics")<<":{";
    const auto& m=d.simulation.metrics;
    const std::pair<const char*,double> fields[]={{"maxSpeed",m.maxSpeed},{"heightAboveStation",m.heightAboveStation},{"maxGroundHeight",m.maxGroundHeight},{"verticalRelief",m.verticalRelief},{"inversionGroundHeight",m.inversionGroundHeight},{"launchTo180",m.launchTo180},{"exposure10Seconds",m.exposure10Seconds},{"minVerticalG",m.minVerticalG},{"maxVerticalG",m.maxVerticalG},{"maxLateralG",m.maxLateralG},{"maxLongitudinalG",m.maxLongitudinalG},{"maxJerkGps",m.maxJerkGps},{"maxJerkDistance",m.maxJerkDistance},{"duration",m.duration},{"minGroundClearance",m.minGroundClearance},{"driveWorkPerMass",m.driveWorkPerMass},{"brakeWorkPerMass",m.brakeWorkPerMass},{"lossWorkPerMass",m.lossWorkPerMass},{"maxEnergyResidual",m.maxEnergyResidual},{"peakDrivePowerWatts",m.peakDrivePowerWatts},{"peakBrakePowerWatts",m.peakBrakePowerWatts}};
    bool first=true;for(auto [key,value]:fields){if(!first)o<<',';first=false;o<<quote(key)<<':';number(o,value);}o<<"},\"inversionDimensionMethod\":\"Canonical labeled element extrema; entry-forward horizontal axis, not a loop diameter\",\"inversionDimensions\":[";
    bool dimensionComma=false;for(const auto& x:d.inversionDimensions){if(dimensionComma)o<<',';dimensionComma=true;o<<'{';bool fieldComma=false;
        for(auto [key,value]:std::vector<std::pair<const char*,double>>{{"startDistance",x.startDistance},{"endDistance",x.endDistance},{"pathLength",x.pathLength},{"verticalMinimum",x.verticalMinimum},{"verticalMaximum",x.verticalMaximum},{"verticalExtent",x.verticalExtent},{"forwardExtent",x.forwardExtent},{"lateralExtent",x.lateralExtent}}){if(fieldComma)o<<',';fieldComma=true;o<<quote(key)<<':';number(o,value);}
        o<<",\"role\":"<<quote(roleName(x.role))<<",\"recipeId\":"<<quote(x.recipeId)<<",\"wrapsSeam\":"<<(x.wrapsSeam?"true":"false")<<",\"horizontalAxisFallback\":"<<(x.horizontalAxisFallback?"true":"false")<<",\"horizontalForward\":["<<x.horizontalForward.x<<','<<x.horizontalForward.y<<','<<x.horizontalForward.z<<"],\"horizontalRight\":["<<x.horizontalRight.x<<','<<x.horizontalRight.y<<','<<x.horizontalRight.z<<"]}";
    }
    o<<"],\"errors\":[";first=true;
    for(const auto* r:{&d.report,&d.simulation.report})for(const auto& f:r->errors){if(!first)o<<',';first=false;o<<"{\"code\":"<<quote(f.code)<<",\"message\":"<<quote(f.message)<<",\"distance\":";number(o,f.distance);o<<",\"actual\":";number(o,f.actual);o<<",\"limit\":";number(o,f.limit);o<<'}';}
    o<<"],\"reference\":"<<referenceReportJson(d.request.targets)<<",\"seatStatistics\":[";
    for(int seat=0;seat<3;++seat){if(seat)o<<',';const auto& st=m.seats[seat];o<<"{\"exposure10Seconds\":";number(o,st.exposure10Seconds);o<<",\"airtimeBelowZeroSeconds\":";number(o,st.airtimeBelowZeroSeconds);o<<",\"positiveAbove2Seconds\":";number(o,st.positiveAbove2Seconds);o<<",\"positiveAbove3Seconds\":";number(o,st.positiveAbove3Seconds);o<<",\"positiveAbove4Seconds\":";number(o,st.positiveAbove4Seconds);o<<",\"longestAirtimeSeconds\":";number(o,st.longestAirtimeSeconds);o<<",\"longestAbove2Seconds\":";number(o,st.longestAbove2Seconds);o<<",\"longestAbove3Seconds\":";number(o,st.longestAbove3Seconds);o<<",\"longestAbove4Seconds\":";number(o,st.longestAbove4Seconds);o<<",\"maxInertialJerkMps3\":";number(o,st.maxInertialJerk);o<<",\"maxAngularVelocityRadps\":";number(o,st.maxAngularVelocity);o<<",\"maxAngularAccelerationRadps2\":";number(o,st.maxAngularAcceleration);o<<",\"maxAngularJerkRadps3\":";number(o,st.maxAngularJerk);o<<",\"maxAngularJerkDistance\":";number(o,st.maxAngularJerkDistance);o<<",\"maxLateralRateDistance\":";number(o,st.maxLateralRateDistance);o<<",\"axes\":[";for(int axis=0;axis<3;++axis){if(axis)o<<',';const auto& x=st.axes[axis];o<<'{';bool comma=false;for(auto [key,value]:std::vector<std::pair<const char*,double>>{{"minG",x.minG},{"maxG",x.maxG},{"meanG",x.meanG},{"maxRateGps",x.maxRateGps},{"mean1sMin",x.mean1sMin},{"mean1sMax",x.mean1sMax},{"mean10sMin",x.mean10sMin},{"mean10sMax",x.mean10sMax}}){if(comma)o<<',';comma=true;o<<quote(key)<<':';number(o,value);}o<<'}';}o<<"]}";}
    o<<"],\"motionAudit\":"<<motionReportJson(d)<<",\"componentRateAssessment\":{\"vertical\":\"provisional\",\"lateral\":"<<quote(std::isfinite(l.maxLateralRateGps)?"provisional":"unassessed")<<",\"longitudinal\":"<<quote(std::isfinite(l.maxLongitudinalRateGps)?"provisional":"unassessed")<<",\"lateralLimitGps\":";number(o,l.maxLateralRateGps);o<<",\"longitudinalLimitGps\":";number(o,l.maxLongitudinalRateGps);o<<"},\"statisticsMethod\":\"Rider-axis specific force from analytic canonical frame derivatives; all component rates and inertial/angular jerk use analytic spatial derivatives with a native-step actuator acceleration derivative; means and durations use zero-order sample intervals; strongest-ten-second exposure uses fixed-step integration\",\"warnings\":[";first=true;for(const auto& w:d.report.warnings){if(!first)o<<',';first=false;o<<quote(w);}o<<"]}";return o.str();
}
bool saveDesign(const Design& source,const std::string& path,std::string& error,Cancel cancel,Progress progress){
    WorkRecorder work(std::move(progress),WorkPhase::Serialization);
    if(!source.accepted()){error="REJECTED_DESIGN: only a completed, validated design may be saved";return false;}
    for(const auto& operation:source.operations)if(!validDriveParameters(operation)){error="DRIVE_CONFIG: Invalid explicit drive operation";return false;}
    try{
        const Design& d=source;
        work.enter(WorkPhase::Serialization,d.candidate,"Writing unchanged accepted revision");
        const auto payload=designPayload(d,cancel);
        if(!d.acceptedPayload_||payload!=*d.acceptedPayload_||!d.acceptedCache_||cachePayload(d.track)!=*d.acceptedCache_){
            error="REVALIDATION_REQUIRED: edits invalidate the accepted in-memory revision; compile or load it through fresh validation";return false;
        }
        if(cancel&&cancel()){error="CANCELLED";return false;}
        // Each writer exclusively creates its own same-directory temporary.
        // Sharing '<destination>.tmp' lets one writer commit another's payload.
        struct TemporaryCleanup {
            std::filesystem::path path;std::FILE* stream{};bool armed{};
            ~TemporaryCleanup(){if(stream)std::fclose(stream);if(armed){std::error_code ignored;std::filesystem::remove(path,ignored);}}
        } cleanup;
        static std::atomic<uint64_t> nextTemporary{0};
        const auto stamp=std::chrono::steady_clock::now().time_since_epoch().count();
        for(int attempt=0;attempt<32;++attempt){
#ifdef _WIN32
            const auto process=GetCurrentProcessId();
#else
            const auto process=getpid();
#endif
            auto temporary=utf8path(path+".tmp."+std::to_string(process)+"."+std::to_string(stamp)+"."+std::to_string(nextTemporary.fetch_add(1)));
#ifdef _WIN32
            int descriptor=-1;
            const int openError=_wsopen_s(&descriptor,temporary.c_str(),_O_WRONLY|_O_CREAT|_O_EXCL|_O_BINARY,_SH_DENYRW,_S_IREAD|_S_IWRITE);
#else
            int descriptor=::open(temporary.c_str(),O_WRONLY|O_CREAT|O_EXCL,0600);
            const int openError=descriptor<0?errno:0;
#endif
            if(descriptor<0){if(openError==EEXIST)continue;error="Cannot open temporary save file";return false;}
            cleanup.path=std::move(temporary);cleanup.armed=true;
#ifdef _WIN32
            cleanup.stream=_fdopen(descriptor,"wb");
            if(!cleanup.stream)_close(descriptor);
#else
            cleanup.stream=fdopen(descriptor,"wb");
            if(!cleanup.stream)::close(descriptor);
#endif
            break;
        }
        if(!cleanup.stream){error="Cannot open temporary save file";return false;}
        const std::string header="COASTER 6 "+std::to_string(payload.size())+" "+std::to_string(checksum(payload))+"\n";
        const bool written=std::fwrite(header.data(),1,header.size(),cleanup.stream)==header.size()&&
            std::fwrite(payload.data(),1,payload.size(),cleanup.stream)==payload.size();
        const int closed=std::fclose(cleanup.stream);cleanup.stream=nullptr;
        if(!written||closed!=0){error="Cannot write save file";return false;}
        // Cooperative commit boundary: cancellation through this check preserves
        // the destination. Once replacement succeeds, report success even if a
        // later cancellation request arrives; the committed save is already real.
        if(cancel&&cancel()){error="CANCELLED";return false;}
        // A completed temp file is renamed; never write partial geometry to the destination.
        #ifdef _WIN32
        if(!MoveFileExW(cleanup.path.c_str(),utf8path(path).c_str(),MOVEFILE_REPLACE_EXISTING|MOVEFILE_WRITE_THROUGH)){error="Atomic save replacement failed (Windows error "+std::to_string(GetLastError())+")";return false;}
#else
        std::filesystem::rename(cleanup.path,utf8path(path));
#endif
        cleanup.armed=false;
        return true;
    }catch(const std::exception& e){error=e.what();return false;}
}
static bool readDesign(const std::string& path,Design& out,std::string& error,Cancel cancel,Progress progress,bool retainRejected){
    WorkRecorder work(std::move(progress),WorkPhase::Parsing);
    work.enter(WorkPhase::Parsing,0,"Reading saved ride");
    try{
        std::ifstream f(utf8path(path),std::ios::binary);if(!f){error="Cannot open design file";return false;}
        std::string magic;int schema;size_t bytes;uint64_t expected;
        if(!(f>>magic>>schema>>bytes>>expected)||magic!="COASTER"||schema!=6||bytes>64*1024*1024){error="Invalid design header or unsupported schema";return false;}if(f.get()!='\n'){error="Invalid design header terminator";return false;}
        std::string payload(bytes,'\0');f.read(payload.data(),std::streamsize(bytes));if(size_t(f.gcount())!=bytes||f.peek()!=EOF||checksum(payload)!=expected){error="Design checksum/length mismatch";return false;}
        Design d;d.track.authoredGeometry=d.track.authoredFrame=true;std::istringstream p(payload);p.imbue(std::locale::classic());std::string version;int terrain=-1;auto& r=d.request;
        p>>std::quoted(version)>>r.seed>>terrain>>r.maxCandidates>>r.simulationStep;
        if(!supportedGeneratorVersion(version)){error="Unsupported generator version";return false;}
        if(terrain<0||terrain>int(TerrainKind::Highlands)){error="Unsupported saved terrain kind";return false;}r.terrain.kind=TerrainKind(terrain);d.generationVersion=version;
        bool hasReference=false;p>>r.targets.height>>r.targets.speed>>r.targets.inversionHeight>>r.targets.launchSeconds>>r.targets.requireIntensity>>hasReference>>r.targets.referenceExposure>>std::quoted(r.targets.referenceId);if(!hasReference)r.targets.referenceExposure=std::numeric_limits<double>::quiet_NaN();
        auto& l=r.limits;l.maxLateralRateGps=NAN;l.maxLongitudinalRateGps=NAN;p>>l.minVerticalG>>l.maxVerticalG>>l.maxLateralG>>l.maxLongitudinalG>>l.maxJerkGps>>l.minClearance;
        auto& t=r.train;p>>t.cars>>t.carMass>>t.spacing>>t.seatHeight>>t.dragCdA>>t.rollingResistance>>t.airDensity;
        size_t knots,ops,supports;p>>std::quoted(d.topology)>>d.candidate>>d.track.closed>>knots>>ops>>supports;
        if(!p||knots<4||knots>200000||ops>1000||supports>20000||!d.track.closed){error="Invalid design metadata or size bounds";return false;}
        for(size_t i=0;i<knots;++i){if((i&255)==0&&cancel&&cancel()){error="CANCELLED";return false;}Knot k;int element=-1;vec(p,k.position);vec(p,k.tangent);vec(p,k.curvature);vec(p,k.third);vec(p,k.fourth);vec(p,k.up);vec(p,k.upFirst);vec(p,k.upSecond);vec(p,k.upThird);p>>k.bank>>element;if(element<0||element>7){error="Invalid element";return false;}k.element=Element(element);d.track.knots.push_back(k);}
        for(size_t i=0;i<ops;++i){Operation op;int kind=-1;p>>op.start>>op.end>>kind>>op.targetSpeed>>op.maxForce>>op.maxPower>>op.rampSeconds;p>>op.stopDeceleration>>op.stopOffset>>op.exitFadeMeters;if(kind<0||kind>4){error="Invalid operation kind";return false;}op.kind=DriveKind(kind);d.operations.push_back(op);}
        size_t totalMembers=0;
        for(size_t i=0;i<supports;++i){
            if(cancel&&cancel()){error="CANCELLED";return false;}
            Support s;vec(p,s.base);vec(p,s.top);vec(p,s.attachment);p>>s.hasAttachment>>s.trackDistance;
            size_t count=0;p>>count;
            if(!p||count>maxSupportMembers||count>maxTotalSupportMembers-totalMembers){error="Support member budget exceeded or malformed count";return false;}
            totalMembers+=count;s.members.reserve(count);
            for(size_t j=0;j<count;++j){
                if(cancel&&cancel()){error="CANCELLED";return false;}
                SupportMember m;int kind=-1;vec(p,m.base);vec(p,m.top);p>>m.radiusBase>>m.radiusTop>>kind>>m.spineContact;
                if(!p||kind<0||kind>1){error="Malformed support member";return false;}m.kind=SupportMemberKind(kind);s.members.push_back(m);
            }
            d.supports.push_back(std::move(s));
        }
        if(!p){error="Malformed design payload";return false;}if(!parseExtensions(p,d,error,cancel))return false;p>>std::ws;if(!p.eof()){error="Unexpected trailing payload";return false;}
        if(!recheck(d,cancel,&work)){error=d.simulation.cancelled?"CANCELLED":"REVALIDATION_FAILED: saved geometry or physics is not accepted";
            for(const auto* report:{&d.report,&d.simulation.report})if(!report->errors.empty()){const auto& finding=report->errors.front();error+=" | "+finding.code+" at "+std::to_string(finding.distance)+" m: "+finding.message+" actual="+std::to_string(finding.actual)+" limit="+std::to_string(finding.limit);break;}
            d.timings=work.snapshot();if(retainRejected)out=std::move(d);return false;}
        d.timings=work.snapshot();out=std::move(d);return true;
    }catch(const std::exception& e){error=e.what();return false;}
}
bool loadDesign(const std::string& path,Design& out,std::string& error,Cancel cancel,Progress progress){
    return readDesign(path,out,error,std::move(cancel),std::move(progress),false);
}
Design inspectDesign(const std::string& path,std::string& error,Cancel cancel,Progress progress){
    Design result;
    if(!readDesign(path,result,error,std::move(cancel),std::move(progress),true)&&result.report.valid())result.report.fail("DESIGN_READ",error);
    return result;
}
}
