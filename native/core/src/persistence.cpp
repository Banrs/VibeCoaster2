#include "coaster/coaster.hpp"
#include "coaster/force_envelope.hpp"
#include <fstream>
#include <iomanip>
#include <sstream>
#include <filesystem>
#include <set>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cstdio>
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
bool supportedVersion(const std::string& version){return version==generatorVersion;}
uint64_t checksum(const std::string& s){uint64_t h=14695981039346656037ull;for(unsigned char c:s){h^=c;h*=1099511628211ull;}return h;}
std::string quote(const std::string& s){std::ostringstream o;o<<'"';for(unsigned char c:s){switch(c){case '"':o<<"\\\"";break;case '\\':o<<"\\\\";break;case '\n':o<<"\\n";break;case '\r':o<<"\\r";break;case '\t':o<<"\\t";break;default:if(c<32)o<<"\\u"<<std::hex<<std::setw(4)<<std::setfill('0')<<int(c)<<std::dec;else o<<c;}}o<<'"';return o.str();}
void number(std::ostream& o,double x){if(std::isfinite(x))o<<x;else o<<"null";}
void forceEnvelopeJson(std::ostream& o,const SimulationResult& simulation,double step){
    o<<",\"forceEnvelope\":{\"profile\":"<<quote(forceEnvelopeProfile)
     <<",\"modelRequirements\":"<<quote(forceEnvelopeModelRequirements)
     <<",\"modelRequirementsStatus\":\"design requirements; restraint verification pending\""
     <<",\"scope\":\"Historical force assessment; not complete F2291 certification\",\"sampleRateHz\":";number(o,1/step);
    o<<",\"filter\":\"5 Hz fourth-order single-pass Butterworth, steady-state initialized\""
     <<",\"onset\":\"Centered 100 ms least-squares slope on each filtered rider axis\""
     <<",\"directionOrder\":[\"+z\",\"-z\",\"+y\",\"-y\",\"+x\",\"-x\"],\"pairOrder\":[\"zy\",\"zx\",\"yx\"],\"seatOrder\":[\"front\",\"middle\",\"rear\"],\"seats\":[";
    auto value=[&](const ForceEnvelopeCase& c){
        o<<"{\"utilization\":";number(o,c.utilization);o<<",\"actual\":";number(o,c.actual);o<<",\"limit\":";number(o,c.limit);
        o<<",\"startSeconds\":";number(o,c.startSeconds);o<<",\"durationSeconds\":";number(o,c.durationSeconds);o<<'}';
    };
    auto cases=[&](const char* name,const auto& values){o<<','<<quote(name)<<":[";bool comma=false;for(const auto& c:values){if(comma)o<<',';comma=true;value(c);}o<<']';};
    for(size_t seat=0;seat<simulation.forceEnvelope.size();++seat){
        if(seat)o<<',';const auto& f=simulation.forceEnvelope[seat];
        o<<"{\"performed\":"<<(f.performed?"true":"false")<<",\"passed\":"<<(f.performed&&!f.cancelled&&f.report.valid()?"true":"false")<<",\"axes\":[";
        for(size_t axis=0;axis<3;++axis){if(axis)o<<',';const auto& a=f.axes[axis];
            o<<"{\"minimumG\":";number(o,a.minimumG);o<<",\"maximumG\":";number(o,a.maximumG);
            o<<",\"minimumOnsetGps\":";number(o,a.minimumOnsetGps);o<<",\"maximumOnsetGps\":";number(o,a.maximumOnsetGps);
            o<<",\"minimumOnsetTimeSeconds\":";number(o,a.minimumOnsetTimeSeconds);o<<",\"maximumOnsetTimeSeconds\":";number(o,a.maximumOnsetTimeSeconds);o<<'}';
        }
        o<<']';cases("directionalG",f.directional);cases("pairedSquaredUtilization",f.paired);
        cases("horizontalReversalG",f.horizontalReversal);cases("durationExtentSeconds",f.durationExtent);
        o<<",\"reducedPositiveG\":";value(f.reducedPositive);o<<",\"zeroToTwoSeconds\":";value(f.zeroToTwo);
        o<<",\"enhancedLongitudinalOnsetGps\":";value(f.enhancedLongitudinalOnset);
        o<<",\"reducedPositiveFromSeconds\":";number(o,f.reducedPositiveFromSeconds);o<<'}';
    }
    o<<"]}";
}
void vec(std::ostream& o,Vec3 v){o<<v.x<<' '<<v.y<<' '<<v.z<<' ';}
void vec(std::istream& i,Vec3& v){i>>v.x>>v.y>>v.z;}
std::string extensionTail(const Design& d,Cancel cancel){
    const auto& r=d.request;
    std::vector<std::pair<std::string,std::string>> blocks;
    {
        const auto& terrain=r.terrain;std::ostringstream b;b.imbue(std::locale::classic());b<<std::setprecision(17);
        b<<terrain.verticalScale<<' '<<terrain.horizontalScale<<' '<<terrain.offsetX<<' '<<terrain.offsetY<<' '<<terrain.headingRadians<<' '<<terrain.cliffHeight<<' '<<terrain.cliffWidth<<'\n';
        blocks.push_back({"TERRAIN_PROFILE",b.str()});
    }
    if(d.station.enabled)blocks.push_back({"STATION",stationPayload(d.station,cancel)});
    if(r.targets.reference.processed)blocks.push_back({"REFERENCE",serializeReference(r.targets.reference)});
    if(std::isfinite(r.limits.maxLateralRateGps)||std::isfinite(r.limits.maxLongitudinalRateGps)){
        std::ostringstream b;b.imbue(std::locale::classic());b<<std::setprecision(17);
        for(double x:{r.limits.maxLateralRateGps,r.limits.maxLongitudinalRateGps})b<<std::isfinite(x)<<' '<<(std::isfinite(x)?x:0)<<' ';b<<'\n';blocks.push_back({"AXIS_RATE_LIMITS",b.str()});
    }
    std::ostringstream out;out<<"EXTENSIONS "<<blocks.size()<<'\n';for(const auto& [name,bytes]:blocks)out<<name<<" 1 "<<bytes.size()<<'\n'<<bytes;return out.str();
}
bool parseExtensions(std::istream& p,Design& out,std::string& error,Cancel cancel){
    p>>std::ws;if(p.eof()){error="Missing COASTER5 extension directory";return false;}
    Design result=out;auto& request=result.request;std::string tag;size_t count;p>>tag>>count;
    if(!p||tag!="EXTENSIONS"||count>8){error="Invalid extension directory";return false;}
    std::set<std::string> seen;
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
            auto terrain=request.terrain;std::istringstream b(bytes);b.imbue(std::locale::classic());
            b>>terrain.verticalScale>>terrain.horizontalScale>>terrain.offsetX>>terrain.offsetY>>terrain.headingRadians>>terrain.cliffHeight>>terrain.cliffWidth;
            if(!b||!terrain.valid()){error="Invalid terrain profile";return false;}b>>std::ws;if(!b.eof()){error="Trailing terrain profile data";return false;}request.terrain=terrain;
        }else if(name=="STATION"){
            if(!parseStationPayload(bytes,result.station,error,cancel))return false;
        }else{error="Unknown extension: "+name;return false;}
    }
    if(!seen.count("TERRAIN_PROFILE")){error="Missing required terrain profile";return false;}
    p>>std::ws;if(!p.eof()){error="Unexpected data after extensions";return false;}out=std::move(result);return true;
}
bool recheck(Design& d,Cancel cancel){if(!supportedVersion(d.generationVersion)){d.report.fail("GENERATOR_VERSION","Unsupported generation provenance");return false;}d.report=validateRequest(d.request);if(!d.report.valid())return false;d.track.rebuild();d.inversionDimensions=measureInversionDimensions(d.track,cancel);d.report=validateGeometry(d.track,d.request.terrain,d.request.limits,d.request.train,d.supports,cancel);auto structures=validateDesignStructures(d,cancel);d.report.errors.insert(d.report.errors.end(),structures.errors.begin(),structures.errors.end());d.convergence={};d.simulation=simulate(d.track,d.operations,d.request.train,d.request.simulationStep,cancel);evaluateTargets(d);verifyConvergence(d,cancel);return d.accepted();}
}
std::string reportJson(const Design& d){
    std::ostringstream o;o.imbue(std::locale::classic());o<<std::setprecision(17);
    o<<"{\"schemaVersion\":1,\"generatorVersion\":"<<quote(d.generationVersion)<<",\"runtimeVersion\":"<<quote(generatorVersion)<<",\"seed\":"<<d.request.seed<<",\"terrain\":"<<quote(d.request.terrain.name())<<",\"preset\":"<<quote(d.request.targets.requireIntensity?"all-records":"physics-proof")<<",\"intensityRequired\":"<<(d.request.targets.requireIntensity?"true":"false")<<",\"accepted\":"<<(d.accepted()?"true":"false")<<",\"completed\":"<<(d.simulation.completed?"true":"false")<<",\"cancelled\":"<<(d.simulation.cancelled?"true":"false")<<",\"candidate\":"<<d.candidate<<",\"topology\":"<<quote(d.topology)<<",\"lengthMeters\":";number(o,d.track.length);
    o<<",\"stationEnabled\":"<<(d.station.enabled?"true":"false")<<",\"stationPartCount\":"<<d.station.boxes.size();
    o<<",\"maxCandidates\":"<<d.request.maxCandidates<<",\"simulationStep\":";number(o,d.request.simulationStep);
    const auto& c=d.convergence;
    o<<",\"convergence\":{\"performed\":"<<(c.performed?"true":"false")<<",\"passed\":"<<(c.passed?"true":"false")<<",\"coarseStep\":";number(o,c.coarseStep);o<<",\"fineStep\":";number(o,c.fineStep);
    o<<",\"maxSpeedRelativeError\":";number(o,c.maxSpeedRelativeError);o<<",\"maxForceRelativeError\":";number(o,c.maxForceRelativeError);o<<",\"metrics\":[";
    bool convergenceComma=false;for(const auto& metric:c.metrics){if(convergenceComma)o<<',';convergenceComma=true;o<<"{\"name\":"<<quote(metric.name)<<",\"coarse\":";number(o,metric.coarse);o<<",\"fine\":";number(o,metric.fine);o<<",\"absoluteDifference\":";number(o,metric.absoluteDifference);o<<",\"tolerance\":";number(o,metric.tolerance);o<<'}';}o<<"]}";
    o<<",\"intensityComparison\":\"Maximum over physical front/middle/rear seats of the strongest ten-second integral of max(vertical_g,0); configured reference identity is external\"";
    forceEnvelopeJson(o,d.simulation,d.request.simulationStep);
    auto object=[&](const char* name,const std::vector<std::pair<const char*,double>>& fields){o<<','<<quote(name)<<":{";bool first=true;for(auto [key,value]:fields){if(!first)o<<',';first=false;o<<quote(key)<<':';number(o,value);}o<<'}';};
    const auto& terrain=d.request.terrain;object("terrainProfile",{{"verticalScale",terrain.verticalScale},{"horizontalScale",terrain.horizontalScale},{"offsetXMeters",terrain.offsetX},{"offsetYMeters",terrain.offsetY},{"headingRadians",terrain.headingRadians},{"cliffHeightMeters",terrain.cliffHeight},{"cliffWidthMeters",terrain.cliffWidth},{"globalSlopeBound",terrain.slopeBound()}});
    const auto& l=d.request.limits;object("limits",{{"minVerticalG",l.minVerticalG},{"maxVerticalG",l.maxVerticalG},{"maxLateralG",l.maxLateralG},{"maxLongitudinalG",l.maxLongitudinalG},{"maxJerkGps",l.maxJerkGps},{"minClearance",l.minClearance},{"maxLateralRateGps",l.maxLateralRateGps},{"maxLongitudinalRateGps",l.maxLongitudinalRateGps}});
    const auto& t=d.request.train;object("train",{{"cars",double(t.cars)},{"carMass",t.carMass},{"spacing",t.spacing},{"seatHeight",t.seatHeight},{"dragCdA",t.dragCdA},{"rollingResistance",t.rollingResistance},{"airDensity",t.airDensity}});
    o<<",\"targets\":{\"heightMeters\":";number(o,d.request.targets.height);o<<",\"speedMps\":";number(o,d.request.targets.speed);o<<",\"inversionHeightMeters\":";number(o,d.request.targets.inversionHeight);o<<",\"launchSeconds\":";number(o,d.request.targets.launchSeconds);o<<",\"referenceExposure\":";number(o,d.request.targets.referenceExposure);o<<",\"referenceId\":"<<quote(d.request.targets.referenceId)<<"},\"metrics\":{";
    const auto& m=d.simulation.metrics;
    const std::pair<const char*,double> fields[]={{"maxSpeed",m.maxSpeed},{"heightAboveStation",m.heightAboveStation},{"maxGroundHeight",m.maxGroundHeight},{"verticalRelief",m.verticalRelief},{"inversionGroundHeight",m.inversionGroundHeight},{"launchTo180",m.launchTo180},{"exposure10Seconds",m.exposure10Seconds},{"minVerticalG",m.minVerticalG},{"maxVerticalG",m.maxVerticalG},{"maxLateralG",m.maxLateralG},{"maxLongitudinalG",m.maxLongitudinalG},{"maxJerkGps",m.maxJerkGps},{"maxJerkDistance",m.maxJerkDistance},{"duration",m.duration},{"minGroundClearance",m.minGroundClearance}};
    bool first=true;for(auto [key,value]:fields){if(!first)o<<',';first=false;o<<quote(key)<<':';number(o,value);}o<<"},\"inversionDimensionMethod\":\"Canonical labeled element extrema; entry-forward horizontal axis, not a loop diameter\",\"inversionDimensions\":[";
    bool dimensionComma=false;for(const auto& x:d.inversionDimensions){if(dimensionComma)o<<',';dimensionComma=true;o<<'{';bool fieldComma=false;
        for(auto [key,value]:std::vector<std::pair<const char*,double>>{{"startDistance",x.startDistance},{"endDistance",x.endDistance},{"pathLength",x.pathLength},{"verticalMinimum",x.verticalMinimum},{"verticalMaximum",x.verticalMaximum},{"verticalExtent",x.verticalExtent},{"forwardExtent",x.forwardExtent},{"lateralExtent",x.lateralExtent}}){if(fieldComma)o<<',';fieldComma=true;o<<quote(key)<<':';number(o,value);}
        o<<",\"wrapsSeam\":"<<(x.wrapsSeam?"true":"false")<<",\"horizontalAxisFallback\":"<<(x.horizontalAxisFallback?"true":"false")<<",\"horizontalForward\":["<<x.horizontalForward.x<<','<<x.horizontalForward.y<<','<<x.horizontalForward.z<<"],\"horizontalRight\":["<<x.horizontalRight.x<<','<<x.horizontalRight.y<<','<<x.horizontalRight.z<<"]}";
    }
    o<<"],\"errors\":[";first=true;
    for(const auto* r:{&d.report,&d.simulation.report})for(const auto& f:r->errors){if(!first)o<<',';first=false;o<<"{\"code\":"<<quote(f.code)<<",\"message\":"<<quote(f.message)<<",\"distance\":";number(o,f.distance);o<<",\"actual\":";number(o,f.actual);o<<",\"limit\":";number(o,f.limit);o<<'}';}
    o<<"],\"reference\":"<<referenceReportJson(d.request.targets)<<",\"seatStatistics\":[";
    for(int seat=0;seat<3;++seat){if(seat)o<<',';const auto& st=m.seats[seat];o<<"{\"exposure10Seconds\":";number(o,st.exposure10Seconds);o<<",\"airtimeBelowZeroSeconds\":";number(o,st.airtimeBelowZeroSeconds);o<<",\"positiveAbove2Seconds\":";number(o,st.positiveAbove2Seconds);o<<",\"positiveAbove3Seconds\":";number(o,st.positiveAbove3Seconds);o<<",\"positiveAbove4Seconds\":";number(o,st.positiveAbove4Seconds);o<<",\"longestAirtimeSeconds\":";number(o,st.longestAirtimeSeconds);o<<",\"longestAbove2Seconds\":";number(o,st.longestAbove2Seconds);o<<",\"longestAbove3Seconds\":";number(o,st.longestAbove3Seconds);o<<",\"longestAbove4Seconds\":";number(o,st.longestAbove4Seconds);o<<",\"axes\":[";for(int axis=0;axis<3;++axis){if(axis)o<<',';const auto& x=st.axes[axis];o<<'{';bool comma=false;for(auto [key,value]:std::vector<std::pair<const char*,double>>{{"minG",x.minG},{"maxG",x.maxG},{"meanG",x.meanG},{"maxRateGps",x.maxRateGps},{"mean1sMin",x.mean1sMin},{"mean1sMax",x.mean1sMax},{"mean10sMin",x.mean10sMin},{"mean10sMax",x.mean10sMax}}){if(comma)o<<',';comma=true;o<<quote(key)<<':';number(o,value);}o<<'}';}o<<"]}";}
    o<<"],\"componentRateAssessment\":{\"vertical\":\"provisional\",\"lateral\":"<<quote(std::isfinite(l.maxLateralRateGps)?"provisional":"unassessed")<<",\"longitudinal\":"<<quote(std::isfinite(l.maxLongitudinalRateGps)?"provisional":"unassessed")<<",\"lateralLimitGps\":";number(o,l.maxLateralRateGps);o<<",\"longitudinalLimitGps\":";number(o,l.maxLongitudinalRateGps);o<<"},\"statisticsMethod\":\"Rider-axis specific force from analytic canonical frame derivatives; vertical rate is analytic and lateral/longitudinal rates use integration-step differences; means and durations use zero-order sample intervals; strongest-ten-second exposure uses fixed-step integration\",\"warnings\":[";first=true;for(const auto& w:d.report.warnings){if(!first)o<<',';first=false;o<<quote(w);}o<<"]}";return o.str();
}
bool saveDesign(const Design& source,const std::string& path,std::string& error,Cancel cancel){
    if(!source.accepted()){error="REJECTED_DESIGN: only a completed, validated design may be saved";return false;}
    try{
        Design d=source;if(!recheck(d,cancel)){error="REVALIDATION_FAILED: geometry or independent replay rejected this design";return false;}
        std::ostringstream p;p.imbue(std::locale::classic());p<<std::setprecision(17);
        const auto& r=d.request;p<<std::quoted(d.generationVersion)<<' '<<r.seed<<' '<<int(r.terrain.kind)<<' '<<r.maxCandidates<<' '<<r.simulationStep<<'\n';
        p<<r.targets.height<<' '<<r.targets.speed<<' '<<r.targets.inversionHeight<<' '<<r.targets.launchSeconds<<' '<<r.targets.requireIntensity<<' '<<std::isfinite(r.targets.referenceExposure)<<' '<<(std::isfinite(r.targets.referenceExposure)?r.targets.referenceExposure:0)<<' '<<std::quoted(r.targets.referenceId)<<'\n';
        const auto& l=r.limits;p<<l.minVerticalG<<' '<<l.maxVerticalG<<' '<<l.maxLateralG<<' '<<l.maxLongitudinalG<<' '<<l.maxJerkGps<<' '<<l.minClearance<<'\n';
        const auto& t=r.train;p<<t.cars<<' '<<t.carMass<<' '<<t.spacing<<' '<<t.seatHeight<<' '<<t.dragCdA<<' '<<t.rollingResistance<<' '<<t.airDensity<<'\n';
        p<<std::quoted(d.topology)<<' '<<d.candidate<<' '<<d.track.closed<<' '<<d.track.knots.size()<<' '<<d.operations.size()<<' '<<d.supports.size()<<'\n';
        for(const auto& k:d.track.knots){vec(p,k.position);vec(p,k.tangent);vec(p,k.curvature);vec(p,k.up);p<<k.bank<<' '<<int(k.element)<<'\n';}
        for(const auto& op:d.operations)p<<op.start<<' '<<op.end<<' '<<int(op.kind)<<' '<<op.targetSpeed<<' '<<op.maxForce<<' '<<op.maxPower<<' '<<op.rampSeconds<<' '<<op.stopDeceleration<<' '<<op.stopOffset<<' '<<op.exitFadeMeters<<'\n';
        for(const auto& s:d.supports){vec(p,s.base);vec(p,s.top);vec(p,s.attachment);p<<s.hasAttachment<<' '<<s.trackDistance<<' '<<s.members.size()<<'\n';
            for(const auto& m:s.members){if(cancel&&cancel()){error="CANCELLED";return false;}vec(p,m.base);vec(p,m.top);p<<m.radiusBase<<' '<<m.radiusTop<<' '<<int(m.kind)<<' '<<m.spineContact<<'\n';}}
        p<<extensionTail(d,cancel);
        auto payload=p.str();
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
        const std::string header="COASTER 5 "+std::to_string(payload.size())+" "+std::to_string(checksum(payload))+"\n";
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
bool loadDesign(const std::string& path,Design& out,std::string& error,Cancel cancel){
    try{
        std::ifstream f(utf8path(path),std::ios::binary);if(!f){error="Cannot open design file";return false;}
        std::string magic;int schema;size_t bytes;uint64_t expected;
        if(!(f>>magic>>schema>>bytes>>expected)||magic!="COASTER"||schema!=5||bytes>64*1024*1024){error="Invalid design header or unsupported schema";return false;}if(f.get()!='\n'){error="Invalid design header terminator";return false;}
        std::string payload(bytes,'\0');f.read(payload.data(),std::streamsize(bytes));if(size_t(f.gcount())!=bytes||f.peek()!=EOF||checksum(payload)!=expected){error="Design checksum/length mismatch";return false;}
        Design d;std::istringstream p(payload);p.imbue(std::locale::classic());std::string version;int terrain=-1;auto& r=d.request;
        p>>std::quoted(version)>>r.seed>>terrain>>r.maxCandidates>>r.simulationStep;
        if(!supportedVersion(version)||terrain<0||terrain>2){error="Unsupported generator version or terrain";return false;}r.terrain.kind=TerrainKind(terrain);d.generationVersion=version;
        bool hasReference=false;p>>r.targets.height>>r.targets.speed>>r.targets.inversionHeight>>r.targets.launchSeconds>>r.targets.requireIntensity>>hasReference>>r.targets.referenceExposure>>std::quoted(r.targets.referenceId);if(!hasReference)r.targets.referenceExposure=std::numeric_limits<double>::quiet_NaN();
        auto& l=r.limits;p>>l.minVerticalG>>l.maxVerticalG>>l.maxLateralG>>l.maxLongitudinalG>>l.maxJerkGps>>l.minClearance;
        auto& t=r.train;p>>t.cars>>t.carMass>>t.spacing>>t.seatHeight>>t.dragCdA>>t.rollingResistance>>t.airDensity;
        size_t knots,ops,supports;p>>std::quoted(d.topology)>>d.candidate>>d.track.closed>>knots>>ops>>supports;
        if(!p||knots<4||knots>200000||ops>1000||supports>20000||!d.track.closed){error="Invalid design metadata or size bounds";return false;}
        for(size_t i=0;i<knots;++i){if((i&255)==0&&cancel&&cancel()){error="CANCELLED";return false;}Knot k;int element=-1;vec(p,k.position);vec(p,k.tangent);vec(p,k.curvature);vec(p,k.up);p>>k.bank>>element;if(element<0||element>7){error="Invalid element";return false;}k.element=Element(element);d.track.knots.push_back(k);}
        for(size_t i=0;i<ops;++i){Operation op;int kind=-1;p>>op.start>>op.end>>kind>>op.targetSpeed>>op.maxForce>>op.maxPower>>op.rampSeconds;p>>op.stopDeceleration>>op.stopOffset>>op.exitFadeMeters;if(kind<0||kind>3){error="Invalid operation kind";return false;}op.kind=DriveKind(kind);d.operations.push_back(op);}
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
        if(!recheck(d,cancel)){error=d.simulation.cancelled?"CANCELLED":"REVALIDATION_FAILED: saved geometry or physics is not accepted";
            for(const auto* report:{&d.report,&d.simulation.report})if(!report->errors.empty()){const auto& finding=report->errors.front();error+=" | "+finding.code+" at "+std::to_string(finding.distance)+" m: "+finding.message;break;}return false;}
        out=std::move(d);return true;
    }catch(const std::exception& e){error=e.what();return false;}
}
}
