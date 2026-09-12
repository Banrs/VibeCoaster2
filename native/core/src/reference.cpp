#include "coaster/coaster.hpp"
#include <fstream>
#include <filesystem>
#include <iomanip>
#include <sstream>
#include <set>

namespace coaster {
namespace {
constexpr size_t maxReferenceBytes=1024*1024;
bool textOk(const std::string& s,bool required=true){
    if(s.size()>16384||(required&&s.find_first_not_of(' ')==std::string::npos))return false;
    for(size_t i=0;i<s.size();){unsigned char c=s[i++];if(c<32||c==127)return false;if(c<128)continue;
        unsigned value=0,remaining=0,minimum=0;if(c>=0xc2&&c<=0xdf){value=c&31;remaining=1;minimum=0x80;}else if(c>=0xe0&&c<=0xef){value=c&15;remaining=2;minimum=0x800;}else if(c>=0xf0&&c<=0xf4){value=c&7;remaining=3;minimum=0x10000;}else return false;
        if(i+remaining>s.size())return false;for(unsigned j=0;j<remaining;++j){unsigned char next=s[i++];if((next&0xc0)!=0x80)return false;value=(value<<6)|(next&63);}if(value<minimum||value>0x10ffff||(value>=0xd800&&value<=0xdfff))return false;
    }return true;
}
bool hashOk(const std::string& s){return s.size()==64&&std::all_of(s.begin(),s.end(),[](char c){return(c>='0'&&c<='9')||(c>='a'&&c<='f');});}
bool almost(double a,double b){return std::isfinite(a)&&std::isfinite(b)&&std::abs(a-b)<=1e-10*std::max({1.,std::abs(a),std::abs(b)});}
std::string quotedJson(const std::string& s){std::ostringstream o;bool validText=textOk(s,false);o<<'"';for(unsigned char c:s){if(c=='"'||c=='\\')o<<'\\'<<c;else if(c<32||(!validText&&c>=128))o<<"\\u"<<std::hex<<std::setw(4)<<std::setfill('0')<<int(c)<<std::dec;else o<<c;}o<<'"';return o.str();}
void numberJson(std::ostream& o,double x){if(std::isfinite(x))o<<x;else o<<"null";}
std::string fixed(double n,int digits=2){if(!std::isfinite(n))return "unavailable";std::ostringstream o;o.imbue(std::locale::classic());o<<std::fixed<<std::setprecision(digits)<<n;return o.str();}
double quantileExclusive(const std::vector<double>& x,int quartile){double index=(x.size()+1)*quartile/4.;size_t j=size_t(std::floor(index));return x[j-1]+(x[j]-x[j-1])*(index-j);}
}
ValidationReport validateReference(const Targets& t){
    ValidationReport report;const auto& b=t.reference;
    auto fail=[&](const char* why){report.fail("REFERENCE_METADATA",why);};
    if(!b.processed){if(!b.method.empty()||!b.groupId.empty()||!b.recordings.empty())fail("Unprocessed reference contains typed metadata");return report;}
    if(b.method!="piecewise-linear-positive10s-v1"||b.groupId.rfind("rf-v1:",0)!=0||!hashOk(b.groupId.substr(6)))fail("Unsupported reference method or group identity");
    for(const auto* s:{&b.ride,&b.configuration,&b.seat,&b.device,&b.calibrationId})if(!textOk(*s))fail("Missing or oversized reference condition identity");
    if(b.recordings.size()<3||b.recordings.size()>256){fail("Reference needs 3 to 256 independent eligible recordings");return report;}
    if(!std::isfinite(b.minimum)||b.minimum<=0||!std::isfinite(b.median)||!std::isfinite(b.maximum)||b.minimum>b.median||b.median>b.maximum)fail("Invalid reference spread");
    if(b.hasQuartiles&&(!std::isfinite(b.q1)||!std::isfinite(b.q3)||b.minimum>b.q1||b.q1>b.median||b.median>b.q3||b.q3>b.maximum))fail("Invalid reference quartiles");
    if(t.referenceId!=b.groupId||!almost(t.referenceExposure,b.median))fail("Typed reference disagrees with scalar target or identity");
    std::set<std::string> ids,hashes;std::vector<double> values;
    for(const auto& r:b.recordings){
        if(!textOk(r.recordingId)||!textOk(r.source)||!textOk(r.notes)||!hashOk(r.rawSha256)||!hashOk(r.canonicalSha256)||!hashOk(r.analysisSha256))fail("Incomplete recording provenance");
        if(!ids.insert(r.recordingId).second||!hashes.insert(r.canonicalSha256).second)fail("Reference recordings are not independent");
        if(!std::isfinite(r.sampleRateMinHz)||!std::isfinite(r.sampleRateHz)||!std::isfinite(r.sampleRateMaxHz)||r.sampleRateMinHz<=0||r.sampleRateMinHz>r.sampleRateHz||r.sampleRateHz>r.sampleRateMaxHz||!std::isfinite(r.exposure)||r.exposure<=0)fail("Invalid recording timing or exposure");
        values.push_back(r.exposure);
    }
    std::sort(values.begin(),values.end());size_t n=values.size();double median=(values[(n-1)/2]+values[n/2])*.5;
    if(!almost(b.minimum,values.front())||!almost(b.maximum,values.back())||!almost(b.median,median))fail("Reference summary does not match recording evidence");
    if(!b.hasQuartiles&&(b.q1!=0||b.q3!=0))fail("Unavailable quartiles must have zero placeholders");
    if(serializeReference(b).size()>maxReferenceBytes)fail("Reference metadata exceeds 1 MiB");
    if(b.hasQuartiles!=(n>=4))fail("Quartile availability does not match processor method");
    else if(b.hasQuartiles&&(!almost(b.q1,quantileExclusive(values,1))||!almost(b.q3,quantileExclusive(values,3))))fail("Quartiles do not match recording evidence");
    return report;
}
std::string referenceStatus(const Targets& t){if(t.reference.processed)return validateReference(t).valid()?"processed-eligible; authenticity-not-independently-verified":"invalid-reference-metadata";return std::isfinite(t.referenceExposure)&&!t.referenceId.empty()?"user-configured-unverified":"unavailable";}
std::string serializeReference(const ReferenceBenchmark& b){
    std::ostringstream p;p.imbue(std::locale::classic());p<<std::setprecision(17)<<"COASTER_REFERENCE 1\n"<<std::quoted(b.method)<<' '<<std::quoted(b.groupId)<<'\n';
    p<<std::quoted(b.ride)<<' '<<std::quoted(b.configuration)<<' '<<std::quoted(b.seat)<<' '<<std::quoted(b.device)<<' '<<std::quoted(b.calibrationId)<<'\n';
    p<<b.recordings.size()<<' '<<b.median<<' '<<b.minimum<<' '<<b.maximum<<' '<<b.hasQuartiles<<' '<<b.q1<<' '<<b.q3<<'\n';
    for(const auto& r:b.recordings)p<<std::quoted(r.recordingId)<<' '<<std::quoted(r.rawSha256)<<' '<<std::quoted(r.canonicalSha256)<<' '<<std::quoted(r.analysisSha256)<<' '<<std::quoted(r.source)<<' '<<std::quoted(r.notes)<<' '<<r.sampleRateHz<<' '<<r.sampleRateMinHz<<' '<<r.sampleRateMaxHz<<' '<<r.exposure<<'\n';
    return p.str();
}
bool parseReference(const std::string& bytes,Targets& out,std::string& error){
    if(bytes.empty()||bytes.size()>maxReferenceBytes){error="Reference file is empty or exceeds 1 MiB";return false;}
    std::istringstream p(bytes);p.imbue(std::locale::classic());Targets result=out;ReferenceBenchmark b;std::string magic;int schema;size_t n;
    p>>magic>>schema;if(!p||magic!="COASTER_REFERENCE"||schema!=1){error="Unsupported reference schema";return false;}
    p>>std::quoted(b.method)>>std::quoted(b.groupId)>>std::quoted(b.ride)>>std::quoted(b.configuration)>>std::quoted(b.seat)>>std::quoted(b.device)>>std::quoted(b.calibrationId);
    p>>n>>b.median>>b.minimum>>b.maximum>>b.hasQuartiles>>b.q1>>b.q3;
    if(!p||n<3||n>256){error="Malformed reference or recording count";return false;}
    for(size_t i=0;i<n;++i){ReferenceRecording r;p>>std::quoted(r.recordingId)>>std::quoted(r.rawSha256)>>std::quoted(r.canonicalSha256)>>std::quoted(r.analysisSha256)>>std::quoted(r.source)>>std::quoted(r.notes)>>r.sampleRateHz>>r.sampleRateMinHz>>r.sampleRateMaxHz>>r.exposure;b.recordings.push_back(r);}
    if(!p){error="Malformed recording evidence";return false;}p>>std::ws;if(!p.eof()){error="Unexpected reference payload";return false;}
    b.processed=true;result.referenceExposure=b.median;result.referenceId=b.groupId;result.reference=std::move(b);auto validation=validateReference(result);
    if(!validation.valid()){error=validation.errors.front().message;return false;}out=std::move(result);return true;
}
bool loadReference(const std::string& path,Targets& out,std::string& error){
    try{auto file=std::filesystem::path(std::u8string(reinterpret_cast<const char8_t*>(path.data()),path.size()));std::ifstream f(file,std::ios::binary);if(!f){error="Cannot open reference file";return false;}
        f.seekg(0,std::ios::end);auto n=f.tellg();if(n<=0||n>std::streamoff(maxReferenceBytes)){error="Invalid reference file size";return false;}f.seekg(0);std::string bytes(size_t(n),'\0');if(!f.read(bytes.data(),n)){error="Cannot read reference file";return false;}return parseReference(bytes,out,error);
    }catch(const std::exception& e){error=e.what();return false;}
}
std::string referenceReportJson(const Targets& t){
    std::ostringstream o;o.imbue(std::locale::classic());o<<std::setprecision(12)<<"{\"status\":"<<quotedJson(referenceStatus(t))<<",\"measurementUncertainty\":\"not-quantified\",\"transitionCalibration\":\"unassessed\",\"spreadMeaning\":\"observed recording distribution; not a confidence interval\"";
    const auto& b=t.reference;if(b.processed){o<<",\"method\":"<<quotedJson(b.method)<<",\"groupId\":"<<quotedJson(b.groupId)<<",\"ride\":"<<quotedJson(b.ride)<<",\"configuration\":"<<quotedJson(b.configuration)<<",\"seat\":"<<quotedJson(b.seat)<<",\"device\":"<<quotedJson(b.device)<<",\"calibrationId\":"<<quotedJson(b.calibrationId)<<",\"nEligible\":"<<b.recordings.size()<<",\"median\":";numberJson(o,b.median);o<<",\"min\":";numberJson(o,b.minimum);o<<",\"max\":";numberJson(o,b.maximum);o<<",\"q1\":";numberJson(o,b.hasQuartiles?b.q1:NAN);o<<",\"q3\":";numberJson(o,b.hasQuartiles?b.q3:NAN);o<<",\"recordings\":[";
        bool comma=false;for(const auto& r:b.recordings){if(comma)o<<',';comma=true;o<<"{\"id\":"<<quotedJson(r.recordingId)<<",\"rawSha256\":"<<quotedJson(r.rawSha256)<<",\"canonicalSha256\":"<<quotedJson(r.canonicalSha256)<<",\"analysisSha256\":"<<quotedJson(r.analysisSha256)<<",\"source\":"<<quotedJson(r.source)<<",\"notes\":"<<quotedJson(r.notes)<<",\"sampleRateHz\":";numberJson(o,r.sampleRateHz);o<<",\"sampleRateMinHz\":";numberJson(o,r.sampleRateMinHz);o<<",\"sampleRateMaxHz\":";numberJson(o,r.sampleRateMaxHz);o<<",\"exposure\":";numberJson(o,r.exposure);o<<'}';}o<<']';}
    o<<'}';return o.str();
}
AxisStatistics summarizeAxis(const std::vector<double>& v,double dt){
    AxisStatistics m;if(v.empty()||!std::isfinite(dt)||dt<=0)return m;
    m.minG=*std::min_element(v.begin(),v.end());m.maxG=*std::max_element(v.begin(),v.end());std::vector<double> sum(1,0);
    for(size_t i=0;i<v.size();++i){sum.push_back(sum.back()+v[i]*dt);if(i)m.maxRateGps=std::max(m.maxRateGps,std::abs(v[i]-v[i-1])/dt);}m.meanG=sum.back()/(v.size()*dt);
    auto integral=[&](double t){if(t<=0)return 0.;if(t>=v.size()*dt)return sum.back();size_t i=std::min(v.size()-1,size_t(t/dt));return sum[i]+v[i]*(t-i*dt);};
    auto window=[&](double w,double& low,double& high){double end=v.size()*dt-w;if(end<-1e-10)return;end=std::max(0.,end);low=INFINITY;high=-INFINITY;
        auto sample=[&](double start){if(start<0||start>end)return;double x=(integral(start+w)-integral(start))/w;low=std::min(low,x);high=std::max(high,x);};sample(0);sample(end);for(size_t i=0;i<=v.size();++i){sample(i*dt);sample(i*dt-w);}};
    window(1,m.mean1sMin,m.mean1sMax);window(10,m.mean10sMin,m.mean10sMax);return m;
}
std::vector<std::string> referenceLines(const Targets& t){
    std::vector<std::string> out;out.push_back("Reference: "+referenceStatus(t));
    if(!std::isfinite(t.referenceExposure)){out.push_back("No measured benchmark loaded; intensity comparison unavailable.");return out;}
    const auto& b=t.reference;if(b.processed){out.push_back(b.ride+" | "+b.configuration+" | reference seat: "+b.seat);out.push_back("Median "+fixed(b.median)+" g*s | range "+fixed(b.minimum)+" to "+fixed(b.maximum)+" | n="+std::to_string(b.recordings.size()));out.push_back(b.hasQuartiles?"Recording IQR "+fixed(b.q1)+" to "+fixed(b.q3)+" g*s":"Recording IQR unavailable (n < 4)");out.push_back("Device "+b.device+" | calibration "+b.calibrationId);}
    else out.push_back("Manual target "+fixed(t.referenceExposure)+" g*s | "+t.referenceId);
    out.push_back("Measurement uncertainty not quantified; transition calibration unassessed.");return out;
}
bool ComparisonHistory::commit(const Design& d){if(!d.accepted())return false;previous=current;current={true,d.request.seed,d.request.terrain.name(),d.request.targets.referenceId,referenceStatus(d.request.targets),d.simulation.metrics};return true;}
std::vector<std::string> comparisonLines(const Design& d,const RideSummary* previous){
    const auto& m=d.simulation.metrics;const auto& t=d.request.targets;std::vector<std::string> out={"Current accepted: seed "+std::to_string(d.request.seed)+" / "+d.request.terrain.name()};auto reference=referenceLines(t);out.insert(out.end(),reference.begin(),reference.end());
    out.push_back("Generated exposure: maximum of physical front/middle/rear seats; reference seat shown above.");
    if(std::isfinite(t.referenceExposure)){double target=t.referenceExposure*1.1;out.push_back("10s exposure "+fixed(m.exposure10Seconds)+" / target "+fixed(target)+" g*s | margin "+fixed(m.exposure10Seconds-target)+" ("+fixed((m.exposure10Seconds/target-1)*100)+"%)");}
    else out.push_back("10s exposure "+fixed(m.exposure10Seconds)+" g*s | reference comparison unavailable");
    out.push_back("Height above ground "+fixed(m.maxGroundHeight)+" / "+fixed(t.height)+" m | inversion apex above ground "+fixed(m.inversionGroundHeight)+" / "+fixed(t.inversionHeight)+" m");
    for(size_t i=0;i<std::min<size_t>(8,d.inversionDimensions.size());++i){const auto& x=d.inversionDimensions[i];
        out.push_back("Inversion element "+std::to_string(i+1)+": vertical extent "+fixed(x.verticalExtent)+" m | entry-axis span "+fixed(x.forwardExtent)+" m | lateral span "+fixed(x.lateralExtent)+" m");
    }
    if(d.inversionDimensions.size()>8)out.push_back(std::to_string(d.inversionDimensions.size()-8)+" further inversion elements in the complete report");
    out.push_back("Speed "+fixed(m.maxSpeed*3.6)+" / "+fixed(t.speed*3.6)+" km/h | 0-180 "+fixed(m.launchTo180)+" / "+fixed(t.launchSeconds)+" s");
    const char* seats[]={"Front","Middle","Rear"};const char* axes[]={"V","Lat","Long"};
    for(int seat=0;seat<3;++seat){const auto& st=m.seats[seat];std::string row=std::string(seats[seat])+" S10="+fixed(st.exposure10Seconds)+" | ";for(int a=0;a<3;++a){const auto& x=st.axes[a];row+=std::string(axes[a])+" "+fixed(x.minG)+".."+fixed(x.maxG)+"g, rate "+fixed(x.maxRateGps)+"g/s  ";}out.push_back(row);
        std::string means=std::string(seats[seat])+" signed rolling means: ";for(int a=0;a<3;++a){const auto& x=st.axes[a];means+=std::string(axes[a])+" 1s "+fixed(x.mean1sMin)+".."+fixed(x.mean1sMax)+", 10s "+fixed(x.mean10sMin)+".."+fixed(x.mean10sMax)+"g  ";}out.push_back(means);
        out.push_back(std::string(seats[seat])+" durations: vertical <0g "+fixed(st.airtimeBelowZeroSeconds)+"s | >2g "+fixed(st.positiveAbove2Seconds)+"s | >3g "+fixed(st.positiveAbove3Seconds)+"s | >4g "+fixed(st.positiveAbove4Seconds)+"s");out.push_back(std::string(seats[seat])+" longest bouts: <0g "+fixed(st.longestAirtimeSeconds)+"s | >2g "+fixed(st.longestAbove2Seconds)+"s | >3g "+fixed(st.longestAbove3Seconds)+"s | >4g "+fixed(st.longestAbove4Seconds)+"s");}
    out.push_back("Above station "+fixed(m.heightAboveStation)+" m | vertical relief "+fixed(m.verticalRelief)+" m | duration "+fixed(m.duration)+" s");
    const auto& l=d.request.limits;out.push_back("Force limits (provisional): V "+fixed(l.minVerticalG)+".."+fixed(l.maxVerticalG)+"g | Lat +/-"+fixed(l.maxLateralG)+"g | Long +/-"+fixed(l.maxLongitudinalG)+"g");out.push_back("Rate limits: V "+fixed(l.maxJerkGps)+" provisional | Lat "+(std::isfinite(l.maxLateralRateGps)?fixed(l.maxLateralRateGps)+" provisional":"UNASSESSED")+" | Long "+(std::isfinite(l.maxLongitudinalRateGps)?fixed(l.maxLongitudinalRateGps)+" provisional":"UNASSESSED"));
    out.push_back("Vertical rate is analytic; horizontal rates are integration-step differences and bandwidth dependent. All include rider rotation. No calibrated safety or universal-record claim.");
    if(previous&&previous->available){out.push_back("Previous accepted: seed "+std::to_string(previous->seed)+" / "+previous->terrain+" | reference "+previous->referenceState);out.push_back("Current - previous: S10 "+fixed(m.exposure10Seconds-previous->metrics.exposure10Seconds)+" g*s | speed "+fixed((m.maxSpeed-previous->metrics.maxSpeed)*3.6)+" km/h | duration "+fixed(m.duration-previous->metrics.duration)+" s");out.push_back("Height delta "+fixed(m.maxGroundHeight-previous->metrics.maxGroundHeight)+" m | inversion delta "+fixed(m.inversionGroundHeight-previous->metrics.inversionGroundHeight)+" m");out.push_back("Force peak deltas: vertical "+fixed(m.maxVerticalG-previous->metrics.maxVerticalG)+"g | lateral "+fixed(m.maxLateralG-previous->metrics.maxLateralG)+"g | longitudinal "+fixed(m.maxLongitudinalG-previous->metrics.maxLongitudinalG)+"g");if(previous->referenceId!=t.referenceId)out.push_back("References differ: previous "+previous->referenceId+"; target-relative results are not directly equivalent.");}
    else out.push_back("Previous accepted ride: none in this session");return out;
}
}
