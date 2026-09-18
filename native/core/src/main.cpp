#include "coaster/coaster.hpp"
#include <chrono>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <stdexcept>
using namespace coaster;
template<class Parser> static auto completeNumber(const std::string& text,Parser parse){
    size_t consumed=0;auto value=parse(text,&consumed);
    if(consumed!=text.size())throw std::runtime_error("Numeric option contains trailing characters: "+text);
    return value;
}
static uint64_t seedNumber(const std::string& text){
    if(text.empty()||!std::all_of(text.begin(),text.end(),[](char c){return c>='0'&&c<='9';}))
        throw std::runtime_error("Seed must contain only unsigned decimal digits");
    return completeNumber(text,[](const std::string& s,size_t* n){return std::stoull(s,n);});
}
static int integerNumber(const std::string& text){return completeNumber(text,[](const std::string& s,size_t* n){return std::stoi(s,n);});}
static double realNumber(const std::string& text){return completeNumber(text,[](const std::string& s,size_t* n){return std::stod(s,n);});}
static void write(const std::string& path,const std::string& data){std::ofstream f(path,std::ios::binary);if(!f||!(f<<data))throw std::runtime_error("Cannot write "+path);}
static void trace(const Design& d,const std::string& path){std::ofstream f(path);if(!f)throw std::runtime_error("Cannot write trace");f<<std::setprecision(12)<<"{\"sampleRateHz\":60,\"seatOrder\":[\"front\",\"middle\",\"rear\"],\"frames\":[";bool comma=false;for(auto& x:d.simulation.frames){if(comma)f<<',';comma=true;f<<"{\"time\":"<<x.time<<",\"distance\":"<<x.distance<<",\"speed\":"<<x.speed<<",\"seats\":[";for(int i=0;i<3;++i){if(i)f<<',';auto a=x.seats[i];f<<'['<<a.vertical<<','<<a.lateral<<','<<a.longitudinal<<']';}f<<"]}";}f<<"],\"geometry\":[";comma=false;for(double s=0;s<d.track.length;s+=5){auto p=d.track.sample(s);if(comma)f<<',';comma=true;f<<'['<<s<<','<<p.position.x<<','<<p.position.y<<','<<p.position.z<<','<<p.up.x<<','<<p.up.y<<','<<p.up.z<<','<<int(p.element)<<']';}f<<"]}";}
int main(int argc,char** argv){try{
    if(argc<2){std::cerr<<"Usage: coaster_cli generate [--seed N --terrain flat --preset all-records|physics-proof --height METRES --speed-kmh KMH --inversion-height METRES --candidates N --step SECONDS --json PATH --trace PATH --plan PATH --out PATH]\n       coaster_cli validate FILE [--json PATH]\n";return 1;}
    std::string command=argv[1],jsonPath,tracePath,outPath,planPath;GenerationRequest r;int begin=2;Design d;std::string error;
    auto started=std::chrono::steady_clock::now();
    if(command=="validate"){if(argc<3)throw std::runtime_error("validate requires FILE");begin=3;if(!loadDesign(argv[2],d,error)){std::cerr<<error<<'\n';return 2;}}
    else if(command!="generate")throw std::runtime_error("Unknown command");
    for(int i=begin;i<argc;++i){std::string key=argv[i];if(i+1>=argc)throw std::runtime_error("Missing option value");std::string value=argv[++i];
        if(key=="--seed")r.seed=seedNumber(value);else if(key=="--candidates")r.maxCandidates=integerNumber(value);else if(key=="--step")r.simulationStep=realNumber(value);else if(key=="--launch-seconds")r.targets.launchSeconds=realNumber(value);else if(key=="--height")r.targets.height=realNumber(value);else if(key=="--speed-kmh")r.targets.speed=realNumber(value)/3.6;else if(key=="--inversion-height")r.targets.inversionHeight=realNumber(value);else if(key=="--reference-exposure"){r.targets.referenceExposure=realNumber(value);if(!std::isfinite(r.targets.referenceExposure)||r.targets.referenceExposure<=0)throw std::runtime_error("Reference exposure must be finite and positive");}else if(key=="--reference-id")r.targets.referenceId=value;else if(key=="--reference-file"){if(!loadReference(value,r.targets,error))throw std::runtime_error(error);}else if(key=="--lateral-rate-limit"||key=="--longitudinal-rate-limit"){double limit=realNumber(value);if(!std::isfinite(limit)||limit<=0)throw std::runtime_error("Explicit rate limit must be finite and positive");if(key=="--lateral-rate-limit")r.limits.maxLateralRateGps=limit;else r.limits.maxLongitudinalRateGps=limit;}else if(key=="--json")jsonPath=value;else if(key=="--trace")tracePath=value;else if(key=="--out")outPath=value;else if(key=="--plan")planPath=value;
        else if(key=="--terrain"){if(value!="flat")throw std::runtime_error("Only flat ground is available in this build");}
        else if(key=="--preset"){if(value=="physics-proof")r.targets.requireIntensity=false;else if(value=="all-records")r.targets.requireIntensity=true;else throw std::runtime_error("Unknown preset");}
        else throw std::runtime_error("Unknown option: "+key);
    }
    if(command=="generate")d=generate(r,{},[started](int candidate,const std::string& message){std::cerr<<"candidate="<<candidate<<" elapsedSeconds="<<std::chrono::duration<double>(std::chrono::steady_clock::now()-started).count()<<' '<<message<<'\n';});
    std::string json=reportJson(d);double seconds=std::chrono::duration<double>(std::chrono::steady_clock::now()-started).count();json.pop_back();json+=",\"generationSeconds\":"+std::to_string(seconds)+"}";
    if(!jsonPath.empty())write(jsonPath,json+"\n");if(!tracePath.empty()&&!d.track.spans.empty())trace(d,tracePath);
    if(!planPath.empty())write(planPath,d.planningDiagnostics.empty()?"null\n":d.planningDiagnostics+"\n");
    std::cout<<json<<'\n';if(!outPath.empty()&&!saveDesign(d,outPath,error)){std::cerr<<error<<'\n';return 2;}return d.accepted()?0:2;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
