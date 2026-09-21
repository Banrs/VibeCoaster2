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
static void trace(const Design& d,const std::string& path){
    std::ofstream f(path);if(!f)throw std::runtime_error("Cannot write trace");
    f<<std::setprecision(12)<<"{\"sampleRateHz\":60,\"acceptanceSampleRateHz\":"<<1/d.request.simulationStep
        <<",\"displayTraceOnly\":true,\"orientationReference\":\"train-centre; each dynamics entry carries its own seat frame\",\"seatOrder\":[\"front\",\"middle\",\"rear\"],\"seatForceOrder\":[\"Gz_up\",\"Gy_right\",\"Gx_forward\"],\"angleUnits\":\"radians\",\"frames\":[";
    auto vector=[&](Vec3 q){f<<'['<<q.x<<','<<q.y<<','<<q.z<<']';};
    std::array<std::array<double,3>,4> previous{};bool comma=false;
    auto orientation=[](const TrackSample& q,std::array<double,3>& prior){
        // Choose the nearest equivalent Euler branch; frame vectors remain
        // authoritative at vertical pitch where Euler yaw is indeterminate.
        double yaw=std::atan2(q.tangent.y,q.tangent.x),pitch=std::asin(std::clamp(q.tangent.z,-1.,1.));
        std::array<double,3> angles{};double best=INFINITY;
        for(int branch=0;branch<2;++branch){double y=yaw+branch*pi,p=branch?pi-pitch:pitch;
            Vec3 up{-std::cos(y)*std::sin(p),-std::sin(y)*std::sin(p),std::cos(p)};
            double roll=std::atan2(dot(q.up,cross(q.tangent,up)),dot(q.up,up));
            std::array<double,3> candidate{roll,p,y};double score=0;
            for(int i=0;i<3;++i){candidate[i]=prior[i]+std::remainder(candidate[i]-prior[i],2*pi);score+=std::pow(candidate[i]-prior[i],2);}
            if(score<best){angles=candidate;best=score;}
        }prior=angles;return angles;
    };
    for(const auto& x:d.simulation.frames){
        const auto q=d.track.sample(x.distance);const auto angles=orientation(q,previous[0]);
        if(comma)f<<',';comma=true;
        f<<"{\"time\":"<<x.time<<",\"distance\":"<<x.distance<<",\"speed\":"<<x.speed
            <<",\"roll\":"<<angles[0]<<",\"pitch\":"<<angles[1]<<",\"yaw\":"<<angles[2]
            <<",\"acceleration\":"<<x.acceleration<<",\"accelerationRate\":"<<x.accelerationRate
            <<",\"driveWorkPerMass\":"<<x.driveWorkPerMass<<",\"brakeWorkPerMass\":"<<x.brakeWorkPerMass
            <<",\"lossWorkPerMass\":"<<x.lossWorkPerMass<<",\"energyResidual\":"<<x.energyResidual<<",\"forward\":";vector(q.tangent);
        f<<",\"up\":";vector(q.up);f<<",\"seats\":[";
        for(int i=0;i<3;++i){if(i)f<<',';auto a=x.seats[i];f<<'['<<a.vertical<<','<<a.lateral<<','<<a.longitudinal<<']';}
        f<<"],\"dynamics\":[";
        for(int i=0;i<3;++i){if(i)f<<',';
            const double seatDistance=x.distance+seatDistanceOffset(d.request.train,i);
            const auto a=measureSeatDynamics(d.track,seatDistance,x.speed,x.acceleration,x.accelerationRate,d.request.train.seatHeight);
            const auto pose=d.track.sample(seatDistance);const auto seatAngles=orientation(pose,previous[i+1]);
            f<<"{\"roll\":"<<seatAngles[0]<<",\"pitch\":"<<seatAngles[1]<<",\"yaw\":"<<seatAngles[2]<<",\"forward\":";vector(pose.tangent);f<<",\"up\":";vector(pose.up);
            f<<",\"forceRateGps\":["<<a.rate.vertical<<','<<a.rate.lateral<<','<<a.rate.longitudinal<<"],\"inertialJerkMps3\":";vector(a.inertialJerk);
            f<<",\"angularVelocityRadps\":";vector(a.angularVelocity);f<<",\"angularAccelerationRadps2\":";vector(a.angularAcceleration);f<<",\"angularJerkRadps3\":";vector(a.angularJerk);f<<'}';
        }f<<"]}";
    }
    f<<"],\"geometry\":[";comma=false;
    for(double s=0;s<d.track.length;s+=2){auto p=d.track.sample(s);if(comma)f<<',';comma=true;f<<'['<<s<<','<<p.position.x<<','<<p.position.y<<','<<p.position.z<<','<<p.up.x<<','<<p.up.y<<','<<p.up.z<<','<<int(p.element)<<']';}
    f<<"]}";
}
int main(int argc,char** argv){try{
    if(argc<2){std::cerr<<"Usage: coaster_cli generate [--recipe PATH --seed N --terrain flat|highlands --preset default|reference --height METRES --speed-kmh KMH --inversion-height METRES --airtime 0.75..1.2 --signature-roll 30..60 --return-style auto|flowing|airtime --trims auto|off --candidates N --step SECONDS --json PATH --trace PATH --plan PATH --recipe-out PATH --out PATH]\n       coaster_cli validate FILE [--json PATH --recipe-out PATH]\n       coaster_cli recipe [--recipe PATH --out PATH]\n";return 1;}
    std::string command=argv[1],jsonPath,tracePath,outPath,planPath,recipeOutPath;GenerationRequest r;int begin=2;Design d;std::string error;
    auto started=std::chrono::steady_clock::now();
    if(command=="recipe"){
        auto recipe=defaultRideRecipe();
        for(int i=2;i<argc;++i){std::string key=argv[i];if(i+1>=argc)throw std::runtime_error("Missing option value");std::string value=argv[++i];
            if(key=="--recipe"){if(!loadRecipe(value,recipe,error))throw std::runtime_error(error);}
            else if(key=="--out")outPath=value;else throw std::runtime_error("Unknown recipe option: "+key);
        }
        if(outPath.empty())std::cout<<recipePayload(recipe);else if(!saveRecipe(recipe,outPath,error))throw std::runtime_error(error);
        return 0;
    }
    if(command=="validate"){if(argc<3)throw std::runtime_error("validate requires FILE");begin=3;}
    else if(command!="generate")throw std::runtime_error("Unknown command");
    for(int i=begin;i<argc;++i){std::string key=argv[i];if(i+1>=argc)throw std::runtime_error("Missing option value");std::string value=argv[++i];
        if(command=="validate"&&key!="--json"&&key!="--trace"&&key!="--out"&&key!="--plan"&&key!="--recipe-out")throw std::runtime_error(key+" is only valid for generate");
        if(key=="--airtime")r.style.airtime=realNumber(value);
        else if(key=="--signature-roll")r.style.signatureRollDegrees=realNumber(value);
        else if(key=="--return-style"){if(value=="auto")r.style.returnStyle=-1;else if(value=="flowing")r.style.returnStyle=0;else if(value=="airtime")r.style.returnStyle=1;else throw std::runtime_error("Return style must be auto, flowing or airtime");}
        else if(key=="--trims"){if(value=="auto")r.style.automaticTrims=true;else if(value=="off")r.style.automaticTrims=false;else throw std::runtime_error("Trims must be auto or off");}
        else if(key=="--seed")r.seed=seedNumber(value);else if(key=="--candidates")r.maxCandidates=integerNumber(value);else if(key=="--step")r.simulationStep=realNumber(value);else if(key=="--launch-seconds")r.targets.launchSeconds=realNumber(value);else if(key=="--height")r.targets.height=realNumber(value);else if(key=="--speed-kmh")r.targets.speed=realNumber(value)/3.6;else if(key=="--inversion-height")r.targets.inversionHeight=realNumber(value);else if(key=="--reference-exposure"){r.targets.referenceExposure=realNumber(value);if(!std::isfinite(r.targets.referenceExposure)||r.targets.referenceExposure<=0)throw std::runtime_error("Reference exposure must be finite and positive");}else if(key=="--reference-id")r.targets.referenceId=value;else if(key=="--reference-file"){if(!loadReference(value,r.targets,error))throw std::runtime_error(error);}else if(key=="--lateral-rate-limit"||key=="--longitudinal-rate-limit"){double limit=realNumber(value);if(!std::isfinite(limit)||limit<=0)throw std::runtime_error("Explicit rate limit must be finite and positive");if(key=="--lateral-rate-limit")r.limits.maxLateralRateGps=limit;else r.limits.maxLongitudinalRateGps=limit;}else if(key=="--json")jsonPath=value;else if(key=="--trace")tracePath=value;else if(key=="--out")outPath=value;else if(key=="--plan")planPath=value;
        else if(key=="--terrain"){if(value=="flat")r.terrain.kind=TerrainKind::Flat;else if(value=="highlands")r.terrain.kind=TerrainKind::Highlands;else throw std::runtime_error("Terrain must be flat or highlands");}
        else if(key=="--recipe"){if(!loadRecipe(value,r.recipe,error))throw std::runtime_error(error);}
        else if(key=="--recipe-out")recipeOutPath=value;
        else if(key=="--preset"){if(value=="default"||value=="physics-proof")r.targets.requireIntensity=false;else if(value=="reference")r.targets.requireIntensity=true;else throw std::runtime_error("Unknown preset");}
        else throw std::runtime_error("Unknown option: "+key);
    }
    const Progress progress=[started](const WorkProgress& stage){std::cerr<<"phase="<<phaseKey(stage.phase)<<" candidate="<<stage.candidate<<" elapsedSeconds="<<std::chrono::duration<double>(std::chrono::steady_clock::now()-started).count();if(stage.totalWork>0)std::cerr<<" work="<<stage.completedWork<<'/'<<stage.totalWork;std::cerr<<' '<<stage.detail<<'\n';};
    if(command=="validate"){d=inspectDesign(argv[2],error,{},progress);if(!d.accepted())std::cerr<<error<<'\n';}
    if(command=="generate")d=generate(r,{},progress);
    const double seconds=std::chrono::duration<double>(std::chrono::steady_clock::now()-started).count();
    double saveSeconds=0;bool saved=true;
    if(!outPath.empty()){const auto saveStart=std::chrono::steady_clock::now();saved=saveDesign(d,outPath,error,{},progress);saveSeconds=std::chrono::duration<double>(std::chrono::steady_clock::now()-saveStart).count();}
    if(!recipeOutPath.empty()){
        if(d.request.recipe.elements.empty())throw std::runtime_error("This legacy ride has no high-level recipe; its retained FVD/spline sources remain available");
        if(!saveRecipe(d.request.recipe,recipeOutPath,error))throw std::runtime_error(error);
    }
    std::string json=reportJson(d);json.pop_back();json+=",\"operation\":\""+command+"\",\""+(command=="generate"?std::string("generationSeconds"):std::string("validationSeconds"))+"\":"+std::to_string(seconds)+",\"serializationSeconds\":"+std::to_string(saveSeconds)+"}";
    if(!jsonPath.empty())write(jsonPath,json+"\n");if(!tracePath.empty()&&!d.track.spans.empty())trace(d,tracePath);
    if(!planPath.empty())write(planPath,d.planningDiagnostics.empty()?"null\n":d.planningDiagnostics+"\n");
    std::cout<<json<<'\n';if(!saved){std::cerr<<error<<'\n';return 2;}return d.accepted()?0:2;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
