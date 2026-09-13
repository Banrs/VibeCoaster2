#include "coaster/coaster.hpp"
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <sstream>
using namespace coaster;
static int checks=0;
static void check(bool b,const std::string& message){++checks;if(!b)throw std::runtime_error(message);}
static void near(double a,double b,double tolerance,const std::string& what){check(std::isfinite(a)&&std::abs(a-b)<=tolerance,what+": "+std::to_string(a)+" versus "+std::to_string(b));}
static bool code(const ValidationReport& r,const std::string& c){return std::any_of(r.errors.begin(),r.errors.end(),[&](const Finding& f){return f.code==c;});}
static Track line(double length=150){std::vector<AuthoredPoint> p;for(int i=0;i<=int(length);++i)p.push_back({{double(i),0,20},0,Element::Launch,{0,0,1}});return compile(p,false);}
static Track circle(bool vertical,double radius=100,double bank=0){std::vector<AuthoredPoint> p;int n=int(std::ceil(2*pi*radius));for(int i=0;i<=n;++i){double a=2*pi*i/n;if(vertical)p.push_back({{radius*std::sin(a),0,20+radius*(1-std::cos(a))},bank,Element::Inversion,{-std::sin(a),0,std::cos(a)}});else p.push_back({{radius*std::sin(a),radius*(1-std::cos(a)),20},bank,Element::Turn,{0,0,1}});}p.back()=p.front();return compile(p,true);}
static void analytical(){
    auto straight=line();auto f=measureSeatForces(straight,50,40,3,0);near(f.vertical,1,1e-9,"Straight vertical gravity");near(f.lateral,0,1e-9,"Straight lateral gravity");near(f.longitudinal,3/gravity,1e-9,"Explicit tangential force");
    auto horizontal=circle(false);double speed=30,r=100;
    for(double s=10;s<horizontal.length;s+=17){auto q=measureSeatForces(horizontal,s,speed,0,0);near(q.vertical,1,1e-8,"Unbanked turn vertical");near(q.lateral,-speed*speed/(gravity*r),.0003,"Unbanked centripetal force");near(q.longitudinal,0,.0003,"Horizontal turn longitudinal");}
    double bank=-std::atan(speed*speed/(gravity*r));auto banked=circle(false,r,bank);
    for(double s=10;s<banked.length;s+=17){auto q=measureSeatForces(banked,s,speed,0,0);near(q.vertical,std::sqrt(1+std::pow(speed*speed/(gravity*r),2)),.0003,"Banked normal force");near(q.lateral,0,.0003,"Balanced bank lateral force");}
    auto loop=circle(true,40);
    for(double s=5;s<loop.length;s+=11){auto p=loop.sample(s);for(double height:{0.,1.2}){auto q=measureSeatForces(loop,s,30,-gravity*p.tangent.z,height);near(q.vertical,(1-height/40)*900/(gravity*40)+p.up.z,.001,"Vertical loop and seat offset");near(q.longitudinal,height/40*p.tangent.z,.001,"Finite seat tangential force");near(q.lateral,0,1e-6,"Planar loop lateral force");}}
    std::vector<AuthoredPoint> fast;for(int i=0;i<=1500;++i){double x=i*.1;fast.push_back({{x,0,20},2*pi*std::clamp(x-50.,-1.,1.)/.4,Element::Return,{0,0,1}});}auto rapid=compile(fast,false);
    auto fastForce=measureSeatForces(rapid,50,30,0,1.2);near(fastForce.vertical,1-1.2*900*std::pow(2*pi/.4,2)/gravity,5,"Analytic frame derivative resolves rapid-roll alias");
    rapid.closed=true;check(code(validateGeometry(rapid,Terrain{},Limits{},TrainConfig{},{}),"FRAME_RATE"),"Rapid bank rotation rejected by canonical domain gate");
    TrainConfig train;near(seatDistanceOffset(train,0),8.5,1e-12,"Front physical car");near(seatDistanceOffset(train,1),1.7,1e-12,"Middle physical car");near(seatDistanceOffset(train,2),-8.5,1e-12,"Rear physical car");
    std::vector<double> constant(2400,3);near(forceExposure(constant,1./240),30,1e-9,"Ten-second force integral");check(forceExposure(std::vector<double>(20,3),.1)==0,"Incomplete trace cannot establish ten-second exposure");
    TrainConfig one;one.cars=1;one.carMass=1000;one.dragCdA=0;one.rollingResistance=0;one.seatHeight=0;
    std::vector<Operation> ops{{0,straight.length,DriveKind::Launch,100,2000,200000,.001}};
    auto result=simulate(straight,ops,one,1./480);check(result.completed,"Constant-acceleration run completes");
    for(const auto& frame:result.frames)if(frame.time>.05){near(frame.speed,2*frame.time,.004,"Force integration speed");near(frame.seats[0].longitudinal,2/gravity*smooth((straight.length-frame.distance)/ops[0].exitFadeMeters),1e-9,"Measured explicit motor force including its authored exit fade");}
    auto six=one;six.cars=6;auto finiteTrain=simulate(straight,ops,six,1./480);for(size_t i=10;i<std::min(result.frames.size(),finiteTrain.frames.size());i+=50)if(finiteTrain.frames[i].distance+(six.cars-1)*six.spacing*.5<straight.length-2*ops[0].exitFadeMeters)near(result.frames[i].speed,finiteTrain.frames[i].speed,1e-10,"Per-car force/mass conservation inside common full-power coverage");
    for(const auto& frame:finiteTrain.frames)if(frame.time>.05){double sum=0;for(int car=0;car<six.cars;++car)sum+=smooth((straight.length-frame.distance-((six.cars-1)*.5-car)*six.spacing)/ops[0].exitFadeMeters);near(frame.seats[0].longitudinal,2/gravity*sum/six.cars,1e-9,"Finite train averages the exact per-car exit fade without force assignment");}
    auto longTrain=one;longTrain.cars=16;longTrain.spacing=20;check(code(simulate(circle(false,20),{},longTrain).report,"TRAIN_LENGTH"),"Closed circuit must exceed physical train length");
    auto invalidOps=ops;invalidOps.push_back({0,0,DriveKind(99),0,0,0,.1});check(code(simulate(straight,invalidOps,one).report,"DRIVE_CONFIG"),"Invalid operation enum rejected");
    auto brakingTrain=one;brakingTrain.rollingResistance=.002;
    std::vector<Operation> brakingOps{{0,30,DriveKind::Launch,10,2000,200000,.08},{30,straight.length,DriveKind::Brake,0,4000,400000,.08}};
    auto stopped=simulate(straight,brakingOps,brakingTrain,1./240);
    check(!stopped.completed&&code(stopped.report,"STALL"),"Open track stops before its end");
    near(stopped.frames.back().speed,0,0,"Coulomb stopping event must not leave midpoint creep");
    near(stopped.frames.back().distance,stopped.frames[stopped.frames.size()-2].distance,0,"Stopped train does not creep spatially");
    std::vector<Operation> weakOps{{0,straight.length,DriveKind::Launch,10,10,1000,.01}};
    auto held=simulate(straight,weakOps,brakingTrain);near(held.metrics.maxSpeed,0,0,"Sub-friction motor cannot move train");near(held.frames.back().distance,held.frames.front().distance,0,"Static friction cannot create displacement");
    auto shallow=straight;for(auto& k:shallow.knots){k.position.z-=.0001*k.position.x;k.tangent=unit(Vec3{1,0,-.0001});k.up=unit(Vec3{.0001,0,1});}shallow.rebuild();
    auto slopeHold=simulate(shallow,{},brakingTrain);near(slopeHold.metrics.maxSpeed,0,0,"Sub-friction downhill slope is held");near(slopeHold.frames.back().distance,slopeHold.frames.front().distance,0,"Sub-friction downhill slope has no position drift");
    auto noDrive=simulate(straight,{},one);check(!noDrive.completed&&code(noDrive.report,"STALL"),"No hidden launch motor");
}
static void geometry(){
    auto loop=circle(false);TrainConfig train;Limits limits;Terrain terrain;
    check(validateGeometry(loop,terrain,limits,train,{}).valid(),"Closed clear circle");
    auto a=loop.sample(0),b=loop.sample(loop.length);near(norm(a.position-b.position),0,1e-12,"Canonical seam position");near(norm(a.tangent-b.tangent),0,1e-12,"Canonical seam tangent");near(norm(a.curvature-b.curvature),0,1e-12,"Canonical seam curvature");
    auto displaced=loop;displaced.knots.back().position.x+=1;bool seamRejected=false;try{displaced.rebuild();}catch(...){seamRejected=true;}check(seamRejected,"Open canonical seam rejected before rebuilding shared G3 jets");
    auto low=loop;for(auto& k:low.knots)k.position.z=4;low.rebuild();check(code(validateGeometry(low,terrain,limits,train,{}),"TERRAIN_CLEARANCE"),"Terrain envelope clearance");
    std::vector<AuthoredPoint> p;for(int i=0;i<=600;++i){double a=2*pi*i/600;p.push_back({{50*std::sin(a),30*std::sin(2*a),20},0,Element::Turn,{0,0,1}});}p.back()=p.front();auto crossing=compile(p);check(code(validateGeometry(crossing,terrain,limits,train,{}),"TRACK_CLEARANCE"),"Nonadjacent figure-eight crossing");
    auto enormous=train;enormous.cars=1;enormous.spacing=1e6;check(code(validateGeometry(crossing,terrain,limits,enormous,{}),"TRAIN_CONFIG"),"Irrelevant one-car spacing cannot disable collision checks");
    auto q=loop.sample(80);Support column{{q.position.x,q.position.y,0},{q.position.x,q.position.y,40},{},false,0};check(code(validateGeometry(loop,terrain,limits,train,{column}),"SUPPORT_CLEARANCE"),"Full support column collision");
    Support invalid{{NAN,0,0},{0,0,20},{},false,0};check(code(validateGeometry(loop,terrain,limits,train,{invalid}),"SUPPORT_CONFIG"),"Nonfinite support rejected");
    check(code(validateGeometry(loop,terrain,limits,train,{},[]{return true;}),"CANCELLED"),"Geometry cancellation");
    bool threw=false;try{auto bad=loop;bad.knots.back().up={NAN,0,0};bad.rebuild();}catch(...){threw=true;}check(threw,"Nonfinite last knot rejected");
    threw=false;try{auto bad=loop;bad.knots[1].element=Element(99);bad.rebuild();}catch(...){threw=true;}check(threw,"Invalid element enum rejected");
}
static void generation(){
    GenerationRequest r;r.targets.requireIntensity=false;r.seed=42;
    auto d=generate(r);if(!d.accepted())std::cerr<<reportJson(d)<<'\n';check(d.accepted(),"Physics-proof seed42 accepted");
    auto replay=simulate(d.track,d.operations,r.train,r.simulationStep);check(replay.completed&&replay.report.valid(),"Independent final replay completes");near(replay.frames.back().time,replay.metrics.duration,0,"Terminal trace timestamp");near(replay.frames.back().speed,0,0,"Physical terminal stop speed");near(replay.metrics.maxSpeed,d.simulation.metrics.maxSpeed,1e-12,"Replay max speed");
    auto again=generate(r);check(again.accepted(),"Repeated seed accepted");check(d.track.knots.size()==again.track.knots.size(),"Deterministic knot count");for(size_t i=0;i<d.track.knots.size();i+=31){near(norm(d.track.knots[i].position-again.track.knots[i].position),0,0,"Deterministic geometry");near(d.track.knots[i].bank,again.track.knots[i].bank,0,"Deterministic banking");}
    auto finer=simulate(d.track,d.operations,r.train,1./480);check(finer.completed,"Half-step replay completes");near(finer.metrics.maxSpeed,replay.metrics.maxSpeed,.01,"Time-step max speed convergence");near(finer.metrics.launchTo180,replay.metrics.launchTo180,.005,"Time-step launch convergence");near(finer.metrics.maxVerticalG,replay.metrics.maxVerticalG,.03,"Time-step vertical force convergence");near(finer.metrics.maxLateralG,replay.metrics.maxLateralG,.03,"Time-step lateral force convergence");near(finer.metrics.exposure10Seconds,replay.metrics.exposure10Seconds,.05,"Time-step exposure convergence");near(finer.metrics.maxJerkGps,replay.metrics.maxJerkGps,1.,"Time-step jerk convergence");
    auto strict=d;strict.report={};strict.request.targets.requireIntensity=true;evaluateTargets(strict);check(!strict.accepted()&&code(strict.report,"REFERENCE_UNAVAILABLE"),"Unavailable benchmark rejects strict record claim");
    auto impossible=r;impossible.targets.speed=NAN;check(code(generate(impossible).report,"REQUEST_RANGE"),"NaN target rejected");
    auto stationRegression=r;stationRegression.seed=10;stationRegression.maxCandidates=1;
    auto stationRide=generate(stationRegression);check(stationRide.accepted(),"Seed10 flat must finish its station return");near(stationRide.simulation.frames.back().speed,0,0,"Seed10 terminal speed");
    auto cancelled=generate(r,[]{return true;});check(cancelled.simulation.cancelled&&!cancelled.accepted(),"Generation cancellation");int calls=0;auto interrupted=simulate(d.track,d.operations,r.train,r.simulationStep,[&]{return ++calls>3;});check(interrupted.cancelled&&!interrupted.completed,"Mid-simulation cancellation");
}
static void persistence(const Design& d){
    auto folder=std::filesystem::temp_directory_path()/"coaster-foundation-core-tests";std::filesystem::create_directories(folder);auto path=(folder/"roundtrip.coaster").string();std::filesystem::remove(path);std::string error;
    check(saveDesign(d,path,error),"Save accepted canonical geometry: "+error);check(saveDesign(d,path,error),"Atomic replacement of an existing save: "+error);Design loaded;check(loadDesign(path,loaded,error),"Load/revalidate geometry: "+error);check(loaded.accepted(),"Loaded design accepted");check(loaded.request.seed==d.request.seed&&loaded.track.knots.size()==d.track.knots.size(),"Persisted identity and exact geometry");near(loaded.track.length,d.track.length,1e-10,"Roundtrip canonical length");near(loaded.simulation.metrics.maxVerticalG,d.simulation.metrics.maxVerticalG,1e-10,"Roundtrip independent physics");
    for(size_t i=0;i<d.track.knots.size();i+=37)near(norm(loaded.track.knots[i].position-d.track.knots[i].position),0,0,"Exact double geometry roundtrip");
    auto rejected=d;rejected.report.fail("TEST","Rejected");check(!saveDesign(rejected,(folder/"rejected.coaster").string(),error),"Rejected save refused");
    auto tampered=d;for(auto& k:tampered.track.knots)k.position.z-=1000;check(!saveDesign(tampered,(folder/"tampered.coaster").string(),error),"Stale cached spans cannot bypass canonical save validation");
    std::ifstream f(path,std::ios::binary);std::string bytes((std::istreambuf_iterator<char>(f)),{});f.close();bytes[bytes.size()/2]^=1;auto corrupt=(folder/"corrupt.coaster").string();std::ofstream(corrupt,std::ios::binary)<<bytes;Design unchanged=d;check(!loadDesign(corrupt,unchanged,error),"Corruption rejected");near(unchanged.track.length,d.track.length,0,"Failed load preserves current design");check(!loadDesign(path,unchanged,error,[]{return true;}),"Load cancellation");
    for(auto name:{"roundtrip.coaster","corrupt.coaster"})std::filesystem::remove(folder/name);
}

static std::string readBytes(const std::filesystem::path& path){
    std::ifstream f(path,std::ios::binary);check(bool(f),"Open regression fixture/file: "+path.string());
    return std::string(std::istreambuf_iterator<char>(f),{});
}
static std::vector<std::string> payloadLines(const std::string& raw){
    auto end=raw.find('\n');check(end!=std::string::npos,"Save envelope line exists");
    std::istringstream in(raw.substr(end+1));std::vector<std::string> lines;std::string line;
    while(std::getline(in,line))lines.push_back(line);return lines;
}
static std::vector<std::string> tokens(const std::string& line){
    std::istringstream in(line);std::vector<std::string> values;std::string value;
    while(in>>value)values.push_back(value);return values;
}
static uint64_t fixtureChecksum(const std::string& bytes){
    uint64_t h=14695981039346656037ull;for(unsigned char c:bytes){h^=c;h*=1099511628211ull;}return h;
}
static void writeBadProfile(const std::filesystem::path& out,const std::string& good,size_t knotCount,const std::string& decel,const std::string& offset,const std::string& fade=""){
    auto lines=payloadLines(good);auto values=tokens(lines.at(5+knotCount));check(values.size()==10,"Schema5 work2 operation has ten explicit fields");
    values[7]=decel;values[8]=offset;if(!fade.empty())values[9]=fade;std::string replacement;
    for(size_t i=0;i<values.size();++i)replacement+=(i?" ":"")+values[i];lines[5+knotCount]=replacement;
    std::string payload;for(const auto& line:lines)payload+=line+"\n";
    std::ofstream f(out,std::ios::binary);f<<"COASTER 5 "<<payload.size()<<' '<<fixtureChecksum(payload)<<'\n'<<payload;
    check(bool(f),"Write correctly checksummed malformed profile");
}
static void migration(const Design& current){
    const auto folder=std::filesystem::temp_directory_path()/"coaster-geometry-schema5-tests";std::filesystem::create_directories(folder);
    const auto newPath=folder/"current.coaster",badPath=folder/"bad-profile.coaster";
    std::string error;Design replay;
    for(const char* name:{"legacy-v021.coaster","foundation-v1.coaster"}){
        const auto fixture=std::filesystem::path(__FILE__).parent_path()/"fixtures"/name;const auto original=readBytes(fixture);auto unchanged=current;
        check(!loadDesign(fixture.string(),unchanged,error),"Older canonical semantics explicitly rejected");
        check(error.find("unsupported schema")!=std::string::npos,"Old geometry rejection identifies unsupported schema");
        check(reportJson(unchanged)==reportJson(current),"Unsupported load preserves last accepted design");
        check(readBytes(fixture)==original,"Archived original geometry remains byte-identical");
    }
    check(saveDesign(current,newPath.string(),error),"Save current explicit profile: "+error);const auto currentBytes=readBytes(newPath);check(currentBytes.rfind("COASTER 5 ",0)==0,"New canonical semantics use COASTER5");
    check(loadDesign(newPath.string(),replay,error),"Reload current explicit profile: "+error);
    check(reportJson(current)==reportJson(replay),"Current profile physics/provenance roundtrip");
    auto rejectedSurface=[&](std::vector<std::string> lines){
        std::string payload;for(const auto& line:lines)payload+=line+"\n";
        {std::ofstream file(badPath,std::ios::binary);file<<"COASTER 5 "<<payload.size()<<' '<<fixtureChecksum(payload)<<'\n'<<payload;}
        auto unchanged=current;check(!loadDesign(badPath.string(),unchanged,error)&&error.find("flat")!=std::string::npos,"Checksummed non-flat save is explicitly refused");
        check(reportJson(unchanged)==reportJson(current),"Unsupported ground leaves the accepted design intact");
    };
    for(int kind:{1,2}){auto lines=payloadLines(currentBytes);auto fields=tokens(lines[0]);fields[2]=std::to_string(kind);lines[0].clear();for(const auto& field:fields){if(!lines[0].empty())lines[0]+=' ';lines[0]+=field;}rejectedSurface(lines);}
    for(const std::string profile:{"2 1 0 0 0 0 600","1 1 0 0 0 5 600"}){auto lines=payloadLines(currentBytes);bool found=false;for(size_t i=0;i+1<lines.size();++i)if(lines[i].starts_with("TERRAIN_PROFILE ")){lines[i]="TERRAIN_PROFILE 1 "+std::to_string(profile.size()+1);lines[i+1]=profile;found=true;break;}check(found,"Saved flat-profile extension exists");rejectedSurface(lines);}

    check(current.operations.size()==replay.operations.size(),"Current operation count roundtrip");bool station=false;
    for(size_t i=0;i<current.operations.size();++i){
        near(replay.operations[i].stopDeceleration,current.operations[i].stopDeceleration,0,"Explicit deceleration roundtrip");
        near(replay.operations[i].stopOffset,current.operations[i].stopOffset,0,"Explicit offset roundtrip");
        near(replay.operations[i].exitFadeMeters,current.operations[i].exitFadeMeters,0,"Explicit exit fade roundtrip");
        near(current.operations[i].exitFadeMeters,std::max(1.,current.operations[i].targetSpeed*current.operations[i].rampSeconds),0,"Generator authors a speed-scaled exit fade");
        if(current.operations[i].kind==DriveKind::Station){station=true;near(replay.operations[i].stopDeceleration,2.4,0,"Packed station preferred deceleration");near(replay.operations[i].stopOffset,.2,0,"Packed station stop offset");near(replay.operations[i].exitFadeMeters,1,0,"Station physical endpoint fade remains beyond its stopped train");}
    }
    check(station,"Current design has an explicit station operation");
    for(const auto& fields:std::vector<std::pair<std::string,std::string>>{{"0","0.2"},{"-1","0.2"},{"21","0.2"},{"nan","0.2"},{"1e309","0.2"},{"2.4","-1"},{"2.4","6"},{"2.4","nan"},{"2.4","1e309"}}){
        writeBadProfile(badPath,currentBytes,current.track.knots.size(),fields.first,fields.second);
        auto unchanged=current;check(!loadDesign(badPath.string(),unchanged,error),"Invalid checksummed profile is rejected");
        check(reportJson(unchanged)==reportJson(current),"Invalid profile load preserves previous accepted design");
    }
    for(const auto& fields:std::vector<std::pair<double,double>>{{0,.2},{-1,.2},{21,.2},{NAN,.2},{INFINITY,.2},{2.4,-1},{2.4,6},{2.4,NAN},{2.4,INFINITY}}){
        auto invalid=current;invalid.operations[0].stopDeceleration=fields.first;invalid.operations[0].stopOffset=fields.second;
        check(code(simulate(invalid.track,invalid.operations,invalid.request.train).report,"DRIVE_CONFIG"),"Invalid direct-API profile rejected before integration");
        int probes=0;check(!saveDesign(invalid,newPath.string(),error,[&]{++probes;return false;}),"Invalid profile cannot overwrite accepted save");
        check(probes==0,"Invalid motor parameters reject before geometry or simulation work");check(readBytes(newPath)==currentBytes,"Rejected profile save preserves previous file");
    }
    for(const auto& fade:std::vector<std::string>{"0","0.001","1000.1","nan","inf","1e309","-1"}){
        const auto& op=current.operations.front();writeBadProfile(badPath,currentBytes,current.track.knots.size(),std::to_string(op.stopDeceleration),std::to_string(op.stopOffset),fade);
        auto unchanged=current;check(!loadDesign(badPath.string(),unchanged,error),"Invalid checksummed fade is rejected");check(reportJson(unchanged)==reportJson(current),"Invalid fade load preserves last-good design");
    }
    for(double fade:std::array<double,6>{0.,.001,1000.1,NAN,INFINITY,-1.}){
        auto invalid=current;invalid.operations.front().exitFadeMeters=fade;
        check(code(simulate(invalid.track,invalid.operations,invalid.request.train).report,"DRIVE_CONFIG"),"Invalid direct-API fade rejected before integration");
        int probes=0;check(!saveDesign(invalid,newPath.string(),error,[&]{++probes;return false;}),"Invalid fade cannot overwrite accepted save");
        check(probes==0,"Invalid fade rejects before geometry or simulation work");check(readBytes(newPath)==currentBytes,"Rejected fade save preserves prior bytes");
    }
    const auto priorFixture=std::filesystem::path(__FILE__).parent_path()/"fixtures/geometry-v1-work.coaster";const auto priorBytes=readBytes(priorFixture);auto priorOutput=current;
    check(!loadDesign(priorFixture.string(),priorOutput,error)&&error.find("Unsupported generator version")!=std::string::npos,"Previous work1 operation semantics are explicitly unsupported");
    check(reportJson(priorOutput)==reportJson(current)&&readBytes(priorFixture)==priorBytes,"Unsupported work1 load preserves current design and archived file");
    for(const auto& path:{newPath,badPath})std::filesystem::remove(path);
}


static void planningAndTargets(){
    GenerationRequest req;req.seed=1;req.targets.requireIntensity=false;auto flat=generate(req);check(flat.accepted(),"Variety flat fixture accepted");
    auto repeated=generate(req);check(repeated.planningDiagnostics==flat.planningDiagnostics,"Deterministic ranked plan and search diagnostics");
    req.seed=2;auto varied=generate(req);check(varied.accepted(),"Second seeded flat profile completes");
    // The folded family deliberately retains its record hill before the full
    // loop and reversing pair. Variation is in canonical shape, not a legacy
    // inversion-first ordering. The organic suite measures each inversion's
    // topology and the physically separated crossovers independently.
    for(Element element:{Element::Hill,Element::Inversion,Element::Airtime})
        check(std::any_of(varied.track.knots.begin(),varied.track.knots.end(),[&](const Knot& k){return k.element==element;}),"Seeded folded profile retains hill, inversion and airtime geometry");
    auto intrinsicPoint=[](const Track& track,double fraction){const auto origin=track.sample(0);const auto delta=track.sample(track.length*fraction).position-origin.position;return Vec3{dot(delta,origin.tangent),dot(delta,origin.right),delta.z};};
    double shapeDifference=0;for(int i=1;i<64;++i)shapeDifference=std::max(shapeDifference,norm(intrinsicPoint(varied.track,i/64.)-intrinsicPoint(flat.track,i/64.)));
    check(shapeDifference>1,"Different seeds change circuit shape after removing station translation and heading");
    check(varied.simulation.metrics.launchTo180<=1.4,"Second seeded flat actual departure meets launch target");
    req.seed=1;req.terrain.kind=TerrainKind::Flat;req.targets.requireIntensity=true;req.targets.referenceId="TEST_ONLY_SYNTHETIC_NOT_I305";req.targets.referenceExposure=20;
    auto lowReference=generate(req);check(lowReference.accepted(),"Synthetic 20 g*s reference drives a feasible design");
    req.targets.referenceExposure=30;auto highReference=generate(req);check(highReference.accepted(),"Synthetic 30 g*s reference drives a feasible design");
    check(highReference.simulation.metrics.exposure10Seconds>=33,"Independent measurement meets synthetic exposure goal");
    check(highReference.simulation.metrics.exposure10Seconds>lowReference.simulation.metrics.exposure10Seconds+.1,"Reference magnitude changes force design and measured exposure");
    check(std::abs(highReference.track.length-lowReference.track.length)>1,"Reference changes geometry, not just acceptance postfilter");
    check(highReference.simulation.metrics.maxLateralG<=req.limits.maxLateralG&&highReference.simulation.metrics.maxVerticalG<=req.limits.maxVerticalG,"Target adaptation preserves force gates");
    auto halfStep=simulate(highReference.track,highReference.operations,req.train,1./480);check(halfStep.completed,"Target-driven sustained-force half-step replay completes");
    near(halfStep.metrics.maxSpeed,highReference.simulation.metrics.maxSpeed,.01,"Target-driven speed convergence");near(halfStep.metrics.maxVerticalG,highReference.simulation.metrics.maxVerticalG,.03,"Target-driven normal-force convergence");near(halfStep.metrics.maxLateralG,highReference.simulation.metrics.maxLateralG,.03,"Target-driven lateral-force convergence");near(halfStep.metrics.exposure10Seconds,highReference.simulation.metrics.exposure10Seconds,.05,"Target-driven exposure convergence");near(halfStep.frames.back().speed,0,0,"Target-driven physical terminal stop");
    auto path=(std::filesystem::temp_directory_path()/"coaster-synthetic-target.coaster").string();std::string error;check(saveDesign(highReference,path,error),"Synthetic target geometry independently validates before save: "+error);Design loaded;check(loadDesign(path,loaded,error),"Synthetic target saved geometry replays: "+error);check(reportJson(loaded)==reportJson(highReference),"Synthetic configured target and exact measured result survive save/replay");std::filesystem::remove(path);

    req.targets.referenceExposure=51;check(code(generate(req).report,"INTENSITY_FEASIBILITY"),"Exposure above the force-duration ceiling rejected honestly");
    req.seed=2;req.targets.requireIntensity=false;req.targets.launchSeconds=1.2;auto fastLaunch=generate(req);
    check(fastLaunch.accepted()&&fastLaunch.simulation.metrics.launchTo180<=1.2,"Launch request sizes explicit motor force");
    check(fastLaunch.simulation.metrics.maxLongitudinalG<=req.limits.maxLongitudinalG,"Faster departure preserves longitudinal force ceiling");
    auto launchForce=[](const Design& d){for(const auto& op:d.operations)if(op.kind==DriveKind::Launch)return op.maxForce;return 0.;};
    check(launchForce(fastLaunch)>launchForce(varied),"Tighter launch time changes authored bounded motor force");
    req.targets.launchSeconds=1.15;check(code(generate(req).report,"LAUNCH_FEASIBILITY"),"Unreachable flat-departure target rejected with physical bound");
}

int main(int argc,char** argv){try{
    if(argc==2&&std::string(argv[1])=="--persistence"){
        GenerationRequest request;request.targets.requireIntensity=false;
        auto d=generate(request);check(d.accepted(),"Persistence fixture accepted");
        persistence(d);migration(d);
        std::cout<<"PASS "<<checks<<" canonical persistence and invalid-input checks\n";
        return 0;
    }
    check(argc==1,"Unknown test selection");
    analytical();geometry();generation();planningAndTargets();std::cout<<"PASS "<<checks<<" checks: analytical forces, explicit motors, finite train, geometry/terrain/support clearances, canonical seam, determinism, timestep convergence, cancellation, explicit stop and exit-fade profiles, terrain/order variety and target-driven planning\n";return 0;
}catch(const std::exception& e){std::cerr<<"FAIL after "<<checks<<" checks: "<<e.what()<<'\n';return 1;}}
