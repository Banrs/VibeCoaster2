#include "station.hpp"
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <stdexcept>
namespace coaster::station_validation {double footprintRadius(Vec3,Vec3,double,double);}
using namespace coaster;
int checks=0;
void require(bool condition,const char* message){++checks;if(!condition)throw std::runtime_error(message);}
StationBox box(Vec3 c,Vec3 h){return {c,{1,0,0},{0,-1,0},{0,0,1},h,StationRole::Platform};}
int main(int argc,char**argv){try{
    auto a=box({0,0,0},{1,1,1});auto b=box({3,0,0},{1,1,1});
    require(!stationBoxesOverlap(a,b),"Separated boxes collide");b.center.x=2;
    require(stationBoxesOverlap(a,b),"Touching boxes missed");b.center.x=1.9;
    require(stationBoxesOverlap(a,b),"Overlapping boxes missed");
    auto c=box({0,0,0},{3,.1,.1}); c.forward=unit(Vec3{1,1,0});c.right=cross(c.forward,c.up);
    auto d=c;d.center=c.right*.5;
    require(!stationBoxesOverlap(c,d),"Rotated disjoint boxes collide despite overlapping AABBs");
    auto axis=unit(Vec3{.2,.3,.7});for(auto* v:{&c,&d}){v->center=rotate(v->center,axis,.75);v->forward=rotate(v->forward,axis,.75);v->right=rotate(v->right,axis,.75);v->up=rotate(v->up,axis,.75);}
    require(!stationBoxesOverlap(c,d),"Rigid rotation changes SAT decision");
    // This frame is legal under the persisted 1e-6 axis tolerance, but its
    // actual projected corner extends beyond hypot(half.x,half.y).
    auto skew=box({0,0,0},{30,10,1});
    skew.forward=rotate({1+4e-7,0,0},{0,0,1},.73);
    skew.right=rotate({4e-7,-1-4e-7,0},{0,0,1},.73);
    StationGeometry skewDefinition;skewDefinition.enabled=true;skewDefinition.boardingBegin=-18;skewDefinition.boardingEnd=64;
    for(int role=0;role<5;++role){auto part=skew;part.role=StationRole(role);skewDefinition.boxes.push_back(part);}
    require(validateStationDefinition(skewDefinition).valid(),"Skew footprint fixture is inside the actual accepted frame domain");
    const double skewRadius=station_validation::footprintRadius(skew.forward,skew.right,skew.half.x,skew.half.y);
    require(skewRadius>std::hypot(skew.half.x,skew.half.y)+1e-6,"Near-orthonormal fixture does not expose the old undersized disk");
    for(int ix=0;ix<=32;++ix)for(int iy=0;iy<=32;++iy){
        const Vec3 delta=skew.forward*((ix/16.-1)*skew.half.x)+skew.right*((iy/16.-1)*skew.half.y);
        require(std::hypot(delta.x,delta.y)<=skewRadius+1e-12,"Production local query disk encloses skew footprint corners and interior");
    }
    if(argc==2&&std::string(argv[1])=="--terrain-footprints-only"){std::cout<<"PASS "<<checks<<" focused station terrain-footprint checks\n";return 0;}
    std::string error;StationGeometry disabled,parsed;require(parseStationPayload(stationPayload(disabled),parsed,error),"Empty legacy station failed");
    require(!parsed.enabled&&parsed.boxes.empty(),"Legacy station acquired geometry");
    std::vector<std::pair<std::string,Design>> fixtures;
    if(argc==1){
        for(int terrain=0;terrain<3;++terrain){GenerationRequest request;request.seed=terrain==0?1:terrain==1?2:24;request.terrain.kind=TerrainKind(terrain);request.targets.requireIntensity=false;
            auto generated=generate(request);if(!generated.accepted())std::cerr<<reportJson(generated)<<'\n';require(generated.accepted(),"Current default station fixture was not accepted");
            const auto name="station-generated-"+std::to_string(request.seed)+"-"+request.terrain.name();
            fixtures.emplace_back(name,std::move(generated));
        }
    }else for(int i=1;i<argc;++i){Design design;require(loadDesign(argv[i],design,error),error.c_str());fixtures.emplace_back(std::filesystem::path(argv[i]).stem().string(),std::move(design));}
    for(auto& [name,design]:fixtures){
        auto station=buildStation(design.track,design.request.terrain,design.request.train);
        auto report=validateStation(design.track,design.request.terrain,design.request.train,station);
        for(auto& e:report.errors)std::cerr<<e.code<<" "<<e.message<<" s="<<e.distance<<" part="<<e.actual<<"\n";
        require(report.valid(),"Canonical station rejected accepted ride");
        const auto payload=stationPayload(station);StationGeometry restored;
        require(parseStationPayload(payload,restored,error),"Station parse failed");
        require(stationPayload(restored)==payload,"Station roundtrip changed geometry");
        auto previous=restored;require(!parseStationPayload(payload+"garbage",restored,error),"Trailing payload accepted");
        require(stationPayload(restored)==stationPayload(previous),"Failed parse replaced previous station");
        auto malformed=station;malformed.boxes[0].half.x=-1;require(!validateStationDefinition(malformed).valid(),"Negative extent accepted");
        malformed=station;malformed.boxes[0].forward={0,0,0};require(!validateStationDefinition(malformed).valid(),"Degenerate frame accepted");
        malformed=station;malformed.boxes[0].role=static_cast<StationRole>(99);require(!validateStationDefinition(malformed).valid(),"Unknown role accepted");
        malformed=station;malformed.boxes[0].center.x=std::numeric_limits<double>::quiet_NaN();require(!validateStationDefinition(malformed).valid(),"Nonfinite part accepted");
        malformed=station;malformed.boxes.back().center.z+=50;require(!validateStationDefinition(malformed).valid(),"Floating station part accepted");
        malformed=station;for(auto& part:malformed.boxes)part.center.x+=1000;
        require(!validateStation(design.track,design.request.terrain,design.request.train,malformed).valid(),"Remote building accepted as station");
        malformed=station;auto q=design.track.sample(0);
        // This part intersects the train and a platform, so connection validation alone cannot reject it.
        malformed.boxes.push_back({q.position+q.up*.6,q.tangent,q.right,q.up,{.5,2.,.8},StationRole::Post});
        report=validateStation(design.track,design.request.terrain,design.request.train,malformed);
        require(!report.valid()&&std::any_of(report.errors.begin(),report.errors.end(),[](const Finding& e){return e.code=="STATION_CLEARANCE";}),"Station train collision missed");
        require(!validateStation(design.track,design.request.terrain,design.request.train,station,[]{return true;}).valid(),"Station cancellation ignored");
        auto hasCode=[](const ValidationReport& report,const std::string& code){return std::any_of(report.errors.begin(),report.errors.end(),[&](const Finding& f){return f.code==code;});};
        auto brokenTrack=design.track;brokenTrack.knots.clear();
        require(hasCode(validateStation(brokenTrack,design.request.terrain,design.request.train,station),"STATION_CONFIG"),"Missing knots were sampled unsafely");
        bool threw=false;try{buildStation(brokenTrack,design.request.terrain,design.request.train);}catch(...){threw=true;}require(threw,"Station builder sampled missing knots");
        brokenTrack=design.track;brokenTrack.spans[0].c[0].x+=1;
        require(hasCode(validateStation(brokenTrack,design.request.terrain,design.request.train,station),"STATION_CONFIG"),"Stale cached spans accepted");
        brokenTrack=design.track;brokenTrack.length=std::numeric_limits<double>::quiet_NaN();
        require(hasCode(validateStation(brokenTrack,design.request.terrain,design.request.train,station),"STATION_CONFIG"),"Nonfinite track length accepted");
        auto invalidTerrain=design.request.terrain;invalidTerrain.kind=static_cast<TerrainKind>(99);
        threw=false;try{buildStation(design.track,invalidTerrain,design.request.train);}catch(...){threw=true;}require(threw,"Station builder accepted invalid terrain");
        malformed=station;malformed.boxes[0].right=malformed.boxes[0].right*(-1);
        require(hasCode(validateStationDefinition(malformed),"STATION_CONFIG"),"Mirrored station frame accepted");
        malformed=station;auto tilted=std::find_if(malformed.boxes.begin(),malformed.boxes.end(),[](const StationBox& b){return b.role==StationRole::Footing;});
        tilted->forward=rotate(tilted->forward,tilted->right,.0001);tilted->up=rotate(tilted->up,tilted->right,.0001);
        require(validateStationDefinition(malformed).valid(),"Tilted-footing regression should be a valid connected box");
        require(hasCode(validateStation(design.track,design.request.terrain,design.request.train,malformed),"STATION_FOUNDATION"),"Tilted footing escaped the certified world-vertical foundation domain");
        auto invalidTrain=design.request.train;invalidTrain.seatHeight=std::numeric_limits<double>::quiet_NaN();
        require(hasCode(validateStation(design.track,design.request.terrain,invalidTrain,station),"STATION_CONFIG"),"Nonfinite camera/rider height accepted");
        auto tallRider=design.request.train;tallRider.seatHeight=3;malformed=station;
        malformed.boxes.push_back({q.position+q.up*3.,q.tangent,q.right,q.up,{2.,5.1,.05},StationRole::Post});
        require(validateStationDefinition(malformed).valid(),"Tall-rider obstruction is not connected");
        require(hasCode(validateStation(design.track,design.request.terrain,tallRider,malformed),"STATION_CLEARANCE"),"Supported high rider/camera position escapes station envelope");

        require(hasCode(validateStationDefinition(station,[]{return true;}),"CANCELLED"),"Definition cancellation ignored");
        malformed=station;while(malformed.boxes.size()<439)malformed.boxes.push_back(station.boxes[2]);int checkpoints=0;
        require(hasCode(validateStationDefinition(malformed,[&]{return ++checkpoints>12;}),"CANCELLED"),"Connectivity traversal ignored mid-work cancellation");
        require(checkpoints==13,"Cancellation did not stop at its first requested checkpoint");
        auto parseBefore=stationPayload(restored);require(!parseStationPayload(payload,restored,error,[]{return true;}),"Parser cancellation ignored");require(stationPayload(restored)==parseBefore,"Cancelled parse replaced previous station");
        threw=false;try{buildStation(design.track,design.request.terrain,design.request.train,[]{return true;});}catch(...){threw=true;}require(threw,"Builder cancellation ignored");
        if(design.request.terrain.kind==TerrainKind::Hills){
            // Fixed analytic peak, independent of seeded terrain and generated
            // footing margins. A 3x3 footprint grid misses this interior peak
            // by >1.3 m, even though all nine probes enclose their local terrain.
            const Vec3 knownPeak{696.1446760456421,583.5781706660686,0};
            const auto anchor=*std::find_if(station.boxes.begin(),station.boxes.end(),[](const StationBox& b){return b.role==StationRole::Footing;});
            Terrain peakTerrain{TerrainKind::Hills};peakTerrain.headingRadians=std::atan2(anchor.forward.y,anchor.forward.x);
            const double fixtureX=knownPeak.x-200,fixtureY=knownPeak.y,c=std::cos(peakTerrain.headingRadians),sn=std::sin(peakTerrain.headingRadians);
            peakTerrain.offsetX=anchor.center.x-(c*fixtureX-sn*fixtureY);peakTerrain.offsetY=anchor.center.y-(sn*fixtureX+c*fixtureY);
            require(peakTerrain.valid(),"Interior-peak profile is outside the supported domain");
            auto peakTrack=design.track;const auto departure=peakTrack.sample(0).position;
            const double shift=peakTerrain.height(departure.x,departure.y)+20-departure.z;
            for(auto& knot:peakTrack.knots)knot.position.z+=shift;peakTrack.rebuild();
            auto peakStation=buildStation(peakTrack,peakTerrain,design.request.train);
            auto large=*std::find_if(peakStation.boxes.begin(),peakStation.boxes.end(),[](const StationBox& b){return b.role==StationRole::Footing;});
            large.half.x=400;large.half.y=50;double low=1e9,high=-1e9;
            for(double x:{-large.half.x,0.,large.half.x})for(double y:{-large.half.y,0.,large.half.y}){auto p=large.center+large.forward*x+large.right*y;double h=peakTerrain.height(p.x,p.y);low=std::min(low,h);high=std::max(high,h);}
            const double bottom=low-.4,top=high+.021;large.center.z=(bottom+top)*.5;large.half.z=(top-bottom)*.5;
            for(double x:{-large.half.x,0.,large.half.x})for(double y:{-large.half.y,0.,large.half.y}){auto p=large.center+large.forward*x+large.right*y;double h=peakTerrain.height(p.x,p.y);require(bottom<=h+.02&&top>=h+.02,"Sparse footprint probes do not actually pass their old terrain test");}
            const Vec3 peakWorld{peakTerrain.offsetX+c*knownPeak.x-sn*knownPeak.y,peakTerrain.offsetY+sn*knownPeak.x+c*knownPeak.y,0};
            const Vec3 peakOffset=peakWorld-large.center;
            require(std::abs(dot(peakOffset,large.forward))<large.half.x&&std::abs(dot(peakOffset,large.right))<large.half.y,"Known terrain peak is outside the footprint interior");
            const double interiorGap=peakTerrain.height(peakWorld.x,peakWorld.y)+.02-top;
            require(interiorGap>1.3,"Sparse-footprint regression no longer exposes the interior peak");
            malformed=peakStation;malformed.boxes.push_back(large);
            require(validateStationDefinition(malformed).valid(),"Anchoring counterexample is disconnected or malformed");
            require(hasCode(validateStation(peakTrack,peakTerrain,design.request.train,malformed),"STATION_FOUNDATION"),"Nine-point anchoring false acceptance remains");
            large.half.x=large.half.y=512;malformed=peakStation;malformed.boxes.push_back(large);
            require(hasCode(validateStation(peakTrack,peakTerrain,design.request.train,malformed),"STATION_FOUNDATION_DOMAIN"),"Oversized footprint bypassed bounded work budget");
            std::cout<<"Independent interior-peak fixture: nineProbeTop="<<top<<" interiorGap="<<interiorGap<<" m\n";
        }

        std::ofstream out(std::filesystem::absolute(argv[0]).parent_path()/(name+"-station.txt"));out<<payload;
        std::cout<<"Station passes: "<<name<<" parts="<<station.boxes.size()<<"\n";
    }
    std::cout<<"PASS "<<checks<<" station checks\n";return 0;
}catch(const std::exception& e){std::cerr<<"FAIL after "<<checks<<": "<<e.what()<<"\n";return 1;}}
