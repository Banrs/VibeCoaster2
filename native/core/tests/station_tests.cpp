#include "coaster/coaster.hpp"
#include <filesystem>
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
    std::string error;StationGeometry disabled,parsed;require(parseStationPayload(stationPayload(disabled),parsed,error),"Empty legacy station failed");
    require(!parsed.enabled&&parsed.boxes.empty(),"Legacy station acquired geometry");
    std::vector<std::pair<std::string,Design>> fixtures;
    if(argc==1){
        {GenerationRequest request;request.seed=1;request.targets.requireIntensity=false;
            auto generated=generate(request);if(!generated.accepted())std::cerr<<reportJson(generated)<<'\n';require(generated.accepted(),"Current default station fixture was not accepted");
            const auto name="station-generated-"+std::to_string(request.seed)+"-"+request.terrain.name();
            fixtures.emplace_back(name,std::move(generated));
        }
    }else for(int i=1;i<argc;++i){Design design;require(loadDesign(argv[i],design,error),error.c_str());fixtures.emplace_back(std::filesystem::path(argv[i]).stem().string(),std::move(design));}
    for(auto& [name,design]:fixtures){
        auto station=buildStation(design.track,design.request.terrain,design.request.train);
        auto countRole=[&](StationRole role){return std::count_if(station.boxes.begin(),station.boxes.end(),
            [=](const StationBox& part){return part.role==role;});};
        require(countRole(StationRole::HoldingLane)==design.request.train.cars,"Holding lanes do not match physical row modules");
        require(countRole(StationRole::BoardingGate)==design.request.train.cars,"Boarding gates do not match physical row modules");
        require(countRole(StationRole::QueueRail)==3&&countRole(StationRole::DispatchCabin)==1,
            "Covered controlled queue or dispatch cabin is missing");
        require(countRole(StationRole::Lift)==2&&countRole(StationRole::Stair)==2&&
            countRole(StationRole::Underpass)==1,"Separate step-free entry and exit are missing");
        const auto stationOrigin=design.track.sample(0).position;
        for(int row=0;row<design.request.train.cars;++row){
            const double wanted=30+row*design.request.train.spacing;
            const auto found=std::find_if(station.boxes.begin(),station.boxes.end(),
                [&](const StationBox& part){return part.role==StationRole::HoldingLane&&
                    std::abs(dot(part.center-stationOrigin,part.forward)-wanted)<1e-6;});
            require(found!=station.boxes.end(),"Holding lane is not aligned with its parked train row");
        }
        auto entryStair=std::find_if(station.boxes.begin(),station.boxes.end(),
            [&](const StationBox& b){return b.role==StationRole::Stair&&dot(b.center-stationOrigin,b.right)>0;});
        auto entryLift=std::find_if(station.boxes.begin(),station.boxes.end(),
            [&](const StationBox& b){return b.role==StationRole::Lift&&dot(b.center-stationOrigin,b.right)>0;});
        auto queueDeck=std::find_if(station.boxes.begin(),station.boxes.end(),
            [](const StationBox& b){return b.role==StationRole::QueueDeck;});
        auto mergeDeck=std::find_if(station.boxes.begin(),station.boxes.end(),
            [](const StationBox& b){return b.role==StationRole::MergeDeck;});
        require(entryStair!=station.boxes.end()&&entryLift!=station.boxes.end()&&
            queueDeck!=station.boxes.end()&&mergeDeck!=station.boxes.end(),
            "Accessible entry route lacks a stair, lift, queue or merge deck");
        const double queueTop=dot(queueDeck->center-stationOrigin,entryStair->up)+queueDeck->half.z;
        const double mergeTop=dot(mergeDeck->center-stationOrigin,entryStair->up)+mergeDeck->half.z;
        const double riser=(mergeTop-queueTop)/24;
        require(riser>=.16&&riser<=.18,"Entry stair requires unsafe riser height");
        const double stairY=dot(entryStair->center-stationOrigin,entryStair->right);
        const double queueY=dot(queueDeck->center-stationOrigin,entryStair->right);
        const double mergeY=dot(mergeDeck->center-stationOrigin,entryStair->right);
        auto overlaps=[](double a,double ah,double b,double bh){return std::abs(a-b)<=ah+bh;};
        const double stepHalfY=entryStair->half.y/24;
        const double topTreadY=stairY-entryStair->half.y+stepHalfY;
        const double lowTreadY=stairY+entryStair->half.y-stepHalfY;
        require(overlaps(topTreadY,stepHalfY,mergeY,mergeDeck->half.y)&&
                overlaps(lowTreadY,stepHalfY,queueY,queueDeck->half.y),
                "Stair treads do not land on both walking decks");
        for(int i=0;i<24;++i){
            const double y=topTreadY+2*stepHalfY*i,treadTop=mergeTop-riser*i;
            for(const auto& roof:station.boxes)if(roof.role==StationRole::RouteRoof){
                const Vec3 delta=roof.center-entryStair->center;
                if(!overlaps(dot(delta,entryStair->forward),roof.half.x,0,entryStair->half.x)||
                   !overlaps(dot(roof.center-stationOrigin,entryStair->right),roof.half.y,y,stepHalfY))
                    continue;
                const double bottom=dot(roof.center-stationOrigin,entryStair->up)-roof.half.z;
                const double top=bottom+2*roof.half.z;
                require(bottom>=treadTop+2.2||top<treadTop-.1,
                        "Queue roof cuts the stair walking/headroom corridor");
            }
        }
        for(const auto& roof:station.boxes)if(roof.role==StationRole::RouteRoof)
            require(!stationBoxesOverlap(*entryLift,roof),"Queue roof cuts the step-free lift shaft");
        auto report=validateStation(design.track,design.request.terrain,design.request.train,station);
        for(auto& e:report.errors)std::cerr<<e.code<<" "<<e.message<<" s="<<e.distance<<" part="<<e.actual<<"\n";
        require(report.valid(),"Canonical station rejected accepted ride");
        for(const auto& part:station.boxes){
            if(part.role!=StationRole::QueueDeck&&part.role!=StationRole::ExitWalkway&&part.role!=StationRole::Underpass)continue;
            if(part.center.z>stationOrigin.z-1)continue; // elevated exit bridge
            const double walkingTop=part.role==StationRole::Underpass?
                part.center.z-part.half.z+.20:part.center.z+part.half.z;
            for(int ix=-2;ix<=2;++ix)for(int iy=-2;iy<=2;++iy){
                const Vec3 p=part.center+part.forward*(part.half.x*ix/2.)+
                    part.right*(part.half.y*iy/2.);
                require(design.request.terrain.height(p.x,p.y)<=walkingTop+.20,
                    "Station ground route is buried in uncut terrain");
            }
        }
        if(!design.supports.empty()&&std::all_of(design.supports.begin(),design.supports.end(),
            [](const Support& support){return !support.members.empty();})){
            auto integrated=design;integrated.station=station;
            auto structures=validateDesignStructures(integrated);
            for(const auto& e:structures.errors)std::cerr<<"structure: "<<e.code<<" "<<e.message<<" s="<<e.distance<<"\n";
            require(structures.valid(),"New station collides with retained canonical supports");
        }
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
        if(design.request.train.cars!=7){
            auto seven=design.request.train;seven.cars=7;
            auto sevenStation=buildStation(design.track,design.request.terrain,seven);
            auto sevenReport=validateStation(design.track,design.request.terrain,seven,sevenStation);
            for(const auto& e:sevenReport.errors)std::cerr<<"seven-row: "<<e.code<<" "<<e.message<<" s="<<e.distance<<" part="<<e.actual<<"\n";
            require(sevenReport.valid(),"Seven-row physical stop failed station clearance on saved track");
            require(std::count_if(sevenStation.boxes.begin(),sevenStation.boxes.end(),
                [](const StationBox& part){return part.role==StationRole::HoldingLane;})==7,
                "Seven-row adaptation did not create seven loading lanes");
        }
        std::cout<<"Station passes: "<<name<<" cars="<<design.request.train.cars
                 <<" parts="<<station.boxes.size()<<"\n";
    }
    std::cout<<"PASS "<<checks<<" station checks\n";return 0;
}catch(const std::exception& e){std::cerr<<"FAIL after "<<checks<<": "<<e.what()<<"\n";return 1;}}
