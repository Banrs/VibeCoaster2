#include "coaster/coaster.hpp"
#include "coaster/layout_modules.hpp"
#include "coaster/fvd.hpp"
#include "baseline_jets.hpp"
#include "flow_bridge.hpp"
#include "terrain_module_placement.hpp"
#include "passive_transfer.hpp"
#include "terrain_motion.hpp"
#include "bank_target.hpp"
#include "turn_shape.hpp"
#include <numeric>
#include <optional>
#include <memory>
#include <stdexcept>
#include <sstream>
#include <iomanip>

namespace coaster {
struct SourceFamilyUnsupported : std::runtime_error {using std::runtime_error::runtime_error;};
constexpr double departureRampSeconds=.16;
struct Random {
    uint64_t state;
    uint64_t next(){uint64_t z=(state+=0x9e3779b97f4a7c15ull);z=(z^(z>>30))*0xbf58476d1ce4e5b9ull;z=(z^(z>>27))*0x94d049bb133111ebull;return z^(z>>31);}
    double range(double a,double b){return a+(b-a)*double(next()>>11)*0x1.0p-53;}
};


// Flat departure sizing is a planning estimate only. The canonical finite-train
// simulator independently measures the first actual crossing of 50 m/s.
static double plannedLaunchTime(double motorAcceleration,const TrainConfig& train){
    constexpr double dt=.0005;double speed=0,time=0,drag=.5*train.airDensity*train.dragCdA/(train.cars*train.carMass);
    auto acceleration=[&](double v,double t){double external=motorAcceleration*smooth(t/departureRampSeconds)-drag*v*v,friction=gravity*train.rollingResistance;return v>0?external-friction:std::max(0.,external-friction);};
    while(time<10){double a=acceleration(speed,time),mid=std::max(0.,speed+a*dt*.5),next=std::max(0.,speed+acceleration(mid,time+dt*.5)*dt);if(next>=50)return time+dt*(50-speed)/(next-speed);speed=next;time+=dt;}return std::numeric_limits<double>::infinity();
}
static double sizedLaunchAcceleration(const GenerationRequest& req,double seedPreference){
    double upper=req.limits.maxLongitudinalG*gravity-.02,lower=0;
    for(int i=0;i<32;++i){double mid=(lower+upper)*.5;if(plannedLaunchTime(mid,req.train)>req.targets.launchSeconds-.003)lower=mid;else upper=mid;}
    return std::min(req.limits.maxLongitudinalG*gravity-.02,std::max(seedPreference,upper));
}
using detail::TurnShape;
using detail::makeTurn;
struct RoutePlan {
    bool stationChecked{},stationFeasible{},stationSelectionEligible{};double stationBayMinimum{},stationBayMaximum{},stationRequiredDatum{};
    int shape{},placement{},transferFailures{};double heading{},length{},score{},relief{},deviation{},grade{},stationGrade{},groundMinimum{},groundMaximum{},valleyFraction{},valleyDistance{};int valleyCrossings{};
    Vec3 origin;std::vector<double> angles,lengths;
};
constexpr double stationReturnLength=100;
// Intamin reports a 150 km/h powered ascent for Falcon's Flight. This is
// a distinct climb intent; signature speed targets remain unchanged.
constexpr double terrainAscentSpeed=150/3.6;
// Normal service braking is distinct from the extreme departure launch.
// These mechanical design intents remain subject to actual signed rider loads.
constexpr double serviceBrakeAcceleration=7,terminalDeceleration=6;
static double stationBankFactor(double remaining,const TrainConfig& train){double upright=std::max(40.,(train.cars-1)*train.spacing*.5+18);return smooth((remaining-upright)/100);}
static double layoutWarp(double u,double shape){return u+shape*std::sin(2*pi*u)/(2*pi);}
static double layoutSmooth(double u){u=std::clamp(u,0.,1.);return u*u*u*u*(35+u*(-84+u*(70-20*u)));}
static Vec3 inFrame(Vec3 p,double h){return {p.x*std::cos(h)-p.y*std::sin(h),p.x*std::sin(h)+p.y*std::cos(h),p.z};}
static double canyonAcross(const Terrain& terrain,Vec3 point){
    const Vec3 local=inFrame((point-Vec3{terrain.offsetX,terrain.offsetY,0})/terrain.horizontalScale,-terrain.headingRadians);
    return local.y-120*std::sin(local.x/850);
}
static bool onCanyonFloor(const Terrain& terrain,Vec3 point){return std::abs(canyonAcross(terrain,point))<=260;}
static std::vector<RoutePlan> planRoutes(const GenerationRequest& req,int sides,const std::vector<double>& minimum,const std::array<double,4>& radii,const std::array<double,4>& ramps,const std::array<double,4>& banks,double terminalReturnMinimum,double loopApproachLength,double loopTrimLength,const std::array<LayoutModulePose,4>& sourceExits,const std::array<detail::TerrainMotion,4>& terrainMotion,const std::array<detail::TerrainModuleFootprint,4>& footprints,int pinnedShape,int pinnedPlacement,Random& rng,Cancel cancel){
    std::vector<RoutePlan> feasible;
    const bool dramaticTerrain=req.terrain.kind==TerrainKind::Canyon&&!req.terrain.isDefaultProfile();
    const double reliefLength=detail::minimumTerrainMotionLength(req.terrain.cliffHeight,2.,terrainMotion[0],cancel);
    std::array<double,4> sourceHeadings{};
    for(int side=0;side<sides;++side)sourceHeadings[side]=std::atan2(sourceExits[side].forward.y,sourceExits[side].forward.x);
    double startingHeading=rng.range(-pi,pi);Vec3 stationAnchor{rng.range(-200,200),rng.range(-160,160),0};
    for(int shape=0;shape<48;++shape){
        Random shapeRng{rng.next()};
        if(cancel&&cancel())throw std::runtime_error("CANCELLED");
        if(pinnedShape>=0&&shape!=pinnedShape)continue;
        RoutePlan p;p.shape=shape;p.angles.resize(sides);p.lengths=minimum;
        // The reversal supplies its own half-turn. Place the long source runs
        // on opposing legs so exact closure does not demand a long reset lane.
        // The fourth turn closes the actual integrated exit heading.
        const double alphaSeed=shapeRng.range(80,100)*pi/180,betaSeed=shapeRng.range(80,100)*pi/180,gammaSeed=shapeRng.range(110,140)*pi/180;
        for(int placement=0;placement<36;++placement){
            if(pinnedShape>=0&&placement!=pinnedPlacement)continue;
            auto q=p;q.placement=placement;
            // Shape, orientation and anchored station placement are jointly
            // ranked from actual terrain; the landscape is never modified.
            q.heading=startingHeading+(placement/3)*pi/6;
            q.origin=stationAnchor+inFrame({0,double(placement%3-1)*180,0},q.heading);
            if(dramaticTerrain){
                // Place the departure signature along the actual canyon floor.
                // Keep the same bounded36 sites: three longitudinal positions,
                // both travel directions and six nearby tangent orientations.
                const double stationAlong=stationAnchor.x+600*(placement%3-1);
                const double stationAcross=120*std::sin(stationAlong/850)+stationAnchor.y*.5;
                q.origin=Vec3{req.terrain.offsetX,req.terrain.offsetY,0}+inFrame(Vec3{stationAlong,stationAcross,0}*req.terrain.horizontalScale,req.terrain.headingRadians);
                q.heading=req.terrain.headingRadians+(placement/18)*pi+std::atan((120./850)*std::cos(stationAlong/850))+
                    (placement/3%6-2.5)*5*pi/180;
            }
            // Terrain headings are world-relative: rotating the station must
            // not make one cliff crossing oblique while steepening the other.
            const double orientation=dramaticTerrain?std::remainder(q.heading-req.terrain.headingRadians,pi):0;
            const double alpha=alphaSeed-orientation,beta=betaSeed+orientation;
            q.angles={alpha,beta,-gammaSeed,std::remainder(gammaSeed-alpha-beta-std::accumulate(sourceHeadings.begin(),sourceHeadings.end(),0.),2*pi)};
            std::array<TurnShape,4> turns;
            std::vector<Vec3> directions;Vec3 sum{stationReturnLength,0,0};double h=0;
            for(int side=0;side<sides;++side){
                directions.push_back(inFrame(sourceExits[side].forward,h));
                sum=sum+inFrame(sourceExits[side].position,h)-directions.back()*minimum[side];h+=sourceHeadings[side];
                turns[side]=makeTurn(q.angles[side],radii[side],ramps[side],banks[side]);sum=sum+inFrame(turns[side].points.back(),h);h+=q.angles[side];}
            // Exact closure leaves two free corridor lengths. Solve their small
            // feasible polygon instead of fixing a randomly padded first leg and
            // charging the resulting slack to the next element's recovery.
            auto baseBounds=minimum;
            // The shared approach is geometry. Motor and brake work fit inside
            // it; changing feasible trim demand does not move the loop source.
            const double approachMinimum=std::max(loopTrimLength,dramaticTerrain?reliefLength:0.);
            baseBounds[0]+=std::max(0.,approachMinimum-turns[0].length-loopApproachLength);
            baseBounds[3]+=std::max(0.,terminalReturnMinimum-turns[3].length-stationReturnLength);
            Vec3 x=directions[2],y=directions[3];double determinant=cross(x,y).z;
            if(std::abs(determinant)<.2)continue;
            auto closeFor=[&](double first,double second){Vec3 rhs=(sum+directions[0]*first+directions[1]*second)*(-1);return Vec3{cross(rhs,y).z/determinant,cross(x,rhs).z/determinant,0};};
            Vec3 origin=closeFor(0,0),firstSlope=closeFor(1,0)-origin,secondSlope=closeFor(0,1)-origin;
            const std::array<Vec3,4> coefficients{{{1,0,0},{0,1,0},{firstSlope.x,secondSlope.x,origin.x},{firstSlope.y,secondSlope.y,origin.y}}};
            auto bounds=baseBounds;
            if(dramaticTerrain){
                // The final turn is anchored to the station. Solve where the
                // last rigid source can end on the real rim along its return ray.
                const double finalHeading=q.heading-q.angles[3];
                auto tailPoint=[&](double length){return q.origin-inFrame({stationReturnLength,0,0},q.heading)-inFrame(Vec3{length-minimum[3],0,0}+turns[3].points.back(),finalHeading);};
                auto rim=[&](double length){return std::abs(canyonAcross(req.terrain,tailPoint(length)))-260-req.terrain.cliffWidth;};
                if(rim(bounds[3])<0){
                    double lo=bounds[3],hi=lo;
                    while(hi<2800&&rim(hi)<0){lo=hi;hi=std::min(2800.,hi+20);}
                    if(rim(hi)<0)continue;
                    for(int iteration=0;iteration<48;++iteration){const double mid=(lo+hi)*.5;if(rim(mid)<0)lo=mid;else hi=mid;}
                    bounds[3]=hi+1e-6;
                }
            }
            struct Boundary{double x,y,value;};std::array<Boundary,9> boundaries;
            for(int side=0;side<sides;++side){const auto c=coefficients[side];boundaries[2*side]={c.x,c.y,bounds[side]-c.z};boundaries[2*side+1]={-c.x,-c.y,c.z-2800};}
            // The footprint budget is another linear closure constraint. A
            // terrain-feasible longer vertex must remain inside that budget.
            Boundary budget{0,0,stationReturnLength-9800};
            for(int side=0;side<sides;++side){budget.x-=coefficients[side].x;budget.y-=coefficients[side].y;budget.value+=turns[side].length+coefficients[side].z;}
            boundaries.back()=budget;
            std::vector<std::vector<double>> closureVertices;
            double shortest=INFINITY;
            for(size_t i=0;i<boundaries.size();++i)for(size_t j=i+1;j<boundaries.size();++j){
                const auto a=boundaries[i],b=boundaries[j];const double det=a.x*b.y-a.y*b.x;
                if(std::abs(det)<1e-10)continue;
                const double first=(a.value*b.y-a.y*b.value)/det,second=(a.x*b.value-a.value*b.x)/det;
                bool valid=true;for(const auto& bound:boundaries)if(bound.x*first+bound.y*second<bound.value-1e-7){valid=false;break;}
                if(!valid)continue;
                std::vector<double> lengths(sides);double total=0;
                for(int side=0;side<sides;++side){const auto c=coefficients[side];lengths[side]=c.x*first+c.y*second+c.z;total+=lengths[side];}
                if(std::none_of(closureVertices.begin(),closureVertices.end(),[&](const auto& vertex){return std::abs(vertex[0]-lengths[0])+std::abs(vertex[1]-lengths[1])<1e-7;}))closureVertices.push_back(lengths);
                if(total<shortest){shortest=total;q.lengths=std::move(lengths);}
            }
            if(!std::isfinite(shortest))continue;
            const auto planTemplate=q;
            // Closure and ranking use the same terrain paths, source datums
            // and signed motion screen. Nominal cliff height only supplies the
            // initial linear bound; it cannot certify an actual terrain rise.
            auto assessTerrain=[&](const std::vector<double>& lengths)->std::optional<RoutePlan>{
                auto q=planTemplate;q.lengths=lengths;
            if(dramaticTerrain){const Vec3 tail=q.origin-inFrame({stationReturnLength,0,0},q.heading)-inFrame(Vec3{q.lengths[3]-minimum[3],0,0}+turns[3].points.back(),q.heading-q.angles[3]);
                if(std::abs(canyonAcross(req.terrain,tail))<260+req.terrain.cliffWidth-1e-7)return std::nullopt;}

            bool valid=true;q.length=stationReturnLength;
            for(int side=0;side<sides;++side){if(q.lengths[side]<bounds[side]-1e-7||q.lengths[side]>2800+1e-7)valid=false;q.length+=q.lengths[side]+turns[side].length;}
            // Major hills/inversion add approximately 400--650 m of 3-D rail to
            // this horizontal budget. Do not build a huge fallback corridor.
            if(!valid||q.length>9800+1e-7)return std::nullopt;
            std::vector<Vec3> corridor;Vec3 cursor{};double h=0;
            std::array<std::vector<Vec3>,2> transferPaths;
            std::array<std::pair<size_t,size_t>,2> transferTurnRange;
            auto append=[](std::vector<Vec3>& path,Vec3 point){if(path.empty()||norm(point-path.back())>1e-7)path.push_back(point);};
            auto line=[&](std::vector<Vec3>& path,Vec3 first,Vec3 last){const int n=std::max(1,int(std::ceil(norm(last-first)/20)));for(int i=0;i<=n;++i)append(path,first+(last-first)*(double(i)/n));};
            std::array<detail::TerrainModuleFootprint,4> placedFootprints;
            constexpr std::array<int,4> footprintCorridor{1,2,1,3};
            for(int side=0;side<sides;++side){
                for(size_t index=0;index<footprints.size();++index)if(footprintCorridor[index]==side){
                    const auto& source=footprints[index];auto& placed=placedFootprints[index];
                    placed.entrance=cursor+inFrame(source.entrance,h);
                    for(const auto& point:source.samples)placed.samples.push_back(cursor+inFrame(point,h));
                    for(auto frame:source.rigidFrames){
                        frame.position=cursor+inFrame(frame.position,h);frame.tangent=inFrame(frame.tangent,h);
                        frame.up=inFrame(frame.up,h);frame.right=inFrame(frame.right,h);placed.rigidFrames.push_back(frame);
                    }
                }
                // Continue along each source's actual exit ray. The Immelmann
                // changes heading itself; no turnaround is needed to undo it.
                const Vec3 sourceBegin=cursor;
                const double corridorLength=q.lengths[side]-minimum[side];
                if(side==2||side==3){
                    const auto& source=placedFootprints[side==2?1:3];
                    line(corridor,cursor,source.entrance);
                    for(const auto& point:source.samples)corridor.push_back(point);
                }else line(corridor,cursor,cursor+inFrame(sourceExits[side].position,h));
                cursor=cursor+inFrame(sourceExits[side].position,h);h+=sourceHeadings[side];
                int n=int(std::ceil(corridorLength/25));Vec3 forward{std::cos(h),std::sin(h),0};
                if(dramaticTerrain){
                    if(side==0)line(transferPaths[0],cursor,cursor+forward*corridorLength);
                    if(side==1)line(transferPaths[0],sourceBegin,placedFootprints[0].entrance);
                    if(side==3)line(transferPaths[1],cursor,cursor+forward*corridorLength);
                }
                if(n>0)for(int i=0;i<=n;++i)corridor.push_back(cursor+forward*(corridorLength*i/n));cursor=cursor+forward*corridorLength;
                for(size_t i=14;i<turns[side].points.size();i+=14)corridor.push_back(cursor+inFrame(turns[side].points[i],h));
                if(dramaticTerrain&&(side==0||side==3)){
                    const int index=side==0?0:1;auto& path=transferPaths[index];
                    transferTurnRange[index].first=path.size()-1;
                    for(size_t i=1;i<turns[side].points.size();++i)append(path,cursor+inFrame(turns[side].points[i],h));
                    append(path,cursor+inFrame(turns[side].points.back(),h));
                    transferTurnRange[index].second=path.size()-1;
                }
                cursor=cursor+inFrame(turns[side].points.back(),h);if(side==2)cursor.z-=sourceExits[side].position.z;h+=q.angles[side];
            }
            const Vec3 stationApproachEnd=cursor+Vec3{stationReturnLength,0,0};
            corridor.push_back(stationApproachEnd);cursor=stationApproachEnd;
            if(norm(cursor)>1e-6)return std::nullopt;
            std::array<std::vector<double>,2> transferDistance;
            if(dramaticTerrain){
                for(size_t index=0;index<transferPaths.size();++index){
                    const auto& path=transferPaths[index];auto& distance=transferDistance[index];distance.push_back(0);
                    for(size_t i=1;i<path.size();++i)distance.push_back(distance.back()+std::hypot(path[i].x-path[i-1].x,path[i].y-path[i-1].y));
                }
            }
            double station=req.terrain.height(q.origin.x,q.origin.y),lo=1e9,hi=-1e9,totalDeviation=0,slope2=0,valleyNear=0,totalValleyDistance=0;Vec3 previous{};double previousGround=0,previousValley=0;bool first=true;
            for(const auto& local:corridor){
                Vec3 point=q.origin+inFrame(local,q.heading);double ground=req.terrain.height(point.x,point.y);lo=std::min(lo,ground);hi=std::max(hi,ground);totalDeviation+=std::abs(ground-station);
                Vec3 terrainPoint=inFrame({(point.x-req.terrain.offsetX)/req.terrain.horizontalScale,(point.y-req.terrain.offsetY)/req.terrain.horizontalScale,0},-req.terrain.headingRadians);
                double valley=terrainPoint.y-120*std::sin(terrainPoint.x/850);totalValleyDistance+=std::abs(valley);valleyNear+=std::abs(valley)<210?1:0;if(!first&&valley*previousValley<0)++q.valleyCrossings;previousValley=valley;
                if(!first){double span=norm(point-previous);if(span>1)slope2+=std::pow((ground-previousGround)/span,2);}first=false;previous=point;previousGround=ground;
            }
            q.groundMinimum=lo;q.groundMaximum=hi;q.valleyFraction=valleyNear/corridor.size();q.valleyDistance=totalValleyDistance/corridor.size();
            q.relief=hi-lo;q.deviation=totalDeviation/corridor.size();q.grade=std::sqrt(slope2/corridor.size());
            Vec3 stationEnd=q.origin+inFrame({80,0,0},q.heading);q.stationGrade=std::abs(req.terrain.height(stationEnd.x,stationEnd.y)-station)/80;
            if(req.terrain.kind==TerrainKind::Canyon&&!dramaticTerrain&&q.valleyFraction<.12)return std::nullopt;
            q.score=dramaticTerrain?2*std::abs(q.relief-req.terrain.cliffHeight)+.1*q.deviation+200*q.grade+350*q.stationGrade+q.length/65+100*(1-q.valleyFraction):1.5*q.relief+.5*q.deviation+200*q.grade+150*q.stationGrade+q.length/65+(req.terrain.kind==TerrainKind::Canyon?100*(1-q.valleyFraction):0);
            // Rank every rigid force-designed interior, including the complete
            // tail chain. A low rim entry can still leave its exit on a tower
            // above the canyon floor, so assess both ports and the full footprint.
            std::array<detail::TerrainModulePlacement,4> modulePlacement;
            std::array<double,4> sourceDatums{};
            for(size_t index=0;index<placedFootprints.size();++index){
                const auto& footprint=placedFootprints[index];
                auto& assessment=modulePlacement[index];
                assessment=detail::assessTerrainModulePlacement(req.terrain,q.origin,q.heading,footprint,req.limits.minClearance+4.5);
                const auto& exit=footprint.samples.back();const Vec3 worldExit=q.origin+inFrame(exit,q.heading);
                const double exitGroundHeight=assessment.minimumDatum+exit.z-req.terrain.height(worldExit.x,worldExit.y);
                q.score+=assessment.score+2*std::max(0.,exitGroundHeight-24.);
                sourceDatums[index]=assessment.minimumDatum;
                if(!footprint.rigidFrames.empty()){
                    double datum=-INFINITY;
                    for(size_t sample=0;sample<footprint.rigidFrames.size();++sample){
                        auto frame=footprint.rigidFrames[sample];frame.position=q.origin+inFrame(frame.position,q.heading);
                        frame.tangent=inFrame(frame.tangent,q.heading);frame.up=inFrame(frame.up,q.heading);frame.right=inFrame(frame.right,q.heading);
                        const int side=footprintCorridor[index];
                        const bool turnPort=index!=0&&sample+1==footprint.rigidFrames.size()&&q.lengths[side]-minimum[side]<=1e-6;
                        datum=std::max(datum,detail::terrainEnvelopeDatum(req.terrain,req.limits,frame,turnPort));
                    }
                    sourceDatums[index]=datum+.3;
                }
            }
            // The fixed post-loop drive must fit the same floor-constrained
            // timing family as composition. Endpoint relief alone cannot see
            // a cliff or small ridge forcing a steep, high-load warped profile.
            const Vec3 launchFirst=placedFootprints[0].samples.back(),launchLast=placedFootprints[2].entrance;
            const double span=std::hypot(launchLast.x-launchFirst.x,launchLast.y-launchFirst.y);
            const int transferSamples=std::max(1,int(std::ceil(span/1.8)));
            std::vector<double> launchDistance(transferSamples+1),launchFloor(transferSamples+1);
            for(int sample=0;sample<=transferSamples;++sample){
                const double u=double(sample)/transferSamples;const Vec3 world=q.origin+inFrame(launchFirst+(launchLast-launchFirst)*u,q.heading);
                const Vec3 forward=unit(inFrame(launchLast-launchFirst,q.heading)),up{0,0,1};
                const TrackSample frame{world,forward,{},up,cross(forward,up),Element::Launch};
                launchDistance[sample]=span*u;launchFloor[sample]=detail::terrainEnvelopeDatum(req.terrain,req.limits,frame)+world.z;
            }
            const size_t high=sourceDatums[0]+launchFirst.z>sourceDatums[2]+launchLast.z?0:2;
            sourceDatums[high]=std::max(sourceDatums[high],*std::max_element(launchFloor.begin(),launchFloor.end())+.3-(high==0?launchFirst.z:launchLast.z));
            try{detail::fitTerrainMotionTransfer(launchDistance,launchFloor,
                sourceDatums[0]+launchFirst.z,sourceDatums[2]+launchLast.z,2.,terrainMotion[1],cancel);}
            catch(const detail::TerrainTransferInfeasible&){++q.transferFailures;}
            if(dramaticTerrain){
                // Assess the whole authored climb and return, including their
                // real turns. Estimated source datums and centreline floors
                // screen route feasibility; complete envelope/physics still gate.
                const double clearance=req.limits.minClearance+4.5;
                double stationDatum=std::max(station,req.terrain.height(stationEnd.x,stationEnd.y))+clearance;
                for(const auto& local:transferPaths[0]){const Vec3 world=q.origin+inFrame(local,q.heading);
                    if(!onCanyonFloor(req.terrain,world))break;
                    stationDatum=std::max(stationDatum,req.terrain.height(world.x,world.y)+clearance);
                }
                const std::array<double,2> starts{stationDatum,sourceDatums[3]+placedFootprints[3].samples.back().z};
                const std::array<double,2> finishes{sourceDatums[0]+placedFootprints[0].entrance.z,stationDatum};
                for(size_t index=0;index<transferPaths.size();++index){
                    const auto& path=transferPaths[index];const auto& distance=transferDistance[index];std::vector<double> floor;floor.reserve(path.size());
                    for(size_t sample=0;sample<path.size();++sample){
                        const Vec3 world=q.origin+inFrame(path[sample],q.heading);
                        const Vec3 forward=unit(inFrame(path[std::min(sample+1,path.size()-1)]-path[sample?sample-1:0],q.heading)),up{0,0,1};
                        const TrackSample frame{world,forward,{},up,cross(forward,up),Element::Return};
                        floor.push_back(detail::terrainEnvelopeDatum(req.terrain,req.limits,frame,sample>=transferTurnRange[index].first&&sample<=transferTurnRange[index].second)+world.z);
                    }
                    try{
                        const auto profile=detail::fitTerrainMotionWindow(distance,floor,starts[index],finishes[index],2.,terrainMotion[index==0?0:3],cancel);
                        double excess=0,previousExcess=std::max(0.,profile.height(0)-floor.front()+clearance-24.);
                        for(size_t i=1;i<path.size();++i){const double height=std::max(0.,profile.height(distance[i])-floor[i]+clearance-24.);
                            excess+=(previousExcess+height)*.5*(distance[i]-distance[i-1]);previousExcess=height;}
                        // Integrated support exposure must not reward extending a neutral
                        // corridor merely to dilute its mean pedestal height.
                        q.score+=excess/std::max(1.,reliefLength);
                        const double necessaryLevel=index==0?0.:stationReturnLength;
                        const double neutral=profile.activeStart+std::max(0.,distance.back()-profile.activeStart-profile.activeLength-necessaryLevel);
                        if(profile.activeLength>0)q.score+=neutral/65;
                    }catch(const detail::TerrainTransferInfeasible&){++q.transferFailures;}
                }
            }
                return q;
            };
            auto selected=assessTerrain(q.lengths);
            if(pinnedShape>=0&&(!selected||selected->transferFailures)){
                selected.reset();
                const auto shortestVertex=q.lengths;
                // Every segment lies inside the convex linear closure polygon.
                // Keep an explicitly checked feasible endpoint while locating a
                // boundary. Terrain need not be monotone along that segment;
                // this bounded family does not claim global optimality.
                for(const auto& vertex:closureVertices){
                    auto endpoint=assessTerrain(vertex);
                    if(!endpoint||endpoint->transferFailures)continue;
                    double low=0,high=1;
                    for(int iteration=0;iteration<32;++iteration){
                        if(cancel&&cancel())throw std::runtime_error("CANCELLED");
                        const double middle=(low+high)*.5;std::vector<double> trial(sides);
                        for(int side=0;side<sides;++side)trial[side]=shortestVertex[side]+middle*(vertex[side]-shortestVertex[side]);
                        auto assessment=assessTerrain(trial);
                        if(assessment&&assessment->transferFailures==0){high=middle;endpoint=std::move(assessment);}else low=middle;
                        if((high-low)*std::abs(std::accumulate(vertex.begin(),vertex.end(),0.)-shortest)<1e-5)break;
                    }
                    if(!selected||endpoint->length<selected->length)selected=std::move(endpoint);
                }
            }
            if(selected&&selected->transferFailures==0)feasible.push_back(std::move(*selected));
        }
    }
    std::stable_sort(feasible.begin(),feasible.end(),[](const RoutePlan& a,const RoutePlan& b){return a.score<b.score;});return feasible;
}
struct AuthoringFeedback {
    std::array<double,2> airtimeSpeed{65,62};
    std::array<double,4> turnSpeed{65,65,65,65};
    double reversalEnergyCorrection{},loopEnergyCorrection{},loopSupplyEnergyCorrection{},relaunchEnergyCorrection{},loopApproachSpeed{65};
    double loopApproachDomain{-1};
    int planShape{-1},planPlacement{-1};
};
struct CandidatePorts {
    struct SpeedIntent {double distance,speed,time,normalG,lateralG;};
    std::array<std::vector<SpeedIntent>,2> airtimeProfile;
    std::array<double,4> turnBegin{},turnEnd{};
    std::array<bool,4> ordinaryTurn{};
    bool loopSupplyIndependent{};
    double loopApproachDomain{};
    double signatureEntry{},signatureExit{},signatureEntrySpeedHint{},signatureExitSpeedHint{};
    double reversalExit{},reversalSpeedHint{},reversalEntry{},reversalBrakeStart{},relaunchEnd{},relaunchTarget{},reversalEntrySpeedHint{},pulloutExit{},pulloutSpeedHint{};
    double loopApex{},loopBrakeStart{},loopEntry{},loopEntrySpeedHint{},loopExit{},loopExitSpeedHint{},loopSupplyEnd{},loopSupplyTarget{},loopSpeedHint{};
    int planShape{-1},planPlacement{-1};
};
struct PreparedSources {
    std::unique_ptr<const FvdTallHillResult> signature;
    std::unique_ptr<const EnergyLoopModule> loop;
};
static double replayValueAt(const std::vector<Frame>& frames,double distance,bool time=false){
    if(frames.empty()||distance<frames.front().distance||distance>frames.back().distance)
        throw std::runtime_error("Authoring feedback at "+std::to_string(distance)+" m is outside actual replay "+
            (frames.empty()?std::string("(empty)"):std::to_string(frames.front().distance)+".."+std::to_string(frames.back().distance)+" m"));
    auto right=std::lower_bound(frames.begin(),frames.end(),distance,[](const Frame& f,double s){return f.distance<s;});
    if(right==frames.begin())return time?right->time:right->speed;
    if(right==frames.end())return time?frames.back().time:frames.back().speed;
    const auto& left=*(right-1);double u=(distance-left.distance)/(right->distance-left.distance);
    return time?left.time+u*(right->time-left.time):left.speed+u*(right->speed-left.speed);
}
double movingRideSeconds(const Design& d){
    for(const auto& op:d.operations)if(op.kind==DriveKind::Station)return replayValueAt(d.simulation.frames,op.start,true);
    throw std::runtime_error("Moving duration requires the terminal station brake");
}
static Design candidate(const GenerationRequest& req,int attempt,Cancel cancel,const AuthoringFeedback& feedback,CandidatePorts& ports,PreparedSources& preparedSources){
    auto& preparedSignature=preparedSources.signature;
    const int geometryAttempt=attempt/2,placementVariant=attempt%2;
    const bool dramaticTerrain=req.terrain.kind==TerrainKind::Canyon&&!req.terrain.isDefaultProfile();
    const double rollingAcceleration=gravity*req.train.rollingResistance;
    const double dragAccelerationCoefficient=.5*req.train.airDensity*req.train.dragCdA/(req.train.cars*req.train.carMass);
    Design d;d.request=req;d.candidate=attempt;Random rng{req.seed};rng.next();constexpr int sides=4;
    rng.next(); // Preserve seeded route choices from the original grammar.
    // Reference exposure is an independently measured circuit target. It does
    // not prescribe an extra lap or turn load by dividing a ten-second score.
    const double designNormalG=rng.range(3.8,4.2);
    // Chosen ordinary-turn twist/onset intentions, not ASTM limits. The
    // quintic bank phase sets both analytically; final terrain placement and
    // measured finite-train forces still decide acceptance.
    constexpr double turnTwistRate=80*pi/180,turnTwistOnset=150*pi/180;
    double elevation=req.targets.height+rng.range(8,30),loopHeight=rng.range(68,78);
    rng.range(1020,1120); // Preserve unrelated seeded route choices after replacing geometric hill width.
    rng.range(330,380); // Preserve unrelated seeded route choices.
    rng.range(7,11); // Source physics now selects supply; retain the route RNG sequence.
    double launchAcceleration=sizedLaunchAcceleration(req,rng.range(38.4,38.9));
    if(!preparedSignature){
        if(elevation<220||elevation>280||req.targets.speed>90){
            std::ostringstream reason;reason<<"Tall signature source family supports selected heights 220..280 m and source speeds 75..90 m/s; selected seeded height "
                <<elevation<<" m, requested minimum speed "<<req.targets.speed<<" m/s. Choose supported targets; broader source families are not authored.";
            throw SourceFamilyUnsupported(reason.str());
        }
        FvdTallHillRequest signatureRequest;signatureRequest.height=elevation;
        signatureRequest.normalG=5.0;signatureRequest.crestG=-1.45;signatureRequest.crestPulseSeconds=2.2;
        signatureRequest.rollingAcceleration=rollingAcceleration;
        signatureRequest.dragAccelerationCoefficient=dragAccelerationCoefficient;
        FvdTallHillResult signature;bool signatureFound=false;
        for(double sourceSpeed=std::max(75.,req.targets.speed);sourceSpeed<=90;sourceSpeed+=.25){
            signatureRequest.speed=sourceSpeed;auto trial=designFvdTallHill(signatureRequest,cancel);
            if(trial.section.cancelled)throw std::runtime_error("CANCELLED");
            if(!trial.section.report.valid()||!trial.section.assessment.passed)continue;
            double minimumSpeed=INFINITY;for(const auto& sample:trial.section.samples)minimumSpeed=std::min(minimumSpeed,sample.speed);
            if(minimumSpeed<25)continue;
            signature=std::move(trial);signatureFound=true;break;
        }
        if(!signatureFound)throw std::runtime_error("Tall signature has no bounded source meeting requested speed and 25 m/s traversal intent");
        preparedSignature=std::make_unique<const FvdTallHillResult>(std::move(signature));
    }
    if(cancel&&cancel())throw std::runtime_error("CANCELLED");
    const auto& signature=*preparedSignature;
    if(std::abs(signature.height-elevation)>1e-4)throw std::runtime_error("Prepared signature height differs from seeded source request");
    const double topSpeed=signature.authoring.speed+.5; // Physical motor margin; actual source agreement is checked.
    const double hillWidth=signature.span;
    rng.range(17,23); // Retain the independent route random sequence.
    // Source variations use a separate stream from route selection.
    Random detail{req.seed^0x8d2f41b79a5c630eull};
    double reversalHeight=std::max(84.,req.targets.inversionHeight+detail.range(7,14));
    detail.range(56,68);detail.range(190,240);
    detail.range(95,125);detail.range(72,96);
    detail.range(-.12,.08);detail.range(-.3,.3);
    detail.range(23,26); // Preserve unrelated seeded source choices during this replacement.
    const int reversalHand=(detail.next()&1)?1:-1;
    detail.range(930,1010);
    // Specify the load history independently of the acceptance ceiling.
    // Loss-aware FVD solves curvature and true displacement; the final
    // finite-train replay independently measures rider-offset forces.
    const double reversalScale=std::sqrt(reversalHeight/88.);
    FvdImmelmannRequest reversalRequest;
    reversalRequest.entrySpeed=53*reversalScale;reversalRequest.height=95*reversalScale*reversalScale;
    reversalRequest.exitHeight=10*reversalScale*reversalScale;reversalRequest.rampSeconds=1.2*reversalScale;
    reversalRequest.hand=reversalHand;reversalRequest.rollingAcceleration=rollingAcceleration;
    reversalRequest.dragAccelerationCoefficient=dragAccelerationCoefficient;
    auto reversalSource=designFvdImmelmann(reversalRequest,cancel);
    if(!reversalSource.section.assessment.passed||!reversalSource.section.report.valid())
        throw std::runtime_error(reversalSource.section.cancelled?"CANCELLED":"Coupled Immelmann source failed");
    std::vector<AuthoredPoint> reversalPoints;std::vector<double> reversalLocations;
    const auto& reversalTrack=reversalSource.section.track;const int reversalCount=int(std::ceil(reversalTrack.length/1.5));
    for(int i=0;i<=reversalCount;++i)reversalLocations.push_back(reversalTrack.length*i/reversalCount);
    reversalLocations.push_back(reversalSource.rollExit.distance);std::sort(reversalLocations.begin(),reversalLocations.end());
    reversalLocations.erase(std::unique(reversalLocations.begin(),reversalLocations.end()),reversalLocations.end());
    size_t reversalSourceEnd=0;
    for(double location:reversalLocations){auto p=reversalTrack.sample(location);reversalPoints.push_back({p.position,0,Element::Inversion,p.up});
        if(location==reversalSource.rollExit.distance)reversalSourceEnd=reversalPoints.size()-1;}
    const size_t pulloutSourceEnd=reversalPoints.size()-1;const auto valley=reversalSource.exit;
    const double reversalHeading=std::atan2(valley.forward.y,valley.forward.x);
    EnergyLoopModuleRequest loopRequest;loopRequest.height=loopHeight;loopRequest.crossingOffset=18;
    loopRequest.apexSpeed=26;loopRequest.normalG=4.1;
    loopRequest.rollingAcceleration=rollingAcceleration;loopRequest.dragAccelerationCoefficient=dragAccelerationCoefficient;
    if(!preparedSources.loop){
        auto source=buildEnergyLoopModule(loopRequest,cancel);
        if(!source.canonicalBuilt||!source.report.valid())throw std::runtime_error(source.cancelled?"CANCELLED":"Force-designed vertical loop: "+(source.report.errors.empty()?std::string("canonical replay failed"):source.report.errors.front().message));
        preparedSources.loop=std::make_unique<const EnergyLoopModule>(std::move(source));
    }
    const auto& loopShape=*preparedSources.loop;
    double loopPlanLength=0;for(size_t i=1;i<loopShape.samples.size();++i){const Vec3 delta=loopShape.samples[i].position-loopShape.samples[i-1].position;loopPlanLength+=std::hypot(delta.x,delta.y);}
    const double loopHeading=std::atan2(loopShape.exit.forward.y,loopShape.exit.forward.x);
    const double trainLength=(req.train.cars-1)*req.train.spacing;
    const double loopTrimTarget=std::sqrt(loopShape.samples.front().speed*loopShape.samples.front().speed+feedback.loopEnergyCorrection);
    if(!std::isfinite(loopTrimTarget)||loopTrimTarget<=0)throw std::runtime_error("Measured loop energy cannot produce a positive entry-speed target");
    const double loopTrimLength=std::max(0.,feedback.loopApproachSpeed*feedback.loopApproachSpeed-loopTrimTarget*loopTrimTarget)/(2*serviceBrakeAcceleration)
        +2*trainLength+.5*(feedback.loopApproachSpeed+loopTrimTarget);
    const double loopApproachLength=feedback.loopApproachDomain>=0?feedback.loopApproachDomain:loopTrimLength;
    const double reversalEntrySpeed=std::sqrt(reversalRequest.entrySpeed*reversalRequest.entrySpeed+feedback.reversalEnergyCorrection);
    if(!std::isfinite(reversalEntrySpeed)||reversalEntrySpeed<=0)throw std::runtime_error("Measured reversal energy cannot produce a positive entry-speed target");
    const double approachBound=feedback.turnSpeed[1];
    const double reversalLead=std::max(0.,(approachBound*approachBound-reversalEntrySpeed*reversalEntrySpeed)/(2*serviceBrakeAcceleration))+approachBound*.5+trainLength;
    const LayoutModulePose reversalExit{{reversalLead+valley.position.x,valley.position.y,valley.position.z},valley.forward,valley.up};
    double reversalPlanLength=reversalLead;
    for(size_t i=1;i<reversalPoints.size();++i){const Vec3 delta=reversalPoints[i].position-reversalPoints[i-1].position;reversalPlanLength+=std::hypot(delta.x,delta.y);}
    detail.range(-.05,.05); // Preserve unrelated seeded sources after replacing sin^4 shaping.
    detail.range(-.04,.04); // Preserve the following seeded hill choices; the removed loop warp has no geometry parameter.
    detail.range(1.6,1.85); // Retain the independent shape random sequence.
    std::vector<double> turnRiseScale(sides),turnRiseShape(sides);for(int side=0;side<sides;++side){turnRiseScale[side]=detail.range(.8,1.15);turnRiseShape[side]=detail.range(-.08,.08);}
    std::vector<FvdAirtimeResult> forceHills;
    Random banking{req.seed^0x29a35d467ce819bfull};const double bankHand=(banking.next()&1)?1.:-1.;
    for(int hill=0;hill<2;++hill){FvdAirtimeRequest force;force.speed=feedback.airtimeSpeed[hill];force.hills.clear();
        force.rollingAcceleration=rollingAcceleration;force.dragAccelerationCoefficient=dragAccelerationCoefficient;
        // Stronger valleys use shorter positive impulses. At a different inlet
        // speed, scale time with v (and resulting size approximately with v^2),
        // preserving the intended load history instead of stretching geometry.
        const double timeScale=force.speed/65;force.portRampSeconds=.6*timeScale;
        for(int element=0;element<(hill==0?1:3);++element){
            FvdAirtimeHill intent;intent.pushG=detail.range(4.8,5.0)-.15*element;
            // Distinct crest roles: ejector support, then floater, positive
            // relief and stronger closing airtime. FVD solves actual size.
            const std::array<double,3> chainCrests{-.35,.15,-1.0};
            intent.crestG=(hill==0?-1.25:chainCrests[element])+detail.range(-.05,.05);
            intent.pushHoldSeconds=detail.range(.20,.22)*timeScale;intent.crestRampSeconds=detail.range(.80,.82)*timeScale;
            // The final hill also carries the positive impulse needed for a
            // gradual exit ramp; intermediate valleys continue into the chain.
            if(hill==1&&element==2)intent.pushHoldSeconds+=force.portRampSeconds*.5;
            intent.crestPulseSeconds=1.6*timeScale;
            if(hill==1)intent.bankRadians=bankHand*(element==2?-1.:1.)*banking.range(20,28)*pi/180;
            force.hills.push_back(intent);
        }
        auto authored=designFvdAirtime(force,cancel);
        if(!authored.section.integrated||!authored.section.canonicalBuilt||!authored.section.report.valid()||!authored.section.assessment.passed){
            if(authored.section.cancelled)throw std::runtime_error("CANCELLED");
            throw std::runtime_error("FVD hill "+std::to_string(hill)+" at "+std::to_string(force.speed)+" m/s: "+(authored.section.report.errors.empty()?"canonical replay failed":authored.section.report.errors.front().message));
        }
        forceHills.push_back(std::move(authored));}
    const double forceReturnSpan=forceHills[1].span;
    std::vector<double> minimum{200+hillWidth,loopApproachLength+loopPlanLength+400+forceHills[0].span,
        reversalPlanLength,400+forceReturnSpan};
    std::array<LayoutModulePose,4> sourceExits;
    sourceExits[0]={signature.exit.position+Vec3{200,0,0},signature.exit.forward,signature.exit.up};
    for(int side:{1,3}){const auto& exit=forceHills[side==1?0:1].section.samples.back();
        const double heading=side==1?loopHeading:0;const Vec3 base=side==1?Vec3{loopApproachLength,0,0}+loopShape.exit.position:Vec3{};
        sourceExits[side]={base+inFrame(Vec3{400,0,0}+exit.position,heading),inFrame(exit.forward,heading),inFrame(exit.up,heading)};}
    sourceExits[2]=reversalExit;
    std::array<detail::TerrainModuleFootprint,4> footprints;
    auto& loopFootprint=footprints[0];loopFootprint.entrance={loopApproachLength,0,0};
    for(int i=0;i<=32;++i)loopFootprint.samples.push_back({loopApproachLength*i/32,0,0});
    for(size_t i=0;i<loopShape.points.size();i+=8)loopFootprint.samples.push_back(loopFootprint.entrance+loopShape.points[i].position);
    loopFootprint.samples.push_back(loopFootprint.entrance+loopShape.exit.position);
    for(size_t i=0;i<loopShape.points.size();++i){
        const auto& point=loopShape.points[i];
        const Vec3 tangent=i==0?loopShape.entry.forward:i+1==loopShape.points.size()?loopShape.exit.forward:unit(loopShape.points[i+1].position-loopShape.points[i-1].position);
        Vec3 up=unit(point.upHint-tangent*dot(point.upHint,tangent));up=rotate(up,tangent,point.bank);
        loopFootprint.rigidFrames.push_back({loopFootprint.entrance+point.position,tangent,{},up,cross(tangent,up),point.element});
    }
    auto& reversalFootprint=footprints[1];reversalFootprint.entrance={reversalLead,0,0};
    for(int i=0;i<=32;++i){double u=i/32.;reversalFootprint.samples.push_back({reversalLead*u,0,0});}
    for(size_t i=0;i<reversalPoints.size();i+=8)reversalFootprint.samples.push_back(reversalFootprint.entrance+reversalPoints[i].position);
    reversalFootprint.samples.push_back(reversalExit.position);
    // Use the same source curves and endpoint displacement as forceHill below.
    // Continue from the integrated loop exit. Its heading and lateral
    // displacement are real source outputs, not offsets to undo afterward.
    footprints[2].entrance=loopFootprint.samples.back()+inFrame({400,0,0},loopHeading);
    footprints[3].entrance={400,0,0};
    for(size_t index:{size_t(2),size_t(3)}){
        auto& footprint=footprints[index];Vec3 cursor=footprint.entrance;
        const auto& source=forceHills[index==2?0:1].section;
        for(int sample=0;sample<=64;++sample)
            footprint.samples.push_back(cursor+inFrame(source.track.sample(source.track.length*sample/64).position,index==2?loopHeading:0));
        footprint.samples.push_back(cursor+inFrame(source.samples.back().position,index==2?loopHeading:0));
        const int count=int(std::ceil(source.track.length/1.5));const double heading=index==2?loopHeading:0;
        for(int sample=0;sample<=count;++sample){
            auto frame=source.track.sample(source.track.length*sample/count);
            frame.position=cursor+inFrame(frame.position,heading);frame.tangent=inFrame(frame.tangent,heading);
            frame.up=inFrame(frame.up,heading);frame.right=inFrame(frame.right,heading);footprint.rigidFrames.push_back(frame);
        }
    }
    // Ordinary turn dimensions follow their own measured energy. The first
    // pass uses the previous conservative planning speed; subsequent passes
    // retain the same seeded shape/site while solving exact closure again.
    std::array<double,4> turnRadii{},turnRamps{},turnBanks{};
    for(int side=0;side<sides;++side){
        const double speed=feedback.turnSpeed[side];
        turnRadii[side]=speed*speed/(gravity*std::sqrt(designNormalG*designNormalG-1))+geometryAttempt*12;
        turnBanks[side]=std::atan(speed*speed/(gravity*turnRadii[side]));
        const double bank=turnBanks[side];
        turnRamps[side]=speed*std::max(1.875*bank/turnTwistRate,
            std::sqrt((10*std::sqrt(3.)/3)*bank/turnTwistOnset));
    }
    // Keep real terminal stopping room rather than a generic recovery pad.
    // Source exit speed includes rolling/drag; downhill brake force and the
    // actual complete stop are separately assessed after terrain placement.
    const double tailExitSpeed=forceHills.back().section.samples.back().speed;
    const double terminalReturnMinimum=tailExitSpeed*tailExitSpeed/(2*terminalDeceleration)+2*trainLength+67;
    // One transfer intent supplies route ranking and actual terrain composition.
    // The terrain ascent retains its existing 0..2 g authoring intent; the
    // post-loop launch retains the planner's 3.5 g intent. Return bounds remain
    // explicit and separate from final finite-train force acceptance.
    const double relaunchTarget=std::sqrt(65*65+feedback.relaunchEnergyCorrection);
    if(!std::isfinite(relaunchTarget)||relaunchTarget<=0||relaunchTarget>100)throw std::runtime_error("Relaunch energy left the supported drive domain");
    const double relaunchEffectiveLength=400-trainLength-relaunchTarget;
    if(relaunchEffectiveLength<=0)throw std::runtime_error("Train and motor fades leave no effective relaunch length");
    const double relaunchDrag=dragAccelerationCoefficient*relaunchTarget*relaunchTarget+rollingAcceleration;
    const double relaunchAcceleration=std::min(req.limits.maxLongitudinalG*gravity-.1,std::max(4.5,(relaunchTarget*relaunchTarget-loopShape.samples.back().speed*loopShape.samples.back().speed)/(2*relaunchEffectiveLength)+relaunchDrag));
    const auto motorBound=[&](double base){return std::min(req.limits.maxLongitudinalG*gravity-.1,base+gravity*2/std::sqrt(5.));};
    const double terrainSupplyTarget=std::sqrt(terrainAscentSpeed*terrainAscentSpeed+feedback.loopSupplyEnergyCorrection);
    if(!std::isfinite(terrainSupplyTarget)||terrainSupplyTarget<=0)throw std::runtime_error("Terrain supply target left its finite energy domain");
    // Actual motor rating is sized after the canonical rail/fade length is
    // known and is clamped to this same physical capacity. Never assess an
    // increased feedback target against the old 150 km/h speed bound.
    const double terrainSupplyCapacity=feedback.loopSupplyEnergyCorrection>0?req.limits.maxLongitudinalG*gravity-.1:motorBound(3.5);
    std::array<detail::TerrainMotion,4> terrainMotion{{
        {signature.exit.speed,rollingAcceleration,dragAccelerationCoefficient,std::max(0.,req.limits.minVerticalG),std::min(2.,req.limits.maxVerticalG),terrainSupplyTarget,terrainSupplyCapacity},
        {loopShape.samples.back().speed,rollingAcceleration,dragAccelerationCoefficient,req.limits.minVerticalG,std::min(3.5,req.limits.maxVerticalG),relaunchTarget,motorBound(relaunchAcceleration)},
        {valley.speed,rollingAcceleration,dragAccelerationCoefficient,req.limits.minVerticalG,req.limits.maxVerticalG,60,motorBound(3.5)},
        {tailExitSpeed,rollingAcceleration,dragAccelerationCoefficient,std::max(-1.,req.limits.minVerticalG),std::min(3.5,req.limits.maxVerticalG),0,0}
    }};
    auto plans=planRoutes(req,sides,minimum,turnRadii,turnRamps,turnBanks,terminalReturnMinimum,loopApproachLength,loopTrimLength,sourceExits,terrainMotion,footprints,feedback.planShape,feedback.planPlacement,rng,cancel);if(plans.empty())throw std::runtime_error("No terrain route fits exact closure and footprint budget");
    // Visit terrain-ranked plans in order. Exact raw departure/return boundary
    // and bay must fit the local budget before committing to a station site.
    // The candidate list is bounded by 48 shapes x 36 placements; rejected
    // station sites do not spend a geometry/physics attempt or alter its limits.
    int firstFeasiblePlacement=-1,transferRejectedSites=0;
    std::array<int,4> rejectedTransferPlacements{};
    for(auto& plan:plans){
    if(std::find(rejectedTransferPlacements.begin(),rejectedTransferPlacements.begin()+transferRejectedSites,plan.placement)!=rejectedTransferPlacements.begin()+transferRejectedSites)continue;
    if(placementVariant&&feedback.planShape<0&&plan.placement==firstFeasiblePlacement)continue;
    if(cancel&&cancel())throw std::runtime_error("CANCELLED");
    double heading=plan.heading;Vec3 origin=plan.origin,cursor=origin;
    // Reconstruct only visited plans; placement ranking retains compact parameters
    // instead of copying the same sampled turn geometry for all 36 sites.
    std::array<TurnShape,4> turns;
    for(int side=0;side<sides;++side)turns[side]=makeTurn(plan.angles[side],turnRadii[side],turnRamps[side],turnBanks[side]);
    const std::string orderName="H-I-A";
    d.topology="port-routed/H-I-IM-A4";
    // Pending intervals are train-center work domains. Actual rail endpoints
    // are inset once, after canonical lengths are known, to keep each source
    // passive after its inlet and until its exit has been crossed.
    std::vector<AuthoredPoint> raw;struct Pending{size_t begin,end;DriveKind kind;double speed;double acceleration{3.5};};std::vector<Pending> pending;
    struct ModuleRun{size_t begin,end;int corridor;std::string identity;};std::vector<ModuleRun> modules;
    auto append=[&](Vec3 p,double bank,Element e,Vec3 up=Vec3{0,0,1}){if((raw.size()&1023)==0&&cancel&&cancel())throw std::runtime_error("CANCELLED");if(raw.empty()||norm(raw.back().position-p)>1e-6)raw.push_back({p,bank,e,up});};
    auto piece=[&](double samplingLength,const std::function<Vec3(double)>& fn,Element element){
        Vec3 forward{std::cos(heading),std::sin(heading),0},left{-std::sin(heading),std::cos(heading),0},base=cursor;int n=int(std::ceil(samplingLength/1.8));size_t start=raw.empty()?0:raw.size()-1;
        for(int i=0;i<=n;++i){double u=double(i)/n;Vec3 p=fn(u),dv=fn(std::min(1.,u+1e-5))-fn(std::max(0.,u-1e-5));Vec3 tangent=unit(forward*dv.x+left*dv.y+Vec3{0,0,dv.z});Vec3 up=unit(cross(left*(-1),tangent));append(base+forward*p.x+left*p.y+Vec3{0,0,p.z},0,element,up);}
        Vec3 end=fn(1);cursor=base+forward*end.x+left*end.y+Vec3{0,0,end.z};return std::pair{start,raw.size()-1};
    };
    auto line=[&](double length,Element e){return piece(length,[=](double u){return Vec3{length*u,0,0};},e);};
    auto drive=[&](double length,DriveKind kind,double speed){auto range=line(length,kind==DriveKind::Brake?Element::Brake:Element::Launch);pending.push_back({range.first,range.second,kind,speed});return range;};
    auto record=[&](std::pair<size_t,size_t> range,int side,const std::string& identity){modules.push_back({range.first,range.second,side,identity});};
    struct SourceKnot {size_t index;double sourceDistance;Knot source;};
    std::vector<SourceKnot> signatureKnots;
    std::array<std::vector<SourceKnot>,2> airtimeKnots;
    std::array<std::vector<std::pair<size_t,CandidatePorts::SpeedIntent>>,2> airtimeChecks;
    size_t reversalExitIndex=0,reversalEntryIndex=0,pulloutExitIndex=0,reversalBrakeStartIndex=0,relaunchEndIndex=0,loopApexIndex=0,loopBrakeStartIndex=0,loopEntryIndex=0,loopExitIndex=0;
    auto forceHill=[&](int index,int side){const auto& authored=forceHills[index];const auto& track=authored.section.track;
        Vec3 base=cursor;size_t start=raw.empty()?0:raw.size()-1;
        const auto intent=[](const FvdSample& point){const Vec3 force=point.curvature*(point.speed*point.speed)+Vec3{0,0,gravity};
            return CandidatePorts::SpeedIntent{point.distance,point.speed,point.time,dot(force,point.up)/gravity,dot(force,cross(point.forward,point.up))/gravity};};
        std::vector<CandidatePorts::SpeedIntent> profile{intent(authored.section.samples.front())};
        for(const auto& hill:authored.hills){profile.push_back(intent(hill.apex));profile.push_back(intent(hill.exit));}
        profile.push_back(intent(authored.section.samples.back()));profile.back().distance=track.length;
        const int samples=int(std::ceil(track.length/1.5));
        std::vector<double> locations;locations.reserve(samples+1+profile.size());
        for(int i=0;i<=samples;++i)locations.push_back(track.length*i/samples);
        for(const auto& check:profile)locations.push_back(check.distance);
        std::sort(locations.begin(),locations.end());locations.erase(std::unique(locations.begin(),locations.end()),locations.end());
        size_t nextCheck=0;
        for(double location:locations){auto point=track.sample(location);append(base+inFrame(point.position,heading),0,Element::Airtime,inFrame(point.up,heading));
            airtimeKnots[index].push_back({raw.size()-1,location,{base+inFrame(point.position,heading),inFrame(point.tangent,heading),inFrame(point.curvature,heading),inFrame(point.up,heading),0,Element::Airtime}});
            while(nextCheck<profile.size()&&profile[nextCheck].distance<=location+1e-9){airtimeChecks[index].push_back({raw.size()-1,profile[nextCheck]});++nextCheck;}}
        cursor=base+inFrame(authored.section.samples.back().position,heading);record({start,raw.size()-1},side,"fvd-airtime");
        return authored.span;
    };
    for(int side=0;side<sides;++side){
        const double sourceHeading=heading;
        double used=0,straight=plan.lengths[side];
        if(side==0){record(line(20,Element::Station),side,"station");record(drive(180,DriveKind::Launch,topSpeed),side,"departure-launch");used=200;}
        if(side==0){
            const Vec3 base=cursor;const size_t first=raw.size()-1;
            const int samples=int(std::ceil(signature.section.track.length/1.5));
            for(int i=0;i<=samples;++i){
                const double s=signature.section.track.length*i/samples;auto point=signature.section.track.sample(s);
                const Vec3 position=base+inFrame(point.position,heading),up=inFrame(point.up,heading);
                append(position,0,Element::Hill,up);
                signatureKnots.push_back({raw.size()-1,s,{position,inFrame(point.tangent,heading),inFrame(point.curvature,heading),up,0,Element::Hill}});
            }
            cursor=base+inFrame(signature.exit.position,heading);
            record({first,raw.size()-1},side,"record-hill");used+=signature.span;
        }else if(side==1){
            record(line(loopApproachLength,Element::Return),side,"loop-approach");used+=loopApproachLength;
            loopEntryIndex=raw.size()-1;
            Vec3 base=cursor;size_t start=raw.size()-1;
            for(const auto& point:loopShape.points)append(base+inFrame(point.position,heading),point.bank,point.element,inFrame(point.upHint,heading));
            loopApexIndex=size_t(std::max_element(raw.begin()+start,raw.end(),[](const auto& a,const auto& b){return a.position.z<b.position.z;})-raw.begin());
            cursor=base+inFrame(loopShape.exit.position,heading);heading+=loopHeading;
            loopExitIndex=raw.size()-1;record({start,loopExitIndex},side,"record-inversion");used+=loopPlanLength;
            auto launch=line(400,Element::Launch);
            ports.relaunchTarget=relaunchTarget;
            relaunchEndIndex=launch.second;pending.push_back({launch.first,launch.second,DriveKind::Boost,relaunchTarget,relaunchAcceleration});record(launch,side,"post-loop-launch");used+=400;
            used+=forceHill(0,side);
        }else if(side==2){
            // The rolling source provides actual lateral separation. This
            // approach carries only the required finite-train trim operation.
            double entrySpeed=reversalEntrySpeed;
            auto lead=line(reversalLead,Element::Brake);
            reversalBrakeStartIndex=lead.first;pending.push_back({lead.first,lead.second,DriveKind::Brake,entrySpeed,serviceBrakeAcceleration});record(lead,side,"immelmann-inward-entry-brake");
            reversalEntryIndex=raw.size()-1;
            const Vec3 islandBase=cursor;size_t segmentStart=raw.size()-1;
            for(size_t i=1;i<reversalPoints.size();++i){const auto& point=reversalPoints[i];
                append(islandBase+inFrame(point.position,heading),point.bank,point.element,inFrame(point.upHint,heading));
                if(i==reversalSourceEnd){record({segmentStart,raw.size()-1},side,"high-immelmann");reversalExitIndex=raw.size()-1;segmentStart=raw.size()-1;}
                if(i==pulloutSourceEnd){record({segmentStart,raw.size()-1},side,"descending-pullout");pulloutExitIndex=raw.size()-1;segmentStart=raw.size()-1;}
            }
            cursor=islandBase+inFrame(valley.position,heading);
            used+=reversalPlanLength;
        }else{
            record(drive(400,DriveKind::Boost,60),side,"airtime-entry-launch");used+=400;
            used+=forceHill(1,side);
        }
        heading=sourceHeading+std::atan2(sourceExits[side].forward.y,sourceExits[side].forward.x);
        if(used>straight+1e-6)throw std::runtime_error("Module plan exceeded its actual source-port budget");
        // Source guards already carry matching G3 jets. A positive remainder
        // is a solved positional transfer, not a mandatory recovery/bump.
        const double recoveryLength=std::max(0.,straight-used);
        if(recoveryLength>1e-6)record(line(recoveryLength,Element::Return),side,"corridor-recovery");
        const auto& turn=turns[side];Vec3 base=cursor;size_t turnStart=raw.size()-1;double dz=side==2?-valley.position.z:0;
        const double turnSpeed=feedback.turnSpeed[side];
        const double bankedRise=std::min(26.,.65*gravity*turn.length*turn.length/(4*pi*pi*turnSpeed*turnSpeed))*turnRiseScale[side];
        for(size_t i=1;i<turn.points.size();++i){double along=turn.length*i/(turn.points.size()-1);cursor=base+inFrame(turn.points[i],heading);cursor.z=base.z+dz*smooth(along/turn.length)+bankedRise*std::pow(std::sin(pi*layoutWarp(along/turn.length,turnRiseShape[side])),4);append(cursor,turn.bankAt(along),Element::Turn);}
        modules.push_back({turnStart,raw.size()-1,side,"banked-camelback-turn"});
        heading+=plan.angles[side];
    }
    // The boarding envelope needs an actual straight approach, not merely
    // a flat height and faded bank on the end of a shrinking turn.
    const auto stationApproach=line(stationReturnLength,Element::Return);
    record(stationApproach,3,"station-approach");
    if(std::hypot(cursor.x-origin.x,cursor.y-origin.y)>.001)throw std::runtime_error("Solved route failed canonical XY closure");
    size_t unique=raw.size()-1;std::vector<double> distance(raw.size());for(size_t i=1;i<raw.size();++i)distance[i]=distance[i-1]+norm(raw[i].position-raw[i-1].position);
    // Locate actual authored diagonal crossings after loop offsets and inward
    // routing. Their separation is solved after terrain datums are known.
    // An XY crossing never authorizes acceptance (full sweep/support tests do).
    size_t branchBegin[4]={raw.size(),raw.size(),raw.size(),raw.size()},branchEnd[4]={};
    for(const auto& run:modules){branchBegin[run.corridor]=std::min(branchBegin[run.corridor],run.begin);branchEnd[run.corridor]=std::max(branchEnd[run.corridor],run.end);}
    // Two worst-case supported 4.2m body envelopes plus a planning reserve.
    // This replaces the arbitrary 42m pedestal; the unchanged six-metre
    // certified chord model and every support/body sweep still gate acceptance.
    constexpr double crossingSeparation=2*4.2+.6;
    struct Crossing{size_t under,over;double u,v;};std::vector<Crossing> crossings;
    for(size_t i=branchBegin[1];i<branchEnd[1];++i){Vec3 a=raw[i].position,b=raw[i+1].position-a;
        for(size_t j=branchBegin[3];j<branchEnd[3];++j){Vec3 c=raw[j].position,e=raw[j+1].position-c;double det=cross(b,e).z;if(std::abs(det)<1e-8)continue;
            double u=cross(c-a,e).z/det,v=cross(c-a,b).z/det;if(u<0||u>1||v<0||v>1)continue;
            double along=distance[j]+v*(distance[j+1]-distance[j]);
            if(along-distance[branchBegin[3]]<300||distance[branchEnd[3]]-along<300)continue;
            crossings.push_back({i,j,u,v});
        }
    }
    // A flyover translates the complete tail element chain. A varying lift
    // through its interiors would replace the requested FVD force histories.
    size_t tailBegin=branchEnd[3],tailEnd=branchBegin[3];
    for(const auto& run:modules)if(run.corridor==3&&run.identity=="fvd-airtime"){
        tailBegin=std::min(tailBegin,run.begin);tailEnd=std::max(tailEnd,run.end);}
    for(size_t i=0;i<raw.size();++i)raw[i].bank*=stationBankFactor(distance.back()-distance[i],req.train);
    if(norm(raw.back().position-raw.front().position)>.001)throw std::runtime_error("Solved route failed canonical height closure");raw.back().position=raw.front().position;

    // Fix the station to its local ground, then solve a smooth baseline away
    // from the boarding/launch boundary. A distant canyon rim cannot lift it.
    Vec3 stationForward{std::cos(plan.heading),std::sin(plan.heading),0},stationRight=cross(stationForward,Vec3{0,0,1});
    double trainHalf=(req.train.cars-1)*req.train.spacing*.5,stationBegin=-std::max(18.,trainHalf+8.),stationEnd=std::max(64.,2*trainHalf+38.);
    double stationGroundMin=1e9,stationGroundMax=-1e9;
    for(double x=stationBegin-2;x<=stationEnd+4;x+=2)for(double y=-6;y<=6;y+=2){Vec3 p=origin+stationForward*x+stationRight*y;double ground=req.terrain.height(p.x,p.y);stationGroundMin=std::min(stationGroundMin,ground);stationGroundMax=std::max(stationGroundMax,ground);}
    double stationDatum=stationGroundMax+req.limits.minClearance+4;
    constexpr double spacing=50;
    std::vector<double> required(unique),absoluteFloor(unique),baselineTarget(unique);
    // A shared FVD knot keeps its preceding element tag, but also belongs to
    // the adjoining turn. Its declared body envelope includes both true ports
    // before source datums and the intervening terrain baseline are solved.
    std::vector<bool> turnEnvelope(unique);
    for(const auto& run:modules)if(run.identity=="banked-camelback-turn")
        for(size_t i=run.begin;i<=run.end;++i)turnEnvelope[i%unique]=true;
    size_t tallest=0,inverted=0;double maxHill=-1e9,maxLoop=-1e9;
    for(size_t i=0;i<unique;++i){if(raw[i].element==Element::Hill&&raw[i].position.z>maxHill){maxHill=raw[i].position.z;tallest=i;}if(raw[i].element==Element::Inversion&&raw[i].position.z>maxLoop){maxLoop=raw[i].position.z;inverted=i;}}
    for(size_t i=0;i<unique;++i){
        if((i&255)==0&&cancel&&cancel())throw std::runtime_error("CANCELLED");
        Vec3 tangent=unit(raw[(i+1)%unique].position-raw[(i+unique-1)%unique].position),up=unit(raw[i].upHint-tangent*dot(raw[i].upHint,tangent));up=rotate(up,tangent,raw[i].bank);Vec3 right=unit(cross(tangent,up));
        const TrackSample frame{raw[i].position,tangent,{},up,right,raw[i].element};
        double floor=detail::terrainEnvelopeDatum(req.terrain,req.limits,frame,raw[i].element==Element::Turn||turnEnvelope[i]);
        double ground=req.terrain.height(raw[i].position.x,raw[i].position.y);
        if(i==tallest)floor=std::max(floor,ground+req.targets.height+4-raw[i].position.z);
        if(i==inverted)floor=std::max(floor,ground+req.targets.inversionHeight+4-raw[i].position.z);
        required[i]=floor;absoluteFloor[i]=floor+raw[i].position.z;baselineTarget[i]=ground+req.limits.minClearance+4;
    }
    // Clear the actual low-valley approach before fitting the major climb.
    // Small floor undulations must not force a kilometre-long premature ascent.
    // Only the needed datum is used; the local station budget below still gates.
    if(dramaticTerrain)for(const auto& run:modules)if(run.identity=="record-hill"){
        for(size_t i=run.end;i<=loopEntryIndex;++i){
            if(!onCanyonFloor(req.terrain,raw[i].position))break;
            stationDatum=std::max(stationDatum,absoluteFloor[i]+.3);
        }
    }
    // The first signature shares the departure datum. Preserve its source
    // shape instead of bending its lower ascent to follow a distant rim.
    // A site needing a higher datum must still fit the local station budget.
    for(const auto& run:modules)if(run.identity=="record-hill"&&run.corridor==0){
        for(size_t i=run.begin;i<=run.end;++i)stationDatum=std::max(stationDatum,required[i]+.3);
    }
    const double fixedDeparture=200+trainHalf+12,fixedReturn=stationReturnLength;
    // The same local flat datum must clear both the boarding bay and the
    // actual fixed launch/return boundary. Bound local platform height at 16 m
    // rather than inheriting the highest terrain elsewhere on the circuit.
    for(size_t i=0;i<unique;++i)if(distance[i]<=fixedDeparture||i>=stationApproach.first)stationDatum=std::max(stationDatum,required[i]+.3);
    // The terminal approach is flattened to this absolute station height.
    // Its authored rise cannot count towards clearance after that replacement.
    for(size_t i=stationApproach.first;i<unique;++i)
        stationDatum=std::max(stationDatum,absoluteFloor[i]+.3);
    plan.stationChecked=true;plan.stationBayMinimum=stationGroundMin;plan.stationBayMaximum=stationGroundMax;plan.stationRequiredDatum=stationDatum;
    plan.stationFeasible=stationDatum-stationGroundMin<=16;
    if(!plan.stationFeasible)continue;
    // Exact local source anchors, independent of the surrounding control grid.
    struct SourceDatum {size_t begin,end;double height;};std::vector<SourceDatum> sourceDatums;
    size_t pairBegin=0;
    for(const auto& run:modules){
        if(run.identity=="record-hill")sourceDatums.push_back({0,run.end,stationDatum});
        if(run.identity=="station-approach")sourceDatums.push_back({run.begin,run.end,stationDatum});
        if(run.identity=="record-inversion"||run.identity=="fvd-airtime")
            sourceDatums.push_back({run.begin,run.end,*std::max_element(required.begin()+run.begin,required.begin()+run.end+1)+.3});
        if(run.identity=="immelmann-inward-entry-brake")pairBegin=run.begin;
        if(run.identity=="descending-pullout")
            sourceDatums.push_back({pairBegin,run.end,*std::max_element(required.begin()+pairBegin,required.begin()+run.end+1)+.3});
    }
    // Monotone transitions share their local port-clearance requirement with
    // the adjoining rigid source. A global padded grid cannot supply that lift.
    auto bindTransferPorts=[&](size_t first,size_t last){
        auto owner=[&](size_t index){return std::find_if(sourceDatums.begin(),sourceDatums.end(),[&](const SourceDatum& source){return index>=source.begin&&index<=source.end;});};
        auto start=owner(first),finish=owner(last);
        const double startHeight=start==sourceDatums.end()?absoluteFloor[first]+.3:start->height+raw[first].position.z;
        const double finishHeight=finish==sourceDatums.end()?absoluteFloor[last]+.3:finish->height+raw[last].position.z;
        auto high=startHeight>finishHeight?start:finish;const size_t port=startHeight>finishHeight?first:last;
        if(high!=sourceDatums.end())high->height=std::max(high->height,*std::max_element(absoluteFloor.begin()+first,absoluteFloor.begin()+last+1)+.3-raw[port].position.z);
    };
    for(const auto& run:modules)if(run.identity=="post-loop-launch"||run.identity=="airtime-entry-launch")bindTransferPorts(run.begin,run.end);
    if(req.terrain.cliffHeight>0){
        for(const auto& run:modules)if(run.identity=="record-hill")bindTransferPorts(run.end,loopEntryIndex);
        for(const auto& run:modules)if(run.identity=="station-approach")bindTransferPorts(tailEnd,run.begin);
    }
    // Each free gap has its actual source ports in the solve from the start.
    // Uniform controls within that gap cannot leave an arbitrarily short last
    // cell when a distant source changes the total circuit length.
    struct BaselineGap {
        double begin,end,spacing;
        std::vector<double> base,target,lower;
        std::vector<Vec3> jets;
    };
    std::vector<BaselineGap> baselineGaps;
    for(size_t source=1;source<sourceDatums.size();++source){
        const auto& left=sourceDatums[source-1];const auto& right=sourceDatums[source];
        if(left.end==right.begin)continue;
        const double begin=distance[left.end],end=distance[right.begin];
        const int cells=std::max(3,int(std::ceil((end-begin)/spacing)));
        BaselineGap gap{begin,end,(end-begin)/cells,std::vector<double>(cells+1),std::vector<double>(cells+1),std::vector<double>(cells+1,-1e9),std::vector<Vec3>(cells+1)};
        std::vector<double> weight(cells+1);
        for(size_t i=left.end+1;i<right.begin;++i){
            const int k=std::clamp(int(std::llround((distance[i]-begin)/gap.spacing)),1,cells-1);
            gap.lower[k]=std::max(gap.lower[k],required[i]);gap.target[k]+=baselineTarget[i];++weight[k];
        }
        gap.base.front()=left.height;gap.base.back()=right.height;
        for(int k=1;k<cells;++k){gap.target[k]=weight[k]?gap.target[k]/weight[k]:left.height+(right.height-left.height)*k/cells;gap.base[k]=std::max(gap.target[k],gap.lower[k]);}
        baselineGaps.push_back(std::move(gap));
    }
    auto solveBaseline=[&]{for(auto& gap:baselineGaps){
        const int last=int(gap.base.size())-1;
        auto value=[&](int k){return gap.base[std::clamp(k,0,last)];};
        for(int iteration=0;iteration<1500;++iteration){
            if((iteration&31)==0&&cancel&&cancel())throw std::runtime_error("CANCELLED");
            double maximumChange=0;
            for(int k=1;k<last;++k){double next=std::max(gap.lower[k],(64*(value(k-1)+value(k+1))-25*(value(k-2)+value(k+2))+4*(value(k-3)+value(k+3))+.001*gap.target[k])/86.001);maximumChange=std::max(maximumChange,std::abs(next-gap.base[k]));gap.base[k]=next;}
            if(maximumChange<1e-6)break;
        }
        std::vector<bool> fixed(gap.base.size());fixed.front()=fixed.back()=true;
        gap.jets=detail::minimumSnapJets(gap.base,fixed,cancel);
    }};
    auto baseline=[&](double s){
        for(const auto& source:sourceDatums)if(s>=distance[source.begin]&&s<=distance[source.end])return source.height;
        for(const auto& gap:baselineGaps)if(s>gap.begin&&s<gap.end){
            const int k=std::min(int(gap.base.size())-2,int((s-gap.begin)/gap.spacing));
            const double u=(s-gap.begin)/gap.spacing-k;const Vec3 j0=gap.jets[k],j1=gap.jets[k+1];
            const double c0=gap.base[k],c1=j0.x,c2=j0.y*.5,c3=j0.z/6;
            const double p=gap.base[k+1]-c0-c1-c2-c3,v=j1.x-c1-2*c2-3*c3,a=j1.y-2*c2-6*c3,j=j1.z-6*c3;
            const double c4=35*p-15*v+2.5*a-j/6,c5=-84*p+39*v-7*a+j*.5,c6=70*p-34*v+6.5*a-j*.5,c7=-20*p+10*v-2*a+j/6;
            return c0+u*(c1+u*(c2+u*(c3+u*(c4+u*(c5+u*(c6+u*c7))))));
        }
        throw std::runtime_error("Baseline point has no physical source or gap owner");
    };
    // Transfer ports are fixed heights in the subsequent terrain fit. The
    // generic centimetre refinement tolerance cannot leave their conservative
    // floor a millimetre above the solved baseline and then reject the port.
    std::vector<bool> transferPort(unique);
    for(const auto& run:modules){
        if(run.identity=="post-loop-launch"||run.identity=="airtime-entry-launch"){transferPort[run.begin]=true;transferPort[run.end]=true;}
        if(run.identity=="record-hill")transferPort[run.end]=true;
        if(run.identity=="record-inversion")transferPort[run.begin]=true;
    }
    transferPort[tailEnd]=true;
    // These interiors are subsequently fitted against their full terrain floor.
    // The general baseline must not reject them using padded endpoint controls.
    std::vector<bool> transferInterior(unique);
    auto ownsTransfer=[&](size_t first,size_t last){for(size_t i=first+1;i<last;++i)transferInterior[i]=true;};
    for(const auto& run:modules)if(run.identity=="post-loop-launch"||run.identity=="airtime-entry-launch")ownsTransfer(run.begin,run.end);
    if(req.terrain.cliffHeight>0&&baseline(distance[loopEntryIndex])-stationDatum>12)
        for(const auto& run:modules)if(run.identity=="record-hill")ownsTransfer(run.end,loopEntryIndex);
    if(baseline(distance[tailEnd])-stationDatum>12)
        ownsTransfer(tailEnd,stationApproach.first);
    for(int refinement=0;refinement<3;++refinement){
        solveBaseline();bool raised=false;
        for(size_t i=0;i<unique;++i){
            if(transferInterior[i])continue;
            const double deficit=required[i]-baseline(distance[i]);if(deficit<=(transferPort[i]?1e-9:.01))continue;
            auto gap=std::find_if(baselineGaps.begin(),baselineGaps.end(),[&](const BaselineGap& g){return distance[i]>g.begin&&distance[i]<g.end;});
            if(gap==baselineGaps.end())throw std::runtime_error("Local source boundary violates terrain envelope");
            const int k=std::clamp(int(std::llround((distance[i]-gap->begin)/gap->spacing)),1,int(gap->base.size())-2);
            gap->lower[k]=std::max(gap->lower[k],gap->base[k]+deficit*1.5);raised=true;
        }
        if(!raised)break;
    }
    solveBaseline();for(size_t i=0;i<unique;++i)raw[i].position.z+=baseline(distance[i]);
    // Terrain datums can differ across a crossing. Solve the separation after
    // that composition, translating every tail FVD interior by one constant.
    // The adjacent physical launch/terminal transfers are fitted below.
    auto crossingWeight=[&](size_t i){
        if(i<tailBegin)return layoutSmooth((distance[i]-distance[branchBegin[3]])/(distance[tailBegin]-distance[branchBegin[3]]));
        if(i>tailEnd)return 1-layoutSmooth((distance[i]-distance[tailEnd])/(distance[branchEnd[3]]-distance[tailEnd]));
        return 1.;
    };
    // Every crossing forbids an interval of tail heights. Keep an already
    // separated underpass instead of lifting it above the other branch.
    // All tail sources still share one rigid nonnegative translation.
    std::vector<std::pair<double,double>> blockedLifts;
    for(const auto& crossing:crossings){
        const auto i=crossing.under,j=crossing.over;const double u=crossing.u,v=crossing.v;
        const double first=raw[i].position.z*(1-u)+raw[i+1].position.z*u;
        const double tail=raw[j].position.z*(1-v)+raw[j+1].position.z*v;
        const double influence=crossingWeight(j)*(1-v)+crossingWeight(j+1)*v;
        if(influence<=1e-9)throw std::runtime_error("Crossing conflicts with the fixed return boundary");
        blockedLifts.emplace_back((first-tail-crossingSeparation)/influence,(first-tail+crossingSeparation)/influence);
    }
    std::sort(blockedLifts.begin(),blockedLifts.end());
    double crossingLift=0;
    for(const auto& interval:blockedLifts)if(crossingLift>interval.first&&crossingLift<interval.second)crossingLift=interval.second;
    for(size_t i=branchBegin[3];i<=branchEnd[3];++i)raw[i].position.z+=crossingLift*crossingWeight(i);
    // Give purposeful transfers their full grade-change domain instead of
    // compressing terrain relief between neighbouring rigid guard plateaus.
    size_t ascentMotorBegin=0;
    auto heightTransfer=[&](size_t first,size_t last,const detail::TerrainMotion& motion,bool terrainWindow=false){
        std::vector<double> horizontal(last-first+1),floor(last-first+1);
        for(size_t i=first;i<=last;++i){
            if(i>first){const Vec3 delta=raw[i].position-raw[i-1].position;horizontal[i-first]=horizontal[i-first-1]+std::hypot(delta.x,delta.y);}
            floor[i-first]=absoluteFloor[i];
        }
        const bool poweredClimb=terrainWindow&&raw[last].position.z>raw[first].position.z;
        detail::TerrainTransfer transfer;
        // Preserved source ports are level with zero curvature. The active
        // terrain window can span operation boundaries without inserting a
        // level reset inside the climb.
        try{
            if(terrainWindow)
                transfer=detail::fitTerrainMotionWindow(horizontal,floor,raw[first].position.z,raw[last].position.z,2.,motion,cancel);
            else transfer=detail::fitTerrainMotionTransfer(horizontal,floor,raw[first].position.z,raw[last].position.z,2.,motion,cancel);
        }
        catch(const detail::TerrainTransferInfeasible&){
            // Terrain fitting is station-site feasibility, before physics or
            // energy feedback. A pinned feedback site must remain exact.
            if(feedback.planShape>=0)throw;
            rejectedTransferPlacements[transferRejectedSites++]=plan.placement;
            if(transferRejectedSites==int(rejectedTransferPlacements.size()))throw;
            return false;
        }
        if(poweredClimb){
            const double start=std::max(0.,transfer.activeStart-trainLength-terrainSupplyTarget*.5);
            ascentMotorBegin=first+size_t(std::lower_bound(horizontal.begin(),horizontal.end(),start)-horizontal.begin());
        }
        for(size_t i=first;i<=last;++i)raw[i].position.z=transfer.height(horizontal[i-first]);
        return true;
    };
    bool transfersFit=true;
    for(const auto& run:modules)if(run.identity=="post-loop-launch"||run.identity=="airtime-entry-launch"){
        auto motion=terrainMotion[run.identity=="post-loop-launch"?1:2];
        if(run.identity=="airtime-entry-launch"){
            // The reversing source exits before an unpowered connector/turn.
            // Carry that actual path's potential and losses to the motor inlet.
            double w=motion.entrySpeed*motion.entrySpeed;
            for(size_t i=pulloutExitIndex+1;i<=run.begin;++i){
                const Vec3 step=raw[i].position-raw[i-1].position;
                const auto energy=detail::pathEnergyStep(norm(step),step.z,-rollingAcceleration,dragAccelerationCoefficient);
                w=w*energy.attenuation+energy.offsetSpeedSquared;
                if(w<=0)throw detail::TerrainTransferInfeasible("Reversal source cannot coast to the existing tail motor");
            }
            motion.entrySpeed=std::sqrt(w);
        }
        if(!heightTransfer(run.begin,run.end,motion)){transfersFit=false;break;}
    }
    if(!transfersFit)continue;
    size_t ascentBegin=0;
    for(const auto& run:modules)if(run.corridor==0&&run.identity=="record-hill")ascentBegin=run.end;
    const bool poweredAscent=req.terrain.cliffHeight>0&&raw[loopEntryIndex].position.z-raw[ascentBegin].position.z>12;
    if(poweredAscent){
        if(!heightTransfer(ascentBegin,loopEntryIndex,terrainMotion[0],true))continue;
        auto first=std::find_if(modules.begin(),modules.end(),[&](const ModuleRun& run){return run.begin==ascentBegin;});
        auto last=std::find_if(first,modules.end(),[&](const ModuleRun& run){return run.begin==loopEntryIndex;});
        auto next=modules.erase(first,last);modules.insert(next,{ascentBegin,loopEntryIndex,0,"terrain-ascent-transfer"});
    }
    if(raw[tailEnd].position.z-stationDatum>12){
        for(size_t i=stationApproach.first;i<raw.size();++i)raw[i].position.z=stationDatum;
        if(!heightTransfer(tailEnd,stationApproach.first,terrainMotion[3],true))continue;
    }
    // Defer the first fully terrain-feasible site, not a station bay whose
    // connecting transfers cannot fit. This keeps the alternate geometry
    // attempt on a genuinely different placement after bounded site retries.
    if(placementVariant&&feedback.planShape<0&&firstFeasiblePlacement<0){firstFeasiblePlacement=plan.placement;continue;}
    plan.stationSelectionEligible=true;
    // Terrain composition fixes the source ports before operation work is fit.
    for(size_t i=1;i<raw.size();++i)distance[i]=distance[i-1]+norm(raw[i].position-raw[i-1].position);
    if(distance[loopEntryIndex]-distance[ascentBegin]<=loopTrimLength)
        throw std::runtime_error("Existing loop approach has insufficient physical trim domain");
    loopBrakeStartIndex=size_t(std::lower_bound(distance.begin()+ascentBegin,distance.begin()+loopEntryIndex,distance[loopEntryIndex]-loopTrimLength)-distance.begin());
    pending.push_back({loopBrakeStartIndex,loopEntryIndex,DriveKind::Brake,loopTrimTarget,serviceBrakeAcceleration});
    if(poweredAscent){
        if(ascentMotorBegin>=loopBrakeStartIndex)throw std::runtime_error("Terrain ascent leaves no physical motor interval before loop trim");
        pending.push_back({ascentMotorBegin,loopBrakeStartIndex,DriveKind::Boost,terrainAscentSpeed});
    }
    // Match motion locally at each boundary. A single polynomial across a
    // whole recovery erased its authored crest and amplified endpoint jets
    // into an unintended hill. Keep the connector interior and its energy
    // intent; drive zones retain their indices through compilation.
    std::vector<std::pair<size_t,size_t>> flowJoins;
    for(size_t m=0;m+1<modules.size();++m){
        const auto& left=modules[m];const auto& right=modules[m+1];
        if(left.identity=="station"||left.identity=="departure-launch")continue;
        if(left.identity=="terrain-ascent-transfer"&&right.identity=="record-inversion")continue;
        // Airtime sections already have level zero-curvature guards. Keep
        // their matching source ports; refitting them can amplify tiny jets.
        if(left.identity=="fvd-airtime"||right.identity=="fvd-airtime"||left.identity=="record-hill"||right.identity=="record-hill")continue;
        // Joint FVD sources already meet at their actual pose and zero-load
        // derivative ports. Do not replace their interiors with a join warp.
        if(right.identity=="record-inversion"||right.identity=="immelmann-inward-entry-brake"||right.identity=="high-immelmann"||right.identity=="descending-pullout"||left.identity=="descending-pullout")continue;
        auto forceInversion=[](const std::string& identity){return identity=="record-inversion"||identity=="high-immelmann"||identity=="dive-loop"||identity=="fvd-airtime";};
        // FVD pitch/roll interiors are already force-designed. Borrow only
        // their straight guards, never overwrite the force ramp itself.
        const double begin=distance[left.end]-std::min(forceInversion(left.identity)?6.:65.,.25*(distance[left.end]-distance[left.begin]));
        const double end=distance[right.begin]+std::min(forceInversion(right.identity)?6.:65.,.25*(distance[right.end]-distance[right.begin]));
        // Terminal braking and the level station approach retain their own
        // physical profiles; this pass composes the moving ride's elements.
        size_t first=size_t(std::lower_bound(distance.begin(),distance.end(),begin)-distance.begin());
        if(first>=tailEnd)continue;
        size_t last=size_t(std::lower_bound(distance.begin(),distance.end(),end)-distance.begin());
        try{detail::blendAuthoredJoin(raw,first,last,cancel);}catch(const std::exception& e){throw std::runtime_error("Join "+left.identity+" to "+right.identity+": "+e.what());}flowJoins.push_back({first,last});
        // The whole curved part of a drive participates in force-aligned
        // banking, including shared endpoints. Its motor remains an operation.
        if(raw[first].element==Element::Turn||raw[last].element==Element::Turn)
            for(size_t i=first;i<=last;++i)if(raw[i].element==Element::Launch||raw[i].element==Element::Brake||raw[i].element==Element::Return)raw[i].element=Element::Turn;
    }
    for(size_t i=1;i<raw.size();++i)distance[i]=distance[i-1]+norm(raw[i].position-raw[i-1].position);
    raw.back()=raw.front();try{d.track=compile(raw);}catch(const std::exception& e){throw std::runtime_error("Composed circuit: "+std::string(e.what()));}
    // Terrain may translate a force source, but compiling a surrounding
    // polyline must not replace its known physical tangent, curvature or up.
    const auto preserveSource=[&](const std::vector<SourceKnot>& source){
        const Vec3 shift=raw[source.front().index].position-source.front().source.position;
        for(const auto& point:source){
            if(norm(raw[point.index].position-point.source.position-shift)>1e-5)
                throw std::runtime_error("Force source was warped by terrain or a connector");
            auto& knot=d.track.knots[point.index];knot.tangent=point.source.tangent;knot.curvature=point.source.curvature;knot.up=point.source.up;knot.bank=0;
        }
    };
    preserveSource(signatureKnots);for(const auto& source:airtimeKnots)preserveSource(source);
    const Vec3 signatureShift=raw[signatureKnots.front().index].position-signatureKnots.front().source.position;
    d.track.rebuild();
    auto at=[&](size_t index){return index>=d.track.spans.size()?d.track.length:d.track.spans[index].start;};
    double signaturePositionResidual=0,signatureCurvatureResidual=0;
    const double signatureBegin=at(signatureKnots.front().index),signatureEnd=at(signatureKnots.back().index);
    for(const auto& point:signatureKnots){
        auto q=d.track.sample(signatureBegin+point.sourceDistance);
        signaturePositionResidual=std::max(signaturePositionResidual,norm(q.position-point.source.position-signatureShift));
        signatureCurvatureResidual=std::max(signatureCurvatureResidual,norm(q.curvature-point.source.curvature));
    }
    std::array<double,2> signaturePortCurvatureS{},signaturePortUpSS{},signatureSourceCurvatureS{},signatureSourceUpSS{};
    for(int port=0;port<2;++port){
        auto actual=sampleKinematics(d.track,port?signatureEnd:signatureBegin);
        auto source=sampleKinematics(signature.section.track,port?signature.section.track.length:0);
        signaturePortCurvatureS[port]=norm(actual.curvatureS);signaturePortUpSS[port]=norm(actual.upSS);
        signatureSourceCurvatureS[port]=norm(source.curvatureS);signatureSourceUpSS[port]=norm(source.upSS);
    }
    ports.signatureEntry=signatureBegin;ports.signatureExit=signatureEnd;
    ports.signatureEntrySpeedHint=signature.authoring.speed;ports.signatureExitSpeedHint=signature.exit.speed;
    for(size_t i=0;i<airtimeChecks.size();++i){ports.airtimeProfile[i].clear();for(const auto& [index,check]:airtimeChecks[i]){
        auto placed=check;placed.distance=at(index);ports.airtimeProfile[i].push_back(placed);}}
    ports.reversalExit=at(reversalExitIndex);ports.reversalSpeedHint=reversalSource.rollExit.speed;ports.reversalEntry=at(reversalEntryIndex);ports.reversalEntrySpeedHint=reversalRequest.entrySpeed;
    ports.pulloutExit=at(pulloutExitIndex);ports.pulloutSpeedHint=valley.speed;
    ports.reversalBrakeStart=at(reversalBrakeStartIndex);ports.relaunchEnd=at(relaunchEndIndex);
    ports.loopEntry=at(loopEntryIndex);ports.loopBrakeStart=at(loopBrakeStartIndex);ports.loopApex=at(loopApexIndex);ports.loopSpeedHint=loopShape.apex.speed;
    ports.loopEntrySpeedHint=loopShape.samples.front().speed;ports.loopExit=at(loopExitIndex);ports.loopExitSpeedHint=loopShape.samples.back().speed;
    ports.planShape=plan.shape;ports.planPlacement=plan.placement;ports.loopApproachDomain=loopApproachLength;
    ports.ordinaryTurn.fill(false);
    for(const auto& run:modules)if(run.identity=="banked-camelback-turn"){
        ports.ordinaryTurn[run.corridor]=true;ports.turnBegin[run.corridor]=at(run.begin);ports.turnEnd[run.corridor]=at(run.end);
    }
    // Supply feedback belongs to the nearest existing motor before the trim,
    // including a terrain ascent when present. A brake cannot add missing work.
    size_t loopSupply=pending.size();
    for(size_t j=0;j<pending.size();++j)if((pending[j].kind==DriveKind::Launch||pending[j].kind==DriveKind::Boost)&&pending[j].end<=loopBrakeStartIndex&&
        (loopSupply==pending.size()||pending[j].end>pending[loopSupply].end))loopSupply=j;
    if(loopSupply==pending.size())throw std::runtime_error("Loop energy planning has no preceding powered section");
    ports.loopSupplyIndependent=pending[loopSupply].kind==DriveKind::Boost;
    ports.loopSupplyEnd=at(pending[loopSupply].end);
    for(size_t pendingIndex=0;pendingIndex<pending.size();++pendingIndex){const auto p=pending[pendingIndex];
        bool hard=p.kind==DriveKind::Launch;double acc=hard?launchAcceleration:p.acceleration,targetSpeed=p.speed;
        // Keep the initial upstream rail under the complete stopped train.
        // Every later rail is inside its center-domain at both ends. Adjacent
        // motor/brake domains then have one train length of hardware separation.
        const double hardwareStart=at(p.begin)+(hard?0:trainHalf),hardwareEnd=at(p.end)-trainHalf;
        if(hardwareEnd<=hardwareStart)throw std::runtime_error("Operation work domain cannot contain the complete train and physical rail");
        if(pendingIndex==loopSupply){
            if(ports.loopSupplyIndependent){
                targetSpeed=terrainSupplyTarget;
                if(!std::isfinite(targetSpeed)||targetSpeed<=0)throw std::runtime_error("Loop supply target left its finite energy domain");
            }
            if(ports.loopSupplyIndependent&&feedback.loopSupplyEnergyCorrection>0){
                const double usable=hardwareEnd-hardwareStart-targetSpeed*(hard?2*departureRampSeconds:1.);
                if(usable<=0)throw std::runtime_error("Loop supply has no length after finite train and drive ramps");
                const double extraDrag=.5*req.train.airDensity*req.train.dragCdA/(req.train.cars*req.train.carMass)*feedback.loopSupplyEnergyCorrection;
                acc=std::min(req.limits.maxLongitudinalG*gravity-(hard?.02:.1),acc+feedback.loopSupplyEnergyCorrection/(2*usable)+extraDrag);
            }
            ports.loopSupplyTarget=targetSpeed;
        }
        // Gravity and drag already remove work on a climbing approach. Size
        // only the remaining trim work, after canonical terrain fitting.
        const bool loopTrim=p.kind==DriveKind::Brake&&p.begin==loopBrakeStartIndex&&p.end==loopEntryIndex;
        if(loopTrim){
            const double start=at(p.begin),end=at(p.end);
            const auto coast=detail::estimatePassiveTransfer(d.track,req.train,start,end,feedback.loopApproachSpeed,cancel);
            const double effective=hardwareEnd-hardwareStart-.5*(feedback.loopApproachSpeed+targetSpeed);
            if(effective<=0)throw std::runtime_error("Loop trim has no physical braking length");
            acc=coast.reached?std::max(0.,(coast.speed*coast.speed-targetSpeed*targetSpeed)/(2*effective)):0;
            acc=std::min(req.limits.maxLongitudinalG*gravity-.1,acc);
        }
        if(p.kind==DriveKind::Boost||(p.kind==DriveKind::Brake&&!loopTrim)){
            double opposingGrade=0;for(double s=hardwareStart;s<hardwareEnd;s+=2)opposingGrade=std::max(opposingGrade,(p.kind==DriveKind::Brake?-1:1)*d.track.sample(s).tangent.z);
            acc=std::min(req.limits.maxLongitudinalG*gravity-.1,acc+gravity*opposingGrade);
        }
        d.operations.push_back({hardwareStart,hardwareEnd,p.kind,targetSpeed,req.train.carMass*acc,req.train.carMass*acc*100,hard?departureRampSeconds:.5});

    }
    // Terminal hardware starts after the last authored airtime section, with
    // room for the rear car to clear it before the front car reaches a brake.
    // Size stopping work from the available return, never put brake hardware
    // inside a thrill element merely to obtain a fixed stopping distance.
    const double brakingStart=at(tailEnd)+2*trainHalf+2;
    const double stoppingRoom=d.track.length-brakingStart-2*trainHalf-65;
    if(stoppingRoom<=0)throw std::runtime_error("Final element leaves no physical terminal brake run");
    const double stopDeceleration=std::max(terminalDeceleration,65*65/(2*stoppingRoom));
    if(stopDeceleration>std::min(20.,req.limits.maxLongitudinalG*gravity-.1))
        throw std::runtime_error("Final element leaves insufficient bounded stopping work");
    // A real descent can exert more than the former 4 m/s^2 brake rating.
    // Size the hardware against the canonical downhill grade plus the same
    // requested stopping deceleration; the controller still applies bounded
    // force/power through its ramp and the complete train is resimulated.
    double terminalDownhill=0;
    for(double s=brakingStart;s<d.track.length;s+=1)terminalDownhill=std::max(terminalDownhill,-d.track.sample(s).tangent.z);
    const double terminalBrakeAcceleration=std::min(req.limits.maxLongitudinalG*gravity-.1,std::max(4.,stopDeceleration+gravity*terminalDownhill+1));
    d.operations.push_back({brakingStart,80,DriveKind::Station,0,req.train.carMass*terminalBrakeAcceleration,req.train.carMass*terminalBrakeAcceleration*100,.5,stopDeceleration,.2});
    for(auto& operation:d.operations)operation.exitFadeMeters=std::max(1.,operation.targetSpeed*operation.rampSeconds);
    coalesceDriveProfiles(d.operations);
    d.topology+="/coupled-fvd-immelmann";
    if(!flowJoins.empty())d.topology+="/continuous-module-joins";
    std::ostringstream diagnostic;diagnostic<<std::setprecision(12)<<"{\"schemaVersion\":1,\"generationOnly\":true,\"seed\":"<<req.seed<<",\"candidate\":"<<attempt<<",\"order\":\""<<orderName<<"\",\"stationAnchoring\":{\"geometryScale\":"<<geometryAttempt<<",\"placementVariant\":"<<placementVariant<<",\"deferredFirstPlacement\":"<<firstFeasiblePlacement<<",\"selectedTerrainRank\":"<<(&plan-plans.data())<<",\"checkedPlanCount\":"<<(&plan-plans.data()+1)<<",\"transferRejectedSites\":"<<transferRejectedSites<<",\"heightBudget\":16,\"datum\":"<<stationDatum<<",\"groundMinimum\":"<<stationGroundMin<<",\"groundMaximum\":"<<stationGroundMax<<",\"datumAboveLowestGround\":"<<stationDatum-stationGroundMin<<",\"datumAboveHighestGround\":"<<stationDatum-stationGroundMax<<",\"fixedDepartureMeters\":"<<fixedDeparture<<",\"fixedReturnMeters\":"<<fixedReturn<<",\"maximumControlSpacing\":"<<spacing<<"},\"launchPlanning\":{\"requestedSeconds\":"<<req.targets.launchSeconds<<",\"motorAcceleration\":"<<launchAcceleration<<",\"predictedSeconds\":"<<plannedLaunchTime(launchAcceleration,req.train)<<"},\"turnLoadIntentG\":"<<designNormalG<<",\"sides\":"<<sides<<",\"plannedHorizontalLength\":"<<plan.length<<",\"canonicalLength\":"<<d.track.length<<",\"selected\":{\"shape\":"<<plan.shape<<",\"placement\":"<<plan.placement<<",\"score\":"<<plan.score<<",\"transferEstimateFailures\":"<<plan.transferFailures<<",\"terrainRelief\":"<<plan.relief<<",\"terrainStationDeviation\":"<<plan.deviation<<",\"terrainGradeRms\":"<<plan.grade<<",\"stationGrade\":"<<plan.stationGrade<<",\"groundMinimum\":"<<plan.groundMinimum<<",\"groundMaximum\":"<<plan.groundMaximum<<",\"valleyFraction\":"<<plan.valleyFraction<<",\"meanValleyDistance\":"<<plan.valleyDistance<<",\"valleyCrossings\":"<<plan.valleyCrossings<<"},\"scoreWeights\":{\"terrainRelief\":"<<(dramaticTerrain?0:1.5)<<",\"terrainReliefTarget\":"<<(dramaticTerrain?req.terrain.cliffHeight:0)<<",\"terrainReliefTargetDeviation\":"<<(dramaticTerrain?2:0)<<",\"terrainStationDeviation\":"<<(dramaticTerrain?.1:.5)<<",\"terrainGradeRms\":200,\"stationGrade\":"<<(dramaticTerrain?350:150)<<",\"horizontalLength\":0.015384615384615385,\"canyonOutsideValleyFraction\":"<<(req.terrain.kind==TerrainKind::Canyon?100:0)<<",\"valleyCoordinateUsesTerrainProfile\":"<<"true"<<"},\"corridors\":[";
    for(int i=0;i<sides;++i){if(i)diagnostic<<',';diagnostic<<"{\"length\":"<<plan.lengths[i]<<",\"turnAngle\":"<<plan.angles[i]<<",\"turnSpeedMps\":"<<feedback.turnSpeed[i]<<",\"turnRadiusMeters\":"<<turns[i].radius<<",\"turnRampMeters\":"<<turns[i].ramp<<'}';}diagnostic<<"],\"rankedPlans\":[";
    for(size_t i=0;i<plans.size();++i){if(i)diagnostic<<',';auto& p=plans[i];diagnostic<<"{\"shape\":"<<p.shape<<",\"placement\":"<<p.placement<<",\"score\":"<<p.score<<",\"transferEstimateFailures\":"<<p.transferFailures<<",\"relief\":"<<p.relief<<",\"deviation\":"<<p.deviation<<",\"grade\":"<<p.grade<<",\"stationGrade\":"<<p.stationGrade<<",\"length\":"<<p.length<<",\"groundMinimum\":"<<p.groundMinimum<<",\"groundMaximum\":"<<p.groundMaximum<<",\"valleyFraction\":"<<p.valleyFraction<<",\"meanValleyDistance\":"<<p.valleyDistance<<",\"valleyCrossings\":"<<p.valleyCrossings<<",\"stationBudgetChecked\":"<<(p.stationChecked?"true":"false")<<",\"stationBudgetFeasible\":"<<(p.stationFeasible?"true":"false")<<",\"stationSelectionEligible\":"<<(p.stationSelectionEligible?"true":"false");if(p.stationChecked)diagnostic<<",\"stationBayMinimum\":"<<p.stationBayMinimum<<",\"stationBayMaximum\":"<<p.stationBayMaximum<<",\"stationRequiredDatum\":"<<p.stationRequiredDatum;diagnostic<<'}';}diagnostic<<"],\"modules\":[";
    for(size_t i=0;i<modules.size();++i){if(i)diagnostic<<',';auto& m=modules[i];diagnostic<<"{\"identity\":\""<<m.identity<<"\",\"corridor\":"<<m.corridor<<",\"start\":"<<at(m.begin)<<",\"end\":"<<at(m.end)<<'}';}diagnostic<<"]}";d.planningDiagnostics=diagnostic.str();
    d.planningDiagnostics.pop_back();std::ostringstream expansion;expansion<<std::setprecision(12)<<",\"layoutExpansion\":{\"source\":\"coupled-immelmann\",\"crossingLiftMeters\":"<<crossingLift;
    expansion<<",\"tallHill\":{\"height\":"<<signature.height<<",\"span\":"<<signature.span<<",\"arc\":"<<signature.section.track.length<<",\"sourceInlet\":"<<signature.authoring.speed<<",\"sourceExit\":"<<signature.exit.speed<<",\"rollingAcceleration\":"<<signature.authoring.rollingAcceleration<<",\"dragCoefficient\":"<<signature.authoring.dragAccelerationCoefficient<<",\"maxSourceReplayNormalResidual\":"<<signature.section.assessment.maxNormalResidualG<<",\"positionResidual\":"<<signaturePositionResidual<<",\"curvatureResidual\":"<<signatureCurvatureResidual<<",\"portCurvatureS\":["<<signaturePortCurvatureS[0]<<','<<signaturePortCurvatureS[1]<<"],\"portUpSS\":["<<signaturePortUpSS[0]<<','<<signaturePortUpSS[1]<<"],\"sourcePortCurvatureS\":["<<signatureSourceCurvatureS[0]<<','<<signatureSourceCurvatureS[1]<<"],\"sourcePortUpSS\":["<<signatureSourceUpSS[0]<<','<<signatureSourceUpSS[1]<<"]}";
    expansion<<",\"sourceEntrySpeedMps\":"<<reversalRequest.entrySpeed<<",\"sourceInvertedHeightMeters\":"<<reversalSource.apex.position.z<<",\"sourceApexHeightMeters\":"<<reversalSource.apex.position.z<<",\"sourceExitHeightMeters\":"<<reversalSource.rollExit.position.z<<",\"sourceExitSpeedMps\":"<<reversalSource.rollExit.speed<<",\"sourceExitPitchRadians\":"<<std::asin(reversalSource.rollExit.forward.z)<<",\"pulloutDurationSeconds\":"<<(reversalSource.exit.time-reversalSource.rollExit.time)<<",\"moduleHorizontalPathLength\":"<<reversalPlanLength<<",\"entryEnergyCorrection\":"<<feedback.reversalEnergyCorrection<<",\"entrySpeedHint\":"<<reversalEntrySpeed<<",\"leadHorizontalLength\":"<<reversalLead<<",\"exitHeadingRadians\":"<<reversalHeading;
    expansion<<",\"flowJoins\":[";
    for(size_t i=0;i<flowJoins.size();++i){if(i)expansion<<',';expansion<<"{\"start\":"<<at(flowJoins[i].first)<<",\"end\":"<<at(flowJoins[i].second)<<'}';}
    expansion<<"],\"forceDesignedAirtime\":[";
    for(size_t i=0;i<forceHills.size();++i){if(i)expansion<<',';const auto& hill=forceHills[i];const auto& exit=hill.section.samples.back();
        expansion<<"{\"span\":"<<hill.span<<",\"height\":"<<hill.height<<",\"sourceSpeedMps\":"<<hill.authoring.speed
            <<",\"sourceExitSpeedMps\":"<<exit.speed<<",\"sourceDurationSeconds\":"<<exit.time<<",\"dissipatedWorkPerMass\":"<<exit.dissipatedWorkPerMass
            <<",\"sourceReplayPassed\":"<<(hill.section.assessment.passed?"true":"false")<<",\"maxSourceNormalResidualG\":"<<hill.section.assessment.maxNormalResidualG<<'}';}
    expansion<<"]}}";d.planningDiagnostics+=expansion.str();return d;
    }
    throw std::runtime_error(placementVariant?"No second distinct eligible placement survives source, station and terrain-transfer constraints":"No eligible terrain-ranked placement survives source, station and terrain-transfer constraints");
}
static void placeSupports(Design& d,Cancel cancel){buildSupportLayout(d,cancel);}
static void improveBanking(Design& d){
    const auto& frames=d.simulation.frames;if(frames.empty())return;
    const size_t count=d.track.spans.size();std::vector<double> authored(count),target(count),along(count),left(count),right(count),speed(count);
    for(size_t i=0;i<count;++i){const auto& k=d.track.knots[i];authored[i]=target[i]=k.bank;along[i]=d.track.spans[i].start;
        if(k.element!=Element::Turn)continue;speed[i]=replayValueAt(frames,along[i]);
        Vec3 required=k.curvature*(speed[i]*speed[i])+Vec3{0,0,gravity},side=cross(k.tangent,k.up);double normal=dot(required,k.up),lateral=dot(required,side);
        target[i]=detail::forceAxisBank(normal,lateral,k.bank);
    }
    double edge=0;
    for(size_t i=0;i<count;++i){if(d.track.knots[i].element!=Element::Turn)edge=along[i]+d.track.spans[i].length;left[i]=std::max(0.,along[i]-edge);}
    edge=d.track.length;for(size_t i=count;i-->0;){if(d.track.knots[i].element!=Element::Turn)edge=along[i];right[i]=std::max(0.,edge-along[i]);}
    // A 1.2-second authoring window on either side allows finite roll response
    // through low-load S crests. This is not an ASTM limit; full-train replay
    // still assesses the resulting canonical frame and rider-offset forces.
    for(size_t i=0;i<count;++i){auto& k=d.track.knots[i];if(k.element!=Element::Turn)continue;double weighted=0,total=0,radius=std::max(1.,speed[i]*1.2);
        size_t first=i;while(first&&along[i]-along[first-1]<radius)--first;
        for(size_t j=first;j<count&&along[j]-along[i]<radius;++j){double u=(along[j]-along[i])/radius,w=std::pow(std::max(0.,1-u*u),4)*d.track.spans[j].length;weighted+=w*target[j];total+=w;}
        double fade=layoutSmooth(std::min(left[i],right[i])/radius);double bank=authored[i]+fade*(weighted/total-authored[i]);
        k.bank=bank*stationBankFactor(d.track.length-along[i],d.request.train);
    }
    d.track.knots.back()=d.track.knots.front();d.track.rebuild();
}

void evaluateTargets(Design& d){
    if(d.supports.empty())d.report.fail("SUPPORT_LAYOUT","Design contains no connected supports");
    auto& m=d.simulation.metrics;const auto& req=d.request;double low=1e9,high=-1e9,station=d.track.knots.front().position.z;
    for(double s=0;s<d.track.length;s+=1){auto p=d.track.sample(s);double gh=p.position.z-req.terrain.height(p.position.x,p.position.y);low=std::min(low,p.position.z);high=std::max(high,p.position.z);m.maxGroundHeight=std::max(m.maxGroundHeight,gh);m.minGroundClearance=std::min(m.minGroundClearance,gh);
        if(p.element==Element::Inversion&&p.up.z<-.5)m.inversionGroundHeight=std::max(m.inversionGroundHeight,gh);
    }m.heightAboveStation=high-station;m.verticalRelief=high-low;
    auto min=[&](const char* code,double value,double goal){if(!std::isfinite(value)||value+1e-6<goal)d.report.fail(code,"Measured result misses requested target",0,value,goal);};
    min("HEIGHT_TARGET",m.maxGroundHeight,req.targets.height);min("INVERSION_TARGET",m.inversionGroundHeight,req.targets.inversionHeight);
    auto dynamic=validateSimulationTargets(d.simulation,req.targets,req.limits);
    d.report.errors.insert(d.report.errors.end(),dynamic.errors.begin(),dynamic.errors.end());
    d.report.warnings.push_back("Higher historical restraint-dependent force profile selected; train/restraint provisions and reference calibration remain unverified.");
    if(d.simulation.completed&&movingRideSeconds(d)>180)d.report.warnings.push_back("Moving ride exceeds the 180-second pacing goal; physical acceptance is unchanged.");
}
ValidationReport validateRequest(const GenerationRequest& req){
    ValidationReport r;if(!req.terrain.valid()){r.fail("TERRAIN_PROFILE","Terrain profile is outside its supported domain");return r;}const auto& t=req.targets;const auto& l=req.limits;
    for(double value:{t.height,t.speed,t.inversionHeight,t.launchSeconds,l.minVerticalG,l.maxVerticalG,l.maxLateralG,l.maxLongitudinalG,l.maxJerkGps,l.minClearance,req.simulationStep})if(!std::isfinite(value)){r.fail("REQUEST_RANGE","Request contains a nonfinite value");return r;}
    if(req.maxCandidates<1||req.maxCandidates>64||t.height<0||t.height>350||t.speed<1||t.speed>110||t.inversionHeight<0||t.inversionHeight>140||t.launchSeconds<.8||t.launchSeconds>10||l.maxVerticalG<=l.minVerticalG||l.maxLateralG<=0||l.maxLongitudinalG<=0||l.maxJerkGps<=0||l.minClearance<0||req.simulationStep!=1./960||int(req.terrain.kind)<0||int(req.terrain.kind)>2)r.fail("REQUEST_RANGE","Request is outside the prototype's supported domain");
    if(std::isinf(t.referenceExposure)||(std::isfinite(t.referenceExposure)&&t.referenceExposure<=0)||(!t.referenceId.empty()&&!std::isfinite(t.referenceExposure))||(std::isfinite(t.referenceExposure)&&t.referenceId.empty()))r.fail("REFERENCE_CONFIG","Configured reference needs a finite positive exposure and a nonempty ID");
    auto reference=validateReference(t);r.errors.insert(r.errors.end(),reference.errors.begin(),reference.errors.end());
    for(double rate:{l.maxLateralRateGps,l.maxLongitudinalRateGps})if(std::isinf(rate)||(std::isfinite(rate)&&rate<=0))r.fail("AXIS_RATE_CONFIG","Optional component rate gates must be finite positive values, or unset");
    const auto& train=req.train;
    for(double value:{train.carMass,train.spacing,train.seatHeight,train.dragCdA,train.rollingResistance,train.airDensity})if(!std::isfinite(value)){r.fail("TRAIN_CONFIG","Train contains nonfinite settings");return r;}
    if(train.cars<1||train.cars>16||train.spacing<=0||train.spacing>20||train.carMass<=0||train.seatHeight<0||train.seatHeight>3||train.dragCdA<0||train.rollingResistance<0||train.airDensity<0)r.fail("TRAIN_CONFIG","Train is outside the supported model domain");
    return r;
}
Design generate(const GenerationRequest& input,Cancel cancel,std::function<void(int,const std::string&)> progress){
    GenerationRequest req=input;if(req.terrain.isDefaultProfile())req.terrain=Terrain::seeded(req.terrain.kind,req.seed);
    Design last;last.request=req;
    last.report=validateRequest(req);if(!last.report.valid())return last;
    double fastestDeparture=plannedLaunchTime(req.limits.maxLongitudinalG*gravity-.02,req.train);
    if(fastestDeparture>req.targets.launchSeconds){last.report.fail("LAUNCH_FEASIBILITY","Requested departure is below the force-limited flat-station prototype bound",0,fastestDeparture,req.targets.launchSeconds);return last;}
    if(req.targets.requireIntensity&&std::isfinite(req.targets.referenceExposure)&&req.targets.referenceExposure*1.1>10*std::max(0.,req.limits.maxVerticalG)){last.report.fail("INTENSITY_FEASIBILITY","Requested exposure exceeds ten seconds at the selected vertical force ceiling",0,req.targets.referenceExposure*1.1,10*std::max(0.,req.limits.maxVerticalG));return last;}
    std::vector<std::string> history;Design bestIntensity,bestPacing;bool haveBestIntensity=false,haveBestPacing=false;double bestDuration=INFINITY;
    auto remember=[&](const Design* d,int index,const char* failure="CANDIDATE_FAILURE"){
        std::ostringstream h;h<<std::setprecision(12)<<"{\"candidate\":"<<index<<",\"constructed\":"<<(d?"true":"false");
        if(d){h<<",\"completed\":"<<(d->simulation.completed?"true":"false")<<",\"exposure10Seconds\":"<<d->simulation.metrics.exposure10Seconds<<",\"launchSeconds\":";if(std::isfinite(d->simulation.metrics.launchTo180))h<<d->simulation.metrics.launchTo180;else h<<"null";if(d->simulation.completed)h<<",\"movingDurationSeconds\":"<<movingRideSeconds(*d);h<<",\"errors\":[";bool comma=false;for(const auto* report:{&d->report,&d->simulation.report})for(const auto& f:report->errors){if(comma)h<<',';comma=true;h<<'"'<<f.code<<'"';}h<<']';}
        else h<<",\"errors\":[\""<<failure<<"\"]";h<<'}';history.push_back(h.str());
    };
    auto withHistory=[&](Design d,const char* selection){
        if(d.planningDiagnostics.empty())d.planningDiagnostics="{\"generationOnly\":true}";
        d.planningDiagnostics.pop_back();
        if(d.simulation.completed){std::ostringstream pacing;pacing<<std::setprecision(12)<<",\"movingDurationSeconds\":"<<movingRideSeconds(d)<<",\"movingDurationGoalMaximumSeconds\":180,\"movingDurationDefinition\":\"Powered departure to train center reaching terminal Station operation start; excludes terminal stopping\"";d.planningDiagnostics+=pacing.str();}
        d.planningDiagnostics+=",\"searchSelection\":\""+std::string(selection)+"\",\"candidateHistory\":[";
        for(size_t i=0;i<history.size();++i){if(i)d.planningDiagnostics+=',';d.planningDiagnostics+=history[i];}d.planningDiagnostics+="]}";return d;
    };
    for(int i=0;i<req.maxCandidates;++i){
        if(cancel&&cancel()){last.simulation.cancelled=true;last.report.fail("CANCELLED","Generation cancelled");return last;}
        if(progress)progress(i,"Solving terrain corridor and circuit");
        Design d;
        try{
            AuthoringFeedback feedback;CandidatePorts ports;
            PreparedSources preparedSources;
            d=candidate(req,i,cancel,feedback,ports,preparedSources);d.simulation=simulate(d.track,d.operations,req.train,req.simulationStep,cancel);
            if(d.simulation.cancelled){d.report.fail("CANCELLED","Generation cancelled");return d;}
            bool feedbackConverged=false;double energyResidual=INFINITY;int energyIterations=0;std::ostringstream energyHistory;energyHistory<<std::setprecision(12);
            // Hold the selected route family/site while solving actual entry
            // energy. Re-ranking after every correction could oscillate
            // between different terrain itineraries instead of converging.
            feedback.planShape=ports.planShape;feedback.planPlacement=ports.planPlacement;feedback.loopApproachDomain=ports.loopApproachDomain;
            const int provisionalShape=ports.planShape,provisionalPlacement=ports.planPlacement;
            bool routeReselected=false;int routeReselectionIteration=-1;
            Design convergedProvisional;CandidatePorts convergedPorts;double provisionalResidual=INFINITY;
            bool haveConvergedProvisional=false,retainedProvisional=false;int attemptedShape=-1,attemptedPlacement=-1;
            double failedAlternateResidual=INFINITY;
            for(int correction=0;correction<=8;++correction){
                if(!ports.loopSupplyIndependent)feedback.loopSupplyEnergyCorrection=0;
                const auto& frames=d.simulation.frames;
                const bool completed=d.simulation.completed;
                const bool stalled=!d.simulation.report.errors.empty()&&std::all_of(d.simulation.report.errors.begin(),d.simulation.report.errors.end(),[](const Finding& f){return f.code=="STALL";});
                auto reached=[&](double at){return !frames.empty()&&at>=frames.front().distance&&at<=frames.back().distance;};
                // One measured-port update handles complete trajectories and a
                // genuine stall. Other failures cannot be repaired with energy.
                // An optional new route must not bootstrap past a converged one.
                energyResidual=completed?0:INFINITY;
                if(!completed&&(!stalled||haveConvergedProvisional))break;
                AuthoringFeedback next=feedback;bool changed=false;
                auto relax=[&](double& target,double observation,double weight=.75){
                    const double delta=weight*(observation-target);target+=delta;changed|=std::abs(delta)>1e-6;
                };
                auto compare=[&](double at,double intended){if(completed||reached(at))energyResidual=std::max(energyResidual,std::abs(replayValueAt(frames,at)-intended));};
                compare(ports.signatureEntry,ports.signatureEntrySpeedHint);
                compare(ports.signatureExit,ports.signatureExitSpeedHint);
                compare(ports.loopEntry,ports.loopEntrySpeedHint);
                compare(ports.loopApex,ports.loopSpeedHint);
                compare(ports.loopExit,ports.loopExitSpeedHint);
                compare(ports.reversalEntry,ports.reversalEntrySpeedHint);
                compare(ports.reversalExit,ports.reversalSpeedHint);
                compare(ports.pulloutExit,ports.pulloutSpeedHint);
                for(size_t h=0;h<ports.airtimeProfile.size();++h){
                    for(const auto& check:ports.airtimeProfile[h])compare(check.distance,check.speed);
                    if(reached(ports.airtimeProfile[h].front().distance))
                        relax(next.airtimeSpeed[h],replayValueAt(frames,ports.airtimeProfile[h].front().distance));
                }
                // Bound turn authoring with the maximum measured speed while
                // any car occupies it. An unreached turn has no observation.
                const double halfTrain=(req.train.cars-1)*req.train.spacing*.5;
                for(size_t side=0;side<ports.ordinaryTurn.size();++side)if(ports.ordinaryTurn[side]&&reached(ports.turnEnd[side]+halfTrain)){
                    double measured=0;
                    for(const auto& frame:frames)if(frame.distance>=ports.turnBegin[side]-halfTrain&&frame.distance<=ports.turnEnd[side]+halfTrain)
                        measured=std::max(measured,frame.speed);
                    energyResidual=std::max(energyResidual,std::abs(measured-feedback.turnSpeed[side]));
                    relax(next.turnSpeed[side],measured,side==3?1.:.5);
                }
                // Rails are wholly inside these train-center work domains.
                // Correct trim against the clean source inlet; independently
                // compare later apices/exits rather than masking source losses.
                auto trim=[&](double entry,double intended,double& correction){if(reached(entry)){
                    const double observed=replayValueAt(frames,entry);
                    relax(correction,correction+intended*intended-observed*observed);
                }};
                trim(ports.loopEntry,ports.loopEntrySpeedHint,next.loopEnergyCorrection);
                trim(ports.reversalEntry,ports.reversalEntrySpeedHint,next.reversalEnergyCorrection);
                // A brake removes energy; its passive map therefore supplies a
                // minimum upstream requirement, not an exact braked trajectory.
                // The real motor can add or give back its previous correction.
                auto supply=[&](double motorEnd,double motorTarget,double brakeBegin,double entry,double intended,double& correction){
                    if(!reached(motorEnd)||!reached(brakeBegin))return;
                    const auto approach=detail::makePassiveTransfer(d.track,req.train,brakeBegin,entry,cancel);
                    const auto upstream=detail::makePassiveTransfer(d.track,req.train,motorEnd,brakeBegin,cancel);
                    const double required=std::max({0.,(intended*intended-approach.offsetSpeedSquared)/approach.retention,
                        approach.minimumEntrySpeedSquared});
                    if(required<=approach.minimumEntrySpeedSquared)
                        throw std::runtime_error("Braked approach has an interior energy barrier that needs an authored traversal speed");
                    const double observed=replayValueAt(frames,brakeBegin);
                    const double needed=std::max(0.,correction+(required-observed*observed)/upstream.retention);
                    // Matching inlet speed through braking cannot hide surplus
                    // propulsion. Its removal must converge in the same units.
                    energyResidual=std::max(energyResidual,std::abs(std::sqrt(motorTarget*motorTarget+needed-correction)-motorTarget));
                    relax(correction,needed);
                };
                // The initial launch already owns an immutable signature inlet.
                // Only a separate terrain motor can add downstream loop energy.
                if(ports.loopSupplyIndependent)
                    supply(ports.loopSupplyEnd,ports.loopSupplyTarget,ports.loopBrakeStart,ports.loopEntry,ports.loopEntrySpeedHint,next.loopSupplyEnergyCorrection);
                supply(ports.relaunchEnd,ports.relaunchTarget,ports.reversalBrakeStart,ports.reversalEntry,ports.reversalEntrySpeedHint,next.relaunchEnergyCorrection);
                if(reached(ports.loopBrakeStart))relax(next.loopApproachSpeed,replayValueAt(frames,ports.loopBrakeStart),1.);
                if(correction)energyHistory<<',';
                energyHistory<<"{\"iteration\":"<<correction<<",\"completed\":"<<(completed?"true":"false")<<",\"maximumResidualMps\":";
                if(std::isfinite(energyResidual))energyHistory<<energyResidual;else energyHistory<<"null";
                auto sample=[&](const char* name,double at){energyHistory<<",\""<<name<<"\":";if(reached(at))energyHistory<<replayValueAt(frames,at);else energyHistory<<"null";};
                sample("signatureEntryMps",ports.signatureEntry);sample("signatureExitMps",ports.signatureExit);
                sample("loopApproachMps",ports.loopBrakeStart);sample("loopEntryMps",ports.loopEntry);sample("loopApexMps",ports.loopApex);
                sample("reversalEntryMps",ports.reversalEntry);sample("reversalExitMps",ports.reversalExit);
                energyHistory<<",\"loopSupplyCorrectionM2ps2\":"<<feedback.loopSupplyEnergyCorrection
                    <<",\"nextLoopSupplyCorrectionM2ps2\":"<<next.loopSupplyEnergyCorrection
                    <<",\"relaunchCorrectionM2ps2\":"<<feedback.relaunchEnergyCorrection
                    <<",\"nextRelaunchCorrectionM2ps2\":"<<next.relaunchEnergyCorrection<<'}';
                // A provisional route supplies actual source energy first. Rank once
                // with those converged dimensions, then keep the final route
                // fixed. Both stages share this same eight-rebuild budget.
                const bool reselectRoute=energyResidual<=.5&&!routeReselected;
                if(energyResidual<=.5&&(routeReselected||correction==8)){feedbackConverged=true;break;}
                if(correction==8)break;
                if(reselectRoute){
                    // Re-ranking is optional. Keep the measured converged route
                    // until its replacement earns the same energy tolerance.
                    convergedProvisional=std::move(d);convergedPorts=ports;provisionalResidual=energyResidual;haveConvergedProvisional=true;
                }
                if(!changed&&!reselectRoute)break;
                feedback=next;
                if(reselectRoute){feedback.planShape=-1;feedback.planPlacement=-1;routeReselected=true;routeReselectionIteration=correction+1;}
                ++energyIterations;
                try{d=candidate(req,i,cancel,feedback,ports,preparedSources);}
                catch(const std::runtime_error& error){
                    if(std::string(error.what())=="CANCELLED"||(cancel&&cancel())){
                        Design cancelled;cancelled.request=req;cancelled.candidate=i;cancelled.simulation.cancelled=true;
                        cancelled.report.fail("CANCELLED","Generation cancelled");return cancelled;
                    }
                    if(!haveConvergedProvisional)throw;
                    energyResidual=INFINITY;
                    energyHistory<<",{\"iteration\":"<<correction+1<<",\"constructionError\":\"";
                    for(unsigned char c:std::string(error.what())){
                        if(c=='"'||c=='\\')energyHistory<<'\\'<<c;
                        else if(c<32)energyHistory<<"\\u"<<std::hex<<std::setw(4)<<std::setfill('0')<<int(c)<<std::dec;
                        else energyHistory<<c;
                    }
                    energyHistory<<"\",\"maximumResidualMps\":null}";break;
                }
                if(routeReselected){attemptedShape=ports.planShape;attemptedPlacement=ports.planPlacement;}
                d.simulation=simulate(d.track,d.operations,req.train,req.simulationStep,cancel);
                if(reselectRoute){feedback.planShape=ports.planShape;feedback.planPlacement=ports.planPlacement;}
                if(d.simulation.cancelled){d.report.fail("CANCELLED","Generation cancelled");return d;}
            }
            if(cancel&&cancel()){d.simulation.cancelled=true;d.report.fail("CANCELLED","Generation cancelled");return d;}
            if(!feedbackConverged&&haveConvergedProvisional){
                failedAlternateResidual=energyResidual;d=std::move(convergedProvisional);ports=convergedPorts;
                energyResidual=provisionalResidual;feedbackConverged=true;retainedProvisional=true;
            }
            if(!d.planningDiagnostics.empty()){
                d.planningDiagnostics.pop_back();std::ostringstream energy;energy<<std::setprecision(12)<<",\"authoringEnergy\":{\"corrections\":"<<energyIterations<<",\"converged\":"<<(feedbackConverged?"true":"false")<<",\"maximumSpeedResidualMps\":";
                if(std::isfinite(energyResidual))energy<<energyResidual;else energy<<"null";
                energy<<",\"routeSelection\":{\"reselected\":"<<(routeReselected?"true":"false")<<",\"iteration\":"<<routeReselectionIteration<<",\"provisionalShape\":"<<provisionalShape<<",\"provisionalPlacement\":"<<provisionalPlacement<<",\"finalShape\":"<<ports.planShape<<",\"finalPlacement\":"<<ports.planPlacement<<",\"attemptedShape\":"<<attemptedShape<<",\"attemptedPlacement\":"<<attemptedPlacement<<",\"retainedConvergedProvisional\":"<<(retainedProvisional?"true":"false")<<",\"failedAlternateResidualMps\":";
                if(retainedProvisional&&std::isfinite(failedAlternateResidual))energy<<failedAlternateResidual;else energy<<"null";
                energy<<"},\"toleranceMps\":0.5,\"loopApexTargetMps\":"<<ports.loopSpeedHint<<",\"reversalExitTargetMps\":"<<ports.reversalSpeedHint<<",\"iterations\":["<<energyHistory.str()<<"]}}";d.planningDiagnostics+=energy.str();
            }
            if(d.simulation.completed){improveBanking(d);d.simulation=simulate(d.track,d.operations,req.train,req.simulationStep,cancel);}
            d.station=buildStation(d.track,req.terrain,req.train,cancel);
            placeSupports(d,cancel);
            d.inversionDimensions=measureInversionDimensions(d.track,cancel);
            if(progress)progress(i,"Checking measured targets and clearance");
            d.report=validateGeometry(d.track,req.terrain,req.limits,req.train,d.supports,cancel);
            auto structures=validateDesignStructures(d,cancel);d.report.errors.insert(d.report.errors.end(),structures.errors.begin(),structures.errors.end());
            evaluateTargets(d);
            if(d.simulation.completed){
                const double entry=replayValueAt(d.simulation.frames,ports.signatureEntry),exit=replayValueAt(d.simulation.frames,ports.signatureExit);
                const double entryResidual=std::abs(entry-ports.signatureEntrySpeedHint),exitResidual=std::abs(exit-ports.signatureExitSpeedHint);
                if(std::max(entryResidual,exitResidual)>.5)d.report.fail("AUTHORING_SIGNATURE",
                    "Actual finite-train signature inlet/exit differs from its force-source energy intent",ports.signatureEntry,std::max(entryResidual,exitResidual),.5);
                d.planningDiagnostics.pop_back();std::ostringstream intent;intent<<std::setprecision(12)
                    <<",\"signatureIntent\":{\"sourceInletMps\":"<<ports.signatureEntrySpeedHint<<",\"actualInletMps\":"<<entry
                    <<",\"sourceExitMps\":"<<ports.signatureExitSpeedHint<<",\"actualExitMps\":"<<exit
                    <<",\"inletResidualMps\":"<<entryResidual<<",\"exitResidualMps\":"<<exitResidual
                    <<",\"toleranceMps\":0.5,\"minimumSourceTraversalMps\":25,\"preparedSourceReused\":true},\"airtimePhaseIntent\":[";
                bool firstPoint=true;
                for(size_t chain=0;chain<ports.airtimeProfile.size();++chain){const auto& profile=ports.airtimeProfile[chain];
                    const double beginTime=replayValueAt(d.simulation.frames,profile.front().distance,true);
                    for(size_t point=0;point<profile.size();++point){const auto& check=profile[point];
                        if(!firstPoint)intent<<',';firstPoint=false;
                        const double actualSpeed=replayValueAt(d.simulation.frames,check.distance);
                        intent<<"{\"chain\":"<<chain<<",\"phase\":\""<<(point==0?"entry":point+1==profile.size()?"exit":point%2?"apex":"valley")
                            <<"\",\"hill\":"<<(point+1==profile.size()?int((profile.size()-3)/2):point?int((point-1)/2):0)<<",\"distance\":"<<check.distance
                            <<",\"sourceSeconds\":"<<check.time<<",\"actualSeconds\":"<<replayValueAt(d.simulation.frames,check.distance,true)-beginTime
                            <<",\"sourceSpeedMps\":"<<check.speed<<",\"actualSpeedMps\":"<<actualSpeed<<",\"speedResidualMps\":"<<actualSpeed-check.speed
                            <<",\"sourceCenterlineNormalG\":"<<check.normalG<<",\"sourceCenterlineLateralG\":"<<check.lateralG<<",\"ridersAtPhase\":[";
                        for(int seat=0;seat<3;++seat){if(seat)intent<<',';
                            const double center=check.distance-seatDistanceOffset(req.train,seat);
                            auto right=std::lower_bound(d.simulation.frames.begin(),d.simulation.frames.end(),center,[](const Frame& frame,double distance){return frame.distance<distance;});
                            const auto& left=*(right-1);const double u=(center-left.distance)/(right->distance-left.distance);
                            const auto& a=left.seats[seat];const auto& b=right->seats[seat];
                            intent<<"{\"time\":"<<left.time+u*(right->time-left.time)<<",\"speedMps\":"<<left.speed+u*(right->speed-left.speed)
                                <<",\"Gzyx\":["<<a.vertical+u*(b.vertical-a.vertical)<<','<<a.lateral+u*(b.lateral-a.lateral)<<','<<a.longitudinal+u*(b.longitudinal-a.longitudinal)<<"]}";}
                        intent<<"]}";
                    }
                }
                intent<<"]}";
                d.planningDiagnostics+=intent.str();
            }
            if(d.simulation.completed&&!feedbackConverged)d.report.fail("AUTHORING_ENERGY","Joined ride did not converge to its element entry/apex energy intent",0,energyResidual,.5);
            if(d.report.valid()&&d.simulation.completed&&d.simulation.report.valid()&&!d.simulation.cancelled){
                if(progress)progress(i,"Checking independent half-step simulation");
                verifyConvergence(d,cancel);
                if(d.simulation.cancelled)return d;
            }
            remember(&d,i);if(d.accepted()){
                double duration=movingRideSeconds(d);
                if(duration<=180)return withHistory(std::move(d),"accepted-within-moving-duration-goal");
                if(duration<bestDuration){bestDuration=duration;bestPacing=std::move(d);haveBestPacing=true;}
                continue;
            }
            bool intensityOnly=d.simulation.completed&&d.simulation.report.valid()&&d.report.errors.size()==1&&d.report.errors.front().code=="INTENSITY_TARGET";
            if(intensityOnly&&(!haveBestIntensity||d.simulation.metrics.exposure10Seconds>bestIntensity.simulation.metrics.exposure10Seconds)){bestIntensity=d;haveBestIntensity=true;}last=std::move(d);
            // Missing reference data cannot be repaired by searching other seeds.
            bool onlyMissing=last.simulation.completed&&last.simulation.report.valid()&&last.report.errors.size()==1&&last.report.errors.front().code=="REFERENCE_UNAVAILABLE";
            if(onlyMissing)return withHistory(std::move(last),"reference-unavailable");
        }catch(const SourceFamilyUnsupported& e){
            remember(nullptr,i,"SOURCE_FAMILY");last.report.fail("SOURCE_FAMILY",e.what());
            return withHistory(std::move(last),"unsupported-source-family");
        }catch(const std::exception& e){
            if(std::string(e.what())=="CANCELLED"||(cancel&&cancel())){last.simulation.cancelled=true;last.report.fail("CANCELLED","Generation cancelled");return last;}
            if(d.track.spans.empty()){remember(nullptr,i);last.report.fail("CANDIDATE_FAILURE",e.what());}
            else{
                // Preserve the last actual geometry/trace when later refinement
                // or structure placement fails. It remains explicitly rejected.
                d.report.fail("CANDIDATE_FAILURE",std::string("Refinement/validation failed; showing the last constructed circuit: ")+e.what());
                remember(&d,i);last=std::move(d);
            }
            if(progress)progress(i,std::string("Candidate rejected: ")+e.what());
        }
    }if(haveBestPacing)return withHistory(std::move(bestPacing),"best-accepted-moving-duration-shortfall");if(haveBestIntensity)return withHistory(std::move(bestIntensity),"best-physically-valid-intensity-shortfall");return withHistory(std::move(last),"last-constructed-rejection");
}
}
