#include "../src/itinerary_generation.hpp"
#include "coaster/layout_modules.hpp"
#include <iostream>
#include <stdexcept>
using namespace coaster;
namespace {
int checks=0;
void check(bool value,const char* message){++checks;if(!value)throw std::runtime_error(message);}
detail::CircuitElement element(Vec3 exit,double heading=0){
    const double speed=55,bank=std::acos(1/3.5);
    return {exit,heading,norm(exit),speed*speed/(gravity*std::tan(bank)),90,bank};
}
void verify(const std::vector<detail::CircuitElement>& elements,const detail::CircuitLayout& layout){
    check(std::isfinite(layout.length),"Circuit is feasible");Vec3 cursor{};double length=0;
    for(size_t i=0;i<elements.size();++i){
        const auto rotate=[&](Vec3 p,double h){return Vec3{p.x*std::cos(h)-p.y*std::sin(h),p.x*std::sin(h)+p.y*std::cos(h),p.z};};
        const auto& source=elements[i];const auto& turn=layout.turns[i];
        const double heading=layout.headings[i]+source.exitHeading+turn.angle;
        cursor=cursor+rotate(source.exit,layout.headings[i])+rotate(turn.points.back(),layout.headings[i]+source.exitHeading)+rotate({layout.straights[i],0,0},heading);
        check(std::abs(std::remainder(heading-layout.headings[(i+1)%elements.size()],2*pi))<1e-10,"Real source and turn headings join");
        check(layout.straights[i]>=0,"No negative physical rail");
        length+=source.length+turn.length+layout.straights[i];
    }
    check(std::hypot(cursor.x,cursor.y)<1e-6,"Independent composition closes station XY");
    check(std::abs(length-layout.length)<1e-7,"Cost includes all source, turn and work rail");
}
void verifySourceEnergy(const GenerationRequest& request,const detail::RideRoute& routed){
    for(size_t i=0;i<routed.order.size();++i){const auto& before=routed.sources[routed.order[i]];const auto& next=routed.sources[routed.order[(i+1)%routed.order.size()]];
        if(!next.airtime())continue;
        const double length=routed.layout.turns[i].length+routed.layout.straights[i];
        const double rise=detail::rideInletHeight(request,next)-detail::rideInletHeight(request,before)-before.geometry.points.back().frame.position.z;
        const double drag=.5*request.train.airDensity*request.train.dragCdA/(request.train.cars*request.train.carMass);
        const auto derivative=[&](double energy){return -2*gravity*(request.train.rollingResistance+rise/length)-2*drag*energy;};
        // Independent RK4 transport over the final closed lengths. The former
        // pre-closure speed allocation can be several m/s above this supply.
        double energy=before.exitSpeed*before.exitSpeed;const int steps=int(std::ceil(length));const double ds=length/steps;
        for(int step=0;step<steps;++step){const double a=derivative(energy),b=derivative(energy+ds*a*.5),c=derivative(energy+ds*b*.5),d=derivative(energy+ds*c);energy+=ds*(a+2*b+2*c+d)/6;}
        check(energy>0&&std::abs(std::sqrt(energy)-next.entrySpeed)<=.500001,"Passive source intent agrees with independent transport over the actual closed route");
    }
}
}
int main(){try{
    for(double angle:{.5,pi/2,pi}){
        const double bank=std::acos(1/4.1);detail::CircuitElement port{{},0,0,80*80/(gravity*std::tan(bank)),140,bank,60};
        const auto turn=detail::circuitTurn(port,angle);std::array<std::vector<double>,3> axes;
        for(double at=0;at<=turn.length;at+=60./960){axes[0].push_back(1/std::cos(turn.bankAt(at)));axes[1].push_back(0);axes[2].push_back(0);}
        const auto assessment=assessForceEnvelope(axes,1./960);
        check(assessment.performed&&!assessment.cancelled&&assessment.report.valid(),"Different heading changes retain the unchanged independent signed force-duration acceptance");
    }
    GenerationRequest request;request.seed=5;detail::RideFeedback feedback;
    Operation launch{0,1,DriveKind::Launch,60,request.train.carMass*38.9,request.train.carMass*3890,.16};launch.exitFadeMeters=9.6;
    const auto ride=detail::buildRideSources(request,feedback,launch);
    {
        EnergyLoopModuleRequest specification;specification.height=72;specification.apexSpeed=26;specification.normalG=4.1;
        const auto loop=buildEnergyLoopModule(specification);check(loop.assessment.passed,"Independent lossless loop source exists");
        for(int cars:{6,12}){TrainConfig train;train.cars=cars;train.dragCdA=0;train.rollingResistance=0;
            const auto source=detail::forceRideSource(detail::RideRole::Loop,loop.track,loop.samples,{},
                {loop.samples.front(),loop.apex,loop.samples.back()},train);
            const auto mean=[&](double at){double height=0;for(int car=0;car<cars;++car)height+=loop.track.sample(at+((cars-1)*.5-car)*train.spacing).position.z;return height/cars;};
            const double expected=std::sqrt(source.entrySpeed*source.entrySpeed-2*gravity*(mean(loop.apex.distance)-mean(0)));
            check(std::abs(source.phases[1].speed-expected)<1e-5,"Source phase reference follows independent finite-train mean-potential energy rather than point-mass apex speed");
        }
    }
    check(ride.size()==8,"Requested physical sources are present");
    for(const auto& source:ride){check(source.geometry.points.size()>4,"Each source has canonical geometry");
        check(source.exitSpeed>0,"Each source retains positive exit energy");}
    const auto routed=detail::routeRide(request,ride,feedback);
    check(std::isfinite(routed.layout.length),"The physical itinerary closes");
    verifySourceEnergy(request,routed);
    {
        // This order exposed source targets chosen before closure changed
        // the two passive lengths. Keep the same independent energy check.
        auto regression=request;regression.seed=11;detail::RideFeedback initial;
        const auto sources=detail::buildRideSources(regression,initial,launch);
        verifySourceEnergy(regression,detail::routeRide(regression,sources,initial));
    }
    for(size_t id=0;id<routed.sources.size();++id)
        check(feedback.turnSpeed[id]==routed.sources[id].exitSpeed&&feedback.linkSpeed[id]==routed.sources[id].exitSpeed,"Initial terrain and route constraints use the selected source's same outgoing energy");
    std::cout<<"Itinerary";for(auto id:routed.order)std::cout<<' '<<detail::rideName(routed.sources[id].role);
    std::cout<<" length "<<routed.layout.length<<" work "<<routed.work<<'\n';
    auto placed=detail::placeRide(request,routed,feedback);
    check(!placed.track.spans.empty(),"The itinerary admits a shared terrain solution");
    for(size_t occurrence=0;occurrence<routed.order.size();++occurrence){const auto& source=routed.sources[routed.order[occurrence]];
        if(source.role!=detail::RideRole::Climb&&source.role!=detail::RideRole::Dive)continue;
        const auto interval=placed.sources[occurrence];const auto& first=placed.track.knots[interval.first];
        const double heading=std::atan2(first.tangent.y,first.tangent.x);bool rigid=interval.last-interval.first+1==source.geometry.points.size();
        for(size_t j=0;j<source.geometry.points.size();++j){const auto& original=source.geometry.points[j].frame;const auto& actual=placed.track.knots[interval.first+j];
            rigid&=norm(actual.position-first.position-detail::sourceYaw(original.position,heading))<1e-8&&
                norm(actual.tangent-detail::sourceYaw(original.tangent,heading))<1e-10&&norm(actual.curvature-detail::sourceYaw(original.curvature,heading))<1e-10;
        }
        check(rigid,"Actual itinerary placement preserves the complete certified climb/dive shape and active pulse, including derivatives");
        check(std::abs(placed.track.knots[interval.last].position.z-first.position.z-(source.role==detail::RideRole::Climb?request.targets.height:-request.targets.height))<1e-8,
            "The shared terrain solve cannot alter the actual climb or dive's selected rise");
    }
    const auto work=[](size_t i,double){return 60.+15*i;};
    for(size_t count:{size_t(3),size_t(5),size_t(8)})for(int hand:{-1,1}){
        std::vector<detail::CircuitElement> elements;std::vector<double> headings;
        for(size_t i=0;i<count;++i){elements.push_back(element({180.+75*i,i%2?40.:-25.,0},i==1?hand*2.8:0));headings.push_back(hand*2*pi*i/count);}
        const auto initial=detail::closeCircuit(elements,headings,work),solved=detail::solveCircuitLayout(elements,headings,work);
        verify(elements,solved);check(solved.length<=initial.length,"Bounded layout solve cannot add unnecessary total rail");
        for(size_t i=0;i<count;++i)check(solved.straights[i]>=work(i,solved.turns[i].length),"Closure preserves complete physical work domains");
        auto translatedHeading=headings;for(auto& angle:translatedHeading)angle+=.71;
        const auto rotated=detail::solveCircuitLayout(elements,translatedHeading,work);verify(elements,rotated);
        check(std::abs(rotated.length-solved.length)<1e-6,"World orientation cannot change a terrain-free length solution");
    }
    const auto zero=[](size_t,double){return 0.;};
    const auto rankOne=detail::closeCircuit({element({-100,0,0})},{0},zero);verify({element({-100,0,0})},rankOne);
    check(rankOne.straights[0]==100,"Rank-one closure has an exact single-straight solution");
    check(!std::isfinite(detail::closeCircuit({element({100,0,0})},{0},zero).length),"Negative rail demand is explicitly infeasible");
    Track departure;departure.closed=false;
    for(int i=0;i<=4;++i)departure.knots.push_back({{50.*i,0,0},{1,0,0},{},{0,0,1},0,Element::Station});departure.rebuild();
    const detail::SourceGeometry landmarkPartition(departure,{}, {200.*37/std::ceil(200./1.5)+1e-5});
    for(size_t i=1;i<landmarkPartition.points.size();++i)
        check(landmarkPartition.points[i].distance-landmarkPartition.points[i-1].distance>.5,
            "A physical phase near an unrelated grid cannot create almost coincident source knots");
    const auto loop=buildEnergyLoopModule(EnergyLoopModuleRequest{});
    const auto reversal=designFvdImmelmann(FvdImmelmannRequest{});
    check(loop.report.valid()&&reversal.section.report.valid(),"Independent sources for whole composition are valid");
    const std::vector<detail::CircuitOccurrence> occurrences{
        {detail::SourceGeometry(departure),0},
        {detail::SourceGeometry(loop.track),12},
        {detail::SourceGeometry(reversal.section.track,Element::Inversion),4}};
    std::vector<detail::CircuitElement> ports;
    for(const auto& occurrence:occurrences){const auto& point=occurrence.source.points.back();
        auto port=element(point.frame.position,std::atan2(point.frame.tangent.y,point.frame.tangent.x));port.length=point.distance;ports.push_back(port);}
    const auto solved=detail::solveCircuitLayout(ports,{0,2*pi/3,4*pi/3},work);
    auto shortCoast=solved;for(size_t i=0;i<shortCoast.straights.size();++i)shortCoast.workLengths[i]=shortCoast.straights[i]-1e-4;
    const auto sharedJoin=detail::composeCircuit(occurrences,shortCoast);
    for(const auto& link:sharedJoin.links)for(size_t i=link.turn.first;i<link.straight.last;++i)
        check(norm(sharedJoin.track.knots[i+1].position-sharedJoin.track.knots[i].position)>.5,
            "A short passive domain shares its adjoining turn sample without creating a sub-millimetre span");
    auto nearGrid=solved;
    for(size_t i=0;i<nearGrid.straights.size();++i){const int count=int(std::ceil(nearGrid.straights[i]/1.5));
        nearGrid.workLengths[i]=nearGrid.straights[i]*(count/3)/count+1e-5;}
    const auto partitioned=detail::composeCircuit(occurrences,nearGrid);
    for(const auto& link:partitioned.links){double minimum=INFINITY;
        for(size_t i=link.straight.first;i<link.straight.last;++i)minimum=std::min(minimum,norm(partitioned.track.knots[i+1].position-partitioned.track.knots[i].position));
        check(minimum>.5,"A work boundary near an unrelated sample grid retains well-spaced physical-domain knots");
        for(size_t i=link.workFirst;i<=link.straight.last;++i)check(std::abs(partitioned.track.knots[i].tangent.z)<1e-12&&norm(partitioned.track.knots[i].curvature)<1e-10,
            "Level work rail retains its analytic tangent and curvature at every authored boundary");
    }
    const auto composed=detail::composeCircuit(occurrences,solved,{432,-183,20},.71);
    check(composed.sources.size()==occurrences.size()&&composed.links.size()==occurrences.size(),"All source and connecting intervals have one owner");
    for(size_t i=0;i<composed.links.size();++i){const auto& link=composed.links[i];
        const auto a=composed.track.knots[link.workFirst].position,b=composed.track.knots[link.straight.last].position;
        check(std::abs(std::hypot(b.x-a.x,b.y-a.y)-solved.workLengths[i])<1e-7,
            "Closure-added rail cannot lengthen the independently sized work domain");
        for(size_t point=link.workFirst;point<=link.straight.last;++point)
            check(std::abs(composed.track.knots[point].position.z-a.z)<1e-8,"Authored source-height recovery finishes before the work domain");
    }
    for(size_t i=0;i<occurrences.size();++i){
        const auto& interval=composed.sources[i];const auto& first=composed.track.knots[interval.first];
        const double heading=.71+solved.headings[i];const Vec3 forward{std::cos(heading),std::sin(heading),0},left{-std::sin(heading),std::cos(heading),0};
        const auto rotate=[&](Vec3 p){return forward*p.x+left*p.y+Vec3{0,0,p.z};};
        const auto& original=occurrences[i].source;
        check(interval.last-interval.first+1==original.points.size(),"Source control count survives whole-circuit composition");
        for(size_t j=0;j<original.points.size();++j){const auto& expected=original.points[j].frame;const auto& actual=composed.track.knots[interval.first+j];
            check(norm(actual.position-first.position-rotate(expected.position))<1e-8,"Composed source is rigid at every control");
            check(norm(actual.curvature-rotate(expected.curvature))<1e-10,"Composed source retains authored curvature");
            check(norm(actual.up-rotate(expected.up))<1e-10,"Composed source retains authored physical orientation");
        }
    }
    for(size_t i=1;i<composed.track.spans.size();++i){
        const auto a=sampleSpanKinematics(composed.track,i-1,1),b=sampleSpanKinematics(composed.track,i,0);
        check(norm(a.sample.position-b.sample.position)<1e-8&&norm(a.sample.tangent-b.sample.tangent)<1e-9&&norm(a.sample.curvature-b.sample.curvature)<1e-8&&norm(a.curvatureS-b.curvatureS)<1e-7,"Whole composition retains canonical C3 joins");
    }
    bool cancelled=false;try{detail::solveCircuitLayout({element({100,0,0}),element({100,0,0})},{0,pi},zero,[]{return true;});}
    catch(const std::runtime_error& e){cancelled=std::string(e.what())=="CANCELLED";}check(cancelled,"Layout cancellation propagates");
    std::cout<<"PASS "<<checks<<" circuit closure checks\n";return 0;
}catch(const std::exception& e){std::cerr<<"FAIL after "<<checks<<": "<<e.what()<<'\n';return 1;}}
