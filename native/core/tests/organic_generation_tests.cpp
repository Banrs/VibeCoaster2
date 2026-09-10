#include "coaster/coaster.hpp"
#include <iostream>
#include <stdexcept>
#include <tuple>
using namespace coaster;
namespace {
int checks=0;
void check(bool value,const char* message){++checks;if(!value)throw std::runtime_error(message);}
struct ObservedInversions {int fullLoop{},immelmann{},diveLoop{};double immelmannRise{},immelmannForwardExtent{};};
ObservedInversions classifyGeometry(const Design& design){
    ObservedInversions observations;
    // Type is inferred from canonical tangent, up and elevation, independently
    // of generator module names or planning diagnostics. The label merely
    // supplies each measured span group's interval.
    for(const auto& region:design.inversionDimensions){
        auto start=design.track.sample(region.startDistance),end=design.track.sample(region.endDistance);
        Vec3 forward=unit(Vec3{start.tangent.x,start.tangent.y,0});
        Vec3 exitForward=unit(Vec3{end.tangent.x,end.tangent.y,0});
        double headingAgreement=dot(forward,exitForward),rise=end.position.z-start.position.z;
        constexpr double portAngle=15*pi/180,phaseTolerance=2*portAngle,headingCosine=.984807753012208;
        for(const auto& port:{start,end}){
            Vec3 upright=unit(Vec3{0,0,1}-port.tangent*port.tangent.z);
            check(std::abs(port.tangent.z)<std::sin(portAngle)&&dot(port.up,upright)>std::cos(portAngle),"Flowing inversion ports remain within 15 degrees of upright and level");
        }
        double pitch=std::atan2(start.tangent.z,dot(start.tangent,forward)),initialPitch=pitch,roll=0,initialRoll=0;
        double firstAscendingVertical=INFINITY,firstDescendingVertical=INFINITY,ascendingRoll=0,descendingRoll=0;
        double invertedApex=-INFINITY,returnUprightSlope=INFINITY;bool seenInverted=false;const int samples=int(std::ceil(region.pathLength/.25));
        for(int i=0;i<=samples;++i){
            double distance=region.startDistance+region.pathLength*i/samples;auto sample=design.track.sample(distance);
            check(finite(sample.position)&&finite(sample.tangent)&&finite(sample.up),"Generated canonical inversion has finite frames");
            double projected=dot(sample.tangent,forward);
            check(std::hypot(projected,sample.tangent.z)>.5,"Pitch topology retains a resolved vertical-plane projection");
            pitch+=std::remainder(std::atan2(sample.tangent.z,projected)-pitch,2*pi);
            Vec3 pitchUp=Vec3{0,0,1}*std::cos(pitch)-forward*std::sin(pitch);
            pitchUp=unit(pitchUp-sample.tangent*dot(pitchUp,sample.tangent));
            double angle=std::atan2(dot(sample.up,cross(sample.tangent,pitchUp)),dot(sample.up,pitchUp));
            roll+=std::remainder(angle-roll,2*pi);if(i==0)initialRoll=roll;
            if(sample.tangent.z>std::cos(portAngle)&&!std::isfinite(firstAscendingVertical)){firstAscendingVertical=distance;ascendingRoll=roll;}
            if(sample.tangent.z< -std::cos(portAngle)&&!std::isfinite(firstDescendingVertical)){firstDescendingVertical=distance;descendingRoll=roll;}
            if(sample.up.z<-.5)seenInverted=true;
            if(seenInverted&&sample.up.z>.5&&!std::isfinite(returnUprightSlope))returnUprightSlope=sample.tangent.z;
            if(sample.up.z<-.5)invertedApex=std::max(invertedApex,sample.position.z-design.request.terrain.height(sample.position.x,sample.position.y));
        }
        double pitchSweep=pitch-initialPitch,rollSweep=roll-initialRoll;
        // A full pitch revolution survives borrowed rising ports. A barrel
        // roll has no pitch revolution; a horizontal hairpin lacks a vertical
        // landmark. Neither can substitute for these actual inversion shapes.
        if(headingAgreement>headingCosine&&std::abs(pitchSweep-2*pi)<phaseTolerance){
            check(std::isfinite(firstAscendingVertical)&&std::isfinite(firstDescendingVertical)&&firstAscendingVertical<firstDescendingVertical,"Retained full loop contains ascent and descent pitch in its original heading");
            check(std::abs(rollSweep)<phaseTolerance,"Full loop is a pitch revolution without an added barrel roll");
            check(region.verticalExtent>=54&&region.verticalExtent<=70,"Ordinary loop keeps its approximately55-68m physical size instead of borrowing the inversion record target");
            double minimumSpeed=INFINITY;
            for(const auto& frame:design.simulation.frames)if(frame.distance>=region.startDistance&&frame.distance<=region.endDistance)minimumSpeed=std::min(minimumSpeed,frame.speed);
            check(minimumSpeed>17,"Fixture loop retains moving crest energy rather than approaching stall");
            ++observations.fullLoop;
        }else if(headingAgreement< -headingCosine&&region.verticalExtent>70&&rise>=0&&std::abs(pitchSweep-pi)<phaseTolerance){
            check(std::isfinite(firstAscendingVertical)&&!std::isfinite(firstDescendingVertical),"Immelmann contains an actual ascending half-loop");
            check(std::abs(std::abs(rollSweep)-pi)<phaseTolerance&&std::abs(ascendingRoll-initialRoll)<pi/3,"Compound Immelmann reaches ascending vertical before completing most of its half-roll");
            check(invertedApex>=design.request.targets.inversionHeight,"High Immelmann retains the selected terrain-relative inverted-apex target");
            double peak=-INFINITY;
            for(const auto& frame:design.simulation.frames)for(int seat=0;seat<3;++seat){double at=frame.distance+seatDistanceOffset(design.request.train,seat);
                if(at>=region.startDistance&&at<=region.endDistance)peak=std::max(peak,frame.seats[seat].vertical);}
            check(peak<4.4,"Fixture Immelmann has the intended ordinary-force passage; this is not a universal safety cap");
            check(returnUprightSlope<-.1,"Immelmann rolls upright on its curved descent instead of a level rolling tail");
            check(region.verticalMaximum-end.position.z>40,"Signature returns toward a lower valley before the ordinary turnaround");
            observations.immelmannRise=region.verticalExtent;observations.immelmannForwardExtent=region.forwardExtent;++observations.immelmann;
        }else if(headingAgreement< -headingCosine&&rise< -70&&std::abs(pitchSweep+pi)<phaseTolerance){
            check(std::isfinite(firstDescendingVertical)&&!std::isfinite(firstAscendingVertical),"Dive loop contains an actual descending half-loop");
            check(std::abs(std::abs(rollSweep)-pi)<phaseTolerance&&std::abs(descendingRoll-initialRoll)>2*pi/3,"Compound dive loop completes most of its half-roll before descending vertical");
            ++observations.diveLoop;
        }else check(false,"Every inversion group in the descending-reversal fixture has a geometrically recognized complete topology");
        check(std::isfinite(invertedApex),"Each classified inversion contains an actual inverted passage");
    }
    check(observations.fullLoop==1&&observations.immelmann==1&&observations.diveLoop==0,"Fixture retains the full loop and descending Immelmann without a mandatory mirrored dive to undo its raised exit");
    return observations;
}
void checkFoldedGeometry(const Design& design){
    struct Segment{double s;TrackSample sample;};
    // Widely separated, transverse canonical branches are required. Local
    // vertical-loop projections and almost parallel reversal overlaps do not
    // count as a folded spatial crossing.
    constexpr double step=5;std::vector<Segment> points;
    for(double s=0;s<design.track.length;s+=step)points.push_back({s,design.track.sample(s)});
    int crossings=0;double minimumSeparation=INFINITY,gradeFlat=0,straightFlat=0;
    for(size_t i=0;i+1<points.size();++i){const auto& a=points[i].sample;double width=std::min(step,design.track.length-points[i].s);
        if(std::abs(a.tangent.z)<.015){gradeFlat+=width;if(norm(a.curvature)<1e-4)straightFlat+=width;}
        Vec3 deltaA=points[i+1].sample.position-a.position;
        for(size_t j=i+1;j+1<points.size();++j){double arc=points[j].s-points[i].s;if(std::min(arc,design.track.length-arc)<1000)continue;
            Vec3 deltaB=points[j+1].sample.position-points[j].sample.position;double determinant=cross(deltaA,deltaB).z;
            if(std::abs(determinant)<.3*norm(deltaA)*norm(deltaB))continue;
            Vec3 between=points[j].sample.position-a.position;double u=cross(between,deltaB).z/determinant,v=cross(between,deltaA).z/determinant;
            if(u<0||u>=1||v<0||v>=1)continue;
            double first=points[i].s+step*u,second=points[j].s+step*v;
            for(int iteration=0;iteration<8;++iteration){auto x=design.track.sample(first),y=design.track.sample(second);Vec3 error=y.position-x.position;double jacobian=cross(x.tangent,y.tangent).z;
                first+=cross(error,y.tangent).z/jacobian;second+=cross(error,x.tangent).z/jacobian;}
            auto x=design.track.sample(first),y=design.track.sample(second);
            check(std::hypot(x.position.x-y.position.x,x.position.y-y.position.y)<1e-5,"Projected crossover refines on the actual canonical curves");
            double separation=std::abs(x.position.z-y.position.z);
            minimumSeparation=std::min(minimumSeparation,separation);++crossings;
        }
    }
    check(crossings>=1,"Complete circuit has a transverse crossover between widely separated route branches");
    check(straightFlat/design.track.length<.35,"Straight and grade-flat geometry occupies less than35percent of the complete circuit");
    int airtimePeaks=0;bool rising=false;
    for(size_t i=0;i<points.size();++i){const auto& sample=points[i].sample;if(sample.element!=Element::Airtime){rising=false;continue;}
        if(sample.tangent.z>.03)rising=true;if(rising&&sample.tangent.z<-.03){++airtimePeaks;rising=false;}}
    check(airtimePeaks>=4,"Complete circuit contains at least four actual ascending-descending force-authored crest shapes");
    std::cout<<"crossings="<<crossings<<" minimumCanonicalSeparation="<<minimumSeparation<<" straightFlatShare="<<straightFlat/design.track.length<<" gradeFlatShare="<<gradeFlat/design.track.length<<" airtimePeaks="<<airtimePeaks<<'\n';
}
double fieldNumber(const std::string& text,size_t begin,size_t end,const std::string& key){
    size_t field=text.find(key,begin);check(field!=std::string::npos&&field<end,"Expected diagnostic field belongs to its object");
    return std::stod(text.substr(field+key.size()));
}
std::pair<double,double> moduleInterval(const Design& design,const std::string& name){
    const auto& text=design.planningDiagnostics;size_t begin=text.find("\"identity\":\""+name+"\"");
    check(begin!=std::string::npos,"Intended operation module has a canonical interval");size_t end=text.find('}',begin);
    return {fieldNumber(text,begin,end,"\"start\":"),fieldNumber(text,begin,end,"\"end\":")};
}
double measuredSpeed(const Design& design,double at){
    const auto& frames=design.simulation.frames;
    auto right=std::lower_bound(frames.begin(),frames.end(),at,[](const Frame& f,double distance){return f.distance<distance;});
    check(right!=frames.begin()&&right!=frames.end(),"Element boundary is covered by independent replay frames");
    const auto& left=*(right-1);double u=(at-left.distance)/(right->distance-left.distance);
    return left.speed+u*(right->speed-left.speed);
}
void checkLocalTurnLoads(const Design& design){
    // This fixture previously spent its last moving turn near 1.4 g because
    // its radius was sized at 65 m/s despite a much lower actual entry speed.
    // Measure the resulting ride, not an implementation formula or new gate.
    const auto& text=design.planningDiagnostics;size_t at=0;bool found=false;
    while((at=text.find("\"identity\":\"banked-camelback-turn\"",at))!=std::string::npos){
        size_t end=text.find('}',at);
        if(fieldNumber(text,at,end,"\"corridor\":")==2){
            const double beginDistance=fieldNumber(text,at,end,"\"start\":"),endDistance=fieldNumber(text,at,end,"\"end\":");
            std::vector<double> middleLoads;
            for(const auto& frame:design.simulation.frames)if(frame.distance>=beginDistance&&frame.distance<=endDistance)middleLoads.push_back(frame.seats[1].vertical);
            check(!middleLoads.empty(),"Late ordinary turn is covered by actual finite-train telemetry");
            std::sort(middleLoads.begin(),middleLoads.end());
            check(middleLoads[middleLoads.size()/2]>2.,"Flat42 ordinary turn sustains useful load at its own speed; this is a fixture regression, not an acceptance floor");
            check(endDistance-beginDistance<350,"Flat42 ordinary turn does not retain the former roughly500m high-speed footprint");found=true;
        }
        at=end+1;
    }
    check(found,"Fixture includes the final moving ordinary turn before its tail launch");
}
void checkSupportSpacing(const Design& design){
    check(!design.supports.empty()&&design.supports.front().trackDistance==0,"Support fallback retains the anchored first attachment");
    for(size_t i=1;i<design.supports.size();++i){
        const auto& previous=design.supports[i-1];const auto& current=design.supports[i];
        auto p=design.track.sample(previous.trackDistance);Vec3 contact=p.position-p.up*(spineDepth+spineRadius);
        const double minimum=contact.z-design.request.terrain.height(contact.x,contact.y)>90?32.:24.;
        const double gap=current.trackDistance-previous.trackDistance;
        check(gap>=minimum-1e-8&&gap<=40+1e-8,"Adaptive attachment placement preserves actual support-span bounds");
    }
    check(design.track.length-design.supports.back().trackDistance<=40+1e-8,"Closing support span stays within the same maximum");
    // generateChecked already requires every-member, station and continuous
    // train/hardware clearance validation; spacing is checked independently here.
}
void checkOperationIntent(const Design& design){
    const auto launch=moduleInterval(design,"post-loop-launch");
    const auto nextBrake=moduleInterval(design,"immelmann-inward-entry-brake");
    check(launch.first>design.inversionDimensions.front().endDistance&&nextBrake.first>launch.second,"Explicit relaunch lies after full loop and before the passive run");
    int motors=0;
    for(const auto& op:design.operations){
        if(op.kind!=DriveKind::Boost&&op.kind!=DriveKind::Launch)continue;
        if(op.start<launch.second-1e-6&&op.end>launch.first+1e-6){++motors;
            check(op.kind==DriveKind::Boost&&std::abs(op.start-launch.first)<1e-6&&std::abs(op.end-launch.second)<1e-6,"One bounded relaunch supplies this section without spilling into coasting track");
            const double capacity=op.maxForce/design.request.train.carMass;
            check(std::isfinite(capacity)&&capacity>=4.5-1e-10&&capacity<=design.request.limits.maxLongitudinalG*gravity&&
                std::isfinite(op.maxPower)&&op.maxPower>0&&std::isfinite(op.targetSpeed)&&op.targetSpeed>0&&op.targetSpeed<=100&&op.rampSeconds>0&&op.exitFadeMeters>0,
                "Energy-sized relaunch retains bounded force, power, target speed and real entry/exit fades");
        }
        check(op.end<=launch.second+1e-6||op.start>=nextBrake.first-1e-6,"First airtime, recovery and following turn coast without hidden motors");
    }
    check(motors==1,"Exactly one positive drive occupies the post-loop relaunch");
    auto begin=design.track.sample(launch.first),end=design.track.sample(launch.second);
    const double horizontalSpan=std::hypot(end.position.x-begin.position.x,end.position.y-begin.position.y);
    check(horizontalSpan>390&&horizontalSpan<420,"Relaunch retains its approximately400m physical corridor, including gentle lateral offset");
    check(measuredSpeed(design,launch.second)>measuredSpeed(design,launch.first)+6,"Actual finite-train replay demonstrates useful energy restoration in the relaunch");
    // Check convergence against measured boundaries, not just a reported flag.
    const auto& text=design.planningDiagnostics;size_t module=0,source=text.find("\"forceDesignedAirtime\":[");
    check(source!=std::string::npos,"Force source speeds are retained for comparison with final replay");
    for(int hill=0;hill<4;++hill){module=text.find("\"identity\":\"fvd-airtime\"",module);check(module!=std::string::npos,"All four physical airtime entries are present");
        size_t endModule=text.find('}',module);double at=fieldNumber(text,module,endModule,"\"start\":");
        source=text.find('{',source);check(source!=std::string::npos,"Each airtime source has an authored entry speed");size_t endSource=text.find('}',source);
        double authored=fieldNumber(text,source,endSource,"\"sourceSpeedMps\":");
        check(std::abs(measuredSpeed(design,at)-authored)<=.500001,"Final finite-train airtime entry converges within the0.5m/s authoring tolerance");
        const double end=fieldNumber(text,module,endModule,"\"end\":"),sourceHeight=fieldNumber(text,source,endSource,"\"height\":");
        auto entry=design.track.sample(at),exit=design.track.sample(end);double maximum=entry.position.z;
        for(double distance=at;distance<end;distance+=.5)maximum=std::max(maximum,design.track.sample(distance).position.z);
        maximum=std::max(maximum,exit.position.z);
        check(std::abs((maximum-entry.position.z)-sourceHeight)<.02,"Final canonical airtime height retains its authored force profile under terrain and crossing placement");
        check(std::abs(exit.position.z-entry.position.z)<.01&&std::abs(entry.tangent.z)<.001&&std::abs(exit.tangent.z)<.001,
            "Terrain and crossing placement translate the source hill without warping its level entry and exit ports");
        for(const auto& operation:design.operations)if(operation.kind==DriveKind::Station)
            check(operation.start>=end+(design.request.train.cars-1)*design.request.train.spacing,
                "Terminal brake hardware begins after every airtime element and a full train clearance");
        module=endModule+1;source=endSource+1;
    }
    size_t energy=text.find("\"authoringEnergy\":{");check(energy!=std::string::npos,"Whole-route energy convergence is reported");size_t endEnergy=text.find('}',energy);
    check(fieldNumber(text,energy,endEnergy,"\"maximumSpeedResidualMps\":")<=.500001,"Reported loop and reversal energy residuals also meet the authoring tolerance");
}
void checkAscentTrimFlow(const Design& design){
    const auto ascent=moduleInterval(design,"terrain-ascent-launch"),trim=moduleInterval(design,"inversion-entry-brake"),loop=moduleInterval(design,"record-inversion");
    check(std::abs(ascent.second-trim.first)<1e-6&&std::abs(trim.second-loop.first)<1e-6,"Ascent, trim and retained full loop share their actual physical boundaries");
    check(std::abs(design.track.sample(trim.second).tangent.z)<std::sin(.01*pi/180),"Terrain ascent reaches the actual level inversion inlet");
    for(double boundary:{trim.first,trim.second}){
        const auto before=design.track.sample(boundary-.01),after=design.track.sample(boundary+.01);
        check(norm(after.tangent-before.tangent)<.001,"Ascent, trim and inversion inlet have no abrupt tangent change across their physical joins");
        check(norm(after.curvature-before.curvature)<1e-4,"Ascent, trim and inversion inlet retain smooth curvature across their physical joins");
    }
    check(std::any_of(design.operations.begin(),design.operations.end(),[&](const Operation& op){return op.kind==DriveKind::Brake&&std::abs(op.start-trim.first)<1e-6&&std::abs(op.end-trim.second)<1e-6;}),"Real bounded braking covers exactly the authored trim interval");
    double maximumHeight=0;
    for(double at=ascent.first;at<=trim.second;at+=.5){const auto point=design.track.sample(at).position;
        maximumHeight=std::max(maximumHeight,point.z-design.request.terrain.height(point.x,point.y));}
    check(maximumHeight<60,"Canyon1 ascent follows the escarpment without the former elevated approach towers");
    const auto& text=design.planningDiagnostics;const size_t tail=text.rfind("\"identity\":\"fvd-airtime\"");
    check(tail!=std::string::npos,"Canyon fixture retains its final rigid airtime source");
    const auto point=design.track.sample(fieldNumber(text,tail,text.find('}',tail),"\"end\":")).position;
    const auto& terrain=design.request.terrain;
    const double dx=(point.x-terrain.offsetX)/terrain.horizontalScale,dy=(point.y-terrain.offsetY)/terrain.horizontalScale;
    const double x=std::cos(terrain.headingRadians)*dx+std::sin(terrain.headingRadians)*dy,y=-std::sin(terrain.headingRadians)*dx+std::cos(terrain.headingRadians)*dy;
    check(std::abs(y-120*std::sin(x/850))>=260+terrain.cliffWidth-1e-5,"Final rigid airtime source exits on the actual seeded cliff rim before the adaptive descent");
}
void checkTerminalTurnSpeed(const Design& design){
    const auto& text=design.planningDiagnostics;size_t turn=text.rfind("\"identity\":\"banked-camelback-turn\"");
    check(turn!=std::string::npos,"Terminal turn has a measured canonical interval");const size_t end=text.find('}',turn);
    check(fieldNumber(text,turn,end,"\"corridor\":")==3,"Measured terminal turn belongs to the final corridor");
    const double beginDistance=fieldNumber(text,turn,end,"\"start\":"),endDistance=fieldNumber(text,turn,end,"\"end\":");
    size_t corridor=text.find("\"corridors\":[");check(corridor!=std::string::npos,"Final corridor retains its source speed hint");
    for(int side=0;side<4;++side){corridor=text.find('{',corridor);check(corridor!=std::string::npos,"Each physical corridor has a source speed hint");if(side<3)corridor=text.find('}',corridor)+1;}
    const double authored=fieldNumber(text,corridor,text.find('}',corridor),"\"turnSpeedMps\":");
    double maximumSpeed=0;const double halfTrain=(design.request.train.cars-1)*design.request.train.spacing*.5;
    for(const auto& frame:design.simulation.frames)if(frame.distance+halfTrain>=beginDistance&&frame.distance-halfTrain<=endDistance)
        maximumSpeed=std::max(maximumSpeed,frame.speed);
    check(maximumSpeed>0&&std::abs(maximumSpeed-authored)<=.500001,"Terminal turn is authored at the maximum actual speed while any car occupies it, within0.5m/s");
}
Design generateChecked(uint64_t seed,TerrainKind terrain=TerrainKind::Flat){
    GenerationRequest request;request.seed=seed;request.terrain.kind=terrain;request.targets.requireIntensity=false;
    if(terrain==TerrainKind::Hills)request.maxCandidates=2;
    auto design=generate(request);
    if(!design.accepted())for(const auto* report:{&design.report,&design.simulation.report})for(const auto& error:report->errors)std::cerr<<"seed "<<seed<<' '<<error.code<<": "<<error.message<<'\n';
    check(design.accepted(),"Entire generated circuit passes unmodified geometry, train forces, target and convergence gates");
    check(design.convergence.coarseStep==1./960&&design.convergence.fineStep==1./1920,"Full ride uses required 960/1920 Hz simulation and verification");
    check(design.simulation.metrics.maxGroundHeight>=request.targets.height&&design.simulation.metrics.maxSpeed>=request.targets.speed&&design.simulation.metrics.inversionGroundHeight>=request.targets.inversionHeight,"Existing record hill and speed targets remain selected and satisfied");
    std::cout<<"seed="<<seed<<" accepted=true candidate="<<design.candidate<<" length="<<design.track.length<<" topology="<<design.topology<<'\n';
    return design;
}
bool same(Vec3 a,Vec3 b){return a.x==b.x&&a.y==b.y&&a.z==b.z;}
}
int main(){try{
    // This terrain/S-crest combination previously demanded a rapid bank
    // reversal whose rider-offset force reached -3.88 g despite positive
    // centerline normal load. Exercise the actual generation/acceptance path.
    auto hills=generateChecked(9,TerrainKind::Hills);
    // The optional terrain rerank previously discarded a converged first
    // route, then exhausted the shared eight-rebuild budget on its alternate.
    const auto& hillsPlan=hills.planningDiagnostics;
    check(hills.candidate==0,"Ordinary turn feedback converges the first hills9 candidate without a terrain/energy limit cycle");
    size_t hillsEnergy=hillsPlan.find("\"authoringEnergy\":{");
    size_t selection=hillsPlan.find("\"routeSelection\":",hillsEnergy),selectionEnd=hillsPlan.find('}',selection);
    check(hillsEnergy!=std::string::npos&&selection!=std::string::npos,"Retained route has explicit energy and route-selection evidence");
    check(fieldNumber(hillsPlan,hillsEnergy,selection,"\"corrections\":")<=8,"Optional reranking shares the existing eight-rebuild budget");
    check(fieldNumber(hillsPlan,hillsEnergy,selection,"\"maximumSpeedResidualMps\":")<=.5,"Selected hills route meets the unchanged source energy tolerance");
    // Geometry may select another bounded candidate. A converged provisional
    // route still must survive an unsuccessful optional rerank, and diagnostics
    // must identify the route that actually supplied the accepted telemetry.
    const bool retained=hillsPlan.find("\"retainedConvergedProvisional\":true",selection)<selectionEnd;
    const bool reselected=hillsPlan.find("\"reselected\":true",selection)<selectionEnd;
    for(const char* key:{"Shape","Placement"}){
        const std::string chosen="\""+std::string(retained||!reselected?"provisional":"attempted")+key+"\":",final="\"final"+std::string(key)+"\":";
        check(fieldNumber(hillsPlan,selection,selectionEnd,chosen)==fieldNumber(hillsPlan,selection,selectionEnd,final),"Selected route diagnostics identify the actual converged source site");
    }
    // The current terrain/source itinerary must complete on its first
    // candidate; its initial loop energy is not required to be deficient.
    auto canyon=generateChecked(1,TerrainKind::Canyon);classifyGeometry(canyon);checkFoldedGeometry(canyon);checkOperationIntent(canyon);checkAscentTrimFlow(canyon);checkTerminalTurnSpeed(canyon);
    check(canyon.candidate==0,"Adaptive crossing and support placement retain candidate zero under all acceptance gates");checkSupportSpacing(canyon);
    auto first=generateChecked(42);auto firstShapes=classifyGeometry(first);checkFoldedGeometry(first);checkOperationIntent(first);checkLocalTurnLoads(first);checkTerminalTurnSpeed(first);
    auto second=generateChecked(5);auto secondShapes=classifyGeometry(second);checkFoldedGeometry(second);checkOperationIntent(second);
    check(std::abs(first.track.length-second.track.length)>1,"Different seeds change the actual complete circuit geometry");
    check(std::abs(firstShapes.immelmannRise-secondShapes.immelmannRise)>.1&&std::abs(firstShapes.immelmannForwardExtent-secondShapes.immelmannForwardExtent)>.1,"Seeded Immelmann variation changes physical height and proportions");
    auto repeated=generateChecked(42);
    check(first.topology==repeated.topology&&first.candidate==repeated.candidate&&first.track.knots.size()==repeated.track.knots.size(),"Repeated seed reproduces the selected circuit and knot count");
    bool identical=true;
    for(size_t i=0;i<first.track.knots.size();++i){const auto& a=first.track.knots[i];const auto& b=repeated.track.knots[i];identical&=same(a.position,b.position)&&same(a.tangent,b.tangent)&&same(a.curvature,b.curvature)&&same(a.up,b.up)&&a.bank==b.bank&&a.element==b.element;}
    check(identical,"Repeated seed reproduces every persisted canonical knot exactly");
    check(reportJson(first)==reportJson(repeated),"Repeated seed reproduces complete measured telemetry, acceptance and numerical convergence");
    std::cout<<"PASS "<<checks<<" organic full-circuit checks: genuine inversion geometry/order, retained hill/full loop, selected targets, seeded proportions, exact repeatability and mandatory convergence\n";
    return 0;
}catch(const std::exception& e){std::cerr<<"FAIL after "<<checks<<": "<<e.what()<<'\n';return 1;}}
