#include "coaster/coaster.hpp"
#include "../src/generation_internal.hpp"
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
        // A curved rolling descent can change yaw after the half-loop. Test
        // the actual vertical/roll sequence, allowing that authored return.
        constexpr double portAngle=15*pi/180,phaseTolerance=2*portAngle,headingCosine=.866025403784439;
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
        if(std::abs(pitchSweep-2*pi)<phaseTolerance){
            check(std::isfinite(firstAscendingVertical)&&std::isfinite(firstDescendingVertical)&&firstAscendingVertical<firstDescendingVertical,"Retained full loop contains ascent and descent pitch in its original heading");
            check(std::abs(rollSweep)<phaseTolerance,"Full loop is a pitch revolution without an added barrel roll");
            check(region.verticalExtent>=67&&region.verticalExtent<=80,"Record-scale full loop keeps its own68-78m physical height separately from the tall Immelmann");
            double minimumSpeed=INFINITY;
            for(const auto& frame:design.simulation.frames)if(frame.distance>=region.startDistance&&frame.distance<=region.endDistance)minimumSpeed=std::min(minimumSpeed,frame.speed);
            check(minimumSpeed>24,"Larger full loop retains its authored26m/s moving crest energy");
            ++observations.fullLoop;
        }else if(headingAgreement< -headingCosine&&region.verticalExtent>70&&rise>=0&&std::abs(pitchSweep-pi)<phaseTolerance){
            check(std::isfinite(firstAscendingVertical)&&!std::isfinite(firstDescendingVertical),"Immelmann contains an actual ascending half-loop");
            check(std::abs(std::abs(rollSweep)-pi)<phaseTolerance&&std::abs(ascendingRoll-initialRoll)<pi/3,"Compound Immelmann reaches ascending vertical before completing most of its half-roll");
            check(invertedApex>=design.request.targets.inversionHeight,"High Immelmann retains the selected terrain-relative inverted-apex target");
            double peak=-INFINITY;
            for(const auto& frame:design.simulation.frames)for(int seat=0;seat<3;++seat){double at=frame.distance+seatDistanceOffset(design.request.train,seat);
                if(at>=region.startDistance&&at<=region.endDistance)peak=std::max(peak,frame.seats[seat].vertical);}
            check(peak>4.5,"Signature Immelmann delivers its stronger entry/pullout intention; full signed force acceptance remains independent");
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
int checkCrestCharacter(const Design& design){
    int peaks=0,positiveRelief=0,strongAirtime=0;bool rising=false;double ascending=0;
    for(double distance=0;distance<design.track.length;distance+=5){
        const auto sample=design.track.sample(distance);
        if(sample.element!=Element::Airtime){rising=false;continue;}
        if(sample.tangent.z>.03)rising=true;
        if(sample.tangent.z>0)ascending=distance;
        if(!rising||sample.tangent.z>=-.03)continue;
        double lo=ascending,hi=distance;
        for(int iteration=0;iteration<24;++iteration){const double mid=(lo+hi)*.5;if(design.track.sample(mid).tangent.z>0)lo=mid;else hi=mid;}
        const double at=(lo+hi)*.5-seatDistanceOffset(design.request.train,1);
        const auto& frames=design.simulation.frames;
        const auto after=std::lower_bound(frames.begin(),frames.end(),at,[](const auto& frame,double distance){return frame.distance<distance;});
        check(after!=frames.begin()&&after!=frames.end(),"A geometric airtime crest has actual rider telemetry on both sides");
        const auto& before=*(after-1);const double u=(at-before.distance)/(after->distance-before.distance);
        const double normal=before.seats[1].vertical*(1-u)+after->seats[1].vertical*u;
        positiveRelief+=normal>0&&normal<.75;strongAirtime+=normal<-.8;
        ++peaks;rising=false;
    }
    check(peaks>=4,"Complete circuit contains at least four actual ascending-descending force-authored crest shapes");
    check(positiveRelief>0&&strongAirtime>0,"Actual geometric crests include both positive relief and stronger negative airtime; one force recipe cannot replace the sequence");
    return peaks;
}
void checkFoldedGeometry(const Design& design,bool requireCrossing=false){
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
    if(requireCrossing)check(crossings>=1,"Dedicated hills2 circuit has a transverse crossover between widely separated route branches");
    check(straightFlat/design.track.length<.35,"Straight and grade-flat geometry occupies less than35percent of the complete circuit");
    const int airtimePeaks=checkCrestCharacter(design);
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
    const double half=(design.request.train.cars-1)*design.request.train.spacing*.5;
    const auto ascent=moduleInterval(design,"cliff-ascent"),dive=moduleInterval(design,"cliff-dive"),tall=moduleInterval(design,"record-hill");
    check(ascent.second<dive.first&&dive.second<tall.first,"Cliff ascent, summit, dive, final launch and giant camelback retain their physical progression");
    const auto climbStart=design.track.sample(ascent.first),summit=design.track.sample(ascent.second),diveStart=design.track.sample(dive.first),diveEnd=design.track.sample(dive.second);
    check(std::abs(summit.position.z-climbStart.position.z-design.request.targets.height)<.01&&
          std::abs(diveStart.position.z-diveEnd.position.z-design.request.targets.height)<.01&&
          std::abs(summit.position.z-diveStart.position.z)<.01,"Certified cliff rise and common summit datum survive terrain composition exactly");
    int fastest=0,departures=0;
    for(const auto& op:design.operations){
        check(op.maxForce<=design.request.train.carMass*design.request.limits.maxLongitudinalG*gravity&&op.maxPower>0&&op.rampSeconds>0&&op.exitFadeMeters>0,"Every motor retains bounded force, power and real ramps/fades");
        if(op.kind==DriveKind::Launch)++departures;
        if(op.kind==DriveKind::Boost&&op.start>=dive.second&&op.end<=tall.first){
            ++fastest;check(op.end+half<=tall.first+1e-6&&op.start-half>=dive.second-1e-6,"Final launch hardware clears the complete train before either passive signature");
            check(measuredSpeed(design,tall.first)>measuredSpeed(design,op.start-half),"The final launch supplies actual useful acceleration");
            for(const auto& other:design.operations)if(other.kind==DriveKind::Boost||other.kind==DriveKind::Launch)
                check(op.targetSpeed>=other.targetSpeed,"Fastest launch belongs after the dive and before the giant camelback");
        }
        if(op.kind==DriveKind::Boost&&(op.end+half<ascent.first||op.start-half>ascent.second))
            check(std::abs(op.maxForce/design.request.train.carMass-.8*gravity)<1e-8,"Ordinary boosters retain the nominal0.8g hardware rating");
    }
    check(departures==1&&fastest==1,"One departure and one final fastest launch retain separate physical roles");
    const auto& text=design.planningDiagnostics;
    for(const char* name:{"record-hill","record-inversion","high-immelmann","fvd-airtime","closing-airtime"}){
        const auto interval=moduleInterval(design,name);const size_t object=text.find("\"identity\":\""+std::string(name)+"\""),objectEnd=text.find('}',object);
        const double entry=fieldNumber(text,object,objectEnd,"\"sourceEntryMps\":"),exit=fieldNumber(text,object,objectEnd,"\"sourceExitMps\":");
        check(std::abs(measuredSpeed(design,interval.first)-entry)<=.500001&&std::abs(measuredSpeed(design,interval.second)-exit)<=.500001,"Independent final finite-train inlet AND exit agree with each source within0.5m/s");
        const size_t phases=text.find("\"phases\":[",object),phaseEnd=text.find(']',phases);size_t point=phases;
        while((point=text.find("\"distance\":",point))<phaseEnd){
            const size_t end=text.find('}',point);const double at=fieldNumber(text,point,end,"\"distance\":"),speed=fieldNumber(text,point,end,"\"sourceSpeedMps\":");
            check(std::abs(measuredSpeed(design,at)-speed)<=.500001,"Independent apex and valley speeds retain source energy throughout each passive element");point=end+1;
        }
        auto start=design.track.sample(interval.first),finish=design.track.sample(interval.second);double height=0;
        for(double at=interval.first;at<=interval.second;at+=.5)height=std::max(height,design.track.sample(at).position.z-start.position.z);
        check(std::abs(height-fieldNumber(text,object,objectEnd,"\"sourceHeightMeters\":"))<.02,"Canonical source height is translated rigidly, including through crossing placement");
        check(std::abs(start.tangent.z)<.001&&std::abs(finish.tangent.z)<.001,"Passive source ports remain level");
        for(const auto& op:design.operations){
            if(op.kind==DriveKind::Station)check(op.start>=interval.second+2*half,"Terminal brake begins after all passive elements and a full train");
            else check(op.end+half<=interval.first+1e-5||op.start-half>=interval.second-1e-5,"No positive or negative hardware work intrudes into a passive source");
        }
    }
    const size_t energy=text.find("\"authoringEnergy\":{"),energyEnd=text.find('}',energy);
    check(energy!=std::string::npos&&fieldNumber(text,energy,energyEnd,"\"corrections\":")<=8,"All geometry and energy feedback shares the eight-rebuild budget");
    check(fieldNumber(text,energy,energyEnd,"\"maximumSpeedResidualMps\":")<=.500001,"Final source and motor residuals meet the unchanged0.5m/s tolerance");
}
void checkTurnSpeed(const Design& design){
    const auto& text=design.planningDiagnostics;size_t turn=text.find("\"connectingProfiles\":["),end=text.find(']',turn);
    check(turn!=std::string::npos,"Complete passive connecting profiles have physical intervals and measured speed intent");
    const double half=(design.request.train.cars-1)*design.request.train.spacing*.5;
    while((turn=text.find('{',turn))<end){
        const size_t finish=text.find('}',turn);const double beginDistance=fieldNumber(text,turn,finish,"\"start\":"),endDistance=fieldNumber(text,turn,finish,"\"end\":");
        const double authored=fieldNumber(text,turn,finish,"\"sourceSpeedMps\":");double maximum=0;
        for(const auto& frame:design.simulation.frames)if(frame.distance+half>=beginDistance&&frame.distance-half<=endDistance)maximum=std::max(maximum,frame.speed);
        check(maximum>0&&std::abs(maximum-authored)<=.500001,"Every connecting profile is authored for the maximum speed while any car occupies it");
        const double turnEnd=fieldNumber(text,turn,finish,"\"turnEnd\":"),turnSpeed=fieldNumber(text,turn,finish,"\"turnSpeedMps\":");double turnMaximum=0;
        for(const auto& frame:design.simulation.frames)if(frame.distance+half>=beginDistance&&frame.distance-half<=turnEnd)turnMaximum=std::max(turnMaximum,frame.speed);
        check(turnMaximum>0&&std::abs(turnMaximum-turnSpeed)<=.500001,"Planar turn geometry is authored for its own occupied speed, independently of later recovery rail");turn=finish+1;
    }
}
Design generateChecked(uint64_t seed,TerrainKind terrain=TerrainKind::Flat,bool requireCrossing=false){
    GenerationRequest request;request.seed=seed;request.terrain.kind=terrain;request.targets.requireIntensity=false;
    request.maxCandidates=1;
    auto design=requireCrossing?detail::generateRide(request,true):generate(request);
    if(!design.accepted())for(const auto* report:{&design.report,&design.simulation.report})for(const auto& error:report->errors)std::cerr<<"seed "<<seed<<' '<<error.code<<": "<<error.message<<'\n';
    check(design.accepted(),"Entire generated circuit passes unmodified geometry, train forces, target and convergence gates");
    check(design.convergence.coarseStep==1./960&&design.convergence.fineStep==1./1920,"Full ride uses required 960/1920 Hz simulation and verification");
    check(design.simulation.metrics.maxGroundHeight>=request.targets.height&&design.simulation.metrics.maxSpeed>=request.targets.speed&&design.simulation.metrics.inversionGroundHeight>=request.targets.inversionHeight,"Existing record hill and speed targets remain selected and satisfied");
    std::cout<<"seed="<<seed<<" accepted=true candidate="<<design.candidate<<" length="<<design.track.length<<" topology="<<design.topology<<'\n';
    return design;
}
bool same(Vec3 a,Vec3 b){return a.x==b.x&&a.y==b.y&&a.z==b.z;}
}
int main(int argc,char** argv){try{
    const std::string group=argc>1?argv[1]:"all";
    if(argc>2||(group!="all"&&group!="hills"&&group!="crossing"&&group!="canyon"&&group!="variety"))
        throw std::invalid_argument("Expected all, hills, crossing, canyon or variety");
    if(group=="all"||group=="hills"){
    // This terrain/S-crest combination previously demanded a rapid bank
    // reversal whose rider-offset force reached -3.88 g despite positive
    // centerline normal load. Exercise the actual generation/acceptance path.
    auto hills=generateChecked(9,TerrainKind::Hills);
    check(hills.candidate==0,"The first hills9 candidate converges without an outer terrain/energy retry");checkOperationIntent(hills);checkTurnSpeed(hills);
    }
    if(group=="all"||group=="crossing"){
    // Source closure changes individual seed layouts. Keep one explicit real
    // crossover fixture, with the same transverse/nonlocal geometry definition.
    auto crossing=generateChecked(2,TerrainKind::Hills,true);checkFoldedGeometry(crossing,true);
    }
    if(group=="all"||group=="canyon"){
    // The current terrain/source itinerary must complete on its first
    // candidate; its initial loop energy is not required to be deficient.
    auto canyon=generateChecked(1,TerrainKind::Canyon);classifyGeometry(canyon);checkFoldedGeometry(canyon);checkOperationIntent(canyon);checkTurnSpeed(canyon);
    check(canyon.candidate==0,"Adaptive crossing and support placement retain candidate zero under all acceptance gates");checkSupportSpacing(canyon);
    }
    if(group=="all"||group=="variety"){
    auto first=generateChecked(42);auto firstShapes=classifyGeometry(first);checkFoldedGeometry(first);checkOperationIntent(first);checkTurnSpeed(first);
    auto second=generateChecked(5);auto secondShapes=classifyGeometry(second);checkFoldedGeometry(second);checkOperationIntent(second);
    check(std::abs(first.track.length-second.track.length)>1,"Different seeds change the actual complete circuit geometry");
    check(std::abs(firstShapes.immelmannRise-secondShapes.immelmannRise)>.1&&std::abs(firstShapes.immelmannForwardExtent-secondShapes.immelmannForwardExtent)>.1,"Seeded Immelmann variation changes physical height and proportions");
    auto repeated=generateChecked(42);
    check(first.topology==repeated.topology&&first.candidate==repeated.candidate&&first.track.knots.size()==repeated.track.knots.size(),"Repeated seed reproduces the selected circuit and knot count");
    bool identical=true;
    for(size_t i=0;i<first.track.knots.size();++i){const auto& a=first.track.knots[i];const auto& b=repeated.track.knots[i];identical&=same(a.position,b.position)&&same(a.tangent,b.tangent)&&same(a.curvature,b.curvature)&&same(a.up,b.up)&&a.bank==b.bank&&a.element==b.element;}
    check(identical,"Repeated seed reproduces every persisted canonical knot exactly");
    check(reportJson(first)==reportJson(repeated),"Repeated seed reproduces complete measured telemetry, acceptance and numerical convergence");
    }
    std::cout<<"PASS "<<checks<<" organic full-circuit checks ("<<group<<")\n";
    return 0;
}catch(const std::exception& e){std::cerr<<"FAIL after "<<checks<<": "<<e.what()<<'\n';return 1;}}
