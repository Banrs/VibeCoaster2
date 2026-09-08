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
        check(start.up.z>.98&&end.up.z>.98,"Each complete inversion group enters and exits upright");
        double firstAscendingVertical=INFINITY,firstDescendingVertical=INFINITY;
        double firstInvertedHorizontal=INFINITY,firstEntryAxisRollMid=INFINITY,firstExitAxisRollMid=INFINITY;
        double invertedApex=-INFINITY;
        for(double distance=region.startDistance;distance<=region.endDistance;distance+=.25){
            auto sample=design.track.sample(distance);
            check(finite(sample.position)&&finite(sample.tangent)&&finite(sample.up),"Generated canonical inversion has finite frames");
            if(sample.tangent.z>.98)firstAscendingVertical=std::min(firstAscendingVertical,distance);
            if(sample.tangent.z<-.98)firstDescendingVertical=std::min(firstDescendingVertical,distance);
            if(sample.up.z<-.95&&std::abs(sample.tangent.z)<.15)firstInvertedHorizontal=std::min(firstInvertedHorizontal,distance);
            if(std::abs(sample.up.z)<.12&&dot(sample.tangent,forward)>.98)firstEntryAxisRollMid=std::min(firstEntryAxisRollMid,distance);
            if(std::abs(sample.up.z)<.12&&dot(sample.tangent,exitForward)>.98)firstExitAxisRollMid=std::min(firstExitAxisRollMid,distance);
            if(sample.up.z<-.5)invertedApex=std::max(invertedApex,sample.position.z-design.request.terrain.height(sample.position.x,sample.position.y));
        }
        if(headingAgreement>.995&&std::abs(rise)<1){
            check(std::isfinite(firstAscendingVertical)&&std::isfinite(firstDescendingVertical)&&firstAscendingVertical<firstDescendingVertical,"Retained full loop contains ascent and descent pitch in its original heading");
            ++observations.fullLoop;
        }else if(headingAgreement<-.995&&rise>70){
            check(std::isfinite(firstAscendingVertical)&&std::isfinite(firstInvertedHorizontal)&&std::isfinite(firstExitAxisRollMid),"Immelmann has actual ascending half-loop, inverted horizontal apex and exit half-roll");
            check(firstAscendingVertical<firstInvertedHorizontal&&firstInvertedHorizontal<firstExitAxisRollMid,"Immelmann pitches first and then rolls upright on its reversed heading");
            check(invertedApex>=design.request.targets.inversionHeight,"The real Immelmann inverted apex meets the terrain-relative target");
            observations.immelmannRise=rise;observations.immelmannForwardExtent=region.forwardExtent;++observations.immelmann;
        }else if(headingAgreement<-.995&&rise< -70){
            check(std::isfinite(firstEntryAxisRollMid)&&std::isfinite(firstInvertedHorizontal)&&std::isfinite(firstDescendingVertical),"Dive loop has actual entry half-roll followed by descending half-loop");
            check(firstEntryAxisRollMid<firstInvertedHorizontal&&firstInvertedHorizontal<firstDescendingVertical,"Dive loop rolls inverted before pitching down to its reversed exit");
            ++observations.diveLoop;
        }else check(false,"Every inversion group in the paired fixture has a geometrically recognized complete topology");
    }
    check(observations.fullLoop==1&&observations.immelmann==1&&observations.diveLoop==1,"Generated ride retains its full loop and adds exactly one genuine Immelmann and dive loop");
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
            double separation=std::abs(x.position.z-y.position.z);check(separation>25,"Folded branches have substantial real vertical separation, with full train/support clearance independently accepted");
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
void checkBridgeDrive(const Design& design){
    const auto& text=design.planningDiagnostics;size_t object=text.find("\"returnFlowBridge\":{");
    check(object!=std::string::npos,"Production flow bridge exposes its actual canonical interval");
    const auto number=[&](const char* key){size_t field=text.find(key,object);check(field!=std::string::npos,"Flow bridge endpoint is present");return std::stod(text.substr(field+std::char_traits<char>::length(key)));};
    const double begin=number("\"start\":"),end=number("\"end\":");
    check(end>begin&&begin>0&&end<design.track.length,"Bridge remains inside the closed circuit");
    bool retained=false;
    for(const auto& operation:design.operations)if(operation.kind==DriveKind::Boost&&operation.start>begin&&operation.start<end&&operation.end>end){
        retained=std::isfinite(operation.maxForce)&&operation.maxForce>0&&std::isfinite(operation.maxPower)&&operation.maxPower>0&&operation.rampSeconds>0&&operation.exitFadeMeters>0;
        check(design.track.sample(operation.start+1).element==Element::Turn,"Original recovery boost now acts on genuinely curved production track");
    }
    check(retained,"Original recovery motor keeps finite force/power ramps and continues through the following turn");
}
void checkConnectorPacing(const Design& design){
    // Independently observe the post-loop recovery and boost between actual
    // inversion and airtime geometry; labels do not establish activity.
    const auto& loop=design.inversionDimensions.front();double begin=loop.endDistance,end=begin;
    while(end<design.track.length&&design.track.sample(end).element!=Element::Airtime)end+=2;
    check(end-begin>300,"Post-loop connector is measured over its complete recovery and boost");
    double low=INFINITY,high=-INFINITY,peak=-INFINITY,quiet=0,longest=0;const double entry=design.track.sample(begin).position.z;
    for(double s=begin;s<end;s+=2)peak=std::max(peak,design.track.sample(s).position.z-entry);
    for(size_t i=1;i<design.simulation.frames.size();++i){const auto& frame=design.simulation.frames[i];if(frame.distance<begin||frame.distance>end)continue;low=std::min(low,frame.seats[0].vertical);high=std::max(high,frame.seats[0].vertical);if(std::abs(frame.seats[0].vertical-1)<.1)quiet+=frame.time-design.simulation.frames[i-1].time;else quiet=0;longest=std::max(longest,quiet);}
    check(peak>8&&high-low>.5,"Connector has substantial actual height and measured force variation");
    check(longest<2,"Post-loop connector avoids a long ordinary1g hold");
}
Design generateChecked(uint64_t seed){
    GenerationRequest request;request.seed=seed;request.targets.requireIntensity=false;
    auto design=generate(request);
    if(!design.accepted())for(const auto* report:{&design.report,&design.simulation.report})for(const auto& error:report->errors)std::cerr<<"seed "<<seed<<' '<<error.code<<": "<<error.message<<'\n';
    check(design.accepted(),"Entire generated circuit passes unmodified geometry, train forces, target and convergence gates");
    check(design.convergence.coarseStep==1./960&&design.convergence.fineStep==1./1920,"Full ride uses required 960/1920 Hz simulation and verification");
    check(design.simulation.metrics.maxGroundHeight>=request.targets.height&&design.simulation.metrics.maxSpeed>=request.targets.speed,"Existing record hill and speed targets remain selected and satisfied");
    std::cout<<"seed="<<seed<<" accepted=true candidate="<<design.candidate<<" length="<<design.track.length<<" topology="<<design.topology<<'\n';
    return design;
}
bool same(Vec3 a,Vec3 b){return a.x==b.x&&a.y==b.y&&a.z==b.z;}
}
int main(){try{
    auto first=generateChecked(42);auto firstShapes=classifyGeometry(first);checkFoldedGeometry(first);checkBridgeDrive(first);checkConnectorPacing(first);
    auto second=generateChecked(5);auto secondShapes=classifyGeometry(second);checkFoldedGeometry(second);checkBridgeDrive(second);checkConnectorPacing(second);
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