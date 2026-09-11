#include "coaster/coaster.hpp"
#include "reference_fixture.hpp"
#include <iostream>
#include <stdexcept>
using namespace coaster;
namespace {
int checks=0;
void require(bool okay,const char* message){++checks;if(!okay)throw std::runtime_error(message);}
bool has(const ValidationReport& r,const std::string& code){return std::any_of(r.errors.begin(),r.errors.end(),[&](const Finding& f){return f.code==code;});}
SimulationResult result(){
    SimulationResult r;r.completed=true;auto& m=r.metrics;
    m.maxSpeed=80;m.minVerticalG=-.5;m.maxVerticalG=5;m.maxLateralG=1;m.maxLongitudinalG=4;m.maxJerkGps=5;m.exposure10Seconds=40;m.duration=120;m.launchTo180=1.3;
    for(int s=0;s<3;++s){auto& seat=m.seats[s];seat.exposure10Seconds=40;
        for(int a=0;a<3;++a){auto& x=seat.axes[a];x.minG=a==0?-.5:-.2;x.maxG=a==0?(s==0?5:4):(a==1?1:4);x.meanG=a==0?2:.1;x.maxRateGps=5;x.mean1sMin=x.minG;x.mean1sMax=x.maxG*.8;x.mean10sMin=0;x.mean10sMax=a==0?4:.2;}}
    for(auto& envelope:r.forceEnvelope)envelope.performed=true;
    r.frames.push_back({0,0,0,{}});r.frames.push_back({120,5000,0,{}});return r;
}
}
int main(){try{
    // Production generation and persistence share this request validator.
    // Standalone simulation remains available at other resolutions for tests.
    GenerationRequest request;request.targets.requireIntensity=false;
    require(validateRequest(request).valid(),"Production request accepts exactly 960 Hz");
    for(double step:{1./30,1./480,1./1920,std::nextafter(1./960,0.),std::nextafter(1./960,1.)}){
        request.simulationStep=step;
        require(has(validateRequest(request),"REQUEST_RANGE"),"Production request rejects a substituted simulation rate");
        bool started=false;auto rejected=generate(request,{},[&](int,const std::string&){started=true;});
        require(!started&&!rejected.accepted()&&rejected.track.spans.empty()&&has(rejected.report,"REQUEST_RANGE"),"Unsupported rate rejects before candidate work");
    }
    request.simulationStep=1./960;request.targets.requireIntensity=true;request.targets.referenceExposure=1;request.targets.referenceId="unverified-test-scalar";
    bool started=false;auto scalar=generate(request,{},[&](int,const std::string&){started=true;});
    require(!started&&!scalar.accepted()&&scalar.track.spans.empty()&&has(scalar.report,"REFERENCE_UNAVAILABLE"),"Unverified scalar rejects before candidate search");
    request.targets.requireIntensity=false;require(validateRequest(request).valid(),"Proof exploration can retain explicitly unverified comparison data");
    std::vector<AuthoredPoint> straight;
    for(int i=0;i<=100;++i)straight.push_back({{double(i),0,20},0,Element::Launch,{0,0,1}});
    const auto track=compile(straight,false);TrainConfig train;train.cars=1;
    const Operation launch{0,99,DriveKind::Launch,20,train.carMass*4,train.carMass*400,.1};
    for(double step:{1./30,1./480,1./1920}){
        const auto replay=simulate(track,{launch},train,step);
        require(replay.completed&&replay.report.valid(),"Standalone simulation retains independent resolution controls");
    }
    {
        // A physical launch can finish under every raw peak guard while
        // sustaining +X too long. The full solver history must reject it.
        auto longStraight=straight;for(auto& point:longStraight)point.position.x*=8;
        const auto launchTrack=compile(longStraight,false);
        const Operation sustained{0,790,DriveKind::Launch,120,train.carMass*gravity*5.8,train.carMass*gravity*5.8*150,.15};
        for(double step:{1./960,1./1920}){
            const auto replay=simulate(launchTrack,{sustained},train,step);
            require(replay.completed&&replay.report.valid()&&replay.metrics.maxLongitudinalG<Limits{}.maxLongitudinalG,"Duration fixture must physically finish below the configured peak cap");
            Targets proof;proof.requireIntensity=false;
            const auto acceptance=validateSimulationTargets(replay,proof,Limits{});
            require(has(acceptance,"FORCE_DURATION")&&!replay.forceEnvelopePassed(),"A usable physical trajectory remains rejected by signed duration acceptance");
            for(const auto& envelope:replay.forceEnvelope)require(envelope.performed&&envelope.directional[4].utilization>1,"All physical rider histories receive the mandatory duration check");
            const auto failure=std::find_if(acceptance.errors.begin(),acceptance.errors.end(),[](const Finding& f){return f.code=="FORCE_DURATION";});
            require(failure!=acceptance.errors.end()&&failure->distance>=1,"Force diagnostic maps elapsed time back to track distance");
        }
    }
    Limits limits;Targets targets;targets.requireIntensity=false;
    auto a=result(),b=a;ConvergenceAssessment assessment;
    auto equal=compareSimulationConvergence(a,b,limits,assessment);
    require(equal.valid()&&assessment.performed&&assessment.passed,"Identical successful simulations do not converge");
    require(assessment.metrics.size()==160,"Expected global and every-seat assessed fields are missing");
    require(assessment.maxSpeedRelativeError==0&&assessment.maxForceRelativeError==0,"Equal metrics have nonzero error");
    b.metrics.seats[2].axes[0].maxG=4.2;
    require(!compareSimulationConvergence(a,b,limits,assessment).valid(),"Stable global peak masks changed rear peak");
    require(std::any_of(assessment.metrics.begin(),assessment.metrics.end(),[](const ConvergenceMetric& x){return x.name=="rear.vertical.maxG"&&x.absoluteDifference>x.tolerance;}),"Rear diagnostic missing");
    b=a;b.metrics.seats[1].axes[1].meanG=.115;
    require(compareSimulationConvergence(a,b,limits,assessment).valid(),"Documented near-zero absolute floor not respected");
    b.metrics.seats[1].axes[1].meanG=.13;
    require(!compareSimulationConvergence(a,b,limits,assessment).valid(),"Near-zero force change beyond .02 g accepted");
    b=a;b.metrics.seats[1].axes[1].mean10sMin=.02;
    require(!compareSimulationConvergence(a,b,limits,assessment).valid(),"Tolerance boundary must be strictly below two percent");
    b=a;b.metrics.maxSpeed=80.81;
    require(!compareSimulationConvergence(a,b,limits,assessment).valid()&&assessment.maxSpeedRelativeError>.01,"One-percent speed limit ignored");
    for(int axis:{1,2}){
        limits={};b=a;b.metrics.seats[2].axes[axis].maxRateGps=1e9;
        require(compareSimulationConvergence(a,b,limits,assessment).valid(),"Unassessed component rate silently gated");
        (axis==1?limits.maxLateralRateGps:limits.maxLongitudinalRateGps)=10;
        require(!compareSimulationConvergence(a,b,limits,assessment).valid()&&assessment.metrics.size()==163,"Configured component rate not checked at every seat");
        require(has(validateSimulationTargets(b,targets,limits),axis==1?"LATERAL_FORCE_RATE":"LONGITUDINAL_FORCE_RATE"),"Configured fine component rate gate omitted");
    }
    limits={};b=a;b.metrics.seats[0].axes[2].mean10sMax=NAN;
    require(!compareSimulationConvergence(a,b,limits,assessment).valid(),"Missing required statistic accepted");
    b=a;b.completed=false;
    require(has(compareSimulationConvergence(a,b,limits,assessment),"CONVERGENCE_SIMULATION")&&!assessment.passed,"Fine stall accepted");
    b=a;b.cancelled=true;
    require(has(compareSimulationConvergence(a,b,limits,assessment),"CANCELLED")&&!assessment.passed,"Fine cancellation accepted");
    b=a;b.report.fail("TEST","Numerically invalid");
    require(!compareSimulationConvergence(a,b,limits,assessment).valid(),"Fine numerical error accepted");
    b=a;b.metrics.maxLateralG=limits.maxLateralG+.01;a.metrics.maxLateralG=limits.maxLateralG-.01;
    require(compareSimulationConvergence(a,b,limits,assessment).valid(),"Small lateral change should converge independently of envelope");
    require(has(validateSimulationTargets(b,targets,limits),"LATERAL_FORCE"),"Converged fine force breach accepted");
    a=result();b=a;b.metrics.launchTo180=1.401;
    require(compareSimulationConvergence(a,b,limits,assessment).valid(),"Launch time is a target rather than a substituted force metric");
    require(has(validateSimulationTargets(b,targets,limits),"LAUNCH_TARGET"),"Fine launch target omitted");
    b=a;targets.requireIntensity=true;
    require(has(validateSimulationTargets(b,targets,limits),"REFERENCE_UNAVAILABLE"),"Missing real reference silently bypassed");
    targets.referenceId="explicit-test-reference";targets.referenceExposure=37;
    require(has(validateSimulationTargets(b,targets,limits),"REFERENCE_UNAVAILABLE"),"Manual scalar cannot bypass strict target evaluation");
    targets=syntheticReference(37);
    require(has(validateSimulationTargets(b,targets,limits),"INTENSITY_TARGET"),"Fine intensity threshold omitted");
    targets.requireIntensity=false;b.metrics.maxVerticalG=NAN;
    require(has(validateSimulationTargets(b,targets,limits),"NONFINITE_METRIC"),"NaN comparison bypasses force validation");
    Design design;design.simulation=result();design.request.targets.requireIntensity=false;
    require(!design.accepted(),"Coarse-only design is rideable without convergence");
    // Synthetic acceptance-state fixture: agreement flags cannot substitute
    // for the exact production rates, independently of any particular route.
    design.convergence.performed=design.convergence.passed=true;
    design.convergence.coarseStep=1./960;design.convergence.fineStep=1./1920;
    require(design.accepted(),"Complete acceptance state at production rates must be recognized");
    auto missing=design;missing.simulation.forceEnvelope[2].performed=false;
    require(!missing.accepted()&&has(validateSimulationTargets(missing.simulation,targets,limits),"FORCE_ENVELOPE_REQUIRED"),"Missing rear force assessment cannot ride or pass targets");
    require(has(compareSimulationConvergence(design.simulation,missing.simulation,limits,assessment),"CONVERGENCE_FORCE_ENVELOPE"),"Half-step comparison cannot trust a missing rider assessment");
    auto changed=design.simulation;changed.forceEnvelope[1].directional[1].utilization=.03;
    require(has(compareSimulationConvergence(design.simulation,changed,limits,assessment),"CONVERGENCE_METRIC"),"Changed signed duration assessment participates in half-step verification");
    for(double step:{1./30,1./480,1./1920,std::nextafter(1./960,0.),std::numeric_limits<double>::quiet_NaN()}){
        auto substituted=design;substituted.request.simulationStep=step;
        require(!substituted.accepted(),"A substituted requested rate cannot trust stale passing flags");
        verifyConvergence(substituted);
        require(!substituted.accepted()&&!substituted.convergence.performed&&has(substituted.report,"CONVERGENCE_RATE"),"Wrong rate fails explicitly before attempting finer replay");
    }
    auto substituted=design;substituted.convergence.coarseStep=1./480;
    require(!substituted.accepted(),"Passing comparison at a substituted coarse rate cannot qualify");
    substituted=design;substituted.convergence.fineStep=1./960;
    require(!substituted.accepted(),"Passing comparison at a substituted fine rate cannot qualify");
    design.convergence={};
    verifyConvergence(design,[]{return true;});
    require(!design.accepted()&&design.simulation.cancelled&&has(design.report,"CANCELLED"),"Cancellation loses failed-closed state");
    require(design.simulation.frames.size()==2&&design.simulation.frames.back().distance==5000,"Cancelled verification mutates coarse presentation trace");
    design={};design.simulation=result();design.request.targets.requireIntensity=false;
    verifyConvergence(design); // Empty canonical track cannot complete a finer replay.
    require(!design.accepted()&&!design.convergence.passed,"Invalid finer replay becomes accepted");
    require(design.simulation.frames.size()==2&&design.simulation.frames.back().distance==5000,"Failed verification replaces coarse presentation trace");
    std::cout<<"PASS "<<checks<<" convergence acceptance checks\n";return 0;
}catch(const std::exception& e){std::cerr<<"FAIL after "<<checks<<": "<<e.what()<<"\n";return 1;}}
