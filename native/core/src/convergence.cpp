#include "coaster/coaster.hpp"
#include "coaster/force_envelope.hpp"
#include "simulation_internal.hpp"
#include <stdexcept>

namespace coaster {
ValidationReport detail::validateSimulationForces(const SimulationResult& simulation,const Limits& limits){
    ValidationReport report;
    const auto& m=simulation.metrics;
    for(const auto& envelope:simulation.forceEnvelope){
        if(!envelope.performed||envelope.cancelled)report.fail("FORCE_ENVELOPE_REQUIRED","Every rider position requires a completed full-rate force-envelope assessment");
        report.errors.insert(report.errors.end(),envelope.report.errors.begin(),envelope.report.errors.end());
    }
    for(double v:{m.maxSpeed,m.exposure10Seconds,m.minVerticalG,m.maxVerticalG,m.maxLateralG,m.maxLongitudinalG,m.maxJerkGps}){
        if(!std::isfinite(v)){report.fail("NONFINITE_METRIC","A required simulated target or force metric is nonfinite");return report;}
    }
    if(m.maxVerticalG>limits.maxVerticalG)report.fail("VERTICAL_FORCE","Positive rider force exceeds configured raw peak guard",0,m.maxVerticalG,limits.maxVerticalG);
    if(m.minVerticalG<limits.minVerticalG)report.fail("VERTICAL_FORCE","Negative rider force exceeds configured raw peak guard",0,m.minVerticalG,limits.minVerticalG);
    if(m.maxLateralG>limits.maxLateralG)report.fail("LATERAL_FORCE","Rider lateral force exceeds configured raw peak guard",0,m.maxLateralG,limits.maxLateralG);
    if(m.maxLongitudinalG>limits.maxLongitudinalG)report.fail("LONGITUDINAL_FORCE","Rider longitudinal force exceeds configured raw peak guard",0,m.maxLongitudinalG,limits.maxLongitudinalG);
    const double negativeLongitudinalLimit=-historicalForceLimit(ForceAxis::Longitudinal,false,.2);
    for(const auto& seat:m.seats)if(seat.axes[2].minG<negativeLongitudinalLimit)report.fail("LONGITUDINAL_FORCE","Braking force exceeds the selected restraint profile's raw peak guard",0,seat.axes[2].minG,negativeLongitudinalLimit);
    if(m.maxJerkGps>limits.maxJerkGps)report.fail("FORCE_TRANSITION","Vertical force transition exceeds provisional envelope",m.maxJerkDistance,m.maxJerkGps,limits.maxJerkGps);
    for(int axis=1;axis<=2;++axis){
        double limit=axis==1?limits.maxLateralRateGps:limits.maxLongitudinalRateGps;
        if(!std::isfinite(limit))continue;
        double peak=0;
        for(const auto& seat:m.seats){
            double value=seat.axes[axis].maxRateGps;
            if(!std::isfinite(value)){report.fail("NONFINITE_METRIC","An assessed component force rate is nonfinite");return report;}
            peak=std::max(peak,value);
        }
        if(peak>limit)report.fail(axis==1?"LATERAL_FORCE_RATE":"LONGITUDINAL_FORCE_RATE","Rider-axis rate exceeds configured provisional gate",0,peak,limit);
    }
    return report;
}

ValidationReport validateSimulationTargets(const SimulationResult& simulation,const Targets& targets,const Limits& limits){
    auto report=detail::validateSimulationForces(simulation,limits);const auto& m=simulation.metrics;
    if(std::any_of(report.errors.begin(),report.errors.end(),[](const Finding& f){return f.code=="NONFINITE_METRIC";}))return report;
    auto minimum=[&](const char* code,double value,double target){
        if(!std::isfinite(value)||value+1e-6<target)report.fail(code,"Measured result misses requested target",0,value,target);
    };
    minimum("SPEED_TARGET",m.maxSpeed,targets.speed);
    if(!std::isfinite(m.launchTo180)||m.launchTo180>targets.launchSeconds)
        report.fail("LAUNCH_TARGET","Measured 0-180 km/h launch exceeds target",0,m.launchTo180,targets.launchSeconds);
    if(targets.requireIntensity){
        auto reference=validateReference(targets);
        report.errors.insert(report.errors.end(),reference.errors.begin(),reference.errors.end());
        if(reference.valid())minimum("INTENSITY_TARGET",m.exposure10Seconds,targets.referenceExposure*1.1);
    }
    return report;
}

ValidationReport compareSimulationConvergence(const SimulationResult& coarse,const SimulationResult& fine,const Limits& limits,ConvergenceAssessment& assessment){
    ValidationReport report;
    assessment.performed=true;assessment.passed=false;assessment.metrics.clear();
    assessment.maxSpeedRelativeError=0;assessment.maxForceRelativeError=0;
    if(coarse.cancelled||fine.cancelled){report.fail("CANCELLED","Convergence simulation cancelled");return report;}
    if(!coarse.completed||!fine.completed||!coarse.report.valid()||!fine.report.valid()){
        report.fail("CONVERGENCE_SIMULATION","Both simulation resolutions must complete without numerical errors");return report;
    }
    if(!coarse.forceEnvelopePassed()||!fine.forceEnvelopePassed()){
        report.fail("CONVERGENCE_FORCE_ENVELOPE","Both resolutions require every rider position to pass the force envelope");return report;
    }
    auto check=[&](std::string name,double a,double b,double fraction,bool speed=false){
        const double scale=std::max(1.,std::abs(b));
        const double difference=std::abs(a-b),tolerance=fraction*scale;
        const bool finiteValues=std::isfinite(a)&&std::isfinite(b)&&std::isfinite(difference)&&std::isfinite(tolerance);
        const double error=finiteValues?difference/scale:std::numeric_limits<double>::infinity();
        assessment.metrics.push_back({name,a,b,difference,tolerance});
        auto& maximum=speed?assessment.maxSpeedRelativeError:assessment.maxForceRelativeError;
        maximum=std::max(maximum,error);
        if(!finiteValues||!(difference<tolerance))
            report.fail("CONVERGENCE_METRIC",name+" differs beyond the required half-step tolerance",0,error,fraction);
    };
    const auto& a=coarse.metrics;const auto& b=fine.metrics;
    check("maxSpeed",a.maxSpeed,b.maxSpeed,.01,true);
    check("minVerticalG",a.minVerticalG,b.minVerticalG,.02);
    check("maxVerticalG",a.maxVerticalG,b.maxVerticalG,.02);
    check("maxLateralG",a.maxLateralG,b.maxLateralG,.02);
    check("maxLongitudinalG",a.maxLongitudinalG,b.maxLongitudinalG,.02);
    check("exposure10Seconds",a.exposure10Seconds,b.exposure10Seconds,.02);
    check("maxVerticalRateGps",a.maxJerkGps,b.maxJerkGps,.02);
    const char* names[]={"front","middle","rear"};const char* axes[]={"vertical","lateral","longitudinal"};
    for(int seat=0;seat<3;++seat){
        std::string prefix=names[seat];
        check(prefix+".exposure10Seconds",a.seats[seat].exposure10Seconds,b.seats[seat].exposure10Seconds,.02);
        for(int axis=0;axis<3;++axis){
            const auto& x=a.seats[seat].axes[axis];const auto& y=b.seats[seat].axes[axis];
            std::string key=prefix+"."+axes[axis]+".";
            check(key+"minG",x.minG,y.minG,.02);check(key+"maxG",x.maxG,y.maxG,.02);check(key+"meanG",x.meanG,y.meanG,.02);
            check(key+"mean1sMin",x.mean1sMin,y.mean1sMin,.02);check(key+"mean1sMax",x.mean1sMax,y.mean1sMax,.02);
            check(key+"mean10sMin",x.mean10sMin,y.mean10sMin,.02);check(key+"mean10sMax",x.mean10sMax,y.mean10sMax,.02);
            bool assessed=axis==0||(axis==1?std::isfinite(limits.maxLateralRateGps):std::isfinite(limits.maxLongitudinalRateGps));
            if(assessed)check(key+"maxRateGps",x.maxRateGps,y.maxRateGps,.02);
        }
        const auto& x=coarse.forceEnvelope[seat];const auto& y=fine.forceEnvelope[seat];
        const std::string key=prefix+".forceEnvelope.";
        for(size_t axis=0;axis<3;++axis){
            const std::string axisKey=key+axes[axis]+".";
            check(axisKey+"minimumG",x.axes[axis].minimumG,y.axes[axis].minimumG,.02);
            check(axisKey+"maximumG",x.axes[axis].maximumG,y.axes[axis].maximumG,.02);
            check(axisKey+"minimumOnsetGps",x.axes[axis].minimumOnsetGps,y.axes[axis].minimumOnsetGps,.02);
            check(axisKey+"maximumOnsetGps",x.axes[axis].maximumOnsetGps,y.axes[axis].maximumOnsetGps,.02);
        }
        auto cases=[&](const char* name,const auto& first,const auto& second){for(size_t i=0;i<first.size();++i)check(key+name+std::to_string(i),first[i].utilization,second[i].utilization,.02);};
        cases("directional",x.directional,y.directional);cases("paired",x.paired,y.paired);
        cases("horizontalReversal",x.horizontalReversal,y.horizontalReversal);cases("durationExtent",x.durationExtent,y.durationExtent);
        check(key+"reducedPositive",x.reducedPositive.utilization,y.reducedPositive.utilization,.02);
        check(key+"zeroToTwo",x.zeroToTwo.utilization,y.zeroToTwo.utilization,.02);
        check(key+"enhancedLongitudinalOnset",x.enhancedLongitudinalOnset.utilization,y.enhancedLongitudinalOnset.utilization,.02);
    }
    assessment.passed=report.valid();return report;
}

void verifyConvergence(Design& design,Cancel cancel){
    design.convergence={};
    design.convergence.coarseStep=design.request.simulationStep;
    design.convergence.fineStep=design.request.simulationStep*.5;
    if(cancel&&cancel()){design.simulation.cancelled=true;design.report.fail("CANCELLED","Convergence verification cancelled");return;}
    if(design.request.simulationStep!=1./960){design.report.fail("CONVERGENCE_RATE","Ride acceptance requires 960 Hz simulation and 1920 Hz verification");return;}
    if(!design.report.valid()||!design.simulation.completed||!design.simulation.report.valid()||design.simulation.cancelled)return;
    try{
        const auto fine=simulate(design.track,design.operations,design.request.train,design.convergence.fineStep,cancel);
        auto comparison=compareSimulationConvergence(design.simulation,fine,design.request.limits,design.convergence);
        design.report.errors.insert(design.report.errors.end(),comparison.errors.begin(),comparison.errors.end());
        if(fine.cancelled||(cancel&&cancel())){
            design.simulation.cancelled=true;design.convergence.passed=false;
            if(!fine.cancelled)design.report.fail("CANCELLED","Convergence verification cancelled");return;
        }
        if(!fine.report.valid()){
            for(const auto& error:fine.report.errors)
                design.report.fail("CONVERGENCE_FINE_"+error.code,error.message,error.distance,error.actual,error.limit);
        }
        if(fine.completed&&fine.report.valid()){
            auto targets=validateSimulationTargets(fine,design.request.targets,design.request.limits);
            for(const auto& error:targets.errors)
                design.report.fail("CONVERGENCE_FINE_"+error.code,error.message,error.distance,error.actual,error.limit);
        }
        design.convergence.passed=design.convergence.passed&&design.report.valid();
    }catch(const std::exception& error){
        design.convergence.passed=false;
        if(cancel&&cancel()){design.simulation.cancelled=true;design.report.fail("CANCELLED","Convergence verification cancelled");}
        else design.report.fail("CONVERGENCE_EXCEPTION",error.what());
    }
}
}
