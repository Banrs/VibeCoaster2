#include "coaster/coaster.hpp"
#include "simulation_internal.hpp"
#include <stdexcept>

namespace coaster {
ValidationReport validateSimulationTargets(const SimulationResult& simulation,const Targets& targets,const Limits& limits){
    ValidationReport report;
    const auto& m=simulation.metrics;
    for(double v:{m.maxSpeed,m.exposure10Seconds,m.minVerticalG,m.maxVerticalG,m.maxLateralG,m.maxLongitudinalG,m.maxJerkGps}){
        if(!std::isfinite(v)){report.fail("NONFINITE_METRIC","A required simulated target or force metric is nonfinite");return report;}
    }
    auto minimum=[&](const char* code,double value,double target){
        if(!std::isfinite(value)||value+1e-6<target)report.fail(code,"Measured result misses requested target",0,value,target);
    };
    // The speed dial is a setpoint the ride must land on, not a floor to clear:
    // a layout that runs 12% fast missed the dial as surely as one that runs slow.
    // The height dial can force the issue, though - a train launched to 180 km/h
    // and dropped from the dialled height arrives faster than that whatever the
    // layout does - so the ceiling is whichever of the two dials demands more.
    // The floor stays the speed dial alone, which is what holds the ride on it.
    // This is deliberately computed from the dials rather than from a measured
    // height: the convergence passes validate a bare simulation whose metrics
    // have not been through evaluateTargets, so anything it reads there is zero.
    constexpr double launchReference=50;
    const double forcedByHeight=std::sqrt(launchReference*launchReference+2*gravity*std::max(0.,targets.height));
    if(!std::isfinite(m.maxSpeed)||m.maxSpeed<targets.speed*.99)
        report.fail("SPEED_TARGET","Measured top speed misses the requested speed",0,m.maxSpeed,targets.speed);
    else if(m.maxSpeed>std::max(targets.speed,forcedByHeight)*1.01)
        report.fail("SPEED_TARGET","Measured top speed exceeds the requested speed beyond its tolerance",0,m.maxSpeed,targets.speed);
    if(!std::isfinite(m.launchTo180)||m.launchTo180>targets.launchSeconds)
        report.fail("LAUNCH_TARGET","Measured 0-180 km/h launch exceeds target",0,m.launchTo180,targets.launchSeconds);
    if(targets.requireIntensity){
        if(!std::isfinite(targets.referenceExposure)||targets.referenceId.empty())
            report.fail("REFERENCE_UNAVAILABLE","I305 ten-second force trace benchmark has not been calibrated; all-record claim unavailable");
        else minimum("INTENSITY_TARGET",m.exposure10Seconds,targets.referenceExposure*1.1);
    }
    if(m.maxVerticalG>limits.maxVerticalG)report.fail("VERTICAL_FORCE","Positive rider force exceeds provisional envelope",0,m.maxVerticalG,limits.maxVerticalG);
    if(m.minVerticalG<limits.minVerticalG)report.fail("VERTICAL_FORCE","Negative rider force exceeds provisional envelope",0,m.minVerticalG,limits.minVerticalG);
    if(m.maxLateralG>limits.maxLateralG)report.fail("LATERAL_FORCE","Rider lateral force exceeds provisional envelope",0,m.maxLateralG,limits.maxLateralG);
    if(m.maxLongitudinalG>limits.maxLongitudinalG)report.fail("LONGITUDINAL_FORCE","Rider longitudinal force exceeds provisional envelope",0,m.maxLongitudinalG,limits.maxLongitudinalG);
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

ValidationReport compareSimulationConvergence(const SimulationResult& coarse,const SimulationResult& fine,const Limits& limits,ConvergenceAssessment& assessment){
    ValidationReport report;
    assessment.performed=true;assessment.passed=false;assessment.metrics.clear();
    assessment.maxSpeedRelativeError=0;assessment.maxForceRelativeError=0;
    if(coarse.cancelled||fine.cancelled){report.fail("CANCELLED","Convergence simulation cancelled");return report;}
    if(!coarse.completed||!fine.completed||!coarse.report.valid()||!fine.report.valid()){
        report.fail("CONVERGENCE_SIMULATION","Both simulation resolutions must complete without numerical errors");return report;
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
    }
    assessment.passed=report.valid();return report;
}

void verifyConvergenceWith(Design& design,const std::function<SimulationResult()>& fineReplay,Cancel cancel){
    design.convergence={};
    design.convergence.coarseStep=design.request.simulationStep;
    design.convergence.fineStep=design.request.simulationStep*.5;
    if(cancel&&cancel()){design.simulation.cancelled=true;design.report.fail("CANCELLED","Convergence verification cancelled");return;}
    if(!design.report.valid()||!design.simulation.completed||!design.simulation.report.valid()||design.simulation.cancelled)return;
    try{
        const auto fine=fineReplay();
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
void verifyConvergence(Design& design,Cancel cancel){
    verifyConvergenceWith(design,[&]{return simulate(design.track,design.operations,design.request.train,design.request.simulationStep*.5,cancel);},cancel);
}
}
