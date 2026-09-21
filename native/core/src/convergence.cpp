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
    // Height conflicts must be solved or rejected, never hidden by a speed allowance.
    if(!std::isfinite(m.maxSpeed)||m.maxSpeed<targets.speed*.99)
        report.fail("SPEED_TARGET","Measured top speed misses the requested speed",0,m.maxSpeed,targets.speed);
    else if(m.maxSpeed>targets.speed*1.01)
        report.fail("SPEED_TARGET","Measured top speed exceeds the requested speed beyond its tolerance",0,m.maxSpeed,targets.speed);
    if(!std::isfinite(m.launchTo180)||m.launchTo180>targets.launchSeconds)
        report.fail("LAUNCH_TARGET","Measured 0-180 km/h launch exceeds target",0,m.launchTo180,targets.launchSeconds);
    if(targets.requireIntensity){
        if(!std::isfinite(targets.referenceExposure)||targets.referenceId.empty())
            report.fail("REFERENCE_UNAVAILABLE","The requested ten-second exposure comparison has no calibrated reference");
        else minimum("INTENSITY_TARGET",m.exposure10Seconds,targets.referenceExposure);
    }
    auto forcePeak=[&](const char* code,const char* axis,double magnitude,double nominal){
        if(magnitude<=nominal)return;
        if(magnitude<nominal*1.01)
            report.warnings.push_back(std::string(axis)+" peak exceeds nominal by "+std::to_string(100*(magnitude/nominal-1))+"%; within the strictly-below-1% project allowance. ASTM limits are assessed separately.");
        else report.fail(code,std::string(axis)+" peak reaches or exceeds the one-percent project allowance",0,magnitude,nominal);
    };
    forcePeak("VERTICAL_FORCE","Positive Gz",m.maxVerticalG,limits.maxVerticalG);
    forcePeak("VERTICAL_FORCE","Negative Gz",-m.minVerticalG,-limits.minVerticalG);
    forcePeak("LATERAL_FORCE","Absolute Gy",m.maxLateralG,limits.maxLateralG);
    forcePeak("LONGITUDINAL_FORCE","Absolute Gx",m.maxLongitudinalG,limits.maxLongitudinalG);
    if(m.maxJerkGps>limits.maxJerkGps)report.fail("FORCE_TRANSITION","Vertical force transition exceeds provisional envelope",m.maxJerkDistance,m.maxJerkGps,limits.maxJerkGps);
    if(!std::isfinite(m.maxEnergyResidual)||m.maxEnergyResidual>.5)report.fail("ENERGY_RESIDUAL","Finite-train energy balance exceeds 0.5 J/kg numerical allowance",0,m.maxEnergyResidual,.5);
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
    for(const auto& assessment:simulation.acceleration){
        if(!assessment.performed)continue; // Synthetic metric fixtures are not complete Design acceptance.
        for(const auto& finding:assessment.diagnostics){
            auto frame=std::lower_bound(simulation.frames.begin(),simulation.frames.end(),finding.startTimeSeconds,[](const Frame& f,double time){return f.time<time;});
            const double distance=frame==simulation.frames.end()?0:frame->distance;
            report.fail("F2291_25_"+finding.rule,"F2291-25 "+finding.clause+", seat "+std::to_string(finding.seatIndex)+", "+finding.axis+finding.sign+", t="+std::to_string(finding.startTimeSeconds)+".."+std::to_string(finding.endTimeSeconds)+" s",distance,finding.actual,finding.limit);
        }
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
    auto check=[&](std::string name,double a,double b,double fraction,bool summarizeForce=true){
        const double scale=std::max(1.,std::abs(b));
        const double difference=std::abs(a-b),tolerance=fraction*scale;
        const bool finiteValues=std::isfinite(a)&&std::isfinite(b)&&std::isfinite(difference)&&std::isfinite(tolerance);
        const double error=finiteValues?difference/scale:std::numeric_limits<double>::infinity();
        assessment.metrics.push_back({name,a,b,difference,tolerance});
        if(name=="maxSpeed")assessment.maxSpeedRelativeError=std::max(assessment.maxSpeedRelativeError,error);
        else if(summarizeForce)assessment.maxForceRelativeError=std::max(assessment.maxForceRelativeError,error);
        if(!finiteValues||!(difference<tolerance))
            report.fail("CONVERGENCE_METRIC",name+" differs beyond the required half-step tolerance",0,error,fraction);
    };
    const auto& a=coarse.metrics;const auto& b=fine.metrics;
    check("maxSpeed",a.maxSpeed,b.maxSpeed,.01,false);
    check("minVerticalG",a.minVerticalG,b.minVerticalG,.02);
    check("maxVerticalG",a.maxVerticalG,b.maxVerticalG,.02);
    check("maxLateralG",a.maxLateralG,b.maxLateralG,.02);
    check("maxLongitudinalG",a.maxLongitudinalG,b.maxLongitudinalG,.02);
    check("exposure10Seconds",a.exposure10Seconds,b.exposure10Seconds,.02);
    check("maxVerticalRateGps",a.maxJerkGps,b.maxJerkGps,.02);
    check("launchTo180",a.launchTo180,b.launchTo180,.001,false);
    check("duration",a.duration,b.duration,.001,false);
    check("driveWorkPerMass",a.driveWorkPerMass,b.driveWorkPerMass,.002,false);
    check("brakeWorkPerMass",a.brakeWorkPerMass,b.brakeWorkPerMass,.002,false);
    check("lossWorkPerMass",a.lossWorkPerMass,b.lossWorkPerMass,.002,false);
    const char* names[]={"front","middle","rear"};const char* axes[]={"vertical","lateral","longitudinal"};
    for(int seat=0;seat<3;++seat){
        std::string prefix=names[seat];
        check(prefix+".exposure10Seconds",a.seats[seat].exposure10Seconds,b.seats[seat].exposure10Seconds,.02);
        check(prefix+".inertialJerk",a.seats[seat].maxInertialJerk,b.seats[seat].maxInertialJerk,.02,false);
        check(prefix+".angularVelocity",a.seats[seat].maxAngularVelocity,b.seats[seat].maxAngularVelocity,.02,false);
        check(prefix+".angularAcceleration",a.seats[seat].maxAngularAcceleration,b.seats[seat].maxAngularAcceleration,.02,false);
        check(prefix+".angularJerk",a.seats[seat].maxAngularJerk,b.seats[seat].maxAngularJerk,.02,false);
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
        auto fine=fineReplay();
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
            // Bare simulation omits geometry metrics; both replays use the same hill.
            fine.metrics.heightAboveStation=design.simulation.metrics.heightAboveStation;
            fine.metrics.maxGroundHeight=design.simulation.metrics.maxGroundHeight;
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
