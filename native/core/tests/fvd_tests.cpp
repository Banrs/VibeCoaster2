#include "coaster/fvd.hpp"
#include <iostream>
#include <stdexcept>
#include <future>
#include <source_location>
using namespace coaster;
namespace {
int checks=0;
void check(bool condition,const char* message){++checks;if(!condition)throw std::runtime_error(message);}
void near(double value,double expected,double tolerance,const char* message){check(std::isfinite(value)&&std::abs(value-expected)<=tolerance,message);}
void good(const FvdResult& r,const std::source_location call=std::source_location::current()){
    if(!r.report.valid()){
        std::cerr<<call.function_name()<<':'<<call.line()<<'\n';
        for(const auto& e:r.report.errors)std::cerr<<e.code<<": "<<e.message<<'\n';
        std::cerr<<"residual normal="<<r.assessment.maxNormalResidualG<<" lateral="<<r.assessment.maxLateralResidualG<<" roll="<<r.assessment.maxRollResidualRadPerSecond<<" distance="<<r.assessment.endDistanceError<<" speed="<<r.assessment.endSpeedError<<'\n';
    }
    check(r.integrated&&r.canonicalBuilt&&!r.cancelled&&r.report.valid()&&r.assessment.performed&&r.assessment.passed,"Section integrates, fits and passes its sampled point replay");
}
FvdRequest constant(double normal,double lateral,double duration=1){FvdRequest r;r.controls={{0,normal,lateral,0},{duration,normal,lateral,0}};return r;}
FvdRequest halfStep(FvdRequest r){r.step*=.5;r.maxSamples=size_t(std::ceil(r.controls.back().time/r.step))+r.controls.size();return r;}
void checkPointLosses(){
    for(int mode:{0,1,2}){
        FvdRequest r;r.position={0,0,0};r.speed=75;r.step=.005;
        r.controls={{0,1,0,0},{8,1,0,0}};
        r.rollingAcceleration=mode==1?0:.1;
        r.dragAccelerationCoefficient=mode==0?0:.00016;
        double speed,displacement;
        if(mode==0){
            speed=75-.1*8;displacement=75*8-.5*.1*8*8;
        }else if(mode==1){
            speed=75/(1+.00016*75*8);displacement=std::log(1+.00016*75*8)/.00016;
        }else{
            const double a=.1,b=.00016;
            const double initial=std::atan(75*std::sqrt(b/a));
            const double angle=initial-std::sqrt(a*b)*8;
            speed=std::sqrt(a/b)*std::tan(angle);
            displacement=std::log(std::cos(angle)/std::cos(initial))/b;
        }
        auto q=designFvdSection(r);good(q);const auto& end=q.samples.back();
        near(end.speed,speed,1e-8,"Analytic rolling/drag speed");
        near(end.position.x,displacement,1e-7,"Analytic rolling/drag displacement");
        near(end.dissipatedWorkPerMass,.5*(75*75-speed*speed),1e-7,"Analytic dissipated work");
        near(q.assessment.maxEnergyDrift,0,1e-7,"Mechanical energy plus work stays conserved");
        near(end.position.z,0,1e-10,"Tangential losses do not alter straight-track elevation");
        near(norm(end.curvature),0,1e-10,"Tangential losses do not introduce curvature");
    }
    FvdRequest invalid;invalid.dragAccelerationCoefficient=-.001;
    auto q=designFvdSection(invalid);
    check(!q.report.valid()&&q.report.errors.front().code=="FVD_INPUT","Negative loss coefficient rejected");
}
struct ForceTiming {double longestNegative{},minZeroToTwo{100};size_t negativeEvents{};};
ForceTiming timing(const std::vector<std::pair<double,double>>& loads){
    ForceTiming out;double negativeStart=0,lastZero=0;bool recovering=false;
    for(size_t i=1;i<loads.size();++i){
        const auto [t0,g0]=loads[i-1];const auto [t1,g1]=loads[i];
        const auto crossing=[&](double force){return t0+(t1-t0)*(force-g0)/(g1-g0);};
        if(g0>=0&&g1<0)negativeStart=crossing(0);
        if(g0<0&&g1>=0){
            lastZero=crossing(0);recovering=true;++out.negativeEvents;
            out.longestNegative=std::max(out.longestNegative,lastZero-negativeStart);
        }
        if(recovering&&g0<2&&g1>=2){out.minZeroToTwo=std::min(out.minZeroToTwo,crossing(2)-lastZero);recovering=false;}
    }
    return out;
}
void checkNormalControlSlopes(){
    FvdRequest linear;linear.speed=50;linear.controls={{0,1,0,0,2},{1,3,0,0,2},{2,5,0,0,2}};
    const auto q=designFvdSection(linear);good(q);
    for(const auto& s:q.samples){
        const double normal=dot(s.curvature*(s.speed*s.speed)+Vec3{0,0,gravity},s.up)/gravity;
        near(normal,1+2*s.time,1e-9,"Equal secant slopes reproduce an exact linear load through the middle control");
    }
    const auto rate=[&](double distance){const auto k=sampleKinematics(q.track,distance);const auto& s=k.sample;
        const double v2=linear.speed*linear.speed-2*gravity*(s.position.z-linear.position.z),v=std::sqrt(v2),a=-gravity*s.tangent.z;
        return (dot(s.curvature*(2*v*a)+k.curvatureS*(v2*v),s.up)+dot(s.curvature*v2+Vec3{0,0,gravity},k.upS*v))/gravity;};
    const auto joint=std::lower_bound(q.samples.begin(),q.samples.end(),1.,[](const auto& s,double time){return s.time<time;});
    for(double offset:{-.01,0.,.01})near(rate(joint->distance+offset),2,2e-4,"Canonical force slope stays continuous and nonzero through the control knee");
    near((rate(joint->distance+.01)-rate(joint->distance-.01))*joint->speed/.02,0,.02,
        "Independent canonical force second derivative remains zero through the linear-load joint");
    FvdRequest knee;knee.speed=50;knee.step=.001;knee.controls={{0,1,0,0},{1,2,0,0,.75},{3,3,0,0}};
    const auto curved=designFvdSection(knee);good(curved);
    const auto at=std::lower_bound(curved.samples.begin(),curved.samples.end(),1.,[](const auto& s,double time){return s.time<time;});
    const size_t center=size_t(at-curved.samples.begin());const double dt=knee.step;
    const auto load=[&](size_t i){const auto& s=curved.samples[i];return dot(s.curvature*(s.speed*s.speed)+Vec3{0,0,gravity},s.up)/gravity;};
    const double value=load(center);
    near(value,2,1e-9,"Asymmetric C2 load curve reaches its requested knee value");
    for(int side:{-1,1}){
        const auto next=[&](int n){return load(size_t(int(center)+side*n));};
        near(side*(-3*value+4*next(1)-next(2))/(2*dt),.75,1e-4,"Load slopes agree on both sides of an asymmetric knee");
        near((2*value-5*next(1)+4*next(2)-next(3))/(dt*dt),0,.002,"Load second derivatives independently tend to zero on both sides of the knee");
    }
    for(int invalid=0;invalid<3;++invalid){
        auto r=linear;
        if(invalid==0)r.controls[1].normalRateGps=std::nan("");
        if(invalid==1)r.controls[1].normalRateGps=-1;
        if(invalid==2)r.controls[1].normalRateGps=5;
        const auto bad=designFvdSection(r);
        check(!bad.integrated&&!bad.report.valid()&&bad.report.errors.front().code=="FVD_INPUT",
            "Invalid or nonmonotone normal-force slope rejects before integration");
    }
}
void checkTallHill(){
    const auto source=[&](double height,double speed,bool lossy,double normal=3.35,double crest=-.025,double pulse=1.6,double ramp=1.2){
        FvdTallHillRequest r;r.height=height;r.speed=speed;r.normalG=normal;r.crestG=crest;r.crestPulseSeconds=pulse;r.rampSeconds=ramp;
        if(lossy){
            TrainConfig t;r.rollingAcceleration=gravity*t.rollingResistance;
            r.dragAccelerationCoefficient=.5*t.airDensity*t.dragCdA/(t.cars*t.carMass);
        }
        auto q=designFvdTallHill(r);
        if(!q.section.report.valid())std::cerr<<"Tall request: "<<height<<" m, "<<speed<<" m/s, "<<normal<<"/"<<crest<<" g, "<<pulse<<" s, losses="<<lossy<<'\n';
        good(q.section);
        near(q.height,height,1e-4,"Tall hill closes requested height without spatial scaling");
        near(q.exit.position.z,0,1e-5,"Independent descent closes inlet elevation");
        near(norm(q.exit.forward-Vec3{1,0,0}),0,1e-6,"Independent descent closes level heading");
        near(norm(q.section.samples.front().curvature),0,1e-8,"Tall hill has a zero-curvature inlet");
        near(norm(q.exit.curvature),0,1e-8,"Tall hill has a zero-curvature outlet");
        near(.5*q.exit.speed*q.exit.speed+q.exit.dissipatedWorkPerMass,.5*speed*speed,1e-5,
            "Exit energy accounts for dissipated work");
        if(lossy){
            check(q.exit.speed<speed-.1&&q.exit.dissipatedWorkPerMass>1,"Lossy descent retains lower exit energy");
        }else{
            near(q.exit.speed,speed,1e-5,"Default-zero hill retains gravity-only exit energy");
            check(q.exit.dissipatedWorkPerMass==0,"Default loss coefficients are zero");
        }
        std::vector<std::pair<double,double>> sourceLoads,replayLoads;double lowest=1,highest=1;
        const auto force=[](const Vec3& k,const Vec3& up,double velocity){return dot(k*(velocity*velocity)+Vec3{0,0,gravity},up)/gravity;};
        for(size_t i=0;i<q.section.samples.size();++i){
            const auto& at=q.section.samples[i];const double g=force(at.curvature,at.up,at.speed);
            sourceLoads.push_back({at.time,g});lowest=std::min(lowest,g);highest=std::max(highest,g);
            near(.5*at.speed*at.speed+gravity*at.position.z+at.dissipatedWorkPerMass,.5*speed*speed,1e-6,
                "Every tall-source state accounts for actual dissipated work");
        }
        near(lowest,crest,1e-8,"Short apex pulse preserves the requested negative peak");
        near(highest,normal,1e-8,"Height shooting preserves requested positive peak instead of scaling forces");
        for(size_t i=2;i+2<q.authoring.controls.size();++i){
            const auto& a=q.authoring.controls[i-1];const auto& b=q.authoring.controls[i];
            check(a.normalG!=b.normalG||a.normalG>r.unloadG,
                "Continuous unloading/recovery has no held positive shoulder below the pull-in load");
        }
        // Independent RK4 integrates distance and speed from the canonical
        // tangent and explicit losses. No authored velocity or force is replayed.
        const auto derivative=[&](double at,double velocity){const auto frame=q.section.track.sample(at);
            return std::pair{velocity,-gravity*frame.tangent.z-r.rollingAcceleration-r.dragAccelerationCoefficient*velocity*velocity};};
        constexpr double dt=1./800;double distance=0,velocity=speed,time=0;
        while(distance+velocity*dt*1.1<q.section.track.length){
            const auto at=q.section.track.sample(distance);replayLoads.push_back({time,force(at.curvature,at.up,velocity)});
            const auto a=derivative(distance,velocity),b=derivative(distance+a.first*dt*.5,velocity+a.second*dt*.5);
            const auto c=derivative(distance+b.first*dt*.5,velocity+b.second*dt*.5),d=derivative(distance+c.first*dt,velocity+c.second*dt);
            distance+=(a.first+2*b.first+2*c.first+d.first)*dt/6;velocity+=(a.second+2*b.second+2*c.second+d.second)*dt/6;time+=dt;
        }
        const auto authored=timing(sourceLoads),replayed=timing(replayLoads);
        for(const auto& measured:{authored,replayed}){
            check(measured.negativeEvents==1,"Tall hill has one intentional negative crest passage");
            near(measured.longestNegative,r.crestPulseSeconds,.002,"Crest-phase intent defines actual zero-to-zero negative duration");
            check(measured.minZeroToTwo>=.133,"Actual tall-hill recovery passes the historical zero-to-two-g duration case");
        }
        near(replayed.longestNegative,authored.longestNegative,.002,"Loss-aware canonical replay reproduces signed negative duration");
        near(replayed.minZeroToTwo,authored.minZeroToTwo,.002,"Loss-aware canonical replay reproduces zero-to-two-g recovery");
        const auto apex=std::max_element(q.section.samples.begin(),q.section.samples.end(),[](const auto& a,const auto& b){return a.position.z<b.position.z;});
        near(apex->forward.z,0,1e-7,"Geometric apex closes at the pulse center");
        near(force(apex->curvature,apex->up,apex->speed),crest,1e-7,"Apex load is measured from geometry at the actual apex speed");
        if(normal==6){
            auto refined=designFvdSection(halfStep(q.authoring));good(refined);
            near(norm(refined.samples.back().position-q.exit.position),0,1e-6,"High-peak source survives half-step integration without changing geometry or energy");
        }
    };
    source(230,75,false);
    source(230,75,true);
    source(280,90,true); // Upper height/speed boundary remains a real solved source.
    source(220,75,true,3.95,-.25);
    source(240,80,true,3.95,-.25);
    source(280,90,true,3.95,-.25);
    source(220,75,true,6,-2,1.6,1.0); // Stronger positive intent needs a shorter ramp impulse for this height/energy.
    source(240,85,true,6,-2,1.6,1.0); // Source-domain capability; actual rider duration/combination limits remain separate.
    source(240,85,true,6,-2.8,1.0,1.0); // A finite pulse at the requested upper negative peak.
    source(240,85,true,3.95,-.25,4.0); // Longer negative histories remain authorable; the separate rider envelope assesses their recovery.
    FvdTallHillRequest impossible;impossible.height=280;impossible.speed=75;
    impossible.rollingAcceleration=gravity*.002;
    impossible.dragAccelerationCoefficient=.5*1.225*2.4/9000;
    auto failure=designFvdTallHill(impossible);
    check(!failure.section.report.valid()&&failure.section.report.errors.front().code=="FVD_TALL_ENERGY",
        "Even ideal vertical-ascent energy cannot reach the requested lossy height");
    auto excessiveImpulse=impossible;excessiveImpulse.height=220;excessiveImpulse.normalG=6;excessiveImpulse.crestG=-2;
    failure=designFvdTallHill(excessiveImpulse);
    check(!failure.section.integrated&&!failure.section.report.valid()&&failure.section.report.errors.front().code=="FVD_TALL_ASCENT",
        "A force program whose minimum ramp impulse overshoots the requested height rejects instead of silently shortening its ramps");
    for(int variant=0;variant<6;++variant){
        FvdTallHillRequest invalid;
        if(variant==0)invalid.height=281;if(variant==1)invalid.normalG=6.01;if(variant==2)invalid.crestG=-2.81;
        if(variant==3)invalid.crestPulseSeconds=6.01;if(variant==4)invalid.unloadG=0;if(variant==5)invalid.rampSeconds=std::nan("");
        failure=designFvdTallHill(invalid);
        check(!failure.section.canonicalBuilt&&!failure.section.report.valid()&&
            failure.section.report.errors.front().code=="FVD_TALL_INPUT","Out-of-family tall geometry, force or pulse duration rejected");
    }
    for(int stop:{1,1000}){
        int polls=0;failure=designFvdTallHill({},[&]{return ++polls>=stop;});
        check(failure.section.cancelled&&!failure.section.report.valid()&&
            failure.section.report.errors.front().code=="CANCELLED","Cancellation preserved before and during source shooting");
    }
}
void checkAirtimeChain(){
    FvdAirtimeRequest request;request.portRampSeconds=.6*request.speed/65;request.hills={{3.5,-.5,.32,1.1,1.4},{4.3,-.9,.34,1.0,1.6},{3.8,-.7,.5,1.2,1.8}};
    double previousSpan=0;
    for(double speed:{60.,65.,68.}){
        request.speed=speed;request.portRampSeconds=.6*speed/65;auto q=designFvdAirtime(request);good(q.section);
        check(q.hills.size()==3,"The chain exposes every real source apex and valley frame");
        check(q.span>previousSpan,"A faster source produces a larger chain without spatial scaling");previousSpan=q.span;
        near(norm(q.section.samples.front().curvature)+norm(q.section.samples.back().curvature),0,1e-8,
            "Only the chain's outside ports unload to zero curvature");
        for(size_t i=0;i<q.hills.size();++i){
            const auto& h=q.hills[i];const auto& intent=request.hills[i];
            near(h.entry.position.z,0,1e-5,"Hill entry remains on the shared valley datum");
            near(h.exit.position.z,0,1e-5,"An independently solved descent returns to the valley datum");
            near(h.apex.forward.z,0,1e-7,"Each apex closes horizontal pitch through force integration");
            check(h.apex.position.z>35&&h.apex.position.z<140,"High-speed nonrecord hills retain meaningful physical scale");
            near(dot(h.apex.curvature*(h.apex.speed*h.apex.speed)+Vec3{0,0,gravity},h.apex.up)/gravity,
                intent.crestG,1e-7,"Each geometric apex supplies its individual negative-force intent");
            near(h.exit.speed,speed,1e-5,"Valley energy follows gravity without a hidden speed reset");
            if(i+1==q.hills.size())continue;
            const auto& next=q.hills[i+1].entry;
            near(norm(h.exit.position-next.position)+norm(h.exit.forward-next.forward)+norm(h.exit.curvature-next.curvature),
                0,1e-12,"Adjacent hills share one integrated valley frame and curvature");
            for(double offset:{-2.,0.,2.}){
                const auto at=sampleKinematics(q.section.track,h.exit.distance+offset);
                const double squared=speed*speed-2*gravity*at.sample.position.z;
                const double normal=dot(at.sample.curvature*squared+Vec3{0,0,gravity},at.sample.up)/gravity;
                check(normal>3.3&&norm(at.sample.curvature)>.004,"The valley and both sides remain loaded, not a flat 1g reset");
                check(offset==0||at.sample.tangent.z*offset>0,"A shared valley continuously changes from descending to ascending");
                check(norm(at.curvatureS)<.0001,"Canonical valley curvature has no derivative spike");
            }
            const auto left=sampleKinematics(q.section.track,h.exit.distance-.001),right=sampleKinematics(q.section.track,h.exit.distance+.001);
            const Vec3 predicted=left.sample.curvature+(left.curvatureS+right.curvatureS)*.001;
            check(norm(predicted-right.sample.curvature)<1e-8&&norm(left.curvatureS-right.curvatureS)<1e-6,
                "Canonical third-order geometry remains continuous through the loaded valley");
        }
        for(size_t i=0;i<q.section.samples.size();i+=23){
            const auto& at=q.section.samples[i];
            near(at.speed*at.speed+2*gravity*at.position.z,speed*speed,1e-5,"Independent chain mechanical-energy oracle");
        }
        std::vector<std::pair<double,double>> sourceLoads,replayLoads;
        for(const auto& at:q.section.samples)sourceLoads.push_back({at.time,
            dot(at.curvature*(at.speed*at.speed)+Vec3{0,0,gravity},at.up)/gravity});
        // Independent RK4 replay advances only arc length using mechanical
        // energy on canonical geometry. It uses no authoring samples/control
        // force as measured loads and stops inside the final positive guard.
        const auto velocity=[&](double at){return std::sqrt(speed*speed-2*gravity*q.section.track.sample(at).position.z);};
        constexpr double dt=1./800;double distance=0,time=0;
        while(distance+speed*dt*1.1<q.section.track.length){
            const auto at=q.section.track.sample(distance);const double v=velocity(distance);
            replayLoads.push_back({time,dot(at.curvature*(v*v)+Vec3{0,0,gravity},at.up)/gravity});
            const double b=velocity(distance+dt*v*.5),c=velocity(distance+dt*b*.5),d=velocity(distance+dt*c);
            distance+=dt*(v+2*b+2*c+d)/6;time+=dt;
        }
        const auto authored=timing(sourceLoads),replayed=timing(replayLoads);
        for(const auto& measured:{authored,replayed}){
            check(measured.negativeEvents==3&&measured.longestNegative>.5&&measured.longestNegative<2.0,
                "Actual signed negative exposure stays short at every apex, including transitions");
            check(measured.minZeroToTwo>=.133,"Actual recovery from zero to two g meets the historical transition-duration case");
        }
        near(replayed.longestNegative,authored.longestNegative,.002,"Independent canonical replay reproduces signed negative duration");
        near(replayed.minZeroToTwo,authored.minZeroToTwo,.002,"Independent canonical replay reproduces zero-to-two-g transition duration");
    }
    request.hills.resize(2);request.speed=65;
    auto first=designFvdAirtime(request),repeat=designFvdAirtime(request);good(first.section);good(repeat.section);
    check(first.section.samples.size()==repeat.section.samples.size(),"Identical chain intent has deterministic sample count");
    for(size_t i=0;i<first.section.samples.size();i+=29)
        check(norm(first.section.samples[i].position-repeat.section.samples[i].position)==0,"Identical chain intent is deterministic");
    auto refined=designFvdSection(halfStep(first.authoring));good(refined);
    near(norm(refined.samples.back().position-first.section.samples.back().position),0,1e-6,
        "The complete chain survives actual force integration at half the time step");
    for(double speed:{60.,68.}){
        auto boundary=request;boundary.speed=speed;good(designFvdAirtime(boundary).section);
    }
    request.hills[1].crestG=-.6;
    auto varied=designFvdAirtime(request);good(varied.section);
    near(varied.hills[0].apex.position.z,first.hills[0].apex.position.z,1e-9,"A later hill's force change preserves earlier authoring");
    check(std::abs(varied.hills[1].apex.position.z-first.hills[1].apex.position.z)>.3&&std::abs(varied.span-first.span)>5&&
        norm(varied.hills[1].apex.curvature-first.hills[1].apex.curvature)>.0005,
        "Individual pulse force changes actual crest curvature, height and span");
    for(int invalid=0;invalid<6;++invalid){
        auto bad=request;
        if(invalid==0)bad.hills.clear();if(invalid==1)bad.hills.resize(4);
        if(invalid==2)bad.hills[1].pushG=6.01;if(invalid==3)bad.hills[1].crestG=std::nan("");
        if(invalid==4)bad.hills[1].crestPulseSeconds=6.01;if(invalid==5)bad.unloadG=0;
        auto rejected=designFvdAirtime(bad);
        check(!rejected.section.integrated&&!rejected.section.report.valid()&&rejected.section.report.errors.front().code=="FVD_AIRTIME_INPUT",
            "Invalid count or later-hill force rejects before source integration");
    }
    int polls=0;auto cancelled=designFvdAirtime(request,[&]{return ++polls>2000;});
    check(cancelled.section.cancelled&&!cancelled.section.integrated&&!cancelled.section.report.valid(),
        "Cancellation interrupts multi-hill shooting before canonical construction");
}
void checkBankedAirtime(){
    const auto bank=[](const FvdSample& q){const Vec3 up=unit(Vec3{0,0,1}-q.forward*q.forward.z);
        return std::atan2(dot(q.up,cross(q.forward,up)),dot(q.up,up));};
    FvdAirtimeRequest request;request.portRampSeconds=.6*request.speed/65;request.hills={{3.5,-.5,.32,1.1,1.4},{4.3,-.9,.34,1.0,1.6},{3.8,-.7,.5,1.2,1.8}};
    for(size_t i=0;i<3;++i)request.hills[i].bankRadians=(i==2?-25:25)*pi/180;
    for(double speed:{60.,65.,68.}){
        request.speed=speed;request.portRampSeconds=.6*speed/65;auto q=designFvdAirtime(request);good(q.section);
        const auto& end=q.section.samples.back();
        check(q.authoring.twists.size()==6,"Complete authoring retains each actual ascent and descent twist phase");
        check(end.position.y>100&&std::abs(end.forward.y)>.01,"Integrated bank returns a real free lateral displacement and heading");
        check(q.span>end.position.x+10&&q.span<q.section.track.length,"Banked span measures horizontal path arc instead of X displacement");
        near(end.position.z,0,1e-5,"Spatial chain closes the valley datum");
        near(std::abs(end.forward.z)+norm(end.up-Vec3{0,0,1}),0,1e-6,"Spatial chain closes level upright ports without resetting heading");
        size_t reversals=0;int previous=0;std::vector<std::pair<double,double>> loads;
        for(const auto& at:q.section.samples){
            const double angle=bank(at);const int sign=angle>.1?1:angle<-.1?-1:0;
            if(sign&&previous&&sign!=previous)++reversals;if(sign)previous=sign;
            const Vec3 specific=at.curvature*(at.speed*at.speed)+at.forward*(-gravity*at.forward.z)+Vec3{0,0,gravity};
            near(dot(specific,cross(at.forward,at.up))/gravity,0,1e-9,"Turning comes from banked normal force, with no fictitious body lateral compensation");
            loads.push_back({at.time,dot(specific,at.up)/gravity});
            near(at.speed*at.speed+2*gravity*at.position.z,speed*speed,1e-5,"Spatial source independently conserves energy");
        }
        check(reversals==3,"Three genuine bank-direction reversals survive joint 3D authoring");
        const auto exposure=timing(loads);
        check(exposure.negativeEvents==3&&exposure.longestNegative<2&&exposure.minZeroToTwo>=.133,
            "Banking preserves short actual negative pulses and gradual zero-to-two-g recovery");
        for(size_t i=0;i<q.hills.size();++i){
            const auto& h=q.hills[i];near(h.apex.forward.z,0,1e-7,"Each banked ejector pulse remains at the geometric apex");
            near(h.exit.position.z,0,1e-5,"Each spatial descent closes its actual loaded valley height");
            if(i+1==q.hills.size())continue;
            near(bank(h.exit),-request.hills[i+1].bankRadians,1e-7,"Shared valley closes the next hill's opposing bank intention");
            check(norm(h.exit.curvature)>.004,"Banked internal valley remains curved and loaded");
            const auto left=sampleKinematics(q.section.track,h.exit.distance-.001),right=sampleKinematics(q.section.track,h.exit.distance+.001);
            check(norm(left.curvatureS-right.curvatureS)<1e-6&&norm(left.upS-right.upS)<1e-6,
                "Curvature and orientation derivatives remain continuous across banked valley joins");
        }
        if(speed!=65)continue;
        auto refined=designFvdSection(halfStep(q.authoring));good(refined);
        near(norm(refined.samples.back().position-end.position),0,1e-6,"Stored force and twist intent reproduces the whole spatial source at half step");
        auto mirrorRequest=request;for(auto& h:mirrorRequest.hills)h.bankRadians=-h.bankRadians;
        auto mirror=designFvdAirtime(mirrorRequest);good(mirror.section);const auto& other=mirror.section.samples.back();
        near(norm(other.position-Vec3{end.position.x,-end.position.y,end.position.z}),0,1e-5,"Mirrored twist yields a physically mirrored free exit");
        near(norm(other.forward-Vec3{end.forward.x,-end.forward.y,end.forward.z}),0,1e-7,"Mirrored source preserves the actual reflected exit heading");
    }
    request.hills[1].bankRadians=36*pi/180;
    check(!designFvdAirtime(request).section.report.valid(),"Unsupported bank intent rejects before geometry authoring");
    FvdRequest invalid;invalid.twists={{.2,.8,.3}};invalid.controls[0].rollRate=.1;
    check(!designFvdSection(invalid).report.valid(),"Force-control roll and twist phases cannot have competing ownership");
    for(int variant=0;variant<4;++variant){
        invalid=FvdRequest{};invalid.twists={{.2,.8,.3}};
        if(variant==0)invalid.twists.push_back({.7,.9,.1});if(variant==1)invalid.twists[0].end=2;
        if(variant==2)invalid.twists[0].angle=std::nan("");if(variant==3)invalid.twists[0]={.2,.21,1};
        const auto bad=designFvdSection(invalid);
        check(!bad.integrated&&!bad.report.valid()&&bad.report.errors.front().code=="FVD_INPUT","Malformed or excessive physical twist rejects explicitly");
    }
}
void checkLossyReplay(const FvdResult& source,double rolling,double drag,const std::vector<FvdSample>& checkpoints){
    // Independent spatial RK4: d(v²)/ds = -2g dz/ds -2a_roll -2b_drag*v².
    // It samples only canonical geometry, never source speeds or controls,
    // and lands exactly on each requested source arc coordinate.
    const auto& start=source.samples.front();double distance=0;Vec3 state{start.speed*start.speed,0,0};
    const double energy=.5*state.x+gravity*start.position.z;
    const auto rate=[&](double at,const Vec3& value){const auto frame=source.track.sample(at);
        check(value.x>0,"Independent lossy replay retains positive squared speed without clamping");
        const double dissipation=rolling+drag*value.x;
        return Vec3{-2*gravity*frame.tangent.z-2*dissipation,1/std::sqrt(value.x),dissipation};};
    for(const auto& expected:checkpoints){
        const size_t steps=size_t(std::ceil((expected.distance-distance)/.05));
        const double ds=steps?(expected.distance-distance)/steps:0;
        for(size_t i=0;i<steps;++i){
            const auto a=rate(distance,state),b=rate(distance+ds*.5,state+a*(ds*.5));
            const auto c=rate(distance+ds*.5,state+b*(ds*.5)),d=rate(distance+ds,state+c*ds);
            state=state+(a+b*2+c*2+d)*(ds/6);distance+=ds;
        }
        distance=expected.distance;
        near(std::sqrt(state.x),expected.speed,1e-5,"Independent canonical replay reproduces each actual source speed");
        near(state.y,expected.time,1e-5,"Independent canonical replay reproduces each actual source time");
        near(state.z,expected.dissipatedWorkPerMass,1e-5,"Independent canonical replay reproduces each source's dissipated work");
        const auto frame=source.track.sample(distance);
        near(.5*state.x+gravity*frame.position.z+state.z,energy,1e-5,"Independent loss-aware replay closes the physical energy balance");
        const Vec3 actualForce=frame.curvature*state.x+Vec3{0,0,gravity},expectedForce=expected.curvature*(expected.speed*expected.speed)+Vec3{0,0,gravity};
        near(dot(actualForce,frame.up)/gravity,dot(expectedForce,expected.up)/gravity,1e-5,"Independent speed and canonical geometry reproduce actual normal force");
        near(dot(actualForce,frame.right)/gravity,dot(expectedForce,cross(expected.forward,expected.up))/gravity,1e-5,
            "Independent speed and canonical geometry reproduce actual lateral force");
    }
}
void checkLossyAirtime(){
    const TrainConfig train;
    for(size_t count:{size_t(1),size_t(3)})for(double speed:{59.714756,65.}){
        FvdAirtimeRequest request;request.speed=speed;
        request.portRampSeconds=.6*request.speed/65;request.hills={{3.5,-.5,.32,1.1,1.4},{4.3,-.9,.34,1.0,1.6},{3.8,-.7,.5,1.2,1.8}};
        request.hills.resize(count);
        if(count==3)for(size_t i=0;i<count;++i)request.hills[i].bankRadians=(i==2?-25:25)*pi/180;
        request.rollingAcceleration=gravity*train.rollingResistance;
        request.dragAccelerationCoefficient=.5*train.airDensity*train.dragCdA/(train.cars*train.carMass);
        const auto q=designFvdAirtime(request);good(q.section);const auto& end=q.section.samples.back();
        check(end.speed<speed-.5&&end.dissipatedWorkPerMass>10,"Airtime must lose real energy between ports, not reset exit speed to entry");
        near(q.authoring.rollingAcceleration,request.rollingAcceleration,0,"Airtime forwards physical rolling loss to its canonical source");
        near(q.authoring.dragAccelerationCoefficient,request.dragAccelerationCoefficient,0,"Airtime forwards physical drag to its canonical source");
        near(norm(end.curvature)+std::abs(end.position.z)+std::abs(end.forward.z)+norm(end.up-Vec3{0,0,1}),0,1e-5,
            "Lossy ascent/descent still close their actual level upright outer ports");
        double work=0;
        for(const auto& s:q.section.samples){
            check(s.dissipatedWorkPerMass>=work,"Dissipated work cannot reset at an internal hill port");work=s.dissipatedWorkPerMass;
            near(.5*s.speed*s.speed+gravity*s.position.z+work,.5*speed*speed,1e-6,
                "Each lossy hill state conserves mechanical energy plus real dissipated work");
        }
        std::vector<FvdSample> checkpoints;
        for(size_t i=0;i<q.hills.size();++i){
            const auto& h=q.hills[i];const auto& intent=request.hills[i];
            check(h.exit.speed<h.entry.speed&&h.exit.dissipatedWorkPerMass>h.apex.dissipatedWorkPerMass&&
                h.apex.dissipatedWorkPerMass>h.entry.dissipatedWorkPerMass,"Every hill depletes the actual carried state at both apex and exit");
            near(h.apex.forward.z,0,1e-7,"Loss-aware apex is the actual geometric apex");
            near(h.exit.position.z,0,1e-5,"Loss-aware descent returns to its valley datum");
            near(dot(h.apex.curvature*(h.apex.speed*h.apex.speed)+Vec3{0,0,gravity},h.apex.up)/gravity,
                intent.crestG,1e-7,"Loss-aware geometry preserves requested apex force at actual depleted speed");
            checkpoints.push_back(h.entry);checkpoints.push_back(h.apex);checkpoints.push_back(h.exit);
            if(i+1==count)continue;
            const auto& next=q.hills[i+1].entry;
            near(norm(h.exit.position-next.position)+norm(h.exit.forward-next.forward)+norm(h.exit.up-next.up)+
                std::abs(h.exit.speed-next.speed)+std::abs(h.exit.dissipatedWorkPerMass-next.dissipatedWorkPerMass),
                0,1e-12,"Next hill begins from the complete actual prior valley state without an energy reset");
            near(dot(h.exit.curvature*(h.exit.speed*h.exit.speed)+Vec3{0,0,gravity},h.exit.up)/gravity,
                request.hills[i+1].pushG,1e-7,"The depleted internal valley retains its actual requested loading");
            const auto left=sampleKinematics(q.section.track,h.exit.distance-.001),right=sampleKinematics(q.section.track,h.exit.distance+.001);
            check(norm(left.curvatureS-right.curvatureS)<1e-6&&norm(left.upS-right.upS)<1e-6,
                "Lossy internal valley retains continuous geometry and orientation derivatives");
        }
        checkpoints.push_back(end);
        checkLossyReplay(q.section,request.rollingAcceleration,request.dragAccelerationCoefficient,checkpoints);
        const auto refined=designFvdSection(halfStep(q.authoring));good(refined);
        for(const auto& expected:checkpoints){
            const auto actual=std::lower_bound(refined.samples.begin(),refined.samples.end(),expected.time-1e-9,
                [](const FvdSample& s,double t){return s.time<t;});
            check(actual!=refined.samples.end(),"Half-step source retains each real apex/port boundary");
            near(actual->time,expected.time,1e-9,"Half-step source preserves control-boundary times");
            near(norm(actual->position-expected.position)+norm(actual->forward-expected.forward)+norm(actual->up-expected.up),0,1e-6,
                "Half-step loss-aware source reproduces all apex/port geometry and orientation");
            near(actual->speed,expected.speed,1e-6,"Half-step loss-aware source reproduces every apex/port speed");
            near(actual->dissipatedWorkPerMass,expected.dissipatedWorkPerMass,1e-6,"Half-step source retains work accumulated across the whole chain");
        }
    }
    for(int variant=0;variant<3;++variant){
        FvdAirtimeRequest invalid;
        if(variant==0)invalid.rollingAcceleration=-.01;if(variant==1)invalid.dragAccelerationCoefficient=.011;
        if(variant==2)invalid.dragAccelerationCoefficient=std::nan("");
        const auto rejected=designFvdAirtime(invalid);
        check(!rejected.section.integrated&&!rejected.section.report.valid(),"Invalid airtime loss coefficients reject before source integration");
    }
}
void checkCompactAirtimeSimilarity(){
    FvdAirtimeRequest base;base.portRampSeconds=.6;
    base.hills={{4.8,-1.4,.21,.8,1.6,25*pi/180},{4.65,-1.3,.21,.8,1.6,25*pi/180},{4.5,-1.2,.21,.8,1.6,-25*pi/180}};
    const auto squeezed=designFvdAirtime(base);
    check(!squeezed.section.report.valid()&&!squeezed.section.integrated,"A short final hill cannot shrink the explicit exit unload to force a level port");
    base.hills.back().pushHoldSeconds+=base.portRampSeconds*.5;
    const auto reference=designFvdAirtime(base);good(reference.section);
    const TrainConfig train;
    for(double speed:{62.,55.}){
        const double scale=speed/base.speed;auto request=base;
        request.speed=speed;request.guardSeconds*=scale;
        // Algebraically equivalent duration expressions can differ by one ulp.
        request.portRampSeconds=base.portRampSeconds*speed/base.speed;
        for(auto& h:request.hills){h.pushHoldSeconds=h.pushHoldSeconds*speed/base.speed;
            h.crestRampSeconds=h.crestRampSeconds*speed/base.speed;h.crestPulseSeconds*=scale;}
        const auto source=designFvdAirtime(request);good(source.section);
        near(source.span,reference.span*scale*scale,1e-4,"Compact chain footprint follows gravity similarity below the old fixed-duration bounds");
        near(source.section.samples.back().time,reference.section.samples.back().time*scale,1e-5,
            "Compact chain preserves the requested time scale without minimum-duration clamping");
        for(size_t i=0;i<source.hills.size();++i){
            const auto& actual=source.hills[i];const auto& expected=reference.hills[i];
            near(norm(actual.apex.position-expected.apex.position*(scale*scale)),0,1e-4,
                "Every banked compact hill preserves its geometrical size relationship");
            near(actual.apex.speed,expected.apex.speed*scale,1e-5,"Every compact crest retains its proportional physical speed");
            near(norm(actual.apex.forward-expected.apex.forward)+norm(actual.apex.up-expected.apex.up),0,1e-6,
                "The speed-scaled chain retains actual crest direction and bank");
        }
        request.rollingAcceleration=gravity*train.rollingResistance;
        request.dragAccelerationCoefficient=.5*train.airDensity*train.dragCdA/(train.cars*train.carMass);
        const auto lossy=designFvdAirtime(request);good(lossy.section);
        const auto& controls=lossy.authoring.controls;
        near(controls[controls.size()-2].time-controls[controls.size()-3].time,request.portRampSeconds,1e-12,
            "Final physical unload retains its requested duration after valley closure");
        std::vector<FvdSample> checkpoints;
        for(const auto& h:lossy.hills){checkpoints.push_back(h.entry);checkpoints.push_back(h.apex);checkpoints.push_back(h.exit);}
        checkLossyReplay(lossy.section,request.rollingAcceleration,request.dragAccelerationCoefficient,checkpoints);
        check(lossy.section.samples.back().speed<speed,"Actual drag remains in each compact chain; lossless similarity is not imposed on its speed");
        std::vector<std::pair<double,double>> loads;
        for(const auto& at:lossy.section.samples)loads.push_back({at.time,
            dot(at.curvature*(at.speed*at.speed)+Vec3{0,0,gravity},at.up)/gravity});
        const auto phases=timing(loads);
        check(phases.negativeEvents==3,"Speed scaling preserves the three real signed crest passages");
        near(phases.longestNegative,1.6*scale,.002,"Actual negative phase durations remain the explicitly requested durations");
        for(int invalid=0;invalid<3;++invalid){
            auto bad=request;
            if(invalid==0)bad.portRampSeconds=0;
            if(invalid==1)bad.hills[1].pushHoldSeconds=0;
            if(invalid==2)bad.hills[1].crestRampSeconds=0;
            const auto rejected=designFvdAirtime(bad);
            check(!rejected.section.integrated&&!rejected.section.report.valid()&&
                rejected.section.report.errors.front().code=="FVD_AIRTIME_INPUT","Degenerate compact phase durations still reject before integration");
        }
    }
}
void checkFullLoop(){
    const TrainConfig train;
    for(double height:{68.,73.,78.})for(bool lossy:{false,true}){
        FvdLoopRequest request;request.height=height;request.apexSpeed=26;request.normalG=4.1;request.crossingOffset=18;
        if(lossy){request.rollingAcceleration=gravity*train.rollingResistance;
            request.dragAccelerationCoefficient=.5*train.airDensity*train.dragCdA/(train.cars*train.carMass);}
        const auto loop=designFvdLoop(request);good(loop.section);const auto& end=loop.section.samples.back();
        near(loop.apex.position.z,height,1e-5,"Full-loop source reaches its actual height target");
        near(loop.apex.speed,request.apexSpeed,1e-5,"Full-loop apex speed is solved with all preceding losses");
        near(loop.apex.forward.z,0,1e-7,"The source checkpoint is the geometric loop apex");
        check(loop.apex.forward.x<0&&loop.apex.up.z<-.99,"The actual horizontal apex is inverted even when the physical loop plane changes heading");
        near(norm(loop.crossingExit.position-loop.crossingEntry.position-Vec3{0,request.crossingOffset,0}),0,1e-5,
            "Integrated physical twist separates the actual crossing arms, not only the exit port");
        check(end.forward.y<-.2,"Separated loop exposes its real changed heading instead of resetting yaw");
        near(std::abs(end.position.z)+std::abs(end.forward.z)+norm(end.up-Vec3{0,0,1})+norm(end.curvature),0,1e-5,
            "The source returns level and upright with a genuine zero-curvature exit");
        near(loop.loopEntry.distance,request.portLength,1e-5,"Entry guard retains actual specified path length with losses");
        near(end.distance-loop.loopExit.distance,request.portLength,1e-5,"Exit guard retains actual specified path length with losses");
        near(dot(loop.apex.curvature*(loop.apex.speed*loop.apex.speed)+Vec3{0,0,gravity},loop.apex.up)/gravity,
            request.apexNormalG,1e-7,"Actual loop crest preserves the requested positive support");
        if(lossy){
            check(end.speed<loop.authoring.speed-1&&end.dissipatedWorkPerMass>50,"Full descent carries true apex state and consumes energy instead of mirroring the ascent");
            check(loop.authoring.speed>std::sqrt(request.apexSpeed*request.apexSpeed+2*gravity*height)+.5,
                "The required inlet speed includes physical ascent and entry-guard losses");
            check(std::abs(loop.apex.time-(end.time-loop.apex.time))>.01,"Loss-aware phase timing is not forced symmetric");
        }else{
            near(end.speed,loop.authoring.speed,1e-6,"Zero-loss full loop retains the equal-height energy special case");
            near(end.dissipatedWorkPerMass,0,0,"Zero-loss source reports no fictitious dissipated work");
        }
        std::vector<FvdSample> checkpoints{loop.loopEntry,loop.crossingEntry,loop.apex,loop.crossingExit,loop.loopExit,end};double work=0,maxWorldLateral=0;
        const double energy=.5*loop.authoring.speed*loop.authoring.speed;
        for(size_t i=0;i<loop.section.samples.size();++i){const auto& s=loop.section.samples[i];
            check(s.dissipatedWorkPerMass>=work,"Full-loop dissipated work remains continuous and monotone through the apex");work=s.dissipatedWorkPerMass;
            near(.5*s.speed*s.speed+gravity*s.position.z+work,energy,1e-6,"Every actual loop state closes energy plus work");
            maxWorldLateral=std::max(maxWorldLateral,std::abs(s.curvature.y*s.speed*s.speed));
            if(i%127==0)checkpoints.push_back(s);
        }
        check(maxWorldLateral>.1,"Zero body-lateral intent still creates actual world-lateral acceleration through the banked normal force");
        std::sort(checkpoints.begin(),checkpoints.end(),[](const auto& a,const auto& b){return a.distance<b.distance;});
        checkLossyReplay(loop.section,request.rollingAcceleration,request.dragAccelerationCoefficient,checkpoints);
        if(height!=73||!lossy)continue;
        // Certify the complete nonadjacent body intervals, not only the
        // selected crossing points. A 4.4 m sphere covers every supported
        // car/rider body orientation plus the existing .2 m motion pad.
        TrainConfig largestBody;largestBody.seatHeight=3;
        const auto sweep=buildClearanceSweep(loop.section.track,largestBody);const auto& cells=sweep.frames();
        double minimumBodyGap=INFINITY;
        for(size_t i=0;i<cells.size();++i)for(size_t j=i+1;j<cells.size();++j){
            if(cells[j].distance-cells[i].distance<11.96)continue;
            const double gap=norm(cells[j].sample.position-cells[i].sample.position)-
                .5*(cells[i].arcLengthBound+cells[j].arcLengthBound)-8.8;
            minimumBodyGap=std::min(minimumBodyGap,gap);
        }
        check(minimumBodyGap>2,"Canonical interval sweep verifies actual arm/body separation throughout the loop");
        auto fineRequest=loop.authoring;fineRequest.step*=.5;const auto fine=designFvdSection(fineRequest);good(fine);
        for(const auto& expected:std::array<FvdSample,4>{loop.loopEntry,loop.apex,loop.loopExit,end}){
            const auto actual=std::lower_bound(fine.samples.begin(),fine.samples.end(),expected.time-1e-9,[](const auto& s,double time){return s.time<time;});
            check(actual!=fine.samples.end(),"Half-step source retains every full-loop phase boundary");
            near(actual->time,expected.time,1e-9,"Full-loop phase times survive independent step halving");
            near(norm(actual->position-expected.position)+norm(actual->forward-expected.forward)+norm(actual->up-expected.up),0,1e-6,
                "Halving the integrated source step preserves actual full-loop pose");
            near(actual->speed,expected.speed,1e-6,"Halved source step preserves actual full-loop speed");
            near(actual->dissipatedWorkPerMass,expected.dissipatedWorkPerMass,1e-6,"Halved source step preserves actual full-loop work");
        }
        // The low crossing observations are integration checkpoints, not
        // artificial force controls. Independent canonical replay verifies
        // their actual state without inserting a new zero-slope boundary.
        for(const auto& crossing:std::array<FvdSample,2>{loop.crossingEntry,loop.crossingExit}){
            const auto actual=fine.track.sample(crossing.distance);
            near(norm(actual.position-crossing.position)+norm(actual.tangent-crossing.forward)+norm(actual.up-crossing.up),0,1e-6,
                "Crossing pose survives half-step replay without an authoring reset at the observation time");
        }
        request.crossingOffset=-request.crossingOffset;const auto mirror=designFvdLoop(request);good(mirror.section);
        const auto& other=mirror.section.samples.back();
        near(norm(other.position-Vec3{end.position.x,-end.position.y,end.position.z})+
            norm(other.forward-Vec3{end.forward.x,-end.forward.y,end.forward.z}),0,1e-6,"Opposite twist intent integrates a physical handed mirror including real exit yaw");
        near(other.speed,end.speed,1e-7,"Handed loop variants retain equivalent physical energy");
    }
    for(int mode:{0,1}){
        FvdLoopRequest request;request.rollingAcceleration=mode==0?.05:0;request.dragAccelerationCoefficient=mode==1?.0002:0;
        const auto loop=designFvdLoop(request);good(loop.section);
        near(loop.loopEntry.distance,request.portLength,1e-5,"Each individual loss law retains the exact inlet guard length");
        near(loop.section.samples.back().distance-loop.loopExit.distance,request.portLength,1e-5,"Each individual loss law retains the exact outlet guard length");
    }
    FvdLoopRequest stronger;stronger.height=73;stronger.apexSpeed=26;stronger.normalG=4.5;
    stronger.rollingAcceleration=gravity*train.rollingResistance;stronger.dragAccelerationCoefficient=.5*train.airDensity*train.dragCdA/(train.cars*train.carMass);
    const auto high=designFvdLoop(stronger);good(high.section);double highNormal=0;
    for(const auto& s:high.section.samples)highNormal=std::max(highNormal,dot(s.curvature*(s.speed*s.speed)+Vec3{0,0,gravity},s.up)/gravity);
    near(highNormal,4.5,1e-7,"Feasible source intent above the former provisional 4.2 g cap retains its actual requested load");
    checkLossyReplay(high.section,stronger.rollingAcceleration,stronger.dragAccelerationCoefficient,
        {high.loopEntry,high.crossingEntry,high.apex,high.crossingExit,high.loopExit,high.section.samples.back()});
    for(int variant=0;variant<7;++variant){FvdLoopRequest invalid;
        if(variant==0)invalid.height=NAN;if(variant==1)invalid.apexSpeed=0;if(variant==2)invalid.crossingOffset=INFINITY;
        if(variant==3)invalid.rollingAcceleration=-.01;if(variant==4)invalid.dragAccelerationCoefficient=NAN;
        if(variant==5)invalid.maxSamples=15;if(variant==6)invalid.step=0;
        const auto rejected=designFvdLoop(invalid);
        check(!rejected.section.integrated&&!rejected.section.report.valid(),"Invalid or under-budget full-loop input cannot produce source geometry");
    }
    auto interrupted=designFvdLoop(FvdLoopRequest{},[]{return true;});
    check(interrupted.section.cancelled&&!interrupted.section.integrated,"Full-loop cancellation is explicit before authoring");
    size_t polls=0;interrupted=designFvdLoop(FvdLoopRequest{},[&]{return ++polls>20;});
    check(interrupted.section.cancelled&&!interrupted.section.integrated,"Cancellation interrupts the joint loss-aware full-loop shooting");
}
void sourceChecks(const FvdImmelmannRequest& r,const FvdImmelmannResult& d){
    check(d.section.report.valid()&&d.section.assessment.passed,"source and independent canonical replay pass");
    check(std::abs(d.apex.position.z-r.height)<1e-5&&d.apex.up.z<-.5&&d.apex.forward.x<-.5,"true inverted half-loop apex");
    check(d.rollExit.time>d.apex.time&&d.rollExit.forward.z<-.1,"roll completes on the descending arm");
    check(std::abs(d.exit.position.z-r.exitHeight)<1e-5&&std::abs(d.exit.forward.z)<1e-6&&norm(d.exit.up-Vec3{0,0,1})<1e-6,"upright level physical valley");
    check(norm(d.rollExit.curvature)>1e-4,"internal roll-to-pullout port remains curved");
    std::vector<FvdSample> checkpoints{d.apex,d.rollExit,d.exit};
    for(size_t i=0;i<d.section.samples.size();i+=127)checkpoints.push_back(d.section.samples[i]);
    std::sort(checkpoints.begin(),checkpoints.end(),[](const auto& a,const auto& b){return a.distance<b.distance;});
    checkLossyReplay(d.section,r.rollingAcceleration,r.dragAccelerationCoefficient,checkpoints);
    double energy=.5*r.entrySpeed*r.entrySpeed;
    for(size_t i=0;i<d.section.samples.size();++i){const auto& q=d.section.samples[i];
        check(std::abs(.5*q.speed*q.speed+gravity*q.position.z+q.dissipatedWorkPerMass-energy)<1e-6,"actual mechanical energy plus losses stays conserved");
        check(q.time<d.apex.time+1e-7?q.forward.z>=-1e-6:q.forward.z<=1e-6,"one ascent and one monotone descent; no extra arc");
        check(std::abs(norm(q.forward)-1)<1e-8&&std::abs(norm(q.up)-1)<1e-8&&std::abs(dot(q.forward,q.up))<1e-8,"continuous orthonormal body frame");
        const Vec3 force=q.curvature*(q.speed*q.speed)+Vec3{0,0,gravity};
        check(dot(force,q.up)/gravity>=std::min(r.crestG,r.rollExitG)-1e-6&&dot(force,q.up)/gravity<=r.normalG+1e-6,"positive crest/roll intent remains within source controls");
        check(std::abs(dot(force,cross(q.forward,q.up))/gravity)<1e-7,"zero body-lateral intent is integrated, not post-warped");
    }
}
void checkImmelmann(){
    FvdImmelmannRequest r;r.rollingAcceleration=gravity*.002;r.dragAccelerationCoefficient=.5*1.225*2.4/9000;
    auto original=designFvdImmelmann(r);sourceChecks(r,original);
    auto mirroredRequest=r;mirroredRequest.hand=-1;auto mirrored=designFvdImmelmann(mirroredRequest);sourceChecks(mirroredRequest,mirrored);
    check(norm(Vec3{original.exit.position.x,-original.exit.position.y,original.exit.position.z}-mirrored.exit.position)<1e-6,"handedness reflects the real exit pose without a geometry warp");
    // The upper scale covers the generator's existing 140 m inversion target
    // plus its seeded dimension margin. This is source proof, not ride acceptance.
    for(double scale:{.97,1.03,std::sqrt(154./88.)}){auto scaled=r;scaled.entrySpeed*=scale;scaled.height*=scale*scale;scaled.exitHeight*=scale*scale;scaled.rampSeconds*=scale;auto d=designFvdImmelmann(scaled);sourceChecks(scaled,d);}
    auto noLoss=r;noLoss.rollingAcceleration=noLoss.dragAccelerationCoefficient=0;auto ideal=designFvdImmelmann(noLoss);sourceChecks(noLoss,ideal);
    check(std::abs(ideal.exit.speed-std::sqrt(r.entrySpeed*r.entrySpeed-2*gravity*r.exitHeight))<1e-7,"zero-loss energy special case follows from integration");
    check(original.exit.speed<ideal.exit.speed-1&&original.exit.dissipatedWorkPerMass>100,"explicit losses materially affect the source exit");
    auto fineRequest=r;fineRequest.step*=.5;auto fine=designFvdImmelmann(fineRequest);sourceChecks(fineRequest,fine);
    check(norm(fine.exit.position-original.exit.position)<1e-6&&std::abs(fine.exit.speed-original.exit.speed)<1e-7,"half-step physical pose and speed converge");
    auto repeat=designFvdImmelmann(r);check(repeat.exit.position.x==original.exit.position.x&&repeat.exit.speed==original.exit.speed,"authoring deterministic");
    size_t calls=0;auto cancelled=designFvdImmelmann(r,[&]{return ++calls>100;});check(cancelled.section.cancelled&&!cancelled.section.report.valid(),"bounded solve honors cancellation");
    auto invalid=r;invalid.hand=0;auto bad=designFvdImmelmann(invalid);check(!bad.section.report.valid()&&bad.section.report.errors.front().code=="FVD_IMMELMANN_INPUT","invalid physical intent fails explicitly");
    for(int kind=0;kind<2;++kind){auto badLoss=r;if(kind==0)badLoss.rollingAcceleration=-.01;else badLoss.dragAccelerationCoefficient=std::nan("");
        const auto rejected=designFvdImmelmann(badLoss);check(!rejected.section.integrated&&!rejected.section.report.valid(),"Invalid explicit loss coefficients cannot author an Immelmann");}
}

}
int main(){try{
    checkPointLosses();checkNormalControlSlopes();checkTallHill();checkAirtimeChain();checkBankedAirtime();checkLossyAirtime();checkCompactAirtimeSimilarity();checkFullLoop();checkImmelmann();
    for(double push:{2.05,2.2,2.5}){
        FvdAirtimeRequest request;request.hills.front().pushG=push;
        auto hill=designFvdAirtime(request);good(hill.section);
        const auto& end=hill.section.samples.back();
        near(end.position.z,0,1e-4,"Airtime closes height without spatial scaling");
        near(norm(end.forward-Vec3{1,0,0}),0,1e-5,"Airtime ends horizontal");
        near(end.speed,request.speed,1e-4,"Symmetric gravity-coupled hill closes energy");
        size_t apex=0;for(size_t i=1;i<hill.section.samples.size();++i)if(hill.section.samples[i].position.z>hill.section.samples[apex].position.z)apex=i;
        const auto& top=hill.section.samples[apex];
        double measured=dot(top.curvature*(top.speed*top.speed)+Vec3{0,0,gravity},top.up)/gravity;
        near(measured,request.hills.front().crestG,.001,"Integrated geometric crest produces requested normal force");
        for(size_t i=0;i<hill.section.samples.size();i+=19){
            const auto& q=hill.section.samples[i];
            near(q.speed*q.speed+2*gravity*q.position.z,request.speed*request.speed,1e-5,"Independent airtime mechanical-energy oracle");
        }
        if(push==2.2){
            constexpr double scale=1.05;auto faster=request;
            faster.speed*=scale;faster.guardSeconds*=scale;faster.portRampSeconds*=scale;
            faster.hills[0].pushHoldSeconds*=scale;faster.hills[0].crestRampSeconds*=scale;faster.hills[0].crestPulseSeconds*=scale;
            const auto enlarged=designFvdAirtime(faster);good(enlarged.section);
            near(enlarged.height,hill.height*scale*scale,1e-4,"Gravity similarity: proportionally faster phase timing scales height by speed squared");
            near(enlarged.span,hill.span*scale*scale,1e-4,"Gravity similarity: horizontal geometry follows speed squared without spatial warping");
            near(enlarged.section.samples.back().time,end.time*scale,1e-5,"Gravity similarity preserves the physical traversal-time scale");
            near(enlarged.hills[0].apex.speed,top.speed*scale,1e-5,"Larger geometry retains proportional actual apex speed");
        }
    }
    check(designFvdAirtime(FvdAirtimeRequest{},[]{return true;}).section.cancelled,"Airtime shooting respects cancellation");
    FvdAirtimeRequest invalidHill;invalidHill.hills.front().pushG=8;
    check(!designFvdAirtime(invalidHill).section.report.valid(),"Invalid airtime control rejects");
    auto straightRequest=constant(1,0,2);auto straight=designFvdSection(straightRequest);good(straight);
    for(const auto& q:straight.samples){
        near(norm(q.position-Vec3{20*q.time,0,50}),0,2e-11,"Analytic level 1g straight position");
        near(q.speed,20,1e-12,"Level track conserves speed without propulsion");
        near(norm(q.curvature),0,1e-12,"1g cancels gravity on level track");
    }
    check(!straight.track.closed,"A section is not a closed circuit");
    // Independent horizontal-circle oracle: rightward bend, radius R, bank b.
    // Up=(sin(b)*inward+cos(b)*Z); normal=sec(b), lateral=0,
    // v^2/R=g*tan(b), tangent twist is zero (angular velocity is vertical).
    const double radius=100,velocity=20,b=std::atan(velocity*velocity/(radius*gravity));
    auto circleRequest=constant(1/std::cos(b),0,3);circleRequest.up={0,-std::sin(b),std::cos(b)};

    auto circle=designFvdSection(circleRequest);good(circle);
    for(const auto& q:circle.samples){
        double angle=velocity*q.time/radius;
        Vec3 p{radius*std::sin(angle),-radius*(1-std::cos(angle)),50};
        near(norm(q.position-p),0,8e-5,"Analytic banked circular turn position");
        near(q.speed,velocity,2e-5,"Gravity tangent vanishes on horizontal circle");
        near(norm(q.curvature),1/radius,2e-7,"Analytic circular curvature");
        near(norm(q.forward),1,2e-14,"SO(3) forward remains unit");
        near(dot(q.forward,q.up),0,2e-14,"SO(3) axes remain perpendicular");
    }
    auto ballisticRequest=constant(0,0,2);ballisticRequest.speed=30;ballisticRequest.step=.01;
    auto coarse=designFvdSection(ballisticRequest);good(coarse);
    ballisticRequest.step=.005;auto fine=designFvdSection(ballisticRequest);good(fine);
    const Vec3 exactPosition{60,0,50-.5*gravity*4};const double exactSpeed=std::hypot(30,gravity*2);
    double coarseError=norm(coarse.samples.back().position-exactPosition),fineError=norm(fine.samples.back().position-exactPosition);
    check(fineError<coarseError*.3&&fineError<.001,"Ballistic trajectory has second-order step convergence");
    near(fine.samples.back().speed,exactSpeed,.0002,"Independent analytic ballistic speed");
    check(fine.assessment.maxNormalResidualG<.005&&fine.assessment.maxLateralResidualG<1e-9,"Canonical zero-g replay measures ballistic forces independently");
    // Vary every channel through smooth controls; compare actual step halves.
    FvdRequest changing;changing.speed=40;changing.step=.01;
    changing.controls={{0,1,0,0},{.8,1.8,.25,.3},{1.6,.4,-.2,-.2},{2.4,1,0,0}};
    auto variedCoarse=designFvdSection(changing);good(variedCoarse);changing.step=.005;
    auto variedFine=designFvdSection(changing);good(variedFine);changing.step=.0025;
    auto variedFinest=designFvdSection(changing);good(variedFinest);
    const double firstDifference=norm(variedCoarse.samples.back().position-variedFine.samples.back().position);
    const double secondDifference=norm(variedFine.samples.back().position-variedFinest.samples.back().position);
    check(secondDifference<firstDifference*.35,"Smooth force and roll controls converge under genuine step halving");
    check(variedFine.assessment.evaluations>variedFine.samples.size(),"Canonical replay samples between authoring knots");
    auto reject=[](FvdRequest r,const char* code){auto result=designFvdSection(r);check(!result.report.valid()&&!result.report.errors.empty()&&result.report.errors.front().code==code,"Unsupported request rejects explicitly");};
    FvdRequest bad;bad.speed=std::numeric_limits<double>::quiet_NaN();reject(bad,"FVD_INPUT");
    bad=FvdRequest{};bad.controls[1].rollRate=INFINITY;reject(bad,"FVD_INPUT");
    bad=FvdRequest{};bad.controls[1].time=0;reject(bad,"FVD_INPUT");
    bad=FvdRequest{};bad.up=bad.forward;reject(bad,"FVD_INPUT");
    bad=FvdRequest{};bad.maxSamples=4;reject(bad,"FVD_BUDGET");
    bad=FvdRequest{};bad.step=0;reject(bad,"FVD_INPUT");
    bad=FvdRequest{};bad.controls[1].normalG=21;reject(bad,"FVD_INPUT");
    bad=constant(0,0,1);bad.forward={0,0,1};bad.up={1,0,0};bad.speed=2;
    reject(bad,"FVD_SPEED");
    auto cancelled=designFvdSection(FvdRequest{},[]{return true;});check(cancelled.cancelled&&!cancelled.integrated&&!cancelled.report.valid(),"Cancellation before integration");
    int polls=0;cancelled=designFvdSection(FvdRequest{},[&]{return ++polls>30;});
    check(cancelled.cancelled&&!cancelled.integrated&&!cancelled.assessment.passed,"Cancellation during integration cannot pass");
    // Force a residual rejection instead of confusing raw integration with fit.
    changing.step=.02;changing.forceToleranceG=1e-8;
    auto residual=designFvdSection(changing);
    check(residual.integrated&&residual.canonicalBuilt&&residual.assessment.performed&&!residual.assessment.passed&&!residual.report.valid(),"A canonical residual failure is distinct from integration completion");
    std::cout<<"PASS "<<checks<<" FVD section checks: analytic straight/circle/ballistic, SO(3), smooth profile convergence, canonical point replay, invalid input and cancellation\n";
    return 0;
}catch(const std::exception& e){std::cerr<<"FAIL after "<<checks<<": "<<e.what()<<'\n';return 1;}}
