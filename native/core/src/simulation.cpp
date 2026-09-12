#include "coaster/coaster.hpp"
#include "simulation_internal.hpp"
#include <deque>

namespace coaster {
double forceExposure(const std::vector<double>& values,double step,double window){
    if(!(step>0&&window>0&&std::isfinite(step)&&std::isfinite(window)))return 0;
    size_t n=size_t(std::llround(window/step));if(n==0||values.size()<n)return 0;
    double sum=0,best=0;for(size_t i=0;i<values.size();++i){sum+=std::max(0.,values[i])*step;if(i>=n)sum-=std::max(0.,values[i-n])*step;if(i+1>=n)best=std::max(best,sum);}return best;
}
namespace {
struct ForceMeasurement { SeatForces force; double verticalRate{}; };
ForceMeasurement measure(const Track& track,double distance,double speed,double acceleration,double height,size_t& spanHint){
    const auto at=track.locate(distance,spanHint);const auto k=sampleSpanKinematics(track,at.span,at.parameter);const auto& p=k.sample;
    Vec3 specific=(p.tangent+k.upS*height)*acceleration+(p.curvature+k.upSS*height)*(speed*speed)+Vec3{0,0,gravity};
    SeatForces force{dot(specific,p.up)/gravity,dot(specific,p.right)/gravity,dot(specific,p.tangent)/gravity};
    // U dot U_s=0 and U dot U_ss=-|U_s|^2 remove acceleration jerk
    // from the vertical component. This is its instantaneous derivative in
    // the rotating rider axis, including both curvature and offset rotation.
    double normal=dot(p.curvature,p.up)-height*dot(k.upS,k.upS);
    double normalS=dot(k.curvatureS,p.up)+dot(p.curvature,k.upS)-2*height*dot(k.upS,k.upSS);
    double rate=(2*speed*acceleration*normal+speed*speed*speed*normalS+gravity*speed*k.upS.z)/gravity;
    return {force,rate};
}
}
SeatForces measureSeatForces(const Track& track,double distance,double speed,double acceleration,double seatHeight){
    size_t hint=track.spans.size();return measure(track,distance,speed,acceleration,seatHeight,hint).force;
}
static SimulationResult simulateImpl(const Track& track,const std::vector<Operation>& ops,const TrainConfig& train,double dt,Cancel cancel,bool forces){
    SimulationResult out;
    if(track.spans.empty()||dt<1./4000||dt>1./30||!std::isfinite(dt)||train.cars<1||train.cars>16||!std::isfinite(train.carMass)||train.carMass<=0||!std::isfinite(train.spacing)||train.spacing<=0||train.spacing>20||!std::isfinite(train.seatHeight)||train.seatHeight<0||!std::isfinite(train.dragCdA)||train.dragCdA<0||!std::isfinite(train.rollingResistance)||train.rollingResistance<0||!std::isfinite(train.airDensity)||train.airDensity<0){out.report.fail("SIM_CONFIG","Invalid simulation configuration");return out;}
    for(const auto& op:ops)if(int(op.kind)<0||int(op.kind)>3||!std::isfinite(op.start)||!std::isfinite(op.end)||!std::isfinite(op.targetSpeed)||!std::isfinite(op.maxForce)||!std::isfinite(op.maxPower)||!std::isfinite(op.rampSeconds)||!std::isfinite(op.stopDeceleration)||!std::isfinite(op.stopOffset)||!std::isfinite(op.exitFadeMeters)||op.exitFadeMeters<.01||op.exitFadeMeters>1000||op.stopDeceleration<=0||op.stopDeceleration>20||op.stopOffset<0||op.stopOffset>5||op.start<0||op.end<0||op.start>track.length||op.end>track.length||op.targetSpeed<0||op.maxForce<0||op.maxPower<0||op.rampSeconds<0){out.report.fail("DRIVE_CONFIG","Invalid explicit drive operation");return out;}
    double half=(train.cars-1)*train.spacing*.5,start=half+30,finish=track.length+start,s=start,v=0,t=0;
    if(!track.closed){start=half+1;s=start;finish=track.length-half-1;}
    if(finish<=start||2*half+2>=track.length){out.report.fail("TRAIN_LENGTH","Track is shorter than the train");return out;}
    std::vector<double> entered(train.cars*ops.size(),-1);
    std::array<size_t,16> carSpans{};std::array<size_t,3> seatSpans{};
    constexpr double traceDt=1./60;double nextTrace=0;
    std::array<double,3> verticalRatePeaks{};std::array<std::vector<double>,3> exposures;std::array<std::array<std::vector<double>,2>,3> horizontalForces;
    auto wrappedDistance=[&](double cs){
        double wrapped=std::fmod(cs,track.length);if(wrapped<0)wrapped+=track.length;
        return wrapped;
    };
    // Operation membership and exit fade use the same wrapped car position.
    // Reuse it across operations without changing their order or force arithmetic.
    auto active=[&](const Operation& op,double wrapped,double at){
        bool inside=op.start<=op.end?(wrapped>=op.start&&wrapped<op.end):(wrapped>=op.start||wrapped<op.end);
        if(track.closed&&op.kind==DriveKind::Launch&&at>track.length*.5)inside=false;
        if(op.kind==DriveKind::Station&&at<track.length*.5)inside=false;
        return inside;
    };
    // All acceleration is a sum of gravity, rolling/air resistance and the
    // bounded force/power of authored operations. There is no speed assignment.
    auto acceleration=[&](double at,double speed,double time){
        double total=0;
        for(int car=0;car<train.cars;++car){
            double cs=at+half-car*train.spacing;const auto tangent=track.tangent(cs,carSpans[car]);
            const double wrapped=wrappedDistance(cs);
            double force=-train.carMass*gravity*tangent.z;
            for(size_t j=0;j<ops.size();++j){const auto& op=ops[j];
                if(!active(op,wrapped,at))continue;
                double entry=entered[car*ops.size()+j];
                double remaining=op.end-wrapped;if(remaining<0)remaining+=track.length;
                // The persisted spatial fade reaches zero force and zero first
                // derivative at the real exit. It cannot raise the drive cap.
                double ramp=smooth((time-(entry<0?time:entry))/std::max(.001,op.rampSeconds))*smooth(remaining/op.exitFadeMeters);
                double target=op.targetSpeed;
                if(op.kind==DriveKind::Station)target=std::sqrt(2*op.stopDeceleration*std::max(0.,finish-at-op.stopOffset));
                double demand=(target-speed)*train.carMass*4;
                double cap=std::min(op.maxForce,op.maxPower/std::max(1.,speed));
                if(op.kind==DriveKind::Launch||op.kind==DriveKind::Boost)force+=std::clamp(demand,0.,cap)*ramp;
                else force+=std::clamp(demand,-cap,0.)*ramp;
            }total+=force;
        }
        total-=.5*train.airDensity*train.dragCdA*speed*std::abs(speed);
        double rollingLimit=train.carMass*train.cars*gravity*train.rollingResistance;
        // Coulomb resistance opposes motion. At rest the same bounded static
        // reaction cancels a sub-limit external force; it cannot add energy.
        if(speed>0)total-=rollingLimit;
        else if(total>rollingLimit)total-=rollingLimit;
        else if(total<-rollingLimit)total+=rollingLimit;
        else total=0;
        return total/(train.carMass*train.cars);
    };
    double stalled=0;size_t step=0;
    while(t<600){
        if((step&31)==0&&cancel&&cancel()){out.cancelled=true;out.report.fail("CANCELLED","Simulation cancelled",s);return out;}
        for(int car=0;car<train.cars;++car){
            const double wrapped=wrappedDistance(s+half-car*train.spacing);
            for(size_t j=0;j<ops.size();++j){auto& entry=entered[car*ops.size()+j];if(!active(ops[j],wrapped,s))entry=-1;else if(entry<0)entry=t;}
        }
        double a=acceleration(s,v,t);
        // Explicit midpoint integration; operation entry state is committed once.
        double predictedMid=v+a*dt*.5,vm=std::max(0.,predictedMid),sm=s+v*dt*.5;
        double am=acceleration(sm,vm,t+dt*.5),vn=std::max(0.,v+am*dt),sn=s+vm*dt;
        // A stop is an event inside the step. Evaluating a zero-speed midpoint
        // and then retaining the old endpoint speed otherwise creates creep.
        double stopAcceleration=0;
        if(v>0&&a<0&&predictedMid<=0)stopAcceleration=a;
        else if(v>0&&am<0&&v+am*dt<=0)stopAcceleration=am;
        if(stopAcceleration<0){
            double stopTime=std::clamp(v/(-stopAcceleration),0.,dt);
            sn=s+v*stopTime+.5*stopAcceleration*stopTime*stopTime;vn=0;
            double remaining=dt-stopTime;
            double restart=acceleration(sn,0,t+stopTime+remaining*.5);
            if(restart>0){vn=restart*remaining;sn+=.5*restart*remaining*remaining;}
        }
        if(!std::isfinite(sn)||!std::isfinite(vn)||vn>200){out.report.fail("NUMERICAL_DIVERGENCE","Simulation became nonfinite or exceeded domain",s);break;}
        if(!std::isfinite(out.metrics.launchTo180)&&v<50&&vn>=50)out.metrics.launchTo180=t+dt*(50-v)/(vn-v);
        out.metrics.maxSpeed=std::max(out.metrics.maxSpeed,vn);
        bool trace=t+1e-9>=nextTrace;Frame frame;frame.time=t;frame.distance=s;frame.speed=v;
        if(forces)for(int seat=0;seat<3;++seat){double offset=seatDistanceOffset(train,seat),cs=s+offset;
            auto measured=measure(track,cs,v,a,train.seatHeight,seatSpans[seat]);SeatForces f=measured.force;frame.seats[seat]=f;
            auto& m=out.metrics;
            if(!std::isfinite(f.vertical)||!std::isfinite(f.lateral)||!std::isfinite(f.longitudinal)||!std::isfinite(measured.verticalRate)){out.report.fail("NONFINITE_FORCE","Seat force is not finite",cs);return out;}
            m.minVerticalG=std::min(m.minVerticalG,f.vertical);m.maxVerticalG=std::max(m.maxVerticalG,f.vertical);m.maxLateralG=std::max(m.maxLateralG,std::abs(f.lateral));m.maxLongitudinalG=std::max(m.maxLongitudinalG,std::abs(f.longitudinal));
            double rate=std::abs(measured.verticalRate);verticalRatePeaks[seat]=std::max(verticalRatePeaks[seat],rate);
            if(rate>m.maxJerkGps){m.maxJerkGps=rate;m.maxJerkDistance=cs;}
            exposures[seat].push_back(f.vertical);horizontalForces[seat][0].push_back(f.lateral);horizontalForces[seat][1].push_back(f.longitudinal);
        }
        if(trace){out.frames.push_back(frame);nextTrace+=traceDt;}
        s=sn;v=vn;t+=dt;++step;
        if(track.closed&&finish-s<.5&&finish-s>=-.25&&v==0){out.completed=true;break;}
        if(!track.closed&&s>=finish){out.completed=true;break;}
        if(s>finish+.25){out.report.fail("STATION_OVERRUN","Train passed its stopping point",s);break;}
        stalled=v<.02?stalled+dt:0;
        if(stalled>3&&t>3){out.report.fail("STALL","Train stopped before returning to its station",s);break;}
    }
    if(!out.frames.empty()&&t>out.frames.back().time){Frame terminal;terminal.time=t;terminal.distance=s;terminal.speed=v;if(forces){double a=acceleration(s,v,t);for(int seat=0;seat<3;++seat)terminal.seats[seat]=measureSeatForces(track,s+seatDistanceOffset(train,seat),v,a,train.seatHeight);}out.frames.push_back(terminal);}
    out.metrics.duration=t;for(const auto& e:exposures)out.metrics.exposure10Seconds=std::max(out.metrics.exposure10Seconds,forceExposure(e,dt));
    for(int seat=0;seat<3;++seat){auto& stats=out.metrics.seats[seat];const auto& vertical=exposures[seat];stats.axes[0]=summarizeAxis(vertical,dt);stats.axes[0].maxRateGps=verticalRatePeaks[seat];stats.axes[1]=summarizeAxis(horizontalForces[seat][0],dt);stats.axes[2]=summarizeAxis(horizontalForces[seat][1],dt);stats.exposure10Seconds=forceExposure(vertical,dt);
        std::array<double,4> current{};for(double f:vertical){bool exposureActive[]={f<0,f>2,f>3,f>4};double* total[]={&stats.airtimeBelowZeroSeconds,&stats.positiveAbove2Seconds,&stats.positiveAbove3Seconds,&stats.positiveAbove4Seconds};double* longest[]={&stats.longestAirtimeSeconds,&stats.longestAbove2Seconds,&stats.longestAbove3Seconds,&stats.longestAbove4Seconds};for(int i=0;i<4;++i){if(exposureActive[i]){*total[i]+=dt;current[i]+=dt;*longest[i]=std::max(*longest[i],current[i]);}else current[i]=0;}}}
    if(!out.completed&&out.report.valid())out.report.fail("TIMEOUT","Ride did not finish within 600 seconds",s);
    return out;
}
SimulationResult simulate(const Track& track,const std::vector<Operation>& ops,const TrainConfig& train,double dt,Cancel cancel){
    return simulateImpl(track,ops,train,dt,cancel,true);
}
MotionResult simulateMotion(const Track& track,const std::vector<Operation>& ops,const TrainConfig& train,double dt,Cancel cancel){
    auto result=simulateImpl(track,ops,train,dt,cancel,false);
    return {std::move(result.frames),result.completed,result.cancelled};
}
}
