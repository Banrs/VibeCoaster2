#include "coaster/trim_brake.hpp"
#include "coaster/coaster.hpp"
#include "simulation_internal.hpp"
#include "motion_spline.hpp"

namespace coaster {
ReplayMotion interpolateMotion(const Frame& a,const Frame& b,double time){
    if(time<=a.time||b.time<=a.time)return {a.distance,a.speed,a.acceleration,a.accelerationRate};
    if(time>=b.time)return {b.distance,b.speed,b.acceleration,b.accelerationRate};
    const double duration=b.time-a.time;
    // The scalar septic Hermite used by angle programmes also preserves the
    // distance/speed/acceleration/jerk time jet across every presentation frame.
    const auto polynomial=detail::anglePolynomial({a.distance,a.speed,a.acceleration,a.accelerationRate},{b.distance,b.speed,b.acceleration,b.accelerationRate},duration);
    const auto q=detail::angleAt(polynomial,time-a.time,duration);
    return {q.value,q.first,q.second,q.third};
}
double forceExposure(const std::vector<double>& values,double step,double window){
    if(!(step>0&&window>0&&std::isfinite(step)&&std::isfinite(window)))return 0;
    size_t n=size_t(std::llround(window/step));if(n==0||values.size()<n)return 0;
    double sum=0,best=0;for(size_t i=0;i<values.size();++i){sum+=std::max(0.,values[i])*step;if(i>=n)sum-=std::max(0.,values[i-n])*step;if(i+1>=n)best=std::max(best,sum);}return best;
}
namespace {
double positiveDemand(double value,double cap){
    if(value<=0)return 0;const double u=value/(cap*.05);if(u>=1)return value;
    return value*std::pow(u,4)*(35+u*(-84+u*(70-20*u)));
}
double saturatedForce(double value,double cap){
    if(value<=cap*.8)return value;if(value>=cap*1.2)return cap;
    const double u=(value-cap*.8)/(cap*.4);
    // Integral of a C3 step: the slope joins linear demand to the force cap
    // without a jerk discontinuity, and force never exceeds the actual cap.
    return cap*.8+cap*.4*(u-7*std::pow(u,5)+14*std::pow(u,6)-10*std::pow(u,7)+2.5*std::pow(u,8));
}
SeatDynamics measure(const Track& track,double distance,double speed,double acceleration,double accelerationRate,double height,size_t& spanHint){
    const auto at=track.locate(distance,spanHint);const auto k=sampleSpanKinematics(track,at.span,at.parameter);const auto& p=k.sample;
    Vec3 specific=(p.tangent+k.upS*height)*acceleration+(p.curvature+k.upSS*height)*(speed*speed)+Vec3{0,0,gravity};
    SeatForces force{dot(specific,p.up)/gravity,dot(specific,p.right)/gravity,dot(specific,p.tangent)/gravity};
    const Vec3 jerk=(p.tangent+k.upS*height)*accelerationRate+
        (p.curvature+k.upSS*height)*(3*speed*acceleration)+(k.curvatureS+k.upSSS*height)*(speed*speed*speed);
    const Vec3 rightS=cross(p.curvature,p.up)+cross(p.tangent,k.upS);
    const Vec3 rightSS=cross(k.curvatureS,p.up)+cross(p.curvature,k.upS)*2+cross(p.tangent,k.upSS);
    const Vec3 rightSSS=cross(k.curvatureSS,p.up)+cross(k.curvatureS,k.upS)*3+cross(p.curvature,k.upSS)*3+cross(p.tangent,k.upSSS);
    const Vec3 omegaS=(cross(p.tangent,p.curvature)+cross(p.right,rightS)+cross(p.up,k.upS))*.5;
    const Vec3 omegaSS=(cross(p.tangent,k.curvatureS)+cross(p.right,rightSS)+cross(p.up,k.upSS))*.5;
    const Vec3 omegaSSS=(cross(p.curvature,k.curvatureS)+cross(p.tangent,k.curvatureSS)+cross(rightS,rightSS)+cross(p.right,rightSSS)+cross(k.upS,k.upSS)+cross(p.up,k.upSSS))*.5;
    const SeatForces rate{(dot(jerk,p.up)+dot(specific,k.upS)*speed)/gravity,
        (dot(jerk,p.right)+dot(specific,rightS)*speed)/gravity,
        (dot(jerk,p.tangent)+dot(specific,p.curvature)*speed)/gravity};
    return {force,rate,jerk,omegaS*speed,omegaSS*(speed*speed)+omegaS*acceleration,
        omegaSSS*(speed*speed*speed)+omegaSS*(3*speed*acceleration)+omegaS*accelerationRate};
}
}
SeatForces measureSeatForces(const Track& track,double distance,double speed,double acceleration,double seatHeight){
    size_t hint=track.spans.size();return measure(track,distance,speed,acceleration,0,seatHeight,hint).force;
}
SeatDynamics measureSeatDynamics(const Track& track,double distance,double speed,double acceleration,double accelerationRate,double height){
    size_t hint=track.spans.size();return measure(track,distance,speed,acceleration,accelerationRate,height,hint);
}
static SimulationResult simulateImpl(const Track& track,const std::vector<Operation>& ops,const TrainConfig& train,double dt,Cancel cancel,bool forces,MotionReplayMode mode=MotionReplayMode::TrackDomain){
    SimulationResult out;
    if(mode!=MotionReplayMode::TrackDomain&&mode!=MotionReplayMode::StationEnergyPrefix){out.report.fail("SIM_MODE","Unsupported motion replay mode");return out;}
    if(mode==MotionReplayMode::StationEnergyPrefix&&(forces||track.closed||std::any_of(ops.begin(),ops.end(),[](const Operation& op){return op.kind==DriveKind::Station;}))){
        out.report.fail("SIM_PREFIX","Station energy calibration requires an open prefix without a terminal station operation");return out;
    }
    if(track.spans.empty()||dt<1./4000||dt>1./30||!std::isfinite(dt)||train.cars<1||train.cars>16||!std::isfinite(train.carMass)||train.carMass<=0||!std::isfinite(train.spacing)||train.spacing<=0||train.spacing>20||!std::isfinite(train.seatHeight)||train.seatHeight<0||!std::isfinite(train.dragCdA)||train.dragCdA<0||!std::isfinite(train.rollingResistance)||train.rollingResistance<0||!std::isfinite(train.airDensity)||train.airDensity<0){out.report.fail("SIM_CONFIG","Invalid simulation configuration");return out;}
    for(const auto& op:ops)if(!validDriveParameters(op)||op.start>track.length||op.end>track.length){out.report.fail("DRIVE_CONFIG","Invalid explicit drive operation");return out;}
    if(!std::isfinite(track.length)||track.length<=0){out.report.fail("SIM_CONFIG","Invalid track length");return out;}
    double half=(train.cars-1)*train.spacing*.5,start=half+30,finish=track.length+start,s=start,v=0,t=0;
    if(!track.closed){start=half+(mode==MotionReplayMode::StationEnergyPrefix?30:1);s=start;finish=track.length-half-1;}
    if(finish<=start||2*half+2>=track.length){out.report.fail("TRAIN_LENGTH","Track is shorter than the train");return out;}
    std::vector<double> entered(train.cars*ops.size(),-1);
    struct TrimCommand {bool sensed{},engaged{};double time{},speed{},deployment{};};
    std::vector<TrimCommand> trimCommands(ops.size());
    for(size_t j=0;j<ops.size();++j)if(ops[j].kind==DriveKind::Trim)out.trims.push_back({j});
    std::array<std::vector<double>,4> trimPower;for(auto& p:trimPower)p.resize(ops.size());
    const DriveIndex driveIndex(ops,track.length);
    std::array<std::vector<size_t>,16> previousDrives;
    for(int car=0;car<train.cars;++car)previousDrives[car].reserve(ops.size());
    std::array<size_t,16> carSpans{},potentialSpans{};std::array<size_t,3> seatSpans{};
    constexpr double traceDt=1./60;double nextTrace=0;
    std::array<std::array<double,3>,3> ratePeaks{};std::array<std::vector<double>,3> exposures;std::array<std::array<std::vector<double>,2>,3> horizontalForces;
    struct Capture {bool active{};size_t operation{};double begin{},duration{},speed{},remaining{},target{};} capture;
    const double rollingAcceleration=gravity*train.rollingResistance,dragCoefficient=.5*train.airDensity*train.dragCdA/(train.carMass*train.cars);
    auto wrappedDistance=[&](double cs){
        if(cs>=0&&cs<track.length)return cs;
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
    auto acceleration=[&](double at,double speed,double time,std::array<double,3>* work=nullptr,std::vector<double>* trimWatts=nullptr){
        if(trimWatts)std::fill(trimWatts->begin(),trimWatts->end(),0.);
        double total=0,drive=0,brake=0;
        for(int car=0;car<train.cars;++car){
            double cs=at+half-car*train.spacing;const auto tangent=track.tangent(cs,carSpans[car]);
            const double wrapped=wrappedDistance(cs);
            double force=-train.carMass*gravity*tangent.z;
            for(size_t j:driveIndex.at(wrapped)){const auto& op=ops[j];
                if(!active(op,wrapped,at))continue;
                if(op.kind==DriveKind::Trim) {
                    const auto& command=trimCommands[j];
                    const double activation=command.deployment*smooth((time-command.time)/op.rampSeconds);
                    const double f=eddyBrakeForce(speed,op.maxForce,op.trimPeakSpeed)*activation*trimFieldCoverage(op,wrapped);
                    force+=f;brake-=f;if(trimWatts)(*trimWatts)[j]-=f*speed;continue;
                }
                double entry=entered[car*ops.size()+j];
                double remaining=op.end-wrapped;if(remaining<0)remaining+=track.length;
                // The persisted spatial fade reaches zero force and zero first
                // derivative at the real exit. It cannot raise the drive cap.
                double ramp=smooth((time-(entry<0?time:entry))/std::max(.001,op.rampSeconds))*smooth(remaining/op.exitFadeMeters);
                double target=op.targetSpeed;
                const double cap=std::min(op.maxForce,op.maxPower/std::max(1.,speed));
                if(op.kind==DriveKind::Station)target=std::sqrt(2*op.stopDeceleration*std::max(0.,finish-at-op.stopOffset));
                double demand=(target-speed)*train.carMass*4;
                if(op.kind==DriveKind::Station){
                    const double remainingToStop=std::max(1e-12,finish-at-.05);
                    // Finish the handover before the legacy square-root target
                    // reaches zero; its endpoint has an unbounded derivative.
                    const double speedWeight=smooth((3-speed)/2),distanceWeight=smooth((op.stopOffset+2.5-(finish-at))/2);
                    const double weight=1-(1-speedWeight)*(1-distanceWeight);
                    double desired=-.75*speed*speed/remainingToStop;
                    if(capture.active&&capture.operation==j){const double u=std::max(0.,1-(time-capture.begin)/capture.duration);desired=-3*capture.speed/capture.duration*u*u;}
                    // Positioning tyres compensate bearing/air losses during
                    // the final soft capture. At rest no drive is requested.
                    const double compensation=speed>0?rollingAcceleration+dragCoefficient*speed*speed+gravity*tangent.z:0;
                    const double tracking=demand+train.carMass*(compensation-op.stopDeceleration);
                    demand=-positiveDemand(-tracking,cap)*(1-weight)+train.carMass*(desired+compensation)*weight;
                }
                if(op.kind==DriveKind::Launch||op.kind==DriveKind::Boost){double f=saturatedForce(positiveDemand(demand,cap),cap)*ramp;force+=f;drive+=f;}
                else if(op.kind==DriveKind::Station){const double f=std::copysign(saturatedForce(std::abs(demand),cap),demand)*ramp;force+=f;drive+=std::max(0.,f);brake+=std::max(0.,-f);}
                else{double f=-saturatedForce(positiveDemand(-demand,cap),cap)*ramp;force+=f;brake-=f;}
            }total+=force;
        }
        total-=.5*train.airDensity*train.dragCdA*speed*std::abs(speed);
        double rollingLimit=train.carMass*train.cars*gravity*train.rollingResistance;
        if(work){const double mass=train.carMass*train.cars;*work={drive/mass,brake/mass,(.5*train.airDensity*train.dragCdA*speed*std::abs(speed)+(speed>0?rollingLimit:0))/mass};}
        // Coulomb resistance opposes motion. At rest the same bounded static
        // reaction cancels a sub-limit external force; it cannot add energy.
        if(speed>0)total-=rollingLimit;
        else if(total>rollingLimit)total-=rollingLimit;
        else if(total<-rollingLimit)total+=rollingLimit;
        else total=0;
        return total/(train.carMass*train.cars);
    };
    auto potential=[&](double at){double height=0;for(int car=0;car<train.cars;++car){
        height+=track.position(at+half-car*train.spacing,potentialSpans[car]).z;
    }return gravity*height/train.cars;};
    const double initialEnergy=potential(s);double driveWork=0,brakeWork=0,lossWork=0;
    double stalled=0;size_t step=0;
    while(t<600){
        if((step&31)==0&&cancel&&cancel()){out.cancelled=true;out.report.fail("CANCELLED","Simulation cancelled",s);return out;}
        for(size_t j=0;j<ops.size();++j)if(ops[j].kind==DriveKind::Trim&&!trimCommands[j].sensed) {
            const auto& op=ops[j];const double front=s+half;
            if(front>=op.start-op.trimSensorLead) {
                auto& command=trimCommands[j];command={true,v>op.targetSpeed,t,v,trimDeployment(op,v,train.carMass)};
                for(auto& observation:out.trims)if(observation.operation==j){observation.sensorTime=t;observation.sensedSpeed=v;observation.deployment=command.deployment;}
                if(command.engaged&&(op.start-front)/std::max(.1,v)<op.rampSeconds+.02) {
                    out.report.fail("TRIM_ARMING","Upstream detector leaves insufficient time to position the brake before train entry",s,(op.start-front)/std::max(.1,v),op.rampSeconds+.02);
                    return out;
                }
            }
        }
        for(int car=0;car<train.cars;++car){
            const double wrapped=wrappedDistance(s+half-car*train.spacing);
            auto& previous=previousDrives[car];
            for(size_t j:previous)if(!active(ops[j],wrapped,s))entered[car*ops.size()+j]=-1;
            previous.clear();
            for(size_t j:driveIndex.at(wrapped))if(active(ops[j],wrapped,s)){
                auto& entry=entered[car*ops.size()+j];if(entry<0)entry=t;previous.push_back(j);
            }
        }
        if(!capture.active&&track.closed&&v>0&&v<=1&&finish-s>.05&&finish-s<1.05){
            const double remaining=finish-s-.05,duration=4*remaining/v;
            if(duration<=8)for(size_t j=0;j<ops.size();++j)if(ops[j].kind==DriveKind::Station){
                bool eligible=true;const auto& op=ops[j];
                for(int car=0;car<train.cars&&eligible;++car){const double cs=s+half-car*train.spacing,wrapped=wrappedDistance(cs);double exit=op.end-wrapped;if(exit<0)exit+=track.length;
                    const auto k=sampleKinematics(track,cs);const double demand=train.carMass*(-3*v/duration+rollingAcceleration+dragCoefficient*v*v+gravity*k.sample.tangent.z);
                    eligible=active(op,wrapped,s)&&entered[car*ops.size()+j]>=0&&t-entered[car*ops.size()+j]>=op.rampSeconds&&exit>=op.exitFadeMeters&&
                        std::abs(k.sample.tangent.z)<1e-8&&norm(k.sample.curvature)<1e-8&&std::abs(demand)<.8*std::min(op.maxForce,op.maxPower);
                    for(size_t other:driveIndex.at(wrapped))if(other!=j&&active(ops[other],wrapped,s))eligible=false;
                }
                if(eligible){capture={true,j,t,duration,v,remaining,finish-.05};break;}
            }
        }
        std::array<double,3> w1{},w2{},w3{},w4{};
        double a=acceleration(s,v,t,&w1,&trimPower[0]);
        // Fourth-order state and work integration. Entry epochs are committed
        // once; stage evaluations read the actual bounded actuators.
        double predictedMid=v+a*dt*.5,vm=std::max(0.,predictedMid),sm=s+v*dt*.5;
        if(capture.active){
            const double u=std::max(0.,1-(t+dt*.5-capture.begin)/capture.duration);
            vm=capture.speed*u*u*u;sm=capture.target-capture.remaining*u*u*u*u;
        }
        const double a2=acceleration(sm,vm,t+dt*.5,&w2,&trimPower[1]);
        double v3=std::max(0.,v+a2*dt*.5),s3=s+vm*dt*.5;
        if(capture.active){v3=vm;s3=sm;}
        const double a3=acceleration(s3,v3,t+dt*.5,&w3,&trimPower[2]);
        double v4=std::max(0.,v+a3*dt),s4=s+v3*dt;
        if(capture.active){const double u=std::max(0.,1-(t+dt-capture.begin)/capture.duration);v4=capture.speed*u*u*u;s4=capture.target-capture.remaining*u*u*u*u;}
        const double a4=acceleration(s4,v4,t+dt,&w4,&trimPower[3]),am=(a2+a3)*.5;
        double vn=std::max(0.,v+dt*(a+2*a2+2*a3+a4)/6),sn=s+dt*(v+2*vm+2*v3+v4)/6;
        const double accelerationRate=(-3*a+4*am-a4)/dt;
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
        if(capture.active){
            // Exact integration of the bounded terminal tyre force pulse:
            // a=-3*v0/T*(1-t/T)^2, v=v0*(1-t/T)^3. Acceleration
            // and jerk both reach zero at the finite stopping event.
            const auto pulse=[&](double time){const double u=std::max(0.,1-(time-capture.begin)/capture.duration);return -3*capture.speed/capture.duration*u*u;};
            if(std::abs(a-pulse(t))>1e-8||std::abs(am-pulse(t+dt*.5))>1e-8||std::abs(a4-pulse(t+dt))>1e-8){
                out.report.fail("STATION_CAPTURE_FORCE","Positioning tyres cannot realize their stopping force within the actual actuator caps",s);break;
            }
            const double u=std::max(0.,1-(t+dt-capture.begin)/capture.duration);
            vn=capture.speed*u*u*u;sn=capture.target-capture.remaining*u*u*u*u;
        }
        if(!std::isfinite(sn)||!std::isfinite(vn)||vn>200){out.report.fail("NUMERICAL_DIVERGENCE","Simulation became nonfinite or exceeded domain",s);break;}
        if(!std::isfinite(out.metrics.launchTo180)&&v<50&&vn>=50)out.metrics.launchTo180=t+dt*(50-v)/(vn-v);
        out.metrics.maxSpeed=std::max(out.metrics.maxSpeed,vn);
        bool trace=t+1e-9>=nextTrace;Frame frame;frame.time=t;frame.distance=s;frame.speed=v;
        frame.acceleration=a;frame.accelerationRate=accelerationRate;
        if(forces){frame.driveWorkPerMass=driveWork;frame.brakeWorkPerMass=brakeWork;frame.lossWorkPerMass=lossWork;
            frame.energyResidual=.5*v*v+potential(s)+brakeWork+lossWork-driveWork-initialEnergy;
            out.metrics.maxEnergyResidual=std::max(out.metrics.maxEnergyResidual,std::abs(frame.energyResidual));}
        if(forces)for(int seat=0;seat<3;++seat){double offset=seatDistanceOffset(train,seat),cs=s+offset;
            auto measured=measure(track,cs,v,a,accelerationRate,train.seatHeight,seatSpans[seat]);SeatForces f=measured.force;frame.seats[seat]=f;
            auto& m=out.metrics;
            if(!std::isfinite(f.vertical)||!std::isfinite(f.lateral)||!std::isfinite(f.longitudinal)||!finite(measured.inertialJerk)||!finite(measured.angularJerk)){out.report.fail("NONFINITE_FORCE","Seat dynamics are not finite",cs);return out;}
            m.minVerticalG=std::min(m.minVerticalG,f.vertical);m.maxVerticalG=std::max(m.maxVerticalG,f.vertical);m.maxLateralG=std::max(m.maxLateralG,std::abs(f.lateral));m.maxLongitudinalG=std::max(m.maxLongitudinalG,std::abs(f.longitudinal));
            double rate=std::abs(measured.rate.vertical);const std::array<double,3> rates{rate,std::abs(measured.rate.lateral),std::abs(measured.rate.longitudinal)};
            if(rates[1]>ratePeaks[seat][1])m.seats[seat].maxLateralRateDistance=cs;
            for(int axis=0;axis<3;++axis)ratePeaks[seat][axis]=std::max(ratePeaks[seat][axis],rates[axis]);
            auto& stats=m.seats[seat];stats.maxInertialJerk=std::max(stats.maxInertialJerk,norm(measured.inertialJerk));
            stats.maxAngularVelocity=std::max(stats.maxAngularVelocity,norm(measured.angularVelocity));
            stats.maxAngularAcceleration=std::max(stats.maxAngularAcceleration,norm(measured.angularAcceleration));
            if(norm(measured.angularJerk)>stats.maxAngularJerk){stats.maxAngularJerk=norm(measured.angularJerk);stats.maxAngularJerkDistance=cs;}
            if(rate>m.maxJerkGps){m.maxJerkGps=rate;m.maxJerkDistance=cs;}
            exposures[seat].push_back(f.vertical);horizontalForces[seat][0].push_back(f.lateral);horizontalForces[seat][1].push_back(f.longitudinal);
        }
        if(trace){out.frames.push_back(frame);nextTrace+=traceDt;}
        std::array<double,3> work{};for(int i=0;i<3;++i)work[i]=dt*(w1[i]*v+2*w2[i]*vm+2*w3[i]*v3+w4[i]*v4)/6;
        out.metrics.peakDrivePowerWatts=std::max(out.metrics.peakDrivePowerWatts,train.carMass*train.cars*std::max({w1[0]*v,w2[0]*vm,w3[0]*v3,w4[0]*v4}));
        out.metrics.peakBrakePowerWatts=std::max(out.metrics.peakBrakePowerWatts,train.carMass*train.cars*std::max({w1[1]*v,w2[1]*vm,w3[1]*v3,w4[1]*v4}));
        for(auto& observation:out.trims){const auto j=observation.operation;
            observation.energyJoules+=dt*(trimPower[0][j]+2*trimPower[1][j]+2*trimPower[2][j]+trimPower[3][j])/6;
            observation.peakPowerWatts=std::max(observation.peakPowerWatts,std::max({trimPower[0][j],trimPower[1][j],trimPower[2][j],trimPower[3][j]}));}
        driveWork+=work[0];brakeWork+=work[1];lossWork+=work[2];
        s=sn;v=vn;t+=dt;++step;
        if(track.closed&&finish-s<.5&&finish-s>=-.25&&v==0){out.completed=true;break;}
        if(!track.closed&&s>=finish){out.completed=true;break;}
        if(s>finish+.25){out.report.fail("STATION_OVERRUN","Train passed its stopping point",s);break;}
        stalled=v<.02?stalled+dt:0;
        if(stalled>3&&t>3&&!capture.active){out.report.fail("STALL","Train stopped before returning to its station",s);break;}
    }
    if(!out.frames.empty()&&t>out.frames.back().time){Frame terminal;terminal.time=t;terminal.distance=s;terminal.speed=v;terminal.acceleration=acceleration(s,v,t);terminal.driveWorkPerMass=driveWork;terminal.brakeWorkPerMass=brakeWork;terminal.lossWorkPerMass=lossWork;
        terminal.energyResidual=.5*v*v+potential(s)+brakeWork+lossWork-driveWork-initialEnergy;
        out.metrics.maxEnergyResidual=std::max(out.metrics.maxEnergyResidual,std::abs(terminal.energyResidual));
        if(forces)for(int seat=0;seat<3;++seat)terminal.seats[seat]=measureSeatForces(track,s+seatDistanceOffset(train,seat),v,terminal.acceleration,train.seatHeight);out.frames.push_back(terminal);}
    out.metrics.duration=t;
    out.metrics.driveWorkPerMass=driveWork;out.metrics.brakeWorkPerMass=brakeWork;out.metrics.lossWorkPerMass=lossWork;
    if(forces)for(int seat=0;seat<3;++seat){auto& stats=out.metrics.seats[seat];const auto& vertical=exposures[seat];stats.axes[0]=summarizeAxis(vertical,dt);stats.axes[1]=summarizeAxis(horizontalForces[seat][0],dt);stats.axes[2]=summarizeAxis(horizontalForces[seat][1],dt);stats.exposure10Seconds=forceExposure(vertical,dt);
        for(int axis=0;axis<3;++axis)stats.axes[axis].maxRateGps=ratePeaks[seat][axis];
        out.metrics.exposure10Seconds=std::max(out.metrics.exposure10Seconds,stats.exposure10Seconds);
        std::array<double,4> current{};for(double f:vertical){bool exposureActive[]={f<0,f>2,f>3,f>4};double* total[]={&stats.airtimeBelowZeroSeconds,&stats.positiveAbove2Seconds,&stats.positiveAbove3Seconds,&stats.positiveAbove4Seconds};double* longest[]={&stats.longestAirtimeSeconds,&stats.longestAbove2Seconds,&stats.longestAbove3Seconds,&stats.longestAbove4Seconds};for(int i=0;i<4;++i){if(exposureActive[i]){*total[i]+=dt;current[i]+=dt;*longest[i]=std::max(*longest[i],current[i]);}else current[i]=0;}}}
    if(forces&&out.completed){
        // Statistics above retain their documented interval convention. The
        // standard's continuous crossing analysis also needs the terminal node
        // at N*dt; it never receives the decimated 60 Hz presentation frames.
        for(size_t seat=0;seat<3;++seat){
            const auto terminal=measureSeatForces(track,s+seatDistanceOffset(train,int(seat)),v,acceleration(s,v,t),train.seatHeight);
            exposures[seat].push_back(terminal.vertical);
            horizontalForces[seat][0].push_back(terminal.lateral);
            horizontalForces[seat][1].push_back(terminal.longitudinal);
            out.acceleration[seat]=assessAccelerationF2291_25({horizontalForces[seat][1],horizontalForces[seat][0],exposures[seat],dt,0,seat},cancel);
            if(out.acceleration[seat].cancelled){out.cancelled=true;out.report.fail("CANCELLED","Acceleration-history assessment cancelled",s);return out;}
        }
    }
    if(!out.completed&&out.report.valid())out.report.fail("TIMEOUT","Ride did not finish within 600 seconds",s);
    return out;
}
SimulationResult simulate(const Track& track,const std::vector<Operation>& ops,const TrainConfig& train,double dt,Cancel cancel){
    return simulateImpl(track,ops,train,dt,cancel,true);
}
MotionResult simulateMotion(const Track& track,const std::vector<Operation>& ops,const TrainConfig& train,double dt,Cancel cancel,MotionReplayMode mode){
    auto result=simulateImpl(track,ops,train,dt,cancel,false,mode);
    MotionResult motion;motion.frames=std::move(result.frames);motion.cancelled=result.cancelled;motion.report=std::move(result.report);
    if(mode==MotionReplayMode::StationEnergyPrefix)motion.prefixReachedEnd=result.completed&&!result.cancelled&&motion.report.valid();
    else motion.completed=result.completed;
    return motion;
}
}
