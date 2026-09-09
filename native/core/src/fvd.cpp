#include "coaster/fvd.hpp"
#include <stdexcept>

namespace coaster {
namespace {
constexpr Vec3 gVector{0,0,-gravity};
struct Failure:std::runtime_error {
    std::string code;
    Failure(const char* c,const char* message):std::runtime_error(message),code(c){}
};
void require(bool condition,const char* code,const char* message){if(!condition)throw Failure(code,message);}
void poll(const Cancel& cancel){if(cancel&&cancel())throw Failure("CANCELLED","FVD section authoring cancelled");}
bool bounded(double value,double lo,double hi){return std::isfinite(value)&&value>=lo&&value<=hi;}
using ControlEvaluator=std::function<FvdControl(double)>;
FvdControl control(const FvdRequest& request,double time,const ControlEvaluator& evaluate={}){
    if(evaluate)return evaluate(time);
    auto hi=std::upper_bound(request.controls.begin(),request.controls.end(),time,
        [](double t,const FvdControl& c){return t<c.time;});
    if(hi==request.controls.end())return request.controls.back();
    if(hi==request.controls.begin())return *hi;
    const auto& a=*(hi-1);const auto& b=*hi;
    const double u=smooth((time-a.time)/(b.time-a.time));
    return {time,a.normalG+(b.normalG-a.normalG)*u,a.lateralG+(b.lateralG-a.lateralG)*u,
        a.rollRate+(b.rollRate-a.rollRate)*u};
}
void validate(const FvdRequest& r){
    require(bounded(r.rollingAcceleration,0,1)&&bounded(r.dragAccelerationCoefficient,0,.01),"FVD_INPUT","Invalid explicit source loss coefficients");
    require(finite(r.position)&&norm(r.position)<=100000,"FVD_INPUT","Invalid FVD start position");
    require(finite(r.forward)&&finite(r.up)&&std::abs(norm(r.forward)-1)<=1e-6&&
        std::abs(norm(r.up)-1)<=1e-6&&std::abs(dot(r.forward,r.up))<=1e-6,
        "FVD_INPUT","FVD start forward/up must form an orthonormal frame");
    require(bounded(r.speed,.5,250)&&bounded(r.step,.0001,.05)&&r.maxSamples>=4&&r.maxSamples<=50000,
        "FVD_INPUT","Invalid FVD speed, step or sample budget");
    require(bounded(r.forceToleranceG,1e-8,1)&&bounded(r.rollToleranceRadPerSecond,1e-8,1)&&
        bounded(r.replayDistanceTolerance,1e-8,1)&&bounded(r.replaySpeedTolerance,1e-8,1),
        "FVD_INPUT","Invalid FVD residual tolerance");
    require(r.controls.size()>=2&&r.controls.size()<=2048&&r.controls.front().time==0,
        "FVD_INPUT","FVD needs two or more controls beginning at time zero");
    double previous=-1;
    for(const auto& c:r.controls){
        require(bounded(c.time,0,60)&&c.time>previous&&(previous<0||c.time-previous>=.001)&&
            bounded(c.normalG,-20,20)&&bounded(c.lateralG,-20,20)&&bounded(c.rollRate,-4*pi,4*pi),
            "FVD_INPUT","Invalid FVD control time, force or roll rate");previous=c.time;
    }
    require(std::ceil(r.controls.back().time/r.step)+1<=double(r.maxSamples),
        "FVD_BUDGET","FVD requested time resolution exceeds sample budget");
}
struct State {Vec3 p,t,u;double v{},s{},work{};};
struct Derivative {Vec3 omega,k;double dv{};};
double loss(const FvdRequest& r,double speed){return r.rollingAcceleration+r.dragAccelerationCoefficient*speed*speed;}
Derivative derivative(const State& state,const FvdControl& input,const FvdRequest& r){
    require(bounded(state.v,.5,250),"FVD_SPEED","FVD speed left [.5,250] m/s; no low-speed clamp is applied");
    const Vec3 right=cross(state.t,state.u);
    const Vec3 a=gVector+state.u*(gravity*input.normalG)+right*(gravity*input.lateralG);
    const double dv=dot(gVector,state.t)-loss(r,state.v);
    const Vec3 dt=(a-state.t*dot(a,state.t))/state.v;
    const Vec3 k=dt/state.v;
    require(finite(k)&&norm(k)<=.15,"FVD_CURVATURE","FVD curvature exceeds the supported canonical section domain");
    return {cross(state.t,dt)+state.t*input.rollRate,k,dv};
}
struct Quaternion {
    double w{1};Vec3 v{};
    Quaternion operator+(Quaternion b)const{return {w+b.w,v+b.v};}
    Quaternion operator*(double scale)const{return {w*scale,v*scale};}
};
Quaternion normalized(Quaternion q){
    const double magnitude=std::sqrt(q.w*q.w+dot(q.v,q.v));
    require(std::isfinite(magnitude)&&magnitude>.5,"FVD_FRAME","Invalid integrated quaternion");
    return q*(1/magnitude);
}
Vec3 turned(Vec3 value,Quaternion q){
    q=normalized(q);return value+cross(q.v,value)*(2*q.w)+cross(q.v,cross(q.v,value))*2;
}
struct Stage {Quaternion dq;Vec3 dp;double dv{},ds{},workRate{};};
// Classical RK4 on position, speed and a world-space quaternion increment.
// The smooth off-manifold extension uses normalized q for the rotation but
// raw q in qdot=1/2*(0,omega)*q. Final normalization restores SO(3), removing
// the quaternion norm truncation error without changing fourth-order accuracy.
State advance(const State& start,const FvdRequest& r,double time,double dt,const ControlEvaluator& evaluate={}){
    const auto stage=[&](Quaternion q,double velocity,double at){
        State state=start;state.t=turned(start.t,q);state.u=turned(start.u,q);state.v=velocity;
        const auto d=derivative(state,control(r,at,evaluate),r);
        require(norm(d.omega)*dt<=.1,"FVD_RESOLUTION","FVD angular step exceeds .1 rad; use a smaller step");
        Quaternion dq{-dot(d.omega,q.v)*.5,(d.omega*q.w+cross(d.omega,q.v))*.5};
        return Stage{dq,state.t*velocity,d.dv,velocity,loss(r,velocity)*velocity};
    };
    const Quaternion identity{};
    const auto a=stage(identity,start.v,time);
    const auto b=stage(identity+a.dq*(dt*.5),start.v+a.dv*dt*.5,time+dt*.5);
    const auto c=stage(identity+b.dq*(dt*.5),start.v+b.dv*dt*.5,time+dt*.5);
    const auto d=stage(identity+c.dq*dt,start.v+c.dv*dt,time+dt);
    const auto rotation=normalized(identity+(a.dq+b.dq*2+c.dq*2+d.dq)*(dt/6));
    State out=start;
    out.p=out.p+(a.dp+b.dp*2+c.dp*2+d.dp)*(dt/6);
    out.v+=(a.dv+2*b.dv+2*c.dv+d.dv)*(dt/6);
    out.s+=(a.ds+2*b.ds+2*c.ds+d.ds)*(dt/6);
    out.work+=(a.workRate+2*b.workRate+2*c.workRate+d.workRate)*(dt/6);
    out.t=unit(turned(start.t,rotation));out.u=turned(start.u,rotation);
    out.u=unit(out.u-out.t*dot(out.t,out.u));
    require(finite(out.p)&&norm(out.p)<=100000,"FVD_POSITION","FVD section left the bounded position domain");
    derivative(out,control(r,time+dt,evaluate),r);return out;
}
FvdSample sample(const State& state,const FvdRequest& r,double time,const ControlEvaluator& evaluate={}){
    return {time,state.s,state.v,state.p,state.t,state.u,derivative(state,control(r,time,evaluate),r).k,state.work};
}
void assess(FvdResult& result,const FvdRequest& r,const Cancel& cancel,const ControlEvaluator& evaluate){
    auto& a=result.assessment;
    const double initialEnergy=.5*r.speed*r.speed+gravity*r.position.z;
    for(const auto& q:result.samples)a.maxEnergyDrift=std::max(a.maxEnergyDrift,
        std::abs(.5*q.speed*q.speed+gravity*q.position.z+q.dissipatedWorkPerMass-initialEnergy));
    // Independent forward replay on the canonical geometry, with twice the
    // authoring time resolution. Speed comes from replay gravity and configured losses, not from the
    // force controls or stored integration samples. No target force is reused
    // as a measured force. This is a point replay, NOT simulate()/acceptance.
    double distance=0,speed=r.speed;
    const auto measure=[&](double time,double at,double velocity){
        require(at>=0&&at<=result.track.length,"FVD_REPLAY_DOMAIN","Canonical replay left the open section before its requested end");
        const auto q=sampleKinematics(result.track,at);const auto target=control(r,time,evaluate);
        const double dv=dot(gVector,q.sample.tangent)-loss(r,velocity);
        const Vec3 specific=q.sample.curvature*(velocity*velocity)+q.sample.tangent*dv-gVector;
        const double normal=dot(specific,q.sample.up)/gravity,lateral=dot(specific,q.sample.right)/gravity;
        const double roll=dot(q.upS,q.sample.right)*velocity;
        require(std::isfinite(normal)&&std::isfinite(lateral)&&std::isfinite(roll),"FVD_REPLAY_DOMAIN","Nonfinite canonical replay measurement");
        a.maxNormalResidualG=std::max(a.maxNormalResidualG,std::abs(normal-target.normalG));
        a.maxLateralResidualG=std::max(a.maxLateralResidualG,std::abs(lateral-target.lateralG));
        a.maxRollResidualRadPerSecond=std::max(a.maxRollResidualRadPerSecond,std::abs(roll-target.rollRate));
        ++a.evaluations;
    };
    measure(0,0,speed);
    for(size_t i=1;i<result.samples.size();++i){
        const double begin=result.samples[i-1].time,dt=(result.samples[i].time-begin)*.5;
        for(int j=0;j<2;++j){
            poll(cancel);const double time=begin+j*dt;
            require(distance>=0&&distance<=result.track.length,"FVD_REPLAY_DOMAIN","Canonical replay ended early");
            const auto q=result.track.sample(distance);const double acceleration=dot(gVector,q.tangent)-loss(r,speed);
            const double midDistance=distance+speed*dt*.5,midSpeed=speed+acceleration*dt*.5;
            require(bounded(midSpeed,.5,250)&&midDistance<=result.track.length,"FVD_REPLAY_DOMAIN","Canonical midpoint replay left its supported domain");
            const auto mid=result.track.sample(midDistance);
            measure(time+dt*.5,midDistance,midSpeed);
            distance+=midSpeed*dt;speed+=(dot(gVector,mid.tangent)-loss(r,midSpeed))*dt;
            require(bounded(speed,.5,250),"FVD_SPEED","Canonical replay speed left supported domain");
        }
    }
    a.endDistanceError=std::abs(distance-result.track.length);
    a.endSpeedError=std::abs(speed-result.samples.back().speed);
    a.performed=true;
    a.passed=a.maxNormalResidualG<=r.forceToleranceG&&a.maxLateralResidualG<=r.forceToleranceG&&
        a.maxRollResidualRadPerSecond<=r.rollToleranceRadPerSecond&&
        a.endDistanceError<=r.replayDistanceTolerance&&a.endSpeedError<=r.replaySpeedTolerance;
    if(!a.passed)result.report.fail("FVD_RESIDUAL","Canonical point replay exceeds a requested force, roll, distance or speed residual tolerance");
}
FvdResult designSection(const FvdRequest& request,const Cancel& cancel,const ControlEvaluator& evaluate){
    FvdResult result;result.track.closed=false;
    try{
        poll(cancel);validate(request);
        State state{request.position,unit(request.forward),unit(request.up),request.speed,0};
        state.u=unit(state.u-state.t*dot(state.t,state.u));
        result.samples.push_back(sample(state,request,0,evaluate));
        // Uniform dt avoids pathological tiny terminal spans. Split each
        // control interval into at least three steps to preserve its boundary.
        for(size_t section=1;section<request.controls.size();++section){
            const double begin=request.controls[section-1].time,duration=request.controls[section].time-begin;
            const size_t count=std::max<size_t>(3,size_t(std::ceil(duration/request.step)));
            require(count<=request.maxSamples-result.samples.size(),"FVD_BUDGET","FVD control boundaries exceed sample budget");
            const double dt=duration/double(count);
            for(size_t j=0;j<count;++j){
                poll(cancel);state=advance(state,request,begin+double(j)*dt,dt,evaluate);
                result.samples.push_back(sample(state,request,j+1==count?request.controls[section].time:begin+double(j+1)*dt,evaluate));
            }
        }
        result.integrated=true;
        result.track.knots.reserve(result.samples.size());
        for(const auto& q:result.samples){poll(cancel);result.track.knots.push_back({q.position,q.forward,q.curvature,q.up,0,Element::Return});}
        poll(cancel);result.track.rebuild();poll(cancel);result.canonicalBuilt=true;
        assess(result,request,cancel,evaluate);
        result.report.warnings.push_back("Experimental point-mass open section with configured rolling/quadratic drag and centerline force reference. Sampled residuals do not establish finite-train, offset-rider, propulsion, clearance or full ride acceptance.");
    }catch(const Failure& e){result.cancelled=e.code=="CANCELLED";result.report.fail(e.code,e.what());}
    catch(const std::exception& e){result.report.fail("FVD_CANONICAL",e.what());}
    return result;
}
}
FvdResult designFvdSection(const FvdRequest& request,Cancel cancel){return designSection(request,cancel,{});}
FvdAirtimeResult designFvdAirtime(const FvdAirtimeRequest& input,Cancel cancel){
    FvdAirtimeResult out;
    try{
        poll(cancel);
        require(bounded(input.speed,45,90)&&bounded(input.pushG,1.8,2.8)&&
            bounded(input.crestG,-.3,.2)&&bounded(input.guardSeconds,.05,.3)&&
            bounded(input.pushRampSeconds,.6,1.2)&&bounded(input.pushHoldSeconds,.2,.6)&&
            bounded(input.crestRampSeconds,.8,1.6),"FVD_AIRTIME_INPUT","Airtime controls left the bounded design domain");
        auto& r=out.authoring;r.position={0,0,0};r.speed=input.speed;
        double t=input.guardSeconds;
        r.controls={{0,1,0,0},{t,1,0,0}};
        t+=input.pushRampSeconds;r.controls.push_back({t,input.pushG,0,0});
        t+=input.pushHoldSeconds;r.controls.push_back({t,input.pushG,0,0});
        t+=input.crestRampSeconds;r.controls.push_back({t,input.crestG,0,0});
        // Integrate only the prefix while shooting. Canonical fitting and its
        // independent sampled replay happen once after the apex is solved.
        State prefix{r.position,r.forward,r.up,r.speed,0};
        for(size_t i=1;i<r.controls.size();++i){
            double begin=r.controls[i-1].time,duration=r.controls[i].time-begin;
            size_t count=size_t(std::ceil(duration/.0025));double dt=duration/count;
            for(size_t j=0;j<count;++j){poll(cancel);prefix=advance(prefix,r,begin+j*dt,dt);}
        }
        require(prefix.t.z>0,"FVD_AIRTIME_SHOOT","Force prefix must still ascend before the crest hold");
        auto atHold=[&](double duration){
            State state=prefix;size_t count=std::max<size_t>(1,size_t(std::ceil(duration/.0025)));double dt=duration/count;
            for(size_t j=0;j<count;++j){poll(cancel);state=advance(state,r,t+j*dt,dt);}return state;
        };
        double low=0,high=6;
        require(atHold(high).t.z<0,"FVD_AIRTIME_SHOOT","Crest hold did not bracket a horizontal apex");
        for(int i=0;i<34;++i){double mid=(low+high)*.5;if(atHold(mid).t.z>0)low=mid;else high=mid;}
        const double apex=t+(low+high)*.5;
        require(apex-t>=.001,"FVD_AIRTIME_SHOOT","Airtime apex hold is too short");
        const auto firstHalf=r.controls;
        r.controls.push_back({apex,input.crestG,0,0});
        for(auto it=firstHalf.rbegin();it!=firstHalf.rend();++it)r.controls.push_back({2*apex-it->time,it->normalG,0,0});
        out.section=designFvdSection(r,cancel);
        if(!out.section.report.valid()||out.section.samples.empty())return out;
        const auto& end=out.section.samples.back();
        require(std::abs(end.position.z)<1e-4&&norm(end.forward-Vec3{1,0,0})<1e-5&&
            std::abs(end.speed-input.speed)<1e-4,"FVD_AIRTIME_PORT","Force-authored hill did not close its level ports");
        out.span=end.position.x;
        for(const auto& q:out.section.samples){
            require(q.forward.x>.65&&q.position.z>=-1e-4,"FVD_AIRTIME_SHAPE","Airtime hill left its forward single-hill domain");
            out.height=std::max(out.height,q.position.z);
        }
    }catch(const Failure& e){out.section.cancelled=e.code=="CANCELLED";out.section.report.fail(e.code,e.what());}
    catch(const std::exception& e){out.section.report.fail("FVD_AIRTIME",e.what());}
    return out;
}

// Gravity-coupled pitch follows Nordmark/Essen Eq. (8), arXiv:1007.1394.
// Smooth force ramps add zero-derivative ports; this is point-mass authoring,
// not a rider standard or a finite-train force prescription.
FvdPitchResult designFvdPitch(const FvdPitchRequest& input,Cancel cancel){
    FvdPitchResult out;
    try{
        poll(cancel);
        require(bounded(input.height,20,250)&&bounded(input.apexSpeed,12,40)&&
            bounded(input.normalG,2.5,4.2)&&bounded(input.pushRampSeconds,.6,2)&&bounded(input.apexNormalG,-1,1.5),
            "FVD_PITCH_INPUT","Pitch intent left the bounded height, speed, force or ramp domain");
        auto& r=out.authoring;r.position={0,0,0};
        r.speed=std::sqrt(input.apexSpeed*input.apexSpeed+2*gravity*input.height);r.step=.0025;
        auto controls=[&](double hold,double fall){const double rise=input.pushRampSeconds;
            r.controls={{0,1,0,0},{rise,input.normalG,0,0},{rise+hold,input.normalG,0,0},{rise+hold+fall,input.apexNormalG,0,0}};};
        struct Shot {State state;double angle;};
        auto shoot=[&](double hold,double fall){
            controls(hold,fall);State state{r.position,r.forward,r.up,r.speed,0};double angle=0;
            for(size_t i=1;i<r.controls.size();++i){double begin=r.controls[i-1].time,duration=r.controls[i].time-begin;
                size_t count=size_t(std::ceil(duration/.005));double dt=duration/count;
                for(size_t j=0;j<count;++j){poll(cancel);auto next=advance(state,r,begin+j*dt,dt);
                    angle+=std::atan2(-cross(state.t,next.t).y,dot(state.t,next.t));state=next;}
            }
            return Shot{state,angle};
        };
        auto residual=[&](const Shot& q){return Vec3{q.angle-pi,(q.state.p.z-input.height)/input.height,0};};
        double hold=input.apexNormalG<0?3:1,fall=input.apexNormalG<0?2:1;bool solved=false;
        for(int iteration=0;iteration<30;++iteration){
            auto q=shoot(hold,fall);Vec3 error=residual(q);
            if(norm(error)<1e-9){solved=true;break;}
            constexpr double epsilon=.001;auto qh=shoot(hold+epsilon,fall),qf=shoot(hold,fall+epsilon);
            Vec3 h=(residual(qh)-error)/epsilon,f=(residual(qf)-error)/epsilon;
            double determinant=h.x*f.y-f.x*h.y;
            if(std::abs(determinant)<1e-12)break;
            const double dh=(-error.x*f.y+f.x*error.y)/determinant,df=(-h.x*error.y+error.x*h.y)/determinant;
            bool improved=false;
            for(int backtrack=0;backtrack<16;++backtrack){
                double scale=std::ldexp(1.,-backtrack),nextHold=hold+scale*dh,nextFall=fall+scale*df;
                if(!bounded(nextHold,.05,20)||!bounded(nextFall,.05,20))continue;
                try{auto candidate=shoot(nextHold,nextFall);
                    if(norm(residual(candidate))<norm(error)){hold=nextHold;fall=nextFall;improved=true;break;}}
                catch(const Failure& e){if(e.code=="CANCELLED")throw;}
            }
            if(!improved)break;
        }
        require(solved,"FVD_PITCH_SHOOT","Requested pitch height, apex speed and normal-force history have no converged bounded duration solution");
        controls(hold,fall);out.holdSeconds=hold;out.exitRampSeconds=fall;
        out.section=designFvdSection(r,cancel);
        if(!out.section.report.valid()||out.section.samples.empty())return out;
        const auto& end=out.section.samples.back();
        require(std::abs(end.position.z-input.height)<1e-5&&norm(end.forward-Vec3{-1,0,0})<1e-7&&
            std::abs(end.speed-input.apexSpeed)<1e-5,"FVD_PITCH_PORT","Canonical pitch source did not close its requested height, direction and energy");
        double lastAngle=0;
        for(const auto& q:out.section.samples){
            double angle=std::atan2(q.forward.z,q.forward.x);if(angle<-.1)angle+=2*pi;
            require(angle>=lastAngle-1e-7&&angle<=pi+1e-7&&q.position.z>=-1e-7&&q.position.z<=input.height+1e-5,
                "FVD_PITCH_SHAPE","Pitch source left its monotone ascending half-loop domain");lastAngle=angle;
        }
        out.forwardDisplacement=end.position.x;out.height=end.position.z;
    }catch(const Failure& e){out.section.cancelled=e.code=="CANCELLED";out.section.report.fail(e.code,e.what());}
    catch(const std::exception& e){out.section.report.fail("FVD_PITCH",e.what());}
    return out;
}

FvdReversalResult designFvdReversal(const FvdReversalRequest& input,Cancel cancel){
    FvdReversalResult out;
    try{
        poll(cancel);
        require(bounded(input.entrySpeed,35,80)&&bounded(input.exitHeight,0,180)&&
            bounded(input.exitPitch,-65*pi/180,-10*pi/180)&&bounded(input.normalG,2.5,4.2)&&
            bounded(input.pushRampSeconds,.6,2)&&bounded(input.rollStartFraction,0,.65)&&
            bounded(input.lateralPulseG,-.5,.5)&&(input.hand==1||input.hand==-1)&&
            bounded(input.step,.0001,.05),"FVD_REVERSAL_INPUT","Reversal intent left its bounded authoring domain");
        require(input.entrySpeed*input.entrySpeed-2*gravity*input.exitHeight>.25,
            "FVD_REVERSAL_ENERGY","Entry energy cannot reach the requested exit height");
        using Parameters=std::array<double,3>; // Force hold, unloading duration, total twist magnitude.
        struct Profile {FvdRequest request;ControlEvaluator evaluate;};
        const auto profile=[&](const Parameters& p){
            Profile result;auto& r=result.request;r.position={0,0,0};r.speed=input.entrySpeed;r.step=input.step;
            const double rise=input.pushRampSeconds,begin=rise+p[0],end=begin+p[1];
            const double rollBegin=begin+input.rollStartFraction*p[1],rollDuration=end-rollBegin;
            const double terminalNormal=std::cos(input.exitPitch),normal=input.normalG;
            const double angle=p[2],lateral=input.lateralPulseG,hand=double(input.hand);
            // These times partition integration; the evaluator supplies all three
            // continuous controls. They are not a dense zero-jet control table.
            r.controls={{0,1,0,0},{rise,normal,0,0},{begin,normal,0,0},{end,terminalNormal,0,0}};
            result.evaluate=[=](double time){
                const double u=std::clamp((time-rollBegin)/rollDuration,0.,1.);
                const double phi=pi*smooth(u),bell=u*u*(1-u)*(1-u);
                const double force=time<rise?1+(normal-1)*smooth(std::clamp(time/rise,0.,1.)):
                    time<begin?normal:normal+(-terminalNormal-normal)*smooth(std::clamp((time-begin)/(end-begin),0.,1.));
                const double twist=hand*angle*30*bell/rollDuration;
                require(std::abs(twist)<=4*pi,"FVD_REVERSAL_ROLL","Reversal twist exceeds the numerical source domain");
                return FvdControl{time,force*std::cos(phi),hand*(-force*std::sin(phi)+lateral*16*bell),twist};
            };
            return result;
        };
        const auto shoot=[&](const Parameters& p){
            auto source=profile(p);const auto& r=source.request;State state{r.position,r.forward,r.up,r.speed,0};
            for(size_t i=1;i<r.controls.size();++i){
                const double begin=r.controls[i-1].time,duration=r.controls[i].time-begin;
                const size_t count=size_t(std::ceil(duration/.005));const double dt=duration/count;
                for(size_t j=0;j<count;++j){poll(cancel);state=advance(state,r,begin+j*dt,dt,source.evaluate);}
            }
            return state;
        };
        const auto residual=[&](const State& state){
            const Vec3 horizontalRight=unit(cross(state.t,Vec3{0,0,1}));
            return Vec3{(state.p.z-input.exitHeight)/std::max(20.,input.exitHeight),state.t.z-std::sin(input.exitPitch),
                std::atan2(dot(state.u,horizontalRight),state.u.z)};
        };
        const auto inDomain=[](const Parameters& p){return bounded(p[0],.05,8)&&bounded(p[1],.5,12)&&bounded(p[2],1,5.2);};
        const auto solve=[&](Parameters& p){
            for(int iteration=0;iteration<35;++iteration){
                State state;
                try{state=shoot(p);}catch(const Failure& e){if(e.code=="CANCELLED")throw;return false;}
                const Vec3 error=residual(state);
                if(norm(error)<1e-8&&state.t.x<-.35)return true;
                double matrix[3][4]{};constexpr double epsilon=.0001;
                for(int j=0;j<3;++j){
                    auto probe=p;probe[j]+=epsilon;Vec3 difference;
                    try{difference=(residual(shoot(probe))-error)/epsilon;}
                    catch(const Failure& e){if(e.code=="CANCELLED")throw;return false;}
                    matrix[0][j]=difference.x;matrix[1][j]=difference.y;matrix[2][j]=difference.z;
                }
                matrix[0][3]=-error.x;matrix[1][3]=-error.y;matrix[2][3]=-error.z;
                for(int col=0;col<3;++col){
                    int pivot=col;for(int i=col+1;i<3;++i)if(std::abs(matrix[i][col])>std::abs(matrix[pivot][col]))pivot=i;
                    if(std::abs(matrix[pivot][col])<1e-10)return false;
                    for(int j=col;j<4;++j)std::swap(matrix[col][j],matrix[pivot][j]);
                    const double divisor=matrix[col][col];for(int j=col;j<4;++j)matrix[col][j]/=divisor;
                    for(int i=0;i<3;++i)if(i!=col){const double factor=matrix[i][col];for(int j=col;j<4;++j)matrix[i][j]-=factor*matrix[col][j];}
                }
                bool improved=false;
                for(int backtrack=0;backtrack<16;++backtrack){
                    auto next=p;const double scale=std::ldexp(1.,-backtrack);
                    for(int i=0;i<3;++i)next[i]+=scale*matrix[i][3];
                    if(!inDomain(next))continue;
                    try{if(norm(residual(shoot(next)))<norm(error)){p=next;improved=true;break;}}
                    catch(const Failure& e){if(e.code=="CANCELLED")throw;}
                }
                if(!improved)return false;
            }
            return false;
        };
        Parameters parameters{};bool solved=false;
        const double timeScale=input.entrySpeed/48.9107707974;
        for(const auto& seed:std::array<Parameters,3>{{{1.25,4.3,pi},{2.5,2.,pi},{.7,5.5,pi}}}){
            parameters={seed[0]*timeScale,seed[1]*timeScale,seed[2]};
            if(solve(parameters)){solved=true;break;}
        }
        require(solved,"FVD_REVERSAL_SHOOT","Reversal force/roll intent has no converged bounded descending-port solution");
        auto source=profile(parameters);
        out.holdSeconds=parameters[0];out.unloadSeconds=parameters[1];out.totalTwist=input.hand*parameters[2];
        out.duration=source.request.controls.back().time;
        out.section=designSection(source.request,cancel,source.evaluate);
        if(!out.section.report.valid()||out.section.samples.empty())return out;
        const auto& end=out.section.samples.back();
        require(norm(residual(State{end.position,end.forward,end.up,end.speed,end.distance}))<1e-6&&end.forward.x<-.35,
            "FVD_REVERSAL_PORT","Integrated reversal missed its descending upright port");
        out.highestInvertedHeight=-1;out.minSpeed=input.entrySpeed;out.intent.reserve(out.section.samples.size());
        double previousPitch=0;
        for(const auto& q:out.section.samples){
            poll(cancel);out.intent.push_back(source.evaluate(q.time));
            out.geometricApexHeight=std::max(out.geometricApexHeight,q.position.z);
            out.minSpeed=std::min(out.minSpeed,q.speed);
            if(q.up.z<-.5)out.highestInvertedHeight=std::max(out.highestInvertedHeight,q.position.z);
            double pitch=std::atan2(q.forward.z,q.forward.x);if(pitch<-.1)pitch+=2*pi;
            require(q.position.z>=-1e-5&&pitch>=previousPitch-1e-5&&pitch<pi+1.3,
                "FVD_REVERSAL_SHAPE","Reversal left its single ascending-then-descending pitch domain");
            previousPitch=pitch;
        }
        require(out.highestInvertedHeight>=0,"FVD_REVERSAL_SHAPE","Reversal did not produce an inverted passage");
        out.entry=sampleKinematics(out.section.track,0);out.exit=sampleKinematics(out.section.track,out.section.track.length);
    }catch(const Failure& e){out.section.cancelled=e.code=="CANCELLED";out.section.report.fail(e.code,e.what());}
    catch(const std::exception& e){out.section.report.fail("FVD_REVERSAL",e.what());}
    return out;
}

FvdPulloutResult designFvdPullout(const FvdPulloutRequest& input,Cancel cancel){
    FvdPulloutResult out;
    try{
        poll(cancel);const auto& entry=input.entry;
        require(finite(entry.position)&&norm(entry.position)<=100000&&finite(entry.forward)&&finite(entry.up)&&
            std::abs(norm(entry.forward)-1)<1e-6&&std::abs(norm(entry.up)-1)<1e-6&&std::abs(dot(entry.forward,entry.up))<1e-6&&
            bounded(entry.forward.z,-.95,-.05)&&finite(entry.curvature)&&norm(entry.curvature)<1e-6&&
            bounded(entry.speed,10,100)&&bounded(input.normalG,1.5,4.2)&&bounded(input.rampSeconds,.6,2)&&
            bounded(input.step,.0001,.05)&&std::isfinite(input.targetHeight)&&std::abs(input.targetHeight)<=100000&&
            input.targetHeight<entry.position.z,
            "FVD_PULLOUT_INPUT","Pullout needs a straight-compatible descending port and a lower target height");
        const Vec3 expectedUp=unit(Vec3{0,0,1}-entry.forward*entry.forward.z);
        require(norm(expectedUp-entry.up)<1e-6,"FVD_PULLOUT_INPUT","Pullout entry must be upright relative to its descending tangent");
        auto& request=out.authoring;request.position=entry.position;request.forward=entry.forward;request.up=entry.up;
        request.speed=entry.speed;request.step=input.step;
        const double slopeNormal=std::sqrt(1-entry.forward.z*entry.forward.z);
        const auto controls=[&](double hold,double unload){
            request.controls={{0,slopeNormal,0,0},{hold,slopeNormal,0,0},
                {hold+input.rampSeconds,input.normalG,0,0},{hold+input.rampSeconds+unload,1,0,0}};
        };
        const auto shoot=[&](double hold,double unload){
            controls(hold,unload);State state{entry.position,entry.forward,entry.up,entry.speed,0};
            for(size_t i=1;i<request.controls.size();++i){
                const double begin=request.controls[i-1].time,duration=request.controls[i].time-begin;
                const size_t count=size_t(std::ceil(duration/.005));const double dt=duration/count;
                for(size_t j=0;j<count;++j){poll(cancel);state=advance(state,request,begin+j*dt,dt);}
            }
            return state;
        };
        const auto residual=[&](const State& state){return Vec3{state.t.z,
            (state.p.z-input.targetHeight)/std::max(20.,entry.position.z-input.targetHeight),0};};
        double hold=0,unload=0;bool solved=false;
        for(const auto& seed:std::array<std::array<double,2>,3>{{{.4,1.75},{1,2.5},{.05,1.}}}){
            hold=seed[0]*entry.speed/37.57124565;unload=seed[1]*entry.speed/37.57124565;
            for(int iteration=0;iteration<30;++iteration){
                State state;
                try{state=shoot(hold,unload);}catch(const Failure& e){if(e.code=="CANCELLED")throw;break;}
                const Vec3 error=residual(state);
                if(norm(error)<1e-9&&dot(state.t,entry.forward)>0){solved=true;break;}
                constexpr double epsilon=.0001;Vec3 h,u;
                try{h=(residual(shoot(hold+epsilon,unload))-error)/epsilon;u=(residual(shoot(hold,unload+epsilon))-error)/epsilon;}
                catch(const Failure& e){if(e.code=="CANCELLED")throw;break;}
                const double determinant=h.x*u.y-u.x*h.y;if(std::abs(determinant)<1e-12)break;
                const double dh=(-error.x*u.y+u.x*error.y)/determinant,du=(-h.x*error.y+error.x*h.y)/determinant;
                bool improved=false;
                for(int backtrack=0;backtrack<16;++backtrack){
                    const double scale=std::ldexp(1.,-backtrack),nextHold=hold+scale*dh,nextUnload=unload+scale*du;
                    if(!bounded(nextHold,.001,10)||!bounded(nextUnload,.1,10))continue;
                    try{if(norm(residual(shoot(nextHold,nextUnload)))<norm(error)){hold=nextHold;unload=nextUnload;improved=true;break;}}
                    catch(const Failure& e){if(e.code=="CANCELLED")throw;}
                }
                if(!improved)break;
            }
            if(solved)break;
        }
        require(solved,"FVD_PULLOUT_SHOOT","Pullout force intent cannot close the requested lower level port within its bounded durations");
        controls(hold,unload);out.slopeHoldSeconds=hold;out.unloadSeconds=unload;out.duration=request.controls.back().time;
        out.section=designFvdSection(request,cancel);
        if(!out.section.report.valid()||out.section.samples.empty())return out;
        const auto& end=out.section.samples.back();const Vec3 horizontal=unit(Vec3{entry.forward.x,entry.forward.y,0});
        require(std::abs(end.position.z-input.targetHeight)<1e-5&&norm(end.forward-horizontal)<1e-7&&norm(end.up-Vec3{0,0,1})<1e-7,
            "FVD_PULLOUT_PORT","Integrated pullout missed its lower level port");
        double previousSlope=entry.forward.z;
        for(const auto& q:out.section.samples){
            poll(cancel);
            require(q.forward.z>=previousSlope-1e-7&&q.forward.z<=1e-7&&q.position.z>=input.targetHeight-1e-5&&
                dot(q.forward,horizontal)>.3,"FVD_PULLOUT_SHAPE","Pullout left its monotone descending-to-level domain");
            previousSlope=q.forward.z;
        }
        out.entry=sampleKinematics(out.section.track,0);out.exit=sampleKinematics(out.section.track,out.section.track.length);
    }catch(const Failure& e){out.section.cancelled=e.code=="CANCELLED";out.section.report.fail(e.code,e.what());}
    catch(const std::exception& e){out.section.report.fail("FVD_PULLOUT",e.what());}
    return out;
}

FvdTallHillResult designFvdTallHill(const FvdTallHillRequest& input,Cancel cancel){
    FvdTallHillResult out;
    try{
        poll(cancel);
        require(bounded(input.height,220,280)&&bounded(input.speed,75,90)&&
            bounded(input.normalG,2.5,4)&&bounded(input.crestG,-.3,.2)&&
            bounded(input.rampSeconds,.8,2),"FVD_TALL_INPUT","Tall-hill request left its bounded authoring family");
        auto& r=out.authoring;r.position={0,0,0};r.speed=input.speed;r.step=.0025;r.maxSamples=30000;
        r.rollingAcceleration=input.rollingAcceleration;r.dragAccelerationCoefficient=input.dragAccelerationCoefficient;
        validate(r);
        const double minimumEntrySquared=input.dragAccelerationCoefficient>0?
            (gravity+input.rollingAcceleration)*std::expm1(2*input.dragAccelerationCoefficient*input.height)/input.dragAccelerationCoefficient:
            2*(gravity+input.rollingAcceleration)*input.height;
        require(input.speed*input.speed>minimumEntrySquared,"FVD_TALL_ENERGY",
            "Requested height exceeds available energy even on an ideal vertical ascent with configured losses");
        struct Shot {State state;FvdRequest request;double time{};bool valid{};};
        const auto trialStep=[&](State& state,const FvdRequest& request,double time,double dt){
            poll(cancel);
            try{state=advance(state,request,time,dt);}
            catch(const Failure& e){if(e.code=="FVD_SPEED"||e.code=="FVD_CURVATURE")return false;throw;}
            return state.t.x>.02;
        };
        const auto ascent=[&](double hold){
            Shot shot;auto& q=shot.request;q=r;
            double time=.1;q.controls={{0,1,0,0},{time,1,0,0}};
            time+=input.rampSeconds;q.controls.push_back({time,input.normalG,0,0});
            time+=hold;q.controls.push_back({time,input.normalG,0,0});
            time+=input.rampSeconds;q.controls.push_back({time,input.crestG,0,0});
            State state{q.position,q.forward,q.up,q.speed,0};
            for(size_t i=1;i<q.controls.size();++i){
                const double start=q.controls[i-1].time,duration=q.controls[i].time-start;
                const size_t count=size_t(std::ceil(duration/.005));const double dt=duration/count;
                for(size_t j=0;j<count;++j)
                    if(!trialStep(state,q,start+j*dt,dt)||state.t.z< -1e-8)return shot;
            }
            if(state.t.z<=0)return shot;
            for(int i=0;i<4000;++i){
                State next=state;if(!trialStep(next,q,time,.005))return shot;
                if(next.t.z<=0){
                    double low=0,high=.005;
                    for(int j=0;j<28;++j){
                        const double middle=(low+high)*.5;State at=state;
                        if(!trialStep(at,q,time,middle))return shot;
                        if(at.t.z>0)low=middle;else high=middle;
                    }
                    shot.time=time+(low+high)*.5;shot.state=state;
                    shot.valid=trialStep(shot.state,q,time,(low+high)*.5);return shot;
                }
                state=next;time+=.005;
            }
            return shot;
        };
        Shot apex;double previous=.01;
        for(double hold=.01;hold<=6;hold+=.025){
            auto trial=ascent(hold);
            if(trial.valid&&trial.state.p.z>=input.height){
                double low=previous,high=hold;
                for(int i=0;i<28;++i){
                    const double middle=(low+high)*.5;auto q=ascent(middle);
                    if(q.valid&&q.state.p.z>=input.height)high=middle;else low=middle;
                }
                apex=ascent((low+high)*.5);break;
            }
            previous=hold;
        }
        require(apex.valid&&std::abs(apex.state.p.z-input.height)<1e-5,"FVD_TALL_ASCENT",
            "Requested height has no forward ascent within bounded force, energy and duration controls");
        const auto descent=[&](double crestHold,double pullHold){
            Shot shot;if(!bounded(crestHold,.01,10)||!bounded(pullHold,.01,6))return shot;
            auto& q=shot.request;q=apex.request;double time=apex.time;
            q.controls.push_back({time,input.crestG,0,0});
            time+=crestHold;q.controls.push_back({time,input.crestG,0,0});
            time+=input.rampSeconds;q.controls.push_back({time,input.normalG,0,0});
            time+=pullHold;q.controls.push_back({time,input.normalG,0,0});
            time+=input.rampSeconds;q.controls.push_back({time,1,0,0});
            time+=.1;q.controls.push_back({time,1,0,0});
            if(time>60)return shot;
            State state=apex.state;
            for(double at=apex.time;at<time-1e-9;){
                const double dt=std::min(.005,time-at);
                if(!trialStep(state,q,at,dt))return shot;
                at+=dt;
            }
            shot.state=state;shot.time=time;shot.valid=true;return shot;
        };
        const auto pitchResidual=[](const State& s){return 100*std::atan2(s.t.z,s.t.x);};
        Shot finish;
        for(double scale:{1.,.8,1.2}){
            double crestHold=(apex.time-apex.request.controls.back().time)*scale;
            double pullHold=apex.request.controls[3].time-apex.request.controls[2].time;
            for(int iteration=0;iteration<30;++iteration){
                auto q=descent(crestHold,pullHold);if(!q.valid)break;
                const double z=q.state.p.z,pitch=pitchResidual(q.state),score=std::hypot(z,pitch);
                if(score<1e-6){finish=std::move(q);break;}
                constexpr double epsilon=.002;
                auto c=descent(crestHold+epsilon,pullHold),p=descent(crestHold,pullHold+epsilon);
                if(!c.valid||!p.valid)break;
                const double a=(c.state.p.z-z)/epsilon,b=(p.state.p.z-z)/epsilon;
                const double d=(pitchResidual(c.state)-pitch)/epsilon,e=(pitchResidual(p.state)-pitch)/epsilon;
                const double determinant=a*e-b*d;if(std::abs(determinant)<1e-8)break;
                const double dc=(-z*e+b*pitch)/determinant,dp=(-a*pitch+z*d)/determinant;
                bool improved=false;
                for(double factor=1;factor>1./128;factor*=.5){
                    auto next=descent(crestHold+factor*dc,pullHold+factor*dp);
                    if(next.valid&&std::hypot(next.state.p.z,pitchResidual(next.state))<score){
                        crestHold+=factor*dc;pullHold+=factor*dp;improved=true;break;
                    }
                }
                if(!improved)break;
            }
            if(finish.valid)break;
        }
        require(finish.valid,"FVD_TALL_DESCENT","No independently solved descent closes level ports within bounded controls");
        r=std::move(finish.request);out.section=designFvdSection(r,cancel);
        if(!out.section.report.valid()||out.section.samples.empty())return out;
        out.exit=out.section.samples.back();out.span=out.exit.position.x;
        bool descending=false;
        for(const auto& q:out.section.samples){
            poll(cancel);out.height=std::max(out.height,q.position.z);
            if(q.forward.z< -1e-6)descending=true;
            require(q.forward.x>.02&&q.position.z>=-1e-5&&(!descending||q.forward.z<1e-6),
                "FVD_TALL_SHAPE","Tall hill left its forward single-apex geometry domain");
        }
        require(std::abs(out.height-input.height)<1e-4&&std::abs(out.exit.position.z)<1e-5&&
            norm(out.exit.forward-Vec3{1,0,0})<1e-6&&norm(out.exit.curvature)<1e-8,
            "FVD_TALL_PORT","Loss-aware tall hill did not close its requested height and level curvature ports");
    }catch(const Failure& e){out.section.cancelled=e.code=="CANCELLED";out.section.report.fail(e.code,e.what());}
    catch(const std::exception& e){out.section.report.fail("FVD_TALL",e.what());}
    return out;
}
}
