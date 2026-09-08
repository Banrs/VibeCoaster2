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
FvdControl control(const FvdRequest& request,double time){
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
struct State {Vec3 p,t,u;double v{},s{};};
struct Derivative {Vec3 omega,k;double dv{};};
Derivative derivative(const State& state,const FvdControl& input){
    require(bounded(state.v,.5,250),"FVD_SPEED","FVD speed left [.5,250] m/s; no low-speed clamp is applied");
    const Vec3 right=cross(state.t,state.u);
    const Vec3 a=gVector+state.u*(gravity*input.normalG)+right*(gravity*input.lateralG);
    const double dv=dot(gVector,state.t);
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
struct Stage {Quaternion dq;Vec3 dp;double dv{},ds{};};
// Classical RK4 on position, speed and a world-space quaternion increment.
// The smooth off-manifold extension uses normalized q for the rotation but
// raw q in qdot=1/2*(0,omega)*q. Final normalization restores SO(3), removing
// the quaternion norm truncation error without changing fourth-order accuracy.
State advance(const State& start,const FvdRequest& r,double time,double dt){
    const auto stage=[&](Quaternion q,double velocity,double at){
        State state=start;state.t=turned(start.t,q);state.u=turned(start.u,q);state.v=velocity;
        const auto d=derivative(state,control(r,at));
        require(norm(d.omega)*dt<=.1,"FVD_RESOLUTION","FVD angular step exceeds .1 rad; use a smaller step");
        Quaternion dq{-dot(d.omega,q.v)*.5,(d.omega*q.w+cross(d.omega,q.v))*.5};
        return Stage{dq,state.t*velocity,d.dv,velocity};
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
    out.t=unit(turned(start.t,rotation));out.u=turned(start.u,rotation);
    out.u=unit(out.u-out.t*dot(out.t,out.u));
    require(finite(out.p)&&norm(out.p)<=100000,"FVD_POSITION","FVD section left the bounded position domain");
    derivative(out,control(r,time+dt));return out;
}
FvdSample sample(const State& state,const FvdRequest& r,double time){
    return {time,state.s,state.v,state.p,state.t,state.u,derivative(state,control(r,time)).k};
}
void assess(FvdResult& result,const FvdRequest& r,const Cancel& cancel){
    auto& a=result.assessment;
    const double initialEnergy=.5*r.speed*r.speed+gravity*r.position.z;
    for(const auto& q:result.samples)a.maxEnergyDrift=std::max(a.maxEnergyDrift,
        std::abs(.5*q.speed*q.speed+gravity*q.position.z-initialEnergy));
    // Independent forward replay on the canonical geometry, with twice the
    // authoring time resolution. Speed comes from replay gravity, not from the
    // force controls or stored integration samples. No target force is reused
    // as a measured force. This is a point replay, NOT simulate()/acceptance.
    double distance=0,speed=r.speed;
    const auto measure=[&](double time,double at,double velocity){
        require(at>=0&&at<=result.track.length,"FVD_REPLAY_DOMAIN","Canonical replay left the open section before its requested end");
        const auto q=sampleKinematics(result.track,at);const auto target=control(r,time);
        const double dv=dot(gVector,q.sample.tangent);
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
            const auto q=result.track.sample(distance);const double acceleration=dot(gVector,q.tangent);
            const double midDistance=distance+speed*dt*.5,midSpeed=speed+acceleration*dt*.5;
            require(bounded(midSpeed,.5,250)&&midDistance<=result.track.length,"FVD_REPLAY_DOMAIN","Canonical midpoint replay left its supported domain");
            const auto mid=result.track.sample(midDistance);
            measure(time+dt*.5,midDistance,midSpeed);
            distance+=midSpeed*dt;speed+=dot(gVector,mid.tangent)*dt;
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
}
FvdResult designFvdSection(const FvdRequest& request,Cancel cancel){
    FvdResult result;result.track.closed=false;
    try{
        poll(cancel);validate(request);
        State state{request.position,unit(request.forward),unit(request.up),request.speed,0};
        state.u=unit(state.u-state.t*dot(state.t,state.u));
        result.samples.push_back(sample(state,request,0));
        // Uniform dt avoids pathological tiny terminal spans. Split each
        // control interval into at least three steps to preserve its boundary.
        for(size_t section=1;section<request.controls.size();++section){
            const double begin=request.controls[section-1].time,duration=request.controls[section].time-begin;
            const size_t count=std::max<size_t>(3,size_t(std::ceil(duration/request.step)));
            require(count<=request.maxSamples-result.samples.size(),"FVD_BUDGET","FVD control boundaries exceed sample budget");
            const double dt=duration/double(count);
            for(size_t j=0;j<count;++j){
                poll(cancel);state=advance(state,request,begin+double(j)*dt,dt);
                result.samples.push_back(sample(state,request,j+1==count?request.controls[section].time:begin+double(j+1)*dt));
            }
        }
        result.integrated=true;
        result.track.knots.reserve(result.samples.size());
        for(const auto& q:result.samples){poll(cancel);result.track.knots.push_back({q.position,q.forward,q.curvature,q.up,0,Element::Return});}
        poll(cancel);result.track.rebuild();poll(cancel);result.canonicalBuilt=true;
        assess(result,request,cancel);
        result.report.warnings.push_back("Experimental gravity-only point-mass open section, with centerline force reference. Sampled residuals do not establish finite-train, offset-rider, propulsion, clearance or full ride acceptance.");
    }catch(const Failure& e){result.cancelled=e.code=="CANCELLED";result.report.fail(e.code,e.what());}
    catch(const std::exception& e){result.report.fail("FVD_CANONICAL",e.what());}
    return result;
}
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

}
