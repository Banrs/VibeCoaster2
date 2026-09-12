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
template<size_t N> bool solveLinear(double (&matrix)[N][N+1]){
    for(size_t col=0;col<N;++col){
        size_t pivot=col;for(size_t i=col+1;i<N;++i)if(std::abs(matrix[i][col])>std::abs(matrix[pivot][col]))pivot=i;
        if(std::abs(matrix[pivot][col])<1e-10)return false;
        for(size_t j=col;j<=N;++j)std::swap(matrix[col][j],matrix[pivot][j]);
        const double divisor=matrix[col][col];for(size_t j=col;j<=N;++j)matrix[col][j]/=divisor;
        for(size_t i=0;i<N;++i)if(i!=col){const double factor=matrix[i][col];for(size_t j=col;j<=N;++j)matrix[i][j]-=factor*matrix[col][j];}
    }
    return true;
}
template<size_t N,class Residual> bool shootParameters(std::array<double,N>& parameters,
    const std::array<std::pair<double,double>,N>& bounds,const Residual& residual){
    for(size_t j=0;j<N;++j)if(!bounded(parameters[j],bounds[j].first,bounds[j].second))return false;
    const auto score=[](const std::array<double,N>& error){double sum=0;for(double e:error)sum+=e*e;return sum;};
    try{
        for(int iteration=0;iteration<30;++iteration){
            const auto error=residual(parameters);const double current=score(error);
            if(!std::isfinite(current))return false;
            if(current<1e-18)return true;
            double matrix[N][N+1]{};constexpr double epsilon=.0001;
            for(size_t j=0;j<N;++j){auto next=parameters;next[j]+=epsilon;const auto shifted=residual(next);
                for(size_t i=0;i<N;++i)matrix[i][j]=(shifted[i]-error[i])/epsilon;}
            for(size_t i=0;i<N;++i)matrix[i][N]=-error[i];
            if(!solveLinear<N>(matrix))return false;
            bool improved=false;
            for(int backtrack=0;backtrack<16;++backtrack){
                const double scale=std::ldexp(1.,-backtrack);auto next=parameters;bool inside=true;
                for(size_t j=0;j<N;++j){next[j]+=scale*matrix[j][N];inside&=bounded(next[j],bounds[j].first,bounds[j].second);}
                if(!inside)continue;
                try{if(score(residual(next))<current){parameters=next;improved=true;break;}}
                catch(const Failure& e){if(e.code=="CANCELLED")throw;}
            }
            if(!improved)return false;
        }
    }catch(const Failure& e){if(e.code=="CANCELLED")throw;}
    return false;
}
FvdControl control(const FvdRequest& request,double time){
    auto hi=std::upper_bound(request.controls.begin(),request.controls.end(),time,
        [](double t,const FvdControl& c){return t<c.time;});
    FvdControl out;
    if(hi==request.controls.end())out=request.controls.back();
    else if(hi==request.controls.begin())out=*hi;
    else{
        const auto& a=*(hi-1);const auto& b=*hi;
        const double u=smooth((time-a.time)/(b.time-a.time));
        out={time,a.normalG+(b.normalG-a.normalG)*u,a.lateralG+(b.lateralG-a.lateralG)*u,
            a.rollRate+(b.rollRate-a.rollRate)*u};
    }
    if(!request.twists.empty()){
        out.rollRate=0;
        auto phase=std::upper_bound(request.twists.begin(),request.twists.end(),time,[](double t,const FvdTwistPhase& p){return t<p.begin;});
        if(phase!=request.twists.begin()){
            --phase;if(time<=phase->end){const double u=(time-phase->begin)/(phase->end-phase->begin);
                out.rollRate=phase->angle*30*u*u*(1-u)*(1-u)/(phase->end-phase->begin);}
        }
    }
    return out;
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
        require(r.twists.empty()||c.rollRate==0,"FVD_INPUT","Twist phases and roll-rate controls cannot both own physical twist");
    }
    require(r.twists.size()<=64,"FVD_INPUT","Too many physical twist phases");previous=0;
    for(const auto& phase:r.twists){
        require(bounded(phase.begin,previous,r.controls.back().time)&&bounded(phase.end,phase.begin+.001,r.controls.back().time)&&
            bounded(phase.angle,-4*pi,4*pi)&&std::abs(phase.angle)*1.875/(phase.end-phase.begin)<=4*pi,
            "FVD_INPUT","Invalid, overlapping or excessive physical twist phase");previous=phase.end;
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
State advance(const State& start,const FvdRequest& r,double time,double dt){
    const auto stage=[&](Quaternion q,double velocity,double at){
        State state=start;state.t=turned(start.t,q);state.u=turned(start.u,q);state.v=velocity;
        const auto d=derivative(state,control(r,at),r);
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
    derivative(out,control(r,time+dt),r);return out;
}
FvdSample sample(const State& state,const FvdRequest& r,double time){
    return {time,state.s,state.v,state.p,state.t,state.u,derivative(state,control(r,time),r).k,state.work};
}
void assess(FvdResult& result,const FvdRequest& r,const Cancel& cancel){
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
        const auto q=sampleKinematics(result.track,at);const auto target=control(r,time);
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
        result.report.warnings.push_back("Experimental point-mass open section with configured rolling/quadratic drag and centerline force reference. Sampled residuals do not establish finite-train, offset-rider, propulsion, clearance or full ride acceptance.");
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


FvdImmelmannResult designFvdImmelmann(const FvdImmelmannRequest& input,Cancel cancel){
    FvdImmelmannResult out;
    try{
        poll(cancel);
        require(bounded(input.entrySpeed,45,80)&&bounded(input.height,75,180)&&bounded(input.exitHeight,0,40)&&
            input.exitHeight<input.height&&bounded(input.normalG,3,5.5)&&bounded(input.crestG,.05,1.5)&&
            bounded(input.rollExitG,.05,1)&&bounded(input.rampSeconds,.8,2)&&bounded(input.rollOverlapFraction,0,.6)&&
            (input.hand==1||input.hand==-1)&&bounded(input.step,.0001,.05),"FVD_IMMELMANN_INPUT","Immelmann intent left its bounded authoring family");
        auto& r=out.authoring;r.position={0,0,0};r.speed=input.entrySpeed;r.step=input.step;
        r.rollingAcceleration=input.rollingAcceleration;r.dragAccelerationCoefficient=input.dragAccelerationCoefficient;validate(r);
        using Parameters=std::array<double,5>;
        // Unknowns: peak hold, apex unload, roll duration, physical twist and
        // valley unload. All other load and phase intents remain prescribed.
        // Residuals: apex height/pitch, upright roll exit, valley height/pitch.
        struct Shot {State apex,rollExit,exit;double minimumZ{},maximumDescentPitch{};};
        const auto controls=[&](const Parameters& p){
            const double apex=input.rampSeconds+p[0]+p[1],rollEnd=apex+p[2],pullPeak=rollEnd+input.rampSeconds;
            r.controls={{0,1,0,0},{input.rampSeconds,input.normalG,0,0},{input.rampSeconds+p[0],input.normalG,0,0},
                {apex,input.crestG,0,0},{rollEnd,input.rollExitG,0,0},{pullPeak,input.normalG,0,0},{pullPeak+p[4],1,0,0}};
            r.twists={{apex-input.rollOverlapFraction*p[1],rollEnd,input.hand*p[3]}};
        };
        const auto shoot=[&](const Parameters& p){
            controls(p);State state{r.position,r.forward,r.up,r.speed,0};Shot result;result.minimumZ=input.height;
            for(size_t i=1;i<r.controls.size();++i){
                const double begin=r.controls[i-1].time,duration=r.controls[i].time-begin;
                const size_t count=size_t(std::ceil(duration/.005));const double dt=duration/count;
                for(size_t j=0;j<count;++j){poll(cancel);state=advance(state,r,begin+j*dt,dt);
                    if(i>3){result.minimumZ=std::min(result.minimumZ,state.p.z);result.maximumDescentPitch=std::max(result.maximumDescentPitch,state.t.z);}}
                if(i==3)result.apex=state;if(i==4)result.rollExit=state;
            }
            result.exit=state;return result;
        };
        Parameters parameters{};bool solved=false;const double scale=input.entrySpeed/53;
        for(auto seed:std::array<Parameters,3>{{{1.2,2,3,pi,1},{.5,3.2,3,pi,1},{2,1,3.5,pi,1}}}){
            for(size_t i:{size_t(0),size_t(1),size_t(2),size_t(4)})seed[i]*=scale;
            if(!shootParameters<5>(seed,{{{.001,8},{.2,12},{.5,6},{1,5.2},{.6,4}}},[&](const auto& p){
                const auto s=shoot(p);const Vec3 right=unit(cross(s.rollExit.t,Vec3{0,0,1}));
                return std::array<double,5>{(s.apex.p.z-input.height)/100,s.apex.t.z,
                    std::atan2(dot(s.rollExit.u,right),s.rollExit.u.z),(s.exit.p.z-input.exitHeight)/50,s.exit.t.z};
            }))continue;
            const auto s=shoot(seed);
            if(s.apex.u.z>=-.5||s.apex.t.x>=-.5||s.rollExit.t.z>=-.02||s.minimumZ<input.exitHeight-1e-5||s.maximumDescentPitch>1e-5)continue;
            parameters=seed;solved=true;break;
        }
        require(solved,"FVD_IMMELMANN_SHOOT","Coupled Immelmann force/roll intent has no bounded single-crest valley solution");
        controls(parameters);out.section=designFvdSection(r,cancel);
        if(!out.section.report.valid()||out.section.samples.empty())return out;
        const double apexTime=r.controls[3].time,rollTime=r.controls[4].time;
        double previousHeight=0;
        for(const auto& q:out.section.samples){
            poll(cancel);
            if(std::abs(q.time-apexTime)<1e-8)out.apex=q;
            if(std::abs(q.time-rollTime)<1e-8)out.rollExit=q;
            require(q.position.z>=std::min(0.,input.exitHeight)-1e-5&&
                (q.time>apexTime+1e-8?q.position.z<=previousHeight+1e-5:q.position.z>=previousHeight-1e-5),
                "FVD_IMMELMANN_SHAPE","Integrated Immelmann left its single ascending-then-descending height domain");
            previousHeight=q.position.z;
        }
        out.exit=out.section.samples.back();
        require(out.apex.up.z<-.5&&out.apex.forward.x<-.5&&std::abs(out.apex.forward.z)<1e-6&&
            std::abs(out.apex.position.z-input.height)<1e-5&&std::abs(out.exit.position.z-input.exitHeight)<1e-5&&
            std::abs(out.exit.forward.z)<1e-6&&norm(out.exit.up-Vec3{0,0,1})<1e-6,
            "FVD_IMMELMANN_PORT","Integrated Immelmann missed its apex or upright level exit");
    }catch(const Failure& e){out.section.cancelled=e.code=="CANCELLED";out.section.report.fail(e.code,e.what());}
    catch(const std::exception& e){out.section.report.fail("FVD_IMMELMANN",e.what());}
    return out;
}

}
