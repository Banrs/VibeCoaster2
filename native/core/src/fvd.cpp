#include "coaster/fvd.hpp"
#include <sstream>
#include <stdexcept>

namespace coaster {
namespace {
FvdKnot interpolateControl(const FvdKnot& a,const FvdKnot& b,double time){
    const double h=b.time-a.time,u=std::clamp((time-a.time)/h,0.,1.);
    const double c0=a.value,c1=a.first*h,c2=a.second*h*h*.5;
    const double p=b.value-c0-c1-c2,v=b.first*h-c1-2*c2,d=b.second*h*h-2*c2;
    const std::array<double,6> c{c0,c1,c2,10*p-4*v+d*.5,-15*p+7*v-d,6*p-3*v+d*.5};
    double value=c[5],first=0,second=0;
    for(int i=4;i>=0;--i){second=second*u+2*first;first=first*u+value;value=value*u+c[i];}
    return {time,value,first/h,second/(h*h)};
}
}
FvdControl sampleFvdControl(const std::vector<FvdControl>& controls,double time){
    auto hi=std::upper_bound(controls.begin(),controls.end(),time,[](double t,const FvdControl& c){return t<c.time;});
    if(hi==controls.begin())return controls.front();
    if(hi==controls.end())return controls.back();
    const auto& a=*(hi-1);const auto& b=*hi;
    const std::array<double,4> av{a.normalG,a.lateralG,a.rollRate,a.drive},bv{b.normalG,b.lateralG,b.rollRate,b.drive};
    FvdControl out;out.time=time;std::array<double*,4> values{&out.normalG,&out.lateralG,&out.rollRate,&out.drive};
    for(int i=0;i<4;++i){const auto q=interpolateControl({a.time,av[i],a.first[i],a.second[i]},{b.time,bv[i],b.first[i],b.second[i]},time);*values[i]=q.value;out.first[i]=q.first;out.second[i]=q.second;}
    return out;
}
std::vector<FvdControl> combineFvdChannels(const FvdChannels& channels){
    std::vector<double> times;
    double end=-1;
    for(const auto& channel:channels){
        if(channel.size()<2||channel.front().time!=0||(end>=0&&channel.back().time!=end))throw std::invalid_argument("FVD channels must share a finite time domain starting at zero");
        double previous=-1;for(const auto& k:channel){if(!std::isfinite(k.time)||k.time<=previous||!std::isfinite(k.value)||!std::isfinite(k.first)||!std::isfinite(k.second))throw std::invalid_argument("Invalid FVD channel knot");times.push_back(k.time);previous=k.time;}end=channel.back().time;
    }
    std::sort(times.begin(),times.end());times.erase(std::unique(times.begin(),times.end()),times.end());
    std::vector<FvdControl> out;
    for(double time:times){FvdControl control;control.time=time;std::array<double*,4> values{&control.normalG,&control.lateralG,&control.rollRate,&control.drive};
        for(int i=0;i<4;++i){const auto& c=channels[i];auto hi=std::upper_bound(c.begin(),c.end(),time,[](double t,const FvdKnot& k){return t<k.time;});
            const auto q=hi==c.end()?c.back():interpolateControl(*(hi-1),*hi,time);*values[i]=q.value;control.first[i]=q.first;control.second[i]=q.second;}
        out.push_back(control);
    }return out;
}
namespace {
constexpr Vec3 gVector{0,0,-gravity};
struct Failure:std::runtime_error {
    std::string code;
    Failure(const char* c,const std::string& message):std::runtime_error(message),code(c){}
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
    FvdControl out=sampleFvdControl(request.controls,time);
    if(!request.twists.empty()){
        if(!request.additiveTwists){out.rollRate=0;out.first[2]=out.second[2]=0;}
        auto phase=std::upper_bound(request.twists.begin(),request.twists.end(),time,[](double t,const FvdTwistPhase& p){return t<p.begin;});
        if(phase!=request.twists.begin()){
            --phase;if(time<=phase->end){
                const double duration=phase->end-phase->begin,u=(time-phase->begin)/duration,w=1-u,d=1-2*u;
                // Septic angle / C2 twist rate carries a C3 physical frame
                // through onset and release without a hidden jerk step.
                out.rollRate+=phase->angle*140*u*u*u*w*w*w/duration;
                out.first[2]+=phase->angle*420*u*u*w*w*d/(duration*duration);
                out.second[2]+=phase->angle*840*u*w*(d*d-u*w)/(duration*duration*duration);
            }
        }
    }
    if(request.additiveTwists)require(bounded(out.rollRate,-4*pi,4*pi),"FVD_INPUT","Combined physical twist exceeds the supported roll-rate domain");
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
            bounded(c.normalG,-20,20)&&bounded(c.lateralG,-20,20)&&bounded(c.rollRate,-4*pi,4*pi)&&bounded(c.drive,-100,100),
        "FVD_INPUT","Invalid FVD control time, force or roll rate");previous=c.time;
        for(double x:c.first)require(bounded(x,-1000,1000),"FVD_INPUT","Invalid FVD first control derivative");
        for(double x:c.second)require(bounded(x,-10000,10000),"FVD_INPUT","Invalid FVD second control derivative");
        require(r.additiveTwists||r.twists.empty()||c.rollRate==0,"FVD_INPUT","Twist phases and roll-rate controls require explicit additive ownership");
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
struct State {Vec3 p,t,u;double v{},s{},work{},drivenWork{};};
struct Derivative {Vec3 omega,k;double dv{};};
double loss(const FvdRequest& r,double speed){return r.rollingAcceleration+r.dragAccelerationCoefficient*speed*speed;}
Derivative derivative(const State& state,const FvdControl& input,const FvdRequest& r){
    require(bounded(state.v,.5,250),"FVD_SPEED","FVD speed left [.5,250] m/s; no low-speed clamp is applied");
    const Vec3 right=cross(state.t,state.u);
    const Vec3 a=gVector+state.u*(gravity*input.normalG)+right*(gravity*input.lateralG);
    const double dv=input.drive+dot(gVector,state.t)-loss(r,state.v);
    const Vec3 dt=(a-state.t*dot(a,state.t))/state.v;
    const Vec3 k=dt/state.v;
    require(finite(k)&&norm(k)<=.15,"FVD_CURVATURE","FVD curvature exceeds the supported canonical section domain");
    double twist=input.rollRate;
    if(r.gravityReferencedRoll){
        const double h=state.t.x*state.t.x+state.t.y*state.t.y;
        require(h>1e-5,"FVD_BANK_DOMAIN","Gravity-referenced bank requires a nonvertical tangent");
        twist+=state.t.z*(state.t.x*dt.y-state.t.y*dt.x)/h;
    }
    return {cross(state.t,dt)+state.t*twist,k,dv};
}
// Differentiate the FVD equations themselves. A source port must not inherit
// high-order derivatives estimated from a small interpolating polynomial.
Knot authoredKnot(const FvdSample& q,const FvdRequest& request) {
    const auto c=control(request,q.time);const Vec3 t=q.forward,u=q.up,right=cross(t,u);
    const double v=q.speed,v2=v*v,along=dot(gVector,t);
    const double vd=c.drive+along-loss(request,v);
    const Vec3 a=gVector+u*(gravity*c.normalG)+right*(gravity*c.lateralG);
    const Vec3 transverse=a-t*along,td=transverse/v;
    const double h=t.x*t.x+t.y*t.y,n=t.x*td.y-t.y*td.x;
    double twist=c.rollRate,twistD=c.first[2],twistDD=c.second[2];
    if(request.gravityReferencedRoll){
        require(h>1e-5,"FVD_BANK_DOMAIN","Gravity-referenced bank requires a nonvertical tangent");
        twist+=t.z*n/h;
    }
    const Vec3 omega=cross(t,td)+t*twist;
    const Vec3 ud=cross(omega,u),rd=cross(omega,right);
    const double alongD=dot(gVector,td),vdd=c.first[3]+alongD-2*request.dragAccelerationCoefficient*v*vd;
    const Vec3 ad=(u*c.first[0]+ud*c.normalG+right*c.first[1]+rd*c.lateralG)*gravity;
    const Vec3 transverseD=ad-td*along-t*alongD,tdd=(transverseD-td*vd)/v;
    const double nd=t.x*tdd.y-t.y*tdd.x,hd=2*(t.x*td.x+t.y*td.y);
    const double yawD=request.gravityReferencedRoll?n/h:0;
    const double yawDD=request.gravityReferencedRoll?nd/h-n*hd/(h*h):0;
    if(request.gravityReferencedRoll)twistD+=td.z*yawD+t.z*yawDD;
    const Vec3 omegaD=cross(t,tdd)+td*twist+t*twistD;
    const Vec3 udd=cross(omegaD,u)+cross(omega,ud),rdd=cross(omegaD,right)+cross(omega,rd);
    const Vec3 add=(u*c.second[0]+ud*(2*c.first[0])+udd*c.normalG+right*c.second[1]+rd*(2*c.first[1])+rdd*c.lateralG)*gravity;
    const Vec3 transverseDD=add-tdd*along-td*(2*alongD)-t*dot(gVector,tdd);
    const Vec3 tddd=(transverseDD-tdd*(2*vd)-td*vdd)/v;
    if(request.gravityReferencedRoll){
        const double ndd=td.x*tdd.y-td.y*tdd.x+t.x*tddd.y-t.y*tddd.x;
        const double hdd=2*(td.x*td.x+td.y*td.y+t.x*tdd.x+t.y*tdd.y);
        const double yawDDD=ndd/h-2*nd*hd/(h*h)-n*hdd/(h*h)+2*n*hd*hd/(h*h*h);
        twistDD+=tdd.z*yawD+2*td.z*yawDD+t.z*yawDDD;
    }
    const Vec3 omegaDD=cross(td,tdd)+cross(t,tddd)+tdd*twist+td*(2*twistD)+t*twistDD;
    const Vec3 uddd=cross(omegaDD,u)+cross(omegaD,ud)*2+cross(omega,udd);
    auto arcSecond=[&](Vec3 first,Vec3 second){return second/v2-first*(vd/(v2*v));};
    auto arcThird=[&](Vec3 first,Vec3 second,Vec3 third){return third/(v2*v)-second*(3*vd/(v2*v2))+first*(3*vd*vd/(v2*v2*v)-vdd/(v2*v2));};
    return {q.position,t,td/v,u,0,Element::Return,arcSecond(td,tdd),arcThird(td,tdd,tddd),ud/v,arcSecond(ud,udd),arcThird(ud,udd,uddd)};
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
struct Stage {Quaternion dq;Vec3 dp;double dv{},ds{},workRate{},driveRate{};};
// Classical RK4 on position, speed and a world-space quaternion increment.
// The smooth off-manifold extension uses normalized q for the rotation but
// raw q in qdot=1/2*(0,omega)*q. Final normalization restores SO(3), removing
// the quaternion norm truncation error without changing fourth-order accuracy.
State rk4Advance(const State& start,const FvdRequest& r,double time,double dt){
    const auto stage=[&](Quaternion q,double velocity,double at){
        State state=start;state.t=turned(start.t,q);state.u=turned(start.u,q);state.v=velocity;
        const auto input=control(r,at);const auto d=derivative(state,input,r);
        require(norm(d.omega)*dt<=.1,"FVD_RESOLUTION","FVD angular step exceeds .1 rad; use a smaller step");
        Quaternion dq{-dot(d.omega,q.v)*.5,(d.omega*q.w+cross(d.omega,q.v))*.5};
        return Stage{dq,state.t*velocity,d.dv,velocity,loss(r,velocity)*velocity,input.drive*velocity};
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
    out.drivenWork+=(a.driveRate+2*b.driveRate+2*c.driveRate+d.driveRate)*(dt/6);
    out.t=unit(turned(start.t,rotation));out.u=turned(start.u,rotation);
    out.u=unit(out.u-out.t*dot(out.t,out.u));
    require(finite(out.p)&&norm(out.p)<=100000,"FVD_POSITION","FVD section left the bounded position domain");
    derivative(out,control(r,time+dt),r);return out;
}
// Position errors from a fourth-order step are amplified by the fourth
// derivative of the canonical Hermite span. Step doubling cancels that leading
// local error before the exact ODE endpoint jets are used for reconstruction.
State advance(const State& start,const FvdRequest& r,double time,double dt){
    const auto full=rk4Advance(start,r,time,dt);
    const auto first=rk4Advance(start,r,time,dt*.5);
    const auto half=rk4Advance(first,r,time+dt*.5,dt*.5);
    State out=half;
    out.p=half.p+(half.p-full.p)/15;
    out.t=unit(half.t+(half.t-full.t)/15);
    out.u=half.u+(half.u-full.u)/15;out.u=unit(out.u-out.t*dot(out.t,out.u));
    out.v+=(half.v-full.v)/15;out.s+=(half.s-full.s)/15;
    out.work+=(half.work-full.work)/15;out.drivenWork+=(half.drivenWork-full.drivenWork)/15;
    derivative(out,control(r,time+dt),r);return out;
}

FvdSample sample(const State& state,const FvdRequest& r,double time){
    return {time,state.s,state.v,state.p,state.t,state.u,derivative(state,control(r,time),r).k,state.work,state.drivenWork};
}
void assess(FvdResult& result,const FvdRequest& r,const Cancel& cancel){
    auto& a=result.assessment;
    const double initialEnergy=.5*r.speed*r.speed+gravity*r.position.z;
    for(const auto& q:result.samples)a.maxEnergyDrift=std::max(a.maxEnergyDrift,
        std::abs(.5*q.speed*q.speed+gravity*q.position.z+q.dissipatedWorkPerMass-q.drivenWorkPerMass-initialEnergy));
    // Canonical point replay uses half the authoring timestep, with speed
    // derived from gravity and losses independently of the force controls.
    double distance=0,speed=r.speed;
    const auto measure=[&](double time,double at,double velocity){
        require(at>=0&&at<=result.track.length,"FVD_REPLAY_DOMAIN","Canonical replay left the open section before its requested end");
        const auto q=sampleKinematics(result.track,at);const auto target=control(r,time);
        const double dv=target.drive+dot(gVector,q.sample.tangent)-loss(r,velocity);
        const Vec3 specific=q.sample.curvature*(velocity*velocity)+q.sample.tangent*dv-gVector;
        const double normal=dot(specific,q.sample.up)/gravity,lateral=dot(specific,q.sample.right)/gravity;
        const double roll=dot(q.upS,q.sample.right)*velocity;
        require(std::isfinite(normal)&&std::isfinite(lateral)&&std::isfinite(roll),"FVD_REPLAY_DOMAIN","Nonfinite canonical replay measurement");
        a.maxNormalResidualG=std::max(a.maxNormalResidualG,std::abs(normal-target.normalG));
        a.maxLateralResidualG=std::max(a.maxLateralResidualG,std::abs(lateral-target.lateralG));
        double requestedTwist=target.rollRate;
        if(r.gravityReferencedRoll){
            const auto t=q.sample.tangent,td=q.sample.curvature*velocity;const double h=t.x*t.x+t.y*t.y;
            require(h>1e-5,"FVD_BANK_DOMAIN","Canonical bank replay reached a vertical tangent");
            requestedTwist+=t.z*(t.x*td.y-t.y*td.x)/h;
        }
        a.maxRollResidualRadPerSecond=std::max(a.maxRollResidualRadPerSecond,std::abs(roll-requestedTwist));
        ++a.evaluations;
    };
    measure(0,0,speed);
    for(size_t i=1;i<result.samples.size();++i){
        const double begin=result.samples[i-1].time,dt=(result.samples[i].time-begin)*.5;
        for(int j=0;j<2;++j){
            poll(cancel);const double time=begin+j*dt;
            require(distance>=0&&distance<=result.track.length,"FVD_REPLAY_DOMAIN","Canonical replay ended early");
            const auto q=result.track.sample(distance);const double acceleration=control(r,time).drive+dot(gVector,q.tangent)-loss(r,speed);
            const double midDistance=distance+speed*dt*.5,midSpeed=speed+acceleration*dt*.5;
            require(bounded(midSpeed,.5,250)&&midDistance<=result.track.length,"FVD_REPLAY_DOMAIN","Canonical midpoint replay left its supported domain");
            const auto mid=result.track.sample(midDistance);
            measure(time+dt*.5,midDistance,midSpeed);
            distance+=midSpeed*dt;speed+=(control(r,time+dt*.5).drive+dot(gVector,mid.tangent)-loss(r,midSpeed))*dt;
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
FvdEntry makeFvdEntry(const Knot& k,double speed,double rolling,double drag,FvdDriveJet drive){
    for(const auto vector:{k.position,k.tangent,k.curvature,k.third,k.fourth,k.up,k.upFirst,k.upSecond,k.upThird})
        require(finite(vector),"FVD_ENTRY","Inherited port contains nonfinite spatial jets");
    require(bounded(speed,.5,250)&&bounded(rolling,0,1)&&bounded(drag,0,.01)&&
        std::isfinite(drive.value)&&std::isfinite(drive.first)&&std::isfinite(drive.second),"FVD_ENTRY","Inherited port has invalid speed, losses or drive jets");
    require(std::abs(norm(k.tangent)-1)<1e-7&&std::abs(norm(k.up)-1)<1e-7&&std::abs(dot(k.tangent,k.up))<1e-7,
        "FVD_ENTRY","Inherited port must retain an orthonormal physical frame");
    const double v=speed,a=drive.value+dot(gVector,k.tangent)-rolling-drag*v*v;
    const double jerk=drive.first+dot(gVector,k.curvature)*v-2*drag*v*a;
    const Vec3 right=cross(k.tangent,k.up);
    const Vec3 rightS=cross(k.curvature,k.up)+cross(k.tangent,k.upFirst);
    const Vec3 rightSS=cross(k.third,k.up)+cross(k.curvature,k.upFirst)*2+cross(k.tangent,k.upSecond);
    const Vec3 specific=k.curvature*(v*v)-gVector;
    const Vec3 specificD=k.curvature*(2*v*a)+k.third*(v*v*v);
    const Vec3 specificDD=k.curvature*(2*(a*a+v*jerk))+k.third*(5*v*v*a)+k.fourth*(v*v*v*v);
    FvdEntry out{k,speed,{}};auto& c=out.initial;
    auto force=[&](Vec3 axis,Vec3 first,Vec3 second,int channel,double& value){
        const Vec3 axisD=first*v,axisDD=first*a+second*(v*v);
        value=dot(specific,axis)/gravity;
        c.first[channel]=(dot(specificD,axis)+dot(specific,axisD))/gravity;
        c.second[channel]=(dot(specificDD,axis)+2*dot(specificD,axisD)+dot(specific,axisDD))/gravity;
    };
    force(k.up,k.upFirst,k.upSecond,0,c.normalG);force(right,rightS,rightSS,1,c.lateralG);
    const double h=dot(k.upFirst,right),hs=dot(k.upSecond,right)+dot(k.upFirst,rightS);
    const double hss=dot(k.upThird,right)+2*dot(k.upSecond,rightS)+dot(k.upFirst,rightSS);
    c.rollRate=v*h;c.first[2]=a*h+v*v*hs;c.second[2]=jerk*h+3*v*a*hs+v*v*v*hss;
    c.drive=drive.value;c.first[3]=drive.first;c.second[3]=drive.second;
    // Round-trip the differential equations themselves. This catches a spline
    // placeholder frame (or inconsistent unit-vector derivatives) without fitting
    // a small polynomial to neighboring points or rewriting the supplied port.
    FvdRequest r;r.position=k.position;r.forward=k.tangent;r.up=k.up;r.speed=v;
    r.rollingAcceleration=rolling;r.dragAccelerationCoefficient=drag;r.controls={c,c};r.controls.back().time=1;
    validate(r);
    const FvdSample sample{0,0,v,k.position,k.tangent,k.up,k.curvature,0,0};
    const auto rebuilt=authoredKnot(sample,r);
    require(norm(rebuilt.curvature-k.curvature)<1e-7&&norm(rebuilt.third-k.third)<1e-7&&norm(rebuilt.fourth-k.fourth)<1e-7&&
        norm(rebuilt.upFirst-k.upFirst)<1e-7&&norm(rebuilt.upSecond-k.upSecond)<1e-7&&norm(rebuilt.upThird-k.upThird)<1e-7,
        "FVD_ENTRY","Inherited geometry and physical frame jets are inconsistent");
    return out;
}
namespace {
FvdControl initializeEntry(FvdRequest& r,const std::optional<FvdEntry>& entry,FvdControl legacy){
    if(!entry)return legacy;
    const auto& e=*entry;const auto& k=e.jet;
    const auto checked=makeFvdEntry(k,e.speed,r.rollingAcceleration,r.dragAccelerationCoefficient,
        {e.initial.drive,e.initial.first[3],e.initial.second[3]});
    const auto& expected=checked.initial;const auto& supplied=e.initial;
    require(supplied.time==0&&std::abs(expected.normalG-supplied.normalG)<1e-7&&std::abs(expected.lateralG-supplied.lateralG)<1e-7&&
        std::abs(expected.rollRate-supplied.rollRate)<1e-7,"FVD_ENTRY","Inherited force controls disagree with the physical port");
    for(size_t i=0;i<4;++i)require(std::abs(expected.first[i]-supplied.first[i])<1e-7&&std::abs(expected.second[i]-supplied.second[i])<1e-7,
        "FVD_ENTRY","Inherited control derivatives disagree with the physical port");
    r.position=k.position;r.forward=k.tangent;r.up=k.up;r.speed=e.speed;r.additiveTwists=true;
    auto initial=expected;
    if(r.gravityReferencedRoll){
        const double v=r.speed,a=initial.drive+dot(gVector,k.tangent)-loss(r,v);
        const double jerk=initial.first[3]+dot(gVector,k.curvature)*v-2*r.dragAccelerationCoefficient*v*a;
        const Vec3 t=k.tangent,td=k.curvature*v,tdd=k.curvature*a+k.third*(v*v),tddd=k.curvature*jerk+k.third*(3*v*a)+k.fourth*(v*v*v);
        const double h=t.x*t.x+t.y*t.y,n=t.x*td.y-t.y*td.x;
        require(h>1e-5,"FVD_BANK_DOMAIN","Inherited gravity-bank frame is vertical");
        const double hd=2*(t.x*td.x+t.y*td.y),nd=t.x*tdd.y-t.y*tdd.x;
        const double hdd=2*(td.x*td.x+td.y*td.y+t.x*tdd.x+t.y*tdd.y);
        const double ndd=td.x*tdd.y-td.y*tdd.x+t.x*tddd.y-t.y*tddd.x;
        const double yawD=n/h,yawDD=nd/h-n*hd/(h*h);
        const double yawDDD=ndd/h-2*nd*hd/(h*h)-n*hdd/(h*h)+2*n*hd*hd/(h*h*h);
        initial.rollRate-=t.z*yawD;
        initial.first[2]-=td.z*yawD+t.z*yawDD;
        initial.second[2]-=tdd.z*yawD+2*td.z*yawDD+t.z*yawDDD;
    }
    return initial;
}
void requireInheritedPort(const FvdResult& result,const std::optional<FvdEntry>& entry){
    if(!entry||!result.assessment.passed)return;
    const auto& expected=entry->jet;const auto& actual=result.track.knots.front();
    require(norm(actual.position-expected.position)<1e-7&&norm(actual.tangent-expected.tangent)<1e-7&&norm(actual.curvature-expected.curvature)<1e-7&&
        norm(actual.third-expected.third)<1e-7&&norm(actual.fourth-expected.fourth)<1e-7&&norm(actual.up-expected.up)<1e-7&&
        norm(actual.upFirst-expected.upFirst)<1e-7&&norm(actual.upSecond-expected.upSecond)<1e-7&&norm(actual.upThird-expected.upThird)<1e-7,
        "FVD_ENTRY","Compiled signature did not retain its inherited geometry and frame jets");
}
}
FvdResult designFvdSection(const FvdRequest& request,Cancel cancel){
    FvdResult result;result.track.closed=false;
    try{
        poll(cancel);validate(request);
        State state{request.position,unit(request.forward),unit(request.up),request.speed,0};
        state.u=unit(state.u-state.t*dot(state.t,state.u));
        result.samples.push_back(sample(state,request,0));
        // Uniform dt avoids pathological tiny terminal spans. Preserve both
        // force and twist boundaries, where higher derivatives can change.
        std::vector<double> boundaries;for(const auto& c:request.controls)boundaries.push_back(c.time);
        for(const auto& phase:request.twists){boundaries.push_back(phase.begin);boundaries.push_back(phase.end);}
        std::sort(boundaries.begin(),boundaries.end());boundaries.erase(std::unique(boundaries.begin(),boundaries.end()),boundaries.end());
        for(size_t section=1;section<boundaries.size();++section){
            const double begin=boundaries[section-1],duration=boundaries[section]-begin;
            const size_t count=std::max<size_t>(3,size_t(std::ceil(duration/request.step)));
            require(count<=request.maxSamples-result.samples.size(),"FVD_BUDGET","FVD control boundaries exceed sample budget");
            const double dt=duration/double(count);
            for(size_t j=0;j<count;++j){
                poll(cancel);state=advance(state,request,begin+double(j)*dt,dt);
                result.samples.push_back(sample(state,request,j+1==count?boundaries[section]:begin+double(j+1)*dt));
            }
        }
        result.integrated=true;
        result.track.knots.reserve(result.samples.size());
        for(const auto& q:result.samples){poll(cancel);result.track.knots.push_back(authoredKnot(q,request));}
        result.track.authoredGeometry=result.track.authoredFrame=true;
        poll(cancel);result.track.rebuild();poll(cancel);result.canonicalBuilt=true;
        assess(result,request,cancel);
        result.report.warnings.push_back("Experimental point-mass open section with configured rolling/quadratic drag and centerline force reference. Sampled residuals do not establish finite-train, offset-rider, propulsion, clearance or full ride acceptance.");
    }catch(const Failure& e){result.cancelled=e.code=="CANCELLED";result.report.fail(e.code,e.what());}
    catch(const std::exception& e){result.report.fail("FVD_CANONICAL",e.what());}
    return result;
}
FvdHillResult designFvdHill(const FvdHillRequest& request,Cancel cancel){
    auto input=request;if(input.entry)input.entrySpeed=input.entry->speed;
    FvdHillResult out;
    try {
        poll(cancel);
        require(bounded(input.entrySpeed,35,90)&&bounded(input.height,20,350)&&bounded(input.positiveG,2,5)&&bounded(input.airtimeG,-1.35,.2),"FVD_HILL_INPUT","Hill intent is outside its authoring domain");
        const double descentAirG=input.airtimeG+input.crestLoadChangeG;
        require(bounded(descentAirG,-1.35,.2),"FVD_HILL_INPUT","Crest exit load is outside its authoring domain");
        const double exitG=input.exitPositiveG==0?input.positiveG:input.exitPositiveG;
        const double exitRamp=input.exitRampSeconds==0?input.rampSeconds:input.exitRampSeconds;
        const double releaseRamp=input.ascentReleaseSeconds==0?input.rampSeconds:input.ascentReleaseSeconds;
        // A duration domain, not a force limit: reference-scale flanks and speed
        // scaling need longer ramps. Even at these maxima, the whole programme
        // remains inside the source time/sample budget; replay limits are unchanged.
        require(bounded(exitG,2,5)&&bounded(input.rampSeconds,.2,6)&&bounded(exitRamp,.2,6)&&bounded(releaseRamp,.2,6),"FVD_HILL_INPUT","Hill recovery is outside its authoring domain");
        require(input.exitNormalG==0||(bounded(input.exitNormalG,1,2)&&bounded(input.exitReleaseSeconds,.2,3)),
            "FVD_HILL_INPUT","Final hill force release left its bounded domain");
        auto& r=out.authoring;r.position={0,0,0};r.speed=input.entrySpeed;r.step=.0025;
        r.gravityReferencedRoll=input.exitNormalG!=0;
        r.rollingAcceleration=input.rollingAcceleration;r.dragAccelerationCoefficient=input.dragAccelerationCoefficient;
        const auto initial=initializeEntry(r,input.entry,{0,1,0,0});
        const Vec3 entryForward=unit(Vec3{r.forward.x,r.forward.y,0});
        using Parameters=std::array<double,3>;
        auto controls=[&](const Parameters& p){
            const double peak=input.rampSeconds+p[0],release=peak+releaseRamp,airEnd=release+p[1],pull=airEnd+exitRamp;
            r.controls={initial,{input.rampSeconds,input.positiveG,0,0},{peak,input.positiveG,0,0},
                {release,input.airtimeG,0,0},{airEnd,descentAirG,0,0},{pull,exitG,0,0},{pull+p[2],exitG,0,0}};
            r.twists=input.twistAngle==0?std::vector<FvdTwistPhase>{}:std::vector<FvdTwistPhase>{{release+p[1]*.5,pull+p[2]*.7,input.twistAngle}};
            if(input.exitNormalG!=0){
                const double end=pull+p[2]+input.exitReleaseSeconds;
                r.controls.push_back({end,input.exitNormalG,0,0});
                if(input.twistAngle!=0)r.twists.push_back({pull+p[2]*.7,end,-input.twistAngle});
            }
        };
        auto shoot=[&](const Parameters& p){
            controls(p);State state{r.position,r.forward,r.up,r.speed,0};double height=r.position.z;
            for(size_t i=1;i<r.controls.size();++i){const double begin=r.controls[i-1].time,duration=r.controls[i].time-begin;
                const int count=int(std::ceil(duration/.005));const double dt=duration/count;
                for(int j=0;j<count;++j){poll(cancel);state=advance(state,r,begin+j*dt,dt);height=std::max(height,state.p.z);}}
            // A terminal upright release feeds a long straight station line.
            // Resolve its small pitch/height residual before that line can
            // amplify it; do not snap the inherited frame to horizontal.
            const double accuracy=input.exitNormalG!=0?100:1;
            return std::array<double,3>{accuracy*(height-r.position.z-input.height)/100,accuracy*(state.p.z-r.position.z-input.exitHeight)/100,
                accuracy*(std::atan2(state.t.z,std::hypot(state.t.x,state.t.y))-input.exitPitch)};
        };
        Parameters solved{};bool found=false;
        for(Parameters seed:std::array<Parameters,3>{{{.8,4.5,.8},{1.5,5.5,.5},{.1,3.5,1.2}}}) {
            if(shootParameters<3>(seed,{{{.001,4},{.5,10},{.001,4}}},shoot)){solved=seed;found=true;break;}
        }
        require(found,"FVD_HILL_SHOOT","Force hill cannot meet its crest and live exit with the selected energy");
        controls(solved);out.section=designFvdSection(r,cancel);requireInheritedPort(out.section,input.entry);
        if(out.section.assessment.passed&&input.twistAngle==0) {
            int changes=0,previous=1;
            for(const auto& q:out.section.samples) {
                require(dot(q.forward,entryForward)>0,"FVD_HILL_SHAPE","Planar hill reversed its forward direction");
                const int sign=q.forward.z>1e-5?1:q.forward.z< -1e-5?-1:0;
                if(sign&&sign!=previous){++changes;previous=sign;}
            }
            require(changes==(input.exitPitch>0?2:1),"FVD_HILL_SHAPE","Planar hill gained an extra crest or valley");
        }
    }catch(const Failure& e){out.section.cancelled=e.code=="CANCELLED";out.section.report.fail(e.code,e.what());}
    catch(const std::exception& e){out.section.report.fail("FVD_HILL",e.what());}
    return out;
}

FvdCamelbackResult designFvdCamelback(const FvdCamelbackRequest& input,Cancel cancel){
    FvdCamelbackResult out;
    try{
        poll(cancel);
        require(input.hill.twistAngle==0&&bounded(input.tailCutSeconds,0,3)&&bounded(input.releaseSeconds,.2,2)&&
            bounded(input.exitNormalG,1,2)&&bounded(input.minimumExitPitch,0,.45),"FVD_CAMELBACK_INPUT","Protected camelback recovery left its planar bounded domain");
        auto original=designFvdHill(input.hill,cancel);
        if(!original.section.report.valid()||!original.section.assessment.passed){out.authoring=std::move(original.authoring);out.section=std::move(original.section);return out;}
        out.authoring=original.authoring;auto& r=out.authoring;
        out.originalEndTime=r.controls.back().time;
        out.protectedEndTime=out.originalEndTime-input.tailCutSeconds;
        require(out.protectedEndTime>=r.controls[r.controls.size()-2].time+.001,"FVD_CAMELBACK_CUT","Tail cut must remain inside the protected hill's final constant-load hold");
        r.controls.back().time=out.protectedEndTime;
        const double releaseEnd=out.protectedEndTime+input.releaseSeconds;
        r.controls.push_back({releaseEnd,input.exitNormalG,0,0});
        r.controls.push_back({releaseEnd+8,input.exitNormalG,0,0});
        State state{r.position,r.forward,r.up,r.speed,0};double ready=-1;
        // Find the first low-load upward port, with no target XYZ displacement.
        // The bounded probe tail is only a search domain, not returned geometry.
        for(size_t i=1;i<r.controls.size()&&ready<0;++i){
            const double begin=r.controls[i-1].time,duration=r.controls[i].time-begin;
            const size_t count=std::max<size_t>(3,size_t(std::ceil(duration/r.step)));const double dt=duration/count;
            for(size_t j=0;j<count;++j){
                poll(cancel);const double time=begin+j*dt;const State before=state;state=advance(state,r,time,dt);
                if(time+dt<releaseEnd-1e-10)continue;
                require(state.t.x>0,"FVD_CAMELBACK_RECOVERY","Low-load recovery reversed its forward direction");
                if(std::atan2(state.t.z,state.t.x)<input.minimumExitPitch)continue;
                if(time<releaseEnd){ready=releaseEnd;break;}
                double lo=0,hi=dt;
                for(int iteration=0;iteration<28;++iteration){const double mid=(lo+hi)*.5;const auto probe=advance(before,r,time,mid);
                    if(std::atan2(probe.t.z,probe.t.x)<input.minimumExitPitch)lo=mid;else hi=mid;}
                ready=time+(lo+hi)*.5;break;
            }
        }
        require(ready>=releaseEnd,"FVD_CAMELBACK_RECOVERY","Low-load recovery cannot reach the requested rising port within eight seconds");
        if(ready-releaseEnd<.001)r.controls.pop_back();else r.controls.back().time=ready;
        out.section=designFvdSection(r,cancel);
        if(out.section.assessment.passed){
            const auto boundary=std::lower_bound(out.section.samples.begin(),out.section.samples.end(),out.protectedEndTime,
                [](const FvdSample& q,double time){return q.time<time;});
            const size_t index=size_t(boundary-out.section.samples.begin());
            out.protectedEndDistance=index<out.section.track.spans.size()?out.section.track.spans[index].start:out.section.track.length;
            for(const auto& q:out.section.samples)require(std::abs(q.position.y)<1e-10,"FVD_CAMELBACK_PLANE","Protected camelback left its approved plane");
        }
    }catch(const Failure& e){out.section.cancelled=e.code=="CANCELLED";out.section.report.fail(e.code,e.what());}
    catch(const std::exception& e){out.section.report.fail("FVD_CAMELBACK",e.what());}
    return out;
}

FvdHillResult designFvdDive(const FvdDiveRequest& input,Cancel cancel){
    FvdHillResult out;
    try {
        require(bounded(input.entrySpeed,7,25)&&bounded(input.exitPitch,-.7,-.08)&&bounded(input.maximumPitch,1.3,pi*.5),"FVD_DIVE_INPUT","Cliff intent is outside its authoring domain");
        auto& r=out.authoring;r.position={0,0,0};r.speed=input.entrySpeed;r.step=.0025;
        r.rollingAcceleration=input.rollingAcceleration;r.dragAccelerationCoefficient=input.dragAccelerationCoefficient;
        using Parameters=std::array<double,3>;
        auto controls=[&](const Parameters& p){
            const double crest=input.crestRampSeconds+p[0],vertical=crest+.7,fall=vertical+p[1],load=fall+input.pulloutRampSeconds,release=load+p[2];
            r.controls={{0,1,0,0},{input.crestRampSeconds,input.crestG,0,0},{crest,input.crestG,0,0},
                {vertical,std::cos(input.maximumPitch),0,0},{fall,std::cos(input.maximumPitch),0,0},
                {load,input.pulloutG,0,0},{release,input.pulloutG,0,0},{release+input.exitRampSeconds,std::cos(input.exitPitch),0,0}};
        };
        auto shoot=[&](const Parameters& p){
            controls(p);State state{r.position,r.forward,r.up,r.speed,0};double pitch=0,verticalPitch=0;
            for(size_t i=1;i<r.controls.size();++i){const double begin=r.controls[i-1].time,duration=r.controls[i].time-begin;
                const int count=int(std::ceil(duration/.005));const double dt=duration/count;
                for(int j=0;j<count;++j){poll(cancel);state=advance(state,r,begin+j*dt,dt);pitch+=std::remainder(std::atan2(state.t.z,state.t.x)-pitch,2*pi);}if(i==3)verticalPitch=pitch;}
            return std::array<double,3>{verticalPitch+input.maximumPitch,(state.p.z+input.drop)/100,pitch-input.exitPitch};
        };
        Parameters solved{};bool found=false;std::ostringstream diagnostic;
        for(Parameters seed:std::array<Parameters,4>{{{1.,1.,1.},{.3,2.,1.5},{1.5,.5,.5},{.7,3.,1.}}}){
            if(shootParameters<3>(seed,{{{.001,8},{.02,8},{.001,8}}},shoot)){solved=seed;found=true;break;}const auto e=shoot(seed);diagnostic<<" ["<<seed[0]<<","<<seed[1]<<","<<seed[2]<<" -> "<<e[0]<<","<<e[1]<<","<<e[2]<<"]";}
        if(!found)throw std::runtime_error("Cliff force solve failed"+diagnostic.str());
        controls(solved);out.section=designFvdSection(r,cancel);
        for(const auto& q:out.section.samples)require(q.forward.z<=1e-6&&q.forward.x>=-1e-5,"FVD_DIVE_SHAPE","Cliff programme gained a rise or reversed horizontally");
    }catch(const Failure& e){out.section.cancelled=e.code=="CANCELLED";out.section.report.fail(e.code,e.what());}
    catch(const std::exception& e){out.section.report.fail("FVD_DIVE",e.what());}
    return out;
}

FvdHillResult designFvdWave(const FvdWaveRequest& input,Cancel cancel){
    FvdHillResult out;
    try{
        poll(cancel);
        const double entrySpeed=input.entry?input.entry->speed:input.entrySpeed;
        const double entryPitch=input.entry?std::asin(input.entry->jet.tangent.z):input.entryPitch;
        const double entryG=input.entry?input.entry->initial.normalG:(input.entryNormalG==0?std::cos(input.entryPitch):input.entryNormalG);
        require(bounded(entrySpeed,60,80)&&bounded(entryPitch,-.4,.45)&&bounded(input.height,40,160)&&
            bounded(input.exitHeight,-30,10)&&bounded(input.turnAngle,2.8,3.5)&&bounded(input.bankAngle,65*pi/180,75*pi/180)&&
            bounded(input.exitNormalG,1.5,4.3)&&bounded(entryG,.1,4.3)&&bounded(input.entryHoldSeconds,0,3),"FVD_WAVE_INPUT","Wave intent left its force-authoring domain");
        auto& r=out.authoring;r.position={0,0,0};r.speed=input.entrySpeed;r.step=.0025;r.gravityReferencedRoll=true;
        r.forward={std::cos(input.entryPitch),0,std::sin(input.entryPitch)};r.up={-r.forward.z,0,r.forward.x};
        r.rollingAcceleration=input.rollingAcceleration;r.dragAccelerationCoefficient=input.dragAccelerationCoefficient;
        const auto initial=initializeEntry(r,input.entry,{0,entryG,0,0});
        const double startHeight=r.position.z,startYaw=std::atan2(r.forward.y,r.forward.x);
        using Parameters=std::array<double,5>;
        auto controls=[&](const Parameters& p){
            const double hold=input.entryHoldSeconds,high=hold+.75+p[0],low=high+1.5,fall=low+p[1],loaded=fall+1.5,end=loaded+p[3];
            r.controls={initial,{hold+.75,4.1,0,0},{high,4.1,0,0},{low,1.3,0,0},
                {fall,1.3,0,0},{loaded,input.exitNormalG,0,0},{end,input.exitNormalG,0,0}};
            if(hold>0)r.controls.insert(r.controls.begin()+1,{hold,entryG,0,0});
            // Banking can turn the inherited rising motion during its low-load
            // hold. Delaying both creates an unintended near-vertical climb.
            r.twists={{.4,3.2,-input.bankAngle},{3.2+p[2]*(end-4.4),end,input.bankAngle+p[4]}};
        };
        auto shoot=[&](const Parameters& p){
            controls(p);State state{r.position,r.forward,r.up,r.speed,0};double height=startHeight,yaw=startYaw;
            for(size_t i=1;i<r.controls.size();++i){const double begin=r.controls[i-1].time,duration=r.controls[i].time-begin;
                const int count=int(std::ceil(duration/.005));const double dt=duration/count;
                for(int j=0;j<count;++j){poll(cancel);state=advance(state,r,begin+j*dt,dt);height=std::max(height,state.p.z);
                    yaw+=std::remainder(std::atan2(state.t.y,state.t.x)-yaw,2*pi);}}
            const Vec3 upright=unit(Vec3{0,0,1}-state.t*state.t.z),right=cross(state.t,upright);
            return Parameters{(height-startHeight-input.height)/100,(state.p.z-startHeight-input.exitHeight)/100,
                std::atan2(state.t.z,std::hypot(state.t.x,state.t.y)),yaw-startYaw-input.turnAngle,
                input.entry?std::atan2(dot(state.u,right),dot(state.u,upright)):0};
        };
        Parameters solved{};bool found=false;std::ostringstream diagnostic;
        for(auto seed:std::array<Parameters,9>{{{.66,7.95,.98,1.02},{.5,9,.95,.8},{1,7,.8,.3},
            {.1,3,.8,.5},{.1,5,.8,.5},{.1,1,.5,.5},{.3,4,.5,1.5},{.1,4,.95,3},{.1,6,1,1}}}){
            bool converged=false;
            if(input.entry)converged=shootParameters<5>(seed,{{{.001,6},{1,12},{0,1},{.1,8},{-pi,pi}}},shoot);
            else {std::array<double,4> legacy{seed[0],seed[1],seed[2],seed[3]};
                converged=shootParameters<4>(legacy,{{{.001,6},{1,12},{0,1},{.1,8}}},[&](const auto& value){
                    const auto error=shoot({value[0],value[1],value[2],value[3],0});return std::array<double,4>{error[0],error[1],error[2],error[3]};});
                for(size_t i=0;i<4;++i)seed[i]=legacy[i];}
            if(converged){solved=seed;found=true;break;}
            try{const auto error=shoot(seed);diagnostic<<" [";for(double value:seed)diagnostic<<value<<',';diagnostic<<" -> ";for(double value:error)diagnostic<<value<<',';diagnostic<<']';}catch(const Failure& failure){if(failure.code=="CANCELLED")throw;diagnostic<<" ["<<failure.code<<": "<<failure.what()<<']';}
        }
        if(!found){out.section.report.fail("FVD_WAVE_SHOOT","Wave cannot meet its height, reversal and live exit:"+diagnostic.str());return out;}
        controls(solved);out.section=designFvdSection(r,cancel);requireInheritedPort(out.section,input.entry);
        if(out.section.assessment.passed){
            const bool rising=entryPitch>=0;
            int reversals=0,previous=rising?1:-1;
            for(const auto& q:out.section.samples){const int sign=q.forward.z>1e-5?1:q.forward.z< -1e-5?-1:0;
                if(sign&&sign!=previous){++reversals;previous=sign;}
                require(q.up.z>0,"FVD_WAVE_SHAPE","Wave unintentionally inverted");}
            require(reversals==(rising?1:2),"FVD_WAVE_SHAPE","Wave gained an extra vertical extremum before its level exit");
        }
    }catch(const Failure& e){out.section.cancelled=e.code=="CANCELLED";out.section.report.fail(e.code,e.what());}
    catch(const std::exception& e){out.section.report.fail("FVD_WAVE",e.what());}
    return out;
}

namespace {
FvdHillResult inheritedLoopWithPlaneYaw(const FvdLoopRequest& input,Cancel cancel){
    FvdHillResult out;auto& r=out.authoring;
    r.step=.0025;r.rollingAcceleration=input.rollingAcceleration;r.dragAccelerationCoefficient=input.dragAccelerationCoefficient;
    const auto initial=initializeEntry(r,input.entry,{0,1,0,0});
    const double startYaw=std::atan2(r.forward.y,r.forward.x),startHeight=r.position.z;
    const Vec3 horizontal{std::cos(startYaw),std::sin(startYaw),0},right{horizontal.y,-horizontal.x,0};
    const double startPitch=std::atan2(r.forward.z,dot(r.forward,horizontal));
    const Vec3 expectedUp=horizontal*(-std::sin(startPitch))+Vec3{0,0,std::cos(startPitch)};
    const auto& port=input.entry->jet;
    bool planar=bounded(startPitch,-1e-7,pi/3)&&norm(port.up-expectedUp)<1e-7;
    for(Vec3 jet:{port.curvature,port.third,port.fourth,port.upFirst,port.upSecond,port.upThird})planar&=std::abs(dot(jet,right))<1e-7;
    for(int channel:{1,2}){
        const double value=channel==1?initial.lateralG:initial.rollRate;
        planar&=std::abs(value)<1e-7&&std::abs(initial.first[channel])<1e-7&&std::abs(initial.second[channel])<1e-7;
    }
    require(planar,"FVD_LOOP_ENTRY_FAMILY","Coordinated loop-plane yaw requires an upright planar inherited port; lateral/twist jets cannot be reset");
    constexpr double ramp=1.2,crestHold=.7,release=1;
    const auto apexAt=[&](const auto& p){return ramp+p[0]+p[1]+input.ascentReleaseSeconds;};
    using Parameters=std::array<double,3>;
    auto controls=[&](const Parameters& p){
        const double peak=ramp+p[0],apex=apexAt(p),crestEnd=apex+crestHold,loaded=crestEnd+ramp,unload=loaded+p[2],end=unload+release;
        r.controls={initial,{ramp,input.normalG,0,0},{peak,input.normalG,0,0},{apex,input.crestG,0,0},
            {crestEnd,input.crestG,0,0},{loaded,input.exitPositiveG==0?input.normalG:input.exitPositiveG,0,0},{unload,input.exitPositiveG==0?input.normalG:input.exitPositiveG,0,0},{end,input.exitNormalG,0,0}};
        if(input.ascentReleaseSeconds>0)r.controls.insert(r.controls.begin()+3,{peak+input.ascentReleaseSeconds,input.crestG,0,0});
        r.twists.clear();
    };
    auto shoot=[&](const Parameters& p){
        controls(p);State state{r.position,r.forward,r.up,r.speed,0};double turn=startPitch,apexTurn=0,apexHeight=startHeight;
        for(size_t i=1;i<r.controls.size();++i){
            const double begin=r.controls[i-1].time,duration=r.controls[i].time-begin;
            const int count=int(std::ceil(duration/.005));const double dt=duration/count;
            for(int j=0;j<count;++j){poll(cancel);state=advance(state,r,begin+j*dt,dt);
                turn+=std::remainder(std::atan2(state.t.z,dot(state.t,horizontal))-turn,2*pi);}
            if(r.controls[i].time==apexAt(p)){apexTurn=turn;apexHeight=state.p.z;}
        }
        return Parameters{(apexHeight-startHeight-input.height)/100,apexTurn-pi,turn-(2*pi+input.exitPitch)};
    };
    Parameters solved{};bool found=false;std::ostringstream diagnostic;
    for(auto seed:std::array<Parameters,4>{{{2.0,1.8,2.3},{2.5,1.0,2.5},{1.5,2.5,2.0},{3.0,.7,3.0}}}){
        if(shootParameters<3>(seed,{{{.001,6},{input.ascentReleaseSeconds>0?.001:.6,4},{.001,7}}},shoot)){solved=seed;found=true;break;}
        try{const auto error=shoot(seed);diagnostic<<" [";for(double value:seed)diagnostic<<value<<',';diagnostic<<" -> ";for(double value:error)diagnostic<<value<<',';diagnostic<<']';}
        catch(const Failure& failure){if(failure.code=="CANCELLED")throw;diagnostic<<" ["<<failure.code<<"]";}
    }
    if(!found){out.section.report.fail("FVD_LOOP_SHOOT","Planar inherited loop cannot meet its apex and final pitch with the selected loads/energy:"+diagnostic.str());return out;}
    controls(solved);const FvdRequest planarSource=r;const double duration=r.controls.back().time,apexTime=apexAt(solved),yawDuration=apexTime;
    // Complete the plane yaw at the apex so the descending branch retains
    // its lateral separation from the rising branch.
    // t=cos(theta)e(psi)+sin(theta)Z, u=-sin(theta)e(psi)+cos(theta)Z.
    // Prescribing a monotone plane angle psi requires BOTH Gy=-v*psi'*cos(theta)/g
    // and transported twist=psi'*sin(theta). Constant Gy gives the opposite
    // ascending hand and an S-shaped drift through the inverted half.
    auto coordinated=[&](const State& state,double time){
        auto c=sampleFvdControl(planarSource.controls,time);const double v=state.v,sn=state.t.z,cs=dot(state.t,horizontal);
        const double thetaD=gravity*(c.normalG-cs)/v;
        const double vd=c.drive-gravity*sn-loss(planarSource,v);
        const double vdd=c.first[3]-gravity*cs*thetaD-2*planarSource.dragAccelerationCoefficient*v*vd;
        const double thetaDD=gravity*(c.first[0]+sn*thetaD)/v-thetaD*vd/v;
        const double u=std::clamp(time/yawDuration,0.,1.),w=1-u,d=1-2*u;
        const double yawD=input.yawAngle*140*u*u*u*w*w*w/yawDuration;
        const double yawDD=input.yawAngle*420*u*u*w*w*d/(yawDuration*yawDuration);
        const double yawDDD=input.yawAngle*840*u*w*(d*d-u*w)/(yawDuration*yawDuration*yawDuration);
        const double a=vd*yawD+v*yawDD,b=vdd*yawD+2*vd*yawDD+v*yawDDD;
        c.lateralG=-v*yawD*cs/gravity;
        c.first[1]=-(a*cs-v*yawD*sn*thetaD)/gravity;
        c.second[1]=-(b*cs-2*a*sn*thetaD-v*yawD*(cs*thetaD*thetaD+sn*thetaDD))/gravity;
        c.rollRate=yawD*sn;
        c.first[2]=yawDD*sn+yawD*cs*thetaD;
        c.second[2]=yawDDD*sn+2*yawDD*cs*thetaD+yawD*(cs*thetaDD-sn*thetaD*thetaD);
        return c;
    };
    // Preserve the original normal-load quintics exactly at every added knot.
    // At most 128 controls fit the existing saved-source format. The yaw/force
    // interpolation is subsequently checked by a fresh independent FVD replay.
    r.controls.clear();r.controls.push_back(initial);State state{r.position,r.forward,r.up,r.speed,0};
    for(size_t i=1;i<planarSource.controls.size();++i){
        const double begin=planarSource.controls[i-1].time,length=planarSource.controls[i].time-begin;
        const int controlsCount=std::max(1,int(std::ceil(length/duration*120)));
        double previous=begin;
        for(int j=1;j<=controlsCount;++j){
            const double time=j==controlsCount?planarSource.controls[i].time:begin+length*j/controlsCount;
            const int steps=std::max(1,int(std::ceil((time-previous)/r.step)));const double dt=(time-previous)/steps;
            for(int k=0;k<steps;++k){poll(cancel);state=advance(state,planarSource,previous+k*dt,dt);}
            r.controls.push_back(coordinated(state,time));previous=time;
        }
    }
    require(r.controls.size()<=128,"FVD_LOOP_BUDGET","Coordinated loop exceeds the persisted control budget");
    out.section=designFvdSection(r,cancel);requireInheritedPort(out.section,input.entry);
    if(out.section.assessment.passed){
        double previousYaw=startYaw,turn=startPitch;
        for(const auto& q:out.section.samples){
            const Vec3 binormal=cross(q.forward,q.up);
            const double planeYaw=std::atan2(binormal.x,-binormal.y);
            const double change=std::remainder(planeYaw-previousYaw,2*pi);previousYaw+=change;
            require(std::abs(binormal.z)<1e-5&&change*input.yawAngle>=-1e-8,"FVD_LOOP_YAW","Loop plane acquired unintended tilt or reversed yaw direction");
            const double u=std::clamp(q.time/yawDuration,0.,1.);
            const double expected=startYaw+input.yawAngle*u*u*u*u*(35+u*(-84+u*(70-20*u)));
            require(std::abs(previousYaw-expected)<1e-5,"FVD_LOOP_YAW","Independent replay differs from the authored loop-plane yaw");
            const Vec3 plane{std::cos(planeYaw),std::sin(planeYaw),0};
            turn+=std::remainder(std::atan2(q.forward.z,dot(q.forward,plane))-turn,2*pi);
            require(q.position.z<=startHeight+input.height+1e-4,"FVD_LOOP_SHAPE","Loop gained a higher unintended crest");
            if(q.time<apexTime-1e-8)require(q.forward.z>=-1e-6,"FVD_LOOP_SHAPE","Loop gained a descent before its intended apex");
        }
        const auto& end=out.section.samples.back();
        require(std::abs(turn-(2*pi+input.exitPitch))<1e-5&&end.up.z>.9,"FVD_LOOP_FRAME","Coordinated loop missed its upright final pitch");
    }
    return out;
}
}

FvdHillResult designFvdLoop(const FvdLoopRequest& input,Cancel cancel){
    FvdHillResult out;
    try{
        poll(cancel);
        const double entrySpeed=input.entry?input.entry->speed:input.entrySpeed;
        require(bounded(entrySpeed,45,80)&&bounded(input.height,50,220)&&input.height<entrySpeed*entrySpeed/(2*gravity)&&
            (input.ascentReleaseSeconds==0||bounded(input.ascentReleaseSeconds,.4,3))&&bounded(input.normalG,2.5,5)&&(input.exitPositiveG==0||bounded(input.exitPositiveG,2.5,5))&&bounded(input.crestG,.7,3)&&bounded(input.yawAngle,-.8,.8)&&
            bounded(input.exitPitch,0,.2)&&bounded(input.exitNormalG,1,3),"FVD_LOOP_INPUT","Loop intent left its bounded force-authoring domain");
        if(input.entry)return inheritedLoopWithPlaneYaw(input,cancel);
        auto& r=out.authoring;r.position={};r.speed=input.entrySpeed;r.step=.0025;
        r.rollingAcceleration=input.rollingAcceleration;r.dragAccelerationCoefficient=input.dragAccelerationCoefficient;
        const auto initial=initializeEntry(r,input.entry,{0,1,0,0});
        const double startHeight=r.position.z,startYaw=std::atan2(r.forward.y,r.forward.x);
        using Parameters=std::array<double,5>;
        // Peak hold, apex unload duration, descending peak hold, real lateral
        // load and a physical closing twist. No endpoint position is imposed.
        constexpr double ramp=1.2,crestHold=.7,release=1;
        const auto apexAt=[&](const auto& p){return ramp+p[0]+p[1]+input.ascentReleaseSeconds;};
        auto controls=[&](const Parameters& p){
            const double peak=ramp+p[0],apex=apexAt(p),crestEnd=apex+crestHold,loaded=crestEnd+ramp,unload=loaded+p[2],end=unload+release;
            r.controls={initial,{ramp,input.normalG,p[3],0},{peak,input.normalG,p[3],0},
                {apex,input.crestG,0,0},{crestEnd,input.crestG,0,0},{loaded,input.exitPositiveG==0?input.normalG:input.exitPositiveG,0,0},
                {unload,input.exitPositiveG==0?input.normalG:input.exitPositiveG,0,0},{end,input.exitNormalG,0,0}};
            if(input.ascentReleaseSeconds>0)r.controls.insert(r.controls.begin()+3,{peak+input.ascentReleaseSeconds,input.crestG,0,0});
            r.twists={{crestEnd,end,p[4]}};
        };
        auto shoot=[&](const Parameters& p){
            controls(p);State state{r.position,r.forward,r.up,r.speed,0};double apexTurn=0,apexHeight=startHeight;
            const double planeX=std::cos(startYaw+input.yawAngle*.5),planeY=std::sin(startYaw+input.yawAngle*.5);
            double turn=input.entry?std::atan2(state.t.z,state.t.x*planeX+state.t.y*planeY):0;
            for(size_t i=1;i<r.controls.size();++i){
                const double begin=r.controls[i-1].time,duration=r.controls[i].time-begin;
                const int count=int(std::ceil(duration/.005));const double dt=duration/count;
                for(int j=0;j<count;++j){poll(cancel);state=advance(state,r,begin+j*dt,dt);
                    turn+=std::remainder(std::atan2(state.t.z,state.t.x*planeX+state.t.y*planeY)-turn,2*pi);}
                if(r.controls[i].time==apexAt(p)){apexTurn=turn;apexHeight=state.p.z;}
            }
            const double yaw=std::atan2(state.t.y,state.t.x);
            return Parameters{(apexHeight-startHeight-input.height)/100,apexTurn-pi,turn-(2*pi+input.exitPitch),
                std::remainder(yaw-startYaw-input.yawAngle,2*pi),dot(state.u,Vec3{std::sin(yaw),-std::cos(yaw),0})};
        };
        Parameters solved{};bool found=false;std::ostringstream diagnostic;
        const double lateral=-2*input.yawAngle;
        for(auto seed:std::array<Parameters,4>{{{2.0,1.8,2.3,lateral,0},{2.5,1.0,2.5,lateral,.1},{1.5,2.5,2.0,lateral,-.1},{3.0,.7,3.0,lateral,0}}}){
            if(shootParameters<5>(seed,{{{.001,6},{input.ascentReleaseSeconds>0?.001:.6,4},{.001,7},{-1.2,1.2},{-.9,.9}}},shoot)){solved=seed;found=true;break;}
            try{const auto error=shoot(seed);diagnostic<<" [";for(double value:seed)diagnostic<<value<<',';diagnostic<<" -> ";for(double value:error)diagnostic<<value<<',';diagnostic<<']';}catch(const Failure& failure){if(failure.code=="CANCELLED")throw;}
        }
        if(!found){out.section.report.fail("FVD_LOOP_SHOOT","Loop cannot meet its apex and frame with selected loads/energy:"+diagnostic.str());return out;}
        controls(solved);out.section=designFvdSection(r,cancel);requireInheritedPort(out.section,input.entry);
        if(out.section.assessment.passed){
            const auto& end=out.section.samples.back();
            require(end.up.z>.9,"FVD_LOOP_FRAME","Loop closing frame is inverted instead of upright");
            const double apexTime=apexAt(solved);
            for(const auto& q:out.section.samples){
                require(q.position.z<=startHeight+input.height+1e-4,"FVD_LOOP_SHAPE","Loop gained a higher unintended crest");
                if(q.time<apexTime-1e-8)require(q.forward.z>=-1e-6,"FVD_LOOP_SHAPE","Loop gained a descent before its intended apex");
            }
        }
    }catch(const Failure& e){out.section.cancelled=e.code=="CANCELLED";out.section.report.fail(e.code,e.what());}
    catch(const std::exception& e){out.section.report.fail("FVD_LOOP",e.what());}
    return out;
}

FvdHillResult designFvdTerrainAct(const FvdTerrainActRequest& input,Cancel cancel){
    FvdHillResult out;
    try{
        require((input.kind==TerrainAct::Clifftop||input.kind==TerrainAct::RavineRoll)&&bounded(input.entrySpeed,20,80)&&bounded(input.entryPitch,-.7,.5)&&bounded(input.exitPitch,-.4,.1)&&bounded(input.heightChange,-160,30)&&
            bounded(std::abs(input.headingChange),.2,3.5)&&bounded(input.airtimeG,-1.35,-.3)&&bounded(input.outwardBank,10*pi/180,60*pi/180)&&bounded(input.durationScale,.6,1.6),"FVD_TERRAIN_ACT_INPUT","Terrain act is outside its bounded force family");
        auto& r=out.authoring;r.position={0,0,0};r.speed=input.entrySpeed;r.step=.0025;r.gravityReferencedRoll=true;
        r.forward={std::cos(input.entryPitch),0,std::sin(input.entryPitch)};r.up={-r.forward.z,0,r.forward.x};
        r.rollingAcceleration=input.rollingAcceleration;r.dragAccelerationCoefficient=input.dragAccelerationCoefficient;
        const double v=r.speed,k=input.pitchRateS,ks=input.pitchSecondS,kss=input.pitchThirdS,pitch=input.entryPitch;
        const double a=-gravity*std::sin(pitch)-r.rollingAcceleration-r.dragAccelerationCoefficient*v*v;
        const double at=-gravity*std::cos(pitch)*v*k-2*r.dragAccelerationCoefficient*v*a;
        FvdControl initial{0,v*v*k/gravity+std::cos(pitch),0,0};
        initial.first[0]=(v*v*v*ks+2*v*a*k)/gravity-std::sin(pitch)*v*k;
        initial.second[0]=(v*v*v*v*kss+5*v*v*a*ks+2*(a*a+v*at)*k)/gravity-std::cos(pitch)*v*v*k*k-std::sin(pitch)*(a*k+v*v*ks);
        using Parameters=std::array<double,3>;const double scale=input.durationScale,hand=std::copysign(1.,input.headingChange);
        auto controls=[&](const Parameters& p){
            if(input.kind==TerrainAct::Clifftop){
                const double t1=.9*scale,t2=t1+p[0],t3=t2+.9*scale,t4=t3+.35*scale,t5=t4+scale,t6=t5+p[1],t7=t6+scale;
                r.controls={initial,{t1,3.2,0,0},{t2,3.2,0,0},{t3,input.airtimeG,0,0},{t4,input.airtimeG,0,0},{t5,2.6,0,0},{t6,2.6,0,0},{t7,std::cos(input.exitPitch),0,0}};
                // Finish each inbank before its loaded dwell, keeping the
                // positive force directed through the turn instead of lofting
                // a large hill while the bank is still catching up.
                r.twists={{.15*scale,t1,-55*pi/180},{t2,t4,55*pi/180+input.outwardBank},{t4,t5,-input.outwardBank-p[2]},{t6,t7,p[2]}};
            }else{
                const double t1=1.1*scale,t2=t1+p[0],t3=t2+1.2*scale,t4=t3+p[1],t5=t4+1.4*scale;
                r.controls={initial,{t1,input.airtimeG,0,0},{t2,input.airtimeG,0,0},{t3,3.6,0,0},{t4,3.6,0,0},{t5,std::cos(input.exitPitch),0,0}};
                r.twists={{.15*scale,t1+std::min(.3*scale,p[0]*.4),input.outwardBank},{t2,t3+std::min(scale,p[1]*.5),-input.outwardBank-p[2]},{t4,t5,p[2]}};
            }
            for(auto& twist:r.twists)twist.angle*=hand;
        };
        struct Shot {State state;double maximumHeight{},maximumPitch{},yaw{};};
        auto shot=[&](const Parameters& p){controls(p);Shot result;result.state={r.position,r.forward,r.up,r.speed,0};
            for(size_t i=1;i<r.controls.size();++i){const double begin=r.controls[i-1].time,duration=r.controls[i].time-begin;const int count=int(std::ceil(duration/.005));const double dt=duration/count;
                for(int j=0;j<count;++j){poll(cancel);result.state=advance(result.state,r,begin+j*dt,dt);result.maximumHeight=std::max(result.maximumHeight,result.state.p.z);result.maximumPitch=std::max(result.maximumPitch,std::abs(std::asin(result.state.t.z)));result.yaw+=std::remainder(std::atan2(result.state.t.y,result.state.t.x)-result.yaw,2*pi);}}
            return result;};
        auto residual=[&](const Parameters& p){const auto q=shot(p);return Parameters{(q.state.p.z-input.heightChange)/50,std::asin(q.state.t.z)-input.exitPitch,q.yaw-input.headingChange};};
        bool found=false;Parameters selected{};std::string failure;
        const std::array<Parameters,5> seeds=input.kind==TerrainAct::Clifftop?
            std::array<Parameters,5>{{{1.1,2,45*pi/180},{.5,2.5,35*pi/180},{2,2,60*pi/180},{.3,3,30*pi/180},{2,3,45*pi/180}}}:
            std::array<Parameters,5>{{{.8,5,65*pi/180},{.4,5.5,60*pi/180},{1.2,6,70*pi/180},{.5,4,50*pi/180},{1.6,7,70*pi/180}}};
        const std::array<std::pair<double,double>,3> bounds{{{.01,input.kind==TerrainAct::Clifftop?4.:2.5},{.2,9},{20*pi/180,78*pi/180}}};
        for(auto p:seeds){try{const bool converged=shootParameters<3>(p,bounds,residual);const auto q=shot(p);
            if(!converged||q.maximumPitch>.9||(input.kind==TerrainAct::RavineRoll&&q.maximumHeight>.001)){
                std::ostringstream detail;detail<<"holds="<<p[0]<<','<<p[1]<<", bank="<<p[2]<<", residual="<<(q.state.p.z-input.heightChange)<<','<<(std::asin(q.state.t.z)-input.exitPitch)<<','<<(q.yaw-input.headingChange)<<", maxPitch="<<q.maximumPitch<<", maxRise="<<q.maximumHeight;failure=detail.str();continue;}
            selected=p;found=true;break;
        }catch(const Failure& e){if(e.code=="CANCELLED")throw;failure=e.what();}catch(const std::exception& e){failure=e.what();}}
        if(!found)throw Failure("FVD_TERRAIN_ACT_SHOOT","Coordinated terrain act cannot meet its height, upright exit and yaw: "+failure);
        controls(selected);out.section=designFvdSection(r,cancel);
        if(out.section.assessment.passed){const auto& end=out.section.samples.back();const auto upright=unit(Vec3{0,0,1}-end.forward*end.forward.z);require(std::abs(end.forward.z-std::sin(input.exitPitch))<1e-6&&norm(end.up-upright)<1e-6,"FVD_TERRAIN_ACT_EXIT","Terrain act did not reach its aligned brake/coast exit");}
    }catch(const Failure& e){out.section.cancelled=e.code=="CANCELLED";out.section.report.fail(e.code,e.what());}
    catch(const std::exception& e){out.section.report.fail("FVD_TERRAIN_ACT",e.what());}
    return out;
}

FvdHillResult designFvdApproach(const FvdApproachRequest& input,Cancel cancel){
    FvdHillResult out;
    try{
        poll(cancel);const auto& port=input.entry;const auto& k=port.jet;
        require(bounded(port.speed,15,80)&&finite(input.endPosition)&&bounded(input.normalG,1.5,4)&&
            bounded(input.bankRampSeconds,1,3)&&std::isfinite(input.endHeading),"FVD_APPROACH_INPUT","Approach intent left its bounded force family");
        require(std::abs(k.position.z-input.endPosition.z)<1e-7&&std::abs(k.tangent.z)<1e-8&&norm(k.up-Vec3{0,0,1})<1e-8&&
            norm(k.curvature)+norm(k.third)+norm(k.fourth)+norm(k.upFirst)+norm(k.upSecond)+norm(k.upThird)<1e-8,
            "FVD_APPROACH_ENTRY","Level approach requires the actual straight upright entry jet; it cannot reset a curved port");
        const double yaw=std::atan2(k.tangent.y,k.tangent.x),turn=std::remainder(input.endHeading-yaw,2*pi);
        require(bounded(std::abs(turn),.35,2.5),"FVD_APPROACH_TURN","Approach needs a resolved single turn; recompose the upstream return for this heading");
        auto& r=out.authoring;r.step=.0025;r.gravityReferencedRoll=true;
        r.rollingAcceleration=input.rollingAcceleration;r.dragAccelerationCoefficient=input.dragAccelerationCoefficient;
        const auto initial=initializeEntry(r,port,{0,1,0,0});
        const double bank=-std::copysign(std::acos(1/input.normalG),turn),ramp=input.bankRampSeconds;
        using Parameters=std::array<double,3>;
        auto controls=[&](const Parameters& p){
            r.controls={initial,{p[0],1,0,0}};
            auto transition=[&](double begin,bool entering){
                for(int j=1;j<=60;++j){
                    const double u=j/60.,w=1-u,d=1-2*u,sign=entering?1.:-1.;
                    const double q=u*u*u*u*(35+u*(-84+u*(70-20*u)));
                    const double phi=bank*(entering?q:1-q);
                    const double rate=sign*bank*140*u*u*u*w*w*w/ramp;
                    const double acceleration=sign*bank*420*u*u*w*w*d/(ramp*ramp);
                    const double jerk=sign*bank*840*u*w*(d*d-u*w)/(ramp*ramp*ramp);
                    const double n=1/std::cos(phi),tan=std::tan(phi);
                    FvdControl c{begin+ramp*u,n,0,rate};
                    c.first[0]=n*tan*rate;c.second[0]=n*((tan*tan+n*n)*rate*rate+tan*acceleration);
                    c.first[2]=acceleration;c.second[2]=jerk;r.controls.push_back(c);
                }
            };
            transition(p[0],true);const double release=p[0]+ramp+p[1];
            r.controls.push_back({release,input.normalG,0,0});transition(release,false);
            r.controls.push_back({release+ramp+p[2],1,0,0});
        };
        auto residual=[&](const Parameters& p){
            controls(p);State state{r.position,r.forward,r.up,r.speed,0};
            for(size_t i=1;i<r.controls.size();++i){const double begin=r.controls[i-1].time,duration=r.controls[i].time-begin;
                const int count=int(std::ceil(duration/.005));const double dt=duration/count;
                for(int j=0;j<count;++j){poll(cancel);state=advance(state,r,begin+j*dt,dt);}}
            return Parameters{state.p.x-input.endPosition.x,state.p.y-input.endPosition.y,
                50*std::remainder(std::atan2(state.t.y,state.t.x)-input.endHeading,2*pi)};
        };
        bool solved=false;Parameters selected{};std::ostringstream diagnostic;
        for(auto p:std::array<Parameters,4>{{{2,.5,1},{4,.1,1},{1,2,4},{1,.2,6}}}){
            if(shootParameters<3>(p,{{{.001,12},{.001,8},{.001,12}}},residual)){selected=p;solved=true;break;}
            try{const auto e=residual(p);diagnostic<<" [";for(double v:p)diagnostic<<v<<',';diagnostic<<" -> ";for(double v:e)diagnostic<<v<<',';diagnostic<<']';}
            catch(const Failure& e){if(e.code=="CANCELLED")throw;diagnostic<<" ["<<e.code<<']';}
        }
        if(!solved)throw Failure("FVD_APPROACH_SHOOT","Level force-authored approach cannot reach the station from this return port:"+diagnostic.str());
        controls(selected);out.section=designFvdSection(r,cancel);requireInheritedPort(out.section,port);
        if(out.section.assessment.passed){const auto& end=out.section.samples.back();
            require(norm(end.position-input.endPosition)<1e-7&&std::abs(end.forward.z)<1e-8&&
                std::abs(std::remainder(std::atan2(end.forward.y,end.forward.x)-input.endHeading,2*pi))<1e-8&&norm(end.up-Vec3{0,0,1})<1e-8,
                "FVD_APPROACH_EXIT","Independent force replay missed the level station approach port");}
    }catch(const Failure& e){out.section.cancelled=e.code=="CANCELLED";out.section.report.fail(e.code,e.what());}
    catch(const std::exception& e){out.section.report.fail("FVD_APPROACH",e.what());}
    return out;
}

namespace {
// Add a world-vertical frame yaw through actual normal/lateral/twist forces.
// Rotating the frame preserves speed and height, but its horizontal path is
// newly integrated; no endpoint or track positions are transformed afterward.
FvdRequest yawAscendingFrame(const FvdRequest& base,double angle,double end,Cancel cancel){
    FvdRequest result=base;result.controls.clear();
    const double yawBegin=base.controls[1].time,duration=end-yawBegin;
    State state{base.position,base.forward,base.up,base.speed,0};
    auto adjusted=[&](double time){
        auto c=sampleFvdControl(base.controls,time);
        if(time<=yawBegin||time>=end)return c;
        const double u=std::clamp((time-yawBegin)/duration,0.,1.),w=1-u,d=1-2*u;
        const double yd=angle*140*u*u*u*w*w*w/duration;
        const double ydd=angle*420*u*u*w*w*d/(duration*duration);
        const double yddd=angle*840*u*w*(d*d-u*w)/(duration*duration*duration);
        const double v=state.v,a=c.drive+dot(gVector,state.t)-loss(base,v);
        const FvdSample sample{time,0,v,state.p,state.t,state.u,{},0,0};
        const auto k=authoredKnot(sample,base);
        const double jerk=c.first[3]+dot(gVector,k.curvature)*v-2*base.dragAccelerationCoefficient*v*a;
        const Vec3 right=cross(state.t,state.u);
        const Vec3 rightS=cross(k.curvature,state.u)+cross(state.t,k.upFirst);
        const Vec3 rightSS=cross(k.third,state.u)+cross(k.curvature,k.upFirst)*2+cross(state.t,k.upSecond);
        const double factor=v*yd,first=a*yd+v*ydd,second=jerk*yd+2*a*ydd+v*yddd;
        const auto force=[&](double sign,Vec3 axis,Vec3 axisS,Vec3 axisSS,int channel,double& value){
            const double axisD=axisS.z*v,axisDD=axisSS.z*v*v+axisS.z*a;
            value+=sign*factor*axis.z/gravity;
            c.first[channel]+=sign*(first*axis.z+factor*axisD)/gravity;
            c.second[channel]+=sign*(second*axis.z+2*first*axisD+factor*axisDD)/gravity;
        };
        force(1,right,rightS,rightSS,0,c.normalG);
        force(-1,state.u,k.upFirst,k.upSecond,1,c.lateralG);
        c.rollRate+=yd*state.t.z;
        c.first[2]+=ydd*state.t.z+yd*k.curvature.z*v;
        c.second[2]+=yddd*state.t.z+2*ydd*k.curvature.z*v+yd*(k.third.z*v*v+k.curvature.z*a);
        return c;
    };
    result.controls.push_back(base.controls.front());
    for(size_t i=1;i<base.controls.size();++i){
        const double begin=base.controls[i-1].time,length=base.controls[i].time-begin;
        const int count=begin>=yawBegin&&begin<end?std::max(1,int(std::ceil(length/duration*100))):1;
        double previous=begin;
        for(int j=1;j<=count;++j){
            const double time=j==count?base.controls[i].time:begin+length*j/count;
            const int steps=std::max(1,int(std::ceil((time-previous)/base.step)));const double dt=(time-previous)/steps;
            for(int k=0;k<steps;++k){poll(cancel);state=advance(state,base,previous+k*dt,dt);}
            result.controls.push_back(adjusted(time));previous=time;
        }
    }
    require(result.controls.size()<=128,"FVD_IMMELMANN_BUDGET","Yawing half-loop exceeds the persisted control budget");
    return result;
}
}

FvdImmelmannResult designFvdImmelmann(const FvdImmelmannRequest& input,Cancel cancel){
    FvdImmelmannResult out;
    try{
        poll(cancel);
        const double entrySpeed=input.entry?input.entry->speed:input.entrySpeed;
        const double exitRamp=input.exitRampSeconds==0?input.rampSeconds:input.exitRampSeconds;
        require(bounded(entrySpeed,45,80)&&bounded(input.height,75,180)&&bounded(input.exitHeight,-180,40)&&
            input.exitHeight<input.height&&bounded(input.exitPitch,-.3,0)&&bounded(input.exitNormalG,.8,4)&&bounded(input.normalG,3,5.5)&&(input.exitPositiveG==0||bounded(input.exitPositiveG,3,5.5))&&bounded(input.crestG,.05,4)&&
            bounded(input.rollExitG,.05,3.5)&&bounded(input.rampSeconds,.8,2)&&bounded(exitRamp,.8,2)&&bounded(input.rollOverlapFraction,0,.6)&&
            bounded(input.yawAngle,-1.4,1.4)&&(input.yawAngle==0||input.rollOverlapFraction==0)&&bounded(input.rollReleaseFraction,0,.6)&&
            (input.ascentReleaseSeconds==0||(bounded(input.ascentReleaseSeconds,.4,3)&&input.rollOverlapFraction==0))&&
            (input.hand==1||input.hand==-1)&&bounded(input.step,.0001,.05),"FVD_IMMELMANN_INPUT","Immelmann intent left its bounded authoring family");
        auto& r=out.authoring;r.position={0,0,0};r.speed=input.entrySpeed;r.step=input.step;
        r.rollingAcceleration=input.rollingAcceleration;r.dragAccelerationCoefficient=input.dragAccelerationCoefficient;
        const auto initial=initializeEntry(r,input.entry,{0,1,0,0});
        const double startHeight=r.position.z;const auto horizontal=unit(Vec3{r.forward.x,r.forward.y,0});
        validate(r);
        using Parameters=std::array<double,5>;
        // Unknowns: peak hold, apex unload (or crown hold), roll duration, physical twist and
        // valley unload. All other load and phase intents remain prescribed.
        // Residuals: apex height/pitch, upright roll exit, valley height/pitch.
        struct Shot {State apex,rollExit,exit;double minimumZ{},maximumDescentPitch{};};
        const auto apexAt=[&](const Parameters& p){return input.rampSeconds+p[0]+p[1]+input.ascentReleaseSeconds;};
        const auto controls=[&](const Parameters& p){
            const double apex=apexAt(p),rollEnd=apex+p[2],pullPeak=rollEnd+exitRamp;
            r.controls={initial,{input.rampSeconds,input.normalG,0,0},{input.rampSeconds+p[0],input.normalG,0,0},
                {apex,input.crestG,0,0},{rollEnd,input.rollExitG,0,0},{pullPeak,input.exitPositiveG==0?input.normalG:input.exitPositiveG,0,0},{pullPeak+p[4],input.exitNormalG,0,0}};
            if(input.rollReleaseFraction>0)r.controls.insert(r.controls.begin()+4,
                {apex+input.rollReleaseFraction*p[2],input.crestG,0,0});
            if(input.ascentReleaseSeconds>0)r.controls.insert(r.controls.begin()+3,
                {input.rampSeconds+p[0]+input.ascentReleaseSeconds,input.crestG,0,0});
            // Give acceleration and release most of the half-roll. The former
            // 20% ramps concentrated lateral seat acceleration at both ends;
            // a broad C2 rate profile preserves the authored twist without
            // using a short angular impulse to manufacture the sensation.
            const double rollBegin=apex-input.rollOverlapFraction*p[1],ramp=.4*(rollEnd-rollBegin);
            const double rate=input.hand*p[3]/(rollEnd-rollBegin-ramp);
            const std::vector<FvdControl> roll{{rollBegin,1,0,0},{rollBegin+ramp,1,0,rate},
                {rollEnd-ramp,1,0,rate},{rollEnd,1,0,0}};
            const auto force=r.controls;std::vector<double> times;
            for(const auto& c:force)times.push_back(c.time);
            for(const auto& c:roll)times.push_back(c.time);
            std::sort(times.begin(),times.end());
            // The same authored boundary can differ by roundoff when formed
            // from an interval length. Only machine-equivalent times coincide;
            // genuinely distinct sub-millisecond phases still reject below.
            times.erase(std::unique(times.begin(),times.end(),[](double a,double b){
                return std::abs(a-b)<=16*std::numeric_limits<double>::epsilon()*std::max({1.,std::abs(a),std::abs(b)});
            }),times.end());
            for(size_t i=1;i<times.size();++i)require(times[i]-times[i-1]>=.001,
                "FVD_IMMELMANN_KNOT_SPACING","Merged force and roll phases must retain at least 0.001 s between distinct controls");
            r.controls.clear();r.twists.clear();
            for(double time:times){auto c=sampleFvdControl(force,time);
                if(time>=rollBegin&&time<=rollEnd){const auto spin=sampleFvdControl(roll,time);
                    c.rollRate+=spin.rollRate;c.first[2]+=spin.first[2];c.second[2]+=spin.second[2];}
                r.controls.push_back(c);
            }
        };
        const auto shoot=[&](const Parameters& p,bool apexOnly=false){
            controls(p);const double apexTime=apexAt(p),rollTime=apexTime+p[2];State state{r.position,r.forward,r.up,r.speed,0};Shot result;result.minimumZ=startHeight+input.height;
            for(size_t i=1;i<r.controls.size();++i){
                const double begin=r.controls[i-1].time,duration=r.controls[i].time-begin;
                const size_t count=size_t(std::ceil(duration/.005));const double dt=duration/count;
                for(size_t j=0;j<count;++j){poll(cancel);state=advance(state,r,begin+j*dt,dt);
                    if(begin>=apexTime){result.minimumZ=std::min(result.minimumZ,state.p.z);result.maximumDescentPitch=std::max(result.maximumDescentPitch,state.t.z);}}
                if(r.controls[i].time==apexTime){result.apex=state;if(apexOnly)return result;}if(r.controls[i].time==rollTime)result.rollExit=state;
            }
            result.exit=state;return result;
        };
        auto residual=[&](const Parameters& p){
            const auto s=shoot(p);const Vec3 right=unit(cross(s.rollExit.t,Vec3{0,0,1}));
            return std::array<double,5>{(s.apex.p.z-startHeight-input.height)/100,s.apex.t.z,
                std::atan2(dot(s.rollExit.u,right),s.rollExit.u.z),(s.exit.p.z-startHeight-input.exitHeight)/50,s.exit.t.z-std::sin(input.exitPitch)};
        };
        const double minimumUnload=input.ascentReleaseSeconds>0?.001:input.rollOverlapFraction>0?std::max(.2,.001/input.rollOverlapFraction):.2;
        const double minimumRoll=input.rollReleaseFraction>0?std::max(.5,.001/input.rollReleaseFraction):.5;
        require(minimumUnload<=12&&minimumRoll<=8,"FVD_IMMELMANN_KNOT_SPACING",
            "Requested roll onset cannot retain 0.001 s control spacing inside the bounded phase durations");
        Parameters parameters{};bool solved=false;const double scale=entrySpeed/53;std::ostringstream diagnostic;
        std::array<double,2> ascent{};
        if(input.rollOverlapFraction==0){
            bool found=false;
            const auto apexResidual=[&](const std::array<double,2>& p){
                const auto s=shoot({p[0],p[1],std::max(2.,minimumRoll),2,1},true);
                return std::array<double,2>{(s.apex.p.z-startHeight-input.height)/100,s.apex.t.z};
            };
            // A held crown follows the explicit unload. Include a short hold
            // seed so nearby speeds do not begin beyond the half-loop apex.
            for(auto seed:std::array<std::array<double,2>,5>{{{.5,3.2},{1.2,2},{2,1},{.04,2.5},{.5,1.2}}}){
                for(double& value:seed)value*=scale;
                if(shootParameters<2>(seed,{{{.001,8},{minimumUnload,12}}},apexResidual)){ascent=seed;found=true;break;}
            }
            require(found,"FVD_IMMELMANN_SHOOT","Ascending half-loop cannot meet its height and apex pitch within the bounded force phases");
        }
        for(auto seed:std::array<Parameters,4>{{{.5,3.2,2,2,1},{1.2,2,3,pi,1},{.5,3.2,3,pi,1},{2,1,3.5,pi,1}}}){
            for(size_t i:{size_t(0),size_t(1),size_t(2),size_t(4)})seed[i]*=scale;
            seed[2]*=std::sqrt((input.height-input.exitHeight)/85.);
            seed[1]=std::max(seed[1],minimumUnload);seed[2]=std::max(seed[2],minimumRoll);
            bool converged=false;
            if(input.rollOverlapFraction==0){
                // A solved ascent is shared by every independent valley seed.
                seed[0]=ascent[0];seed[1]=ascent[1];std::array<double,3> valley{seed[2],seed[3],seed[4]};
                const auto valleyResidual=[&](const std::array<double,3>& p){auto trial=seed;for(size_t j=0;j<3;++j)trial[j+2]=p[j];
                    const auto error=residual(trial);return std::array<double,3>{error[2],error[3],error[4]};};
                converged=shootParameters<3>(valley,{{{minimumRoll,8},{1,5.2},{.4,4}}},valleyResidual);
                for(size_t j=0;j<3;++j)seed[j+2]=valley[j];
            }else converged=shootParameters<5>(seed,{{{.001,8},{minimumUnload,12},{minimumRoll,8},{1,5.2},{.4,4}}},residual);
            if(!converged){
                try{const auto error=residual(seed);diagnostic<<" [";for(double x:seed)diagnostic<<x<<',';diagnostic<<" -> ";for(double x:error)diagnostic<<x<<',';diagnostic<<']';}
                catch(const Failure& e){if(e.code=="CANCELLED")throw;diagnostic<<" ["<<e.code<<']';}
                continue;
            }
            const auto s=shoot(seed);
            if(s.apex.u.z>=-.5||dot(s.apex.t,horizontal)>=-.5||s.rollExit.t.z>=-.02||s.minimumZ<startHeight+input.exitHeight-1e-5||s.maximumDescentPitch>1e-5){diagnostic<<" [shape guard]";continue;}
            parameters=seed;solved=true;break;
        }
        if(!solved)throw Failure("FVD_IMMELMANN_SHOOT","Coupled Immelmann force/roll intent has no bounded single-crest valley solution:"+diagnostic.str());
        controls(parameters);
        const double apexTime=apexAt(parameters),rollTime=apexTime+parameters[2];
        if(input.yawAngle!=0)r=yawAscendingFrame(r,input.yawAngle,apexTime,cancel);
        out.section=designFvdSection(r,cancel);requireInheritedPort(out.section,input.entry);
        if(!out.section.report.valid()||out.section.samples.empty())return out;
        double previousHeight=startHeight;
        for(const auto& q:out.section.samples){
            poll(cancel);
            if(std::abs(q.time-apexTime)<1e-8)out.apex=q;
            if(std::abs(q.time-rollTime)<1e-8)out.rollExit=q;
            require(q.position.z>=startHeight+std::min(0.,input.exitHeight)-1e-5&&
                (q.time>apexTime+1e-8?q.position.z<=previousHeight+1e-5:q.position.z>=previousHeight-1e-5),
                "FVD_IMMELMANN_SHAPE","Integrated Immelmann left its single ascending-then-descending height domain");
            previousHeight=q.position.z;
        }
        out.exit=out.section.samples.back();
        const Vec3 ascentHorizontal{horizontal.x*std::cos(input.yawAngle)-horizontal.y*std::sin(input.yawAngle),
            horizontal.x*std::sin(input.yawAngle)+horizontal.y*std::cos(input.yawAngle),0};
        require(out.apex.up.z<-.5&&dot(out.apex.forward,ascentHorizontal)<-.5&&std::abs(out.apex.forward.z)<1e-6&&
            std::abs(out.apex.position.z-startHeight-input.height)<1e-5&&std::abs(out.exit.position.z-startHeight-input.exitHeight)<1e-5&&
            std::abs(out.exit.forward.z-std::sin(input.exitPitch))<1e-6&&norm(out.exit.up-unit(Vec3{0,0,1}-out.exit.forward*out.exit.forward.z))<1e-6,
            "FVD_IMMELMANN_PORT","Integrated Immelmann missed its apex or live upright exit");
        const double headingAlignment=dot(horizontal,unit(Vec3{out.exit.forward.x,out.exit.forward.y,0}));
        if(headingAlignment>=-.8)throw Failure("FVD_IMMELMANN_HEADING",
            "Immelmann must reverse its physical travel heading; exit change="+
            std::to_string(std::acos(std::clamp(headingAlignment,-1.,1.))*180/pi));
    }catch(const Failure& e){out.section.cancelled=e.code=="CANCELLED";out.section.report.fail(e.code,e.what());}
    catch(const std::exception& e){out.section.report.fail("FVD_IMMELMANN",e.what());}
    return out;
}

}
