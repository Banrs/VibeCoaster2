#include "coaster/angular_phase_intent.hpp"
#include "../src/launch_camelback.hpp"
#include <iomanip>
#include "coaster/clearance.hpp"
#include <iostream>
#include <fstream>
#include <stdexcept>
using namespace coaster;
namespace {
void vec(const char* name,const std::array<double,3>& value){std::cout<<' '<<name<<'='<<value[0]<<','<<value[1]<<','<<value[2];}
std::array<double,3> planarMotion(const FvdRequest& request,const FvdSample& q){
    const auto c=sampleFvdControl(request.controls,q.time);
    const double v=q.speed,sine=q.forward.z,cosine=q.forward.x;
    const double a=c.drive-gravity*sine-request.rollingAcceleration-request.dragAccelerationCoefficient*v*v;
    const double w=gravity*(c.normalG-cosine)/v;
    const double alpha=(gravity*(c.first[0]+sine*w)-w*a)/v;
    const double j=c.first[3]-gravity*cosine*w-2*request.dragAccelerationCoefficient*v*a;
    const double beta=(gravity*(c.second[0]+cosine*w*w+sine*alpha)-2*alpha*a-w*j)/v;
    return {w,alpha,beta};
}
void planarProbe(){
    auto launch=designLaunchCamelback(83.5,-10*pi/180,CamelbackParameters{},gravity*.004,.0002041666666667);
    for(double offset:{0.,4000.})for(double step:{.01,.005,.0025}){
        auto request=launch.camelback.authoring;request.step=step;request.maxSamples=50000;request.position.x+=offset;request.position.z+=offset;
        const auto source=designFvdSection(request);
        if(!source.assessment.passed)throw std::runtime_error("Planar source replay failed");
        double knotError=0,interiorError=0,numericError=0,worstTime=0,worstWidth=0;
        for(size_t i=1;i+1<source.samples.size();++i){
            const auto& q=source.samples[i];const auto c=sampleFvdControl(request.controls,q.time);
            const auto exact=planarMotion(request,q);const auto& k=source.track.knots[i];
            const double a=c.drive-gravity*q.forward.z-request.rollingAcceleration-request.dragAccelerationCoefficient*q.speed*q.speed;
            const double j=c.first[3]-gravity*q.forward.x*exact[0]-2*request.dragAccelerationCoefficient*q.speed*a;
            const TrackKinematics jet{{k.position,k.tangent,k.curvature,k.up,cross(k.tangent,k.up),k.element},k.upFirst,k.upSecond,k.third,k.upThird,k.fourth};
            knotError=std::max(knotError,std::abs(signedAngularMotion(jet,q.speed,a,j).jerk[1]-exact[2]));
            const auto& next=source.samples[i+1];
            for(double u:{.0625,.125,.25,.5,.75,.875,.9375}){
            FvdSample middle=q;middle.time=std::lerp(q.time,next.time,u);middle.speed=std::lerp(q.speed,next.speed,u);
            const auto sample=sampleSpanKinematics(source.track,i,u);middle.forward=sample.sample.tangent;
            const auto mid=planarMotion(request,middle);const auto mc=sampleFvdControl(request.controls,middle.time);
            const double ma=mc.drive-gravity*middle.forward.z-request.rollingAcceleration-request.dragAccelerationCoefficient*middle.speed*middle.speed;
            const double mj=mc.first[3]-gravity*middle.forward.x*mid[0]-2*request.dragAccelerationCoefficient*middle.speed*ma;
            const double error=std::abs(signedAngularMotion(sample,middle.speed,ma,mj).jerk[1]-mid[2]);
            if(error>interiorError){interiorError=error;worstTime=middle.time;worstWidth=source.track.spans[i].length;}
            }
            const auto& prev=source.samples[i-1];const double h=q.time-prev.time;
            if(std::abs(next.time-q.time-h)<1e-10){
                const double numeric=(planarMotion(request,prev)[0]-2*exact[0]+planarMotion(request,next)[0])/(h*h);
                numericError=std::max(numericError,std::abs(numeric-exact[2]));
            }
        }
        std::cout<<"planar offset="<<offset<<" step="<<step<<" knotJerkError="<<knotError<<" rawDifferenceError="<<numericError
            <<" interiorJerkError="<<interiorError<<" worstTime="<<worstTime<<" spanMeters="<<worstWidth<<'\n';
    }
}

void camelbackProfile(const char* path){
    const auto r=designLaunchCamelback(83.5,-10*pi/180,CamelbackParameters{},gravity*.004,.0002041666666667);
    std::ofstream file(path);file<<std::setprecision(17)<<"s,t,x,z,tx,tz,kx,kz,v,gz,phase\n";
    auto write=[&](const FvdSample& q,double ds,double dt,Vec3 offset,const FvdRequest& program,const char* phase){
        file<<q.distance+ds<<','<<q.time+dt<<','<<q.position.x+offset.x<<','<<q.position.z+offset.z<<','<<q.forward.x<<','<<q.forward.z
            <<','<<q.curvature.x<<','<<q.curvature.z<<','<<q.speed<<','<<sampleFvdControl(program.controls,q.time).normalG<<','<<phase<<'\n';
    };
    for(const auto& q:r.pullout.section.samples)write(q,0,0,{},r.pullout.authoring,"pullout");
    const auto& samples=r.camelback.section.samples;
    const auto first=std::lower_bound(samples.begin(),samples.end(),r.camelback.authoring.controls[1].time,[](const auto& q,double t){return q.time<t;});
    const auto& last=r.pullout.section.samples.back();
    for(auto q=first+1;q!=samples.end();++q)write(*q,last.distance-first->distance,last.time-first->time,last.position-first->position,r.camelback.authoring,"reference");
    if(!file)throw std::runtime_error("Cannot export camelback diagnostic");
    std::cout<<"Exported actual coupled FVD force profile\n";
}

void immelmannSourceProbe(const char* path){
    Design saved;std::string error;if(!loadDesign(path,saved,error))throw std::runtime_error(error);
    const auto at=std::find_if(saved.forcePrograms.begin(),saved.forcePrograms.end(),[](const auto& p){return p.name=="loop";});
    if(at==saved.forcePrograms.end())throw std::runtime_error("Saved source has no loop");
    const auto source=designFvdSection(at->program);if(!source.assessment.passed)throw std::runtime_error("Saved loop did not replay");
    for(double normal:{4.85})for(double rise:{85.,87.,89.})for(double crest:{3.2})for(double rollLoad:{3.8,4.})for(double recovery:{3.})for(double overlap:{0.})for(double ramp:{1.0})for(double exitHeight:{10.})for(double yaw:{55.,65.})for(double exitRamp:{1.2})for(double release:{0.})for(double ascentRelease:{1.2}){
        FvdImmelmannRequest request;request.rollingAcceleration=at->program.rollingAcceleration;request.dragAccelerationCoefficient=at->program.dragAccelerationCoefficient;
        request.entry=makeFvdEntry(source.track.knots.back(),source.samples.back().speed,request.rollingAcceleration,request.dragAccelerationCoefficient);
        request.height=rise*std::pow(request.entry->speed/53,2);request.exitHeight=exitHeight;
        request.exitPitch=-5*pi/180;request.exitNormalG=std::cos(request.exitPitch);request.normalG=normal;request.exitPositiveG=recovery;request.crestG=crest;
        request.rollExitG=rollLoad;request.rampSeconds=ramp;request.rollOverlapFraction=overlap;request.yawAngle=yaw*pi/180;request.exitRampSeconds=exitRamp;request.rollReleaseFraction=release;request.ascentReleaseSeconds=ascentRelease;request.hand=-1;
        const auto result=designFvdImmelmann(request);
        std::cout<<"production-immelmann normal="<<normal<<" referenceRise="<<rise<<" crest="<<crest<<" rollG="<<rollLoad<<" recovery="<<recovery<<" ramp="<<ramp<<" overlap="<<overlap<<" yaw="<<yaw<<" exitRamp="<<exitRamp<<" release="<<release<<" ascentRelease="<<ascentRelease<<" entrySpeed="<<request.entry->speed
            <<" height="<<request.height<<" exitHeight="<<request.exitHeight<<" passed="<<(result.section.report.valid()&&result.section.assessment.passed);
        if(result.section.assessment.passed){double duration=0;
            for(size_t i=1;i<result.section.samples.size();++i)if(sampleFvdControl(result.authoring.controls,result.section.samples[i].time).normalG>4.1)duration+=result.section.samples[i].time-result.section.samples[i-1].time;
            int sign=0,changes=0;const double apexTime=result.apex.time;
            for(const auto& q:result.section.samples)if(q.time>=apexTime){const double dz=result.section.track.sample(q.distance).curvature.z;if(std::abs(dz)>3e-5){const int next=dz>0?1:-1;if(sign&&sign!=next)++changes;sign=next;}}
            double peakRoll=0,peakLateral=0,nearest=INFINITY,halfRollG=0;
            for(const auto& q:result.section.samples){
                const auto k=sampleKinematics(result.section.track,q.distance);
                peakRoll=std::max(peakRoll,std::abs(dot(cross(k.sample.up,k.upS),k.sample.tangent)*q.speed));
                const double acceleration=-gravity*q.forward.z-request.rollingAcceleration-request.dragAccelerationCoefficient*q.speed*q.speed;
                const auto seatForce=measureSeatForces(result.section.track,q.distance,q.speed,acceleration,TrainConfig{}.seatHeight);
                peakLateral=std::max(peakLateral,std::abs(seatForce.lateral));
                if(q.time>=result.apex.time&&q.time<=result.rollExit.time&&std::abs(q.up.z)<nearest){nearest=std::abs(q.up.z);
                    const double acceleration=-gravity*q.forward.z-request.rollingAcceleration-request.dragAccelerationCoefficient*q.speed*q.speed;
                    halfRollG=measureSeatForces(result.section.track,q.distance,q.speed,acceleration,TrainConfig{}.seatHeight).vertical;}
            }
            const Vec3 initialFlat=unit(Vec3{request.entry->jet.tangent.x,request.entry->jet.tangent.y,0});
            const Vec3 terminalFlat=unit(Vec3{result.exit.forward.x,result.exit.forward.y,0});
            const double headingChange=std::atan2(cross(initialFlat,terminalFlat).z,dot(initialFlat,terminalFlat))*180/pi;
            std::cout<<" exitHeadingChange="<<headingChange<<" duration="<<result.section.samples.back().time<<" sourceAbove4.1="<<duration<<" descentPitchExtrema="<<changes<<" sourceSeatHalfRollG="<<halfRollG<<" peakRollRadps="<<peakRoll<<" sourceSeatPeakGy="<<peakLateral;
        }
        for(const auto& e:result.section.report.errors)std::cout<<" error="<<e.code<<':'<<e.message;
        std::cout<<'\n';
    }
}

void camelbackScaleProbe(const char* path,const char* csvPath){
    Design saved;std::string error;if(!loadDesign(path,saved,error))throw std::runtime_error(error);
    double begin=INFINITY,end=0;
    for(const auto& section:saved.sections)if(section.role==RideRole::Camelback){begin=std::min(begin,section.start);end=std::max(end,section.end);}
    if(!std::isfinite(begin)||end<=begin)throw std::runtime_error("Saved design has no camelback");
    double top=-INFINITY,low=INFINITY,apex=0;
    for(double at=begin;at<=end;at+=.1){const double z=saved.track.sample(at).position.z;if(z>top){top=z;apex=at;}}
    for(double at=begin;at<=apex;at+=.1)low=std::min(low,saved.track.sample(at).position.z);
    const double rise=top-low,half=(saved.request.train.cars-1)*saved.request.train.spacing*.5;
    const auto& train=saved.request.train;const double rolling=gravity*train.rollingResistance,drag=.5*train.airDensity*train.dragCdA/(train.cars*train.carMass);
    std::ofstream csv(csvPath);csv<<std::setprecision(17)<<"case,time,baseDistance,physicalDistance,speed,frontGz,middleGz,rearGz,frontGy,middleGy,rearGy\n";
    struct State {double s,v,work;};
    struct Result {double peak{},minimum{INFINITY},gzLow{INFINITY},gzHigh{-INFINITY},gy{},rate{},energy{},duration{};};
    const std::array<const char*,6> names{"current","165m_250kmh_proxy","206p25m_312p5kmh_proxy","206p25m_airtime_cap_proxy","312p5kmh_airtime_cap_size_proxy","219p45m_300kmh_proxy"};
    const std::array<double,6> scales{1,165/rise,206.25/rise,206.25/rise,206.25/rise,219.45/rise};
    const std::array<double,6> targets{saved.simulation.metrics.maxSpeed,250/3.6,312.5/3.6,0,312.5/3.6,300/3.6};
    for(size_t scenario=0;scenario<names.size();++scenario){
        double scale=scales[scenario];Track track;
        auto rebuild=[&](){track=saved.track;track.closed=false;
            for(auto& k:track.knots){k.position=k.position*scale;k.curvature=k.curvature/scale;k.third=k.third/(scale*scale);k.fourth=k.fourth/(scale*scale*scale);
                k.upFirst=k.upFirst/scale;k.upSecond=k.upSecond/(scale*scale);k.upThird=k.upThird/(scale*scale*scale);}
            track.rebuild();};
        rebuild();
        auto motion=[&](double s,double v){double grade=0,curvature=0;
            for(int car=0;car<train.cars;++car){const auto q=track.sample(s+half-car*train.spacing);grade+=q.tangent.z/train.cars;curvature+=q.curvature.z/train.cars;}
            const double a=-gravity*grade-rolling-drag*v*v;
            return std::array<double,2>{a,-gravity*curvature*v-2*drag*v*a};};
        auto potential=[&](double s){double z=0;for(int car=0;car<train.cars;++car)z+=track.sample(s+half-car*train.spacing).position.z/train.cars;return gravity*z;};
        auto replay=[&](double initial,double dt,bool capture,bool write){
            State state{begin*scale,initial,0};const double energy=.5*initial*initial+potential(state.s);Result out;double time=0;size_t count=0;
            auto derivative=[&](State q){return State{q.v,motion(q.s,q.v)[0],(rolling+drag*q.v*q.v)*q.v};};
            auto added=[](State a,State b,double h){return State{a.s+b.s*h,a.v+b.v*h,a.work+b.work*h};};
            while(state.s<end*scale+half&&time<60){
                if(state.v<=1)throw std::runtime_error("Scaled camelback stalls");
                out.peak=std::max(out.peak,state.v);out.minimum=std::min(out.minimum,state.v);
                const auto a=derivative(state),b=derivative(added(state,a,dt*.5)),c=derivative(added(state,b,dt*.5)),d=derivative(added(state,c,dt));
                if(capture){const auto dynamics=motion(state.s,state.v);std::array<SeatDynamics,3> seats;
                    for(int seat=0;seat<3;++seat){const double at=state.s+seatDistanceOffset(train,seat);
                        seats[seat]=measureSeatDynamics(track,at,state.v,dynamics[0],dynamics[1],train.seatHeight);
                        if(at>=begin*scale&&at<=end*scale){const auto f=seats[seat].force;out.gzLow=std::min(out.gzLow,f.vertical);out.gzHigh=std::max(out.gzHigh,f.vertical);out.gy=std::max(out.gy,std::abs(f.lateral));out.rate=std::max(out.rate,std::abs(seats[seat].rate.vertical));}}
                    out.energy=std::max(out.energy,std::abs(.5*state.v*state.v+potential(state.s)+state.work-energy));
                    if(write&&count%8==0){csv<<names[scenario]<<','<<time<<','<<state.s/scale<<','<<state.s<<','<<state.v;
                        for(const auto& seat:seats)csv<<','<<seat.force.vertical;for(const auto& seat:seats)csv<<','<<seat.force.lateral;csv<<'\n';}
                }
                state.s+=dt*(a.s+2*b.s+2*c.s+d.s)/6;state.v+=dt*(a.v+2*b.v+2*c.v+d.v)/6;state.work+=dt*(a.work+2*b.work+2*c.work+d.work)/6;time+=dt;++count;
            }
            out.duration=time;return out;
        };
        double initial=angular_detail::replayAt(saved.simulation.frames,begin).speed*targets[scenario]/targets[0];
        if(scenario==4){double lo=206.25/rise,hi=260/rise;
            for(int i=0;i<14;++i){scale=(lo+hi)*.5;rebuild();
                for(int j=0;j<4;++j)initial+=targets[scenario]-replay(initial,1./480,false,false).peak;
                if(replay(initial,1./480,true,false).gzLow< -1.45)lo=scale;else hi=scale;}
            scale=hi;rebuild();for(int j=0;j<4;++j)initial+=targets[scenario]-replay(initial,1./480,false,false).peak;
        }else if(scenario==3){double lo=76,hi=87;
            for(int i=0;i<16;++i){const double mid=(lo+hi)*.5;if(replay(mid,1./480,true,false).gzLow< -1.45)hi=mid;else lo=mid;}initial=lo;
        }else for(int i=0;i<4;++i)initial+=targets[scenario]-replay(initial,1./480,false,false).peak;
        const auto result=replay(initial,1./960,true,true),fine=replay(initial,1./1920,true,false);
        std::cout<<"camelback-scale case="<<names[scenario]<<" scale="<<scale<<" entryLowRise="<<rise*scale<<" initialSpeed="<<initial<<" peakKmh="<<result.peak*3.6
            <<" minimumSpeed="<<result.minimum<<" Gz="<<result.gzLow<<','<<result.gzHigh<<" maxGy="<<result.gy<<" maxVerticalRate="<<result.rate<<" energyResidual="<<result.energy<<" duration="<<result.duration<<" fineMinGChange="<<fine.gzLow-result.gzLow<<" fineMaxGChange="<<fine.gzHigh-result.gzHigh<<'\n';
    }
    if(!csv)throw std::runtime_error("Cannot write scaled camelback diagnostic");
}

void loopSourceProbe(const char* path){
    Design saved;std::string error;if(!loadDesign(path,saved,error))throw std::runtime_error(error);
    const auto at=std::find_if(saved.forcePrograms.begin(),saved.forcePrograms.end(),[](const auto& p){return p.name=="loop";});
    if(at==saved.forcePrograms.end())throw std::runtime_error("Saved source has no loop");
    const auto original=designFvdSection(at->program);
    if(!original.assessment.passed)throw std::runtime_error("Saved loop source did not replay");
    for(double normal:{4.85})for(double yaw:{25.,33.})for(double rise:{135.,140.,145.})for(double crest:{2.}){
        FvdLoopRequest request;request.rollingAcceleration=at->program.rollingAcceleration;
        request.dragAccelerationCoefficient=at->program.dragAccelerationCoefficient;
        request.entry=makeFvdEntry(original.track.knots.front(),at->program.speed,request.rollingAcceleration,request.dragAccelerationCoefficient);
        request.height=rise*std::pow(at->program.speed/65,2);request.crestG=crest;request.normalG=normal;request.exitPositiveG=3.9;request.yawAngle=yaw*pi/180;request.ascentReleaseSeconds=1.2;
        const auto result=designFvdLoop(request);
        std::cout<<"production-loop normal="<<normal<<" rise="<<rise<<" crest="<<crest<<" yaw="<<yaw<<" entrySpeed="<<at->program.speed<<" sourcePassed="<<result.section.assessment.passed;
        if(result.section.assessment.passed){
            const auto check=validateSelfClearance(result.section.track,TrainConfig{});
            size_t conflicts=0;double gap=INFINITY;
            for(const auto& f:check.errors)if(f.code=="TRACK_CLEARANCE"){++conflicts;gap=std::min(gap,f.actual);}
            double high=0,above=0,run=0;
            for(size_t i=1;i<result.section.samples.size();++i){const auto& q=result.section.samples[i];const auto c=sampleFvdControl(result.authoring.controls,q.time);
                high=std::max(high,c.normalG);if(c.normalG>4.1)run+=q.time-result.section.samples[i-1].time;else run=0;above=std::max(above,run);}
            std::cout<<" conflicts="<<conflicts<<" minimumReportedGap="<<gap<<" normalPeak="<<high<<" sourceAbove4.1="<<above;
        }else for(const auto& f:result.section.report.errors)std::cout<<" error="<<f.code<<':'<<f.message;
        std::cout<<'\n';
    }
}

Vec3 placed(Vec3 p,const ForceAuthoring& source){return {p.x*std::cos(source.heading)-p.y*std::sin(source.heading),source.hand*(p.x*std::sin(source.heading)+p.y*std::cos(source.heading)),p.z};}
std::array<double,3> bodyAcceleration(const Track& track,double distance,double speed,double acceleration,double jerk){
    // Independent existing dynamics implementation; central time differences
    // below include basis motion without reusing the new jerk expression.
    const auto q=track.sample(distance);const auto dynamics=measureSeatDynamics(track,distance,speed,acceleration,jerk,0);
    return {dot(dynamics.angularAcceleration,q.tangent),dot(dynamics.angularAcceleration,q.right),dot(dynamics.angularAcceleration,q.up)};
}
void differenceProbe(const char* label,const Track& track,double distance,const ReplayMotion& motion){
    const auto measured=signedAngularMotion(sampleKinematics(track,distance),motion.speed,motion.acceleration,motion.jerk);
    std::cout<<"analytic "<<label;vec("velocity",measured.velocity);vec("acceleration",measured.acceleration);vec("jerk",measured.jerk);std::cout<<'\n';
    for(double h:{.001,.0002,.00004,.000008}){
        auto sample=[&](double time){const double s=distance+motion.speed*time+.5*motion.acceleration*time*time+motion.jerk*time*time*time/6;
            if(s<0||s>track.length)throw std::runtime_error("Finite-difference probe would leave the source domain");
            return bodyAcceleration(track,s,motion.speed+motion.acceleration*time+.5*motion.jerk*time*time,motion.acceleration+motion.jerk*time,motion.jerk);};
        try{const auto before=sample(-h),after=sample(h);std::array<double,3> numeric{},error{};
            for(int axis=0;axis<3;++axis){numeric[axis]=(after[axis]-before[axis])/(2*h);error[axis]=numeric[axis]-measured.jerk[axis];}
            std::cout<<"finite-difference "<<label<<" dt="<<h;vec("body-jerk",numeric);vec("error",error);std::cout<<'\n';
        }catch(const std::exception& e){std::cout<<"finite-difference "<<label<<" dt="<<h<<" unavailable="<<e.what()<<'\n';}
    }
}
struct Recompiled {Track track;Vec3 shift;double begin{},end{};};
Recompiled recompile(const Track& source,double begin,double end,double spacing,bool centered){
    Recompiled result;result.begin=begin;result.end=end;result.track.closed=false;result.track.authoredGeometry=result.track.authoredFrame=true;
    result.shift=centered?source.sample(begin).position:Vec3{};const size_t count=std::max<size_t>(3,size_t(std::ceil((end-begin)/spacing)));
    for(size_t i=0;i<=count;++i){const auto q=sampleKinematics(source,std::lerp(begin,end,double(i)/count));const auto& p=q.sample;
        result.track.knots.push_back({p.position-result.shift,p.tangent,p.curvature,p.up,0,p.element,q.curvatureS,q.curvatureSS,q.upS,q.upSS,q.upSSS});}
    result.track.rebuild();return result;
}
void densityProbe(const FvdResult& original,double at,double retainedBegin,double retainedEnd,const ReplayMotion& motion){
    const double begin=std::max(retainedBegin,at-6.137),end=std::min(retainedEnd,at+6.719);
    for(bool centered:{false,true})for(double spacing:{.6,.3,.15,.075}){
        const auto rebuilt=recompile(original.track,begin,end,spacing,centered);double positionError=0,curvatureSecondError=0,upThirdError=0;
        std::array<double,3> angularError{};
        for(int i=0;i<=400;++i){const double s=std::lerp(std::max(begin,at-1.07),std::min(end,at+1.13),i/400.);
            const double where=(s-begin)/(end-begin)*rebuilt.track.length;
            const auto a=sampleKinematics(original.track,s),b=sampleKinematics(rebuilt.track,where);
            positionError=std::max(positionError,norm(a.sample.position-b.sample.position-rebuilt.shift));
            curvatureSecondError=std::max(curvatureSecondError,norm(a.curvatureSS-b.curvatureSS));upThirdError=std::max(upThirdError,norm(a.upSSS-b.upSSS));
            const auto expected=signedAngularMotion(a,motion.speed,motion.acceleration,motion.jerk),actual=signedAngularMotion(b,motion.speed,motion.acceleration,motion.jerk);
            const std::array<std::array<double,3>,3> values{actual.velocity,actual.acceleration,actual.jerk},reference{expected.velocity,expected.acceleration,expected.jerk};
            for(int order=0;order<3;++order)for(int axis=0;axis<3;++axis)angularError[order]=std::max(angularError[order],std::abs(values[order][axis]-reference[order][axis]));
        }
        std::cout<<"resampling spacing="<<spacing<<" centered="<<centered<<" knots="<<rebuilt.track.knots.size()<<" position="<<positionError<<" curvatureSS="<<curvatureSecondError<<" upSSS="<<upThirdError;
        vec("angular-errors",angularError);std::cout<<'\n';
    }
}
}
int main(int argc,char** argv){try{
    std::cout<<std::setprecision(17);
    if(argc>1&&std::string(argv[1])=="--planar"){planarProbe();return 0;}
    if(argc==3&&std::string(argv[1])=="--loop-source"){loopSourceProbe(argv[2]);return 0;}
    if(argc==3&&std::string(argv[1])=="--camelback-profile"){camelbackProfile(argv[2]);return 0;}
    if(argc==3&&std::string(argv[1])=="--immelmann-source"){immelmannSourceProbe(argv[2]);return 0;}
    if(argc==4&&std::string(argv[1])=="--camelback-scale"){camelbackScaleProbe(argv[2],argv[3]);return 0;}
    GenerationRequest request;request.targets.requireIntensity=false;request.terrain.kind=TerrainKind::Highlands;request.maxCandidates=1;
    if(argc==4&&std::string(argv[1])=="--export-sources"){std::string error;if(!loadRecipe(argv[3],request.recipe,error))throw std::runtime_error(error);}
    const auto design=generate(request);
    std::cout<<"candidate completed="<<design.simulation.completed<<" accepted="<<design.accepted()<<" errors="<<design.report.errors.size()<<" length="<<design.track.length<<" sources="<<design.forcePrograms.size()<<'\n';
    if(argc>=3&&std::string(argv[1])=="--export-sources"){std::ofstream file(argv[2]);file<<authorshipPayload(design);if(!file)throw std::runtime_error("Cannot export source diagnostic");return 0;}
    if(design.simulation.frames.empty()||design.forcePrograms.empty())throw std::runtime_error("Candidate lacks retained source/replay data");
    size_t worst=0;double worstError=-1,worstAt=0;
    for(size_t index=0;index<design.forcePrograms.size();++index){const auto& source=design.forcePrograms[index];const auto rebuilt=designFvdSection(source.program);
        if(!rebuilt.assessment.passed){std::cout<<"source-failed "<<source.name<<'\n';continue;}
        for(unsigned subdivisions:{1,2,4,8}){
            const auto audit=assessFvdAngularAgreement(design.track,source,rebuilt,design.simulation.frames,{1e6,1e6,1e6},subdivisions);
            std::cout<<"source "<<source.name<<" subdivisions="<<subdivisions<<" samples="<<audit.samples;vec("errors",audit.maximumError);vec("locations",audit.distance);std::cout<<'\n';
            if(!audit.report.valid())for(const auto& error:audit.report.errors)std::cout<<"diagnostic-error "<<error.code<<' '<<error.message<<'\n';
            if(audit.maximumError[2]>worstError){worstError=audit.maximumError[2];worst=index;worstAt=audit.distance[2];}
        }
    }
    const auto& source=design.forcePrograms[worst];const auto original=designFvdSection(source.program);
    const auto finalAt=design.track.locate(worstAt);const size_t local=std::min(source.sourceDistances.size()-2,finalAt.span-source.firstKnot);
    const double first=design.track.spans[source.firstKnot+local].start,last=design.track.distanceAtSpan(source.firstKnot+local,1);
    const double fraction=(worstAt-first)/(last-first),sourceDistance=std::lerp(source.sourceDistances[local],source.sourceDistances[local+1],fraction);
    const auto sourceAt=original.track.locate(sourceDistance);const auto actual=sampleKinematics(design.track,worstAt),expected=sampleKinematics(original.track,sourceDistance);
    const auto motion=angular_detail::replayAt(design.simulation.frames,worstAt);
    std::cout<<"worst source="<<source.name<<" finalDistance="<<worstAt<<" sourceDistance="<<sourceDistance<<" retainedFraction="<<fraction<<" finalParameter="<<finalAt.parameter<<" sourceParameter="<<sourceAt.parameter<<" speed="<<motion.speed<<" acceleration="<<motion.acceleration<<" accelerationRate="<<motion.jerk<<'\n';
    std::cout<<"jets curvature="<<norm(actual.sample.curvature-placed(expected.sample.curvature,source))<<" curvatureS="<<norm(actual.curvatureS-placed(expected.curvatureS,source))<<" curvatureSS="<<norm(actual.curvatureSS-placed(expected.curvatureSS,source))<<" upS="<<norm(actual.upS-placed(expected.upS,source))<<" upSS="<<norm(actual.upSS-placed(expected.upSS,source))<<" upSSS="<<norm(actual.upSSS-placed(expected.upSSS,source))<<'\n';
    for(size_t endpoint=0;endpoint<2;++endpoint){
        const size_t at=source.firstKnot+endpoint;const auto& knot=design.track.knots[at];
        const auto q=sampleKinematics(original.track,source.sourceDistances[endpoint]);
        const Vec3 offset{source.origin.x,source.origin.y*source.hand,source.origin.z};
        std::cout<<"join endpoint="<<endpoint<<" position="<<norm(knot.position-offset-placed(q.sample.position,source))
            <<" tangent="<<norm(knot.tangent-placed(q.sample.tangent,source))
            <<" curvature="<<norm(knot.curvature-placed(q.sample.curvature,source))
            <<" third="<<norm(knot.third-placed(q.curvatureS,source))<<" fourth="<<norm(knot.fourth-placed(q.curvatureSS,source))
            <<" up="<<norm(knot.up-placed(q.sample.up,source))<<" upS="<<norm(knot.upFirst-placed(q.upS,source))
            <<" upSS="<<norm(knot.upSecond-placed(q.upSS,source))<<" upSSS="<<norm(knot.upThird-placed(q.upSSS,source))<<'\n';
    }
    differenceProbe("final",design.track,worstAt,motion);differenceProbe("source",original.track,sourceDistance,motion);
    densityProbe(original,sourceDistance,source.sourceDistances.front(),source.sourceDistances.back(),motion);
    double low=original.samples.front().time,high=original.samples.back().time;
    for(int i=0;i<50;++i){const double time=(low+high)*.5;if(angular_detail::sourceDistance(original,time)<sourceDistance)low=time;else high=time;}
    const double sourceTime=(low+high)*.5;const auto coarse=signedAngularMotion(expected,motion.speed,motion.acceleration,motion.jerk);
    for(double divisor:{2.,4.}){
        auto fineRequest=source.program;fineRequest.step/=divisor;fineRequest.maxSamples=50000;
        const auto fine=designFvdSection(fineRequest);if(!fine.assessment.passed){std::cout<<"source-refinement-failed divisor="<<divisor<<'\n';continue;}
        const double at=angular_detail::sourceDistance(fine,sourceTime);const auto q=sampleKinematics(fine.track,at);const auto dynamics=signedAngularMotion(q,motion.speed,motion.acceleration,motion.jerk);
        std::array<double,3> difference{};for(int axis=0;axis<3;++axis)difference[axis]=dynamics.jerk[axis]-coarse.jerk[axis];
        std::cout<<"source-step divisor="<<divisor<<" time="<<sourceTime<<" positionChange="<<norm(q.sample.position-expected.sample.position);vec("jerk",dynamics.jerk);vec("jerk-change",difference);std::cout<<'\n';
        differenceProbe(divisor==2?"source-half-step":"source-quarter-step",fine.track,at,motion);
    }
    return 0;
}catch(const std::exception& e){std::cerr<<"PROBE FAILED: "<<e.what()<<'\n';return 1;}}
