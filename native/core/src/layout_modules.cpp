#include "coaster/layout_modules.hpp"
#include "coaster/fvd.hpp"
#include <stdexcept>

namespace coaster {
namespace {
constexpr std::array<double,4> gaussX{.18343464249564980494,.52553240991632898582,.79666647741362673959,.96028985649753623168};
constexpr std::array<double,4> gaussW{.36268378337836198297,.31370664587788728734,.22238103445337447054,.10122853629037625915};
double smoothDerivative(double u){return 30*u*u*(1-u)*(1-u);}
double warped(double u,double shape){return u+shape*std::sin(2*pi*u)/(2*pi);}
double progress(double u,double shape){return smooth(warped(u,shape));}
double progressDerivative(double u,double shape){return smoothDerivative(warped(u,shape))*(1+shape*std::cos(2*pi*u));}
Vec3 pitchIntegral(double a,double b,double shape){
    Vec3 value{};double mid=(a+b)*.5,half=(b-a)*.5;
    for(size_t i=0;i<gaussX.size();++i)for(double sign:{-1.,1.}){
        double theta=pi*progress(mid+sign*half*gaussX[i],shape);
        value=value+Vec3{std::cos(theta),0,std::sin(theta)}*(gaussW[i]*half);
    }
    return value;
}
bool validPose(const LayoutModulePose& p){
    return finite(p.position)&&finite(p.forward)&&finite(p.up)&&norm(p.position)<=100000&&
        std::abs(norm(p.forward)-1)<1e-9&&std::abs(norm(p.up)-1)<1e-9&&std::abs(dot(p.forward,p.up))<1e-9;
}
}
ReversingModule buildReversingModule(const ReversingModuleRequest& request,Cancel cancel){
    ReversingModule result;result.kind=request.kind;result.entry=request.entry;
    result.height=request.height;result.rollLength=request.rollLength;result.track.closed=false;
    auto fail=[&](const char* code,const char* message){result.report.fail(code,message);};
    if(!validPose(request.entry)||!std::isfinite(request.height)||!std::isfinite(request.rollLength)||
       !std::isfinite(request.portLength)||!std::isfinite(request.sampleSpacing)||
       !std::isfinite(request.pitchShape)||!std::isfinite(request.rollShape)||
       !std::isfinite(request.rollOverlap)||request.rollOverlap<0||request.rollOverlap>.4||
       std::abs(request.pitchShape)>.65||std::abs(request.rollShape)>.65||
       request.height<20||request.height>250||request.rollLength<20||request.rollLength>1000||
       request.sampleSpacing<.25||request.sampleSpacing>2||request.portLength<4*request.sampleSpacing||request.portLength>100||
       (request.rollDirection!=1&&request.rollDirection!=-1)||
       (request.kind!=ReversingModuleKind::Immelmann&&request.kind!=ReversingModuleKind::DiveLoop)){
        fail("LAYOUT_MODULE_INPUT","Unsupported reversing-module dimensions, pose or handedness");return result;
    }
    if(request.maxSamples<12||request.maxSamples>50000){fail("LAYOUT_MODULE_BUDGET","Unsupported module sample budget");return result;}
    auto cancelled=[&]{if(cancel&&cancel()){result.cancelled=true;fail("CANCELLED","Reversing-module authoring cancelled");return true;}return false;};
    if(cancelled())return result;
    // Composite integration establishes the fixed dimension independently of
    // the chosen sample spacing. The integrand is smooth and symmetric.
    double normalizedHeight=0;for(int i=0;i<64;++i)normalizedHeight+=pitchIntegral(i/64.,(i+1)/64.,request.pitchShape).z;
    result.pitchLength=request.height/normalizedHeight;
    const double overlap=result.pitchLength*request.rollOverlap;
    auto rollAngle=[&](double distance){return request.rollDirection*pi*progress(std::clamp(distance/(request.rollLength+overlap),0.,1.),request.rollShape);};
    int pitchSteps=int(std::ceil(result.pitchLength/request.sampleSpacing));pitchSteps+=pitchSteps%2;
    const int rollSteps=int(std::ceil(request.rollLength/request.sampleSpacing));
    const int portSteps=int(std::ceil(request.portLength/request.sampleSpacing));
    const size_t sampleCount=size_t(pitchSteps)+size_t(rollSteps)+2*size_t(portSteps)+1;
    if(sampleCount>request.maxSamples){fail("LAYOUT_MODULE_BUDGET","Reversing module exceeds its sample budget");return result;}
    result.points.reserve(sampleCount);result.track.knots.reserve(sampleCount);
    const Vec3 forward=request.entry.forward,up=request.entry.up,origin=request.entry.position;
    auto append=[&](Vec3 p,Vec3 t,Vec3 k,Vec3 u,Element element){
        result.points.push_back({p,0,element,u});result.track.knots.push_back({p,t,k,u,0,element});
    };
    auto straight=[&](Vec3 start,Vec3 direction,Vec3 startUp,double length,int steps,bool rolling){
        // The starting point belongs to the preceding piece when already set.
        int first=result.points.empty()?0:1;
        for(int i=first;i<=steps;++i){
            if((i&63)==0&&cancelled())return false;
            double u=double(i)/steps;
            double angle=rollAngle(length*u+(request.kind==ReversingModuleKind::Immelmann?overlap:0));
            Vec3 pointUp=rolling?rotate(startUp,direction,angle):startUp;
            append(start+direction*(length*u),direction,{},pointUp,rolling?Element::Inversion:Element::Return);
        }
        return true;
    };
    auto pitch=[&](Vec3 start,double sign){
        Vec3 integral{};
        for(int i=1;i<=pitchSteps;++i){
            if((i&63)==0&&cancelled())return false;
            double u=double(i)/pitchSteps,theta=pi*progress(u,request.pitchShape);
            integral=integral+pitchIntegral(double(i-1)/pitchSteps,u,request.pitchShape)*result.pitchLength;
            // Enforce exact symmetric endpoint dimensions, correcting only
            // roundoff of the quadrature sums, never an endpoint gap.
            if(i==pitchSteps)integral={0,0,request.height};
            Vec3 tangent=forward*std::cos(theta)+up*(sign*std::sin(theta));
            Vec3 pitchUp=forward*(-std::sin(theta))+up*(sign*std::cos(theta));
            if(i==pitchSteps){tangent=forward*(-1);pitchUp=up*(-sign);}
            Vec3 curvature=pitchUp*(pi*progressDerivative(u,request.pitchShape)/result.pitchLength);
            double angle=sign>0?rollAngle(u*result.pitchLength-(result.pitchLength-overlap)):
                rollAngle(request.rollLength+u*result.pitchLength)-request.rollDirection*pi;
            append(start+forward*integral.x+up*(sign*integral.z),tangent,curvature,rotate(pitchUp,tangent,angle),Element::Inversion);
        }
        return true;
    };
    if(!straight(origin,forward,up,request.portLength,portSteps,false))return result;
    if(request.kind==ReversingModuleKind::Immelmann){
        result.pitchBeginIndex=result.points.size()-1;
        if(!pitch(result.points.back().position,1))return result;
        result.pitchEndIndex=result.points.size()-1;result.rollBeginIndex=result.pitchEndIndex-size_t(std::ceil(pitchSteps*request.rollOverlap));
        if(!straight(result.points.back().position,forward*(-1),up*(-1),request.rollLength,rollSteps,true))return result;
        result.rollEndIndex=result.points.size()-1;
    }else{
        result.rollBeginIndex=result.points.size()-1;
        if(!straight(result.points.back().position,forward,up,request.rollLength,rollSteps,true))return result;
        result.rollEndIndex=result.points.size()-1;result.pitchBeginIndex=result.rollEndIndex;
        if(!pitch(result.points.back().position,-1))return result;
        result.pitchEndIndex=result.points.size()-1;
        result.rollEndIndex=result.pitchBeginIndex+size_t(std::ceil(pitchSteps*request.rollOverlap));
    }
    if(!straight(result.points.back().position,forward*(-1),up,request.portLength,portSteps,false))return result;
    result.exit={result.points.back().position,forward*(-1),up};result.geometryBuilt=true;
    if(cancelled())return result;
    try{
        result.track.rebuild();result.canonicalBuilt=true;
        for(size_t span=0;span<result.track.spans.size();++span){
            if((span&63)==0&&cancelled()){result.canonicalBuilt=false;return result;}
            for(double u:{0.,.5,1.}){
                const auto k=sampleSpanKinematics(result.track,span,u);
                result.sampledMaxCurvature=std::max(result.sampledMaxCurvature,norm(k.sample.curvature));
                result.sampledMaxFrameTwistPerMeter=std::max(result.sampledMaxFrameTwistPerMeter,std::abs(dot(k.upS,k.sample.right)));
            }
        }
    }
    catch(const std::exception& e){result.report.fail("LAYOUT_MODULE_CANONICAL",e.what());}
    if(cancelled())result.canonicalBuilt=false;
    return result;
}
ReversingModule buildEnergyReversingModule(const EnergyReversingModuleRequest& input,Cancel cancel){
    const auto& r=input.geometry;ReversingModule result;result.kind=r.kind;result.entry=r.entry;result.track.closed=false;
    auto fail=[&](const char* code,const char* message){result.report.fail(code,message);};
    if(!validPose(r.entry)||std::abs(r.entry.forward.z)>1e-9||norm(r.entry.up-Vec3{0,0,1})>1e-9||
        r.pitchShape!=0||!std::isfinite(r.rollLength)||r.rollLength<20||r.rollLength>1000||
        !std::isfinite(r.rollShape)||std::abs(r.rollShape)>.65||!std::isfinite(r.rollOverlap)||r.rollOverlap<0||r.rollOverlap>.4||
        !std::isfinite(r.sampleSpacing)||r.sampleSpacing<.25||r.sampleSpacing>2||
        !std::isfinite(r.portLength)||r.portLength<4*r.sampleSpacing||r.portLength>100||
        (r.rollDirection!=1&&r.rollDirection!=-1)||(r.kind!=ReversingModuleKind::Immelmann&&r.kind!=ReversingModuleKind::DiveLoop)){
        fail("LAYOUT_MODULE_INPUT","Unsupported energy-authored module pose, roll or sampling");return result;}
    if(r.maxSamples<12||r.maxSamples>50000){fail("LAYOUT_MODULE_BUDGET","Unsupported module sample budget");return result;}
    auto cancelled=[&]{if(cancel&&cancel()){result.cancelled=true;fail("CANCELLED","Energy-authored module cancelled");return true;}return false;};
    if(cancelled())return result;
    auto pitch=designFvdPitch({r.height,input.apexSpeed,input.normalG,input.pushRampSeconds},cancel);
    result.report=pitch.section.report;result.cancelled=pitch.section.cancelled;
    if(!result.report.valid()||!pitch.section.assessment.passed)return result;
    result.pitchLength=pitch.section.track.length;result.height=pitch.height;result.rollLength=r.rollLength;
    result.pitchForwardDisplacement=pitch.forwardDisplacement;result.idealApexSpeed=input.apexSpeed;
    result.idealEntrySpeed=pitch.authoring.speed;
    const double overlap=result.pitchLength*r.rollOverlap;
    const int pitchSteps=int(std::ceil(result.pitchLength/r.sampleSpacing)),rollSteps=int(std::ceil(r.rollLength/r.sampleSpacing)),portSteps=int(std::ceil(r.portLength/r.sampleSpacing));
    if(size_t(pitchSteps)+rollSteps+2*portSteps+1>r.maxSamples){fail("LAYOUT_MODULE_BUDGET","Energy-authored module exceeds its sample budget");return result;}
    std::vector<AuthoredPoint> local;
    auto append=[&](Vec3 p,Vec3 up,Element e){local.push_back({p,0,e,up});};
    for(int i=0;i<=portSteps;++i)append({r.portLength*i/portSteps,0,0},{0,0,1},Element::Return);
    result.pitchBeginIndex=local.size()-1;Vec3 start=local.back().position;
    for(int i=1;i<=pitchSteps;++i){if((i&63)==0&&cancelled())return result;
        double s=result.pitchLength*i/pitchSteps;auto p=pitch.section.track.sample(s);
        double angle=r.rollDirection*pi*progress(std::clamp((s-result.pitchLength+overlap)/(r.rollLength+overlap),0.,1.),r.rollShape);
        append(start+p.position,rotate(p.up,p.tangent,angle),Element::Inversion);}
    result.pitchEndIndex=local.size()-1;
    result.rollBeginIndex=result.pitchBeginIndex+size_t(std::floor(pitchSteps*(1-r.rollOverlap)));
    auto top=pitch.section.track.sample(result.pitchLength);start=local.back().position;
    for(int i=1;i<=rollSteps;++i){if((i&63)==0&&cancelled())return result;
        double s=r.rollLength*i/rollSteps,angle=r.rollDirection*pi*progress((overlap+s)/(r.rollLength+overlap),r.rollShape);
        append(start+top.tangent*s,rotate(top.up,top.tangent,angle),Element::Inversion);}
    result.rollEndIndex=local.size()-1;start=local.back().position;Vec3 endUp=local.back().upHint;
    for(int i=1;i<=portSteps;++i)append(start+top.tangent*(r.portLength*i/portSteps),endUp,Element::Return);
    if(r.kind==ReversingModuleKind::DiveLoop){Vec3 end=local.back().position;std::reverse(local.begin(),local.end());for(auto& p:local)p.position=p.position-end;
        const size_t last=local.size()-1;size_t a=result.pitchBeginIndex,b=result.rollBeginIndex;
        result.pitchBeginIndex=last-result.pitchEndIndex;result.pitchEndIndex=last-a;
        result.rollBeginIndex=last-result.rollEndIndex;result.rollEndIndex=last-b;
        result.idealEntrySpeed=input.apexSpeed;
    }
    const Vec3 right=cross(r.entry.forward,r.entry.up);
    auto transform=[&](Vec3 p){return r.entry.forward*p.x+right*(-p.y)+r.entry.up*p.z;};
    result.points.reserve(local.size());for(auto p:local){p.position=r.entry.position+transform(p.position);p.upHint=transform(p.upHint);result.points.push_back(p);}
    result.geometryBuilt=true;if(cancelled())return result;
    try{result.track=compile(result.points,false);result.canonicalBuilt=true;
        auto exit=result.track.sample(result.track.length);result.exit={exit.position,exit.tangent,exit.up};
        for(size_t span=0;span<result.track.spans.size();++span){if((span&63)==0&&cancelled())return result;
            for(double u:{0.,.5,1.}){auto k=sampleSpanKinematics(result.track,span,u);
                result.sampledMaxCurvature=std::max(result.sampledMaxCurvature,norm(k.sample.curvature));
                result.sampledMaxFrameTwistPerMeter=std::max(result.sampledMaxFrameTwistPerMeter,std::abs(dot(k.upS,k.sample.right)));}}
    }catch(const std::exception& e){fail("LAYOUT_MODULE_CANONICAL",e.what());}
    return result;
}

EnergyLoopModule buildEnergyLoopModule(const EnergyLoopModuleRequest& r,Cancel cancel){
    EnergyLoopModule result;result.entry=r.entry;result.track.closed=false;
    auto fail=[&](const char* code,const char* message){result.report.fail(code,message);};
    if(!validPose(r.entry)||std::abs(r.entry.forward.z)>1e-9||norm(r.entry.up-Vec3{0,0,1})>1e-9||
        !std::isfinite(r.lateralOffset)||std::abs(r.lateralOffset)>100||!std::isfinite(r.apexNormalG)||r.apexNormalG<0||r.apexNormalG>1.5||
        !std::isfinite(r.sampleSpacing)||r.sampleSpacing<.25||r.sampleSpacing>2||
        !std::isfinite(r.portLength)||r.portLength<4*r.sampleSpacing||r.portLength>100){
        fail("LAYOUT_MODULE_INPUT","Unsupported energy-authored full-loop pose, offset or sampling");return result;}
    if(r.maxSamples<12||r.maxSamples>50000){fail("LAYOUT_MODULE_BUDGET","Unsupported loop sample budget");return result;}
    auto cancelled=[&]{if(cancel&&cancel()){result.cancelled=true;fail("CANCELLED","Energy-authored full loop cancelled");return true;}return false;};
    if(cancelled())return result;
    auto pitch=designFvdPitch({r.height,r.apexSpeed,r.normalG,r.pushRampSeconds,r.apexNormalG},cancel);
    result.report=pitch.section.report;result.cancelled=pitch.section.cancelled;
    if(!result.report.valid()||!pitch.section.assessment.passed)return result;
    result.height=pitch.height;result.pitchForwardDisplacement=pitch.forwardDisplacement;
    result.idealEntrySpeed=pitch.authoring.speed;result.idealApexSpeed=r.apexSpeed;
    const double L=pitch.section.track.length,D=pitch.forwardDisplacement;
    const int steps=int(std::ceil(L/r.sampleSpacing)),guards=int(std::ceil(r.portLength/r.sampleSpacing));
    if(2*size_t(steps)+2*guards+1>r.maxSamples){fail("LAYOUT_MODULE_BUDGET","Energy-authored full loop exceeds sample budget");return result;}
    const Vec3 right=cross(r.entry.forward,r.entry.up);
    auto transform=[&](Vec3 p){return r.entry.forward*p.x-right*p.y+r.entry.up*p.z;};
    auto append=[&](Vec3 p,Vec3 up,Element e){result.points.push_back({r.entry.position+transform(p),0,e,transform(up)});};
    for(int i=0;i<=guards;++i)append({r.portLength*i/guards,0,0},{0,0,1},Element::Return);
    for(int i=1;i<=2*steps;++i){if((i&63)==0&&cancelled())return result;
        double s=L*i/steps,u=double(i)/(2*steps);bool descending=i>steps;
        auto q=pitch.section.track.sample(descending?2*L-s:s);
        Vec3 p=descending?Vec3{2*D-q.position.x,q.position.y,q.position.z}:q.position;
        Vec3 up=descending?Vec3{-q.up.x,q.up.y,q.up.z}:q.up;
        p.x+=r.portLength;p.y+=r.lateralOffset*u*u*u*u*(35+u*(-84+u*(70-20*u)));
        append(p,up,Element::Inversion);
    }
    for(int i=1;i<=guards;++i)append({r.portLength+2*D+r.portLength*i/guards,r.lateralOffset,0},{0,0,1},Element::Return);
    result.geometryBuilt=true;if(cancelled())return result;
    try{result.track=compile(result.points,false);result.canonicalBuilt=true;auto end=result.track.sample(result.track.length);result.exit={end.position,end.tangent,end.up};}
    catch(const std::exception& e){fail("LAYOUT_MODULE_CANONICAL",e.what());}
    return result;
}

}
