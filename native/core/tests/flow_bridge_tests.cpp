#include "../src/flow_bridge.hpp"
#include <iostream>
#include <limits>
using namespace coaster;
namespace {
int checks=0;
void check(bool passed,const char* message){++checks;if(!passed)throw std::runtime_error(message);}
Vec3 derivative(const std::array<Vec3,8>& c,int order,double u){Vec3 result{};for(int degree=7;degree>=order;--degree){double factor=1;for(int j=0;j<order;++j)factor*=degree-j;result=result*u+c[degree]*factor;}return result;}
void close(Vec3 actual,Vec3 expected){check(norm(actual-expected)<1e-8*std::max(1.,norm(expected)),"Septic endpoint jet or physical scaling changed");}
}
int main(){try{
    for(double span:{35.,160.,323.,640.})for(double bend:{-.004,0.,.007}){
        TrackKinematics a{},b{};a.sample.position={13,-19,4};b.sample.position={span+7,41,26};a.sample.tangent=unit(Vec3{1,.1,.02});b.sample.tangent=unit(Vec3{1,-.2,.07});a.sample.curvature={0,bend,.001};b.sample.curvature={.0001,-bend,.002};a.curvatureS={1e-6,-3e-6,2e-6};b.curvatureS={-2e-6,1e-6,-1e-6};
        auto c=detail::flowBridgePolynomial(a,b,span);
        for(int end=0;end<2;++end){const auto& port=end?b:a;std::array<Vec3,4> jets{port.sample.position,port.sample.tangent*span,port.sample.curvature*(span*span),port.curvatureS*(span*span*span)};for(int order=0;order<4;++order)close(derivative(c,order,double(end)),jets[order]);}
        for(double scale:{.2,3.,10.}){auto x=a,y=b;for(auto* port:{&x,&y}){port->sample.position=port->sample.position*scale;port->sample.curvature=port->sample.curvature/scale;port->curvatureS=port->curvatureS/(scale*scale);}auto scaled=detail::flowBridgePolynomial(x,y,span*scale);for(size_t i=0;i<c.size();++i)close(scaled[i],c[i]*scale);}
    }
    TrackKinematics port{};
    // Two curved, banked sides with a deliberately settled central segment.
    // The join must carry motion through that segment and survive the actual
    // canonical compile path, including unequal authored sample spacing.
    std::vector<AuthoredPoint> points;
    for(int i=0;i<=160;++i){double x=i*1.3+.001*i*i;
        double y=x<80?.000004*std::pow(80-x,3):x>125?.000004*std::pow(x-125,3):0;
        double bank=x<80?.4*smooth((80-x)/80):x>125?-.4*smooth((x-125)/110):0;
        points.push_back({{x,y,20},bank,Element::Turn,{0,0,1}});
    }
    auto original=points;
    detail::blendAuthoredJoin(points,32,126);const auto joined=compile(points,false);
    double change=0;for(size_t i=32;i<=126;++i)change=std::max(change,norm(points[i].position-original[i].position));
    std::cout<<"Joined transition maximum displacement: "<<change<<" m\n";
    check(norm(sampleSpanKinematics(joined,80,0).sample.curvature)>1e-7,"Join carries curvature through the previously straight middle");
    for(size_t i=1;i<joined.spans.size();++i){auto a=sampleSpanKinematics(joined,i-1,1),b=sampleSpanKinematics(joined,i,0);
        close(a.sample.position,b.sample.position);close(a.sample.tangent,b.sample.tangent);close(a.sample.curvature,b.sample.curvature);close(a.curvatureS,b.curvatureS);
        close(a.sample.up,b.sample.up);close(a.upS,b.upS);close(a.upSS,b.upSS);close(a.upSSS,b.upSSS);close(a.curvatureSS,b.curvatureSS);
    }
    for(size_t i:{size_t(32),size_t(126)}){close(points[i].position,original[i].position);check(std::abs(points[i].bank-original[i].bank)<1e-9,"Join preserves endpoint bank");}
    // Recompilation estimates new endpoint jets from the resampled curve.
    // Bound the resulting rider disturbance outside the edited interval;
    // exact polynomial endpoint matching alone does not establish this.
    auto rider=[](const Track& track,size_t span,double u){
        constexpr double speed=65,height=1.2,delta=.01;
        double s=track.distanceAtSpan(span,u);auto f=measureSeatForces(track,s,speed,0,height);
        double rate=speed*(measureSeatForces(track,s+delta,speed,0,height).vertical-measureSeatForces(track,s-delta,speed,0,height).vertical)/(2*delta);
        return std::array<double,4>{f.vertical,f.lateral,f.longitudinal,rate};
    };
    std::array<double,2> previousError{};
    for(int refinement:{1,2}){
        std::vector<AuthoredPoint> curved;
        for(int i=0;i<=160*refinement;++i){double t=double(i)/refinement,x=t*1.3+.001*t*t;
            double y=x<80?.00007*std::pow(80-x,3):x>125?.00007*std::pow(x-125,3):0;
            double dy=x<80?-.00021*std::pow(80-x,2):x>125?.00021*std::pow(x-125,2):0;
            double bank=x<80?.9*smooth((80-x)/80):x>125?-.9*smooth((x-125)/110):0;
            Vec3 up=rotate(Vec3{0,0,1},unit(Vec3{1,dy,0}),.3*std::sin(x/100));
            curved.push_back({{x,y,20},bank,Element::Turn,up});
        }
        const auto before=compile(curved,false);size_t first=32*refinement,last=126*refinement;
        detail::blendAuthoredJoin(curved,first,last);const auto after=compile(curved,false);
        std::array<double,2> error{};
        for(size_t begin:{first-5,last})for(size_t i=begin;i<begin+5;++i)for(double u:{0.,.25,.5,.75,1.}){
            auto a=rider(before,i,u),b=rider(after,i,u);
            for(int axis=0;axis<3;++axis)error[0]=std::max(error[0],std::abs(a[axis]-b[axis]));
            error[1]=std::max(error[1],std::abs(a[3]-b[3]));
        }
        check(error[0]<.01,"Resampled join perturbs outside rider forces by less than .01 g at 65 m/s");
        check(error[1]<.2,"Resampled join perturbs outside vertical force rate by less than .2 g/s at 65 m/s");
        if(refinement==2)for(size_t metric=0;metric<error.size();++metric)
            check(error[metric]<previousError[metric]*.75+1e-8,"Halving authored spacing reduces outside rider disturbance");
        previousError=error;
    }
    bool cancelled=false;try{detail::blendAuthoredJoin(points,32,126,[]{return true;});}catch(const std::runtime_error&){cancelled=true;}check(cancelled,"Join cancellation is retained");
    for(double span:{0.,-1.,std::numeric_limits<double>::infinity(),std::numeric_limits<double>::quiet_NaN()}){bool rejected=false;try{detail::flowBridgePolynomial(port,port,span);}catch(const std::invalid_argument&){rejected=true;}check(rejected,"Invalid physical span rejected explicitly");}
    port.curvatureS.x=std::numeric_limits<double>::quiet_NaN();bool rejected=false;try{detail::flowBridgePolynomial(port,port,100);}catch(const std::invalid_argument&){rejected=true;}check(rejected,"Nonfinite endpoint jet rejected explicitly");
    std::cout<<"PASS "<<checks<<" flow bridge endpoint/scaling checks\n";return 0;
}catch(const std::exception& e){std::cerr<<"FAIL "<<e.what()<<'\n';return 1;}}
