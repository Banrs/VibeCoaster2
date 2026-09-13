#include "coaster/coaster.hpp"
#include <iostream>
#include <stdexcept>
using namespace coaster;
static int checks;
static void check(bool ok,const char* what){++checks;if(!ok)throw std::runtime_error(what);}
static void close(Vec3 a,Vec3 b,double eps,const char* what){check(norm(a-b)<=eps,what);}
static Vec3 derivative(const Span& sp,double u,int order){Vec3 sum{};for(int k=7;k>=order;--k){double f=1;for(int j=0;j<order;++j)f*=k-j;sum=sum*u+sp.c[k]*f;}return sum;}
static double scalarArc(const Span& sp,double u){
    constexpr double x[]={.18343464249564980494,.52553240991632898582,.79666647741362673959,.96028985649753623168};
    constexpr double w[]={.36268378337836198297,.31370664587788728734,.22238103445337447054,.10122853629037625915};
    auto speed=[&](double at){Vec3 v=sp.c[7]*7;for(int k=6;k>=1;--k)v=v*at+sp.c[k]*k;return norm(v);};
    double sum=0;for(int i=0;i<4;++i)sum+=w[i]*(speed((1-x[i])*u*.5)+speed((1+x[i])*u*.5));return sum*u*.5;
}
static Vec3 geometricJ(const Span& sp,double u){
    Vec3 d=derivative(sp,u,1),dd=derivative(sp,u,2),ddd=derivative(sp,u,3);double q=norm(d);Vec3 t=d/q;
    const double qu=dot(t,dd);Vec3 tu=(dd-t*qu)/q;
    const double quu=dot(tu,dd)+dot(t,ddd);
    return ddd/(q*q*q)-dd*(3*qu/(q*q*q*q))-d*(quu/(q*q*q*q))+d*(3*qu*qu/(q*q*q*q*q));
}
static void verifyJoins(const Track& t){
    bool highOrder=false;
    for(size_t i=0;i<t.spans.size();++i){
        const auto& sp=t.spans[i];highOrder=highOrder||norm(sp.c[6])+norm(sp.c[7])>1e-13;
        for(double u:{0.,.013,.23,.5,.91,1.})check(t.distanceAtSpan(i,u)==sp.start+scalarArc(sp,u),"Paired quadrature is bit-exact with the scalar Gaussian rule");
        close(derivative(sp,0,0),t.knots[i].position,2e-10,"Septic start interpolation");close(derivative(sp,1,0),t.knots[i+1].position,2e-9,"Septic end interpolation");
        for(int k=0;k<=10;++k){double u=k/10.,distance=t.distanceAtSpan(i,u);if(t.closed&&i+1==t.spans.size()&&k==10)distance=0;
            auto at=t.locate(distance);auto a=t.sampleSpan(i,u),b=t.sample(distance);close(a.position,b.position,2e-9,"Distance/parameter roundtrip");close(a.up,b.up,2e-9,"Distance/frame parameter roundtrip");check(at.parameter>=0&&at.parameter<=1,"Inversion bounded parameter");}
        if(i%7==0){
            // Independent composite Simpson integration checks the Gaussian arc
            // coordinate rather than merely testing its own inverse against it.
            constexpr int cells=1024;
            for(double end:{.37,1.}){double sum=0;for(int k=0;k<=cells;++k){double weight=k==0||k==cells?1:k%2?4:2;sum+=weight*norm(derivative(sp,end*k/cells,1));}double oracle=sum*end/(3*cells);check(std::abs((t.distanceAtSpan(i,end)-sp.start)-oracle)<2e-9,"Gaussian arc agrees with independent composite integration");}
        }
        if(i+1==t.spans.size()&&!t.closed)continue;size_t j=(i+1)%t.spans.size();auto a=sampleSpanKinematics(t,i,1),b=sampleSpanKinematics(t,j,0);
        close(a.sample.position,b.sample.position,2e-9,"G0 join");close(a.sample.tangent,b.sample.tangent,2e-9,"G1 join");close(a.sample.curvature,b.sample.curvature,2e-8,"G2 join");close(geometricJ(sp,1),geometricJ(t.spans[j],0),2e-7,"G3 true arc join");
        close(a.curvatureS,geometricJ(sp,1),2e-8,"Analytic curvature derivative matches independent formula");
        close(a.sample.up,b.sample.up,2e-9,"Frame C0 join");close(a.upS,b.upS,2e-8,"Frame C1 true arc join");close(a.upSS,b.upSS,2e-7,"Frame C2 true arc join");
    }
    check(highOrder,"Nontrivial fixture exercises c6 and c7");
    auto copy=t;copy.rebuild();check(copy.spans.size()==t.spans.size()&&copy.length==t.length,"Rebuild length determinism");
    size_t hint=t.spans.size()+5;
    for(double s=-2;s<t.length+2;s+=.37){
        close(t.tangent(s,hint),t.sample(s).tangent,0,"Cached gravity tangent remains bit-exact across knots, endpoints and wrapped distances");
        const auto a=t.locate(s,hint),b=t.locate(s);
        check(a.span==b.span&&a.parameter==b.parameter,"Lookup hint does not alter canonical interpolation");
    }
    for(double s:{t.length*.7,0.,t.length*2.1,-t.length*.3,t.length*.2}){
        const auto a=t.locate(s,hint),b=t.locate(s);
        check(a.span==b.span&&a.parameter==b.parameter,"Arbitrary backward jumps and seam wraps invalidate stale hints");
    }
    if(t.closed)for(double s:{-t.length*2,-t.length,-.1,-0.,0.,std::nextafter(t.length,0.),t.length,std::nextafter(t.length,INFINITY),t.length*2}){
        double wrapped=std::fmod(s,t.length);if(wrapped<0)wrapped+=t.length;
        const auto a=t.locate(s),b=t.locate(wrapped);
        check(a.span==b.span&&a.parameter==b.parameter,"In-range fast path preserves exact wrapping at lap boundaries");
    }
    for(size_t i=0;i<t.spans.size();++i){for(int k=0;k<8;++k)close(t.spans[i].c[k],copy.spans[i].c[k],0,"Septic coefficient determinism");for(int k=0;k<6;++k){close(t.spans[i].referenceUp[k],copy.spans[i].referenceUp[k],0,"Reference frame cache determinism");check(t.spans[i].bank[k]==copy.spans[i].bank[k],"Bank cache determinism");}}
}
static void reportPrecision(){
    // A twelve-digit JSON report rounded the first value to1.02, changing a
    // valid just-below2% decision into a strict-boundary rejection in the runner.
    for(double coarse:{1.0199999999999,1.0200000000001}){
        Design d;d.simulation.metrics.minVerticalG=coarse;d.convergence.metrics.push_back({"minVerticalG",coarse,1.,std::abs(coarse-1.),.02});
        const std::string json=reportJson(d),key="\"name\":\"minVerticalG\",\"coarse\":";auto at=json.find(key);check(at!=std::string::npos,"Serialized convergence metric exists");
        double decoded=std::stod(json.substr(at+key.size()));check(decoded==coarse,"Convergence JSON preserves the exact double");
        check((std::abs(decoded-1.)<.02)==(std::abs(coarse-1.)<.02),"Serialization preserves strict threshold decision on both sides");
    }
}
int main(){try{reportPrecision();
    std::vector<AuthoredPoint> p;for(int i=0;i<=120;++i){double x=i+0.002*i*i;p.push_back({{x,4*std::sin(x/35),20+3*std::sin(x/43)},.2*std::sin(x/31),Element::Turn,{0,0,1}});}auto open=compile(p,false);verifyJoins(open);
    p.clear();constexpr int n=210;for(int i=0;i<=n;++i){double f=double(i)/n,angle=2*pi*(f+.06*std::sin(2*pi*f));p.push_back({{50*std::sin(angle),50*(1-std::cos(angle)),20+2*std::sin(2*angle)},.12*std::sin(angle),Element::Turn,{0,0,1}});}p.back()=p.front();auto closed=compile(p,true);verifyJoins(closed);
    p.clear();for(int i=0;i<5;++i)p.push_back({{double(i*i),0,20},0,Element::Launch,{0,0,1}});auto straight=compile(p,false);check(std::abs(straight.length-16)<1e-12,"Analytic straight arc length");
    for(double s:{0.,.1,3.4,12.3,16.})close(straight.sample(s).position,{s,0,20},2e-11,"Analytic straight distance");
    for(auto bad:{NAN,INFINITY,-INFINITY}){bool rejected=false;try{straight.locate(bad);}catch(...){rejected=true;}check(rejected,"Nonfinite sample rejected");}
    auto broken=closed;broken.knots.back().bank+=.01;bool rejected=false;try{broken.rebuild();}catch(...){rejected=true;}check(rejected,"Mismatched closed canonical jets rejected");
    broken=closed;broken.knots.back().bank+=1e-12;rejected=false;try{broken.rebuild();}catch(...){rejected=true;}check(rejected,"Even small authored seam discontinuity is not hidden by tolerance");
    std::cout<<"PASS "<<checks<<" G3 geometry/cache/arc/C2 join checks\n";return 0;
}catch(const std::exception& e){std::cerr<<"FAIL after "<<checks<<": "<<e.what()<<'\n';return 1;}}
