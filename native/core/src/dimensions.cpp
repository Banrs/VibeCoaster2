#include "coaster/coaster.hpp"
#include <stdexcept>
namespace coaster {
namespace {
using Polynomial=std::vector<double>;
double value(const Polynomial& p,double x){double y=0;for(auto it=p.rbegin();it!=p.rend();++it)y=y*x+*it;return y;}
std::vector<double> roots(Polynomial p){
    double scale=0;for(double c:p)scale=std::max(scale,std::abs(c));if(scale==0)return {};
    for(double& c:p)c/=scale;
    while(p.size()>1&&std::abs(p.back())<1e-15)p.pop_back();
    if(p.size()<2)return {};
    if(p.size()==2){double r=-p[0]/p[1];return r>=0&&r<=1?std::vector<double>{r}:std::vector<double>{};}
    Polynomial derivative;for(size_t i=1;i<p.size();++i)derivative.push_back(p[i]*i);
    auto partitions=roots(derivative);partitions.push_back(0);partitions.push_back(1);std::sort(partitions.begin(),partitions.end());
    std::vector<double> found;
    for(double x:partitions)if(std::abs(value(p,x))<1e-13)found.push_back(x);
    for(size_t i=1;i<partitions.size();++i){
        double a=partitions[i-1],b=partitions[i],fa=value(p,a),fb=value(p,b);
        if((fa<0)==(fb<0)||fa==0||fb==0)continue;
        for(int k=0;k<60;++k){double m=(a+b)*.5,fm=value(p,m);if((fa<0)==(fm<0)){a=m;fa=fm;}else b=m;}
        found.push_back((a+b)*.5);
    }
    std::sort(found.begin(),found.end());found.erase(std::unique(found.begin(),found.end(),[](double a,double b){return std::abs(a-b)<1e-12;}),found.end());return found;
}
}
template<size_t N>static std::pair<double,double> bounds(const std::array<double,N>& coefficients){
    Polynomial p(coefficients.begin(),coefficients.end()),derivative;
    for(double c:p)if(!std::isfinite(c))throw std::runtime_error("Nonfinite dimension polynomial");
    for(size_t i=1;i<p.size();++i)derivative.push_back(p[i]*i);
    double lo=std::min(value(p,0),value(p,1)),hi=std::max(value(p,0),value(p,1));
    for(double r:roots(derivative)){double v=value(p,r);lo=std::min(lo,v);hi=std::max(hi,v);}
    return {lo,hi};
}
std::pair<double,double> polynomialBounds(const std::array<double,8>& coefficients){return bounds(coefficients);}
std::vector<InversionDimensions> measureInversionDimensions(const Track& track,Cancel cancel){
    if(cancel&&cancel())throw std::runtime_error("CANCELLED");
    size_t n=track.spans.size();
    if(!n||n>200000||track.knots.size()!=n+1||!std::isfinite(track.length)||track.length<=0||track.length>150000)throw std::runtime_error("Invalid canonical dimension input");
    double end=0;for(size_t i=0;i<n;++i){if((i&127)==0&&cancel&&cancel())throw std::runtime_error("CANCELLED");const auto& sp=track.spans[i];
        if(!std::isfinite(sp.start)||!std::isfinite(sp.length)||sp.length<=0||std::abs(sp.start-end)>1e-6||int(track.knots[i].element)<0||int(track.knots[i].element)>7)throw std::runtime_error("Invalid canonical dimension span");
        for(auto c:sp.c)if(!finite(c))throw std::runtime_error("Nonfinite canonical dimension span");end=sp.start+sp.length;
    }
    if(std::abs(end-track.length)>1e-6)throw std::runtime_error("Invalid canonical dimension length");
    size_t start=0;if(track.closed)for(size_t i=0;i<n;++i)if(track.knots[i].element!=Element::Inversion){start=(i+1)%n;break;}
    std::vector<InversionDimensions> result;bool active=false;std::array<double,3> low{},high{};size_t lastIndex=0;
    auto finish=[&](){auto& d=result.back();d.verticalMinimum=low[2];d.verticalMaximum=high[2];d.verticalExtent=high[2]-low[2];d.forwardExtent=high[0]-low[0];d.lateralExtent=high[1]-low[1];};
    for(size_t k=0;k<n;++k){if((k&127)==0&&cancel&&cancel())throw std::runtime_error("CANCELLED");size_t i=(start+k)%n;const auto& sp=track.spans[i];
        if(track.knots[i].element!=Element::Inversion){if(active)finish();active=false;continue;}
        if(!active){InversionDimensions d;d.startDistance=sp.start;d.horizontalForward=unit(Vec3{sp.c[1].x,sp.c[1].y,0});d.horizontalAxisFallback=norm(d.horizontalForward)<.5;if(d.horizontalAxisFallback)d.horizontalForward={1,0,0};d.horizontalRight=cross(d.horizontalForward,{0,0,1});result.push_back(d);low.fill(INFINITY);high.fill(-INFINITY);active=true;}
        auto& d=result.back();if(d.pathLength>0&&i<lastIndex)d.wrapsSeam=true;lastIndex=i;d.pathLength+=sp.length;d.endDistance=sp.start+sp.length;
        std::array<Vec3,3> axes{d.horizontalForward,d.horizontalRight,Vec3{0,0,1}};
        for(size_t a=0;a<3;++a){std::array<double,10> c;for(size_t j=0;j<c.size();++j)c[j]=dot(sp.c[j],axes[a]);auto [lo,hi]=bounds(c);low[a]=std::min(low[a],lo);high[a]=std::max(high[a],hi);}
    }
    if(active)finish();return result;
}
}
