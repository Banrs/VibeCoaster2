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
    for(double span:{0.,-1.,std::numeric_limits<double>::infinity(),std::numeric_limits<double>::quiet_NaN()}){bool rejected=false;try{detail::flowBridgePolynomial(port,port,span);}catch(const std::invalid_argument&){rejected=true;}check(rejected,"Invalid physical span rejected explicitly");}
    port.curvatureS.x=std::numeric_limits<double>::quiet_NaN();bool rejected=false;try{detail::flowBridgePolynomial(port,port,100);}catch(const std::invalid_argument&){rejected=true;}check(rejected,"Nonfinite endpoint jet rejected explicitly");
    std::cout<<"PASS "<<checks<<" flow bridge endpoint/scaling checks\n";return 0;
}catch(const std::exception& e){std::cerr<<"FAIL "<<e.what()<<'\n';return 1;}}
