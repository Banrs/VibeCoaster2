#include "../src/connector_profile.hpp"
#include <iostream>
using namespace coaster;
namespace {int checks=0;void check(bool good,const char* why){++checks;if(!good)throw std::runtime_error(why);}void near(double a,double b,double tolerance){check(std::abs(a-b)<tolerance,"Physical connector identity or analytic derivative failed");}}
int main(){try{
    for(double length:{200.,380.,420.,630.,850.})for(double speed:{35.,46.,55.,65.,80.}){
        const double h=detail::connectorCrestHeight(length,speed),apexSpeed2=speed*speed-2*gravity*h;
        check(h>0&&h<=48&&apexSpeed2>0,"Crest retains a positive gravity-only energy budget");
        const double delta=4*pi*pi*h*apexSpeed2/(gravity*length*length),maximumDelta=pi*pi*std::pow(speed,4)/(2*gravity*gravity*length*length);
        if(h<48)near(delta,std::min(.85,maximumDelta*.8),1e-11);
        // For uncapped shapes, length and gravitational energy scale together.
        const double smaller=detail::connectorCrestHeight(length*.25,speed*.5);if(h<48)near(smaller,h*.25,1e-11);
    }
    for(double shape:{-.04,0.,.04}){
        for(double end:{0.,1.})for(double jet:detail::connectorProfileJet(end,shape))near(jet,0,1e-15);
        near(detail::connectorProfileJet(.5,shape)[0],1,1e-14);
        for(double u=.02;u<.99;u+=.02){const auto j=detail::connectorProfileJet(u,shape),a=detail::connectorProfileJet(u-1e-6,shape),b=detail::connectorProfileJet(u+1e-6,shape);for(int derivative=1;derivative<=3;++derivative)near((b[derivative-1]-a[derivative-1])/2e-6,j[derivative],1e-5);check(j[0]>=0&&j[0]<=1,"Connector remains within its authored height envelope");}
    }
    for(double bad:{0.,-1.,double(INFINITY)}){bool failed=false;try{detail::connectorCrestHeight(bad,50);}catch(const std::invalid_argument&){failed=true;}check(failed,"Invalid geometry rejected explicitly");}
    std::cout<<"PASS "<<checks<<" connector energy, curvature, physical scaling and C3 port checks\n";return 0;
}catch(const std::exception& e){std::cerr<<"FAIL "<<e.what()<<'\n';return 1;}}
