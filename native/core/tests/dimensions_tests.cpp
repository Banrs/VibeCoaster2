#include "coaster/coaster.hpp"
#include "../src/acceptance_internal.hpp"
#include <iostream>
#include <random>
#include <stdexcept>
using namespace coaster;
int checks=0;void require(bool value,const char* message){++checks;if(!value)throw std::runtime_error(message);}
Track fixture(){Track t;t.closed=false;t.length=3;t.knots.resize(4);t.spans.resize(3);for(int i=0;i<3;++i){t.spans[i].start=i;t.spans[i].length=1;t.spans[i].c[0]={double(i),0,100};t.spans[i].c[1]={1,0,0};t.knots[i].element=Element::Inversion;}t.spans[1].c[1].z=4;t.spans[1].c[2].z=-4;return t;}
int main(int argc,char**argv){try{
    auto [lo,hi]=polynomialBounds({100,4,-4,0,0,0});require(std::abs(lo-100)<1e-12&&std::abs(hi-101)<1e-12,"Interior quadratic maximum missed");
    auto [a,b]=polynomialBounds({0,1,-6,8,0,0});require(a<0&&b==3,"Multiple cubic extrema missed");
    auto constant=polynomialBounds({32,0,0,0,0,0});require(constant.first==32&&constant.second==32,"Constant bounds wrong");
    auto repeated=polynomialBounds({.0625,-.5,1.5,-2,1,0});require(std::abs(repeated.first)<1e-12&&std::abs(repeated.second-.0625)<1e-12,"Repeated stationary root missed");
    // T7(2u-1) has six interior stationary points and exact range [-1,1].
    auto chebyshev=polynomialBounds({-1,98,-1568,9408,-26880,39424,-28672,8192});
    require(std::abs(chebyshev.first+1)<1e-8&&std::abs(chebyshev.second-1)<1e-8,"Septic Chebyshev extrema missed");
    auto septic=polynomialBounds({0,0,0,1,-4,6,-4,1});
    double expected=std::pow(3./7,3)*std::pow(4./7,4);
    require(std::abs(septic.first)<1e-12&&std::abs(septic.second-expected)<1e-12,"Repeated boundary/septic interior root missed");
    std::array<double,10> ninth{};ninth[1]=1;ninth[9]=-1;
    const auto canonical=canonicalPolynomialBounds(ninth);const double critical=std::pow(1./9,1./8);
    require(std::abs(canonical.first)<1e-12&&std::abs(canonical.second-(critical-std::pow(critical,9)))<1e-12,"Canonical ninth-degree interior extremum missed");
    bool nonfinite=false;try{polynomialBounds({0,0,0,0,0,0,0,INFINITY});}catch(...){nonfinite=true;}
    require(nonfinite,"Nonfinite highest coefficient accepted");
    auto highDegree=fixture();highDegree.spans[1].c[1].z=0;highDegree.spans[1].c[2].z=0;
    highDegree.spans[1].c[6].z=1;highDegree.spans[1].c[7].z=-1;
    auto highDimensions=measureInversionDimensions(highDegree);
    require(std::abs(highDimensions[0].verticalExtent-std::pow(6./7,6)/7)<1e-10,"Canonical sixth/seventh terms ignored");
    Design capped;capped.generationVersion=generatorVersion;capped.request.recipe=defaultRideRecipe();capped.track=fixture();
    capped.sections={{"loop",0,3,0,false,RideRole::Loop,"loop"}};
    capped.track.spans[1].c[1].z=360;capped.track.spans[1].c[2].z=-360;
    require(!validateReferenceDimensions(capped).valid(),"Actual loop extent is checked independently of its nominal recipe height");
    capped.track.spans[1].c[1].z=300;capped.track.spans[1].c[2].z=-300;
    require(validateReferenceDimensions(capped).valid(),"A same-category extent below the cap passes");
    capped.sections.front().role=RideRole::Return;
    for(auto& span:capped.track.spans){span.c[0].z=291.25;for(size_t n=1;n<span.c.size();++n)span.c[n].z=0;}
    capped.supports.resize(1);capped.supports.front().members.resize(1);auto& footing=capped.supports.front().members.front();footing.kind=SupportMemberKind::Footing;footing.base={0,0,-1.5};
    require(!validateReferenceDimensions(capped).valid(),"Embedded footing datum cannot be hidden by a small rail-to-rail extent");
    for(auto& span:capped.track.spans)span.c[0].z-=.5;
    require(validateReferenceDimensions(capped).valid(),"Overall size passes when measured conservatively from the embedded footing");
    capped.generationVersion="2.0.0-default.3";for(auto& span:capped.track.spans)span.c[0].z+=10;
    require(validateReferenceDimensions(capped).valid(),"Prior saved generation versions retain their original size contract");
    std::mt19937 random(7654321);std::uniform_real_distribution<double> dist(-100,100);
    for(int k=0;k<100;++k){std::array<double,8> p;for(double& c:p)c=dist(random);auto bounds=polynomialBounds(p);for(int j=0;j<=1000;++j){double u=j/1000.,v=0;for(int i=7;i>=0;--i)v=v*u+p[i];require(v>=bounds.first-1e-9&&v<=bounds.second+1e-9,"Polynomial bounds miss dense sample");}}
    auto t=fixture();auto dimensions=measureInversionDimensions(t);require(dimensions.size()==1,"Contiguous inversion split");auto d=dimensions[0];require(d.verticalMinimum==100&&d.verticalMaximum==101&&d.verticalExtent==1,"Vertical extent conflated with absolute height");require(d.forwardExtent==3&&d.lateralExtent==0&&d.pathLength==3,"Element dimensions incorrect");
    const std::vector<RideSection> typed{{"loop-0",0,2,0,false,RideRole::Loop,"loop"},{"immelmann-0",2,3,0,false,RideRole::Immelmann,"immelmann"}};
    const auto separated=measureInversionDimensions(t,typed);require(separated.size()==2&&separated[0].recipeId=="loop"&&separated[1].role==RideRole::Immelmann,"Adjacent inversion recipes remain independently measured without an artificial track gap");
    require(separated[0].pathLength==2&&separated[1].pathLength==1&&separated[0].verticalMaximum==101,"Typed grouping retains analytic extrema and exact covered spans");
    t.closed=true;t.knots[1].element=Element::Return;dimensions=measureInversionDimensions(t);require(dimensions.size()==1&&dimensions[0].wrapsSeam&&dimensions[0].pathLength==2,"Seam wrapping inversion split");
    t.closed=false;dimensions=measureInversionDimensions(t);require(dimensions.size()==2&&!dimensions[0].wrapsSeam&&!dimensions[1].wrapsSeam,"Open inversion runs joined");
    for(auto& sp:t.spans)for(auto& c:sp.c)c=rotate(c,{0,0,1},.7);dimensions=measureInversionDimensions(t);require(std::abs(dimensions[0].forwardExtent-1)<1e-12&&dimensions[0].lateralExtent<1e-12,"Horizontal rotation changes dimensions");
    bool threw=false;try{measureInversionDimensions(t,[]{return true;});}catch(...){threw=true;}require(threw,"Cancellation ignored");
    t.knots.pop_back();threw=false;try{measureInversionDimensions(t);}catch(...){threw=true;}require(threw,"Malformed span/knots accepted");
    for(int i=1;i<argc;++i){Design ride;std::string error;require(loadDesign(argv[i],ride,error),error.c_str());auto found=measureInversionDimensions(ride.track);require(!found.empty(),"Saved ride has no inversion dimensions");for(auto& x:found){require(x.verticalExtent>0&&x.forwardExtent>0,"Saved inversion has no positive physical extent");std::cout<<argv[i]<<" verticalExtent="<<x.verticalExtent<<" forwardExtent="<<x.forwardExtent<<" lateralExtent="<<x.lateralExtent<<" pathLength="<<x.pathLength<<"\n";}}
    std::cout<<"PASS "<<checks<<" dimension checks\n";return 0;
}catch(const std::exception& e){std::cerr<<"FAIL after "<<checks<<": "<<e.what()<<"\n";return 1;}}
