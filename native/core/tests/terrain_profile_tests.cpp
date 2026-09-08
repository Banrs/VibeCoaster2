#include "coaster/coaster.hpp"
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <random>
#include <sstream>
#include <stdexcept>
namespace coaster::terrain_validation {double lowerBound(const TrackSample&,const Terrain&,double,double);}
using namespace coaster;
namespace {
size_t checks=0;
void require(bool condition,const char* message){++checks;if(!condition)throw std::runtime_error(message);}
void requireDefaultHeight(const Terrain& terrain,double x,double y,double expected){
    // The core disables contraction; this test may use FMA and a different libm.
    // Budget roundoff against the formula's amplitude, including cancellation:
    // at most 1.1e-12 m, far below any physical terrain/clearance tolerance.
    const double tolerance=64*std::numeric_limits<double>::epsilon()*(terrain.kind==TerrainKind::Hills?20:77);
    const double actual=terrain.height(x,y);
    if(!std::isfinite(actual)||std::abs(actual-expected)>tolerance)std::cerr<<std::setprecision(17)<<"default terrain x="<<x<<" y="<<y<<" actual="<<actual<<" expected="<<expected<<" tolerance="<<tolerance<<'\n';
    require(std::isfinite(actual)&&std::abs(actual-expected)<=tolerance,"Default analytic terrain changed");
}
bool same(const Terrain& a,const Terrain& b){return a.kind==b.kind&&a.verticalScale==b.verticalScale&&a.horizontalScale==b.horizontalScale&&a.offsetX==b.offsetX&&a.offsetY==b.offsetY&&a.headingRadians==b.headingRadians&&a.cliffHeight==b.cliffHeight&&a.cliffWidth==b.cliffWidth;}
Vec3 world(const Terrain& t,double x,double y){const double c=std::cos(t.headingRadians),s=std::sin(t.headingRadians);return {t.offsetX+t.horizontalScale*(c*x-s*y),t.offsetY+t.horizontalScale*(s*x+c*y),0};}
uint64_t checksum(const std::string& bytes){uint64_t h=14695981039346656037ull;for(unsigned char c:bytes){h^=c;h*=1099511628211ull;}return h;}
// Well-sized deliberately invalid geometry isolates extension rejection before replay.
std::string payload(const std::string& extension){std::ostringstream p;p<<std::quoted(generatorVersion)<<" 7 2 1 0.0010416666666666667\n220 75 80 1.4 0 0 0 \"\"\n-1.5 5.5 1.5 4.5 20 4\n6 1500 3.4 1.2 2.4 0.002 1.225\n\"fixture\" 0 1 4 0 0\n";for(int i=0;i<4;++i)p<<"0 0 0 1 0 0 0 0 0 0 0 1 0 0\n";p<<extension;return p.str();}
std::string extension(const std::string& profile){return "EXTENSIONS 1\nTERRAIN_PROFILE 1 "+std::to_string(profile.size())+"\n"+profile;}
void write(const std::filesystem::path& path,const std::string& bytes){std::ofstream f(path,std::ios::binary);f<<"COASTER 5 "<<bytes.size()<<' '<<checksum(bytes)<<'\n'<<bytes;if(!f)throw std::runtime_error("Cannot write scoped persistence fixture");}
}
int main(int argc,char** argv){try{
    (void)argc;std::mt19937_64 random(45123);std::uniform_real_distribution<double> place(-12000,12000),signedUnit(-1,1),fraction(0,1);
    Terrain flat;require(flat.valid()&&flat.isDefaultProfile()&&flat.slopeBound()==0,"Default flat profile changed");
    for(auto kind:{TerrainKind::Hills,TerrainKind::Canyon}){
        Terrain base{kind};require(base.valid()&&base.isDefaultProfile(),"Base fixture profile invalid");
        // Independent 100-digit decimal evaluations, rounded to double. Binary-
        // exact coordinates cover the origin, cancellation and the sampled extent.
        const double fixtures[][4]={{0,0,0,-65},{470,390,15.936973934518379,2.3899500449468665},{-470,-390,1.0567881040473286,-13.899566918981202},
            {12000,-12000,2.8102364660439707,2.3612483272294997},{-7319.125,9860.5,-3.2703423064961008,9.751986659905036},
            {891.25,115.75,11.206924991957404,-52.942856078667276},{-4096,4096,6.8895725783354935,-2.6000382663018802},{-.125,.125,-1.0229132273830759e-6,-65.002350960718758}};
        for(const auto& fixture:fixtures)requireDefaultHeight(base,fixture[0],fixture[1],fixture[kind==TerrainKind::Hills?2:3]);
        for(int i=0;i<100;++i){double x=place(random),y=place(random);double expected=kind==TerrainKind::Hills?12*std::sin(x/470)*std::sin(y/390)+8*std::sin((x+y)/720):-65*std::exp(-std::pow((y-120*std::sin(x/850))/210,2))+12*std::sin(x/630);requireDefaultHeight(base,x,y,expected);}
        Terrain previous=base;
        for(uint64_t seed=0;seed<64;++seed){auto t=Terrain::seeded(kind,seed);require(t.valid()&&!t.isDefaultProfile(),"Seeded profile invalid or unchanged");require(same(t,Terrain::seeded(kind,seed)),"Seed does not reproduce profile");require(!same(t,previous),"Adjacent seeds duplicate landscape profile");previous=t;
            if(kind==TerrainKind::Canyon){
                require(t.cliffHeight>=195&&t.cliffHeight<=225,"Cliff amplitude outside authored range");
                require(t.cliffWidth>=120&&t.cliffWidth<=180,"Seeded wall transition is no longer steep and bounded");
                Terrain baseOnly=t;baseOnly.cliffHeight=0;
                auto floor=world(t,0,0);require(t.localSlopeBound(floor.x,floor.y,20)<.05,"Valley floor inherits remote cliff gradient");
                for(double side:{-1.,1.}){
                    auto rim=world(t,0,side*(260+t.cliffWidth+30));require(t.height(rim.x,rim.y)-t.height(floor.x,floor.y)>195,"Valley floor and rim do not have real ~200 m separation");require(t.localSlopeBound(rim.x,rim.y,20)<.05,"Plateau inherits remote wall gradient");
                    auto midpoint=world(t,0,side*(260+t.cliffWidth*.5));const double e=.001;
                    const double gx=(t.height(midpoint.x+e,midpoint.y)-t.height(midpoint.x-e,midpoint.y))/(2*e),gy=(t.height(midpoint.x,midpoint.y+e)-t.height(midpoint.x,midpoint.y-e))/(2*e);
                    require(std::hypot(gx,gy)>1.5,"Canyon wall is still a gentle hill");
                    for(double u:{-.2,0.,.02,.25,.5,.75,.98,1.,1.2}){
                        auto center=world(t,137,120*std::sin(137./850)+side*(260+t.cliffWidth*u));
                        const double wall=t.height(center.x,center.y)-baseOnly.height(center.x,center.y);
                        if(u<=0)require(std::abs(wall)<1e-10,"C3 wall spills onto the flat valley floor");
                        if(u>=1)require(std::abs(wall-t.cliffHeight)<1e-9,"C3 wall does not reach a constant plateau");
                        for(double radius:{0.,2.,40.,300.}){
                            const double local=t.localSlopeBound(center.x,center.y,radius);
                            require(local<=t.slopeBound()+1e-10,"Local gradient bound exceeds its global maximum");
                            for(int sample=0;sample<32;++sample){const double angle=2*pi*sample/32;const double rr=radius*(sample%2?1.:.5);const double x=center.x+rr*std::cos(angle),y=center.y+rr*std::sin(angle);const double dx=(t.height(x+e,y)-t.height(x-e,y))/(2*e),dy=(t.height(x,y+e)-t.height(x,y-e))/(2*e);require(std::hypot(dx,dy)<=local+1e-7,"Local disk bound misses wall/join/peak gradients");require(std::abs(t.height(x,y)-t.height(center.x,center.y))<=local*rr+1e-8,"Local disk bound misses intervening wall height");}
                        }
                        TrackSample pose{{center.x,center.y,t.height(center.x,center.y)+8},{1,0,0},{},{0,0,1},{0,-1,0},Element::Return};
                        const double lower=terrain_validation::lowerBound(pose,t,3.6,.2);
                        for(int sample=0;sample<32;++sample){auto q=pose.position+Vec3{signedUnit(random)*1.275,signedUnit(random)*1.5,-.8+4.4*fraction(random)};q=q+unit(Vec3{signedUnit(random),signedUnit(random),signedUnit(random)})*(.2*fraction(random));require(lower<=q.z-t.height(q.x,q.y)+1e-8,"Full-body local certificate misses steep wall motion/interior");}
                    }
                }
                require(!std::isfinite(t.localSlopeBound(floor.x,floor.y,-1)),"Negative local-bound radius accepted");
            }
            for(int i=0;i<100;++i){double x=place(random),y=place(random);constexpr double e=.005;const double gx=(t.height(x+e,y)-t.height(x-e,y))/(2*e),gy=(t.height(x,y+e)-t.height(x,y-e))/(2*e);require(std::hypot(gx,gy)<=t.slopeBound()+1e-8,"Analytic slope bound underestimates sampled gradient");Vec3 delta{signedUnit(random)*100,signedUnit(random)*100,0};require(std::abs(t.height(x+delta.x,y+delta.y)-t.height(x,y))<=t.slopeBound()*norm(delta)+1e-8,"Global terrain Lipschitz bound violated");
                Vec3 axis=unit(Vec3{signedUnit(random),signedUnit(random),signedUnit(random)});double angle=fraction(random)*2*pi;TrackSample pose{{x,y,t.height(x,y)+8},rotate({1,0,0},axis,angle),{},rotate({0,0,1},axis,angle),{},Element::Return};pose.right=cross(pose.tangent,pose.up);double bound=terrain_validation::lowerBound(pose,t,3.6,.2);
                for(int j=0;j<8;++j){Vec3 q=pose.position+pose.tangent*(signedUnit(random)*1.275)+pose.right*(signedUnit(random)*1.5)+pose.up*(-.8+4.4*fraction(random));q=q+unit(Vec3{signedUnit(random),signedUnit(random),signedUnit(random)})*(.2*fraction(random));require(bound<=q.z-t.height(q.x,q.y)+1e-8,"Full-body clearance certificate omits profiled terrain");}
            }
        }
    }
    Terrain extreme{TerrainKind::Canyon};extreme.verticalScale=6;extreme.horizontalScale=.5;extreme.cliffHeight=250;extreme.cliffWidth=80;require(extreme.valid(),"Supported extreme terrain rejected");
    for(double x=-500;x<=500;x+=13)for(double y=-500;y<=500;y+=17){constexpr double e=.001;double gx=(extreme.height(x+e,y)-extreme.height(x-e,y))/(2*e),gy=(extreme.height(x,y+e)-extreme.height(x,y-e))/(2*e);require(std::hypot(gx,gy)<=extreme.slopeBound()+1e-8,"Extreme valid profile gradient exceeds certificate");}
    for(int field=0;field<7;++field){auto invalid=extreme;double* fields[]={&invalid.verticalScale,&invalid.horizontalScale,&invalid.offsetX,&invalid.offsetY,&invalid.headingRadians,&invalid.cliffHeight,&invalid.cliffWidth};*fields[field]=std::numeric_limits<double>::quiet_NaN();require(!invalid.valid()&&!std::isfinite(invalid.slopeBound()),"Nonfinite profile accepted");}
    auto invalid=extreme;invalid.horizontalScale=0;require(!invalid.valid(),"Zero wavelength accepted");invalid=extreme;invalid.kind=TerrainKind::Hills;require(!invalid.valid(),"Canyon-only shelf accepted for hills");
    const auto output=std::filesystem::absolute(argv[0]).parent_path()/"terrain-profile-malformed.coaster";
    Design destination;destination.request.seed=993;std::string error;
    for(const std::string& bad:{"1 0 0 0 0 200 600\n","nan 1 0 0 0 200 600\n","1 1 0 0 0 251 600\n","1 1 0 0 4 200 600\n","1 1 0 0 0 200 600 trailing\n"}){write(output,payload(extension(bad)));require(!loadDesign(output.string(),destination,error),"Malformed persisted terrain accepted");require(error.find("terrain profile")!=std::string::npos,"Malformed terrain not rejected before geometry replay");require(destination.request.seed==993,"Failed profile parse mutated destination");}
    write(output,payload("EXTENSIONS 0\n"));require(!loadDesign(output.string(),destination,error)&&error=="Missing required terrain profile","Missing exact-version terrain profile accepted");
    // A valid extension reaches geometry revalidation, proving fields are parsed before geometry use.
    write(output,payload(extension("0.3 1.2 30 -50 0.7 210 500\n")));require(!loadDesign(output.string(),destination,error)&&error.find("terrain profile")==std::string::npos,"Valid profile rejected by parser");
    Design report;report.request.terrain=extreme;require(reportJson(report).find("\"cliffHeightMeters\":250")!=std::string::npos,"Report omits actual persisted landscape parameters");
    std::cout<<"PASS "<<checks<<" terrain profile checks: independent default fixtures, seeded variation, physical cliff relief, analytic gradient/full-body bounds and fail-closed persistence\n";return 0;
}catch(const std::exception& e){std::cerr<<"FAIL after "<<checks<<": "<<e.what()<<'\n';return 1;}}
