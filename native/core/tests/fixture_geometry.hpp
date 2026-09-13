#pragma once
#include "coaster/coaster.hpp"
#include <fstream>
#include <iomanip>
#include <sstream>
// Reads schema 2/3 geometry for isolated station tests.
inline coaster::Design stationGeometryFixture(const std::string& path){
    using namespace coaster;std::ifstream f(path,std::ios::binary);std::string magic;int schema;size_t bytes;uint64_t expected;
    if(!(f>>magic>>schema>>bytes>>expected)||magic!="COASTER"||(schema!=2&&schema!=3)||bytes>64*1024*1024||f.get()!='\n')throw std::runtime_error("Invalid station fixture header");
    std::string body(bytes,'\0');f.read(body.data(),std::streamsize(bytes));uint64_t actual=14695981039346656037ull;for(unsigned char c:body){actual^=c;actual*=1099511628211ull;}
    if(size_t(f.gcount())!=bytes||f.peek()!=EOF||actual!=expected)throw std::runtime_error("Station fixture checksum mismatch");
    std::istringstream p(body);Design d;int terrain;auto& r=d.request;p>>std::quoted(d.generationVersion)>>r.seed>>terrain>>r.maxCandidates>>r.simulationStep;r.terrain.kind=TerrainKind(terrain);
    bool hasReference;p>>r.targets.height>>r.targets.speed>>r.targets.inversionHeight>>r.targets.launchSeconds>>r.targets.requireIntensity>>hasReference>>r.targets.referenceExposure>>std::quoted(r.targets.referenceId);
    auto& l=r.limits;p>>l.minVerticalG>>l.maxVerticalG>>l.maxLateralG>>l.maxLongitudinalG>>l.maxJerkGps>>l.minClearance;
    auto& t=r.train;p>>t.cars>>t.carMass>>t.spacing>>t.seatHeight>>t.dragCdA>>t.rollingResistance>>t.airDensity;
    size_t knots,ops,supports;p>>std::quoted(d.topology)>>d.candidate>>d.track.closed>>knots>>ops>>supports;if(!p||knots<4||knots>200000||ops>1000||supports>20000)throw std::runtime_error("Station fixture counts invalid");
    for(size_t i=0;i<knots;++i){Knot k;int element;for(Vec3* v:{&k.position,&k.tangent,&k.curvature,&k.up})p>>v->x>>v->y>>v->z;p>>k.bank>>element;k.element=Element(element);d.track.knots.push_back(k);}
    if(!p)throw std::runtime_error("Station fixture knots truncated");d.track.rebuild();return d;
}
