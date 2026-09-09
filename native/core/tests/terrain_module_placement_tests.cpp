#include "../src/terrain_module_placement.hpp"
#include <iostream>
#include <stdexcept>
using namespace coaster;
namespace {
int checks=0;
void check(bool passed,const char* message){++checks;if(!passed)throw std::runtime_error(message);}
void close(double a,double b){check(std::abs(a-b)<1e-8,"Terrain placement coordinate invariance");}
}
int main(){try{
    detail::TerrainModuleFootprint loop;loop.entrance={0,0,0};
    for(int i=0;i<=64;++i){double u=i/64.;loop.samples.push_back({0,500*u,100*std::pow(std::sin(pi*u),2)});}
    Terrain terrain;terrain.kind=TerrainKind::Canyon;terrain.verticalScale=.1;terrain.cliffHeight=210;terrain.cliffWidth=140;
    auto crossing=detail::assessTerrainModulePlacement(terrain,{},0,loop,6.5);
    auto alongFloor=detail::assessTerrainModulePlacement(terrain,{},pi/2,loop,6.5);
    check(crossing.entranceGroundHeight>210,"A rigid loop crossing from canyon floor to rim needs a tall entrance");
    check(alongFloor.entranceGroundHeight<9,"An along-floor inversion has a low feasible entrance");
    check(crossing.score>alongFloor.score+400,"Placement ranking prefers terrain-compatible inversion footprint");
    Terrain transformed=terrain;transformed.offsetX=123;transformed.offsetY=-84;transformed.headingRadians=.73;
    auto moved=detail::assessTerrainModulePlacement(transformed,{123,-84,0},.73,loop,6.5);
    close(crossing.minimumDatum,moved.minimumDatum);close(crossing.score,moved.score);
    Terrain flat;
    auto a=detail::assessTerrainModulePlacement(flat,{},0,loop,6.5);
    for(auto& p:loop.samples)p.z*=3;
    auto b=detail::assessTerrainModulePlacement(flat,{},0,loop,6.5);
    close(a.score,0);close(b.score,0);close(a.minimumDatum,b.minimumDatum);
    // A translated local Z origin must not change the physical placement cost.
    loop.entrance.z+=80;for(auto& p:loop.samples)p.z+=80;
    auto shifted=detail::assessTerrainModulePlacement(flat,{},0,loop,6.5);
    close(shifted.score,b.score);close(shifted.minimumDatum+80,b.minimumDatum);
    std::cout<<"Terrain module placement: "<<checks<<" checks passed\n";return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
