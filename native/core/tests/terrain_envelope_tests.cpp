#include "../src/terrain_envelope.hpp"
#include <iostream>
#include <stdexcept>
using namespace coaster;
int main(){try{
    int checks=0;const auto check=[&](bool value){++checks;if(!value)throw std::runtime_error("Physical terrain envelope contract failed");};
    Limits limits;
    for(auto kind:{TerrainKind::Flat,TerrainKind::Hills,TerrainKind::Canyon})for(int i=0;i<20;++i){
        const auto terrain=Terrain::seeded(kind,37);const double x=80*i-700,y=47*i-400;
        TrackSample frame{{x,y,70},{1,0,0},{},{0,0,1},{0,-1,0},Element::Turn};
        const double datum=detail::terrainEnvelopeDatum(terrain,limits,frame,true);
        for(int bank=-12;bank<=12;++bank){const auto up=rotate(frame.up,frame.tangent,bank*pi/24),right=cross(frame.tangent,up);
            for(double side:{-1.5,1.5})for(double height:{-.8,2.4}){const auto corner=frame.position+right*side+up*height;
                check(corner.z+datum>=terrain.height(corner.x,corner.y)+limits.minClearance);}}
        frame.position.z+=80;check(std::abs(detail::terrainEnvelopeDatum(terrain,limits,frame,true)+80-datum)<1e-10);
        frame.position.z-=80;auto moved=terrain;moved.offsetX+=1300;moved.offsetY-=700;
        frame.position.x+=1300;frame.position.y-=700;check(std::abs(detail::terrainEnvelopeDatum(moved,limits,frame,true)-datum)<1e-8);
    }
    std::cout<<"PASS "<<checks<<" independent physical terrain-envelope checks\n";return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
