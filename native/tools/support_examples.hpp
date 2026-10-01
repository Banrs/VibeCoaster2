#pragma once
#include "coaster/coaster.hpp"

// Geometric review fixtures, not complete rides or physics-accepted saves.
inline coaster::Design supportExample(coaster::SupportStructureKind kind,unsigned seed,double inversionScale=1.){
    using namespace coaster;
    Design d;d.request.seed=seed;std::vector<AuthoredPoint> points;
    const double yaw=.31*seed,c=std::cos(yaw),s=std::sin(yaw),lift=seed?12.+seed*3.:0.;
    const auto turn=[&](Vec3 p){return Vec3{c*p.x-s*p.y,s*p.x+c*p.y,p.z};};
    const auto point=[&](Vec3 p,Vec3 up,double bank,Element e){p.z+=lift;points.push_back({turn(p),bank,e,turn(up)});};
    if(seed){
        d.request.terrain.kind=TerrainKind::Highlands;d.request.terrain.heightMeters=0;
        const Vec3 a=turn({-250,0,0}),b=turn({300,0,0});
        d.request.terrain.ramps.push_back({a.x,a.y,b.x,b.y,3.,10.+seed*2.,.02,.03,180});
    }
    if(kind==SupportStructureKind::Camelback){
        for(int i=0;i<=300;++i){double x=i*2.-300,u=std::clamp((x+250)/500.,0.,1.);double z=18+147*std::pow(std::sin(pi*u),2);
            point({x,0,z},{0,0,1},0,Element::Hill);}
    }else{
        const bool loop=kind==SupportStructureKind::Loop;const double r=(loop?27.3:33.2)*inversionScale,angle=loop?2*pi:pi;
        for(int x=-60;x<0;x+=2)point({double(x),0,14},{0,0,1},0,Element::Inversion);
        for(int i=0;i<=240;++i){double a=angle*i/240.,u=double(i)/240.;
            point({r*std::sin(a),loop?14*smooth(u):0,14+r*(1-std::cos(a))},{-std::sin(a),0,std::cos(a)},0,Element::Inversion);}
        if(loop){for(int x=2;x<=60;x+=2)point({double(x),14,14},{0,0,1},0,Element::Inversion);}
        else{for(int i=1;i<=90;++i){double u=i/90.;point({-2.*i,0,14+2*r-12*smooth(u)},{0,0,-1},pi*smooth(std::min(1.,u*2)),Element::Inversion);}}
    }
    d.track=compile(points,false);return d;
}
