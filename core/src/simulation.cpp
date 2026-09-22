#include "coaster/simulation.hpp"
#include <limits>
#include <unordered_map>

namespace coaster {
namespace {
constexpr std::array<double,6> carOffsets{-8.5,-5.1,-1.7,1.7,5.1,8.5};
constexpr std::array<double,3> seats{8.5,0,-8.5};
constexpr double seatHeight=1.2;
struct Dynamics {double a{},potentialSlope{},drive{},loss{},metric{1},metricS{};};
Dynamics acceleration(const Track& track,double s,double v,const Scenario& scenario,std::array<std::size_t,6>& hints){
    Dynamics d;d.metric=0;
    for(std::size_t i=0;i<carOffsets.size();++i){const auto f=track.at(s+carOffsets[i],hints[i]);const Vec3 first=f.t+f.upS*.5,second=f.k+f.upSS*.5;d.metric+=dot(first,first)/6;d.metricS+=2*dot(first,second)/6;d.potentialSlope+=gravity*first.z/6;}
    const auto center=track.at(s);const auto& p=track.source[center.element];
    // A distributed drive command acts on the train, with finite train mass
    // and offset energy. Spatial motor coverage is a separate hardware audit.
    d.drive=center.drive/scenario.massScale;
    d.loss=(p.rolling*std::tanh(v/.2)+p.drag*scenario.dragScale*v*v)/scenario.massScale;
    d.a=(d.drive-d.loss-d.potentialSlope-.5*d.metricS*v*v)/d.metric;return d;
}
void include(Vec3 x,Vec3& lo,Vec3& hi){lo={std::min(lo.x,x.x),std::min(lo.y,x.y),std::min(lo.z,x.z)};hi={std::max(hi.x,x.x),std::max(hi.y,x.y),std::max(hi.z,x.z)};}
}
Simulation simulate(const Track& track,const Scenario& scenario,double dt,const Cancel& cancel){
    if(!(dt>0&&dt<=.01)||scenario.dragScale<=0||scenario.massScale<=0)throw std::runtime_error("Invalid simulation configuration");
    Simulation result;result.dt=dt;for(auto& m:result.minimum)m={1e9,1e9,1e9};for(auto& m:result.maximum)m={-1e9,-1e9,-1e9};
    std::array<std::array<std::vector<double>,3>,3> traces;
    for(auto& seat:traces)for(auto& axis:seat)axis.reserve(std::size_t(210/dt));
    double s=0,v=track.source.front().initial.v,t=0;std::array<std::size_t,6> hints{};std::array<std::size_t,3> seatHints{};std::array<Vec3,3> previous{};
    // Time-commanded launch avoids the singular inverse t(s) at standstill.
    const auto& first=track.source.front();
    for(std::size_t index=0;index<std::size_t(260/dt);++index){poll(cancel);Sample q;q.time=t;q.s=s;q.speed=v;auto dynamics=acceleration(track,s,v,scenario,hints);
        if(t<=first.duration()&&first.role==Role::Launch){dynamics.drive=controlAt(first,t).drive/scenario.massScale;dynamics.a=(dynamics.drive-dynamics.loss-dynamics.potentialSlope-.5*dynamics.metricS*v*v)/dynamics.metric;}
        result.maxSpeed=std::max(result.maxSpeed,v);if(!result.launchTime&&v>=50-1e-5)result.launchTime=t;
        for(std::size_t i=0;i<seats.size();++i){const auto f=track.at(s+seats[i],seatHints[i]);const Vec3 firstDerivative=f.t+f.upS*seatHeight,secondDerivative=f.k+f.upSS*seatHeight;const Vec3 a=secondDerivative*(v*v)+firstDerivative*dynamics.a+Vec3{0,0,gravity};q.force[i]={dot(a,f.t)/gravity,dot(a,f.r)/gravity,dot(a,f.u)/gravity};include(q.force[i],result.minimum[i],result.maximum[i]);
            traces[i][0].push_back(q.force[i].x);traces[i][1].push_back(q.force[i].y);traces[i][2].push_back(q.force[i].z);
            if(index){const Vec3 rate=(q.force[i]-previous[i])/dt;Vec3 unused{1e9,1e9,1e9};include({std::abs(rate.x),std::abs(rate.y),std::abs(rate.z)},unused,result.rate[i]);}previous[i]=q.force[i];
        }
        if(index%std::max(std::size_t(1),std::size_t(std::llround(1/(60*dt))))==0)result.playback.push_back(q);
        if(s>=track.length){result.completed=true;result.duration=t;break;}
        const auto role=track.source[track.at(s).element].role;if(role==Role::Terminal)result.terminal+=dt;else result.active+=dt;if(role==Role::Clifftop||role==Role::Edge)result.clifftopActive+=dt;if(role==Role::Lip)result.lip+=dt;
        // Midpoint independent energy evolution on the canonical geometry.
        const double vm=std::max(0.,v+dynamics.a*dt/2),sm=s+v*dt/2;auto middle=acceleration(track,sm,vm,scenario,hints);
        if(t+dt/2<=first.duration()&&first.role==Role::Launch){middle.drive=controlAt(first,t+dt/2).drive/scenario.massScale;middle.a=(middle.drive-middle.loss-middle.potentialSlope-.5*middle.metricS*vm*vm)/middle.metric;}
        s+=vm*dt;v+=middle.a*dt;t+=dt;
        if(v<-.01||!std::isfinite(v)||v>150){result.failures.push_back("Train stalled or left supported speed domain");break;}v=std::max(0.,v);
    }
    if(!result.completed)result.failures.push_back("Train did not finish");
    for(std::size_t i=0;i<seats.size();++i){result.acceleration[i]=assessAccelerationF2291_25({traces[i][0],traces[i][1],traces[i][2],dt,0,i},cancel);if(!result.acceleration[i].passed)result.failures.push_back("F2291-25 scoped acceleration assessment failed at seat "+std::to_string(i));
        const auto lo=result.minimum[i],hi=result.maximum[i],rate=result.rate[i];
        if(lo.x< -4.5||hi.x>4.5||lo.y< -1.5||hi.y>1.5||lo.z< -1.5||hi.z>5)result.failures.push_back("Nominal force envelope exceeded at seat "+std::to_string(i));
        if(std::max({rate.x,rate.y,rate.z})>20)result.failures.push_back("Component rate exceeded at seat "+std::to_string(i));
    }return result;
}
Clearance assessClearance(const Track& track,double plateau,double spacing,const Cancel& cancel){
    Clearance result;double ordinary=0;std::size_t ordinaryCount=0;
    struct Cell {int x,y,z;bool operator==(const Cell&)const=default;};struct Hash {std::size_t operator()(const Cell& c)const{return std::hash<int>{}(c.x)^(std::hash<int>{}(c.y)<<1)^(std::hash<int>{}(c.z)<<2);}};
    struct Entry {Vec3 p;double s;};std::unordered_map<Cell,std::vector<Entry>,Hash> grid;
    for(double s=0;s<=track.length;s+=spacing){poll(cancel);const auto f=track.at(s);++result.samples;const auto role=track.source[f.element].role;
        const bool low=role==Role::Clifftop||role==Role::Journey||role==Role::Launch||role==Role::Terminal;
        const double above=f.p.z-ground(f.p.x,f.p.y,plateau);if(low){ordinary+=above;++ordinaryCount;result.ordinaryMaxHeight=std::max(result.ordinaryMaxHeight,above);}
        // Full oriented containment box at sampled swept stations. Refinement
        // and continuous gap bounds are required before clearance acceptance.
        for(double longitudinal:{-1.9,0.,1.9})for(double lateral:{-1.35,0.,1.35})for(double vertical:{-.9,0.,3.0}){const Vec3 p=f.p+f.t*longitudinal+f.r*lateral+f.u*vertical;const double gap=p.z-ground(p.x,p.y,plateau);result.minimumGround=std::min(result.minimumGround,gap);if(gap<.25)++result.terrainHits;}
        const Vec3 center=f.p+f.u;const Cell cell{int(std::floor(center.x/8)),int(std::floor(center.y/8)),int(std::floor(center.z/8))};
        for(int x=-1;x<=1;++x)for(int y=-1;y<=1;++y)for(int z=-1;z<=1;++z){const auto it=grid.find({cell.x+x,cell.y+y,cell.z+z});if(it==grid.end())continue;for(const auto& e:it->second)if(s-e.s>25){const double distance=norm(center-e.p);result.minimumNonlocal=std::min(result.minimumNonlocal,distance-5.5);if(distance<5.5)++result.trackHits;}}
        grid[cell].push_back({center,s});
    }result.ordinaryMeanHeight=ordinaryCount?ordinary/ordinaryCount:0;return result;
}
}
