#include "coaster/coaster.hpp"
#include <iostream>
#include <random>
#include <stdexcept>
#include <chrono>
// Internal validation kernel; no additional public or renderer geometry API.
namespace coaster::terrain_validation {double lowerBound(const TrackSample&,const Terrain&,double,double);}
namespace coaster::chord_validation {std::vector<double> arcBounds(const Track&,const ClearanceSweep&,int,Cancel);ValidationReport validate(const Track&,const ClearanceSweep&,int,Cancel);}
using namespace coaster;
static size_t checks=0;
static void check(bool x,const char* message){++checks;if(!x)throw std::runtime_error(message);}
static bool code(const ValidationReport& r,const char* text){for(const auto& f:r.errors)if(f.code==text)return true;return false;}
static double clear(Vec3 p,const Terrain& t){return p.z-t.height(p.x,p.y);}
static Track circle(double height,double bank=0){std::vector<AuthoredPoint> p;constexpr int n=128;for(int i=0;i<=n;++i){double a=2*pi*i/n;p.push_back({{20*std::sin(a),20*(1-std::cos(a)),height},bank,Element::Turn,{0,0,1}});}p.back()=p.front();return compile(p,true);}
int main(){try{
    Terrain flat;TrackSample pose{{0,0,3},{1,0,0},{},{0,0,-1},{0,1,0},Element::Inversion};
    // Scoped geometry counterexample, NOT an old full-validator accepted ride:
    // its extra1.6m reserve can independently reject it. Four old points are
    // above ground while supported seatHeight3 +.6 headroom penetrates terrain.
    double old=1e100;for(double side:{-1.5,1.5})for(double up:{-.8,2.4})old=std::min(old,clear(pose.position+pose.right*side+pose.up*up,flat));
    check(old>0,"Old four sampled cross-section points miss the headroom penetration");
    check(clear(pose.position+pose.up*3.6,flat)<0,"Full supported headroom intersects terrain");
    check(terrain_validation::lowerBound(pose,flat,3.6,.2)<0,"Certificate rejects omitted headroom");
    std::cout<<"Scoped headroom fixture: oldFourPointClearance="<<old<<" actualHeadClearance="<<clear(pose.position+pose.up*3.6,flat)<<" (no full old-validator acceptance claim)\n";
    // Longitudinal chassis extent lowers a tilted body below its cross section.
    const double a=pi/4;pose.tangent={std::cos(a),0,-std::sin(a)};pose.up={std::sin(a),0,std::cos(a)};pose.right=cross(pose.tangent,pose.up);pose.position.z=1;
    old=1e100;for(double side:{-1.5,1.5})for(double up:{-.8,2.4})old=std::min(old,clear(pose.position+pose.right*side+pose.up*up,flat));
    auto front=pose.position+pose.tangent*1.275-pose.up*.8;check(old>0&&front.z<0,"Longitudinal chassis can penetrate beneath clear cross-section points");check(terrain_validation::lowerBound(pose,flat,2.4,0)<=front.z+1e-12,"Longitudinal dimension included");
    // Terrain has a smooth interior maximum. Bare vertex probes cannot certify
    // the footprint; the Lipschitz lower bound must enclose this interior too.
    Terrain hills{TerrainKind::Hills};const double px=696.1446760456421,py=583.5781706660686,h=hills.height(px,py);
    pose={{px,py,h+.8-1e-5},{1,0,0},{},{0,0,1},{0,-1,0},Element::Return};
    double cornerMin=1e100;for(double x:{-1.275,1.275})for(double y:{-1.5,1.5})cornerMin=std::min(cornerMin,clear(pose.position+pose.tangent*x+pose.right*y-pose.up*.8,hills));
    check(cornerMin>0&&clear(pose.position-pose.up*.8,hills)<0,"Terrain interior maximum lies above all bottom-corner terrain probes");
    check(terrain_validation::lowerBound(pose,hills,2.4,0)<0,"Lipschitz certificate covers interior footprint");
    std::mt19937 rng(91827);std::uniform_real_distribution<double> angle(-pi,pi),place(-2000,2000),unitValue(-1,1),fraction(0,1);
    for(auto terrain:{Terrain{TerrainKind::Flat},hills,Terrain{TerrainKind::Canyon}})for(int n=0;n<80;++n){
        Vec3 axis=unit(Vec3{unitValue(rng),unitValue(rng),unitValue(rng)});double theta=angle(rng),top=2.4+1.2*fraction(rng);
        pose={{place(rng),place(rng),100},{},{}, {},{},Element::Return};pose.tangent=rotate({1,0,0},axis,theta);pose.up=rotate({0,0,1},axis,theta);pose.right=cross(pose.tangent,pose.up);
        double bound=terrain_validation::lowerBound(pose,terrain,top,.2);
        for(int k=0;k<100;++k){Vec3 p=pose.position+pose.tangent*(unitValue(rng)*1.275)+pose.right*(unitValue(rng)*1.5)+pose.up*(-.8+(top+.8)*fraction(rng));
            Vec3 motion=unit(Vec3{unitValue(rng),unitValue(rng),unitValue(rng)})*(.2*fraction(rng));check(bound<=clear(p+motion,terrain)+1e-9,"Full interior body plus Euclidean motion stays above certified lower bound");}
    }
    // Canonical interval endpoints on simultaneous up/bank rotation share the
    // same checked sweep; evaluate every body corner across the actual u cell.
    std::vector<AuthoredPoint> points;for(int i=0;i<=6;++i){double roll=.1*i;points.push_back({{double(i),0,20},roll,Element::Return,rotate({0,0,1},{1,0,0},roll)});}auto line=compile(points,false);TrainConfig train;train.seatHeight=3;auto sweep=buildClearanceSweep(line,train);const auto& cell=sweep.frames()[sweep.frames().size()/2];double bound=terrain_validation::lowerBound(cell.sample,hills,sweep.trainTop(),sweep.padding());
    for(int j=0;j<=20;++j){auto q=line.sampleSpan(cell.span,cell.parameterBegin+(cell.parameterEnd-cell.parameterBegin)*j/20);for(double x:{-1.275,1.275})for(double y:{-1.5,1.5})for(double z:{-.8,3.6})check(bound<=clear(q.position+q.tangent*x+q.right*y+q.up*z,hills)+1e-9,"Continuous canonical cell includes intermediate body corners");}
    Limits limits;auto safe=circle(20);check(validateGeometry(safe,flat,limits,train,{}).valid(),"Clear closed curve accepted with full headroom");
    auto safeSweep=buildClearanceSweep(safe,train);int nominal=int(std::ceil(safe.length/2));
    auto chordBounds=chord_validation::arcBounds(safe,safeSweep,nominal,{});check(chordBounds.size()==size_t(nominal),"Every chord gets one linear-time bound");
    for(size_t i=0;i<chordBounds.size();++i){double sampledLength=0;auto previous=safe.sample(safe.length*i/nominal).position;
        for(int j=1;j<=30;++j){auto point=safe.sample(safe.length*(i+j/30.)/nominal).position;sampledLength+=norm(point-previous);previous=point;}
        check(sampledLength<=chordBounds[i]+1e-8&&chordBounds[i]<=2.1,"Actual subchords obey the canonical integrated arc bound");}
    check(chord_validation::validate(safe,safeSweep,nominal,{}).valid(),"Nominal chord domain certified");
    check(code(chord_validation::validate(safe,safeSweep,int(std::ceil(safe.length/3)),{}),"TRACK_SAMPLING_DOMAIN"),"Uncertified coarse chords fail the explicit domain gate");
    check(code(chord_validation::validate(safe,safeSweep,nominal,[]{return true;}),"CANCELLED"),"Chord certificate cancellation");
    auto low=circle(3,pi);auto invalid=validateGeometry(low,flat,limits,train,{});check(code(invalid,"TERRAIN_CLEARANCE"),"Original terrain gate remains");check(code(invalid,"TERRAIN_SWEEP_CLEARANCE"),"Complete swept terrain certificate rejects low inverted headroom");
    limits.minClearance=30;check(!validateGeometry(safe,flat,limits,train,{}).valid(),"Configured clearance is never ignored");limits.minClearance=4;
    auto stale=safe;stale.spans[0].c[0].z+=1;check(code(validateGeometry(stale,flat,limits,train,{}),"SWEEP_DOMAIN"),"Empty support list cannot bypass canonical cache verification");
    auto malformed=safe;malformed.knots.clear();check(code(validateGeometry(malformed,flat,limits,train,{}),"GEOMETRY_DOMAIN"),"Malformed cardinality rejected before sampling");
    check(code(validateGeometry(safe,flat,limits,train,{},[]{return true;}),"CANCELLED"),"Geometry cancellation remains authoritative");
    size_t calls=0;check(code(validateGeometry(safe,flat,limits,train,{},[&]{return ++calls>50;}),"CANCELLED"),"Cancellation during prepared full-body coverage");
    std::cout<<"PASS "<<checks<<" terrain certificate checks: omitted headroom/length, interior peak, full body and motion bounds, canonical intervals, old gate, stale cache and cancellation\n";return 0;
}catch(const std::exception& e){std::cerr<<"FAIL after "<<checks<<": "<<e.what()<<'\n';return 1;}}
