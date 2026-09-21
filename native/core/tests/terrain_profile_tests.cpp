#include "coaster/coaster.hpp"
#include <iostream>
#include <stdexcept>
using namespace coaster;
static int checks=0;
static void check(bool ok,const char* message){++checks;if(!ok)throw std::runtime_error(message);}
// Frozen pre-extension surface formula: disabled new controls must preserve
// saved landscapes, including exact grid-node values, without migration.
static double legacyVertex(const Terrain& t,double x,double y){
    const double u=(x-t.centerX)/t.radiusX,v=(y-t.centerY)/t.radiusY-t.bend*u*u,r=u*u+v*v;
    auto smooth=[](double q){q=std::clamp(q,0.,1.);return q*q*q*(10+q*(-15+6*q));};
    double h=0;if(r<1){if(t.plateau>0)h=t.heightMeters*(1-smooth((std::sqrt(r)-t.plateau)/(1-t.plateau)))*(1-smooth(((x-t.cliffX)*std::cos(t.cliffHeading)+(y-t.cliffY)*std::sin(t.cliffHeading))/t.cliffWidth));else{const double shoulder=1-r*r;h=t.heightMeters*shoulder*shoulder*shoulder;}}
    for(const auto& ramp:t.ramps)h=std::max(h,ramp.height(x,y));double mask=1;if(t.plateau>0){const double q=std::clamp(((x-t.cliffX)*std::cos(t.cliffHeading)+(y-t.cliffY)*std::sin(t.cliffHeading))/t.cliffWidth,0.,1.);mask=1-q*q*q*(10+q*(-15+6*q));}
    for(const auto& knoll:t.knolls)h=std::max(h,knoll.at(x,y)*mask);return std::max(h,t.ridge.at(x,y)*mask);
}
int main(){try{
    Terrain flat;check(flat.valid()&&flat.name()=="flat"&&flat.slopeBound()==0,"Flat ground is supported");
    for(double x:{-100000.,0.,100000.})for(double y:{-100000.,0.,100000.})
        check(flat.height(x,y)==0&&flat.localSlopeBound(x,y,100)==0,"Flat ground remains level across the supported footprint");
    Terrain contour;contour.kind=TerrainKind::Highlands;contour.heightMeters=0;
    contour.ridge.points={{0,0,50,.2,0},{20,0,54,.2,0},{40,0,58,.2,0}};contour.ridge.spineCount=3;
    check(contour.valid(),"Contoured summit accepts finite grade controls");
    for(const auto& p:contour.ridge.points)check(std::abs(contour.vertexHeight(p.x,p.y)-p.height)<1e-10,"Each isolated terrain control owns its grade-plane height");
    const auto terrainBounds=contour.gridBounds();
    for(double x=terrainBounds[0];x<terrainBounds[2];x+=8)for(double y=terrainBounds[1];y<terrainBounds[3];y+=8){
        const double a=contour.height(x,y),b=contour.height(x+8,y),c=contour.height(x+8,y+8),d=contour.height(x,y+8);
        check(std::abs(contour.height(x+16./3,y+8./3)-(a+b+c)/3)<1e-10,"Contoured terrain uses the actual visible triangle interior");
        check(std::abs(contour.height(x+8./3,y+16./3)-(a+c+d)/3)<1e-10,"Both contoured triangle orientations agree");
        const double bound=contour.localSlopeBound(x+4,y+4,8);
        check(bound<=contour.slopeBound()+1e-9,"Contoured local slope fits its independent analytic global enclosure");
        check(std::abs(contour.height(x+1,y+2)-contour.height(x+7,y+6))<=bound*std::sqrt(52.)+1e-9,"Local slope encloses cross-face terrain relief");
    }
    const double outer=contour.ridge.at(80,0);
    contour.ridge.points.push_back({0,0,30,.2,0});
    check(contour.ridge.at(0,0)==30,"A lower passage constrains the summit instead of lifting track");
    check(contour.ridge.at(80,0)<=outer&&contour.ridge.at(-101,0)==0,"Lower constraints neither raise terrain nor expand its footprint");
    auto invalidContour=contour;invalidContour.ridge.spineCount=5;
    check(!invalidContour.valid(),"Malformed contour source ranges reject before sampling");
    for(int kind:{-1,2,3,99}){
        GenerationRequest request;request.terrain.kind=TerrainKind(kind);request.targets.requireIntensity=false;
        check(!request.terrain.valid(),"Removed or invalid terrain kind is refused");
        const auto report=validateRequest(request);
        check(!report.valid()&&report.errors.front().code=="TERRAIN_PROFILE","Unsupported surface fails at the request boundary");
        int callbacks=0;const auto rejected=generate(request,{},[&](const WorkProgress&){++callbacks;});
        check(!rejected.accepted()&&rejected.track.knots.empty()&&callbacks==0,"Removed terrain cannot construct or simulate a ride");
    }
    for(double value:{NAN,INFINITY,-INFINITY})
        check(!std::isfinite(flat.localSlopeBound(value,0,1))&&!std::isfinite(flat.localSlopeBound(0,value,1))&&!std::isfinite(flat.localSlopeBound(0,0,value)),"Nonfinite footprint queries fail closed");
    check(!std::isfinite(flat.localSlopeBound(0,0,-1)),"Negative footprint radius is refused");
    Terrain ridge;ridge.kind=TerrainKind::Highlands;ridge.centerX=17.3;ridge.centerY=-51.7;ridge.bend=-.28;
    check(ridge.valid()&&ridge.slopeBound()>0,"A curved highlands landform is valid");
    const auto bounds=ridge.gridBounds();
    for(double x=bounds[0];x<=bounds[2];x+=Terrain::gridStep)for(double y=bounds[1];y<=bounds[3];y+=Terrain::gridStep){
        const double h=ridge.height(x,y),hx=ridge.height(x+8,y),hy=ridge.height(x,y+8),hxy=ridge.height(x+8,y+8);
        check(std::abs(ridge.height(x+16./3,y+8./3)-(h+hx+hxy)/3)<1e-10,"Ground inside the first rendered triangle agrees with its affine face");
        check(std::abs(ridge.height(x+8./3,y+16./3)-(h+hy+hxy)/3)<1e-10,"Ground inside the second rendered triangle agrees with its affine face");
        const double cx=x+3.1,cy=y+2.7,base=ridge.height(cx,cy),bound=ridge.localSlopeBound(cx,cy,14);
        for(int i=0;i<12;++i){const double angle=i*pi/6,dx=13*std::cos(angle),dy=13*std::sin(angle);
            check(std::abs(ridge.height(cx+dx,cy+dy)-base)<=bound*13+1e-9,"Conservative local slope contains cross-cell ground relief throughout a footprint");}
        check(bound<=ridge.slopeBound()+1e-9,"Local terrain slope remains inside its global enclosure");
    }
    for(double x:{bounds[0],bounds[2]})for(double y:{bounds[1],bounds[3]})check(ridge.height(x,y)==0,"Terrain patch closes exactly onto the surrounding basin");
    Terrain escarpment=ridge;escarpment.plateau=.75;escarpment.cliffX=20;escarpment.cliffY=0;escarpment.cliffHeading=.4;
    escarpment.ramps.push_back({-300,-150,20,50,20,130,.25,.42,100});
    escarpment.knolls.push_back({-50,80,140,180});
    check(escarpment.valid(),"Shelf and graded ramp parameters form a valid shared terrain");
    for(int x=-350;x<400;x+=17)for(int y=-350;y<400;y+=19){
        const auto range=escarpment.heightRange(x,y,4.7);const double slope=escarpment.localSlopeBound(x,y,4.7);
        check(slope<=escarpment.slopeBound()+1e-9,"Ramp extrapolation and cliff faces fit the global slope enclosure");
        for(int k=0;k<64;++k){double a=k*pi/32;const double h=escarpment.height(x+4.7*std::cos(a),y+4.7*std::sin(a));check(h>=range[0]-1e-9&&h<=range[1]+1e-9,"Actual triangle heights throughout the footing disk fit the exact square enclosure");}
    }
    for(const auto* old:{&ridge,&escarpment,&contour})for(int x=-400;x<=400;x+=16)for(int y=-400;y<=400;y+=16)
        check(old->vertexHeight(x,y)==legacyVertex(*old,x,y),"Zero curvature and empty ravines retain exact legacy terrain vertices");
    Terrain curved;curved.kind=TerrainKind::Highlands;curved.radiusX=curved.radiusY=1000;curved.heightMeters=100;curved.plateau=.8;curved.bend=0;curved.cliffCurvature=.001;
    check(curved.valid(),"A finite concave cliff-front curvature is supported");
    check(std::abs(curved.vertexHeight(12,0)-50)<1e-12&&std::abs(curved.vertexHeight(52,200)-50)<1e-12,"The curved front has the independent parabolic half-height contour");
    check(curved.vertexHeight(12,200)==100,"Concave shoulders reach farther into the basin than the recessed centre");
    auto rotated=curved;rotated.cliffX=70;rotated.cliffY=-80;rotated.cliffHeading=.7;
    const double nx=std::cos(.7),ny=std::sin(.7),normal=35,side=150;
    check(std::abs(rotated.cliffCoordinate(70+normal*nx-side*ny,-80+normal*ny+side*nx)-(normal-.001*side*side))<1e-11,"Cliff curvature is expressed in the local front frame under rotation and translation");
    TerrainRavine tapered{0,0,200,0,20,80,40,100};
    check(tapered.valid(),"A tapered, graded, capped ravine has finite bounded controls");
    check(tapered.depth(0,0)==20&&tapered.depth(200,0)==80&&tapered.depth(100,0)==50,"Ravine centreline depths follow the independent endpoint and midpoint oracle");
    check(std::abs(tapered.depth(100,35)-25)<1e-12&&std::abs(tapered.depth(-20,0)-10)<1e-12,"Quintic cross-section and rounded cap reach half depth at half width");
    check(tapered.depth(-40,0)==0&&tapered.depth(300,0)==0&&tapered.depth(100,70)==0,"Ravine caps and tapered sides close continuously at their footprint edge");
    for(int x=-45;x<305;x+=11)for(int y=-105;y<=105;y+=13){
        const double a=tapered.depth(x,y),b=tapered.depth(x+.17,y-.23);
        check(std::abs(b-a)<=tapered.slopeBound()*std::hypot(.17,.23)+1e-10,"Independent point pairs fit the analytic ravine slope enclosure");
    }
    Terrain basin=curved;basin.cliffX=600;basin.cliffCurvature=0;
    const auto uncutBounds=basin.gridBounds();
    basin.ravines={{-100,0,100,0,30,30,60,60},{0,-100,0,100,50,50,60,60}};
    check(basin.valid()&&basin.vertexHeight(0,0)==50,"Connected branch cuts use the deepest excavation without adding depths");
    const double branchHeight=basin.vertexHeight(24,16);basin.ravines.push_back(basin.ravines[0]);
    check(basin.vertexHeight(24,16)==branchHeight,"Duplicate branch controls are idempotent");
    basin.ravines.push_back({4900,5000,5100,5000,100,100,80,80});
    check(basin.gridBounds()==uncutBounds&&basin.vertexHeight(5000,5000)==0,"Subtractive cuts outside the positive landscape cannot grow its grid bounds or excavate the zero basin");
    auto deep=basin;deep.ravines[0].depth0=deep.ravines[0].depth1=200;
    check(deep.vertexHeight(0,0)==0,"Excavation is clamped at the shared zero-metre basin floor");
    auto foothill=curved;foothill.cliffCurvature=0;foothill.foothills={{1800,0,60,180}};
    check(foothill.valid()&&foothill.vertexHeight(1800,0)==60,"Independent foothills survive beyond the escarpment mask");
    foothill.ravines={{1750,0,1850,0,20,20,50,50}};
    check(foothill.vertexHeight(1800,0)==40,"Ravines cut an actual positive bench beyond the cliff");
    check(foothill.gridBounds()[2]>1980,"Unmasked foothills are included in the rendered terrain domain");
    const auto foothillRange=foothill.heightRange(1800,0,7);const auto foothillSlope=foothill.localSlopeBound(1800,0,7);
    for(int k=0;k<32;++k){const double a=k*pi/16,x=1800+7*std::cos(a),y=7*std::sin(a),h=foothill.height(x,y);
        check(h>=foothillRange[0]-1e-9&&h<=foothillRange[1]+1e-9&&std::abs(h-foothill.height(1800,0))<=foothillSlope*7+1e-9,"Exact triangle bounds also enclose carved independent foothills");}
    auto flatFoothill=flat;flatFoothill.foothills=foothill.foothills;check(!flatFoothill.valid(),"Flat terrain rejects unmasked positive terrain controls");
    auto slope=curved;slope.cliffX=700;slope.cliffCurvature=0;slope.backSlope=TerrainSlope{0,0,80,.5,.25};
    check(slope.valid()&&slope.vertexHeight(0,0)==80&&slope.vertexHeight(-80,0)==40&&slope.vertexHeight(0,80)==100,"A broad planar rear slope clips the plateau at independently calculated heights");
    check(slope.gridBounds()==curved.gridBounds(),"A rear slope never expands positive terrain bounds");
    for(int x=-200;x<=200;x+=17)for(int y=-100;y<=100;y+=19){const auto range=slope.heightRange(x,y,7);const auto bound=slope.localSlopeBound(x,y,7);check(bound<=slope.slopeBound()+1e-9,"Rear-slope grid faces fit the global gradient enclosure");
        for(int k=0;k<12;++k){const double angle=k*pi/6,h=slope.height(x+7*std::cos(angle),y+7*std::sin(angle));check(h>=range[0]-1e-9&&h<=range[1]+1e-9,"Rear-slope footing bounds enclose both rendered triangles");}}
    for(double bad:{double(NAN),double(INFINITY),4.01}){auto invalid=slope;invalid.backSlope->gradeX=bad;check(!invalid.valid(),"Rear slopes reject invalid or unsupported gradients");}
    auto flatSlope=flat;flatSlope.backSlope=slope.backSlope;check(!flatSlope.valid(),"Flat terrain does not ignore a rear slope");
    auto carved=escarpment;carved.cliffCurvature=.002;carved.ravines={{-250,-50,250,70,20,100,60,110},{-80,-220,70,180,65,30,90,50}};
    check(carved.valid(),"Cliffs, positive benches, crowns and branch cuts share one terrain field");
    for(int x=-350;x<400;x+=41)for(int y=-350;y<400;y+=43){
        const double bound=carved.localSlopeBound(x,y,11),global=carved.slopeBound();const auto range=carved.heightRange(x,y,11);
        check(bound<=global+1e-9,"Curved and carved grid faces fit the conservative combined global slope enclosure");
        const double centre=carved.height(x,y);
        for(int k=0;k<16;++k){const double a=k*pi/8,px=x+11*std::cos(a),py=y+11*std::sin(a),h=carved.height(px,py);
            check(h>=range[0]-1e-9&&h<=range[1]+1e-9&&std::abs(h-centre)<=11*bound+1e-9,"Exact local slope and height bounds enclose carved terrain across source-grid cells");}
        const double gx=std::floor(x/8.)*8,gy=std::floor(y/8.)*8,a=carved.vertexHeight(gx,gy),b=carved.vertexHeight(gx+8,gy),c=carved.vertexHeight(gx+8,gy+8),d=carved.vertexHeight(gx,gy+8);
        check(std::abs(carved.height(gx+16./3,gy+8./3)-(a+b+c)/3)<1e-10&&std::abs(carved.height(gx+8./3,gy+16./3)-(a+c+d)/3)<1e-10,"Both source triangle orientations retain exact interpolation after carving");
    }
    for(double value:{-1.,.01001,double(NAN),double(INFINITY)}){auto bad=curved;bad.cliffCurvature=value;check(!bad.valid(),"Unsupported or nonfinite cliff curvature rejects before sampling");}
    auto noShelf=curved;noShelf.plateau=0;check(!noShelf.valid(),"A curved cliff requires a resolved plateau front");
    auto flatCut=flat;flatCut.ravines.push_back(tapered);check(!flatCut.valid(),"Flat terrain does not silently ignore ravine controls");
    for(int field=0;field<8;++field){auto bad=tapered;switch(field){case 0:bad.x0=100001;break;case 1:bad.y1=NAN;break;case 2:bad.x1=bad.x0;bad.y1=bad.y0;break;case 3:bad.x1=6000;break;case 4:bad.depth0=-1;break;case 5:bad.depth1=351;break;case 6:bad.width0=19;break;case 7:bad.width1=501;break;}check(!bad.valid(),"Ravine coordinates, length, depth and width have explicit finite limits");}
    auto tooMany=basin;tooMany.ravines.assign(17,tapered);check(!tooMany.valid(),"Terrain branch count is bounded");
    auto invalid=ridge;invalid.radiusX=0;check(!invalid.valid(),"Collapsed terrain width is rejected");
    invalid=ridge;invalid.bend=NAN;check(!invalid.valid(),"Nonfinite terrain shape is rejected");
    std::cout<<"PASS "<<checks<<" flat/highlands surface, mesh parity, slope enclosures and invalid-profile checks\n";return 0;
}catch(const std::exception& e){std::cerr<<"FAIL: "<<e.what()<<'\n';return 1;}}
