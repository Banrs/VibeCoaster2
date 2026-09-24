#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <optional>
#include <string>
#include <vector>

namespace coaster {
enum class TerrainKind { Flat, Highlands };
// Broad talus/approach ramps join a raised landscape to its surrounding basin.
// Their two tangent planes fit a constant or smoothly increasing grade.
struct TerrainRamp {
    bool operator==(const TerrainRamp&) const=default;
    double x0{},y0{},x1{},y1{},h0{},h1{},grade0{},grade1{},width{100};
    double length() const{return std::hypot(x1-x0,y1-y0);}
    bool valid() const{for(double v:{x0,y0,x1,y1,h0,h1,grade0,grade1,width})if(!std::isfinite(v))return false;return length()>1&&length()<3000&&h0>=0&&h1>=0&&h0<=350&&h1<=350&&width>=20&&width<=500&&std::abs(grade0)<=2&&std::abs(grade1)<=2;}
    double height(double x,double y) const {
        const double len=length(),fx=(x1-x0)/len,fy=(y1-y0)/len;
        const double d=(x-x0)*fx+(y-y0)*fy,side=std::abs((y-y0)*fx-(x-x0)*fy);
        auto ease=[](double u){u=std::clamp(u,0.,1.);return u*u*u*(10+u*(-15+6*u));};
        const double along=ease((d+80)/80)*ease((len+80-d)/80),across=1-ease((side-width*.35)/(width*.65));
        const double at=std::clamp(d,0.,len),level=d<0?h0+grade0*d:d>len?h1+grade1*(d-len):std::max(h0+grade0*at,h1+grade1*(at-len));
        return std::max(0.,level)*along*across;
    }
    std::array<double,4> bounds() const {
        const double len=length(),fx=(x1-x0)/len,fy=(y1-y0)/len;std::array<double,4> b{INFINITY,INFINITY,-INFINITY,-INFINITY};
        for(double d:{-80.,len+80})for(double side:{-width,width}){double x=x0+d*fx-side*fy,y=y0+d*fy+side*fx;b[0]=std::min(b[0],x);b[1]=std::min(b[1],y);b[2]=std::max(b[2],x);b[3]=std::max(b[3],y);}return b;
    }
};
// Broad rounded crowns on the highland shelf, shared with the visible mesh.
struct TerrainKnoll {
    bool operator==(const TerrainKnoll&) const=default;
    double x{},y{},height{},radius{180};
    bool valid() const{return std::isfinite(x)&&std::isfinite(y)&&std::isfinite(height)&&std::isfinite(radius)&&std::abs(x)<=100000&&std::abs(y)<=100000&&height>=0&&height<=350&&radius>=50&&radius<=1000;}
    double at(double px,double py) const{const double t=((px-x)*(px-x)+(py-y)*(py-y))/(radius*radius),u=1-t;return t>=1?0:height*u*u*u;}
};
// A single broad rear slope limits the plateau instead of fitting a sequence
// of rail samples. It cannot expand the landform's positive footprint.
struct TerrainSlope {
    double x{},y{},height{},gradeX{},gradeY{};
    double width{}; // Lateral half-width; zero retains the historical unbounded clip.
    bool operator==(const TerrainSlope&) const=default;
    bool valid() const {
        const double grade=std::hypot(gradeX,gradeY);
        return std::isfinite(x)&&std::isfinite(y)&&std::isfinite(height)&&std::isfinite(gradeX)&&std::isfinite(gradeY)&&std::isfinite(width)&&
            std::abs(x)<=100000&&std::abs(y)<=100000&&height>=0&&height<=350&&grade<=4&&
            (width==0||(width>=50&&width<=1000&&grade>1e-12));
    }
    double at(double px,double py) const{return std::max(0.,height+gradeX*(px-x)+gradeY*(py-y));}
    double limit(double px,double py,double surface) const {
        const double clipped=std::min(surface,at(px,py));
        if(width==0)return clipped;
        // A bounded cut needs an explicit direction. Invalid direct callers
        // receive no usable surface, rather than an unbounded fallback cut.
        if(!valid())return std::numeric_limits<double>::quiet_NaN();
        const double side=std::abs(-(px-x)*gradeY+(py-y)*gradeX)/std::hypot(gradeX,gradeY);
        const double u=std::clamp((side-width*.35)/(width*.65),0.,1.);
        return std::lerp(clipped,surface,u*u*u*(10+u*(-15+6*u)));
    }
};
// A broad valley cut with rounded end caps. Widths are half-widths; depth and
// width change smoothly along the segment. Branches combine by maximum depth,
// so intersections do not accidentally add their excavations together.
struct TerrainRavine {
    bool operator==(const TerrainRavine&) const=default;
    double x0{},y0{},x1{},y1{},depth0{},depth1{},width0{80},width1{80};
    double length() const{return std::hypot(x1-x0,y1-y0);}
    bool valid() const {
        for(double v:{x0,y0,x1,y1,depth0,depth1,width0,width1})if(!std::isfinite(v))return false;
        return std::abs(x0)<=100000&&std::abs(y0)<=100000&&std::abs(x1)<=100000&&std::abs(y1)<=100000&&
            length()>1&&length()<6000&&depth0>=0&&depth0<=350&&depth1>=0&&depth1<=350&&
            width0>=20&&width0<=500&&width1>=20&&width1<=500;
    }
    double depth(double x,double y) const {
        const double dx=x1-x0,dy=y1-y0;
        const double t=std::clamp(((x-x0)*dx+(y-y0)*dy)/(dx*dx+dy*dy),0.,1.);
        const double along=t*t*t*(10+t*(-15+6*t));
        const double width=std::lerp(width0,width1,along),level=std::lerp(depth0,depth1,along);
        const double r=std::hypot(x-x0-t*dx,y-y0-t*dy)/width;
        if(r>=1)return 0;
        return level*(1-r*r*r*(10+r*(-15+6*r)));
    }
    double slopeBound() const {
        const double rate=1.875/length(),width=std::min(width0,width1);
        return rate*std::abs(depth1-depth0)+1.875*std::max(depth0,depth1)/width*(1+rate*std::abs(width1-width0));
    }
    std::array<double,4> bounds() const {
        const double width=std::max(width0,width1);
        return {std::min(x0,x1)-width,std::min(y0,y1)-width,std::max(x0,x1)+width,std::max(y0,y1)+width};
    }
};
// A broad contoured summit. Local grade planes with quadratic shoulders form
// one lower envelope. Additional constraints preserve nearby lower passages;
// they do not expand the landform's footprint. The visible 8m grid remains the
// sole collision surface, so fitting this intent is not clearance acceptance.
struct TerrainRidge {
    struct Point {double x{},y{},height{},gx{},gy{};bool operator==(const Point&) const=default;};
    bool operator==(const TerrainRidge&) const=default;
    std::vector<Point> points;
    size_t spineCount{};double width{100},curvature{.04};
    bool valid() const {
        if(points.empty())return spineCount==0;
        if(points.size()>512||spineCount<2||spineCount>points.size()||!std::isfinite(width)||width<30||width>200||!std::isfinite(curvature)||curvature<=0||curvature>.1)return false;
        for(const auto& p:points){for(double v:{p.x,p.y,p.height,p.gx,p.gy})if(!std::isfinite(v))return false;
            if(std::abs(p.x)>100000||std::abs(p.y)>100000||p.height< -10||p.height>350||std::hypot(p.gx,p.gy)>4)return false;}
        return true;
    }
    double at(double x,double y) const {
        if(points.empty())return 0;double distance=INFINITY,level=INFINITY;
        for(size_t i=0;i<points.size();++i){const auto& p=points[i];const double dx=x-p.x,dy=y-p.y,d2=dx*dx+dy*dy;
            if(i<spineCount)distance=std::min(distance,d2);
            level=std::min(level,p.height+p.gx*dx+p.gy*dy+curvature*d2);}
        const double u=std::sqrt(distance)/width;if(u>=1)return 0;
        return std::max(0.,level)*(1-u*u*u*(10+u*(-15+6*u)));
    }
    std::array<double,4> bounds() const {
        std::array<double,4> b{INFINITY,INFINITY,-INFINITY,-INFINITY};
        for(size_t i=0;i<spineCount;++i){const auto& p=points[i];b[0]=std::min(b[0],p.x-width);b[1]=std::min(b[1],p.y-width);b[2]=std::max(b[2],p.x+width);b[3]=std::max(b[3],p.y+width);}return b;
    }
    std::array<double,2> enclosure() const {
        if(points.empty())return {0,0};double high=0,grade=0;auto b=bounds();
        for(const auto& p:points){high=std::max(high,p.height);grade=std::max(grade,std::hypot(p.gx,p.gy));b[0]=std::min(b[0],p.x);b[1]=std::min(b[1],p.y);b[2]=std::max(b[2],p.x);b[3]=std::max(b[3],p.y);}
        const double height=high+grade*width+curvature*width*width;
        return {height,grade+2*curvature*std::hypot(b[2]-b[0],b[3]-b[1])+1.875*height/width};
    }
};
// One resolved landform, shared by planning, solid clearance, save data and
// rendering. A bent ridge has rounded shoulders instead of a copied plateau.
struct Terrain {
    bool operator==(const Terrain&) const=default;
    TerrainKind kind{TerrainKind::Flat};
    double centerX{},centerY{},heightMeters{110},radiusX{350},radiusY{240},bend{.2};
    double plateau{},cliffX{},cliffY{},cliffHeading{},cliffWidth{24};
    double cliffCurvature{}; // 1/m; positive bends the front into a concave bay
    std::optional<TerrainSlope> backSlope;
    std::vector<TerrainRamp> ramps;
    std::vector<TerrainKnoll> knolls;
    std::vector<TerrainKnoll> foothills; // Independent positive terrain beyond the escarpment front.
    std::vector<TerrainRavine> ravines;
    TerrainRidge ridge;
    static constexpr double gridStep=8;
    bool valid() const {
        if(kind!=TerrainKind::Flat&&kind!=TerrainKind::Highlands)return false;
        if(backSlope&&(!backSlope->valid()||kind!=TerrainKind::Highlands||plateau<=0))return false;
        if(ramps.size()>8||knolls.size()>8||foothills.size()>8||ravines.size()>16||(kind==TerrainKind::Flat&&(!ramps.empty()||!knolls.empty()||!foothills.empty()||!ravines.empty())))return false;
        if(!ridge.valid()||(kind==TerrainKind::Flat&&!ridge.points.empty()))return false;
        for(const auto& ramp:ramps)if(!ramp.valid())return false;
        for(const auto& knoll:knolls)if(!knoll.valid())return false;
        for(const auto& foothill:foothills)if(!foothill.valid())return false;
        for(const auto& ravine:ravines)if(!ravine.valid())return false;
        if(!std::isfinite(cliffCurvature)||cliffCurvature<0||cliffCurvature>.01||(cliffCurvature>0&&(kind!=TerrainKind::Highlands||plateau<=0)))return false;
        for(double x:{centerX,centerY,heightMeters,radiusX,radiusY,bend,plateau,cliffX,cliffY,cliffHeading,cliffWidth})if(!std::isfinite(x))return false;
        return std::abs(centerX)<=100000&&std::abs(centerY)<=100000&&heightMeters>=0&&heightMeters<=350&&radiusX>=50&&radiusX<=3000&&radiusY>=50&&radiusY<=3000&&std::abs(bend)<=.6&&plateau>=0&&plateau<=.9&&std::abs(cliffX)<=100000&&std::abs(cliffY)<=100000&&cliffWidth>=8&&cliffWidth<=200;
    }
    std::string name() const {return kind==TerrainKind::Flat?"flat":kind==TerrainKind::Highlands?"highlands":"unsupported";}
    double cliffCoordinate(double x,double y) const {
        const double normal=(x-cliffX)*std::cos(cliffHeading)+(y-cliffY)*std::sin(cliffHeading);
        if(cliffCurvature==0)return normal; // bit-preserving legacy path
        const double tangent=-(x-cliffX)*std::sin(cliffHeading)+(y-cliffY)*std::cos(cliffHeading);
        return normal-cliffCurvature*tangent*tangent;
    }
    double landformHeight(double x,double y) const {
        if(kind==TerrainKind::Flat)return 0;
        const double u=(x-centerX)/radiusX,v=(y-centerY)/radiusY-bend*u*u,t=u*u+v*v;
        if(t>=1)return 0;
        if(plateau>0){
            auto smooth=[](double a){a=std::clamp(a,0.,1.);return a*a*a*(10+a*(-15+6*a));};
            const double shoulder=1-smooth((std::sqrt(t)-plateau)/(1-plateau));
            const double cliff=cliffCoordinate(x,y);
            const double h=heightMeters*shoulder*(1-smooth(cliff/cliffWidth));
            return backSlope?backSlope->limit(x,y,h):h;
        }
        const double shoulder=1-t*t;
        return heightMeters*shoulder*shoulder*shoulder;
    }
    double vertexHeight(double x,double y) const {
        double h=landformHeight(x,y);for(const auto& ramp:ramps)h=std::max(h,ramp.height(x,y));double mask=1;if(plateau>0){const double u=std::clamp(cliffCoordinate(x,y)/cliffWidth,0.,1.);mask=1-u*u*u*(10+u*(-15+6*u));}for(const auto& knoll:knolls)h=std::max(h,knoll.at(x,y)*mask);h=std::max(h,ridge.at(x,y)*mask);
        for(const auto& foothill:foothills)h=std::max(h,foothill.at(x,y));
        if(ravines.empty())return h;
        double cut=0;for(const auto& ravine:ravines)cut=std::max(cut,ravine.depth(x,y));return std::max(0.,h-cut);
    }
    double height(double x,double y) const {
        if(kind==TerrainKind::Flat)return 0;
        // Exact same two triangles as the visible mesh, including negative cells.
        const double x0=std::floor(x/gridStep)*gridStep,y0=std::floor(y/gridStep)*gridStep;
        const double u=(x-x0)/gridStep,v=(y-y0)/gridStep;
        const double a=vertexHeight(x0,y0),c=vertexHeight(x0+gridStep,y0+gridStep);
        if(u>=v){const double b=vertexHeight(x0+gridStep,y0);return a+(b-a)*u+(c-b)*v;}
        const double b=vertexHeight(x0,y0+gridStep);return a+(c-b)*u+(b-a)*v;
    }
    std::array<double,2> heightRange(double x,double y,double radius) const {
        if(!std::isfinite(x)||!std::isfinite(y)||!std::isfinite(radius)||radius<0)return {-INFINITY,INFINITY};
        if(kind==TerrainKind::Flat)return {0,0};
        std::array<double,2> result{INFINITY,-INFINITY};
        // A square encloses the circular footing/steel footprint. Each visible
        // triangle is affine: corners and diagonal intersections contain all
        // extrema, including a cliff that only falls away from the foundation.
        for(double gx=std::floor((x-radius)/gridStep)*gridStep;gx<=x+radius;gx+=gridStep)
            for(double gy=std::floor((y-radius)/gridStep)*gridStep;gy<=y+radius;gy+=gridStep){
                const double loX=std::max(x-radius,gx),hiX=std::min(x+radius,gx+gridStep),loY=std::max(y-radius,gy),hiY=std::min(y+radius,gy+gridStep);
                auto sample=[&](double px,double py){if(px<loX||px>hiX||py<loY||py>hiY)return;const double h=height(px,py);result[0]=std::min(result[0],h);result[1]=std::max(result[1],h);};
                for(double px:{loX,hiX})for(double py:{loY,hiY})sample(px,py);
                for(double px:{loX,hiX})sample(px,gy+px-gx);
                for(double py:{loY,hiY})sample(gx+py-gy,py);
            }
        return result;
    }
    double slopeBound() const {
        if(!valid())return INFINITY;if(kind==TerrainKind::Flat)return 0;
        const auto ridgeBound=ridge.enclosure();
        double cliffGradient=1/cliffWidth;
        if(cliffCurvature>0){const auto b=gridBounds();double side=0;for(double x:{b[0],b[2]})for(double y:{b[1],b[3]})side=std::max(side,std::abs(-(x-cliffX)*std::sin(cliffHeading)+(y-cliffY)*std::cos(cliffHeading)));cliffGradient=std::hypot(1.,2*cliffCurvature*side)/cliffWidth;}
        double bound=plateau>0?std::sqrt(2.)*1.875*heightMeters*(std::hypot((1+2*std::abs(bend))/radiusX,1/radiusY)/(1-plateau)+cliffGradient):std::sqrt(2.)*12*heightMeters*(.64/std::sqrt(5.))*std::hypot((1+2*std::abs(bend))/radiusX,1/radiusY);
        if(backSlope){
            bound=std::max(bound,std::sqrt(2.)*std::hypot(backSlope->gradeX,backSlope->gradeY));
            // Blend slope <= the larger source slope plus the maximum cut
            // depth times the quintic shoulder's maximum derivative.
            if(backSlope->width>0)bound+=std::sqrt(2.)*1.875*heightMeters/(backSlope->width*.65);
        }
        for(const auto& r:ramps)bound=std::max(bound,std::sqrt(2.)*(std::max(std::abs(r.grade0),std::abs(r.grade1))+1.875*(std::max(r.h0,r.h1)+80*std::max(std::abs(r.grade0),std::abs(r.grade1)))*(1./40+1/(r.width*.65))));for(const auto& k:knolls)bound=std::max(bound,24*k.height/(std::sqrt(5.)*k.radius)+(plateau>0?std::sqrt(2.)*1.875*k.height*cliffGradient:0));bound=std::max(bound,std::sqrt(2.)*(ridgeBound[1]+(plateau>0?1.875*ridgeBound[0]*cliffGradient:0)));
        for(const auto& k:foothills)bound=std::max(bound,24*k.height/(std::sqrt(5.)*k.radius));
        double cutBound=0;for(const auto& ravine:ravines)cutBound=std::max(cutBound,ravine.slopeBound());return bound+std::sqrt(2.)*cutBound;
    }
    double localSlopeBound(double x,double y,double radius) const {
        if(!valid()||!std::isfinite(x)||!std::isfinite(y)||!std::isfinite(radius)||radius<0)return std::numeric_limits<double>::infinity();
        if(kind==TerrainKind::Flat)return 0;
        if(plateau>0||!ramps.empty()||!knolls.empty()||!foothills.empty()||!ridge.points.empty()||!ravines.empty()){
            double bound=0;
            for(double gx=std::floor((x-radius)/gridStep)*gridStep;gx<=x+radius;gx+=gridStep)
                for(double gy=std::floor((y-radius)/gridStep)*gridStep;gy<=y+radius;gy+=gridStep){
                    const double a=vertexHeight(gx,gy),b=vertexHeight(gx+gridStep,gy),c=vertexHeight(gx+gridStep,gy+gridStep),d=vertexHeight(gx,gy+gridStep);
                    bound=std::max({bound,std::hypot(b-a,c-b)/gridStep,std::hypot(c-d,d-a)/gridStep});
                }
            return bound;
        }
        // Enclose every source-grid corner affecting this footprint. The
        // sqrt(2) factor also bounds the gradients of the interpolating faces.
        const double reach=radius+std::sqrt(2.)*gridStep;
        const double u=(x-centerX)/radiusX,v=(y-centerY)/radiusY-bend*u*u;
        const double du=reach/radiusX,dv=reach/radiusY+std::abs(bend)*(2*std::abs(u)*du+du*du);
        const double delta=std::hypot(du,dv),r=std::hypot(u,v),low=std::max(0.,r-delta);
        if(low>=1)return 0;
        const double t0=low*low,t1=std::min(1.,(r+delta)*(r+delta));
        const double t=std::clamp(1/std::sqrt(5.),t0,t1),d=6*heightMeters*t*std::pow(1-t*t,2);
        const double umax=std::min(1.,std::abs(u)+du),vmax=std::min(1.,std::abs(v)+dv);
        return std::sqrt(2.)*d*2*std::hypot((umax+2*std::abs(bend)*umax*vmax)/radiusX,vmax/radiusY);
    }
    std::array<double,4> gridBounds() const {
        // Outside this rectangle every terrain-grid vertex is exactly zero.
        // Curving a cliff only changes its mask. Ravines only subtract and are
        // clamped at the existing basin, so neither expands the positive base.
        const double yRadius=radiusY*(1+std::abs(bend));
        std::array<double,4> bounds{centerX-radiusX,centerY-yRadius,centerX+radiusX,centerY+yRadius};
        if(!ridge.points.empty()){const auto b=ridge.bounds();for(int i=0;i<2;++i){bounds[i]=std::min(bounds[i],b[i]);bounds[i+2]=std::max(bounds[i+2],b[i+2]);}}
        for(const auto& ramp:ramps){const auto b=ramp.bounds();for(int i=0;i<2;++i){bounds[i]=std::min(bounds[i],b[i]);bounds[i+2]=std::max(bounds[i+2],b[i+2]);}}
        for(const auto& k:knolls){bounds[0]=std::min(bounds[0],k.x-k.radius);bounds[1]=std::min(bounds[1],k.y-k.radius);bounds[2]=std::max(bounds[2],k.x+k.radius);bounds[3]=std::max(bounds[3],k.y+k.radius);}
        for(const auto& k:foothills){bounds[0]=std::min(bounds[0],k.x-k.radius);bounds[1]=std::min(bounds[1],k.y-k.radius);bounds[2]=std::max(bounds[2],k.x+k.radius);bounds[3]=std::max(bounds[3],k.y+k.radius);}
        for(int i=0;i<2;++i){bounds[i]=std::floor(bounds[i]/gridStep)*gridStep-gridStep;bounds[i+2]=std::ceil(bounds[i+2]/gridStep)*gridStep+gridStep;}return bounds;
    }
};
}
