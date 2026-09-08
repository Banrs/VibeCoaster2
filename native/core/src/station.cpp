#include "coaster/coaster.hpp"
#include "coaster/track_hardware.hpp"
#include <iomanip>
#include <locale>
#include <sstream>
#include <stdexcept>
#include <deque>

namespace coaster {
namespace station_validation {
// Internal kernel shared by construction, validation and focused domain tests.
// Norm is convex, so the four projected corners enclose the whole footprint,
// including persisted axes within the accepted near-orthonormal tolerance.
double footprintRadius(Vec3 forward,Vec3 right,double halfX,double halfY){
    double radius=0;
    for(double x:{-halfX,halfX})for(double y:{-halfY,halfY}){
        const Vec3 delta=forward*x+right*y;radius=std::max(radius,std::hypot(delta.x,delta.y));
    }
    return radius;
}
}
namespace {
std::array<Vec3,3> axes(const StationBox& b) { return {b.forward,b.right,b.up}; }
std::array<double,3> extents(const StationBox& b) { return {b.half.x,b.half.y,b.half.z}; }
double projectedRadius(const StationBox& b, Vec3 axis) {
    auto a=axes(b); auto h=extents(b);
    return h[0]*std::abs(dot(a[0],axis))+h[1]*std::abs(dot(a[1],axis))+h[2]*std::abs(dot(a[2],axis));
}
StationBox expanded(StationBox b, double margin) { b.half=b.half+Vec3{margin,margin,margin}; return b; }

ValidationReport stationTrackDomain(const Track& track,const Terrain& terrain,const TrainConfig& train,Cancel cancel) {
    ValidationReport out;
    if(cancel&&cancel()){out.fail("CANCELLED","Station validation cancelled");return out;}
    if(track.spans.empty()||track.spans.size()>199999||track.knots.size()!=track.spans.size()+1||track.knots.size()<4||!track.closed||!std::isfinite(track.length)||track.length<=0||track.length>150000||train.cars<1||train.cars>16||!std::isfinite(train.spacing)||train.spacing<=0||train.spacing>20||!std::isfinite(train.seatHeight)||train.seatHeight<0||train.seatHeight>3||!terrain.valid()){out.fail("STATION_CONFIG","Invalid station track/train/terrain domain");return out;}
    try {
        // A station must inspect the same canonical curve the renderer sees.
        // Rebuild into a temporary, then reject missing or stale cached spans.
        Track canonical=track;canonical.rebuild();
        if(cancel&&cancel()){out.fail("CANCELLED","Station validation cancelled");return out;}
        if(canonical.length!=track.length)throw std::runtime_error("Stale canonical arc length");
        const auto& first=track.knots.front();const auto& last=track.knots.back();
        if(norm(first.position-last.position)>.001||dot(unit(first.tangent),unit(last.tangent))<std::cos(.05*pi/180)||dot(unit(first.up),unit(last.up))<std::cos(.05*pi/180)||norm(first.curvature-last.curvature)>1e-6||std::abs(first.bank-last.bank)>1e-6)throw std::runtime_error("Station circuit seam is discontinuous");
        for(size_t i=0;i<track.spans.size();++i){
            if((i&255)==0&&cancel&&cancel()){out.fail("CANCELLED","Station validation cancelled");return out;}
            const auto& a=track.spans[i];const auto& b=canonical.spans[i];
            if(a.start!=b.start||a.length!=b.length)throw std::runtime_error("Stale canonical span range");
            for(size_t k=0;k<a.c.size();++k)if(!finite(a.c[k])||norm(a.c[k]-b.c[k])!=0)throw std::runtime_error("Stale canonical span coefficients");
            for(size_t k=0;k<a.referenceUp.size();++k)if(!finite(a.referenceUp[k])||norm(a.referenceUp[k]-b.referenceUp[k])!=0||!std::isfinite(a.bank[k])||a.bank[k]!=b.bank[k])throw std::runtime_error("Stale canonical frame coefficients");
            double angle=std::atan2(norm(cross(unit(track.knots[i].up),unit(track.knots[i+1].up))),std::clamp(dot(unit(track.knots[i].up),unit(track.knots[i+1].up)),-1.,1.));
            if(std::abs(track.knots[i+1].bank-track.knots[i].bank)/a.length>.12||angle/a.length>.12||angle>.2)throw std::runtime_error("Unsupported canonical station frame rate");
        }
    }catch(const std::exception& e){out.fail("STATION_CONFIG",e.what());}
    return out;
}

}
bool stationBoxesOverlap(const StationBox& a, const StationBox& b) {
    const auto aa=axes(a),bb=axes(b); const Vec3 delta=b.center-a.center;
    auto separates=[&](Vec3 axis) {
        const double n=norm(axis); if(n<1e-8) return false;
        return std::abs(dot(delta,axis))>projectedRadius(a,axis)+projectedRadius(b,axis)+1e-9*n;
    };
    for(Vec3 axis:aa) if(separates(axis)) return false;
    for(Vec3 axis:bb) if(separates(axis)) return false;
    for(Vec3 x:aa) for(Vec3 y:bb) if(separates(cross(x,y))) return false;
    return true;
}
ValidationReport validateStationDefinition(const StationGeometry& station,Cancel cancel) {
    ValidationReport out;
    if(cancel&&cancel()){out.fail("CANCELLED","Station definition cancelled");return out;}
    if(!std::isfinite(station.boardingBegin)||!std::isfinite(station.boardingEnd)) {out.fail("STATION_CONFIG","Nonfinite station bounds");return out;}
    if(!station.enabled) {
        if(!station.boxes.empty()) out.fail("STATION_CONFIG","Disabled station has geometry");
        return out;
    }
    if(!std::isfinite(station.boardingBegin)||!std::isfinite(station.boardingEnd)||
       station.boardingBegin>=0||station.boardingEnd<=0||station.boardingEnd-station.boardingBegin>600||
       station.boxes.size()<4||station.boxes.size()>512) {
        out.fail("STATION_CONFIG","Station bounds or part count are invalid"); return out;
    }
    std::array<int,5> roles{};
    size_t checked=0;for(const auto& b:station.boxes) {
        if((checked++&63)==0&&cancel&&cancel()){out.fail("CANCELLED","Station definition cancelled");return out;}
        bool valid=finite(b.center)&&norm(b.center)<1000000&&finite(b.half)&&
            b.half.x>0&&b.half.y>0&&b.half.z>0&&b.half.x<=512&&b.half.y<=512&&b.half.z<=512;
        auto a=axes(b);
        for(Vec3 axis:a) valid=valid&&finite(axis)&&std::abs(norm(axis)-1)<1e-6;
        valid=valid&&std::abs(dot(b.forward,b.right))<1e-6&&std::abs(dot(b.forward,b.up))<1e-6&&
            std::abs(dot(b.right,b.up))<1e-6&&dot(cross(b.forward,b.up),b.right)>1-1e-6;
        int role=int(b.role); valid=valid&&role>=0&&role<5;
        if(!valid) { out.fail("STATION_CONFIG","Invalid station part frame, dimensions or role"); return out; }
        ++roles[role];
    }
    for(int count:roles) if(count==0) {out.fail("STATION_CONFIG","Station is missing a required structural role");return out;}
    std::vector<bool> reached(station.boxes.size()); reached[0]=true;
    std::deque<size_t> queue{0};size_t tested=0;
    // Bounded breadth-first traversal: each reached part is expanded once.
    while(!queue.empty()) {
        size_t i=queue.front();queue.pop_front();
        for(size_t j=0;j<station.boxes.size();++j){
            if((tested++&127)==0&&cancel&&cancel()){out.fail("CANCELLED","Station connectivity cancelled");return out;}
            if(!reached[j]&&stationBoxesOverlap(expanded(station.boxes[i],1e-6),station.boxes[j])){reached[j]=true;queue.push_back(j);}
        }
    }
    if(std::find(reached.begin(),reached.end(),false)!=reached.end()) out.fail("STATION_CONNECTION","Station contains a disconnected part");
    return out;
}
StationGeometry buildStation(const Track& track,const Terrain& terrain,const TrainConfig& train,Cancel cancel) {
    auto domain=stationTrackDomain(track,terrain,train,cancel);if(!domain.valid())throw std::runtime_error(domain.errors.front().code+": "+domain.errors.front().message);
    const auto sample=track.sample(0); const Vec3 u{0,0,1};
    if(std::abs(sample.tangent.z)>.005||dot(sample.up,u)<.999) throw std::runtime_error("Station requires an upright level start");
    const Vec3 f=unit(Vec3{sample.tangent.x,sample.tangent.y,0}),r=cross(f,u),origin=sample.position;
    StationGeometry station; station.enabled=true;
    const double half=(train.cars-1)*train.spacing*.5;
    station.boardingBegin=-std::max(18.,half+8.); station.boardingEnd=std::max(64.,2*half+38.);
    const double mid=(station.boardingBegin+station.boardingEnd)*.5,len=station.boardingEnd-station.boardingBegin;
    auto add=[&](double x,double y,double z,Vec3 size,StationRole role) {
        station.boxes.push_back({origin+f*x+r*y+u*z,f,r,u,size,role});
    };
    for(double side:{-1.,1.}) add(mid,side*3.275,-.4,{len*.5,1.925,.4},StationRole::Platform);
    add(mid,0,5.4,{len*.5+1,5.65,.18},StationRole::Canopy);
    const int count=std::max(2,int(std::ceil((len-8)/16))+1);
    for(int i=0;i<count;++i) {
        if(cancel&&cancel())throw std::runtime_error("CANCELLED: Station construction cancelled");
        const double x=station.boardingBegin+4+(len-8)*i/(count-1);
        for(double side:{-1.,1.}) {
            const double y=side*4.8;
            add(x,y,2.61,{.2,.2,2.61},StationRole::Post);
            Vec3 anchor=origin+f*x+r*y;
            double low=terrain.height(anchor.x,anchor.y),high=low;
            // Match the foundation certificate's grid and enclose unsampled
            // terrain between vertices using the full profile gradient bound.
            constexpr double footingStep=1.8/8;
            for(int ix=0;ix<=8;++ix)for(int iy=0;iy<=8;++iy) {
                Vec3 p=anchor+f*(-.9+ix*footingStep)+r*(-.9+iy*footingStep);double h=terrain.height(p.x,p.y);low=std::min(low,h);high=std::max(high,h);
            }
            const double footprintRadius=station_validation::footprintRadius(f,r,.9,.9);
            const double enclosure=terrain.localSlopeBound(anchor.x,anchor.y,footprintRadius)*.500005*std::hypot(footingStep,footingStep);
            const double bottom=low-.4-enclosure,top=high+.1+enclosure,platformBottom=origin.z-.8;
            if(top>=platformBottom-.1) throw std::runtime_error("Terrain leaves no station pier clearance");
            add(x,y,(bottom+top)*.5-origin.z,{.9,.9,(top-bottom)*.5},StationRole::Footing);
            add(x,y,(top+platformBottom)*.5-origin.z,{.3,.3,(platformBottom-top)*.5},StationRole::Pier);
        }
    }
    return station;
}
ValidationReport validateStation(const Track& track,const Terrain& terrain,const TrainConfig& train,const StationGeometry& station,Cancel cancel) {
    auto out=validateStationDefinition(station,cancel); if(!out.valid()||!station.enabled) return out;
    out=stationTrackDomain(track,terrain,train,cancel);if(!out.valid())return out;
    auto stationStart=track.sample(0);
    if(std::abs(stationStart.tangent.z)>.005||dot(stationStart.up,Vec3{0,0,1})<.999){out.fail("STATION_BAY","Station start must be upright and level");return out;}
    const double trainHalf=(train.cars-1)*train.spacing*.5;
    if(station.boardingBegin>-trainHalf-1.4||station.boardingEnd<2*trainHalf+31.4) {
        out.fail("STATION_BAY","Station does not contain the initial and returned train");return out;
    }
    // A remote, collision-free building cannot impersonate the boarding station.
    for(double distance:{-trainHalf,trainHalf,30.,2*trainHalf+30.}) {
        auto q=track.sample(distance); bool left=false,right=false;
        for(const auto& platform:station.boxes) if(platform.role==StationRole::Platform) {
            Vec3 delta=platform.center-q.position;
            double edge=std::abs(dot(delta,q.right))-projectedRadius(platform,q.right);
            bool covers=std::abs(dot(delta,platform.forward))<=platform.half.x-1.3 &&
                std::abs((platform.center+platform.up*platform.half.z-q.position).z)<.5 && edge>.9 && edge<2.;
            if(covers) {if(dot(delta,q.right)<0)left=true;else right=true;}
        }
        if(!left||!right) {out.fail("STATION_BAY","Boarding platforms do not flank both parked train positions",distance);return out;}
    }
    size_t foundationSamples=0;constexpr size_t foundationBudget=2000000;
    // Global terrain gradient bounds certify the unsampled footprint between
    // grid vertices. Generated .9 m footings use only 81 vertices each.
    for(const auto& box:station.boxes) if(box.role==StationRole::Footing) {
        // Use persisted axes themselves: the accepted orthonormality tolerance
        // must not make the local query smaller than the actual footprint.
        const double footprintRadius=station_validation::footprintRadius(box.forward,box.right,box.half.x,box.half.y);
        const double terrainSlope=terrain.localSlopeBound(box.center.x,box.center.y,footprintRadius);
        if(cancel&&cancel()){out.fail("CANCELLED","Station foundation validation cancelled");return out;}
        if(box.up.x!=0||box.up.y!=0||box.up.z!=1) {out.fail("STATION_FOUNDATION","Footing must use the exact world-vertical axis for certified terrain enclosure");return out;}
        const int nx=terrainSlope==0?1:int(std::ceil(2*box.half.x/.25)),ny=terrainSlope==0?1:int(std::ceil(2*box.half.y/.25));
        const size_t needed=size_t(nx+1)*size_t(ny+1);
        if(needed>foundationBudget-foundationSamples){out.fail("STATION_FOUNDATION_DOMAIN","Station footprint exceeds the bounded terrain sampling budget");return out;}
        const double sx=2*box.half.x/nx,sy=2*box.half.y/ny;
        const double unsampled=terrainSlope*.500005*std::hypot(sx,sy)+(terrainSlope==0?0:.5*(std::abs(box.forward.z)*sx+std::abs(box.right.z)*sy));
        for(int ix=0;ix<=nx;++ix)for(int iy=0;iy<=ny;++iy){
            if((foundationSamples++&63)==0&&cancel&&cancel()){out.fail("CANCELLED","Station foundation validation cancelled");return out;}
            Vec3 p=box.center+box.forward*(-box.half.x+sx*ix)+box.right*(-box.half.y+sy*iy);
            const double ground=terrain.height(p.x,p.y),verticalHalf=box.half.z*box.up.z;
            if(p.z-verticalHalf>ground+.02-unsampled||p.z+verticalHalf<ground+.02+unsampled){out.fail("STATION_FOUNDATION","Footing does not conservatively enclose its whole terrain footprint");return out;}
        }
    }

    // Car centres traverse the whole closed curve. Shared restricted-polynomial
    // bounds certify each cell has arc width <=.04 m and angular variation
    // <=.08 rad. Every body/rider point lies within 4.2 m of its origin,
    // including seatHeight <=3 m plus .6 m headroom. Midpoint displacement
    // is <=.02+.04*4.2=.188 m < the .20 m pad. The new quintic frame uses its
    // actual polynomial bounds; no sampled-rate or old nlerp assumption applies.
    const double riderBottom=.4,riderTop=std::max(2.4,train.seatHeight+.6);
    try {
        const auto sweep=buildClearanceSweep(track,train,cancel);size_t scan=0;
        for(const auto& frame:sweep.frames()){
            if((scan++&127)==0&&cancel&&cancel()){out.fail("CANCELLED","Station validation cancelled");return out;}
            const auto& q=frame.sample;const double s=frame.distance;
            std::array<StationBox,3> trainBoxes{{
                {q.position,q.tangent,q.right,q.up,{1.4,.95,.3},StationRole::Post},
                {q.position+q.up*.35,q.tangent,q.right,q.up,{1.275,.85,.225},StationRole::Post},
                {q.position+q.up*((riderBottom+riderTop)*.5),q.tangent,q.right,q.up,{1.3,1.5,(riderTop-riderBottom)*.5},StationRole::Post}
            }};
            for(auto& b:trainBoxes)b=expanded(b,sweep.padding());
            // Rail, tie and spine corners all lie <.9 m from the canonical origin.
            // True cell midpoint motion <=.02*(1+2*.9)=.056 m; .06 encloses it.
            std::array<StationBox,6> hardware{{
                {q.position-q.up*spineDepth,q.tangent,q.right,q.up,{spineRadius,spineRadius,spineRadius},StationRole::Post},
                {q.position-q.right*.65,q.tangent,q.right,q.up,{.085,.085,.085},StationRole::Post},
                {q.position+q.right*.65,q.tangent,q.right,q.up,{.085,.085,.085},StationRole::Post},
                {q.position-q.up*.19,q.tangent,q.right,q.up,{.07,.825,.08},StationRole::Post},
                trackWebWorld(trackWebsLocal()[0],q),trackWebWorld(trackWebsLocal()[1],q)
            }};
            for(auto& b:hardware)b=expanded(b,.06);

            for(size_t j=0;j<station.boxes.size();++j){
                const auto& obstacle=station.boxes[j];
                if(norm(q.position-obstacle.center)>norm(obstacle.half)+4.6)continue;
                for(const auto& part:hardware)if(stationBoxesOverlap(part,obstacle)){
                    out.fail("STATION_HARDWARE_CLEARANCE","Cannot certify canonical track hardware clearance from a station part",s,double(j),0);
                    if(out.errors.size()>=8)return out;break;
                }
                for(const auto& car:trainBoxes)if(stationBoxesOverlap(car,obstacle)){
                    out.fail("STATION_CLEARANCE","Cannot certify swept train clearance from a canonical station part",s,double(j),0);
                    if(out.errors.size()>=8)return out;break;
                }
            }
        }
    }catch(const std::exception& e){const std::string message=e.what();out.fail(message=="CANCELLED"?"CANCELLED":"STATION_CONFIG",message);}
    return out;
}
std::string stationPayload(const StationGeometry& station,Cancel cancel) {
    auto checked=validateStationDefinition(station,cancel);if(!checked.valid()) throw std::runtime_error(checked.errors.front().message);
    std::ostringstream out;out.imbue(std::locale::classic());out<<std::setprecision(17);
    out<<1<<' '<<station.enabled<<' '<<station.boardingBegin<<' '<<station.boardingEnd<<' '<<station.boxes.size()<<'\n';
    for(const auto& b:station.boxes) {
        out<<int(b.role);
        for(Vec3 v:{b.center,b.forward,b.right,b.up,b.half})out<<' '<<v.x<<' '<<v.y<<' '<<v.z;
        out<<'\n';
    }
    return out.str();
}
bool parseStationPayload(const std::string& text,StationGeometry& destination,std::string& error,Cancel cancel) {
    if(cancel&&cancel()){error="CANCELLED";return false;}
    if(text.size()>150000) {error="Station payload exceeds bound";return false;}
    std::istringstream in(text);in.imbue(std::locale::classic());int version=0,enabled=0;size_t n=0;StationGeometry value;
    if(!(in>>version>>enabled>>value.boardingBegin>>value.boardingEnd>>n)||version!=1||(enabled!=0&&enabled!=1)||n>512) {error="Invalid station payload header";return false;}
    value.enabled=enabled!=0;value.boxes.resize(n);
    for(auto& b:value.boxes) {
        if(cancel&&cancel()){error="CANCELLED";return false;}
        int role=-1;if(!(in>>role)) {error="Truncated station payload";return false;}b.role=static_cast<StationRole>(role);
        for(Vec3* v:{&b.center,&b.forward,&b.right,&b.up,&b.half}) if(!(in>>v->x>>v->y>>v->z)) {error="Truncated station frame";return false;}
    }
    in>>std::ws;if(!in.eof()) {error="Trailing station payload tokens";return false;}
    auto checked=validateStationDefinition(value,cancel);if(!checked.valid()) {error=checked.errors.front().code=="CANCELLED"?"CANCELLED":checked.errors.front().message;return false;}
    destination=std::move(value);error.clear();return true;
}
}

