#include "coaster/coaster.hpp"
#include <stdexcept>

namespace coaster {
bool supportStationCollision(const Support& support,const StationGeometry& station,Cancel cancel){
    if(cancel&&cancel())throw std::runtime_error("CANCELLED");
    if(!station.enabled)return false;
    std::vector<SupportMember> legacy;
    if(support.members.empty()){
        legacy.push_back({support.base,support.top,supportRadius,supportRadius,SupportMemberKind::Steel,false});
        if(support.hasAttachment)legacy.push_back({support.top,support.attachment,supportRadius,supportRadius,SupportMemberKind::Steel,true});
    }
    const auto& members=support.members.empty()?legacy:support.members;
    for(const auto& m:members){
        if(cancel&&cancel())throw std::runtime_error("CANCELLED");
        double radius=std::max(m.radiusBase,m.radiusTop);
        for(const auto& box:station.boxes){
            // The maximum-radius capsule encloses every tapered solid and footing.
            // Expanding all OBB axes conservatively encloses its Minkowski sum.
            auto local=[&](Vec3 v){v=v-box.center;return Vec3{dot(v,box.forward),dot(v,box.right),dot(v,box.up)};};
            Vec3 a=local(m.base),b=local(m.top),d=b-a,h=box.half+Vec3{radius,radius,radius};
            const double aa[]={a.x,a.y,a.z},dd[]={d.x,d.y,d.z},hh[]={h.x,h.y,h.z};
            double begin=0,end=1;bool overlaps=true;
            for(int axis=0;axis<3;++axis){
                if(std::abs(dd[axis])<1e-12){if(aa[axis]<-hh[axis]||aa[axis]>hh[axis]){overlaps=false;break;}}
                else{double x=(-hh[axis]-aa[axis])/dd[axis],y=(hh[axis]-aa[axis])/dd[axis];if(x>y)std::swap(x,y);begin=std::max(begin,x);end=std::min(end,y);if(begin>end){overlaps=false;break;}}
            }
            if(overlaps)return true;
        }
    }
    return false;
}
ValidationReport validateDesignStructures(const Design& d,Cancel cancel){
    ValidationReport out;
    if(cancel&&cancel()){out.fail("CANCELLED","Structure validation cancelled");return out;}
    // Original 0.8.0 and 0.8.1 saves share the same canonical physics/structure semantics.
    if(d.generationVersion!=generatorVersion&&d.generationVersion!="0.8.0-immelmann.2"&&d.generationVersion!="0.8.1-linear.1"){out.fail("GENERATOR_VERSION","Unsupported generation provenance for canonical geometry schema5");return out;}
    if(!d.station.enabled||d.supports.empty()||std::any_of(d.supports.begin(),d.supports.end(),[](const Support& s){return s.members.empty();})){
        out.fail("REQUIRED_STRUCTURE","New geometry designs require canonical station and explicit support members");return out;
    }
    out=validateStation(d.track,d.request.terrain,d.request.train,d.station,cancel);
    if(!out.valid())return out;
    try{for(const auto& support:d.supports){
        if(supportStationCollision(support,d.station,cancel)){
            out.fail("STATION_SUPPORT_CLEARANCE","Cannot certify clearance between canonical support solids and station parts",support.trackDistance);
            if(out.errors.size()>=8)return out;
        }
    }}catch(const std::exception& e){out.fail(std::string(e.what())=="CANCELLED"?"CANCELLED":"STRUCTURE_CONFIG",e.what());}
    return out;
}
}
