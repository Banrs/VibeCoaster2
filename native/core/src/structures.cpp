#include "coaster/coaster.hpp"
#include "coaster/clearance.hpp"
#include <stdexcept>

namespace coaster {
bool memberSeparatedFromBox(const SupportMember& member,const StationBox& box,double padding){
    const Vec3 delta=member.top-member.base;const double length=norm(delta);if(length<=1e-12)return false;
    const Vec3 axis=delta/length;const Vec3 axes[]{box.forward,box.right,box.up};const double half[]{box.half.x,box.half.y,box.half.z};
    auto separated=[&](Vec3 direction){
        const double size=norm(direction);if(size<1e-12)return false;direction=direction/size;
        const double radial=norm(cross(direction,axis));
        const double a=dot(member.base-box.center,direction),b=dot(member.top-box.center,direction);
        const double low=std::min(a-member.radiusBase*radial,b-member.radiusTop*radial),high=std::max(a+member.radiusBase*radial,b+member.radiusTop*radial);
        double extent=padding;for(int i=0;i<3;++i)extent+=half[i]*std::abs(dot(direction,axes[i]));
        return low>extent||high< -extent;
    };
    // Any separating projection is a proof of disjoint solids. In addition
    // to box faces, test the member axis and edge cross-products: a short
    // vertical footing beside an inclined car often separates only here.
    if(separated(axis))return true;
    for(auto direction:axes)if(separated(direction)||separated(cross(axis,direction)))return true;
    // Gilbert search supplies additional separating directions for a round
    // footing beside an OBB corner. Only an explicit projection gap accepts
    // separation; non-convergence remains a conservative collision candidate.
    Vec3 nearest=(member.base+member.top)*.5-box.center;
    for(int iteration=0;iteration<32;++iteration){
        if(separated(nearest))return true;
        const Vec3 direction=nearest*-1,radial=direction-axis*dot(direction,axis);const double radialSize=norm(radial);
        const bool top=dot(member.top-member.base,direction)+(member.radiusTop-member.radiusBase)*radialSize>0;
        Vec3 point=(top?member.top:member.base)+(radialSize>1e-12?radial*((top?member.radiusTop:member.radiusBase)/radialSize):Vec3{});
        Vec3 corner=box.center;for(int i=0;i<3;++i)corner=corner+axes[i]*(dot(nearest,axes[i])>=0?half[i]:-half[i]);
        const Vec3 next=point-corner,step=next-nearest;const double squared=dot(step,step);
        if(squared<1e-20)break;
        const double fraction=std::clamp(-dot(nearest,step)/squared,0.,1.);if(fraction<1e-12)break;
        nearest=nearest+step*fraction;
    }
    return false;
}
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
            if(memberSeparatedFromBox(m,box))continue;
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
static ValidationReport validateDesignStructuresImpl(const Design& d,const ClearanceSweep* prepared,Cancel cancel){
    ValidationReport out;
    if(cancel&&cancel()){out.fail("CANCELLED","Structure validation cancelled");return out;}
    // Earlier saves retain their original canonical interpolation on load.
    if(!supportedGeneratorVersion(d.generationVersion)){out.fail("GENERATOR_VERSION","Unsupported VibeCoaster2 generation provenance");return out;}
    if(!d.station.enabled||d.supports.empty()||std::any_of(d.supports.begin(),d.supports.end(),[](const Support& s){return s.members.empty();})){
        out.fail("REQUIRED_STRUCTURE","New geometry designs require canonical station and explicit support members");return out;
    }
    out=prepared?validateStation(d.track,d.request.terrain,d.request.train,d.station,*prepared,cancel):validateStation(d.track,d.request.terrain,d.request.train,d.station,cancel);
    if(!out.valid())return out;
    try{for(const auto& support:d.supports){
        if(supportStationCollision(support,d.station,cancel)){
            out.fail("STATION_SUPPORT_CLEARANCE","Cannot certify clearance between canonical support solids and station parts",support.trackDistance);
            if(out.errors.size()>=8)return out;
        }
    }}catch(const std::exception& e){out.fail(std::string(e.what())=="CANCELLED"?"CANCELLED":"STRUCTURE_CONFIG",e.what());}
    return out;
}
ValidationReport validateDesignStructures(const Design& d,Cancel cancel){return validateDesignStructuresImpl(d,nullptr,cancel);}
ValidationReport validateDesignStructures(const Design& d,const ClearanceSweep& sweep,Cancel cancel){return validateDesignStructuresImpl(d,&sweep,cancel);}
}
