#pragma once
#include "coaster/coaster.hpp"

namespace coaster {
// Prototype track hardware contract: SI, rail-midpoint datum, X forward/Y right/Z up.
// Two closed rectangular diagonal prisms, repeated at the existing 3 m tie stations.
// Validation conservatively sweeps both prisms continuously along the canonical track.
inline std::array<StationBox,2> trackWebsLocal() {
    std::array<StationBox,2> out{};
    for(int i=0;i<2;++i){
        const double side=i==0?-1.:1.;
        const Vec3 a{0,side*.09,-.43},b{0,side*.65,-.19};
        const Vec3 forward{1,0,0},right=unit(b-a),up=cross(forward,right);
        out[i]={(a+b)*.5,forward,right,up,{.06,norm(b-a)*.5,.04},StationRole::Post};
    }
    return out;
}
inline StationBox trackWebWorld(const StationBox& local,const TrackSample& q) {
    const auto v=[&](Vec3 x){return q.tangent*x.x+q.right*x.y+q.up*x.z;};
    return {q.position+v(local.center),v(local.forward),v(local.right),v(local.up),local.half,local.role};
}
inline std::array<Vec3,8> trackWebCorners(const StationBox& b) {
    std::array<Vec3,8> out{};int i=0;
    for(double x:{-1.,1.})for(double y:{-1.,1.})for(double z:{-1.,1.})
        out[i++]=b.center+b.forward*(x*b.half.x)+b.right*(y*b.half.y)+b.up*(z*b.half.z);
    return out;
}
// Exact piecewise-quadratic segment/AABB distance. Breakpoints are crossings of
// the six box faces. Each interval has a fixed active set; minimize its quadratic.
// This avoids the diagonal-corner inflation of testing an axis-expanded OBB.
inline double segmentWebDistanceSquared(Vec3 a,Vec3 b,const StationBox& box) {
    const auto local=[&](Vec3 p){p=p-box.center;return Vec3{dot(p,box.forward),dot(p,box.right),dot(p,box.up)};};
    a=local(a);b=local(b);const Vec3 delta=b-a;
    const double aa[]{a.x,a.y,a.z},dd[]{delta.x,delta.y,delta.z},hh[]{box.half.x,box.half.y,box.half.z};
    std::array<double,8> breaks{};size_t count=2;breaks[0]=0;breaks[1]=1;
    for(int k=0;k<3;++k)if(std::abs(dd[k])>1e-15)for(double sign:{-1.,1.}){
        const double t=(sign*hh[k]-aa[k])/dd[k];if(t>0&&t<1)breaks[count++]=t;
    }
    std::sort(breaks.begin(),breaks.begin()+count);
    const auto distance=[&](double t){double sum=0;for(int k=0;k<3;++k){const double d=std::max(0.,std::abs(aa[k]+t*dd[k])-hh[k]);sum+=d*d;}return sum;};
    double best=std::min(distance(0),distance(1));
    for(size_t i=1;i<count;++i){const double lo=breaks[i-1],hi=breaks[i],mid=(lo+hi)*.5;double A=0,B=0;
        best=std::min({best,distance(lo),distance(hi)});
        for(int k=0;k<3;++k){const double p=aa[k]+mid*dd[k];if(std::abs(p)>hh[k]){const double offset=aa[k]-(p>0?hh[k]:-hh[k]);A+=dd[k]*dd[k];B+=dd[k]*offset;}}
        if(A>0)best=std::min(best,distance(std::clamp(-B/A,lo,hi)));
    }
    return best;
}
}
