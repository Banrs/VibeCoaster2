#pragma once
#include "coaster/support_mesh.hpp"
namespace coaster {
// Exa circular rails/spine. Keep canonical frame samples and metre dimensions
// identical in offline review and runtime adapters.
inline SupportMeshBuffer trackTubeMesh(const Track& track,double begin,double end,double side,double height,double radius){
    if(!std::isfinite(begin)||!std::isfinite(end)||begin<0||end<=begin||end>track.length+1e-6||radius<=0)
        throw std::runtime_error("Invalid track mesh interval");
    constexpr uint32_t sides=32;
    const uint32_t count=uint32_t(std::ceil((end-begin)/.5))+1;
    const double wall=side==0?.024:.02;const uint32_t layer=count*sides;
    SupportMeshBuffer out;out.positions.reserve(2*layer);out.normals.reserve(2*layer);
    for(uint32_t shell=0;shell<2;++shell)for(uint32_t i=0;i<count;++i){const auto q=track.sample(begin+(end-begin)*i/(count-1));
        const Vec3 centre=q.position+q.right*side+q.up*height;
        for(uint32_t j=0;j<sides;++j){const double angle=2*pi*j/sides;const Vec3 normal=q.right*std::cos(angle)+q.up*std::sin(angle);
            out.positions.push_back(centre+normal*(radius-shell*wall));out.normals.push_back(normal*(shell?-1:1));
            if(i+1<count){const uint32_t a=shell*layer+i*sides+j,b=a+sides,c=shell*layer+i*sides+(j+1)%sides,d=c+sides;
                if(shell)out.indices.insert(out.indices.end(),{a,c,b,c,d,b});else out.indices.insert(out.indices.end(),{a,b,c,c,b,d});}
        }
    }
    // Only cap actual open track ends, never create overlapping annuli at
    // renderer chunk seams or at the seam of a closed ride.
    if(!track.closed)for(uint32_t endIndex:{0u,count-1})if((endIndex==0&&begin==0)||(endIndex==count-1&&end==track.length)){
        const uint32_t base=endIndex*sides;
        for(uint32_t j=0;j<sides;++j){const uint32_t a=base+j,b=base+(j+1)%sides,c=a+layer,d=b+layer;
            if(endIndex==0)out.indices.insert(out.indices.end(),{a,b,c,b,d,c});else out.indices.insert(out.indices.end(),{a,c,b,b,c,d});}
    }
    return out;
}
}
