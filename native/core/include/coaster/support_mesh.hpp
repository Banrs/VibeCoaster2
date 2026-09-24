#pragma once
#include "coaster/coaster.hpp"
#include <stdexcept>

namespace coaster {
struct SupportMeshBuffer {std::vector<Vec3> positions,normals;std::vector<uint32_t> indices;};
// Portable CPU adapter: a closed sixteen-sided approximation entirely inside the
// canonical tapered circular solid. UE reflects coordinates/winding once later.
inline SupportMeshBuffer supportMemberMesh(const SupportMember& member){
    const Vec3 delta=member.top-member.base;const double length=norm(delta);
    if(!finite(member.base)||!finite(member.top)||!std::isfinite(member.radiusBase)||!std::isfinite(member.radiusTop)||
       member.radiusBase<=0||member.radiusTop<=0||std::max(member.radiusBase,member.radiusTop)>5||length<.01||length>1000)
        throw std::runtime_error("Invalid canonical support mesh member");
    const Vec3 axis=delta/length;
    const Vec3 u=unit(cross(axis,std::abs(axis.z)<.9?Vec3{0,0,1}:Vec3{0,1,0}));
    const Vec3 v=cross(axis,u);constexpr uint32_t sides=16;
    SupportMeshBuffer mesh;
    auto vertex=[&](Vec3 p,Vec3 n){mesh.positions.push_back(p);mesh.normals.push_back(n);};
    auto triangle=[&](uint32_t a,uint32_t b,uint32_t c){mesh.indices.insert(mesh.indices.end(),{a,b,c});};
    for(int ring=0;ring<2;++ring)for(uint32_t j=0;j<sides;++j){
        const double angle=2*pi*j/sides;Vec3 radial=u*std::cos(angle)+v*std::sin(angle);
        vertex((ring?member.top:member.base)+radial*(ring?member.radiusTop:member.radiusBase),unit(radial+axis*((member.radiusBase-member.radiusTop)/length)));
    }
    for(uint32_t j=0;j<sides;++j){uint32_t next=(j+1)%sides;triangle(j,next,j+sides);triangle(next,next+sides,j+sides);}
    for(int end=0;end<2;++end){
        const uint32_t centre=uint32_t(mesh.positions.size());Vec3 p=end?member.top:member.base,n=axis*(end?1:-1);vertex(p,n);
        for(uint32_t j=0;j<sides;++j){double a=2*pi*j/sides;vertex(p+(u*std::cos(a)+v*std::sin(a))*(end?member.radiusTop:member.radiusBase),n);}
        for(uint32_t j=0;j<sides;++j){uint32_t a=centre+1+j,b=centre+1+(j+1)%sides;if(end)triangle(centre,a,b);else triangle(centre,b,a);}
    }
    return mesh;
}
}
