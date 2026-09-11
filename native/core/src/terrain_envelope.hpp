#pragma once
#include "coaster/coaster.hpp"

namespace coaster::detail {
// Required rigid Z translation for the same oriented frame in planning and
// composition. The final continuous train-body certificate remains separate.
inline double terrainEnvelopeDatum(const Terrain& terrain,const Limits& limits,const TrackSample& frame,bool anyBank=false){
    double datum=-INFINITY;
    const double margin=2*terrain.localSlopeBound(frame.position.x,frame.position.y,8);
    for(double side:{-1.5,1.5})for(double height:{-.8,2.4}){
        const Vec3 corner=frame.position+frame.right*side+frame.up*height;
        datum=std::max(datum,terrain.height(corner.x,corner.y)+limits.minClearance+.3+margin-corner.z);
    }
    if(anyBank){
        const double radius=std::hypot(1.5,2.4),slope=terrain.localSlopeBound(frame.position.x,frame.position.y,radius);
        datum=std::max(datum,terrain.height(frame.position.x,frame.position.y)+limits.minClearance+radius*std::hypot(1.,slope)+.3-frame.position.z);
    }
    return datum;
}
}
