#pragma once
#include "coaster/coaster.hpp"
#include <optional>

namespace coaster::detail {
inline Vec3 sourceYaw(Vec3 value,double heading){
    const double c=std::cos(heading),s=std::sin(heading);
    return {c*value.x-s*value.y,s*value.x+c*value.y,value.z};
}
inline TrackSample placeSourceFrame(TrackSample frame,Vec3 origin,double heading){
    frame.position=origin+sourceYaw(frame.position,heading);
    frame.tangent=sourceYaw(frame.tangent,heading);frame.curvature=sourceYaw(frame.curvature,heading);
    frame.up=sourceYaw(frame.up,heading);frame.right=sourceYaw(frame.right,heading);
    return frame;
}
// Sample a physical source once. Terrain planning and circuit emission consume
// these same frames; a routing polyline never becomes a second source model.
struct SourceGeometry {
    struct Point {double distance;TrackSample frame;};
    std::vector<Point> points;
    SourceGeometry(const Track& track,std::optional<Element> element={},std::vector<double> landmarks={}){
        landmarks.push_back(0);landmarks.push_back(track.length);
        std::sort(landmarks.begin(),landmarks.end());
        landmarks.erase(std::unique(landmarks.begin(),landmarks.end()),landmarks.end());
        const auto append=[&](double distance){auto frame=track.sample(distance);if(element)frame.element=*element;points.push_back({distance,frame});};
        append(0);
        for(size_t interval=1;interval<landmarks.size();++interval){
            const double begin=landmarks[interval-1],end=landmarks[interval];const int count=int(std::ceil((end-begin)/1.5));
            for(int i=1;i<=count;++i)append(i==count?end:begin+(end-begin)*i/count);
        }
    }
};
}
