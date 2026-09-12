#pragma once
#include "circuit_layout.hpp"
#include "terrain_transfer.hpp"

namespace coaster::detail {
struct CircuitOccurrence {
    SourceGeometry source;
    double inletHeight{};
};
struct CircuitGeometry {
    struct Interval {size_t first,last;};
    struct Link {Interval turn,straight;size_t workFirst;};
    Track track;
    std::vector<Interval> sources;
    std::vector<Link> links;
};
// Layout intent is evaluated on the actual source and turn paths. This coarse
// planning witness does not certify clearance: composition's shared height
// solve and final canonical vehicle/hardware checks still own acceptance.
inline bool circuitHasCrossing(const std::vector<CircuitOccurrence>& occurrences,const CircuitLayout& layout){
    struct Point {Vec3 position;double distance;};std::vector<Point> points;
    Vec3 cursor{};double distance=0;
    const auto append=[&](Vec3 p,double s,bool endpoint){
        if(points.empty()||endpoint||s-points.back().distance>=10)points.push_back({p,s});
    };
    for(size_t i=0;i<occurrences.size();++i){const auto& source=occurrences[i].source;const double heading=layout.headings[i];
        cursor.z=occurrences[i].inletHeight;const auto base=cursor;
        for(size_t j=0;j<source.points.size();++j){const auto& p=source.points[j];
            append(base+sourceYaw(p.frame.position,heading),distance+p.distance,j+1==source.points.size());}
        cursor=points.back().position;distance+=source.points.back().distance;
        const auto& exit=source.points.back().frame.tangent;const double exitHeading=heading+std::atan2(exit.y,exit.x);
        const auto& turn=layout.turns[i];const auto turnBegin=cursor;
        for(size_t j=1;j<turn.points.size();++j)
            append(turnBegin+sourceYaw(turn.points[j],exitHeading),distance+turn.length*j/(turn.points.size()-1),j+1==turn.points.size());
        cursor=turnBegin+sourceYaw(turn.points.back(),exitHeading);distance+=turn.length;
        cursor=cursor+sourceYaw({layout.straights[i],0,0},exitHeading+turn.angle);distance+=layout.straights[i];
        append(cursor,distance,true);
    }
    for(size_t i=0;i+1<points.size();++i){const auto a=points[i].position,b=points[i+1].position-a;
        for(size_t j=i+2;j+1<points.size();++j){const auto c=points[j].position,e=points[j+1].position-c;
            if(std::max(a.x,a.x+b.x)<std::min(c.x,c.x+e.x)||std::max(c.x,c.x+e.x)<std::min(a.x,a.x+b.x)||
               std::max(a.y,a.y+b.y)<std::min(c.y,c.y+e.y)||std::max(c.y,c.y+e.y)<std::min(a.y,a.y+b.y))continue;
            const double determinant=cross(b,e).z;
            if(determinant==0||std::abs(determinant)<.3*norm(b)*norm(e))continue;
            const double u=cross(c-a,e).z/determinant,v=cross(c-a,b).z/determinant;
            if(u<0||u>=1||v<0||v>=1)continue;
            const double arc=points[j].distance+v*(points[j+1].distance-points[j].distance)-
                points[i].distance-u*(points[i+1].distance-points[i].distance);
            if(std::min(arc,distance-arc)>=1000)return true;
        }
    }
    return false;
}
// Compose the exact same occurrences used for layout. Heights here are the
// authored port heights; the terrain owner later adds source translations and
// its connecting baseline. A source's position/frame data is never refitted.
inline CircuitGeometry composeCircuit(const std::vector<CircuitOccurrence>& occurrences,const CircuitLayout& layout,
    Vec3 origin={},double worldHeading=0,Cancel cancel={}){
    if(occurrences.size()!=layout.headings.size()||!std::isfinite(layout.length))
        throw std::invalid_argument("Circuit composition needs a solved occurrence layout");
    CircuitGeometry result;std::vector<AuthoredPoint> points;
    struct KnownKnot {size_t index;Knot knot;};std::vector<KnownKnot> knownKnots;
    Vec3 cursor=origin;
    const auto append=[&](Vec3 position,double bank,Element element,Vec3 up=Vec3{0,0,1}){
        if(points.empty()||norm(points.back().position-position)>1e-7)points.push_back({position,bank,element,up});
    };
    for(size_t i=0;i<occurrences.size();++i){
        if(cancel&&cancel())throw std::runtime_error("CANCELLED");
        const auto& occurrence=occurrences[i];const double heading=worldHeading+layout.headings[i];
        cursor.z=origin.z+occurrence.inletHeight;const Vec3 base=cursor;
        const size_t first=points.empty()?0:points.size()-1;
        for(const auto& point:occurrence.source.points){const auto frame=placeSourceFrame(point.frame,base,heading);
            append(frame.position,0,frame.element,frame.up);
            knownKnots.push_back({points.size()-1,{frame.position,frame.tangent,frame.curvature,frame.up,0,frame.element}});
        }
        result.sources.push_back({first,points.size()-1});cursor=points.back().position;
        const auto& exit=occurrence.source.points.back().frame;
        const double exitHeading=heading+std::atan2(exit.tangent.y,exit.tangent.x);
        const auto& turn=layout.turns[i];const double straight=layout.straights[i],linkLength=turn.length+straight;
        const Vec3 linkBegin=cursor;
        const double nextHeight=origin.z+occurrences[(i+1)%occurrences.size()].inletHeight;
        // The connection owns actual work and passive intervals. Resolve port
        // height before the physical work rail used by the motor and train.
        const double passiveStraight=straight-layout.workLengths[i];
        const double passiveLength=turn.length+passiveStraight;
        const TerrainTransfer height{cursor.z,nextHeight,passiveLength>0?passiveLength:linkLength};
        if(linkLength==0&&std::abs(cursor.z-nextHeight)>1e-8)
            throw std::runtime_error("Coincident circuit ports disagree in height");
        CircuitGeometry::Link link;link.turn.first=points.size()-1;
        for(size_t j=1;j<turn.points.size();++j){
            // The following coast sample owns the turn-to-straight join. A
            // separate turn endpoint next to a short coast creates a tiny
            // polynomial span even though the physical path is smooth.
            if(passiveStraight>0&&j+1==turn.points.size())continue;
            const double along=turn.length*j/(turn.points.size()-1);
            auto point=linkBegin+sourceYaw(turn.points[j],exitHeading);point.z=height.height(along);
            append(point,turn.bankAt(along),Element::Turn);
        }
        link.turn.last=points.size()-1;link.straight.first=points.size()-1;
        const Vec3 turnEnd=linkBegin+sourceYaw(turn.points.back(),exitHeading);
        // Sample each physical domain on its own uniform partition. Inserting
        // a work boundary into another grid creates almost coincident knots
        // and ill-conditioned derivative estimates at an otherwise level port.
        const auto levelKnot=[&]{knownKnots.push_back({points.size()-1,
            {points.back().position,sourceYaw({1,0,0},exitHeading+turn.angle),{},{0,0,1},0,Element::Return}});};
        for(int domain=0;domain<2;++domain){const double start=domain?passiveStraight:0,length=domain?layout.workLengths[i]:passiveStraight;
            const int count=int(std::ceil(length/1.5));
            for(int j=1;j<=count;++j){const double along=j==count?(domain?straight:passiveStraight):start+length*j/count;
                auto point=turnEnd+sourceYaw({along,0,0},exitHeading+turn.angle);point.z=height.height(turn.length+along);
                append(point,0,Element::Return);if(domain)levelKnot();
                if(!domain&&j==1&&turn.points.size()>1)link.turn.last=link.straight.first=points.size()-1;
            }
            if(!domain){link.workFirst=points.size()-1;levelKnot();}
        }
        link.straight.last=points.size()-1;result.links.push_back(link);cursor=points.back().position;
    }
    if(norm(cursor-points.front().position)>1e-6)throw std::runtime_error("Composed circuit does not close its actual source ports");
    points.back()=points.front();result.track=compile(points);
    for(const auto& known:knownKnots)result.track.knots[known.index]=known.knot;
    result.track.knots.back()=result.track.knots.front();result.track.rebuild();return result;
}
}
