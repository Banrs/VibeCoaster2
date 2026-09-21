#pragma once
#include "coaster/coaster.hpp"

namespace coaster {
struct OperationHardware {size_t operation;double distance;bool powered;StationBox box;bool movingFin{};};
// Segmented assemblies follow the canonical frame, including graded motors.
// Cross-sections stay inside the already reserved track/train corridor.
// Deployed hardware tops stay at +6 cm, below the imported body at +10 cm.
inline std::vector<OperationHardware> buildOperationHardware(const Track& track,const std::vector<Operation>& operations) {
    std::vector<OperationHardware> out;
    for(size_t index=0;index<operations.size();++index) {
        const auto& op=operations[index];const bool powered=op.kind==DriveKind::Launch||op.kind==DriveKind::Boost;
        const double end=op.end<op.start?track.length+op.end:op.end;
        const double spacing=powered?1.5:op.kind==DriveKind::Trim?.9:2.;
        for(double begin=op.start;begin<end;begin+=spacing) {
            const double length=std::min(spacing*.8,end-begin),s=std::fmod(begin+length*.5,track.length);
            const auto q=track.sample(s);
            auto box=[&](Vec3 centre,Vec3 half,bool moving=false){out.push_back({index,s,powered,{q.position+q.tangent*centre.x+q.right*centre.y+q.up*centre.z,q.tangent,q.right,q.up,half,StationRole::Post},moving});};
            if(powered) {
                for(double side:{-1.,1.})box({0,side*.34,-.11},{length*.5,.10,.09});
                box({0,0,-.27},{length*.35,.40,.055});
            } else if(op.kind==DriveKind::Trim) {
                box({0,0,-.08},{length*.5,.01,.14},true); // conductive fin
                box({0,0,-.28},{length*.30,.10,.13}); // spine mounting bracket
                box({0,.24,-.22},{length*.20,.11,.07}); // retracting actuator housing
            } else {
                for(double side:{-1.,1.})box({0,side*.10,-.045},{length*.5,.05,.105}); // friction jaws
                box({0,0,-.28},{length*.30,.30,.13});
            }
        }
    }
    return out;
}
}
