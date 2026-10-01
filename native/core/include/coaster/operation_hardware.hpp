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
        if(track.profile==TrackProfile::Exa){
            // Mount between the centred 1.4 m crossheads. Short modules leave
            // the transverse tube and its longitudinal webs unobstructed.
            constexpr double pitch=1.4,halfLength=.28;
            for(double at=std::ceil((op.start+halfLength)/pitch-.5)*pitch+pitch*.5;at+halfLength<=end;at+=pitch){
                const double s=std::fmod(at,track.length);const auto q=track.sample(s);
                auto box=[&](Vec3 centre,Vec3 half,bool moving=false){out.push_back({index,s,powered,{q.position+q.tangent*centre.x+q.right*centre.y+q.up*centre.z,q.tangent,q.right,q.up,half,StationRole::Post},moving});};
                if(powered){
                    for(double side:{-1.,1.})box({0,side*.34,-.11},{halfLength,.10,.09});
                    box({0,0,-.335},{halfLength,.40,.135});
                }else if(op.kind==DriveKind::Trim){
                    box({0,0,-.08},{halfLength,.01,.14},true);
                    box({0,0,-.31},{halfLength,.10,.16});
                    box({0,.24,-.22},{halfLength*.65,.11,.07});
                }else{
                    for(double side:{-1.,1.})box({0,side*.10,-.045},{halfLength,.05,.105});
                    box({0,0,-.31},{halfLength,.30,.16});
                }
            }
            continue;
        }
        const double spacing=powered?1.5:op.kind==DriveKind::Trim?.9:2.;
        for(double begin=op.start;begin<end;begin+=spacing) {
            const double length=std::min(spacing*.8,end-begin),s=std::fmod(begin+length*.5,track.length);
            const auto q=track.sample(s);
            auto box=[&](Vec3 centre,Vec3 half,bool moving=false){out.push_back({index,s,powered,{q.position+q.tangent*centre.x+q.right*centre.y+q.up*centre.z,q.tangent,q.right,q.up,half,StationRole::Post},moving});};
            if(powered) {
                for(double side:{-1.,1.})box({0,side*.34,-.11},{length*.5,.10,.09});
                box({0,0,-.295},{length*.35,.40,.105}); // Mounting bridge meets the spine and lower stator edges
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
