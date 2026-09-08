#include "coaster/coaster.hpp"
namespace coaster {
namespace {
bool mergeable(const Operation& op){
    for(double x:{op.start,op.end,op.targetSpeed,op.maxForce,op.maxPower,op.rampSeconds,op.stopDeceleration,op.stopOffset,op.exitFadeMeters})if(!std::isfinite(x))return false;
    return op.start>=0&&op.end>op.start&&int(op.kind)>=0&&int(op.kind)<=3&&op.targetSpeed>=0&&op.maxForce>=0&&op.maxPower>=0&&op.rampSeconds>=0&&op.stopDeceleration>0&&op.stopDeceleration<=20&&op.stopOffset>=0&&op.stopOffset<=5&&op.exitFadeMeters>=.01&&op.exitFadeMeters<=1000;
}
bool sameProfile(const Operation& a,const Operation& b){
    return a.kind==b.kind&&a.targetSpeed==b.targetSpeed&&a.maxForce==b.maxForce&&a.maxPower==b.maxPower&&a.rampSeconds==b.rampSeconds&&a.stopDeceleration==b.stopDeceleration&&a.stopOffset==b.stopOffset&&a.exitFadeMeters==b.exitFadeMeters;
}
}
void coalesceDriveProfiles(std::vector<Operation>& operations){
    // Authored-order maximal runs only. Never sort, merge overlapping regions,
    // bridge even a one-ULP gap, reinterpret zero-length/wrapping intervals or
    // combine distinct profiles. One physical continuous motor region then has
    // one entry ramp, instead of restarting at a construction-module boundary.
    size_t output=0;
    for(size_t input=0;input<operations.size();++input){
        const auto& next=operations[input];
        if(output&&mergeable(operations[output-1])&&mergeable(next)&&operations[output-1].end==next.start&&sameProfile(operations[output-1],next))operations[output-1].end=next.end;
        else{if(output!=input)operations[output]=next;++output;}
    }
    operations.resize(output);
}
}
