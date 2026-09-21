#include "coaster/coaster.hpp"
#include "simulation_internal.hpp"
#include <stdexcept>
namespace coaster {
DriveIndex::DriveIndex(const std::vector<Operation>& operations,double length){
    if(!std::isfinite(length)||length<=0)throw std::invalid_argument("Drive index requires positive finite track length");
    if(length/cellWidth>65536||length/cellWidth>1000000./std::max(size_t(1),operations.size()))cellWidth=length;
    cells.resize(size_t(std::ceil(length/cellWidth)));
    for(size_t cell=0;cell<cells.size();++cell){const double begin=cell*cellWidth,end=std::min(length,begin+cellWidth);
        for(size_t j=0;j<operations.size();++j){const auto& op=operations[j];
            if(op.start<=op.end?(end>=op.start&&begin<=op.end):(end>=op.start||begin<=op.end))cells[cell].push_back(j);
        }
    }
}
const std::vector<size_t>& DriveIndex::at(double distance) const {return cells[std::min(cells.size()-1,size_t(distance/cellWidth))];}
bool validDriveParameters(const Operation& op){
    for(double x:{op.start,op.end,op.targetSpeed,op.maxForce,op.maxPower,op.rampSeconds,op.stopDeceleration,op.stopOffset,op.exitFadeMeters})if(!std::isfinite(x))return false;
    if(!std::isfinite(op.trimPeakSpeed)||!std::isfinite(op.trimSensorLead))return false;
    if(op.kind==DriveKind::Trim) {
        if(op.start>=op.end||op.trimPeakSpeed<=0||op.trimPeakSpeed>150||op.trimSensorLead<=0||op.trimSensorLead>op.start||op.maxPower<2*op.maxForce*op.trimPeakSpeed||op.rampSeconds<.05||2*op.exitFadeMeters>op.end-op.start)return false;
    } else if(op.trimPeakSpeed!=0||op.trimSensorLead!=0)return false;
    return op.start>=0&&op.end>=0&&int(op.kind)>=0&&int(op.kind)<=4&&op.targetSpeed>=0&&op.maxForce>=0&&op.maxPower>=0&&op.rampSeconds>=0&&op.stopDeceleration>0&&op.stopDeceleration<=20&&op.stopOffset>=0&&op.stopOffset<=5&&op.exitFadeMeters>=.01&&op.exitFadeMeters<=1000;
}
namespace {
bool mergeable(const Operation& op){return validDriveParameters(op)&&op.kind!=DriveKind::Trim&&op.end>op.start;}
bool sameProfile(const Operation& a,const Operation& b){
    return a.kind==b.kind&&a.targetSpeed==b.targetSpeed&&a.maxForce==b.maxForce&&a.maxPower==b.maxPower&&a.rampSeconds==b.rampSeconds&&a.stopDeceleration==b.stopDeceleration&&a.stopOffset==b.stopOffset&&a.exitFadeMeters==b.exitFadeMeters;
}
}
void coalesceDriveProfiles(std::vector<Operation>& operations){
    // Only identical contiguous nonwrapping runs coalesce, preserving authored order.
    // Each resulting motor region has one entry ramp.
    size_t output=0;
    for(size_t input=0;input<operations.size();++input){
        const auto& next=operations[input];
        if(output&&mergeable(operations[output-1])&&mergeable(next)&&operations[output-1].end==next.start&&sameProfile(operations[output-1],next))operations[output-1].end=next.end;
        else{if(output!=input)operations[output]=next;++output;}
    }
    operations.resize(output);
}
}
