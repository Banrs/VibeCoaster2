#pragma once
#include "coaster/coaster.hpp"
#include <stdexcept>
namespace coaster::detail {
struct PassiveTransferEstimate { bool reached{}; double speed{}; };
// Planning only: remove every motor/brake and transport observed train energy
// to a later center-distance. Finite-train mean potential is sampled directly;
// rolling resistance and aerodynamic drag match simulate(). No seat-force or
// full-ride acceptance claim is made. Losses are integrated in w=v^2 space.
inline PassiveTransferEstimate estimatePassiveTransfer(const Track& track,const TrainConfig& train,
    double begin,double end,double speed,Cancel cancel={},double maxStep=2){
    if(!std::isfinite(begin)||!std::isfinite(end)||!std::isfinite(speed)||begin<0||end<begin||end>track.length||speed<0||
       !std::isfinite(maxStep)||maxStep<.25||maxStep>2||end-begin>100000)
        throw std::invalid_argument("Passive transfer left its distance, speed or step domain");
    auto height=[&](double s){double sum=0,half=(train.cars-1)*train.spacing*.5;
        for(int car=0;car<train.cars;++car)sum+=track.sample(s+half-car*train.spacing).position.z;
        return sum/train.cars;};
    const double drag=train.airDensity*train.dragCdA/(train.cars*train.carMass),rolling=gravity*train.rollingResistance;
    double w=speed*speed,previousHeight=height(begin),distance=begin;
    const size_t steps=size_t(std::ceil((end-begin)/maxStep));
    for(size_t i=0;i<steps;++i){
        if(cancel&&cancel())throw std::runtime_error("CANCELLED");
        double next=begin+(end-begin)*(i+1)/steps,ds=next-distance,nextHeight=height(next);
        // Exact drag attenuation; mean potential change is exact at step ends.
        // The source is constant over this short interval, giving second-order
        // convergence for changing grade without differentiating track height.
        double x=drag*ds,attenuation=std::exp(-x),weight=x>1e-10?-std::expm1(-x)/x:1-x*.5;
        w=w*attenuation-2*(gravity*(nextHeight-previousHeight)+rolling*ds)*weight;
        if(!(w>0))return {false,0};
        distance=next;previousHeight=nextHeight;
    }
    return {true,std::sqrt(w)};
}
}
