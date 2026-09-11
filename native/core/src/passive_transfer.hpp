#pragma once
#include "coaster/coaster.hpp"
#include <stdexcept>
namespace coaster::detail {
// Exact affine transport over one interval with constant mean grade and
// nongravitational acceleration. Positive acceleration adds work; drag is
// the coefficient b in dv/dt=-b*v*v. Both planners use the same energy step.
struct PathEnergyStep {double attenuation,offsetSpeedSquared;};
inline PathEnergyStep pathEnergyStep(double length,double heightChange,double acceleration,double drag){
    const double x=2*drag*length,attenuation=std::exp(-x),weight=x>1e-10?-std::expm1(-x)/x:1-x*.5;
    return {attenuation,2*(acceleration*length-gravity*heightChange)*weight};
}
struct PassiveTransferEstimate { bool reached{}; double speed{}; };
struct PassiveTransfer {
    // w_exit = retention*w_entry + offsetSpeedSquared, where w=v^2
    // (twice kinetic energy per unit mass). Traversal requires strictly more
    // than minimumEntrySpeedSquared at every sampled downstream boundary.
    double retention{1},offsetSpeedSquared{},minimumEntrySpeedSquared{-INFINITY};
    PassiveTransferEstimate forward(double entrySpeed) const {
        const double w=entrySpeed*entrySpeed;
        if(!std::isfinite(w)||entrySpeed<0)
            throw std::invalid_argument("Passive transfer speed is outside its finite domain");
        const double exit=retention*w+offsetSpeedSquared;
        if(!std::isfinite(exit)||!(w>minimumEntrySpeedSquared)||(exit<=0&&minimumEntrySpeedSquared!=-INFINITY))return {};
        return {true,std::sqrt(exit)};
    }
    PassiveTransferEstimate inverse(double exitSpeed) const {
        const double w=exitSpeed*exitSpeed;
        if(!std::isfinite(w)||exitSpeed<0)
            throw std::invalid_argument("Passive transfer speed is outside its finite domain");
        const double entry=(w-offsetSpeedSquared)/retention;
        if(!std::isfinite(entry)||entry<0)return {};
        // An unattainable exact exit speed remains unattainable. Do not raise
        // it to disguise an interior stall or excess downhill energy.
        return {entry>minimumEntrySpeedSquared,std::sqrt(entry)};
    }
};
// Planning only: remove every motor/brake and transport observed train energy
// to a later center-distance. Finite-train mean potential is sampled directly;
// rolling resistance and aerodynamic drag match simulate(). No seat-force or
// full-ride acceptance claim is made. One bounded integration in w=v^2 space
// supplies both forward estimates and direct inverse requirements.
inline PassiveTransfer makePassiveTransfer(const Track& track,const TrainConfig& train,
    double begin,double end,Cancel cancel={},double maxStep=2){
    if(!std::isfinite(begin)||!std::isfinite(end)||begin<0||end<begin||end>track.length||
       !std::isfinite(maxStep)||maxStep<.25||maxStep>2||end-begin>100000)
        throw std::invalid_argument("Passive transfer left its distance or step domain");
    if(cancel&&cancel())throw std::runtime_error("CANCELLED");
    auto height=[&](double s){double sum=0,half=(train.cars-1)*train.spacing*.5;
        for(int car=0;car<train.cars;++car)sum+=track.sample(s+half-car*train.spacing).position.z;
        return sum/train.cars;};
    const double drag=train.airDensity*train.dragCdA/(train.cars*train.carMass),rolling=gravity*train.rollingResistance;
    PassiveTransfer out;double previousHeight=height(begin),distance=begin;
    const size_t steps=size_t(std::ceil((end-begin)/maxStep));
    for(size_t i=0;i<steps;++i){
        if(cancel&&cancel())throw std::runtime_error("CANCELLED");
        double next=begin+(end-begin)*(i+1)/steps,ds=next-distance,nextHeight=height(next);
        // Exact drag attenuation; mean potential change is exact at step ends.
        // The source is constant over this short interval, giving second-order
        // convergence for changing grade without differentiating track height.
        const auto step=pathEnergyStep(ds,nextHeight-previousHeight,-rolling,drag*.5);
        out.retention*=step.attenuation;
        out.offsetSpeedSquared=out.offsetSpeedSquared*step.attenuation+step.offsetSpeedSquared;
        if(!(out.retention>0)||!std::isfinite(out.offsetSpeedSquared/out.retention))
            throw std::runtime_error("Passive transfer exceeds its representable energy domain");
        out.minimumEntrySpeedSquared=std::max(out.minimumEntrySpeedSquared,-out.offsetSpeedSquared/out.retention);
        distance=next;previousHeight=nextHeight;
    }
    return out;
}
inline PassiveTransferEstimate estimatePassiveTransfer(const Track& track,const TrainConfig& train,
    double begin,double end,double speed,Cancel cancel={},double maxStep=2){
    return makePassiveTransfer(track,train,begin,end,cancel,maxStep).forward(speed);
}
}
