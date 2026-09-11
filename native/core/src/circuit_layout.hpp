#pragma once
#include "source_geometry.hpp"
#include "turn_shape.hpp"
#include "coaster/force_envelope.hpp"

namespace coaster::detail {
// Derived ports of the actual itinerary, in each source's inlet frame. The
// caller owns source geometry and operation work; this component owns planar
// closure. No particular source count, corridor assignment or turn hand is fixed.
struct CircuitElement {
    Vec3 exit;
    double exitHeading{},length{},turnRadius{},turnRamp{},turnBank{};
    double minimumSpeed{};
};
struct CircuitLayout {
    std::vector<double> headings,straights,workLengths,workEntrySpeeds,minimumSpeeds;
    std::vector<TurnShape> turns;
    double length{INFINITY};
};
struct CircuitLinkLengths {
    double work{},passive{},entrySpeed{};
    CircuitLinkLengths(double workLength,double passiveLength=0,double speed=0):work(workLength),passive(passiveLength),entrySpeed(speed){}
};
inline TurnShape circuitTurn(const CircuitElement& element,double angle){
    if(angle==0){TurnShape turn;turn.points.push_back({});return turn;}
    const double speedRadius=element.turnRadius*std::tan(element.turnBank);double bank=element.turnBank;
    if(element.minimumSpeed>0){
        const auto fits=[&](double proposed){
            // This includes both complete roll ramps, so it bounds the
            // actual turn length from above without another geometry solve.
            const double duration=(2*element.turnRamp+speedRadius*std::abs(angle)/std::tan(proposed))/element.minimumSpeed;
            return 1/std::cos(proposed)<=historicalForceLimit(ForceAxis::Vertical,true,duration);
        };
        if(!fits(bank)){double low=0,high=bank;
            for(int iteration=0;iteration<40;++iteration){const double middle=(low+high)*.5;if(fits(middle))low=middle;else high=middle;}
            bank=low;
        }
    }
    return makeTurn(angle,speedRadius/std::tan(bank),element.turnRamp,bank);
}
// Heading closure is explicit. XY closure is a two-row nonnegative length
// problem, whose minimum has at most two nonzero added lengths. Enumerating
// pairs solves it directly; an independent final composition checks the result.
inline CircuitLayout closeCircuit(const std::vector<CircuitElement>& elements,std::vector<double> headings,
    const std::function<CircuitLinkLengths(size_t,double)>& minimumStraight,const CircuitLayout* previous=nullptr,Cancel cancel={}){
    if(elements.empty()||headings.size()!=elements.size())throw std::invalid_argument("Circuit needs one heading per source");
    CircuitLayout result;result.headings=std::move(headings);result.straights.resize(elements.size());result.workLengths.resize(elements.size());result.workEntrySpeeds.resize(elements.size());result.minimumSpeeds.resize(elements.size());result.turns.resize(elements.size());
    std::vector<Vec3> direction(elements.size());Vec3 residual{};double length=0;
    for(size_t i=0;i<elements.size();++i){
        if(cancel&&cancel())throw std::runtime_error("CANCELLED");
        const auto& element=elements[i];const double next=result.headings[(i+1)%elements.size()];
        result.minimumSpeeds[i]=element.minimumSpeed;
        const double angle=std::remainder(next-result.headings[i]-element.exitHeading,2*pi);
        auto& turn=result.turns[i];
        if(angle==0){turn.points.push_back({});}
        else if(previous&&previous->turns[i].angle==angle)turn=previous->turns[i];
        else turn=circuitTurn(element,angle);
        const auto required=minimumStraight(i,turn.length);const double straight=required.work+required.passive;
        if(!std::isfinite(straight)||straight<0)return result;
        result.straights[i]=straight;result.workLengths[i]=required.work;result.workEntrySpeeds[i]=required.entrySpeed;direction[i]={std::cos(next),std::sin(next),0};
        residual=residual+sourceYaw(element.exit,result.headings[i])+sourceYaw(turn.points.back(),result.headings[i]+element.exitHeading)+direction[i]*straight;
        length+=element.length+turn.length+straight;
    }
    residual.z=0;
    double added=INFINITY;size_t first=0,second=0;double firstLength=0,secondLength=0;
    if(norm(residual)<1e-8)added=0;
    // Collinear directions can have a rank-one solution. Do not demand a
    // nonsingular pair when one positive straight closes the circuit exactly.
    for(size_t i=0;i<elements.size();++i){const double extra=-dot(residual,direction[i]);
        if(extra>=0&&extra<added&&norm(residual+direction[i]*extra)<1e-8){added=extra;first=i;second=i;firstLength=extra;secondLength=0;}}
    for(size_t i=0;i<elements.size();++i)for(size_t j=i+1;j<elements.size();++j){
        const double determinant=cross(direction[i],direction[j]).z;
        if(std::abs(determinant)<1e-12)continue;
        const double a=cross(residual*(-1),direction[j]).z/determinant,b=cross(direction[i],residual*(-1)).z/determinant;
        if(a>=0&&b>=0&&a+b<added){added=a+b;first=i;second=j;firstLength=a;secondLength=b;}
    }
    if(!std::isfinite(added))return result;
    result.straights[first]+=firstLength;result.straights[second]+=secondLength;
    const auto closed=residual+direction[first]*firstLength+direction[second]*secondLength;
    if(norm(closed)>1e-6)throw std::runtime_error("Circuit length solve did not close its actual ports");
    result.length=length+added;return result;
}
inline CircuitLayout solveCircuitLayout(const std::vector<CircuitElement>& elements,std::vector<double> headings,
    const std::function<CircuitLinkLengths(size_t,double)>& minimumStraight,Cancel cancel={}){
    auto best=closeCircuit(elements,std::move(headings),minimumStraight,nullptr,cancel);
    // One bounded solve moves every intermediate inlet heading while reclosing
    // the entire circuit. Cost includes turns and physical work domains, so a
    // shorter closure straight cannot conceal an unnecessarily long turn.
    for(double step:{pi,pi/2,.4,.2,.1,.05})for(int sweep=0;sweep<2;++sweep){
        bool changed=false;
        for(size_t i=1;i<elements.size();++i){auto winner=best;
            for(double sign:{-1.,1.}){auto proposed=best.headings;proposed[i]+=sign*step;
                if(step==pi&&sign>0)continue;
                auto trial=closeCircuit(elements,std::move(proposed),minimumStraight,&best,cancel);
                if(trial.length<winner.length-1e-6)winner=std::move(trial);
            }
            if(winner.length<best.length-1e-6){best=std::move(winner);changed=true;}
        }
        if(!changed)break;
    }
    return best;
}
}
