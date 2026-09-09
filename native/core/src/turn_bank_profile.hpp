#pragma once
#include "flow_bridge.hpp"

namespace coaster::detail {
// Author one bounded continuous bank trajectory from signed force demand.
// Four untouched knots at each run boundary preserve its canonical frame jets.
// This is a centerline planning fit; finite-train replay remains authoritative.
inline bool fitTurnBankProfile(Track& track,size_t first,size_t last,const std::vector<double>& speed){
    if(last<=first+8||last>track.spans.size()||speed.size()!=track.spans.size())return false;
    const size_t a=first+3,b=last-3;
    const double start=track.spans[a].start,length=track.spans[b].start-start;
    if(!(length>0))return false;
    auto jet=[&](size_t i){const auto& span=track.spans[i];double q=norm(span.c[1]),qu=dot(span.c[1],span.c[2]*2)/q;
        return std::array<double,3>{span.bank[0],span.bank[1]/q,2*span.bank[2]/(q*q)-span.bank[1]*qu/(q*q*q)};};
    const auto left=jet(a),right=jet(b);
    const auto base=flowFramePolynomial(left[0],left[1],left[2],right[0],right[1],right[2],length);
    struct Point{size_t index;double base,bump,normal,lateral,weight;};
    std::vector<Point> points;points.reserve(b-a-1);
    double lower=-1.5,upper=1.5;
    for(size_t i=a+1;i<b;++i){
        const double u=(track.spans[i].start-start)/length,bank=flowValue(base,u),bump=std::pow(std::sin(pi*u),4);
        const auto& k=track.knots[i];Vec3 up=unit(k.up-k.tangent*dot(k.up,k.tangent));
        Vec3 required=k.curvature*(speed[i]*speed[i])+Vec3{0,0,gravity};
        points.push_back({i,bank,bump,dot(required,up),dot(required,cross(k.tangent,up)),track.spans[i].length/std::max(1.,speed[i])});
        if(bump>1e-12){lower=std::max(lower,(-1.5-bank)/bump);upper=std::min(upper,(1.5-bank)/bump);}
        else if(std::abs(bank)>1.5)return false;
    }
    if(points.empty()||lower>upper)return false;
    auto cost=[&](double amplitude){double result=0;for(const auto& p:points){double bank=p.base+amplitude*p.bump;
        double lateral=p.lateral*std::cos(bank)-p.normal*std::sin(bank);result+=p.weight*lateral*lateral;}return result;};
    // A short global bracket avoids assuming a single minimum when normal
    // load changes sign; refine only the best bounded interval.
    constexpr int brackets=24;double best=lower,bestCost=cost(best);int bestIndex=0;
    for(int i=1;i<=brackets;++i){double value=lower+(upper-lower)*i/brackets,c=cost(value);if(c<bestCost){best=value;bestCost=c;bestIndex=i;}}
    double lo=lower+(upper-lower)*std::max(0,bestIndex-1)/brackets,hi=lower+(upper-lower)*std::min(brackets,bestIndex+1)/brackets;
    for(int i=0;i<20;++i){double x=lo+(hi-lo)/3,y=hi-(hi-lo)/3;if(cost(x)<cost(y))hi=y;else lo=x;}
    double candidate=(lo+hi)*.5;if(cost(candidate)<bestCost)best=candidate;
    for(const auto& p:points)track.knots[p.index].bank=p.base+best*p.bump;
    track.rebuild();return true;
}
}
