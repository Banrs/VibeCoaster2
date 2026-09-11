#pragma once
#include "coaster/coaster.hpp"
#include <stdexcept>

namespace coaster::detail {
struct TurnShape {
    double angle{},radius{},ramp{},length{},peakBank{};
    std::vector<Vec3> points;
    double bankAt(double s) const{return -std::copysign(peakBank*smooth(std::min(s,length-s)/ramp),angle);}
    double curvatureAt(double s) const{
        const double u=smooth(std::min(s,length-s)/ramp);
        return std::tan(peakBank*u)/(radius*std::tan(peakBank));
    }
};
inline TurnShape makeTurn(double angle,double radius,double ramp,double peakBank){
    if(!std::isfinite(peakBank)||peakBank<=0||peakBank>1.5)throw std::invalid_argument("Turn bank intent is outside its bounded domain");
    const double sign=angle<0?-1.:1.;double magnitude=std::abs(angle);
    TurnShape t;t.angle=angle;t.radius=radius;t.ramp=ramp;t.peakBank=peakBank;
    // Integrate the curvature implied by smooth bank intent. Integrating a
    // curvature ramp first and then taking atan concentrates roll acceleration.
    auto integral=[&](double u){
        constexpr double nodes[]{.1834346424956498,.5255324099163290,.7966664774136267,.9602898564975363};
        constexpr double weights[]{.3626837833783620,.3137066458778873,.2223810344533745,.1012285362903763};
        double result=0;const double half=u/16,denominator=std::tan(peakBank);
        for(int part=0;part<8;++part){double middle=(part+.5)*u/8;
            for(int k=0;k<4;++k)for(double side:{-1.,1.})result+=half*weights[k]*std::tan(peakBank*smooth(middle+side*half*nodes[k]))/denominator;
        }return result;
    };
    // Short turns cannot reach an arbitrary peak load while retaining the
    // requested roll duration. Solve their attainable peak at the same speed;
    // shortening the ramp instead would silently concentrate rider loads.
    const double speedRadius=radius*std::tan(peakBank);
    if(radius*magnitude<2*t.ramp*integral(1)){
        double low=0,high=peakBank;
        for(int i=0;i<48;++i){peakBank=(low+high)*.5;radius=speedRadius/std::tan(peakBank);
            if(radius*magnitude<2*t.ramp*integral(1))high=peakBank;else low=peakBank;}
        peakBank=low;radius=speedRadius/std::tan(peakBank);t.peakBank=peakBank;t.radius=radius;
    }
    const double rampIntegral=integral(1);
    const double plateau=radius*magnitude-2*t.ramp*rampIntegral;
    if(plateau<0)throw std::invalid_argument("Turn bank ramps exceed the requested heading");
    t.length=2*t.ramp+plateau;
    auto theta=[&](double s){if(s<t.ramp)return t.ramp/radius*integral(s/t.ramp);if(s>t.length-t.ramp)return magnitude-t.ramp/radius*integral((t.length-s)/t.ramp);return (s-t.ramp+t.ramp*rampIntegral)/radius;};
    int n=int(std::ceil(t.length/1.8));double ds=t.length/n;Vec3 cursor{};t.points.push_back(cursor);
    for(int i=1;i<=n;++i){double h=sign*theta((i-.5)*ds);cursor.x+=ds*std::cos(h);cursor.y+=ds*std::sin(h);t.points.push_back(cursor);}return t;
}
}
