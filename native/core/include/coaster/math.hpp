#pragma once
#include <algorithm>
#include <cmath>

namespace coaster {
constexpr double pi=3.14159265358979323846, gravity=9.80665;
struct Vec3 {
    double x{},y{},z{};
    Vec3 operator+(Vec3 b) const { return {x+b.x,y+b.y,z+b.z}; }
    Vec3 operator-(Vec3 b) const { return {x-b.x,y-b.y,z-b.z}; }
    Vec3 operator*(double a) const { return {x*a,y*a,z*a}; }
    Vec3 operator/(double a) const { return *this*(1/a); }
};
inline double dot(Vec3 a,Vec3 b){return a.x*b.x+a.y*b.y+a.z*b.z;}
inline Vec3 cross(Vec3 a,Vec3 b){return {a.y*b.z-a.z*b.y,a.z*b.x-a.x*b.z,a.x*b.y-a.y*b.x};}
inline double norm(Vec3 a){return std::sqrt(dot(a,a));}
inline Vec3 unit(Vec3 a){double n=norm(a);return n>1e-12?a/n:Vec3{};}
inline bool finite(Vec3 a){return std::isfinite(a.x)&&std::isfinite(a.y)&&std::isfinite(a.z);}
inline Vec3 rotate(Vec3 v,Vec3 axis,double angle){return v*std::cos(angle)+cross(axis,v)*std::sin(angle)+axis*(dot(axis,v)*(1-std::cos(angle)));}
inline double smooth(double u){u=std::clamp(u,0.,1.);return u*u*u*(10+u*(-15+6*u));}

}
