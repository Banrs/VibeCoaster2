#pragma once
#include "coaster/coaster.hpp"
#if defined(__aarch64__) || defined(_M_ARM64)
#include <arm_neon.h>
#elif defined(__SSE2__) || defined(_M_X64)
#include <emmintrin.h>
#endif

namespace coaster::detail {
namespace arc_pair {
#if defined(__aarch64__) || defined(_M_ARM64)
using Pair=float64x2_t;
inline Pair pair(double a,double b){const double v[]{a,b};return vld1q_f64(v);}
inline Pair add(Pair a,Pair b){return vaddq_f64(a,b);}
inline Pair mul(Pair a,Pair b){return vmulq_f64(a,b);}
inline Pair root(Pair a){return vsqrtq_f64(a);}
inline double sum(Pair a){return vgetq_lane_f64(a,0)+vgetq_lane_f64(a,1);}
#elif defined(__SSE2__) || defined(_M_X64)
using Pair=__m128d;
inline Pair pair(double a,double b){return _mm_set_pd(b,a);}
inline Pair add(Pair a,Pair b){return _mm_add_pd(a,b);}
inline Pair mul(Pair a,Pair b){return _mm_mul_pd(a,b);}
inline Pair root(Pair a){return _mm_sqrt_pd(a);}
inline double sum(Pair a){return _mm_cvtsd_f64(a)+_mm_cvtsd_f64(_mm_unpackhi_pd(a,a));}
#else
struct Pair {double a,b;};
inline Pair pair(double a,double b){return {a,b};}
inline Pair add(Pair a,Pair b){return {a.a+b.a,a.b+b.b};}
inline Pair mul(Pair a,Pair b){return {a.a*b.a,a.b*b.b};}
inline Pair root(Pair a){return {std::sqrt(a.a),std::sqrt(a.b)};}
inline double sum(Pair a){return a.a+a.b;}
#endif
inline Pair repeat(double a){return pair(a,a);}
}
inline double spanArcLength(const Span& span,double u){
    // Evaluate the same eight Gaussian nodes in pairs. Keep each Horner chain,
    // norm and weighted addition in scalar order; do not fuse multiply/add.
    constexpr double nodes[]={.18343464249564980494,.52553240991632898582,.79666647741362673959,.96028985649753623168};
    constexpr double weights[]={.36268378337836198297,.31370664587788728734,.22238103445337447054,.10122853629037625915};
    std::array<Vec3,9> c;for(size_t i=0;i<c.size();++i)c[i]=span.c[i+1]*double(i+1);
    using namespace arc_pair;
    double result=0;
    for(int i=0;i<4;++i){
        const auto parameter=pair((1-nodes[i])*u*.5,(1+nodes[i])*u*.5);
        auto x=repeat(c.back().x),y=repeat(c.back().y),z=repeat(c.back().z);
        for(int k=int(c.size())-2;k>=0;--k){
            x=add(mul(x,parameter),repeat(c[k].x));
            y=add(mul(y,parameter),repeat(c[k].y));
            z=add(mul(z,parameter),repeat(c[k].z));
        }
        result+=weights[i]*sum(root(add(add(mul(x,x),mul(y,y)),mul(z,z))));
    }
    return result*u*.5;
}
}
