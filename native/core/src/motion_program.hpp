#pragma once
#include "motion_spline.hpp"

namespace coaster {
struct MotionIntent {
    double entrySpeed{},rollingAcceleration{},dragCoefficient{},outwardRoll{};
    double maxNormalG{4.6},minNormalG{-1.2};
};
// Smooth controls in true arc length. Endpoint derivatives are inherited;
// ordered pitch controls prevent extra extrema inside ordinary motion.
struct MotionProgram {
    static constexpr int degree=7,controlCount=32;
    detail::MotionJet begin,end;
    double length{};
    std::array<double,controlCount> pitch{},heading{};
    detail::MotionJet direction(double distance) const;
    Vec3 displacement(double endDistance) const;
};
MotionProgram polynomialMotion(const detail::MotionJet& begin,const std::array<double,8>& pitch,const std::array<double,8>& heading,double length);
MotionProgram solveMotion(const detail::MotionJet& begin,const detail::MotionJet& end,double estimatedLength,const MotionIntent&);
// Inherits direction jets while allowing the integrated displacement to place the next element.
MotionProgram solveHeightMotion(const detail::MotionJet&,const detail::MotionJet&,double,const MotionIntent&);
MotionProgram solveDirectionMotion(const detail::MotionJet&,const detail::MotionJet&,double,const MotionIntent&);
}
