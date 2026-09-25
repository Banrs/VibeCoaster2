#pragma once
#include "motion_program.hpp"
#include "coaster/fvd.hpp"

namespace coaster {
double replayValueAt(const std::vector<Frame>&,double distance,bool time=false);
// Owns the one live motion state. Named sections describe intent; they do not
// reset pitch, heading, curvature, speed or higher derivatives.
class MotionBuilder {
public:
    using Range = std::pair<size_t,size_t>;
    struct Module {size_t begin,end;std::string name;int reversals{};bool planar{};};
    struct Motor {size_t begin,end;DriveKind kind;double speed,acceleration;double rampSeconds{},exitFadeMeters{};};
    Design& d;
    const GenerationRequest& req;
    Cancel cancel;
    double datum,drag,nominalSpeed{};
    detail::MotionJet cursor;
    std::vector<Module> modules;
    std::vector<Motor> motors;

    explicit MotionBuilder(Design&,Cancel = {});
    // Uses a source-owned or finalized physical frame, never a spline placeholder.
    FvdEntry fvdEntry(double speed,FvdDriveJet drive={}) const;
    double coastEnergy(double squaredSpeed,Vec3 from,Vec3 to) const;
    void append(const detail::MotionJet&,Element,Vec3 up={0,0,1},double bank=0);
    Range curve(detail::MotionJet end,double length,Element,const char* name,double releaseBank=0);
    Range force(const FvdResult&,const FvdRequest&,Element,const char* name,Vec3 origin,double heading=0,double begin=0,double end=-1);
    Range programme(const MotionProgram&,Element,const char* name,double releaseBank=0);
    MotionProgram heightMotion(double height,detail::AngleJet endPitch,detail::AngleJet endHeading) const;
    Range heightCurve(double height,detail::AngleJet endPitch,detail::AngleJet endHeading,Element,const char* name);
    Range pitchToHeight(double height,detail::AngleJet endPitch,Element,const char* name);
    Range line(double length,Element,const char* name);
    void drive(double length,DriveKind,double speed,double acceleration,const char* name);
    void driveGraded(double endHeight,double endGrade,double speed,double acceleration,const char* name,double blendEntrySpeed=0);
    double crestCurvature(double rise,double length,double targetG);
};
}
