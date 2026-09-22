#pragma once
#include "coaster/math.hpp"
#include <functional>
#include <string>
#include <vector>

namespace coaster {
using Cancel=std::function<bool()>;
struct Cancelled:std::runtime_error {Cancelled():std::runtime_error("Cancelled") {}};
inline void poll(const Cancel& cancel){if(cancel&&cancel())throw Cancelled();}
enum class Role {Launch,Opening,Clifftop,Edge,Lip,Cliff,DownhillLaunch,Camelback,Wave,Loop,Immelmann,Ravine,Journey,Terminal};
const char* roleName(Role role);
struct Control {
    double time{},normal{1},lateral{},twist{},drive{};
    std::array<double,4> first{},second{};
};
struct Twist {double begin{},end{},angle{};};
struct State {Vec3 p{},t{1,0,0},u{0,0,1};double v{4},s{},lossWork{},driveWork{};};
struct Program {
    std::string id,label;
    Role role{Role::Journey};
    State initial;
    std::vector<Control> controls;
    std::vector<Twist> twists;
    bool gravityBank{};
    double rolling{.035},drag{.000035};
    double duration() const {return controls.empty()?0:controls.back().time;}
};
// Derivatives are with respect to physical arc distance in metres.
struct Jet {
    std::array<Vec3,5> position;
    std::array<Vec3,4> up;
    double s{},time{},speed{},drive{};
    std::size_t element{};
};
struct Shot {State end;double maximumHeight{},minimumHeight{},yaw{},pitchTurn{};};
Control controlAt(const Program&,double time);
void validateProgram(const Program&);
State advance(const State&,const Program&,double time,double dt);
Jet jetAt(const State&,const Program&,double time);
Shot shoot(const Program&,double step=.02,const Cancel& cancel={});
std::vector<Jet> replay(const Program&,double step=.02,const Cancel& cancel={});

struct HillShape {
    double height{70},exitHeight{},positive{3.6},negative{-1.25},exitPositive{3.8};
    double riseRamp{1.6},releaseRamp{1.6},recoveryRamp{1.8},exitRelease{1.2};
    double descentNegative{-1.25},bankDegrees{};
};
Program hill(const State&,const HillShape&,const Cancel& cancel={});
Program dive(const State&,double drop,double exitPitch,const Cancel& cancel={});
Program loop(const State&,double height,double yaw,const Cancel& cancel={});
Program immelmann(const State&,double height,double exitHeight,const Cancel& cancel={});
}
