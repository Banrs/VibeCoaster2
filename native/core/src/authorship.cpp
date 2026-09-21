#include "coaster/fvd.hpp"
#include "authoring.hpp"
#include "motion_program.hpp"
#include <iomanip>
#include <sstream>

namespace coaster {
namespace {
void vec(std::ostream& o,Vec3 v){o<<v.x<<' '<<v.y<<' '<<v.z<<' ';}
void vec(std::istream& i,Vec3& v){i>>v.x>>v.y>>v.z;}
Vec3 transform(Vec3 p,double heading,double hand){return {p.x*std::cos(heading)-p.y*std::sin(heading),hand*(p.x*std::sin(heading)+p.y*std::cos(heading)),p.z};}
}
std::string authorshipPayload(const Design& d) {
    std::ostringstream o;o.imbue(std::locale::classic());o<<std::setprecision(17)<<d.forcePrograms.size()<<' '<<d.splinePrograms.size()<<'\n';
    for(const auto& source:d.forcePrograms) {
        const auto& p=source.program;
        o<<std::quoted(source.name)<<' '<<source.firstKnot<<' ';vec(o,source.origin);
        o<<source.heading<<' '<<source.hand<<' '<<source.laneShift<<' '<<source.laneLength<<'\n';
        vec(o,p.position);vec(o,p.forward);vec(o,p.up);
        o<<p.speed<<' '<<p.step<<' '<<p.rollingAcceleration<<' '<<p.dragAccelerationCoefficient<<' '<<p.maxSamples<<' '
         <<p.forceToleranceG<<' '<<p.rollToleranceRadPerSecond<<' '<<p.replayDistanceTolerance<<' '<<p.replaySpeedTolerance<<'\n';
        o<<p.controls.size()<<' '<<p.twists.size()<<' '<<p.chapters.size()<<' '<<source.sourceDistances.size()<<'\n';
        for(const auto& c:p.controls){o<<c.time<<' '<<c.normalG<<' '<<c.lateralG<<' '<<c.rollRate<<' '<<c.drive<<' ';for(double x:c.first)o<<x<<' ';for(double x:c.second)o<<x<<' ';o<<'\n';}
        for(const auto& t:p.twists)o<<t.begin<<' '<<t.end<<' '<<t.angle<<'\n';
        for(const auto& c:p.chapters)o<<c.first<<' '<<std::quoted(c.second)<<'\n';
        for(double x:source.sourceDistances)o<<x<<' ';o<<'\n';
    }
    for(const auto& source:d.splinePrograms) {
        o<<std::quoted(source.name)<<' '<<source.firstKnot<<' '<<source.lastKnot<<' ';vec(o,source.origin);
        o<<source.length<<' '<<source.entrySpeed<<' '<<source.rollingAcceleration<<' '<<source.dragCoefficient<<' '<<source.hand<<' '<<source.requestedRoll<<'\n';
        for(const auto& channel:{source.pitch,source.heading}){for(double x:channel)o<<x<<' ';o<<'\n';}
    }
    const size_t banks=std::count_if(d.forcePrograms.begin(),d.forcePrograms.end(),[](const auto& f){return f.program.gravityReferencedRoll;});
    if(banks){o<<"BANK_REFERENCE "<<banks;for(size_t i=0;i<d.forcePrograms.size();++i)if(d.forcePrograms[i].program.gravityReferencedRoll)o<<' '<<i;o<<'\n';}
    return o.str();
}
bool parseAuthorshipPayload(const std::string& bytes,Design& d,std::string& error) {
    std::istringstream in(bytes);in.imbue(std::locale::classic());size_t forces=0,splines=0;in>>forces>>splines;
    if(!in||forces>64||splines>512){error="Invalid authoring programme counts";return false;}
    std::vector<ForceAuthoring> f(forces);std::vector<SplineAuthoring> s(splines);
    for(auto& source:f) {
        auto& p=source.program;
        in>>std::quoted(source.name)>>source.firstKnot;vec(in,source.origin);
        in>>source.heading>>source.hand>>source.laneShift>>source.laneLength;
        vec(in,p.position);vec(in,p.forward);vec(in,p.up);
        in>>p.speed>>p.step>>p.rollingAcceleration>>p.dragAccelerationCoefficient>>p.maxSamples
          >>p.forceToleranceG>>p.rollToleranceRadPerSecond>>p.replayDistanceTolerance>>p.replaySpeedTolerance;
        size_t controls=0,twists=0,chapters=0,distances=0;in>>controls>>twists>>chapters>>distances;
        if(!in||!validIdentifier(source.name,128)||controls<2||controls>128||twists>32||chapters>64||distances<2||distances>20000){error="Invalid force programme";return false;}
        p.controls.resize(controls);p.twists.resize(twists);p.chapters.resize(chapters);source.sourceDistances.resize(distances);
        for(auto& c:p.controls){in>>c.time>>c.normalG>>c.lateralG>>c.rollRate>>c.drive;for(double& x:c.first)in>>x;for(double& x:c.second)in>>x;}
        for(auto& t:p.twists)in>>t.begin>>t.end>>t.angle;
        for(auto& c:p.chapters)in>>c.first>>std::quoted(c.second);
        for(double& x:source.sourceDistances)in>>x;
    }
    for(auto& source:s) {
        in>>std::quoted(source.name)>>source.firstKnot>>source.lastKnot;vec(in,source.origin);
        in>>source.length>>source.entrySpeed>>source.rollingAcceleration>>source.dragCoefficient>>source.hand>>source.requestedRoll;
        for(auto* channel:{&source.pitch,&source.heading})for(double& x:*channel)in>>x;
        if(!validIdentifier(source.name,128)){error="Invalid spline programme name";return false;}
    }
    if(!in){error="Truncated authoring programme";return false;}in>>std::ws;
    if(!in.eof()){
        std::string tag;size_t count=0;in>>tag>>count;
        if(!in||tag!="BANK_REFERENCE"||count==0||count>forces){error="Invalid authoring bank reference";return false;}
        for(size_t i=0;i<count;++i){size_t index=forces;in>>index;
            if(!in||index>=forces||f[index].program.gravityReferencedRoll){error="Invalid or duplicate bank reference";return false;}
            f[index].program.gravityReferencedRoll=true;}
        in>>std::ws;if(!in.eof()){error="Trailing authoring programme data";return false;}
    }
    d.forcePrograms=std::move(f);d.splinePrograms=std::move(s);return true;
}
void assessAuthorship(Design& d,Cancel cancel) {
    d.authorship={};auto& a=d.authorship;
    if(!d.simulation.completed)return;
    if(d.forcePrograms.empty()||d.splinePrograms.empty()){d.report.fail("AUTHORING_MISSING","V2 requires retained force and spline source programmes");return;}
    const auto errors=d.report.errors.size();a.performed=true;
    for(const auto& source:d.forcePrograms) {
        if(cancel&&cancel()){d.simulation.cancelled=true;return;}
        if(source.sourceDistances.size()<2||source.firstKnot>=d.track.knots.size()||source.sourceDistances.size()>d.track.knots.size()-source.firstKnot||!finite(source.origin)||!std::isfinite(source.heading)||std::abs(source.hand)!=1||!std::isfinite(source.laneShift)||!std::isfinite(source.laneLength)||(source.laneShift!=0&&source.laneLength<=0)) {d.report.fail("AUTHORING_RANGE","Invalid force source mapping");continue;}
        const auto rebuilt=designFvdSection(source.program,cancel);
        if(!rebuilt.assessment.passed){d.report.fail("AUTHORING_SOURCE","Retained force programme failed its independent canonical replay");continue;}
        double previous=-1;
        for(size_t i=0;i<source.sourceDistances.size();++i) {
            const double at=source.sourceDistances[i];
            if(!std::isfinite(at)||at<=previous||at<0||at>rebuilt.track.length+1e-6){d.report.fail("AUTHORING_RANGE","Force source distances must progress within the source");break;}previous=at;

            const auto point=rebuilt.track.sample(std::min(at,rebuilt.track.length));auto expected=point.position;
            if(source.laneShift){double u=std::clamp(at/source.laneLength,0.,1.);expected.y+=source.laneShift*u*u*u*u*(35+u*(-84+u*(70-20*u)));}
            expected=transform(expected,source.heading,source.hand)+Vec3{source.origin.x,source.origin.y*source.hand,source.origin.z};
            const size_t index=source.firstKnot+i;const double s=index<d.track.spans.size()?d.track.spans[index].start:d.track.length;
            a.maximumSourcePositionError=std::max(a.maximumSourcePositionError,norm(expected-d.track.knots[index].position));
            const auto actual=d.track.sample(s);const double speed=replayValueAt(d.simulation.frames,s);
            const double acceleration=-gravity*actual.tangent.z-source.program.rollingAcceleration-source.program.dragAccelerationCoefficient*speed*speed;
            const auto force=measureSeatForces(d.track,s,speed,acceleration,0);
            auto hi=std::lower_bound(rebuilt.samples.begin(),rebuilt.samples.end(),at,[](const FvdSample& q,double x){return q.distance<x;});
            double time=hi==rebuilt.samples.end()?rebuilt.samples.back().time:hi->time;
            if(hi!=rebuilt.samples.begin()&&hi!=rebuilt.samples.end()){const auto& lo=*(hi-1);time=lo.time+(hi->time-lo.time)*(at-lo.distance)/(hi->distance-lo.distance);}
            const auto intent=sampleFvdControl(source.program.controls,time);
            a.maximumNormalResidualG=std::max(a.maximumNormalResidualG,std::abs(force.vertical-intent.normalG));
            a.maximumLateralResidualG=std::max(a.maximumLateralResidualG,std::abs(force.lateral-source.hand*intent.lateralG));
            const auto desired=unit(transform(point.up,source.heading,source.hand)-actual.tangent*dot(transform(point.up,source.heading,source.hand),actual.tangent));
            a.maximumRollResidualRadians=std::max(a.maximumRollResidualRadians,std::acos(std::clamp(dot(actual.up,desired),-1.,1.)));
        }
    }
    for(const auto& source:d.splinePrograms) {
        if(source.firstKnot>=source.lastKnot||source.lastKnot>=d.track.knots.size()||!finite(source.origin)||!std::isfinite(source.length)||source.length<=0||std::abs(source.hand)!=1){d.report.fail("AUTHORING_RANGE","Invalid spline source mapping");continue;}
        MotionProgram program;program.length=source.length;program.pitch=source.pitch;program.heading=source.heading;
        bool finiteControls=true;for(const auto& channel:{source.pitch,source.heading})for(double x:channel)finiteControls&=std::isfinite(x);
        if(!finiteControls){d.report.fail("AUTHORING_RANGE","Nonfinite spline programme");continue;}
        const size_t count=source.lastKnot-source.firstKnot;
        Vec3 displacement{},roundoff{};
        for(size_t i=0;i<=count;++i) {
            if(i){const Vec3 step=detail::integrateDirection([&](double x){return program.direction(x).tangent;},source.length*(i-1)/count,source.length*i/count)-roundoff;
                const Vec3 next=displacement+step;roundoff=(next-displacement)-step;displacement=next;}
            const Vec3 expected=source.origin+displacement;
            a.maximumSourcePositionError=std::max(a.maximumSourcePositionError,norm(Vec3{expected.x,expected.y*source.hand,expected.z}-d.track.knots[source.firstKnot+i].position));
        }
    }
    if(a.maximumSourcePositionError>1e-4)d.report.fail("AUTHORING_GEOMETRY","Canonical geometry disagrees with retained force/spline source",0,a.maximumSourcePositionError,1e-4);
    // These are model-agreement goals, separate from the unchanged rider limits.
    if(a.maximumNormalResidualG>.35||a.maximumLateralResidualG>.35)d.report.fail("AUTHORING_FORCE","Final point-mass motion no longer follows its authored FVD forces",0,std::max(a.maximumNormalResidualG,a.maximumLateralResidualG),.35);
    if(a.maximumRollResidualRadians>.12)d.report.fail("AUTHORING_ROLL","Banking changed an authored force programme's physical orientation",0,a.maximumRollResidualRadians,.12);
    a.passed=d.report.errors.size()==errors;
}
}
