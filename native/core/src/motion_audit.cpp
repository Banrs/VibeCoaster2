#include "coaster/coaster.hpp"
#include "coaster/clearance.hpp"
#include "simulation_internal.hpp"
#include <iomanip>
#include <sstream>
#include <stdexcept>

namespace coaster {
namespace {
Frame frameAt(const std::vector<Frame>& frames,double distance){
    auto hi=std::lower_bound(frames.begin(),frames.end(),distance,[](const Frame& f,double s){return f.distance<s;});
    if(hi==frames.begin())return *hi;if(hi==frames.end())return frames.back();
    const auto& a=*(hi-1);const auto& b=*hi;const double u=(distance-a.distance)/(b.distance-a.distance);
    auto mix=[&](double x,double y){return x+(y-x)*u;};Frame out;
    out.distance=distance;out.time=mix(a.time,b.time);out.speed=mix(a.speed,b.speed);
    out.driveWorkPerMass=mix(a.driveWorkPerMass,b.driveWorkPerMass);out.brakeWorkPerMass=mix(a.brakeWorkPerMass,b.brakeWorkPerMass);
    out.lossWorkPerMass=mix(a.lossWorkPerMass,b.lossWorkPerMass);out.energyResidual=mix(a.energyResidual,b.energyResidual);return out;
}
double cross2(Vec3 a,Vec3 b){return a.x*b.y-a.y*b.x;}
void number(std::ostream& out,double x){if(std::isfinite(x))out<<x;else out<<"null";}
}
void assessMotion(Design& d,Cancel cancel){
    d.motion={};auto& audit=d.motion;
    if(d.track.spans.empty()||d.simulation.frames.empty()||!d.simulation.completed)return;
    if(d.sections.empty()){d.report.fail("MOTION_INTENT","New rides require their authored section intent");return;}
    if(!d.request.recipe.elements.empty()){
        size_t current=0;std::vector<bool> seen(d.request.recipe.elements.size());
        for(const auto& section:d.sections){
            const auto found=std::find_if(d.request.recipe.elements.begin(),d.request.recipe.elements.end(),[&](const RecipeElement& e){return e.id==section.recipeId;});
            const size_t index=size_t(found-d.request.recipe.elements.begin());
            if(found==d.request.recipe.elements.end()||found->role!=section.role||index<current||index>current+1){d.report.fail("MOTION_RECIPE_MAP","Compiled section roles/IDs disagree with the ordered recipe",section.start);return;}
            seen[index]=true;current=index;
        }
        if(!std::all_of(seen.begin(),seen.end(),[](bool value){return value;})){d.report.fail("MOTION_RECIPE_MAP","Compiled sections omit an element from the saved recipe");return;}
    }
    audit.performed=true;const auto errors=d.report.errors.size();
    double previousEnd=0;
    for(const auto& section:d.sections){
        if(!std::isfinite(section.start)||!std::isfinite(section.end)||section.end<=section.start||std::abs(section.start-previousEnd)>1e-5||section.end>d.track.length+1e-5||section.heightReversals<0||section.heightReversals>2){d.report.fail("MOTION_INTENT","Section ranges or declared interior shape are invalid");return;}
        previousEnd=section.end;
        const auto first=frameAt(d.simulation.frames,section.start),last=frameAt(d.simulation.frames,section.end);
        SectionAssessment result;result.beginTime=first.time;result.endTime=last.time;result.entrySpeed=first.speed;result.exitSpeed=last.speed;
        result.minimumSpeed=std::min(first.speed,last.speed);result.maximumSpeed=std::max(first.speed,last.speed);
        result.driveWorkPerMass=last.driveWorkPerMass-first.driveWorkPerMass;result.brakeWorkPerMass=last.brakeWorkPerMass-first.brakeWorkPerMass;
        result.lossWorkPerMass=last.lossWorkPerMass-first.lossWorkPerMass;result.energyResidual=last.energyResidual-first.energyResidual;
        size_t sectionHint=d.track.spans.size();
        auto sectionSample=[&](double distance){const auto where=d.track.locate(distance,sectionHint);return d.track.sampleSpan(where.span,where.parameter);};
        const auto origin=sectionSample(section.start);result.minimumHeight=result.maximumHeight=origin.position.z;
        result.minimumRailGroundHeight=result.maximumRailGroundHeight=origin.position.z-d.request.terrain.height(origin.position.x,origin.position.y);
        const Vec3 planeRight=unit(Vec3{-origin.tangent.y,origin.tangent.x,0});
        int sign=0,pitchSign=0;bool inverted=false;Vec3 previousTangent=origin.tangent;
        const int samples=std::max(2,int(std::ceil((section.end-section.start)/.25)));
        for(int i=0;i<=samples;++i){
            if((i&255)==0&&cancel&&cancel()){d.simulation.cancelled=true;d.report.fail("CANCELLED","Motion audit cancelled");return;}
            const double s=section.start+(section.end-section.start)*i/samples;const auto q=sectionSample(s);
            result.minimumHeight=std::min(result.minimumHeight,q.position.z);result.maximumHeight=std::max(result.maximumHeight,q.position.z);
            const double gap=q.position.z-d.request.terrain.height(q.position.x,q.position.y);
            result.meanRailGroundHeight+=gap*((i==0||i==samples)?.5:1.)/samples;
            result.minimumRailGroundHeight=std::min(result.minimumRailGroundHeight,gap);result.maximumRailGroundHeight=std::max(result.maximumRailGroundHeight,gap);
            result.maximumPitch=std::max(result.maximumPitch,std::asin(std::clamp(std::abs(q.tangent.z),0.,1.)));
            result.minimumSignedPitch=std::min(result.minimumSignedPitch,std::asin(std::clamp(q.tangent.z,-1.,1.)));
            if(std::hypot(q.tangent.x,q.tangent.y)>.01){const auto upright=unit(Vec3{0,0,1}-q.tangent*q.tangent.z);result.maximumBank=std::max(result.maximumBank,std::abs(std::atan2(dot(q.up,cross(q.tangent,upright)),dot(q.up,upright))));}
            if(std::hypot(q.tangent.x,q.tangent.y)>.2&&std::hypot(previousTangent.x,previousTangent.y)>.2)
            {const double change=std::remainder(std::atan2(q.tangent.y,q.tangent.x)-std::atan2(previousTangent.y,previousTangent.x),2*pi);result.headingChange+=std::abs(change);result.signedHeadingChange+=change;}
            previousTangent=q.tangent;
            inverted|=q.up.z<-.5;
            if(std::abs(q.curvature.z)>3e-5){const int next=q.curvature.z>0?1:-1;if(pitchSign&&next!=pitchSign)++result.pitchExtrema;pitchSign=next;}
            if(std::abs(q.tangent.z)>.008){int next=q.tangent.z>0?1:-1;if(sign&&next!=sign)++result.heightReversals;sign=next;}
            if(section.planar&&std::abs(dot(q.position-origin.position,planeRight))>1e-4){d.report.fail("PLANAR_INTENT","A planar hill acquired sideways curvature",s);break;}
        }
        auto trainHeight=[&](double distance){double h=0;for(int car=0;car<d.request.train.cars;++car)h+=d.track.sample(distance+((d.request.train.cars-1)*.5-car)*d.request.train.spacing).position.z/d.request.train.cars;return h;};
        // No resistance: this is an upper bound on gravity-only exit speed.
        // A zero result means even this lossless train cannot reach the exit.
        result.passiveExitSpeedUpperBound=std::sqrt(std::max(0.,first.speed*first.speed+2*gravity*(trainHeight(section.start)-trainHeight(section.end))));
        const auto& train=d.request.train;
        std::array<size_t,16> gradeHints{};gradeHints.fill(d.track.spans.size());
        for(const auto& f:d.simulation.frames)if(f.distance>=section.start&&f.distance<=section.end){
            result.minimumSpeed=std::min(result.minimumSpeed,f.speed);result.maximumSpeed=std::max(result.maximumSpeed,f.speed);
            double grade=0;for(int car=0;car<train.cars;++car){const auto where=d.track.locate(f.distance+((train.cars-1)*.5-car)*train.spacing,gradeHints[car]);grade+=d.track.sampleSpan(where.span,where.parameter).tangent.z/train.cars;}
            const double actuator=f.acceleration+gravity*grade+gravity*train.rollingResistance+.5*train.airDensity*train.dragCdA*f.speed*f.speed/(train.cars*train.carMass);
            result.maximumActuatorAcceleration=std::max(result.maximumActuatorAcceleration,actuator);result.maximumNetAcceleration=std::max(result.maximumNetAcceleration,f.acceleration);
        }
        const bool boost=std::any_of(d.operations.begin(),d.operations.end(),[&](const Operation& op){return op.kind==DriveKind::Boost&&op.start>=section.start&&op.end<=section.end;});
        if(boost&&d.generationVersion==generatorVersion&&(result.exitSpeed<result.entrySpeed||result.driveWorkPerMass<=0||result.exitSpeed<=result.passiveExitSpeedUpperBound||result.maximumActuatorAcceleration<=gravity))
            d.report.fail("PURPOSEFUL_BOOST","Visible LSM must gain speed, outperform gravity alone and produce over 1g of actual motor acceleration",section.start);
        if(result.heightReversals>section.heightReversals)d.report.fail("INTERIOR_SHAPE",section.name+" has an unintended height reversal",section.start,result.heightReversals,section.heightReversals);
        if(result.pitchExtrema>(inverted?2:1))d.report.fail("INTERIOR_PITCH_SHAPE",section.name+" contains an extra pitch shoulder or oscillation",section.start,result.pitchExtrema,inverted?2:1);
        audit.sections.push_back(result);
    }
    if(std::abs(previousEnd-d.track.length)>1e-5)d.report.fail("MOTION_INTENT","Section intent does not cover the final canonical circuit");
    if(!d.request.recipe.elements.empty()){
        std::array<bool,size_t(LandmarkKind::BrakeEntry)+1> seen{};double plateauTime=NAN,dropTime=NAN;
        for(const auto& landmark:d.landmarks){const auto kind=size_t(landmark.kind);
            if(kind>=seen.size()||seen[kind]||!std::isfinite(landmark.distance)||landmark.distance<0||landmark.distance>d.track.length){d.report.fail("LANDMARK_INTENT","Invalid, duplicate or out-of-range verification landmark");continue;}
            seen[kind]=true;const double time=frameAt(d.simulation.frames,landmark.distance-seatDistanceOffset(d.request.train,0)).time;
            if(landmark.kind==LandmarkKind::PlateauArrival)plateauTime=time;if(landmark.kind==LandmarkKind::CliffDeparture)dropTime=time;
        }
        if(!std::all_of(seen.begin(),seen.end(),[](bool value){return value;}))d.report.fail("LANDMARK_INTENT","Default recipe is missing a required verification landmark");
        // Paused front POV/media-time inspection brackets the banked crest
        // around 6:38 and the first lip departure around 6:58. The resulting
        // approximately20s interval has about1s visual landmark uncertainty;
        // 20.5s is a conservative cap below1.1 times its plausible lower end.
        constexpr double clifftopLimit=20.5;
        if(!std::isfinite(plateauTime)||!std::isfinite(dropTime)||dropTime<plateauTime||dropTime-plateauTime>clifftopLimit)
            d.report.fail("CLIFFTOP_PACING","Front-seat crest-to-cliff commitment exceeds the conservative FF pacing cap (braking/holding included)",0,dropTime-plateauTime,clifftopLimit);
    }
    // Semantic pacing is reported separately from the unchanged acceptance
    // gates. Elements may compile into several motion sections.
    auto frontTime=[&](double distance){return frameAt(d.simulation.frames,distance-seatDistanceOffset(d.request.train,0)).time;};
    auto landmarkDistance=[&](LandmarkKind kind){
        double distance=NAN;bool found=false;
        for(const auto& landmark:d.landmarks)if(landmark.kind==kind){
            if(found||!std::isfinite(landmark.distance)||landmark.distance<0||landmark.distance>d.track.length)return double(NAN);
            distance=landmark.distance;found=true;
        }
        return distance;
    };
    double lipStart=INFINITY,signatureEnd=-INFINITY;
    for(const auto& section:d.sections){
        if(section.role==RideRole::CliffLip)lipStart=std::min(lipStart,section.start);
        if(section.role==RideRole::Signature)signatureEnd=std::max(signatureEnd,section.end);
    }
    const double plateau=landmarkDistance(LandmarkKind::PlateauArrival),departure=landmarkDistance(LandmarkKind::CliffDeparture);
    if(std::isfinite(plateau)&&std::isfinite(departure)&&std::isfinite(lipStart)&&plateau<=lipStart&&lipStart<=departure){
        audit.clifftopActiveSeconds=frontTime(lipStart)-frontTime(plateau);
        audit.clifftopBrakingSeconds=frontTime(departure)-frontTime(lipStart);
    }
    if(std::isfinite(signatureEnd)&&std::isfinite(d.simulation.metrics.duration)){
        const double duration=d.simulation.metrics.duration-frontTime(signatureEnd);
        if(duration>=0)audit.returnSeconds=duration;
    }
    for(size_t i=0;i<d.track.spans.size();++i){
        const auto a=sampleSpanKinematics(d.track,i,1),b=sampleSpanKinematics(d.track,(i+1)%d.track.spans.size(),0);
        const std::array<double,4> position{norm(a.sample.position-b.sample.position),norm(a.sample.tangent-b.sample.tangent),norm(a.sample.curvature-b.sample.curvature),norm(a.curvatureS-b.curvatureS)};
        const std::array<double,4> orientation{norm(a.sample.up-b.sample.up),norm(a.upS-b.upS),norm(a.upSS-b.upSS),std::max(norm(a.upSSS-b.upSSS),norm(a.curvatureSS-b.curvatureSS))};
        for(int j=0;j<4;++j){audit.positionJoinError[j]=std::max(audit.positionJoinError[j],position[j]);audit.orientationJoinError[j]=std::max(audit.orientationJoinError[j],orientation[j]);}
    }
    for(int j=0;j<4;++j)if(audit.positionJoinError[j]>1e-7||audit.orientationJoinError[j]>1e-7)d.report.fail("MOTION_CONTINUITY","Final geometry or physical orientation does not meet its C3 arc-length tolerance",0,std::max(audit.positionJoinError[j],audit.orientationJoinError[j]),1e-7);
    // Meaningful route interactions: actual transverse plan intersections,
    // separated by at least 150 m of travel, deduplicated over whole encounters.
    const int count=int(std::ceil(d.track.length/3));std::vector<Vec3> points;points.reserve(count+1);size_t crossingHint=d.track.spans.size();
    for(int i=0;i<=count;++i){const auto where=d.track.locate(d.track.length*i/count,crossingHint);points.push_back(d.track.sampleSpan(where.span,where.parameter).position);}
    for(int i=0;i<count;++i)for(int j=i+1;j<count;++j){
        const double separation=d.track.length*(j-i)/count;
        if(separation<150||d.track.length-separation<150)continue;
        const Vec3 a=points[i],b=points[i+1],c=points[j],e=points[j+1];
        if(std::max(a.x,b.x)<std::min(c.x,e.x)||std::max(c.x,e.x)<std::min(a.x,b.x)||std::max(a.y,b.y)<std::min(c.y,e.y)||std::max(c.y,e.y)<std::min(a.y,b.y))continue;
        const Vec3 u=b-a,v=e-c;const double denominator=cross2(u,v);
        const double horizontalProduct=std::hypot(u.x,u.y)*std::hypot(v.x,v.y);
        if(horizontalProduct<1e-8||std::abs(denominator)<horizontalProduct*std::sin(20*pi/180))continue;
        const double x=cross2(c-a,v)/denominator,y=cross2(c-a,u)/denominator;if(x<0||x>1||y<0||y>1)continue;
        const double first=d.track.length*(i+x)/count,second=d.track.length*(j+y)/count;
        if(std::any_of(audit.crossings.begin(),audit.crossings.end(),[&](const Crossing& q){return std::abs(q.firstDistance-first)<60&&std::abs(q.secondDistance-second)<60;}))continue;
        const Vec3 p=a+u*x,q=c+v*y;
        audit.crossings.push_back({first,second,std::abs(p.z-q.z),std::asin(std::min(1.,std::abs(denominator)/horizontalProduct)),(p+q)*.5});
    }
    const double half=(d.request.train.cars-1)*d.request.train.spacing*.5;double run=0;
    double levelRun=0,levelBegin=0,returnRun=0;size_t coastSection=0;
    size_t coastHint=d.track.spans.size();
    for(size_t i=1;i<d.simulation.frames.size();++i){const auto& f=d.simulation.frames[i];
        const double s=f.distance;bool hardware=s<half+30||s>d.track.length;
        for(const auto& op:d.operations){const double end=op.kind==DriveKind::Station?d.track.length:op.end;if(s+half+2>=op.start&&s-half-2<=end)hardware=true;}
        const auto where=d.track.locate(s,coastHint);const auto k=sampleSpanKinematics(d.track,where.span,where.parameter);
        const bool flat=std::abs(k.sample.tangent.z)<std::sin(3*pi/180)&&norm(k.sample.curvature)*f.speed<.015&&norm(k.upS)*f.speed<.015;
        if(flat&&!hardware){const double dt=f.time-d.simulation.frames[i-1].time;run+=dt;audit.flatCoastSeconds+=dt;audit.longestFlatCoastSeconds=std::max(audit.longestFlatCoastSeconds,run);}else run=0;
        // Turning and banking can hide level backhaul from the frame-hold
        // rule. This separate diagnostic does not prescribe extra hills.
        const bool level=std::abs(k.sample.tangent.z)<std::sin(3*pi/180)&&std::abs(k.sample.curvature.z)*f.speed<.015;
        while(coastSection+1<d.sections.size()&&s>=d.sections[coastSection].end)++coastSection;
        const bool returning=d.sections[coastSection].role==RideRole::Return&&s>=d.sections[coastSection].start&&s<=d.sections[coastSection].end;
        if(level&&!hardware){
            const double dt=f.time-d.simulation.frames[i-1].time;
            if(levelRun==0)levelBegin=d.simulation.frames[i-1].distance;
            levelRun+=dt;audit.levelCoastSeconds+=dt;
            if(levelRun>audit.longestLevelCoastSeconds){
                audit.longestLevelCoastSeconds=levelRun;audit.longestLevelCoastStartDistance=levelBegin;audit.longestLevelCoastEndDistance=s;
            }
            if(returning){returnRun+=dt;audit.returnLevelCoastSeconds+=dt;audit.longestReturnLevelCoastSeconds=std::max(audit.longestReturnLevelCoastSeconds,returnRun);}
            else returnRun=0;
        }else{levelRun=0;returnRun=0;}
    }
    if(audit.longestFlatCoastSeconds>2*Limits::allowanceFactor)d.report.fail("WAITING_TRACK","Unpowered level track exceeds the two-second pacing target and its five-percent allowance",0,audit.longestFlatCoastSeconds,2*Limits::allowanceFactor);
    else if(audit.longestFlatCoastSeconds>2)d.report.warnings.push_back("Quiet coast exceeds the nominal two-second pacing target within its five-percent allowance");
    audit.passed=d.report.errors.size()==errors;
}
SpatialReplay replaySpatialRefinement(const Design& d,Cancel cancel){
    SpatialReplay work;auto& result=work.assessment;result.performed=true;
    try{
        Track refined;refined.closed=d.track.closed;refined.authoredGeometry=refined.authoredFrame=true;refined.knots.reserve(d.track.knots.size()*2);
        for(size_t i=0;i<d.track.spans.size();++i){
            if(cancel&&cancel())throw std::runtime_error("CANCELLED");
            for(double u:{0.,.5}) {
                const auto q=sampleSpanKinematics(d.track,i,u);const auto& a=q.sample;
                refined.knots.push_back({a.position,a.tangent,a.curvature,a.up,0,a.element,q.curvatureS,q.curvatureSS,q.upS,q.upSS,q.upSSS});
            }
        }
        refined.knots.push_back(refined.knots.front());refined.rebuild();
        auto map=[&](double s){const auto at=d.track.locate(s);const bool second=at.parameter>.5;return refined.distanceAtSpan(2*at.span+(second?1:0),at.parameter*2-(second?1:0));};
        auto operations=d.operations;for(auto& op:operations){op.start=map(op.start);op.end=map(op.end);}
        for(size_t i=0;i<d.track.spans.size();++i)for(double u:{.125,.375,.625,.875}){
            const auto a=d.track.sampleSpan(i,u),b=refined.sampleSpan(2*i+(u>.5?1:0),u*2-(u>.5?1:0));
            result.maximumPositionError=std::max(result.maximumPositionError,norm(a.position-b.position));result.maximumOrientationError=std::max(result.maximumOrientationError,norm(a.up-b.up));
        }
        if(result.maximumPositionError>.001||result.maximumOrientationError>.001)work.report.fail("SPATIAL_REFINEMENT","Half-spacing canonical reconstruction changes geometry beyond 1 mm / 0.001 frame-vector tolerance",0,std::max(result.maximumPositionError,result.maximumOrientationError),.001);
        work.simulation=simulate(refined,operations,d.request.train,d.request.simulationStep,cancel);
        result.replay.coarseStep=result.replay.fineStep=d.request.simulationStep;
        auto supports=d.supports;for(auto& support:supports)support.trackDistance=map(support.trackDistance);
        const auto refinedSweep=buildClearanceSweepVerified(refined,d.request.train,cancel);
        const auto clearance=validateGeometry(refined,d.request.terrain,d.request.limits,d.request.train,supports,refinedSweep,cancel);
        for(const auto& error:clearance.errors)work.report.fail("SPATIAL_"+error.code,error.message,error.distance,error.actual,error.limit);
    }catch(const std::exception& e){if(cancel&&cancel())work.simulation.cancelled=true;work.report.fail("SPATIAL_REFINEMENT",e.what());}
    return work;
}
void verifySpatialRefinementWith(Design& d,const std::function<SpatialReplay()>& replay,Cancel cancel){
    d.spatial={};
    try{
        auto work=replay();d.spatial=std::move(work.assessment);auto& result=d.spatial;
        const auto comparison=compareSimulationConvergence(d.simulation,work.simulation,d.request.limits,result.replay);
        for(const auto& error:comparison.errors)work.report.fail("SPATIAL_"+error.code,error.message,error.distance,error.actual,error.limit);
        const auto targets=validateSimulationTargets(work.simulation,d.request.targets,d.request.limits);
        for(const auto& error:targets.errors)work.report.fail("SPATIAL_"+error.code,error.message,error.distance,error.actual,error.limit);
        if(work.simulation.cancelled||(cancel&&cancel())){d.simulation.cancelled=true;work.report.fail("CANCELLED","Spatial refinement cancelled");}
        result.passed=work.report.valid()&&result.performed;
        d.report.errors.insert(d.report.errors.end(),work.report.errors.begin(),work.report.errors.end());
    }catch(const std::exception& e){if(cancel&&cancel())d.simulation.cancelled=true;d.report.fail("SPATIAL_REFINEMENT",e.what());}
}
void verifySpatialRefinement(Design& d,Cancel cancel){
    verifySpatialRefinementWith(d,[&]{return replaySpatialRefinement(d,cancel);},cancel);
}
std::string motionReportJson(const Design& d){
    std::ostringstream o;o<<std::setprecision(12);const auto& m=d.motion;
    o<<"{\"performed\":"<<(m.performed?"true":"false")<<",\"passed\":"<<(m.passed?"true":"false")<<",\"positionJoinErrors\":[";
    for(int i=0;i<4;++i){if(i)o<<',';number(o,m.positionJoinError[i]);}o<<"],\"physicalFrameJoinErrors\":[";
    for(int i=0;i<4;++i){if(i)o<<',';number(o,m.orientationJoinError[i]);}
    o<<"],\"flatCoastSeconds\":"<<m.flatCoastSeconds<<",\"longestFlatCoastSeconds\":"<<m.longestFlatCoastSeconds;
    for(auto [key,value]:std::vector<std::pair<const char*,double>>{{"clifftopActiveSeconds",m.clifftopActiveSeconds},{"clifftopBrakingSeconds",m.clifftopBrakingSeconds},{"returnSeconds",m.returnSeconds},
        {"levelCoastSeconds",m.levelCoastSeconds},{"longestLevelCoastSeconds",m.longestLevelCoastSeconds},{"longestLevelCoastStartDistance",m.longestLevelCoastStartDistance},
        {"longestLevelCoastEndDistance",m.longestLevelCoastEndDistance},{"returnLevelCoastSeconds",m.returnLevelCoastSeconds},{"longestReturnLevelCoastSeconds",m.longestReturnLevelCoastSeconds}}){o<<",\""<<key<<"\":";number(o,value);}
    o<<",\"pacingMethod\":\"Front-seat plateau arrival to first lip section; lip section to cliff departure; final signature exit to complete stop. Semantic intervals, not actuator deployment times.\""
        <<",\"levelCoastMethod\":\"Diagnostic only: pitch below 3 degrees and vertical curvature times speed below 0.015 per second, including turns/banking. Hardware contact excluded; 60 Hz presentation intervals. Return totals use Return-role sections.\",\"crossings\":[";
    bool comma=false;for(const auto& c:m.crossings){if(comma)o<<',';comma=true;o<<"{\"firstDistance\":"<<c.firstDistance<<",\"secondDistance\":"<<c.secondDistance<<",\"heightSeparation\":"<<c.heightSeparation<<",\"angleRadians\":"<<c.angle<<'}';}
    o<<"],\"sectionSampling\":\"Quarter-metre shape sampling; 60 Hz speed/work event interpolation. Force/rate/jerk acceptance uses native simulation and half-step refinement.\",\"sections\":[";
    for(size_t i=0;i<m.sections.size();++i){if(i)o<<',';const auto& s=m.sections[i];o<<"{\"name\":"<<std::quoted(d.sections[i].name)<<",\"role\":"<<std::quoted(roleName(d.sections[i].role))<<",\"recipeId\":"<<std::quoted(d.sections[i].recipeId)<<",\"start\":"<<d.sections[i].start<<",\"end\":"<<d.sections[i].end;
        for(auto [key,value]:std::vector<std::pair<const char*,double>>{{"beginTime",s.beginTime},{"endTime",s.endTime},{"entrySpeed",s.entrySpeed},{"exitSpeed",s.exitSpeed},{"minimumSpeed",s.minimumSpeed},{"maximumSpeed",s.maximumSpeed},{"minimumHeight",s.minimumHeight},{"maximumHeight",s.maximumHeight},{"meanRailGroundHeight",s.meanRailGroundHeight},{"minimumRailGroundHeight",s.minimumRailGroundHeight},{"maximumRailGroundHeight",s.maximumRailGroundHeight},{"passiveExitSpeedUpperBound",s.passiveExitSpeedUpperBound},{"maximumActuatorAcceleration",s.maximumActuatorAcceleration},{"maximumNetAcceleration",s.maximumNetAcceleration},{"maximumPitchRadians",s.maximumPitch},{"minimumSignedPitchRadians",s.minimumSignedPitch},{"maximumBankRadians",s.maximumBank},{"headingChangeRadians",s.headingChange},{"signedHeadingChangeRadians",s.signedHeadingChange},{"driveWorkPerMass",s.driveWorkPerMass},{"brakeWorkPerMass",s.brakeWorkPerMass},{"lossWorkPerMass",s.lossWorkPerMass},{"energyResidual",s.energyResidual}}){o<<",\""<<key<<"\":";number(o,value);}
        o<<",\"heightReversals\":"<<s.heightReversals<<",\"pitchExtrema\":"<<s.pitchExtrema<<'}';
    }
    o<<"],\"landmarks\":[";
    for(size_t i=0;i<d.landmarks.size();++i){if(i)o<<',';const auto& landmark=d.landmarks[i];o<<"{\"name\":"<<std::quoted(landmarkName(landmark.kind))<<",\"distance\":";number(o,landmark.distance);
        if(!d.simulation.frames.empty()){o<<",\"frontTime\":";number(o,frameAt(d.simulation.frames,landmark.distance-seatDistanceOffset(d.request.train,0)).time);o<<",\"rearTime\":";number(o,frameAt(d.simulation.frames,landmark.distance-seatDistanceOffset(d.request.train,2)).time);}
        o<<'}';
    }
    const auto& s=d.spatial;o<<"],\"spatialRefinement\":{\"method\":\"Half-spacing reconstruction of the frozen canonical motion, remapped operations and independent train replay/clearance\",\"performed\":"<<(s.performed?"true":"false")<<",\"passed\":"<<(s.passed?"true":"false")<<",\"maximumPositionError\":";number(o,s.maximumPositionError);o<<",\"maximumOrientationError\":";number(o,s.maximumOrientationError);o<<",\"maximumForceRelativeError\":";number(o,s.replay.maxForceRelativeError);o<<"}}";return o.str();
}
}
