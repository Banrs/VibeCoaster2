#pragma once
#include "coaster/angular_phase_intent.hpp"
#include <stdexcept>

inline int angularPhaseMetadataChecks(){
    using namespace coaster;int count=0;
    auto check=[&](bool ok,const char* message){++count;if(!ok)throw std::runtime_error(message);};
    FvdRequest program;program.speed=30;program.controls={{0,1,0,0},{3,1,0,0}};program.twists={{.3,2.7,.5}};
    const auto derived=deriveFvdRollVelocityPhases(program);
    check(derived.unresolved.empty()&&derived.phases.size()==3,"Physical twist metadata declares stationary / roll / stationary phases without sampling geometry");
    check(derived.phases[0].direction==AngularDirection::Stationary&&derived.phases[1].direction==AngularDirection::Nonnegative&&derived.phases[2].direction==AngularDirection::Stationary,"Explicit twist angle sets its intended physical hand");
    auto pair=program;pair.twists={{.2,1.4,.5},{1.6,2.8,-.3}};
    const auto paired=deriveFvdRollVelocityPhases(pair);
    check(paired.unresolved.empty()&&paired.phases.size()==5&&paired.phases[3].direction==AngularDirection::Nonpositive,"Separate authored twist phases retain their deliberate reversal and quiet gap");
    auto additive=program;additive.additiveTwists=true;for(auto& control:additive.controls)control.rollRate=.02;
    const auto together=deriveFvdRollVelocityPhases(additive);
    check(together.unresolved.empty()&&together.phases.size()==3,"Same-handed additive baseline and twist have a provable shared direction");
    for(const auto& phase:together.phases)check(phase.direction==AngularDirection::Nonnegative,"An additive baseline is not mislabeled stationary outside the twist window");
    for(auto& control:additive.controls)control.rollRate=-.1;
    const auto opposed=deriveFvdRollVelocityPhases(additive);
    check(opposed.unresolved.size()==1&&opposed.phases.size()==2,"Opposed additive controls remain explicitly unresolved rather than inventing an expected reversal from geometry");
    auto gravityBank=program;gravityBank.gravityReferencedRoll=true;
    const auto unsupported=deriveFvdRollVelocityPhases(gravityBank);
    check(unsupported.phases.empty()&&unsupported.unresolved.size()==3,"Gravity-referenced bank is not falsely equated to physical tangent twist");

    const auto rebuilt=designFvdSection(program);check(rebuilt.assessment.passed,"Independent FVD angular-source fixture integrates and replays");
    ForceAuthoring source;source.name="authored-twist";source.program=program;
    for(const auto& span:rebuilt.track.spans)source.sourceDistances.push_back(span.start);source.sourceDistances.push_back(rebuilt.track.length);
    auto replay=[](const Track& track){Frame a,b;a.speed=b.speed=17;b.distance=track.length;b.time=track.length/17;return std::vector<Frame>{a,b};};
    const auto frames=replay(rebuilt.track);
    const auto phases=resolveFvdAngularPhases(rebuilt.track,source,rebuilt,derived.phases);
    check(phases.size()==3&&phases.front().beginDistance==0&&phases.back().endDistance==rebuilt.track.length,"Source clock phases resolve onto complete canonical coverage");
    check(assessAngularPhases(rebuilt.track,frames,phases).report.valid(),"Physical twist phases accept the independent authored motion at changed actual train speed");
    const std::array<double,3> tolerance{1e-6,1e-5,1e-4};
    const auto original=assessFvdAngularAgreement(rebuilt.track,source,rebuilt,frames,tolerance);
    check(original.report.valid()&&original.samples>100,"Fresh source and final signed V/A/J agree at the actual replay dynamics");
    auto mirrored=rebuilt.track;
    for(auto& k:mirrored.knots){k.position.y*=-1;k.tangent.y*=-1;k.curvature.y*=-1;k.third.y*=-1;k.fourth.y*=-1;k.up.y*=-1;k.upFirst.y*=-1;k.upSecond.y*=-1;k.upThird.y*=-1;}
    mirrored.rebuild();auto reflectedSource=source;reflectedSource.hand=-1;
    const auto reflectedPhases=resolveFvdAngularPhases(mirrored,reflectedSource,rebuilt,derived.phases);
    check(reflectedPhases[1].direction==AngularDirection::Nonpositive,"Mirroring reverses authored physical roll hand");
    auto explicitPitch=derived.phases;explicitPitch[1].axis=AngularAxis::Pitch;
    check(resolveFvdAngularPhases(mirrored,reflectedSource,rebuilt,explicitPitch)[1].direction==AngularDirection::Nonnegative,"Mirroring preserves the signed pitch convention while roll/yaw reverse");
    check(assessAngularPhases(mirrored,replay(mirrored),reflectedPhases).report.valid()&&assessFvdAngularAgreement(mirrored,reflectedSource,rebuilt,replay(mirrored),tolerance).report.valid(),"Mirrored source metadata and all signed derivatives agree without false hand failures");

    auto altered=rebuilt.track;const size_t middle=altered.knots.size()/2;
    altered.knots[middle].upFirst=altered.knots[middle].upFirst+cross(altered.knots[middle].tangent,altered.knots[middle].up)*.001;
    altered.rebuild();
    const auto left=sampleSpanKinematics(altered,middle-1,1),right=sampleSpanKinematics(altered,middle,0);
    check(norm(left.sample.up-right.sample.up)+norm(left.upS-right.upS)+norm(left.upSS-right.upSS)+norm(left.upSSS-right.upSSS)<1e-7,"Derivative-only tampering preserves shared C3 frame ports");
    for(size_t i=0;i<altered.knots.size();++i)check(norm(altered.knots[i].up-rebuilt.track.knots[i].up)<1e-14,"The derivative mutation leaves every authored orientation value unchanged");
    const auto changed=assessFvdAngularAgreement(altered,source,rebuilt,replay(altered),tolerance);
    check(!changed.report.valid()&&changed.maximumError[0]>.001&&changed.maximumError[2]>.001,"Independent signed derivative audit rejects a C3 mutation invisible to knot-orientation value checks");
    const auto refined=assessFvdAngularAgreement(altered,source,rebuilt,replay(altered),tolerance,4);
    check(!refined.report.valid(),"Spatial refinement does not hide the authored derivative mismatch");
    const auto cancelled=assessFvdAngularAgreement(rebuilt.track,source,rebuilt,frames,tolerance,2,[]{return true;});
    check(cancelled.cancelled&&!cancelled.report.valid(),"Source angular audit preserves cancellation");
    auto partial=rebuilt.track;constexpr size_t cut=100;partial.knots=std::vector<Knot>(partial.knots.begin()+cut,partial.knots.end()-cut);partial.rebuild();
    auto retained=source;retained.sourceDistances=std::vector<double>(source.sourceDistances.begin()+cut,source.sourceDistances.end()-cut);
    const auto clipped=resolveFvdAngularPhases(partial,retained,rebuilt,derived.phases);
    check(!clipped.empty()&&clipped.front().beginDistance==0&&std::abs(clipped.back().endDistance-partial.length)<1e-10,"Trimmed source retention clips existing intended phases without inventing new ones");
    check(assessFvdAngularAgreement(partial,retained,rebuilt,replay(partial),tolerance).report.valid(),"A clipped retained FVD source preserves independent angular derivative agreement");
    return count;
}
