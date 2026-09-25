#include "coaster/operation_hardware.hpp"
#include "coaster/trim_brake.hpp"
#include "coaster/coaster.hpp"
#include "../src/simulation_internal.hpp"
#include "../src/trim_layout.hpp"
#include <iostream>
#include <stdexcept>
using namespace coaster;
static int checks;
static void check(bool ok,const char* why){++checks;if(!ok)throw std::runtime_error(why);}
static bool equal(const Operation& a,const Operation& b){return a.start==b.start&&a.end==b.end&&a.kind==b.kind&&a.targetSpeed==b.targetSpeed&&a.maxForce==b.maxForce&&a.maxPower==b.maxPower&&a.rampSeconds==b.rampSeconds&&a.stopDeceleration==b.stopDeceleration&&a.stopOffset==b.stopOffset&&a.exitFadeMeters==b.exitFadeMeters;}
static Operation op(double start,double end){return {start,end,DriveKind::Boost,65,5250,525000,.5,2.4,.2};}
int main(){try{
    std::vector<Operation> indexed;
    for(int i=0;i<97;++i)indexed.push_back(op(std::fmod(i*137.,1024.),std::fmod(i*59.,1024.)));
    indexed.push_back(op(1024,0));indexed.push_back(op(0,1024));
    const DriveIndex index(indexed,1024);
    for(double s=0;s<=1024;s+=.25)for(double probe:{s,std::nextafter(s,0.),std::nextafter(s,INFINITY)})if(probe<1024){
        const auto& candidates=index.at(probe);check(std::is_sorted(candidates.begin(),candidates.end()),"Spatial drive lookup preserves force summation order");
        for(size_t i=0;i<indexed.size();++i){const auto& op=indexed[i];bool active=op.start<=op.end?(probe>=op.start&&probe<op.end):(probe>=op.start||probe<op.end);
            check(!active||std::binary_search(candidates.begin(),candidates.end(),i),"Spatial lookup includes overlapping, wrapping and exact-boundary operations");}
    }
    std::vector<Operation> empty;coalesceDriveProfiles(empty);check(empty.empty(),"Empty operations remain empty");
    auto a=op(0,100),b=op(100,200),c=op(200,300);std::vector<Operation> chain{a,b,c};coalesceDriveProfiles(chain);check(chain.size()==1&&equal(chain[0],op(0,300)),"Exactly identical contiguous run coalesces");auto once=chain;coalesceDriveProfiles(chain);check(chain.size()==once.size()&&equal(chain[0],once[0]),"Coalescing is idempotent");
    for(int field=0;field<8;++field){auto changed=b;switch(field){case 0:changed.kind=DriveKind::Brake;break;case 1:changed.targetSpeed=66;break;case 2:changed.maxForce+=1;break;case 3:changed.maxPower+=1;break;case 4:changed.rampSeconds+=.1;break;case 5:changed.stopDeceleration+=.1;break;case 6:changed.stopOffset+=.1;break;case 7:changed.exitFadeMeters+=.1;break;}
        std::vector<Operation> x{a,changed};coalesceDriveProfiles(x);check(x.size()==2&&equal(x[0],a)&&equal(x[1],changed),"Every different profile field preserves its boundary");}
    for(double start:{101.,std::nextafter(100.,INFINITY),99.,std::nextafter(100.,-INFINITY)}){auto changed=b;changed.start=start;std::vector<Operation>x{a,changed};coalesceDriveProfiles(x);check(x.size()==2&&equal(x[0],a)&&equal(x[1],changed),"Gaps/overlaps are not merged, including single ULP");}
    for(auto special:std::vector<Operation>{op(100,20),op(100,100),op(100,INFINITY)}){std::vector<Operation>x{a,special};coalesceDriveProfiles(x);check(x.size()==2,"Wrapping/empty/nonfinite intervals are preserved for normal validation");}
    auto invalid=b;invalid.rampSeconds=NAN;std::vector<Operation>x{a,invalid};coalesceDriveProfiles(x);check(x.size()==2&&std::isnan(x.back().rampSeconds),"Invalid profile is never hidden");
    for(double fade:std::array<double,5>{0.,.001,1000.1,NAN,INFINITY}){auto first=a,second=b;first.exitFadeMeters=second.exitFadeMeters=fade;std::vector<Operation> invalidFade{first,second};coalesceDriveProfiles(invalidFade);check(invalidFade.size()==2,"Invalid fade profiles remain explicit for rejection");}
    auto wrap=op(300,20),after=op(20,100);x={wrap,after};coalesceDriveProfiles(x);check(x.size()==2&&equal(x[0],wrap)&&equal(x[1],after),"Wrapped regions are left explicit");
    x={b,a};coalesceDriveProfiles(x);check(x.size()==2&&equal(x[0],b)&&equal(x[1],a),"Authored operations are never reordered");
    std::vector<AuthoredPoint> points;for(int i=0;i<=350;++i)points.push_back({{double(i),0,20},0,Element::Launch,{0,0,1}});auto track=compile(points,false);TrainConfig train;train.cars=1;train.carMass=1500;train.dragCdA=0;train.rollingResistance=0;train.seatHeight=0;
    TrainConfig sevenCar=train;sevenCar.cars=7;
    std::vector<AuthoredPoint> bendPoints;
    for(int i=0;i<=350;++i){const double x=i;double y=0;if(i<=30){const double u=x/30,v=1-u;y=64*u*u*u*v*v*v;}bendPoints.push_back({{x,y,20-.05*x},0,Element::Turn,{0,0,1}});}
    const auto bend=compile(bendPoints,false);const double bendEnd=bend.spans[30].start;
    check(std::abs(poweredAlignmentMargin()-2.)<1e-12,"Powered envelope is one 1.275 m car half-length, 0.6 m stator half-module and one 0.125 m geometry sample");
    check(!propulsionGeometry(bend,bend.spans[15].start),"A local horizontal bend is an invalid powered site");
    auto constantGradeWindowStart=[&](const Track& candidate,double start){for(;start+84<candidate.length;start+=.125){bool valid=true;for(double s=start;s<=start+84;s+=.125)if(!propulsionGeometry(candidate,s)){valid=false;break;}if(valid)return start;}return -1.;};
    const double bendAlignedStart=constantGradeWindowStart(bend,bendEnd);
    check(bendAlignedStart>=bendEnd&&bendAlignedStart<bendEnd+20,"The compiled bend-to-grade transition reaches a valid straight powered boundary");
    Operation alignedBoost=op(bendAlignedStart+poweredAlignmentMargin(),bendAlignedStart+82);alignedBoost.rampSeconds=.05;alignedBoost.exitFadeMeters=.2;
    for(double s=alignedBoost.start-poweredAlignmentMargin();s<=alignedBoost.end+poweredAlignmentMargin();s+=.125)check(propulsionGeometry(bend,s),"The full two-metre guard lies on constant-grade straight track");
    Operation clippedBoost=alignedBoost;clippedBoost.start=bend.spans[15].start+poweredAlignmentMargin();
    check(!propulsionGeometry(bend,clippedBoost.start-poweredAlignmentMargin()),"The local guard rejects a motor site that intrudes into the bend");
    std::vector<AuthoredPoint> pitchPoints;
    for(int i=0;i<=250;++i){const double x=i;double z=22-.05*x;if(i<=40){const double u=x/40;z=20-.05*x+2*(10*u*u*u-15*u*u*u*u+6*u*u*u*u*u);}pitchPoints.push_back({{x,0,z},0,Element::Hill,{0,0,1}});}
    const auto pitchTransition=compile(pitchPoints,false);const double pitchEnd=pitchTransition.spans[40].start;
    check(!propulsionGeometry(pitchTransition,pitchTransition.spans[12].start),"A changing-grade pitch transition is not a constant-grade powered site");
    const double pitchAlignedStart=constantGradeWindowStart(pitchTransition,pitchEnd);
    check(pitchAlignedStart>=pitchEnd&&pitchAlignedStart<pitchEnd+20,"The compiled pitch transition reaches a valid constant-grade powered boundary");
    Operation pitchBoost=op(pitchAlignedStart+poweredAlignmentMargin(),pitchAlignedStart+82);
    for(double s=pitchBoost.start-poweredAlignmentMargin();s<=pitchBoost.end+poweredAlignmentMargin();s+=.125)check(propulsionGeometry(pitchTransition,s),"The pitch-transition motor guard stays within the constant-grade section");
    const std::vector<Operation> localBoostOps{alignedBoost};
    const auto straddlingBoost=simulate(bend,localBoostOps,sevenCar,1./960);
    check(straddlingBoost.completed&&straddlingBoost.metrics.maxEnergyResidual<1e-6,"A local aligned Boost physically completes with closed full-train energy");
    const double trainHalf=(sevenCar.cars-1)*sevenCar.spacing*.5,firstEntryCenter=alignedBoost.start-trainHalf;
    const Frame *soloFirst=nullptr,*soloLast=nullptr;
    for(const auto& frame:straddlingBoost.frames)if(frame.distance-firstEntryCenter>=2.4&&frame.distance-firstEntryCenter<=3.1){
        int poweredCars=0;for(int car=0;car<sevenCar.cars;++car){const double carDistance=frame.distance+trainHalf-car*sevenCar.spacing;if(carDistance>=alignedBoost.start&&carDistance<alignedBoost.end)++poweredCars;}
        if(poweredCars==1){if(!soloFirst)soloFirst=&frame;soloLast=&frame;}
    }
    check(soloFirst&&soloLast&&soloLast->distance>soloFirst->distance,"The motor applies while only the leading car occupies the powered section");
    check(!propulsionGeometry(bend,soloFirst->distance-trainHalf),"Unpowered rear cars may remain on the bend while the lead car is aligned");
    const double oneCarDriveWorkSlope=(soloLast->driveWorkPerMass-soloFirst->driveWorkPerMass)/(soloLast->distance-soloFirst->distance);
    check(std::abs(oneCarDriveWorkSlope-alignedBoost.maxForce/(sevenCar.carMass*sevenCar.cars))<.005,"Straddling Boost work uses one car's rated force divided by full train mass");
    const auto localHardware=buildOperationHardware(bend,localBoostOps);bool alignedHardware=false;
    for(const auto& piece:localHardware)if(piece.operation==0&&piece.powered){check(propulsionGeometry(bend,piece.distance),"Every physical powered clip is installed on constant-grade aligned track");alignedHardware=true;}
    check(alignedHardware,"The straddling Boost has physical stator hardware");
    Operation launchDrive=op(0,78);launchDrive.kind=DriveKind::Launch;
    Operation physicalBrake=op(110,150);physicalBrake.kind=DriveKind::Brake;physicalBrake.targetSpeed=0;physicalBrake.maxForce=2100;physicalBrake.maxPower=210000;physicalBrake.rampSeconds=.02;
    const std::vector<Operation> brakeOps{launchDrive,physicalBrake};const auto brakeRun=simulate(track,brakeOps,sevenCar,1./960);
    check(brakeRun.completed&&brakeRun.metrics.brakeWorkPerMass>0&&brakeRun.metrics.maxEnergyResidual<1e-6,"A real rated Brake operation does negative work with closed energy balance");
    const double firstBrakeCenter=physicalBrake.start-trainHalf,oneCarBrakeAcceleration=-physicalBrake.maxForce/(sevenCar.carMass*sevenCar.cars);bool oneCarBrake=false;
    for(const auto& frame:brakeRun.frames)if(frame.distance-firstBrakeCenter>=1&&frame.distance-firstBrakeCenter<=3){
        int brakingCars=0;for(int car=0;car<sevenCar.cars;++car){const double carDistance=frame.distance+trainHalf-car*sevenCar.spacing;if(carDistance>=physicalBrake.start&&carDistance<physicalBrake.end)++brakingCars;}
        if(brakingCars==1&&std::abs(frame.acceleration-oneCarBrakeAcceleration)<.01)oneCarBrake=true;
    }
    check(oneCarBrake,"A single straddling car receives the rated Brake force divided by full train mass");
    const auto brakeHardware=buildOperationHardware(track,brakeOps);bool physicalBrakeJaws=false;
    for(const auto& piece:brakeHardware)if(piece.operation==1){
        check(!piece.powered,"Brake hardware is a physical friction assembly");const auto pose=track.sample(piece.distance);const auto delta=piece.box.center-pose.position;
        if(std::abs(std::abs(dot(delta,pose.right))-.1)<1e-12&&std::abs(dot(delta,pose.up)+.045)<1e-12&&std::abs(piece.box.half.y-.05)<1e-12&&std::abs(piece.box.half.z-.105)<1e-12)physicalBrakeJaws=true;
    }
    check(physicalBrakeJaws,"The rated Brake has paired physical friction jaws on the track");
    std::vector<Operation> separate{op(0,100),op(100,300)},merged=separate;coalesceDriveProfiles(merged);const double expected=3.5/gravity;
    for(double step:{1./480,1./960}){
        auto before=simulate(track,separate,train,step),afterRun=simulate(track,merged,train,step);check(before.completed&&afterRun.completed,"Both explicit-drive control runs physically complete");
        auto nearBoundary=[](const SimulationResult& r){size_t i=1;while(i<r.frames.size()&&r.frames[i].distance<100)++i;return i;};size_t oldIndex=nearBoundary(before),newIndex=nearBoundary(afterRun);check(oldIndex<before.frames.size()&&newIndex<afterRun.frames.size(),"Boundary was traversed in both runs");
        double earlier=0;for(const auto& frame:before.frames)if(frame.distance>90&&frame.distance<98)earlier=std::max(earlier,frame.seats[0].longitudinal);
        check(earlier>expected*.99&&before.frames[oldIndex].seats[0].longitudinal<expected*.1,"Unmerged module boundary unnecessarily fades and restarts a continuous motor region");
        check(std::abs(afterRun.frames[newIndex-1].seats[0].longitudinal-expected)<1e-12&&std::abs(afterRun.frames[newIndex].seats[0].longitudinal-expected)<1e-12,"Merged boundary retains analytic constant motor acceleration");
        int interior=0;for(const auto& frame:afterRun.frames)if(frame.distance>90&&frame.distance<110){++interior;check(std::abs(frame.seats[0].longitudinal-expected)<1e-12,"All samples across former boundary equal analytic F/m/g");}check(interior>20,"Continuity test has multiple independent time samples");
        check(afterRun.metrics.maxSpeed>before.metrics.maxSpeed,"Removed artificial interruption changes speed through explicit accumulated force");
    }
    check(eddyBrakeForce(0,4000,25)==0,"An eddy-current trim cannot hold a stopped train");
    check(std::abs(eddyBrakeForce(25,4000,25)+4000)<1e-12,"Rated eddy brake peak occurs at its characteristic speed");
    check(eddyBrakeForce(10,4000,25)<0&&eddyBrakeForce(-10,4000,25)>0,"Eddy drag opposes either direction of travel");
    check(std::abs(eddyBrakeForce(100,4000,25))<std::abs(eddyBrakeForce(25,4000,25)),"High-speed eddy force decreases beyond the drag peak");
    Operation trim{160,310,DriveKind::Trim,5,3750,200000,.3,1.8,.025,8,25,30};
    check(validDriveParameters(trim),"Physical trim characteristic and upstream sensor are explicit");
    check(trimFieldCoverage(trim,trim.start)==0&&trimFieldCoverage(trim,trim.end)==0,"Both physical brake ends have zero engagement");
    auto missing=trim;missing.trimPeakSpeed=0;check(!validDriveParameters(missing),"A trim without a force-speed characteristic is rejected");
    missing=trim;missing.trimSensorLead=0;check(!validDriveParameters(missing),"A trim without an upstream detector is rejected");
    std::vector<Operation> trims{trim,trim};trims[1].start=trim.end;trims[1].end+=150;coalesceDriveProfiles(trims);check(trims.size()==2,"Separate brake controllers are never merged");
    auto runTrim=[&](int cars,double threshold,double step){TrainConfig t=train;t.cars=cars;auto brake=trim;brake.targetSpeed=threshold;return simulate(track,{op(0,120),brake},t,step);};
    for(int cars:{1,6}) {
        const auto active=runTrim(cars,5,1./960),inactive=runTrim(cars,100,1./960),fine=runTrim(cars,5,1./1920);
        check(active.completed&&inactive.completed&&fine.completed,"Trimmed and retracted finite trains complete the same track");
        check(active.metrics.brakeWorkPerMass>100&&inactive.metrics.brakeWorkPerMass==0,"Upstream sensing latches brake deployment; below threshold the brake remains retracted");
        check(active.frames.back().speed<inactive.frames.back().speed,"Measured speed falls through physical braking work");
        check(active.metrics.maxEnergyResidual<1e-6,"Per-car magnetic trim work closes the energy balance");
        check(active.trims.size()==1&&std::abs(active.trims[0].energyJoules-active.metrics.brakeWorkPerMass*cars*train.carMass)<1e-5,"Per-zone dissipated energy equals independently accumulated total brake work");
        check(active.trims[0].peakPowerWatts>0&&inactive.trims[0].deployment==0,"Brake telemetry records actual active and retracted states");
        check(std::abs(active.frames.back().speed-fine.frames.back().speed)<1e-4,"Trim entry, occupancy and exit converge at half timestep");
        if(cars==1) {
            const Frame *first=nullptr,*last=nullptr;
            for(const auto& f:active.frames)if(f.distance>170&&f.distance<295){if(!first)first=&f;last=&f;}
            const auto primitive=[](double v){return v+v*v*v/(3*25*25);};
            check(first&&last&&std::abs(primitive(first->speed)-primitive(last->speed)-2*2.5/25*(last->distance-first->distance))<1e-6,"Uniform brake interior matches the independent analytic speed-distance solution");
        }
    }
    auto partial=trim;partial.targetSpeed=29;const double opening=trimDeployment(partial,30,train.carMass);
    check(opening>0&&opening<1&&trimDeployment(partial,29,train.carMass)==0,"A small overspeed selects partial deployment before arrival");
    auto terminal=op(315,345);terminal.kind=DriveKind::Brake;
    const std::vector<Operation> hardwareOperations{op(0,120),trim,terminal};
    const auto hardware=buildOperationHardware(track,hardwareOperations);
    check(!hardware.empty(),"Physical operation hardware is generated from simulation zones");
    bool poweredMountBridgesTrack=false;
    for(const auto& piece:hardware) {
        const auto& operation=hardwareOperations[piece.operation];
        check(piece.distance>=operation.start&&piece.distance<=operation.end,"Hardware lies on its actual operation interval");
        const auto pose=track.sample(piece.distance);const auto delta=piece.box.center-pose.position;
        if(piece.powered&&std::abs(piece.box.half.y-.40)<1e-12&&std::abs(piece.box.half.z-.105)<1e-12){
            const double centreUp=dot(delta,pose.up);
            check(std::abs(centreUp+.295)<1e-12&&centreUp-piece.box.half.z<=-.39&&centreUp+piece.box.half.z>=-.20,"LSM mounting box bridges the spine top and lower stator edge");
            poweredMountBridgesTrack=true;
        }
        check(std::abs(dot(delta,pose.right))+piece.box.half.y<=1.5&&dot(delta,pose.up)-piece.box.half.z>=-.8&&dot(delta,pose.up)+piece.box.half.z<=2.4,"Fixed assemblies stay inside the reserved nonlocal-clearance envelope");
        check(dot(delta,pose.up)+piece.box.half.z<.075,"Deployed drive/brake solids retain clearance below the actual train body bottom at +10 cm");
        check(dot(piece.box.forward,pose.tangent)>1-1e-12&&dot(piece.box.up,pose.up)>1-1e-12,"Hardware uses the canonical physical track frame");
    }
    check(poweredMountBridgesTrack,"Powered hardware includes a continuous LSM-to-spine mounting box");
    Design graded;graded.request.train=train;
    std::vector<AuthoredPoint> incline;for(int i=0;i<=800;++i)incline.push_back({{double(i),0,20+i*.05},0,Element::Hill,{0,0,1}});
    graded.track=compile(incline,false);graded.operations={op(0,180)};graded.sections={{"graded-coast",250,700,0,true}};
    const auto nominal=simulate(graded.track,graded.operations,train);check(nominal.completed,"Graded trim placement fixture completes");
    planTrimBrakes(graded,nominal.frames);check(graded.operations.size()==2,"Automatic trim layout finds a suitable inclined coast");
    const auto protectedRun=simulate(graded.track,graded.operations,train);
    check(protectedRun.completed&&protectedRun.trims.size()==1&&protectedRun.trims[0].deployment==0,"Nominal trim installation cannot conceal rejected dynamics");
    check(std::abs(protectedRun.frames.back().speed-nominal.frames.back().speed)<1e-12,"Retracted trims leave the actual energy trajectory unchanged");
    const auto inclinedHardware=buildOperationHardware(graded.track,graded.operations);
    for(const auto& h:inclinedHardware)check(std::abs(h.box.forward.z-1/std::sqrt(401.))<1e-9,"LSM and brake hardware follow the real graded track");
    auto makeTrimPolicyDesign=[&](double recoveryEnd,double boostStart,bool forceBetween){
        Design candidate;candidate.request.train=train;candidate.track=graded.track;candidate.operations={op(0,180)};
        if(forceBetween){auto brake=op(recoveryEnd,recoveryEnd+10);brake.kind=DriveKind::Brake;brake.targetSpeed=50;brake.maxForce=2100;brake.maxPower=210000;candidate.operations.push_back(brake);}
        candidate.operations.push_back(op(boostStart,780));
        candidate.sections={{"cliff-approach-recovery",recoveryEnd-150,recoveryEnd,0,true,RideRole::CliffApproach,"cliff-approach"},
            {"cliff-approach-ascent",recoveryEnd,800,0,true,RideRole::CliffApproach,"cliff-approach"}};
        return candidate;
    };
    auto plannedTrimEnd=[&](const Design& candidate,const std::vector<Frame>& frames){
        const auto& section=candidate.sections.front();const auto middle=candidate.track.sample((section.start+section.end)*.5);
        const bool loopExit=section.role==RideRole::Loop&&middle.tangent.z<0;
        const bool inversionApproach=(middle.element==Element::Turn||loopExit)&&candidate.track.sample(std::min(candidate.track.length,section.end+1)).element==Element::Inversion;
        const double centre=section.start+(section.end-section.start)*(loopExit?.70:inversionApproach?.48:.62),speed=replayValueAt(frames,centre);
        const double length=std::clamp(speed*(inversionApproach?1.2:.8),18.,inversionApproach?80.:55.);return centre+length*.5;
    };
    auto nearBoost=makeTrimPolicyDesign(700,723.4,false);const auto nearBoostMotion=simulate(nearBoost.track,nearBoost.operations,train,1./960);
    const double nearTrimEnd=plannedTrimEnd(nearBoost,nearBoostMotion.frames),nearBoostGap=723.4-nearTrimEnd,nearBoostSpeed=replayValueAt(nearBoostMotion.frames,nearTrimEnd);
    check(nearBoostMotion.completed&&nearBoostSpeed>20&&nearBoostGap/nearBoostSpeed<4,"The candidate trim end is within four seconds of the adjacent Boost");
    planTrimBrakes(nearBoost,nearBoostMotion.frames);
    check(nearBoost.operations.size()==2,"A redundant recovery trim directly before an adjacent same-element Boost is omitted");
    auto renamedBoost=makeTrimPolicyDesign(700,723.4,false);renamedBoost.sections[0].name="settle-window";renamedBoost.sections[1].name="motor-runup";
    const auto renamedBoostMotion=simulate(renamedBoost.track,renamedBoost.operations,train,1./960);planTrimBrakes(renamedBoost,renamedBoostMotion.frames);
    check(renamedBoost.operations.size()==2,"Trim suppression follows typed adjacent sections after their display names change");
    auto farBoost=makeTrimPolicyDesign(400,723.4,false);const auto farBoostMotion=simulate(farBoost.track,farBoost.operations,train,1./960);
    const double farTrimEnd=plannedTrimEnd(farBoost,farBoostMotion.frames),farBoostGap=723.4-farTrimEnd,farBoostSpeed=replayValueAt(farBoostMotion.frames,farTrimEnd);
    check(farBoostMotion.completed&&farBoostSpeed>20&&farBoostGap/farBoostSpeed>4,"An upstream recovery remains outside the four-second Boost redundancy window");
    planTrimBrakes(farBoost,farBoostMotion.frames);
    check(farBoost.operations.size()==3&&farBoost.operations.back().kind==DriveKind::Trim,"A useful far-upstream trim remains available");
    auto forceBetweenBoost=makeTrimPolicyDesign(700,723.4,true);const auto forceBetweenMotion=simulate(forceBetweenBoost.track,forceBetweenBoost.operations,train,1./960);
    check(forceBetweenMotion.completed,"The force-critical boundary fixture completes before trim planning");
    planTrimBrakes(forceBetweenBoost,forceBetweenMotion.frames);
    check(forceBetweenBoost.operations.size()==4&&forceBetweenBoost.operations.back().kind==DriveKind::Trim,"A real rated Brake between recovery and Boost preserves the trim candidate");
    auto checkChangingSpeedTrimPolicy=[&](double grade,bool descent,double boostStart){
        std::vector<AuthoredPoint> points;for(int i=0;i<=1200;++i){const Element element=descent?(i<700?Element::Turn:Element::Inversion):Element::Hill;
            points.push_back({{double(i),0,20+grade*i},0,element,{0,0,1}});}
        Design candidate;candidate.request.train=sevenCar;candidate.track=compile(points,false);candidate.operations={op(0,420),op(boostStart,1200)};
        candidate.sections={{"cliff-approach-recovery",550,700,0,true,RideRole::CliffApproach,"cliff-approach"},
            {"cliff-approach-ascent",700,1200,0,true,RideRole::CliffApproach,"cliff-approach"}};
        const auto motion=simulate(candidate.track,candidate.operations,sevenCar,1./960);check(motion.completed,"The multi-car speed-change timing fixture completes");
        const double trimEnd=plannedTrimEnd(candidate,motion.frames),approachSpeed=replayValueAt(motion.frames,trimEnd);
        const double estimate=(boostStart-trimEnd)/approachSpeed,elapsed=replayValueAt(motion.frames,boostStart,true)-replayValueAt(motion.frames,trimEnd,true);
        check(descent?(estimate>6&&elapsed<6):(estimate<6&&elapsed>6),"Replay elapsed time corrects the end-speed estimate across a changing-grade approach");
        planTrimBrakes(candidate,motion.frames);
        check(candidate.operations.size()==(descent?2u:3u),descent?
            "An accelerating multi-car approach suppresses a trim within six replay seconds":
            "An uphill multi-car approach keeps a trim when replay time exceeds six seconds");
    };
    checkChangingSpeedTrimPolicy(.05,false,940.);
    checkChangingSpeedTrimPolicy(-.05,true,1020.);
    auto late=trim;late.trimSensorLead=1;const auto rejectedTrim=simulate(track,{op(0,120),late},train,1./960);
    check(!rejectedTrim.report.valid()&&rejectedTrim.report.errors.front().code=="TRIM_ARMING","Late brake actuation fails explicitly instead of teleporting the actuator into place");
    std::cout<<"PASS "<<checks<<" exact-drive coalescing and force continuity checks\n";return 0;
}catch(const std::exception& e){std::cerr<<"FAIL "<<checks<<": "<<e.what()<<'\n';return 1;}}
