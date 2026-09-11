#include "coaster/coaster.hpp"
#include "../src/drive_force.hpp"
#include <iostream>
#include <stdexcept>
using namespace coaster;
static int checks;
static void check(bool ok,const char* why){++checks;if(!ok)throw std::runtime_error(why);}
static bool equal(const Operation& a,const Operation& b){return a.start==b.start&&a.end==b.end&&a.kind==b.kind&&a.targetSpeed==b.targetSpeed&&a.maxForce==b.maxForce&&a.maxPower==b.maxPower&&a.rampSeconds==b.rampSeconds&&a.stopDeceleration==b.stopDeceleration&&a.stopOffset==b.stopOffset&&a.exitFadeMeters==b.exitFadeMeters;}
static Operation op(double start,double end){return {start,end,DriveKind::Boost,65,5250,525000,.5,2.4,.2};}
int main(){try{
    // Independent force/power and fade cases protect the one force authority
    // shared by dynamics and the derived replay indication.
    auto rated=op(0,100);rated.maxForce=6000;rated.maxPower=180000;rated.rampSeconds=1;rated.exitFadeMeters=10;
    auto applied=[&](double speed,double time,double entry,double remaining,double stopping=100){return detail::appliedDriveForce(rated,1500,speed,time,entry,remaining,stopping);};
    check(applied(20,2,0,50)==6000,"Drive is force-limited at low speed");
    check(applied(60,2,0,50)==3000,"Drive is power-limited at high speed");
    check(applied(70,2,0,50)==0,"Installed motor above target supplies no force");
    check(applied(20,.5,0,5)==1500,"Independent half entry and exit fades multiply actual force");
    check(applied(20,2,-1,50)==0,"Uncommitted midpoint entry cannot invent a completed ramp");
    rated.kind=DriveKind::Brake;rated.targetSpeed=30;
    check(applied(60,2,0,50)==-3000&&applied(20,2,0,50)==0,"Controlled braking removes bounded work and releases below target");
    rated.kind=DriveKind::Station;rated.stopDeceleration=2;rated.stopOffset=1;
    check(applied(10,2,0,50,26)==-3000&&applied(20,2,0,50,26)==-6000,"Station tracks the derivative of its persisted distance-to-stop target within hardware limits");
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
    std::vector<Operation> separate{op(0,100),op(100,300)},merged=separate;coalesceDriveProfiles(merged);const double expected=3.5/gravity;
    for(double step:{1./480,1./960}){
        auto before=simulate(track,separate,train,step),afterRun=simulate(track,merged,train,step);check(before.completed&&afterRun.completed,"Both explicit-drive control runs physically complete");
        auto nearBoundary=[](const SimulationResult& r){size_t i=1;while(i<r.frames.size()&&r.frames[i].distance<100)++i;return i;};size_t oldIndex=nearBoundary(before),newIndex=nearBoundary(afterRun);check(oldIndex<before.frames.size()&&newIndex<afterRun.frames.size(),"Boundary was traversed in both runs");
        double earlier=0;for(const auto& frame:before.frames)if(frame.distance>90&&frame.distance<98)earlier=std::max(earlier,frame.seats[0].longitudinal);
        check(earlier>expected*.99&&before.frames[oldIndex].seats[0].longitudinal<expected*.1,"Unmerged module boundary unnecessarily fades and restarts a continuous motor region");
        check(std::abs(afterRun.frames[newIndex-1].seats[0].longitudinal-expected)<1e-12&&std::abs(afterRun.frames[newIndex].seats[0].longitudinal-expected)<1e-12,"Merged boundary retains analytic constant motor acceleration");
        int interior=0;for(const auto& frame:afterRun.frames)if(frame.distance>90&&frame.distance<110){++interior;check(std::abs(frame.seats[0].longitudinal-expected)<1e-12,"All samples across former boundary equal analytic F/m/g");}check(interior>20,"Continuity test has multiple independent time samples");
        for(const auto& frame:afterRun.frames){
            check(frame.drives.braking==0&&!frame.drives.brakePresent,"Launch-only replay does not invent braking hardware or force");
            check(std::abs(frame.drives.propulsion-frame.seats[0].longitudinal*gravity*train.carMass)<1e-8,"Recorded propulsion equals independently measured flat, lossless train force");
            if(frame.distance>300)check(!frame.drives.motorPresent&&frame.drives.propulsion==0,"Motor contact and applied force end at the actual installed exit");
        }
        check(afterRun.metrics.maxSpeed>before.metrics.maxSpeed,"Removed artificial interruption changes speed through explicit accumulated force");
    }
    auto governed=op(0,300);governed.targetSpeed=12;
    auto cruise=simulate(track,{governed},train,1./960);
    check(cruise.completed&&std::any_of(cruise.frames.begin(),cruise.frames.end(),[](const Frame& f){return f.speed>11.9&&f.drives.motorPresent&&f.drives.propulsion<1;}),"Installed LSM contact remains visible when its governor stops propelling");
    auto service=op(150,300);service.kind=DriveKind::Brake;service.targetSpeed=10;
    auto slowing=simulate(track,{op(0,100),service},train,1./960);int brakingFrames=0;
    check(slowing.completed,"Finite controlled-brake fixture completes");
    for(const auto& frame:slowing.frames){
        check(std::abs(frame.drives.propulsion+frame.drives.braking-frame.seats[0].longitudinal*gravity*train.carMass)<1e-8,"Signed recorded drive force agrees with independent flat train force during braking");
        if(frame.drives.braking<0){++brakingFrames;check(frame.drives.brakePresent,"Applied braking is associated with actual hardware contact");}
    }
    check(brakingFrames>20,"Braking indication is tested throughout actual deceleration");
    auto disabled=op(0,300);disabled.maxForce=0;
    auto stopped=simulate(track,{disabled},train,1./960);
    check(std::all_of(stopped.frames.begin(),stopped.frames.end(),[](const Frame& f){return !f.drives.motorPresent&&!f.drives.brakePresent&&f.drives.propulsion==0&&f.drives.braking==0;}),"Zero-capacity operation has neither force nor installed-hardware indication");
    // A passive source begins at a train-center port. Ending hardware at that
    // same distance leaves rear cars working inside the source's energy audit.
    for(int i=351;i<=1000;++i)points.push_back({{double(i),0,20},0,Element::Return,{0,0,1}});
    const auto sourceTrack=compile(points,false);TrainConfig sourceTrain;sourceTrain.dragCdA=0;sourceTrain.rollingResistance=0;
    const double half=(sourceTrain.cars-1)*sourceTrain.spacing*.5;
    auto hardware=[&](double begin,double end,DriveKind kind,double target,double acceleration){auto o=op(begin,end);o.kind=kind;o.targetSpeed=target;o.maxForce=sourceTrain.carMass*acceleration;o.maxPower=o.maxForce*100;o.exitFadeMeters=target*o.rampSeconds;return o;};
    auto speedAt=[](const SimulationResult& r,double distance){auto next=std::lower_bound(r.frames.begin(),r.frames.end(),distance,[](const Frame& f,double s){return f.distance<s;});check(next!=r.frames.begin()&&next!=r.frames.end(),"Physical trace covers source port");const auto& before=*(next-1);return before.speed+(next->speed-before.speed)*(distance-before.distance)/(next->distance-before.distance);};
    for(double step:{1./960,1./1920}){
        for(auto kind:{DriveKind::Boost,DriveKind::Brake})for(bool inset:{false,true}){
            auto launch=hardware(0,300,DriveKind::Launch,45,12),work=hardware(450,550,kind,kind==DriveKind::Boost?80:20,7);
            if(inset){work.start+=half;work.end-=half;}
            const auto run=simulate(sourceTrack,{launch,work},sourceTrain,step);check(run.completed,"Finite-train source-port fixture physically completes");
            double force=0;int interior=0;for(const auto& frame:run.frames)if(frame.distance>=550&&frame.distance<=550+half){force=std::max(force,std::abs(frame.drives.propulsion+frame.drives.braking));++interior;}
            const double change=speedAt(run,700)-speedAt(run,550);
            check(interior>5,"Several trace samples resolve the rear cars crossing the source inlet");
            if(inset){check(force==0,"All cars have cleared upstream hardware at the source-center inlet");check(std::abs(change)<1e-6,"A lossless passive source retains its measured inlet energy");}
            else{check(force>100,"Rail ending at the source-center inlet still applies rear-car force");check(std::abs(change)>.001,"Uncleared actuator work changes energy after the source inlet");}
        }
        for(bool inset:{false,true}){
            auto launch=hardware(0,300,DriveKind::Launch,45,12),motor=hardware(450,550,DriveKind::Boost,80,7),brake=hardware(550,680,DriveKind::Brake,20,7);
            if(inset){motor.start+=half;motor.end-=half;brake.start+=half;brake.end-=half;}
            const auto run=simulate(sourceTrack,{launch,motor,brake},sourceTrain,step);check(run.completed,"Adjacent finite-train operation fixture completes");
            int mixed=0;for(const auto& frame:run.frames)if(frame.drives.propulsion>0&&frame.drives.braking<0)++mixed;
            check(inset?mixed==0:mixed>0,"Partitioned center work domains prevent simultaneous rear propulsion and front braking");
        }
    }
    std::vector<AuthoredPoint> circuit;
    for(int i=0;i<=512;++i){double angle=2*pi*i/512;circuit.push_back({{500*std::sin(angle),500*(1-std::cos(angle)),20},0,Element::Turn,{0,0,1}});}
    circuit.back()=circuit.front();auto closed=compile(circuit);
    // Departure starts with cars already on the installed rail. Keep that
    // upstream start while clearing its downstream end before the first source.
    for(int cars:{6,12}){TrainConfig departure;departure.cars=cars;double baseline=0;const double h=(cars-1)*departure.spacing*.5;
        for(int mode=0;mode<3;++mode){Operation launch{20,200,DriveKind::Launch,76,departure.carMass*38.85,departure.carMass*3885,.16};launch.exitFadeMeters=76*.16;
            if(mode){launch.end-=h;if(mode==1)launch.start+=h;}
            // This fixture exercises initial launch only; stop after the actual
            // speed crossing rather than simulating an unprovided station stop.
            int polls=0;const auto run=simulate(closed,{launch},departure,1./960,[&]{return ++polls>200;});
            check(run.cancelled&&std::isfinite(run.metrics.launchTo180),"Bounded departure fixture physically measures 0 to 50 m/s before cancellation");
            if(mode==0)baseline=run.metrics.launchTo180;
            if(mode==1)check(cars==6?run.metrics.launchTo180==baseline:run.metrics.launchTo180>baseline+.1,"Moving the initial rail start removes contact and delays the longer train's launch");
            if(mode==2){check(run.metrics.launchTo180==baseline,"Preserving initial rail start preserves physical launch timing");check(run.metrics.launchTo180<1.4,"Six- and twelve-car departures retain the selected 0-to-50 m/s target");}
        }
    }
    for(int cars:{1,6})for(double deceleration:{3.,4.,5.,6.,7.,8.}){
      double previousStop=0;
      for(double step:{1./960,1./1920}){
        TrainConfig consist;consist.cars=cars;consist.dragCdA=.4*cars;
        Operation launch{20,200,DriveKind::Launch,55,consist.carMass*39,consist.carMass*3900,.16};
        Operation station{closed.length-500,80,DriveKind::Station,0,consist.carMass*9,consist.carMass*900,.5,deceleration,.2};
        const auto arrival=simulate(closed,{launch,station},consist,step);
        check(arrival.completed,"Stronger service braking completes a physical closed circuit without station overrun");
        const auto& last=arrival.frames.back();const double finish=closed.length+(cars-1)*consist.spacing*.5+30;
        check(last.speed==0&&finish-last.distance>=-.25&&finish-last.distance<.5,"One-car and finite-train profiles stop within the unchanged station position tolerance");
        if(previousStop)check(std::abs(last.distance-previousStop)<.001,"Station stopping distance converges within one millimetre at mandatory verification rates");
        previousStop=last.distance;
        bool bounded=true,braked=false;
        for(const auto& frame:arrival.frames){
            bounded=bounded&&frame.drives.propulsion>=0&&frame.drives.braking<=0&&
                -frame.drives.braking<=cars*station.maxForce+1e-8&&
                -frame.drives.braking*frame.speed<=cars*station.maxPower+1e-8;
            braked=braked||frame.drives.braking<0;
        }
        check(bounded&&braked,"Station tracking removes work through the unchanged signed force and mechanical power bounds");
      }
    }
    std::cout<<"PASS "<<checks<<" drive force, source-port and station checks\n";return 0;
}catch(const std::exception& e){std::cerr<<"FAIL "<<checks<<": "<<e.what()<<'\n';return 1;}}
