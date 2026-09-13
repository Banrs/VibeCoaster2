#include "coaster/coaster.hpp"
#include "../src/simulation_internal.hpp"
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
    std::cout<<"PASS "<<checks<<" exact-drive coalescing and force continuity checks\n";return 0;
}catch(const std::exception& e){std::cerr<<"FAIL "<<checks<<": "<<e.what()<<'\n';return 1;}}
