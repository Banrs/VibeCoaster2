#include "coaster/coaster.hpp"
#include "coaster/support_mesh.hpp"
#include <iostream>
#include <stdexcept>
using namespace coaster;
namespace {
int checks=0;
void check(bool condition,const char* message){++checks;if(!condition)throw std::runtime_error(message);}
size_t feet(const Support& s){return std::count_if(s.members.begin(),s.members.end(),[](const SupportMember& m){return m.kind==SupportMemberKind::Footing;});}
Design circle(double height,double bank=0){
    Design d;
    std::vector<AuthoredPoint> points;
    for(int i=0;i<=360;++i){double a=2*pi*i/360;points.push_back({{200*std::cos(a),200*std::sin(a),height},bank,Element::Return});}
    d.track=compile(points,true);buildSupportLayout(d);return d;
}
void validate(const Design& d){
    auto sweep=buildClearanceSweep(d.track,d.request.train);
    for(const auto& s:d.supports){
        check(validateSupportMembers(s,d.request.terrain).valid(),"Family member shape, terrain and endpoint connectivity");
        check(supportCollision(s,sweep)<0,"Every family clears the actual continuous train/hardware sweep");
        auto q=d.track.sample(s.trackDistance);
        check(norm(s.attachment-(q.position-q.up*(spineDepth+spineRadius)))<1e-8,"Exact canonical spine attachment");
        const double standoff=-dot(s.top-s.attachment,q.up);check((standoff>=.6-1e-8&&standoff<=2+1e-8)||std::abs(standoff-6)<1e-8||std::abs(standoff-10)<1e-8,"Cap uses a bounded canonical under-spine stand-off");
        for(const auto& m:s.members){auto mesh=supportMemberMesh(m);check(mesh.positions.size()==34&&mesh.indices.size()==96,"Every compact member uses the canonical closed solid mesh");}
    }
}
}
int main(){try{
    // Standalone geometric fixtures do not claim physics or record acceptance.
    auto low=circle(10),medium=circle(45),tall=circle(180);
    validate(low);validate(medium);validate(tall);
    for(const auto& s:low.supports){check(feet(s)==1&&s.members.size()==3,"Low upright track uses one compact post");check(norm(s.top-s.attachment)<2.00000001,"Low support has no arbitrary lateral outreach");}
    for(const auto& s:medium.supports)check(feet(s)==2&&s.members.size()==5,"Intermediate track uses a paired bent");
    for(const auto& s:tall.supports){
        check(feet(s)==4&&s.members.size()>100,"Tall track retains a braced tower");
        check(norm(s.top-s.attachment)<2.00000001,"Unobstructed upright tall track uses centred towers without arbitrary cantilevers");
    }
    Design transition;std::vector<AuthoredPoint> transitionPoints;
    for(int i=0;i<=800;++i){double x=i;double u=std::clamp((x-200)/400.,0.,1.);double hump=35*std::pow(std::sin(pi*u),4);transitionPoints.push_back({{x,0,15+hump},0,Element::Return});}
    transition.track=compile(transitionPoints,false);buildSupportLayout(transition);validate(transition);
    double minimumGap=40,maximumGap=0;
    for(size_t i=1;i<transition.supports.size();++i){double gap=transition.supports[i].trackDistance-transition.supports[i-1].trackDistance;minimumGap=std::min(minimumGap,gap);maximumGap=std::max(maximumGap,gap);check(gap>=24-1e-8&&gap<=40+1e-8,"Adaptive spans preserve bounded density");}
    check(maximumGap-minimumGap>3,"Curved crest and straight approach receive different support spacing");
    auto groundHugging=circle(4.7);validate(groundHugging);
    for(const auto& s:groundHugging.supports)check(feet(s)==1&&norm(s.top-s.attachment)<1.1,"Low rail uses a shortened connected pier neck while retaining full swept clearance");
    auto lower=circle(7),higher=circle(20);validate(lower);validate(higher);
    check(higher.supports[0].members[1].radiusBase>lower.supports[0].members[1].radiusBase,"Post thickness adapts to height");
    check(higher.supports[0].members[0].radiusBase>lower.supports[0].members[0].radiusBase,"Footing radius adapts to height");
    auto banked=circle(45,.55);validate(banked);
    for(const auto& s:banked.supports)check(feet(s)==2,"Moderately banked sections remain connected paired bents");
    Design sideways;std::vector<AuthoredPoint> rollingPoints;
    for(int i=0;i<=200;++i)rollingPoints.push_back({{double(i),0,100},pi*smooth(i/80.),Element::Inversion});
    sideways.track=compile(rollingPoints,false);buildSupportLayout(sideways);validate(sideways);
    // A clear ordinary placement is preferable here; the frozen production
    // regression independently exercises the additional bank-normal candidates.
    check(!sideways.supports.empty(),"Rolling open-section fixture produces certified supports");
    // Near-vertical axes are accepted within 1e-6 m of XY drift. Their actual
    // frustum caps still tilt: an axis endpoint at the old enclosure boundary
    // must not stand in for the highest/lowest point of the circular cap.
    Terrain flatFootingTerrain;
    const auto footingFixture=[](double bottom,double top){
        Vec3 base{0,0,bottom},cap{0,0,top},joint=cap+Vec3{0,0,.1};
        return Support{{0,0,0},cap,joint,true,0,{
            {base,cap,.5,.5,SupportMemberKind::Footing,false},
            {cap,joint,supportRadius,supportRadius,SupportMemberKind::Steel,true}}};
    };
    const auto tiltFixture=[](Support support){
        constexpr double drift=9e-7;
        support.top.x+=drift;support.attachment.x+=drift;
        support.members[0].top.x+=drift;support.members[1].base.x+=drift;support.members[1].top.x+=drift;
        return support;
    };
    auto comfortable=tiltFixture(footingFixture(-1,1));
    check(validateSupportMembers(comfortable,flatFootingTerrain).valid(),"Comfortably anchored near-vertical footing remains permitted");
    for(auto boundary:std::array<Support,2>{footingFixture(-.5+1e-6,1),footingFixture(-1,.2-1e-6)}){
        check(validateSupportMembers(boundary,flatFootingTerrain).valid(),"Exactly vertical boundary footing retains the original enclosure tolerance");
        auto tilted=tiltFixture(boundary);const auto& foot=tilted.members[0];
        const double capExcursion=.5*9e-7/norm(foot.top-foot.base);
        check(foot.base.z+capExcursion>-.5+1e-6||foot.top.z-capExcursion<.2-1e-6,"Tilted cap actually crosses the tolerated anchoring enclosure");
        auto rejected=validateSupportMembers(tilted,flatFootingTerrain);
        check(rejected.errors.size()==1&&rejected.errors.front().code=="SUPPORT_FOOTING","Tolerated axis tilt cannot hide a cap anchoring gap");
    }
    int calls=0;bool cancelled=false;
    try{buildSupportLayout(low,[&]{return ++calls>12;});}catch(const std::exception& e){cancelled=std::string(e.what())=="CANCELLED";}
    check(cancelled,"New placement path preserves cancellation");
    {
        GenerationRequest request;request.seed=42;request.targets.requireIntensity=false;
        auto d=generate(request);
        if(!d.accepted())for(const auto& e:d.report.errors)std::cerr<<e.code<<": "<<e.message<<'\n';
        check(d.accepted(),"Actual seed42 generation passes unchanged physics and mandatory convergence");
        check(validateDesignStructures(d).valid(),"Actual families clear station structures");validate(d);
        size_t posts=0,bents=0,towers=0,lowCount=0,compactLow=0,members=0;double maxLowOutreach=0;
        for(const auto& s:d.supports){size_t n=feet(s);posts+=n==1;bents+=n==2;towers+=n==4;members+=s.members.size();if(s.top.z-s.base.z<20){++lowCount;compactLow+=n<4;maxLowOutreach=std::max(maxLowOutreach,norm(s.top-s.attachment));}}
        auto original=d.supports;buildSupportLayout(d);check(original.size()==d.supports.size(),"Support regeneration is deterministic");
        for(size_t i=0;i<original.size();++i){const auto&a=original[i];const auto&b=d.supports[i];check(a.members.size()==b.members.size(),"Seeded support family stable");for(size_t j=0;j<a.members.size();++j){const auto&x=a.members[j];const auto&y=b.members[j];check(norm(x.base-y.base)==0&&norm(x.top-y.top)==0&&x.radiusBase==y.radiusBase&&x.radiusTop==y.radiusTop&&x.kind==y.kind&&x.spineContact==y.spineContact,"Every persisted support member regenerates exactly");}}
        check(compactLow>lowCount*3/4,"Most low sections shed oversized cantilever towers");
        std::cout<<"TERRAIN "<<d.request.terrain.name()<<" seed42 posts="<<posts<<" bents="<<bents<<" towers="<<towers<<" members="<<members<<" low="<<lowCount<<" compactLow="<<compactLow<<" maxRemainingLowOutreach="<<maxLowOutreach<<'\n';
    }
    std::cout<<"PASS "<<checks<<" support family assertions; proportions are not a structural-load certification.\n";
}catch(const std::exception& e){std::cerr<<"FAIL after "<<checks<<": "<<e.what()<<'\n';return 1;}}
