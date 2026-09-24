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
        const auto delta=s.top-s.attachment;const double standoff=-dot(delta,q.up);
        bool bounded=standoff>=.18-1e-8&&standoff<=2+1e-8&&norm(delta+q.up*standoff)<1e-8;
        for(double offset:{2.,6.,10.}){const auto outreach=delta+q.up*offset;
            bounded|=norm(outreach)<=48+1e-8&&(std::abs(outreach.z)<1e-8||(std::abs(dot(outreach,q.up))+std::abs(dot(outreach,q.tangent))<1e-8));}
        check(bounded,"Cap has a bounded stand-off with either bank-normal or level cliff outreach");
        for(const auto& m:s.members){auto mesh=supportMemberMesh(m);check(mesh.positions.size()==66&&mesh.indices.size()==192,"Every compact member uses the canonical closed solid mesh");}
    }
}
}
int main(){try{
    // Standalone geometric fixtures do not claim physics or record acceptance.
    auto low=circle(10),medium=circle(45),tall=circle(180);
    validate(low);validate(medium);validate(tall);
    for(const auto& s:low.supports){check(feet(s)==1&&s.members.size()<=4,"Low upright track uses one track-responsive tubular leg");check(norm(s.top-s.attachment)<2.00000001,"Low support has no arbitrary lateral outreach");}
    for(const auto& s:medium.supports)check(feet(s)==2&&s.members.size()==7,"Intermediate track uses an asymmetric tubular wishbone");
    for(const auto& s:tall.supports){
        check(feet(s)==3&&s.members.size()>=15&&s.members.size()<28,"Tall track uses a broad A-frame and rear raker");
        check(norm(s.top-s.attachment)<2.00000001,"Unobstructed upright tall track retains direct spine contacts");
    }
    for(const auto& s:low.supports){
        const auto q=low.track.sample(s.trackDistance);
        check(dot(s.members.front().top-s.top,q.curvature)<0,"Low curve footings rake outward with curvature");
    }
    size_t sharedJoints=0;
    for(size_t i=1;i<tall.supports.size();++i){
        const auto& a=tall.supports[i-1];const auto& b=tall.supports[i];
        for(const auto& m:a.members)if(m.kind==SupportMemberKind::Steel&&!m.spineContact)
            for(const auto& n:b.members)if(norm(m.top-n.base)<1e-8||norm(m.top-n.top)<1e-8){++sharedJoints;break;}
    }
    check(sharedJoints>=tall.supports.size()-1,"Tall bents share exact canonical nodes through spanwise bracing");
    Design transition;std::vector<AuthoredPoint> transitionPoints;
    for(int i=0;i<=800;++i){double x=i;double u=std::clamp((x-200)/400.,0.,1.);double hump=35*std::pow(std::sin(pi*u),4);transitionPoints.push_back({{x,0,15+hump},0,Element::Return});}
    transition.track=compile(transitionPoints,false);buildSupportLayout(transition);validate(transition);
    double minimumGap=40,maximumGap=0;
    for(size_t i=1;i<transition.supports.size();++i){double gap=transition.supports[i].trackDistance-transition.supports[i-1].trackDistance;minimumGap=std::min(minimumGap,gap);maximumGap=std::max(maximumGap,gap);check(gap>=24-1e-8&&gap<=40+1e-8,"Adaptive spans preserve bounded density");}
    check(maximumGap-minimumGap>3,"Curved crest and straight approach receive different support spacing");
    auto groundHugging=circle(4.7);validate(groundHugging);
    for(const auto& s:groundHugging.supports)check(feet(s)==1&&norm(s.top-s.attachment)<1.1,"Low rail uses a shortened connected pier neck while retaining full swept clearance");
    auto nearGround=circle(2);validate(nearGround);
    for(double yaw:{0.,1.1}){
        Design slope;slope.request.terrain.kind=TerrainKind::Highlands;slope.request.terrain.heightMeters=0;
        const double c=std::cos(yaw),s=std::sin(yaw),grade=std::tan(26*pi/180);
        slope.request.terrain.ramps.push_back({0,0,200*c,200*s,20,20+200*grade,grade,grade,100});
        std::vector<AuthoredPoint> points;for(int x=0;x<=200;++x)points.push_back({{x*c,x*s,22+x*grade},0,Element::Launch});
        slope.track=compile(points,false);buildSupportLayout(slope);validate(slope);
        check(!slope.supports.empty(),"Two-metre rail-to-ground on a 26-degree ramp has real connected clear supports");
    }
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
    check(!sideways.supports.empty(),"Rolling open-section fixture produces clearance-checked supports");
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
    for(auto boundary:std::array<Support,2>{footingFixture(-.5+1e-6,1),footingFixture(-1,-1e-6)}){
        check(validateSupportMembers(boundary,flatFootingTerrain).valid(),"Vertical footing touches the actual ground anchoring boundary");
        auto tilted=tiltFixture(boundary);const auto& foot=tilted.members[0];
        const double capExcursion=.5*9e-7/norm(foot.top-foot.base);
        check(foot.base.z+capExcursion>-.5+1e-6||foot.top.z-capExcursion< -1e-6,"Tilted cap actually crosses the tolerated anchoring enclosure");
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
        for(const auto& s:d.supports){size_t n=feet(s);posts+=n==1;bents+=n==2;towers+=n==3;members+=s.members.size();if(s.top.z-s.base.z<20){++lowCount;compactLow+=n<4;maxLowOutreach=std::max(maxLowOutreach,norm(s.top-s.attachment));}}
        auto original=d.supports;buildSupportLayout(d);check(original.size()==d.supports.size(),"Support regeneration is deterministic");
        for(size_t i=0;i<original.size();++i){const auto&a=original[i];const auto&b=d.supports[i];check(a.members.size()==b.members.size(),"Seeded support family stable");for(size_t j=0;j<a.members.size();++j){const auto&x=a.members[j];const auto&y=b.members[j];check(norm(x.base-y.base)==0&&norm(x.top-y.top)==0&&x.radiusBase==y.radiusBase&&x.radiusTop==y.radiusTop&&x.kind==y.kind&&x.spineContact==y.spineContact,"Every persisted support member regenerates exactly");}}
        check(compactLow>lowCount*3/4,"Most low sections shed oversized cantilever towers");
        std::cout<<"TERRAIN "<<d.request.terrain.name()<<" seed42 posts="<<posts<<" bents="<<bents<<" towers="<<towers<<" members="<<members<<" low="<<lowCount<<" compactLow="<<compactLow<<" maxRemainingLowOutreach="<<maxLowOutreach<<'\n';
    }
    std::cout<<"PASS "<<checks<<" support family assertions; proportions are not a structural-load certification.\n";
}catch(const std::exception& e){std::cerr<<"FAIL after "<<checks<<": "<<e.what()<<'\n';return 1;}}
