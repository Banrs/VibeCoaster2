#include "coaster/coaster.hpp"
#include "coaster/support_mesh.hpp"
#include <iostream>
#include <stdexcept>
using namespace coaster;
namespace {
int checks=0;
void check(bool condition,const char* message){++checks;if(!condition)throw std::runtime_error(message);}
size_t feet(const Support& s){return std::count_if(s.members.begin(),s.members.end(),[](const SupportMember& m){return m.kind==SupportMemberKind::Footing;});}
Design circle(double height,TerrainKind terrain=TerrainKind::Flat,double bank=0,const Terrain* profile=nullptr){
    Design d;d.request.terrain.kind=terrain;if(profile)d.request.terrain=*profile;
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
int main(int argc,char** argv){try{
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
    for(double height:{3.4,4.1,4.7})for(double bank:{0.,.1}){
        auto groundHugging=circle(height,TerrainKind::Flat,bank);validate(groundHugging);
        for(const auto& s:groundHugging.supports){
            check(feet(s)==1&&norm(s.top-s.attachment)<1.1,"Low rail uses a shortened connected pier neck while retaining full swept clearance");
            check(s.top.z-s.members.front().top.z>=1,"Low post retains usable steel above its actual foundation");
        }
    }
    // Lowering a cap along tilted rail-up also changes its ground location.
    // The actual footing top must leave room for its steel post.
    Design slopedLow;slopedLow.request.terrain=Terrain::seeded(TerrainKind::Hills,9);
    std::vector<AuthoredPoint> slopedLowPoints;
    for(int i=0;i<=20;++i)slopedLowPoints.push_back({{-263.03-.534*i,390.986+.845*i,11.086+.041*i},0,Element::Return});
    slopedLow.track=compile(slopedLowPoints,false);buildSupportLayout(slopedLow);validate(slopedLow);
    check(slopedLow.supports.size()==1&&feet(slopedLow.supports.front())==1,"Low inclined track on sloped ground retains a connected compact pier");
    const auto& slopedPier=slopedLow.supports.front();
    check(slopedPier.top.z-slopedPier.members.front().top.z>=1,"Actual sloped foundation leaves at least one metre of steel post");
    auto lower=circle(7),higher=circle(20);validate(lower);validate(higher);
    check(higher.supports[0].members[1].radiusBase>lower.supports[0].members[1].radiusBase,"Post thickness adapts to height");
    check(higher.supports[0].members[0].radiusBase>lower.supports[0].members[0].radiusBase,"Footing radius adapts to height");
    auto banked=circle(45,TerrainKind::Flat,.55);validate(banked);
    for(const auto& s:banked.supports)check(feet(s)==2,"Moderately banked sections remain connected paired bents");
    Design sideways;std::vector<AuthoredPoint> rollingPoints;
    for(int i=0;i<=200;++i)rollingPoints.push_back({{double(i),0,100},pi*smooth(i/80.),Element::Inversion});
    sideways.track=compile(rollingPoints,false);buildSupportLayout(sideways);validate(sideways);
    // A clear ordinary placement is preferable here; the frozen production
    // regression independently exercises the additional bank-normal candidates.
    check(!sideways.supports.empty(),"Rolling open-section fixture produces certified supports");
    auto hill=circle(45,TerrainKind::Hills),canyon=circle(100,TerrainKind::Canyon);validate(hill);validate(canyon);
    bool changedTerrainHeights=false;
    for(const auto& s:hill.supports)if(feet(s)==2)changedTerrainHeights|=std::abs(s.members[0].base.z-s.members[2].base.z)>1e-5;
    check(changedTerrainHeights,"Each paired footing adapts independently to local terrain");
    // Support-specific regression: a rotated, compressed seeded cliff needs a
    // larger footing envelope than the former fixed canyon slope of .30.
    auto cliffProfile=Terrain::seeded(TerrainKind::Canyon,42);
    cliffProfile.horizontalScale=.5;cliffProfile.verticalScale=.4;
    check(cliffProfile.valid()&&cliffProfile.slopeBound()>.30,"Cliff footprint fixture exceeds the historical slope bound");
    double cliffTop=-1e30;
    for(int i=0;i<360;++i){double a=2*pi*i/360;cliffTop=std::max(cliffTop,cliffProfile.height(200*std::cos(a),200*std::sin(a)));}
    auto cliff=circle(cliffTop+35,TerrainKind::Canyon,0,&cliffProfile);validate(cliff);
    size_t cliffFootings=0;
    for(const auto& s:cliff.supports)for(const auto& m:s.members)if(m.kind==SupportMemberKind::Footing){
        ++cliffFootings;const double radius=std::max(m.radiusBase,m.radiusTop);
        for(int i=0;i<32;++i){double a=2*pi*i/32;double ground=cliffProfile.height(m.base.x+radius*std::cos(a),m.base.y+radius*std::sin(a));
            check(m.base.z<=ground-.5&&m.top.z>=ground+.2,"Full footing circumference remains terrain anchored on rotated cliff");}
    }
    check(cliffFootings>0,"Cliff footprint fixture exercises real canonical footings");
    // A tall tower on a steep wall must fit its steel and anchoring within the
    // same supported foundation depth, without moving the track or its tower.
    Design tallWall;tallWall.request.terrain.kind=TerrainKind::Canyon;
    tallWall.request.terrain.verticalScale=.12;tallWall.request.terrain.cliffHeight=210;tallWall.request.terrain.cliffWidth=170;
    std::vector<AuthoredPoint> wallTrack;
    const double wallCentre=260+tallWall.request.terrain.cliffWidth*.5;
    const double towerElevation=tallWall.request.terrain.height(0,wallCentre)+360;
    for(int i=0;i<=20;++i)wallTrack.push_back({{double(i),wallCentre,towerElevation},0,Element::Return});
    tallWall.track=compile(wallTrack,false);buildSupportLayout(tallWall);validate(tallWall);
    check(tallWall.supports.size()==1&&feet(tallWall.supports.front())==4,"Steep-wall tall tower has four connected anchored feet");
    for(const auto& member:tallWall.supports.front().members)if(member.kind==SupportMemberKind::Footing){
        check(member.top.z-member.base.z<=12,"Solved footing retains the existing maximum depth");
        for(int i=0;i<64;++i){const double angle=2*pi*i/64;
            const double ground=tallWall.request.terrain.height(member.base.x+member.radiusBase*std::cos(angle),member.base.y+member.radiusBase*std::sin(angle));
            check(member.base.z<=ground-.5&&member.top.z>=ground+.2,"Independent footing circumference clears both anchoring faces");
        }
    }
    for(double elevation:{500.,600.}){
        Design tall;std::vector<AuthoredPoint> points;
        for(int x=0;x<=20;++x)points.push_back({{double(x),0,elevation},0,Element::Return});
        tall.track=compile(points,false);buildSupportLayout(tall);validate(tall);
        check(tall.supports.size()==1&&feet(tall.supports.front())==4,"The complete declared tall family fits its canonical resource budget");
        const auto& tower=tall.supports.front();
        check(norm(tower.top-tower.attachment)<2.00000001,"A clear tall tower retains its natural spine joint instead of lengthening it to evade a member count");
        check(tower.members.size()>512&&tower.members.size()<=maxSupportMembers,"Upper tower tiers remain represented and accepted within the derived count bound");
    }
    Design stacked;std::vector<AuthoredPoint> stackedPoints;
    for(int i=0;i<=720;++i){const double t=2*pi*i/720;
        stackedPoints.push_back({{200*std::sin(t),100*std::sin(2*t),200+100*std::cos(t)},0,Element::Return});
    }
    stacked.track=compile(stackedPoints,true);buildSupportLayout(stacked);validate(stacked);
    // The first generated footing can correctly be on the flat floor/rim.
    // Put this negative control at the actual wall derivative maximum instead.
    const double wallY=260+.5*cliffProfile.cliffWidth;
    const double heading=cliffProfile.headingRadians;
    Vec3 wall{cliffProfile.offsetX-cliffProfile.horizontalScale*std::sin(heading)*wallY,
        cliffProfile.offsetY+cliffProfile.horizontalScale*std::cos(heading)*wallY,0};
    wall.z=cliffProfile.height(wall.x,wall.y);
    constexpr double wallRadius=.5;
    const double wallSlope=cliffProfile.localSlopeBound(wall.x,wall.y,wallRadius);
    check(wallSlope>.30,"Negative footing control lies on an actual steep wall disk");
    const Vec3 wallBase=wall-Vec3{0,0,wallSlope*wallRadius+.6};
    const Vec3 wallTop=wall+Vec3{0,0,wallSlope*wallRadius+.3};
    const Vec3 wallJoint=wallTop+Vec3{0,0,.1};
    // Isolated canonical support-definition fixture, not an accepted ride.
    Support wallSupport{wall,wallTop,wallJoint,true,0,{
        {wallBase,wallTop,wallRadius,wallRadius,SupportMemberKind::Footing,false},
        {wallTop,wallJoint,supportRadius,supportRadius,SupportMemberKind::Steel,true}}};
    check(validateSupportMembers(wallSupport,cliffProfile).valid(),"Full-depth wall footing positive control is geometrically valid");
    auto wallColumn=wallSupport;wallColumn.attachment.z+=4;wallColumn.members.back().top.z+=4;
    for(int i=0;i<128;++i){const double angle=2*pi*i/128;
        const double ground=cliffProfile.height(wallTop.x+supportRadius*std::cos(angle),wallTop.y+supportRadius*std::sin(angle));
        check(wallTop.z-supportRadius>ground,"Independent lower cylinder enclosing the entire vertical steel clears the wall");
    }
    check(validateSupportMembers(wallColumn,cliffProfile).valid(),"Vertical steel travel does not consume horizontal terrain clearance or extra footing depth");
    {
        Terrain hills;hills.kind=TerrainKind::Hills;hills.horizontalScale=.5;
        const Vec3 a{pi*180-450,0,4},b{pi*180+450,0,4},end=b+Vec3{0,0,1};
        const Vec3 ground{a.x,0,hills.height(a.x,0)},footTop=ground+Vec3{0,0,1};
        Support crossing{ground,b,end,true,0,{
            {ground-Vec3{0,0,1},footTop,.5,.5,SupportMemberKind::Footing,false},
            {footTop,a,supportRadius,supportRadius,SupportMemberKind::Steel,false},
            {a,b,supportRadius,supportRadius,SupportMemberKind::Steel,false},
            {b,end,supportRadius,supportRadius,SupportMemberKind::Steel,true}}};
        check(hills.valid()&&norm(b-a)<1000,"Interior crossing control stays within supported terrain and member domains");
        check(a.z-hills.height(a.x,a.y)>supportRadius&&b.z-hills.height(b.x,b.y)>supportRadius,"Both steel endpoints clear terrain");
        const auto middle=(a+b)*.5;
        check(middle.z<hills.height(middle.x,middle.y),"Independent beam midpoint penetrates the intervening hill");
        const auto rejected=validateSupportMembers(crossing,hills);
        check(!rejected.valid()&&rejected.errors.front().code=="SUPPORT_TERRAIN","Continuous steel enclosure rejects an interior terrain crossing despite clear endpoints");
    }
    auto undersized=wallSupport;
    undersized.members.front().base.z=wall.z-.30*wallRadius-.5;
    bool actualAnchorGap=false;
    for(int i=0;i<64;++i){double angle=2*pi*i/64;double ground=cliffProfile.height(wall.x+wallRadius*std::cos(angle),wall.y+wallRadius*std::sin(angle));
        actualAnchorGap|=undersized.members.front().base.z>ground-.5;}
    check(actualAnchorGap,"Old .30 envelope actually misses terrain anchoring around the wall footing");
    auto insufficient=validateSupportMembers(undersized,cliffProfile);
    check(insufficient.errors.size()==1&&insufficient.errors.front().code=="SUPPORT_FOOTING","Under-depth steep-wall footing fails specifically SUPPORT_FOOTING");
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
    auto wallBoundary=wallSupport;
    wallBoundary.members[0].base.z=wall.z-wallSlope*wallRadius-.5+1e-6;
    check(validateSupportMembers(wallBoundary,cliffProfile).valid(),"Vertical wall boundary is the valid local-disk positive control");
    auto tiltedWall=tiltFixture(wallBoundary);
    auto tiltedWallReport=validateSupportMembers(tiltedWall,cliffProfile);
    check(tiltedWallReport.errors.size()==1&&tiltedWallReport.errors.front().code=="SUPPORT_FOOTING","Tilted wall footing includes XY drift in the local terrain disk");
    auto invalidProfile=cliffProfile;invalidProfile.horizontalScale=0;
    check(!validateSupportMembers(cliff.supports.front(),invalidProfile).valid(),"Invalid terrain profile fails before support validation");
    if(argc==2&&std::string(argv[1])=="--terrain-footprints-only"){std::cout<<"PASS "<<checks<<" focused support terrain-footprint checks\n";return 0;}
    int calls=0;bool cancelled=false;
    try{buildSupportLayout(low,[&]{return ++calls>12;});}catch(const std::exception& e){cancelled=std::string(e.what())=="CANCELLED";}
    check(cancelled,"New placement path preserves cancellation");
    for(const auto kind:{TerrainKind::Flat,TerrainKind::Hills,TerrainKind::Canyon}){
        GenerationRequest request;request.seed=42;request.terrain.kind=kind;request.targets.requireIntensity=false;
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
