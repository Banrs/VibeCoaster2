#include "coaster/coaster.hpp"
#include "coaster/track_hardware.hpp"
#include <iostream>
#include <stdexcept>
using namespace coaster;
static int checks=0;static void check(bool x,const char* m){++checks;if(!x)throw std::runtime_error(m);}
static bool hitBox(Vec3 a,Vec3 b,Vec3 lo,Vec3 hi){double begin=0,end=1;Vec3 d=b-a;double aa[]={a.x,a.y,a.z},dd[]={d.x,d.y,d.z},ll[]={lo.x,lo.y,lo.z},hh[]={hi.x,hi.y,hi.z};for(int i=0;i<3;++i){if(std::abs(dd[i])<1e-12){if(aa[i]<ll[i]||aa[i]>hh[i])return false;}else{double x=(ll[i]-aa[i])/dd[i],y=(hh[i]-aa[i])/dd[i];if(x>y)std::swap(x,y);begin=std::max(begin,x);end=std::min(end,y);if(begin>end)return false;}}return true;}
static void coverage(const Track& t,const ClearanceSweep& sweep,size_t stride=1){
    size_t previousSpan=0;double end=0;bool first=true;
    for(size_t index=0;index<sweep.frames().size();++index){const auto& f=sweep.frames()[index];
        check(f.parameterBegin>=0&&f.parameterEnd>f.parameterBegin&&f.parameterEnd<=1,"Cell has a nonempty canonical interval");
        if(first||f.span!=previousSpan){if(!first)check(end==1&&f.span==previousSpan+1,"Coverage spans are consecutive and complete");check(f.parameterBegin==0,"Coverage begins at span origin");}else check(f.parameterBegin==end,"Canonical interval coverage has no gap");
        first=false;previousSpan=f.span;end=f.parameterEnd;
        check(f.arcLengthBound<=.04*(1+1e-9)&&f.angularVariationBound<=.08*(1+1e-9),"Motion certificate respects fixed budgets");
        if(index%stride)continue;
        const auto& m=f.sample;
        for(double part:{0.,.125,.25,.5,.75,.875,1.}){
            auto q=t.sampleSpan(f.span,f.parameterBegin+(f.parameterEnd-f.parameterBegin)*part);
            check(norm(q.position-m.position)<=f.arcLengthBound*.5+1e-9,"Exact position obeys parameter-interval arc bound");
            double theta=std::acos(std::clamp((dot(q.tangent,m.tangent)+dot(q.right,m.right)+dot(q.up,m.up)-1)*.5,-1.,1.));
            check(theta<=f.angularVariationBound*.5+1e-7,"Actual orthonormal frame obeys angular variation bound");
            for(double x:{-1.275,1.275})for(double y:{-1.5,1.5})for(double z:{-.8,3.6}){
                auto a=q.position+q.tangent*x+q.right*y+q.up*z,b=m.position+m.tangent*x+m.right*y+m.up*z;
                check(norm(a-b)<=f.arcLengthBound*.5+norm(Vec3{x,y,z})*f.angularVariationBound*.5+1e-8,"All body corners obey the shared continuous displacement bound");
                check(norm(a-b)<sweep.padding(),"Complete supported body remains inside the unchanged .20m pad");
            }
            for(const auto& web:trackWebsLocal())for(Vec3 v:trackWebCorners(web)){
                check(norm(v)<.9,"New web corners preserve canonical hardware radius");
                auto a=q.position+q.tangent*v.x+q.right*v.y+q.up*v.z,b=m.position+m.tangent*v.x+m.right*v.y+m.up*v.z;
                check(norm(a-b)<.06,"New web corners remain inside continuous motion pad");
            }
            for(Vec3 v:std::array<Vec3,4>{{{.07,.825,-.27},{.16,.16,-.71},{.085,.735,.085},{.085,-.735,-.085}}}){
                auto a=q.position+q.tangent*v.x+q.right*v.y+q.up*v.z,b=m.position+m.tangent*v.x+m.right*v.y+m.up*v.z;
                check(norm(a-b)<.06,"Canonical hardware remains inside unchanged .06m pad");
            }
        }
    }
    check(!first&&previousSpan+1==t.spans.size()&&end==1,"Coverage finishes at final canonical endpoint");
}
int main(){try{
    std::vector<AuthoredPoint> points;for(int i=0;i<=100;++i){double a=.119*(i-50);points.push_back({{double(i),0,50},a,Element::Return,rotate({0,0,1},{1,0,0},a)});}Track t=compile(points,false);TrainConfig train;
    auto sweep=buildClearanceSweep(t,train);check(sweep.frames().size()>=2500,"Canonical cell coverage prepared");
    Support member;member.members={{{49.99,-1.49,52.39},{50.01,-1.49,52.39},.01,.01,SupportMemberKind::Steel,false}};
    bool sampled=false;for(double distance:{49.,51.}){auto p=t.sample(distance);auto local=[&](Vec3 a){a=a-p.position;return Vec3{dot(a,p.tangent),dot(a,p.right),dot(a,p.up)};};sampled|=hitBox(local(member.members[0].base),local(member.members[0].top),{-1.23,-1.68,-.18},{1.23,1.68,2.58});}
    check(!sampled,"Historical two-frame diagnostic reproduces missed intermediate corner");check(supportCollision(member,sweep)>=0,"Canonical sweep catches the rolled intermediate member");
    auto far=member;far.members[0].base.y=far.members[0].top.y=100;check(supportCollision(far,sweep)<0,"Spatial broad phase retains true distant misses");
    // Raising the configured POV seat also raises the conservative physical headroom.
    train.seatHeight=3;auto high=buildClearanceSweep(t,train);check(high.trainTop()==3.6,"Configured headroom included");Support overhead;overhead.members={{{49.99,0,53.55},{50.01,0,53.55},.01,.01,SupportMemberKind::Steel,false}};check(supportCollision(overhead,high)>=0,"High rider headroom cannot be skipped");
    // Tiny-angle false zero caused by non-unit endpoint dot products.
    Track bad;bad.closed=false;bad.knots.resize(4);for(int i=0;i<4;++i)bad.knots[i]={{i*.0001,0,50},{1,0,0},{},rotate({0,0,1.000005},{1,0,0},i?.004:0),0,Element::Return};bad.rebuild();
    check(dot(bad.knots[0].up,bad.knots[1].up)>1,"Unnormalized-dot alias fixture is real");
    auto rapid=buildClearanceSweep(bad,TrainConfig{});coverage(bad,rapid);
    bool rejected=false;
    // Adaptive coverage uses the real quintic bank derivative even on very
    // short spans; an endpoint-angle estimate cannot certify this case.
    for(size_t i=0;i<bad.knots.size();++i)bad.knots[i].bank=(i%2?1.2:-1.2);bad.rebuild();
    auto rapidBank=buildClearanceSweep(bad,TrainConfig{});check(rapidBank.frames().size()>100,"Angular variation drives subdivision independently of arc length");coverage(bad,rapidBank);
    coverage(t,high,13);
    std::vector<AuthoredPoint> curved;double angle=0;
    for(int i=0;i<=30;++i){angle+=.025+.004*(i%3);Vec3 tangent{std::cos(angle),std::sin(angle),0};curved.push_back({{80*std::sin(angle),80*(1-std::cos(angle)),50},.4*std::sin(3*angle),Element::Turn,rotate({0,0,1},tangent,.3*std::cos(2*angle))});}
    auto curve=compile(curved,false);auto curveSweep=buildClearanceSweep(curve,train);coverage(curve,curveSweep,3);
    Track singular;singular.closed=false;for(int i=0;i<4;++i)singular.knots.push_back({{double(i),0,50},{1,0,0},{},{0,0,i%2?-1.:1.},0,Element::Return});singular.rebuild();
    rejected=false;try{buildClearanceSweep(singular,TrainConfig{});}catch(const std::exception&){rejected=true;}check(rejected,"Interior raw-up singularity cannot gain a continuous sweep certificate");

    for(int kind=0;kind<5;++kind){auto stale=t;if(kind==0)stale.spans[0].c[0].z+=1;if(kind==1)stale.spans[0].c[7].z+=1;if(kind==2)stale.spans[0].referenceUp[5].z+=.01;if(kind==3)stale.spans[0].bank[5]+=.01;if(kind==4)stale.spans[0].referenceUp[0].x=std::numeric_limits<double>::quiet_NaN();rejected=false;try{buildClearanceSweep(stale,TrainConfig{});}catch(...){rejected=true;}check(rejected,"Every canonical position and frame cache must match its rebuilt knots");}
    int calls=0;rejected=false;try{buildClearanceSweep(t,TrainConfig{},[&]{return ++calls>10;});}catch(...){rejected=true;}check(rejected,"Cancellation inside sweep construction");
    calls=0;check(supportCollision(member,sweep,[&]{return ++calls>1;})==-2,"Cancellation inside spatial collision query");
    for(size_t i=0;i<t.spans.size();i+=7)for(double u:{0.,.1,.5,.9,1.}){auto a=t.sampleSpan(i,u);auto b=t.sample(t.distanceAtSpan(i,u));check(norm(a.position-b.position)<1e-10&&norm(a.up-b.up)<1e-10,"Exact parameter frames share the canonical distance adapter");}
    std::cout<<"PASS "<<checks<<" continuous sweep checks: rolled corner, adaptive quintic frame/bank bounds, normalized-angle alias, headroom, stale cache, cancellation and shared frame adapter\n";return 0;
}catch(const std::exception& e){std::cerr<<"FAIL after "<<checks<<": "<<e.what()<<'\n';return 1;}}

