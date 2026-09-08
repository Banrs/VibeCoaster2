#include "coaster/track_hardware.hpp"
#include <iostream>


#include <stdexcept>

using namespace coaster;
static int count=0;static void check(bool b,const char* message){++count;if(!b)throw std::runtime_error(message);}
// Independent point-to-OBB oracle: project onto each axis and clamp the point,
// without reusing the segment minimizer or its active-face quadratic.
static double pointBoxDistanceSquared(Vec3 point,const StationBox& box){
    const Vec3 relative=point-box.center;
    const double coordinates[]{dot(relative,box.forward),dot(relative,box.right),dot(relative,box.up)};
    const double half[]{box.half.x,box.half.y,box.half.z};double squared=0;
    for(int i=0;i<3;++i){const double nearest=std::clamp(coordinates[i],-half[i],half[i]);const double offset=coordinates[i]-nearest;squared+=offset*offset;}
    return squared;
}
int main(){try{
    Track t;t.closed=false;for(int i=0;i<4;++i)t.knots.push_back({{double(i)*40,0,50},{1,0,0},{},{0,0,1},0,Element::Return});t.rebuild();
    auto sweep=buildClearanceSweep(t,TrainConfig{});
    const auto p=t.sample(50);const auto world=[&](Vec3 v){return p.position+p.tangent*v.x+p.right*v.y+p.up*v.z;};
    for(double side:{-1.,1.}){
        Support gap;gap.members={{{world({-.01,side*.3,-.34})},{world({.01,side*.3,-.34})},.001,.001,SupportMemberKind::Steel,false}};
        // Historical boxes miss this fixture; independently demonstrated in the isolated prototype.
        check(supportCollision(gap,sweep)>=0,"Web must reject former-gap crossing");
        gap.members[0].spineContact=true;gap.hasAttachment=true;gap.attachment=gap.members[0].top;gap.trackDistance=50;
        check(supportCollision(gap,sweep)>=0,"Forged local spine-contact flag must never exempt a web crossing");
        Support joint;joint.hasAttachment=true;joint.trackDistance=50;joint.attachment=world({0,0,-.71});
        joint.members={{world({0,side*1.5,-.71}),joint.attachment,.18,.18,SupportMemberKind::Steel,true}};

        check(supportCollision(joint,sweep)<0,"Actual spine attachment passes web geometry without exemption");
        joint.members[0].spineContact=false;check(supportCollision(joint,sweep)>=0,"Unapproved spine contact remains rejected");
    }
    const auto webs=trackWebsLocal();double radius=0;
    {
        // This first left-web case exposed a 1.692 mm clearance overestimate
        // under MSVC /O2 /fp:precise. Its nearest point is this exact box corner.
        const auto& box=webs[0];const Vec3 a{0,1,-.8},b{1,0,1},delta=b-a;
        const Vec3 corner=box.center+box.forward*box.half.x-box.right*box.half.y-box.up*box.half.z;
        const double parameter=dot(corner-a,delta)/dot(delta,delta);
        const Vec3 separation=a+delta*parameter-corner;const double analytic=dot(separation,separation);
        check(parameter>0&&parameter<1&&std::abs(analytic-.65826115694940687)<1e-12,"Independent corner projection matches analytic reference");
        const double actual=segmentWebDistanceSquared(a,b,box);
        check(std::abs(actual-analytic)<1e-12,"Zero-origin positive-face contribution retains its negative sign");
        check(std::abs(segmentWebDistanceSquared(b,a,box)-analytic)<1e-12,"Segment reversal preserves the analytic minimum");
        const double margin=std::sqrt(analytic)+.001;
        check(actual<=margin*margin,"Millimetre-near web contact is not missed by distance overestimation");
    }
    for(const auto& box:webs){
        for(Vec3 v:trackWebCorners(box)){radius=std::max(radius,norm(v));check(norm(v)<.9,"Preserved hardware radius");}
        check(segmentWebDistanceSquared(box.center,box.center,box)<1e-20,"Interior distance zero");
        auto a=box.center+box.forward*(box.half.x+.2);check(std::abs(segmentWebDistanceSquared(a,a,box)-.04)<1e-12,"Face distance exact");
        auto b=a+box.up*(box.half.z+.3);check(std::abs(segmentWebDistanceSquared(b,b,box)-.13)<1e-12,"Corner distance exact");
        check(segmentWebDistanceSquared(box.center-box.right*3,box.center+box.right*3,box)<1e-20,"Segment interior crossing exact");
        auto a2=a+box.right*3,b2=a-box.right*3;check(std::abs(segmentWebDistanceSquared(a2,b2,box)-.04)<1e-12,"Interior minimizer exact");
        const auto obstacle=StationBox{world({0,box.center.y<0?-.3:.3,-.34}),p.tangent,p.right,p.up,{.001,.001,.001},StationRole::Post};
        check(stationBoxesOverlap(trackWebWorld(box,p),obstacle),"Shared station web catches former-gap obstacle");
        for(int n=0;n<50;++n){const Vec3 a3{std::sin(n)*.9,std::cos(n*.4),-.8+std::sin(n*.7)},b3{std::cos(n*.8),std::sin(n*.3),std::cos(n*.6)};
            const double exact=segmentWebDistanceSquared(a3,b3,box);double dense=1e9;
            for(int k=0;k<=2000;++k){const auto v=a3+(b3-a3)*(double(k)/2000);dense=std::min(dense,pointBoxDistanceSquared(v,box));}
            // Distance to a closed box is 1-Lipschitz. A closest segment point
            // lies within half a sample spacing of one of the sampled points.
            const double lower=std::max(0.,std::sqrt(dense)-norm(b3-a3)/4000);
            check(exact<=dense+1e-10&&exact+1e-10>=lower*lower&&dense-exact<1e-5,"Segment distance agrees with independent dense point oracle and sampling bound");
        }
    }
    std::cout<<"PASS "<<count<<" web checks; corner radius="<<radius<<"; old-gap rejected on both sides; original spine-only exemption preserved\n";return 0;
}catch(const std::exception& e){std::cerr<<"FAIL after "<<count<<": "<<e.what()<<'\n';return 1;}}

