#include "coaster/track_hardware.hpp"
#include <iostream>


#include <stdexcept>

using namespace coaster;
static int count=0;static void check(bool b,const char* message){++count;if(!b)throw std::runtime_error(message);}
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
            for(int k=0;k<=2000;++k){const auto v=a3+(b3-a3)*(double(k)/2000);dense=std::min(dense,segmentWebDistanceSquared(v,v,box));}
            check(exact<=dense+1e-10&&dense-exact<1e-5,"Segment distance agrees with independent dense point oracle");
        }
    }
    std::cout<<"PASS "<<count<<" web checks; corner radius="<<radius<<"; old-gap rejected on both sides; original spine-only exemption preserved\n";return 0;
}catch(const std::exception& e){std::cerr<<"FAIL after "<<count<<": "<<e.what()<<'\n';return 1;}}

