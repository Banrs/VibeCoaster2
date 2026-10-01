#include "coaster/coaster.hpp"
#include "../../tools/support_examples.hpp"
#include <iostream>
#include <stdexcept>
using namespace coaster;
namespace {
int checks;
void check(bool ok,const char* why){++checks;if(!ok)throw std::runtime_error(why);}
size_t feet(const Design& d){size_t n=0;for(const auto& s:d.supports)for(const auto& m:s.members)n+=m.kind==SupportMemberKind::Footing;return n;}
bool sameGraph(const std::vector<Support>& a,const std::vector<Support>& b){
    const auto same=[](Vec3 x,Vec3 y){return x.x==y.x&&x.y==y.y&&x.z==y.z;};
    if(a.size()!=b.size())return false;
    for(size_t i=0;i<a.size();++i){
        const auto& x=a[i];const auto& y=b[i];
        if(!same(x.base,y.base)||!same(x.top,y.top)||!same(x.attachment,y.attachment)||
            x.hasAttachment!=y.hasAttachment||x.trackDistance!=y.trackDistance||x.members.size()!=y.members.size())return false;
        for(size_t j=0;j<x.members.size();++j){
            const auto& m=x.members[j];const auto& n=y.members[j];
            if(!same(m.base,n.base)||!same(m.top,n.top)||m.radiusBase!=n.radiusBase||m.radiusTop!=n.radiusTop||
                m.kind!=n.kind||m.spineContact!=n.spineContact)return false;
        }
    }
    return true;
}
}
int main(){try{
    for(auto kind:{SupportStructureKind::Camelback,SupportStructureKind::Loop,SupportStructureKind::Immelmann})for(unsigned seed:{0u,1u,3u}){
        auto d=supportExample(kind,seed);const auto original=d.track;
        const auto regions=planSupportRegions(d.track,d.request.terrain);
        check(std::any_of(regions.begin(),regions.end(),[&](const SupportRegion& r){return r.kind==kind;}),"Whole element is recognised from its sampled frame and ground clearance");
        buildSupportLayout(d);size_t shared=0;for(const auto& s:d.supports)shared+=std::none_of(s.members.begin(),s.members.end(),[](const SupportMember& m){return m.kind==SupportMemberKind::Footing;});
        std::cout<<"kind="<<int(kind)<<" seed="<<seed<<" attachments="<<d.supports.size()<<" shared="<<shared<<" feet="<<feet(d)<<std::endl;
        check(shared>=4,"Signature structure has multiple attachments sharing foundations, not one tall frame per site");
        check(validateSupportLayout(d.supports,d.request.terrain).valid(),"All shared nodes have a real path to terrain");
        auto sweep=buildClearanceSweep(d.track,d.request.train);
        for(const auto& s:d.supports)check(supportCollision(s,sweep)<0,"Full element structure clears all track and rider sweeps");
        for(double distance=0;distance<d.track.length;distance+=3.){auto a=original.sample(distance),b=d.track.sample(distance);check(norm(a.position-b.position)+norm(a.up-b.up)==0,"Support layout never changes track to evade the structure");}
        auto unsupported=d.supports;
        for(auto& s:unsupported)std::erase_if(s.members,[](const SupportMember& m){return m.kind==SupportMemberKind::Footing;});
        check(!validateSupportLayout(unsupported,d.request.terrain).valid(),"Shared frame without actual foundations is rejected");
        auto floating=d.supports;floating.front().members.push_back({{900,900,200},{905,900,200},.2,.2,SupportMemberKind::Steel,false});
        check(!validateSupportLayout(floating,d.request.terrain).valid(),"A detached brace cannot borrow an unrelated foundation");
        auto originalSupports=d.supports;buildSupportLayout(d);check(d.supports.size()==originalSupports.size(),"Deterministic attachment count");
        for(size_t i=0;i<d.supports.size();++i){check(d.supports[i].members.size()==originalSupports[i].members.size(),"Deterministic member count");for(size_t j=0;j<d.supports[i].members.size();++j){const auto& a=d.supports[i].members[j];const auto& b=originalSupports[i].members[j];check(norm(a.base-b.base)+norm(a.top-b.top)==0&&a.radiusBase==b.radiusBase&&a.radiusTop==b.radiusTop,"Exact deterministic node positions and sections");}}
    }
    // Isolate terrain from layout/seed: identical canonical track over two
    // surfaces must move/resize real foundations rather than stretch an asset.
    auto flat=supportExample(SupportStructureKind::Camelback,1),slope=flat;flat.request.terrain={};
    buildSupportLayout(flat);buildSupportLayout(slope);
    bool changed=flat.supports.size()!=slope.supports.size();
    for(size_t i=0;i<std::min(flat.supports.size(),slope.supports.size());++i)changed|=norm(flat.supports[i].base-slope.supports[i].base)>.1;
    check(changed,"Changing only ground clearance changes the generated foundation graph");
    int calls=0;auto cancelled=validateSupportLayout(slope.supports,slope.request.terrain,[&]{return ++calls>20;});
    check(!cancelled.valid()&&cancelled.errors.front().code=="CANCELLED","Shared-member validation remains cancellable");
    const auto beforeCancel=slope.supports;
    const auto cancelBuild=[&](Cancel request){
        bool requested=false,cancelledBuild=false;
        try{buildSupportLayout(slope,[&]{requested=requested||request();return requested;});}
        catch(const std::exception& e){if(std::string(e.what())!="CANCELLED")throw;cancelledBuild=true;}
        check(requested,"Regeneration reaches the requested cancellation point");
        check(cancelledBuild,"Regeneration reports cancellation explicitly");
        check(slope.supports.size()==beforeCancel.size(),"Cancelled regeneration retains the previous complete graph");
        check(sameGraph(slope.supports,beforeCancel),"Cancellation restores every support field and canonical member exactly");
    };
    // Cover unwind both before construction and after a partial replacement
    // exists. The latter follows actual progress rather than wall-clock timing.
    cancelBuild([]{return true;});
    calls=0;cancelBuild([&]{return ++calls>100;});
    bool partialGraph=false;
    cancelBuild([&]{
        if(slope.supports.empty())return false;
        partialGraph=slope.supports.size()<beforeCancel.size();return true;
    });
    check(partialGraph,"Mid-build cancellation interrupts a partially generated graph");
    std::cout<<"PASS "<<checks<<" adaptive support system checks\n";
}catch(const std::exception& e){std::cerr<<"FAIL after "<<checks<<": "<<e.what()<<'\n';return 1;}}
