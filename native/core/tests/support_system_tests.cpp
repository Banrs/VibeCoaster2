#include "coaster/coaster.hpp"
#include "../../tools/support_examples.hpp"
#include <iostream>
#include <stdexcept>
using namespace coaster;
namespace {
int checks;
void check(bool ok,const char* why){++checks;if(!ok)throw std::runtime_error(why);}
size_t feet(const Design& d){size_t n=0;for(const auto& s:d.supports)for(const auto& m:s.members)n+=m.kind==SupportMemberKind::Footing;return n;}
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
    const auto beforeCancel=slope.supports;calls=0;bool cancelledBuild=false;
    try{buildSupportLayout(slope,[&]{return ++calls>100;});}catch(const std::exception& e){cancelledBuild=std::string(e.what())=="CANCELLED";}
    check(cancelledBuild&&slope.supports.size()==beforeCancel.size(),"Cancelled regeneration retains the previous complete graph");
    for(size_t i=0;i<beforeCancel.size();++i)check(slope.supports[i].members.size()==beforeCancel[i].members.size()&&norm(slope.supports[i].base-beforeCancel[i].base)==0,"Cancellation does not replace existing supports");
    std::cout<<"PASS "<<checks<<" adaptive support system checks\n";
}catch(const std::exception& e){std::cerr<<"FAIL after "<<checks<<": "<<e.what()<<'\n';return 1;}}
