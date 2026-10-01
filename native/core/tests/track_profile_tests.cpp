#include "coaster/coaster.hpp"
#include "coaster/track_hardware.hpp"
#include "coaster/support_fabrication.hpp"
#include "coaster/track_mesh.hpp"
#include "coaster/operation_hardware.hpp"
#include <map>
#include "../../tools/support_examples.hpp"
#include <iostream>
#include <stdexcept>
using namespace coaster;
namespace {
int checks=0;
void check(bool value,const char* why){++checks;if(!value)throw std::runtime_error(why);}
}
int main(){try{
    const auto legacy=trackSection(TrackProfile::Legacy),exa=trackSection(TrackProfile::Exa);
    check(legacy.spineDepth==spineDepth&&legacy.spineRadius==spineRadius&&legacy.bottom()==trainEnvelopeBottom,"Legacy section must retain saved geometry");
    check(exa.gauge==1.4&&exa.railRadius==.105&&exa.spineRadius==.34&&exa.spineDepth==.8&&exa.tieHeight==0,"Accepted Exa dimensions");
    check(exa.trainWidth==2.35&&exa.hardwarePadding()>.06,"Exa body width and larger hardware motion reserve");
    auto bad=GenerationRequest{};bad.trackProfile=TrackProfile(99);check(!validateRequest(bad).valid(),"Unknown profiles rejected");
    for(auto profile:{TrackProfile::Legacy,TrackProfile::Exa}){
        const auto section=trackSection(profile);
        for(const auto& part:trackHardwareLocal(profile))for(Vec3 v:trackWebCorners(part))
            check(norm(v)<section.hardwareRadius,"Hardware radius must enclose all clearance solids");
        auto d=supportExample(SupportStructureKind::Camelback,0);d.track.profile=profile;d.request.trackProfile=profile;
        const auto original=d.track.sample(d.track.length*.5);
        const auto sweep=buildClearanceSweep(d.track,d.request.train);
        check(sweep.trackProfile()==profile&&sweep.trainBottom()==section.bottom(),"Sweep retains the physical profile");
        buildSupportLayout(d);
        for(const auto& s:d.supports){
            const auto q=d.track.sample(s.trackDistance);
            check(norm(s.attachment-(q.position+q.up*section.bottom()))<1e-8,"Generated contact uses this section's spine");
            check(supportCollision(s,sweep)<0,"Profile-specific support and hardware clearance");
        }
        const auto after=d.track.sample(d.track.length*.5);
        check(norm(after.position-original.position)+norm(after.up-original.up)==0,"Profile never changes the accepted path");
        if(profile==TrackProfile::Exa){
            const auto rail=trackTubeMesh(d.track,0,10,-.7,0,.105);
            const auto q=d.track.sample(0);const Vec3 centre=q.position-q.right*.7;
            check(std::abs(norm(rail.positions[0]-centre)-.105)<1e-9,"Runtime rail outer radius remains 105 mm");
            check(std::abs(norm(rail.positions[rail.positions.size()/2]-centre)-.085)<1e-9,"Runtime rail retains 20 mm wall");
            for(const auto& v:exaCrosshead(q).vertices){const Vec3 d=v-q.position;
                check(std::abs(dot(d,q.up))<1.2&&std::abs(dot(d,q.right))<=.620000001,"Centred crosshead bounds");}
            auto legacyTrack=d.track;legacyTrack.profile=TrackProfile::Legacy;
            const auto wrongSweep=buildClearanceSweep(legacyTrack,d.request.train);
            check(!validateSupportFabrication(d.track,d.supports,d.request.terrain,d.station,wrongSweep).valid(),"Mismatched clearance profile rejected");
            auto parts=buildSupportFabrication(d.track,d.supports);
            check(!parts.empty(),"Exa fittings generated");
            size_t vertices=0;
            for(const auto& part:parts){
                std::map<std::pair<uint32_t,uint32_t>,std::pair<int,int>> edges;
                double volume=0;const Vec3 origin=part.vertices.front();
                for(const auto& f:part.faces){
                    for(size_t i=0;i<f.size();++i){auto a=f[i],b=f[(i+1)%f.size()];auto& e=edges[std::minmax(a,b)];++e.first;e.second+=a<b?1:-1;}
                    for(size_t i=1;i+1<f.size();++i)volume+=dot(part.vertices[f[0]]-origin,cross(part.vertices[f[i]]-origin,part.vertices[f[i+1]]-origin));
                }
                for(const auto& [key,e]:edges)check(e.first==2&&e.second==0,"Fabricated solids must be closed and consistently wound");
                if(volume<=1e-12)throw std::runtime_error("Nonpositive volume: "+part.name);
                auto mesh=fabricationMesh(part);vertices+=mesh.positions.size();
                check(mesh.positions.size()==mesh.normals.size(),"Mesh normal cardinality");
                for(auto n:mesh.normals)check(finite(n)&&std::abs(norm(n)-1)<1e-6,"Finite unit normals");
            }
            std::cout<<"Fabrication: "<<parts.size()<<" parts, "<<vertices<<" vertices\n";
            const auto report=validateSupportFabrication(d.track,d.supports,d.request.terrain,d.station,sweep);
            if(!report.valid())throw std::runtime_error(report.errors.front().code+": "+report.errors.front().message);
        }
    }
    std::cout<<"PASS "<<checks<<" track profile checks\n";return 0;
}catch(const std::exception& e){std::cerr<<"FAIL after "<<checks<<": "<<e.what()<<'\n';return 1;}}
