#include "coaster/support_fabrication.hpp"
#include "../../tools/support_examples.hpp"
#include <iostream>
using namespace coaster;
int main(){try{
    for(auto kind:{SupportStructureKind::Camelback,SupportStructureKind::Loop,SupportStructureKind::Immelmann})for(unsigned seed:{0u,1u,3u}){
        auto d=supportExample(kind,seed);d.track.profile=d.request.trackProfile=TrackProfile::Exa;
        buildSupportLayout(d);const auto sweep=buildClearanceSweep(d.track,d.request.train);
        const auto shared=std::count_if(d.supports.begin(),d.supports.end(),[](const Support& s){
            return std::none_of(s.members.begin(),s.members.end(),[](const SupportMember& m){return m.kind==SupportMemberKind::Footing;});
        });
        if(kind==SupportStructureKind::Camelback&&shared<4)throw std::runtime_error("Camelback frame silently fell back to independent columns");
        if(kind!=SupportStructureKind::Camelback){
            const auto regions=planSupportRegions(d.track,d.request.terrain);
            for(const auto& region:regions)if(region.kind!=SupportStructureKind::Camelback){
                double previous=region.begin;size_t count=0,directHeads=0;
                for(const auto& s:d.supports)if(s.trackDistance>=region.begin-1e-6&&s.trackDistance<=region.end+1e-6){
                    if(s.trackDistance-previous>50.01)throw std::runtime_error("Inversion attachment span exceeds the planned spacing");
                    previous=s.trackDistance;++count;
                    std::vector<Vec3> directions;
                    for(const auto& m:s.members)if(m.kind==SupportMemberKind::Steel&&!m.spineContact){
                        if(norm(m.base-s.top)<1e-6)directions.push_back(unit(m.top-m.base));
                        if(norm(m.top-s.top)<1e-6)directions.push_back(unit(m.base-m.top));
                    }
                    bool through=false;for(size_t a=0;a<directions.size();++a)for(size_t b=a+1;b<directions.size();++b)through|=dot(directions[a],directions[b])<-.999;
                    directHeads+=through;
                    if(std::count_if(s.members.begin(),s.members.end(),[](const SupportMember& m){return m.kind==SupportMemberKind::Footing;})==1)
                        throw std::runtime_error("Inversion regressed to a single pillar");
                }
                if(count<4||region.end-previous>50.01)throw std::runtime_error("Incomplete inversion support coverage");
                if(directHeads<2)throw std::runtime_error("Inversion lost its direct through crossheads");
            }
        }
        auto report=validateSupportFabrication(d.track,d.supports,d.request.terrain,d.station,sweep);
        std::cout<<"kind="<<int(kind)<<" seed="<<seed<<" supports="<<d.supports.size()<<std::endl;
        if(!report.valid())throw std::runtime_error(report.errors.front().code+": "+report.errors.front().message);
        // A station obstacle intersecting a real flange must be rejected even
        // though it does not intersect the original thin contact member.
        const auto s=d.supports.front();const auto q=d.track.sample(s.trackDistance);
        StationGeometry obstacle;obstacle.enabled=true;
        obstacle.boxes.push_back({q.position+q.right*.46+q.up*(-1.74),q.tangent,q.right,q.up,{.08,.08,.08},StationRole::Post});
        report=validateSupportFabrication(d.track,d.supports,d.request.terrain,obstacle,sweep);
        if(report.valid())throw std::runtime_error("Flange obstacle was not rejected");
        report=validateSupportFabrication(d.track,d.supports,d.request.terrain,d.station,sweep,[]{return true;});
        if(report.valid()||report.errors.front().code!="CANCELLED")throw std::runtime_error("Fabrication cancellation failed");
        if(kind==SupportStructureKind::Camelback&&seed==0){
            auto obstructed=d.supports;
            obstructed.front().members.push_back({s.attachment-q.up*3,q.position+q.up*1.5,.1,.1,SupportMemberKind::Steel,false});
            report=validateSupportFabrication(d.track,obstructed,d.request.terrain,d.station,sweep);
            if(report.valid()||(report.errors.front().code!="FABRICATION_RIDER"&&report.errors.front().code!="FABRICATION_TRACK"))throw std::runtime_error("Added steel intrusion escaped swept clearance");
        }
    }
    // A steep upright hill must keep its approaches in the same element
    // frame. World-up shrinks with pitch even though the track is not inverted.
    auto steep=supportExample(SupportStructureKind::Camelback,0);
    std::vector<AuthoredPoint> points;
    for(int i=0;i<=300;++i){const double x=i*2.-300,u=std::clamp((x+250)/500.,0.,1.);
        points.push_back({{x,0,18+229*std::pow(std::sin(pi*u),2)},0,Element::Hill,{0,0,1}});}
    steep.track=compile(points,false);steep.track.profile=steep.request.trackProfile=TrackProfile::Exa;
    const auto regions=planSupportRegions(steep.track,steep.request.terrain);
    const auto crest=std::find_if(regions.begin(),regions.end(),[](const SupportRegion& r){return r.kind==SupportStructureKind::Camelback;});
    if(crest==regions.end()||steep.track.sample(crest->begin).position.z>50||steep.track.sample(crest->end).position.z>50)
        throw std::runtime_error("Steep upright approaches were cut out of the camelback frame");
    buildSupportLayout(steep);
    const auto steepSweep=buildClearanceSweep(steep.track,steep.request.train);
    const auto steepReport=validateSupportFabrication(steep.track,steep.supports,steep.request.terrain,steep.station,steepSweep);
    if(!steepReport.valid())throw std::runtime_error("Steep camelback fabrication does not clear its terrain and track");
    if(std::count_if(steep.supports.begin(),steep.supports.end(),[](const Support& s){return std::none_of(s.members.begin(),s.members.end(),[](const SupportMember& m){return m.kind==SupportMemberKind::Footing;});})<12)
        throw std::runtime_error("Steep camelback reverted to independent approach columns");
    // Tall versions exercise the braced family at actual large-ride scale;
    // the compact 55--70 m inversion fixtures alone cannot cover this path.
    for(auto kind:{SupportStructureKind::Loop,SupportStructureKind::Immelmann})for(unsigned seed:{0u,3u}){
        auto d=supportExample(kind,seed,2.5);d.track.profile=d.request.trackProfile=TrackProfile::Exa;
        buildSupportLayout(d);
        const auto report=validateSupportFabrication(d.track,d.supports,d.request.terrain,d.station,buildClearanceSweep(d.track,d.request.train));
        if(!report.valid())throw std::runtime_error("Tall inversion fabrication: "+report.errors.front().code);
        size_t towers=0,portals=0;
        for(const auto& s:d.supports)if(s.top.z-d.request.terrain.height(s.top.x,s.top.y)>100){
            const auto feet=std::count_if(s.members.begin(),s.members.end(),[](const SupportMember& m){return m.kind==SupportMemberKind::Footing;});
            if(feet&&d.track.sample(s.trackDistance).up.z<-.4&&feet<3)throw std::runtime_error("Tall inverted arc lost its spatial backstay");
            portals+=feet>=3;
            ++towers;
        }
        if(towers<2)throw std::runtime_error("Tall inversion did not exercise braced towers");
        if(portals<2)throw std::runtime_error("Tall inversion silently fell back to the old independent bents");
    }
    auto twist=supportExample(SupportStructureKind::Camelback,0);
    std::vector<AuthoredPoint> twistedPoints;
    for(int i=0;i<=300;++i){const double x=i*2.-300,u=std::clamp((x+250)/500.,0.,1.);
        const double bank=1.05*smooth(std::clamp(x/160.,0.,1.))*(1-smooth(std::clamp((x-160)/100.,0.,1.)));
        twistedPoints.push_back({{x,0,18+147*std::pow(std::sin(pi*u),2)},bank,Element::Hill,{0,0,1}});
    }
    twist.track=compile(twistedPoints,false);twist.track.profile=twist.request.trackProfile=TrackProfile::Exa;
    const auto twistRegions=planSupportRegions(twist.track,twist.request.terrain);
    if(std::none_of(twistRegions.begin(),twistRegions.end(),[](const SupportRegion& r){return r.kind==SupportStructureKind::TwistedDrop;}))
        throw std::runtime_error("Banked descending hill was not distinguished from a camelback");
    buildSupportLayout(twist);
    const auto twistReport=validateSupportFabrication(twist.track,twist.supports,twist.request.terrain,twist.station,buildClearanceSweep(twist.track,twist.request.train));
    if(!twistReport.valid())throw std::runtime_error("Bank-aligned drop frame clearance failed");
    for(const auto& region:twistRegions)if(region.kind==SupportStructureKind::TwistedDrop){
        double previous=region.begin;size_t mounts=0,shared=0,foundations=0;
        for(const auto& s:twist.supports)if(s.trackDistance>=region.begin&&s.trackDistance<=region.end){
            if(s.trackDistance-previous>55.01)throw std::runtime_error("Twisted drop has an unsupported interval");
            previous=s.trackDistance;++mounts;
            const auto feet=std::count_if(s.members.begin(),s.members.end(),[](const SupportMember& m){return m.kind==SupportMemberKind::Footing;});
            foundations+=feet;shared+=feet==0;
        }
        if(mounts<4||region.end-previous>55.01)throw std::runtime_error("Twisted-drop coverage incomplete");
        if(shared<8||foundations>12)throw std::runtime_error("Twisted drop regressed to a forest of independent bents");
    }
    // Independent position, yaw, size and ground profile must still produce a
    // connected shared frame. Exercise this outside the canonical ride's axes.
    for(double scale:{.8,1.3}){
        auto varied=supportExample(SupportStructureKind::Camelback,3);
        std::vector<AuthoredPoint> rotated;const double yaw=.63,c=std::cos(yaw),s=std::sin(yaw);
        for(const auto& point:twistedPoints){const auto p=point.position;
            rotated.push_back({{(c*p.x-s*p.y)*scale,(s*p.x+c*p.y)*scale,p.z*scale+24},point.bank,Element::Hill,{0,0,1}});}
        varied.track=compile(rotated,false);varied.track.profile=varied.request.trackProfile=TrackProfile::Exa;
        buildSupportLayout(varied);
        const auto report=validateSupportFabrication(varied.track,varied.supports,varied.request.terrain,varied.station,buildClearanceSweep(varied.track,varied.request.train));
        if(!report.valid())throw std::runtime_error("Resized/rotated girder lost terrain or train clearance");
        size_t shared=0;for(const auto& support:varied.supports)
            shared+=std::none_of(support.members.begin(),support.members.end(),[](const SupportMember& m){return m.kind==SupportMemberKind::Footing;});
        if(shared<6)throw std::runtime_error("Resized/rotated drop silently lost its shared structure");
    }
    // Rock bearings are a distinct persisted material domain. A real wall can
    // accept a buried socket; relabelling free-standing steel is insufficient.
    for(double yaw:{0.,.63}){
        Design wall;auto& t=wall.request.terrain;t.kind=TerrainKind::Highlands;t.heightMeters=200;
        t.plateau=.8;t.radiusX=t.radiusY=700;t.cliffWidth=40;t.cliffHeading=yaw;
        const Vec3 n{std::cos(yaw),std::sin(yaw),0};t.centerX=-150*n.x;t.centerY=-150*n.y;
        std::vector<AuthoredPoint> p;
        for(int i=0;i<=100;++i){const double z=168-i;double lo=0,hi=50;
            for(int k=0;k<40;++k){const double x=(lo+hi)*.5;if(t.height(n.x*x,n.y*x)>z)lo=x;else hi=x;}
            p.push_back({n*((lo+hi)*.5+13)+Vec3{0,0,z},0,Element::Hill,n});
        }
        wall.track=compile(p,false);wall.track.profile=wall.request.trackProfile=TrackProfile::Exa;
        const auto original=wall.track;fitCliffTerrain(wall);buildSupportLayout(wall);
        size_t sockets=0,groundFeet=0;for(const auto& s:wall.supports)for(const auto& m:s.members){sockets+=m.kind==SupportMemberKind::RockAnchor;groundFeet+=m.kind==SupportMemberKind::Footing;}
        if(sockets<3||groundFeet)throw std::runtime_error("Steep wall drop fell back to ground towers");
        for(const auto& s:wall.supports){const auto roots=std::count_if(s.members.begin(),s.members.end(),[](const SupportMember& m){return m.kind==SupportMemberKind::RockAnchor;});
            if(roots<1||roots>2)throw std::runtime_error("Wall bracket lost its cantilever/haunch topology");}
        const auto checked=validateSupportFabrication(wall.track,wall.supports,t,wall.station,buildClearanceSweep(wall.track,wall.request.train));
        if(!checked.valid())throw std::runtime_error("Rock bracket fabrication: "+checked.errors.front().code);
        for(double at=0;at<original.length;at+=3)if(norm(original.sample(at).position-wall.track.sample(at).position)>1e-9)
            throw std::runtime_error("Cliff fitting changed the canonical track");
        auto bad=wall.supports;auto& anchor=*std::find_if(bad[0].members.begin(),bad[0].members.end(),[](const SupportMember& m){return m.kind==SupportMemberKind::RockAnchor;});
        anchor.base=anchor.base+n*35;anchor.top=anchor.top+n*35;
        const auto invalid=validateSupportLayout(bad,t);
        if(invalid.valid()||invalid.errors.front().code!="SUPPORT_ROCK_ANCHOR")throw std::runtime_error("Unembedded rock socket accepted");
        const auto before=t;bool cancelled=false;try{fitCliffTerrain(wall,[]{return true;});}catch(const std::exception& e){cancelled=std::string(e.what())=="CANCELLED";}
        if(!cancelled||!(t==before))throw std::runtime_error("Cliff fit cancellation modified terrain");
    }
    std::cout<<"PASS 9 fabrication families/terrain cases, 4 tall inversions, steep camelback, rotated rock brackets, flange obstruction and cancellation\n";
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
