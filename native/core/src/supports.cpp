#include "coaster/coaster.hpp"
#include "coaster/support_fabrication.hpp"
#include "coaster/clearance.hpp"
#include <stdexcept>
#include <queue>
#include <map>
#include <numeric>
#include <tuple>

namespace coaster {
namespace terrain_validation {
double boxLowerBound(const StationBox&,const Terrain&,double);
double lowerBound(const TrackSample&,const Terrain&,double,double,double);
}
namespace {
struct FootingGeometry {Vec3 base,top;double radius;};
FootingGeometry footing(Vec3 p,const Terrain& terrain,double broadRadius,double legRadius,bool shortBent){
    double radius=broadRadius;auto ground=terrain.heightRange(p.x,p.y,radius);
    Vec3 base{p.x,p.y,ground[0]-(shortBent?1.2:1.5)},top{p.x,p.y,terrain.heightRange(p.x,p.y,legRadius)[1]+.2+legRadius*.5};
    if(top.z-base.z>12){
        radius=std::min(broadRadius,std::max(.9,legRadius*1.65));ground=terrain.heightRange(p.x,p.y,radius);
        base.z=ground[0]-1.5;top.z=terrain.heightRange(p.x,p.y,legRadius)[1]+.2+legRadius*.5;
    }
    return {base,top,radius};
}
// The support axes follow the rail frame and local curvature.  Heights select
// load-path families; the shapes are not copies of a decorative tower asset.
Vec3 footingRake(const TrackSample& q,double height){
    const Vec3 bank{-q.up.x,-q.up.y,0},curve{q.curvature.x,q.curvature.y,0};
    return bank*std::min(height*.28,15.)-curve*std::min(height*9.,600.);
}
Support compactBent(const TrackSample& q,Vec3 right,double distance,const Terrain& terrain,bool paired,bool rake=true,TrackProfile profile=TrackProfile::Legacy){
    const auto section=trackSection(profile);const double sectionDepth=section.spineDepth,sectionRadius=section.spineRadius;
    const Vec3 attachment=q.position-q.up*(sectionDepth+sectionRadius);
    const double localHeight=attachment.z-terrain.height(attachment.x,attachment.y);
    const double standoff=std::clamp(localHeight*.16,profile==TrackProfile::Exa?1.5:.18,2.);
    const Vec3 cap=attachment-q.up*standoff;
    const Vec3 centre{cap.x,cap.y,terrain.height(cap.x,cap.y)};
    Support s{centre,cap,attachment,true,distance,{}};
    const double height=cap.z-centre.z;
    if(height<=0||height>(paired?72.:18.)||q.up.z<=.35)return s;
    const double radius=paired?.30+height*.0035:.23+height*.010;
    const Vec3 drift=rake?footingRake(q,height):Vec3{};
    auto add=[&](Vec3 a,Vec3 b,double ra,double rb,SupportMemberKind kind=SupportMemberKind::Steel,bool contact=false){
        s.members.push_back({a,b,ra,rb,kind,contact});
    };
    const double spread=std::clamp(height*.19,2.2,13.5);
    // Low straights need only a raking tube. Taller/banked track receives an
    // asymmetric wishbone: its backstay joins the main leg below the rail.
    const Vec3 root=centre+drift+(paired?right*(-spread*.42):Vec3{});
    const auto first=footing(root,terrain,.75+radius*1.65,radius,true);
    if(cap.z-first.top.z<=.08)return Support{centre,cap,attachment,true,distance,{}};
    add(first.base,first.top,first.radius,std::max(.55,radius*1.35),SupportMemberKind::Footing);
    if(paired){
        const auto second=footing(centre+drift+right*spread,terrain,.75+radius*1.65,radius,true);
        const Vec3 fork=first.top+(cap-first.top)*std::clamp(.69+q.up.z*.08,.69,.77);
        if(fork.z-second.top.z<=.08)return Support{centre,cap,attachment,true,distance,{}};
        add(second.base,second.top,second.radius,std::max(.55,radius*1.35),SupportMemberKind::Footing);
        add(first.top,fork,radius,radius*.88);
        add(second.top,fork,radius*.90,radius*.80);
        add(fork,cap,radius*.88,radius*.78);
    }else add(first.top,cap,radius,radius*.82);
    if(standoff>1.2){
        const Vec3 neck=attachment-q.up*.9;
        add(cap,neck,std::max(.25,radius*.72),.22);
        add(neck,attachment,supportRadius,supportRadius,SupportMemberKind::Steel,true);
    }else add(cap,attachment,supportRadius,supportRadius,SupportMemberKind::Steel,true);
    return s;
}
Support rakedFrame(const TrackSample& q,Vec3 right,Vec3 outreach,double offset,double distance,const Terrain& terrain,double standoff=2,double rakeFactor=1,double rearDirection=1,TrackProfile profile=TrackProfile::Legacy,bool inversion=false){
    const auto section=trackSection(profile);const double sectionDepth=section.spineDepth,sectionRadius=section.spineRadius;
    const Vec3 attachment=q.position-q.up*(sectionDepth+sectionRadius);
    const Vec3 cap=attachment+outreach*offset-q.up*standoff;
    const Vec3 centre{cap.x,cap.y,terrain.height(cap.x,cap.y)};
    Support s{centre,cap,attachment,true,distance,{}};
    const double height=cap.z-centre.z;
    if(height<4||height>600)return s;
    const Vec3 forward=unit(Vec3{q.tangent.x,q.tangent.y,0});
    const Vec3 along=norm(forward)>.5?forward:unit(cross(right,Vec3{0,0,1}));
    const Vec3 drift=footingRake(q,height)*rakeFactor;
    const bool tall=height>72;
    const bool heavy=profile==TrackProfile::Exa;
    const double halfWidth=std::clamp(height*(tall?.145:.18),2.8,42.);
    const double shoulderWidth=tall?(heavy?std::clamp(height*.028,2.,6.):std::clamp(height*.008,.7,2.5)):0.;
    const double headDepth=tall?(heavy?std::clamp(height*.06,4.5,16.):std::clamp(height*.035,3.,10.)):0.;
    // Rolling inversion exits need the same primary-member hierarchy as the
    // adjacent transverse frames, even when their bank requires this topology.
    const double radius=inversion?std::clamp(.60+height*.007,.65,1.85):std::clamp(.32+height*(heavy?.0052:.0032),.34,heavy?1.85:1.6);
    const double topRadius=radius*(inversion?.88:(heavy?.62:.78));
    const double braceRadius=std::clamp(radius*(inversion?.38:(heavy?.29:.40)),.18,inversion?.70:.55);
    auto add=[&](Vec3 a,Vec3 b,double ra,double rb,SupportMemberKind kind=SupportMemberKind::Steel,bool contact=false){
        s.members.push_back({a,b,ra,rb,kind,contact});
    };
    std::array<Vec3,2> feet,shoulders;
    for(int side=0;side<2;++side){
        const double sign=side?1.:-1.;
        const Vec3 p=centre+drift+right*(sign*halfWidth);
        const auto foundation=footing(p,terrain,1.+radius*1.7,radius,false);
        add(foundation.base,foundation.top,foundation.radius,std::max(.6,radius*1.35),SupportMemberKind::Footing);
        feet[side]=foundation.top;
        shoulders[side]=cap+right*(sign*shoulderWidth)-Vec3{0,0,headDepth};
        if(shoulders[side].z-feet[side].z<.1)return Support{centre,cap,attachment,true,distance,{}};
    }
    if(tall){
        // Broad shoulders, open complete bays and substantial primary tubes
        // avoid the dense stack of small bracing panels on every tall bent.
        const int bays=std::max(heavy?3:2,int(std::ceil(height/(heavy?48.:65.))));
        std::array<Vec3,2> toes=feet;
        if(heavy)for(int side=0;side<2;++side){
            toes[side]=feet[side]+unit(shoulders[side]-feet[side])*std::max(.8,radius*3.);
            add(feet[side],toes[side],radius,radius);
        }
        std::array<Vec3,2> previous=toes;
        std::vector<std::array<Vec3,2>> levels{toes};
        Vec3 rearJoint;
        for(int bay=1;bay<=bays;++bay){
            const double u=double(bay)/bays;
            std::array<Vec3,2> node;
            for(int side=0;side<2;++side){
                node[side]=toes[side]*(1-u)+shoulders[side]*u;
                add(previous[side],node[side],radius+(topRadius-radius)*(double(bay-1)/bays),radius+(topRadius-radius)*u);
            }
            add(node[0],node[1],braceRadius,braceRadius);
            if(heavy||bay>1)add(previous[(bay-1)%2],node[bay%2],braceRadius,braceRadius);
            if(bay==bays-1)rearJoint=node[0];
            levels.push_back(node);
            previous=node;
        }
        const double rakeSign=(q.tangent.z>=0?-1.:1.)*rearDirection;
        const auto rear=footing(centre+drift+along*(rakeSign*std::clamp(height*.13,8.,28.)),terrain,1.+radius*1.7,radius,false);
        if(rearJoint.z-rear.top.z<.1)return Support{centre,cap,attachment,true,distance,{}};
        add(rear.base,rear.top,rear.radius,std::max(.6,radius*1.35),SupportMemberKind::Footing);
        if(!heavy)add(rear.top,rearJoint,radius*.86,topRadius*.86);
        else{
            const Vec3 toe=rear.top+unit(rearJoint-rear.top)*(radius*3.);
            add(rear.top,toe,radius*.86,radius*.86);Vec3 rearPrevious=toe;
            for(int k=1;k<bays;++k){
                const double u=double(k)/(bays-1);const Vec3 next=toe+(rearJoint-toe)*u;
                add(rearPrevious,next,(radius+(topRadius-radius)*double(k-1)/(bays-1))*.86,(radius+(topRadius-radius)*u)*.86);
                if(k<bays-1)for(int side=0;side<2;++side)add(next,levels[k][side],braceRadius,braceRadius);
                for(int side=0;side<2;++side){
                    const Vec3 a=k%2?rearPrevious:next,b=levels[k%2?k:k-1][side];
                    if(norm(b-a)>.01&&!(norm(a-rearPrevious)<1e-6&&norm(b-next)<1e-6))add(a,b,braceRadius,braceRadius);
                }
                rearPrevious=next;
            }
        }
        for(const auto& shoulder:shoulders)add(shoulder,cap,topRadius,inversion?topRadius*.90:std::min(.5,topRadius));
    }else{
        for(int side=0;side<2;++side)add(feet[side],cap,radius,topRadius);
    }
    // All head geometry stays below the banked spine; only the small final
    // contact receives the existing own-spine endpoint exemption.
    const Vec3 neck=attachment-q.up*.9;
    if(norm(cap-neck)>.02)add(cap,neck,std::min(.48,topRadius),.22);
    add(neck,attachment,supportRadius,supportRadius,SupportMemberKind::Steel,true);
    return s;
}
}
static ValidationReport validateMemberShapes(const Support& support,const Terrain& terrain,Cancel cancel){
    ValidationReport r;
    if(!terrain.valid()){r.fail("TERRAIN_CONFIG","Support terrain profile is invalid");return r;}
    if(support.members.empty())return r; // Immutable legacy column/arm semantics.
    if(support.members.size()>maxSupportMembers){r.fail("SUPPORT_MEMBER_BUDGET","Too many canonical members in one support");return r;}
    int contacts=0,footings=0;
    for(size_t i=0;i<support.members.size();++i){
        if(cancel&&cancel()){r.fail("CANCELLED","Support member validation cancelled");return r;}
        const auto& m=support.members[i];double radius=std::max(m.radiusBase,m.radiusTop),length=norm(m.top-m.base);
        if(!finite(m.base)||!finite(m.top)||!std::isfinite(m.radiusBase)||!std::isfinite(m.radiusTop)||
           m.radiusBase<=0||m.radiusTop<=0||radius>5||norm(m.base)+radius>1000000||norm(m.top)+radius>1000000||
           length<.01||length>1000||int(m.kind)<0||int(m.kind)>2){r.fail("SUPPORT_MEMBER_CONFIG","Invalid canonical member dimensions or kind");return r;}
        if(m.spineContact){
            ++contacts;
            if(m.kind!=SupportMemberKind::Steel||norm(m.top-support.attachment)>1e-5||m.radiusTop>supportRadius+1e-9||radius>.35||length>100){r.fail("SUPPORT_JOINT","Only a bounded steel endpoint may meet the verified own spine contact");return r;}
        }
        if(m.kind==SupportMemberKind::Footing){
            ++footings;const auto ground=terrain.heightRange(m.base.x,m.base.y,radius+std::hypot(m.top.x-m.base.x,m.top.y-m.base.y));
            const double horizontalDrift=std::hypot(m.top.x-m.base.x,m.top.y-m.base.y);
            // Include the accepted tiny axis tilt in both the entire XY query
            // disk and each cap's vertical extent. Exactly vertical generated
            // footings retain precisely their former enclosure and tolerance.
            const double baseCapRise=m.radiusBase*horizontalDrift/length;
            const double topCapFall=m.radiusTop*horizontalDrift/length;
            // Foundations may be partly buried on a slope. Their full lower cap
            // anchors below soil; exposed steel is checked independently.
            if(m.spineContact||horizontalDrift>1e-6||m.top.z<=m.base.z||length>12||radius<.5||
               m.base.z+baseCapRise>ground[0]-.5+1e-6||
               m.top.z-topCapFall<terrain.height(m.top.x,m.top.y)-1e-6){r.fail("SUPPORT_FOOTING","Footing full shape is not terrain anchored within its supported domain");return r;}
        }else if(m.kind==SupportMemberKind::RockAnchor){
            ++footings;
            const Vec3 axis=unit(m.top-m.base),right=unit(cross(axis,Vec3{0,0,1})),up=cross(axis,right);
            const auto buried=terrain.heightRange(m.base.x,m.base.y,m.radiusBase);
            const StationBox cap{m.top,axis,right,up,{.01,m.radiusTop,m.radiusTop},StationRole::Footing};
            if(m.spineContact||terrain.kind!=TerrainKind::Highlands||length>12||length<2||radius<.5||
               std::hypot(axis.x,axis.y)<.70||axis.z<-.01||
               m.base.z+m.radiusBase>buried[0]-.35||terrain_validation::boxLowerBound(cap,terrain,0)<.03){
                r.fail("SUPPORT_ROCK_ANCHOR","Rock anchor needs an embedded rear cap and an exposed bearing on a steep face");return r;
            }
        }else{
            if(radius>2){r.fail("SUPPORT_MEMBER_CONFIG","Steel member radius exceeds the prototype bound");return r;}
            const int count=std::max(1,int(std::ceil(length/2)));
            for(int k=0;k<count;++k){
                if((k&31)==0&&cancel&&cancel()){r.fail("CANCELLED","Steel terrain validation cancelled");return r;}
                const Vec3 a=m.base+(m.top-m.base)*(double(k)/count),b=m.base+(m.top-m.base)*(double(k+1)/count),middle=(a+b)*.5;
                const double footprint=radius+std::hypot(b.x-a.x,b.y-a.y)*.5;
                const double radialZ=radius*std::hypot(m.top.x-m.base.x,m.top.y-m.base.y)/length;
                const auto ground=terrain.heightRange(middle.x,middle.y,footprint);
                if(std::min(a.z,b.z)-radialZ<ground[1]-1e-6){
                    const Vec3 axis=unit(b-a),right=unit(cross(axis,std::abs(axis.z)<.9?Vec3{0,0,1}:Vec3{0,1,0})),up=cross(axis,right);
                    const StationBox box{middle,axis,right,up,{norm(b-a)*.5,radius,radius},StationRole::Post};
                    if(terrain_validation::boxLowerBound(box,terrain,0)<-1e-6){r.fail("SUPPORT_TERRAIN","Steel solid intersects the terrain enclosure");return r;}
                }
            }

        }
    }
    if(contacts!=1||footings>8)r.fail("SUPPORT_CONNECTIVITY","Explicit attachment needs exactly one spine joint and at most eight footings");
    return r;
}
ValidationReport validateSupportLayout(const std::vector<Support>& supports,const Terrain& terrain,Cancel cancel){
    ValidationReport r;
    if(!terrain.valid()){r.fail("TERRAIN_CONFIG","Support terrain profile is invalid");return r;}
    if(cancel&&cancel()){r.fail("CANCELLED","Support graph validation cancelled");return r;}
    std::vector<const SupportMember*> members;
    for(const auto& support:supports){
        if(support.members.size()>maxTotalSupportMembers-members.size()){r.fail("SUPPORT_MEMBER_BUDGET","Total canonical support member budget exceeded");return r;}
        r=validateMemberShapes(support,terrain,cancel);if(!r.valid())return r;
        for(const auto& member:support.members)members.push_back(&member);
    }
    // Sparse endpoint union, not an O(member_count squared) all-pairs scan.
    // A crossing through the middle of a tube is deliberately NOT a joint.
    std::vector<size_t> parent(members.size());std::iota(parent.begin(),parent.end(),0);
    auto root=[&](size_t i){while(parent[i]!=i){parent[i]=parent[parent[i]];i=parent[i];}return i;};
    using Key=std::tuple<long long,long long,long long>;
    std::map<Key,std::vector<std::pair<Vec3,size_t>>> nodes;
    constexpr double tolerance=1e-5;
    for(size_t i=0;i<members.size();++i){
        if(cancel&&cancel()){r.fail("CANCELLED","Support graph validation cancelled");return r;}
        for(Vec3 p:{members[i]->base,members[i]->top}){
            const auto x=static_cast<long long>(std::floor(p.x/tolerance)),y=static_cast<long long>(std::floor(p.y/tolerance)),z=static_cast<long long>(std::floor(p.z/tolerance));
            for(int dx=-1;dx<=1;++dx)for(int dy=-1;dy<=1;++dy)for(int dz=-1;dz<=1;++dz){
                auto it=nodes.find({x+dx,y+dy,z+dz});if(it==nodes.end())continue;
                for(const auto& [q,j]:it->second)if(norm(p-q)<tolerance)parent[root(i)]=root(j);
            }
            nodes[{x,y,z}].push_back({p,i});
        }
    }
    std::vector<unsigned> flags(members.size());
    for(size_t i=0;i<members.size();++i)flags[root(i)]|=(members[i]->spineContact?1u:0u)|(members[i]->kind!=SupportMemberKind::Steel?2u:0u);
    for(size_t i=0;i<members.size();++i)if(flags[root(i)]!=3){r.fail("SUPPORT_CONNECTIVITY","Every member must reach a spine joint and an actual terrain footing through shared endpoints");return r;}
    return r;
}
ValidationReport validateSupportMembers(const Support& support,const Terrain& terrain,Cancel cancel){
    return validateSupportLayout({support},terrain,std::move(cancel));
}
#include "support_cliff.hpp"
#include "support_assemblies.hpp"
void fitCliffTerrain(Design& design,Cancel cancel){support_cliff::fitTerrain(design,std::move(cancel));}
namespace {
// A short upper branch lets one complete spatial bent carry two adjacent
// mounts. Keep the rail attachment spacing and its fitted saddle unchanged;
// remove the receiver's duplicate tower only after checking the whole graph.
void shareTallBents(Design& d,const ClearanceSweep& sweep,Cancel cancel){
    auto feet=[](const Support& s){return std::count_if(s.members.begin(),s.members.end(),[](const SupportMember& m){return m.kind==SupportMemberKind::Footing;});};
    auto raked=[](const Support& s){const auto n=s.members.size();return n>=6&&s.members.back().spineContact&&
        norm(s.members[n-4].top-s.top)<1e-6&&norm(s.members[n-3].top-s.top)<1e-6&&norm(s.members[n-2].base-s.top)<1e-6;};
    for(size_t i=1;i<d.supports.size();++i){
        if(cancel&&cancel())throw std::runtime_error("CANCELLED");
        const auto& first=d.supports[i-1];const auto& second=d.supports[i];
        if(feet(first)!=3||feet(second)!=3||!raked(first)||!raked(second)||second.trackDistance-first.trackDistance>40.000001)continue;
        const auto qa=d.track.sample(first.trackDistance),qb=d.track.sample(second.trackDistance);
        if(dot(qa.tangent,qb.tangent)<.75)continue;
        if(first.top.z-first.base.z<90||second.top.z-second.base.z<90)continue;
        const size_t carrier=first.top.z>second.top.z?i-1:i,receiver=carrier==i?i-1:i;
        const auto& a=d.supports[carrier];const auto& b=d.supports[receiver];
        // Never bridge through an apex or feed a branch from an already shared
        // receiver. A pair has one ground-connected carrier and one receiver.
        if(qa.tangent.z*qb.tangent.z<-.05||norm(a.top-b.top)>48)continue;
        const Vec3 across=unit(a.members[a.members.size()-3].base-a.members[a.members.size()-4].base);
        if(norm(across)<.5)continue;
        std::array<Vec3,2> landing{};std::array<double,2> best{-INFINITY,-INFINITY};
        double radius=0;for(const auto& m:a.members)if(m.kind==SupportMemberKind::Steel)radius=std::max(radius,m.radiusBase);
        for(const auto& m:a.members)if(m.kind==SupportMemberKind::Steel&&m.radiusBase>radius*.7)for(Vec3 p:{m.base,m.top}){
            if(p.z>b.top.z-12||p.z<b.top.z-55)continue;
            const double side=dot(p-a.top,across);if(std::abs(side)<1)continue;
            const int index=side>0?1:0;if(p.z>best[index]){best[index]=p.z;landing[index]=p;}
        }
        if(!std::isfinite(best[0])||!std::isfinite(best[1]))continue;
        const double branch=std::clamp(radius*.72,.42,.90);
        std::vector<Vec3> branchRoots{a.top};
        // During a bank transition the direct upper chord may cross the rider.
        // A third, non-coplanar primary node can receive a spatial branch below
        // the roll. This is chosen by clearance, not by a bank-angle exemption.
        for(const auto& m:a.members)if(m.kind==SupportMemberKind::Steel&&m.radiusBase>radius*.7){
            const Vec3 p=m.top;
            if(p.z>b.top.z-12||p.z<b.top.z-55||norm(p-landing[0])<3||norm(p-landing[1])<3)continue;
            if(std::none_of(branchRoots.begin(),branchRoots.end(),[&](Vec3 q){return norm(p-q)<1e-5;}))branchRoots.push_back(p);
        }
        for(const Vec3 high:branchRoots){
            if(cancel&&cancel())throw std::runtime_error("CANCELLED");
            const Vec3 plane=cross(landing[0]-high,landing[1]-high);
            if(norm(plane)<4||std::abs(dot(unit(plane),b.top-high))<1.)continue;
            Support reduced=b;
            reduced.members.assign(b.members.end()-2,b.members.end());reduced.base=a.base;
            reduced.members.push_back({high,b.top,branch,branch,SupportMemberKind::Steel,false});
            for(const Vec3 p:landing)reduced.members.push_back({p,b.top,branch*.8,branch*.8,SupportMemberKind::Steel,false});
            // Only the pair's junctions change. The final complete-ride check
            // still runs after assembly, without remeshing it for each trial.
            const std::vector<Support> pair{a,reduced};
            if(!validateSupportLayout(pair,d.request.terrain,cancel).valid())continue;
            if(supportStationCollision(reduced,d.station,cancel))continue;
            const int hit=supportCollision(reduced,sweep,cancel);
            if(hit==-2)throw std::runtime_error("CANCELLED");if(hit>=0)continue;
            if(!validateSupportFabrication(d.track,pair,d.request.terrain,d.station,sweep,cancel).valid())continue;
            d.supports[receiver]=std::move(reduced);++i;break;
        }
    }
}
}
std::vector<SupportRegion> planSupportRegions(const Track& track,const Terrain& terrain,Cancel cancel){
    return support_assemblies::regions(track,terrain,std::move(cancel));
}
void buildSupportLayout(Design& d,Cancel cancel){
    const auto section=trackSection(d.track.profile);const double sectionDepth=section.spineDepth,sectionRadius=section.spineRadius;
    if(!d.request.terrain.valid())throw std::runtime_error("Invalid support terrain profile");
    // An unsuccessful re-site must not replace the caller's working graph with
    // a partial structure. Moving the vector keeps cancellation cheap.
    struct Restore {
        Design& design;std::vector<Support> original;bool complete{false};
        ~Restore(){if(!complete)design.supports=std::move(original);}
    } restore{d,std::move(d.supports)};
    d.supports.clear();const auto sweep=buildClearanceSweep(d.track,d.request.train,cancel);
    const auto regions=planSupportRegions(d.track,d.request.terrain,cancel);size_t nextRegion=0;
    size_t totalMembers=0;
    for(double distance=0;distance<d.track.length;){
        if(cancel&&cancel())throw std::runtime_error("CANCELLED");
        if(nextRegion<regions.size()&&distance>=regions[nextRegion].begin){
            auto region=regions[nextRegion++];
            if(!d.supports.empty())region.begin=std::max(region.begin,d.supports.back().trackDistance+4.);
            if(region.end-region.begin>36){
                auto assembly=support_assemblies::build(d,region,sweep,cancel);
                if(!assembly.empty()){
                    for(auto& s:assembly){totalMembers+=s.members.size();d.supports.push_back(std::move(s));}
                    if(totalMembers>maxTotalSupportMembers)throw std::runtime_error("SUPPORT_MEMBER_BUDGET");
                    distance=region.end+24.;continue;
                }
            }
        }
        double blockedAt=0;std::string failure,cliffFailure;
        auto placeAt=[&](double site){
            const auto q=d.track.sample(site);Vec3 right=unit({q.right.x,q.right.y,0});
            if(norm(right)<.5)right=unit(Vec3{q.tangent.y,-q.tangent.x,0});
            Vec3 attachment=q.position-q.up*(sectionDepth+sectionRadius);bool placed=false;
            const auto tryPlace=[&](Support support){
                if(cancel&&cancel())throw std::runtime_error("CANCELLED");
                if(support.members.empty()){failure="empty support";return false;}
                auto valid=validateSupportMembers(support,d.request.terrain,cancel);
                if(!valid.valid()){failure=valid.errors.front().code;return false;}
                if(supportStationCollision(support,d.station,cancel)){failure="station";return false;}
                int hit=supportCollision(support,sweep,cancel);if(hit==-2)throw std::runtime_error("CANCELLED");
                if(hit>=0){failure="swept member collision";blockedAt=sweep.frames()[size_t(hit)].distance;return false;}
                const auto fittings=validateSupportFabrication(d.track,{support},d.request.terrain,d.station,sweep,cancel);
                if(!fittings.valid()){failure=fittings.errors.front().code+": "+fittings.errors.front().message;if(fittings.errors.front().code=="CANCELLED")throw std::runtime_error("CANCELLED");return false;}
                totalMembers+=support.members.size();
                if(totalMembers>maxTotalSupportMembers)throw std::runtime_error("SUPPORT_MEMBER_BUDGET");
                d.supports.push_back(std::move(support));return true;
            };
            const auto tryFrame=[&](Vec3 frameRight,Vec3 outreach,double offset,double standoff=2.){
                for(double rakeFactor:{1.,0.})for(double rearDirection:{1.,-1.})
                    if(tryPlace(rakedFrame(q,frameRight,outreach,offset,site,d.request.terrain,standoff,rakeFactor,rearDirection,d.track.profile)))return true;
                return false;
            };
            // Steep escarpment drops resolve into the actual rock face. The
            // only buried parts are validated rock sockets, not ordinary steel.
            if(d.track.profile==TrackProfile::Exa)for(double standoff:{2.2,3.2,4.5}){
                auto wall=support_cliff::bracket(d,q,site,standoff);const bool hasWall=!wall.members.empty();
                if(tryPlace(std::move(wall))){placed=true;break;}
                if(hasWall)cliffFailure=failure;
            }
            // Prefer a track-normal tube or wishbone before a broad A-frame.
            if(!placed)placed=tryPlace(compactBent(q,right,site,d.request.terrain,false,true,d.track.profile));
            if(!placed)placed=tryPlace(compactBent(q,right,site,d.request.terrain,true,true,d.track.profile));
            if(!placed)placed=tryPlace(compactBent(q,right,site,d.request.terrain,false,false,d.track.profile));
            // Centred bents first; offsets are selected only by real site clearance.
            if(!placed&&q.up.z>.65){
                const double height=attachment.z-d.request.terrain.height(attachment.x,attachment.y);
                const double nearOffset=std::clamp(height*.025,3.,8.);
                for(double offset:{0.,nearOffset,-nearOffset}){
                    if(tryFrame(right,q.right,offset)){placed=true;break;}
                }
            }
            if(!placed)for(double offset:{12.,-12.,18.,-18.,26.,-26.,36.,-36.,48.,-48.}){
                if(tryFrame(right,q.right,offset)){placed=true;break;}
            }
            // A cliff face may leave no lateral foundation site. Fore/aft
            // cantilevers move the foundations onto real level ground while
            // preserving exactly the same solid and rider checks.
            if(!placed){const Vec3 longitudinal=unit(Vec3{q.tangent.x,q.tangent.y,0});
                for(double offset:{12.,-12.,18.,-18.,26.,-26.,36.,-36.,48.,-48.})
                    if(tryFrame(longitudinal,longitudinal,offset)){placed=true;break;}}
            // On a cliff, move both away from the face and sideways from the
            // lower track. These candidates use the same full solid checks.
            if(!placed){const Vec3 longitudinal=unit(Vec3{q.tangent.x,q.tangent.y,0});
                for(double offset:{18.,26.,36.,48.}){for(double side:{-1.,1.})for(double forward:{-1.,1.})for(double standoff:{2.,6.,10.}){
                    const Vec3 outward=unit(right*side+longitudinal*forward);
                    if(!placed&&tryFrame(right,outward,offset,standoff))placed=true;
                }if(placed)break;}}
            // Larger under-spine standoffs provide additional rider clearance during rolls.
            if(!placed)for(double standoff:{6.,10.}){
                for(double offset:{12.,-12.,18.,-18.,26.,-26.,36.,-36.,48.,-48.}){
                    if(tryFrame(right,q.right,offset,standoff)){placed=true;break;}
                }
                if(placed)break;
            }
            return placed;
        };
        bool placed=placeAt(distance);
        // Attachment stations are a placement corridor, not an immutable grid.
        // Move at most four metres, retaining the maximum structural span and
        // checking the same complete rider, rail, footing and station envelopes.
        if(!placed)for(double shift:{-1.,1.,-2.,2.,-3.,3.,-4.,4.}) {
            const double site=distance+shift,previous=d.supports.empty()?0:d.supports.back().trackDistance;
            if(site<=previous+1||site>=d.track.length||site-previous>40)continue;
            if(placeAt(site)){distance=site;placed=true;break;}
        }
        const auto q=d.track.sample(distance);const auto attachment=q.position-q.up*(sectionDepth+sectionRadius);
        if(!placed)throw std::runtime_error("No validated support placement near "+std::to_string(distance)+"; last conflicting track station="+std::to_string(blockedAt)+"; last reason="+failure+"; wall candidate="+cliffFailure+"; rail="+std::to_string(q.position.z)+"; ground="+std::to_string(d.request.terrain.height(q.position.x,q.position.y)));
        // Curvature and frame-rate look-ahead set support spacing, capped at 40 m.
        double curvature=0,frameRate=0;
        for(double ahead:{0.,10.,20.,30.,40.}){
            const auto k=sampleKinematics(d.track,std::min(distance+ahead,d.track.length));
            curvature=std::max(curvature,norm(k.sample.curvature));
            frameRate=std::max(frameRate,norm(k.upS));
        }
        const double height=attachment.z-d.request.terrain.height(attachment.x,attachment.y);
        // Tall bents need room for their rear raker as well as transverse feet.
        const double minimumSpan=height>90?32.:24.;
        distance+=std::clamp(40./(1+12*curvature+5*frameRate),minimumSpan,40.);
    }
    if(d.track.profile==TrackProfile::Exa)shareTallBents(d,sweep,cancel);
    // Tall neighbouring bents form a single element-scale structure. Shared
    // node endpoints make the longitudinal chords and diagonals real canonical
    // members; no duplicate legs or visual-only truss overlays are created.
    for(size_t i=1;i<d.supports.size();++i){
        if(cancel&&cancel())throw std::runtime_error("CANCELLED");
        auto& a=d.supports[i-1];const auto& b=d.supports[i];
        // Inversion frames deliberately have compact, separate crossheads.
        // Their member order differs from the ordinary tall bent; treating the
        // backstay as a shoulder recreated spurious return chords between them.
        if(d.track.profile==TrackProfile::Exa&&std::any_of(regions.begin(),regions.end(),[&](const SupportRegion& r){
            return r.kind!=SupportStructureKind::Camelback&&b.trackDistance>=r.begin-1e-6&&a.trackDistance<=r.end+1e-6;
        }))continue;
        auto tall=[](const Support& s){return std::count_if(s.members.begin(),s.members.end(),[](const SupportMember& m){return m.kind==SupportMemberKind::Footing;})==3;};
        if(!tall(a)||!tall(b)||b.trackDistance-a.trackDistance>40.000001)continue;
        const auto qa=d.track.sample(a.trackDistance),qb=d.track.sample(b.trackDistance);
        if(dot(qa.tangent,qb.tangent)<.7||dot(qa.right,qb.right)<.7)continue;
        const size_t count=a.members.size();
        const std::array<Vec3,2> shoulderA{a.members[count-4].base,a.members[count-3].base};
        const std::array<Vec3,2> shoulderB{b.members[b.members.size()-4].base,b.members[b.members.size()-3].base};
        auto link=[&](Vec3 from,Vec3 to,double radius){
            Support candidate=a;
            candidate.members.insert(candidate.members.end()-1,{from,to,radius,radius,SupportMemberKind::Steel,false});
            const auto valid=validateSupportMembers(candidate,d.request.terrain,cancel);
            if(!valid.valid()){if(valid.errors.front().code=="CANCELLED")throw std::runtime_error("CANCELLED");return;}
            if(supportStationCollision(candidate,d.station,cancel))return;
            if(cancel&&cancel())throw std::runtime_error("CANCELLED");
            const int hit=supportCollision(candidate,sweep,cancel);
            if(hit==-2)throw std::runtime_error("CANCELLED");
            if(hit>=0)return;
            if(d.track.profile==TrackProfile::Exa){
                auto graph=d.supports;graph[i-1]=candidate;
                if(!validateSupportFabrication(d.track,graph,d.request.terrain,d.station,sweep,cancel).valid())return;
            }
            if(++totalMembers>maxTotalSupportMembers)throw std::runtime_error("SUPPORT_MEMBER_BUDGET");
            a=std::move(candidate);
        };
        const double radius=std::clamp(.18+(a.top.z-a.base.z)*.0006,.22,.4);
        for(int side=0;side<2;++side)link(shoulderA[side],shoulderB[side],radius);
        // The diagonal lands at an existing primary-leg node, not at an
        // arbitrary point on a tube. Alternation is the continuous bracing bay.
        Vec3 lower=a.members[2+(i%2)].top;
        const double level=a.base.z+(a.top.z-a.base.z)*.72;
        for(size_t m=2;m<count-4;++m){const auto& member=a.members[m];
            if(member.kind==SupportMemberKind::Steel&&member.top.z<level&&member.top.z>lower.z&&
               dot(member.top-a.top,qa.right)*(i%2?1.:-1.)>0)lower=member.top;
        }
        link(lower,shoulderB[i%2],radius*.9);
    }
    auto report=validateSupportLayout(d.supports,d.request.terrain,cancel);
    if(!report.valid())throw std::runtime_error(report.errors.front().code);
    report=validateSupportFabrication(d.track,d.supports,d.request.terrain,d.station,sweep,cancel);
    if(!report.valid())throw std::runtime_error(report.errors.front().code+": "+report.errors.front().message);
    restore.complete=true;
}
}
