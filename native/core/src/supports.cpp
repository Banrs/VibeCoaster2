#include "coaster/coaster.hpp"
#include <stdexcept>
#include <queue>

namespace coaster {
namespace {
struct FootingGeometry {Vec3 base,top;double radius;};
FootingGeometry footing(Vec3 p,const Terrain& terrain,double broadRadius,double legRadius,bool shortBent){
    const double ground=terrain.height(p.x,p.y);
    double radius=broadRadius,slope=terrain.localSlopeBound(p.x,p.y,radius+2);
    Vec3 base{p.x,p.y,ground-slope*radius-(shortBent?1.2:1.5)};
    Vec3 top=shortBent?Vec3{p.x,p.y,ground+slope*radius+legRadius*(1+slope)+slope+.35}:
        Vec3{p.x,p.y,ground+slope*radius+legRadius+.35};
    if(top.z-base.z>12){
        // Narrow the footing if its calculated embedment exceeds 12 m.
        radius=std::min(broadRadius,std::max(.9,legRadius*1.65));
        slope=terrain.localSlopeBound(p.x,p.y,radius+2);
        base.z=ground-slope*radius-1.5;
        top.z=ground+std::max(slope*radius+.2,legRadius*(1+slope)+slope+.35);
    }
    return {base,top,radius};
}
// Short bents use the same persisted members and collision checks as towers.
Support compactBent(const TrackSample& q,Vec3 right,double distance,const Terrain& terrain,bool paired){
    const Vec3 attachment=q.position-q.up*(spineDepth+spineRadius);
    const double localHeight=attachment.z-terrain.height(attachment.x,attachment.y);
    // Short piers use shorter rail joints.
    double standoff=paired?2.:std::clamp(localHeight-3.,.6,2.);
    Vec3 cap=attachment-q.up*standoff;
    if(!paired&&standoff>.6&&cap.z-terrain.height(cap.x,cap.y)<3){
        // Tilt moves the cap onto different terrain. Shorten the neck at its
        // actual ground location instead of relaxing the pier-height minimum.
        double lower=.6,upper=standoff;
        for(int i=0;i<32;++i){double mid=(lower+upper)*.5;Vec3 trial=attachment-q.up*mid;
            if(trial.z-terrain.height(trial.x,trial.y)>=3)lower=mid;else upper=mid;}
        standoff=lower;cap=attachment-q.up*standoff;
    }
    const Vec3 centre{cap.x,cap.y,terrain.height(cap.x,cap.y)};
    Support s{centre,cap,attachment,true,distance,{}};
    const double height=cap.z-centre.z;
    if(height<3||height>(paired?90.:22.)||q.up.z<(paired?.65:.92)||std::abs(q.tangent.z)>(paired?.65:.35))return s;
    const double radiusBase=paired?.28+height*.004:.24+height*.009;
    const double radiusTop=paired?.20+height*.001:.18+height*.002;
    const double footingRadius=.85+radiusBase*1.8;
    const double halfWidth=paired?std::clamp(1.6+height*.085,2.2,9.):0.;
    auto add=[&](Vec3 a,Vec3 b,double ra,double rb,SupportMemberKind kind=SupportMemberKind::Steel,bool contact=false){
        s.members.push_back({a,b,ra,rb,kind,contact});
    };
    for(int side=0;side<(paired?2:1);++side){
        const Vec3 p=centre+right*(side?halfWidth:-halfWidth);
        const auto foundation=footing(p,terrain,footingRadius,radiusBase,true);
        if(cap.z-foundation.top.z<1)return Support{centre,cap,attachment,true,distance,{}};
        add(foundation.base,foundation.top,foundation.radius,foundation.radius,SupportMemberKind::Footing);
        add(foundation.top,cap,radiusBase,radiusTop);
    }
    add(cap,attachment,supportRadius,supportRadius,SupportMemberKind::Steel,true);
    return s;
}
Support tower(Vec3 attachment,Vec3 right,Vec3 up,Vec3 outreach,double offset,double distance,const Terrain& terrain,bool belowDeck,double standoff=2){
    // Outreach follows the canonical banked rail-right axis. Footprint right stays
    // horizontal: mixing these axes can route the cap beam through riders on a turn.
    Vec3 cap=attachment+outreach*offset-up*standoff;
    Vec3 centre{cap.x,cap.y,terrain.height(cap.x,cap.y)};
    Support s{centre,cap,attachment,true,distance,{}};
    const double height=cap.z-centre.z;
    if(height<4||height>600)return s;
    const Vec3 along=unit(cross(right,Vec3{0,0,1}));
    const double baseWidth=std::clamp(1.8+height*.035,2.1,12.);
    const double topWidth=std::clamp(.7+height*.003,.8,1.8);
    const double legBase=.32+height*.0022,legTop=.19+height*.00045;
    const double braceRadius=.10+height*.00045,ringRadius=.13+height*.0005;
    const double footingRadius=1.2+2.3*legBase;
    const int tiers=std::max(1,int(std::ceil(height/16)));
    std::array<Vec3,4> bottom,top;
    const int sx[4]={-1,1,1,-1},sy[4]={-1,-1,1,1};
    auto add=[&](Vec3 a,Vec3 b,double ra,double rb,SupportMemberKind kind=SupportMemberKind::Steel,bool contact=false){
        s.members.push_back({a,b,ra,rb,kind,contact});
    };
    for(int corner=0;corner<4;++corner){
        Vec3 p=centre+right*(sx[corner]*baseWidth)+along*(sy[corner]*baseWidth);
        const auto foundation=footing(p,terrain,footingRadius,legBase,false);
        add(foundation.base,foundation.top,foundation.radius,foundation.radius,SupportMemberKind::Footing);
        bottom[corner]=foundation.top;
        top[corner]=cap+right*(sx[corner]*topWidth)+along*(sy[corner]*topWidth);
    }
    for(int tier=0;tier<tiers;++tier){
        double u=double(tier)/tiers,v=double(tier+1)/tiers;
        double ra=legBase+(legTop-legBase)*u,rb=legBase+(legTop-legBase)*v;
        for(int corner=0;corner<4;++corner){
            int next=(corner+1)%4;
            Vec3 a=bottom[corner]*(1-u)+top[corner]*u,b=bottom[corner]*(1-v)+top[corner]*v;
            Vec3 c=bottom[next]*(1-u)+top[next]*u,d=bottom[next]*(1-v)+top[next]*v;
            add(a,b,ra,rb);
            add(b,d,ringRadius,ringRadius);
            add(a,d,braceRadius,braceRadius);
            add(c,b,braceRadius,braceRadius);
        }
    }
    for(auto corner:top)add(corner,cap,legTop,legTop);
    // A triangular outrigger carries the long offset to a short, narrow final
    // spine contact. Its two diagonal roots are existing tower tier nodes.
    Vec3 knee=attachment+(belowDeck?up*-1.5:unit(cap-attachment)*1.5);
    add(cap,knee,.45,.24);
    const double rootU=double(tiers-1)/tiers;
    for(int corner=0;corner<4;++corner)if(sx[corner]==(offset>0?-1:1)){
        Vec3 root=bottom[corner]*(1-rootU)+top[corner]*rootU;
        add(root,knee,std::clamp(legTop*.8,.18,.32),.18);
    }
    add(knee,attachment,supportRadius,supportRadius,SupportMemberKind::Steel,true);
    return s;
}
}
ValidationReport validateSupportMembers(const Support& support,const Terrain& terrain,Cancel cancel){
    ValidationReport r;
    if(!terrain.valid()){r.fail("TERRAIN_CONFIG","Support terrain profile is invalid");return r;}
    if(support.members.empty())return r; // Immutable legacy column/arm semantics.
    if(support.members.size()>maxSupportMembers){r.fail("SUPPORT_MEMBER_BUDGET","Too many canonical members in one support");return r;}
    int contacts=0,footings=0;size_t contactIndex=0;
    for(size_t i=0;i<support.members.size();++i){
        if(cancel&&cancel()){r.fail("CANCELLED","Support member validation cancelled");return r;}
        const auto& m=support.members[i];double radius=std::max(m.radiusBase,m.radiusTop),length=norm(m.top-m.base);
        if(!finite(m.base)||!finite(m.top)||!std::isfinite(m.radiusBase)||!std::isfinite(m.radiusTop)||
           m.radiusBase<=0||m.radiusTop<=0||radius>5||norm(m.base)+radius>1000000||norm(m.top)+radius>1000000||
           length<.01||length>1000||int(m.kind)<0||int(m.kind)>1){r.fail("SUPPORT_MEMBER_CONFIG","Invalid canonical member dimensions or kind");return r;}
        if(m.spineContact){
            ++contacts;contactIndex=i;
            if(m.kind!=SupportMemberKind::Steel||norm(m.top-support.attachment)>1e-5||m.radiusTop>supportRadius+1e-9||radius>.35||length>100){r.fail("SUPPORT_JOINT","Only a bounded steel endpoint may meet the verified own spine contact");return r;}
        }
        if(m.kind==SupportMemberKind::Footing){
            ++footings;const double ground=terrain.height(m.base.x,m.base.y);
            const double horizontalDrift=std::hypot(m.top.x-m.base.x,m.top.y-m.base.y);
            // Include the accepted tiny axis tilt in both the entire XY query
            // disk and each cap's vertical extent. Exactly vertical generated
            // footings retain precisely their former enclosure and tolerance.
            const double footprintRadius=radius+horizontalDrift;
            const double slope=terrain.localSlopeBound(m.base.x,m.base.y,footprintRadius);
            const double baseCapRise=m.radiusBase*horizontalDrift/length;
            const double topCapFall=m.radiusTop*horizontalDrift/length;
            // Only footings may intersect terrain; their full solid must anchor.
            if(m.spineContact||horizontalDrift>1e-6||m.top.z<=m.base.z||length>12||radius<.5||
               m.base.z+baseCapRise>ground-slope*footprintRadius-.5+1e-6||
               m.top.z-topCapFall<ground+slope*footprintRadius+.2-1e-6){r.fail("SUPPORT_FOOTING","Footing full shape is not terrain anchored within its supported domain");return r;}
        }else{
            if(radius>2){r.fail("SUPPORT_MEMBER_CONFIG","Steel member radius exceeds the prototype bound");return r;}
            int count=std::max(1,int(std::ceil(length/2)));
            // A max-radius capsule conservatively encloses the tapered solid. The
            // slope bound covers terrain between probes and across the full radius.
            for(int k=0;k<=count;++k){
                if((k&31)==0&&cancel&&cancel()){r.fail("CANCELLED","Steel terrain validation cancelled");return r;}
                Vec3 p=m.base+(m.top-m.base)*(double(k)/count);
                const double slope=terrain.localSlopeBound(p.x,p.y,radius+length/count*.5);
                double interpolationMargin=slope*length/count*.5;
                if(p.z-terrain.height(p.x,p.y)<radius*(1+slope)+interpolationMargin-1e-6){r.fail("SUPPORT_TERRAIN","Steel solid intersects the conservative terrain envelope");return r;}
            }
        }
    }
    if(contacts!=1||footings<1||footings>8){r.fail("SUPPORT_CONNECTIVITY","Explicit support needs exactly one spine joint and one to eight footings");return r;}
    // A visible member may not float. Endpoints must form one graph reaching
    // the spine joint and terrain anchors. Overlapping mid-spans do not count.
    std::vector<bool> reached(support.members.size());std::queue<size_t> pending;reached[contactIndex]=true;pending.push(contactIndex);
    while(!pending.empty()){
        if(cancel&&cancel()){r.fail("CANCELLED","Support connectivity validation cancelled");return r;}
        size_t i=pending.front();pending.pop();const auto& a=support.members[i];
        for(size_t j=0;j<support.members.size();++j)if(!reached[j]){const auto& b=support.members[j];
            if(norm(a.base-b.base)<1e-5||norm(a.base-b.top)<1e-5||norm(a.top-b.base)<1e-5||norm(a.top-b.top)<1e-5){reached[j]=true;pending.push(j);}}
    }
    if(std::find(reached.begin(),reached.end(),false)!=reached.end())r.fail("SUPPORT_CONNECTIVITY","Canonical member is disconnected from its spine joint and footings");
    return r;
}
void buildSupportLayout(Design& d,Cancel cancel){
    if(!d.request.terrain.valid())throw std::runtime_error("Invalid support terrain profile");
    d.supports.clear();const auto sweep=buildClearanceSweep(d.track,d.request.train,cancel);
    size_t totalMembers=0;
    for(double distance=0;distance<d.track.length;){
        if(cancel&&cancel())throw std::runtime_error("CANCELLED");
        const auto q=d.track.sample(distance);Vec3 right=unit({q.right.x,q.right.y,0});
        if(norm(right)<.5)right=unit(Vec3{q.tangent.y,-q.tangent.x,0});
        Vec3 attachment=q.position-q.up*(spineDepth+spineRadius);bool placed=false;
        const auto tryPlace=[&](Support support){
            if(cancel&&cancel())throw std::runtime_error("CANCELLED");
            if(support.members.empty())return false;
            auto valid=validateSupportMembers(support,d.request.terrain,cancel);
            if(!valid.valid())return false;
            if(supportStationCollision(support,d.station,cancel))return false;
            int hit=supportCollision(support,sweep,cancel);if(hit==-2)throw std::runtime_error("CANCELLED");
            if(hit>=0)return false;
            totalMembers+=support.members.size();
            if(totalMembers>maxTotalSupportMembers)throw std::runtime_error("SUPPORT_MEMBER_BUDGET");
            d.supports.push_back(std::move(support));return true;
        };
        // Try a single post, then a paired bent, before taller supports.
        placed=tryPlace(compactBent(q,right,distance,d.request.terrain,false));
        if(!placed)placed=tryPlace(compactBent(q,right,distance,d.request.terrain,true));
        // Try centred and nearby towers before larger cantilever offsets.
        if(!placed&&q.up.z>.65){
            const double height=attachment.z-d.request.terrain.height(attachment.x,attachment.y);
            const double nearOffset=std::clamp(height*.025,3.,8.);
            for(double offset:{0.,nearOffset,-nearOffset}){
                if(tryPlace(tower(attachment,right,q.up,q.right,offset,distance,d.request.terrain,d.station.enabled))){placed=true;break;}
            }
        }
        if(!placed)for(double offset:{12.,-12.,18.,-18.,26.,-26.,36.,-36.,48.,-48.}){
            if(tryPlace(tower(attachment,right,q.up,q.right,offset,distance,d.request.terrain,d.station.enabled))){placed=true;break;}
        }
        // Larger under-spine standoffs provide additional rider clearance during rolls.
        if(!placed)for(double standoff:{6.,10.}){
            for(double offset:{12.,-12.,18.,-18.,26.,-26.,36.,-36.,48.,-48.}){
                if(tryPlace(tower(attachment,right,q.up,q.right,offset,distance,d.request.terrain,d.station.enabled,standoff))){placed=true;break;}
            }
            if(placed)break;
        }
        if(!placed)throw std::runtime_error("No validated connected tower placement at distance "+std::to_string(distance));
        // Curvature and frame-rate look-ahead set support spacing, capped at 40 m.
        double curvature=0,frameRate=0;
        for(double ahead:{0.,10.,20.,30.,40.}){
            const auto k=sampleKinematics(d.track,std::min(distance+ahead,d.track.length));
            curvature=std::max(curvature,norm(k.sample.curvature));
            frameRate=std::max(frameRate,norm(k.upS));
        }
        const double height=attachment.z-d.request.terrain.height(attachment.x,attachment.y);
        // Tall tower bases need room along the rail as well as across it.
        const double minimumSpan=height>90?32.:24.;
        distance+=std::clamp(40./(1+12*curvature+5*frameRate),minimumSpan,40.);
    }
}
}
