// Included by supports.cpp: element planning and connected multi-attachment
// structures share the same canonical member, terrain and collision functions.
namespace support_assemblies {
inline void cancelled(const Cancel& cancel){if(cancel&&cancel())throw std::runtime_error("CANCELLED");}
struct Probe {double s,height;TrackSample q;};
inline std::vector<SupportRegion> regions(const Track& track,const Terrain& terrain,Cancel cancel){
    if(!terrain.valid()||track.spans.empty()||!std::isfinite(track.length)||track.length<=0||track.length>1000000)
        throw std::runtime_error("Invalid support planning domain");
    std::vector<Probe> p;std::vector<SupportRegion> out;
    const int count=std::max(1,int(std::ceil(track.length/6.)));
    for(int i=0;i<=count;++i){
        cancelled(cancel);const double s=std::min(track.length-1e-6,track.length*i/count);auto q=track.sample(s);
        p.push_back({s,q.position.z-terrain.height(q.position.x,q.position.y),q});
    }
    // Orientation, not a frozen seed, element name or world axis, locates the
    // inversion. Split at the next upright valley so a loop/Immelmann pair can
    // have distinct frames. Include their inclined shoulders in the structure.
    for(size_t i=0;i<p.size();++i)if(p[i].q.up.z<-.2){
        size_t first=i,last=i;
        while(first>0&&(p[first].q.up.z<.8||std::abs(p[first].q.tangent.z)>.45))--first;
        while(last+1<p.size()&&(p[last].q.up.z<.8||std::abs(p[last].q.tangent.z)>.45))++last;
        if(last>first+3){
            const auto a=unit(Vec3{p[first].q.tangent.x,p[first].q.tangent.y,0}),b=unit(Vec3{p[last].q.tangent.x,p[last].q.tangent.y,0});
            out.push_back({p[first].s,p[last].s,dot(a,b)<-.25?SupportStructureKind::Immelmann:SupportStructureKind::Loop});
        }
        i=last;
    }
    // A prominent upright crest spans its two lower shoulders. A tall level
    // straight or a track following a raised terrain shelf is not a camelback.
    // The world-Z component of up also decreases with pitch. Requiring .65 on
    // Exa approaches cut a steep camelback off above 200 m and filled the rest
    // with independent towers. Positive up admits the steep upright shoulders;
    // the inversion regions above still prevent combining different elements.
    const double uprightLimit=track.profile==TrackProfile::Exa?.05:.65;
    for(size_t i=1;i+1<p.size();++i){
        if(p[i].height<72||p[i].q.up.z<.65||p[i].q.position.z<=p[i-1].q.position.z||p[i].q.position.z<p[i+1].q.position.z)continue;
        size_t first=i,last=i;const double floor=std::max(18.,p[i].height*.12);
        while(first>0&&p[first].height>floor&&p[first-1].q.up.z>uprightLimit&&p[first].s-p[first-1].s<8&&p[i].s-p[first-1].s<420){
            if(first+1<i&&p[first-1].q.position.z>p[first].q.position.z+.05)break;--first;
        }
        while(last+1<p.size()&&p[last].height>floor&&p[last+1].q.up.z>uprightLimit&&p[last+1].s-p[i].s<420){
            if(last>i+1&&p[last+1].q.position.z>p[last].q.position.z+.05)break;++last;
        }
        if(i-first<3||last-i<3||p[i].q.position.z-std::max(p[first].q.position.z,p[last].q.position.z)<30)continue;
        bool overlap=false;for(const auto& r:out)overlap|=p[first].s<r.end+12&&p[last].s>r.begin-12;
        if(!overlap){
            auto kind=SupportStructureKind::Camelback;
            const Vec3 across=unit(Vec3{p[i].q.right.x,p[i].q.right.y,0});
            if(track.profile==TrackProfile::Exa)for(size_t j=i+1;j<=last;++j)
                if(p[j].q.tangent.z<-.35&&std::abs(dot(p[j].q.up,across))>.35){kind=SupportStructureKind::TwistedDrop;break;}
            out.push_back({p[first].s,p[last].s,kind});
        }
    }
    std::sort(out.begin(),out.end(),[](const SupportRegion& a,const SupportRegion& b){return a.begin<b.begin;});
    return out;
}
#include "support_bridge.hpp"
// Inversions use discrete transverse frames. The former secondary tube followed
// the complete outside of the loop and made an extra hoop with return bends.
// Here the short crosshead has a straight through chord, so the two rakers and
// normal track mount meet its wall instead of competing for a pointed mitre.
inline Support inversionBent(const Design& d,double distance,double spread,double reach,double standoff,Vec3 across,double lateralMin,double lateralMax){
    const auto q=d.track.sample(distance);const auto section=trackSection(d.track.profile);
    const Vec3 attachment=q.position-q.up*(section.spineDepth+section.spineRadius);
    const Vec3 hub=attachment-q.up*standoff;
    const auto& terrain=d.request.terrain;
    const double height=hub.z-terrain.height(hub.x,hub.y);
    Support out{{hub.x,hub.y,terrain.height(hub.x,hub.y)},hub,attachment,true,distance,{}};
    if(height<5)return out;
    // A banked roll needs its own track-normal bent; a horizontal crosshead
    // would pass through the train when the normal points sideways.
    if(std::abs(dot(across,q.up))>.30)return out;
    const Vec3 along=unit(cross(across,Vec3{0,0,1}));
    Vec3 outward={-q.up.x,-q.up.y,0};
    outward=outward-across*dot(outward,across);
    if(norm(outward)>.12)outward=unit(outward);else outward=along;
    // Upright shoulders can use direct splayed rakers at this scale; reserve
    // the spatial frames for the inverted upper arc and higher mounts.
    const bool tall=height>(q.up.z>.20?125.:85.);
    const double radius=std::clamp(.60+height*(tall?.0070:.0060),.65,1.85);
    const double topRadius=radius*.88;
    // Near an inverted crest, the legs have to pass outside the train before
    // descending. On the near-vertical shoulders a compact direct bipod fits.
    const double headWidth=(q.up.z<-.35?3.2+radius:std::max(1.3,radius*2.4))*spread;
    const Vec3 drift=outward*(std::clamp(height*.20,4.,32.)+reach);
    auto add=[&](Vec3 a,Vec3 b,double r){if(norm(b-a)>.01)out.members.push_back({a,b,r,r,SupportMemberKind::Steel,false});};
    auto foot=[&](Vec3 p){
        auto f=footing(p,terrain,1.+radius*1.8,radius,false);
        f.top.z=std::max(f.top.z,terrain.heightRange(p.x,p.y,radius*1.6)[1]+radius+.25);
        out.members.push_back({f.base,f.top,f.radius,std::max(.6,radius*1.35),SupportMemberKind::Footing,false});return f.top;
    };
    std::array<Vec3,2> feet,heads;
    for(int side=0;side<2;++side){const double sign=side?1.:-1.;
        const double lateral=dot(hub,across);
        const double width=tall?std::max(headWidth,sign*((side?lateralMax:lateralMin)-lateral)+3.2+radius):headWidth;
        heads[side]=hub+across*(sign*width);
        feet[side]=heads[side]+drift+across*(sign*std::clamp(height*.08,2.,16.)*spread);
        if(!tall){feet[side]=foot(feet[side]);if(heads[side].z-feet[side].z<2)return Support{};}
        // A slightly larger receiver avoids a tangent, zero-wall fishmouth
        // where two equal-diameter inclined legs meet the crosshead.
        const double headerRadius=topRadius*1.20;
        add(heads[side],hub,headerRadius);
        // A small closed end beyond the leg makes this a welded T rather than
        // an extended sharp elbow. It does not curve back around the track.
        add(heads[side],heads[side]+across*(sign*(topRadius*1.35+.04)),headerRadius);
    }
    if(!tall){for(int side=0;side<2;++side)out.members.push_back({feet[side],heads[side],radius,topRadius,SupportMemberKind::Steel,false});}
    else{
        // Two substantial moment-frame legs and a fore/aft backstay replace
        // the repeated four-leg lattice. Intermediate nodes are real branch
        // receivers for neighbouring rail attachments, not decorative collars.
        for(int side=0;side<2;++side){
            const Vec3 base=foot(feet[side]);if(heads[side].z-base.z<4)return Support{};
            const Vec3 mid=base+(heads[side]-base)*.57;
            out.members.push_back({base,mid,radius,radius*.94,SupportMemberKind::Steel,false});
            out.members.push_back({mid,heads[side],radius*.94,topRadius,SupportMemberKind::Steel,false});
            if(side==0){
                const Vec3 back=foot(feet[side]+outward*std::clamp(height*.23,14.,48.)*spread);
                out.members.push_back({back,mid,radius*.75,radius*.67,SupportMemberKind::Steel,false});
            }
        }
    }
    const Vec3 neck=attachment-q.up*.9;
    add(hub,neck,.375);
    out.members.push_back({neck,attachment,supportRadius,supportRadius,SupportMemberKind::Steel,true});
    return out;
}
inline std::vector<Support> inversion(const Design& d,const SupportRegion& region,const ClearanceSweep& sweep,Cancel cancel){
    std::vector<Support> out;
    const auto entry=d.track.sample(region.begin);
    Vec3 along=unit(Vec3{entry.tangent.x,entry.tangent.y,0});if(norm(along)<.5)along={1,0,0};
    const Vec3 across=unit(cross(along,Vec3{0,0,1}));
    double lateralMin=INFINITY,lateralMax=-INFINITY;
    for(double s=region.begin;s<=region.end;s+=3.){const double v=dot(d.track.sample(s).position,across);lateralMin=std::min(lateralMin,v);lateralMax=std::max(lateralMax,v);}
    double maximumZ=-INFINITY,minimumZ=INFINITY;
    for(double s=region.begin;s<=region.end;s+=3.){const double z=d.track.sample(s).position.z;maximumZ=std::max(maximumZ,z);minimumZ=std::min(minimumZ,z);}
    double crownBegin=INFINITY,crownEnd=-INFINITY;
    const double crownLevel=minimumZ+(maximumZ-minimumZ)*.70;
    for(double s=region.begin;s<=region.end;s+=3.)if(d.track.sample(s).position.z>=crownLevel){crownBegin=std::min(crownBegin,s);crownEnd=std::max(crownEnd,s);}
    std::vector<Support> crown;
    if(crownEnd-crownBegin>36.)for(double depth:{1.,.75,1.35}){
        for(double spread:{1.,1.35}){for(double reach:{0.,18.,-12.}){
            cancelled(cancel);auto candidate=sweptGirder(d,crownBegin,crownEnd,false,spread,reach,depth,cancel);
            if(assemblyClear(d,candidate,sweep,cancel)){crown=std::move(candidate);break;}
        }if(!crown.empty())break;}if(!crown.empty())break;
    }
    std::vector<double> sites;
    auto intervals=[&](double from,double to,bool includeEnd){const int n=std::max(1,int(std::ceil((to-from)/48.)));
        for(int i=0;i<n;++i)sites.push_back(from+(to-from)*i/n);if(includeEnd)sites.push_back(to);};
    if(crown.empty())intervals(region.begin,region.end,true);
    else{intervals(region.begin,crownBegin,false);const int n=std::max(1,int(std::ceil((region.end-crownEnd)/48.)));
        for(int i=1;i<=n;++i)sites.push_back(crownEnd+(region.end-crownEnd)*i/n);}
    for(const double distance:sites){
        bool found=false;
        for(double standoff:{2.2,3.2,4.5}){
            for(double spread:{1.,1.5,2.}){
                for(double reach:{0.,16.,-8.,32.}){
                    cancelled(cancel);auto s=inversionBent(d,distance,spread,reach,standoff,across,lateralMin,lateralMax);
                    if(s.members.empty())continue;
                    const auto shape=validateSupportLayout({s},d.request.terrain,cancel);
                    if(!shape.valid())continue;
                    if(supportStationCollision(s,d.station,cancel))continue;
                    const int hit=supportCollision(s,sweep,cancel);
                    if(hit==-2)throw std::runtime_error("CANCELLED");if(hit>=0)continue;
                    const auto mesh=validateSupportFabrication(d.track,{s},d.request.terrain,d.station,sweep,cancel);
                    if(!mesh.valid()){if(mesh.errors.front().code=="CANCELLED")throw std::runtime_error("CANCELLED");continue;}
                    out.push_back(std::move(s));found=true;break;
                }if(found)break;
            }if(found)break;
        }
        if(!found&&std::abs(dot(across,d.track.sample(distance).up))>.30){
            // The Immelmann rollout rotates the normal through sideways.
            // Use a bank-aligned bent locally; keep the discrete frames on
            // the rest of the inversion instead of discarding the element.
            const auto q=d.track.sample(distance);Vec3 right=unit(Vec3{q.right.x,q.right.y,0});
            if(norm(right)<.5)right=across;
            Vec3 outreach=unit(Vec3{-q.up.x,-q.up.y,0});if(norm(outreach)<.5)outreach=right;
            // Move onto the outside footprint horizontally. Using the rolling
            // local-right vector lifted a tower tip tens of metres above rail.
            for(double offset:{0.,6.,12.,18.,26.,36.,48.}){
                for(double standoff:{2.,4.,6.}){for(double rear:{1.,-1.}){
                    cancelled(cancel);auto s=rakedFrame(q,right,outreach,offset,distance,d.request.terrain,standoff,1.,rear,TrackProfile::Exa,true);
                    if(s.members.empty()||!validateSupportLayout({s},d.request.terrain,cancel).valid())continue;
                    if(supportStationCollision(s,d.station,cancel)||supportCollision(s,sweep,cancel)>=0)continue;
                    if(!validateSupportFabrication(d.track,{s},d.request.terrain,d.station,sweep,cancel).valid())continue;
                    out.push_back(std::move(s));found=true;break;
                }if(found)break;}if(found)break;
            }
        }
        if(!found)return {};
    }
    out.insert(out.end(),std::make_move_iterator(crown.begin()),std::make_move_iterator(crown.end()));
    std::sort(out.begin(),out.end(),[](const Support& a,const Support& b){return a.trackDistance<b.trackDistance;});
    return out;
}
inline std::vector<Support> candidate(const Design& d,const SupportRegion& region,double spreadScale,double reach,double depthScale,Cancel cancel){
    const auto section=trackSection(d.track.profile);const double sectionDepth=section.spineDepth,sectionRadius=section.spineRadius;
    const bool arch=region.kind==SupportStructureKind::Camelback;
    const int bays=std::max(4,int(std::ceil((region.end-region.begin)/(arch?24.:12.))));
    std::vector<TrackSample> q;std::vector<Support> out;std::vector<std::array<Vec3,2>> upper;
    size_t peak=0;double height=0;
    for(int i=0;i<=bays;++i){
        cancelled(cancel);const double s=region.begin+(region.end-region.begin)*i/bays;auto sample=d.track.sample(s);q.push_back(sample);
        if(sample.position.z>q[peak].position.z)peak=size_t(i);
        height=std::max(height,sample.position.z-d.request.terrain.height(sample.position.x,sample.position.y));
        const auto a=sample.position-sample.up*(sectionDepth+sectionRadius),cap=a-sample.up*(arch?2.:1.8*depthScale);
        out.push_back({{cap.x,cap.y,d.request.terrain.height(cap.x,cap.y)},cap,a,true,s,{}});
    }
    // Geometric sizing for this game's structure model, not a stress solver.
    const bool heavy=d.track.profile==TrackProfile::Exa;
    const double radius=std::clamp(.32+height*(heavy?.0042:.0034),.38,heavy?1.65:1.55),brace=std::clamp(radius*(heavy?.48:.40),heavy?.20:.18,heavy?.68:.56);
    const double halfWidth=std::clamp(2.+height*.008,2.5,4.)*spreadScale;
    auto add=[&](size_t owner,Vec3 a,Vec3 b,double r,SupportMemberKind kind=SupportMemberKind::Steel){
        if(norm(b-a)>.01)out[owner].members.push_back({a,b,r,r,kind,false});
    };
    auto foundation=[&](Vec3 p){
        auto f=footing(p,d.request.terrain,1.+radius*1.8,radius,false);
        // A shallow raker's circular end has almost a full radius of vertical
        // extent; a vertical-column pedestal height is insufficient here.
        f.top.z=std::max(f.top.z,d.request.terrain.heightRange(p.x,p.y,radius*1.6)[1]+radius+.25);
        return f;
    };
    auto anchor=[&](size_t owner,Vec3 p,Vec3 target){
        const auto f=foundation(p);
        out[owner].members.push_back({f.base,f.top,f.radius,std::max(.6,radius*1.35),SupportMemberKind::Footing,false});
        add(owner,f.top,target,radius);return f.top;
    };
    if(arch){
        if(peak<3||q.size()-1-peak<3)return {};
        Vec3 across=unit(Vec3{q[peak].right.x,q[peak].right.y,0});
        if(norm(across)<.5)return {};
        const Vec3 along=unit(Vec3{q.back().position.x-q.front().position.x,q.back().position.y-q.front().position.y,0});
        for(size_t i=0;i<q.size();++i){
            std::array<Vec3,2> u;
            for(int side=0;side<2;++side)
                u[side]=out[i].attachment-q[i].up*(3.*depthScale)+across*((side?1.:-1.)*halfWidth);
            upper.push_back(u);
        }
        // Two braced trestles carry a deep crown truss. Main legs stop at the
        // high shoulders instead of forming one unrestrained giant A at the
        // crown. Their two side frames share transverse ties and open diagonals.
        const size_t leftShoulder=peak-std::max(size_t(2),peak/4);
        const size_t rightShoulder=peak+std::max(size_t(2),(q.size()-1-peak)/4);
        if(leftShoulder<1||rightShoulder+1>=q.size())return {};
        std::vector<std::array<Vec3,2>> crown=upper;
        const double crownDepth=std::clamp(height*.075,6.,14.)*depthScale;
        for(size_t i=leftShoulder;i<=rightShoulder;++i){
            for(int side=0;side<2;++side){
                crown[i][side]=upper[i][side]-Vec3{0,0,crownDepth};
                if(i==leftShoulder||i==rightShoulder)add(i,crown[i][side],upper[i][side],brace);
                if(i>leftShoulder){
                    add(i,crown[i-1][side],crown[i][side],radius);
                    add(i,i%2?crown[i-1][side]:upper[i-1][side],i%2?upper[i][side]:crown[i][side],brace);
                }
            }
            add(i,crown[i][0],crown[i][1],brace);
            if(i>leftShoulder&&i%2)add(i,crown[i-1][i%2],crown[i][1-i%2],brace);
        }
        for(int end=0;end<2;++end){
            const size_t first=end?q.size()-1:0;
            const size_t shoulder=end?rightShoulder:leftShoulder;
            const size_t intervals=end?first-shoulder:shoulder;
            const int panels=std::max(3,int(std::ceil(height/(heavy?48.:32.))));
            std::vector<std::array<Vec3,2>> inner;
            for(int side=0;side<2;++side){
                const double sign=side?1.:-1.;
                Vec3 p=q[peak].position+(q[first].position-q[peak].position)*.56;
                p=p+across*(sign*std::clamp(height*.075,7.,22.)*spreadScale)+along*((end?1.:-1.)*reach);
                const auto f=foundation(p);
                out[first].members.push_back({f.base,f.top,f.radius,std::max(.6,radius*1.35),SupportMemberKind::Footing,false});
                if(inner.empty())inner.resize(size_t(panels)+1);
                const Vec3 start=heavy?f.top+unit(crown[shoulder][side]-f.top)*(radius*3.):f.top;
                if(heavy)add(first,f.top,start,radius);
                for(int k=0;k<=panels;++k)
                    inner[k][side]=start+(crown[shoulder][side]-start)*(double(k)/panels);
                // The outside chord also resolves to ground, leaving the
                // centre of the element open and keeping the end bay honest.
                anchor(first,upper[first][side]+along*((end?1.:-1.)*8.),upper[first][side]);
            }
            for(int k=0;k<panels;++k){
                const size_t offset=size_t(std::lround(double(intervals)*k/panels));
                const size_t i=end?first-offset:offset;
                for(int side=0;side<2;++side){
                    add(i,inner[k][side],inner[k+1][side],radius);
                    const size_t nextOffset=size_t(std::lround(double(intervals)*(k+1)/panels));
                    const size_t nextIndex=end?first-nextOffset:nextOffset;
                    // One alternating diagonal per clear bay. The old pair
                    // of spokes from every outer point formed overlapping
                    // fans and obscured the accepted primary-leg silhouette.
                    if(k%2)add(i,inner[k][side],upper[nextIndex][side],brace*1.20);
                    else add(i,upper[i][side],inner[k+1][side],brace*1.20);
                }
                // Tie the paired legs at panel nodes and brace alternate bays,
                // keeping a visible hierarchy between main legs and lattice.
                if(k>0)add(i,inner[k][0],inner[k][1],brace);
                if(k%2==0)add(i,inner[k][k%2],inner[k+1][1-k%2],brace*1.15);
            }
        }
    }else{
        // A continuous outside header shares loads between the paired rakers.
        // Follow the actual curve, including between raker groups, so the
        // header has no disconnected ends above the loop or Immelmann.
        Vec3 along=unit(Vec3{q.front().tangent.x,q.front().tangent.y,0});if(norm(along)<.5)along={1,0,0};
        const Vec3 across=unit(cross(along,Vec3{0,0,1}));
        double lateralMin=INFINITY,lateralMax=-INFINITY;
        for(const auto& sample:q){lateralMin=std::min(lateralMin,dot(sample.position,across));lateralMax=std::max(lateralMax,dot(sample.position,across));}
        const size_t groups=std::clamp(size_t(std::ceil((region.end-region.begin)/48.)),size_t(2),size_t(6));
        for(size_t group=0;group<groups;++group){
            cancelled(cancel);
            const size_t first=q.size()*group/groups,last=q.size()*(group+1)/groups-1,mid=(first+last)/2;
            std::vector<Vec3> header;
            for(size_t i=first;i<=last;++i)
                header.push_back(out[i].top);
            const Vec3 hub=header[mid-first];
            for(int side=0;side<2;++side){
                const double sign=side?1.:-1.;
                const double outside=(side?lateralMax:lateralMin)+sign*7.*spreadScale;
                const Vec3 shoulder=hub+across*(outside-dot(hub,across));
                const Vec3 p=shoulder+across*(sign*std::clamp(height*.19,7.,28.)*spreadScale)+along*(sign*(height*.10+reach));
                if(!heavy){anchor(mid,p,shoulder);add(mid,shoulder,hub,radius*.82);continue;}
                // Keep the rakers outside the swept train. A short segmented
                // knee turns into the header without the old pointed mitre.
                const auto f=foundation(p);
                out[mid].members.push_back({f.base,f.top,f.radius,std::max(.6,radius*1.35),SupportMemberKind::Footing,false});
                const double bend=std::min({radius*3.,norm(shoulder-f.top)*.18,norm(hub-shoulder)*.3});
                const Vec3 a=shoulder-unit(shoulder-f.top)*bend,b=shoulder+unit(hub-shoulder)*bend;
                add(mid,f.top,a,radius);
                Vec3 previous=a;
                for(int k=1;k<=5;++k){const double t=k/5.;const Vec3 next=a*((1-t)*(1-t))+shoulder*(2*t*(1-t))+b*(t*t);add(mid,previous,next,radius);previous=next;}
                add(mid,b,hub,radius);
            }
            for(size_t i=first;i<=last;++i){
                if(i>0&&(heavy||i>first)){
                    // Follow the actual outside frame between attachments.
                    // Coarse chords produced angular clips above the loop.
                    Vec3 previous=out[i-1].top;
                    const int pieces=std::max(1,int(std::ceil((out[i].trackDistance-out[i-1].trackDistance)/2.)));
                    for(int k=1;k<=pieces;++k){
                        auto sample=d.track.sample(out[i-1].trackDistance+(out[i].trackDistance-out[i-1].trackDistance)*k/pieces);
                        const Vec3 next=k==pieces?out[i].top:sample.position-sample.up*(sectionDepth+sectionRadius+1.8*depthScale);
                        add(i,previous,next,radius*(heavy?1.:.82));previous=next;
                    }
                }
            }
        }
    }
    for(size_t i=0;i<q.size();++i){
        cancelled(cancel);
        if(arch){
          add(i,upper[i][0],upper[i][1],brace);
          for(int side=0;side<2;++side){
            add(i,upper[i][side],out[i].top,std::min(.48,radius*.65));
            if(i){
                add(i,upper[i-1][side],upper[i][side],radius);
            }
          }
          if(i&&i%2)add(i,upper[i-1][i%2],upper[i][1-i%2],brace);
        }
        const Vec3 neck=out[i].attachment-q[i].up*.9;
        out[i].members.push_back({out[i].top,neck,heavy?.50:std::min(.35,radius*.5),heavy?.50:.22,SupportMemberKind::Steel,false});
        out[i].members.push_back({neck,out[i].attachment,supportRadius,supportRadius,SupportMemberKind::Steel,true});
    }
    // The compatibility datum refers to a real shared foundation. Rendering,
    // collision and persistence all use the same explicit member graph.
    std::vector<Vec3> anchors;
    for(const auto& s:out)for(const auto& m:s.members)if(m.kind==SupportMemberKind::Footing)anchors.push_back({m.base.x,m.base.y,d.request.terrain.height(m.base.x,m.base.y)});
    for(auto& s:out){
        auto nearest=std::min_element(anchors.begin(),anchors.end(),[&](Vec3 a,Vec3 b){return norm(a-s.top)<norm(b-s.top);});
        if(nearest==anchors.end()||s.top.z<=nearest->z)return {};s.base=*nearest;
    }
    return out;
}
#include "support_twisted.hpp"
inline std::vector<Support> build(const Design& d,const SupportRegion& region,const ClearanceSweep& sweep,Cancel cancel){
    if(d.track.profile==TrackProfile::Exa&&(region.kind==SupportStructureKind::Loop||region.kind==SupportStructureKind::Immelmann))
        return inversion(d,region,sweep,cancel);
    // Re-site the assembly's footprint and deepen its outside chord. Failure
    // never edits the track or cuts a clearance exemption around an obstacle.
    for(double depth:{1.,.7,1.35,1.8,2.4})for(double spread:{1.,1.5})for(double reach:{0.,18.,-12.}){
        cancelled(cancel);auto parts=region.kind==SupportStructureKind::TwistedDrop?twisted(d,region,spread,reach,depth,cancel):candidate(d,region,spread,reach,depth,cancel);if(parts.empty())continue;
        auto report=validateSupportLayout(parts,d.request.terrain,cancel);
        if(!report.valid()){if(report.errors.front().code=="CANCELLED")throw std::runtime_error("CANCELLED");continue;}
        bool valid=true;
        for(const auto& s:parts){
            if(supportStationCollision(s,d.station,cancel)){valid=false;break;}
            const int hit=supportCollision(s,sweep,cancel);if(hit==-2)throw std::runtime_error("CANCELLED");if(hit>=0){valid=false;break;}
        }
        if(valid){
            const auto fittings=validateSupportFabrication(d.track,parts,d.request.terrain,d.station,sweep,cancel);
            if(fittings.valid())return parts;
            if(fittings.errors.front().code=="CANCELLED")throw std::runtime_error("CANCELLED");
        }
    }
    return {};
}
}
