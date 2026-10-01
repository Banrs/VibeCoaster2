// Included by supports.cpp. Escarpment fitting and wall brackets use the actual
// triangulated terrain, including its seed-dependent heading and curved front.
namespace support_cliff {
inline void poll(const Cancel& cancel){if(cancel&&cancel())throw std::runtime_error("CANCELLED");}
inline Vec3 outward(const Terrain& t,Vec3 p){
    const Vec3 n{std::cos(t.cliffHeading),std::sin(t.cliffHeading),0},side{-n.y,n.x,0};
    return unit(n-side*(2*t.cliffCurvature*dot(p-Vec3{t.cliffX,t.cliffY,0},side)));
}
inline std::optional<Vec3> surface(const Terrain& t,Vec3 p,Vec3 normal,double maximum=80){
    if(p.z<=t.height(p.x,p.y))return {};
    double low=0,high=0;
    for(double s=2;s<=maximum;s+=2){const Vec3 q=p-normal*s;
        if(t.height(q.x,q.y)>=q.z){high=s;break;}low=s;}
    if(high==0)return {};
    for(int i=0;i<36;++i){const double s=(low+high)*.5;const Vec3 q=p-normal*s;
        if(t.height(q.x,q.y)>=q.z)high=s;else low=s;}
    return p-normal*((low+high)*.5);
}
inline void fitTerrain(Design& d,Cancel cancel){
    if(d.track.profile!=TrackProfile::Exa||d.request.terrain.kind!=TerrainKind::Highlands||d.request.terrain.plateau<=0)return;
    poll(cancel);const Terrain original=d.request.terrain;
    const auto section=std::find_if(d.sections.begin(),d.sections.end(),[](const auto& s){return s.role==RideRole::CliffDrop;});
    // A saved typed section supplies the drop bounds; a geometric fallback also
    // lets focused terrain fixtures exercise this without a named recipe.
    const double begin=section==d.sections.end()?0:section->start,end=section==d.sections.end()?d.track.length:section->end;
    std::vector<TrackSample> drop,checks;
    const Vec3 normal{std::cos(original.cliffHeading),std::sin(original.cliffHeading),0};
    for(double s=0;s<d.track.length;s+=3.){poll(cancel);const auto q=d.track.sample(s);
        const double front=original.cliffCoordinate(q.position.x,q.position.y);
        if(front>-100&&front<180)checks.push_back(q);
        if(s>=begin&&s<=end&&q.tangent.z<-.70&&q.position.z>40&&front>-20&&front<140)drop.push_back(q);
    }
    if(drop.size()<4)return;
    const Vec3 origin=section==d.sections.end()?Vec3{original.cliffX,original.cliffY,0}:d.track.sample(begin).position+normal*7.;
    const auto sweep=buildClearanceSweep(d.track,d.request.train,cancel);
    const double required=std::max(0.,d.request.limits.minClearance);
    struct Choice {double score;Terrain terrain;};std::vector<Choice> choices;
    // Reduce the old tens-of-metres scenic gap to an approximately eight-metre
    // centreline-to-face bracket reach. Do not reduce the occupied train envelope.
    for(double width:{24.,32.,40.,48.,56.,64.,80.})for(double shift=0;shift<=48;shift+=4.){
        poll(cancel);Terrain t=original;t.cliffX=origin.x+normal.x*shift;t.cliffY=origin.y+normal.y*shift;t.cliffWidth=width;
        double score=0;bool valid=true;
        for(const auto& q:drop){const auto wall=surface(t,q.position,outward(t,q.position),100);
            if(!wall){valid=false;break;}const double gap=norm(*wall-q.position);
            if(gap<5.5){valid=false;break;}score+=(gap-8.)*(gap-8.);}
        if(!valid)continue;
        for(const auto& q:checks)if(terrain_validation::lowerBound(q,t,sweep.trainTop(),.40,sweep.trainBottom())<required+.10){valid=false;break;}
        if(valid)choices.push_back({score/drop.size(),std::move(t)});
    }
    std::sort(choices.begin(),choices.end(),[](const Choice& a,const Choice& b){return a.score<b.score;});
    // The winning coarse fit must pass the complete continuous swept test.
    // Only then replace the terrain, so cancellation keeps the previous surface.
    for(size_t i=0;i<std::min(size_t(8),choices.size());++i){poll(cancel);
        if(minimumSweptGroundClearance(sweep,choices[i].terrain,cancel)>=required){
            d.request.terrain=std::move(choices[i].terrain);return;
        }
    }
}
inline std::optional<SupportMember> socket(const Terrain& t,Vec3 query,double legRadius){
    const auto point=surface(t,query,outward(t,query));if(!point)return {};
    const Vec3 p=*point;const double e=.05;
    const Vec3 n=unit(Vec3{-(t.height(p.x+e,p.y)-t.height(p.x-e,p.y))/(2*e),
                           -(t.height(p.x,p.y+e)-t.height(p.x,p.y-e))/(2*e),1});
    if(std::hypot(n.x,n.y)<.80)return {};
    const Vec3 right=unit(cross(n,Vec3{0,0,1})),up=cross(n,right);
    const double rb=legRadius*2.6+.4,rt=legRadius*1.45+.25;
    for(double exposure:{.55,.9,1.4,2.})for(double embed:{3.,5.,8.}){
        const Vec3 base=p-n*embed,top=p+n*exposure;
        const auto buried=t.heightRange(base.x,base.y,rb);
        const StationBox cap{top,n,right,up,{.01,rt,rt},StationRole::Footing};
        if(base.z+rb>buried[0]-.35||terrain_validation::boxLowerBound(cap,t,0)<.04)continue;
        return SupportMember{base,top,rb,rt,SupportMemberKind::RockAnchor,false};
    }
    return {};
}
inline Support bracket(const Design& d,const TrackSample& q,double distance,double standoff){
    const auto& t=d.request.terrain;Support s;
    if(t.kind!=TerrainKind::Highlands||t.plateau<=0||q.tangent.z>-.25)return s;
    const auto section=trackSection(d.track.profile);
    s.attachment=q.position-q.up*(section.spineDepth+section.spineRadius);s.hasAttachment=true;s.trackDistance=distance;
    s.top=s.attachment-q.up*standoff;
    const Vec3 n=outward(t,s.top);
    const auto face=surface(t,s.top,n,64);if(!face)return Support{};
    const double reach=norm(s.top-*face);if(reach<2||reach>55)return Support{};
    const double radius=std::clamp(.70+reach*.018,.82,1.45);
    const auto anchor=socket(t,s.top,radius);if(!anchor)return Support{};
    s.base=anchor->base;s.members.push_back(*anchor);
    s.members.push_back({anchor->top,s.top,radius,radius*.90,SupportMemberKind::Steel,false});
    // Short reaches use one substantial cantilever. A long outreach adds a
    // single lower haunch in the wall-normal plane, not a three-legged fan.
    // Leave an axial socket stub before the inclined brace, so its unfitted
    // circular end also clears the wall; do not exempt buried steel.
    if(reach>12.){
        const double braceRadius=radius*.68,depth=std::clamp(reach*.45,5.,14.);
        const auto lower=socket(t,s.top-Vec3{0,0,depth},braceRadius);if(!lower)return Support{};
        const Vec3 axis=unit(lower->top-lower->base),toe=lower->top+axis*(braceRadius*1.6+.35);
        s.members.push_back(*lower);
        s.members.push_back({lower->top,toe,braceRadius,braceRadius,SupportMemberKind::Steel,false});
        s.members.push_back({toe,s.top,braceRadius,braceRadius*.90,SupportMemberKind::Steel,false});
    }
    const Vec3 neck=s.attachment-q.up*.9;
    s.members.push_back({s.top,neck,.50,.50,SupportMemberKind::Steel,false});
    s.members.push_back({neck,s.attachment,supportRadius,supportRadius,SupportMemberKind::Steel,true});
    return s;
}
}
