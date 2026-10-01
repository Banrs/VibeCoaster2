// Included inside support_assemblies. Endpoint-authored beams, local rail
// connectors and foundations are separate, as in the reviewed NoLimits/Rollygon
// workflows. Original native implementation; no third-party add-on code bundled.
struct BeamSections {
    double primary,chord,web,neck;
    static BeamSections forSpan(double height,double span){
        return {std::clamp(.48+height*.0055,.70,1.85),std::clamp(.30+span*.006,.48,1.05),
                std::clamp(.16+span*.0024,.24,.46),.50};
    }
};
// A shallow spatial girder carries local mounts to a few real trestles. Three
// continuous chords and alternating side/underside diagonals close every bay.
inline std::vector<Support> sweptGirder(const Design& d,double begin,double end,
        bool upright,double spread,double reach,double depthScale,Cancel cancel){
    const auto section=trackSection(d.track.profile);const auto& terrain=d.request.terrain;
    const int bays=std::max(4,int(std::ceil((end-begin)/24.)));
    std::vector<TrackSample> q;std::vector<Support> out;
    std::vector<std::array<Vec3,2>> bottom;double height=0;
    for(int i=0;i<=bays;++i){
        cancelled(cancel);q.push_back(d.track.sample(begin+(end-begin)*i/bays));
        height=std::max(height,q.back().position.z-terrain.height(q.back().position.x,q.back().position.y));
    }
    const size_t firstPier=upright?size_t(std::lround(bays*.31)):0;
    const size_t lastPier=upright?size_t(std::lround(bays*.69)):size_t(bays);
    const double pierSpan=norm(q[lastPier].position-q[firstPier].position);
    auto sections=BeamSections::forSpan(height,pierSpan);
    if(!upright)sections.chord*=.85;
    const double depth=(upright?std::clamp(pierSpan*.075,5.,13.):std::clamp(pierSpan*.045,3.5,7.))*depthScale;
    const double width=std::clamp(depth*.42,2.6,5.2)*spread;
    Vec3 across=unit(Vec3{q[bays/2].right.x,q[bays/2].right.y,0});
    if(norm(across)<.5)across=unit(Vec3{q.front().right.x,q.front().right.y,0});
    if(norm(across)<.5)return {};
    for(int i=0;i<=bays;++i){
        const auto& sample=q[i];const Vec3 attachment=sample.position-sample.up*(section.spineDepth+section.spineRadius);
        const Vec3 head=attachment-sample.up*3.2;
        // A rolling hill's girder stays below the ride; only its short saddle
        // follows bank. An inversion's crown girder sits outside the arc.
        const Vec3 deep=head-(upright?Vec3{0,0,1}:sample.up)*depth;
        const Vec3 side=upright?across:sample.right;
        bottom.push_back({deep-side*width,deep+side*width});
        out.push_back({{head.x,head.y,terrain.height(head.x,head.y)},head,attachment,true,begin+(end-begin)*i/bays,{}});
    }
    auto add=[&](size_t owner,Vec3 a,Vec3 b,double ra,double rb){
        if(norm(b-a)>.01)out[owner].members.push_back({a,b,ra,rb,SupportMemberKind::Steel,false});};
    auto beam=[&](size_t owner,Vec3 a,Vec3 b,double r){add(owner,a,b,r,r);};
    auto chord=[&](size_t owner,int which){
        const double from=out[owner-1].trackDistance,to=out[owner].trackDistance;
        const int pieces=std::max(1,int(std::ceil((to-from)/6.)));
        Vec3 previous=which<0?out[owner-1].top:bottom[owner-1][which];
        for(int k=1;k<=pieces;++k){
            Vec3 next;
            if(k==pieces)next=which<0?out[owner].top:bottom[owner][which];
            else{
                const auto sample=d.track.sample(from+(to-from)*k/pieces);
                next=sample.position-sample.up*(section.spineDepth+section.spineRadius+3.2);
                if(which>=0)next=next-(upright?Vec3{0,0,1}:sample.up)*depth+(upright?across:sample.right)*((which?1.:-1.)*width);
            }
            beam(owner,previous,next,sections.chord);previous=next;
        }
    };
    auto foot=[&](size_t owner,Vec3 p,double r){
        auto f=footing(p,terrain,1.+r*1.9,r,false);
        f.top.z=std::max(f.top.z,terrain.heightRange(p.x,p.y,r*1.6)[1]+r+.25);
        out[owner].members.push_back({f.base,f.top,f.radius,std::max(.6,r*1.35),SupportMemberKind::Footing,false});return f.top;};
    for(size_t i=0;i<out.size();++i){
        cancelled(cancel);const Vec3 head=out[i].top;
        beam(i,bottom[i][0],bottom[i][1],sections.web);
        for(int side=0;side<2;++side)beam(i,bottom[i][side],head,sections.web);
        if(i){
            chord(i,-1);
            for(int side=0;side<2;++side){
                chord(i,side);
                beam(i,i%2?out[i-1].top:bottom[i-1][side],i%2?bottom[i][side]:head,sections.web);
            }
            beam(i,bottom[i-1][i%2],bottom[i][1-i%2],sections.web);
        }
        const Vec3 neck=out[i].attachment-q[i].up*.9;
        beam(i,head,neck,sections.neck);
        out[i].members.push_back({neck,out[i].attachment,supportRadius,supportRadius,SupportMemberKind::Steel,true});
    }
    for(size_t pier:{firstPier,lastPier}){
        const Vec3 centre=(bottom[pier][0]+bottom[pier][1])*.5;
        const double localHeight=centre.z-terrain.height(centre.x,centre.y);if(localHeight<8)return {};
        Vec3 outward=upright?unit(Vec3{q[pier].position.x-q[bays/2].position.x,q[pier].position.y-q[bays/2].position.y,0}):unit(Vec3{-q[pier].up.x,-q[pier].up.y,0});
        if(norm(outward)<.5)outward=unit(cross(across,Vec3{0,0,1}))*(pier==firstPier?-1.:1.);
        const Vec3 side=unit(cross(outward,Vec3{0,0,1}));
        const double rake=std::clamp(localHeight*(upright?.32:.55),8.,90.)+reach;
        const double halfFoot=std::clamp(localHeight*.10,5.,20.)*spread;
        std::array<Vec3,2> roots,mids;
        const Vec3 back=foot(pier,centre+outward*(rake+std::clamp(localHeight*.22,12.,40.)),sections.primary*.82);
        for(int k=0;k<2;++k){
            const double sign=dot(bottom[pier][k]-centre,side)<0?-1.:1.;
            roots[k]=foot(pier,centre+outward*rake+side*(sign*halfFoot),sections.primary);
            if(bottom[pier][k].z-roots[k].z<5)return {};
            mids[k]=roots[k]+(bottom[pier][k]-roots[k])*.52;
            add(pier,roots[k],mids[k],sections.primary,sections.primary*.94);
            add(pier,mids[k],bottom[pier][k],sections.primary*.94,sections.chord);
            add(pier,back,mids[k],sections.primary*.72,sections.primary*.65);
        }
        beam(pier,mids[0],mids[1],sections.web*1.30);
        beam(pier,mids[pier==firstPier?0:1],bottom[pier][pier==firstPier?1:0],sections.web*1.30);
    }
    if(upright)for(size_t i:{size_t(0),size_t(bays)})for(int side=0;side<2;++side){
        const Vec3 root=foot(i,bottom[i][side],sections.chord);
        if(bottom[i][side].z-root.z<1)return {};
        beam(i,root,bottom[i][side],sections.chord);
    }
    std::vector<Vec3> anchors;
    for(const auto& s:out)for(const auto& m:s.members)if(m.kind==SupportMemberKind::Footing)anchors.push_back(m.base);
    for(auto& s:out){
        const auto nearest=std::min_element(anchors.begin(),anchors.end(),[&](Vec3 a,Vec3 b){return norm(a-s.top)<norm(b-s.top);});
        if(nearest==anchors.end())return {};
        s.base={nearest->x,nearest->y,terrain.height(nearest->x,nearest->y)};
    }
    return out;
}
inline bool assemblyClear(const Design& d,const std::vector<Support>& parts,const ClearanceSweep& sweep,Cancel cancel){
    if(parts.empty()||!validateSupportLayout(parts,d.request.terrain,cancel).valid())return false;
    for(const auto& s:parts){
        cancelled(cancel);if(supportStationCollision(s,d.station,cancel))return false;
        const int hit=supportCollision(s,sweep,cancel);if(hit==-2)throw std::runtime_error("CANCELLED");if(hit>=0)return false;
    }
    const auto report=validateSupportFabrication(d.track,parts,d.request.terrain,d.station,sweep,cancel);
    if(!report.valid()&&report.errors.front().code=="CANCELLED")throw std::runtime_error("CANCELLED");
    return report.valid();
}
