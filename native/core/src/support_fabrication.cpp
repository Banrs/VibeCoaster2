#include "coaster/support_fabrication.hpp"
#include "coaster/track_hardware.hpp"
#include <map>
#include <tuple>
#include <numeric>

namespace coaster {
namespace {
using Ring=std::vector<Vec3>;
using Key=std::pair<size_t,int>;
using Node=std::tuple<int64_t,int64_t,int64_t>;
constexpr int sides=32;
constexpr int fittedSides=64;
void poll(const Cancel& c){if(c&&c())throw std::runtime_error("CANCELLED");}
Node node(Vec3 p){return {int64_t(std::llround(p.x*1e5)),int64_t(std::llround(p.y*1e5)),int64_t(std::llround(p.z*1e5))};}
struct Basis {Vec3 axis,right,up;};
Basis basis(Vec3 a,Vec3 b){
    if(norm(b-a)<1e-9)throw std::runtime_error("Degenerate fabrication axis");
    const Vec3 axis=unit(b-a),right=unit(cross(axis,std::abs(axis.z)<.9?Vec3{0,0,1}:Vec3{0,1,0}));
    return {axis,right,cross(axis,right)};
}
FabricationPart prism(Ring a,Ring b,std::string name,bool smooth=false){
    if(a.size()!=b.size()||a.size()<3)throw std::runtime_error("Invalid fabrication rings");
    FabricationPart out;out.name=std::move(name);out.smooth=smooth;out.vertices=a;
    out.vertices.insert(out.vertices.end(),b.begin(),b.end());const uint32_t n=uint32_t(a.size());
    for(uint32_t i=0;i<n;++i)out.faces.push_back({i,(i+1)%n,(i+1)%n+n,i+n});
    std::vector<uint32_t> cap(n);std::iota(cap.begin(),cap.end(),0);std::reverse(cap.begin(),cap.end());out.faces.push_back(cap);
    std::iota(cap.begin(),cap.end(),n);out.faces.push_back(cap);
    double volume=0;const Vec3 origin=out.vertices.front();
    for(const auto& face:out.faces)for(size_t i=1;i+1<face.size();++i)
        volume+=dot(out.vertices[face[0]]-origin,cross(out.vertices[face[i]]-origin,out.vertices[face[i+1]]-origin));
    if(volume<0)for(auto& face:out.faces)std::reverse(face.begin(),face.end());
    return out;
}
Ring circle(Vec3 centre,Vec3 right,Vec3 up,double radius,int count=sides){
    Ring out;for(int j=0;j<count;++j){const double angle=2*pi*j/count;out.push_back(centre+(right*std::cos(angle)+up*std::sin(angle))*radius);}return out;
}
FabricationPart tube(Vec3 a,Vec3 b,double r0,double r1,std::string name,int count=sides){
    const auto f=basis(a,b);auto out=prism(circle(a,f.right,f.up,r0,count),circle(b,f.right,f.up,r1,count),std::move(name),count>6);out.axis=f.axis;return out;
}
FabricationPart ring(Vec3 a,Vec3 b,double inner,double outer,std::string name){
    const auto f=basis(a,b);FabricationPart out;out.name=std::move(name);out.smooth=true;out.axis=f.axis;
    for(Vec3 p:{a,b})for(double radius:{inner,outer}){auto r=circle(p,f.right,f.up,radius);out.vertices.insert(out.vertices.end(),r.begin(),r.end());}
    for(uint32_t j=0;j<sides;++j){const uint32_t k=(j+1)%sides,n=sides;
        out.faces.push_back({j,k,k+n,j+n});out.faces.push_back({j+2*n,j+3*n,k+3*n,k+2*n});
        out.faces.push_back({j,j+2*n,k+2*n,k});out.faces.push_back({j+n,k+n,k+3*n,j+3*n});}
    return out;
}
void bolt(std::vector<FabricationPart>& out,Vec3 a,Vec3 axis,double length,double radius,size_t owner){
    // Compact game LOD: keep the visible washer and hex at both ends; the shank
    // is enclosed by the flange. Blender's fabrication study retains the shank.
    for(int sign:{-1,1}){
        const Vec3 base=sign<0?a:a+axis*length;
        auto washer=tube(base,base+axis*(sign*.01),radius*1.5,radius*1.5,"Washer",12);
        auto nut=tube(base+axis*(sign*.01),base+axis*(sign*.04),radius,radius,"Hex fastener",6);
        for(auto* p:{&washer,&nut}){p->material=FabricationMaterial::Fastener;p->owner=owner;out.push_back(std::move(*p));}
    }
}
struct Head {
    size_t parent{},contact{};TrackSample frame;
    Vec3 forward,origin,end;double radius{},mount{},cap{},plate{},flange{},upper{},lower{};
    bool inlineStem{};
    Vec3 world(Vec3 p)const{return origin+forward*p.x+frame.right*p.y+frame.up*p.z;}
};
Head head(const Track& track,const Support& s){
    Head h;size_t contacts=0,parents=0;
    for(size_t i=0;i<s.members.size();++i)if(s.members[i].spineContact){h.contact=i;++contacts;}
    if(contacts!=1)throw std::runtime_error("Fabricated support requires one spine contact");
    const auto& contact=s.members[h.contact];
    for(size_t i=0;i<s.members.size();++i)if(s.members[i].kind==SupportMemberKind::Steel&&!s.members[i].spineContact&&norm(s.members[i].top-contact.base)<1e-6){h.parent=i;++parents;}
    if(parents!=1)throw std::runtime_error("Fabricated support requires one incoming member");
    const auto& parent=s.members[h.parent];h.radius=std::max({.375,parent.radiusBase,parent.radiusTop});
    h.frame=track.sample(s.trackDistance);h.forward=unit(cross(h.frame.right,h.frame.up));h.origin=h.frame.position;
    const double available=norm(contact.top-contact.base),alignment=std::clamp(dot(unit(parent.top-parent.base),h.frame.up),-1.,1.);
    h.inlineStem=alignment>1-1e-8;double drop=std::min(.6,available*.6);
    if(!h.inlineStem)drop=std::min(drop,available-h.radius*std::tan(std::acos(alignment)*.5)-.065);
    if(drop<.16)throw std::runtime_error("Insufficient contact length for the full-diameter fabricated head");
    const double capDrop=std::min(.26,drop*.5),bottom=trackSection(track.profile).bottom();
    h.mount=bottom-drop;h.cap=bottom-capDrop;h.plate=std::min(.06,capDrop*.4);
    h.flange=std::max(.525,h.radius+.15);h.upper=std::min(.065,available*.15);h.lower=std::min(.045,h.upper);
    h.end=h.world({0,0,h.mount+.002});return h;
}
void addHead(std::vector<FabricationPart>& out,const Track& track,const Support& s,const Head& h,size_t owner){
    const auto p=trackSection(track.profile);const double bottom=p.bottom(),r=p.spineRadius;
    auto emit=[&](FabricationPart part,bool seat=false){
        for(auto& v:part.vertices){
            if(seat&&v.z>bottom){
                double station=s.trackDistance-v.x;
                if(!track.closed)station=std::clamp(station,0.,track.length-1e-6);
                const auto q=track.sample(station);
                v=q.position+q.right*v.y+q.up*(-p.spineDepth-std::sqrt(r*r-v.y*v.y)+.008);
            }else v=h.world(v);
        }
        part.axis=h.forward*part.axis.x+h.frame.right*part.axis.y+h.frame.up*part.axis.z;
        part.owner=owner;part.spineSeat=seat;out.push_back(std::move(part));
    };
    emit(tube({0,0,h.mount+h.upper-.002},{0,0,h.cap},h.radius,h.radius,"Constant diameter head"));
    emit(ring({0,0,h.mount},{0,0,h.mount+h.upper},h.radius-.003,h.flange,"Upper flange"));
    emit(ring({0,0,h.mount-h.lower},{0,0,h.mount-.001},h.radius-.003,h.flange,"Lower flange"));
    // Compact full-width saddle. Long rectangular overhangs and four square
    // ended webs made the previous mount read as a platform around the spine.
    const double hx=std::max(.50,h.radius+.045),hy=h.radius+.035,chamfer=.105;
    Ring plateRing,bottomRing;
    for(auto xy:std::vector<std::pair<double,double>>{{-hx+chamfer,-hy},{hx-chamfer,-hy},{hx,-hy+chamfer},{hx,hy-chamfer},{hx-chamfer,hy},{-hx+chamfer,hy},{-hx,hy-chamfer},{-hx,-hy+chamfer}}){plateRing.push_back({xy.first,xy.second,h.cap-.002});bottomRing.push_back({xy.first,xy.second,h.cap+h.plate-.002});}
    emit(prism(plateRing,bottomRing,"Load plate"));
    double xmin=-hx+.025,xmax=hx-.025;
    if(!track.closed){xmin=std::max(xmin,s.trackDistance-track.length+1e-6);xmax=std::min(xmax,s.trackDistance);}
    for(double t:{.23,.77}){
        const double x=xmin+(xmax-xmin)*t,w=r*.86,low=h.cap+h.plate*.6;
        Ring a{{x-.025,-w,low},{x-.025,w,low}},b;
        for(int k=0;k<=24;++k){const double y=w*(1-2.*k/24);a.push_back({x-.025,y,-p.spineDepth-std::sqrt(r*r-y*y)+.008});}
        for(auto v:a){v.x+=.05;b.push_back(v);}emit(prism(a,b,"Bearing web"),true);
    }
    for(int sign:{-1,1}){
        Ring a,b;const double low=h.cap+h.plate*.65;
        for(double y:{sign*r*.65-.0275,sign*r*.65+.0275}){
            Ring line{{xmin,y,bottom-.02},{xmin+.16,y,low},{xmax-.16,y,low},{xmax,y,bottom-.02}};
            for(int k=0;k<=8;++k)line.push_back({xmax-(xmax-xmin)*k/8.,y,-p.spineDepth-std::sqrt(r*r-y*y)+.008});
            if(a.empty())a=line;else b=line;
        }
        emit(prism(a,b,"Longitudinal cheek"),true);
    }
    const double circle=(h.radius+h.flange)*.5;const int count=std::max(8,2*int(std::ceil(pi*circle/.25)));
    for(int k=0;k<count;++k){const double angle=2*pi*k/count;
        bolt(out,h.world({circle*std::cos(angle),circle*std::sin(angle),h.mount-h.lower}),h.frame.up,h.lower+h.upper,.024,owner);}
}
struct Edge {Vec3 a,b;double ra,rb;};
struct Entry {Key key;int end;Vec3 outward;double radius;};
double plateThickness(double r){return std::min(.09,std::max(.045,r*.065));}
}

std::vector<FabricationPart> buildSupportFabrication(const Track& track,const std::vector<Support>& supports,Cancel cancel){
    if(track.profile!=TrackProfile::Exa)throw std::runtime_error("Fabrication requires the Exa section profile");
    std::vector<Head> heads;std::vector<FabricationPart> out;
    std::map<Key,Edge> edges;std::map<Node,std::vector<Entry>> nodes;
    std::map<Node,std::pair<Key,SupportMember>> foundations;
    auto addEdge=[&](Key key,Edge e){
        if(norm(e.b-e.a)<.01)throw std::runtime_error("Degenerate fabricated member");
        edges.emplace(key,e);
        nodes[node(e.a)].push_back({key,0,unit(e.b-e.a),e.ra});nodes[node(e.b)].push_back({key,1,unit(e.a-e.b),e.rb});
    };
    for(size_t i=0;i<supports.size();++i){
        poll(cancel);heads.push_back(head(track,supports[i]));const auto& h=heads.back();
        for(size_t j=0;j<supports[i].members.size();++j){
            const auto& m=supports[i].members[j];const Key key{i,int(j)};
            if(m.kind!=SupportMemberKind::Steel){foundations[node(m.top)]={key,m};continue;}
            if(m.spineContact)continue;
            Edge e{m.base,m.top,m.radiusBase,m.radiusTop};
            if(j==h.parent){e.ra=e.rb=h.radius;if(h.inlineStem)e.b=h.end;}
            addEdge(key,e);
        }
        if(!h.inlineStem)addEdge({i,-1},{supports[i].members[h.parent].top,h.end,h.radius,h.radius});
        addHead(out,track,supports[i],h,i);
    }
    // Footing tops contain the inclined circular end and anchors. The original
    // ground footprint and foundation axis stay fixed.
    for(auto& [n,value]:foundations){
        auto& [key,m]=value;
        const Vec3 bearing=unit(m.top-m.base);const auto bearingFrame=basis(m.base,m.top);
        for(int iteration=0;iteration<4;++iteration){
            const double height=plateThickness(m.radiusTop)-.002;
            for(const auto& entry:nodes[n])if(entry.end==0){
                const auto& e=edges.at(entry.key);const auto f=basis(e.a,e.b);const double taper=(e.rb-e.ra)/norm(e.b-e.a);
                if(dot(f.axis,bearing)<=.05)throw std::runtime_error("Foundation member does not rise away from its bearing");
                for(int k=0;k<sides;++k){const double angle=2*pi*k/sides;const Vec3 radial=f.right*std::cos(angle)+f.up*std::sin(angle);
                    const double t=(height-dot(e.a-m.top,bearing)-dot(radial,bearing)*e.ra)/(dot(f.axis,bearing)+dot(radial,bearing)*taper);
                    const Vec3 p=e.a+f.axis*t+radial*(e.ra+t*taper);
                    const Vec3 v=p-m.top;m.radiusTop=std::max(m.radiusTop,(norm(v-bearing*dot(v,bearing))+.14)/.94);
                }
            }
        }
        if(m.radiusTop>m.radiusBase-.05)throw std::runtime_error("Fabricated bearing exceeds its retained foundation footprint");
        auto concrete=tube(m.base,m.top,m.radiusBase,m.radiusTop,m.kind==SupportMemberKind::RockAnchor?"Rock socket":"Foundation",48);
        concrete.material=FabricationMaterial::Concrete;concrete.owner=key.first;concrete.member=key.second;concrete.buried=true;out.push_back(std::move(concrete));
        const double radius=m.radiusTop*.94,thickness=plateThickness(m.radiusTop),circle=radius-.05;
        auto plate=tube(m.top-bearing*.005,m.top+bearing*thickness,radius,radius,m.kind==SupportMemberKind::RockAnchor?"Rock bearing plate":"Foundation plate");plate.owner=key.first;out.push_back(std::move(plate));
        if(m.kind==SupportMemberKind::RockAnchor)for(const auto& entry:nodes[n])if(entry.end==0){
            const auto& e=edges.at(entry.key);const auto frame=basis(e.a,e.b);
            const double taper=(e.rb-e.ra)/norm(e.b-e.a);
            for(int k=0;k<4;++k){const double angle=pi*k*.5;
                const Vec3 radial=frame.right*std::cos(angle)+frame.up*std::sin(angle);
                const double start=(thickness-dot(radial,bearing)*e.ra)/(dot(frame.axis,bearing)+dot(radial,bearing)*taper);
                const Vec3 toe=e.a+frame.axis*start+radial*(e.ra+start*taper);
                const Vec3 planar=toe-m.top-bearing*dot(toe-m.top,bearing);
                if(norm(planar)>radius-.22)continue;
                const Vec3 outer=m.top+unit(planar)*(radius-.13)+bearing*thickness;
                const double end=start+std::min(e.ra*1.8,norm(e.b-e.a)*.20);
                const Vec3 high=e.a+frame.axis*end+radial*((e.ra+end*taper)*.985);
                const Vec3 width=unit(cross(high-toe,outer-toe))*.028;
                auto web=prism({toe-width,outer-width,high-width},{toe+width,outer+width,high+width},"Rock bracket stiffener");
                web.owner=key.first;out.push_back(std::move(web));
            }
        }
        const int count=std::max(12,2*int(std::ceil(pi*circle/.24)));
        for(int k=0;k<count;++k){const double angle=2*pi*k/count;const Vec3 p=m.top+bearingFrame.right*(circle*std::cos(angle))+bearingFrame.up*(circle*std::sin(angle))+bearing*thickness;bool buried=false;
            for(const auto& entry:nodes[n])if(entry.end==0){const auto& e=edges.at(entry.key);const auto axis=unit(e.b-e.a);
                for(double height:{0.,.04}){const Vec3 d=p+bearing*height-e.a;buried|=norm(d-axis*dot(d,axis))<e.ra+.07;}}
            if(!buried)bolt(out,p-bearing*thickness,bearing,thickness,.025,key.first);
        }
    }
    using Endpoint=std::pair<Key,int>;
    std::map<Endpoint,Ring> rings;std::map<Endpoint,Vec3> planes;std::map<Node,std::array<Entry,2>> masters;
    std::map<Node,size_t> barrels;
    for(const auto& [n,entries]:nodes){
        // Multiple legs on one pedestal each bear on the horizontal plate;
        // they must never borrow a pipe-to-pipe mitre below ground level.
        if(foundations.contains(n))continue;
        poll(cancel);bool found=false;std::tuple<double,double,size_t,size_t> choice;
        double largest=0;for(const auto& entry:entries)largest=std::max(largest,entry.radius);
        for(size_t i=0;i<entries.size();++i)for(size_t j=i+1;j<entries.size();++j){
            const auto& a=entries[i];const auto& b=entries[j];const double cosine=dot(a.outward,b.outward);
            if(cosine<(entries.size()==2?.65:-.2)&&(entries.size()==2||std::min(a.radius,b.radius)>=largest*.65)&&
               std::max(a.radius,b.radius)>=largest-.000001&&(std::min(a.radius,b.radius)/std::max(a.radius,b.radius)>=.65||entries.size()==2)){
                // Keep the straight through chord as the host at a crowded
                // node. Choosing the two thickest rakers made an acute mitre
                // protrude through the chord at crowns and portal heads.
                // At least one host must carry the largest incoming diameter;
                // a smaller straight pair cannot enclose a wider branch port.
                auto option=std::tuple{-cosine,std::min(a.radius,b.radius),i,j};if(!found||option>choice){choice=option;found=true;}
            }
        }
        if(!found){
            if(entries.size()<2)continue;
            // A return arm and both legs can all approach from the same side.
            // Their open discs cannot form a through mitre. A short closed
            // welded node can receives the fitted ports below its end cap.
            Vec3 axis{};for(const auto& entry:entries)axis=axis+entry.outward*entry.radius;
            axis=norm(axis)>.1?unit(axis):entries.front().outward;
            const auto& first=entries.front();const auto& edge=edges.at(first.key);
            const Vec3 centre=first.end?edge.b:edge.a;const double radius=largest*1.04+.012;
            auto barrel=tube(centre-axis*(largest*1.25),centre+axis*(largest*2.),radius,radius,"Welded node can");
            barrel.owner=first.key.first;barrels[n]=out.size();out.push_back(std::move(barrel));continue;
        }
        const auto& a=entries[std::get<2>(choice)];const auto& b=entries[std::get<3>(choice)];
        const Vec3 normal=unit(a.outward-b.outward),crossAxis=cross(a.outward,b.outward);
        const Vec3 right=norm(crossAxis)<1e-6?basis({},normal).right:unit(crossAxis),up=cross(normal,right);
        const auto& e=edges.at(a.key);const Vec3 centre=a.end?e.b:e.a;
        const double radius=std::max(a.radius,b.radius),stretch=radius/std::abs(dot(normal,a.outward));
        Ring r;for(int k=0;k<fittedSides;++k){const double angle=2*pi*k/fittedSides;r.push_back(centre+right*(radius*std::cos(angle))+up*(stretch*std::sin(angle)));}
        rings[{a.key,a.end}]=r;rings[{b.key,b.end}]=r;planes[{a.key,a.end}]=normal;planes[{b.key,b.end}]=normal*-1;
        masters[n]={a,b};
    }
    auto tubeExit=[](Vec3 v,Vec3 ray,Vec3 axis,double radius,double taper,Vec3 plane,double length)->std::optional<double>{
        const Vec3 vp=v-axis*dot(v,axis),rp=ray-axis*dot(ray,axis);
        // The host may taper. Intersecting an infinite constant-radius cylinder
        // left near-parallel braces centimetres short of the visible cone.
        const double r=radius+taper*dot(v,axis),dr=taper*dot(ray,axis);
        const double a=dot(rp,rp)-dr*dr,b=2*(dot(vp,rp)-r*dr),c=dot(vp,vp)-r*r,discriminant=b*b-4*a*c;
        if(a<1e-12||discriminant<0)return {};
        double lo=(-b-std::sqrt(discriminant))/(2*a),hi=(-b+std::sqrt(discriminant))/(2*a);
        for(const auto& [n,limit]:std::array<std::pair<Vec3,double>,2>{{{plane,0.},{axis*-1,-length}}}){
            const double value=dot(n,v)-limit,slope=dot(n,ray);
            if(std::abs(slope)<1e-12){if(value< -1e-9)return {};}
            else if(slope>0)lo=std::max(lo,-value/slope);else hi=std::min(hi,-value/slope);
        }
        return hi>=std::max(0.,lo)?std::optional<double>(hi):std::nullopt;
    };
    std::map<Key,size_t> tubeParts;
    for(const auto& [key,e]:edges){
        poll(cancel);const auto f=basis(e.a,e.b);const double length=norm(e.b-e.a);std::array<Ring,2> endRings;
        for(int end:{0,1}){
            const Endpoint endpoint{key,end};auto paired=rings.find(endpoint);
            if(paired!=rings.end()){endRings[end]=paired->second;continue;}
            const Vec3 centre=end?e.b:e.a,outward=f.axis*(end?-1.:1.);const double radius=end?e.rb:e.ra;
            const auto ground=foundations.find(node(centre));const auto host=masters.find(node(centre));
            for(int k=0;k<fittedSides;++k){
                const double angle=2*pi*k/fittedSides;const Vec3 radial=f.right*std::cos(angle)+f.up*std::sin(angle);Vec3 v=radial*radius;double distance=0;
                if(end==0&&ground!=foundations.end()){
                    const auto& m=ground->second.second;const Vec3 bearing=unit(m.top-m.base);const double height=plateThickness(m.radiusTop)-.002,taper=(e.rb-e.ra)/length;
                    distance=(height-dot(centre-m.top+v,bearing))/(dot(f.axis,bearing)+dot(radial,bearing)*taper);v=radial*(radius+distance*taper);
                }else if(host!=masters.end()){
                    std::optional<double> farthest;
                    for(const auto& h:host->second){const auto& he=edges.at(h.key);
                        const double hostLength=norm(he.b-he.a),taper=((h.end?he.ra:he.rb)-h.radius)/hostLength;
                        auto value=tubeExit(v,outward,h.outward,h.radius,taper,planes.at({h.key,h.end}),hostLength);
                        if(value&&(!farthest||*value>*farthest))farthest=value;}
                    if(farthest){distance=*farthest-.003;if(distance>length*.95)throw std::runtime_error("Near-parallel branch lacks fabrication length");}
                }
                endRings[end].push_back(centre+v+outward*distance);
            }
        }
        const auto radial=[&](Vec3 p,Vec3 centre){const Vec3 d=p-centre;return unit(d-f.axis*dot(d,f.axis));};
        double best=INFINITY;Ring second;
        for(int reverse:{0,1})for(int shift=0;shift<fittedSides;++shift){
            double cost=0;Ring candidate;
            for(int k=0;k<fittedSides;++k){int j=(k+shift)%fittedSides;if(reverse)j=fittedSides-1-j;const Vec3 p=endRings[1][j];
                candidate.push_back(p);cost+=1-dot(radial(p,e.b),radial(endRings[0][k],e.a));}
            if(cost<best){best=cost;second=std::move(candidate);}
        }
        auto part=prism(endRings[0],second,"Fitted tube",true);part.axis=f.axis;part.owner=key.first;part.member=key.second;tubeParts[key]=out.size();out.push_back(std::move(part));
        if(key.second>=0&&length>=12&&std::max(e.ra,e.rb)>=.28){
            const int count=int(std::ceil(length/12));
            for(int j=1;j<count;++j){const double t=double(j)/count,radius=e.ra+(e.rb-e.ra)*t,outer=radius+std::max(.12,radius*.18),thickness=std::min(.075,std::max(.028,radius*.07));
                const Vec3 centre=e.a+(e.b-e.a)*t;
                for(int sign:{-1,1}){auto flange=ring(centre+f.axis*(sign<0?-thickness:.001),centre+f.axis*(sign<0?-.001:thickness),radius-.003,outer,"Pipe flange");flange.owner=key.first;out.push_back(std::move(flange));}
                const double circle=(radius+outer)*.5;const int bolts=std::max(8,2*int(std::ceil(pi*circle/.28)));
                for(int k=0;k<bolts;++k){const double a=2*pi*k/bolts;bolt(out,centre-f.axis*thickness+(f.right*std::cos(a)+f.up*std::sin(a))*circle,f.axis,thickness*2,.022,key.first);}
            }
        }
    }
    // Finish branch welds against the rendered host faces, not an idealised
    // circular solid. Mitres, tapered endpoints and polygon phase all change
    // that surface; the cylinder-only cope left exposed crescent-shaped caps.
    auto exitFace=[](Vec3 origin,Vec3 ray,Vec3 a,Vec3 b,Vec3 c)->std::optional<double>{
        const Vec3 ab=b-a,ac=c-a,p=cross(ray,ac);const double det=dot(ab,p);
        if(std::abs(det)<1e-11||dot(cross(ab,ac),ray)<=1e-10)return {};
        const Vec3 v=origin-a;const double u=dot(v,p)/det;
        if(u< -1e-8||u>1+1e-8)return {};
        const Vec3 q=cross(v,ab);const double w=dot(ray,q)/det;
        if(w< -1e-8||u+w>1+1e-8)return {};
        const double t=dot(ac,q)/det;return t>=-1e-8?std::optional<double>(std::max(0.,t)):std::nullopt;
    };
    for(int pass=0;pass<3;++pass){
        double movement=0;
        for(const auto& [key,e]:edges){
            poll(cancel);auto& part=out[tubeParts.at(key)];const auto f=basis(e.a,e.b);const double length=norm(e.b-e.a);
            for(int end:{0,1}){
                const Vec3 centre=end?e.b:e.a,ray=f.axis*(end?-1.:1.);
                const auto host=masters.find(node(centre));const auto barrel=barrels.find(node(centre));
                if(rings.contains({key,end})||(host==masters.end()&&barrel==barrels.end())||foundations.contains(node(centre)))continue;
                std::vector<size_t> hosts;
                if(host!=masters.end())for(const auto& h:host->second)hosts.push_back(tubeParts.at(h.key));
                if(barrel!=barrels.end())hosts.push_back(barrel->second);
                for(auto index:part.faces[part.faces.size()-2+end]){
                    const Vec3 old=part.vertices[index],offset=old-centre,origin=centre+offset-ray*dot(offset,ray);
                    std::optional<double> farthest;
                    for(size_t hostIndex:hosts){const auto& surface=out[hostIndex];
                        for(const auto& face:surface.faces)for(size_t j=1;j+1<face.size();++j){
                            auto hit=exitFace(origin,ray,surface.vertices[face[0]],surface.vertices[face[j]],surface.vertices[face[j+1]]);
                            if(hit&&*hit<length*.95&&(!farthest||*hit>*farthest))farthest=hit;
                        }
                    }
                    if(farthest){const Vec3 next=origin+ray*(*farthest-.003);movement=std::max(movement,norm(next-old));part.vertices[index]=next;}
                }
            }
        }
        if(movement<1e-7)break;
    }
    return out;
}

SupportMeshBuffer fabricationMesh(const FabricationPart& part){
    SupportMeshBuffer out;std::vector<Vec3> smooth(part.vertices.size());
    auto faceNormal=[&](const std::vector<uint32_t>& f){Vec3 n{};for(size_t j=1;j+1<f.size();++j)n=n+cross(part.vertices[f[j]]-part.vertices[f[0]],part.vertices[f[j+1]]-part.vertices[f[0]]);return unit(n);};
    auto smoothing=[&](size_t index){return part.smooth&&part.faces[index].size()==4&&(part.name.find("flange")==std::string::npos||index%4>=2);};
    for(size_t i=0;i<part.faces.size();++i)if(smoothing(i)){const Vec3 n=faceNormal(part.faces[i]);for(auto j:part.faces[i])smooth[j]=smooth[j]+n;}
    for(size_t i=0;i<part.faces.size();++i){
        const auto& f=part.faces[i];const Vec3 n=faceNormal(f);const uint32_t base=uint32_t(out.positions.size());
        for(auto j:f){out.positions.push_back(part.vertices[j]);out.normals.push_back(smoothing(i)?unit(smooth[j]):n);}
        for(uint32_t j=1;j+1<f.size();++j)out.indices.insert(out.indices.end(),{base,base+j,base+j+1});
    }
    return out;
}

namespace {
StationBox bounds(const FabricationPart& p,const TrackSample& q){
    const auto f=norm(p.axis)>.5?basis({},p.axis):Basis{q.tangent,q.right,q.up};
    Vec3 lo{INFINITY,INFINITY,INFINITY},hi{-INFINITY,-INFINITY,-INFINITY};
    const Vec3 origin=p.vertices.front();
    for(Vec3 v:p.vertices){v=v-origin;const Vec3 x{dot(v,f.axis),dot(v,f.right),dot(v,f.up)};
        lo={std::min(lo.x,x.x),std::min(lo.y,x.y),std::min(lo.z,x.z)};hi={std::max(hi.x,x.x),std::max(hi.y,x.y),std::max(hi.z,x.z)};}
    const Vec3 c=(lo+hi)*.5;
    return {origin+f.axis*c.x+f.right*c.y+f.up*c.z,f.axis,f.right,f.up,(hi-lo)*.5,StationRole::Post};
}
// Separating projections against the entire part's convex hull. This remains
// conservative for hollow/concave parts; it never exempts material in a flange.
bool separated(const FabricationPart& p,const StationBox& b){
    const auto gap=[&](Vec3 axis){
        if(norm(axis)<1e-10)return false;
        const double r=std::abs(dot(axis,b.forward))*b.half.x+std::abs(dot(axis,b.right))*b.half.y+std::abs(dot(axis,b.up))*b.half.z;
        double lo=INFINITY,hi=-INFINITY;
        for(Vec3 v:p.vertices){const double x=dot(v-b.center,axis);lo=std::min(lo,x);hi=std::max(hi,x);}
        return lo>r+1e-9||hi< -r-1e-9;
    };
    for(Vec3 a:{b.forward,b.right,b.up})if(gap(a))return true;
    for(const auto& face:p.faces){
        if(gap(cross(p.vertices[face[1]]-p.vertices[face[0]],p.vertices[face[2]]-p.vertices[face[0]])))return true;
        for(size_t i=0;i<face.size();++i){const Vec3 edge=p.vertices[face[(i+1)%face.size()]]-p.vertices[face[i]];
            for(Vec3 a:{b.forward,b.right,b.up})if(gap(cross(edge,a)))return true;}
    }
    return false;
}
double groundBound(const FabricationPart& p,const Terrain& terrain,Cancel cancel){
    double low=INFINITY,minX=INFINITY,minY=INFINITY,maxX=-INFINITY,maxY=-INFINITY;
    for(Vec3 v:p.vertices){low=std::min(low,v.z);minX=std::min(minX,v.x);maxX=std::max(maxX,v.x);minY=std::min(minY,v.y);maxY=std::max(maxY,v.y);}
    if(terrain.kind==TerrainKind::Flat)return low;
    const auto range=terrain.heightRange((minX+maxX)*.5,(minY+maxY)*.5,std::hypot(maxX-minX,maxY-minY)*.5);
    if(low>=range[1])return low-range[1];
    // Clip each triangle to the exact rendered terrain triangles. Affine height
    // difference has its minimum at a clipped vertex, including face interiors.
    double result=INFINITY;const double step=Terrain::gridStep;
    for(const auto& face:p.faces)for(size_t j=1;j+1<face.size();++j){
        poll(cancel);const std::array<Vec3,3> tri{p.vertices[face[0]],p.vertices[face[j]],p.vertices[face[j+1]]};
        double ax=INFINITY,ay=INFINITY,bx=-INFINITY,by=-INFINITY;
        for(Vec3 v:tri){ax=std::min(ax,v.x);ay=std::min(ay,v.y);bx=std::max(bx,v.x);by=std::max(by,v.y);}
        for(double x=std::floor(ax/step)*step;x<=bx;x+=step)for(double y=std::floor(ay/step)*step;y<=by;y+=step){
            const Vec3 a{x,y,terrain.vertexHeight(x,y)},b{x+step,y,terrain.vertexHeight(x+step,y)},c{x+step,y+step,terrain.vertexHeight(x+step,y+step)},d{x,y+step,terrain.vertexHeight(x,y+step)};
            for(const std::array<Vec3,3>& t:{std::array<Vec3,3>{a,b,c},std::array<Vec3,3>{a,c,d}}){
                std::vector<Vec3> polygon(tri.begin(),tri.end());
                for(int edge=0;edge<3&&!polygon.empty();++edge){std::vector<Vec3> next;const Vec3 from=t[edge],line=t[(edge+1)%3]-from;
                    const auto side=[&](Vec3 v){return line.x*(v.y-from.y)-line.y*(v.x-from.x);};
                    for(size_t k=0;k<polygon.size();++k){const Vec3 u=polygon[k],v=polygon[(k+1)%polygon.size()];const double du=side(u),dv=side(v);
                        if(du>=0)next.push_back(u);if((du<0)!=(dv<0))next.push_back(u+(v-u)*(du/(du-dv)));}
                    polygon=std::move(next);
                }
                const Vec3 normal=cross(t[1]-t[0],t[2]-t[0]);
                for(Vec3 v:polygon)result=std::min(result,dot(v-t[0],normal)/normal.z);
            }
        }
    }
    return result-1e-9;
}
}
ValidationReport validateSupportFabrication(const Track& track,const std::vector<Support>& supports,const Terrain& terrain,
    const StationGeometry& station,const ClearanceSweep& sweep,Cancel cancel){
    ValidationReport report;
    if(track.profile==TrackProfile::Legacy)return report;
    try{
        if(sweep.trackProfile()!=track.profile)throw std::runtime_error("Fabrication clearance profile mismatch");
        const auto parts=buildSupportFabrication(track,supports,cancel);
        const auto hardware=trackHardwareLocal(track.profile);
        const auto section=trackSection(track.profile);
        std::map<std::array<int,6>,std::vector<size_t>> cache;
        const double reserve=sweep.bodyRadius()+sweep.padding();
        for(const auto& part:parts){
            poll(cancel);const auto& support=supports.at(part.owner);const auto frame=track.sample(support.trackDistance);
            const auto box=bounds(part,frame);
            auto fail=[&](std::string code,std::string cause){report.fail(std::move(code),part.name+" / support "+std::to_string(part.owner)+" / member "+std::to_string(part.member)+": "+cause,support.trackDistance);};
            if(!part.buried&&groundBound(part,terrain,cancel)<-1e-6){fail("FABRICATION_TERRAIN","visible steel intersects terrain");return report;}
            if(station.enabled)for(const auto& b:station.boxes)if(stationBoxesOverlap(box,b)&&!separated(part,b)){
                fail("FABRICATION_STATION","visible fitting intersects station");return report;}
            Vec3 lo{INFINITY,INFINITY,INFINITY},hi{-INFINITY,-INFINITY,-INFINITY};
            for(Vec3 v:part.vertices){lo={std::min(lo.x,v.x),std::min(lo.y,v.y),std::min(lo.z,v.z)};hi={std::max(hi.x,v.x),std::max(hi.y,v.y),std::max(hi.z,v.z)};}
            lo=lo-Vec3{reserve,reserve,reserve};hi=hi+Vec3{reserve,reserve,reserve};
            const std::array<int,6> key{int(std::floor(lo.x/16)),int(std::floor(lo.y/16)),int(std::floor(lo.z/16)),int(std::floor(hi.x/16)),int(std::floor(hi.y/16)),int(std::floor(hi.z/16))};
            auto it=cache.find(key);if(it==cache.end())it=cache.emplace(key,sweep.nearbyFrames(lo,hi)).first;
            for(size_t index:it->second){
                const auto& cell=sweep.frames()[index];const auto& q=cell.sample;
                // Cheap world-space rejection before the oriented hull checks.
                if(q.position.x<lo.x||q.position.x>hi.x||q.position.y<lo.y||q.position.y>hi.y||q.position.z<lo.z||q.position.z>hi.z)continue;
                const double pad=sweep.padding();
                const StationBox rider{q.position+q.up*(sweep.trainTop()*.5),q.tangent,q.right,q.up,{trainHalfLength+pad,patronHalfWidth+pad,sweep.trainTop()*.5+pad},StationRole::Post};
                if(stationBoxesOverlap(box,rider)&&!separated(part,rider)){fail("FABRICATION_RIDER","visible fitting enters swept rider clearance at "+std::to_string(cell.distance));return report;}
                double distance=std::abs(cell.distance-support.trackDistance);if(track.closed)distance=std::min(distance,track.length-distance);
                for(size_t h=0;h<hardware.size();++h){
                    if(h==0&&part.spineSeat&&distance<1.35)continue;
                    auto b=trackWebWorld(hardware[h],q);const double hp=section.hardwarePadding();b.half=b.half+Vec3{hp,hp,hp};
                    if(stationBoxesOverlap(box,b)&&!separated(part,b)){fail("FABRICATION_TRACK","visible fitting intersects track hardware "+std::to_string(h)+" at "+std::to_string(cell.distance));return report;}
                }
            }
        }
    }catch(const std::exception& e){report.fail(std::string(e.what())=="CANCELLED"?"CANCELLED":"FABRICATION_GEOMETRY",e.what());}
    return report;
}
FabricationPart exaCrosshead(const TrackSample& q){
    auto result=tube({0,-.62,0},{0,.62,0},.065,.065,"Centred crosshead");
    const double bottom=-.8+std::sqrt(.34*.34-.18*.18)-.012,top=-.065+.008;
    for(int sign:{-1,1}){
        Ring a{{-.19,sign*.18-.012,bottom},{.19,sign*.18-.012,bottom},{.34,sign*.18-.012,top},{-.34,sign*.18-.012,top}},b=a;
        for(auto& v:b)v.y+=.024;auto web=prism(a,b,"Crosshead web");const uint32_t offset=uint32_t(result.vertices.size());
        result.vertices.insert(result.vertices.end(),web.vertices.begin(),web.vertices.end());
        for(auto f:web.faces){for(auto& i:f)i+=offset;result.faces.push_back(std::move(f));}
    }
    // Mixed round/plate assembly uses flat normals; the individual cylindrical
    // crosshead can be smoothed by render adapters without rounding the webs.
    result.smooth=false;result.axis=q.right;
    for(auto& v:result.vertices)v=q.position+q.tangent*v.x+q.right*v.y+q.up*v.z;
    return result;
}
}
