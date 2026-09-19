#include "coaster/coaster.hpp"
#include "coaster/track_hardware.hpp"
#include <stdexcept>

namespace coaster {
namespace {
constexpr double arcCell=.04,angleCell=.08,cellSize=16;
constexpr size_t maxCells=1000000,maxNodes=2000000;
constexpr unsigned maxDepth=30;
constexpr double choose(unsigned n,unsigned k){double x=1;for(unsigned j=1;j<=k;++j)x=x*(n+1-j)/j;return x;}
template<class T,size_t N> std::array<T,N> bernstein(const std::array<T,N>& power){
    std::array<T,N> out{};for(size_t j=0;j<N;++j)for(size_t k=0;k<=j;++k)out[j]=out[j]+power[k]*(choose(unsigned(j),unsigned(k))/choose(unsigned(N-1),unsigned(k)));return out;
}
template<class T,size_t N> std::array<T,N-1> derivative(const std::array<T,N>& power){
    std::array<T,N-1> out{};for(size_t k=1;k<N;++k)out[k-1]=power[k]*double(k);return out;
}
template<class T,size_t N> void split(const std::array<T,N>& source,std::array<T,N>& left,std::array<T,N>& right){
    auto temp=source;left[0]=temp[0];right[N-1]=temp[N-1];for(size_t r=1;r<N;++r){for(size_t j=0;j<N-r;++j)temp[j]=(temp[j]+temp[j+1])*.5;left[r]=temp[0];right[N-1-r]=temp[N-1-r];}
}
template<class T,size_t N> T midpoint(std::array<T,N> temp){for(size_t r=1;r<N;++r)for(size_t j=0;j<N-r;++j)temp[j]=(temp[j]+temp[j+1])*.5;return temp[0];}
double magnitude(double x){return std::abs(x);}double magnitude(Vec3 x){return norm(x);}
template<class T,size_t N> double errorReserve(const std::array<T,N>& power){double x=1;for(const auto& p:power)x+=magnitude(p);return x*1e-10;}
template<class T,size_t N> double maximum(const std::array<T,N>& controls,double reserve){double x=0;for(const auto& p:controls)x=std::max(x,magnitude(p));return x+reserve;}
struct Node {
    std::array<Vec3,9> velocity;
    std::array<Vec3,8> acceleration,rawUp;
    std::array<Vec3,7> upDerivative;
    std::array<double,7> bankDerivative;
    double begin{},end{1};unsigned depth{};
};
struct Reserves {double velocity,acceleration,up,upDerivative,bankDerivative;};
void splitNode(const Node& node,Node& left,Node& right){
    split(node.velocity,left.velocity,right.velocity);split(node.acceleration,left.acceleration,right.acceleration);
    split(node.rawUp,left.rawUp,right.rawUp);split(node.upDerivative,left.upDerivative,right.upDerivative);split(node.bankDerivative,left.bankDerivative,right.bankDerivative);
    const double mid=(node.begin+node.end)*.5;left.begin=node.begin;left.end=mid;right.begin=mid;right.end=node.end;left.depth=right.depth=node.depth+1;
}
bool motionBounds(const Node& node,const Reserves& reserve,double& speed,double& omega){
    const Vec3 vmid=midpoint(node.velocity),wmid=midpoint(node.rawUp);
    const double vnorm=norm(vmid),wnorm=norm(wmid);
    if(!std::isfinite(vnorm)||!std::isfinite(wnorm)||vnorm<=reserve.velocity||wnorm<=reserve.up)return false;
    const Vec3 axis=vmid/vnorm,waxis=wmid/wnorm;
    double vmin=std::numeric_limits<double>::infinity(),wmin=vmin;
    for(auto p:node.velocity)vmin=std::min(vmin,dot(p,axis));
    for(auto p:node.rawUp)wmin=std::min(wmin,dot(p,waxis));
    vmin-=reserve.velocity;wmin-=reserve.up;
    if(vmin<=1e-10||wmin<=1e-10)return false;
    speed=maximum(node.velocity,reserve.velocity);
    const double K=maximum(node.acceleration,reserve.acceleration)/vmin;
    const double D=maximum(node.upDerivative,reserve.upDerivative),wmax=maximum(node.rawUp,reserve.up);
    const double half=(node.end-node.begin)*.5;
    // P' and W controls are restricted in u, but their derivative values remain
    // derivatives with respect to the ORIGINAL span parameter. Projection axis
    // error is reserved too; near-degenerate speed/projection fails closed.
    const double centreError=2*reserve.velocity/vmin*wmax+reserve.up;
    const double c=std::abs(dot(axis,wmid))+centreError+(K*wmax+D)*half;
    if(!std::isfinite(c)||c>=wmin)return false;
    const double nmin=std::sqrt((wmin-c)*(wmin+c));
    omega=K+(D+c*K)/nmin+maximum(node.bankDerivative,reserve.bankDerivative);
    return std::isfinite(speed)&&speed>0&&std::isfinite(omega)&&omega>=0;
}
bool intersectsBox(Vec3 a,Vec3 b,Vec3 lo,Vec3 hi){
    double begin=0,end=1;Vec3 delta=b-a;
    const double aa[]={a.x,a.y,a.z},dd[]={delta.x,delta.y,delta.z},ll[]={lo.x,lo.y,lo.z},hh[]={hi.x,hi.y,hi.z};
    for(int axis=0;axis<3;++axis){if(std::abs(dd[axis])<1e-12){if(aa[axis]<ll[axis]||aa[axis]>hh[axis])return false;}else{double x=(ll[axis]-aa[axis])/dd[axis],y=(hh[axis]-aa[axis])/dd[axis];if(x>y)std::swap(x,y);begin=std::max(begin,x);end=std::min(end,y);if(begin>end)return false;}}
    return true;
}
}
ClearanceSweep buildClearanceSweep(const Track& source,const TrainConfig& train,Cancel cancel){
    if(cancel&&cancel())throw std::runtime_error("CANCELLED");
    if(!std::isfinite(train.seatHeight)||train.seatHeight<0||train.seatHeight>3)throw std::runtime_error("Invalid clearance seat height");
    Track t=source;t.rebuild();
    if(t.spans.size()!=source.spans.size()||t.length!=source.length)throw std::runtime_error("Stale canonical span cache");
    for(size_t i=0;i<t.spans.size();++i){
        if((i&63)==0&&cancel&&cancel())throw std::runtime_error("CANCELLED");
        if(t.spans[i].start!=source.spans[i].start||t.spans[i].length!=source.spans[i].length)throw std::runtime_error("Stale canonical arc cache");
        for(size_t k=0;k<t.spans[i].c.size();++k){auto a=t.spans[i].c[k],b=source.spans[i].c[k];if(a.x!=b.x||a.y!=b.y||a.z!=b.z)throw std::runtime_error("Stale canonical polynomial cache");}
        for(size_t k=0;k<t.spans[i].referenceUp.size();++k){auto a=t.spans[i].referenceUp[k],b=source.spans[i].referenceUp[k];if(a.x!=b.x||a.y!=b.y||a.z!=b.z||t.spans[i].bank[k]!=source.spans[i].bank[k])throw std::runtime_error("Stale canonical frame cache");}
    }
    ClearanceSweep out;out.top=std::max(2.4,train.seatHeight+.6);out.length=t.length;
    // Each accepted canonical-u cell has true arc bound <=.04 m and frame
    // angular variation <=.08 rad. From its exact midpoint every body point
    // moves <=.02+.04*4.2=.188 m; every hardware point <=.02+.04*.9=.056 m.
    // The existing .20/.06 pads therefore cover the complete interval. This
    // uses the actual cached raw frame/bank, with no nlerp rate premise.
    size_t visited=0;
    for(size_t i=0;i<t.spans.size();++i){
        if(cancel&&cancel())throw std::runtime_error("CANCELLED");
        const auto& span=t.spans[i];const auto velocity=derivative(span.c);const auto acceleration=derivative(velocity);
        const auto upDerivative=derivative(span.referenceUp);const auto bankDerivative=derivative(span.bank);
        const Reserves reserve{errorReserve(velocity),errorReserve(acceleration),errorReserve(span.referenceUp),errorReserve(upDerivative),errorReserve(bankDerivative)};
        std::vector<Node> stack;stack.push_back({bernstein(velocity),bernstein(acceleration),bernstein(span.referenceUp),bernstein(upDerivative),bernstein(bankDerivative)});
        while(!stack.empty()){
            if((visited++&127)==0&&cancel&&cancel())throw std::runtime_error("CANCELLED");
            if(visited>maxNodes)throw std::runtime_error("Continuous clearance subdivision budget exceeded");
            Node node=stack.back();stack.pop_back();double speed=0,omega=0;
            if(!motionBounds(node,reserve,speed,omega)){
                if(node.depth>=maxDepth)throw std::runtime_error("Cannot certify canonical frame projection or derivative domain");
                Node left,right;splitNode(node,left,right);stack.push_back(right);stack.push_back(left);continue;
            }
            const double width=node.end-node.begin;
            const double required=std::max({1.,speed*width/arcCell,omega*width/angleCell});
            if(!std::isfinite(required)||required>double(maxCells-out.samples.size()))throw std::runtime_error("Continuous clearance cell budget exceeded");
            const size_t count=size_t(std::ceil(required));
            for(size_t j=0;j<count;++j){
                if((j&127)==0&&cancel&&cancel())throw std::runtime_error("CANCELLED");
                const double begin=node.begin+width*(double(j)/count),end=node.begin+width*(double(j+1)/count),u=(begin+end)*.5;
                const double arcBound=speed*(end-begin),angleBound=omega*(end-begin);
                // A conservative floating arithmetic reserve is already inside speed and
                // omega. Rounding of the uniform partition is tolerated only at
                // floating precision; body/hardware pads keep >.01/.003 margins.
                if(arcBound>arcCell*(1+1e-9)||angleBound>angleCell*(1+1e-9))throw std::runtime_error("Invalid clearance cell partition");
                auto p=t.sampleSpan(i,u);if(!finite(p.position)||!finite(p.tangent)||!finite(p.up)||!finite(p.right))throw std::runtime_error("Nonfinite canonical clearance frame");
                size_t index=out.samples.size();out.samples.push_back({p,t.distanceAtSpan(i,u),i,begin,end,arcBound,angleBound});
                ClearanceSweep::Key key{int(std::floor(p.position.x/cellSize)),int(std::floor(p.position.y/cellSize)),int(std::floor(p.position.z/cellSize))};out.cells[key].push_back(index);
            }
        }
    }
    return out;
}
int supportCollision(const Support& support,const ClearanceSweep& sweep,Cancel cancel){
    // Legacy solids retain exactly their saved endpoints and historical radius.
    // They receive the same corrected sweep checks; no clearance grandfathering.
    std::vector<SupportMember> legacy;
    if(support.members.empty()){
        legacy.push_back({support.base,support.top,supportRadius,supportRadius,SupportMemberKind::Steel,false});
        if(support.hasAttachment)legacy.push_back({support.top,support.attachment,supportRadius,supportRadius,SupportMemberKind::Steel,true});
    }
    const auto& members=support.members.empty()?legacy:support.members;
    for(const auto& member:members){
        if(cancel&&cancel())return -2;
        Vec3 a=member.base,b=member.top;double radius=std::max(member.radiusBase,member.radiusTop);
        // Every canonical solid is inside this maximum-radius capsule. Query only
        // occupied spatial cells intersecting its AABB expanded by the full body.
        double broad=radius+4.2+sweep.padding();
        ClearanceSweep::Key lo{int(std::floor((std::min(a.x,b.x)-broad)/cellSize)),int(std::floor((std::min(a.y,b.y)-broad)/cellSize)),int(std::floor((std::min(a.z,b.z)-broad)/cellSize))};
        ClearanceSweep::Key hi{int(std::floor((std::max(a.x,b.x)+broad)/cellSize)),int(std::floor((std::max(a.y,b.y)+broad)/cellSize)),int(std::floor((std::max(a.z,b.z)+broad)/cellSize))};
        auto inspect=[&](const std::vector<size_t>& indices){
            for(size_t index:indices){
                if((index&63)==0&&cancel&&cancel())return -2;
                const auto& f=sweep.samples[index];const auto& p=f.sample;
                auto local=[&](Vec3 v){v=v-p.position;return Vec3{dot(v,p.tangent),dot(v,p.right),dot(v,p.up)};};
                Vec3 al=local(a),bl=local(b);double margin=radius+sweep.padding();
                if(intersectsBox(al,bl,{-1.275-margin,-1.5-margin,-margin},{1.275+margin,1.5+margin,sweep.top+margin}))return int(index);
                // This box contains every hardware solid below and uses the
                // larger train pad. A miss cannot reach any detailed web test.
                if(!intersectsBox(al,bl,{-1.275-margin,-1.5-margin,-spineDepth-spineRadius-margin},
                    {1.275+margin,1.5+margin,sweep.top+margin}))continue;
                // Track hardware has corner radius <.9 m, so midpoint motion is
                // at most .02+.04*.9=.056 m. Keep the larger train pad above.
                margin=radius+.06;
                // Webs have no attachment exemption: test the entire member,
                // including an approved spine-contact endpoint. Euclidean capsule
                // distance avoids falsely filling the diagonal OBB's corners.
                for(const auto& web:trackWebsLocal())
                    if(segmentWebDistanceSquared(al,bl,web)<=(margin+1e-9)*(margin+1e-9))return int(index);
                double separation=std::abs(f.distance-support.trackDistance);separation=std::min(separation,sweep.length-separation);
                Vec3 spineEnd=b;
                if(member.spineContact&&member.kind==SupportMemberKind::Steel&&norm(b-support.attachment)<1e-5&&separation<2.5)
                    spineEnd=b+unit(a-b)*std::min(.85,norm(a-b));
                if(intersectsBox(al,local(spineEnd),{-spineRadius-margin,-spineRadius-margin,-spineDepth-spineRadius-margin},{spineRadius+margin,spineRadius+margin,-spineDepth+spineRadius+margin})||
                   intersectsBox(al,bl,{-.085-margin,-.735-margin,-.085-margin},{.085+margin,-.565+margin,.085+margin})||
                   intersectsBox(al,bl,{-.085-margin,.565-margin,-.085-margin},{.085+margin,.735+margin,.085+margin})||
                   intersectsBox(al,bl,{-.07-margin,-.825-margin,-.27-margin},{.07+margin,.825+margin,-.11+margin}))return int(index);
            }return -1;
        };
        size_t volume=size_t(hi.x-lo.x+1)*size_t(hi.y-lo.y+1)*size_t(hi.z-lo.z+1);
        if(volume<=2048){
            for(int x=lo.x;x<=hi.x;++x)for(int y=lo.y;y<=hi.y;++y)for(int z=lo.z;z<=hi.z;++z){
                if(cancel&&cancel())return -2;auto it=sweep.cells.find({x,y,z});if(it!=sweep.cells.end()){int hit=inspect(it->second);if(hit!=-1)return hit;}}
        }else{
            for(const auto& [key,indices]:sweep.cells){if(cancel&&cancel())return -2;if(key.x<lo.x||key.x>hi.x||key.y<lo.y||key.y>hi.y||key.z<lo.z||key.z>hi.z)continue;int hit=inspect(indices);if(hit!=-1)return hit;}
        }
    }
    return -1;
}
}
