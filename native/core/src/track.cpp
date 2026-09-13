#include "coaster/coaster.hpp"
#include "arc_length.hpp"
#include <stdexcept>
#include <unordered_map>
#include <optional>

namespace coaster {
bool Terrain::isDefaultProfile() const {
    return verticalScale==1&&horizontalScale==1&&offsetX==0&&offsetY==0&&headingRadians==0&&cliffHeight==0&&cliffWidth==600;
}
bool Terrain::valid() const {
    for(double value:{verticalScale,horizontalScale,offsetX,offsetY,headingRadians,cliffHeight,cliffWidth})if(!std::isfinite(value))return false;
    return int(kind)>=0&&int(kind)<=2&&verticalScale>=.1&&verticalScale<=6&&horizontalScale>=.5&&horizontalScale<=4&&
        std::abs(offsetX)<=100000&&std::abs(offsetY)<=100000&&std::abs(headingRadians)<=pi&&cliffHeight>=0&&cliffHeight<=250&&
        cliffWidth>=80&&cliffWidth<=2500&&(kind==TerrainKind::Canyon||cliffHeight==0);
}
Terrain Terrain::seeded(TerrainKind terrainKind,uint64_t seed) {
    Terrain result;result.kind=terrainKind;if(terrainKind==TerrainKind::Flat)return result;
    uint64_t state=seed^0xa07e1c9d3b5264f8ull;
    auto range=[&](double lo,double hi){uint64_t z=(state+=0x9e3779b97f4a7c15ull);z=(z^(z>>30))*0xbf58476d1ce4e5b9ull;z=(z^(z>>27))*0x94d049bb133111ebull;z^=z>>31;return lo+(hi-lo)*double(z>>11)*0x1.0p-53;};
    result.horizontalScale=range(.9,1.4);result.offsetX=range(-500,500);result.offsetY=range(-500,500);result.headingRadians=range(-pi,pi);
    if(terrainKind==TerrainKind::Hills)result.verticalScale=range(2.5,4);
    else {result.verticalScale=range(.1,.16);result.cliffHeight=range(195,225);result.cliffWidth=range(120,180);}
    return result;
}
namespace {
constexpr double canyonHalfFloorWidth=260;
// C3 compact wall: derivatives 1..3 vanish at both floor and rim.
double canyonStep(double u){u=std::clamp(u,0.,1.);return u*u*u*u*(35+u*(-84+u*(70-20*u)));}
double canyonStepDerivative(double u){u=std::clamp(u,0.,1.);const double v=u*(1-u);return 140*v*v*v;}
double canyonCoordinateSlope(){return std::hypot(1.,120./850);}
}
double Terrain::slopeBound() const {
    if(!valid())return std::numeric_limits<double>::infinity();
    const double base=kind==TerrainKind::Flat?0:kind==TerrainKind::Hills?.056:.28;
    // max S7'(u)=140*(1/4)^3=2.1875. The meandering valley coordinate
    // y-120*sin(x/850) has gradient norm <= hypot(1,120/850).
    return (base*verticalScale+2.1875*cliffHeight*canyonCoordinateSlope()/cliffWidth)/horizontalScale;
}
double Terrain::localSlopeBound(double x,double y,double radius) const {
    if(!valid()||!std::isfinite(x)||!std::isfinite(y)||!std::isfinite(radius)||radius<0)return std::numeric_limits<double>::infinity();
    const double base=(kind==TerrainKind::Flat?0:kind==TerrainKind::Hills?.056:.28)*verticalScale/horizontalScale;
    if(cliffHeight==0)return base;
    const double dx=(x-offsetX)/horizontalScale,dy=(y-offsetY)/horizontalScale,c=std::cos(headingRadians),s=std::sin(headingRadians);
    const double localX=c*dx+s*dy,localY=-s*dx+c*dy;
    const double coordinate=std::abs(localY-120*std::sin(localX/850));
    // The entire query disk maps into this interval, even when it crosses
    // the valley center, floor/wall join, derivative maximum or wall/rim join.
    const double delta=canyonCoordinateSlope()*radius/horizontalScale;
    const double lo=(std::max(0.,coordinate-delta)-canyonHalfFloorWidth)/cliffWidth;
    const double hi=(coordinate+delta-canyonHalfFloorWidth)/cliffWidth;
    if(hi<=0||lo>=1)return base;
    const double peak=std::clamp(.5,std::max(0.,lo),std::min(1.,hi));
    return base+cliffHeight*canyonStepDerivative(peak)*canyonCoordinateSlope()/(cliffWidth*horizontalScale);
}
double Terrain::height(double x,double y) const {
    if(kind==TerrainKind::Flat)return 0;
    const double dx=(x-offsetX)/horizontalScale,dy=(y-offsetY)/horizontalScale;
    const double c=std::cos(headingRadians),s=std::sin(headingRadians);
    x=c*dx+s*dy;y=-s*dx+c*dy;
    switch(kind){
    case TerrainKind::Flat:return 0;
    case TerrainKind::Hills:return verticalScale*(12*std::sin(x/470)*std::sin(y/390)+8*std::sin((x+y)/720));
    case TerrainKind::Canyon:return verticalScale*(-65*std::exp(-std::pow((y-120*std::sin(x/850))/210,2))+12*std::sin(x/630))+
        cliffHeight*canyonStep((std::abs(y-120*std::sin(x/850))-canyonHalfFloorWidth)/cliffWidth);
    }return 0;
}
std::string Terrain::name() const {return kind==TerrainKind::Flat?"flat":kind==TerrainKind::Hills?"hills":"canyon";}
static Vec3 transport(Vec3 up,Vec3 a,Vec3 b){Vec3 c=cross(a,b);double s=norm(c);if(s>1e-10)up=rotate(up,c/s,std::atan2(s,dot(a,b)));return unit(up-b*dot(up,b));}
static Vec3 der(const Span& sp,double u){Vec3 v=sp.c[7]*7;for(int i=6;i>=1;--i)v=v*u+sp.c[i]*i;return v;}
static double arc(const Span& sp,double u){return detail::spanArcLength(sp,u);}
static double binomial(int n,int k){double result=1;for(int i=1;i<=k;++i)result=result*(n-i+1)/i;return result;}
void Track::rebuild(){
    if(knots.size()<4||knots.size()>200000)throw std::runtime_error("Invalid knot count");
    for(const auto& k:knots)if(!finite(k.position)||!finite(k.tangent)||!finite(k.curvature)||!finite(k.up)||int(k.element)<0||int(k.element)>7||!std::isfinite(k.bank)||std::abs(k.bank)>64*pi||norm(k.position)>1000000||std::abs(norm(k.tangent)-1)>1e-5||std::abs(norm(k.up)-1)>1e-5||std::abs(dot(k.tangent,k.up))>1e-5||std::abs(dot(k.tangent,k.curvature))>1e-5||norm(k.curvature)>2)throw std::runtime_error("Invalid canonical frame or curvature");
    const size_t count=knots.size(),unique=count-(closed?1:0);
    const auto equal=[](Vec3 a,Vec3 b){return a.x==b.x&&a.y==b.y&&a.z==b.z;};
    if(closed&&(!equal(knots.front().position,knots.back().position)||!equal(knots.front().tangent,knots.back().tangent)||!equal(knots.front().curvature,knots.back().curvature)||!equal(knots.front().up,knots.back().up)||knots.front().bank!=knots.back().bank))throw std::runtime_error("Canonical closed seam has inconsistent knot values");
    std::vector<Vec3> tangent(count),curvature(count),jerk(count);
    std::vector<double> metric(count-1);
    for(size_t i=0;i<count;++i){tangent[i]=unit(knots[i].tangent);curvature[i]=knots[i].curvature-tangent[i]*dot(tangent[i],knots[i].curvature);}
    for(size_t i=0;i+1<count;++i){double h=norm(knots[i+1].position-knots[i].position);if(!std::isfinite(h)||h<1e-5||h>100)throw std::runtime_error("Invalid track knot or spacing");metric[i]=h*(1+(dot(curvature[i],curvature[i])+dot(curvature[i+1],curvature[i+1]))*h*h/48);}
    // Shared geometric third derivative. The tangential component is fixed by
    // d(T dot K)/ds=0. One identical value is used on both sides of every join.
    for(size_t i=0;i<unique;++i){
        Vec3 change;
        if(!closed&&i==0)change=(curvature[1]-curvature[0])/metric[0];
        else if(!closed&&i+1==unique)change=(curvature[i]-curvature[i-1])/metric[i-1];
        else{size_t lo=i?i-1:unique-1,hi=(i+1)%unique;double dl=metric[lo],dr=metric[i];change=((curvature[i]-curvature[lo])*(dr/dl)+(curvature[hi]-curvature[i])*(dl/dr))/(dl+dr);}
        jerk[i]=change-tangent[i]*(dot(change,tangent[i])+dot(curvature[i],curvature[i]));
        if(!finite(jerk[i]))throw std::runtime_error("Invalid derived curvature derivative");
    }
    if(closed)jerk.back()=jerk.front();
    spans.clear();spans.reserve(count-1);length=0;
    for(size_t i=0;i+1<count;++i){
        const double h=metric[i],h2=h*h,h3=h2*h;
        Span sp;sp.start=length;sp.c[0]=knots[i].position;sp.c[1]=tangent[i]*h;sp.c[2]=curvature[i]*(h2*.5);sp.c[3]=jerk[i]*(h3/6);
        const Vec3 p=knots[i+1].position-sp.c[0]-sp.c[1]-sp.c[2]-sp.c[3];
        const Vec3 v=tangent[i+1]*h-sp.c[1]-sp.c[2]*2-sp.c[3]*3;
        const Vec3 a=curvature[i+1]*h2-sp.c[2]*2-sp.c[3]*6;
        const Vec3 j=jerk[i+1]*h3-sp.c[3]*6;
        sp.c[4]=p*35-v*15+a*2.5-j/6;sp.c[5]=p*(-84)+v*39-a*7+j*.5;
        sp.c[6]=p*70-v*34+a*6.5-j*.5;sp.c[7]=p*(-20)+v*10-a*2+j/6;
        // Degree-six derivative Bernstein controls enclose the entire span.
        std::array<Vec3,7> derivative{};Vec3 forward=unit(knots[i+1].position-knots[i].position);double minimumSpeed=1e100,secondBound=0;
        for(int b=0;b<7;++b){for(int k=0;k<=b;++k)derivative[b]=derivative[b]+sp.c[k+1]*((k+1)*binomial(b,k)/binomial(6,k));double speed=dot(derivative[b],forward);
            if(!finite(derivative[b])||speed<h*.5||speed<norm(derivative[b])*.95)throw std::runtime_error("Canonical span is outside the supported tangent cone");minimumSpeed=std::min(minimumSpeed,speed);}
        for(int b=0;b<6;++b)secondBound=std::max(secondBound,6*norm(derivative[b+1]-derivative[b]));
        if(secondBound/(minimumSpeed*minimumSpeed)>.2)throw std::runtime_error("Canonical span exceeds the interval curvature bound");
        sp.length=arc(sp,1);if(!std::isfinite(sp.length)||sp.length<1e-5||sp.length>200)throw std::runtime_error("Invalid canonical span length");length+=sp.length;spans.push_back(sp);
    }
    rebuildFramePolynomials(*this);
}
Track compile(const std::vector<AuthoredPoint>& p,bool closed){
    if(p.size()<4)throw std::runtime_error("Not enough track points");
    Track t;t.closed=closed;size_t n=p.size()-(closed?1:0);t.knots.resize(p.size());
    // Five-point derivatives of the authored curve avoid the curvature ripple
    // produced by second-order chord tangents in a high-order interpolant.
    std::vector<double> chord(n+1);
    for(size_t i=1;i<=n;++i)chord[i]=chord[i-1]+norm(p[i%n].position-p[(i-1)%n].position);
    for(size_t i=0;i<n;++i){
        int count=int(std::min<size_t>(5,n)),first=int(i)-count/2;
        if(!closed)first=std::clamp(first,0,int(n)-count);
        std::array<int,5> index{};std::array<double,5> xs{};
        double scale=0;
        for(int j=0;j<count;++j){int k=first+j,wrapped=(k%int(n)+int(n))%int(n);index[j]=wrapped;
            int lap=k<0?-1:k>=int(n)?1:0;xs[j]=chord[wrapped]+lap*chord[n]-chord[i];scale=std::max(scale,std::abs(xs[j]));}
        if(scale<1e-8)throw std::runtime_error("Degenerate authored points");
        double matrix[5][7]{};
        for(int row=0;row<count;++row){for(int j=0;j<count;++j)matrix[row][j]=std::pow(xs[j]/scale,row);matrix[row][count]=row==1?1:0;matrix[row][count+1]=row==2?2:0;}
        for(int col=0;col<count;++col){int pivot=col;for(int row=col+1;row<count;++row)if(std::abs(matrix[row][col])>std::abs(matrix[pivot][col]))pivot=row;
            for(int k=col;k<count+2;++k)std::swap(matrix[col][k],matrix[pivot][k]);double divisor=matrix[col][col];if(std::abs(divisor)<1e-12)throw std::runtime_error("Repeated authored points");
            for(int k=col;k<count+2;++k)matrix[col][k]/=divisor;
            for(int row=0;row<count;++row)if(row!=col){double factor=matrix[row][col];for(int k=col;k<count+2;++k)matrix[row][k]-=factor*matrix[col][k];}}
        Vec3 d1{},d2{};for(int j=0;j<count;++j){Vec3 delta=p[index[j]].position-p[i].position;d1=d1+delta*(matrix[j][count]/scale);d2=d2+delta*(matrix[j][count+1]/(scale*scale));}
        Vec3 tangent=unit(d1),curvature=(d2-tangent*dot(tangent,d2))/dot(d1,d1);
        t.knots[i]={p[i].position,tangent,curvature,{},p[i].bank,p[i].element};
    }
    Vec3 up=unit(Vec3{0,0,1}-t.knots[0].tangent*t.knots[0].tangent.z);
    t.knots[0].up=up;
    for(size_t i=1;i<n;++i)t.knots[i].up=up=transport(up,t.knots[i-1].tangent,t.knots[i].tangent);
    bool hinted=std::all_of(p.begin(),p.end(),[](const AuthoredPoint& q){return norm(q.upHint)>.5;});
    if(hinted)for(size_t i=0;i<n;++i)t.knots[i].up=unit(p[i].upHint-t.knots[i].tangent*dot(p[i].upHint,t.knots[i].tangent));
    if(closed){
        Vec3 end=transport(up,t.knots[n-1].tangent,t.knots[0].tangent),start=t.knots[0].up;
        double residual=std::atan2(dot(cross(end,start),t.knots[0].tangent),dot(end,start));
        if(!hinted)for(size_t i=0;i<n;++i)t.knots[i].up=rotate(t.knots[i].up,t.knots[i].tangent,residual*double(i)/n);
        t.knots[n]=t.knots[0];
    }
    t.rebuild();return t;
}
TrackLocation Track::locate(double distance) const {
    size_t hint=spans.size();return locate(distance,hint);
}
TrackLocation Track::locate(double distance,size_t& hint) const {
    if(spans.empty()||!std::isfinite(distance)||!std::isfinite(length)||length<=0)throw std::runtime_error("Invalid track distance or empty track");
    if(closed){if(distance<0||distance>=length){distance=std::fmod(distance,length);if(distance<0)distance+=length;}}else distance=std::clamp(distance,0.,length);
    auto contains=[&](size_t i){return i<spans.size()&&distance>=spans[i].start&&(i+1==spans.size()||distance<spans[i+1].start);};
    if(!contains(hint)){
        if(hint<spans.size()&&contains(hint+1))++hint;
        else{auto it=std::upper_bound(spans.begin(),spans.end(),distance,[](double s,const Span& sp){return s<sp.start;});hint=it==spans.begin()?0:size_t(it-spans.begin()-1);}
    }
    const size_t i=hint;const auto& sp=spans[i];
    const double local=std::clamp(distance-sp.start,0.,sp.length);
    if(local==0)return {i,0};if(local==sp.length)return {i,1};
    double lo=0,hi=1,u=local/sp.length;
    // Converged, safeguarded inversion shared by rendering and rider forces.
    for(int k=0;k<16;++k){double residual=arc(sp,u)-local;if(std::abs(residual)<=2e-13*std::max(1.,sp.length))return {i,u};if(residual>0)hi=u;else lo=u;double next=u-residual/norm(der(sp,u));u=std::isfinite(next)&&next>lo&&next<hi?next:(lo+hi)*.5;}
    if(std::abs(arc(sp,u)-local)>2e-11*std::max(1.,sp.length))throw std::runtime_error("Canonical arc inversion did not converge");
    return {i,u};
}
TrackSample Track::sample(double distance) const {auto at=locate(distance);return sampleSpan(at.span,at.parameter);}
double Track::distanceAtSpan(size_t i,double u) const {
    if(i>=spans.size()||!std::isfinite(u)||u<0||u>1)throw std::runtime_error("Invalid canonical parameter");
    return spans[i].start+arc(spans[i],u);
}

static double segmentDistance(Vec3 p,Vec3 q,Vec3 a,Vec3 b){
    Vec3 d1=q-p,d2=b-a,r=p-a;double aa=dot(d1,d1),ee=dot(d2,d2),ff=dot(d2,r),s=0,t=0;
    if(aa<=1e-12&&ee<=1e-12)return norm(r);
    if(aa<=1e-12)t=std::clamp(ff/ee,0.,1.);else{double cc=dot(d1,r);if(ee<=1e-12)s=std::clamp(-cc/aa,0.,1.);else{double bb=dot(d1,d2),den=aa*ee-bb*bb;if(den>1e-12)s=std::clamp((bb*ff-cc*ee)/den,0.,1.);t=(bb*s+ff)/ee;if(t<0){t=0;s=std::clamp(-cc/aa,0.,1.);}else if(t>1){t=1;s=std::clamp((bb-cc)/aa,0.,1.);}}}return norm(r+d1*s-d2*t);
}
namespace terrain_validation {
// Internal kernel exposed only to the focused validation tests. q comes from a
// checked canonical sweep; trainTop is the supported headroom, never a render guess.
double lowerBound(const TrackSample& q,const Terrain& terrain,double trainTop,double motionPadding){
    constexpr double halfLength=1.275,halfWidth=1.5,bottom=-.8;
    const double middle=(bottom+trainTop)*.5,halfHeight=(trainTop-bottom)*.5;
    const Vec3 centre=q.position+q.up*middle;
    double footprintRadius=0;
    for(double x:{-halfLength,halfLength})for(double y:{-halfWidth,halfWidth})for(double z:{-halfHeight,halfHeight}){const Vec3 delta=q.tangent*x+q.right*y+q.up*z;footprintRadius=std::max(footprintRadius,std::hypot(delta.x,delta.y));}
    const double slope=terrain.localSlopeBound(centre.x,centre.y,footprintRadius+motionPadding);
    const double ground=terrain.height(centre.x,centre.y);
    double bound=std::numeric_limits<double>::infinity();
    if(slope==0){
        bound=centre.z-ground-std::abs(q.tangent.z)*halfLength-std::abs(q.right.z)*halfWidth-std::abs(q.up.z)*halfHeight;
    }else{
        // H(x,y)<=H(centreXY)+L*horizontalDistance. The resulting clearance
        // lower-bound function z-H(centreXY)-L*norm(xy-centreXY) is concave,
        // so its minimum over the full convex OBB occurs at a vertex. This
        // certifies its interior too, unlike terrain queries at corners alone.
        for(double x:{-halfLength,halfLength})for(double y:{-halfWidth,halfWidth})for(double z:{-halfHeight,halfHeight}){
            Vec3 delta=q.tangent*x+q.right*y+q.up*z;
            bound=std::min(bound,centre.z+delta.z-ground-slope*std::hypot(delta.x,delta.y));
        }
    }
    // Clearance z-H(x,y) is sqrt(1+L^2)-Lipschitz in 3D. Every body point
    // moves <.188 m from its exact midpoint pose under the shared sweep proof;
    // .20 m is the unchanged conservative Euclidean motion reserve.
    return bound-motionPadding*std::hypot(1.,slope);
}
}
namespace chord_validation {
// Integrate each already-certified M du bound between the ACTUAL parameters
// used by Track.sample. One cursor crosses each cell at most once: O(cells+chords).
std::vector<double> arcBounds(const Track& track,const ClearanceSweep& sweep,int count,Cancel cancel){
    if(count<=0)throw std::runtime_error("Invalid chord count");
    const auto& cells=sweep.frames();if(cells.empty())throw std::runtime_error("Empty canonical sweep");
    std::vector<double> bounds;bounds.reserve(size_t(count));size_t cursor=0,visits=0;double parameter=cells.front().parameterBegin;
    for(int i=1;i<=count;++i){
        if(cancel&&cancel())throw std::runtime_error("CANCELLED");
        const auto end=i==count?TrackLocation{track.spans.size()-1,1}:track.locate(track.length*i/count);
        double bound=0;
        while(cursor<cells.size()){
            if((visits++&127)==0&&cancel&&cancel())throw std::runtime_error("CANCELLED");
            const auto& cell=cells[cursor];
            if(cell.span>end.span||(cell.span==end.span&&parameter>=end.parameter))break;
            const double upper=cell.span==end.span?std::min(cell.parameterEnd,end.parameter):cell.parameterEnd;
            if(upper<parameter)throw std::runtime_error("Nonmonotonic canonical chord parameter");
            bound+=cell.arcLengthBound*((upper-parameter)/(cell.parameterEnd-cell.parameterBegin));
            parameter=upper;
            if(parameter==cell.parameterEnd){++cursor;if(cursor<cells.size())parameter=cells[cursor].parameterBegin;}else break;
        }
        // Reserve summation/interpolation roundoff without relying on subtracting
        // two large cumulative integrals at nearby parameters.
        bounds.push_back(bound*(1+1e-10)+1e-9);
    }
    return bounds;
}
ValidationReport validate(const Track& track,const ClearanceSweep& sweep,int count,Cancel cancel){
    ValidationReport out;
    try{const auto bounds=arcBounds(track,sweep,count,cancel);for(size_t i=0;i<bounds.size();++i){
        if(!std::isfinite(bounds[i])||bounds[i]>2.1){out.fail("TRACK_SAMPLING_DOMAIN","Cannot certify true arc length of the central clearance chord",track.length*i/count,bounds[i],2.1);return out;}
    }}catch(const std::exception& e){out.fail(std::string(e.what())=="CANCELLED"?"CANCELLED":"TRACK_SAMPLING_DOMAIN",e.what());}
    return out;
}
}
ValidationReport validateGeometry(const Track& t,const Terrain& terrain,const Limits& limits,const TrainConfig& train,const std::vector<Support>& supports,Cancel cancel){
    ValidationReport r;if(train.cars<1||train.cars>16||!std::isfinite(train.spacing)||train.spacing<=0||train.spacing>20){r.fail("TRAIN_CONFIG","Invalid train geometry settings");return r;}if(t.spans.empty()){r.fail("EMPTY_TRACK","Track is empty");return r;}
    if(t.knots.size()!=t.spans.size()+1||t.knots.size()<4||!std::isfinite(t.length)||t.length<=0){r.fail("GEOMETRY_DOMAIN","Invalid canonical track cardinality or length");return r;}
    if(!std::isfinite(limits.minClearance)||limits.minClearance<0||!terrain.valid()){r.fail("TERRAIN_CONFIG","Invalid terrain or configured minimum clearance");return r;}
    if(!t.closed){r.fail("OPEN_CIRCUIT","Generation requires a closed circuit");return r;}
    const auto &a=t.knots.front(),&b=t.knots.back();
    if(norm(a.position-b.position)>.001)r.fail("SEAM_POSITION","Station seam does not close");
    if(dot(a.tangent,b.tangent)<std::cos(.05*pi/180)||dot(a.up,b.up)<std::cos(.05*pi/180))r.fail("SEAM_FRAME","Station seam orientation is discontinuous");
    if(norm(a.curvature-b.curvature)>1e-6||std::abs(a.bank-b.bank)>1e-6)r.fail("SEAM_CURVATURE","Station seam curvature or bank differs");
    for(size_t i=0;i<t.spans.size();++i){
        const auto& k=t.knots[i];const auto& next=t.knots[i+1];double ds=t.spans[i].length;
        double bankRate=std::abs(next.bank-k.bank)/ds;
        double angle=std::atan2(norm(cross(unit(k.up),unit(next.up))),std::clamp(dot(unit(k.up),unit(next.up)),-1.,1.));
        double frameRate=angle/ds;
        if(bankRate>.12||frameRate>.12||angle>.2){r.fail("FRAME_RATE","Canonical orientation changes faster than the supported spatial resolution",t.spans[i].start,std::max(bankRate,frameRate),.12);return r;}
    }
    std::optional<ClearanceSweep> sweep;
    try{sweep.emplace(buildClearanceSweep(t,train,cancel));}
    catch(const std::exception& e){r.fail((std::string(e.what())=="CANCELLED"||(cancel&&cancel()))?"CANCELLED":"SWEEP_DOMAIN",e.what());return r;}
    TrackSample previous=t.sample(0);
    for(double s=.5;s<t.length;s+=.5){
        if(cancel&&cancel()){r.fail("CANCELLED","Frame validation cancelled");return r;}
        auto current=t.sample(s);double rate=std::acos(std::clamp(dot(previous.up,current.up),-1.,1.))/.5;
        if(!finite(current.curvature)||norm(current.curvature)>.2||rate>.25){r.fail("FRAME_DOMAIN","Canonical frame or curvature exceeds the supported spatial domain",s,std::max(rate,norm(current.curvature)),.25);return r;}
        previous=current;
    }
    // Central-chord model for nonadjacent branches;12 m wrap adjacency denotes
    // the same local rail. It is not an exemption for supports or station parts.
    constexpr double step=2,cell=16;std::vector<Vec3> p;std::vector<double> ds;
    int count=int(std::ceil(t.length/step));
    auto chordReport=chord_validation::validate(t,*sweep,count,cancel);if(!chordReport.valid())return chordReport;
    // True arc per chord <=2.1 m and continuous curvature <=.2 imply deviation
    // <=.2*2.1^2/8=.11025 m from the straight chord. Against the unchanged6 m
    // test, body radius4.2 + hardware radius.9 + two deviations leave>.679 m.
    p.reserve(count+1);ds.reserve(count+1);
    for(int i=0;i<=count;++i){if((i&255)==0&&cancel&&cancel()){r.fail("CANCELLED","Geometry validation cancelled");return r;}double s=t.length*i/count;auto q=t.sample(s);p.push_back(q.position);ds.push_back(s);
        for(double side:{-1.5,1.5})for(double height:{-.8,2.4}){Vec3 e=q.position+q.right*side+q.up*height;double clear=e.z-terrain.height(e.x,e.y);if(clear<limits.minClearance+1.6&&r.errors.size()<10)r.fail("TERRAIN_CLEARANCE","Train envelope intersects terrain clearance",s,clear,limits.minClearance);}
    }
    struct Key {int x,y,z;bool operator==(const Key&) const=default;};struct Hash{size_t operator()(Key k)const{return uint64_t(k.x)*73856093ull^uint64_t(k.y)*19349663ull^uint64_t(k.z)*83492791ull;}};
    std::unordered_map<Key,std::vector<int>,Hash> grid;
    for(int i=0;i<count;++i){if((i&255)==0&&cancel&&cancel()){r.fail("CANCELLED","Geometry validation cancelled");return r;}Vec3 m=(p[i]+p[i+1])*.5;Key k{int(std::floor(m.x/cell)),int(std::floor(m.y/cell)),int(std::floor(m.z/cell))};
        for(int x=-1;x<=1;++x)for(int y=-1;y<=1;++y)for(int z=-1;z<=1;++z){auto it=grid.find({k.x+x,k.y+y,k.z+z});if(it==grid.end())continue;for(int j:it->second){double sep=std::abs(ds[i]-ds[j]);sep=std::min(sep,t.length-sep);if(sep<12)continue;double d=segmentDistance(p[i],p[i+1],p[j],p[j+1]);if(d<6&&r.errors.size()<10)r.fail("TRACK_CLEARANCE","Nonadjacent central clearance chords are closer than the required distance; other track distance="+std::to_string(ds[j])+" m",ds[i],d,6);}}
        grid[k].push_back(i);
    }
    // Retain the original 2 m corner gate and its +1.6 m reserve above. This
    // additional certificate covers every point of the full moving body and
    // compares its conservative lower bound to the configured minClearance.
    // Reuse this same prepared sweep for all support-member checks below.
    size_t terrainFrame=0;
    for(const auto& f:sweep->frames()){
        if((terrainFrame++&127)==0&&cancel&&cancel()){r.fail("CANCELLED","Swept terrain validation cancelled");return r;}
        double lower=terrain_validation::lowerBound(f.sample,terrain,sweep->trainTop(),sweep->padding());
        if(!std::isfinite(lower)||lower<limits.minClearance){
            r.fail("TERRAIN_SWEEP_CLEARANCE","Cannot certify configured terrain clearance for the complete swept train body",f.distance,lower,limits.minClearance);
            if(r.errors.size()>=10)return r;
        }
    }
    size_t totalMembers=0;
    for(const auto& support:supports){
        if(cancel&&cancel()){r.fail("CANCELLED","Support validation cancelled");return r;}
        if(!finite(support.base)||!finite(support.top)||norm(support.base)>1000000||norm(support.top)>1000000||norm(support.top-support.base)>1000||support.top.z<=support.base.z||std::abs(support.base.z-terrain.height(support.base.x,support.base.y))>.1){r.fail("SUPPORT_CONFIG","Support endpoints are invalid or its base is not on the terrain");continue;}
        if(!support.hasAttachment)r.fail("SUPPORT_ATTACHMENT","Support lacks its canonical spine attachment");
        if(support.hasAttachment){
            if(!finite(support.attachment)||!std::isfinite(support.trackDistance)||support.trackDistance<0||support.trackDistance>=t.length||norm(support.attachment-support.top)>100){r.fail("SUPPORT_CONFIG","Invalid support attachment");continue;}
            auto q=t.sample(support.trackDistance);
            if(norm(support.attachment-(q.position-q.up*(spineDepth+spineRadius)))>1e-4){r.fail("SUPPORT_ATTACHMENT","Support arm does not meet its canonical spine contact",support.trackDistance);continue;}
        }
        if(support.members.size()>maxTotalSupportMembers-totalMembers){r.fail("SUPPORT_MEMBER_BUDGET","Total canonical support member budget exceeded");return r;}
        totalMembers+=support.members.size();
        auto memberReport=validateSupportMembers(support,terrain,cancel);
        if(!memberReport.valid()){r.errors.insert(r.errors.end(),memberReport.errors.begin(),memberReport.errors.end());return r;}
        int hit=supportCollision(support,*sweep,cancel);
        if(hit==-2){r.fail("CANCELLED","Member collision validation cancelled");return r;}
        if(hit>=0&&r.errors.size()<10)r.fail("SUPPORT_CLEARANCE","Cannot certify support clearance from train or track hardware",sweep->frames()[hit].distance);
    }
    return r;
}
}
