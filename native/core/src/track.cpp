#include "coaster/coaster.hpp"
#include "coaster/clearance.hpp"
#include "arc_length.hpp"
#include <stdexcept>
#include <unordered_map>
#include <optional>

namespace coaster {
static Vec3 transport(Vec3 up,Vec3 a,Vec3 b){Vec3 c=cross(a,b);double s=norm(c);if(s>1e-10)up=rotate(up,c/s,std::atan2(s,dot(a,b)));return unit(up-b*dot(up,b));}
static Vec3 der(const Span& sp,double u){Vec3 v=sp.c.back()*double(sp.c.size()-1);for(int i=int(sp.c.size())-2;i>=1;--i)v=v*u+sp.c[i]*i;return v;}
static double arc(const Span& sp,double u){return detail::cachedArcLength(sp,u);}
static double binomial(int n,int k){double result=1;for(int i=1;i<=k;++i)result=result*(n-i+1)/i;return result;}
void Track::rebuild(){
    if(knots.size()<4||knots.size()>200000)throw std::runtime_error("Invalid knot count");
    for(const auto& k:knots)if(!finite(k.position)||!finite(k.tangent)||!finite(k.curvature)||!finite(k.up)||int(k.element)<0||int(k.element)>7||!std::isfinite(k.bank)||std::abs(k.bank)>64*pi||norm(k.position)>1000000||std::abs(norm(k.tangent)-1)>1e-5||std::abs(norm(k.up)-1)>1e-5||std::abs(dot(k.tangent,k.up))>1e-5||std::abs(dot(k.tangent,k.curvature))>1e-5||norm(k.curvature)>2)throw std::runtime_error("Invalid canonical frame or curvature");
    const size_t count=knots.size(),unique=count-(closed?1:0);
    const auto equal=[](Vec3 a,Vec3 b){return a.x==b.x&&a.y==b.y&&a.z==b.z;};
    if(closed&&(!equal(knots.front().position,knots.back().position)||!equal(knots.front().tangent,knots.back().tangent)||!equal(knots.front().curvature,knots.back().curvature)||!equal(knots.front().up,knots.back().up)||knots.front().bank!=knots.back().bank))throw std::runtime_error("Canonical closed seam has inconsistent knot values");
    std::vector<Vec3> tangent(count),curvature(count),jerk(count),snap(count);
    std::vector<double> metric(count-1);
    for(size_t i=0;i<count;++i){tangent[i]=unit(knots[i].tangent);curvature[i]=knots[i].curvature-tangent[i]*dot(tangent[i],knots[i].curvature);}
    for(size_t i=0;i+1<count;++i){double h=norm(knots[i+1].position-knots[i].position);if(!std::isfinite(h)||h<1e-5||h>100)throw std::runtime_error("Invalid track knot or spacing");metric[i]=h*(1+(dot(curvature[i],curvature[i])+dot(curvature[i+1],curvature[i+1]))*h*h/48);}
    std::vector<Vec3> curvatureFirst(count),curvatureSecond(count);
    if(authoredGeometry) {
        for(size_t i=0;i<count;++i) {
            const auto& k=knots[i];
            if(!finite(k.third)||!finite(k.fourth)||std::abs(dot(k.tangent,k.third)+dot(k.curvature,k.curvature))>1e-6||std::abs(dot(k.tangent,k.fourth)+3*dot(k.curvature,k.third))>1e-6)
                throw std::runtime_error("Invalid authored arc-length derivatives");
            curvatureFirst[i]=k.third;curvatureSecond[i]=k.fourth;
        }
    } else {
        std::vector<double> coordinate(count);for(size_t i=1;i<count;++i)coordinate[i]=coordinate[i-1]+metric[i-1];
        for(size_t i=0;i<unique;++i){
            const int n=int(std::min<size_t>(7,unique));int first=int(i)-n/2;if(!closed)first=std::clamp(first,0,int(unique)-n);
            std::array<size_t,7> index{};std::array<double,7> x{};double scale=0;
            for(int j=0;j<n;++j){const int at=first+j,wrapped=(at%int(unique)+int(unique))%int(unique),lap=at<0?-1:at>=int(unique)?1:0;index[j]=size_t(wrapped);x[j]=coordinate[wrapped]+lap*coordinate.back()-coordinate[i];scale=std::max(scale,std::abs(x[j]));}
            double matrix[7][9]{};for(int row=0;row<n;++row){for(int j=0;j<n;++j)matrix[row][j]=std::pow(x[j]/scale,row);matrix[row][n]=row==1?1:0;matrix[row][n+1]=row==2?2:0;}
            for(int col=0;col<n;++col){int pivot=col;for(int row=col+1;row<n;++row)if(std::abs(matrix[row][col])>std::abs(matrix[pivot][col]))pivot=row;for(int j=col;j<n+2;++j)std::swap(matrix[col][j],matrix[pivot][j]);const double divisor=matrix[col][col];if(std::abs(divisor)<1e-12)throw std::runtime_error("Unresolved curvature derivative stencil");for(int j=col;j<n+2;++j)matrix[col][j]/=divisor;for(int row=0;row<n;++row)if(row!=col){const double factor=matrix[row][col];for(int j=col;j<n+2;++j)matrix[row][j]-=factor*matrix[col][j];}}
            for(int j=0;j<n;++j){const Vec3 delta=curvature[index[j]]-curvature[i];curvatureFirst[i]=curvatureFirst[i]+delta*(matrix[j][n]/scale);curvatureSecond[i]=curvatureSecond[i]+delta*(matrix[j][n+1]/(scale*scale));}
        }
    }
    // Shared geometric third derivative. The tangential component is fixed by
    // d(T dot K)/ds=0. One identical value is used on both sides of every join.
    for(size_t i=0;i<unique;++i){
        const Vec3 change=curvatureFirst[i];
        jerk[i]=authoredGeometry?change:change-tangent[i]*(dot(change,tangent[i])+dot(curvature[i],curvature[i]));
        if(!finite(jerk[i]))throw std::runtime_error("Invalid derived curvature derivative");
    }
    if(closed)jerk.back()=jerk.front();
    {
        // A C3 rider frame needs a C3 tangent, hence a C4 centreline. Keep the
        // original position/tangent/curvature/jerk ports and share the next jet.
        for(size_t i=0;i<unique;++i){
            const Vec3 change=curvatureSecond[i];
            snap[i]=authoredGeometry?change:change-tangent[i]*(dot(change,tangent[i])+3*dot(curvature[i],jerk[i]));
            if(!finite(snap[i]))throw std::runtime_error("Invalid derived fourth position derivative");
        }
        if(closed)snap.back()=snap.front();
    }
    for(size_t i=0;i<count;++i){knots[i].third=jerk[i];knots[i].fourth=snap[i];}
    spans.clear();spans.reserve(count-1);length=0;
    for(size_t i=0;i+1<count;++i){
        double h=metric[i];Vec3 displacement=knots[i+1].position-knots[i].position;
        if(authoredGeometry){
            // Integrating the endpoint tangent jets gives a local displacement
            // without subtracting large world positions. Only use it when the
            // discrepancy is unresolved at the precision of those positions.
            // Otherwise retain the authored displacement exactly.
            const Vec3 t=(tangent[i]+tangent[i+1])*.5,k=(curvature[i]-curvature[i+1])*(3./28),
                j=(jerk[i]+jerk[i+1])/84,s=(snap[i]-snap[i+1])/1680;
            const Vec3 along=unit(displacement);const double chord=norm(displacement);
            double local=h;
            for(int iteration=0;iteration<3;++iteration){
                const Vec3 value=(t+(k+(j+s*local)*local)*local)*local;
                const Vec3 slope=t+(k*2+(j*3+s*(4*local))*local)*local;
                local-=(dot(value,along)-chord)/dot(slope,along);
            }
            const Vec3 resolved=(t+(k+(j+s*local)*local)*local)*local;
            const double roundoff=8*std::numeric_limits<double>::epsilon()*std::max({1.,norm(knots[i].position),norm(knots[i+1].position)});
            if(std::abs(local-h)<roundoff+1e-5*h&&norm(resolved-displacement)<=roundoff){h=local;displacement=resolved;}
        }
        const double h2=h*h,h3=h2*h;
        Span sp;sp.start=length;sp.c[0]=knots[i].position;sp.c[1]=tangent[i]*h;sp.c[2]=curvature[i]*(h2*.5);sp.c[3]=jerk[i]*(h3/6);
        const Vec3 p=displacement-sp.c[1]-sp.c[2]-sp.c[3];
        const Vec3 v=tangent[i+1]*h-sp.c[1]-sp.c[2]*2-sp.c[3]*3;
        const Vec3 a=curvature[i+1]*h2-sp.c[2]*2-sp.c[3]*6;
        const Vec3 j=jerk[i+1]*h3-sp.c[3]*6;
        sp.c[4]=p*35-v*15+a*2.5-j/6;sp.c[5]=p*(-84)+v*39-a*7+j*.5;
        sp.c[6]=p*70-v*34+a*6.5-j*.5;sp.c[7]=p*(-20)+v*10-a*2+j/6;
        {
            Vec3 endFourth{};for(int k=4;k<=7;++k)endFourth=endFourth+sp.c[k]*double(k*(k-1)*(k-2)*(k-3));
            const Vec3 left=(snap[i]*(h2*h2)-sp.c[4]*24)/24;
            const Vec3 slope=(snap[i+1]*(h2*h2)-endFourth)/24-left;
            // u^4(1-u)^4(left+slope*u) changes only the fourth endpoint jets.
            sp.c[4]=sp.c[4]+left;sp.c[5]=sp.c[5]+slope-left*4;
            sp.c[6]=sp.c[6]+left*6-slope*4;sp.c[7]=sp.c[7]-left*4+slope*6;
            sp.c[8]=left-slope*4;sp.c[9]=slope;
        }
        // Bernstein derivative controls enclose the entire canonical span.
        std::array<Vec3,9> derivative{};Vec3 forward=unit(knots[i+1].position-knots[i].position);double minimumSpeed=1e100,secondBound=0;
        for(int b=0;b<9;++b){for(int k=0;k<=b;++k)derivative[b]=derivative[b]+sp.c[k+1]*((k+1)*binomial(b,k)/binomial(8,k));double speed=dot(derivative[b],forward);
            if(!finite(derivative[b])||speed<h*.5||speed<norm(derivative[b])*.95)throw std::runtime_error("Canonical span is outside the supported tangent cone");minimumSpeed=std::min(minimumSpeed,speed);}
        for(int b=0;b<8;++b)secondBound=std::max(secondBound,8*norm(derivative[b+1]-derivative[b]));
        if(secondBound/(minimumSpeed*minimumSpeed)>.2)throw std::runtime_error("Canonical span exceeds the interval curvature bound");
        sp.length=detail::spanArcLength(sp,1);if(!std::isfinite(sp.length)||sp.length<1e-5||sp.length>200)throw std::runtime_error("Invalid canonical span length");
        detail::prepareArcPolynomial(sp,derivative);length+=sp.length;spans.push_back(sp);
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
double boxLowerBound(const StationBox& box,const Terrain& terrain,double motionPadding){
    const Vec3 centre=box.center;const double halfLength=box.half.x,halfWidth=box.half.y,halfHeight=box.half.z;
    if(terrain.kind==TerrainKind::Flat)
        return centre.z-std::abs(box.forward.z)*halfLength-std::abs(box.right.z)*halfWidth-std::abs(box.up.z)*halfHeight-motionPadding;
    // Expand each local box axis by the Euclidean sweep reserve. This contains
    // every moving point without adding a world-space ground-height margin.
    std::array<Vec3,8> corners;double minX=INFINITY,minY=INFINITY,maxX=-INFINITY,maxY=-INFINITY;
    for(int i=0;i<8;++i){corners[i]=centre+box.forward*((i&1?1.:-1.)*(halfLength+motionPadding))+box.right*((i&2?1.:-1.)*(halfWidth+motionPadding))+box.up*((i&4?1.:-1.)*(halfHeight+motionPadding));
        minX=std::min(minX,corners[i].x);maxX=std::max(maxX,corners[i].x);minY=std::min(minY,corners[i].y);maxY=std::max(maxY,corners[i].y);}
    constexpr int faces[6][4]={{0,1,3,2},{4,5,7,6},{0,1,5,4},{2,3,7,6},{0,2,6,4},{1,3,7,5}};
    double bound=INFINITY;const double step=Terrain::gridStep;
    for(double x=std::floor(minX/step)*step;x<=maxX;x+=step)for(double y=std::floor(minY/step)*step;y<=maxY;y+=step){
        const Vec3 a{x,y,terrain.vertexHeight(x,y)},b{x+step,y,terrain.vertexHeight(x+step,y)},c{x+step,y+step,terrain.vertexHeight(x+step,y+step)},e{x,y+step,terrain.vertexHeight(x,y+step)};
        for(const std::array<Vec3,3>& triangle:{std::array<Vec3,3>{a,b,c},std::array<Vec3,3>{a,c,e}}){
            const Vec3 normal=cross(triangle[1]-triangle[0],triangle[2]-triangle[0]);
            for(const auto& face:faces){std::array<Vec3,12> polygon{},next{};int count=4;for(int j=0;j<4;++j)polygon[j]=corners[face[j]];
                for(int edge=0;edge<3&&count;++edge){const Vec3 from=triangle[edge],line=triangle[(edge+1)%3]-from;int size=0;
                    auto side=[&](Vec3 p){return line.x*(p.y-from.y)-line.y*(p.x-from.x);};
                    for(int j=0;j<count;++j){const Vec3 u=polygon[j],v=polygon[(j+1)%count];const double du=side(u),dv=side(v);
                        if(du>=0)next[size++]=u;if((du<0)!=(dv<0))next[size++]=u+(v-u)*(du/(du-dv));}
                    polygon=next;count=size;
                }
                // Vertical separation from the affine terrain face is linear;
                // its minimum on the clipped convex face occurs at a vertex.
                for(int j=0;j<count;++j)bound=std::min(bound,dot(polygon[j]-triangle[0],normal)/normal.z);
            }
        }
    }
    return bound-1e-9;
}
double lowerBound(const TrackSample& q,const Terrain& terrain,double trainTop,double motionPadding){
    const double middle=(trainEnvelopeBottom+trainTop)*.5;
    const StationBox box{q.position+q.up*middle,q.tangent,q.right,q.up,{trainHalfLength,patronHalfWidth,(trainTop-trainEnvelopeBottom)*.5},StationRole::Post};
    return boxLowerBound(box,terrain,motionPadding);
}
}
void ClearanceSweep::prepareGround(const Terrain& terrain,Cancel cancel){
    std::vector<double> values;values.reserve(samples.size());
    for(const auto& cell:samples){if(cancel&&cancel())throw std::runtime_error("CANCELLED");values.push_back(terrain_validation::lowerBound(cell.sample,terrain,top,pad));}
    groundLowerBounds=std::move(values);sampledTerrain=terrain;
}
double minimumSweptGroundClearance(const Track& track,const Terrain& terrain,const TrainConfig& train,Cancel cancel){
    const auto sweep=buildClearanceSweep(track,train,cancel);return minimumSweptGroundClearance(sweep,terrain,cancel);
}
double minimumSweptGroundClearance(const ClearanceSweep& sweep,const Terrain& terrain,Cancel cancel){
    if(cancel&&cancel())throw std::runtime_error("CANCELLED");
    if(const auto* values=sweep.groundBounds(terrain);values&&!values->empty())return *std::min_element(values->begin(),values->end());
    double minimum=INFINITY;
    for(const auto& cell:sweep.frames()){
        if(cancel&&cancel())throw std::runtime_error("CANCELLED");
        minimum=std::min(minimum,terrain_validation::lowerBound(cell.sample,terrain,sweep.trainTop(),sweep.padding()));
    }
    return minimum;
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
static ValidationReport selfClearanceImpl(const Track& t,const ClearanceSweep& sweep,double minClearance,Cancel cancel){
    ValidationReport r;
    auto sampleSequential=[&](double distance,size_t& hint){const auto where=t.locate(distance,hint);return t.sampleSpan(where.span,where.parameter);};
    // Central-chord model for nonadjacent branches;12 m wrap adjacency denotes
    // the same local rail. It is not an exemption for supports or station parts.
    constexpr double step=2,cell=16;std::vector<Vec3> p;std::vector<double> ds;
    int count=int(std::ceil(t.length/step));
    auto chordReport=chord_validation::validate(t,sweep,count,cancel);if(!chordReport.valid())return chordReport;
    // A full rider-body bounding radius plus neighbouring hardware, both
    // chord deviations and the requested free clearance. The sweep enforces
    // the 4.2 m body-radius domain; support contacts use their separate model.
    const double branchClearance=sweep.bodyRadius()+.9+2*(.2*2.1*2.1/8)+minClearance;
    p.reserve(count+1);ds.reserve(count+1);size_t chordHint=t.spans.size();
    for(int i=0;i<=count;++i){if((i&255)==0&&cancel&&cancel()){r.fail("CANCELLED","Geometry validation cancelled");return r;}double s=t.length*i/count;auto q=sampleSequential(s,chordHint);p.push_back(q.position);ds.push_back(s);
    }
    struct Key {int x,y,z;bool operator==(const Key&) const=default;};struct Hash{size_t operator()(Key k)const{return uint64_t(k.x)*73856093ull^uint64_t(k.y)*19349663ull^uint64_t(k.z)*83492791ull;}};
    std::unordered_map<Key,std::vector<int>,Hash> grid;
    for(int i=0;i<count;++i){if((i&255)==0&&cancel&&cancel()){r.fail("CANCELLED","Geometry validation cancelled");return r;}Vec3 m=(p[i]+p[i+1])*.5;Key k{int(std::floor(m.x/cell)),int(std::floor(m.y/cell)),int(std::floor(m.z/cell))};
        for(int x=-1;x<=1;++x)for(int y=-1;y<=1;++y)for(int z=-1;z<=1;++z){auto it=grid.find({k.x+x,k.y+y,k.z+z});if(it==grid.end())continue;for(int j:it->second){double sep=std::abs(ds[i]-ds[j]);if(t.closed)sep=std::min(sep,t.length-sep);if(sep<12)continue;double d=segmentDistance(p[i],p[i+1],p[j],p[j+1]);if(d<branchClearance&&r.errors.size()<10)r.fail("TRACK_CLEARANCE","Nonadjacent central clearance chords are closer than the required distance; other track distance="+std::to_string(ds[j])+" m",ds[i],d,branchClearance);}}
        grid[k].push_back(i);
    }
    return r;
}
ValidationReport validateSelfClearance(const Track& t,const TrainConfig& train,double minClearance,Cancel cancel){
    ValidationReport r;
    if(train.cars<1||train.cars>16||!std::isfinite(train.spacing)||train.spacing<=0||train.spacing>20){r.fail("TRAIN_CONFIG","Invalid train geometry settings");return r;}
    if(t.spans.empty()){r.fail("EMPTY_TRACK","Track is empty");return r;}
    if(t.knots.size()!=t.spans.size()+1||t.knots.size()<4||!std::isfinite(t.length)||t.length<=0){r.fail("GEOMETRY_DOMAIN","Invalid canonical track cardinality or length");return r;}
    if(!std::isfinite(minClearance)||minClearance<0){r.fail("CLEARANCE_CONFIG","Invalid requested track clearance");return r;}
    try{const auto sweep=buildClearanceSweep(t,train,cancel);return selfClearanceImpl(t,sweep,minClearance,cancel);}
    catch(const std::exception& e){r.fail((std::string(e.what())=="CANCELLED"||(cancel&&cancel()))?"CANCELLED":"SWEEP_DOMAIN",e.what());return r;}
}
static ValidationReport validateGeometryImpl(const Track& t,const Terrain& terrain,const Limits& limits,const TrainConfig& train,const std::vector<Support>& supports,const ClearanceSweep* prepared,Cancel cancel){
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
    std::optional<ClearanceSweep> ownedSweep;
    const ClearanceSweep* sweep=prepared;
    if(!sweep){
        try{ownedSweep.emplace(buildClearanceSweep(t,train,cancel));sweep=&*ownedSweep;}
        catch(const std::exception& e){r.fail((std::string(e.what())=="CANCELLED"||(cancel&&cancel()))?"CANCELLED":"SWEEP_DOMAIN",e.what());return r;}
    }
    size_t frameHint=t.spans.size();
    auto sampleSequential=[&](double distance,size_t& hint){const auto where=t.locate(distance,hint);return t.sampleSpan(where.span,where.parameter);};
    TrackSample previous=sampleSequential(0,frameHint);
    for(double s=.5;s<t.length;s+=.5){
        if(cancel&&cancel()){r.fail("CANCELLED","Frame validation cancelled");return r;}
        auto current=sampleSequential(s,frameHint);double rate=std::acos(std::clamp(dot(previous.up,current.up),-1.,1.))/.5;
        if(!finite(current.curvature)||norm(current.curvature)>.2||rate>.25){r.fail("FRAME_DOMAIN","Canonical frame or curvature exceeds the supported spatial domain",s,std::max(rate,norm(current.curvature)),.25);return r;}
        previous=current;
    }
    auto branches=selfClearanceImpl(t,*sweep,limits.minClearance,cancel);
    if(std::any_of(branches.errors.begin(),branches.errors.end(),[](const auto& error){return error.code!="TRACK_CLEARANCE";}))return branches;
    r.errors.insert(r.errors.end(),branches.errors.begin(),branches.errors.end());
    // One ground-contact certificate covers every point of the complete
    // moving envelope. minClearance is an optional user separation outside
    // that envelope; no additional centreline-height gate is imposed.
    // Reuse this same prepared sweep for all support-member checks below.
    size_t terrainFrame=0;
    const auto* groundBounds=sweep->groundBounds(terrain);
    for(const auto& f:sweep->frames()){
        if((terrainFrame++&127)==0&&cancel&&cancel()){r.fail("CANCELLED","Swept terrain validation cancelled");return r;}
        double lower=groundBounds?(*groundBounds)[terrainFrame-1]:terrain_validation::lowerBound(f.sample,terrain,sweep->trainTop(),sweep->padding());
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
ValidationReport validateGeometry(const Track& t,const Terrain& terrain,const Limits& limits,const TrainConfig& train,const std::vector<Support>& supports,Cancel cancel){
    return validateGeometryImpl(t,terrain,limits,train,supports,nullptr,cancel);
}
ValidationReport validateGeometry(const Track& t,const Terrain& terrain,const Limits& limits,const TrainConfig& train,const std::vector<Support>& supports,const ClearanceSweep& sweep,Cancel cancel){
    return validateGeometryImpl(t,terrain,limits,train,supports,&sweep,cancel);
}
}
