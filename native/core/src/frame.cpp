#include "coaster/coaster.hpp"
#include <stdexcept>

namespace coaster {
namespace {
// Third-order jets differentiate the same canonical expression used to render
// the frame. No spatial averaging stencil or independent rider spline is used.
struct ScalarJet { double value{},first{},second{},third{}; };
struct VectorJet { Vec3 value,first,second,third; };
ScalarJet operator-(ScalarJet a,ScalarJet b){return {a.value-b.value,a.first-b.first,a.second-b.second,a.third-b.third};}
ScalarJet operator*(ScalarJet a,ScalarJet b){return {a.value*b.value,a.first*b.value+a.value*b.first,a.second*b.value+2*a.first*b.first+a.value*b.second,a.third*b.value+3*a.second*b.first+3*a.first*b.second+a.value*b.third};}
ScalarJet inverse(ScalarJet a){double r=1/a.value;return {r,-a.first*r*r,2*a.first*a.first*r*r*r-a.second*r*r,-6*a.first*a.first*a.first*r*r*r*r+6*a.first*a.second*r*r*r-a.third*r*r};}
ScalarJet squareRoot(ScalarJet a){double r=std::sqrt(a.value);return {r,a.first/(2*r),a.second/(2*r)-a.first*a.first/(4*r*r*r),a.third/(2*r)-3*a.first*a.second/(4*r*r*r)+3*a.first*a.first*a.first/(8*r*r*r*r*r)};}
VectorJet operator+(VectorJet a,VectorJet b){return {a.value+b.value,a.first+b.first,a.second+b.second,a.third+b.third};}
VectorJet operator-(VectorJet a,VectorJet b){return {a.value-b.value,a.first-b.first,a.second-b.second,a.third-b.third};}
VectorJet operator*(VectorJet a,ScalarJet b){return {a.value*b.value,a.first*b.value+a.value*b.first,a.second*b.value+a.first*(2*b.first)+a.value*b.second,a.third*b.value+a.second*(3*b.first)+a.first*(3*b.second)+a.value*b.third};}
ScalarJet jetDot(VectorJet a,VectorJet b){return {dot(a.value,b.value),dot(a.first,b.value)+dot(a.value,b.first),dot(a.second,b.value)+2*dot(a.first,b.first)+dot(a.value,b.second),dot(a.third,b.value)+3*dot(a.second,b.first)+3*dot(a.first,b.second)+dot(a.value,b.third)};}
VectorJet jetCross(VectorJet a,VectorJet b){return {cross(a.value,b.value),cross(a.first,b.value)+cross(a.value,b.first),cross(a.second,b.value)+cross(a.first,b.first)*2+cross(a.value,b.second),cross(a.third,b.value)+cross(a.second,b.first)*3+cross(a.first,b.second)*3+cross(a.value,b.third)};}
ScalarJet magnitude(VectorJet a){auto n=jetDot(a,a);if(!std::isfinite(n.value)||n.value<1e-20)throw std::runtime_error("Degenerate canonical frame");return squareRoot(n);}
VectorJet normalized(VectorJet a){return a*inverse(magnitude(a));}

template<class T,size_t N>T value(const std::array<T,N>& c,double u){T p=c.back();for(size_t i=N-1;i-->0;)p=p*u+c[i];return p;}
template<size_t N>VectorJet polynomialJet(const std::array<Vec3,N>& c,double u){VectorJet p{c.back(),{}, {},{}};for(size_t i=N-1;i-->0;){p.third=p.third*u+p.second*3;p.second=p.second*u+p.first*2;p.first=p.first*u+p.value;p.value=p.value*u+c[i];}return p;}
template<size_t N>ScalarJet polynomialJet(const std::array<double,N>& c,double u){ScalarJet p{c.back(),0,0,0};for(size_t i=N-1;i-->0;){p.third=p.third*u+3*p.second;p.second=p.second*u+2*p.first;p.first=p.first*u+p.value;p.value=p.value*u+c[i];}return p;}
VectorJet positionDerivative(const Span& span,double u){std::array<Vec3,9> c;for(size_t i=0;i<c.size();++i)c[i]=span.c[i+1]*double(i+1);return polynomialJet(c,u);}

template<class T>std::array<T,8> septic(T a,T b,T a1,T b1,T a2,T b2,T a3,T b3){
    T c2=a2*.5,c3=a3/6,p=b-a-a1-c2-c3,v=b1-a1-c2*2-c3*3,acc=b2-c2*2-c3*6,j=b3-c3*6;
    return {a,a1,c2,c3,p*35-v*15+acc*2.5-j/6,p*(-84)+v*39-acc*7+j*.5,p*70-v*34+acc*6.5-j*.5,p*(-20)+v*10-acc*2+j/6};
}
template<class T>std::vector<std::array<T,3>> sharedDerivatives(const Track& track,const std::vector<T>& values){
    const size_t unique=values.size()-(track.closed?1:0);std::vector<std::array<T,3>> out(values.size());
    std::vector<double> coordinate(values.size());for(size_t i=1;i<coordinate.size();++i)coordinate[i]=coordinate[i-1]+track.spans[i-1].length;
    for(size_t i=0;i<unique;++i){
        const int count=int(std::min<size_t>(7,unique));int first=int(i)-count/2;if(!track.closed)first=std::clamp(first,0,int(unique)-count);
        std::array<size_t,7> index{};std::array<double,7> xs{};double scale=0;
        for(int j=0;j<count;++j){int at=first+j,wrapped=(at%int(unique)+int(unique))%int(unique),lap=at<0?-1:at>=int(unique)?1:0;index[j]=size_t(wrapped);xs[j]=coordinate[wrapped]+lap*track.length-coordinate[i];scale=std::max(scale,std::abs(xs[j]));}
        if(!(scale>0))throw std::runtime_error("Degenerate frame derivative stencil");
        double matrix[7][10]{};for(int row=0;row<count;++row){for(int j=0;j<count;++j)matrix[row][j]=std::pow(xs[j]/scale,row);matrix[row][count]=row==1?1:0;matrix[row][count+1]=row==2?2:0;matrix[row][count+2]=row==3?6:0;}
        for(int col=0;col<count;++col){int pivot=col;for(int row=col+1;row<count;++row)if(std::abs(matrix[row][col])>std::abs(matrix[pivot][col]))pivot=row;
            for(int k=col;k<count+3;++k)std::swap(matrix[col][k],matrix[pivot][k]);double divisor=matrix[col][col];if(std::abs(divisor)<1e-12)throw std::runtime_error("Unresolved frame derivative stencil");
            for(int k=col;k<count+3;++k)matrix[col][k]/=divisor;for(int row=0;row<count;++row)if(row!=col){double factor=matrix[row][col];for(int k=col;k<count+3;++k)matrix[row][k]-=factor*matrix[col][k];}}
        T firstDerivative{},secondDerivative{},thirdDerivative{};for(int j=0;j<count;++j){T delta=values[index[j]]-values[i];firstDerivative=firstDerivative+delta*(matrix[j][count]/scale);secondDerivative=secondDerivative+delta*(matrix[j][count+1]/(scale*scale));thirdDerivative=thirdDerivative+delta*(matrix[j][count+2]/(scale*scale*scale));}out[i]={firstDerivative,secondDerivative,thirdDerivative};
    }
    if(track.closed)out.back()=out.front();return out;
}void checkParameter(const Track& track,size_t i,double u){if(i>=track.spans.size()||!std::isfinite(u)||u<0||u>1)throw std::runtime_error("Invalid canonical frame parameter");}
}

void rebuildFramePolynomials(Track& track){
    if(track.knots.size()<4||track.spans.size()+1!=track.knots.size())throw std::runtime_error("Frame cache requires rebuilt canonical spans");
    std::vector<Vec3> up;std::vector<double> bank;up.reserve(track.knots.size());bank.reserve(track.knots.size());
    for(const auto& k:track.knots){up.push_back(k.up);bank.push_back(k.bank);}
    std::vector<std::array<Vec3,3>> upD;
    std::vector<std::array<double,3>> bankD;
    if(track.authoredFrame){upD.resize(up.size());bankD.resize(bank.size());}
    else{upD=sharedDerivatives(track,up);bankD=sharedDerivatives(track,bank);}
    if(track.authoredFrame)for(size_t i=0;i<track.knots.size();++i) {
        const auto& k=track.knots[i];
        if(k.bank!=0||!finite(k.upFirst)||!finite(k.upSecond)||!finite(k.upThird))throw std::runtime_error("Invalid authored physical frame derivatives");
        upD[i]={k.upFirst,k.upSecond,k.upThird};
    }
    for(size_t i=0;i<track.spans.size();++i){
        auto& span=track.spans[i];auto a=positionDerivative(span,0),b=positionDerivative(span,1);
        double qa=norm(a.value),qb=norm(b.value);if(!(qa>0&&qb>0))throw std::runtime_error("Zero endpoint metric in frame cache");
        double qau=dot(a.value,a.first)/qa,qbu=dot(b.value,b.first)/qb;
        // Parameter jets use actual endpoint metric, not quadrature span length.
        // Shared arc derivatives therefore agree even on unequal curved spans.
        {
            const double qauu=(dot(a.first,a.first)+dot(a.value,a.second)-qau*qau)/qa;
            const double qbuu=(dot(b.first,b.first)+dot(b.value,b.second)-qbu*qbu)/qb;
            span.referenceUp=septic(up[i],up[i+1],upD[i][0]*qa,upD[i+1][0]*qb,
                upD[i][1]*(qa*qa)+upD[i][0]*qau,upD[i+1][1]*(qb*qb)+upD[i+1][0]*qbu,
                upD[i][2]*(qa*qa*qa)+upD[i][1]*(3*qa*qau)+upD[i][0]*qauu,
                upD[i+1][2]*(qb*qb*qb)+upD[i+1][1]*(3*qb*qbu)+upD[i+1][0]*qbuu);
            span.bank=septic(bank[i],bank[i+1],bankD[i][0]*qa,bankD[i+1][0]*qb,
                bankD[i][1]*(qa*qa)+bankD[i][0]*qau,bankD[i+1][1]*(qb*qb)+bankD[i+1][0]*qbu,
                bankD[i][2]*(qa*qa*qa)+bankD[i][1]*(3*qa*qau)+bankD[i][0]*qauu,
                bankD[i+1][2]*(qb*qb*qb)+bankD[i+1][1]*(3*qb*qbu)+bankD[i+1][0]*qbuu);
        }
        for(auto v:span.referenceUp)if(!finite(v))throw std::runtime_error("Nonfinite reference frame polynomial");
        for(double v:span.bank)if(!std::isfinite(v))throw std::runtime_error("Nonfinite bank polynomial");
    }
}

void captureCanonicalDerivatives(Track& track) {
    // Freeze the physical frame, including all jets, in a single gauge. A
    // reload or spatial subdivision must not estimate different derivatives.
    for(size_t i=0;i<track.knots.size();++i) {
        const auto q=i<track.spans.size()?sampleSpanKinematics(track,i,0):sampleSpanKinematics(track,track.spans.size()-1,1);
        auto& k=track.knots[i];k.up=q.sample.up;k.bank=0;
        k.third=q.curvatureS;k.fourth=q.curvatureSS;
        k.upFirst=q.upS;k.upSecond=q.upSS;k.upThird=q.upSSS;
    }
    if(track.closed)track.knots.back()=track.knots.front();
    track.authoredGeometry=track.authoredFrame=true;track.rebuild();
}

TrackSample Track::sampleSpan(size_t i,double u) const{
    checkParameter(*this,i,u);const auto& span=spans[i];auto d=positionDerivative(span,u);double q=norm(d.value);Vec3 tangent=d.value/q;
    Vec3 raw=value(span.referenceUp,u),projected=raw-tangent*dot(raw,tangent);double magnitude=norm(projected);
    if(!std::isfinite(q)||q<=0||!std::isfinite(magnitude)||magnitude<1e-10)throw std::runtime_error("Degenerate canonical frame");
    Vec3 up=rotate(projected/magnitude,tangent,value(span.bank,u));
    return {value(span.c,u),tangent,(d.first-tangent*dot(tangent,d.first))/(q*q),up,unit(cross(tangent,up)),knots[i].element};
}
Vec3 Track::position(double distance,size_t& hint) const{
    const auto at=locate(distance,hint);
    return value(spans[at.span].c,at.parameter);
}
Vec3 Track::tangent(double distance) const{
    size_t hint=spans.size();return tangent(distance,hint);
}
Vec3 Track::tangent(double distance,size_t& hint) const{
    const auto at=locate(distance,hint);const auto& c=spans[at.span].c;
    Vec3 derivative=c.back()*double(c.size()-1);
    for(int i=int(c.size())-2;i>=1;--i)derivative=derivative*at.parameter+c[i]*double(i);
    const double magnitude=norm(derivative);
    if(!std::isfinite(magnitude)||magnitude<=0)throw std::runtime_error("Degenerate canonical tangent");
    return derivative/magnitude;
}
std::array<double,8> Track::bankPolynomial(size_t i) const{if(i>=spans.size())throw std::runtime_error("Invalid canonical bank span");return spans[i].bank;}

TrackKinematics sampleSpanKinematics(const Track& track,size_t i,double u){
    checkParameter(track,i,u);const auto& span=track.spans[i];auto d=positionDerivative(span,u);auto q=magnitude(d);auto tangent=d*inverse(q);
    auto raw=polynomialJet(span.referenceUp,u);auto unbanked=normalized(raw-tangent*jetDot(raw,tangent));
    // Authored frames already include physical roll, with an identically zero
    // separate bank polynomial. Preserve the legacy path for unowned frames.
    auto up=unbanked;
    if(!track.authoredFrame){
        const auto bank=polynomialJet(span.bank,u);const double sinBank=std::sin(bank.value),cosBank=std::cos(bank.value);
        const ScalarJet c{cosBank,-sinBank*bank.first,-sinBank*bank.second-cosBank*bank.first*bank.first,-sinBank*bank.third-3*cosBank*bank.first*bank.second+sinBank*bank.first*bank.first*bank.first};
        const ScalarJet s{sinBank,cosBank*bank.first,cosBank*bank.second-sinBank*bank.first*bank.first,cosBank*bank.third-3*sinBank*bank.first*bank.second-cosBank*bank.first*bank.first*bank.first};
        up=unbanked*c+jetCross(tangent,unbanked)*s+tangent*(jetDot(tangent,unbanked)*(ScalarJet{1,0,0}-c));
    }
    const double q2=q.value*q.value,q3=q2*q.value;
    Vec3 upS=up.first/q.value,upSS=up.second/q2-up.first*(q.first/q3);
    Vec3 curvature=tangent.first/q.value,curvatureS=tangent.second/q2-tangent.first*(q.first/q3);
    const auto thirdArc=[&](const VectorJet& f){return f.third/q3-f.second*(3*q.first/(q3*q.value))-f.first*(q.second/(q3*q.value))+f.first*(3*q.first*q.first/(q3*q2));};
    Vec3 upSSS=thirdArc(up),curvatureSS=thirdArc(tangent);
    TrackSample sample{value(span.c,u),tangent.value,curvature,up.value,unit(cross(tangent.value,up.value)),track.knots[i].element};
    if(!finite(upS)||!finite(upSS)||!finite(curvatureS)||!finite(upSSS)||!finite(curvatureSS))throw std::runtime_error("Nonfinite canonical frame derivative");
    return {sample,upS,upSS,curvatureS,upSSS,curvatureSS};
}
TrackKinematics sampleKinematics(const Track& track,double distance){auto where=track.locate(distance);return sampleSpanKinematics(track,where.span,where.parameter);}
}
