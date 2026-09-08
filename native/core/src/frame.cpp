#include "coaster/coaster.hpp"
#include <stdexcept>

namespace coaster {
namespace {
// Second-order jets differentiate the same canonical expression used to render
// the frame. No spatial averaging stencil or independent rider spline is used.
struct ScalarJet { double value{},first{},second{}; };
struct VectorJet { Vec3 value,first,second; };
ScalarJet operator-(ScalarJet a,ScalarJet b){return {a.value-b.value,a.first-b.first,a.second-b.second};}
ScalarJet operator*(ScalarJet a,ScalarJet b){return {a.value*b.value,a.first*b.value+a.value*b.first,a.second*b.value+2*a.first*b.first+a.value*b.second};}
ScalarJet inverse(ScalarJet a){double r=1/a.value;return {r,-a.first*r*r,2*a.first*a.first*r*r*r-a.second*r*r};}
ScalarJet squareRoot(ScalarJet a){double r=std::sqrt(a.value);return {r,a.first/(2*r),a.second/(2*r)-a.first*a.first/(4*r*r*r)};}
ScalarJet sine(ScalarJet a){double s=std::sin(a.value),c=std::cos(a.value);return {s,c*a.first,c*a.second-s*a.first*a.first};}
ScalarJet cosine(ScalarJet a){double s=std::sin(a.value),c=std::cos(a.value);return {c,-s*a.first,-s*a.second-c*a.first*a.first};}
VectorJet operator+(VectorJet a,VectorJet b){return {a.value+b.value,a.first+b.first,a.second+b.second};}
VectorJet operator-(VectorJet a,VectorJet b){return {a.value-b.value,a.first-b.first,a.second-b.second};}
VectorJet operator*(VectorJet a,ScalarJet b){return {a.value*b.value,a.first*b.value+a.value*b.first,a.second*b.value+a.first*(2*b.first)+a.value*b.second};}
ScalarJet jetDot(VectorJet a,VectorJet b){return {dot(a.value,b.value),dot(a.first,b.value)+dot(a.value,b.first),dot(a.second,b.value)+2*dot(a.first,b.first)+dot(a.value,b.second)};}
VectorJet jetCross(VectorJet a,VectorJet b){return {cross(a.value,b.value),cross(a.first,b.value)+cross(a.value,b.first),cross(a.second,b.value)+cross(a.first,b.first)*2+cross(a.value,b.second)};}
VectorJet normalized(VectorJet a){auto n=jetDot(a,a);if(!std::isfinite(n.value)||n.value<1e-20)throw std::runtime_error("Degenerate canonical frame");return a*inverse(squareRoot(n));}

template<class T,size_t N>T value(const std::array<T,N>& c,double u){T p=c.back();for(size_t i=N-1;i-->0;)p=p*u+c[i];return p;}
template<size_t N>VectorJet polynomialJet(const std::array<Vec3,N>& c,double u){VectorJet p{c.back(),{}, {}};for(size_t i=N-1;i-->0;){p.second=p.second*u+p.first*2;p.first=p.first*u+p.value;p.value=p.value*u+c[i];}return p;}
template<size_t N>ScalarJet polynomialJet(const std::array<double,N>& c,double u){ScalarJet p{c.back(),0,0};for(size_t i=N-1;i-->0;){p.second=p.second*u+2*p.first;p.first=p.first*u+p.value;p.value=p.value*u+c[i];}return p;}
VectorJet positionDerivative(const Span& span,double u){std::array<Vec3,7> c;for(size_t i=0;i<c.size();++i)c[i]=span.c[i+1]*double(i+1);return polynomialJet(c,u);}

template<class T>std::array<T,6> quintic(T a,T b,T a1,T b1,T a2,T b2){
    T c2=a2*.5,p=b-a-a1-c2,v=b1-a1-c2*2,acc=b2-c2*2;
    return {a,a1,c2,p*10-v*4+acc*.5,p*(-15)+v*7-acc,p*6-v*3+acc*.5};
}
template<class T>std::vector<std::array<T,2>> sharedDerivatives(const Track& track,const std::vector<T>& values){
    const size_t unique=values.size()-(track.closed?1:0);std::vector<std::array<T,2>> out(values.size());
    std::vector<double> coordinate(values.size());for(size_t i=1;i<coordinate.size();++i)coordinate[i]=coordinate[i-1]+track.spans[i-1].length;
    for(size_t i=0;i<unique;++i){
        const int count=int(std::min<size_t>(5,unique));int first=int(i)-count/2;if(!track.closed)first=std::clamp(first,0,int(unique)-count);
        std::array<size_t,5> index{};std::array<double,5> xs{};double scale=0;
        for(int j=0;j<count;++j){int at=first+j,wrapped=(at%int(unique)+int(unique))%int(unique),lap=at<0?-1:at>=int(unique)?1:0;index[j]=size_t(wrapped);xs[j]=coordinate[wrapped]+lap*track.length-coordinate[i];scale=std::max(scale,std::abs(xs[j]));}
        if(!(scale>0))throw std::runtime_error("Degenerate frame derivative stencil");
        double matrix[5][7]{};for(int row=0;row<count;++row){for(int j=0;j<count;++j)matrix[row][j]=std::pow(xs[j]/scale,row);matrix[row][count]=row==1?1:0;matrix[row][count+1]=row==2?2:0;}
        for(int col=0;col<count;++col){int pivot=col;for(int row=col+1;row<count;++row)if(std::abs(matrix[row][col])>std::abs(matrix[pivot][col]))pivot=row;
            for(int k=col;k<count+2;++k)std::swap(matrix[col][k],matrix[pivot][k]);double divisor=matrix[col][col];if(std::abs(divisor)<1e-12)throw std::runtime_error("Unresolved frame derivative stencil");
            for(int k=col;k<count+2;++k)matrix[col][k]/=divisor;for(int row=0;row<count;++row)if(row!=col){double factor=matrix[row][col];for(int k=col;k<count+2;++k)matrix[row][k]-=factor*matrix[col][k];}}
        T firstDerivative{},secondDerivative{};for(int j=0;j<count;++j){T delta=values[index[j]]-values[i];firstDerivative=firstDerivative+delta*(matrix[j][count]/scale);secondDerivative=secondDerivative+delta*(matrix[j][count+1]/(scale*scale));}out[i]={firstDerivative,secondDerivative};
    }
    if(track.closed)out.back()=out.front();return out;
}void checkParameter(const Track& track,size_t i,double u){if(i>=track.spans.size()||!std::isfinite(u)||u<0||u>1)throw std::runtime_error("Invalid canonical frame parameter");}
}

void rebuildFramePolynomials(Track& track){
    if(track.knots.size()<4||track.spans.size()+1!=track.knots.size())throw std::runtime_error("Frame cache requires rebuilt canonical spans");
    std::vector<Vec3> up;std::vector<double> bank;up.reserve(track.knots.size());bank.reserve(track.knots.size());
    for(const auto& k:track.knots){up.push_back(k.up);bank.push_back(k.bank);}
    auto upD=sharedDerivatives(track,up);auto bankD=sharedDerivatives(track,bank);
    for(size_t i=0;i<track.spans.size();++i){
        auto& span=track.spans[i];auto a=positionDerivative(span,0),b=positionDerivative(span,1);
        double qa=norm(a.value),qb=norm(b.value);if(!(qa>0&&qb>0))throw std::runtime_error("Zero endpoint metric in frame cache");
        double qau=dot(a.value,a.first)/qa,qbu=dot(b.value,b.first)/qb;
        // Parameter jets use actual endpoint metric, not quadrature span length.
        // Shared arc derivatives therefore agree even on unequal curved spans.
        span.referenceUp=quintic(up[i],up[i+1],upD[i][0]*qa,upD[i+1][0]*qb,
            upD[i][1]*(qa*qa)+upD[i][0]*qau,upD[i+1][1]*(qb*qb)+upD[i+1][0]*qbu);
        span.bank=quintic(bank[i],bank[i+1],bankD[i][0]*qa,bankD[i+1][0]*qb,
            bankD[i][1]*(qa*qa)+bankD[i][0]*qau,bankD[i+1][1]*(qb*qb)+bankD[i+1][0]*qbu);
        for(auto v:span.referenceUp)if(!finite(v))throw std::runtime_error("Nonfinite reference frame polynomial");
        for(double v:span.bank)if(!std::isfinite(v))throw std::runtime_error("Nonfinite bank polynomial");
    }
}

TrackSample Track::sampleSpan(size_t i,double u) const{
    checkParameter(*this,i,u);const auto& span=spans[i];auto d=positionDerivative(span,u);double q=norm(d.value);Vec3 tangent=d.value/q;
    Vec3 raw=value(span.referenceUp,u),projected=raw-tangent*dot(raw,tangent);double magnitude=norm(projected);
    if(!std::isfinite(q)||q<=0||!std::isfinite(magnitude)||magnitude<1e-10)throw std::runtime_error("Degenerate canonical frame");
    Vec3 up=rotate(projected/magnitude,tangent,value(span.bank,u));
    return {value(span.c,u),tangent,(d.first-tangent*dot(tangent,d.first))/(q*q),up,unit(cross(tangent,up)),knots[i].element};
}
std::array<double,6> Track::bankPolynomial(size_t i) const{if(i>=spans.size())throw std::runtime_error("Invalid canonical bank span");return spans[i].bank;}

TrackKinematics sampleSpanKinematics(const Track& track,size_t i,double u){
    checkParameter(track,i,u);const auto& span=track.spans[i];auto d=positionDerivative(span,u);auto q=squareRoot(jetDot(d,d));auto tangent=normalized(d);
    auto raw=polynomialJet(span.referenceUp,u);auto unbanked=normalized(raw-tangent*jetDot(raw,tangent));
    auto bank=polynomialJet(span.bank,u),c=cosine(bank),s=sine(bank);
    auto up=unbanked*c+jetCross(tangent,unbanked)*s+tangent*(jetDot(tangent,unbanked)*(ScalarJet{1,0,0}-c));
    const double q2=q.value*q.value,q3=q2*q.value;
    Vec3 upS=up.first/q.value,upSS=up.second/q2-up.first*(q.first/q3);
    Vec3 curvature=tangent.first/q.value,curvatureS=tangent.second/q2-tangent.first*(q.first/q3);
    TrackSample sample{value(span.c,u),tangent.value,curvature,up.value,unit(cross(tangent.value,up.value)),track.knots[i].element};
    if(!finite(upS)||!finite(upSS)||!finite(curvatureS))throw std::runtime_error("Nonfinite canonical frame derivative");
    return {sample,upS,upSS,curvatureS};
}
TrackKinematics sampleKinematics(const Track& track,double distance){auto where=track.locate(distance);return sampleSpanKinematics(track,where.span,where.parameter);}
}
