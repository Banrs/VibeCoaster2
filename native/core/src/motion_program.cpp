#include "motion_program.hpp"
#include <stdexcept>
#include <sstream>
#include <iomanip>

namespace coaster {
namespace {
constexpr int degree=MotionProgram::degree,count=MotionProgram::controlCount;
constexpr int freeCount=count-8,variables=1+2*freeCount,intervals=count-degree;
using Controls=std::array<double,count>;
using Parameters=std::array<double,variables>;
using Row=std::array<double,variables>;
using Matrix=std::array<Row,variables>;
using Basis=std::array<detail::AngleJet,count>;

struct Differential {
    double value{};Row derivative{};
    Differential(double v=0):value(v){}
    Differential(double v,Row d):value(v),derivative(d){}
};
Differential operator+(Differential a,Differential b) {
    for(int i=0;i<variables;++i)a.derivative[i]+=b.derivative[i];a.value+=b.value;return a;
}
Differential operator-(Differential a,Differential b) {
    for(int i=0;i<variables;++i)a.derivative[i]-=b.derivative[i];a.value-=b.value;return a;
}
Differential operator*(Differential a,Differential b) {
    for(int i=0;i<variables;++i)a.derivative[i]=a.derivative[i]*b.value+a.value*b.derivative[i];a.value*=b.value;return a;
}
Differential operator/(Differential a,Differential b) {
    for(int i=0;i<variables;++i)a.derivative[i]=(a.derivative[i]*b.value-a.value*b.derivative[i])/(b.value*b.value);a.value/=b.value;return a;
}
double sine(double a){return std::sin(a);} double cosine(double a){return std::cos(a);} double root(double a){return std::sqrt(a);}
double exponential(double a){return std::exp(a);} double valueOf(double a){return a;}
double valueOf(const Differential& a){return a.value;}
Differential exponential(Differential a){a.value=std::exp(a.value);for(double& d:a.derivative)d*=a.value;return a;}
Differential sine(Differential a) {for(double& d:a.derivative)d*=std::cos(a.value);a.value=std::sin(a.value);return a;}
Differential cosine(Differential a) {for(double& d:a.derivative)d*=-std::sin(a.value);a.value=std::cos(a.value);return a;}
Differential root(Differential a) {a.value=std::sqrt(a.value);for(double& d:a.derivative)d/=2*a.value;return a;}

template<class S> std::array<S,6> forceState(S pitch,S pitchS,S pitchSS,S yawS,S yawSS,S squaredSpeed,const MotionIntent& intent) {
    const S v=root(squaredSpeed),c=cosine(pitch),s=sine(pitch);
    const S acceleration=S(-gravity)*s-S(intent.rollingAcceleration)-S(intent.dragCoefficient)*squaredSpeed;
    const S normal=c+squaredSpeed*pitchS/S(gravity),lateral=squaredSpeed*c*yawS/S(gravity);
    const S normalRate=S(-1)*v*s*pitchS+(S(2)*v*acceleration*pitchS+v*squaredSpeed*pitchSS)/S(gravity);
    const S lateralRate=(S(2)*v*acceleration*c*yawS+v*squaredSpeed*(c*yawSS-s*pitchS*yawS))/S(gravity);
    const S x=normal*normal+S(.09),y=lateral*normal;
    const S bankRate=(x*(lateralRate*normal+lateral*normalRate)-y*S(2)*normal*normalRate)/(x*x+y*y);
    const S bankMagnitude=root(x*x+y*y);
    return {normal,lateral,normalRate,bankRate,(lateral*x-normal*y)/bankMagnitude,(normal*x+lateral*y)/bankMagnitude};
}

double breakpoint(int i) {return .5*(1-std::cos(pi*std::clamp(double(i)/intervals,0.,1.)));}

Basis basisAt(double u) {
    u=std::clamp(u,0.,std::nextafter(1.,0.));
    std::array<double,count+degree+1> knots{};
    for(int i=0;i<int(knots.size());++i)knots[i]=breakpoint(i-degree);
    std::array<detail::AngleJet,count+degree> values{};
    for(int i=0;i<int(values.size());++i)values[i].value=u>=knots[i]&&u<knots[i+1]?1:0;
    for(int order=1;order<=degree;++order)for(int i=0;i<count+degree-order;++i) {
        const auto a=values[i],b=values[i+1];
        const double left=knots[i+order]-knots[i],right=knots[i+order+1]-knots[i+1];
        const double l=left>0?(u-knots[i])/left:0,r=right>0?(knots[i+order+1]-u)/right:0;
        const double ld=left>0?1/left:0,rd=right>0?-1/right:0;
        values[i]={l*a.value+r*b.value,l*a.first+ld*a.value+r*b.first+rd*b.value,
            l*a.second+2*ld*a.first+r*b.second+2*rd*b.first,
            l*a.third+3*ld*a.second+r*b.third+3*rd*b.second};
    }
    Basis result;std::copy_n(values.begin(),count,result.begin());return result;
}

detail::AngleJet angle(const Controls& controls,const Basis& basis,double length) {
    detail::AngleJet result;
    for(int i=0;i<count;++i) {
        result.value+=controls[i]*basis[i].value;result.first+=controls[i]*basis[i].first;
        result.second+=controls[i]*basis[i].second;result.third+=controls[i]*basis[i].third;
    }
    result.first/=length;result.second/=length*length;result.third/=length*length*length;
    return result;
}

void endControls(Controls& controls,detail::AngleJet jet,double length,bool reverse) {
    if(reverse){jet.first=-jet.first;jet.third=-jet.third;}
    const double h1=length*breakpoint(1),h2=length*breakpoint(2),h3=length*breakpoint(3);
    const double d1=jet.first+h1*jet.second/(degree-1);
    const double d2=d1+h2/(degree-1)*(jet.second+h1*jet.third/(degree-2));
    const std::array<double,4> offsets{0,h1*jet.first/degree,
        (h1*jet.first+h2*d1)/degree,(h1*jet.first+h2*d1+h3*d2)/degree};
    for(int i=0;i<4;++i)controls[reverse?count-1-i:i]=jet.value+offsets[i];
}

struct IntegrationPoint {double weight,parameter;Basis basis;double energyStep{};Basis midpoint;};
const std::vector<IntegrationPoint>& quadrature() {
    static const auto points=[] {
        constexpr double nodes[]{.1834346424956498,.525532409916329,.796666477413627,.960289856497536};
        constexpr double weights[]{.362683783378362,.313706645877887,.222381034453374,.101228536290376};
        std::vector<IntegrationPoint> result;
        for(int i=0;i<intervals;++i)for(int j=0;j<4;++j)for(double side:{-1.,1.}) {
            const double middle=(breakpoint(i)+breakpoint(i+1))*.5,half=(breakpoint(i+1)-breakpoint(i))*.5;
            const double u=middle+side*nodes[j]*half;
            result.push_back({weights[j]*half,u,basisAt(u)});
        }
        std::sort(result.begin(),result.end(),[](const auto& a,const auto& b){return a.parameter<b.parameter;});
        double previous=0;
        for(auto& q:result){q.energyStep=q.parameter-previous;q.midpoint=basisAt((q.parameter+previous)*.5);previous=q.parameter;}
        return result;
    }();return points;
}

template<class S,class Read,class Visit> void visitForces(const MotionIntent& intent,S length,const Read& read,const Visit& visit) {
    S energy(intent.entrySpeed*intent.entrySpeed);
    for(const auto& q:quadrature()) {
        const S distance=length*S(q.energyStep),pitch=read(q.midpoint,false,0);
        const S resistance=S(gravity)*sine(pitch)+S(intent.rollingAcceleration);
        // Integrate d(v^2)/ds along the actual spline grade with exact constant-
        // grade drag over each quadrature interval. Scalar and Jacobian passes
        // share this model; endpoint-linear height is not an energy trajectory.
        if(intent.dragCoefficient>0){const S decay=exponential(S(-2*intent.dragCoefficient)*distance);energy=decay*energy-resistance*(S(1)-decay)/S(intent.dragCoefficient);}
        else energy=energy-S(2)*resistance*distance;
        const S speedSquared=valueOf(energy)>100?energy:S(100);
        visit(q,forceState(read(q.basis,false,0),read(q.basis,false,1),read(q.basis,false,2),
            read(q.basis,true,1),read(q.basis,true,2),speedSquared,intent));
    }
}

Vec3 displacement(const MotionProgram& p) {
    Vec3 result,roundoff;
    for(const auto& q:quadrature()) {
        const double pitch=angle(p.pitch,q.basis,1).value,yaw=angle(p.heading,q.basis,1).value;
        const Vec3 step=Vec3{std::cos(pitch)*std::cos(yaw),std::cos(pitch)*std::sin(yaw),std::sin(pitch)}*(q.weight*p.length)-roundoff;
        const Vec3 next=result+step;roundoff=(next-result)-step;result=next;
    }
    return result;
}

bool linearSolve(std::vector<std::vector<double>> matrix,std::vector<double>& result) {
    const size_t n=matrix.size();
    for(size_t column=0;column<n;++column) {
        size_t pivot=column;
        for(size_t row=column+1;row<n;++row)if(std::abs(matrix[row][column])>std::abs(matrix[pivot][column]))pivot=row;
        if(std::abs(matrix[pivot][column])<1e-11)return false;
        std::swap(matrix[column],matrix[pivot]);const double divisor=matrix[column][column];
        for(size_t j=column;j<=n;++j)matrix[column][j]/=divisor;
        for(size_t row=column+1;row<n;++row) {
            const double factor=matrix[row][column];
            for(size_t j=column;j<=n;++j)matrix[row][j]-=factor*matrix[column][j];
        }
    }
    result.assign(n,0);
    for(size_t i=n;i-->0;) {
        result[i]=matrix[i][n];for(size_t j=i+1;j<n;++j)result[i]-=matrix[i][j]*result[j];
    }
    return true;
}

struct Constraint {Row row{};double value{};}; // row * step >= value
double dotRow(const Row& a,const Row& b) {double sum=0;for(int i=0;i<variables;++i)sum+=a[i]*b[i];return sum;}

bool constrainedStep(const Matrix& hessian,const Row& gradient,const std::vector<Constraint>& equality,
                     const std::vector<Constraint>& inequalities,Row& step,std::string& diagnostic,const Cancel& cancel,int activeVariables) {
    const size_t m=inequalities.size(),n=activeVariables+equality.size();
    Row x{};std::vector<double> y(equality.size()),slack(m,1),dual(m,1);
    for(int iteration=0;iteration<80;++iteration) {
        if(cancel&&cancel())throw std::runtime_error("CANCELLED");
        Row residual=gradient;std::vector<double> equalResidual(equality.size()),inequalResidual(m),central(m);
        double gap=0,maximum=0;
        for(int i=0;i<activeVariables;++i) {
            for(int j=0;j<activeVariables;++j)residual[i]+=hessian[i][j]*x[j];
            for(size_t j=0;j<equality.size();++j)residual[i]+=equality[j].row[i]*y[j];
            for(size_t j=0;j<m;++j)residual[i]-=inequalities[j].row[i]*dual[j];
            maximum=std::max(maximum,std::abs(residual[i]));
        }
        for(size_t i=0;i<equality.size();++i) {
            equalResidual[i]=dotRow(equality[i].row,x)-equality[i].value;
            maximum=std::max(maximum,std::abs(equalResidual[i]));
        }
        for(size_t i=0;i<m;++i) {
            inequalResidual[i]=dotRow(inequalities[i].row,x)-slack[i]-inequalities[i].value;
            maximum=std::max(maximum,std::abs(inequalResidual[i]));gap+=slack[i]*dual[i]/m;
        }
        if(maximum<1e-8&&gap<1e-10){step=x;return true;}
        for(size_t i=0;i<m;++i)central[i]=slack[i]*dual[i]-.1*gap;
        std::vector<std::vector<double>> matrix(n,std::vector<double>(n+1));
        for(int i=0;i<activeVariables;++i) {
            for(int j=0;j<activeVariables;++j)matrix[i][j]=hessian[i][j];
            matrix[i][n]=-residual[i];
            for(size_t c=0;c<m;++c) {
                const double factor=dual[c]/slack[c];
                matrix[i][n]-=inequalities[c].row[i]*(central[c]+dual[c]*inequalResidual[c])/slack[c];
                for(int j=0;j<activeVariables;++j)matrix[i][j]+=factor*inequalities[c].row[i]*inequalities[c].row[j];
            }
        }
        for(size_t c=0;c<equality.size();++c) {
            for(int i=0;i<activeVariables;++i)matrix[i][activeVariables+c]=matrix[activeVariables+c][i]=equality[c].row[i];
            matrix[activeVariables+c][n]=-equalResidual[c];
        }
        std::vector<double> solved;
        if(!linearSolve(std::move(matrix),solved)){diagnostic="singular interior-point system";return false;}
        Row dx{};std::copy_n(solved.begin(),activeVariables,dx.begin());
        std::vector<double> ds(m),dz(m);double fraction=1;
        for(size_t c=0;c<m;++c) {
            ds[c]=dotRow(inequalities[c].row,dx)+inequalResidual[c];
            dz[c]=(-central[c]-dual[c]*ds[c])/slack[c];
            if(ds[c]<0)fraction=std::min(fraction,-.995*slack[c]/ds[c]);
            if(dz[c]<0)fraction=std::min(fraction,-.995*dual[c]/dz[c]);
        }
        for(int i=0;i<activeVariables;++i)x[i]+=fraction*dx[i];
        for(size_t i=0;i<equality.size();++i)y[i]+=fraction*solved[activeVariables+i];
        for(size_t i=0;i<m;++i){slack[i]+=fraction*ds[i];dual[i]+=fraction*dz[i];}
    }
    diagnostic="interior-point iteration limit";return false;
}

}

detail::MotionJet MotionProgram::direction(double distance) const {
    if(distance<=0)return begin;if(distance>=length)return end;
    const auto basis=basisAt(distance/length);
    return detail::directionJet(angle(pitch,basis,length),angle(heading,basis,length));
}

Vec3 MotionProgram::displacement(double endDistance) const {
    Vec3 sum{},roundoff{};
    for(int i=0;i<intervals;++i) {
        const double first=length*breakpoint(i),last=std::min(endDistance,length*breakpoint(i+1));
        if(last<=first)break;
        const Vec3 step=detail::integrateDirection([&](double s){return direction(s).tangent;},first,last)-roundoff;
        const Vec3 next=sum+step;roundoff=(next-sum)-step;sum=next;
    }
    return sum;
}

MotionProgram polynomialMotion(const detail::MotionJet& begin,const std::array<double,8>& pitch,const std::array<double,8>& heading,double length){
    MotionProgram p;p.begin=begin;p.length=length;
    // Convert the one global septic into the existing editable B-spline
    // representation by exact knot insertion; no resampling or refitting.
    for(int channel=0;channel<2;++channel){const auto& polynomial=channel?heading:pitch;
    std::vector<double> controls(8),knots(16,1);std::fill_n(knots.begin(),8,0.);
    auto choose=[](int n,int k){double out=1;for(int i=1;i<=k;++i)out*=double(n+1-i)/i;return out;};
    for(int i=0;i<8;++i)for(int j=0;j<=i;++j)controls[i]+=polynomial[j]*choose(i,j)/choose(7,j);
    for(int j=1;j<intervals;++j){const double u=breakpoint(j);const int k=int(std::upper_bound(knots.begin(),knots.end(),u)-knots.begin())-1;
        std::vector<double> next(controls.size()+1);
        for(int i=0;i<=k-degree;++i)next[i]=controls[i];
        for(int i=k+1;i<int(next.size());++i)next[i]=controls[i-1];
        for(int i=k-degree+1;i<=k;++i){const double alpha=(u-knots[i])/(knots[i+degree]-knots[i]);next[i]=alpha*controls[i]+(1-alpha)*controls[i-1];}
        controls=std::move(next);knots.insert(knots.begin()+k+1,u);
    }
    std::copy(controls.begin(),controls.end(),(channel?p.heading:p.pitch).begin());
    }
    p.end=detail::directionJet(detail::angleAt(pitch,length,length),detail::angleAt(heading,length,length));p.end.position=begin.position+p.displacement(length);return p;
}

enum class PlacementConstraint { Position, Length, Height };
static MotionProgram solveMotionShape(const detail::MotionJet& begin,const detail::MotionJet& end,double estimatedLength,const MotionIntent& intent,bool alternateEntryShape,bool linearInitialHeading,PlacementConstraint placement,const Cancel& cancel) {
    const bool fixedPosition=placement==PlacementConstraint::Position,fixedHeight=placement==PlacementConstraint::Height;
    if(!std::isfinite(estimatedLength)||estimatedLength<=0)throw std::runtime_error("Motion length must be finite and positive");
    const double speed=intent.entrySpeed;
    const Vec3 delta=end.position-begin.position;
    // A free-placement endpoint only supplies a height and derivative jets.
    // Its unused XY coordinates must not become a zero chord or an artificial
    // near-vertical reference direction in the optimizer's scaling/shape rules.
    const double chord=fixedPosition?norm(delta):estimatedLength/1.25;
    if(!std::isfinite(chord)||chord<=0)throw std::runtime_error("Motion requires a nonzero finite placement scale");
    auto first=detail::directionAngles(begin),last=detail::directionAngles(end);
    const auto levelJet=[](detail::AngleJet q){return std::abs(q.value)<1e-8&&std::abs(q.first)+std::abs(q.second)+std::abs(q.third)<1e-10;};
    // Almost-level placed corridors have no meaningful pitch design freedom.
    // Eliminating those variables avoids an interior-point problem whose whole
    // feasible pitch polytope has collapsed to zero volume. The final global
    // equality correction still retains the literal height and endpoint jets.
    const bool nearlyLevel=fixedPosition&&std::abs(delta.z)<1e-6&&levelJet(first[0])&&levelJet(last[0]);
    const int headingOffset=nearlyLevel?0:freeCount,activeVariables=nearlyLevel?1+freeCount:variables;
    double yawChange=std::remainder(last[1].value-first[1].value,2*pi);
    // At a half-turn, atan2 rounding must not pick the opposite physical
    // revolution. The chord's side supplies the otherwise ambiguous hand.
    const double corridorHand=cross(begin.tangent,delta).z;
    if(std::abs(std::abs(yawChange)-pi)<1e-8&&yawChange*corridorHand<0)yawChange+=std::copysign(2*pi,corridorHand);
    last[1].value=first[1].value+yawChange;
    const double chordPitch=fixedPosition?std::atan2(delta.z,std::hypot(delta.x,delta.y)):fixedHeight?std::asin(std::clamp(delta.z/estimatedLength,-1.,1.)):(first[0].value+last[0].value)*.5;
    auto sign=[](double value){return value>=0?1.:-1.;};
    auto entrySign=[&](detail::AngleJet a,double fallback) {
        if(std::abs(a.first)>1e-7)return sign(a.first);
        if(std::abs(a.second)>1e-8)return sign(a.second);return sign(fallback);
    };
    auto exitJet=last[0];exitJet.second=-exitJet.second;
    const double initialSign=(alternateEntryShape?-1.:1.)*entrySign(first[0],chordPitch-first[0].value);
    const double terminalSign=entrySign(exitJet,last[0].value-chordPitch);
    const bool extremum=initialSign!=terminalSign;
    const double yawSign=sign(last[1].value-first[1].value);
    const double chordYaw=fixedPosition?first[1].value+std::remainder(std::atan2(delta.y,delta.x)-first[1].value,2*pi):(first[1].value+last[1].value)*.5;
    const double initialYaw=entrySign(first[1],fixedPosition?chordYaw-first[1].value:yawSign),terminalYaw=entrySign(last[1],fixedPosition?last[1].value-chordYaw:yawSign);
    const bool yawExtremum=initialYaw!=terminalYaw;
    const bool orderedYaw=std::abs(last[1].value-first[1].value)>.05||std::abs(first[1].first)+std::abs(last[1].first)>1e-7||std::abs(chordYaw-first[1].value)>.05;
    auto program=[&](const Parameters& x) {
        MotionProgram p;p.begin=begin;p.end=end;p.length=chord*std::exp(x[0]);
        endControls(p.pitch,first[0],p.length,false);endControls(p.pitch,last[0],p.length,true);
        endControls(p.heading,first[1],p.length,false);endControls(p.heading,last[1],p.length,true);
        for(int i=0;i<freeCount;++i){p.pitch[i+4]=nearlyLevel?std::lerp(p.pitch[3],p.pitch[count-4],double(i+1)/(freeCount+1)):x[i+1];p.heading[i+4]=x[i+1+headingOffset];}
        return p;
    };
    MotionProgram best;int bestPeak=count/2;double bestError=INFINITY,bestFeasibleCost=INFINITY;std::string failure;
    for(int peak:{count/2,count/3,2*count/3}) {
        if(!extremum&&!yawExtremum&&peak!=count/2)continue;
        Parameters x{};x[0]=std::log(std::max(chord,estimatedLength)/chord);const auto boundary=program(x);
        const double apex=initialSign>0?std::max({boundary.pitch[3],boundary.pitch[count-4],2*chordPitch}):std::min({boundary.pitch[3],boundary.pitch[count-4],2*chordPitch});
        for(int i=4;i<count-4;++i) {
            auto greville=[](int control){double u=0;for(int j=1;j<=degree;++j)u+=breakpoint(control+j-degree)/degree;return u;};
            const double u=linearInitialHeading?(greville(i)-greville(3))/(greville(count-4)-greville(3)):double(i-3)/(count-7);
            if(!nearlyLevel)x[i-3]=extremum?(i<=peak?boundary.pitch[3]+(apex-boundary.pitch[3])*double(i-3)/(peak-3):
                apex+(boundary.pitch[count-4]-apex)*double(i-peak)/(count-4-peak)):
                boundary.pitch[3]+u*(boundary.pitch[count-4]-boundary.pitch[3]);
            if(orderedYaw&&yawExtremum){
                const double target=2*chordYaw-(first[1].value+last[1].value)*.5;
                const double turning=initialYaw>0?std::max({boundary.heading[3],boundary.heading[count-4],target}):std::min({boundary.heading[3],boundary.heading[count-4],target});
                x[1+headingOffset+i-4]=i<=peak?boundary.heading[3]+(turning-boundary.heading[3])*double(i-3)/(peak-3):turning+(boundary.heading[count-4]-turning)*double(i-peak)/(count-4-peak);
            }else if(orderedYaw) {
                const double mean=std::clamp((chordYaw-first[1].value)/(last[1].value-first[1].value),.05,.95);
                x[1+headingOffset+i-4]=boundary.heading[3]+std::pow(u,(1-mean)/mean)*(boundary.heading[count-4]-boundary.heading[3]);
            } else {
                const double bend=2*std::remainder(chordYaw-(first[1].value+last[1].value)*.5,2*pi);
                x[1+headingOffset+i-4]=boundary.heading[3]+u*(boundary.heading[count-4]-boundary.heading[3])+bend*std::pow(std::sin(pi*u),2);
            }
        }
        const auto reference=program(x);
        auto pitchSign=[&](int i){return extremum&&i>=peak?terminalSign:initialSign;};
        auto yawDirection=[&](int i){return initialYaw==terminalYaw?initialYaw:(i<peak?initialYaw:terminalYaw);};
        auto shapeError=[&](const MotionProgram& p) {
            double error=0;
            // Endpoint control polygons are fixed by inherited third jets.
            // Ordering constrains the free interior; the actual complete
            // curve still has to pass the independent pitch-shape audit.
            for(int i=3;i<count-4;++i) {
                if(!nearlyLevel)error+=std::max(0.,-pitchSign(i)*(p.pitch[i+1]-p.pitch[i]));
                if(orderedYaw)error+=std::max(0.,-yawDirection(i)*(p.heading[i+1]-p.heading[i]));
            }
            for(int i=4;i<count-4;++i)error+=std::max(0.,std::abs(p.heading[i]-chordYaw)-pi*.5);
            return error;
        };
        auto objective=[&](const MotionProgram& p) {
            double cost=.2*std::pow(std::log(p.length/chord),2);
            for(int channel=0;channel<2;++channel) {
                const auto& c=channel?p.heading:p.pitch;const auto& target=channel?reference.heading:reference.pitch;
                for(int i=0;i<count;++i)cost+=.01*std::pow(c[i]-target[i],2);
                for(const auto& q:quadrature()) {
                    const auto a=angle(c,q.basis,1);
                    const double rate=std::max(5.,speed)/p.length;
                    cost+=q.weight*(.04*a.second*a.second*std::pow(rate,4)+.02*a.third*a.third*std::pow(rate,6));
                }
            }
            visitForces(intent,p.length,[&](const Basis& basis,bool yaw,int order){
                const auto a=angle(yaw?p.heading:p.pitch,basis,p.length);
                return order==0?a.value:order==1?a.first:a.second;
            },[&](const IntegrationPoint& q,const std::array<double,6>& f){
                const double load=f[0]>0?std::hypot(f[0],f[1]):f[0];
                cost+=q.weight*(.01*f[2]*f[2]+.3*f[3]*f[3]+2000*std::pow(std::max(0.,load-intent.maxNormalG),2)+2000*std::pow(std::max(0.,intent.minNormalG-f[5]),2)+2000*std::pow(std::max(0.,std::abs(f[4])-1.15),2));
                if(intent.outwardRoll!=0)cost+=q.weight*50*std::pow(std::max(0.,3-f[0])*f[1],2);
            });
            return cost;
        };
        for(int iteration=0;iteration<90;++iteration) {
            if(cancel&&cancel())throw std::runtime_error("CANCELLED");
            const auto p=program(x);const Vec3 position=displacement(p);
            Vec3 residual=fixedPosition?(position-delta)/chord:fixedHeight?Vec3{0,0,(position.z-delta.z)/chord}:Vec3{};
            if(nearlyLevel)residual.z=0;
            const double error=norm(residual)+shapeError(p),cost=objective(p);
            // The QP solves displacement to finite precision; choose the best
            // physically shaped iterate within that numerical neighbourhood,
            // then polish the exact final equality. Selecting only the tiniest
            // residual could discard a much better force/roll solution.
            if(norm(residual)<1e-7&&shapeError(p)<2e-10&&cost<bestFeasibleCost){bestError=error;best=p;bestPeak=peak;bestFeasibleCost=cost;}
            else if(!std::isfinite(bestFeasibleCost)&&error<bestError){bestError=error;best=p;bestPeak=peak;}
            constexpr double epsilon=1e-5;auto perturbed=x;perturbed[0]+=epsilon;const auto stretched=program(perturbed);
            std::array<Row,count> pitchJac{},headingJac{};
            for(int i=0;i<count;++i) {
                pitchJac[i][0]=(stretched.pitch[i]-p.pitch[i])/epsilon;headingJac[i][0]=(stretched.heading[i]-p.heading[i])/epsilon;
                if(i>=4&&i<count-4){if(!nearlyLevel)pitchJac[i][i-3]=1;headingJac[i][1+headingOffset+i-4]=1;}
            }
            Matrix hessian{};Row gradient{};for(int i=0;i<variables;++i)hessian[i][i]=1e-5;
            auto term=[&](double value,const Row& jac,double weight) {
                // Compact B-spline support makes these Jacobians sparse.
                // Accumulate one triangle and mirror once per solver step.
                for(int i=0;i<variables;++i)if(jac[i]!=0) {
                    gradient[i]+=2*weight*value*jac[i];
                    for(int j=i;j<variables;++j)if(jac[j]!=0)hessian[i][j]+=2*weight*jac[i]*jac[j];
                }
            };
            Row lengthJac{};lengthJac[0]=1;term(x[0],lengthJac,.2);
            for(int channel=0;channel<2;++channel) {
                const auto& c=channel?p.heading:p.pitch;const auto& jac=channel?headingJac:pitchJac;
                const auto& target=channel?reference.heading:reference.pitch;
                for(int i=0;i<count;++i)term(c[i]-target[i],jac[i],.01);
                for(const auto& q:quadrature())for(int order:{2,3}) {
                    Row row{};double value=0;
                    for(int i=0;i<count;++i) {
                        const double coefficient=order==2?q.basis[i].second:q.basis[i].third;
                        if(coefficient==0)continue;
                        value+=coefficient*c[i];
                        for(int j=0;j<variables;++j)row[j]+=coefficient*jac[i][j];
                    }
                    const double scale=std::pow(std::max(5.,speed)/p.length,order);
                    value*=scale;for(double& v:row)v*=scale;row[0]-=order*value;
                    term(value,row,q.weight*(order==2?.04:.02));
                }
            }
            Row physicalLengthJac{};physicalLengthJac[0]=p.length;
            visitForces(intent,Differential(p.length,physicalLengthJac),[&](const Basis& basis,bool yaw,int order){
                    const auto& c=yaw?p.heading:p.pitch;const auto& jac=yaw?headingJac:pitchJac;
                    double value=0;Row row{};
                    for(int i=0;i<count;++i) {
                        const double coefficient=order==0?basis[i].value:order==1?basis[i].first:basis[i].second;
                        if(coefficient==0)continue;
                        value+=c[i]*coefficient;for(int j=0;j<variables;++j)row[j]+=jac[i][j]*coefficient;
                    }
                    const double scale=std::pow(p.length,-order);value*=scale;
                    for(double& d:row)d*=scale;row[0]-=order*value;
                    return Differential(value,row);
            },[&](const IntegrationPoint& q,const std::array<Differential,6>& f){
                term(f[2].value,f[2].derivative,q.weight*.01);term(f[3].value,f[3].derivative,q.weight*.3);
                if(f[0].value>0) {
                    const auto load=root(f[0]*f[0]+f[1]*f[1]);
                    if(load.value>intent.maxNormalG)term(load.value-intent.maxNormalG,load.derivative,q.weight*2000);
                }
                if(f[5].value<intent.minNormalG)term(f[5].value-intent.minNormalG,f[5].derivative,q.weight*2000);
                if(std::abs(f[4].value)>1.15)term(f[4].value-std::copysign(1.15,f[4].value),f[4].derivative,q.weight*2000);
                if(intent.outwardRoll!=0&&f[0].value<3){const auto turn=(Differential(3)-f[0])*f[1];term(turn.value,turn.derivative,q.weight*50);}
            });
            std::array<Vec3,variables> positionJac{};positionJac[0]=(displacement(stretched)-position)/(epsilon*chord);
            for(const auto& q:quadrature()) {
                const double a=angle(p.pitch,q.basis,1).value,b=angle(p.heading,q.basis,1).value;
                const Vec3 pitchAxis{-std::sin(a)*std::cos(b),-std::sin(a)*std::sin(b),std::cos(a)};
                const Vec3 yawAxis{-std::cos(a)*std::sin(b),std::cos(a)*std::cos(b),0};
                for(int i=0;i<freeCount;++i) {
                    const double weight=q.weight*p.length*q.basis[i+4].value/chord;
                    if(!nearlyLevel)positionJac[i+1]=positionJac[i+1]+pitchAxis*weight;
                    positionJac[i+1+headingOffset]=positionJac[i+1+headingOffset]+yawAxis*weight;
                }
            }
            std::vector<Constraint> equality(fixedPosition?(nearlyLevel?2:3):1);
            if(fixedPosition){
                for(int i=0;i<variables;++i){equality[0].row[i]=positionJac[i].x;equality[1].row[i]=positionJac[i].y;if(!nearlyLevel)equality[2].row[i]=positionJac[i].z;}
                equality[0].value=-residual.x;equality[1].value=-residual.y;if(!nearlyLevel)equality[2].value=-residual.z;
            }else if(fixedHeight){for(int i=0;i<variables;++i)equality[0].row[i]=positionJac[i].z;equality[0].value=-residual.z;}
            else{equality[0].row[0]=1;equality[0].value=std::log(estimatedLength/chord)-x[0];}
            std::vector<Constraint> inequalities;
            for(int channel=0;channel<2;++channel)if((channel==0&&!nearlyLevel)||(channel==1&&orderedYaw)) {
                const auto& c=channel?p.heading:p.pitch;const auto& jac=channel?headingJac:pitchJac;
                for(int i=3;i<count-4;++i) {
                    const double s=channel?yawDirection(i):pitchSign(i);Constraint bound;
                    for(int j=0;j<variables;++j)bound.row[j]=s*(jac[i+1][j]-jac[i][j]);
                    bound.value=-s*(c[i+1]-c[i]);inequalities.push_back(bound);
                }
            }
            for(int i=4;i<count-4;++i)for(double side:{-1.,1.}) {
                Constraint direction;
                for(int j=0;j<variables;++j)direction.row[j]=-side*headingJac[i][j];
                direction.value=side*(p.heading[i]-chordYaw)-pi*.5;inequalities.push_back(direction);
            }
            Constraint lower,upper;lower.row[0]=1;lower.value=-x[0];upper.row[0]=-1;upper.value=x[0]-std::log(2.5);
            inequalities.push_back(lower);inequalities.push_back(upper);Row step{};
            std::string diagnostic;
            for(int i=0;i<variables;++i)for(int j=0;j<i;++j)hessian[i][j]=hessian[j][i];
            if(!constrainedStep(hessian,gradient,equality,inequalities,step,diagnostic,cancel,activeVariables)){failure="quadratic constraints at iteration "+std::to_string(iteration)+": "+diagnostic;break;}
            // Feasibility alone is not convergence: keep optimizing the
            // force/shape objective until the constrained step is small.
            double stepSize=0;for(double value:step)stepSize=std::max(stepSize,std::abs(value));
            if(error<2e-10&&stepSize<1e-6)break;
            // An exact endpoint is a constraint, not a trade against comfort.
            // Scale the merit weight from the proposed objective change so a
            // low fixed penalty cannot stall centimetres short of the corridor.
            const double violation=norm(residual)+shapeError(p);
            const double penalty=std::max(100.,2*std::abs(dotRow(gradient,step))/std::max(1e-12,violation));
            const double merit=penalty*violation+cost;bool improved=false;
            for(double fraction=1;fraction>=1./256;fraction*=.5) {
                auto trial=x;for(int i=0;i<variables;++i)trial[i]+=fraction*step[i];
                if(trial[0]<-1e-8||trial[0]>std::log(2.5)+1e-8)continue;
                const auto q=program(trial);
                const auto difference=displacement(q)-delta;
                const double next=penalty*((fixedPosition?(nearlyLevel?std::hypot(difference.x,difference.y):norm(difference))/chord:fixedHeight?std::abs(difference.z)/chord:0)+shapeError(q))+objective(q);
                if(next<merit){x=trial;improved=true;break;}
            }
            if(!improved){failure="line search at iteration "+std::to_string(iteration);break;}
        }
        if(bestError<2e-11)break;
    }
    // A zero-curvature grade can admit either initial pitch trend. Try the
    // other single-extremum shape only if the simpler trend cannot reach.
    if(bestError>2e-10&&!nearlyLevel&&!alternateEntryShape&&std::abs(first[0].first)<1e-7&&std::abs(first[0].second)<1e-8)
        return solveMotionShape(begin,end,estimatedLength,intent,true,linearInitialHeading,placement,cancel);
    if(!std::isfinite(bestError))throw std::runtime_error("No feasible ordered motion initialization: "+failure);
    if(!fixedPosition){
        if(fixedHeight&&bestError<1e-5){
            // Correct micrometre-scale height residual in the whole control
            // field. Endpoint value and derivative jets remain untouched.
            for(int iteration=0;iteration<8;++iteration){const double residual=displacement(best).z-delta.z;
                if(std::abs(residual)<1e-10)break;
                auto moved=best;for(int i=4;i<count-4;++i)moved.pitch[i]+=1e-6;
                const double derivative=(displacement(moved).z-displacement(best).z)/1e-6;
                if(std::abs(derivative)<1e-6)break;
                for(int i=4;i<count-4;++i)best.pitch[i]-=residual/derivative;
            }
        }
        double shape=0;
        for(int i=3;i<count-4;++i){const double pitchDirection=extremum&&i>=bestPeak?terminalSign:initialSign;
            shape+=std::max(0.,-pitchDirection*(best.pitch[i+1]-best.pitch[i]));
            const double yawDirection=initialYaw==terminalYaw?initialYaw:(i<bestPeak?initialYaw:terminalYaw);
            if(orderedYaw)shape+=std::max(0.,-yawDirection*(best.heading[i+1]-best.heading[i]));}
        for(int i=4;i<count-4;++i)shape+=std::max(0.,std::abs(best.heading[i]-chordYaw)-pi*.5);
        if(shape>2e-10||(fixedHeight&&std::abs(displacement(best).z-delta.z)>1e-8))throw std::runtime_error("Free-displacement motion did not retain its exact shape/height constraints: "+failure);
        best.end.position=begin.position+best.displacement(best.length);return best;
    }
    // Nanometres of pose residual, snapped into one 0.3 m final span, become
    // a large fourth derivative. Correct the whole continuous programme with
    // three smooth global modes while retaining all endpoint derivative jets.
    for(int iteration=0;iteration<12;++iteration) {
        const Vec3 residual=displacement(best)-delta;
        if(norm(residual)<2e-13*std::max(1.,chord/100))break;
        auto shifted=[&](std::array<double,3> shift){auto p=best;p.length*=std::exp(shift[0]);
            endControls(p.pitch,first[0],p.length,false);endControls(p.pitch,last[0],p.length,true);
            endControls(p.heading,first[1],p.length,false);endControls(p.heading,last[1],p.length,true);
            for(int i=4;i<count-4;++i){const double weight=std::pow(std::sin(pi*(i-3)/(count-7)),2);p.pitch[i]+=weight*shift[1];p.heading[i]+=weight*shift[2];}
            return p;};
        std::vector<std::vector<double>> system(3,std::vector<double>(4));constexpr double epsilon=1e-5;
        for(int j=0;j<3;++j){std::array<double,3> step{};step[j]=epsilon;const auto d=(displacement(shifted(step))-delta-residual)/epsilon;
            system[0][j]=d.x;system[1][j]=d.y;system[2][j]=d.z;}
        system[0][3]=-residual.x;system[1][3]=-residual.y;system[2][3]=-residual.z;
        std::vector<double> step;if(!linearSolve(std::move(system),step))break;
        const auto corrected=shifted({step[0],step[1],step[2]});
        if(norm(displacement(corrected)-delta)>=norm(residual))break;best=corrected;
    }
    // Finish the exact equality solve before deciding feasibility. The same
    // continuous control field is corrected; no final-span snapping or added
    // connector is allowed. Acceptance tolerances remain unchanged.
    if(norm(displacement(best)-delta)>1e-9){std::ostringstream message;message<<std::setprecision(17)<<"Ordered motion endpoint solve failed: "<<norm(displacement(best)-delta)<<" metres; "<<failure;throw std::runtime_error(message.str());}
    for(int i=3;i<count-4;++i){
        const double p=extremum&&i>=bestPeak?terminalSign:initialSign,y=yawExtremum&&i>=bestPeak?terminalYaw:initialYaw;
        if((!nearlyLevel&&p*(best.pitch[i+1]-best.pitch[i])< -2e-10)||(orderedYaw&&y*(best.heading[i+1]-best.heading[i])< -2e-10))
            {std::ostringstream message;message<<"Final endpoint correction cannot violate the continuous pitch/heading shape: pitch="<<p*(best.pitch[i+1]-best.pitch[i])<<", yaw="<<y*(best.heading[i+1]-best.heading[i])<<", priorError="<<bestError<<", feasibleCost="<<bestFeasibleCost;throw std::runtime_error(message.str());}
    }
    const Vec3 bearing=unit(delta);
    if(nearlyLevel)for(double pitch:best.pitch)if(std::abs(pitch)>1e-6)throw std::runtime_error("Level corridor height correction exceeded its numerical range");
    for(int i=0;i<=160;++i) {
        const auto q=best.direction(best.length*i/160);
        // A semicircle has zero chord projection at its ends. A positive
        // .02 floor rejected valid near-180-degree reversals, even though
        // their travel never reversed along the chord.
        if(!finite(q.tangent)||norm(q.curvature)>.12||dot(q.tangent,bearing)<-1e-9)
            {std::ostringstream message;message<<"Ordered arc motion requires a reversal inside its corridor; u="<<double(i)/160<<", chord projection="<<dot(q.tangent,bearing)<<", curvature="<<norm(q.curvature);throw std::runtime_error(message.str());}
    }
    return best;
}
MotionProgram solveMotion(const detail::MotionJet& begin,const detail::MotionJet& end,double length,const MotionIntent& intent,Cancel cancel) {
    // Nonlinear endpoint solves benefit from two bounded initial control
    // fields. Greville interpolation exactly seeds circular half-turns;
    // endpoint-weighted interpolation also handles inherited mixed jets.
    try{return solveMotionShape(begin,end,length,intent,false,false,PlacementConstraint::Position,cancel);}
    catch(const std::runtime_error&){if(cancel&&cancel())throw;return solveMotionShape(begin,end,length,intent,false,true,PlacementConstraint::Position,cancel);}
}

MotionProgram solveDirectionMotion(const detail::MotionJet& begin,const detail::MotionJet& end,double length,const MotionIntent& intent,Cancel cancel) {
    return solveMotionShape(begin,end,length,intent,false,true,PlacementConstraint::Length,cancel);
}
MotionProgram solveHeightMotion(const detail::MotionJet& begin,const detail::MotionJet& end,double length,const MotionIntent& intent,Cancel cancel) {
    return solveMotionShape(begin,end,length,intent,false,true,PlacementConstraint::Height,cancel);
}


}




