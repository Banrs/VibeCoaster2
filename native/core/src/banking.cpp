#include "banking.hpp"
#include "authoring.hpp"
#include <stdexcept>

namespace coaster {
namespace {
// The quadratic programme has only three neighbours on either side. Keep the
// solver local and explicit instead of introducing a general optimizer.
struct BandMatrix {
    std::vector<std::array<double,4>> rows;
    explicit BandMatrix(size_t n):rows(n){}
    double get(size_t i,size_t j) const {
        if(i<j)std::swap(i,j);
        return i-j<4?rows[i][i-j]:0;
    }
    double& at(size_t i,size_t j) {
        if(i<j)std::swap(i,j);
        return rows[i][i-j];
    }
    void penalty(size_t first,const std::vector<double>& coefficients,double weight) {
        for(size_t i=0;i<coefficients.size();++i)
            for(size_t j=0;j<=i;++j)
                at(first+i,first+j)+=weight*coefficients[i]*coefficients[j];
    }
};

std::vector<double> solve(BandMatrix matrix,std::vector<double> rhs,
                          const std::vector<int>& fixed,const std::vector<double>& values) {
    const size_t n=rhs.size();
    for(size_t i=0;i<n;++i)if(fixed[i]) {
        for(size_t j=i>3?i-3:0;j<std::min(n,i+4);++j)if(j!=i) {
            rhs[j]-=matrix.get(i,j)*values[i];
            matrix.at(i,j)=0;
        }
        matrix.at(i,i)=1;rhs[i]=values[i];
    }
    BandMatrix lower(n);
    for(size_t i=0;i<n;++i)for(size_t j=i>3?i-3:0;j<=i;++j) {
        double value=matrix.get(i,j);
        for(size_t k=i>3?i-3:0;k<j;++k)value-=lower.get(i,k)*lower.get(j,k);
        if(i==j) {
            if(!(value>0))throw std::runtime_error("Banking programme is not positive definite");
            lower.at(i,i)=std::sqrt(value);
        } else lower.at(i,j)=value/lower.get(j,j);
    }
    std::vector<double> result(n);
    for(size_t i=0;i<n;++i) {
        double value=rhs[i];
        for(size_t j=i>3?i-3:0;j<i;++j)value-=lower.get(i,j)*result[j];
        result[i]=value/lower.get(i,i);
    }
    for(size_t i=n;i-->0;) {
        double value=result[i];
        for(size_t j=i+1;j<std::min(n,i+4);++j)value-=lower.get(j,i)*result[j];
        result[i]=value/lower.get(i,i);
    }
    return result;
}


double ease(double u) {
    u=std::clamp(u,0.,1.);
    return u*u*u*u*(35+u*(-84+u*(70-20*u)));
}

double basis(double x) {
    if(std::abs(x)>=3)return 0;
    constexpr double choose[]{1,-6,15,-20,15,-6,1};
    double result=0;
    for(int i=0;i<7;++i)result+=choose[i]*std::pow(std::max(0.,x+3-i),5);
    return result/120;
}

using FrameJet=std::array<Vec3,4>;
constexpr int choose[4][4]{{1},{1,1},{1,2,1},{1,3,3,1}};
FrameJet tangentJet(const Knot& k){return {k.tangent,k.curvature,k.third,k.fourth};}
FrameJet upJet(const Knot& k){return {k.up,k.upFirst,k.upSecond,k.upThird};}
FrameJet crossJet(const FrameJet& a,const FrameJet& b){FrameJet out{};for(int n=0;n<4;++n)for(int i=0;i<=n;++i)out[n]=out[n]+cross(a[i],b[n-i])*choose[n][i];return out;}
std::array<double,4> dotJet(const FrameJet& a,const FrameJet& b){std::array<double,4> out{};for(int n=0;n<4;++n)for(int i=0;i<=n;++i)out[n]+=dot(a[i],b[n-i])*choose[n][i];return out;}
detail::AngleJet frameDifference(const Knot& desired,const Knot& base){
    const auto c=dotJet(upJet(desired),upJet(base)),s=dotJet(upJet(desired),crossJet(tangentJet(base),upJet(base)));
    using C=std::complex<double>;const C z{c[0],s[0]},first=C(c[1],s[1])/z,second=C(c[2],s[2])/z;
    return {std::arg(z),first.imag(),(second-first*first).imag(),(C(c[3],s[3])/z-3.*second*first+2.*first*first*first).imag()};
}
void rotateFrame(Knot& knot,detail::AngleJet angle){
    const auto up=upJet(knot),right=crossJet(tangentJet(knot),up);
    const auto cosine=detail::cosJet(angle),sine=detail::sinJet(angle);
    const std::array<double,4> c{cosine.value,cosine.first,cosine.second,cosine.third},s{sine.value,sine.first,sine.second,sine.third};FrameJet result{};
    for(int n=0;n<4;++n)for(int i=0;i<=n;++i)result[n]=result[n]+(up[i]*c[n-i]+right[i]*s[n-i])*choose[n][i];
    knot.up=result[0];knot.upFirst=result[1];knot.upSecond=result[2];knot.upThird=result[3];
}
void retainAuthoredFrames(Design& d,const std::vector<Knot>& authored,const std::vector<bool>& owned,const std::vector<double>& distance,const std::vector<Frame>& frames){
    auto& knots=d.track.knots;
    // The unconstrained spline frame supplies the interior target. Exact FVD
    // boundary jets enter/release through short C3 angular corrections within
    // the neighbouring spline, so smoothing never rewrites an authored roll.
    for(size_t i=0;i<knots.size();){
        if(owned[i]){++i;continue;}const size_t begin=i;while(i<knots.size()&&!owned[i])++i;const size_t end=i;
        const size_t left=begin?begin-1:0,right=end<knots.size()?end:knots.size()-1;
        const double length=distance[right]-distance[left];if(length<=0)continue;
        const bool hasLeft=begin>0&&owned[left],hasRight=end<knots.size()&&owned[right];
        const double leftWidth=std::min(length*.5,std::max(20.,1.2*replayValueAt(frames,distance[left]))),rightWidth=std::min(length*.5,std::max(20.,1.2*replayValueAt(frames,distance[right])));
        const auto a=detail::anglePolynomial(hasLeft?frameDifference(authored[left],knots[left]):detail::AngleJet{}, {},leftWidth);
        const auto b=detail::anglePolynomial({},hasRight?frameDifference(authored[right],knots[right]):detail::AngleJet{},rightWidth);
        for(size_t k=begin;k<end;++k){
            if(hasLeft&&distance[k]-distance[left]<leftWidth)rotateFrame(knots[k],detail::angleAt(a,distance[k]-distance[left],leftWidth));
            else if(hasRight&&distance[right]-distance[k]<rightWidth)rotateFrame(knots[k],detail::angleAt(b,rightWidth-distance[right]+distance[k],rightWidth));
        }
    }
    for(size_t i=0;i<knots.size();++i)if(owned[i]){knots[i].up=authored[i].up;knots[i].upFirst=authored[i].upFirst;knots[i].upSecond=authored[i].upSecond;knots[i].upThird=authored[i].upThird;}
    if(d.track.closed)knots.back()=knots.front();d.track.authoredFrame=true;d.track.rebuild();
}
}

std::vector<double> continuousRollTarget(const std::vector<double>& upright,const std::vector<double>& reference,size_t crest,double bank) {
    std::vector<double> result(upright.size());
    result[crest]=reference[crest]+std::remainder(upright[crest]+bank-reference[crest],2*pi);
    for(size_t i=crest;i-->0;)result[i]=result[i+1]+std::remainder(upright[i]+bank-result[i+1],2*pi);
    for(size_t i=crest+1;i<result.size();++i)result[i]=result[i-1]+std::remainder(upright[i]+bank-result[i-1],2*pi);
    return result;
}

void authorBanking(Design& d,const std::vector<Frame>& frames) {
    if(frames.empty())return;
    auto authoredKnots=d.track.knots;
    const size_t count=d.track.knots.size();
    std::vector<Vec3> reference(count);
    std::vector<double> distance(count),time(count),target(count),forceAngle(count),load(count),uprightAngle(count);
    std::vector<bool> hardware(count),authoredOrientation(count);
    Vec3 up=d.track.knots.front().up;
    double previousSpeed=0;
    const double margin=(d.request.train.cars-1)*d.request.train.spacing+1.5;
    for(size_t i=0;i<count;++i) {
        const auto& k=d.track.knots[i];
        distance[i]=i+1==count?d.track.length:d.track.spans[i].start;
        const double speed=replayValueAt(frames,distance[i]);
        if(i) {
            const Vec3 axis=cross(d.track.knots[i-1].tangent,k.tangent);
            if(norm(axis)>1e-12)up=rotate(up,unit(axis),std::atan2(norm(axis),dot(d.track.knots[i-1].tangent,k.tangent)));
            time[i]=time[i-1]+(distance[i]-distance[i-1])/std::max(5.,(speed+previousSpeed)*.5);
        }
        previousSpeed=speed;
        reference[i]=up=unit(up-k.tangent*dot(up,k.tangent));
        const Vec3 upright=unit(Vec3{0,0,1}-k.tangent*k.tangent.z);
        const Vec3 required=k.curvature*(speed*speed)+Vec3{0,0,gravity};
        const double normal=dot(required,upright),lateral=dot(required,cross(k.tangent,upright));
        load[i]=norm(required-k.tangent*dot(required,k.tangent))/gravity;
        authoredOrientation[i]=false;
        for(const auto& source:d.forcePrograms)if(i>=source.firstKnot&&i<source.firstKnot+source.sourceDistances.size())authoredOrientation[i]=true;
        // Alignment track remains upright throughout the corridor, including
        // its unpowered lead-in before the first stator or brake fin.
        hardware[i]=k.element==Element::Station||k.element==Element::Launch||k.element==Element::Brake||
            distance[i]<margin||distance[i]>d.track.length-margin;
        if(!authoredOrientation[i]&&hardware[i]&&norm(k.curvature)+norm(k.third)+norm(k.fourth)==0){
            // A straight corridor has an exact constant upright frame. Keep
            // all its jets, so a neighbouring bank cannot leak through the
            // global fit and its subsequent C3 boundary correction.
            authoredOrientation[i]=true;auto& fixed=authoredKnots[i];
            fixed.up=upright;fixed.upFirst=fixed.upSecond=fixed.upThird={};
        }
        for(const auto& op:d.operations) {
            const double end=op.kind==DriveKind::Station?d.track.length:op.end;
            // A quintic control influences three time cells either side.
            // Reserve that support plus the canonical derivative stencil so
            // the complete physical train is aligned when propulsion acts.
            const double programmeMargin=margin+.4*speed+1;
            hardware[i]=hardware[i]||(distance[i]>=op.start-programmeMargin&&distance[i]<=end+programmeMargin);
        }
        auto angle=[&](Vec3 desired){return std::atan2(dot(desired,cross(k.tangent,up)),dot(desired,up));};
        // Negative normal load reverses the useful banking direction. A signed
        // regularized target passes smoothly through upright at zero normal
        // load instead of forcing an antipodal roll branch during airtime.
        const double bank=std::atan2(lateral*normal,normal*normal+.09*gravity*gravity);
        Vec3 desired=authoredOrientation[i]?authoredKnots[i].up:rotate(upright,k.tangent,bank);
        if(hardware[i])desired=upright;
        const double raw=angle(desired);
        target[i]=i?target[i-1]+std::remainder(raw-target[i-1],2*pi):raw;
        const Vec3 projected=required-k.tangent*dot(required,k.tangent);
        const double requiredAngle=angle(unit(projected));
        forceAngle[i]=target[i]+std::remainder(requiredAngle-target[i],pi);
        uprightAngle[i]=angle(upright);
    }
    struct RollPhrase {double crestTime;std::vector<double> angle;};
    std::vector<RollPhrase> phrases;
    for(const auto& source:d.splinePrograms)if(source.requestedRoll!=0) {
        const size_t crest=source.lastKnot;
        const double bank=source.hand*source.requestedRoll;
        RollPhrase p{time[crest],continuousRollTarget(uprightAngle,target,crest,bank)};
        // Choose the physical revolution once at the crest, then unwrap outwards.
        // Choosing a new nearest 2pi branch at every sample created full spins
        // when the prior inversion moved the transported reference past the cut.
        phrases.push_back(std::move(p));
    }
    const size_t controls=size_t(std::ceil(time.back()/.1))+1;
    const double step=time.back()/(controls-1);
    std::vector<double> desired(controls),weights(controls),low(controls),high(controls),values(controls);
    std::vector<int> fixed(controls);
    size_t left=0;
    for(size_t i=0;i<controls;++i) {
        const double t=step*i;
        while(left+2<count&&time[left+1]<t)++left;
        const double u=(t-time[left])/(time[left+1]-time[left]);
        auto sample=[&](const std::vector<double>& v){return v[left]+u*(v[left+1]-v[left]);};
        const double strength=sample(load),balanced=sample(target);
        weights[i]=.4+strength*strength;desired[i]=weights[i]*balanced;
        for(const auto& phrase:phrases) {
            const double weight=40*std::pow(1-ease(std::abs(t-phrase.crestTime)/2.9),2);
            desired[i]+=weight*sample(phrase.angle);weights[i]+=weight;
        }
        desired[i]/=weights[i];
        low[i]=desired[i]-pi;high[i]=desired[i]+pi;
        if(strength>1.15) {
            const double radius=std::asin(1.15/strength),center=sample(forceAngle);
            low[i]=center-radius;high[i]=center+radius;
        }
        if(authoredOrientation[left]||authoredOrientation[left+1])weights[i]*=100;
        if(hardware[left]||hardware[left+1]) {
            // A cell straddling the hardware boundary must remain upright.
            // Its balanced target already blends in the following turn; pinning
            // that mixture creates a step in the last fixed control and a large
            // roll overshoot. Interpolate the physical alignment on one branch.
            const double upright=uprightAngle[left]+u*std::remainder(uprightAngle[left+1]-uprightAngle[left],2*pi);
            const double aligned=balanced+std::remainder(upright-balanced,2*pi);
            fixed[i]=2;low[i]=high[i]=aligned;values[i]=aligned;
        }
    }
    BandMatrix matrix(controls);std::vector<double> rhs(controls);
    for(size_t i=0;i<controls;++i){matrix.at(i,i)=weights[i];rhs[i]=weights[i]*desired[i];}
    for(size_t i=0;i+1<controls;++i)matrix.penalty(i,{-1,1},.16/(step*step));
    for(size_t i=0;i+2<controls;++i)matrix.penalty(i,{1,-2,1},.08/std::pow(step,4));
    for(size_t i=0;i+3<controls;++i)matrix.penalty(i,{-1,3,-3,1},.04/std::pow(step,6));
    std::vector<double> controlsAngle(controls);bool converged=false;
    for(size_t i=0;i<controls;++i)controlsAngle[i]=fixed[i]?values[i]:std::clamp(desired[i],low[i],high[i]);
    for(size_t iteration=0;iteration<8*controls+64;++iteration) {
        const auto optimum=solve(matrix,rhs,fixed,values);
        // Feasible active-set step: stop at the first blocking bound. Clamping
        // every violated coordinate of an infeasible minimizer can cycle.
        double fraction=1;size_t blocker=controls;double bound=0;
        for(size_t i=0;i<controls;++i)if(!fixed[i]) {
            const double direction=optimum[i]-controlsAngle[i];double candidate=1,limit=0;
            if(optimum[i]<low[i]){candidate=(low[i]-controlsAngle[i])/direction;limit=low[i];}
            else if(optimum[i]>high[i]){candidate=(high[i]-controlsAngle[i])/direction;limit=high[i];}
            if(candidate<fraction){fraction=std::max(0.,candidate);blocker=i;bound=limit;}
        }
        for(size_t i=0;i<controls;++i)controlsAngle[i]+=fraction*(optimum[i]-controlsAngle[i]);
        if(blocker!=controls){fixed[blocker]=1;values[blocker]=controlsAngle[blocker]=bound;continue;}
        // Stationarity in angular-displacement units avoids comparing a badly
        // scaled raw gradient against an arbitrary absolute roundoff threshold.
        double worst=1e-10;size_t release=controls;
        for(size_t i=0;i<controls;++i)if(fixed[i]==1) {
            double gradient=-rhs[i];
            for(size_t j=i>3?i-3:0;j<std::min(controls,i+4);++j)gradient+=matrix.get(i,j)*controlsAngle[j];
            const double violation=(values[i]==low[i]?-gradient:gradient)/matrix.get(i,i);
            if(violation>worst){worst=violation;release=i;}
        }
        if(release==controls){converged=true;break;}
        fixed[release]=0;
    }
    if(!converged)throw std::runtime_error("Banking programme did not satisfy its constrained optimum");
    // One C4 programme in elapsed motion time; geometry labels never reset it.
    for(size_t i=0;i<count;++i) {
        const double x=time[i]/step;const int center=int(std::floor(x));double angle=0;
        for(int j=center-2;j<=center+3;++j)
            angle+=controlsAngle[size_t(std::clamp(j,0,int(controls)-1))]*basis(x-j);
        auto& k=d.track.knots[i];k.up=rotate(reference[i],k.tangent,angle);k.bank=0;
    }
    if(d.track.closed)d.track.knots.back()=d.track.knots.front();
    d.track.authoredFrame=false;d.track.rebuild();captureCanonicalDerivatives(d.track);
    if(std::any_of(authoredOrientation.begin(),authoredOrientation.end(),[](bool owned){return owned;}))retainAuthoredFrames(d,authoredKnots,authoredOrientation,distance,frames);
}
}
