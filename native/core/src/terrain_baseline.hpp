#pragma once
#include "coaster/coaster.hpp"
#include "terrain_transfer.hpp"
#include "passive_transfer.hpp"
#include <stdexcept>
#include <sstream>
#include <numeric>
#include <set>

namespace coaster::detail {
using BaselineJet=std::array<double,4>;
using BaselinePolynomial=std::array<double,8>;
inline BaselinePolynomial baselineCell(const BaselineJet& left,const BaselineJet& right){
    BaselinePolynomial c{left[0],left[1],left[2]/2,left[3]/6};
    const double p=right[0]-c[0]-c[1]-c[2]-c[3],v=right[1]-c[1]-2*c[2]-3*c[3],a=right[2]-2*c[2]-6*c[3],j=right[3]-6*c[3];
    c[4]=35*p-15*v+2.5*a-j/6;c[5]=-84*p+39*v-7*a+j/2;c[6]=70*p-34*v+6.5*a-j/2;c[7]=-20*p+10*v-2*a+j/6;return c;
}
inline double baselineDerivative(const BaselinePolynomial& c,double u,int order=0){
    double result=0;for(int i=7;i>=order;--i){double factor=1;for(int j=0;j<order;++j)factor*=i-j;result=result*u+c[i]*factor;}return result;
}
struct ConstrainedBaseline {
    static constexpr double maximumSpacing=50;
    double length{},spacing{};std::vector<BaselineJet> nodes;
    BaselineJet jet(double distance) const {
        const double at=std::clamp(distance/spacing,0.,double(nodes.size()-1));
        const size_t cell=std::min(nodes.size()-2,size_t(at));const auto polynomial=baselineCell(nodes[cell],nodes[cell+1]);
        BaselineJet result{};for(int order=0;order<4;++order)result[order]=baselineDerivative(polynomial,at-cell,order)/std::pow(spacing,order);
        return result;
    }
    double height(double distance) const {return jet(distance)[0];}
};

struct BaselineAnchor {size_t first,last;double minimumHeight;bool fixed;double maximumHeight{INFINITY};bool stationDatum{};};
// These spatial derivative bounds are authoring constraints, not a replacement
// for the actual finite-train force and force-rate assessment.
struct BaselineMotionBounds {
    size_t first,last;double maximumGrade,minimumSecond,maximumSecond,maximumThird;
};
struct BaselineCrossing {size_t under,over;double underFraction,overFraction,minimumSeparation;bool chooseOrder{};};
struct BaselineTransport {size_t first,last;double entrySpeed,minimumExitSpeed,maximumExitSpeed;size_t capacityEnd{};double speedCapacity{INFINITY};};
struct TerrainBaseline {
    struct Source {double begin,end,height;};
    struct Gap {double begin,end;ConstrainedBaseline profile;};
    std::vector<Source> sources;std::vector<Gap> gaps;
    double height(double distance) const {
        for(const auto& source:sources)if(distance>=source.begin&&distance<=source.end)return source.height;
        for(const auto& gap:gaps)if(distance>gap.begin&&distance<gap.end)return gap.profile.height(distance-gap.begin);
        throw std::runtime_error("Terrain baseline queried outside its declared domains");
    }
};
// Preserve the solved derivative information when adding a terrain baseline
// to an authored curve. Re-estimating these jets from sampled heights can
// introduce curvature-rate spikes near a C3 cell boundary.
inline Knot addBaselineJet(Knot knot,const BaselineJet& height){
    const Vec3 first=knot.tangent+Vec3{0,0,height[1]},second=knot.curvature+Vec3{0,0,height[2]};
    knot.position.z+=height[0];knot.tangent=unit(first);
    knot.curvature=(second-knot.tangent*dot(knot.tangent,second))/dot(first,first);
    knot.up=unit(knot.up-knot.tangent*dot(knot.up,knot.tangent));return knot;
}
struct BaselineConstraint {
    std::vector<std::pair<int,double>> terms;double offset,scale;const char* kind;double distance;
    BaselineConstraint(const std::vector<double>& row,double bound,double priority,const char* label="component",double at=0)
        :offset(bound),scale(priority),kind(label),distance(at){
        for(size_t i=0;i<row.size();++i)if(row[i]!=0)terms.emplace_back(int(i),row[i]);
    }
};
using BaselineTransform=std::function<std::vector<double>(std::vector<double>)>;
// Minimum-norm point in whitened coordinates. Keep the accepted thin QR:
// rebuilding nearly dependent rows after a deletion can discard known rank.
inline std::vector<double> projectBaselineConstraints(const std::vector<BaselineConstraint>& constraints,int count,Cancel cancel={},
    const BaselineTransform& forward={},const BaselineTransform& backward={}){
    constexpr double tolerance=1e-8,rankTolerance=1e-12;
    const auto checkCancelled=[&]{if(cancel&&cancel())throw std::runtime_error("CANCELLED");};
    const auto dot=[](const std::vector<double>& a,const std::vector<double>& b){double value=0;for(size_t i=0;i<a.size();++i)value+=a[i]*b[i];return value;};
    std::vector<double> y(count),multipliers;std::vector<size_t> active;
    std::vector<std::vector<double>> q,r;
    std::set<std::vector<size_t>> visited;
    for(;;){
        checkCancelled();size_t selected=0;double violation=tolerance;
        const auto physical=backward?backward(y):y;
        for(size_t i=0;i<constraints.size();++i){if((i&127)==0)checkCancelled();const auto& c=constraints[i];double value=0;
            for(const auto [index,coefficient]:c.terms)value+=coefficient*physical[index];const double deficit=(c.offset-value)*c.scale;
            if(!std::isfinite(deficit))throw std::runtime_error("Nonfinite constrained baseline iterate");
            if(deficit>violation){violation=deficit;selected=i;}}
        if(violation==tolerance)return y;
        // A feasible certificate ends the solve. A repeated active set has
        // the same unique minimum-norm point and signals numerical cycling.
        // An arbitrary iteration count cannot certify a larger input invalid.
        auto identity=active;std::sort(identity.begin(),identity.end());
        if(!visited.insert(std::move(identity)).second)throw std::runtime_error("Terrain baseline active set cycled before its feasible certificate");
        const auto& incoming=constraints[selected];double pending=0;bool added=false;
        std::vector<double> normal(count);for(const auto [index,value]:incoming.terms)normal[index]=value;
        if(forward)normal=forward(std::move(normal));const double magnitude=std::sqrt(dot(normal,normal));
        for(auto& value:normal)value/=magnitude;const double offset=incoming.offset/magnitude;
        for(int pivot=0;pivot<=count;++pivot){
            checkCancelled();const size_t n=active.size();
            std::vector<double> direction=normal,projection(n);
            for(int pass=0;pass<2;++pass)for(size_t i=0;i<n;++i){const double p=dot(q[i],direction);projection[i]+=p;for(int k=0;k<count;++k)direction[k]-=p*q[i][k];}
            auto dualDirection=projection;
            for(size_t i=n;i-->0;){for(size_t j=i+1;j<n;++j)dualDirection[i]-=r[i][j]*dualDirection[j];dualDirection[i]/=r[i][i];}
            const double denominator=dot(direction,direction),primal=denominator>rankTolerance*rankTolerance?(offset-dot(normal,y))/denominator:INFINITY;
            double dual=INFINITY;size_t leaving=0;
            for(size_t i=0;i<n;++i)if(dualDirection[i]>0&&multipliers[i]/dualDirection[i]<dual){dual=multipliers[i]/dualDirection[i];leaving=i;}
            const double step=std::min(primal,dual);
            if(!std::isfinite(step)){
                std::ostringstream message;message<<"Terrain baseline infeasible: "<<incoming.kind<<" at "<<incoming.distance<<" m conflicts with";
                std::vector<size_t> witness(n);std::iota(witness.begin(),witness.end(),0);
                std::sort(witness.begin(),witness.end(),[&](size_t a,size_t b){return std::abs(dualDirection[a])>std::abs(dualDirection[b]);});
                for(size_t i=0;i<std::min(size_t(6),n);++i){const auto& row=constraints[active[witness[i]]];message<<' '<<row.kind<<" at "<<row.distance<<" m ("<<dualDirection[witness[i]]<<')';}
                throw TerrainTransferInfeasible(message.str());
            }
            if(step<0)throw std::runtime_error("Constrained baseline lost numerical dual feasibility");
            for(int k=0;k<count;++k)y[k]+=step*direction[k];
            for(size_t i=0;i<n;++i)multipliers[i]-=step*dualDirection[i];pending+=step;
            if(primal<=dual){
                const double length=std::sqrt(denominator);
                for(auto& value:direction)value/=length;q.push_back(std::move(direction));
                for(size_t i=0;i<n;++i)r[i].push_back(projection[i]);
                r.push_back(std::vector<double>(n+1));r.back().back()=length;
                active.push_back(selected);multipliers.push_back(pending);added=true;break;
            }
            // Delete the active column, then restore triangular R with Givens
            // rotations. Apply the inverse rotations to Q's matching columns.
            // Each new diagonal is a hypot containing a retained old diagonal.
            for(auto& row:r)row.erase(row.begin()+leaving);
            for(size_t j=leaving;j+1<n;++j){
                const double length=std::hypot(r[j][j],r[j+1][j]),c=r[j][j]/length,s=r[j+1][j]/length;
                for(size_t column=j;column+1<n;++column){const double a=r[j][column],b=r[j+1][column];r[j][column]=c*a+s*b;r[j+1][column]=-s*a+c*b;}
                for(int k=0;k<count;++k){const double a=q[j][k],b=q[j+1][k];q[j][k]=c*a+s*b;q[j+1][k]=-s*a+c*b;}
            }
            q.pop_back();r.pop_back();active.erase(active.begin()+leaving);multipliers.erase(multipliers.begin()+leaving);
        }
        if(!added)throw std::runtime_error("Constrained baseline active-set pivot budget exceeded");
    }
}
// Source translations and ordinary C3 gap profiles share one constrained solve.
// Powered climbs and dives enter as rigid sources, with their shape already owned.
inline TerrainBaseline solveTerrainBaseline(const std::vector<double>& distance,const std::vector<double>& floor,
    const std::vector<double>& target,const std::vector<double>& authoredHeight,std::vector<BaselineAnchor> anchors,
    const std::vector<BaselineMotionBounds>& motionBounds={},
    const std::vector<BaselineCrossing>& crossings={},Cancel cancel={},
    const std::vector<BaselineJet>& authoredJets={},const std::vector<BaselineTransport>& transports={},const TrainConfig& train={}){
    if(distance.size()<2||floor.size()!=distance.size()||target.size()!=distance.size()||authoredHeight.size()!=distance.size()||anchors.empty())throw std::invalid_argument("Invalid terrain baseline inputs");
    constexpr double constraintTolerance=1e-8;
    const auto checkCancelled=[&]{if(cancel&&cancel())throw std::runtime_error("CANCELLED");};
    checkCancelled();
    if(!authoredJets.empty()&&authoredJets.size()!=distance.size())throw std::invalid_argument("Authored terrain jets need one value per station");
    for(size_t i=0;i<distance.size();++i)if(!std::isfinite(distance[i])||(i&&distance[i]<=distance[i-1])||!std::isfinite(floor[i])||!std::isfinite(target[i])||!std::isfinite(authoredHeight[i]))throw std::invalid_argument("Invalid terrain baseline station");
    for(auto& source:anchors){if(source.first>source.last||source.last>=distance.size()||!std::isfinite(source.minimumHeight)||std::isnan(source.maximumHeight))throw std::invalid_argument("Invalid terrain baseline source");if(source.fixed)source.maximumHeight=std::min(source.maximumHeight,source.minimumHeight);if(source.stationDatum)source.fixed=false;}
    std::sort(anchors.begin(),anchors.end(),[](const auto& a,const auto& b){return a.first<b.first;});
    for(size_t i=1;i<anchors.size();++i)if(anchors[i].first<=anchors[i-1].last)throw std::invalid_argument("Overlapping terrain baseline sources");
    for(auto& source:anchors){const double required=*std::max_element(floor.begin()+source.first,floor.begin()+source.last+1);
        if(source.fixed&&required>source.minimumHeight)throw TerrainTransferInfeasible("Fixed source violates its terrain floor");
        source.minimumHeight=std::max(source.minimumHeight,required);
        if(source.minimumHeight>source.maximumHeight)throw TerrainTransferInfeasible("Source terrain floor exceeds its height budget");
    }
    TerrainBaseline result;for(const auto& source:anchors)result.sources.push_back({distance[source.first],distance[source.last],source.minimumHeight});
    struct Gap {size_t left,right,resultIndex;int firstVariable,cells;std::vector<std::array<int,4>> variable;};
    std::vector<Gap> gaps;std::vector<int> sourceVariable(anchors.size(),-1);int count=0;
    if(!anchors.front().fixed&&!anchors.front().stationDatum)sourceVariable.front()=count++;
    for(size_t i=1;i<anchors.size();++i){const size_t first=anchors[i-1].last,last=anchors[i].first;
        const double length=distance[last]-distance[first];const int cells=std::max(3,int(std::ceil(length/ConstrainedBaseline::maximumSpacing)));
        const size_t index=result.gaps.size();result.gaps.push_back({distance[first],distance[last],{length,length/cells,std::vector<BaselineJet>(cells+1)}});
        gaps.push_back({i-1,i,index,count,cells,{}});count+=4*(cells-1);
        if(!anchors[i].fixed&&!anchors[i].stationDatum)sourceVariable[i]=count++;
    }
    const bool sharedStation=std::any_of(anchors.begin(),anchors.end(),[](const BaselineAnchor& source){return source.stationDatum;});
    const int bandCount=count,stationVariable=sharedStation?count++:-1;
    if(sharedStation)for(size_t i=0;i<anchors.size();++i)if(anchors[i].stationDatum)sourceVariable[i]=stationVariable;
    for(const auto& bounds:motionBounds)if(bounds.first>=bounds.last||bounds.last>=distance.size()||!std::isfinite(bounds.maximumGrade)||bounds.maximumGrade<=0||!std::isfinite(bounds.minimumSecond)||!std::isfinite(bounds.maximumSecond)||bounds.minimumSecond>=bounds.maximumSecond||!std::isfinite(bounds.maximumThird)||bounds.maximumThird<=0)throw std::invalid_argument("Invalid terrain baseline motion bounds");
    for(const auto& crossing:crossings)if(crossing.under+1>=distance.size()||crossing.over+1>=distance.size()||!std::isfinite(crossing.underFraction)||crossing.underFraction<0||crossing.underFraction>1||!std::isfinite(crossing.overFraction)||crossing.overFraction<0||crossing.overFraction>1||!std::isfinite(crossing.minimumSeparation)||crossing.minimumSeparation<=0)throw std::invalid_argument("Invalid terrain baseline crossing");
    for(const auto& transport:transports)if(transport.first>=transport.last||transport.last>=distance.size()||!std::isfinite(transport.entrySpeed)||transport.entrySpeed<=0||!std::isfinite(transport.minimumExitSpeed)||transport.minimumExitSpeed<0||std::isnan(transport.maximumExitSpeed)||transport.maximumExitSpeed<transport.minimumExitSpeed)throw std::invalid_argument("Invalid terrain baseline passive transport");
    std::array<BaselinePolynomial,8> basis;
    for(int i=0;i<8;++i){BaselineJet left{},right{};(i<4?left:right)[i%4]=1;basis[i]=baselineCell(left,right);}
    auto dot=[](const std::vector<double>& a,const std::vector<double>& b){double value=0;for(size_t i=0;i<a.size();++i)value+=a[i]*b[i];return value;};
    // Targets are absolute rail elevations everywhere. The unknown source and
    // ordinary-gap quantities are translations relative to authored geometry.
    auto interpolate=[&](double at){const size_t right=std::clamp<size_t>(std::upper_bound(distance.begin(),distance.end(),at)-distance.begin(),1,distance.size()-1);const double u=(at-distance[right-1])/(distance[right]-distance[right-1]);return (1-u)*(target[right-1]-authoredHeight[right-1])+u*(target[right]-authoredHeight[right]);};
    // The one shared station datum forms a single border around the same
    // band-seven ordinary/source system; no dense solve or outer lift loop.
    std::vector<std::array<double,8>> lower(bandCount);std::vector<double> rhs(count),border(bandCount);double stationDiagonal=0;
    auto normal=[&](int row,int column,double value){if(row==stationVariable||column==stationVariable){if(row==column)stationDiagonal+=value;else border[row==stationVariable?column:row]+=value;return;}if(row<column)std::swap(row,column);lower[row][row-column]+=value;};
    for(size_t i=0;i<anchors.size();++i)if(sourceVariable[i]>=0){const auto& source=anchors[i];
        for(size_t k=source.first;k<source.last;++k){const double weight=(distance[k+1]-distance[k])/ConstrainedBaseline::maximumSpacing;
            normal(sourceVariable[i],sourceVariable[i],weight);rhs[sourceVariable[i]]+=weight*(target[k]+target[k+1]-authoredHeight[k]-authoredHeight[k+1])*.5;
        }
    }
    constexpr std::array<double,8> abscissa{-.9602898564975363,-.7966664774136267,-.5255324099163290,-.1834346424956498,.1834346424956498,.5255324099163290,.7966664774136267,.9602898564975363};
    constexpr std::array<double,8> weight{.1012285362903763,.2223810344533745,.3137066458778873,.3626837833783620,.3626837833783620,.3137066458778873,.2223810344533745,.1012285362903763};
    auto fixedCell=[&](const Gap& gap,int cell){return baselineCell(cell==0&&anchors[gap.left].fixed?BaselineJet{anchors[gap.left].minimumHeight,0,0,0}:BaselineJet{},cell+1==gap.cells&&anchors[gap.right].fixed?BaselineJet{anchors[gap.right].minimumHeight,0,0,0}:BaselineJet{});};
    for(auto& gap:gaps){const auto& profile=result.gaps[gap.resultIndex].profile;gap.variable.resize(gap.cells+1);
        for(auto& node:gap.variable)node.fill(-1);gap.variable.front()[0]=sourceVariable[gap.left];gap.variable.back()[0]=sourceVariable[gap.right];
        for(int node=1;node<gap.cells;++node)for(int jet=0;jet<4;++jet)gap.variable[node][jet]=gap.firstVariable+4*(node-1)+jet;
        for(int cell=0;cell<gap.cells;++cell){checkCancelled();const auto fixed=fixedCell(gap,cell);
            for(size_t sample=0;sample<abscissa.size();++sample){const double u=(abscissa[sample]+1)/2,w=weight[sample]/2;
                const double goal=interpolate(distance[anchors[gap.left].last]+(cell+u)*profile.spacing),value=baselineDerivative(fixed,u),bend=baselineDerivative(fixed,u,2);
                std::array<double,8> b,d;for(int i=0;i<8;++i){b[i]=baselineDerivative(basis[i],u);d[i]=baselineDerivative(basis[i],u,2);}
                for(int i=0;i<8;++i){const int row=gap.variable[cell+i/4][i%4];if(row<0)continue;rhs[row]+=w*(b[i]*(goal-value)-d[i]*bend);
                    for(int j=0;j<=i;++j){const int column=gap.variable[cell+j/4][j%4];if(column>=0)normal(row,column,w*(b[i]*b[j]+d[i]*d[j]));}
                }
            }
        }
    }
    for(int i=0;i<bandCount;++i)for(int j=std::max(0,i-7);j<=i;++j){double value=lower[i][i-j];
        for(int k=std::max(0,i-7);k<j;++k)value-=lower[i][i-k]*lower[j][j-k];
        if(i==j){if(!std::isfinite(value)||value<=0)throw std::runtime_error("Nonpositive constrained baseline normal matrix");lower[i][0]=std::sqrt(value);}
        else lower[i][i-j]=value/lower[j][0];
    }
    if(sharedStation){for(int i=0;i<bandCount;++i){for(int j=std::max(0,i-7);j<i;++j)border[i]-=lower[i][i-j]*border[j];border[i]/=lower[i][0];}stationDiagonal-=dot(border,border);if(!std::isfinite(stationDiagonal)||stationDiagonal<=0)throw std::runtime_error("Nonpositive shared station datum normal matrix");stationDiagonal=std::sqrt(stationDiagonal);}
    auto forward=[&](std::vector<double> value){for(int i=0;i<bandCount;++i){for(int j=std::max(0,i-7);j<i;++j)value[i]-=lower[i][i-j]*value[j];value[i]/=lower[i][0];}if(sharedStation){for(int i=0;i<bandCount;++i)value[stationVariable]-=border[i]*value[i];value[stationVariable]/=stationDiagonal;}return value;};
    auto backward=[&](std::vector<double> value){if(sharedStation){value[stationVariable]/=stationDiagonal;for(int i=0;i<bandCount;++i)value[i]-=border[i]*value[stationVariable];}for(int i=bandCount;i-->0;){for(int j=i+1;j<std::min(bandCount,i+8);++j)value[i]-=lower[j][j-i]*value[j];value[i]/=lower[i][0];}return value;};
    const auto unconstrained=backward(forward(rhs));
    std::vector<BaselineConstraint> constraints;
    auto constrain=[&](const std::vector<double>& row,double bound,const char* kind="source/domain",double at=0){
        if((constraints.size()&127)==0)checkCancelled();
        const double offset=bound-dot(row,unconstrained);
        if(!std::isfinite(offset))throw std::runtime_error("Nonfinite constrained baseline constraint");
        if(std::all_of(row.begin(),row.end(),[](double value){return value==0;})){
            if(offset>constraintTolerance)throw TerrainTransferInfeasible("Fixed terrain baseline ports violate a declared constraint");return;
        }
        constraints.emplace_back(row,offset,1,kind,at);
    };

    auto choose=[](int n,int k){double value=1;for(int j=1;j<=k;++j)value*=double(n-k+j)/j;return value;};
    // Express the same polynomial and each derivative in Bernstein form over
    // a local interval. Bounding all coefficients certifies its entire span.
    auto bernstein=[&](const BaselinePolynomial& polynomial,int derivative,double u,double width,int coefficient){
        const int degree=7-derivative;double value=0,factor=1;
        for(int order=0;order<=coefficient;++order){if(order)factor*=width/order;value+=choose(coefficient,order)/choose(degree,order)*factor*baselineDerivative(polynomial,u,order+derivative);}return value;
    };
    for(size_t source=0;source<anchors.size();++source)if(sourceVariable[source]>=0){std::vector<double> row(count);row[sourceVariable[source]]=1;constrain(row,anchors[source].minimumHeight,"source floor",distance[anchors[source].first]);if(std::isfinite(anchors[source].maximumHeight)){row[sourceVariable[source]]=-1;constrain(row,-anchors[source].maximumHeight,"source ceiling",distance[anchors[source].first]);}}
    for(const auto& gap:gaps){const auto& profile=result.gaps[gap.resultIndex].profile;const double origin=distance[anchors[gap.left].last];
        for(size_t sample=anchors[gap.left].last;sample<anchors[gap.right].first;++sample){
            if((sample&63)==0)checkCancelled();
            const double first=(distance[sample]-origin)/profile.spacing,last=sample+1==anchors[gap.right].first?double(gap.cells):(distance[sample+1]-origin)/profile.spacing;
            for(double begin=first;begin<last;){const int cell=std::min(gap.cells-1,int(begin));const double end=std::min(last,double(cell+1)),u=begin-cell,width=end-begin;const auto fixed=fixedCell(gap,cell);
                for(int k=0;k<8;++k){
                    const double fraction=((begin+width*k/7)-first)/(last-first),bound=floor[sample]+fraction*(floor[sample+1]-floor[sample])-bernstein(fixed,0,u,width,k);
                    std::vector<double> row(count);for(int i=0;i<8;++i){const int j=gap.variable[cell+i/4][i%4];if(j>=0)row[j]=bernstein(basis[i],0,u,width,k);}constrain(row,bound,"clearance",distance[sample]);
                }
                begin=end;
            }
        }
    }
    for(const auto& bounds:motionBounds)for(const auto& gap:gaps){const auto& profile=result.gaps[gap.resultIndex].profile;const double origin=distance[anchors[gap.left].last];
        const size_t first=std::max(bounds.first,anchors[gap.left].last),last=std::min(bounds.last,anchors[gap.right].first);if(first>=last)continue;
        for(size_t sample=first;sample<last;++sample){const double endpoint=sample+1==anchors[gap.right].first?double(gap.cells):(distance[sample+1]-origin)/profile.spacing;
        const double sourceSpacing=distance[sample+1]-distance[sample];BaselineJet left{},right{};
        if(!authoredJets.empty()){left=authoredJets[sample];right=authoredJets[sample+1];for(int order=1;order<4;++order){left[order]*=std::pow(sourceSpacing,order);right[order]*=std::pow(sourceSpacing,order);}}
        const auto authored=baselineCell(left,right);
        for(double begin=(distance[sample]-origin)/profile.spacing;begin<endpoint;){const int cell=std::min(gap.cells-1,int(begin));const double end=std::min(endpoint,double(cell+1)),u=begin-cell,width=end-begin;const auto fixed=fixedCell(gap,cell);
            for(int order=1;order<=3;++order){const double scale=std::pow(profile.spacing,-order),minimum=order==1?-bounds.maximumGrade:(order==2?bounds.minimumSecond:-bounds.maximumThird),maximum=order==1?bounds.maximumGrade:(order==2?bounds.maximumSecond:bounds.maximumThird);
                for(int k=0;k<=7-order;++k){const double authoredBegin=(origin+begin*profile.spacing-distance[sample])/sourceSpacing,authoredWidth=(end-begin)*profile.spacing/sourceSpacing;
                    const double fixedValue=scale*bernstein(fixed,order,u,width,k)+bernstein(authored,order,authoredBegin,authoredWidth,k)/std::pow(sourceSpacing,order);std::vector<double> row(count);
                    for(int i=0;i<8;++i){const int j=gap.variable[cell+i/4][i%4];if(j>=0)row[j]=scale*bernstein(basis[i],order,u,width,k);}
                    const char* kind=order==1?"grade":order==2?"curvature":"curvature rate";
                    constrain(row,minimum-fixedValue,kind,distance[sample]);for(auto& value:row)value=-value;constrain(row,fixedValue-maximum,kind,distance[sample]);
                }
            }
            begin=end;
        }
        }
    }
    // Source and ordinary-gap evaluations are all affine
    // in this same unknown vector. A planned crossing therefore changes the
    // joint solve; it cannot translate an already-solved tail afterward.
    auto addHeight=[&](size_t sample,double weight,std::vector<double>& row,double& fixed){
        for(size_t source=0;source<anchors.size();++source)if(sample>=anchors[source].first&&sample<=anchors[source].last){fixed+=weight*authoredHeight[sample];if(sourceVariable[source]>=0)row[sourceVariable[source]]+=weight;else fixed+=weight*anchors[source].minimumHeight;return;}
        for(const auto& gap:gaps)if(sample>anchors[gap.left].last&&sample<anchors[gap.right].first){const auto& profile=result.gaps[gap.resultIndex].profile;const double at=(distance[sample]-distance[anchors[gap.left].last])/profile.spacing;const int cell=std::min(gap.cells-1,int(at));const double u=at-cell;fixed+=weight*(authoredHeight[sample]+baselineDerivative(fixedCell(gap,cell),u));for(int i=0;i<8;++i){const int variable=gap.variable[cell+i/4][i%4];if(variable>=0)row[variable]+=weight*baselineDerivative(basis[i],u);}return;}
        throw std::runtime_error("Crossing height has no declared terrain owner");
    };
    const auto crossingHeight=[&](const BaselineCrossing& crossing){std::pair<std::vector<double>,double> value{std::vector<double>(count),0};
        addHeight(crossing.over,1-crossing.overFraction,value.first,value.second);addHeight(crossing.over+1,crossing.overFraction,value.first,value.second);
        addHeight(crossing.under,crossing.underFraction-1,value.first,value.second);addHeight(crossing.under+1,-crossing.underFraction,value.first,value.second);return value;};
    for(const auto& crossing:crossings)if(!crossing.chooseOrder){const auto [row,fixed]=crossingHeight(crossing);constrain(row,crossing.minimumSeparation-fixed,"crossing",distance[crossing.over]);}
    // Mean train potential uses the same affine height owner as clearance and
    // crossings. Passive energy can therefore constrain source placement here,
    // instead of moving an airtime source and repairing its energy afterward.
    const auto meanHeight=[&](double at){std::pair<std::vector<double>,double> value{std::vector<double>(count),0};
        for(int car=0;car<train.cars;++car){const double sample=std::clamp(at+((train.cars-1)*.5-car)*train.spacing,distance.front(),distance.back());
            const size_t right=std::clamp<size_t>(std::upper_bound(distance.begin(),distance.end(),sample)-distance.begin(),1,distance.size()-1);
            const double fraction=(sample-distance[right-1])/(distance[right]-distance[right-1]);
            addHeight(right-1,(1-fraction)/train.cars,value.first,value.second);addHeight(right,fraction/train.cars,value.first,value.second);}
        return value;
    };
    for(const auto& transport:transports){
        const double begin=distance[transport.first],length=distance[transport.last]-begin;const int steps=int(std::ceil(length/2));
        auto previous=meanHeight(begin);std::vector<double> row(count);double fixed=transport.entrySpeed*transport.entrySpeed;
        const double ds=length/steps,drag=.5*train.airDensity*train.dragCdA/(train.cars*train.carMass);
        const auto step=pathEnergyStep(ds,0,-gravity*train.rollingResistance,drag);const double potential=pathEnergyStep(ds,1,0,drag).offsetSpeedSquared;
        for(int sample=1;sample<=steps;++sample){checkCancelled();auto current=meanHeight(begin+ds*sample);
            for(int i=0;i<count;++i)row[i]=step.attenuation*row[i]+potential*(current.first[i]-previous.first[i]);
            fixed=step.attenuation*fixed+step.offsetSpeedSquared+potential*(current.second-previous.second);previous=std::move(current);
            constrain(row,(sample==steps?transport.minimumExitSpeed*transport.minimumExitSpeed:0)-fixed,"minimum energy",begin+ds*sample);
            if(transport.capacityEnd>transport.first&&begin+ds*sample<=distance[transport.capacityEnd]+(train.cars-1)*train.spacing*.5){
                auto upper=row;for(auto& value:upper)value=-value;
                constrain(upper,fixed-transport.speedCapacity*transport.speedCapacity,"turn speed capacity",begin+ds*sample);
            }
        }
        if(std::isfinite(transport.maximumExitSpeed)){for(auto& value:row)value=-value;constrain(row,fixed-transport.maximumExitSpeed*transport.maximumExitSpeed,"maximum energy",distance[transport.last]);}
    }
    auto y=projectBaselineConstraints(constraints,count,cancel,forward,backward);
    // A crossing excludes a height interval. Its two orders are constrained
    // alternatives; the sign of the unconstrained optimum cannot establish
    // which order has enough physical room. Each relaxed QP supplies a lower
    // cost bound, so only a violated crossing needs to branch.
    std::vector<double> best;double bestCost=INFINITY;std::string crossingFailure;
    std::vector<bool> assigned(crossings.size());
    const auto separate=[&](auto&& self,const std::vector<double>& point)->void{
        checkCancelled();const double cost=dot(point,point);if(cost>=bestCost)return;
        for(size_t index=0;index<crossings.size();++index){const auto& crossing=crossings[index];
            if(!crossing.chooseOrder||assigned[index])continue;auto [row,fixed]=crossingHeight(crossing);
            const double difference=fixed+dot(row,unconstrained)+dot(forward(row),point);
            if(std::abs(difference)>=crossing.minimumSeparation-constraintTolerance)continue;
            if(difference<0){for(auto& value:row)value=-value;fixed=-fixed;}
            const size_t retained=constraints.size();assigned[index]=true;
            for(int order=0;order<2;++order){
                try{
                    constrain(row,crossing.minimumSeparation-fixed,"crossing",distance[crossing.over]);
                    self(self,projectBaselineConstraints(constraints,count,cancel,forward,backward));
                }catch(const TerrainTransferInfeasible& error){crossingFailure=error.what();}
                constraints.erase(constraints.begin()+retained,constraints.end());for(auto& value:row)value=-value;fixed=-fixed;
            }
            assigned[index]=false;return;
        }
        best=point;bestCost=cost;
    };
    separate(separate,y);
    if(!std::isfinite(bestCost))throw TerrainTransferInfeasible("No crossing order satisfies the shared physical constraints: "+crossingFailure);
    y=std::move(best);

    const auto correction=backward(y);std::vector<double> solution=unconstrained;for(int i=0;i<count;++i){solution[i]+=correction[i];if(!std::isfinite(solution[i]))throw std::runtime_error("Nonfinite constrained baseline solution");}
    for(size_t source=0;source<anchors.size();++source)if(sourceVariable[source]>=0)result.sources[source].height=solution[sourceVariable[source]];
    for(const auto& gap:gaps){auto& profile=result.gaps[gap.resultIndex].profile;
        for(int node=0;node<=gap.cells;++node)for(int jet=0;jet<4;++jet){const int variable=gap.variable[node][jet];if(variable>=0)profile.nodes[node][jet]=solution[variable];}
        profile.nodes.front()[0]=result.sources[gap.left].height;profile.nodes.back()[0]=result.sources[gap.right].height;
    }
    return result;
}
} // namespace coaster::detail
