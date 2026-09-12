#pragma once
#include "coaster/coaster.hpp"
#include <stdexcept>

namespace coaster::detail {
using BaselineBlock=std::array<std::array<double,3>,3>;
// Cholesky inversion of the small symmetric-positive-definite normal block.
// A nonpositive or nonfinite block is a hard numerical failure, never a clamp.
inline BaselineBlock invertBaselineBlock(const BaselineBlock& matrix){
    BaselineBlock lower{},inverse{};double scale=0;
    for(size_t i=0;i<3;++i)for(size_t j=0;j<3;++j){if(!std::isfinite(matrix[i][j]))throw std::runtime_error("Nonfinite minimum-snap baseline block");scale=std::max(scale,std::abs(matrix[i][j]));}
    for(size_t i=0;i<3;++i)for(size_t j=0;j<=i;++j){
        if(std::abs(matrix[i][j]-matrix[j][i])>1e-8*std::max(1.,scale))throw std::runtime_error("Nonsymmetric minimum-snap baseline block");
        double residual=(matrix[i][j]+matrix[j][i])*.5;for(size_t k=0;k<j;++k)residual-=lower[i][k]*lower[j][k];
        if(i==j){if(residual<=1e-12*std::max(1.,scale))throw std::runtime_error("Nonpositive minimum-snap baseline block");lower[i][j]=std::sqrt(residual);}else lower[i][j]=residual/lower[j][j];
    }
    for(size_t column=0;column<3;++column){double forward[3]{},backward[3]{};
        for(size_t i=0;i<3;++i){double rhs=i==column?1.:0.;for(size_t j=0;j<i;++j)rhs-=lower[i][j]*forward[j];forward[i]=rhs/lower[i][i];}
        for(size_t i=3;i-->0;){double rhs=forward[i];for(size_t j=i+1;j<3;++j)rhs-=lower[j][i]*backward[j];backward[i]=rhs/lower[i][i];if(!std::isfinite(backward[i]))throw std::runtime_error("Nonfinite minimum-snap baseline inverse");inverse[i][column]=backward[i];}
    }
    return inverse;
}
// Minimum integrated squared fourth derivative of the C3 septic baseline.
// Heights remain hard interpolation constraints. A fixed control has zero
// first/second/third jets; free controls solve an SPD block tridiagonal system.
// Jets are derivatives in the normalized uniform-control coordinate.
inline std::vector<Vec3> minimumSnapJets(const std::vector<double>& heights,const std::vector<bool>& fixed,Cancel cancel){
    using Matrix=BaselineBlock;
    using Fourth=std::array<double,4>;
    const size_t count=heights.size();if(count<4||fixed.size()!=count||!fixed[0])throw std::runtime_error("Baseline jet solve requires a fixed seam");
    for(double value:heights)if(!std::isfinite(value))throw std::runtime_error("Nonfinite baseline control height");
    auto q=[](double delta,const std::array<double,6>& v){
        const double p=delta-v[0]-v[1]/2-v[2]/6,d=v[3]-v[0]-v[1]-v[2]/2,a=v[4]-v[1]-v[2],j=v[5]-v[2];
        return Fourth{24*(35*p-15*d+2.5*a-j/6),120*(-84*p+39*d-7*a+j/2),360*(70*p-34*d+6.5*a-j/2),840*(-20*p+10*d-2*a+j/6)};
    };
    auto inner=[](const Fourth& a,const Fourth& b){double value=0;for(size_t i=0;i<4;++i)for(size_t j=0;j<4;++j)value+=a[i]*b[j]/double(i+j+1);return value;};
    auto mul=[](const Matrix& a,const Vec3& v){return Vec3{a[0][0]*v.x+a[0][1]*v.y+a[0][2]*v.z,a[1][0]*v.x+a[1][1]*v.y+a[1][2]*v.z,a[2][0]*v.x+a[2][1]*v.y+a[2][2]*v.z};};
    auto product=[](const Matrix& a,const Matrix& b){Matrix out{};for(size_t i=0;i<3;++i)for(size_t j=0;j<3;++j)for(size_t k=0;k<3;++k)out[i][j]+=a[i][k]*b[k][j];return out;};
    std::array<Fourth,6> basis{};for(size_t i=0;i<6;++i){std::array<double,6> v{};v[i]=1;basis[i]=q(0,v);}const Fourth unitHeight=q(1,{});
    Matrix diagonal{},off{},transpose{};Vec3 fromLeft{},fromRight{};double* left[3]={&fromLeft.x,&fromLeft.y,&fromLeft.z};double* right[3]={&fromRight.x,&fromRight.y,&fromRight.z};
    for(size_t i=0;i<3;++i){*left[i]=inner(basis[i+3],unitHeight);*right[i]=inner(basis[i],unitHeight);
        for(size_t j=0;j<3;++j){diagonal[i][j]=inner(basis[i],basis[j])+inner(basis[i+3],basis[j+3]);off[i][j]=inner(basis[i],basis[j+3]);}}
    for(size_t i=0;i<3;++i)for(size_t j=0;j<3;++j)transpose[i][j]=off[j][i];
    std::vector<Vec3> jets(count),rhs(count);std::vector<Matrix> inverses(count);
    for(size_t first=0;first<count;){if(cancel&&cancel())throw std::runtime_error("CANCELLED");if(fixed[first]){++first;continue;}size_t end=first;while(end<count&&!fixed[end])++end;
        for(size_t i=first;i<end;++i){if((i&31)==0&&cancel&&cancel())throw std::runtime_error("CANCELLED");Matrix reduced=diagonal;rhs[i]=fromLeft*(-(heights[i]-heights[(i+count-1)%count]))+fromRight*(-(heights[(i+1)%count]-heights[i]));
            if(i>first){auto elimination=product(transpose,inverses[i-1]),correction=product(elimination,off);for(size_t a=0;a<3;++a)for(size_t b=0;b<3;++b)reduced[a][b]-=correction[a][b];rhs[i]=rhs[i]-mul(elimination,rhs[i-1]);}inverses[i]=invertBaselineBlock(reduced);}
        for(size_t i=end;i-->first;){Vec3 value=rhs[i];if(i+1<end)value=value-mul(off,jets[i+1]);jets[i]=mul(inverses[i],value);if(!finite(jets[i]))throw std::runtime_error("Nonfinite minimum-snap baseline jet");}first=end;
    }
    return jets;
}

} // namespace coaster::detail
