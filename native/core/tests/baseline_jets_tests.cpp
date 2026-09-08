#include "../src/baseline_jets.hpp"
#include <iostream>
#include <stdexcept>
using namespace coaster;
namespace {
int checks=0;
void check(bool value,const char* message){++checks;if(!value)throw std::runtime_error(message);}
using Polynomial=std::array<double,8>;
Polynomial cell(double first,double last,Vec3 left,Vec3 right){
    Polynomial c{first,left.x,left.y/2,left.z/6};double p=last-c[0]-c[1]-c[2]-c[3],v=right.x-c[1]-2*c[2]-3*c[3],a=right.y-2*c[2]-6*c[3],j=right.z-6*c[3];
    c[4]=35*p-15*v+2.5*a-j/6;c[5]=-84*p+39*v-7*a+j/2;c[6]=70*p-34*v+6.5*a-j/2;c[7]=-20*p+10*v-2*a+j/6;return c;
}
double derivative(const Polynomial& c,int order,double u){double value=0;for(int i=7;i>=order;--i){double factor=1;for(int j=0;j<order;++j)factor*=i-j;value=value*u+c[i]*factor;}return value;}
void verifyJoins(const std::vector<double>& heights,const std::vector<bool>& fixed,const std::vector<Vec3>& jets){
    const size_t n=heights.size();for(size_t i=0;i<n;++i){if(fixed[i])check(norm(jets[i])==0,"Every fixed plateau retains exactly zero first/second/third jets");auto left=cell(heights[(i+n-1)%n],heights[i],jets[(i+n-1)%n],jets[i]);auto right=cell(heights[i],heights[(i+1)%n],jets[i],jets[(i+1)%n]);
        check(std::abs(derivative(left,0,1)-heights[i])<1e-8,"Hard terrain control heights remain interpolation constraints");
        for(int order=0;order<=3;++order)check(std::abs(derivative(left,order,1)-derivative(right,order,0))<1e-7,"All baseline joins retain C3 continuity");
        if(!fixed[i])for(int order=4;order<=6;++order)check(std::abs(derivative(left,order,1)-derivative(right,order,0))<1e-5,"Variationally free knots match fourth through sixth derivatives");
    }
}
}
int main(){try{
    double worst=0;
    for(int count:{7,13,25,51}){
        std::vector<double> heights(count);std::vector<bool> fixed(count);fixed[0]=fixed[count-1]=true;double length=count-1.;
        for(int i=0;i<count;++i){double u=i/length;heights[i]=u*u*u*u*(35+u*(-84+u*(70-20*u)));}
        auto jets=detail::minimumSnapJets(heights,fixed,{});verifyJoins(heights,fixed,jets);
        for(int i=0;i<count;++i){double u=i/length;Vec3 exact{140*u*u*u*std::pow(1-u,3)/length,420*u*u*std::pow(1-u,2)*(1-2*u)/(length*length),840*u*(1-u)*(1-5*u+5*u*u)/(length*length*length)};double error=norm(jets[i]-exact);worst=std::max(worst,error);check(error<1e-7,"A global minimum-snap S7 polynomial must reproduce its exact analytic jets");}
        auto transformed=heights;for(auto& value:transformed)value=73+4.2*value;auto scaled=detail::minimumSnapJets(transformed,fixed,{});
        for(int i=0;i<count;++i)check(norm(scaled[i]-jets[i]*4.2)<1e-7,"Vertical translation leaves derivatives unchanged and geometric scaling scales jets");
        std::fill(heights.begin(),heights.end(),5);for(auto jet:detail::minimumSnapJets(heights,fixed,{}))check(norm(jet)==0,"A constant datum has exactly zero jets");
    }
    std::vector<double> terrain(32);std::vector<bool> fixed(32);for(int i=0;i<32;++i){terrain[i]=10*std::sin(i*.21)+.04*i*i;fixed[i]=i<3||(i>=12&&i<=16)||i>=30;if(i>=12&&i<=16)terrain[i]=14;}
    verifyJoins(terrain,fixed,detail::minimumSnapJets(terrain,fixed,{}));
    bool cancelled=false;try{detail::minimumSnapJets(terrain,fixed,[]{return true;});}catch(const std::exception& e){cancelled=std::string(e.what())=="CANCELLED";}check(cancelled,"Cancellation propagates from the bounded block solve");
    for(int fault=0;fault<3;++fault){detail::BaselineBlock matrix{};matrix[0][0]=matrix[1][1]=matrix[2][2]=1;if(fault==0)matrix[1][1]=0;if(fault==1)matrix[1][1]=-1;if(fault==2)matrix[0][0]=NAN;bool rejected=false;try{detail::invertBaselineBlock(matrix);}catch(const std::exception&){rejected=true;}check(rejected,"Singular, indefinite and nonfinite normal blocks fail explicitly");}
    bool invalid=false;terrain[5]=INFINITY;try{detail::minimumSnapJets(terrain,fixed,{});}catch(const std::exception&){invalid=true;}check(invalid,"Nonfinite control heights fail explicitly");
    std::cout<<"PASS "<<checks<<" minimum-snap analytic polynomial, C3/C6 joins, fixed plateaus, physical scaling, cancellation and SPD failure checks; worst jet error="<<worst<<'\n';return 0;
}catch(const std::exception& e){std::cerr<<"FAIL after "<<checks<<": "<<e.what()<<'\n';return 1;}}
