#pragma once
#include "coaster/coaster.hpp"
#include <stdexcept>
#include <array>
namespace coaster::detail {
enum class TerrainPortContract { LevelAuthored, PreserveIncoming };
struct TerrainTransferInfeasible : std::runtime_error {using std::runtime_error::runtime_error;};
// Read height derivatives only from the preserved side of a replaced span.
// Interior points may still contain the old descent or a station height jump.
inline std::array<double,3> terrainPortHeightJet(const std::vector<AuthoredPoint>& points,size_t index,bool incoming){
    if((incoming&&index<8)||(!incoming&&index+8>=points.size()))throw std::invalid_argument("Terrain port requires eight preserved-side guards");
    const size_t first=incoming?index-8:index;
    const auto guard=compile(std::vector<AuthoredPoint>(points.begin()+first,points.begin()+first+9),false);
    const auto port=sampleSpanKinematics(guard,incoming?7:0,incoming?1:0);
    const double z=port.sample.tangent.z,h=std::hypot(port.sample.tangent.x,port.sample.tangent.y),k=port.sample.curvature.z;
    if(h<.1)throw std::runtime_error("Terrain port is outside the horizontal-transfer domain");
    return {z/h,k/std::pow(h,4),port.curvatureS.z/std::pow(h,5)+4*z*k*k/std::pow(h,7)};
}
struct TerrainTransfer {
    double start{},finish{},length{},timing{1};
    std::array<double,8> coefficients{};bool usesBoundaryJets{};
    double progress(double u) const {
        if(u<=0)return 0;if(u>=1)return 1;
        const double w=finish>=start?-std::expm1(timing*std::log1p(-u)):std::exp(timing*std::log(u));
        const double t=std::min(w,1-w),value=t*t*t*t*(35+t*(-84+t*(70-20*t)));
        return w>.5?1-value:value;
    }
    double height(double s) const {
        if(s<=0)return start;if(s>=length)return finish;
        if(!usesBoundaryJets)return start+(finish-start)*progress(s/length);
        const double u=s/length;double value=coefficients.back();
        for(size_t i=coefficients.size()-1;i-->0;)value=value*u+coefficients[i];return value;
    }
};
// A terrain transfer is a monotone climb/descent with level C3 ports. Moving
// its timing clears an escarpment without raising an artificial summit above
// either port. This authoring family cannot cross a ridge higher than both
// ports; a different placement is required in that case. XY remains authored.
inline TerrainTransfer fitTerrainTransfer(const std::vector<double>& distance,const std::vector<double>& floor,
    double start,double finish,double maxGrade,double maxVerticalCurvature,Cancel cancel={}){
    if(distance.size()<2||distance.size()!=floor.size()||distance.front()!=0||!std::isfinite(start)||!std::isfinite(finish)||
       !std::isfinite(maxGrade)||maxGrade<=0||!std::isfinite(maxVerticalCurvature)||maxVerticalCurvature<=0)
        throw std::invalid_argument("Invalid terrain transfer inputs");
    for(size_t i=0;i<distance.size();++i)if(!std::isfinite(distance[i])||!std::isfinite(floor[i])||(i&&distance[i]<=distance[i-1]))
        throw std::invalid_argument("Terrain transfer needs increasing finite horizontal stations");
    TerrainTransfer result{start,finish,distance.back(),1};
    if(floor.front()>start+1e-8||floor.back()>finish+1e-8)throw TerrainTransferInfeasible("Terrain transfer cannot clear its fixed port");
    for(size_t i=1;i+1<distance.size();++i){
        if((i&63)==0&&cancel&&cancel())throw std::runtime_error("CANCELLED");
        if(floor[i]<=result.height(distance[i]))continue;
        if(floor[i]>=std::max(start,finish))throw TerrainTransferInfeasible("Terrain transfer needs a higher port or another route");
        const double target=(floor[i]-start)/(finish-start),u=distance[i]/result.length;
        double lo=0,hi=1;
        for(int j=0;j<52;++j){double w=(lo+hi)*.5,value=w*w*w*w*(35+w*(-84+w*(70-20*w)));if(value<target)lo=w;else hi=w;}
        // Every interior height increases monotonically with timing, for both
        // climbing and descending transfers. The largest required timing is
        // therefore the least departure from the original S7 transfer.
        const double w=(lo+hi)*.5;
        const double timing=finish>start?std::log1p(-w)/std::log1p(-u):std::log(w)/std::log(u);
        result.timing=std::max(result.timing,timing*(1+1e-12));
    }
    const double p=result.timing,rise=std::abs(finish-start);
    // Exact maximum slope of S7(u^p); ascent is its reflected complement.
    const double peakW=(4*p-1)/(7*p-1),peakU=std::pow(peakW,1/p);
    const double grade=rise/result.length*140*p*std::pow(peakU,4*p-1)*std::pow(1-peakW,3);
    if(!std::isfinite(p)||!std::isfinite(grade)||grade>maxGrade)
        throw TerrainTransferInfeasible("Terrain transfer timing exceeds its grade budget");
    // Sample uniformly in warped progress so compressed timing cannot hide
    // its curvature in a tiny interval of the original parameter. This is a
    // planning radius screen; canonical swept clearance and both simulation
    // rates remain authoritative for actual geometry and rider forces.
    for(int i=1;i<1024;++i){
        if((i&63)==0&&cancel&&cancel())throw std::runtime_error("CANCELLED");
        double w=double(i)/1024,u=std::pow(w,1/p);
        double slope=rise/result.length*140*p*std::pow(u,4*p-1)*std::pow(1-w,3);
        double second=rise/(result.length*result.length)*140*p*std::pow(u,4*p-2)*std::pow(1-w,2)*((4*p-1)-(7*p-1)*w);
        double curvature=std::abs(second)/std::pow(1+slope*slope,1.5);
        if(!std::isfinite(curvature)||curvature>maxVerticalCurvature)
            throw TerrainTransferInfeasible("Terrain transfer timing exceeds its vertical curvature budget");
    }
    for(size_t i=0;i<distance.size();++i)if(result.height(distance[i])+1e-7<floor[i])
        throw std::runtime_error("Terrain transfer did not clear its sampled envelope");
    return result;
}
// A nonlevel incoming/outgoing port must retain its height derivatives with
// respect to horizontal path distance. A single septic owns the full transfer;
// infeasible floor, monotonicity or authoring bounds require another route.
inline TerrainTransfer fitJetTerrainTransfer(const std::vector<double>& distance,const std::vector<double>& floor,
    double start,double finish,const std::array<double,3>& startJet,const std::array<double,3>& finishJet,
    double maxGrade,double maxVerticalCurvature,Cancel cancel={}){
    if(distance.size()<2||distance.size()!=floor.size()||distance.front()!=0||!std::isfinite(start)||!std::isfinite(finish)||
       !std::isfinite(maxGrade)||maxGrade<=0||!std::isfinite(maxVerticalCurvature)||maxVerticalCurvature<=0)
        throw std::invalid_argument("Invalid jet terrain transfer inputs");
    for(double value:startJet)if(!std::isfinite(value))throw std::invalid_argument("Invalid start height jets");
    for(double value:finishJet)if(!std::isfinite(value))throw std::invalid_argument("Invalid finish height jets");
    for(size_t i=0;i<distance.size();++i)if(!std::isfinite(distance[i])||!std::isfinite(floor[i])||(i&&distance[i]<=distance[i-1]))
        throw std::invalid_argument("Jet terrain transfer needs increasing finite horizontal stations");
    TerrainTransfer result{start,finish,distance.back(),1};result.usesBoundaryJets=true;
    const double length=result.length;auto& c=result.coefficients;
    c[0]=start;c[1]=startJet[0]*length;c[2]=startJet[1]*length*length/2;c[3]=startJet[2]*length*length*length/6;
    const double p=finish-c[0]-c[1]-c[2]-c[3],v=finishJet[0]*length-c[1]-2*c[2]-3*c[3];
    const double a=finishJet[1]*length*length-2*c[2]-6*c[3],j=finishJet[2]*length*length*length-6*c[3];
    c[4]=35*p-15*v+2.5*a-j/6;c[5]=-84*p+39*v-7*a+j/2;c[6]=70*p-34*v+6.5*a-j/2;c[7]=-20*p+10*v-2*a+j/6;
    for(double value:c)if(!std::isfinite(value))throw std::invalid_argument("Jet terrain transfer coefficient overflow");
    if(floor.front()>start+1e-8||floor.back()>finish+1e-8)throw TerrainTransferInfeasible("Jet terrain transfer cannot clear its fixed port");
    // These low-degree authoring screens precede the unchanged canonical
    // geometry, swept terrain and complete-train force/jerk gates.
    for(int sample=0;sample<=1024;++sample){
        if((sample&63)==0&&cancel&&cancel())throw std::runtime_error("CANCELLED");
        const double u=double(sample)/1024;double slope=0,second=0;
        for(size_t k=8;k-->1;)slope=slope*u+k*c[k];
        for(size_t k=8;k-->2;)second=second*u+k*(k-1)*c[k];
        slope/=length;second/=length*length;
        const double signedGrade=finish>start?slope:finish<start?-slope:-std::abs(slope);
        if(signedGrade< -1e-8)throw TerrainTransferInfeasible("Jet terrain transfer reverses its intended grade");
        if(std::abs(slope)>maxGrade||std::abs(second)/std::pow(1+slope*slope,1.5)>maxVerticalCurvature)
            throw TerrainTransferInfeasible("Jet terrain transfer exceeds its grade or vertical curvature budget");
    }
    for(size_t i=0;i<distance.size();++i)if(result.height(distance[i])+1e-7<floor[i])
        throw TerrainTransferInfeasible("Jet terrain transfer does not clear its sampled envelope");
    return result;
}

}
