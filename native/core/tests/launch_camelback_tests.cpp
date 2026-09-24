#include "../src/launch_camelback.hpp"
#include "signature_reference.hpp"
#include <iostream>
#include <fstream>
#include <iomanip>
#include <stdexcept>
using namespace coaster;
int main(int argc,char** argv){try{
    const auto result=designLaunchCamelback(83.5,-10*pi/180,CamelbackParameters{},gravity*.004,.0002041666666667);
    const auto& pullout=result.pullout.section;
    if(!pullout.assessment.passed||!result.camelback.section.assessment.passed)throw std::runtime_error("Canonical replay failed");
    const auto start=sampleKinematics(pullout.track,0),end=sampleKinematics(pullout.track,pullout.track.length);
    if(norm(start.sample.curvature)+norm(start.curvatureS)+norm(start.curvatureSS)>1e-7)throw std::runtime_error("LSM exit lost straight aligned geometry");
    if(end.sample.tangent.z<=0||norm(end.sample.curvature)<.001)throw std::runtime_error("Pullout must join the rising loaded shoulder");
    double prior=result.pullout.authoring.controls.front().normalG;
    for(int i=0;i<=1000;++i){const auto q=sampleFvdControl(result.pullout.authoring.controls,result.pullout.authoring.controls.back().time*i/1000.);
        if(q.normalG<prior-1e-10||q.lateralG!=0||q.rollRate!=0)throw std::runtime_error("Pullout added an unnecessary force reversal/yaw/roll");prior=q.normalG;}
    // The FF ascending shoulder is a sustained loaded phase, not merely a
    // whole-element record reached later on the descending recovery.
    double loadedDistance=0,longestLoadedDistance=0;const auto& body=result.camelback;
    const auto top=std::max_element(body.section.samples.begin(),body.section.samples.end(),[](const auto& a,const auto& b){return a.position.z<b.position.z;});
    auto loaded=[&](const FvdSample& q,double distance,const FvdRequest& program){
        if(q.forward.z>0&&sampleFvdControl(program.controls,q.time).normalG>3.65)loadedDistance+=distance;
        else loadedDistance=0;
        longestLoadedDistance=std::max(longestLoadedDistance,loadedDistance);
    };
    for(auto q=pullout.samples.begin()+1;q!=pullout.samples.end();++q)loaded(*q,q->distance-(q-1)->distance,result.pullout.authoring);
    for(auto q=body.section.samples.begin()+1;q!=top;++q){
        if(q->distance<=result.retainedBeginDistance+1e-8)continue;
        loaded(*q,q->distance-(q-1)->distance,body.authoring);
    }
    if(longestLoadedDistance<100)throw std::runtime_error("Reference ascent lost its sustained shoulder above the FF3.65g marker");
    const auto& controls=body.authoring.controls;
    const auto firstRecovery=std::find_if(controls.begin(),controls.end(),[](const auto& c){return c.first[0]>1e-5;});
    const auto lastRecovery=std::find_if(controls.rbegin(),controls.rend(),[](const auto& c){return c.first[0]>1e-5;});
    if(firstRecovery==controls.end()||lastRecovery==controls.rend())throw std::runtime_error("Asymmetric recovery lost its explicit derivative-matched controls");
    const double recoveryBegin=(firstRecovery-1)->time,recoveryEnd=lastRecovery.base()->time;
    const double recoveryPeak=std::max_element(controls.begin(),controls.end(),[](const auto& a,const auto& b){return a.normalG<b.normalG;})->normalG;
    double normal=sampleFvdControl(controls,recoveryBegin).normalG;
    for(int i=1;i<=2000;++i){const auto c=sampleFvdControl(controls,std::lerp(recoveryBegin,recoveryEnd,i/2000.));
        if(c.normalG<normal-1e-10||c.normalG>recoveryPeak+1e-10)throw std::runtime_error("Asymmetric recovery introduced a force reversal or overshoot");normal=c.normalG;}
    std::vector<Vec3> contour;
    for(const auto& q:pullout.samples)contour.push_back(q.position);
    const auto join=result.camelback.section.track.sample(result.retainedBeginDistance).position;
    const auto offset=pullout.samples.back().position-join;
    for(const auto& q:result.camelback.section.samples)if(q.distance>=result.retainedBeginDistance)
        contour.push_back(q.position+offset);
    const auto apex=*std::max_element(contour.begin(),contour.end(),[](Vec3 a,Vec3 b){return a.z<b.z;});
    const double familyScale=std::pow(result.referenceEntrySpeed/83.5,2);
    auto project=[&](Vec3 q,double scale=1,double width=1){return Vec3{signature_reference::apexDisplay[0]+(apex.x-q.x)*signature_reference::pixelsPerMeter/familyScale*scale*width,
        signature_reference::apexDisplay[1]+(apex.z-q.z)*signature_reference::pixelsPerMeter/familyScale*scale,0};};
    auto photoError=[&](double scale,double width=1){
        double squared=0,maximum=0;
        for(const auto& pixel:signature_reference::railPixels){
            const Vec3 target{pixel[0]*signature_reference::sourceToDisplay,pixel[1]*signature_reference::sourceToDisplay,0};
            double error=INFINITY;
            for(size_t i=1;i<contour.size();++i){const auto a=project(contour[i-1],scale,width),delta=project(contour[i],scale,width)-a;
                const double u=std::clamp(dot(target-a,delta)/dot(delta,delta),0.,1.);
                error=std::min(error,norm(target-a-delta*u));}
            squared+=error*error;maximum=std::max(maximum,error);
        }
        return std::array<double,2>{std::sqrt(squared/signature_reference::railPixels.size()),maximum};
    };
    // The reference allows one uniform scale. Fit that single image-space
    // parameter; never refit the flanks separately or change the world geometry.
    auto comparisonScale=[&](double width){
        double lo=.8,hi=1.2;
        for(int i=0;i<40;++i){const double a=lo+(hi-lo)/3,b=hi-(hi-lo)/3;
            if(photoError(a,width)[0]<photoError(b,width)[0])hi=b;else lo=a;}
        return (lo+hi)*.5;
    };
    const double scale=comparisonScale(1);const auto error=photoError(scale);
    const double rms=error[0],maximum=error[1];
    std::cout<<"Combined launch/camelback photo discrepancy: recorded-scale RMS="<<photoError(1)[0]
        <<"px single-scale RMS="<<rms<<"px maximum="<<maximum<<"px comparisonScale="<<scale<<" familyScale="<<familyScale<<'\n';
    for(double width:{1.1,-1.})if(photoError(comparisonScale(width),width)[0]<4)
        throw std::runtime_error("Single-scale photo oracle failed to reject distorted or reversed travel geometry");
    if(argc>1){std::ofstream out(argv[1]);out<<std::setprecision(17)<<"x,z,photoX,photoY\n";
        for(const auto q:contour){const auto projected=project(q);out<<q.x<<','<<q.z<<','<<projected.x<<','<<projected.y<<'\n';}}
    // Practical visual regression envelope; the user prioritises the supplied
    // silhouette over fitting individual ambiguous rail pixels.
    if(rms>=4||maximum>=16)throw std::runtime_error("Connected camelback exceeds the practical single-scale silhouette envelope");
    bool cancelled=false;try{designLaunchCamelback(83.5,-10*pi/180,CamelbackParameters{},.08,.000015,[]{return true;});}catch(...){cancelled=true;}
    if(!cancelled)throw std::runtime_error("Cancellation ignored");
    std::cout<<"PASS direct FVD launch pullout: duration="<<pullout.samples.back().time<<" referenceSpeed="<<result.referenceEntrySpeed<<" retainedShoulder="<<result.retainedBeginDistance<<'\n';
    return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
