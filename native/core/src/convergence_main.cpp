#include "coaster/coaster.hpp"
#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
using namespace coaster;
std::string quote(const std::string& s){std::ostringstream o;o<<'"';for(unsigned char c:s){if(c=='"'||c=='\\')o<<'\\'<<c;else if(c<32)o<<"\\u"<<std::hex<<std::setw(4)<<std::setfill('0')<<int(c)<<std::dec;else o<<c;}return o.str()+'"';}

int main(int argc,char**argv){
    if(argc!=3){std::cerr<<"usage: coaster_convergence save-list.txt output.jsonl\n";return 1;}
    std::ifstream list(argv[1]);std::ofstream out(argv[2]);if(!list||!out)return 1;out<<std::setprecision(17);
    int count=0,failed=0;std::string path;
    while(std::getline(list,path)){if(!path.empty()&&path.back()=='\r')path.pop_back();if(path.empty())continue;++count;
        const auto begin=std::chrono::steady_clock::now();Design d;std::string error;bool loaded=loadDesign(path,d,error);
        out<<"{\"source\":"<<quote(path)<<",\"runtime\":"<<quote(generatorVersion)<<",\"loaded\":"<<(loaded?"true":"false");
        if(!loaded){++failed;out<<",\"passed\":false,\"error\":"<<quote(error)<<"}\n";out.flush();continue;}
        out<<",\"coarseReport\":"<<reportJson(d);
        const auto coarseMetrics=d.simulation.metrics;
        d.simulation=simulate(d.track,d.operations,d.request.train,d.request.simulationStep*.5);
        d.report={};evaluateTargets(d);const auto& fine=d.simulation;
        bool passed=fine.completed&&!fine.cancelled&&fine.report.valid()&&d.report.valid();
        out<<",\"seed\":"<<d.request.seed<<",\"terrain\":"<<quote(d.request.terrain.name())<<",\"step\":"<<d.request.simulationStep<<",\"fineCompleted\":"<<(fine.completed?"true":"false")<<",\"fineSimulationValid\":"<<(fine.report.valid()?"true":"false")<<",\"fineTargetErrors\":[";for(size_t j=0;j<d.report.errors.size();++j){if(j)out<<',';out<<quote(d.report.errors[j].code);}out<<"],\"metrics\":{";
        bool comma=false;auto metric=[&](const char* name,double coarse,double small,double floor,double tolerance){if(comma)out<<',';comma=true;double absolute=std::abs(coarse-small),relative=absolute/std::max(floor,std::abs(small));bool valid=std::isfinite(coarse)&&std::isfinite(small)&&relative<tolerance;passed=passed&&valid;
            out<<quote(name)<<":{\"coarse\":";if(std::isfinite(coarse))out<<coarse;else out<<"null";out<<",\"fine\":";if(std::isfinite(small))out<<small;else out<<"null";out<<",\"normalizedError\":";if(std::isfinite(relative))out<<relative;else out<<"null";out<<",\"normalizationFloor\":"<<floor<<",\"limit\":"<<tolerance<<",\"passed\":"<<(valid?"true":"false")<<'}';};
        const auto& a=coarseMetrics;const auto& b=fine.metrics;
        metric("maxSpeed",a.maxSpeed,b.maxSpeed,1,.01);
        metric("minVerticalG",a.minVerticalG,b.minVerticalG,1,.02);
        metric("maxVerticalG",a.maxVerticalG,b.maxVerticalG,1,.02);
        metric("maxLateralG",a.maxLateralG,b.maxLateralG,1,.02);
        metric("maxLongitudinalG",a.maxLongitudinalG,b.maxLongitudinalG,1,.02);
        metric("exposure10Seconds",a.exposure10Seconds,b.exposure10Seconds,1,.02);
        metric("maxVerticalRateGps",a.maxJerkGps,b.maxJerkGps,1,.02);
        const bool lateralRate=std::isfinite(d.request.limits.maxLateralRateGps),longitudinalRate=std::isfinite(d.request.limits.maxLongitudinalRateGps);
        const char* seats[]={"front","middle","rear"};const char* axes[]={"vertical","lateral","longitudinal"};
        for(int seat=0;seat<3;++seat){std::string prefix=std::string(seats[seat])+".";
            metric((prefix+"exposure10Seconds").c_str(),a.seats[seat].exposure10Seconds,b.seats[seat].exposure10Seconds,1,.02);
            for(int axis=0;axis<3;++axis){const auto& x=a.seats[seat].axes[axis];const auto& y=b.seats[seat].axes[axis];std::string key=prefix+axes[axis]+".";
                auto pair=[&](const char* name,double coarse,double fine){metric((key+name).c_str(),coarse,fine,1,.02);};
                pair("minG",x.minG,y.minG);pair("maxG",x.maxG,y.maxG);pair("meanG",x.meanG,y.meanG);
                pair("mean1sMin",x.mean1sMin,y.mean1sMin);pair("mean1sMax",x.mean1sMax,y.mean1sMax);
                pair("mean10sMin",x.mean10sMin,y.mean10sMin);pair("mean10sMax",x.mean10sMax,y.mean10sMax);
                if(axis==0||(axis==1&&lateralRate)||(axis==2&&longitudinalRate))pair("maxRateGps",x.maxRateGps,y.maxRateGps);
            }
        }
        double elapsed=std::chrono::duration<double>(std::chrono::steady_clock::now()-begin).count();
        out<<"},\"hasSeatStatistics\":true,\"durationCoarse\":"<<a.duration<<",\"durationFine\":"<<b.duration<<",\"elapsedSeconds\":"<<elapsed<<",\"passed\":"<<(passed?"true":"false")<<"}\n";out.flush();if(!passed)++failed;
    }
    std::cout<<"Convergence cases="<<count<<" passed="<<(count-failed)<<" failed="<<failed<<"\n";return count>0&&failed==0?0:2;
}
