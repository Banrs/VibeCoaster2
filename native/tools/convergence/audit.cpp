#include "coaster/coaster.hpp"
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
using namespace coaster;
std::string quote(const std::string& s){std::ostringstream o;o<<'"';for(unsigned char c:s){if(c=='"'||c=='\\')o<<'\\'<<c;else if(c<32)o<<"\\u"<<std::hex<<std::setw(4)<<std::setfill('0')<<int(c)<<std::dec;else o<<c;}return o.str()+'"';}

int main(int argc,char**argv){
    if(argc!=3){std::cerr<<"usage: convergence_audit save-list.txt output.jsonl\n";return 1;}
    std::ifstream list(argv[1]);std::ofstream out(argv[2]);if(!list||!out)return 1;out<<std::setprecision(17);
    int count=0,failed=0;std::string path;
    while(std::getline(list,path)){if(!path.empty()&&path.back()=='\r')path.pop_back();if(path.empty())continue;++count;
        const auto begin=std::chrono::steady_clock::now();Design d;std::string error;bool loaded=loadDesign(path,d,error);
        out<<"{\"source\":"<<quote(path)<<",\"runtime\":"<<quote(generatorVersion)<<",\"loaded\":"<<(loaded?"true":"false");
        if(!loaded){++failed;out<<",\"passed\":false,\"error\":"<<quote(error)<<"}\n";out.flush();continue;}
        out<<",\"coarseReport\":"<<reportJson(d);
        const auto coarse=std::move(d.simulation);
        d.simulation=simulate(d.track,d.operations,d.request.train,d.request.simulationStep*.5);
        d.report={};evaluateTargets(d);const auto& fine=d.simulation;
        ConvergenceAssessment comparison;
        const auto convergence=compareSimulationConvergence(coarse,fine,d.request.limits,comparison);
        bool passed=convergence.valid()&&fine.completed&&!fine.cancelled&&fine.report.valid()&&d.report.valid();
        out<<",\"seed\":"<<d.request.seed<<",\"terrain\":"<<quote(d.request.terrain.name())<<",\"step\":"<<d.request.simulationStep<<",\"fineCompleted\":"<<(fine.completed?"true":"false")<<",\"fineSimulationValid\":"<<(fine.report.valid()?"true":"false")<<",\"fineTargetErrors\":[";for(size_t j=0;j<d.report.errors.size();++j){if(j)out<<',';out<<quote(d.report.errors[j].code);}out<<"],\"metrics\":{";
        bool comma=false;for(const auto& metric:comparison.metrics){
            if(comma)out<<',';comma=true;
            const double scale=std::max(1.,std::abs(metric.fine)),relative=metric.absoluteDifference/scale;
            const bool valid=std::isfinite(relative)&&metric.absoluteDifference<metric.tolerance;
            out<<quote(metric.name)<<":{\"coarse\":";if(std::isfinite(metric.coarse))out<<metric.coarse;else out<<"null";
            out<<",\"fine\":";if(std::isfinite(metric.fine))out<<metric.fine;else out<<"null";
            out<<",\"normalizedError\":";if(std::isfinite(relative))out<<relative;else out<<"null";
            out<<",\"normalizationFloor\":1,\"limit\":"<<metric.tolerance/scale<<",\"passed\":"<<(valid?"true":"false")<<'}';
        }
        double elapsed=std::chrono::duration<double>(std::chrono::steady_clock::now()-begin).count();
        out<<"},\"hasSeatStatistics\":true,\"durationCoarse\":"<<coarse.metrics.duration<<",\"durationFine\":"<<fine.metrics.duration<<",\"elapsedSeconds\":"<<elapsed<<",\"passed\":"<<(passed?"true":"false")<<"}\n";out.flush();if(!passed)++failed;
    }
    std::cout<<"Convergence cases="<<count<<" passed="<<(count-failed)<<" failed="<<failed<<"\n";return count>0&&failed==0?0:2;
}
