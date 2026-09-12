#include "generation_internal.hpp"
#define main ordinaryCliMain
#include CLI_SOURCE
#undef main

int main(int argc,char** argv){try{
    GenerationRequest request;request.seed=argc>1?seedNumber(argv[1]):2;
    request.terrain.kind=TerrainKind::Hills;request.targets.requireIntensity=false;request.maxCandidates=1;
    const auto d=detail::generateRide(request,true);
    write("report.json",reportJson(d)+"\n");write("plan.json",d.planningDiagnostics+"\n");
    if(!d.track.spans.empty())trace(d,"trace.json");
    std::string error;
    if(d.accepted()&&!saveDesign(d,"ride.coaster",error))throw std::runtime_error(error);
    for(const auto* report:{&d.report,&d.simulation.report})for(const auto& issue:report->errors)
        std::cerr<<issue.code<<": "<<issue.message<<'\n';
    std::cout<<"seed="<<request.seed<<" requestedCrossover=true accepted="<<d.accepted()<<" length="<<d.track.length<<'\n';
    return d.accepted()?0:2;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
