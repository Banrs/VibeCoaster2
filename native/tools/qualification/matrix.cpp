#include "coaster/coaster.hpp"
#include <filesystem>
#include <fstream>
#include <iostream>
#include <chrono>
using namespace coaster;
int main(int argc,char** argv){try{
    if(argc!=3)throw std::runtime_error("Expected predeclared case index and fresh output directory");
    const int index=std::stoi(argv[1]);if(index<0||index>=44)throw std::runtime_error("Case index outside the predeclared 44 requests");
    GenerationRequest request;request.targets.requireIntensity=false;request.maxCandidates=1;
    TerrainKind kind;
    if(index<36){
        constexpr std::array<uint64_t,6> seeds{0,3,11,19,73,101};
        request.seed=seeds[index/6];kind=TerrainKind((index/2)%3);request.train.cars=index%2?12:6;
    }else if(index<42){request.seed=37;kind=TerrainKind((index-36)/2);}
    else{request.seed=index==42?23:31;kind=index==42?TerrainKind::Flat:TerrainKind::Hills;
        request.train.cars=index==42?6:12;request.targets.height=index==42?240:250;request.targets.speed=index==42?80:85;}
    request.terrain=Terrain::seeded(kind,request.seed);
    if(index>=36&&index<42){const bool second=index%2;
        request.terrain.offsetX=second?-900:1300;request.terrain.offsetY=second?1100:-700;
        request.terrain.headingRadians=second?-1.17:.61;
        if(second){request.terrain.horizontalScale*=1.2;request.terrain.verticalScale*=.85;}}
    const std::filesystem::path folder=argv[2];
    if(!std::filesystem::create_directory(folder))throw std::runtime_error("Evidence directory already exists");
    const auto start=std::chrono::steady_clock::now();
    auto design=generate(request);
    std::ofstream(folder/"report.json")<<reportJson(design);
    std::ofstream(folder/"plan.json")<<design.planningDiagnostics;
    const double seconds=std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count();
    std::ofstream(folder/"run.json")<<"{\"caseIndex\":"<<index<<",\"generationSeconds\":"<<seconds<<"}";
    if(design.accepted()){
        std::string error;if(!saveDesign(design,(folder/"ride.coaster").string(),error))throw std::runtime_error(error);
    }
    std::cout<<"case="<<index<<" accepted="<<design.accepted()<<" seconds="<<seconds<<'\n';
    return design.accepted()?0:2;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
