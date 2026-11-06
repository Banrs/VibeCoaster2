#include "coaster/ride_feel.hpp"
#include <chrono>
#include <filesystem>
#include <iostream>
#include <stdexcept>
using namespace coaster;
namespace {
void require(bool value,const std::string& message){if(!value)throw std::runtime_error(message);}
struct TemporaryRide {
    std::filesystem::path folder=std::filesystem::temp_directory_path()/
        ("vibecoaster-roundtrip-"+std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
    TemporaryRide(){require(std::filesystem::create_directory(folder),"Create isolated save directory");}
    ~TemporaryRide(){std::error_code ignored;std::filesystem::remove_all(folder,ignored);}
};
RideFeel checkRide(const Design& d){
    for(const auto* report:{&d.report,&d.simulation.report})for(const auto& error:report->errors)
        std::cerr<<error.code<<": "<<error.message<<'\n';
    require(d.accepted(),"Full independent ride acceptance must pass");
    require(d.track.closed&&!d.track.spans.empty(),"Generation must produce a closed physical circuit");
    const auto& frames=d.simulation.frames;
    require(frames.size()>1&&frames.back().speed==0,"The train must complete its course and physically stop");
    for(size_t i=1;i<frames.size();++i)
        require(frames[i].time>frames[i-1].time&&frames[i].distance>=frames[i-1].distance&&
            std::isfinite(frames[i].speed)&&frames[i].speed>=0,"Replay must progress without reversal or invalid motion");
    const auto& metrics=d.simulation.metrics;
    require(metrics.maxEnergyResidual<.5&&metrics.driveWorkPerMass>0&&metrics.brakeWorkPerMass>0,
        "Measured drive, brake and loss work must account for train energy");
    for(const auto& assessment:d.simulation.acceleration)
        require(assessment.performed&&assessment.passed,"Every seating position needs its independent acceleration assessment");
    const auto feel=measureRideFeel(d);
    require(feel.performed&&feel.freeRunningSeconds>10,"The ride needs a substantial free-running course");
    for(const auto& seat:feel.seats)
        require(seat.airtimeSeconds>.75&&seat.loadedSeconds>2&&seat.forceBeats>=4,
            "Front, middle and rear riders need sustained airtime and contrasting force beats");
    return feel;
}
}
int main(int argc,char** argv){try{
    Design ride;std::string error;
    if(argc==1){
        GenerationRequest request;request.seed=42;request.terrain.kind=TerrainKind::Highlands;
        ride=generate(request);
    }else if(argc==3&&(std::string(argv[1])=="--design"||std::string(argv[1])=="--baseline-only")){
        if(!loadDesign(argv[2],ride,error))throw std::runtime_error("Initial reload: "+error);
    }else throw std::runtime_error("Usage: organic_generation_tests [--design ACCEPTED_FILE]");
    const auto before=checkRide(ride);
    TemporaryRide temporary;const auto path=(temporary.folder/"ride.coaster").string();
    if(!saveDesign(ride,path,error))throw std::runtime_error("Save accepted ride: "+error);
    Design reloaded;
    if(!loadDesign(path,reloaded,error))throw std::runtime_error("Freshly validate saved ride: "+error);
    const auto after=checkRide(reloaded);
    require(reloaded.request==ride.request&&std::abs(reloaded.track.length-ride.track.length)<1e-7,
        "Saved intent and physical circuit must survive roundtrip");
    for(size_t seat=0;seat<3;++seat)
        require(std::abs(after.seats[seat].airtimeSeconds-before.seats[seat].airtimeSeconds)<.001&&
            after.seats[seat].forceBeats==before.seats[seat].forceBeats,
            "Fresh replay after loading must preserve the rider's experience");
    const auto cancelled=generate(ride.request,[]{return true;});
    require(cancelled.simulation.cancelled&&!cancelled.accepted(),"Cancellation must not publish a partial ride");
    std::cout<<"PASS one complete ride, independent physics, ride character and fresh save/load validation\n";
    std::cout<<rideFeelJson(after)<<'\n';return 0;
}catch(const std::exception& e){std::cerr<<"FAIL "<<e.what()<<'\n';return 1;}}
