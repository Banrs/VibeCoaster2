#include "coaster/coaster.hpp"
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <condition_variable>
#include <future>
#include <mutex>
#include <sstream>
#include <iomanip>
using namespace coaster;
namespace fs=std::filesystem;
static int checks;
static void check(bool ok,const char* why){++checks;if(!ok)throw std::runtime_error(why);}
static std::string read(const fs::path& p){std::ifstream f(p,std::ios::binary);return {std::istreambuf_iterator<char>(f),{}};}
static void write(const fs::path& p,const std::string& s){std::ofstream f(p,std::ios::binary|std::ios::trunc);f<<s;check(bool(f),"Fixture write succeeds");}
static std::vector<fs::path> temporaries(const fs::path& dest){
 std::vector<fs::path> result;const auto prefix=dest.filename().string()+".tmp";
 for(const auto& entry:fs::directory_iterator(dest.parent_path()))if(entry.path().filename().string().rfind(prefix,0)==0)result.push_back(entry.path());
 return result;
}
struct CommitGate {
 std::mutex mutex;std::condition_variable cv;bool ready=false,released=false;
 bool hold(){std::unique_lock lock(mutex);ready=true;cv.notify_all();return !cv.wait_for(lock,std::chrono::seconds(120),[&]{return released;});}
 bool await(){std::unique_lock lock(mutex);return cv.wait_for(lock,std::chrono::seconds(120),[&]{return ready;});}
 void release(){std::lock_guard lock(mutex);released=true;cv.notify_all();}
};
int main(int argc,char** argv){try{
 Design d;std::string error;
 if(argc>1)check(loadDesign(argv[1],d,error),"Accepted fixture loads");
 else{GenerationRequest r;r.targets.requireIntensity=false;d=generate(r);check(d.accepted(),"Proof fixture generates");}
 const fs::path folder=argc>2?fs::path(argv[2]):fs::current_path()/"persistence-cancel-test-output";
 fs::create_directories(folder);const auto dest=folder/"existing.coaster",tmp=folder/"existing.coaster.tmp",other=folder/"unrelated.tmp";
 const std::string sentinel="PREVIOUS SAVE MUST SURVIVE\n",unrelated="UNRELATED TEMP MUST SURVIVE\n";
 // Learn the final checkpoints from a successful control. Inspect the actual
 // temporary at the final callback, without scanning disk during every physics step.
 int totalCalls=0;const bool initialSaved=saveDesign(d,dest.string(),error,[&]{++totalCalls;return false;});if(!initialSaved)std::cerr<<error<<"\n";check(initialSaved,"Uncancelled save commits");
 const auto committed=read(dest);check(committed.rfind("COASTER 6 ",0)==0,"Committed save retains COASTER6 format");check(totalCalls>=2,"Both final checkpoints were polled");
 // This targets the real late boundary without sleeps or thread scheduling.
 write(dest,sentinel);write(other,unrelated);fs::remove(tmp);bool sawTemp=false;int lateCalls=0;
 const bool cancelled=saveDesign(d,dest.string(),error,[&]{if(++lateCalls!=totalCalls)return false;const auto files=temporaries(dest);sawTemp=!files.empty();check(files.size()==1&&fs::file_size(files.front())>0,"Late cancellation observes a written temp file");return true;});
 check(sawTemp,"Cancellation is polled after temporary output exists");
 check(!cancelled&&error=="CANCELLED","Late save cancellation is reported");
 check(read(dest)==sentinel,"Late cancellation preserves existing destination bytes");
 check(temporaries(dest).empty(),"Late cancellation removes its own temporary output");
 check(read(other)==unrelated,"Late cancellation preserves unrelated temporary file");
 // The penultimate callback is after serialization and before temp-file creation.
 write(dest,sentinel);write(tmp,unrelated);int calls=0;
 check(!saveDesign(d,dest.string(),error,[&]{return ++calls==totalCalls-1;}),"Pre-write cancellation refuses save");
 check(error=="CANCELLED"&&calls==totalCalls-1,"Pre-write callback targets the serialization boundary");
 check(read(dest)==sentinel,"Pre-write cancellation preserves destination");
 check(read(tmp)==unrelated,"Pre-write cancellation does not delete or truncate an unowned temp");fs::remove(tmp);
 // No callback after replacement may relabel an already committed operation.
 calls=0;check(saveDesign(d,dest.string(),error,[&]{return ++calls>totalCalls;}),"Cancellation beyond commit boundary cannot relabel success");
 check(calls==totalCalls,"No cancellation callback runs after commit");
 check(read(dest)==committed,"Successful serialized bytes remain identical");check(temporaries(dest).empty(),"Successful rename leaves no temporary file");
 // Hold A after it writes, then B after it writes. Releasing A first must
 // commit independently. The old shared .tmp implementation consumes B's
 // temporary during A's rename and then fails B's rename. One unchanged
 // accepted revision is sufficient to reproduce that ownership race.
 auto second=d;CommitGate gateA,gateB;int countA=0,countB=0;
 std::string errorA,errorB;
 auto a=std::async(std::launch::async,[&]{return saveDesign(d,dest.string(),errorA,[&]{return ++countA==totalCalls?gateA.hold():false;});});
 const bool readyA=gateA.await();
 auto b=std::async(std::launch::async,[&]{return saveDesign(second,dest.string(),errorB,[&]{return ++countB==totalCalls?gateB.hold():false;});});
 const bool readyB=gateB.await();
 const size_t pending=temporaries(dest).size();gateA.release();const bool savedA=a.get();const auto afterA=read(dest);
 gateB.release();const bool savedB=b.get();const auto afterB=read(dest);
 check(readyA&&readyB,"Both writers reach the actual final commit boundary");
 check(pending==2,"Concurrent writers exclusively own distinct temporary files");
 check(savedA&&afterA==committed,"First writer succeeds with its own exact payload");
 check(savedB&&afterB==committed,"Second writer independently commits the accepted revision");
 auto payloadStart=afterB.find('\n');std::istringstream row(afterB.substr(payloadStart+1));std::string version;uint64_t seed=0;row>>std::quoted(version)>>seed;
 check(bool(row)&&seed==second.request.seed,"Last writer wins with the requested saved identity");
 check(temporaries(dest).empty(),"Both successful commits clean only their own temporaries");
 fs::remove(dest);fs::remove(other);std::cout<<"PASS "<<checks<<" persistence cancellation/commit checks\n";return 0;
}catch(const std::exception& e){std::cerr<<"FAIL "<<checks<<": "<<e.what()<<'\n';return 1;}}
