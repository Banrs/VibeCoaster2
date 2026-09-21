#pragma once
#include "coaster/progress.hpp"
#include <chrono>

namespace coaster {
// Timings are elapsed wall time charged to the foreground phase. Concurrent
// replay work is not double-counted as additional end-to-end latency.
class WorkRecorder {
    using Clock=std::chrono::steady_clock;
    Progress callback;
    WorkProgress current;
    WorkTimings measured;
    Clock::time_point since{Clock::now()};
public:
    explicit WorkRecorder(Progress progress,WorkPhase first=WorkPhase::Authoring):callback(std::move(progress)){current.phase=first;}
    void enter(WorkPhase phase,int candidate=0,std::string detail={},double done=0,double total=0){
        const auto now=Clock::now();measured.seconds[static_cast<size_t>(current.phase)]+=std::chrono::duration<double>(now-since).count();since=now;
        current={phase,candidate,done,total,std::move(detail)};if(callback)callback(current);
    }
    void message(int candidate,const std::string& detail){current.candidate=candidate;current.detail=detail;if(callback)callback(current);}
    WorkTimings snapshot() const{auto result=measured;result.seconds[static_cast<size_t>(current.phase)]+=std::chrono::duration<double>(Clock::now()-since).count();return result;}
};
}
