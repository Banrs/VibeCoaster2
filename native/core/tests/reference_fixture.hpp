#pragma once
#include "coaster/coaster.hpp"

// Structural test data only. This is never a production I305 benchmark.
inline coaster::Targets syntheticReference(double median=11){
    coaster::Targets t;t.referenceExposure=median;t.referenceId="rf-v1:"+std::string(64,'a');
    auto& b=t.reference;b.processed=true;b.method="piecewise-linear-positive10s-v1";b.groupId=t.referenceId;
    b.ride="SYNTHETIC fixture, not I305";b.configuration="synthetic-cfg";b.seat="front";b.device="synthetic-device";b.calibrationId="synthetic-cal";
    b.minimum=median-1;b.median=median;b.maximum=median+1;
    for(int i=0;i<3;++i){coaster::ReferenceRecording r;r.recordingId="synthetic-"+std::to_string(i);r.rawSha256=r.canonicalSha256=r.analysisSha256=std::string(64,char('a'+i));r.source="SYNTHETIC fixture";r.notes="No measured data";r.sampleRateHz=r.sampleRateMinHz=r.sampleRateMaxHz=100;r.exposure=median-1+i;b.recordings.push_back(r);}
    return t;
}
