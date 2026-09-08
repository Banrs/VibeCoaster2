#pragma once
#include "CoreMinimal.h"

#if !UE_BUILD_SHIPPING && !UE_BUILD_TEST
class AVibeCoasterController;

// Opt-in observer/driver of the real gameplay API. Never owns an accepted design.
class FCoasterRuntimeVerification
{
public:
    static TUniquePtr<FCoasterRuntimeVerification> Create();
    FCoasterRuntimeVerification();
    ~FCoasterRuntimeVerification();
    void Tick(AVibeCoasterController& Controller, float DeltaSeconds);
private:
    struct FState;
    TUniquePtr<FState> State;
};
#endif
