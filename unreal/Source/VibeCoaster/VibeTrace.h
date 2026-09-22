#pragma once
#include "CoreMinimal.h"
#include "Components/PrimitiveComponent.h"
#include "VibeTrace.generated.h"
namespace coaster {
struct Track;
}
struct FVibeTraceSegment {
    FVector Start, End;
    FLinearColor Color;
};
UCLASS()
class UVibeTrace : public UPrimitiveComponent {
    GENERATED_BODY()
  public:
    UVibeTrace();
    void SetTrack(const coaster::Track &Track);
    void Clear();
    virtual FPrimitiveSceneProxy *CreateSceneProxy() override;
    virtual FBoxSphereBounds CalcBounds(const FTransform &LocalToWorld) const override;
    TArray<FVibeTraceSegment> Segments;
};
