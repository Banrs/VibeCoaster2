#pragma once
#include "CoreMinimal.h"
#include "GameFramework/GameModeBase.h"
#include "GameFramework/PlayerController.h"
#include "GameFramework/HUD.h"
#include "coaster/coaster.hpp"
#include "VibeCoasterGame.generated.h"

class AVibeCoasterWorld;
class FCoasterRuntimeVerification;
#if !UE_BUILD_SHIPPING && !UE_BUILD_TEST
struct FCoasterRuntimeVerificationDeleter { void operator()(FCoasterRuntimeVerification* Pointer) const; };
#endif

UCLASS()
class VIBECOASTER_API AVibeCoasterController : public APlayerController
{
    GENERATED_BODY()
public:
    AVibeCoasterController();
    virtual ~AVibeCoasterController() override;
    virtual void BeginPlay() override;
    virtual void PlayerTick(float DeltaSeconds) override;
    UPROPERTY() TObjectPtr<AVibeCoasterWorld> Ride;
    coaster::GenerationRequest Settings;
    FString SeedText = TEXT("42");
    FString InputError, ReferenceError;
    bool Menu = true, ShowTelemetry = false, ShowComparison = false;
    int32 ComparisonOffset = 0;
    int32 SelectedRow = 0;
    static constexpr int32 SetupRowCount = 11;
    FString RowText(int32 Row) const;
private:
    friend class FCoasterRuntimeVerification;
    friend class FCoasterSeedInputContract;
#if !UE_BUILD_SHIPPING && !UE_BUILD_TEST
    TUniquePtr<FCoasterRuntimeVerification, FCoasterRuntimeVerificationDeleter> Verification;
#endif
    void ChangeRow(int32 Direction);
    void RequestGeneration();
};

UCLASS()
class VIBECOASTER_API AVibeCoasterHUD : public AHUD
{
    GENERATED_BODY()
public:
    virtual void DrawHUD() override;
private:
    TWeakObjectPtr<AVibeCoasterWorld> OutlineRide;
    uint64 OutlineRevision = MAX_uint64;
    TArray<FVector> OutlineWorldPoints;
    TArray<FLinearColor> OutlineColours;
    TArray<FVector> OutlineScreenPoints;
};

UCLASS()
class VIBECOASTER_API AVibeCoasterGameMode : public AGameModeBase
{
    GENERATED_BODY()
public:
    AVibeCoasterGameMode();
    virtual void StartPlay() override;
};
