#pragma once
#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "GameFramework/GameModeBase.h"
#include "GameFramework/PlayerController.h"
#include "VibeWorld.generated.h"

struct FVibeState;
UCLASS()
class AVibeWorld : public AActor {
    GENERATED_BODY()
  public:
    AVibeWorld();
    virtual ~AVibeWorld();
    virtual void BeginPlay() override;
    virtual void Tick(float DeltaSeconds) override;
    virtual void EndPlay(const EEndPlayReason::Type Reason) override;
    virtual void CalcCamera(float DeltaTime, struct FMinimalViewInfo &OutResult) override;
    void TogglePause();
    void Restart();
    void CycleView();

  private:
    TSharedPtr<FVibeState> State;
    void BuildMenu();
    void Request(bool Load);
    void Cancel();
    void Save();
    void VerifyFlow(double Now);
    void Event(const FString &Name, const FString &Fields = FString());
};
UCLASS()
class AVibeController : public APlayerController {
    GENERATED_BODY()
  public:
    virtual void BeginPlay() override;
    virtual void SetupInputComponent() override;
    void PauseRide();
    void RestartRide();
    void ChangeView();
};
UCLASS()
class AVibeGameMode : public AGameModeBase {
    GENERATED_BODY()
  public:
    AVibeGameMode();
};
