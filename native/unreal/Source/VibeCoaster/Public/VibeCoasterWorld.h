#pragma once
#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "coaster/coaster.hpp"
#include <memory>
#include "VibeCoasterWorld.generated.h"

class UInstancedStaticMeshComponent;
class UProceduralMeshComponent;
class UMaterialInterface;
class ACameraActor;
struct FCoasterRuntime;
struct FCoasterRuntimeDeleter { void operator()(FCoasterRuntime* Pointer) const; };
struct FCoasterPlaybackObservation { double Time, Distance, Speed; };
struct FCoasterLoadingObservation
{
    bool Active = false, Cancelling = false;
    coaster::WorkPhase Phase = coaster::WorkPhase::Complete;
    double ElapsedSeconds = 0, Completed = 0, Total = 0;
    int32 Candidate = 0;
    FString Title;
};

UCLASS()
class VIBECOASTER_API AVibeCoasterAssembly : public AActor
{
    GENERATED_BODY()
public:
    AVibeCoasterAssembly();
    UPROPERTY() TObjectPtr<UInstancedStaticMeshComponent> Ties;
    UPROPERTY() TObjectPtr<UInstancedStaticMeshComponent> Supports;
    UPROPERTY() TObjectPtr<UInstancedStaticMeshComponent> LSMHardware;
    UPROPERTY() TObjectPtr<UInstancedStaticMeshComponent> BrakeHardware;
    UPROPERTY() TObjectPtr<UInstancedStaticMeshComponent> Cars;
    UPROPERTY() TObjectPtr<UInstancedStaticMeshComponent> StationSteel;
    UPROPERTY() TObjectPtr<UInstancedStaticMeshComponent> StationConcrete;
    UPROPERTY() TObjectPtr<UInstancedStaticMeshComponent> PlatformPanels;
    UPROPERTY() TObjectPtr<UInstancedStaticMeshComponent> PlatformEnds;
    UPROPERTY() TObjectPtr<UInstancedStaticMeshComponent> RoofPanels;
    UPROPERTY() TObjectPtr<UInstancedStaticMeshComponent> StationPosts;
    UPROPERTY() TArray<TObjectPtr<UProceduralMeshComponent>> Chunks;
};

UCLASS()
class VIBECOASTER_API AVibeCoasterWorld : public AActor
{
    GENERATED_BODY()
public:
    AVibeCoasterWorld();
    virtual ~AVibeCoasterWorld() override;
    virtual void BeginPlay() override;
    virtual void Tick(float DeltaSeconds) override;
    virtual void EndPlay(const EEndPlayReason::Type Reason) override;
    void Generate(const coaster::GenerationRequest& Request);
    void Cancel();
    void Save();
    void Load();
    void Restart();
    void TogglePause();
    void SetSeat(int32 InSeat);
    void ToggleOverview();
    FString Status() const;
    FCoasterLoadingObservation Loading() const;
    FString Telemetry() const;
    TArray<FString> Comparison() const;
    bool IsBusy() const;
    bool HasRide() const;
    bool IsPaused() const;
    bool IsOverview() const;
    int32 Seat() const;
    const coaster::Design* ActiveDesign() const;
    // Changes only when a fully accepted hidden assembly becomes active.
    uint64 GeometryRevision() const;
    FCoasterPlaybackObservation Playback() const;
private:
    TUniquePtr<FCoasterRuntime, FCoasterRuntimeDeleter> Runtime;
    UPROPERTY() TObjectPtr<AVibeCoasterAssembly> Active;
    UPROPERTY() TObjectPtr<AVibeCoasterAssembly> Staging;
    UPROPERTY() TArray<TObjectPtr<AVibeCoasterAssembly>> Retired;
    UPROPERTY() TObjectPtr<ACameraActor> Camera;
    UPROPERTY() TObjectPtr<UMaterialInterface> RailMaterial;
    UPROPERTY() TObjectPtr<UMaterialInterface> GroundMaterial;
    UPROPERTY() TObjectPtr<UMaterialInterface> StructureMaterial;
    UPROPERTY() TObjectPtr<UMaterialInterface> FootingMaterial;
    UPROPERTY() TObjectPtr<UMaterialInterface> LSMMaterial;
    UPROPERTY() TObjectPtr<UMaterialInterface> BrakeMaterial;
    void StartQueuedJob();
    void PollJob();
    void CommitChunks();
    void UpdateRide(double DeltaSeconds);
    void Retire(AVibeCoasterAssembly* Assembly);
    void CleanRetired();
};
