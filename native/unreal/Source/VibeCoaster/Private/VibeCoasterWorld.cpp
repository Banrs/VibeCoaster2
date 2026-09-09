#include "VibeCoasterWorld.h"
#include "CoasterMesh.h"
#include "Async/Async.h"
#include "Camera/CameraActor.h"
#include "Camera/CameraComponent.h"
#include "Components/DirectionalLightComponent.h"
#include "Components/InstancedStaticMeshComponent.h"
#include "Components/SceneComponent.h"
#include "Components/SkyLightComponent.h"
#include "Engine/DirectionalLight.h"
#include "Engine/SkyLight.h"
#include "Components/SkyAtmosphereComponent.h"
#include "Engine/ExponentialHeightFog.h"
#include "Components/ExponentialHeightFogComponent.h"
#include "Engine/StaticMesh.h"
#include "Engine/World.h"
#include "GameFramework/PlayerController.h"
#include "HAL/FileManager.h"
#include "HAL/PlatformTime.h"
#include "Materials/MaterialInterface.h"
#include "Misc/Paths.h"
#include "ProceduralMeshComponent.h"
#include "UObject/ConstructorHelpers.h"
#include <exception>

namespace
{
enum class EWork { Generate, Load, Save };
struct FJobState { bool IsSave = false; std::atomic<bool> Cancel{false}; std::atomic<int32> Candidate{0}; std::atomic<int32> Stage{0}; };
struct FJobRequest
{
    EWork Kind = EWork::Generate;
    coaster::GenerationRequest Request;
    std::shared_ptr<const coaster::Design> SaveDesign;
    FString Path;
    uint64 Revision = 0;
};
struct FJobResult
{
    TSharedPtr<VibeMesh::FPreparedRide, ESPMode::ThreadSafe> Prepared;
    FString Message;
    uint64 Revision = 0;
    bool WasSave = false, SaveSucceeded = false;
};
FString Failure(const coaster::Design& D)
{
    FString Text = TEXT("Request rejected; no new ride accepted.");
    int32 Count = 0;
    for (const auto* Report : {&D.report, &D.simulation.report}) for (const auto& Error : Report->errors)
    {
        if (Count++ < 6) Text += FString::Printf(TEXT("\n%s: %s"), UTF8_TO_TCHAR(Error.code.c_str()), UTF8_TO_TCHAR(Error.message.c_str()));
    }
    if (Count > 6) Text += FString::Printf(TEXT("\n...and %d further validation findings."), Count - 6);
    return Text;
}
FString DesignPath() { return FPaths::ConvertRelativePathToFull(FPaths::ProjectSavedDir() / TEXT("Designs/Accepted.vcdesign")); }
}

struct FCoasterRuntime
{
    uint64 Revision = 0, GeometryRevision = 0;
    TOptional<FJobRequest> Queued;
    TSharedPtr<FJobState, ESPMode::ThreadSafe> JobState;
    TFuture<FJobResult> Future;
    bool Running = false;
    TSharedPtr<VibeMesh::FPreparedRide, ESPMode::ThreadSafe> Prepared;
    std::shared_ptr<const coaster::Design> Design;
    coaster::ComparisonHistory Comparisons;
    TArray<FString> ComparisonLines;
    TArray<FTransform> CarTransforms;
    bool TrainPoseDirty = true, CameraPoseDirty = true;
    double PresentedDistance = -1;
    int32 NextChunk = 0, NextTie = 0, NextSupport = 0, NextStation = 0;
    double RideTime = 0, Distance = 0, Speed = 0;
    size_t TraceIndex = 0;
    int32 Seat = 0;
    bool Paused = true, Overview = false;
    FBox Bounds{ForceInit};
    FString Message = TEXT("Configure a seed and terrain, then Generate. All-record mode needs an I305 exposure reference.");
};

void FCoasterRuntimeDeleter::operator()(FCoasterRuntime* Pointer) const { delete Pointer; }

AVibeCoasterAssembly::AVibeCoasterAssembly()
{
    RootComponent = CreateDefaultSubobject<USceneComponent>(TEXT("Root"));
    Ties = CreateDefaultSubobject<UInstancedStaticMeshComponent>(TEXT("Ties"));
    Supports = CreateDefaultSubobject<UInstancedStaticMeshComponent>(TEXT("Supports"));
    Cars = CreateDefaultSubobject<UInstancedStaticMeshComponent>(TEXT("Train"));
    StationSteel = CreateDefaultSubobject<UInstancedStaticMeshComponent>(TEXT("StationSteel"));
    StationConcrete = CreateDefaultSubobject<UInstancedStaticMeshComponent>(TEXT("StationConcrete"));
    PlatformPanels = CreateDefaultSubobject<UInstancedStaticMeshComponent>(TEXT("PlatformPanels"));
    PlatformEnds = CreateDefaultSubobject<UInstancedStaticMeshComponent>(TEXT("PlatformEnds"));
    RoofPanels = CreateDefaultSubobject<UInstancedStaticMeshComponent>(TEXT("RoofPanels"));
    StationPosts = CreateDefaultSubobject<UInstancedStaticMeshComponent>(TEXT("StationPosts"));
    static ConstructorHelpers::FObjectFinder<UStaticMesh> TrainCar(TEXT("/Game/Art/V072/Conventional2/SM_TrainCar.SM_TrainCar"));
    static ConstructorHelpers::FObjectFinder<UStaticMesh> TrackTie(TEXT("/Game/Art/V072/TrackWeb1/SM_TrackTieWeb.SM_TrackTieWeb"));
    static ConstructorHelpers::FObjectFinder<UStaticMesh> Platform(TEXT("/Game/Art/V072/Import1/SM_StationPlatformPanel.SM_StationPlatformPanel"));
    static ConstructorHelpers::FObjectFinder<UStaticMesh> PlatformEnd(TEXT("/Game/Art/V072/Import1/SM_StationPlatformEndPanel.SM_StationPlatformEndPanel"));
    static ConstructorHelpers::FObjectFinder<UStaticMesh> Roof(TEXT("/Game/Art/V072/Import1/SM_StationRoofPanel.SM_StationRoofPanel"));
    static ConstructorHelpers::FObjectFinder<UStaticMesh> Post(TEXT("/Game/Art/V072/Import1/SM_StationPost.SM_StationPost"));
    static ConstructorHelpers::FObjectFinder<UStaticMesh> Cube(TEXT("/Engine/BasicShapes/Cube.Cube"));
    static ConstructorHelpers::FObjectFinder<UStaticMesh> Cylinder(TEXT("/Engine/BasicShapes/Cylinder.Cylinder"));
    Ties->SetStaticMesh(TrackTie.Object); Supports->SetStaticMesh(Cylinder.Object); Cars->SetStaticMesh(TrainCar.Object);
    PlatformPanels->SetStaticMesh(Platform.Object); PlatformEnds->SetStaticMesh(PlatformEnd.Object);
    RoofPanels->SetStaticMesh(Roof.Object); StationPosts->SetStaticMesh(Post.Object);
    StationSteel->SetStaticMesh(Cube.Object); StationConcrete->SetStaticMesh(Cube.Object);
    for (auto* Component : {Ties.Get(), Supports.Get(), Cars.Get(), StationSteel.Get(), StationConcrete.Get(), PlatformPanels.Get(), PlatformEnds.Get(), RoofPanels.Get(), StationPosts.Get()})
    {
        Component->SetupAttachment(RootComponent);
        Component->SetMobility(EComponentMobility::Movable);
        Component->SetCollisionEnabled(ECollisionEnabled::NoCollision);
        Component->SetCanEverAffectNavigation(false);
    }
}
AVibeCoasterWorld::AVibeCoasterWorld()
{
    PrimaryActorTick.bCanEverTick = true;
    Runtime.Reset(new FCoasterRuntime());
}
AVibeCoasterWorld::~AVibeCoasterWorld() = default;
void AVibeCoasterWorld::BeginPlay()
{
    Super::BeginPlay();
    RailMaterial = LoadObject<UMaterialInterface>(nullptr, TEXT("/Game/Materials/M_Rail.M_Rail"));
    GroundMaterial = LoadObject<UMaterialInterface>(nullptr, TEXT("/Game/Materials/M_Ground_Relief.M_Ground_Relief"));
    StructureMaterial = LoadObject<UMaterialInterface>(nullptr, TEXT("/Game/Materials/M_Structure.M_Structure"));
    FootingMaterial = LoadObject<UMaterialInterface>(nullptr, TEXT("/Game/Materials/M_Footing.M_Footing"));
    Camera = GetWorld()->SpawnActor<ACameraActor>();
    Camera->SetActorLocationAndRotation(FVector(-6000, 3000, 2500), FRotator(-12, -25, 0));
    Camera->GetCameraComponent()->SetFieldOfView(82);
    auto* Sun = GetWorld()->SpawnActor<ADirectionalLight>();
    Sun->SetActorRotation(FRotator(-35, -35, 0));
    auto* SunComponent = Cast<UDirectionalLightComponent>(Sun->GetLightComponent());
    SunComponent->SetMobility(EComponentMobility::Movable);
    SunComponent->SetIntensity(6.f);
    SunComponent->SetLightColor(FLinearColor(1.f, .92f, .8f));
    SunComponent->bAtmosphereSunLight = true;
    SunComponent->MarkRenderStateDirty();
    GetWorld()->SpawnActor<ASkyAtmosphere>();
    auto* Sky = GetWorld()->SpawnActor<ASkyLight>();
    Sky->GetLightComponent()->SetMobility(EComponentMobility::Movable);
    Sky->GetLightComponent()->SetIntensity(.65f);
    Sky->GetLightComponent()->SetRealTimeCaptureEnabled(true);
    auto* Fog = GetWorld()->SpawnActor<AExponentialHeightFog>();
    Fog->GetComponent()->SetFogDensity(.00015f);
    Fog->GetComponent()->SetStartDistance(60000.f);
    Fog->GetComponent()->SetFogMaxOpacity(.5f);
    if (!RailMaterial || !GroundMaterial || !StructureMaterial || !FootingMaterial)
        Runtime->Message = TEXT("Missing generated materials. Run scripts/package.ps1 -PrepareOnly before playing.");
}
void AVibeCoasterWorld::EndPlay(const EEndPlayReason::Type Reason)
{
    Cancel();
    // No UObject is captured by workers. Drain only at world shutdown, after the
    // cooperative cancel, so live C++ cannot outlast a module/editor unload.
    if (Runtime->Running) { Runtime->Future.Wait(); Runtime->Running = false; }
    Super::EndPlay(Reason);
}
void AVibeCoasterWorld::Retire(AVibeCoasterAssembly* Assembly)
{
    if (!Assembly) return;
    Assembly->SetActorHiddenInGame(true);
    Retired.Add(Assembly);
}
void AVibeCoasterWorld::CleanRetired()
{
    if (Retired.IsEmpty()) return;
    auto* Old = Retired[0].Get();
    // Bounded explicit component retirement; eventual UObject GC is engine-owned.
    for (int32 I = 0; I < 2 && !Old->Chunks.IsEmpty(); ++I)
        Old->Chunks.Pop(EAllowShrinking::No)->DestroyComponent();
    if (Old->Chunks.IsEmpty()) { Old->Destroy(); Retired.RemoveAt(0); }
}
void AVibeCoasterWorld::Cancel()
{
    ++Runtime->Revision;
    if (Runtime->JobState) Runtime->JobState->Cancel.store(true);
    Runtime->Queued.Reset();
    Runtime->Prepared.Reset();
    Retire(Staging); Staging = nullptr;
    Runtime->Message = Runtime->Running && Runtime->JobState && Runtime->JobState->IsSave
        ? TEXT("Save cancellation requested; awaiting commit outcome. Active accepted ride retained.")
        : Runtime->Design ? TEXT("Request cancelled. Active accepted ride retained.") : TEXT("Request cancelled. No accepted ride loaded.");
}
void AVibeCoasterWorld::Generate(const coaster::GenerationRequest& Request)
{
    Cancel();
    FJobRequest Work; Work.Request = Request; Work.Revision = Runtime->Revision;
    Runtime->Queued = MoveTemp(Work);
    Runtime->Message = TEXT("Generating and validating on CPU...");
    StartQueuedJob();
}
void AVibeCoasterWorld::Load()
{
    Cancel();
    FJobRequest Work; Work.Kind = EWork::Load; Work.Path = DesignPath(); Work.Revision = Runtime->Revision;
    Runtime->Queued = MoveTemp(Work);
    Runtime->Message = TEXT("Loading and revalidating complete saved geometry...");
    StartQueuedJob();
}
void AVibeCoasterWorld::Save()
{
    if (!Runtime->Design) { Runtime->Message = TEXT("No accepted ride to save."); return; }
    if (IsBusy()) { Runtime->Message = TEXT("Finish or cancel the current request before saving."); return; }
    IFileManager::Get().MakeDirectory(*FPaths::GetPath(DesignPath()), true);
    FJobRequest Work; Work.Kind = EWork::Save; Work.Path = DesignPath(); Work.SaveDesign = Runtime->Design;
    Work.Revision = ++Runtime->Revision; Runtime->Queued = MoveTemp(Work);
    Runtime->Message = TEXT("Saving accepted geometry and request...");
    StartQueuedJob();
}
void AVibeCoasterWorld::StartQueuedJob()
{
    if (Runtime->Running || !Runtime->Queued.IsSet()) return;
    const FJobRequest Work = Runtime->Queued.GetValue(); Runtime->Queued.Reset();
    const auto State = MakeShared<FJobState, ESPMode::ThreadSafe>(); State->IsSave = Work.Kind == EWork::Save; Runtime->JobState = State;
    Runtime->Running = true;
    Runtime->Future = Async(EAsyncExecution::ThreadPool, [Work, State]() -> FJobResult
    {
        FJobResult Result; Result.Revision = Work.Revision; Result.WasSave = Work.Kind == EWork::Save;
        const coaster::Cancel Cancelled = [State] { return State->Cancel.load(); };
        try
        {
            std::string Error;
            const std::string Path(TCHAR_TO_UTF8(*Work.Path));
            if (Work.Kind == EWork::Save)
            {
                if (Cancelled()) Result.Message = TEXT("Save cancelled before commit; existing saved file retained.");
                else
                {
                    Result.SaveSucceeded = coaster::saveDesign(*Work.SaveDesign, Path, Error, Cancelled);
                    // A completed atomic replacement wins over a later cancel.
                    Result.Message = Result.SaveSucceeded ? TEXT("Saved accepted geometry: ") + Work.Path
                        : Cancelled() ? TEXT("Save cancelled before commit; existing saved file retained.")
                        : TEXT("Save failed: ") + FString(UTF8_TO_TCHAR(Error.c_str()));
                }
                return Result;
            }
            auto Design = std::make_shared<coaster::Design>();
            if (Work.Kind == EWork::Load)
            {
                if (!coaster::loadDesign(Path, *Design, Error, Cancelled))
                { Result.Message = TEXT("Load rejected; active ride retained: ") + FString(UTF8_TO_TCHAR(Error.c_str())); return Result; }
            }
            else *Design = coaster::generate(Work.Request, Cancelled, [State](int N, const std::string&) { State->Candidate.store(N); });
            if (Cancelled()) return Result;
            if (!Design->accepted()) { Result.Message = Failure(*Design); return Result; }
            if (Design->simulation.frames.empty()) { Result.Message = TEXT("Accepted design has no trace; no ride committed."); return Result; }
            State->Stage.store(1);
            auto Prepared = MakeShared<VibeMesh::FPreparedRide, ESPMode::ThreadSafe>();
            Prepared->Design = MoveTemp(Design); Prepared->Revision = Work.Revision;
            if (!VibeMesh::Prepare(*Prepared, Cancelled)) { Result.Message = Prepared->Error; return Result; }
            Result.Prepared = MoveTemp(Prepared);
        }
        catch (const std::exception& Error) { Result.Message = TEXT("Request failed; active ride retained: ") + FString(UTF8_TO_TCHAR(Error.what())); }
        catch (...) { Result.Message = TEXT("Request failed with an unknown native error; active ride retained."); }
        return Result;
    });
}
void AVibeCoasterWorld::PollJob()
{
    if (!Runtime->Running || !Runtime->Future.IsReady()) return;
    FJobResult Result = Runtime->Future.Get(); Runtime->Running = false;
    if (Result.Revision == Runtime->Revision && !Runtime->JobState->Cancel.load())
    {
        if (Result.Prepared && Result.Prepared->Design->accepted())
        {
            Runtime->Prepared = MoveTemp(Result.Prepared);
            Runtime->NextChunk = Runtime->NextTie = Runtime->NextSupport = Runtime->NextStation = 0;
            Staging = GetWorld()->SpawnActor<AVibeCoasterAssembly>();
            Staging->SetActorHiddenInGame(true);
            if (!Staging->Ties->GetStaticMesh() || !Staging->Cars->GetStaticMesh() ||
                !Staging->PlatformPanels->GetStaticMesh() || !Staging->PlatformEnds->GetStaticMesh() ||
                !Staging->RoofPanels->GetStaticMesh() || !Staging->StationPosts->GetStaticMesh())
            {
                Retire(Staging); Staging = nullptr; Runtime->Prepared.Reset();
                Runtime->Message = TEXT("Missing validated model assets; active ride retained. Import the current validated art kit before packaging.");
                Runtime->JobState.Reset(); StartQueuedJob(); return;
            }
            Staging->Ties->PreAllocateInstancesMemory(Runtime->Prepared->Ties.Num());
            Staging->Supports->PreAllocateInstancesMemory(Runtime->Prepared->Supports.Num());
            Staging->Cars->PreAllocateInstancesMemory(Runtime->Prepared->Design->request.train.cars);
            // Authored car/tie/station material slots remain intact; canonical
            // support solids and fallback station boxes use the existing palette.
            Staging->Supports->SetMaterial(0, StructureMaterial);
            Staging->StationSteel->SetMaterial(0, StructureMaterial); Staging->StationConcrete->SetMaterial(0, FootingMaterial);
            Runtime->Message = TEXT("Validated. Preparing hidden render chunks...");
        }
        else if (!Result.Message.IsEmpty())
        { Runtime->Message = Result.Message; UE_LOG(LogTemp, Display, TEXT("%s"), *Result.Message); }
    }
    // Invalidating a request cannot undo an already committed disk transaction.
    // Resolve an outstanding save cancellation truthfully even when its revision
    // was superseded; a queued newer job keeps ownership of the visible status.
    if (Result.WasSave && (Result.Revision != Runtime->Revision || Runtime->JobState->Cancel.load()))
    {
        if (Result.SaveSucceeded) Result.Message += TEXT(" (commit completed before cancellation could stop it)");
        UE_LOG(LogTemp, Display, TEXT("%s"), *Result.Message);
        if (!Runtime->Queued.IsSet()) Runtime->Message = Result.Message;
    }
    Runtime->JobState.Reset(); StartQueuedJob();
}
void AVibeCoasterWorld::CommitChunks()
{
    if (!Runtime->Prepared || !Staging) return;
    auto& Prepared = *Runtime->Prepared;
    if (Prepared.Revision != Runtime->Revision) { Retire(Staging); Staging = nullptr; Runtime->Prepared.Reset(); return; }
    const double Deadline = FPlatformTime::Seconds() + .002;
    // Sections are bounded: up to 6,144 vertices for batched cliff backdrop. At most one section and
    // 64 instances per tick; the 2 ms budget is cooperative, not a frame-time guarantee.
    if (Runtime->NextChunk < Prepared.Chunks.Num())
    {
        auto& Chunk = Prepared.Chunks[Runtime->NextChunk++];
        auto* Mesh = NewObject<UProceduralMeshComponent>(Staging);
        Mesh->SetupAttachment(Staging->GetRootComponent());
        Mesh->SetMobility(EComponentMobility::Movable);
        Mesh->SetCollisionEnabled(ECollisionEnabled::NoCollision);
        Mesh->SetCanEverAffectNavigation(false);
        Staging->AddInstanceComponent(Mesh);
        // Populate the hidden component before registration so its first scene
        // proxy already has the final section and material.
        Mesh->CreateMeshSection(0, Chunk.Vertices, Chunk.Indices, Chunk.Normals, Chunk.UV, TArray<FColor>(), TArray<FProcMeshTangent>(), false);
        UMaterialInterface* Material = Chunk.Terrain ? GroundMaterial : Chunk.Footing ? FootingMaterial : Chunk.Structure ? StructureMaterial : RailMaterial;
        // Shared with the imported conventional train; hardware faces are exposed
        // metal, while the running rails retain their painted finish.
        if (Chunk.Hardware != VibeMesh::EDriveHardwareKind::None && !Chunk.Structure)
            Material = LoadObject<UMaterialInterface>(nullptr, TEXT("/Game/Art/V072/Conventional2/Materials/M_VCTrain4_Metal_BrushedAluminium.M_VCTrain4_Metal_BrushedAluminium"));
        Mesh->SetMaterial(0, Material);
        Mesh->RegisterComponent();
        Staging->Chunks.Add(Mesh);
        Chunk = VibeMesh::FChunk{}; // Release CPU vertex copies once committed.
    }
    int32 Budget = 64;
    while (Budget > 0 && Runtime->NextTie < Prepared.Ties.Num() && FPlatformTime::Seconds() < Deadline)
    { Staging->Ties->AddInstance(Prepared.Ties[Runtime->NextTie++], false); --Budget; }
    while (Budget > 0 && Runtime->NextSupport < Prepared.Supports.Num() && FPlatformTime::Seconds() < Deadline)
    { Staging->Supports->AddInstance(Prepared.Supports[Runtime->NextSupport++], false); --Budget; }
    while (Budget > 0 && Runtime->NextStation < Prepared.Station.Num() && FPlatformTime::Seconds() < Deadline)
    {
        const auto& Part = Prepared.Station[Runtime->NextStation++];
        UInstancedStaticMeshComponent* Component = nullptr;
        switch (Part.Kind)
        {
            case VibeMesh::EStationInstanceKind::CubeSteel: Component = Staging->StationSteel; break;
            case VibeMesh::EStationInstanceKind::CubeConcrete: Component = Staging->StationConcrete; break;
            case VibeMesh::EStationInstanceKind::Platform: Component = Staging->PlatformPanels; break;
            case VibeMesh::EStationInstanceKind::PlatformEnd: Component = Staging->PlatformEnds; break;
            case VibeMesh::EStationInstanceKind::Roof: Component = Staging->RoofPanels; break;
            case VibeMesh::EStationInstanceKind::Post: Component = Staging->StationPosts; break;
        }
        check(Component);
        Component->AddInstance(Part.Transform, false); --Budget;
    }
    if (Runtime->NextChunk != Prepared.Chunks.Num() || Runtime->NextTie != Prepared.Ties.Num() || Runtime->NextSupport != Prepared.Supports.Num() || Runtime->NextStation != Prepared.Station.Num()) return;
    Retire(Active); Active = Staging; Staging = nullptr;
    Runtime->Design = Prepared.Design; Runtime->Bounds = Prepared.Bounds;
    ++Runtime->GeometryRevision;
    Runtime->Comparisons.commit(*Runtime->Design);
    Runtime->ComparisonLines.Reset();
    for (const auto& Line : coaster::comparisonLines(*Runtime->Design, &Runtime->Comparisons.previous))
        Runtime->ComparisonLines.Add(FString(UTF8_TO_TCHAR(Line.c_str())));
    for (int32 I = 0; I < Runtime->Design->request.train.cars; ++I) Active->Cars->AddInstance(FTransform::Identity);
    Active->SetActorHiddenInGame(false);
    Runtime->Message = Runtime->Design->request.targets.requireIntensity
        ? TEXT("Accepted against configured record targets and reference. Space to ride.")
        : TEXT("PHYSICS-PROOF accepted. Intensity comparison disabled by your preset. Space to ride.");
    Runtime->Message += FString::Printf(TEXT("\n%.0f s to final braking | %.0f s to a complete stop"),
        coaster::movingRideSeconds(*Runtime->Design), Runtime->Design->simulation.metrics.duration);
    Runtime->Prepared.Reset(); Runtime->Overview = false; Restart(); Runtime->Paused = true;
}
void AVibeCoasterWorld::Tick(float DeltaSeconds)
{
    Super::Tick(DeltaSeconds); PollJob(); CommitChunks(); CleanRetired(); UpdateRide(DeltaSeconds);
    if (Camera) if (auto* PC = GetWorld()->GetFirstPlayerController()) if (PC->GetViewTarget() != Camera) PC->SetViewTarget(Camera);
}
void AVibeCoasterWorld::Restart()
{
    Runtime->RideTime = 0; Runtime->TraceIndex = 0;
    Runtime->TrainPoseDirty = Runtime->CameraPoseDirty = true;
    UpdateRide(0);
}
void AVibeCoasterWorld::TogglePause() { if (HasRide()) Runtime->Paused = !Runtime->Paused; }
void AVibeCoasterWorld::SetSeat(int32 InSeat)
{
    const int32 NewSeat = FMath::Clamp(InSeat, 0, 2);
    Runtime->CameraPoseDirty |= NewSeat != Runtime->Seat; Runtime->Seat = NewSeat;
}
void AVibeCoasterWorld::ToggleOverview() { Runtime->Overview = !Runtime->Overview; Runtime->CameraPoseDirty = true; }
void AVibeCoasterWorld::UpdateRide(double DeltaSeconds)
{
    if (!Runtime->Design || !Active || !Camera) return;
    const auto& D = *Runtime->Design; const auto& Frames = D.simulation.frames;
    if (Frames.empty()) return;
    if (!Runtime->Paused) Runtime->RideTime += FMath::Min(DeltaSeconds, .1);
    if (Runtime->RideTime >= Frames.back().time) { Runtime->RideTime = Frames.back().time; Runtime->Paused = true; }
    while (Runtime->TraceIndex + 1 < Frames.size() && Frames[Runtime->TraceIndex + 1].time <= Runtime->RideTime) ++Runtime->TraceIndex;
    const auto& A = Frames[Runtime->TraceIndex]; const auto& B = Frames[FMath::Min(Runtime->TraceIndex + 1, Frames.size() - 1)];
    const double Alpha = B.time > A.time ? FMath::Clamp((Runtime->RideTime - A.time) / (B.time - A.time), 0., 1.) : 0;
    Runtime->Distance = FMath::Lerp(A.distance, B.distance, Alpha); Runtime->Speed = FMath::Lerp(A.speed, B.speed, Alpha);
    const bool Moved = Runtime->PresentedDistance != Runtime->Distance;
    if (Runtime->TrainPoseDirty || Moved)
    {
        const double Half = (D.request.train.cars - 1) * D.request.train.spacing * .5;
        Runtime->CarTransforms.Reset(D.request.train.cars);
        for (int32 Car = 0; Car < D.request.train.cars; ++Car)
        {
            const auto P = D.track.sample(Runtime->Distance + Half - Car * D.request.train.spacing);
            // Imported centimetre-sized cars retain the canonical rail pivot,
            // finite-train spacing and the exact sample used before batching.
            Runtime->CarTransforms.Emplace(VibeMesh::Rotation(P), VibeMesh::Position(P.position), FVector::OneVector);
        }
        // UE 5.8's instance data manager marks changed instance transforms and
        // updates bounds/GPU data at end of frame. Recreating the entire scene
        // proxy with MarkRenderStateDirty each tick defeats that incremental path.
        Active->Cars->BatchUpdateInstancesTransforms(0, Runtime->CarTransforms, false, false, true);
        Runtime->TrainPoseDirty = false;
    }
    if (Runtime->CameraPoseDirty || Moved || Runtime->Overview)
    {
        if (Runtime->Overview)
        {
            const FVector Centre = Runtime->Bounds.GetCenter(); const double Radius = Runtime->Bounds.GetExtent().Size();
            int32 Width = 0, Height = 0;
            if (auto* PC = GetWorld()->GetFirstPlayerController()) PC->GetViewportSize(Width, Height);
            const auto* Lens = Camera->GetCameraComponent();
            const double Aspect = Width > 0 && Height > 0 ? double(Width) / Height : Lens->AspectRatio;
            const double HalfHorizontal = FMath::DegreesToRadians(Lens->FieldOfView * .5);
            const double HalfVertical = std::atan(std::tan(HalfHorizontal) / Aspect);
            // Fit the ride's bounding sphere in both viewport dimensions,
            // including after a resize. The previous fixed distance clipped it.
            const double Distance = 1.1 * Radius / std::sin(FMath::Min(HalfHorizontal, HalfVertical));
            const FVector Location = Centre + FVector(-.8, .6, .9).GetSafeNormal() * Distance;
            Camera->SetActorLocationAndRotation(Location, (Centre - Location).Rotation());
        }
        else
        {
            const double Offset = coaster::seatDistanceOffset(D.request.train, Runtime->Seat);
            const auto P = D.track.sample(Runtime->Distance + Offset);
            Camera->SetActorLocationAndRotation(VibeMesh::Position(P.position + P.up * D.request.train.seatHeight), VibeMesh::Rotation(P));
        }
        Runtime->CameraPoseDirty = false;
    }
    Runtime->PresentedDistance = Runtime->Distance;
}
FString AVibeCoasterWorld::Status() const
{
    if (Runtime->Prepared) return FString::Printf(TEXT("Validated; committing render chunks %d / %d. Esc cancels."), Runtime->NextChunk, Runtime->Prepared->Chunks.Num());
    if (Runtime->Running && Runtime->JobState && !Runtime->JobState->Cancel.load())
        return Runtime->Message + FString::Printf(TEXT("\n%s | candidate %d | Esc cancels"), Runtime->JobState->Stage.load() ? TEXT("Preparing meshes") : TEXT("CPU job"), Runtime->JobState->Candidate.load());
    return Runtime->Message;
}
TArray<FString> AVibeCoasterWorld::Comparison() const
{
    if (!Runtime->Design) return {TEXT("No accepted ride to compare.")};
    return Runtime->ComparisonLines;
}
FString AVibeCoasterWorld::Telemetry() const
{
    if (!Runtime->Design || Runtime->Design->simulation.frames.empty()) return TEXT("No accepted trace.");
    const auto& F = Runtime->Design->simulation.frames[Runtime->TraceIndex].seats[Runtime->Seat];
    const auto& Metrics = Runtime->Design->simulation.metrics;
    const auto& D = *Runtime->Design;
    const auto Point = D.track.sample(Runtime->Distance + coaster::seatDistanceOffset(D.request.train, Runtime->Seat));
    const double LocalHeight = Point.position.z - D.request.terrain.height(Point.position.x, Point.position.y);
    return FString::Printf(TEXT("%.1f km/h  |  %.1f s  |  %.0f m along track  |  %.1f m above ground\nVertical %.2f g  |  Lateral %.2f g  |  Longitudinal %.2f g\nPeak track height above ground %.1f m  |  Above station %.1f m\nCore simulation trace; provisional game envelope"),
        Runtime->Speed * 3.6, Runtime->RideTime, Runtime->Distance, LocalHeight, F.vertical, F.lateral, F.longitudinal, Metrics.maxGroundHeight, Metrics.heightAboveStation);
}
bool AVibeCoasterWorld::IsBusy() const { return Runtime->Running || Runtime->Queued.IsSet() || Runtime->Prepared.IsValid(); }
bool AVibeCoasterWorld::HasRide() const { return bool(Runtime->Design); }
bool AVibeCoasterWorld::IsPaused() const { return Runtime->Paused; }
bool AVibeCoasterWorld::IsOverview() const { return Runtime->Overview; }
int32 AVibeCoasterWorld::Seat() const { return Runtime->Seat; }
const coaster::Design* AVibeCoasterWorld::ActiveDesign() const { return Runtime->Design.get(); }







FCoasterPlaybackObservation AVibeCoasterWorld::Playback() const { return {Runtime->RideTime, Runtime->Distance, Runtime->Speed}; }

uint64 AVibeCoasterWorld::GeometryRevision() const { return Runtime->GeometryRevision; }
