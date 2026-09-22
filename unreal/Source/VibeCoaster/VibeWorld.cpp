#include "VibeWorld.h"
#include "VibeMesh.h"
#include "ProceduralMeshComponent.h"
#include "VibeTrace.h"
#include "Camera/CameraTypes.h"
#include "Engine/DirectionalLight.h"
#include "Engine/SkyLight.h"
#include "Components/SkyAtmosphereComponent.h"
#include "Components/DirectionalLightComponent.h"
#include "Components/SkyLightComponent.h"
#include "Engine/Engine.h"
#include "Engine/GameViewportClient.h"
#include "Framework/Application/SlateApplication.h"
#include "Rendering/SlateRenderer.h"
#include "Widgets/SOverlay.h"
#include "Widgets/Layout/SBorder.h"
#include "Widgets/Layout/SBox.h"
#include "Widgets/SBoxPanel.h"
#include "Widgets/Input/SButton.h"
#include "Widgets/Input/SSpinBox.h"
#include "Widgets/Input/SComboBox.h"
#include "Widgets/Text/STextBlock.h"
#include "Misc/Paths.h"
#include "Misc/FileHelper.h"
#include "Misc/CommandLine.h"
#include "Misc/Parse.h"
#include "HAL/FileManager.h"
#include "RHICommandList.h"
#include "UnrealClient.h"
#include "Modules/ModuleManager.h"
#include "UObject/StrongObjectPtr.h"
#include "MaterialShared.h"
#include "Materials/MaterialInterface.h"
#include "ShaderCompiler.h"
#include "RenderingThread.h"
#include <atomic>
#include <future>
#include <chrono>

IMPLEMENT_PRIMARY_GAME_MODULE(FDefaultGameModuleImpl, VibeCoaster, "VibeCoaster");
namespace {
FString Quote(FString S) {
    S.ReplaceInline(TEXT("\\"), TEXT("\\\\"));
    S.ReplaceInline(TEXT("\""), TEXT("\\\""));
    S.ReplaceInline(TEXT("\n"), TEXT("\\n"));
    return TEXT("\"") + S + TEXT("\"");
}
struct FJob {
    std::atomic<bool> Cancel{false};
    std::atomic<int> Phase{0};
    std::shared_ptr<coaster::Design> Design;
    std::unique_ptr<FVibeMeshes> Meshes;
    FString Error;
    double NativeSeconds = 0, MeshSeconds = 0;
};
struct FReady {
    std::atomic<const SWindow *> Window{nullptr};
    std::atomic<int> Request{0}, Frames{0}, AllFrames{0};
    std::atomic<bool> Complete{false};
    FGPUFenceRHIRef Fence;
    TRefCountPtr<FMaterial> Material;
    std::atomic<bool> CheckMaterial{false};
};
struct FVibeScene {
    TWeakObjectPtr<AActor> Root;
    TArray<TWeakObjectPtr<UProceduralMeshComponent>> Cars;
    TWeakObjectPtr<UVibeTrace> Trace;
    std::shared_ptr<coaster::Design> Design;
};
void ReleaseScene(FVibeScene &Scene) {
    if (Scene.Root.IsValid()) {
        TInlineComponentArray<UProceduralMeshComponent *> Components;
        Scene.Root->GetComponents(Components);
        for (auto *C : Components)
            C->ClearAllMeshSections();
        if (Scene.Trace.IsValid())
            Scene.Trace->Clear();
        Scene.Root->Destroy();
    }
    Scene = {};
}
UProceduralMeshComponent *Component(AActor *Root, const FVibeMesh &M, UMaterialInterface *Material) {
    auto *C = NewObject<UProceduralMeshComponent>(Root);
    C->SetupAttachment(Root->GetRootComponent());
    Root->AddInstanceComponent(C);
    C->RegisterComponent();
    C->SetMobility(EComponentMobility::Movable);
    C->SetCollisionEnabled(ECollisionEnabled::NoCollision);
    C->CreateMeshSection_LinearColor(0, M.Vertices, M.Indices, M.Normals, M.UV, M.Colors, {}, false);
    C->SetMaterial(0, Material);
    return C;
}
UVibeTrace *OverviewTrace(AActor *Root, const coaster::Track &Track) {
    auto *Trace = NewObject<UVibeTrace>(Root);
    Trace->SetupAttachment(Root->GetRootComponent());
    Root->AddInstanceComponent(Trace);
    Trace->SetTrack(Track);
    Trace->RegisterComponent();
    return Trace;
}
double RideDistance(const coaster::Design &D, double Time) {
    const auto &Frames = D.baseline->playback;
    auto B = std::lower_bound(Frames.begin(), Frames.end(), Time,
                              [](const auto &F, double T) { return F.time < T; });
    if (B == Frames.begin())
        return B->s;
    if (B == Frames.end())
        return Frames.back().s;
    const auto &A = *(B - 1);
    return std::lerp(A.s, B->s, (Time - A.time) / (B->time - A.time));
}
double RideSpeed(const coaster::Design &D, double Time) {
    const auto &Frames = D.baseline->playback;
    auto B = std::lower_bound(Frames.begin(), Frames.end(), Time,
                              [](const auto &F, double T) { return F.time < T; });
    return B == Frames.end() ? Frames.back().speed : B->speed;
}
} // namespace
struct FVibeState {
    coaster::Recipe Recipe;
    std::shared_ptr<FJob> Job;
    std::future<void> Worker;
    std::shared_ptr<FReady> Ready = std::make_shared<FReady>();
    FVibeScene Active, Staged;
    std::shared_ptr<coaster::Design> ViewDesign;
    TSharedPtr<SWidget> Menu;
    TArray<TSharedPtr<FString>> Styles;
    TSharedPtr<FString> SelectedStyle;
    TSharedPtr<SComboBox<TSharedPtr<FString>>> StylePicker;
    TStrongObjectPtr<UMaterialInterface> Material;
    FDelegateHandle PresentHandle;
    FString Status = TEXT("Author a ride or load a saved design."), SavePath, InputPath, Output;
    double RideTime = 0, Started = FPlatformTime::Seconds(), RequestStarted = 0, LastTick = Started,
           NextAction = 0;
    bool Paused = true, Loading = false, Committing = false, Auto = false, Quit = false, MenuReady = false;
    bool RideVerify = false, FlowVerify = false, FlowCancelIssued = false, AutoLoad = false;
    int FlowStep = 0, FlowFrameStart = 0;
    bool FlowExpectedFailure = false;
    FString FlowOriginalPath, FlowInvalidPath;
    double FlowNextTime = 0, FlowRideStart = 0;
    std::shared_ptr<coaster::Design> FlowPrior;
    int RidePass = 0, RideFrameStart = 0, NextRideShot = 0;
    double RideWallStart = 0;
    bool AwaitResources = false;
    double NextMaterialReport = 0, SceneCommitStarted = 0;
    int View = 0, Upload = 0, Cycles = 1, Completed = 0;
};
AVibeWorld::AVibeWorld() {
    PrimaryActorTick.bCanEverTick = true;
    RootComponent = CreateDefaultSubobject<USceneComponent>(TEXT("Root"));
    State = MakeShared<FVibeState>();
}
AVibeWorld::~AVibeWorld() = default;
AVibeGameMode::AVibeGameMode() {
    PlayerControllerClass = AVibeController::StaticClass();
    DefaultPawnClass = nullptr;
}
void AVibeController::BeginPlay() {
    Super::BeginPlay();
    bShowMouseCursor = true;
    auto *W = GetWorld()->SpawnActor<AVibeWorld>();
    SetViewTarget(W);
    SetInputMode(FInputModeGameAndUI().SetHideCursorDuringCapture(false));
}
void AVibeController::SetupInputComponent() {
    Super::SetupInputComponent();
    InputComponent->BindKey(EKeys::SpaceBar, IE_Pressed, this, &AVibeController::PauseRide);
    InputComponent->BindKey(EKeys::R, IE_Pressed, this, &AVibeController::RestartRide);
    InputComponent->BindKey(EKeys::V, IE_Pressed, this, &AVibeController::ChangeView);
}
void AVibeController::PauseRide() {
    if (auto *W = Cast<AVibeWorld>(GetViewTarget()))
        W->TogglePause();
}
void AVibeController::RestartRide() {
    if (auto *W = Cast<AVibeWorld>(GetViewTarget()))
        W->Restart();
}
void AVibeController::ChangeView() {
    if (auto *W = Cast<AVibeWorld>(GetViewTarget()))
        W->CycleView();
}
void AVibeWorld::BeginPlay() {
    Super::BeginPlay();
    auto &S = *State;
    S.SavePath = FPaths::ConvertRelativePathToFull(FPaths::ProjectSavedDir() / TEXT("Designs/Current.vcd"));
    IFileManager::Get().MakeDirectory(*FPaths::GetPath(S.SavePath), true);
    S.InputPath = S.SavePath;
    FParse::Value(FCommandLine::Get(), TEXT("VibeLoad="), S.InputPath);
    FParse::Value(FCommandLine::Get(), TEXT("VibeVerify="), S.Output);
    FParse::Value(FCommandLine::Get(), TEXT("VibeCycles="), S.Cycles);
    S.Cycles = FMath::Clamp(S.Cycles, 1, 2000);
    S.Quit = FParse::Param(FCommandLine::Get(), TEXT("VibeQuit"));
    S.Auto = !S.Output.IsEmpty();
    S.AutoLoad = FParse::Param(FCommandLine::Get(), TEXT("VibeAutoLoad"));
    S.RideVerify = FParse::Param(FCommandLine::Get(), TEXT("VibeRideVerify"));
    S.FlowVerify = FParse::Param(FCommandLine::Get(), TEXT("VibeFlowVerify"));
    if (S.FlowVerify)
        S.RideVerify = false;
    if (S.RideVerify || S.FlowVerify)
        S.Cycles = 1;
    if (S.Auto) {
        S.Output = FPaths::ConvertRelativePathToFull(S.Output);
        IFileManager::Get().MakeDirectory(*S.Output, true);
    }
    S.Material.Reset(
        LoadObject<UMaterialInterface>(nullptr, TEXT("/Game/Materials/VertexSurface.VertexSurface")));
#if WITH_EDITOR
    if (S.Material.IsValid())
        S.Material->CacheShaders(EMaterialShaderPrecompileMode::Default);
#endif
    auto *Sun = GetWorld()->SpawnActor<ADirectionalLight>();
    Sun->GetLightComponent()->SetMobility(EComponentMobility::Movable);
    Sun->SetActorRotation(FRotator(-42, 34, 0));
    Sun->GetLightComponent()->SetIntensity(80000);
    Cast<UDirectionalLightComponent>(Sun->GetLightComponent())->SetAtmosphereSunLight(true);
    Sun->GetLightComponent()->SetLightColor(FLinearColor(1.f, .91f, .78f));
    auto *Sky = GetWorld()->SpawnActor<ASkyLight>();
    Sky->GetLightComponent()->SetMobility(EComponentMobility::Movable);
    Sky->GetLightComponent()->SetIntensity(1.4f);
    Sky->GetLightComponent()->SetRealTimeCaptureEnabled(true);
    GetWorld()->SpawnActor<ASkyAtmosphere>();
    S.Ready->Material =
        S.Material.IsValid() ? S.Material->GetMaterialResource(GMaxRHIShaderPlatform) : nullptr;
    const auto Ready = S.Ready;
    S.PresentHandle = FSlateApplication::Get().GetRenderer()->OnBackBufferReadyToPresent().AddLambda(
        [Ready](SWindow &Window, ISlateViewportProvider &) {
            if (&Window != Ready->Window.load())
                return;
            ++Ready->AllFrames;
            if (!Ready->Request.load(std::memory_order_acquire))
                return;
            // Use the renderer's actual shader-map completeness contract.
            // The editor's broader compilation-finalized flag can remain
            // pending for an on-demand map after its required shaders exist.
            if (Ready->CheckMaterial.load() &&
                (!Ready->Material || !Ready->Material->GetRenderingThreadShaderMap() ||
                 !Ready->Material->IsRenderingThreadShaderMapComplete())) {
                Ready->Frames = 0;
                return;
            }
            const int Frame = ++Ready->Frames;
            if (Frame == 2) {
                Ready->Fence = RHICreateGPUFence(TEXT("VibeSceneRendered"));
                FRHICommandListExecutor::GetImmediateCommandList().WriteGPUFence(Ready->Fence);
            } else if (Frame > 2 && Ready->Fence.IsValid() && Ready->Fence->Poll()) {
                Ready->Complete.store(true, std::memory_order_release);
                Ready->Request.store(0);
            }
        });
    FParse::Value(FCommandLine::Get(), TEXT("VibeView="), S.View);
    S.View = FMath::Clamp(S.View, 0, 2);
    BuildMenu();
    Event(TEXT("startup"), TEXT(",\"engine\":") + Quote(FEngineVersion::Current().ToString()) +
                               TEXT(",\"save\":") + Quote(S.SavePath));
    S.Ready->Request = 1;
}
void AVibeWorld::Event(const FString &Name, const FString &Fields) {
    if (State->Output.IsEmpty())
        return;
    const FString Row =
        TEXT("{\"event\":") + Quote(Name) +
        FString::Printf(TEXT(",\"wall\":%.9f,\"utc_ticks\":\"%lld\""),
                        FPlatformTime::Seconds() - State->Started, FDateTime::UtcNow().GetTicks()) +
        Fields + TEXT("}\n");
    FFileHelper::SaveStringToFile(Row, *(State->Output / TEXT("events.jsonl")),
                                  FFileHelper::EEncodingOptions::ForceUTF8WithoutBOM, &IFileManager::Get(),
                                  FILEWRITE_Append);
    UE_LOG(LogTemp, Display, TEXT("VibeVerify %s"), *Row);
}
void AVibeWorld::Request(bool Load) {
    auto &S = *State;
    if (S.Loading)
        return;
    S.Job = std::make_shared<FJob>();
    S.Loading = true;
    S.Committing = false;
    S.Upload = 0;
    S.RequestStarted = FPlatformTime::Seconds();
    S.Status = Load ? TEXT("Validating saved ride…") : TEXT("Authoring ride…");
    Event(TEXT("request"), FString::Printf(TEXT(",\"load\":%s,\"cycle\":%d"),
                                           Load ? TEXT("true") : TEXT("false"), S.Completed));
    const auto Job = S.Job;
    const auto Recipe = S.Recipe;
    const auto Path = std::filesystem::path(*S.InputPath);
    S.Worker = std::async(std::launch::async, [Job, Recipe, Path, Load] {
        try {
            const double Started = FPlatformTime::Seconds();
            auto Cancel = [Job] { return Job->Cancel.load(); };
            Job->Phase = 0;
            auto D = Load ? coaster::loadDesign(Path, Cancel) : coaster::generate(Recipe, Cancel);
            Job->Phase = 1;
            if (!Load)
                coaster::validateRide(D, Cancel);
            if (!D.validation)
                throw std::runtime_error("Fresh ride validation evidence is missing");
            Job->NativeSeconds = FPlatformTime::Seconds() - Started;
            Job->Design = std::make_shared<coaster::Design>(std::move(D));
            Job->Phase = 2;
            Job->Meshes = std::make_unique<FVibeMeshes>(BuildVibeMeshes(*Job->Design, Cancel));
            Job->MeshSeconds = FPlatformTime::Seconds() - Started - Job->NativeSeconds;
            Job->Phase = 3;
        } catch (const coaster::Cancelled &) {
            Job->Error = TEXT("Cancelled");
        } catch (const std::exception &E) {
            Job->Error = UTF8_TO_TCHAR(E.what());
        }
    });
}
void AVibeWorld::Cancel() {
    auto &S = *State;
    if (!S.Loading)
        return;
    if (S.Job)
        S.Job->Cancel = true;
    if (S.Committing) {
        S.Ready->Request = 0;
        ReleaseScene(S.Staged);
        if (S.Active.Root.IsValid())
            S.Active.Root->SetActorHiddenInGame(false);
        S.ViewDesign = S.Active.Design;
        S.Committing = false;
        S.Loading = false;
        S.Status = TEXT("Cancelled. Previous ride retained.");
        Event(TEXT("cancelled"));
    }
}
void AVibeWorld::Save() {
    auto &S = *State;
    if (S.Loading || !S.Active.Design)
        return;
    // Saving has a separate worker and the same cancellation contract; the
    // visible scene is never replaced by a save operation.
    S.Job = std::make_shared<FJob>();
    const auto Job = S.Job;
    const auto D = S.Active.Design;
    const auto Path = std::filesystem::path(*S.SavePath);
    S.Loading = true;
    S.RequestStarted = FPlatformTime::Seconds();
    S.Status = TEXT("Saving ride…");
    Job->Phase = 4;
    S.Worker = std::async(std::launch::async, [Job, D, Path] {
        try {
            coaster::saveDesign(*D, Path, [Job] { return Job->Cancel.load(); });
            Job->Phase = 5;
        } catch (const std::exception &E) {
            Job->Error = UTF8_TO_TCHAR(E.what());
        }
    });
}
void AVibeWorld::Tick(float DeltaSeconds) {
    Super::Tick(DeltaSeconds);
    auto &S = *State;
    const double Now = FPlatformTime::Seconds();
    S.LastTick = Now;
#if WITH_EDITOR
    // Standalone editor-game previews must apply completed asynchronous material maps.
    if (GShaderCompilingManager)
        GShaderCompilingManager->ProcessAsyncResults(.002f, false);
#endif
    S.Ready->Window = GEngine->GameViewport->GetWindow().Get();
    if (!S.MenuReady && S.Ready->Complete.load(std::memory_order_acquire)) {
        S.MenuReady = true;
        S.Ready->Complete = false;
        Event(TEXT("ui-gpu-ready"));
        if (S.Auto || S.AutoLoad)
            Request(IFileManager::Get().FileExists(*S.InputPath));
    }
    if (S.Active.Design && !S.Paused)
        S.RideTime = std::min(S.RideTime + double(DeltaSeconds), S.Active.Design->baseline->duration);
    if (S.Loading && !S.Committing && S.Worker.valid() &&
        S.Worker.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
        S.Worker.get();
        if (S.Job->Phase == 5 && S.Job->Error.IsEmpty()) {
            // Cancellation after the atomic rename cannot undo a completed
            // save. Report the actual outcome instead of claiming cancellation.
            S.Status = TEXT("Saved. Ride remains ready.");
            S.InputPath = S.SavePath;
            S.Loading = false;
            Event(TEXT("saved"));
        } else if (!S.Job->Error.IsEmpty() || S.Job->Cancel) {
            S.Loading = false;
            const bool WasCancelled = S.Job->Cancel || S.Job->Error == TEXT("Cancelled");
            S.Status = WasCancelled ? TEXT("Cancelled. Previous ride retained.")
                                    : S.Job->Error + TEXT(". Previous ride retained.");
            const bool ExpectedFailure =
                S.FlowVerify && S.FlowExpectedFailure && S.Job->Error.Contains(TEXT("integrity"));
            Event(WasCancelled ? TEXT("cancelled") : TEXT("request-failed"),
                  TEXT(",\"error\":") + Quote(S.Status));
            if (S.Auto && S.Quit && !ExpectedFailure && !(WasCancelled && S.FlowVerify && S.FlowCancelIssued))
                FPlatformMisc::RequestExitWithStatus(false, 2);
        } else {
            S.Staged.Root = GetWorld()->SpawnActor<AActor>();
            auto *Root = NewObject<USceneComponent>(S.Staged.Root.Get());
            S.Staged.Root->SetRootComponent(Root);
            S.Staged.Root->AddInstanceComponent(Root);
            Root->RegisterComponent();
            S.Staged.Root->SetActorHiddenInGame(true);
            S.Staged.Design = S.Job->Design;
            S.Committing = true;
            S.Upload = 0;
            S.Status = TEXT("Preparing scene…");
            Event(TEXT("native-ready"),
                  FString::Printf(TEXT(",\"seconds\":%.9f,\"worker_native\":%.9f,\"worker_mesh\":%.9f"),
                                  Now - S.RequestStarted, S.Job->NativeSeconds, S.Job->MeshSeconds));
        }
    }
    if (S.Committing && S.Upload < 5) {
        auto &M = *S.Job->Meshes;
        FVibeMesh *Parts[] = {&M.Terrain, &M.Supports, &M.Rails, &M.Ties, &M.Station};
        Component(S.Staged.Root.Get(), *Parts[S.Upload], S.Material.Get());
        ++S.Upload;
        if (S.Upload == 5) {
            S.Staged.Trace = OverviewTrace(S.Staged.Root.Get(), S.Staged.Design->track);
            S.Staged.Trace->SetVisibility(S.View == 0);
            const auto Car = BuildVibeCar();
            for (int I = 0; I < 6; ++I)
                S.Staged.Cars.Add(Component(S.Staged.Root.Get(), Car, S.Material.Get()));

            S.Ready->Complete = false;
            S.Ready->Frames = 0;
            S.Ready->Request.store(0, std::memory_order_release);
            S.AwaitResources = true;
            S.Status = TEXT("Preparing scene materials…");
            Event(TEXT("scene-submit"));
        }
    }
    if (S.Committing && S.Upload == 5 && S.AwaitResources) {
        const auto *Resource =
            S.Material.IsValid() ? S.Material->GetMaterialResource(GMaxRHIShaderPlatform) : nullptr;
#if WITH_EDITOR
        if (Resource)
            Resource->IsCompilationFinished();
#endif
        const bool ShadersReady = Resource && Resource == S.Ready->Material.GetReference() &&
                                  Resource->GetGameThreadShaderMap() &&
                                  Resource->IsGameThreadShaderMapComplete();
        if (!ShadersReady && Now >= S.NextMaterialReport) {
            Event(TEXT("material-wait"),
                  FString::Printf(
                      TEXT(",\"jobs\":%d,\"pending\":%d,\"resource\":%d,\"valid_map\":%d,\"complete_map\":%d,"
                           "\"errors\":%d,\"platform\":%d"),
                      GShaderCompilingManager ? GShaderCompilingManager->GetNumOutstandingJobs() : 0,
                      GShaderCompilingManager ? GShaderCompilingManager->GetNumPendingJobs() : 0,
                      Resource != nullptr, Resource ? Resource->HasValidGameThreadShaderMap() : false,
                      Resource ? Resource->IsGameThreadShaderMapComplete() : false,
                      Resource ? Resource->GetCompileErrors().Num() : 0, int(GMaxRHIShaderPlatform)));
            S.NextMaterialReport = Now + 2;
        }
        if (ShadersReady) {
            // Keep the previous ride visible and playable throughout all
            // CPU preparation and material preparation. This is the visual
            // commit; retain the previous scene until the GPU acknowledges it.
            S.Staged.Root->SetActorHiddenInGame(false);
            if (S.Active.Root.IsValid())
                S.Active.Root->SetActorHiddenInGame(true);
            S.ViewDesign = S.Staged.Design;
            S.SceneCommitStarted = Now;
            Event(TEXT("scene-commit"));
            S.AwaitResources = false;
            S.Ready->CheckMaterial = true;
            S.Ready->Complete = false;
            S.Ready->Frames = 0;
            S.Ready->Request.store(1, std::memory_order_release);
            S.Status = TEXT("Waiting for rendered scene…");
            Event(TEXT("materials-ready"));
        } else if (Now - S.RequestStarted > 30) {
            Cancel();
            S.Status = TEXT("Scene materials were not ready. Previous ride retained.");
            Event(TEXT("request-failed"), TEXT(",\"error\":") + Quote(S.Status));
            if (S.Auto && S.Quit)
                FPlatformMisc::RequestExitWithStatus(false, 2);
        }
    }
    if (S.Committing && S.Upload == 5 && !S.AwaitResources &&
        !S.Ready->Complete.load(std::memory_order_acquire) && Now - S.SceneCommitStarted > 30) {
        Cancel();
        S.Status = TEXT("The renderer did not acknowledge the scene. Previous ride retained.");
        Event(TEXT("request-failed"), TEXT(",\"error\":") + Quote(S.Status));
        if (S.Auto && S.Quit)
            FPlatformMisc::RequestExitWithStatus(false, 2);
    }
    if (S.Committing && S.Upload == 5 && !S.AwaitResources &&
        S.Ready->Complete.load(std::memory_order_acquire)) {
        ReleaseScene(S.Active);
        S.Active = std::move(S.Staged);
        S.Staged = {};
        S.ViewDesign = S.Active.Design;
        S.Recipe = S.Active.Design->recipe;
        for (const auto &Style : S.Styles)
            if (*Style == UTF8_TO_TCHAR(S.Recipe.style.c_str()))
                S.StylePicker->SetSelectedItem(Style);
        S.RideTime = 0;
        S.Paused = true;
        S.Committing = false;
        S.Loading = false;
        ++S.Completed;
        S.Status = TEXT("Preview ready \u00b7 hardware checks pending");
        S.Job.reset();
        const double CompletedAt = FPlatformTime::Seconds();
        Event(TEXT("gpu-ready"),
              FString::Printf(TEXT(",\"seconds\":%.9f,\"rendered_frames\":%d,\"cycle\":%d"),
                              CompletedAt - S.RequestStarted, S.Ready->Frames.load(), S.Completed));
        if (S.Auto) {
            FScreenshotRequest::RequestScreenshot(
                S.Output / FString::Printf(TEXT("scene-%04d.png"), S.Completed), true, false);
            if (S.RideVerify) {
                S.RidePass = 1;
                S.View = 1;
                S.RideTime = 0;
                S.Paused = false;
                S.NextRideShot = 0;
                S.RideFrameStart = S.Ready->AllFrames.load();
                S.RideWallStart = Now;
                Event(TEXT("ride-start"), TEXT(",\"view\":\"front\""));
            } else
                S.NextAction = Now + .3;
        }
    }
    if (S.RideVerify && !S.Loading && S.Active.Design && S.RidePass >= 1 && S.RidePass <= 2) {
        constexpr std::array<double, 15> Captures{5,   30,  46,  54,  70,  76,  86, 91,
                                                  102, 109, 126, 139, 160, 177, 189};
        const TCHAR *ViewName = S.RidePass == 1 ? TEXT("front") : TEXT("rear");
        if (S.NextRideShot < int(Captures.size()) && S.RideTime >= Captures[S.NextRideShot]) {
            FScreenshotRequest::RequestScreenshot(
                S.Output / FString::Printf(TEXT("%s-%03d.png"), ViewName, int(Captures[S.NextRideShot])),
                false, false);
            Event(TEXT("ride-capture"),
                  FString::Printf(TEXT(",\"view\":\"%s\",\"ride_time\":%.6f,\"rendered_frames\":%d"),
                                  ViewName, S.RideTime, S.Ready->AllFrames.load() - S.RideFrameStart));
            ++S.NextRideShot;
        }
        if (S.RideTime >= S.Active.Design->baseline->duration) {
            Event(
                TEXT("ride-complete"),
                FString::Printf(
                    TEXT(",\"view\":\"%s\",\"ride_time\":%.6f,\"wall_seconds\":%.6f,\"rendered_frames\":%d"),
                    ViewName, S.RideTime, Now - S.RideWallStart,
                    S.Ready->AllFrames.load() - S.RideFrameStart));
            ++S.RidePass;
            if (S.RidePass == 2) {
                S.View = 2;
                S.RideTime = 0;
                S.NextRideShot = 0;
                S.RideFrameStart = S.Ready->AllFrames.load();
                S.RideWallStart = Now;
                Event(TEXT("ride-start"), TEXT(",\"view\":\"rear\""));
            } else {
                S.Paused = true;
                S.NextAction = Now + .5;
            }
        }
    }
    auto PlaceCars = [&](FVibeScene &Scene, double Time) {
        if (!Scene.Design)
            return;
        if (Scene.Trace.IsValid())
            Scene.Trace->SetVisibility(S.View == 0);
        const double Distance = RideDistance(*Scene.Design, Time);
        for (int I = 0; I < Scene.Cars.Num(); ++I)
            if (Scene.Cars[I].IsValid()) {
                const auto F = Scene.Design->track.at(Distance + 8.5 - 3.4 * I);
                Scene.Cars[I]->SetWorldLocationAndRotation(
                    VibePosition(F.p),
                    FRotationMatrix::MakeFromXZ(VibePosition(F.t), VibePosition(F.u)).Rotator());
            }
    };
    PlaceCars(S.Active, S.RideTime);
    if (S.Committing)
        PlaceCars(S.Staged, 0);
    if (S.FlowVerify)
        VerifyFlow(Now);
    if (S.Auto && !S.FlowVerify && !S.Loading && S.NextAction > 0 && Now >= S.NextAction) {
        S.NextAction = 0;
        if (S.Completed < S.Cycles)
            Request(true);
        else if (S.Quit) {
            Event(TEXT("finished"));
            FPlatformMisc::RequestExitWithStatus(false, 0);
        }
    }
}
void AVibeWorld::VerifyFlow(double Now) {
    auto &S = *State;
    auto Require = [&](bool Okay, const TCHAR *Message) {
        if (Okay)
            return true;
        S.Status = Message;
        Event(TEXT("flow-failed"), TEXT(",\"error\":") + Quote(S.Status));
        S.Auto = false;
        S.FlowVerify = false;
        S.Paused = true;
        if (S.Quit)
            FPlatformMisc::RequestExitWithStatus(false, 2);
        return false;
    };
    auto BeginCancellation = [&](int Step) {
        S.FlowStep = Step;
        S.FlowCancelIssued = false;
        S.FlowPrior = S.Active.Design;
        S.FlowRideStart = S.RideTime = 20;
        S.FlowFrameStart = S.Ready->AllFrames.load();
        S.Paused = false;
        S.FlowNextTime = Now + .08;
        Request(true);
    };
    if (S.FlowStep == 0) {
        if (S.Loading || !S.Active.Design)
            return;
        Event(TEXT("flow-start"));
        S.FlowOriginalPath = S.InputPath;
        S.Recipe.seed = 77;
        S.Recipe.style = "intense";
        S.FlowStep = 1;
        Request(false);
        return;
    }
    if (S.FlowStep == 1) {
        if (S.Loading)
            return;
        if (!Require(S.Active.Design && S.Active.Design->validation && S.Recipe.seed == 77 &&
                         S.Recipe.style == "intense",
                     TEXT("Generated recipe did not become ready")))
            return;
        S.FlowPrior = S.Active.Design;
        Event(TEXT("flow-generate-pass"));
        FScreenshotRequest::RequestScreenshot(S.Output / TEXT("flow-generated.png"), true, false);
        S.FlowStep = 2;
        Save();
        return;
    }
    if (S.FlowStep == 2) {
        if (S.Loading)
            return;
        if (!Require(S.Job && S.Job->Phase == 5 && IFileManager::Get().FileExists(*S.SavePath) &&
                         S.InputPath == S.SavePath,
                     TEXT("Saved design or the subsequent Load target was not committed")))
            return;
        Event(TEXT("flow-save-pass"));
        S.FlowStep = 3;
        Request(true);
        return;
    }
    if (S.FlowStep == 3) {
        if (S.Loading)
            return;
        const auto Selected = S.StylePicker->GetSelectedItem();
        if (!Require(S.Active.Design && S.Active.Design != S.FlowPrior && S.Active.Design->validation &&
                         S.Recipe.seed == 77 && S.Recipe.style == "intense" && Selected.IsValid() &&
                         *Selected == TEXT("intense") &&
                         std::abs(S.Active.Design->track.length - S.FlowPrior->track.length) < 1e-7,
                     TEXT("Saved round trip or authoring controls changed")))
            return;
        Event(TEXT("flow-roundtrip-pass"));
        S.InputPath = S.FlowOriginalPath;
        BeginCancellation(4);
        return;
    }
    if (S.FlowStep >= 4 && S.FlowStep <= 6) {
        if (!S.FlowCancelIssued && S.Loading) {
            const bool AtStage = S.FlowStep == 4   ? (!S.Committing && Now >= S.FlowNextTime)
                                 : S.FlowStep == 5 ? (S.Committing && S.Upload >= 2 && S.Upload < 5)
                                                   : (S.Committing && S.Upload == 5 && !S.AwaitResources);
            if (AtStage) {
                S.FlowCancelIssued = true;
                Cancel();
                S.FlowNextTime = Now + .08;
                return;
            }
        }
        if (S.Loading || Now < S.FlowNextTime)
            return;
        const int Frames = S.Ready->AllFrames.load() - S.FlowFrameStart;
        if (!Require(S.FlowCancelIssued && S.Active.Design == S.FlowPrior && S.ViewDesign == S.FlowPrior &&
                         S.Active.Root.IsValid() && !S.Active.Root->IsHidden() && !S.Committing &&
                         !S.Paused && S.RideTime > S.FlowRideStart && Frames >= 2,
                     TEXT("Cancellation did not preserve the playing previous ride")))
            return;
        const TCHAR *Stage = S.FlowStep == 4 ? TEXT("cpu") : S.FlowStep == 5 ? TEXT("upload") : TEXT("gpu");
        Event(TEXT("flow-cancel-pass"),
              FString::Printf(TEXT(",\"stage\":\"%s\",\"rendered_frames\":%d,\"ride_advanced\":%.6f"), Stage,
                              Frames, S.RideTime - S.FlowRideStart));
        if (S.FlowStep < 6)
            BeginCancellation(S.FlowStep + 1);
        else {
            TArray<uint8> Bytes;
            S.FlowInvalidPath = S.SavePath + TEXT(".invalid-test");
            if (!Require(FFileHelper::LoadFileToArray(Bytes, *S.SavePath) && Bytes.Num() > 32,
                         TEXT("Could not prepare the rejected-load fixture")))
                return;
            Bytes[Bytes.Num() / 2] ^= 1;
            if (!Require(FFileHelper::SaveArrayToFile(Bytes, *S.FlowInvalidPath),
                         TEXT("Could not write the rejected-load fixture")))
                return;
            S.FlowStep = 7;
            S.FlowExpectedFailure = true;
            S.FlowCancelIssued = false;
            S.FlowNextTime = Now + .08;
            S.FlowRideStart = S.RideTime;
            S.FlowFrameStart = S.Ready->AllFrames.load();
            S.InputPath = S.FlowInvalidPath;
            Request(true);
        }
        return;
    }
    if (S.FlowStep == 7) {
        if (S.Loading || Now < S.FlowNextTime)
            return;
        if (!Require(S.Job && S.Job->Error.Contains(TEXT("integrity")) && S.Active.Design == S.FlowPrior &&
                         S.ViewDesign == S.FlowPrior && S.Active.Root.IsValid() &&
                         !S.Active.Root->IsHidden() && !S.Paused && S.RideTime > S.FlowRideStart &&
                         S.Ready->AllFrames.load() - S.FlowFrameStart >= 2,
                     TEXT("Rejected loading did not preserve the playing previous ride")))
            return;
        IFileManager::Get().Delete(*S.FlowInvalidPath);
        S.InputPath = S.FlowOriginalPath;
        S.FlowExpectedFailure = false;
        S.FlowStep = 8;
        S.FlowNextTime = Now + .4;
        Event(TEXT("flow-rejection-pass"));
        FScreenshotRequest::RequestScreenshot(S.Output / TEXT("flow-retained.png"), true, false);
        Event(TEXT("flow-pass"));
        return;
    }
    if (S.FlowStep == 8 && Now >= S.FlowNextTime) {
        S.FlowStep = 9;
        S.NextAction = 0;
        S.Paused = true;
        Event(TEXT("finished"));
        if (S.Quit)
            FPlatformMisc::RequestExitWithStatus(false, 0);
    }
}
void AVibeWorld::TogglePause() {
    State->Paused = !State->Paused;
}
void AVibeWorld::Restart() {
    State->RideTime = 0;
    State->Paused = false;
}
void AVibeWorld::CycleView() {
    State->View = (State->View + 1) % 3;
}
void AVibeWorld::CalcCamera(float, FMinimalViewInfo &V) {
    auto &S = *State;
    const auto D = S.ViewDesign;
    V.FOV = 90;
    V.PostProcessBlendWeight = 1;
    V.PostProcessSettings.bOverride_AutoExposureMethod = true;
    V.PostProcessSettings.AutoExposureMethod = AEM_Manual;
    V.PostProcessSettings.bOverride_AutoExposureApplyPhysicalCameraExposure = true;
    V.PostProcessSettings.AutoExposureApplyPhysicalCameraExposure = true;
    V.PostProcessSettings.bOverride_CameraISO = true;
    V.PostProcessSettings.CameraISO = 100;
    V.PostProcessSettings.bOverride_CameraShutterSpeed = true;
    V.PostProcessSettings.CameraShutterSpeed = 125;
    V.PostProcessSettings.bOverride_DepthOfFieldFstop = true;
    V.PostProcessSettings.DepthOfFieldFstop = 16;
    if (!D || S.View == 0) {
        V.Location = FVector(140000, -175000, 120000);
        const FVector Target(30000, -35000, 8500);
        V.Rotation = (Target - V.Location).Rotation();
        V.FOV = 62;
        return;
    }
    const double Time = S.Committing ? 0 : S.RideTime;
    const auto F = D->track.at(RideDistance(*D, Time) + (S.View == 1 ? 8.5 : -8.5));
    V.Location = VibePosition(F.p + F.u * 1.65 + F.t * .75);
    V.Rotation = FRotationMatrix::MakeFromXZ(VibePosition(F.t), VibePosition(F.u)).Rotator();
    V.FOV = 94;
}
void AVibeWorld::BuildMenu() {
    auto &S = *State;
    for (const TCHAR *Name : {TEXT("balanced"), TEXT("flow"), TEXT("intense")})
        S.Styles.Add(MakeShared<FString>(Name));
    S.SelectedStyle = S.Styles[0];
    TSharedPtr<SVerticalBox> Fields;
    auto Button = [&](const TCHAR *Label, TFunction<void()> Action) {
        return SNew(SButton).ContentPadding(FMargin(12, 8)).OnClicked_Lambda([Action] {
            Action();
            return FReply::Handled();
        })[SNew(STextBlock).Text(FText::FromString(Label))];
    };
    auto Menu = SNew(SVerticalBox) +
                SVerticalBox::Slot().AutoHeight().Padding(
                    0, 0, 0, 4)[SNew(STextBlock)
                                    .Text(FText::FromString(TEXT("VIBECOASTER")))
                                    .Font(FCoreStyle::GetDefaultFontStyle("Bold", 23))] +
                SVerticalBox::Slot().AutoHeight().Padding(
                    0, 0, 0, 18)[SNew(STextBlock)
                                     .Text(FText::FromString(TEXT("ESCARPMENT / AUTHORING")))
                                     .ColorAndOpacity(FLinearColor(.94f, .59f, .3f))] +
                SVerticalBox::Slot().AutoHeight()[SAssignNew(Fields, SVerticalBox).IsEnabled_Lambda([this] {
                    return !State->Loading;
                })];
    auto Number = [&](const TCHAR *Label, double *Value, double Low, double High) {
        Fields->AddSlot().AutoHeight().Padding(
            0, 4)[SNew(SHorizontalBox) +
                  SHorizontalBox::Slot().FillWidth(1).VAlign(
                      VAlign_Center)[SNew(STextBlock).Text(FText::FromString(Label))] +
                  SHorizontalBox::Slot().AutoWidth()[SNew(SBox).WidthOverride(
                      95)[SNew(SSpinBox<double>)
                              .Value_Lambda([Value] { return *Value; })
                              .MinValue(Low)
                              .MaxValue(High)
                              .MinFractionalDigits(0)
                              .MaxFractionalDigits(2)
                              .OnValueChanged_Lambda([Value](double V) { *Value = V; })]]];
    };
    Fields->AddSlot().AutoHeight().Padding(
        0, 4)[SNew(SHorizontalBox) +
              SHorizontalBox::Slot().FillWidth(1)[SNew(STextBlock).Text(FText::FromString(TEXT("Seed")))] +
              SHorizontalBox::Slot().AutoWidth()[SNew(SBox).WidthOverride(
                  95)[SNew(SSpinBox<int64>)
                          .Value_Lambda([this] { return int64(State->Recipe.seed); })
                          .MinValue(0)
                          .MaxValue(4294967295LL)
                          .OnValueChanged_Lambda([this](int64 V) { State->Recipe.seed = unsigned(V); })]]];
    Fields->AddSlot().AutoHeight().Padding(
        0, 4)[SAssignNew(S.StylePicker, SComboBox<TSharedPtr<FString>>)
                  .OptionsSource(&S.Styles)
                  .InitiallySelectedItem(S.SelectedStyle)
                  .OnGenerateWidget_Lambda(
                      [](TSharedPtr<FString> V) { return SNew(STextBlock).Text(FText::FromString(*V)); })
                  .OnSelectionChanged_Lambda([this](TSharedPtr<FString> V, ESelectInfo::Type) {
                      if (!V.IsValid())
                          return;
                      State->SelectedStyle = V;
                      State->Recipe.style = TCHAR_TO_UTF8(**V);
                  })[SNew(STextBlock).Text_Lambda([this] {
                      return FText::FromString(UTF8_TO_TCHAR(State->Recipe.style.c_str()));
                  })]];
    Number(TEXT("Opening hill · m"), &S.Recipe.openingHeight, 55, 95);
    Number(TEXT("Clifftop · m"), &S.Recipe.plateau, 200, 220);
    Number(TEXT("Camelback · m"), &S.Recipe.camelbackHeight, 200, 240);
    Number(TEXT("Loop · m"), &S.Recipe.loopHeight, 100, 150);
    Number(TEXT("Immelmann · m"), &S.Recipe.immelmannHeight, 75, 120);
    Number(TEXT("Top speed · km/h"), &S.Recipe.topSpeedKph, 285, 310);
    Menu->AddSlot().AutoHeight().Padding(0, 16, 0,
                                         4)[Button(TEXT("Generate ride"), [this] { Request(false); })];
    Menu->AddSlot().AutoHeight().Padding(
        0, 3)[SNew(SHorizontalBox) + SHorizontalBox::Slot().FillWidth(1)[Button(TEXT("Load"), [this] {
        Request(true);
    })] + SHorizontalBox::Slot().FillWidth(1).Padding(5, 0, 0, 0)[Button(TEXT("Save"), [this] { Save(); })]];
    Menu->AddSlot().AutoHeight().Padding(0, 3)[Button(TEXT("Cancel operation"), [this] { Cancel(); })];
    Menu->AddSlot().AutoHeight().Padding(0, 12)[SNew(STextBlock).AutoWrapText(true).Text_Lambda([this] {
        auto &Q = *State;
        FString Text = Q.Status;
        if (Q.Loading)
            Text += FString::Printf(TEXT("  %.1f s"), FPlatformTime::Seconds() - Q.RequestStarted);
        return FText::FromString(Text);
    })];
    Menu->AddSlot().AutoHeight()[SNew(STextBlock)
                                     .Text(FText::FromString(
                                         TEXT("Orange: FVD   Blue: spatial spline\n180 s active · terminal "
                                              "brakes separate\nSpace: pause   R: restart   V: view")))
                                     .AutoWrapText(true)
                                     .ColorAndOpacity(FLinearColor(.68f, .72f, .74f))];
    S.Menu =
        SNew(SOverlay) +
        SOverlay::Slot()
            .HAlign(HAlign_Left)
            .VAlign(VAlign_Top)
            .Padding(24)[SNew(SBox).WidthOverride(290).Visibility_Lambda([this] {
                return State->View == 0 || State->Loading ? EVisibility::Visible : EVisibility::Collapsed;
            })[SNew(SBorder)
                   .BorderImage(FCoreStyle::Get().GetBrush("WhiteBrush"))
                   .Padding(20)
                   .BorderBackgroundColor(FLinearColor(.028f, .05f, .065f, .97f))[Menu]]] +
        SOverlay::Slot()
            .HAlign(HAlign_Center)
            .VAlign(VAlign_Bottom)
            .Padding(
                20)[SNew(SBorder)
                        .BorderImage(FCoreStyle::Get().GetBrush("WhiteBrush"))
                        .Padding(16, 10)
                        .BorderBackgroundColor(FLinearColor(
                            .028f, .05f, .065f,
                            .96f))[SNew(SHorizontalBox) +
                                   SHorizontalBox::Slot()
                                       .AutoWidth()[Button(TEXT("Overview"), [this] { State->View = 0; })] +
                                   SHorizontalBox::Slot().AutoWidth().Padding(
                                       6, 0)[Button(TEXT("Front view"), [this] { State->View = 1; })] +
                                   SHorizontalBox::Slot()
                                       .AutoWidth()[Button(TEXT("Rear view"), [this] { State->View = 2; })] +
                                   SHorizontalBox::Slot().AutoWidth().Padding(
                                       6, 0)[Button(TEXT("Play / pause"), [this] { TogglePause(); })] +
                                   SHorizontalBox::Slot()
                                       .AutoWidth()
                                       .VAlign(VAlign_Center)
                                       .Padding(15, 0)[SNew(STextBlock).Text_Lambda([this] {
                                           auto &Q = *State;
                                           return FText::FromString(FString::Printf(
                                               TEXT("%05.1f s   %03.0f km/h"), Q.RideTime,
                                               Q.Active.Design ? RideSpeed(*Q.Active.Design, Q.RideTime) * 3.6
                                                               : 0));
                                       })]]];
    GEngine->GameViewport->AddViewportWidgetContent(S.Menu.ToSharedRef(), 10);
}
void AVibeWorld::EndPlay(const EEndPlayReason::Type Reason) {
    auto &S = *State;
    if (S.Job)
        S.Job->Cancel = true;
    if (S.Worker.valid())
        S.Worker.wait();
    S.Ready->Request = 0;
    if (FSlateApplication::IsInitialized())
        FSlateApplication::Get().GetRenderer()->OnBackBufferReadyToPresent().Remove(S.PresentHandle);
    if (S.Menu.IsValid() && GEngine && GEngine->GameViewport)
        GEngine->GameViewport->RemoveViewportWidgetContent(S.Menu.ToSharedRef());
    S.Menu.Reset();
    S.StylePicker.Reset();
    ReleaseScene(S.Active);
    ReleaseScene(S.Staged);
    S.ViewDesign.reset();
    // The callback is removed above. Drain its queued draw work before
    // releasing the native material reference, prior to UObject teardown.
    FlushRenderingCommands();
    S.Ready->Material.SafeRelease();
    S.Ready->Fence.SafeRelease();
    S.Material.Reset();
    Super::EndPlay(Reason);
}
