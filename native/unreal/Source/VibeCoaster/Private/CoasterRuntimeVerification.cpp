#include "CoasterRuntimeVerification.h"
#include "CoasterBuildVersion.h"
#include "CoordinateContract.h"

#if !UE_BUILD_SHIPPING && !UE_BUILD_TEST
#include "VibeCoasterGame.h"
#include "VibeCoasterWorld.h"
#include "Engine/Engine.h"
#include "Engine/GameViewportClient.h"
#include "HAL/FileManager.h"
#include "HAL/PlatformProcess.h"
#include "HAL/PlatformTime.h"
#include "Misc/App.h"
#include "Misc/CommandLine.h"
#include "Misc/DateTime.h"
#include "Misc/EngineVersion.h"
#include "Misc/FileHelper.h"
#include "Misc/Parse.h"
#include "Misc/Paths.h"
#include "Misc/SecureHash.h"
#include "ProfilingDebugging/CsvProfiler.h"
#include "UnrealClient.h"
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <locale>
#include <sstream>

namespace
{
FString Q(FString Value)
{
    Value.ReplaceInline(TEXT("\\"), TEXT("\\\\")); Value.ReplaceInline(TEXT("\""), TEXT("\\\""));
    Value.ReplaceInline(TEXT("\r"), TEXT("\\r")); Value.ReplaceInline(TEXT("\n"), TEXT("\\n")); Value.ReplaceInline(TEXT("\t"), TEXT("\\t"));
    return TEXT("\"") + Value + TEXT("\"");
}
FString N(double Value) { return FMath::IsFinite(Value) ? FString::Printf(TEXT("%.17g"), Value) : TEXT("null"); }
FString HashFile(const FString& Path)
{
    TArray<uint8> Bytes;
    return FFileHelper::LoadFileToArray(Bytes, *Path) ? FSHA1::HashBuffer(Bytes.GetData(), Bytes.Num()).ToString() : FString();
}
// Diagnostic canonical geometry identity, independent of object pointers/caches.
// The complete on-disk save gets its own byte hash as well.
FString GeometryIdentity(const coaster::Design& D)
{
    std::ostringstream S; S.imbue(std::locale::classic()); S << std::setprecision(17);
    auto V = [&](coaster::Vec3 P) { S << P.x << ',' << P.y << ',' << P.z << ';'; };
    S << D.generationVersion << ';' << D.request.seed << ';' << int(D.request.terrain.kind) << ';' << D.track.closed << ';';
    const auto& Terrain = D.request.terrain;
    S << Terrain.centerX << ';' << Terrain.centerY << ';' << Terrain.heightMeters << ';' << Terrain.radiusX << ';' << Terrain.radiusY << ';' << Terrain.bend << ';';
    S << Terrain.ridge.spineCount << ';' << Terrain.ridge.width << ';' << Terrain.ridge.curvature << ';';
    for(const auto& P : Terrain.ridge.points) S << P.x << ';' << P.y << ';' << P.height << ';' << P.gx << ';' << P.gy << ';';
    S << Terrain.plateau << ';' << Terrain.cliffX << ';' << Terrain.cliffY << ';' << Terrain.cliffHeading << ';' << Terrain.cliffWidth << ';' << Terrain.cliffCurvature << ';';
    if(Terrain.backSlope){const auto& B=*Terrain.backSlope;S << "back:" << B.x << ';' << B.y << ';' << B.height << ';' << B.gradeX << ';' << B.gradeY << ';';}
    for (const auto& R : Terrain.ravines) S << R.x0 << ';' << R.y0 << ';' << R.x1 << ';' << R.y1 << ';' << R.depth0 << ';' << R.depth1 << ';' << R.width0 << ';' << R.width1 << ';';
    S << coaster::recipePayload(D.request.recipe);
    for (const auto& Section : D.sections) S << int(Section.role) << ';' << Section.recipeId << ';' << Section.start << ';' << Section.end << ';';
    for (const auto& Landmark : D.landmarks) S << int(Landmark.kind) << ';' << Landmark.distance << ';';
    for (const auto& R : Terrain.ramps) S << R.x0 << ';' << R.y0 << ';' << R.x1 << ';' << R.y1 << ';' << R.h0 << ';' << R.h1 << ';' << R.grade0 << ';' << R.grade1 << ';' << R.width << ';';
    for (const auto& K : Terrain.knolls) S << K.x << ';' << K.y << ';' << K.height << ';' << K.radius << ';';
    for (const auto& K : Terrain.foothills) S << K.x << ';' << K.y << ';' << K.height << ';' << K.radius << ';';
    const auto& T = D.request.train;
    S << T.cars << ';' << T.carMass << ';' << T.spacing << ';' << T.seatHeight << ';' << T.dragCdA << ';' << T.rollingResistance << ';' << T.airDensity << ';';
    for (const auto& K : D.track.knots) { V(K.position); V(K.tangent); V(K.curvature); V(K.third); V(K.fourth); V(K.up); V(K.upFirst); V(K.upSecond); V(K.upThird); S << K.bank << ';' << int(K.element) << ';'; }
    for (const auto& O : D.operations) S << O.start << ';' << O.end << ';' << int(O.kind) << ';' << O.targetSpeed << ';' << O.maxForce << ';' << O.maxPower << ';' << O.rampSeconds << ';' << O.stopDeceleration << ';' << O.stopOffset << ';' << O.exitFadeMeters << ';' << O.trimPeakSpeed << ';' << O.trimSensorLead << ';';
    for (const auto& P : D.supports) { V(P.base); V(P.top); V(P.attachment); S << P.hasAttachment << ';' << P.trackDistance << ';'; for (const auto& M : P.members) { V(M.base); V(M.top); S << M.radiusBase << ';' << M.radiusTop << ';' << int(M.kind) << ';' << M.spineContact << ';'; } }
    S << coaster::stationPayload(D.station);
    const std::string Text = S.str(); return FSHA1::HashBuffer(Text.data(), Text.size()).ToString();
}
}

struct FCoasterRuntimeVerification::FState
{
    enum EStage { Init, DefaultView, StartRequest, AwaitRide, AwaitMotion, OverviewView, OverviewCaptured, StationView, StationCaptured, StationQueueView, StationQueueCaptured, StationExitView, StationExitCaptured, PauseProbe, PauseHold, PoseProbe, Warmup, Traverse, ReviewView, ReviewCaptured, EndView, AwaitSave, AwaitReload, AwaitSaveCancel, AwaitGenerationPhase, CancelGeneration, AwaitGenerationCancel, AwaitMeshPhase, AwaitMeshCancel, AwaitScenePhase, AwaitSceneCancel, Finish, Done } Stage = Init;
    FString Output, Profile, SavePath, Error, Identity, SaveHash, PendingShot, FrameRows = TEXT("wall_seconds,ride_seconds,distance_m,speed_ms,wall_frame_ms,engine_delta_ms\n");
    FString Seed = TEXT("42"), Terrain = TEXT("highlands");
    uint64 CommittedRevision = 0;
    int32 PoseProbeIndex = 0;
    int32 Seat = 0, ShotIndex = 0, NextShot = 0, ObservedWidth = 0, ObservedHeight = 0;
    bool LoadOnly = false, Screenshots = true, ReviewCameras = false, RefusalChecked = false, Traversed = false, SaveChecked = false, LoadChecked = false, PauseChecked = false, RestartChecked = false;
    double Started = FPlatformTime::Seconds(), StageStarted = Started, TraversalStarted = 0, PreviousTick = 0, PauseTime = 0, LastRideTime = 0, LastDistance = 0, ShotRequested = 0, Duration = 0, FinalDistance = 0;
    TArray<double> ShotTimes;
    struct FReviewShot
    {
        FString Label, Source, Style;
        double Distance = 0, Time = 0, Begin = 0, End = 0;
        int32 Side = 1; // 0 chooses the lower-terrain side at the target.
    };
    TArray<FReviewShot> ReviewShots;
    int32 NextReview = 0, ReviewCaptures = 0, ReviewExpected = 0;
    double ReviewPauseStarted = 0, ReviewPauseSeconds = 0;
    TWeakObjectPtr<AActor> ReviewTarget;
    FTransform ReviewRiderPose;
    TWeakObjectPtr<AActor> StationReviewTarget;
    FTransform RiderCameraPose;
    FVector QueueReviewEye, QueueReviewTarget, ExitReviewEye, ExitReviewTarget;
    TSharedFuture<FString> CsvFinished;
    bool CsvStarted = false, PoseChecked = false, SaveCancelChecked = false, Benchmark = false;
    bool GenerationCancelChecked = false, MeshCancelChecked = false, SceneCancelChecked = false;
    uint64 CancelRevision = 0;
    double CancelRideTime = 0, GenerationCancelSeconds = 0, MeshCancelSeconds = 0, SceneCancelSeconds = 0;
    int32 MotionFrames = 0;

    bool Write(const FString& Name, const FString& Text, bool Append = false)
    {
        return FFileHelper::SaveStringToFile(Text, *(Output / Name), FFileHelper::EEncodingOptions::ForceUTF8WithoutBOM, &IFileManager::Get(), Append ? FILEWRITE_Append : 0);
    }
    void Event(const FString& Name, const FString& Fields = FString())
    {
        const FString Line = TEXT("{\"event\":") + Q(Name) + TEXT(",\"wall_seconds\":") + N(FPlatformTime::Seconds() - Started) + Fields + TEXT("}\n");
        if (!Write(TEXT("events.jsonl"), Line, true)) Error = TEXT("Could not write events.jsonl");
        UE_LOG(LogTemp, Display, TEXT("CoasterVerify: %s"), *Line.TrimEnd());
    }
    void Advance(EStage Next) { Stage = Next; StageStarted = FPlatformTime::Seconds(); }
    FString CancellationFields() const
    {
        return TEXT(",\"generation_cancel_checked\":") + FString(GenerationCancelChecked ? TEXT("true") : TEXT("false"))
            + TEXT(",\"mesh_cancel_checked\":") + (MeshCancelChecked ? TEXT("true") : TEXT("false"))
            + TEXT(",\"scene_cancel_checked\":") + (SceneCancelChecked ? TEXT("true") : TEXT("false"))
            + TEXT(",\"generation_cancel_seconds\":") + N(GenerationCancelSeconds)
            + TEXT(",\"mesh_cancel_seconds\":") + N(MeshCancelSeconds)
            + TEXT(",\"scene_cancel_seconds\":") + N(SceneCancelSeconds);
    }
    void Fail(const FString& Message) { Error = Message; Advance(Finish); }
    void Capture(AVibeCoasterController& PC, const FString& Label)
    {
        if (!Screenshots || !PendingShot.IsEmpty()) return;
        PendingShot = Output / FString::Printf(TEXT("%02d-%s.png"), ShotIndex++, *Label);
        ShotRequested = FPlatformTime::Seconds();
        FScreenshotRequest::RequestScreenshot(PendingShot, false, false, false);
        Event(TEXT("screenshot-request"), TEXT(",\"file\":") + Q(PendingShot) + TEXT(",\"ride_time\":") + N(PC.Ride->Playback().Time));
    }
    bool PollCapture(AVibeCoasterController& PC)
    {
        if (PendingShot.IsEmpty()) return true;
        if (FScreenshotRequest::IsScreenshotRequested() || IFileManager::Get().FileSize(*PendingShot) <= 0)
        {
            if (FPlatformTime::Seconds() - ShotRequested > 30) Fail(TEXT("Screenshot request did not produce a file: ") + PendingShot);
            return false;
        }
        TArray<uint8> Bytes;
        if (!FFileHelper::LoadFileToArray(Bytes, *PendingShot) || Bytes.Num() < 24 || Bytes[0] != 137 || Bytes[1] != 'P' || Bytes[2] != 'N' || Bytes[3] != 'G')
        { Fail(TEXT("Screenshot is not a readable PNG")); return false; }
        auto U32 = [&](int32 I) { return uint32(Bytes[I]) << 24 | uint32(Bytes[I + 1]) << 16 | uint32(Bytes[I + 2]) << 8 | uint32(Bytes[I + 3]); };
        const uint32 Width = U32(16), Height = U32(20);
        if (Width != 2560 || Height != 1440) { Fail(TEXT("Screenshot dimensions differ from 2560x1440")); return false; }
        FVector Position; FRotator Rotation; PC.GetPlayerViewPoint(Position, Rotation);
        Event(TEXT("screenshot-written"), TEXT(",\"file\":") + Q(PendingShot) + TEXT(",\"sha1\":") + Q(FSHA1::HashBuffer(Bytes.GetData(), Bytes.Num()).ToString()) + TEXT(",\"width\":2560,\"height\":1440,\"observed_ride_time\":") + N(PC.Ride->Playback().Time) + TEXT(",\"camera_position_cm\":") + Q(Position.ToString()) + TEXT(",\"camera_rotation_deg\":") + Q(Rotation.ToString()));
        PendingShot.Empty(); return true;
    }
};

FCoasterRuntimeVerification::FCoasterRuntimeVerification() : State(MakeUnique<FState>()) {}
FCoasterRuntimeVerification::~FCoasterRuntimeVerification() = default;
TUniquePtr<FCoasterRuntimeVerification> FCoasterRuntimeVerification::Create()
{
    FString Out;
    if (!FParse::Value(FCommandLine::Get(), TEXT("CoasterVerify="), Out)) return {};
    auto Result = MakeUnique<FCoasterRuntimeVerification>(); Result->State->Output = Out; return Result;
}

void FCoasterRuntimeVerification::Tick(AVibeCoasterController& PC, float DeltaSeconds)
{
    FState& S = *State; if (S.Stage == FState::Done) return;
    const double Now = FPlatformTime::Seconds();
    auto FatalBeforeOutput = [&](const FString& Message)
    {
        UE_LOG(LogTemp, Error, TEXT("CoasterVerify refused: %s"), *Message); S.Stage = FState::Done; FPlatformMisc::RequestExitWithStatus(false, 1);
    };
    if (S.Stage == FState::Init)
    {
        FString Mode;
        if (!FParse::Value(FCommandLine::Get(), TEXT("CoasterVerifyMode="), Mode) || Mode != TEXT("PhysicsProof")) { FatalBeforeOutput(TEXT("Explicit -CoasterVerifyMode=PhysicsProof is mandatory")); return; }
        if (FPaths::IsRelative(S.Output) || IFileManager::Get().DirectoryExists(*S.Output) || IFileManager::Get().FileExists(*S.Output)) { FatalBeforeOutput(TEXT("CoasterVerify must name a fresh absolute output directory")); return; }
        if (!FParse::Value(FCommandLine::Get(), TEXT("UserDir="), S.Profile) || FPaths::IsRelative(S.Profile)) { FatalBeforeOutput(TEXT("An explicit absolute isolated -UserDir is mandatory")); return; }
        S.Output = FPaths::ConvertRelativePathToFull(S.Output); S.Profile = FPaths::ConvertRelativePathToFull(S.Profile);
        S.SavePath = FPaths::ConvertRelativePathToFull(FPaths::ProjectSavedDir() / TEXT("VibeCoaster2/Designs/Accepted.vcdesign"));
        if (!FPaths::IsUnderDirectory(S.SavePath, S.Profile)) { FatalBeforeOutput(TEXT("Resolved save is outside the selected profile")); return; }
        S.LoadOnly = FParse::Param(FCommandLine::Get(), TEXT("CoasterVerifyLoad"));
        const FString Marker = S.Profile / TEXT("coaster-verification-profile.txt");
        FString MarkerText;
        if (S.LoadOnly)
        {
            if (!FFileHelper::LoadFileToString(MarkerText, *Marker) || MarkerText != TEXT("VibeCoaster isolated runtime verification profile v1\n") || !IFileManager::Get().FileExists(*S.SavePath))
            { FatalBeforeOutput(TEXT("Load-only mode requires a previously marked verification profile and its save")); return; }
        }
        else if (IFileManager::Get().FileExists(*S.SavePath)) { FatalBeforeOutput(TEXT("Generation verification refuses to overwrite an existing accepted save; choose a fresh UserDir")); return; }
        if (!IFileManager::Get().MakeDirectory(*S.Output, true)) { FatalBeforeOutput(TEXT("Could not create output directory")); return; }
        if (!S.LoadOnly && !FFileHelper::SaveStringToFile(TEXT("VibeCoaster isolated runtime verification profile v1\n"), *Marker, FFileHelper::EEncodingOptions::ForceUTF8WithoutBOM)) { S.Fail(TEXT("Could not mark isolated profile")); return; }
        if (FParse::Param(FCommandLine::Get(), TEXT("NullRHI")) || FApp::UseFixedTimeStep()) { S.Fail(TEXT("Real rendering and normal wall-time playback are required")); return; }
        S.Benchmark = FParse::Param(FCommandLine::Get(), TEXT("CoasterVerifyBenchmark"));
        S.Screenshots = !S.Benchmark && !FParse::Param(FCommandLine::Get(), TEXT("CoasterVerifyNoScreenshots"));
        S.ReviewCameras = FParse::Param(FCommandLine::Get(), TEXT("CoasterVerifyReviewCameras"));
        if (S.ReviewCameras && !S.Screenshots)
        { S.Fail(TEXT("Review cameras require real screenshots and full traversal")); return; }
        FParse::Value(FCommandLine::Get(), TEXT("CoasterVerifySeed="), S.Seed);
        FParse::Value(FCommandLine::Get(), TEXT("CoasterVerifyTerrain="), S.Terrain);
        FParse::Value(FCommandLine::Get(), TEXT("CoasterVerifySeat="), S.Seat);
        if (S.Seat < 0 || S.Seat > 2 || (S.Terrain != TEXT("flat") && S.Terrain != TEXT("highlands"))) { S.Fail(TEXT("Terrain must be flat/highlands; seat must be 0 front, 1 middle or 2 rear")); return; }
        S.Write(TEXT("environment.json"), TEXT("{\"utc\":") + Q(FDateTime::UtcNow().ToIso8601()) + TEXT(",\"engine\":") + Q(FEngineVersion::Current().ToString()) + TEXT(",\"cpu\":") + Q(FPlatformMisc::GetCPUBrand()) + TEXT(",\"os\":") + Q(FPlatformMisc::GetOSVersion()) + TEXT(",\"executable\":") + Q(FPlatformProcess::ExecutablePath()) + TEXT(",\"profile\":") + Q(S.Profile) + TEXT(",\"save\":") + Q(S.SavePath) + TEXT(",\"screenshots_enabled\":") + (S.Screenshots ? TEXT("true") : TEXT("false")) + TEXT("}\n"));
        S.Event(TEXT("begin"), TEXT(",\"app_version\":") + Q(VibeCoasterAppVersion) + TEXT(",\"build_commit\":") + Q(UTF8_TO_TCHAR(COASTER_BUILD_COMMIT)) + TEXT(",\"geometry_version\":") + Q(UTF8_TO_TCHAR(coaster::generatorVersion))); S.Advance(FState::DefaultView); return;
    }
    if (!PC.Ride) { if (Now - S.Started > 30) S.Fail(TEXT("Runtime world did not become available")); else return; }
    if (S.Stage != FState::Finish && !S.Error.IsEmpty()) S.Advance(FState::Finish);
    if (S.Stage != FState::Finish && Now - S.Started > 1200) S.Fail(TEXT("Verification exceeded 1200 seconds"));
    if (S.Stage == FState::Finish)
    {
        if (S.Error.IsEmpty() && !S.PollCapture(PC)) return;
        if (S.StationReviewTarget.IsValid())
        {
            S.StationReviewTarget->SetActorTransform(S.RiderCameraPose);
            S.StationReviewTarget.Reset();
        }
        if (S.ReviewTarget.IsValid())
        {
            S.ReviewTarget->SetActorTransform(S.ReviewRiderPose);
            S.ReviewTarget.Reset();
            if (PC.Ride->IsPaused()) PC.Ride->TogglePause();
        }
#if CSV_PROFILER
        if (S.CsvStarted) { S.CsvFinished = FCsvProfiler::Get()->EndCapture(); S.CsvStarted = false; }
        if (S.CsvFinished.IsValid() && !S.CsvFinished.IsReady()) { if (Now - S.StageStarted < 30) return; S.Error += TEXT(" CSV flush timeout."); }
        if (S.CsvFinished.IsValid() && S.CsvFinished.IsReady())
        {
            const FString CsvPath = S.CsvFinished.Get();
            if (CsvPath.IsEmpty() || IFileManager::Get().FileSize(*CsvPath) <= 0) S.Error += TEXT(" CSV capture did not produce a readable file.");
            else S.Event(TEXT("csv-written"), TEXT(",\"file\":") + Q(CsvPath));
        }
#endif
        const bool Passed = S.Error.IsEmpty() && (!S.ReviewCameras ||
            (S.ReviewShots.Num() == S.ReviewExpected &&
             S.ReviewCaptures == S.ReviewExpected)) &&
            S.Traversed && S.PauseChecked && S.RestartChecked && S.PoseChecked && S.SaveCancelChecked && S.GenerationCancelChecked && S.MeshCancelChecked && S.SceneCancelChecked && S.LoadChecked && (S.LoadOnly || S.SaveChecked);
        if (!S.Write(TEXT("wall-frames.csv"), S.FrameRows)) S.Error += TEXT(" Frame output write failed.");
        const FString Report = TEXT("{\"status\":") + Q(Passed && S.Error.IsEmpty() ? TEXT("automated-smoke-passed") : TEXT("failed")) + TEXT(",\"error\":") + Q(S.Error) + TEXT(",\"mode\":\"PHYSICS-PROOF; intensity untested\",\"geometry_sha1\":") + Q(S.Identity) + TEXT(",\"save_sha1\":") + Q(S.SaveHash) + TEXT(",\"full_traversal\":") + (S.Traversed ? TEXT("true") : TEXT("false")) + TEXT(",\"ride_duration_s\":") + N(S.Duration) + TEXT(",\"final_distance_m\":") + N(S.FinalDistance) + TEXT(",\"seat\":") + FString::FromInt(S.Seat) + TEXT(",\"pause_checked\":") + (S.PauseChecked ? TEXT("true") : TEXT("false")) + TEXT(",\"restart_checked\":") + (S.RestartChecked ? TEXT("true") : TEXT("false")) + TEXT(",\"save_checked\":") + (S.SaveChecked ? TEXT("true") : TEXT("false")) + TEXT(",\"load_checked\":") + (S.LoadChecked ? TEXT("true") : TEXT("false")) + TEXT(",\"missing_reference_refusal_checked\":") + (S.RefusalChecked ? TEXT("true") : TEXT("false")) + TEXT(",\"viewport_width\":") + FString::FromInt(S.ObservedWidth) + TEXT(",\"viewport_height\":") + FString::FromInt(S.ObservedHeight) + TEXT(",\"paused_pose_checked\":") + (S.PoseChecked ? TEXT("true") : TEXT("false")) + TEXT(",\"save_cancel_checked\":") + (S.SaveCancelChecked ? TEXT("true") : TEXT("false")) + S.CancellationFields() + TEXT(",\"review_camera_expected\":") + FString::FromInt(S.ReviewExpected) + TEXT(",\"review_camera_captures\":") + FString::FromInt(S.ReviewCaptures) + TEXT(",\"review_camera_pause_seconds\":") + N(S.ReviewPauseSeconds) + TEXT(",\"human_visual_review\":\"pending\",\"keyboard_input\":\"untested\",\"cancellation_phases\":\"generation, mesh preparation, scene commit and save\",\"performance\":\"raw wall-frame samples; no FPS or GPU acceptance claim\"}\n");
        const bool Written = S.Write(TEXT("result.json"), Report);
        UE_LOG(LogTemp, Display, TEXT("CoasterVerify result: %s"), *Report); S.Stage = FState::Done;
        FPlatformMisc::RequestExitWithStatus(false, Passed && S.Error.IsEmpty() && Written ? 0 : 1); return;
    }
    int32 Width = 0, Height = 0; PC.GetViewportSize(Width, Height); S.ObservedWidth = Width; S.ObservedHeight = Height;
    if (Width != 2560 || Height != 1440) { if (S.Stage == FState::DefaultView && Now - S.StageStarted < 10) return; S.Fail(FString::Printf(TEXT("Actual viewport is %dx%d; 2560x1440 required"), Width, Height)); return; }
    const auto P = PC.Ride->Playback();
    if (S.Stage == FState::Traverse)
    {
        const double WallFrame = S.PreviousTick > 0 ? (Now - S.PreviousTick) * 1000 : 0;
        S.FrameRows += N(Now - S.TraversalStarted) + TEXT(",") + N(P.Time) + TEXT(",") + N(P.Distance) + TEXT(",") + N(P.Speed) + TEXT(",") + N(WallFrame) + TEXT(",") + N(DeltaSeconds * 1000.) + TEXT("\n");
        S.PreviousTick = Now;
    }
    if (!S.PollCapture(PC)) return;
    switch (S.Stage)
    {
    case FState::DefaultView:
        if (S.LoadOnly && FParse::Param(FCommandLine::Get(), TEXT("CoasterLoad")))
        {
            S.Event(TEXT("missing-reference-refusal-skipped"), TEXT(",\"reason\":\"shortcut startup load is already active\""));
            S.Advance(FState::StartRequest); break;
        }
        PC.SeedText = S.Seed;
        PC.Settings.targets.requireIntensity = true; // Exercise the unavailable-reference gate independently of the startup preset.
        if (PC.Settings.targets.requireIntensity && !std::isfinite(PC.Settings.targets.referenceExposure))
        {
            PC.RequestGeneration();
            if (PC.InputError.IsEmpty() || PC.Ride->IsBusy() || PC.Ride->HasRide()) { S.Fail(TEXT("Default missing-reference request was not refused")); break; }
            S.RefusalChecked = true; S.Event(TEXT("missing-reference-refused"), TEXT(",\"message\":") + Q(PC.InputError)); S.Capture(PC, TEXT("missing-reference"));
        }
        else S.Event(TEXT("missing-reference-refusal-skipped"), TEXT(",\"reason\":\"reference configured or initial mode differs\""));
        S.Advance(FState::StartRequest); break;
    case FState::StartRequest:
        PC.Settings.targets.requireIntensity = false;
        PC.Settings.terrain = coaster::Terrain{};
        PC.Settings.terrain.kind = S.Terrain == TEXT("highlands") ? coaster::TerrainKind::Highlands : coaster::TerrainKind::Flat;
        if (S.LoadOnly)
        {
            S.SaveHash = HashFile(S.SavePath);
            const bool StartupLoad = FParse::Param(FCommandLine::Get(), TEXT("CoasterLoad"));
            if (!StartupLoad) PC.Ride->Load();
            S.Event(StartupLoad ? TEXT("shortcut-startup-load-requested") : TEXT("cross-process-load-requested"));
        }
        else { PC.RequestGeneration(); S.Event(TEXT("generation-requested"), TEXT(",\"seed\":") + Q(S.Seed) + TEXT(",\"terrain\":") + Q(S.Terrain)); }
        if (!S.LoadOnly && !PC.InputError.IsEmpty()) { S.Fail(PC.InputError); break; }
        S.Advance(FState::AwaitRide); break;
    case FState::AwaitRide:
        if (PC.Ride->IsBusy()) { if (Now - S.StageStarted > 240) S.Fail(TEXT("Generation/load/commit timeout: ") + PC.Ride->Status()); break; }
        if (!PC.Ride->HasRide() || !PC.Ride->ActiveDesign()->accepted()) { S.Fail(TEXT("No accepted committed ride: ") + PC.Ride->Status()); break; }
        {
            const auto& D = *PC.Ride->ActiveDesign();
            if (D.request.targets.requireIntensity || D.simulation.frames.empty()) { S.Fail(TEXT("Expected an accepted physics-proof trace")); break; }
            if (!S.Write(TEXT("accepted-report.json"), FString(UTF8_TO_TCHAR(coaster::reportJson(D).c_str())))) { S.Fail(TEXT("Could not retain accepted numerical report")); break; }
            S.CommittedRevision = PC.Ride->GeometryRevision();
            S.Identity = GeometryIdentity(D); S.Duration = D.simulation.frames.back().time; S.FinalDistance = D.simulation.frames.back().distance;
            if (S.LoadOnly) S.LoadChecked = true;
            S.Event(TEXT("accepted-commit"), TEXT(",\"geometry_sha1\":") + Q(S.Identity) + TEXT(",\"seed\":") + Q(FString::Printf(TEXT("%llu"), static_cast<unsigned long long>(D.request.seed))) + TEXT(",\"terrain\":") + Q(FString(UTF8_TO_TCHAR(D.request.terrain.name().c_str()))) + TEXT(",\"duration_s\":") + N(S.Duration) + TEXT(",\"track_length_m\":") + N(D.track.length) + TEXT(",\"convergence_performed\":true,\"convergence_passed\":true"));
            if (S.Benchmark)
            {
                PC.Menu = false; PC.Ride->SetSeat(S.Seat);
                PC.Ride->Restart();
                if (PC.Ride->IsPaused()) PC.Ride->TogglePause();
                S.Advance(FState::AwaitMotion); break;
            }
            double ApexTime = 0, ApexHeight = -1e30, HighestTime = 0, HighestGround = -1e30;
            double LowPassTime = 0, LowPassHeight = 1e30;
            TArray<double> InversionPassages;
            TArray<double> AirtimeValleys;
            bool WasInverted = false;
            bool WasAirtime = false;
            double PreviousPitch = 0;
            for (const auto& F : D.simulation.frames)
            {
                const auto K = D.track.sample(F.distance + coaster::seatDistanceOffset(D.request.train, S.Seat));
                const double H = K.position.z - D.request.terrain.height(K.position.x, K.position.y);
                if (H > HighestGround) { HighestGround = H; HighestTime = F.time; }
                if (F.speed > 35 && F.time > 10 && (K.element == coaster::Element::Return || K.element == coaster::Element::Turn) && H < LowPassHeight)
                { LowPassHeight = H; LowPassTime = F.time; }
                if (K.element == coaster::Element::Inversion && K.up.z < -.5 && H > ApexHeight) { ApexHeight = H; ApexTime = F.time; }
                const bool Inverted = K.up.z < -.5;
                if (Inverted && !WasInverted) InversionPassages.Add(F.time);
                WasInverted = Inverted;
                const bool Airtime = K.element == coaster::Element::Airtime;
                if (Airtime && WasAirtime && F.distance > D.track.length * .6 && PreviousPitch < 0 && K.tangent.z >= 0)
                    AirtimeValleys.Add(F.time);
                WasAirtime = Airtime; PreviousPitch = K.tangent.z;
            }
            if (ApexHeight == -1e30) { S.Fail(TEXT("Accepted trace lacks an observable inverted-apex passage")); break; }
            S.ShotTimes = { HighestTime, S.Duration * .15, S.Duration * .35, S.Duration * .55, S.Duration * .75, FMath::Max(0., ApexTime - .4), ApexTime };
            S.Event(TEXT("pov-landmarks"), TEXT(",\"highest_ground_time_s\":") + N(HighestTime) + TEXT(",\"highest_ground_m\":") + N(HighestGround) + TEXT(",\"inverted_apex_time_s\":") + N(ApexTime) + TEXT(",\"inverted_apex_ground_m\":") + N(ApexHeight));
            // Inspect each actual inverted passage during full playback.
            for (double Time : InversionPassages) { S.ShotTimes.Add(FMath::Max(0., Time - 1)); S.ShotTimes.Add(Time + .5); }
            // Inspect both sides of the late hill joins, where a continuous
            // curve can still have a visible pitch hesitation.
            for (double Time : AirtimeValleys)
            {
                for (double Offset : {-1., 0., 1.}) S.ShotTimes.Add(FMath::Clamp(Time + Offset, 0., S.Duration));
                S.Event(TEXT("airtime-valley-landmark"), TEXT(",\"time_s\":") + N(Time));
            }
            if (LowPassHeight < 1e30)
            {
                for (double Offset : {-1., 0., 1.}) S.ShotTimes.Add(FMath::Clamp(LowPassTime + Offset, 0., S.Duration));
                S.Event(TEXT("low-pass-landmark"), TEXT(",\"time_s\":") + N(LowPassTime) + TEXT(",\"centerline_ground_m\":") + N(LowPassHeight));
            }
            // Capture the actual new terrain chapters and visible physical hardware.
            for (const auto& Section : D.sections)
                if (Section.role == coaster::RideRole::Opening || Section.role == coaster::RideRole::CliffApproach || Section.role == coaster::RideRole::CliffDrop || Section.role == coaster::RideRole::Wave || Section.role == coaster::RideRole::Signature)
                {
                    const double Distance = (Section.start + Section.end) * .5;
                    const auto Frame = std::lower_bound(D.simulation.frames.begin(), D.simulation.frames.end(), Distance,
                        [](const coaster::Frame& F, double S) { return F.distance < S; });
                    if (Frame != D.simulation.frames.end()) S.ShotTimes.Add(Frame->time);
                }
            for (const auto& Operation : D.operations)
                if (Operation.kind == coaster::DriveKind::Boost || Operation.kind == coaster::DriveKind::Trim)
                {
                    const auto Frame = std::lower_bound(D.simulation.frames.begin(), D.simulation.frames.end(), (Operation.start + Operation.end) * .5,
                        [](const coaster::Frame& F, double S) { return F.distance < S; });
                    if (Frame != D.simulation.frames.end()) S.ShotTimes.Add(Frame->time);
                }
            for (const auto& Landmark : D.landmarks)
            {
                const double Distance = Landmark.distance - coaster::seatDistanceOffset(D.request.train, S.Seat);
                const auto Frame = std::lower_bound(D.simulation.frames.begin(), D.simulation.frames.end(), Distance,
                    [](const coaster::Frame& F, double At) { return F.distance < At; });
                if (Frame != D.simulation.frames.end())
                {
                    S.ShotTimes.Add(Frame->time);
                    S.Event(TEXT("authored-landmark"), TEXT(",\"name\":") + Q(UTF8_TO_TCHAR(coaster::landmarkName(Landmark.kind))) + TEXT(",\"time_s\":") + N(Frame->time) + TEXT(",\"distance_m\":") + N(Landmark.distance));
                }
            }
            S.ShotTimes.Sort();
            for (int32 I = S.ShotTimes.Num() - 1; I > 0; --I) if (S.ShotTimes[I] - S.ShotTimes[I - 1] < .25) S.ShotTimes.RemoveAt(I);
            if (S.ReviewCameras)
            {
                const auto RoleSpan = [&](coaster::RideRole Role, double& Begin, double& End)
                {
                    Begin = D.track.length; End = 0;
                    for (const auto& Section : D.sections) if (Section.role == Role)
                    { Begin = std::min(Begin, Section.start); End = std::max(End, Section.end); }
                    return End > Begin;
                };
                const double FrontOffset = coaster::seatDistanceOffset(D.request.train, 0);
                const auto AddReview = [&](const TCHAR* Label, const TCHAR* Source, const TCHAR* Style,
                    double Distance, double Begin, double End, int32 Side)
                {
                    Distance = FMath::Clamp(Distance, 0., D.track.length);
                    Begin = FMath::Clamp(Begin, 0., D.track.length);
                    End = FMath::Clamp(End, Begin, D.track.length);
                    if (End - Begin < 1.) return false;
                    const double CentreDistance = FMath::Max(0., Distance - FrontOffset);
                    const auto Frame = std::lower_bound(D.simulation.frames.begin(), D.simulation.frames.end(),
                        CentreDistance, [](const coaster::Frame& F, double At) { return F.distance < At; });
                    if (Frame == D.simulation.frames.end() || Frame->time >= S.Duration) return false;
                    S.ReviewShots.Add({Label, Source, Style, Distance, Frame->time, Begin, End, Side});
                    return true;
                };
                double CliffBegin = 0, CliffEnd = 0, LoopBegin = 0, LoopEnd = 0;
                double ImmelBegin = 0, ImmelEnd = 0, SignatureBegin = 0, SignatureEnd = 0;
                double ReturnBegin = 0, ReturnEnd = 0;
                if (!RoleSpan(coaster::RideRole::CliffApproach, CliffBegin, CliffEnd) ||
                    !RoleSpan(coaster::RideRole::Loop, LoopBegin, LoopEnd) ||
                    !RoleSpan(coaster::RideRole::Immelmann, ImmelBegin, ImmelEnd) ||
                    !RoleSpan(coaster::RideRole::Signature, SignatureBegin, SignatureEnd) ||
                    !RoleSpan(coaster::RideRole::Return, ReturnBegin, ReturnEnd) ||
                    !(LoopEnd <= ImmelBegin && ImmelEnd <= SignatureBegin && SignatureEnd <= ReturnEnd))
                { S.Fail(TEXT("Review cameras need ordered clifftop, inversion, signature and return roles")); break; }
                TArray<const coaster::Operation*> Boosts;
                for (const auto& Operation : D.operations)
                    if (Operation.kind == coaster::DriveKind::Boost) Boosts.Add(&Operation);
                Boosts.Sort([](const coaster::Operation& A, const coaster::Operation& B) { return A.start < B.start; });
                if (Boosts.Num() < 2)
                { S.Fail(TEXT("Review cameras need both physical booster phases")); break; }
                S.ReviewExpected = 6 + Boosts.Num();
                double PlateauArrival = -1, CliffDeparture = -1;
                for (const auto& Landmark : D.landmarks)
                {
                    if (Landmark.kind == coaster::LandmarkKind::PlateauArrival)
                        PlateauArrival = Landmark.distance;
                    if (Landmark.kind == coaster::LandmarkKind::CliffDeparture)
                        CliffDeparture = Landmark.distance;
                }
                if (!(PlateauArrival >= CliffBegin && CliffDeparture > PlateauArrival &&
                    CliffDeparture <= D.track.length))
                { S.Fail(TEXT("Review cameras need the plateau arrival and cliff departure landmarks")); break; }
                CliffBegin = PlateauArrival; CliffEnd = CliffDeparture;
                const double CliffLength = CliffEnd - CliffBegin;
                double PeakHorizontalBend = 0;
                for (int32 I = 0; I <= 128; ++I)
                {
                    const auto K = D.track.sample(FMath::Lerp(CliffBegin, CliffEnd, double(I) / 128.));
                    PeakHorizontalBend = std::max(PeakHorizontalBend,
                        std::hypot(K.curvature.x, K.curvature.y));
                }
                if (PeakHorizontalBend <= 1e-9)
                { S.Fail(TEXT("Review cameras found no horizontal curvature on the plateau")); break; }
                double OutwardAt = -1, OutwardScore = 0, OutwardBend = 0;
                for (int32 I = 0; I <= 128; ++I)
                {
                    const double At = FMath::Lerp(CliffBegin, CliffEnd, double(I) / 128.);
                    const auto K = D.track.sample(At);
                    const coaster::Vec3 HorizontalBend{K.curvature.x, K.curvature.y, 0};
                    const coaster::Vec3 HorizontalUp{K.up.x, K.up.y, 0};
                    const double Bend = coaster::norm(HorizontalBend);
                    if (Bend < PeakHorizontalBend * .25 ||
                        std::abs(K.tangent.z) > std::sin(15. * coaster::pi / 180.) ||
                        coaster::norm(HorizontalUp) < std::sin(10. * coaster::pi / 180.))
                        continue;
                    // A bank tilted away from horizontal curvature is the
                    // outward move, irrespective of the section's display name.
                    const double Outward = -coaster::dot(HorizontalBend / Bend, HorizontalUp);
                    const double Score = Outward * Bend;
                    if (Outward > .15 && Score > OutwardScore)
                    { OutwardAt = At; OutwardScore = Score; OutwardBend = Bend; }
                }
                if (OutwardAt < 0)
                { S.Fail(TEXT("Review cameras found no slow, curved outward bank on the plateau")); break; }
                bool Planned = true;
                Planned &= AddReview(TEXT("review-cliff-winding"), TEXT("clifftop-shelf"), TEXT("side"),
                    CliffBegin + .16 * CliffLength, CliffBegin,
                    CliffBegin + .36 * CliffLength, -1);
                const double BankSpan = std::min(75., std::max(35., .5 / OutwardBend));
                Planned &= AddReview(TEXT("review-cliff-outward-bank"), TEXT("outward-bank-on-plateau"), TEXT("near-rail"),
                    OutwardAt, OutwardAt - BankSpan, OutwardAt + BankSpan, 0);
                for (int32 I = 0; I < Boosts.Num(); ++I)
                {
                    const auto& Boost = *Boosts[I];
                    const double Downstream = std::max(120., Boost.targetSpeed * 2.);
                    const FString Label = FString::Printf(TEXT("review-boost-%d-exit"), I + 1);
                    const FString Source = FString::Printf(TEXT("boost-operation-%d"), I + 1);
                    Planned &= AddReview(*Label, *Source, TEXT("side"),
                        Boost.end + .12 * Downstream,
                        Boost.end - std::min(45., (Boost.end - Boost.start) * .5),
                        Boost.end + Downstream, 0);
                }
                const auto& LastBoost = *Boosts.Last();
                const double PullBegin = LastBoost.end + 10.;
                const double PullEnd = FMath::Min(D.track.length, LastBoost.end +
                    std::max(180., LastBoost.targetSpeed * 3.));
                double Pullout = (PullBegin + PullEnd) * .5, Strongest = -1e30;
                for (int32 I = 0; I <= 48; ++I)
                {
                    const double At = FMath::Lerp(PullBegin, PullEnd, double(I) / 48.);
                    const auto K = D.track.sample(At);
                    const double UpwardBend = coaster::dot(K.curvature, K.up);
                    if (UpwardBend > Strongest) { Strongest = UpwardBend; Pullout = At; }
                }
                const double PullSpan = std::max(40., LastBoost.targetSpeed * .7);
                Planned &= AddReview(TEXT("review-boost-pullout"), TEXT("downstream-upward-bend"), TEXT("near-rail"),
                    Pullout, Pullout - PullSpan, Pullout + PullSpan, 0);
                Planned &= AddReview(TEXT("review-loop-immel-spacing"), TEXT("loop-to-immelmann"), TEXT("overview"),
                    (LoopEnd + ImmelBegin) * .5, LoopBegin, ImmelEnd, 1);
                const double SignatureLength = SignatureEnd - SignatureBegin;
                Planned &= AddReview(TEXT("review-post-inversion-closure"), TEXT("signature-after-inversions"), TEXT("side"),
                    SignatureBegin + .33 * SignatureLength, SignatureBegin + .08 * SignatureLength,
                    SignatureBegin + .60 * SignatureLength, 1);
                const double ReturnLength = ReturnEnd - ReturnBegin;
                Planned &= AddReview(TEXT("review-final-return"), TEXT("return-before-brakes"), TEXT("near-rail"),
                    ReturnBegin + .64 * ReturnLength, ReturnBegin + .42 * ReturnLength,
                    ReturnBegin + .86 * ReturnLength, -1);
                if (!Planned || S.ReviewShots.Num() != S.ReviewExpected)
                { S.Fail(TEXT("Review camera targets fall outside the accepted replay")); break; }
                S.ReviewShots.Sort([](const FState::FReviewShot& A, const FState::FReviewShot& B)
                    { return A.Time < B.Time; });
                FString Labels = TEXT("[");
                for (int32 I = 0; I < S.ReviewShots.Num(); ++I)
                {
                    if (I) Labels += TEXT(",");
                    Labels += Q(S.ReviewShots[I].Label);
                }
                Labels += TEXT("]");
                S.Event(TEXT("review-camera-plan"),
                    TEXT(",\"captures\":") + FString::FromInt(S.ReviewExpected) +
                    TEXT(",\"labels\":") + Labels +
                    TEXT(",\"front_offset_m\":") + N(FrontOffset));
            }
            PC.Menu = false; PC.ShowComparison = false; PC.ShowTelemetry = true; PC.Ride->SetSeat(S.Seat);
            if (!PC.Ride->IsPaused()) PC.Ride->TogglePause(); PC.Ride->Restart();
            if (!PC.Ride->IsOverview()) PC.Ride->ToggleOverview();
            S.Advance(FState::OverviewView);
        }
        break;
    case FState::AwaitMotion:
        if (PC.Ride->IsPaused() || !PC.Ride->ActiveDesign()->accepted()) { S.Fail(TEXT("Benchmark lost its accepted playing ride")); break; }
        if (P.Speed <= 0 || P.Time <= 0 || P.Distance <= PC.Ride->ActiveDesign()->simulation.frames.front().distance)
        { if (Now - S.StageStarted > 10) S.Fail(TEXT("Accepted ride did not start moving")); break; }
        // Observe motion across consecutive game ticks. Full traversal,
        // screenshots and save/reload remain separate smoke-test requirements.
        if (++S.MotionFrames < 3) break;
        S.Event(TEXT("first-motion"), TEXT(",\"ride_time\":") + N(P.Time) + TEXT(",\"distance_m\":") + N(P.Distance) + TEXT(",\"speed_ms\":") + N(P.Speed));
        if (!S.Write(TEXT("result.json"), TEXT("{\"status\":\"launch-benchmark-passed\",\"full_traversal\":false,\"verification_seconds\":") + N(Now - S.Started) + TEXT("}\n")))
        { S.Fail(TEXT("Could not retain launch benchmark result")); break; }
        S.Stage = FState::Done; FPlatformMisc::RequestExitWithStatus(false, 0); break;
    case FState::OverviewView:
        if (Now - S.StageStarted < 1) break;
        for (const auto& Knot : PC.Ride->ActiveDesign()->track.knots)
        {
            FVector2D Screen;
            const auto Position = VibeCoordinates::Position(Knot.position);
            if (!PC.ProjectWorldLocationToScreen(FVector(Position.X, Position.Y, Position.Z), Screen) ||
                Screen.X < 0 || Screen.X > Width || Screen.Y < 0 || Screen.Y > Height)
            { S.Fail(TEXT("Overview clips the accepted circuit")); break; }
        }
        if (S.Stage == FState::Finish) break;
        S.Event(TEXT("overview-circuit-framing-passed"));
        S.Capture(PC, TEXT("overview")); S.Advance(FState::OverviewCaptured); break;
    case FState::OverviewCaptured:
        PC.Ride->ToggleOverview(); S.Advance(FState::StationView); break;
    case FState::StationView:
        if (Now - S.StageStarted < 1) break;
        S.Capture(PC, TEXT("station")); S.Advance(FState::StationCaptured); break;
    case FState::StationCaptured:
    {
        const auto& D = *PC.Ride->ActiveDesign();
        const auto Start = D.track.sample(0);
        const auto Origin = Start.position, Forward = Start.tangent, Right = Start.right, Up = Start.up;
        double RowX = 0, LeftEdge = 0, RightEdge = 0;
        int32 Rows = 0;
        for (const auto& Box : D.station.boxes)
        {
            const double Y = coaster::dot(Box.center - Origin, Right);
            LeftEdge = FMath::Min(LeftEdge, Y - Box.half.y);
            RightEdge = FMath::Max(RightEdge, Y + Box.half.y);
            if (Box.role == coaster::StationRole::HoldingLane)
            { RowX += coaster::dot(Box.center - Origin, Forward); ++Rows; }
        }
        if (Rows > 0 && Rows != D.request.train.cars)
        { S.Fail(TEXT("Station holding lanes do not match the actual train rows")); break; }
        if (Rows == 0 || !S.Screenshots)
        { PC.Ride->TogglePause(); S.Advance(FState::PauseProbe); break; }
        RowX /= Rows;
        const auto Position = [&](double X, double Y, double Z)
        {
            const auto P = VibeCoordinates::Position(Origin + Forward * X + Right * Y + Up * Z);
            return FVector(P.X, P.Y, P.Z);
        };
        S.QueueReviewEye = Position(RowX - 3, RightEdge + 15, 7);
        S.QueueReviewTarget = Position(RowX, 7, -.5);
        S.ExitReviewEye = Position(RowX + 8, LeftEdge - 15, 7);
        S.ExitReviewTarget = Position(RowX, -5, -.5);
        S.StationReviewTarget = PC.GetViewTarget();
        if (!S.StationReviewTarget.IsValid()) { S.Fail(TEXT("Station review camera target is unavailable")); break; }
        S.RiderCameraPose = S.StationReviewTarget->GetActorTransform();
        S.StationReviewTarget->SetActorLocationAndRotation(S.QueueReviewEye,
            (S.QueueReviewTarget - S.QueueReviewEye).Rotation());
        S.Event(TEXT("station-queue-review-pose"), TEXT(",\"rows\":") + FString::FromInt(Rows));
        S.Advance(FState::StationQueueView); break;
    }
    case FState::StationQueueView:
        if (Now - S.StageStarted < .5) break;
        S.Capture(PC, TEXT("station-queue-operation")); S.Advance(FState::StationQueueCaptured); break;
    case FState::StationQueueCaptured:
        if (!S.StationReviewTarget.IsValid()) { S.Fail(TEXT("Station review camera was lost")); break; }
        S.StationReviewTarget->SetActorLocationAndRotation(S.ExitReviewEye,
            (S.ExitReviewTarget - S.ExitReviewEye).Rotation());
        S.Advance(FState::StationExitView); break;
    case FState::StationExitView:
        if (Now - S.StageStarted < .5) break;
        S.Capture(PC, TEXT("station-unload-exit")); S.Advance(FState::StationExitCaptured); break;
    case FState::StationExitCaptured:
        if (!S.StationReviewTarget.IsValid()) { S.Fail(TEXT("Station review camera was lost")); break; }
        S.StationReviewTarget->SetActorTransform(S.RiderCameraPose);
        S.StationReviewTarget.Reset();
        PC.Ride->TogglePause(); S.Advance(FState::PauseProbe); break;
    case FState::PauseProbe:
        if (P.Time < 2) break;
        PC.Ride->TogglePause(); S.PauseTime = P.Time; S.Advance(FState::PauseHold); break;
    case FState::PauseHold:
        if (Now - S.StageStarted < .5) break;
        if (!PC.Ride->IsPaused() || FMath::Abs(P.Time - S.PauseTime) > 1e-8) { S.Fail(TEXT("Pause did not hold actual playback time")); break; }
        S.PauseChecked = true; PC.Ride->Restart();
        if (FMath::Abs(PC.Ride->Playback().Time) > 1e-8) { S.Fail(TEXT("Restart did not return playback to time zero")); break; }
        S.RestartChecked = true; S.Event(TEXT("pause-and-restart-passed")); S.Advance(FState::PoseProbe); break;
    case FState::PoseProbe:
    {
        if (Now - S.StageStarted < .15) break;
        if (!PC.Ride->IsPaused() || P.Time != 0 || PC.Ride->GeometryRevision() != S.CommittedRevision)
        { S.Fail(TEXT("Paused pose probe altered playback or accepted geometry")); break; }
        if (S.PoseProbeIndex == 0)
        { PC.Ride->SetSeat(0); ++S.PoseProbeIndex; S.StageStarted = Now; break; }
        FVector ActualPosition; FRotator ActualRotation;
        PC.GetPlayerViewPoint(ActualPosition, ActualRotation);
        if (S.PoseProbeIndex == 4)
        {
            const auto C = VibeCoordinates::Position(PC.Ride->ActiveDesign()->track.sample(P.Distance).position);
            if (!PC.Ride->IsOverview() || FVector::Distance(ActualPosition, FVector(C.X,C.Y,C.Z)) < 10000.)
            { S.Fail(TEXT("Paused overview camera did not update")); break; }
            PC.Ride->ToggleOverview(); PC.Ride->SetSeat(S.Seat);
        }
        else
        {
            const auto& D = *PC.Ride->ActiveDesign();
            const int32 ExpectedSeat = S.PoseProbeIndex <= 3 ? S.PoseProbeIndex - 1 : S.Seat;
            const auto K = D.track.sample(P.Distance + coaster::seatDistanceOffset(D.request.train, ExpectedSeat));
            const auto C = VibeCoordinates::Position(K.position + K.up * D.request.train.seatHeight);
            const auto T = VibeCoordinates::Direction(K.tangent), U = VibeCoordinates::Direction(K.up);
            if (PC.Ride->IsOverview() || FVector::Distance(ActualPosition,FVector(C.X,C.Y,C.Z)) > .1 ||
                FVector::DotProduct(ActualRotation.Vector(),FVector(T.X,T.Y,T.Z)) < .999999 ||
                FVector::DotProduct(ActualRotation.RotateVector(FVector::UpVector),FVector(U.X,U.Y,U.Z)) < .999999)
            { S.Fail(TEXT("Paused seat camera differs from canonical rider pose")); break; }
            if (S.PoseProbeIndex < 3) PC.Ride->SetSeat(S.PoseProbeIndex);
            else if (S.PoseProbeIndex == 3) PC.Ride->ToggleOverview();
            else
            { S.PoseChecked = true; S.Event(TEXT("paused-seat-overview-restore-passed")); S.Advance(FState::Warmup); break; }
        }
        ++S.PoseProbeIndex; S.StageStarted = Now; break;
    }
    case FState::Warmup:
        if (Now - S.StageStarted < 3) break;
        S.TraversalStarted = Now; S.PreviousTick = Now; S.LastRideTime = 0; S.LastDistance = PC.Ride->Playback().Distance;
#if CSV_PROFILER
        if (!S.Screenshots) { FCsvProfiler::Get()->BeginCapture(-1, S.Output, TEXT("render.csv")); S.CsvStarted = true; }
#endif
        PC.Ride->TogglePause(); S.Event(TEXT("full-traversal-started")); S.Advance(FState::Traverse); break;
    case FState::Traverse:
        if (P.Time + 1e-8 < S.LastRideTime || P.Distance + 1e-6 < S.LastDistance) { S.Fail(TEXT("Playback regressed during uninterrupted traversal")); break; }
        S.LastRideTime = P.Time; S.LastDistance = P.Distance;
        if (S.NextShot < S.ShotTimes.Num() && P.Time >= S.ShotTimes[S.NextShot])
        {
            S.Event(TEXT("trace-capture-due"), TEXT(",\"target_time\":") + N(S.ShotTimes[S.NextShot]));
            S.Capture(PC, FString::Printf(TEXT("pov-%d"), S.NextShot++));
            break;
        }
        if (S.ReviewCameras && S.NextReview < S.ReviewShots.Num() &&
            P.Time >= S.ReviewShots[S.NextReview].Time)
        {
            if (PC.Ride->IsPaused())
            { S.Fail(TEXT("Review target was reached after playback stopped")); break; }
            const auto& Shot = S.ReviewShots[S.NextReview];
            const auto& D = *PC.Ride->ActiveDesign();
            S.ReviewTarget = PC.GetViewTarget();
            if (!S.ReviewTarget.IsValid()) { S.Fail(TEXT("Review camera target is unavailable")); break; }
            S.ReviewRiderPose = S.ReviewTarget->GetActorTransform();
            PC.Ride->TogglePause();
            S.ReviewPauseStarted = Now;
            coaster::Vec3 Centre{};
            constexpr int32 Samples = 16;
            for (int32 I = 0; I <= Samples; ++I)
                Centre = Centre + D.track.sample(FMath::Lerp(Shot.Begin, Shot.End, double(I) / Samples)).position / (Samples + 1.);
            double HorizontalRadius = 0, VerticalRadius = 0;
            for (int32 I = 0; I <= Samples; ++I)
            {
                const auto Delta = D.track.sample(FMath::Lerp(Shot.Begin, Shot.End, double(I) / Samples)).position - Centre;
                HorizontalRadius = std::max(HorizontalRadius, std::hypot(Delta.x, Delta.y));
                VerticalRadius = std::max(VerticalRadius, std::abs(Delta.z));
            }
            const auto Feature = D.track.sample(Shot.Distance);
            auto Horizontal = coaster::unit(coaster::Vec3{Feature.tangent.x, Feature.tangent.y, 0});
            if (coaster::norm(Horizontal) < .1)
            {
                const auto Delta = D.track.sample(Shot.End).position - D.track.sample(Shot.Begin).position;
                Horizontal = coaster::unit({Delta.x, Delta.y, 0});
            }
            if (coaster::norm(Horizontal) < .1) Horizontal = {1,0,0};
            const auto Right = coaster::cross(Horizontal, coaster::Vec3{0,0,1});
            int32 Side = Shot.Side;
            if (Side == 0)
            {
                const auto Plus = Feature.position + Right * 80.;
                const auto Minus = Feature.position - Right * 80.;
                Side = D.request.terrain.height(Plus.x, Plus.y) <
                    D.request.terrain.height(Minus.x, Minus.y) ? 1 : -1;
            }
            const bool Wide = Shot.Style == TEXT("overview");
            const bool Near = Shot.Style == TEXT("near-rail");
            const double Distance = std::max({Wide ? 90. : Near ? 35. : 55.,
                HorizontalRadius * (Wide ? 2.5 : Near ? 1.35 : 1.8),
                VerticalRadius * (Wide ? 2.5 : Near ? 1.7 : 2.1)});
            const double Height = std::max(Wide ? 35. : Near ? 10. : 17.,
                Distance * (Wide ? .65 : Near ? .18 : .30));
            auto Eye = Centre + Right * (Side * Distance) - Horizontal * (Distance * .24) +
                coaster::Vec3{0,0,Height};
            Eye.z = std::max(Eye.z, D.request.terrain.height(Eye.x, Eye.y) + 12.);
            const auto Look = Centre * .35 + Feature.position * .65;
            const auto E = VibeCoordinates::Position(Eye), L = VibeCoordinates::Position(Look);
            const FVector EyeCm(E.X, E.Y, E.Z), LookCm(L.X, L.Y, L.Z);
            S.ReviewTarget->SetActorLocationAndRotation(EyeCm, (LookCm - EyeCm).Rotation());
            const double RailGround = Feature.position.z -
                D.request.terrain.height(Feature.position.x, Feature.position.y);
            const auto HorizontalUp = coaster::Vec3{0,0,1} -
                Feature.tangent * Feature.tangent.z;
            const auto LevelUp = coaster::unit(HorizontalUp);
            const double BankDegrees = coaster::norm(HorizontalUp) > 1e-6
                ? std::atan2(coaster::dot(Feature.up, coaster::cross(Feature.tangent, LevelUp)),
                    coaster::dot(Feature.up, LevelUp)) * 180. / coaster::pi
                : std::numeric_limits<double>::quiet_NaN();
            S.Event(TEXT("review-camera-target"),
                TEXT(",\"label\":") + Q(Shot.Label) + TEXT(",\"source\":") + Q(Shot.Source) +
                TEXT(",\"target_distance_m\":") + N(Shot.Distance) +
                TEXT(",\"target_time_s\":") + N(Shot.Time) +
                TEXT(",\"observed_ride_time_s\":") + N(P.Time) +
                TEXT(",\"observed_front_distance_m\":") +
                    N(P.Distance + coaster::seatDistanceOffset(D.request.train, 0)) +
                TEXT(",\"span_start_m\":") + N(Shot.Begin) +
                TEXT(",\"span_end_m\":") + N(Shot.End) +
                TEXT(",\"style\":") + Q(Shot.Style) +
                TEXT(",\"bank_deg\":") + N(BankDegrees) +
                TEXT(",\"rail_ground_m\":") + N(RailGround) +
                TEXT(",\"camera_position_cm\":") + Q(EyeCm.ToString()) +
                TEXT(",\"camera_rotation_deg\":") + Q((LookCm - EyeCm).Rotation().ToString()));
            S.Advance(FState::ReviewView); break;
        }
        if (PC.Ride->IsPaused())
        {
            if (FMath::Abs(P.Time - S.Duration) > 1e-6 || FMath::Abs(P.Distance - S.FinalDistance) > 1e-4 || P.Speed > .01) { S.Fail(TEXT("Ride paused before its accepted terminal trace sample")); break; }
            S.Traversed = true; S.Event(TEXT("full-traversal-completed"), TEXT(",\"wall_duration_s\":") + N(Now - S.TraversalStarted) + TEXT(",\"ride_duration_s\":") + N(P.Time) + TEXT(",\"distance_m\":") + N(P.Distance));
#if CSV_PROFILER
            if (S.CsvStarted) { S.CsvFinished = FCsvProfiler::Get()->EndCapture(); S.CsvStarted = false; }
#endif
            S.Advance(FState::EndView);
        }
        break;
    case FState::ReviewView:
        if (Now - S.StageStarted < .3) break;
        S.Capture(PC, S.ReviewShots[S.NextReview].Label);
        S.Advance(FState::ReviewCaptured); break;
    case FState::ReviewCaptured:
        if (!S.ReviewTarget.IsValid()) { S.Fail(TEXT("Review camera target was lost")); break; }
        S.ReviewTarget->SetActorTransform(S.ReviewRiderPose);
        S.ReviewTarget.Reset();
        if (!PC.Ride->IsPaused()) { S.Fail(TEXT("Review capture changed paused playback")); break; }
        PC.Ride->TogglePause();
        S.ReviewPauseSeconds += Now - S.ReviewPauseStarted;
        S.Event(TEXT("review-camera-resumed"),
            TEXT(",\"label\":") + Q(S.ReviewShots[S.NextReview].Label) +
            TEXT(",\"paused_seconds\":") + N(Now - S.ReviewPauseStarted));
        ++S.NextReview; ++S.ReviewCaptures; S.PreviousTick = Now;
        S.Advance(FState::Traverse); break;
    case FState::EndView:
        S.Capture(PC, TEXT("terminal"));
        if (S.LoadOnly)
        {
            if (HashFile(S.SavePath) != S.SaveHash) S.Fail(TEXT("Cross-process load altered save bytes"));
            else { PC.Ride->Save(); PC.Ride->Cancel(); S.Event(TEXT("save-cancellation-requested")); S.Advance(FState::AwaitSaveCancel); }
        }
        else { PC.Ride->Save(); S.Event(TEXT("save-requested")); S.Advance(FState::AwaitSave); }
        break;
    case FState::AwaitSaveCancel:
        if (PC.Ride->IsBusy()) { if (Now - S.StageStarted > 120) S.Fail(TEXT("Cancelled save did not finish")); break; }
        if (HashFile(S.SavePath) != S.SaveHash || !PC.Ride->HasRide() ||
            GeometryIdentity(*PC.Ride->ActiveDesign()) != S.Identity || !PC.Ride->Status().StartsWith(TEXT("Save cancelled before commit;")))
        { S.Fail(TEXT("Pre-commit save cancellation did not preserve file, ride and truthful status")); break; }
        S.SaveCancelChecked = true; S.Event(TEXT("save-cancellation-preserved-file-and-ride"));
        // Load-only traversal finishes at the terminal stop; establish moving
        // playback before checking that a replacement request preserves it.
        PC.Ride->Restart();
        S.CancelRevision = PC.Ride->GeometryRevision(); S.CancelRideTime = PC.Ride->Playback().Time;
        if(PC.Ride->IsPaused()) PC.Ride->TogglePause();
        PC.Ride->Generate(PC.Ride->ActiveDesign()->request);
        S.Advance(FState::AwaitGenerationPhase); break;
    case FState::AwaitGenerationPhase:
        if(!PC.Ride->IsBusy() || PC.Ride->GeometryRevision()!=S.CancelRevision) { S.Fail(TEXT("Generation cancellation probe lost its previous ride")); break; }
        if(PC.Ride->Loading().Phase==coaster::WorkPhase::Authoring && Now-S.StageStarted>.15)
        { S.Capture(PC,TEXT("loading-generation")); S.Advance(FState::CancelGeneration); }
        else if(Now-S.StageStarted>5) S.Fail(TEXT("Generation probe did not observe active authoring"));
        break;
    case FState::CancelGeneration:
        PC.Ride->Cancel(); S.Event(TEXT("generation-cancellation-requested")); S.Advance(FState::AwaitGenerationCancel); break;
    case FState::AwaitGenerationCancel:
    case FState::AwaitMeshCancel:
    case FState::AwaitSceneCancel:
        if(PC.Ride->IsBusy()) { if(Now-S.StageStarted>2) S.Fail(TEXT("Cancellation did not complete within two seconds")); break; }
        if(!PC.Ride->HasRide() || PC.Ride->GeometryRevision()!=S.CancelRevision || GeometryIdentity(*PC.Ride->ActiveDesign())!=S.Identity || HashFile(S.SavePath)!=S.SaveHash || PC.Ride->Playback().Time<=S.CancelRideTime)
        { S.Fail(TEXT("Cancellation changed the accepted ride/save or stopped prior playback")); break; }
        if(S.Stage==FState::AwaitGenerationCancel)
        {
            S.GenerationCancelChecked=true; S.GenerationCancelSeconds=Now-S.StageStarted;
            S.Event(TEXT("generation-cancellation-preserved-playing-ride"),TEXT(",\"latency_s\":")+N(S.GenerationCancelSeconds));
            PC.Ride->Load(); S.Advance(FState::AwaitMeshPhase);
        }
        else if(S.Stage==FState::AwaitMeshCancel)
        {
            S.MeshCancelChecked=true; S.MeshCancelSeconds=Now-S.StageStarted;
            S.Event(TEXT("mesh-cancellation-preserved-playing-ride"),TEXT(",\"latency_s\":")+N(S.MeshCancelSeconds));
            PC.Ride->Load(); S.Advance(FState::AwaitScenePhase);
        }
        else
        {
            S.SceneCancelChecked=true; S.SceneCancelSeconds=Now-S.StageStarted;
            S.Event(TEXT("scene-cancellation-preserved-playing-ride"),TEXT(",\"latency_s\":")+N(S.SceneCancelSeconds));
            if(!PC.Ride->IsPaused()) PC.Ride->TogglePause(); S.Advance(FState::Finish);
        }
        break;
    case FState::AwaitMeshPhase:
    case FState::AwaitScenePhase:
        if(!PC.Ride->IsBusy() || PC.Ride->GeometryRevision()!=S.CancelRevision) { S.Fail(TEXT("Replacement committed before the cancellation phase was observed")); break; }
        if(PC.Ride->Loading().Phase==(S.Stage==FState::AwaitMeshPhase?coaster::WorkPhase::MeshPreparation:coaster::WorkPhase::SceneCommit))
        {
            const bool Mesh=S.Stage==FState::AwaitMeshPhase; PC.Ride->Cancel();
            S.Event(Mesh?TEXT("mesh-cancellation-requested"):TEXT("scene-cancellation-requested"));
            S.Advance(Mesh?FState::AwaitMeshCancel:FState::AwaitSceneCancel);
        }
        else if(Now-S.StageStarted>30) S.Fail(TEXT("Load did not reach the requested cancellation phase"));
        break;
    case FState::AwaitSave:
        if (PC.Ride->IsBusy()) { if (Now - S.StageStarted > 120) S.Fail(TEXT("Save timeout")); break; }
        S.SaveHash = HashFile(S.SavePath);
        if (S.SaveHash.IsEmpty() || !PC.Ride->Status().StartsWith(TEXT("Saved accepted geometry:"))) { S.Fail(TEXT("Save did not report success: ") + PC.Ride->Status()); break; }
        S.SaveChecked = true; S.Event(TEXT("save-completed"), TEXT(",\"sha1\":") + Q(S.SaveHash)); PC.Ride->Load(); S.Advance(FState::AwaitReload); break;
    case FState::AwaitReload:
        if (PC.Ride->IsBusy()) { if (Now - S.StageStarted > 180) S.Fail(TEXT("Load/commit timeout")); break; }
        if (!PC.Ride->HasRide() || !PC.Ride->ActiveDesign()->accepted() || GeometryIdentity(*PC.Ride->ActiveDesign()) != S.Identity || PC.Ride->GeometryRevision() <= S.CommittedRevision || HashFile(S.SavePath) != S.SaveHash || !PC.Ride->IsPaused() || PC.Ride->Playback().Time != 0)
        { S.Fail(TEXT("Saved design did not load, revalidate and commit identically")); break; }
        S.LoadChecked = true; S.Event(TEXT("load-and-revalidation-completed"));
        PC.Ride->Save(); PC.Ride->Cancel(); S.Event(TEXT("save-cancellation-requested")); S.Advance(FState::AwaitSaveCancel); break;
    default: break;
    }
}
#endif
