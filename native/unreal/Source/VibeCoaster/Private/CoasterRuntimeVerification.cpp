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
#if WITH_EDITOR
#include "ShaderCompiler.h"
#endif
#include <iomanip>
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
    const auto& Landscape = D.request.terrain;
    S << Landscape.verticalScale << ';' << Landscape.horizontalScale << ';' << Landscape.offsetX << ';' << Landscape.offsetY << ';' << Landscape.headingRadians << ';' << Landscape.cliffHeight << ';' << Landscape.cliffWidth << ';';
    const auto& T = D.request.train;
    S << T.cars << ';' << T.carMass << ';' << T.spacing << ';' << T.seatHeight << ';' << T.dragCdA << ';' << T.rollingResistance << ';' << T.airDensity << ';';
    for (const auto& K : D.track.knots) { V(K.position); V(K.tangent); V(K.curvature); V(K.up); S << K.bank << ';' << int(K.element) << ';'; }
    for (const auto& O : D.operations) S << O.start << ';' << O.end << ';' << int(O.kind) << ';' << O.targetSpeed << ';' << O.maxForce << ';' << O.maxPower << ';' << O.rampSeconds << ';' << O.stopDeceleration << ';' << O.stopOffset << ';' << O.exitFadeMeters << ';';
    for (const auto& P : D.supports) { V(P.base); V(P.top); V(P.attachment); S << P.hasAttachment << ';' << P.trackDistance << ';'; for (const auto& M : P.members) { V(M.base); V(M.top); S << M.radiusBase << ';' << M.radiusTop << ';' << int(M.kind) << ';' << M.spineContact << ';'; } }
    S << coaster::stationPayload(D.station);
    const std::string Text = S.str(); return FSHA1::HashBuffer(Text.data(), Text.size()).ToString();
}
}

struct FCoasterRuntimeVerification::FState
{
    enum EStage { Init, DefaultView, StartRequest, AwaitRide, OverviewView, OverviewCaptured, StationView, StationCaptured, PauseProbe, PauseHold, PoseProbe, Warmup, Traverse, EndView, AwaitSave, AwaitReload, AwaitSaveCancel, Finish, Done } Stage = Init;
    FString Output, Profile, SavePath, Error, Identity, SaveHash, PendingShot, FrameRows = TEXT("wall_seconds,ride_seconds,distance_m,speed_ms,wall_frame_ms,engine_delta_ms\n");
    FString Seed = TEXT("42"), Terrain = TEXT("flat");
    uint64 CommittedRevision = 0;
    int32 PoseProbeIndex = 0;
    int32 Seat = 0, ShotIndex = 0, NextShot = 0, ObservedWidth = 0, ObservedHeight = 0;
    bool LoadOnly = false, Screenshots = true, RefusalChecked = false, Traversed = false, SaveChecked = false, LoadChecked = false, PauseChecked = false, RestartChecked = false;
    double Started = FPlatformTime::Seconds(), StageStarted = Started, TraversalStarted = 0, PreviousTick = 0, PauseTime = 0, LastRideTime = 0, LastDistance = 0, ShotRequested = 0, Duration = 0, FinalDistance = 0;
    TArray<double> ShotTimes;
    TSharedFuture<FString> CsvFinished;
    bool CsvStarted = false, PoseChecked = false, SaveCancelChecked = false;
#if WITH_EDITOR
    bool WaitingForShaders = false;
#endif

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
        S.SavePath = FPaths::ConvertRelativePathToFull(FPaths::ProjectSavedDir() / TEXT("Designs/Accepted.vcdesign"));
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
        S.Screenshots = !FParse::Param(FCommandLine::Get(), TEXT("CoasterVerifyNoScreenshots"));
        FParse::Value(FCommandLine::Get(), TEXT("CoasterVerifySeed="), S.Seed);
        FParse::Value(FCommandLine::Get(), TEXT("CoasterVerifyTerrain="), S.Terrain);
        FParse::Value(FCommandLine::Get(), TEXT("CoasterVerifySeat="), S.Seat);
        if (S.Seat < 0 || S.Seat > 2 || (S.Terrain != TEXT("flat") && S.Terrain != TEXT("hills") && S.Terrain != TEXT("canyon"))) { S.Fail(TEXT("Invalid terrain or seat (0 front, 1 middle, 2 rear)")); return; }
        S.Write(TEXT("environment.json"), TEXT("{\"utc\":") + Q(FDateTime::UtcNow().ToIso8601()) + TEXT(",\"engine\":") + Q(FEngineVersion::Current().ToString()) + TEXT(",\"cpu\":") + Q(FPlatformMisc::GetCPUBrand()) + TEXT(",\"os\":") + Q(FPlatformMisc::GetOSVersion()) + TEXT(",\"executable\":") + Q(FPlatformProcess::ExecutablePath()) + TEXT(",\"profile\":") + Q(S.Profile) + TEXT(",\"save\":") + Q(S.SavePath) + TEXT(",\"screenshots_enabled\":") + (S.Screenshots ? TEXT("true") : TEXT("false")) + TEXT("}\n"));
        S.Event(TEXT("begin"), TEXT(",\"app_version\":") + Q(VibeCoasterAppVersion) + TEXT(",\"geometry_version\":") + Q(UTF8_TO_TCHAR(coaster::generatorVersion))); S.Advance(FState::DefaultView); return;
    }
    if (!PC.Ride) { if (Now - S.Started > 30) S.Fail(TEXT("Runtime world did not become available")); else return; }
    if (S.Stage != FState::Finish && !S.Error.IsEmpty()) S.Advance(FState::Finish);
    if (S.Stage != FState::Finish && Now - S.Started > 1200) S.Fail(TEXT("Verification exceeded 1200 seconds"));
    if (S.Stage == FState::Finish)
    {
        if (S.Error.IsEmpty() && !S.PollCapture(PC)) return;
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
        const bool Passed = S.Error.IsEmpty() && S.Traversed && S.PauseChecked && S.RestartChecked && S.PoseChecked && S.SaveCancelChecked && S.LoadChecked && (S.LoadOnly || S.SaveChecked);
        if (!S.Write(TEXT("wall-frames.csv"), S.FrameRows)) S.Error += TEXT(" Frame output write failed.");
        const FString Report = TEXT("{\"status\":") + Q(Passed && S.Error.IsEmpty() ? TEXT("automated-smoke-passed") : TEXT("failed")) + TEXT(",\"error\":") + Q(S.Error) + TEXT(",\"mode\":\"PHYSICS-PROOF; intensity untested\",\"geometry_sha1\":") + Q(S.Identity) + TEXT(",\"save_sha1\":") + Q(S.SaveHash) + TEXT(",\"full_traversal\":") + (S.Traversed ? TEXT("true") : TEXT("false")) + TEXT(",\"ride_duration_s\":") + N(S.Duration) + TEXT(",\"final_distance_m\":") + N(S.FinalDistance) + TEXT(",\"seat\":") + FString::FromInt(S.Seat) + TEXT(",\"pause_checked\":") + (S.PauseChecked ? TEXT("true") : TEXT("false")) + TEXT(",\"restart_checked\":") + (S.RestartChecked ? TEXT("true") : TEXT("false")) + TEXT(",\"save_checked\":") + (S.SaveChecked ? TEXT("true") : TEXT("false")) + TEXT(",\"load_checked\":") + (S.LoadChecked ? TEXT("true") : TEXT("false")) + TEXT(",\"missing_reference_refusal_checked\":") + (S.RefusalChecked ? TEXT("true") : TEXT("false")) + TEXT(",\"viewport_width\":") + FString::FromInt(S.ObservedWidth) + TEXT(",\"viewport_height\":") + FString::FromInt(S.ObservedHeight) + TEXT(",\"paused_pose_checked\":") + (S.PoseChecked ? TEXT("true") : TEXT("false")) + TEXT(",\"save_cancel_checked\":") + (S.SaveCancelChecked ? TEXT("true") : TEXT("false")) + TEXT(",\"human_visual_review\":\"pending\",\"keyboard_input\":\"untested\",\"cancellation_phases\":\"see save_cancel_checked; other phases untested\",\"performance\":\"raw wall-frame samples; no FPS or GPU acceptance claim\"}\n");
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
#if WITH_EDITOR
    // Editor -game can display the default checkerboard while materials compile.
    // Wait only before playback, so the actual traversal stays uninterrupted.
    if (S.Stage == FState::DefaultView || S.Stage == FState::OverviewView || S.Stage == FState::Warmup)
    {
        if (GShaderCompilingManager && GShaderCompilingManager->IsCompiling())
        {
            if (!S.WaitingForShaders) S.Event(TEXT("shader-readiness-wait"));
            S.WaitingForShaders = true; return;
        }
        if (S.WaitingForShaders)
        {
            S.WaitingForShaders = false; S.StageStarted = Now;
            S.Event(TEXT("shader-readiness-completed")); return;
        }
    }
#endif
    switch (S.Stage)
    {
    case FState::DefaultView:
        PC.SeedText = S.Seed;
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
        PC.Settings.terrain.kind = S.Terrain == TEXT("flat") ? coaster::TerrainKind::Flat : S.Terrain == TEXT("hills") ? coaster::TerrainKind::Hills : coaster::TerrainKind::Canyon;
        if (S.LoadOnly) { S.SaveHash = HashFile(S.SavePath); PC.Ride->Load(); S.Event(TEXT("cross-process-load-requested")); }
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
            double ApexTime = 0, ApexHeight = -1e30, HighestTime = 0, HighestGround = -1e30;
            double CliffTime = 0, CliffGrade = 0, LowPassTime = 0, LowPassHeight = 1e30;
            TArray<double> InversionPassages, UprightReturns;
            bool WasInverted = false, AwaitUprightReturn = false;
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
                if (Inverted) AwaitUprightReturn = true;
                if (AwaitUprightReturn && K.up.z > .5) { UprightReturns.Add(F.time); AwaitUprightReturn = false; }
                WasInverted = Inverted;
                const double Ahead = D.request.terrain.height(K.position.x + K.tangent.x * 5, K.position.y + K.tangent.y * 5);
                const double Behind = D.request.terrain.height(K.position.x - K.tangent.x * 5, K.position.y - K.tangent.y * 5);
                const double GroundGrade = (Ahead - Behind) / 10;
                if (GroundGrade < CliffGrade) { CliffGrade = GroundGrade; CliffTime = F.time; }
            }
            if (ApexHeight == -1e30) { S.Fail(TEXT("Accepted trace lacks an observable inverted-apex passage")); break; }
            S.ShotTimes = { HighestTime, S.Duration * .15, S.Duration * .35, S.Duration * .55, S.Duration * .75, FMath::Max(0., ApexTime - .4), ApexTime };
            S.Event(TEXT("pov-landmarks"), TEXT(",\"highest_ground_time_s\":") + N(HighestTime) + TEXT(",\"highest_ground_m\":") + N(HighestGround) + TEXT(",\"inverted_apex_time_s\":") + N(ApexTime) + TEXT(",\"inverted_apex_ground_m\":") + N(ApexHeight));
            // Inspect each actual inverted passage, plus the descending terrain
            // edge. These are sampled frames during full playback, not video.
            for (double Time : InversionPassages) { S.ShotTimes.Add(FMath::Max(0., Time - 1)); S.ShotTimes.Add(Time + .5); }
            // Entry/apex views alone miss the rolling descent and its pullout.
            for (double Time : UprightReturns) { S.ShotTimes.Add(Time); S.ShotTimes.Add(FMath::Min(S.Duration, Time + 1)); }
            if (D.request.terrain.kind == coaster::TerrainKind::Canyon)
            {
                S.ShotTimes.Add(FMath::Max(0., CliffTime - 2)); S.ShotTimes.Add(CliffTime); S.ShotTimes.Add(FMath::Min(S.Duration, CliffTime + 2));
                S.Event(TEXT("cliff-landmark"), TEXT(",\"time_s\":") + N(CliffTime) + TEXT(",\"ground_grade\":") + N(CliffGrade));
            }
            if (LowPassHeight < 1e30)
            {
                for (double Offset : {-1., 0., 1.}) S.ShotTimes.Add(FMath::Clamp(LowPassTime + Offset, 0., S.Duration));
                S.Event(TEXT("low-pass-landmark"), TEXT(",\"time_s\":") + N(LowPassTime) + TEXT(",\"centerline_ground_m\":") + N(LowPassHeight));
            }
            S.ShotTimes.Sort();
            for (int32 I = S.ShotTimes.Num() - 1; I > 0; --I) if (S.ShotTimes[I] - S.ShotTimes[I - 1] < .25) S.ShotTimes.RemoveAt(I);
            // Retain a view inside every persisted operation, using the selected
            // rider's actual trace passage and the simulator's lap activation.
            // Prefer its spatial midpoint; move inward if a one-second margin
            // at both observed passage ends is feasible. This never alters playback.
            for (size_t I = 0; I < D.operations.size(); ++I)
            {
                const auto& O = D.operations[I];
                const double Length = O.end >= O.start ? O.end - O.start : D.track.length - O.start + O.end;
                double Enter = -1, Exit = -1, Midpoint = -1, Nearest = 1e30;
                for (const auto& F : D.simulation.frames)
                {
                    if ((O.kind == coaster::DriveKind::Launch && F.distance > D.track.length * .5) ||
                        (O.kind == coaster::DriveKind::Station && F.distance < D.track.length * .5)) continue;
                    double At = std::fmod(F.distance + coaster::seatDistanceOffset(D.request.train, S.Seat), D.track.length);
                    if (At < 0) At += D.track.length;
                    if (!(O.start <= O.end ? At >= O.start && At < O.end : At >= O.start || At < O.end)) continue;
                    if (Enter < 0) Enter = F.time;
                    Exit = F.time;
                    double Along = At - O.start; if (Along < 0) Along += D.track.length;
                    const double Difference = std::abs(Along - Length * .5);
                    if (Difference < Nearest) { Nearest = Difference; Midpoint = F.time; }
                }
                const TCHAR* Kind = O.kind == coaster::DriveKind::Launch ? TEXT("Launch") : O.kind == coaster::DriveKind::Boost ? TEXT("Boost") : O.kind == coaster::DriveKind::Brake ? TEXT("Brake") : TEXT("Station");
                const double Time = Exit - Enter >= 2 ? FMath::Clamp(Midpoint, Enter + 1, Exit - 1) : Midpoint;
                S.Event(TEXT("operation-landmark"), TEXT(",\"operation_index\":") + FString::FromInt(int32(I)) +
                    TEXT(",\"kind\":") + Q(Kind) + TEXT(",\"start_m\":") + N(O.start) + TEXT(",\"end_m\":") + N(O.end) +
                    TEXT(",\"target_speed_ms\":") + N(O.targetSpeed) + TEXT(",\"observed\":") + (Enter >= 0 ? TEXT("true") : TEXT("false")) +
                    TEXT(",\"entry_time_s\":") + N(Enter) + TEXT(",\"exit_time_s\":") + N(Exit) + TEXT(",\"spatial_midpoint_time_s\":") + N(Midpoint) +
                    TEXT(",\"scheduled_time_s\":") + N(Time) + TEXT(",\"one_second_margin_feasible\":") + (Exit - Enter >= 2 ? TEXT("true") : TEXT("false")));
                if (Enter >= 0) S.ShotTimes.Add(Time);
            }
            S.ShotTimes.Sort();
            // Exact duplicates share a screenshot (e.g. paired controllers), but
            // nearby operation views must not be discarded by landmark thinning.
            for (int32 I = S.ShotTimes.Num() - 1; I > 0; --I) if (S.ShotTimes[I] == S.ShotTimes[I - 1]) S.ShotTimes.RemoveAt(I);
            PC.Menu = false; PC.ShowComparison = false; PC.ShowTelemetry = true; PC.Ride->SetSeat(S.Seat);
            if (!PC.Ride->IsPaused()) PC.Ride->TogglePause(); PC.Ride->Restart();
            if (!PC.Ride->IsOverview()) PC.Ride->ToggleOverview();
            S.Advance(FState::OverviewView);
        }
        break;
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
        { S.Event(TEXT("trace-capture-due"), TEXT(",\"target_time\":") + N(S.ShotTimes[S.NextShot])); S.Capture(PC, FString::Printf(TEXT("pov-%d"), S.NextShot++)); }
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
        S.SaveCancelChecked = true; S.Event(TEXT("save-cancellation-preserved-file-and-ride")); S.Advance(FState::Finish); break;
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
