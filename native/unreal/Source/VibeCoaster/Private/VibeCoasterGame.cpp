#include "VibeCoasterGame.h"
#include "CoasterBuildVersion.h"
#include "CoasterRuntimeVerification.h"
#include "VibeCoasterWorld.h"
#include "CoordinateContract.h"
#include "Engine/Canvas.h"
#include "Engine/Engine.h"
#include "Engine/World.h"
#include "EngineUtils.h"
#include "InputCoreTypes.h"
#include "Misc/ConfigCacheIni.h"
#include "Misc/CommandLine.h"
#include "Misc/Parse.h"
#include "Misc/Paths.h"
#include <charconv>
#include <limits>

static bool ReadSeed(const FString& Text, uint64& Value)
{
    FTCHARToUTF8 Utf8(*Text, Text.Len());
    const char* End = Utf8.Get() + Utf8.Length();
    const auto Parsed = std::from_chars(Utf8.Get(), End, Value);
    return Parsed.ec == std::errc{} && Parsed.ptr == End;
}

#if !UE_BUILD_SHIPPING && !UE_BUILD_TEST
void FCoasterRuntimeVerificationDeleter::operator()(FCoasterRuntimeVerification* Pointer) const { delete Pointer; }
#endif

AVibeCoasterController::AVibeCoasterController() { PrimaryActorTick.bCanEverTick = true; bAutoManageActiveCameraTarget = false; Settings.terrain.kind = coaster::TerrainKind::Highlands; }
AVibeCoasterController::~AVibeCoasterController() = default;
void AVibeCoasterController::BeginPlay()
{
    Super::BeginPlay();
#if !UE_BUILD_SHIPPING && !UE_BUILD_TEST
    Verification.Reset(FCoasterRuntimeVerification::Create().Release());
#endif
    bShowMouseCursor = false; SetInputMode(FInputModeGameOnly());
    // Optional user-supplied processed reference trace.
    FString ReferenceFile;
    if (GConfig->GetString(TEXT("CoasterReference"), TEXT("File"), ReferenceFile, GGameIni))
    {
        if (FPaths::IsRelative(ReferenceFile)) ReferenceFile = FPaths::ProjectSavedDir() / ReferenceFile;
        std::string Error;
        if (!coaster::loadReference(TCHAR_TO_UTF8(*ReferenceFile), Settings.targets, Error))
            ReferenceError = TEXT("Reference file rejected: ") + FString(UTF8_TO_TCHAR(Error.c_str()));
        return; // An explicitly requested file never silently falls back to a scalar.
    }
    double Exposure = 0; FString Reference;
    if (GConfig->GetDouble(TEXT("CoasterReference"), TEXT("Exposure10Seconds"), Exposure, GGameIni) &&
        GConfig->GetString(TEXT("CoasterReference"), TEXT("ReferenceId"), Reference, GGameIni) &&
        std::isfinite(Exposure) && Exposure > 0 && !Reference.TrimStartAndEnd().IsEmpty())
    {
        Settings.targets.referenceExposure = Exposure;
        Settings.targets.referenceId = TCHAR_TO_UTF8(*Reference);
    }
}
void AVibeCoasterController::ChangeRow(int32 Direction)
{
    switch (SelectedRow)
    {
    case 0:
    {
        uint64 Value = 0;
        if (!ReadSeed(SeedText, Value)) Value = Settings.seed;
        if (Direction > 0 && Value < std::numeric_limits<uint64>::max()) ++Value;
        if (Direction < 0 && Value > 0) --Value;
        SeedText = FString::Printf(TEXT("%llu"), static_cast<unsigned long long>(Value));
        break;
    }
    case 1: Settings.targets.height = FMath::Clamp(Settings.targets.height + Direction * 5, 180., 350.); break;
    case 2: Settings.targets.speed = FMath::Clamp(Settings.targets.speed + Direction * (10 / 3.6), 250 / 3.6, 390 / 3.6); break;
    case 3: Settings.targets.inversionHeight = FMath::Clamp(Settings.targets.inversionHeight + Direction * 5, 80., 140.); break;
    case 4: Settings.targets.launchSeconds = FMath::Clamp(Settings.targets.launchSeconds + Direction * .05, 1.1, 2.); break;
    case 5: Settings.maxCandidates = FMath::Clamp(Settings.maxCandidates + Direction, 1, 32); break;
    case 6: Settings.style.airtime = FMath::Clamp(Settings.style.airtime + Direction * .05, .75, 1.2); break;
    case 7: Settings.style.signatureRollDegrees = FMath::Clamp(Settings.style.signatureRollDegrees + Direction * 5, 30., 60.); break;
    case 8: Settings.style.returnStyle = (Settings.style.returnStyle + Direction + 4) % 3 - 1; break;
    case 9: Settings.style.automaticTrims = !Settings.style.automaticTrims; break;
    case 10: Settings.terrain.kind = Settings.terrain.kind == coaster::TerrainKind::Flat ? coaster::TerrainKind::Highlands : coaster::TerrainKind::Flat; break;
    }
}
void AVibeCoasterController::RequestGeneration()
{
    InputError.Empty();
    if (SeedText.IsEmpty()) { InputError = TEXT("Enter a seed (0 to 18446744073709551615)."); return; }
    uint64 Value = 0;
    if (!ReadSeed(SeedText, Value))
    { InputError = TEXT("Seed is outside unsigned 64-bit range."); return; }
    Settings.seed = Value;
    if (Settings.targets.requireIntensity && (!std::isfinite(Settings.targets.referenceExposure) || Settings.targets.referenceId.empty()))
    {
        InputError = TEXT("External reference comparison requested, but no verified reference metadata is configured.");
        return;
    }
    if (Ride) Ride->Generate(Settings);
}
void AVibeCoasterController::PlayerTick(float DeltaSeconds)
{
    Super::PlayerTick(DeltaSeconds);
    if (!Ride) for (TActorIterator<AVibeCoasterWorld> It(GetWorld()); It; ++It)
    {
        Ride = *It;
        if (FParse::Param(FCommandLine::Get(), TEXT("CoasterLoad"))) { Menu = false; Ride->Load(); }
        break;
    }
#if !UE_BUILD_SHIPPING && !UE_BUILD_TEST
    if (Verification) Verification->Tick(*this, DeltaSeconds);
#endif
    if (WasInputKeyJustPressed(EKeys::Tab)) { Menu = !Menu; ShowComparison = false; }
    if (WasInputKeyJustPressed(EKeys::C)) { ShowComparison = !ShowComparison; ComparisonOffset = 0; }
    if (ShowComparison)
    {
        if (WasInputKeyJustPressed(EKeys::PageDown)) ComparisonOffset += 8;
        if (WasInputKeyJustPressed(EKeys::PageUp)) ComparisonOffset = FMath::Max(0, ComparisonOffset - 8);
    }
    if (WasInputKeyJustPressed(EKeys::Escape)) { if (Ride && Ride->IsBusy()) Ride->Cancel(); else Menu = true; }
    if (WasInputKeyJustPressed(EKeys::G)) RequestGeneration();
    if (Ride)
    {
        if (WasInputKeyJustPressed(EKeys::SpaceBar)) Ride->TogglePause();
        if (WasInputKeyJustPressed(EKeys::R)) Ride->Restart();
        if (WasInputKeyJustPressed(EKeys::M)) Ride->ToggleOverview();
        if (WasInputKeyJustPressed(EKeys::T)) ShowTelemetry = !ShowTelemetry;
        if (WasInputKeyJustPressed(EKeys::F5)) Ride->Save();
        if (WasInputKeyJustPressed(EKeys::F9)) Ride->Load();
        if (!Menu)
        {
            if (WasInputKeyJustPressed(EKeys::One)) Ride->SetSeat(0);
            if (WasInputKeyJustPressed(EKeys::Two)) Ride->SetSeat(1);
            if (WasInputKeyJustPressed(EKeys::Three)) Ride->SetSeat(2);
        }
    }
    if (!Menu || ShowComparison) return;
    if (WasInputKeyJustPressed(EKeys::Up)) SelectedRow = (SelectedRow + SetupRowCount - 1) % SetupRowCount;
    if (WasInputKeyJustPressed(EKeys::Down)) SelectedRow = (SelectedRow + 1) % SetupRowCount;
    if (WasInputKeyJustPressed(EKeys::Left)) ChangeRow(-1);
    if (WasInputKeyJustPressed(EKeys::Right)) ChangeRow(1);
    if (WasInputKeyJustPressed(EKeys::Enter)) RequestGeneration();
    if (SelectedRow == 0)
    {
        const FKey Digits[] = {EKeys::Zero, EKeys::One, EKeys::Two, EKeys::Three, EKeys::Four, EKeys::Five, EKeys::Six, EKeys::Seven, EKeys::Eight, EKeys::Nine};
        const FKey Numpad[] = {EKeys::NumPadZero, EKeys::NumPadOne, EKeys::NumPadTwo, EKeys::NumPadThree, EKeys::NumPadFour, EKeys::NumPadFive, EKeys::NumPadSix, EKeys::NumPadSeven, EKeys::NumPadEight, EKeys::NumPadNine};
        if (WasInputKeyJustPressed(EKeys::BackSpace)) SeedText.LeftChopInline(1);
        if (WasInputKeyJustPressed(EKeys::Delete)) SeedText.Empty();
        for (int32 I = 0; I < 10; ++I) if ((WasInputKeyJustPressed(Digits[I]) || WasInputKeyJustPressed(Numpad[I])) && SeedText.Len() < 20) SeedText += FString::FromInt(I);
    }
}
FString AVibeCoasterController::RowText(int32 Row) const
{
    switch (Row)
    {
    case 0: return TEXT("Seed: ") + SeedText + TEXT("   (Left/Right changes; type digits; Delete clears)");
    case 1: return FString::Printf(TEXT("Maximum track height above ground >= %.0f m"), Settings.targets.height);
    case 2: return FString::Printf(TEXT("Top speed %.0f km/h"), Settings.targets.speed * 3.6);
    case 3: return FString::Printf(TEXT("Inversion height above ground >= %.0f m"), Settings.targets.inversionHeight);
    case 4: return FString::Printf(TEXT("Launch 0-180 km/h in %.2f s   (historical benchmark: 1.56 s)"), Settings.targets.launchSeconds);
    case 5: return FString::Printf(TEXT("Candidate search budget: %d"), Settings.maxCandidates);
    case 6: return FString::Printf(TEXT("Airtime strength: %.0f%%"), Settings.style.airtime * 100);
    case 7: return FString::Printf(TEXT("Signature outward roll: %.0f degrees"), Settings.style.signatureRollDegrees);
    case 8: return FString::Printf(TEXT("Return composition: %s"), Settings.style.returnStyle < 0 ? TEXT("seed choice") : Settings.style.returnStyle == 0 ? TEXT("flowing crest") : TEXT("twin airtime"));
    case 9: return Settings.style.automaticTrims ? TEXT("Trim brakes: automatic") : TEXT("Trim brakes: not installed");
    case 10: return Settings.terrain.kind == coaster::TerrainKind::Highlands ? TEXT("Landscape: green highlands") : TEXT("Landscape: flat basin");
    default: return FString();
    }
}
void AVibeCoasterHUD::DrawHUD()
{
    Super::DrawHUD();
    auto* PC = Cast<AVibeCoasterController>(GetOwningPlayerController()); if (!PC || !Canvas || !GEngine) return;
    const float Scale = FMath::Clamp(Canvas->SizeY / 900.f, .65f, 1.35f);
    const float X = 24 * Scale, Width = FMath::Min(850 * Scale, Canvas->SizeX - X * 2);
    float Y = 20 * Scale;
    auto Line = [&](const FString& Text, FLinearColor Colour = FLinearColor(.9f, .94f, 1.f))
    {
        TArray<FString> Lines; Text.ParseIntoArrayLines(Lines, false);
        for (const FString& Value : Lines) { DrawText(Value, Colour, X + 16 * Scale, Y, GEngine->GetSmallFont(), Scale * 1.15f); Y += 23 * Scale; }
    };
    const FLinearColor Accent(.25f, .85f, .92f), Amber(1.f, .72f, .3f);
    // At kilometre-scale overview distances, correctly sized steel is subpixel.
    // A screen-space route outline keeps the actual canonical circuit legible.
    if (PC->Ride && PC->Ride->IsOverview() && !PC->Menu && !PC->ShowComparison)
    {
        if (const auto* Design = PC->Ride->ActiveDesign())
        {
            // Resample only after an accepted commit; project every draw so
            // camera and viewport changes remain exact.
            const uint64 Revision = PC->Ride->GeometryRevision();
            if (OutlineRide.Get() != PC->Ride.Get() || OutlineRevision != Revision)
            {
                OutlineRide = PC->Ride;
                OutlineRevision = Revision;
                const int32 Count = FMath::CeilToInt(Design->track.length / 12.);
                OutlineWorldPoints.Reset(Count + 1);
                OutlineColours.Reset(Count);
                for (int32 I = 0; I <= Count; ++I)
                {
                    const auto Sample = Design->track.sample(Design->track.length * I / Count);
                    const auto P = VibeCoordinates::Position(Sample.position);
                    OutlineWorldPoints.Add(FVector(P.X, P.Y, P.Z));
                    if (I == 0) continue;
                    const auto Element = Sample.element;
                    OutlineColours.Add(Element == coaster::Element::Inversion ? FLinearColor(1.f,.42f,.22f)
                        : Element == coaster::Element::Hill ? FLinearColor(1.f,.85f,.25f)
                        : Element == coaster::Element::Airtime ? FLinearColor(.25f,1.f,.52f) : FLinearColor(.65f,.92f,1.f));
                }
            }
            OutlineScreenPoints.SetNumUninitialized(OutlineWorldPoints.Num());
            for (int32 I = 0; I < OutlineWorldPoints.Num(); ++I)
                OutlineScreenPoints[I] = Canvas->Project(OutlineWorldPoints[I]);
            const int32 Count = OutlineColours.Num();
            // Draw every under-stroke first so adjacent segments cannot erase the colour at joins.
            for (int32 Pass = 0; Pass < 2; ++Pass)
            {
                for (int32 I = 1; I <= Count; ++I)
                {
                    const FVector& Previous = OutlineScreenPoints[I - 1];
                    const FVector& Current = OutlineScreenPoints[I];
                    if (Previous.Z > 0 && Current.Z > 0)
                        DrawLine(Previous.X, Previous.Y, Current.X, Current.Y,
                            Pass == 0 ? FLinearColor(.012f,.022f,.03f,.9f) : OutlineColours[I - 1],
                            (Pass == 0 ? 5.f : 2.f) * Scale);
                }
            }
            const float FooterY = Canvas->SizeY - 42.f * Scale;
            DrawRect(FLinearColor(.012f,.025f,.044f,.85f), X, FooterY-8*Scale, 920*Scale, 40*Scale);
            DrawText(FString::Printf(TEXT("Circuit %.2f km  |  Gold: record hill  |  Coral: inversions  |  Green: airtime hills"), Design->track.length/1000.),
                FLinearColor::White, X+16*Scale, FooterY, GEngine->GetSmallFont(), Scale*1.15f);
        }
    }

    if (PC->ShowComparison)
    {
        DrawRect(FLinearColor(.012f, .025f, .044f, .96f), X, 8 * Scale, Canvas->SizeX - X * 2, Canvas->SizeY - 16 * Scale);
        Line(TEXT("RIDE COMPARISON  |  C closes  |  Page Up/Down scrolls"), Accent);
        TArray<FString> Lines;
        if (PC->Ride && PC->Ride->HasRide()) Lines = PC->Ride->Comparison();
        else for (const auto& Text : coaster::referenceLines(PC->Settings.targets)) Lines.Add(FString(UTF8_TO_TCHAR(Text.c_str())));
        if (!PC->ReferenceError.IsEmpty()) Lines.Add(PC->ReferenceError);
        const int32 Visible = FMath::Max(1, FMath::FloorToInt((Canvas->SizeY - Y - 50 * Scale) / (23 * Scale)));
        PC->ComparisonOffset = FMath::Clamp(PC->ComparisonOffset, 0, FMath::Max(0, Lines.Num() - Visible));
        for (int32 I = PC->ComparisonOffset; I < FMath::Min(Lines.Num(), PC->ComparisonOffset + Visible); ++I) Line(Lines[I]);
        return;
    }
    if (PC->Menu)
    {
        DrawRect(FLinearColor(.012f, .025f, .044f, .94f), X, 8 * Scale, Width, 585 * Scale);
        Line(FString(TEXT("VIBECOASTER  /  ")) + VibeCoasterAppVersion, Accent);
        Line(TEXT("Up/Down selects  |  Left/Right changes  |  Enter/G generates"));
        Y += 8 * Scale;
        for (int32 Row = 0; Row < AVibeCoasterController::SetupRowCount; ++Row) Line((Row == PC->SelectedRow ? TEXT("> ") : TEXT("  ")) + PC->RowText(Row), Row == PC->SelectedRow ? Accent : FLinearColor::White);
        Y += 8 * Scale;
        if (PC->Settings.targets.reference.processed) Line(TEXT("Optional external comparison metadata available. C opens comparison."), Amber);
        else if (std::isfinite(PC->Settings.targets.referenceExposure) && !PC->Settings.targets.referenceId.empty()) Line(TEXT("Optional external comparison metadata configured. C opens comparison."), Amber);
        if (!PC->ReferenceError.IsEmpty()) Line(PC->ReferenceError, Amber);
        Line(TEXT("Tab hides setup  |  Esc cancels current generation/load"));
        Line(TEXT("Space pause/ride  |  R restart  |  1/2/3 POV when setup hidden"));
        Line(TEXT("C comparison  |  M overview  |  T telemetry  |  F5 save  |  F9 load"));
        if (!PC->InputError.IsEmpty()) Line(PC->InputError, Amber);
        Y = 590 * Scale;
    }
    else
    {
        DrawRect(FLinearColor(.012f, .025f, .044f, .75f), X, 8 * Scale, Width, 78 * Scale);
        const FString View = PC->Ride && PC->Ride->IsOverview() ? TEXT("Overview") : PC->Ride && PC->Ride->Seat() == 1 ? TEXT("Middle POV") : PC->Ride && PC->Ride->Seat() == 2 ? TEXT("Rear POV") : TEXT("Front POV");
        Line((PC->Ride && PC->Ride->HasRide() ? View + (PC->Ride->IsPaused() ? TEXT("  /  paused") : TEXT("  /  riding")) : TEXT("No accepted ride")) + TEXT("  |  ") + VibeCoasterAppVersion, Accent);
        Line(TEXT("Tab setup  |  C comparison  |  Space pause  |  R restart  |  1/2/3 POV  |  M overview"));
        Y = 102 * Scale;
    }
    if (PC->Ride)
    {
        const auto Loading = PC->Ride->Loading();
        if (Loading.Active)
        {
            const float Top = Y - 8 * Scale, BarX = X + 16 * Scale, BarWidth = Width - 32 * Scale;
            DrawRect(FLinearColor(.012f,.025f,.044f,.94f),X,Top,Width,160*Scale);
            DrawRect(Accent,X,Top,4*Scale,160*Scale);
            Line(Loading.Cancelling ? TEXT("Cancelling...") : Loading.Title,Accent);
            Line(FString(UTF8_TO_TCHAR(coaster::phaseName(Loading.Phase))));
            Line(FString::Printf(TEXT("%.1f seconds elapsed%s"),Loading.ElapsedSeconds,
                Loading.Candidate > 0 ? *FString::Printf(TEXT("  |  design attempt %d"),Loading.Candidate+1) : TEXT("")));
            const float BarY = Y + 4*Scale;
            DrawRect(FLinearColor(.08f,.14f,.19f),BarX,BarY,BarWidth,6*Scale);
            if (Loading.Total > 0)
                DrawRect(Accent,BarX,BarY,BarWidth*FMath::Clamp(Loading.Completed/Loading.Total,0.,1.),6*Scale);
            else
            {
                const float Pulse = .5f+.5f*FMath::Sin(Loading.ElapsedSeconds*2);
                DrawRect(Accent,BarX+Pulse*BarWidth*.75f,BarY,BarWidth*.25f,6*Scale);
            }
            Y += 26*Scale;
            Line(PC->Ride->HasRide() ? TEXT("Your current ride is ready to use. Esc cancels this request.") : TEXT("Esc cancels this request."));
            Y = Top + 174*Scale;
        }
        // Keep telemetry legible against bright sky as the camera rolls.
        const FString StatusText = Loading.Active ? FString() : PC->Ride->Status();
        const FString TelemetryText = PC->ShowTelemetry && !Loading.Active ? PC->Ride->Telemetry() : FString();
        TArray<FString> StatusLines, TelemetryLines;
        StatusText.ParseIntoArrayLines(StatusLines, false);
        TelemetryText.ParseIntoArrayLines(TelemetryLines, false);
        const int32 PanelLines = StatusLines.Num() + TelemetryLines.Num() + (PC->Ride->HasRide() ? 1 : 0);
        DrawRect(FLinearColor(.012f, .025f, .044f, .82f), X, Y - 8 * Scale, Width, (PanelLines * 23 + 16) * Scale);
        Line(StatusText, Amber);
        if (const auto* D = PC->Ride->ActiveDesign())
        {
            Line(FString::Printf(TEXT("Active accepted seed %llu | %s%s"), static_cast<unsigned long long>(D->request.seed),
                UTF8_TO_TCHAR(D->request.terrain.name().c_str()), D->request.targets.reference.processed ? TEXT(" | optional comparison metadata") : TEXT("")));
        }
        if (PC->ShowTelemetry) Line(TelemetryText);
    }
}
AVibeCoasterGameMode::AVibeCoasterGameMode()
{
    PlayerControllerClass = AVibeCoasterController::StaticClass(); HUDClass = AVibeCoasterHUD::StaticClass();
    DefaultPawnClass = nullptr;
}
void AVibeCoasterGameMode::StartPlay()
{
    Super::StartPlay(); GetWorld()->SpawnActor<AVibeCoasterWorld>();
}

