#include "VibeCoasterGame.h"
#include "Engine/World.h"
#include "Misc/AutomationTest.h"
#include "Misc/ScopeExit.h"

#if WITH_DEV_AUTOMATION_TESTS
#include "../../../../core/tests/reference_fixture.hpp"

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FCoasterSeedInputContract, "VibeCoaster.SeedInputContract", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FCoasterSeedInputContract::RunTest(const FString& Parameters)
{
    UWorld* World = UWorld::CreateWorld(EWorldType::Game, false);
    if (!TestNotNull(TEXT("Transient input test world"), World)) return false;
    ON_SCOPE_EXIT { World->DestroyWorld(false); };
    auto* Controller = World->SpawnActor<AVibeCoasterController>();
    if (!TestNotNull(TEXT("Controller without a ride or generation worker"), Controller)) return false;
    Controller->Settings.targets.requireIntensity = false;

    struct FValidSeed { const TCHAR* Text; uint64 Value; };
    for (const FValidSeed Seed : {FValidSeed{TEXT("0"), 0}, {TEXT("42"), 42},
        {TEXT("00042"), 42}, {TEXT("18446744073709551615"), MAX_uint64}})
    {
        Controller->SeedText = Seed.Text;
        Controller->InputError = TEXT("previous error");
        Controller->RequestGeneration();
        TestEqual(TEXT("Valid seed is committed exactly"), Controller->Settings.seed, Seed.Value);
        TestTrue(TEXT("Valid request clears prior input error"), Controller->InputError.IsEmpty());
    }
    TArray<FString> Invalid{TEXT(""), TEXT("-1"), TEXT("+1"), TEXT(" 1"), TEXT("1 "),
        TEXT("42junk"), TEXT("18446744073709551616"), TEXT("999999999999999999999999"), TEXT("\u0661")};
    const TCHAR EmbeddedNull[] = {TEXT('4'), TEXT('2'), 0, TEXT('3')};
    Invalid.Add(FString(UE_ARRAY_COUNT(EmbeddedNull), EmbeddedNull));
    for (const FString& Text : Invalid)
    {
        Controller->SeedText = Text;
        Controller->Settings.seed = 17;
        Controller->RequestGeneration();
        TestEqual(TEXT("Invalid request preserves committed seed"), Controller->Settings.seed, uint64(17));
        TestEqual(TEXT("Invalid request preserves its error message"), Controller->InputError,
            FString(Text.IsEmpty() ? TEXT("Enter a seed (0 to 18446744073709551615).") : TEXT("Seed is outside unsigned 64-bit range.")));
        Controller->ChangeRow(1);
        TestEqual(TEXT("Right arrow falls back to committed seed"), Controller->SeedText, FString(TEXT("18")));
        Controller->SeedText = Text;
        Controller->ChangeRow(-1);
        TestEqual(TEXT("Left arrow falls back to committed seed"), Controller->SeedText, FString(TEXT("16")));
    }
    Controller->SeedText = TEXT("0");
    Controller->ChangeRow(-1);
    TestEqual(TEXT("Zero saturates on left arrow"), Controller->SeedText, FString(TEXT("0")));
    Controller->SeedText = TEXT("18446744073709551615");
    Controller->ChangeRow(1);
    TestEqual(TEXT("Maximum seed saturates on right arrow"), Controller->SeedText, FString(TEXT("18446744073709551615")));
    Controller->SeedText = TEXT("00042");
    Controller->ChangeRow(1);
    TestEqual(TEXT("Arrow output uses canonical decimal text"), Controller->SeedText, FString(TEXT("43")));

    Controller->SeedText = TEXT("123");
    Controller->Settings.targets.requireIntensity = true;
    Controller->RequestGeneration();
    TestEqual(TEXT("Reference guard retains the entered seed"), Controller->Settings.seed, uint64(123));
    TestEqual(TEXT("Missing-reference guard remains explicit"), Controller->InputError,
        FString(TEXT("ALL RECORDS unavailable: the measured I305/Pantherian force benchmark is missing.\nThis is not a failed seed search. PHYSICS-PROOF can test the other selected targets.")));

    // Metadata fixtures exercise input eligibility only; no synthetic reference
    // is installed as a production benchmark and this controller has no ride.
    coaster::Targets Scalar;
    Scalar.referenceExposure = 11; Scalar.referenceId = "SYNTHETIC scalar, not I305";
    auto Malformed = syntheticReference(); Malformed.reference.median += 1;
    for (const auto& Rejected : {coaster::Targets{}, Scalar, Malformed})
    {
        Controller->Settings.targets = Rejected;
        Controller->RequestGeneration();
        TestFalse(TEXT("Ineligible reference is blocked before starting generation"), Controller->InputError.IsEmpty());
        TestTrue(TEXT("Ineligible reference is unavailable in the menu"), Controller->RowText(2).Contains(TEXT("UNAVAILABLE")));
    }
    Controller->Settings.targets = syntheticReference();
    Controller->RequestGeneration();
    TestTrue(TEXT("Structurally eligible processed reference passes input preflight"), Controller->InputError.IsEmpty());
    TestFalse(TEXT("Structurally eligible reference is not shown as unavailable"), Controller->RowText(2).Contains(TEXT("UNAVAILABLE")));

    Controller->Settings.targets = Scalar;
    Controller->SelectedRow = 2; Controller->ChangeRow(1);
    Controller->RequestGeneration();
    TestTrue(TEXT("Explicit proof mode permits an unverified scalar diagnostic"), Controller->InputError.IsEmpty());
    TestFalse(TEXT("Proof mode disables only the intensity comparison"), Controller->Settings.targets.requireIntensity);
    TestTrue(TEXT("Proof mode preserves the scalar and selected physical targets"),
        Controller->Settings.targets.referenceId == Scalar.referenceId && Controller->Settings.targets.referenceExposure == Scalar.referenceExposure &&
        Controller->Settings.targets.height == Scalar.height && Controller->Settings.targets.speed == Scalar.speed &&
        Controller->Settings.targets.inversionHeight == Scalar.inversionHeight && Controller->Settings.targets.launchSeconds == Scalar.launchSeconds);
    TestTrue(TEXT("Input validation never created a ride"), Controller->Ride == nullptr);
    return true;
}
#endif
