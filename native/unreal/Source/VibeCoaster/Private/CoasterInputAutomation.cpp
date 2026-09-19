#include "VibeCoasterGame.h"
#include "Engine/World.h"
#include "Misc/AutomationTest.h"
#include "Misc/ScopeExit.h"

#if WITH_DEV_AUTOMATION_TESTS
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
    TestTrue(TEXT("Input validation never created a ride"), Controller->Ride == nullptr);
    // Row 3 reads "Top speed", not "Maximum speed": the dial stopped being a floor
    // the ride could exceed by whatever margin it liked and became a setpoint it
    // lands on, so calling it a maximum in the menu would now be a lie.
    const TCHAR* Prefixes[]={TEXT("Seed:"),TEXT("Mode:"),TEXT("Maximum track height"),TEXT("Top speed"),TEXT("Inversion height"),TEXT("Launch 0-180"),TEXT("Candidate search budget")};
    for (int32 Row=0; Row<7; ++Row)
        TestTrue(TEXT("Flat-only menu retains the correct row mapping"),Controller->RowText(Row).StartsWith(Prefixes[Row]));
    Controller->SelectedRow=1;Controller->Settings.targets.requireIntensity=true;Controller->ChangeRow(1);
    TestFalse(TEXT("Second row changes the mode"),Controller->Settings.targets.requireIntensity);
    Controller->SelectedRow=6;Controller->Settings.maxCandidates=8;Controller->ChangeRow(1);
    TestEqual(TEXT("Last row changes the candidate budget"),Controller->Settings.maxCandidates,9);
    TestTrue(TEXT("Removed terrain row is not present"),Controller->RowText(7).IsEmpty());
    return true;
}
#endif
