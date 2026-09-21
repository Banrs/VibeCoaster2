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
    TestFalse(TEXT("Default user flow leaves optional reference comparison disabled"), Controller->Settings.targets.requireIntensity);

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
    TestEqual(TEXT("Optional-reference guard retains the entered seed"), Controller->Settings.seed, uint64(123));
    TestEqual(TEXT("Missing-reference guard remains explicit"), Controller->InputError,
        FString(TEXT("External reference comparison requested, but no verified reference metadata is configured.")));
    TestTrue(TEXT("Input validation never created a ride"), Controller->Ride == nullptr);
    const TCHAR* Prefixes[]={TEXT("Seed:"),TEXT("Maximum track height"),TEXT("Top speed"),TEXT("Inversion height"),TEXT("Launch 0-180"),TEXT("Candidate search budget"),TEXT("Airtime strength"),TEXT("Signature outward roll"),TEXT("Return composition"),TEXT("Trim brakes"),TEXT("Landscape:")};
    for (int32 Row=0; Row<AVibeCoasterController::SetupRowCount; ++Row)
        TestTrue(TEXT("Main menu retains the correct row mapping"),Controller->RowText(Row).StartsWith(Prefixes[Row]));
    Controller->SelectedRow=1;const double Height=Controller->Settings.targets.height;Controller->ChangeRow(1);
    TestEqual(TEXT("Height row remains editable"),Controller->Settings.targets.height,Height+5.);
    Controller->SelectedRow=5;Controller->Settings.maxCandidates=8;Controller->ChangeRow(1);
    TestEqual(TEXT("Budget row changes the candidate budget"),Controller->Settings.maxCandidates,9);
    Controller->SelectedRow=6;Controller->ChangeRow(1);
    TestEqual(TEXT("Airtime customization is exposed"),Controller->Settings.style.airtime,1.05);
    Controller->SelectedRow=7;Controller->ChangeRow(1);
    TestEqual(TEXT("Signature customization is exposed"),Controller->Settings.style.signatureRollDegrees,50.);
    Controller->SelectedRow=8;Controller->ChangeRow(1);
    TestEqual(TEXT("Return composition cycles from automatic"),Controller->Settings.style.returnStyle,0);
    Controller->SelectedRow=9;Controller->ChangeRow(1);
    TestFalse(TEXT("Automatic trim installation can be disabled"),Controller->Settings.style.automaticTrims);
    Controller->SelectedRow=10;Controller->Settings.terrain.kind=coaster::TerrainKind::Highlands;Controller->ChangeRow(1);
    TestTrue(TEXT("Landscape can be changed to the flat comparison"),Controller->Settings.terrain.kind==coaster::TerrainKind::Flat);
    Controller->ChangeRow(-1);
    TestTrue(TEXT("Landscape selection restores highlands"),Controller->Settings.terrain.kind==coaster::TerrainKind::Highlands);
    TestTrue(TEXT("No hidden extra setup row"),Controller->RowText(AVibeCoasterController::SetupRowCount).IsEmpty());
    return true;
}
#endif
