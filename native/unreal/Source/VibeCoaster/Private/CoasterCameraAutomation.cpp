#include "CoasterCamera.h"
#include "Misc/AutomationTest.h"

#if WITH_DEV_AUTOMATION_TESTS
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FCoasterCameraContract, "VibeCoaster.CameraContract", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FCoasterCameraContract::RunTest(const FString& Parameters)
{
    FCoasterCameraRig Rig;
    const FVector Focus(12000, -3000, 4000);
    Rig.SetView(ECoasterView::ThirdPerson, FTransform::Identity, 45);
    auto Pose = Rig.Pose(Focus);
    TestTrue(TEXT("Orbit aims at the train"), Pose.GetRotation().GetForwardVector().Equals((Focus - Pose.GetLocation()).GetSafeNormal(), 1e-8));
    const FVector Travel(400, 900, -250);
    TestTrue(TEXT("Third person follows translation without train roll"), (Rig.Pose(Focus + Travel).GetLocation() - Pose.GetLocation()).Equals(Travel, 1e-8));
    Rig.Input(FVector2D(100, 50), false, 0, FVector::ZeroVector, false, 0);
    TestFalse(TEXT("Mouse orbit works while playback is paused"), Rig.Pose(Focus).Equals(Pose));
    TestTrue(TEXT("Orbit keeps horizon level"), FMath::IsNearlyZero(Rig.Pose(Focus).Rotator().Roll));
    Pose = Rig.Pose(Focus);
    Rig.Input(FVector2D(40, -25), true, 0, FVector::ZeroVector, false, 0);
    TestTrue(TEXT("Panning preserves view direction"), Rig.Pose(Focus).GetRotation().Equals(Pose.GetRotation(), 1e-8));
    TestFalse(TEXT("Panning changes position"), Rig.Pose(Focus).GetLocation().Equals(Pose.GetLocation()));
    for (int I = 0; I < 100; ++I) Rig.Input(FVector2D(0, 1000), false, 10, FVector::ZeroVector, false, 0);
    TestEqual(TEXT("Zoom cannot cross the focus"), Rig.OrbitDistance, 300.);
    TestEqual(TEXT("Pitch stops before the pole"), Rig.Look.Pitch, 89.);
    Rig.Reset(0);
    TestTrue(TEXT("Reset clears panning"), Rig.Pan.IsZero());
    Pose = Rig.Pose(Focus);
    Rig.SetView(ECoasterView::Free, Pose, 0);
    TestTrue(TEXT("Free view starts at the existing camera"), Rig.Pose(Focus).Equals(Pose, 1e-8));
    TestTrue(TEXT("Free view stays detached as the train moves"), Rig.Pose(Focus + Travel).Equals(Pose, 1e-8));
    Rig.Input(FVector2D::ZeroVector, false, 1, FVector::ZeroVector, false, 0);
    TestTrue(TEXT("Free wheel moves along view direction even when paused"),
        (Rig.FreePosition - Pose.GetLocation()).Equals(Pose.GetRotation().GetForwardVector() * 500., 1e-6));
    TestEqual(TEXT("Free wheel preserves flight speed"), Rig.FlySpeed, 15000.);
    Rig.Input(FVector2D::ZeroVector, false, -1, FVector::ZeroVector, false, 0);
    TestTrue(TEXT("Free wheel zoom is reversible"), Rig.FreePosition.Equals(Pose.GetLocation(), 1e-6));
    Rig.Look = FRotator::ZeroRotator;
    const FVector Start = Rig.FreePosition;
    Rig.Input(FVector2D::ZeroVector, false, 0, FVector(1, 1, 1), false, .1);
    TestTrue(TEXT("Diagonal fly speed is normalized"), FMath::IsNearlyEqual(FVector::Distance(Start, Rig.FreePosition), Rig.FlySpeed * .1, 1e-6));
    FCoasterCameraRig Fine = Rig, Coarse = Rig;
    for (int I = 0; I < 10; ++I) Fine.Input(FVector2D::ZeroVector, false, 0, FVector(1, 0, 0), false, .01);
    Coarse.Input(FVector2D::ZeroVector, false, 0, FVector(1, 0, 0), false, .1);
    TestTrue(TEXT("Fly motion is independent of frame rate"), Fine.FreePosition.Equals(Coarse.FreePosition, 1e-6));
    Rig.SetView(ECoasterView::Rider, Pose, 0);
    const FVector Parked = Rig.FreePosition;
    Rig.Input(FVector2D(100, 100), false, 10, FVector(1), true, 1);
    TestTrue(TEXT("Inspection input cannot move the rider camera"), Rig.FreePosition.Equals(Parked));
    TestEqual(TEXT("Rider wheel narrows field of view"), Rig.RiderFieldOfView, 32.);
    Rig.Input(FVector2D::ZeroVector, false, 10, FVector::ZeroVector, false, 0);
    TestEqual(TEXT("Rider field of view has a lower bound"), Rig.RiderFieldOfView, 30.);
    for (int I = 0; I < 3; ++I) Rig.Input(FVector2D::ZeroVector, false, -10, FVector::ZeroVector, false, 0);
    TestEqual(TEXT("Rider field of view has an upper bound"), Rig.RiderFieldOfView, 110.);
    Rig.Reset(0);
    TestEqual(TEXT("Reset restores the normal rider lens"), Rig.RiderFieldOfView, 82.);
    return true;
}
#endif
