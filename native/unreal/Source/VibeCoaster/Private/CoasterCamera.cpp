#include "CoasterCamera.h"

void FCoasterCameraRig::Reset(double TrainHeading)
{
    Look = FRotator(-18, TrainHeading, 0);
    Pan = FVector::ZeroVector;
    OrbitDistance = 2200;
    RiderFieldOfView = 82;
}

void FCoasterCameraRig::SetView(ECoasterView Next, const FTransform& Current, double TrainHeading)
{
    if (Next == ECoasterView::ThirdPerson) Reset(TrainHeading);
    if (Next == ECoasterView::Free)
    {
        FreePosition = Current.GetLocation();
        Look = Current.Rotator();
        Look.Pitch = FMath::Clamp(FRotator::NormalizeAxis(Look.Pitch), -89., 89.);
        Look.Roll = 0;
    }
    View = Next;
}

void FCoasterCameraRig::Input(FVector2D Mouse, bool Panning, double Wheel, FVector Move, bool Fast, double Seconds)
{
    Wheel = FMath::Clamp(Wheel, -10., 10.);
    if (View == ECoasterView::Rider)
    {
        RiderFieldOfView = FMath::Clamp(RiderFieldOfView - Wheel * 5., 30., 110.);
        return;
    }
    if (View != ECoasterView::ThirdPerson && View != ECoasterView::Free) return;
    if (View == ECoasterView::ThirdPerson && Panning)
    {
        const FRotationMatrix Axes(Look);
        Pan += (-Axes.GetUnitAxis(EAxis::Y) * Mouse.X + Axes.GetUnitAxis(EAxis::Z) * Mouse.Y) * OrbitDistance * .0015;
        Pan = Pan.GetClampedToMaxSize(50000);
    }
    else
    {
        Look.Yaw = FRotator::NormalizeAxis(Look.Yaw + Mouse.X * .2);
        Look.Pitch = FMath::Clamp(Look.Pitch + Mouse.Y * .2, -89., 89.);
    }
    const double Zoom = FMath::Pow(1.2, -Wheel);
    if (View == ECoasterView::ThirdPerson) OrbitDistance = FMath::Clamp(OrbitDistance * Zoom, 300., 200000.);
    else
    {
        const FQuat Axes = Look.Quaternion();
        FreePosition += Axes.GetForwardVector() * Wheel * 500. * (Fast ? 4. : 1.);
        FVector Direction = Axes.GetForwardVector() * Move.X + Axes.GetRightVector() * Move.Y + FVector::UpVector * Move.Z;
        FreePosition += Direction.GetClampedToMaxSize(1.) * FlySpeed * (Fast ? 4. : 1.) * FMath::Clamp(Seconds, 0., .1);
    }
}

FTransform FCoasterCameraRig::Pose(FVector Focus) const
{
    return FTransform(Look, View == ECoasterView::Free ? FreePosition : Focus + Pan - Look.Vector() * OrbitDistance);
}
