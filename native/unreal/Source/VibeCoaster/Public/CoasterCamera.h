#pragma once
#include "CoreMinimal.h"

enum class ECoasterView : uint8 { Rider, Overview, ThirdPerson, Free };

// Inspection cameras use world up, independently of the train's bank and roll.
struct FCoasterCameraRig
{
    ECoasterView View = ECoasterView::Rider;
    FRotator Look{-18, 0, 0};
    FVector Pan = FVector::ZeroVector;
    FVector FreePosition = FVector::ZeroVector;
    double OrbitDistance = 2200;
    double FlySpeed = 15000;
    double RiderFieldOfView = 82;

    void SetView(ECoasterView Next, const FTransform& Current, double TrainHeading);
    void Reset(double TrainHeading);
    void Input(FVector2D Mouse, bool Panning, double Wheel, FVector Move, bool Fast, double Seconds);
    FTransform Pose(FVector Focus) const;
};
